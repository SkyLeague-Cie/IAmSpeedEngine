#include "SpeedAnalyticAuthoredImport.h"
#include "Components/StaticMeshComponent.h"

#include "IAmSpeed/World/Analytic/AnalyticWorldData.h"
#include "IAmSpeed/World/Analytic/SpeedAnalyticCollisionAsset.h"
#include "IAmSpeed/World/Analytic/SpeedAnalyticSourceComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Dom/JsonObject.h"
#include "Engine/World.h"
#include "Engine/Blueprint.h"
#include "Engine/Level.h"
#include "GameFramework/Actor.h"
#include "HAL/FileManager.h"
#include "Misc/Paths.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"
#include "UObject/Package.h"
#include "UObject/StrongObjectPtr.h"

#if PLATFORM_WINDOWS
#include "Windows/WindowsHWrapper.h"
#include "Windows/AllowWindowsPlatformTypes.h"
#include <bcrypt.h>
#include "Windows/HideWindowsPlatformTypes.h"
#if defined(_MSC_VER)
#pragma comment(lib, "bcrypt.lib")
#endif
#endif

namespace SpeedAuthoredImport
{
	using FObject = TSharedPtr<FJsonObject>;
	using FValues = TArray<TSharedPtr<FJsonValue>>;
	struct FInputPin { const TCHAR* RelativePath; int64 Bytes; const TCHAR* Sha256; };
	// These identify already-produced files, not an execution grant or epoch.
	constexpr FInputPin RecipePin{ TEXT("RLDimension/OctagonalV2/Numeric2/recipe.json"), 7362866,
		TEXT("e369f7e28f7124d069b0d023761ebd0081ddbf73f8d8786688b1655b0cba660a") };
	constexpr FInputPin GeometryPin{ TEXT("RLDimension/OctagonalV2/Numeric2/geometry.json"), 2324463,
		TEXT("b3a9f7de034381508e63aa5d1431051dd3ff7518b55c2e48e509c475b8994b3c") };
	constexpr FInputPin BoundsPin{ TEXT("RLDimension/OctagonalV2/Numeric2/conversion-bounds.json"), 8297208,
		TEXT("4d53fd9cafa563418178ca1b5647c12868c17e712cc4d8be956e766b082a06ae") };
	constexpr int64 MaxInputBytes = 8600000;
	constexpr int32 TensorCount = 1078, PlaneCount = 16, DomainVertexCount = 210;
	constexpr const TCHAR* RecipeId = TEXT("C2_ASSEMBLED_STADIUM_OCTAGONAL_V2");
	constexpr const TCHAR* RecipeSchema = TEXT("skyleague.analytic-c2-assembled-stadium/v2");
	constexpr const TCHAR* Frame = TEXT("SL_Xlong_YminusRLX_Zup_cm_v1");

	bool Fail(FString& Reason, const FString& Message) { Reason = Message; return false; }
	bool TextIs(const FObject& Object, const TCHAR* Key, const FString& Expected)
	{
		FString Value;
		return Object.IsValid() && Object->TryGetStringField(Key, Value) && Value == Expected;
	}
	FObject ObjectAt(const FValues& Values, int32 Index)
	{
		return Values.IsValidIndex(Index) && Values[Index].IsValid() && Values[Index]->Type == EJson::Object
			? Values[Index]->AsObject() : nullptr;
	}
	bool Array(const FObject& Object, const TCHAR* Key, const FValues*& Out, int32 Count = INDEX_NONE)
	{
		Out = nullptr;
		return Object.IsValid() && Object->TryGetArrayField(Key, Out) && Out &&
			(Count == INDEX_NONE || Out->Num() == Count);
	}
	bool Int(const FObject& Object, const TCHAR* Key, int32& Out, int32 Min, int32 Max)
	{
		double Value = 0;
		const TSharedPtr<FJsonValue> Field = Object.IsValid() ? Object->TryGetField(Key) : nullptr;
		if (!Field.IsValid() || Field->Type != EJson::Number || !Field->TryGetNumber(Value) ||
			!FMath::IsFinite(Value) || Value < Min || Value > Max || Value != FMath::FloorToDouble(Value)) return false;
		Out = static_cast<int32>(Value); return true;
	}
	bool StringValueIs(const TSharedPtr<FJsonValue>& Value, const TCHAR* Expected)
	{
		FString Text;
		return Value.IsValid() && Value->Type == EJson::String && Value->TryGetString(Text) && Text == Expected;
	}
	bool HashBytes(const void* Data, uint32 Count, FString& Out)
	{
		Out.Reset();
		if (!Data && Count != 0) return false;
		// Installed UE5.8.2 has no Windows GetSHA256Signature override: the
		// generic implementation asserts. Use the OS provider explicitly.
#if PLATFORM_WINDOWS
		BCRYPT_ALG_HANDLE Algorithm = nullptr;
		BCRYPT_HASH_HANDLE Hash = nullptr;
		if (BCryptOpenAlgorithmProvider(&Algorithm, BCRYPT_SHA256_ALGORITHM, nullptr, 0) < 0) return false;
		uint8 Digest[32]{};
		bool bOk = BCryptCreateHash(Algorithm, &Hash, nullptr, 0, nullptr, 0, 0) >= 0;
		if (bOk && Count != 0) bOk = BCryptHashData(Hash, static_cast<PUCHAR>(const_cast<void*>(Data)), Count, 0) >= 0;
		if (bOk) bOk = BCryptFinishHash(Hash, Digest, sizeof(Digest), 0) >= 0;
		if (Hash) BCryptDestroyHash(Hash);
		BCryptCloseAlgorithmProvider(Algorithm, 0);
		if (bOk) for (uint8 Byte : Digest) Out.Appendf(TEXT("%02x"), static_cast<unsigned int>(Byte));
		return bOk;
#else
		return false; // Unsupported import platform, never a fabricated digest.
#endif
	}
	bool ReadPinnedAscii(const FInputPin& Pin, FString& Out, FString& Reason)
	{
		Out.Reset();
		const FString Path = FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() / Pin.RelativePath);
		TUniquePtr<FArchive> Reader(IFileManager::Get().CreateFileReader(*Path));
		if (!Reader || Pin.Bytes <= 0 || Pin.Bytes > MaxInputBytes || Reader->TotalSize() != Pin.Bytes)
			return Fail(Reason, FString::Printf(TEXT("Missing or wrong-size pinned input: %s"), *Path));
		TArray<uint8> Bytes; Bytes.SetNumUninitialized(static_cast<int32>(Pin.Bytes));
		Reader->Serialize(Bytes.GetData(), Bytes.Num());
		if (Reader->IsError() || Reader->Tell() != Pin.Bytes || Reader->TotalSize() != Pin.Bytes)
			return Fail(Reason, TEXT("Pinned input changed or failed during bounded read."));
		FString Hash;
		if (!HashBytes(Bytes.GetData(), Bytes.Num(), Hash) || Hash != Pin.Sha256)
			return Fail(Reason, FString::Printf(TEXT("SHA256 mismatch or unavailable: %s"), *Path));
		// The pinned producers emit ASCII. Reject embedded NUL/invalid encoding.
		for (uint8 Byte : Bytes) if (Byte == 0 || Byte > 127) return Fail(Reason, TEXT("Input is not the pinned ASCII encoding."));
		const FUTF8ToTCHAR Decoded(reinterpret_cast<const ANSICHAR*>(Bytes.GetData()), Bytes.Num());
		Out = FString(Decoded.Length(), Decoded.Get());
		return true;
	}
	bool ReadPinned(const FInputPin& Pin, FObject& Out, FString& Reason)
	{
		Out.Reset();
		FString Json;
		if (!ReadPinnedAscii(Pin, Json, Reason)) return false;
		if (!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Json), Out) || !Out.IsValid())
			return Fail(Reason, TEXT("Pinned JSON could not be decoded."));
		return true;
	}
	FString DoubleHex(double Value)
	{
		static_assert(sizeof(double) == sizeof(uint64), "Binary64 required.");
		uint64 Bits = 0; FMemory::Memcpy(&Bits, &Value, sizeof(Bits));
		return FString::Printf(TEXT("%016llx"), static_cast<unsigned long long>(Bits));
	}
	bool Scalar(const TSharedPtr<FJsonValue>& Value, const TSharedPtr<FJsonValue>& Hex, double& Out)
	{
		FString Expected;
		return Value.IsValid() && Value->Type == EJson::Number && Value->TryGetNumber(Out) &&
			FMath::IsFinite(Out) && Hex.IsValid() && Hex->TryGetString(Expected) &&
			Expected.Len() == 16 && DoubleHex(Out) == Expected;
	}
	bool Coordinates(const TSharedPtr<FJsonValue>& Value, const TSharedPtr<FJsonValue>& Hex,
		int32 Count, double* Out)
	{
		const FValues *Values = nullptr, *Hexes = nullptr;
		if (!Value.IsValid() || !Hex.IsValid() || !Value->TryGetArray(Values) || !Hex->TryGetArray(Hexes) ||
			!Values || !Hexes || Values->Num() != Count || Hexes->Num() != Count) return false;
		for (int32 I = 0; I < Count; ++I) if (!Scalar((*Values)[I], (*Hexes)[I], Out[I])) return false;
		return true;
	}
	bool VectorField(const FObject& Value, const FObject& Hex, const TCHAR* Key, FVector& Out)
	{
		double V[3];
		if (!Coordinates(Value->TryGetField(Key), Hex->TryGetField(Key), 3, V)) return false;
		Out = FVector(V[0], V[1], V[2]); return true;
	}
	bool SameStrings(const FObject& A, const TCHAR* AKey, const FObject& B, const TCHAR* BKey, int32 Count)
	{
		const FValues *AA, *BB;
		if (!Array(A, AKey, AA, Count) || !Array(B, BKey, BB, Count)) return false;
		for (int32 I = 0; I < Count; ++I)
		{
			FString AS, BS;
			if (!(*AA)[I].IsValid() || !(*BB)[I].IsValid() || !(*AA)[I]->TryGetString(AS) ||
				!(*BB)[I]->TryGetString(BS) || AS != BS || AS.IsEmpty()) return false;
		}
		return true;
	}
	bool Capture(USpeedAnalyticSourceComponent* Source, UPrimitiveComponent* Contact,
		FSpeedAnalyticAuthoredSourceRecord& Out, FString& Reason)
	{
		if (!IsInGameThread() || !IsValid(Source) || !IsValid(Contact) || Source->IsTemplate() || Contact->IsTemplate() ||
			!Source->IsRegistered() || !Contact->IsRegistered())
			return Fail(Reason, TEXT("Staging requires actual registered components on the editor game thread."));
		AActor* Owner = Source->GetOwner(); UWorld* World = Source->GetWorld();
		if (!IsValid(Owner) || Owner->IsTemplate() || !World || World->WorldType != EWorldType::Editor ||
			Contact->GetOwner() != Owner || Contact->GetWorld() != World || Owner->GetWorld() != World ||
			Contact->Mobility != EComponentMobility::Static || Source->AuthoredSourceId == 0 ||
			Source->bIncludeOwnedStaticMeshes || Source->AuthoredContactComponentPath != Contact->GetPathName(Owner))
			return Fail(Reason, TEXT("Source declaration, editor world, ownership or static contact does not match."));
		TArray<USpeedAnalyticSourceComponent*> Declarations; Owner->GetComponents(Declarations);
		if (Declarations.Num() != 1 || Declarations[0] != Source)
			return Fail(Reason, TEXT("Owner must have exactly one authored source declaration."));
		Out.Kind = ESpeedAnalyticSourceKind::AuthoredAnalytic; Out.SourceId = Source->AuthoredSourceId;
		Out.RecipeSchema = RecipeSchema; Out.RecipeSha256 = RecipePin.Sha256; Out.PayloadSha256 = GeometryPin.Sha256;
		Out.SourceComponentPath = Source->GetPathName(Owner); Out.ContactComponentPath = Contact->GetPathName(Owner);
		Out.OwnerWorldTransform = Owner->GetActorTransform(); Out.ContactWorldTransform = Contact->GetComponentTransform();
		Out.ObjectType = static_cast<uint32>(Contact->GetCollisionObjectType());
		Out.CollisionEnabled = static_cast<uint8>(Contact->GetCollisionEnabled());
		Out.bQueryCollisionEnabled = Contact->IsQueryCollisionEnabled();
		Out.BlockingChannels = 0; Out.OverlapChannels = 0;
		constexpr uint32 Channels = UE_ARRAY_COUNT(FCollisionResponseContainer::EnumArray);
		static_assert(Channels > 0 && Channels <= 64, "Collision responses must fit the mask.");
		for (uint32 I = 0; I < Channels; ++I)
		{
			const ECollisionResponse Response = Contact->GetCollisionResponseToChannel(static_cast<ECollisionChannel>(I));
			if (Response == ECR_Block) Out.BlockingChannels |= uint64(1) << I;
			else if (Response == ECR_Overlap) Out.OverlapChannels |= uint64(1) << I;
			else if (Response != ECR_Ignore) return Fail(Reason, TEXT("Unknown collision response."));
		}
		return Out.IsValid(&Reason);
	}
	FString TransformBits(const FTransform& T)
	{
		const FVector P = T.GetTranslation(), S = T.GetScale3D(); const FQuat Q = T.GetRotation();
		FString Result;
		for (double V : { P.X, P.Y, P.Z, Q.X, Q.Y, Q.Z, Q.W, S.X, S.Y, S.Z })
			Result += DoubleHex(V) + TEXT(";");
		return Result;
	}
	FString SourceKey(const FSpeedAnalyticAuthoredSourceRecord& S)
	{
		return FString::Printf(TEXT("%016llx|%u|%u|%016llx|%016llx|"),
			static_cast<unsigned long long>(S.SourceId), S.ObjectType, S.CollisionEnabled,
			static_cast<unsigned long long>(S.BlockingChannels), static_cast<unsigned long long>(S.OverlapChannels)) +
			S.SourceComponentPath + TEXT("|") + S.ContactComponentPath + TEXT("|") +
			TransformBits(S.OwnerWorldTransform) + TEXT("|") + TransformBits(S.ContactWorldTransform);
	}
	struct FIdentityBuilder
	{
		FString Prefix;
		TSet<uint64> Used;
		TSet<FString> FaceIds;
		FValues Rows;
		bool Face(const FObject& Geometry, const FObject& Recipe, const FObject& Bounds,
			uint64& Surface, uint64& Feature, uint64& Primitive, uint64* Group, FString& Reason)
		{
			FString Id, Family;
			if (!Geometry.IsValid() || !Geometry->TryGetStringField(TEXT("id"), Id) || Id.IsEmpty() || FaceIds.Contains(Id) ||
				!Geometry->TryGetStringField(TEXT("source_family"), Family) || Family.IsEmpty() ||
				!TextIs(Recipe, TEXT("id"), Id) || !TextIs(Recipe, TEXT("source_family"), Family) || !TextIs(Bounds, TEXT("id"), Id))
				return Fail(Reason, TEXT("Face identity/order/family does not match the pinned recipe and report."));
			FaceIds.Add(Id);
			auto Row = MakeShared<FJsonObject>(); Row->SetStringField(TEXT("recipe_face_id"), Id);
			Row->SetStringField(TEXT("source_family"), Family);
			auto Assign = [&](const TCHAR* Role, uint64& Out)
			{
				Out = Speed::Analytic::StableStringId(Prefix + TEXT("|") + Id + TEXT("|") + Role);
				if (Out == 0 || Used.Contains(Out)) return false;
				Used.Add(Out); Row->SetStringField(Role, FString::Printf(TEXT("%016llx"), static_cast<unsigned long long>(Out))); return true;
			};
			if (!Assign(TEXT("surface"), Surface) || !Assign(TEXT("feature"), Feature) || !Assign(TEXT("primitive"), Primitive) ||
				(Group && !Assign(TEXT("group"), *Group))) return Fail(Reason, TEXT("Derived identity collision or zero identifier."));
			// Each tensor is its own group. This assigns identity, not a symmetry or C2 certificate.
			if (Group)
			{
				Row->SetArrayField(TEXT("original_u_domain"), Geometry->GetArrayField(TEXT("original_u_domain")));
				Row->SetArrayField(TEXT("original_v_domain"), Geometry->GetArrayField(TEXT("original_v_domain")));
			}
			Rows.Add(MakeShared<FJsonValueObject>(Row)); return true;
		}
	};
}

AActor* USpeedAnalyticAuthoredImport::SpawnStadiumEditorProbe(UBlueprint* Blueprint, UWorld* World, FString& OutReason)
{
	OutReason.Reset();
	if (!IsInGameThread() || !IsValid(World) || World->WorldType != EWorldType::Editor || World->IsGameWorld() ||
		World->HasBegunPlay() || !IsValid(World->PersistentLevel) || !IsValid(Blueprint) || Blueprint->bBeingCompiled ||
		!Blueprint->IsUpToDate() || !IsValid(Blueprint->GeneratedClass) ||
		Blueprint->GeneratedClass->ClassGeneratedBy != Blueprint || !Blueprint->GeneratedClass->IsChildOf(AActor::StaticClass()) ||
		Blueprint->GeneratedClass->HasAnyClassFlags(CLASS_Abstract | CLASS_Deprecated | CLASS_NewerVersionExists))
	{ OutReason = TEXT("Expected the actual compiled actor Blueprint and an inactive Editor world."); return nullptr; }
	FActorSpawnParameters Params;
	Params.ObjectFlags = RF_Transient;
	Params.bTemporaryEditorActor = true;
	Params.bCreateActorPackage = false;
	Params.OverrideLevel = World->PersistentLevel;
	Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
	Params.bDeferConstruction = true;
	Params.TransformScaleMethod = ESpawnActorScaleMethod::OverrideRootScale;
	AActor* Actor = World->SpawnActor<AActor>(Blueprint->GeneratedClass, FTransform::Identity, Params);
	if (!Actor) { OutReason = TEXT("Editor probe spawn failed."); return nullptr; }
	Actor->Tags.AddUnique(TEXT("SLAnalyticStadiumEditorProbe"));
	Actor->FinishSpawning(FTransform::Identity, true, nullptr, ESpawnActorScaleMethod::OverrideRootScale);
	if (!IsValid(Actor) || Actor->GetWorld() != World || !Actor->HasAnyFlags(RF_Transient) ||
		!IsValid(Actor->GetRootComponent()) || !Actor->GetRootComponent()->IsRegistered() ||
		!Actor->GetActorTransform().Equals(FTransform::Identity, 0.0))
	{
		if (IsValid(Actor) && !World->EditorDestroyActor(Actor, false))
		{ OutReason = TEXT("Invalid Editor probe could not be destroyed; preserve world and inspect."); return Actor; }
		OutReason = TEXT("Construction invalidated the transient identity Editor probe."); return nullptr;
	}
	return Actor;
}

bool USpeedAnalyticAuthoredImport::DestroyStadiumEditorProbe(AActor* Actor)
{
	if (!IsInGameThread() || !IsValid(Actor) || !Actor->HasAnyFlags(RF_Transient) ||
		!Actor->ActorHasTag(TEXT("SLAnalyticStadiumEditorProbe")) || !IsValid(Actor->GetWorld()) ||
		Actor->GetWorld()->WorldType != EWorldType::Editor || Actor->GetWorld()->HasBegunPlay()) return false;
	return Actor->GetWorld()->EditorDestroyActor(Actor, false);
}

FString USpeedAnalyticAuthoredImport::DescribeContactPolicy(UPrimitiveComponent* Contact)
{
	if (!IsInGameThread() || !IsValid(Contact)) return FString();
	FString Result = FString::Printf(TEXT("%u|%u|"), uint32(Contact->GetCollisionObjectType()), uint32(Contact->GetCollisionEnabled()));
	const FCollisionResponseContainer& Responses = Contact->GetCollisionResponseToChannels();
	for (uint32 I = 0; I < UE_ARRAY_COUNT(Responses.EnumArray); ++I)
	{
		const uint32 Value = uint32(Responses.EnumArray[I]);
		if (Value > uint32(ECR_Block)) return FString();
		Result.AppendChar(TCHAR('0' + Value));
	}
	return Result;
}

bool USpeedAnalyticAuthoredImport::CopyExplicitContactPolicy(UPrimitiveComponent* Actual, UPrimitiveComponent* ContactTemplate)
{
	if (!IsInGameThread() || !IsValid(Actual) || !IsValid(ContactTemplate) || Actual->IsTemplate() ||
		!Actual->IsRegistered() || !Actual->GetWorld() || Actual->GetWorld()->WorldType != EWorldType::Editor ||
		!ContactTemplate->IsTemplate()) return false;
	const FString Before = DescribeContactPolicy(Actual);
	if (Before.IsEmpty()) return false;
	ContactTemplate->Modify();
	if (auto* MeshTemplate = Cast<UStaticMeshComponent>(ContactTemplate)) MeshTemplate->bUseDefaultCollision = false;
	ContactTemplate->SetCollisionEnabled(Actual->GetCollisionEnabled());
	ContactTemplate->SetCollisionObjectType(Actual->GetCollisionObjectType());
	ContactTemplate->SetCollisionResponseToChannels(Actual->GetCollisionResponseToChannels());
	return DescribeContactPolicy(ContactTemplate) == Before;
}

USpeedAnalyticCollisionAsset* USpeedAnalyticAuthoredImport::StageStadiumV1(
	USpeedAnalyticSourceComponent* Source, UPrimitiveComponent* Contact, FString& OutManifestJson, FString& OutReason)
{
	using namespace SpeedAuthoredImport;
	OutManifestJson.Reset(); OutReason.Reset();
	FSpeedAnalyticAuthoredSourceRecord Captured;
	if (!Capture(Source, Contact, Captured, OutReason)) return nullptr;
	const FString CapturedKey = SourceKey(Captured);
	FObject Recipe, Geometry, Bounds;
	if (!ReadPinned(RecipePin, Recipe, OutReason) || !ReadPinned(GeometryPin, Geometry, OutReason) ||
		!ReadPinned(BoundsPin, Bounds, OutReason)) return nullptr;
	if (!TextIs(Recipe, TEXT("schema"), RecipeSchema) || !TextIs(Recipe, TEXT("recipe_id"), RecipeId) ||
		!TextIs(Recipe, TEXT("units"), TEXT("cm")) || !TextIs(Recipe, TEXT("frame"), Frame) ||
		!TextIs(Geometry, TEXT("schema"), TEXT("skyleague.analytic-binary64-geometry-transfer/v1")) ||
		!TextIs(Geometry, TEXT("recipe_id"), RecipeId) || !TextIs(Geometry, TEXT("units"), TEXT("cm")) ||
		!TextIs(Geometry, TEXT("frame"), Frame) || !TextIs(Geometry, TEXT("source_recipe_sha256"), RecipePin.Sha256) ||
		!TextIs(Geometry, TEXT("runtime_source_ids_and_policies"), TEXT("UNBOUND")) ||
		!TextIs(Geometry, TEXT("authority_eligibility"), TEXT("UNASSIGNED")) ||
		!TextIs(Bounds, TEXT("schema"), TEXT("skyleague.analytic-coefficient-conversion-bounds/v1")) ||
		!TextIs(Bounds, TEXT("geometry_payload_sha256"), GeometryPin.Sha256))
	{ OutReason = TEXT("Pinned payload schemas, frame or provenance links are inconsistent."); return nullptr; }
	const FValues *Tensors = nullptr, *Planes = nullptr, *RecipeTensors = nullptr, *RecipePlanes = nullptr,
		*BoundTensors = nullptr, *BoundPlanes = nullptr, *Inputs = nullptr;
	if (!Array(Geometry, TEXT("tensor_patches"), Tensors, TensorCount) || !Array(Geometry, TEXT("bounded_planes"), Planes, PlaneCount) ||
		!Array(Recipe, TEXT("patches"), RecipeTensors, TensorCount) || !Array(Recipe, TEXT("planes"), RecipePlanes, PlaneCount) ||
		!Array(Bounds, TEXT("patches"), BoundTensors, TensorCount) || !Array(Bounds, TEXT("planes"), BoundPlanes, PlaneCount) ||
		!Array(Bounds, TEXT("source_inputs"), Inputs, 4) || !TextIs(ObjectAt(*Inputs, 0), TEXT("sha256"), RecipePin.Sha256))
	{ OutReason = TEXT("Pinned face counts or conversion recipe link are inconsistent."); return nullptr; }
	TStrongObjectPtr<USpeedAnalyticCollisionAsset> Candidate(NewObject<USpeedAnalyticCollisionAsset>(GetTransientPackage(), NAME_None, RF_Transient));
	Candidate->AuthoredSourceSchemaVersion = 1; Candidate->AuthoredSources.Add(Captured);
	FIdentityBuilder Ids;
	Ids.Prefix = FString::Printf(TEXT("AnalyticStadiumImport/v1|%016llx|%s"), static_cast<unsigned long long>(Captured.SourceId), RecipeId);
	Ids.Used.Add(Captured.SourceId);
	for (int32 I = 0; I < TensorCount; ++I)
	{
		const FObject G = ObjectAt(*Tensors, I), R = ObjectAt(*RecipeTensors, I), B = ObjectAt(*BoundTensors, I);
		FSpeedAnalyticTensorBezierPatchRecord Patch;
		int32 RU, RV;
		const FValues *Points = nullptr, *Hexes = nullptr, *UnitDomain = nullptr;
		if (!Int(G, TEXT("degree_u"), Patch.DegreeU, 1, 10) || !Int(G, TEXT("degree_v"), Patch.DegreeV, 1, 10) ||
			!Int(R, TEXT("degree_u"), RU, 1, 10) || !Int(R, TEXT("degree_v"), RV, 1, 10) || RU != Patch.DegreeU || RV != Patch.DegreeV ||
			!SameStrings(G, TEXT("original_u_domain"), R, TEXT("u_domain"), 2) ||
			!SameStrings(G, TEXT("original_v_domain"), R, TEXT("v_domain"), 2) ||
			!TextIs(G, TEXT("control_order"), TEXT("u-major-v-minor")) || !Array(G, TEXT("runtime_parameter_domain"), UnitDomain, 2) ||
			!StringValueIs((*UnitDomain)[0], TEXT("0")) || !StringValueIs((*UnitDomain)[1], TEXT("1")) ||
			!Array(G, TEXT("control_points_cm"), Points, (RU + 1) * (RV + 1)) ||
			!Array(B, TEXT("control_binary64_be_hex"), Hexes, Points->Num()))
		{ OutReason = FString::Printf(TEXT("Tensor %d shape or domain mismatch."), I); return nullptr; }
		if (!Ids.Face(G, R, B, Patch.SurfaceId, Patch.FeatureId, Patch.PrimitiveId, &Patch.CanonicalGroupId, OutReason)) return nullptr;
		for (int32 J = 0; J < Points->Num(); ++J)
		{
			double V[3];
			if (!Coordinates((*Points)[J], (*Hexes)[J], 3, V))
			{ OutReason = FString::Printf(TEXT("Tensor %d control %d binary64 mismatch."), I, J); return nullptr; }
			Patch.ControlPoints.Emplace(V[0], V[1], V[2]);
		}
		Patch.SourceId = Captured.SourceId; Patch.ObjectType = Captured.ObjectType;
		Patch.BlockingChannels = Captured.BlockingChannels; Patch.bQueryCollisionEnabled = Captured.bQueryCollisionEnabled;
		Patch.bAuthorityEligible = false; // Native numeric/physical gates remain unqualified.
		Candidate->TensorBezierPatches.Add(MoveTemp(Patch));
	}
	int32 Vertices = 0;
	for (int32 I = 0; I < PlaneCount; ++I)
	{
		const FObject G = ObjectAt(*Planes, I), R = ObjectAt(*RecipePlanes, I), B = ObjectAt(*BoundPlanes, I);
		FSpeedAnalyticBoundedPlaneRecord Plane;
		if (!Ids.Face(G, R, B, Plane.SurfaceId, Plane.FeatureId, Plane.PrimitiveId, nullptr, OutReason)) return nullptr;
		const TSharedPtr<FJsonObject>* Fields = nullptr;
		const FValues *Domain = nullptr, *HexDomain = nullptr;
		double Half[2]; int32 PreservedVertices;
		if (!B->TryGetObjectField(TEXT("field_binary64_be_hex"), Fields) || !Fields || !Fields->IsValid() ||
			!VectorField(G, *Fields, TEXT("origin_cm"), Plane.Origin) || !VectorField(G, *Fields, TEXT("normal"), Plane.Normal) ||
			!VectorField(G, *Fields, TEXT("axis_u"), Plane.AxisU) || !VectorField(G, *Fields, TEXT("axis_v"), Plane.AxisV) ||
			!Coordinates(G->TryGetField(TEXT("half_extents_cm")), (*Fields)->TryGetField(TEXT("half_extents_cm")), 2, Half) ||
			!Array(G, TEXT("domain_vertices_cm"), Domain) || Domain->Num() < 3 || Domain->Num() > 76 ||
			!Array(B, TEXT("domain_binary64_be_hex"), HexDomain, Domain->Num()) ||
			!Int(B, TEXT("vertex_count_preserved"), PreservedVertices, 3, 76) || PreservedVertices != Domain->Num())
		{ OutReason = FString::Printf(TEXT("Finite plane %d frame/domain mismatch."), I); return nullptr; }
		Plane.HalfExtents = FVector2D(Half[0], Half[1]);
		for (int32 J = 0; J < Domain->Num(); ++J)
		{
			double V[2];
			if (!Coordinates((*Domain)[J], (*HexDomain)[J], 2, V))
			{ OutReason = FString::Printf(TEXT("Finite plane %d domain vertex %d binary64 mismatch."), I, J); return nullptr; }
			Plane.DomainVertices.Emplace(V[0], V[1]);
		}
		Vertices += Domain->Num(); Plane.SourceId = Captured.SourceId; Plane.ObjectType = Captured.ObjectType;
		Plane.BlockingChannels = Captured.BlockingChannels; Plane.bQueryCollisionEnabled = Captured.bQueryCollisionEnabled;
		Plane.bAuthorityEligible = false; Candidate->BoundedPlanes.Add(MoveTemp(Plane));
	}
	if (Vertices != DomainVertexCount) { OutReason = TEXT("Finite domain vertex count changed."); return nullptr; }
	Candidate->TensorBezierPatches.Sort([](const auto& A, const auto& B) { return A.PrimitiveId < B.PrimitiveId; });
	Candidate->BoundedPlanes.Sort([](const auto& A, const auto& B) { return A.PrimitiveId < B.PrimitiveId; });
	Candidate->SourceHash = Speed::Analytic::StableStringId(Ids.Prefix + TEXT("|") + RecipePin.Sha256 + TEXT("|") +
		GeometryPin.Sha256 + TEXT("|") + BoundsPin.Sha256 + TEXT("|") + CapturedKey);
	if (!Candidate->ValidateGeneratedData(&OutReason, false)) return nullptr;
	FSpeedAnalyticAuthoredSourceRecord AtEnd;
	if (!Capture(Source, Contact, AtEnd, OutReason) || SourceKey(AtEnd) != CapturedKey)
	{ if (OutReason.IsEmpty()) OutReason = TEXT("Source/contact changed while staging."); return nullptr; }
	const auto Manifest = MakeShared<FJsonObject>();
	Manifest->SetStringField(TEXT("schema"), TEXT("skyleague.authored-stadium-staging-manifest/v1"));
	Manifest->SetStringField(TEXT("recipe_sha256"), RecipePin.Sha256);
	Manifest->SetStringField(TEXT("geometry_sha256"), GeometryPin.Sha256);
	Manifest->SetStringField(TEXT("conversion_bounds_sha256"), BoundsPin.Sha256);
	Manifest->SetStringField(TEXT("source_identity_and_policy"), CapturedKey);
	Manifest->SetStringField(TEXT("source_hash"), FString::Printf(TEXT("%016llx"), static_cast<unsigned long long>(Candidate->SourceHash)));
	Manifest->SetStringField(TEXT("world"), Source->GetWorld()->GetPathName());
	Manifest->SetStringField(TEXT("owner"), Source->GetOwner()->GetPathName());
	Manifest->SetStringField(TEXT("frame"), Frame); Manifest->SetStringField(TEXT("units"), TEXT("cm"));
	Manifest->SetArrayField(TEXT("faces"), Ids.Rows);
	Manifest->SetNumberField(TEXT("finite_domain_vertices"), Vertices);
	Manifest->SetBoolField(TEXT("authority_eligible"), false); Manifest->SetBoolField(TEXT("materials_assigned"), false);
	Manifest->SetBoolField(TEXT("source_bound"), false); Manifest->SetBoolField(TEXT("saved"), false);
	Manifest->SetBoolField(TEXT("runtime_world_built"), false);
	Manifest->SetStringField(TEXT("numeric_physical_seams_and_evaluator_qualification"), TEXT("NOT_ESTABLISHED"));
	if (!FJsonSerializer::Serialize(Manifest, TJsonWriterFactory<>::Create(&OutManifestJson)))
	{ OutManifestJson.Reset(); OutReason = TEXT("Could not serialize staging manifest."); return nullptr; }
	OutReason = TEXT("Staged transient diagnostic payload; no authority admission, binding, saving, mesh or materials.");
	return Candidate.Get();
}

// Same translation unit so both importers share the bounded pinned-input reader.
#include "SpeedAnalyticRenderImport.inl"
#include "SpeedAnalyticPersistImport.inl"
#include "SpeedAnalyticCurrentImport.inl"

#if WITH_DEV_AUTOMATION_TESTS
#include "Components/BoxComponent.h"
#include "Engine/Engine.h"
#include "Misc/AutomationTest.h"
#include "Misc/ScopeExit.h"

#if PLATFORM_WINDOWS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSpeedAuthoredImportHashTest,
	"IAmSpeed.AnalyticImport.SHA256", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FSpeedAuthoredImportHashTest::RunTest(const FString&)
{
	FString Digest;
	const uint8 Data[]{ 'a', 'b', 'c' };
	TestTrue(TEXT("platform SHA256 available"), SpeedAuthoredImport::HashBytes(Data, 3, Digest));
	TestEqual(TEXT("known SHA256 vector"), Digest, FString(TEXT("ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad")));
	return !HasAnyErrors();
}
#endif

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSpeedAuthoredImportBinary64Test,
	"IAmSpeed.AnalyticImport.Binary64", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FSpeedAuthoredImportBinary64Test::RunTest(const FString&)
{
	using namespace SpeedAuthoredImport;
	FObject Parsed;
	TestTrue(TEXT("parse decimal coefficients"), FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(
		TEXT("{\"v\":[0.1,3788.8,420.5946669226287],\"h\":[\"3fb999999999999a\",\"40ad99999999999a\",\"407a4983c1768b40\"]}")), Parsed));
	if (!Parsed.IsValid()) return false;
	double V[3];
	TestTrue(TEXT("parsed decimals retain all pinned bits"), Coordinates(Parsed->TryGetField(TEXT("v")), Parsed->TryGetField(TEXT("h")), 3, V));
	double Number = 0;
	TestFalse(TEXT("a one-bit drift fails"), Scalar(MakeShared<FJsonValueNumber>(0.1), MakeShared<FJsonValueString>(TEXT("3fb999999999999b")), Number));
	TestFalse(TEXT("number strings fail"), Scalar(MakeShared<FJsonValueString>(TEXT("0.1")), MakeShared<FJsonValueString>(TEXT("3fb999999999999a")), Number));
	TestFalse(TEXT("signed zero mismatch fails"), Scalar(MakeShared<FJsonValueNumber>(-0.0), MakeShared<FJsonValueString>(TEXT("0000000000000000")), Number));
	TestFalse(TEXT("wrong coordinate dimension fails"), Coordinates(Parsed->TryGetField(TEXT("v")), Parsed->TryGetField(TEXT("h")), 2, V));
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSpeedAuthoredImportDeclarationTest,
	"IAmSpeed.AnalyticImport.NoImplicitSource", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FSpeedAuthoredImportDeclarationTest::RunTest(const FString&)
{
	FString Manifest = TEXT("stale"), Reason;
	TestNull(TEXT("no synthetic source/contact fallback"), USpeedAnalyticAuthoredImport::StageStadiumV1(nullptr, nullptr, Manifest, Reason));
	TestTrue(TEXT("stale manifest cleared"), Manifest.IsEmpty());
	TestFalse(TEXT("rejection is explicit"), Reason.IsEmpty());
	return !HasAnyErrors();
}

// Explicit integration test against the three pinned files. This is a synthetic
// declaration in a disposable editor world, not the mission's live SourceId or
// an imported/saved stadium. No BuildRuntimeData or BeginPlay is invoked.
#if PLATFORM_WINDOWS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSpeedAuthoredImportPinnedStadiumTest,
	"IAmSpeed.AnalyticImport.PinnedStadiumDiagnostic", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FSpeedAuthoredImportPinnedStadiumTest::RunTest(const FString&)
{
	if (!GEngine) { AddError(TEXT("Editor engine required.")); return false; }
	const auto Options = UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(true)
		.RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false)
		.ShouldSimulatePhysics(false).SetTransactional(false);
	UWorld* World = UWorld::CreateWorld(EWorldType::Editor, false, TEXT("AuthoredStagingTest"),
		GetTransientPackage(), true, ERHIFeatureLevel::Num, &Options);
	if (!World) { AddError(TEXT("Could not create disposable editor world.")); return false; }
	GEngine->CreateNewWorldContext(EWorldType::Editor).SetCurrentWorld(World);
	ON_SCOPE_EXIT { World->DestroyWorld(false); GEngine->DestroyWorldContext(World); };
	AActor* Owner = World->SpawnActor<AActor>();
	if (!Owner) { AddError(TEXT("Could not create test owner.")); return false; }
	auto* Contact = NewObject<UBoxComponent>(Owner, TEXT("Contact"));
	Owner->AddInstanceComponent(Contact); Owner->SetRootComponent(Contact);
	Contact->SetMobility(EComponentMobility::Static);
	Contact->SetCollisionEnabled(ECollisionEnabled::QueryOnly); Contact->SetCollisionObjectType(ECC_WorldStatic);
	Contact->SetCollisionResponseToAllChannels(ECR_Ignore);
	Contact->SetCollisionResponseToChannel(ECC_PhysicsBody, ECR_Block);
	Contact->SetCollisionResponseToChannel(ECC_Pawn, ECR_Overlap); Contact->RegisterComponent();
	auto* Source = NewObject<USpeedAnalyticSourceComponent>(Owner, TEXT("Source"));
	Owner->AddInstanceComponent(Source); Source->RegisterComponent();
	Source->AuthoredSourceId = 17; // Deliberately synthetic test declaration only.
	Source->AuthoredContactComponentPath = TEXT("Contact"); Source->bIncludeOwnedStaticMeshes = false;
	const bool bDirty = Owner->GetOutermost()->IsDirty();
	const FTransform OwnerTransform = Owner->GetActorTransform(), ContactTransform = Contact->GetComponentTransform();
	FString Manifest, Reason;
	TStrongObjectPtr<USpeedAnalyticCollisionAsset> Candidate(USpeedAnalyticAuthoredImport::StageStadiumV1(Source, Contact, Manifest, Reason));
	if (!Candidate.IsValid()) { AddError(Reason); return false; }
	TestTrue(TEXT("candidate is transient"), Candidate->HasAnyFlags(RF_Transient));
	TestFalse(TEXT("candidate is not a registered persistent asset"), Candidate->HasAnyFlags(RF_Public | RF_Standalone));
	TestEqual(TEXT("all tensors imported"), Candidate->TensorBezierPatches.Num(), 1078);
	TestEqual(TEXT("all finite planes imported"), Candidate->BoundedPlanes.Num(), 16);
	TestTrue(TEXT("no fabricated mesh/triangles"), Candidate->MeshSources.IsEmpty() && Candidate->Triangles.IsEmpty() &&
		Candidate->IndexedTriangles.IsEmpty() && Candidate->VertexPositions.IsEmpty() && Candidate->VertexNormals.IsEmpty());
	TestTrue(TEXT("full diagnostic data validates"), Candidate->ValidateGeneratedData(&Reason, false));
	TestFalse(TEXT("diagnostic staging grants no authority"), Candidate->ValidateGeneratedData(&Reason, true));
	int32 Vertices = 0;
	for (const auto& Plane : Candidate->BoundedPlanes) { Vertices += Plane.DomainVertices.Num(); TestFalse(TEXT("plane not admitted"), Plane.bAuthorityEligible); }
	for (const auto& Patch : Candidate->TensorBezierPatches) TestFalse(TEXT("tensor not admitted"), Patch.bAuthorityEligible);
	TestEqual(TEXT("all finite domain vertices retained"), Vertices, 210);
	TestEqual(TEXT("block mask captured"), Candidate->AuthoredSources[0].BlockingChannels, uint64(1) << ECC_PhysicsBody);
	TestEqual(TEXT("overlap mask captured"), Candidate->AuthoredSources[0].OverlapChannels, uint64(1) << ECC_Pawn);
	FString RepeatedManifest;
	TStrongObjectPtr<USpeedAnalyticCollisionAsset> Repeated(USpeedAnalyticAuthoredImport::StageStadiumV1(Source, Contact, RepeatedManifest, Reason));
	TestTrue(TEXT("repeat stages successfully"), Repeated.IsValid());
	TestEqual(TEXT("stable manifest excludes candidate UObject name"), RepeatedManifest, Manifest);
	TestTrue(TEXT("source remains unbound"), Source->AnalyticCollisionAsset.IsNull());
	TestEqual(TEXT("source identity unchanged"), Source->AuthoredSourceId, uint64(17));
	TestEqual(TEXT("package dirty flag unchanged"), Owner->GetOutermost()->IsDirty(), bDirty);
	TestTrue(TEXT("world transforms unchanged"), OwnerTransform.Equals(Owner->GetActorTransform(), 0.0) &&
		ContactTransform.Equals(Contact->GetComponentTransform(), 0.0));
	Contact->SetMobility(EComponentMobility::Movable);
	TestNull(TEXT("movable contact rejected without stale success manifest"),
		USpeedAnalyticAuthoredImport::StageStadiumV1(Source, Contact, RepeatedManifest, Reason));
	TestTrue(TEXT("failed staging clears manifest"), RepeatedManifest.IsEmpty());
	TestTrue(TEXT("failed staging leaves source unbound"), Source->AnalyticCollisionAsset.IsNull());
	return !HasAnyErrors();
}
#endif
#endif
