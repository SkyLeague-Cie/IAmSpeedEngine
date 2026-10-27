#if WITH_DEV_AUTOMATION_TESTS

#include "SpeedAnalyticCollisionAsset.h"
#include "SpeedAnalyticSourceComponent.h"
#include "AnalyticWorldData.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "Components/BoxComponent.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "HAL/IConsoleManager.h"
#include "Misc/AutomationTest.h"
#include "Misc/ScopeExit.h"
#include "UObject/Package.h"
#include "UObject/StrongObjectPtr.h"

namespace
{
	// Deliberately synthetic metadata, never a stadium import/certificate.
	void FillAuthoredAsset(USpeedAnalyticCollisionAsset& Asset)
	{
		Asset.SourceHash = 41;
		Asset.AuthoredSourceSchemaVersion = 1;
		FSpeedAnalyticAuthoredSourceRecord Source;
		Source.Kind = ESpeedAnalyticSourceKind::AuthoredAnalytic;
		Source.SourceId = 17;
		Source.RecipeSchema = TEXT("test.authored/v1");
		Source.RecipeSha256 = FString::ChrN(64, 'a');
		Source.PayloadSha256 = FString::ChrN(64, 'b');
		Source.SourceComponentPath = TEXT("Source");
		Source.ContactComponentPath = TEXT("Contact");
		Source.ObjectType = ECC_WorldStatic;
		Source.CollisionEnabled = ECollisionEnabled::QueryOnly;
		Source.bQueryCollisionEnabled = true;
		Source.BlockingChannels = uint64(1) << ECC_PhysicsBody;
		Asset.AuthoredSources = { Source };
		FSpeedAnalyticBoundedPlaneRecord Plane;
		Plane.SourceId = Source.SourceId;
		Plane.SurfaceId = 19;
		Plane.FeatureId = 23;
		Plane.PrimitiveId = 29;
		Plane.ObjectType = Source.ObjectType;
		Plane.BlockingChannels = Source.BlockingChannels;
		Plane.HalfExtents = FVector2D(100, 100);
		Plane.bAuthorityEligible = true;
		Asset.BoundedPlanes = { Plane };
	}

	struct FAuthoredWorldFixture
	{
		UWorld* World = nullptr;
		AActor* Owner = nullptr;
		UBoxComponent* Contact = nullptr;
		USpeedAnalyticSourceComponent* Source = nullptr;
		TStrongObjectPtr<USpeedAnalyticCollisionAsset> Asset{ NewObject<USpeedAnalyticCollisionAsset>() };

		explicit FAuthoredWorldFixture(const TCHAR* PackageName)
		{
			if (!GEngine) return;
			const auto Options = UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(true)
				.RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false)
				.ShouldSimulatePhysics(false).SetTransactional(false);
			World = UWorld::CreateWorld(EWorldType::Game, false, TEXT("AuthoredSourceTest"),
				CreatePackage(PackageName), true, ERHIFeatureLevel::Num, &Options);
			if (!World) return;
			GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
			Owner = World->SpawnActor<AActor>();
			if (!Owner) return;
			Contact = NewObject<UBoxComponent>(Owner, TEXT("Contact"));
			Owner->AddInstanceComponent(Contact);
			Owner->SetRootComponent(Contact);
			Contact->SetMobility(EComponentMobility::Static);
			Contact->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
			Contact->SetCollisionObjectType(ECC_WorldStatic);
			Contact->SetCollisionResponseToAllChannels(ECR_Ignore);
			Contact->SetCollisionResponseToChannel(ECC_PhysicsBody, ECR_Block);
			Contact->RegisterComponent();
			Source = NewObject<USpeedAnalyticSourceComponent>(Owner, TEXT("Source"));
			Owner->AddInstanceComponent(Source);
			Source->RegisterComponent();
			FillAuthoredAsset(*Asset);
			Source->AuthoredSourceId = 17;
			Source->AuthoredContactComponentPath = TEXT("Contact");
			Source->bIncludeOwnedStaticMeshes = false;
			Source->AnalyticCollisionAsset = Asset.Get();
		}

		~FAuthoredWorldFixture()
		{
			if (World)
			{
				World->DestroyWorld(false);
				GEngine->DestroyWorldContext(World);
			}
		}
		bool IsReady() const { return Source && Contact && World && Owner; }
	};
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedAuthoredProvenanceTest,
	"IAmSpeed.AnalyticWorld.AuthoredSource.Provenance",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FIAmSpeedAuthoredProvenanceTest::RunTest(const FString&)
{
	TStrongObjectPtr<USpeedAnalyticCollisionAsset> Asset(NewObject<USpeedAnalyticCollisionAsset>());
	FillAuthoredAsset(*Asset);
	FString Reason;
	TestTrue(TEXT("authored planes need no fabricated mesh or triangles"), Asset->ValidateGeneratedData(&Reason));
	TestTrue(TEXT("authority-only validation keeps the same provenance"), Asset->ValidateGeneratedData(&Reason, true));
	Asset->AuthoredSources[0].ObjectType = UE_ARRAY_COUNT(FCollisionResponseContainer::EnumArray);
	TestFalse(TEXT("deprecated channel is not a response-container object type"), Asset->ValidateGeneratedData(&Reason));
	FillAuthoredAsset(*Asset);
	constexpr uint32 StoredChannels = UE_ARRAY_COUNT(FCollisionResponseContainer::EnumArray);
	static_assert(StoredChannels > 0 && StoredChannels <= 64);
	constexpr uint64 UnsupportedMask = ~(~uint64(0) >> (64 - StoredChannels));
	if (UnsupportedMask != 0)
	{
		Asset->AuthoredSources[0].BlockingChannels |= UnsupportedMask;
		TestFalse(TEXT("channels beyond response container are rejected"), Asset->ValidateGeneratedData(&Reason));
	}
	else
	{
		Asset->AuthoredSources[0].BlockingChannels |= uint64(1) << 63;
		Asset->BoundedPlanes[0].BlockingChannels = Asset->AuthoredSources[0].BlockingChannels;
		TestTrue(TEXT("highest stored channel remains representable"), Asset->ValidateGeneratedData(&Reason));
		Asset->AuthoredSources[0].OverlapChannels |= uint64(1) << 63;
		TestFalse(TEXT("highest channel cannot both block and overlap"), Asset->ValidateGeneratedData(&Reason));
	}
	FillAuthoredAsset(*Asset);
	auto DiagnosticSource = Asset->AuthoredSources[0]; DiagnosticSource.SourceId = 18;
	Asset->AuthoredSources.Add(DiagnosticSource);
	auto DiagnosticPlane = Asset->BoundedPlanes[0];
	DiagnosticPlane.SourceId = 18; DiagnosticPlane.PrimitiveId = 30; DiagnosticPlane.FeatureId = 24;
	DiagnosticPlane.bAuthorityEligible = false;
	Asset->BoundedPlanes.Add(DiagnosticPlane);
	TestTrue(TEXT("full diagnostic payload retains both sources"), Asset->ValidateGeneratedData(&Reason));
	TestFalse(TEXT("one admitted source cannot hide another source omitted by authority filtering"), Asset->ValidateGeneratedData(&Reason, true));
	FillAuthoredAsset(*Asset);
	Asset->AuthoredSourceSchemaVersion = 0;
	TestFalse(TEXT("new records cannot masquerade as legacy"), Asset->ValidateGeneratedData(&Reason));
	Asset->AuthoredSourceSchemaVersion = 2;
	TestFalse(TEXT("unknown provenance version rejected"), Asset->ValidateGeneratedData(&Reason));
	Asset->AuthoredSourceSchemaVersion = 1;
	Asset->AuthoredSources.Add(Asset->AuthoredSources[0]);
	TestFalse(TEXT("duplicate authored id rejected"), Asset->ValidateGeneratedData(&Reason));
	Asset->AuthoredSources.SetNum(1);
	Asset->AuthoredSources[0].RecipeSha256 = TEXT("missing");
	TestFalse(TEXT("short recipe digest rejected"), Asset->ValidateGeneratedData(&Reason));
	FillAuthoredAsset(*Asset);
	Asset->BoundedPlanes[0].BlockingChannels = 0;
	TestFalse(TEXT("primitive policy cannot drift from source"), Asset->ValidateGeneratedData(&Reason));
	FillAuthoredAsset(*Asset);
	Asset->BoundedPlanes[0].SourceId = 99;
	TestFalse(TEXT("undeclared authored plane rejected"), Asset->ValidateGeneratedData(&Reason));
	FillAuthoredAsset(*Asset);
	Asset->BoundedPlanes.Reset();
	TestFalse(TEXT("unused declaration rejected"), Asset->ValidateGeneratedData(&Reason));
	FillAuthoredAsset(*Asset);
	FSpeedAnalyticTensorBezierPatchRecord Unclaimed;
	Unclaimed.SourceId = 99; // even a diagnostic primitive must identify its source
	Asset->TensorBezierPatches.Add(Unclaimed);
	TestFalse(TEXT("authority-only mode does not hide unknown authored source"), Asset->ValidateGeneratedData(&Reason, true));
	Asset->TensorBezierPatches.Reset();
	FSpeedAnalyticMeshSourceRecord Mesh;
	Mesh.SourceId = 17; Mesh.MeshId = 43; Mesh.TriangleCount = 1;
	Mesh.TriangleHash = 47; Mesh.NormalHash = 53; Mesh.CollisionPolicyHash = 59;
	Mesh.WorldBounds = FBox(FVector(-1), FVector(1));
	Asset->MeshSources.Add(Mesh);
	TestFalse(TEXT("mesh/authored source id alias rejected"), Asset->ValidateGeneratedData(&Reason));
	Asset->MeshSources[0].SourceId = 61;
	Asset->VertexPositions = { FVector::ZeroVector };
	Asset->VertexNormals = { FVector::UpVector };
	FSpeedAnalyticIndexedTriangleRecord Triangle;
	Triangle.SourceId = 17; Triangle.SurfaceId = 67; Triangle.FeatureId = 71; Triangle.PrimitiveId = 73;
	Asset->IndexedTriangles.Add(Triangle);
	TestFalse(TEXT("triangles cannot claim an authored source even in authority-only mode"), Asset->ValidateGeneratedData(&Reason, true));
	Asset->MeshSources.Reset(); Asset->IndexedTriangles.Reset();
	Asset->AuthoredSourceSchemaVersion = 0; Asset->AuthoredSources.Reset();
	TestTrue(TEXT("legacy bounded plane assets keep their existing validation"), Asset->ValidateGeneratedData(&Reason));
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedAuthoredContactTest,
	"IAmSpeed.AnalyticWorld.AuthoredSource.PortableContact",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FIAmSpeedAuthoredContactTest::RunTest(const FString&)
{
	for (const TCHAR* Package : { TEXT("/Temp/AuthoredFreeplay"), TEXT("/Temp/AuthoredMatch"), TEXT("/Temp/UEDPIE_7_AuthoredFreeplay") })
	{
		FAuthoredWorldFixture Fixture(Package);
		if (!TestTrue(TEXT("fixture ready"), Fixture.IsReady())) return false;
		const auto Record = Fixture.Asset->AuthoredSources[0];
		FString Reason;
		auto Resolve = [&]() { return Fixture.Source->ResolveAuthoredContact(Record, *Fixture.Asset, &Reason); };
		TestTrue(TEXT("relative declaration resolves in both maps and a PIE-style package"), Resolve() == Fixture.Contact);
		Fixture.Source->AuthoredContactComponentPath = TEXT("Missing");
		TestNull(TEXT("no fallback to root or first primitive"), Resolve());
		Fixture.Source->AuthoredContactComponentPath = TEXT("Contact");
		Fixture.Source->bIncludeOwnedStaticMeshes = true;
		TestNull(TEXT("authored mode cannot also discover meshes"), Resolve());
		Fixture.Source->bIncludeOwnedStaticMeshes = false;
		Fixture.Contact->SetCollisionResponseToChannel(ECC_Camera, ECR_Block);
		TestNull(TEXT("live policy drift rejected"), Resolve());
		Fixture.Contact->SetCollisionResponseToChannel(ECC_Camera, ECR_Ignore);
		auto MovedRecord = Record;
		MovedRecord.OwnerWorldTransform.SetTranslation(FVector(1, 0, 0));
		TestNull(TEXT("world-space payload cannot move with a different actor transform"),
			Fixture.Source->ResolveAuthoredContact(MovedRecord, *Fixture.Asset, &Reason));
		Fixture.Source->AnalyticCollisionAsset.Reset();
		TestNull(TEXT("explicit asset identity required"), Resolve());
	}
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedAuthoredBindingTransactionTest,
	"IAmSpeed.AnalyticWorld.AuthoredSource.BindingTransaction",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FIAmSpeedAuthoredBindingTransactionTest::RunTest(const FString&)
{
	IConsoleVariable* Backend = IConsoleManager::Get().FindConsoleVariable(TEXT("p.IAmSpeed.StaticCollision.Backend"));
	if (!TestNotNull(TEXT("backend exists"), Backend)) return false;
	const int32 Previous = Backend->GetInt(); Backend->Set(2, ECVF_SetByCode);
	ON_SCOPE_EXIT { Backend->Set(Previous, ECVF_SetByCode); };
	FAuthoredWorldFixture Fixture(TEXT("/Temp/AuthoredBindingTransaction"));
	if (!TestTrue(TEXT("fixture ready"), Fixture.IsReady())) return false;
	USpeedWorldSubsystem* Bridge = Fixture.World->GetSubsystem<USpeedWorldSubsystem>();
	if (!TestNotNull(TEXT("bridge exists"), Bridge)) return false;
	Bridge->OnWorldBeginPlay(*Fixture.World);
	TestNotNull(TEXT("authored-only immutable world built"), Bridge->GetAnalyticWorldData());
	TestTrue(TEXT("published source resolves actual primitive"), Bridge->FindAnalyticSourceComponent(17).Get() == Fixture.Contact);
	// Binding succeeds first, then immutable payload validation fails. Nothing
	// may publish even though there are no mesh components in this inventory.
	Fixture.Asset->SourceHash = 0;
	AddExpectedError(TEXT("[AnalyticAuthorityCoverageMissing]"), EAutomationExpectedErrorFlags::Contains, 1);
	Bridge->OnWorldBeginPlay(*Fixture.World);
	TestNull(TEXT("invalid zero-mesh bake cannot become an empty world"), Bridge->GetAnalyticWorldData());
	TestNull(TEXT("failed build does not leak provisional source map"), Bridge->FindAnalyticSourceComponent(17).Get());
	Fixture.Asset->SourceHash = 41;
	auto* Duplicate = NewObject<USpeedAnalyticSourceComponent>(Fixture.Owner, TEXT("Duplicate"));
	Fixture.Owner->AddInstanceComponent(Duplicate); Duplicate->RegisterComponent();
	AddExpectedError(TEXT("[AnalyticSourceBinding] Ambiguous declarations"), EAutomationExpectedErrorFlags::Contains, 1);
	Bridge->OnWorldBeginPlay(*Fixture.World);
	TestNull(TEXT("multiple declarations on the same owner rejected"), Bridge->GetAnalyticWorldData());
	TestNull(TEXT("ambiguous declaration publishes no mapping"), Bridge->FindAnalyticSourceComponent(17).Get());
	return !HasAnyErrors();
}

#endif
