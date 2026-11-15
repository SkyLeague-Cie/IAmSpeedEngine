// Included only by SpeedAnalyticAuthoredImport.cpp; no startup or automatic import.
#include "Engine/StaticMesh.h"
#include "MeshDescription.h"
#include "StaticMeshAttributes.h"
#include "StaticMeshResources.h"
#include "PhysicsEngine/BodySetup.h"

namespace SpeedAuthoredImport
{
	constexpr FInputPin RenderPin{ TEXT("RLDimension/OctagonalV2/PostAudit2/render.wavefront"), 8201306,
		TEXT("65e7f46b9bad29db1fc1c1a4a0ec29cf3b99899791cacea7b0d00e67b7fc61d0") };
	constexpr int32 RenderPositions = 53480, RenderNormals = 21660, RenderTriangles = 106956;
	struct FRenderFace { int32 V[3]{}, N[3]{}, Role = INDEX_NONE; };
	struct FRenderInput
	{
		TArray<FVector3d> Positions, Normals;
		TArray<FString> Roles;
		TArray<FRenderFace> Faces;
	};
	bool RenderNumber(const FString& Text, double& Out)
	{
		if (Text.IsEmpty() || Text.Len() > 64) return false;
		int32 I = 0, Digits = 0;
		auto IsDigit = [&](int32 Index) { return Index < Text.Len() && Text[Index] >= TEXT('0') && Text[Index] <= TEXT('9'); };
		if (Text[I] == TEXT('-') || Text[I] == TEXT('+')) ++I;
		while (IsDigit(I)) { ++I; ++Digits; }
		if (I < Text.Len() && Text[I] == TEXT('.'))
		{
			++I; while (IsDigit(I)) { ++I; ++Digits; }
		}
		if (Digits == 0) return false;
		if (I < Text.Len() && (Text[I] == TEXT('e') || Text[I] == TEXT('E')))
		{
			++I;
			if (I < Text.Len() && (Text[I] == TEXT('-') || Text[I] == TEXT('+'))) ++I;
			const int32 Start = I; while (IsDigit(I)) ++I;
			if (I == Start) return false;
		}
		if (I != Text.Len()) return false;
		Out = FCString::Atod(*Text); return FMath::IsFinite(Out);
	}
	bool RenderIndex(const FString& Text, int32 Count, int32& Out)
	{
		if (Text.IsEmpty()) return false;
		int32 Value = 0;
		for (TCHAR C : Text)
		{
			if (C < TEXT('0') || C > TEXT('9') || Value > Count / 10) return false;
			Value = Value * 10 + C - TEXT('0');
			if (Value > Count) return false;
		}
		if (Value == 0) return false;
		Out = Value - 1; return true;
	}
	bool ParseRender(const FString& Text, FRenderInput& Out, FString& Reason)
	{
		Out = FRenderInput(); Reason.Reset();
		if (Text.IsEmpty() || Text.Len() > MaxInputBytes) return Fail(Reason, TEXT("Render input exceeds fixed bound or is empty."));
		TArray<FString> Lines; Text.ParseIntoArrayLines(Lines, false);
		int32 Role = INDEX_NONE;
		bool bFaces = false;
		for (const FString& Line : Lines)
		{
			if (Line.IsEmpty() || Line.StartsWith(TEXT("#"))) continue;
			TArray<FString> Words; Line.ParseIntoArrayWS(Words);
			if (Words.IsEmpty()) continue;
			if (Words[0] == TEXT("v") || Words[0] == TEXT("vn"))
			{
				FVector3d Value;
				if (bFaces || Words.Num() != 4 || !RenderNumber(Words[1], Value.X) ||
					!RenderNumber(Words[2], Value.Y) || !RenderNumber(Words[3], Value.Z))
					return Fail(Reason, TEXT("Invalid or late render position/normal."));
				if (Words[0] == TEXT("v"))
				{
					if (Out.Positions.Num() >= RenderPositions || FMath::Max3(FMath::Abs(Value.X), FMath::Abs(Value.Y), FMath::Abs(Value.Z)) > 10000)
						return Fail(Reason, TEXT("Render position/count exceeds stadium bound."));
					Out.Positions.Add(Value);
				}
				else
				{
					if (Out.Normals.Num() >= RenderNormals || FMath::Abs(Value.SizeSquared() - 1.0) > 1.e-10)
						return Fail(Reason, TEXT("Invalid supplied unit normal or normal count."));
					Out.Normals.Add(Value);
				}
			}
			else if (Words[0] == TEXT("g"))
			{
				if (Words.Num() != 2 || Words[1].Len() > 128) return Fail(Reason, TEXT("Invalid analytic render role."));
				Role = Out.Roles.IndexOfByKey(Words[1]);
				if (Role == INDEX_NONE)
				{
					if (Out.Roles.Num() >= 678) return Fail(Reason, TEXT("Too many render roles."));
					Role = Out.Roles.Add(Words[1]);
				}
			}
			else if (Words[0] == TEXT("f"))
			{
				if (Words.Num() != 4 || Role == INDEX_NONE || Out.Faces.Num() >= RenderTriangles)
					return Fail(Reason, TEXT("Missing role, nontriangle, or excess render faces."));
				FRenderFace Face; Face.Role = Role;
				for (int32 J = 0; J < 3; ++J)
				{
					FString V, N;
					if (!Words[J + 1].Split(TEXT("//"), &V, &N) || !RenderIndex(V, Out.Positions.Num(), Face.V[J]) ||
						!RenderIndex(N, Out.Normals.Num(), Face.N[J])) return Fail(Reason, TEXT("Invalid position//normal corner index."));
				}
				if (Face.V[0] == Face.V[1] || Face.V[0] == Face.V[2] || Face.V[1] == Face.V[2])
					return Fail(Reason, TEXT("Repeated triangle position."));
				Out.Faces.Add(Face); bFaces = true;
			}
			else return Fail(Reason, TEXT("Unsupported render directive; materials/UVs cannot be inferred."));
		}
		return !Out.Faces.IsEmpty() || Fail(Reason, TEXT("No render triangles."));
	}
	bool DescribeRender(const FRenderInput& Input, FMeshDescription& Mesh, double& MaxPositionCastL1, FString& Reason)
	{
		FStaticMeshAttributes Attr(Mesh); Attr.Register();
		auto Positions = Attr.GetVertexPositions(); auto Normals = Attr.GetVertexInstanceNormals();
		auto Tangents = Attr.GetVertexInstanceTangents(); auto Signs = Attr.GetVertexInstanceBinormalSigns();
		auto Colors = Attr.GetVertexInstanceColors(); auto UV = Attr.GetVertexInstanceUVs(); UV.SetNumChannels(1);
		TArray<FVertexID> Vertices; TArray<FPolygonGroupID> Groups;
		MaxPositionCastL1 = 0;
		for (const auto& Position : Input.Positions)
		{
			const auto Id = Mesh.CreateVertex(); Positions[Id] = FVector3f(Position); Vertices.Add(Id);
			const FVector3d Delta = FVector3d(Positions[Id]) - Position;
			MaxPositionCastL1 = FMath::Max(MaxPositionCastL1, FMath::Abs(Delta.X) + FMath::Abs(Delta.Y) + FMath::Abs(Delta.Z));
		}
		for (const FString& Role : Input.Roles)
		{
			const auto Group = Mesh.CreatePolygonGroup(); Groups.Add(Group);
			// Roles stay distinct polygon groups, all mapped to one unassigned slot.
			// A Wavefront `g` is not evidence of a material assignment.
			Attr.GetPolygonGroupMaterialSlotNames()[Group] = FName(TEXT("Unassigned"));
		}
		for (const auto& Face : Input.Faces)
		{
			const FVector3d A(Positions[Vertices[Face.V[0]]]), B(Positions[Vertices[Face.V[1]]]), C(Positions[Vertices[Face.V[2]]]);
			const FVector3d Cross = FVector3d::CrossProduct(B - A, C - A);
			const FVector3d N = Input.Normals[Face.N[0]] + Input.Normals[Face.N[1]] + Input.Normals[Face.N[2]];
			if (Cross.ContainsNaN() || Cross.SizeSquared() == 0 || FVector3d::DotProduct(Cross, N) <= 0)
				return Fail(Reason, TEXT("Float32 render triangle collapsed or changed inward orientation."));
			FVertexInstanceID Corners[3]; int32 Corner = 0;
			// UE uses Cross(C-A,B-A); permute complete corner pairs, not normals.
			for (int32 J : { 0, 2, 1 })
			{
				const auto Id = Mesh.CreateVertexInstance(Vertices[Face.V[J]]); Corners[Corner++] = Id;
				Normals[Id] = FVector3f(Input.Normals[Face.N[J]]);
				const FVector3f Axis = FMath::Abs(Normals[Id].Z) < .9f ? FVector3f(0, 0, 1) : FVector3f(0, 1, 0);
				Tangents[Id] = FVector3f::CrossProduct(Axis, Normals[Id]).GetSafeNormal();
				Signs[Id] = 1.f; Colors[Id] = FVector4f(1, 1, 1, 1);
				UV.Set(Id, 0, FVector2f::ZeroVector); // Structural placeholder, not an authored UV chart.
			}
			Mesh.CreateTriangle(Groups[Face.Role], MakeArrayView(Corners));
		}
		return true;
	}
}

UStaticMesh* USpeedAnalyticAuthoredImport::StageStadiumRenderV1(FString& OutManifestJson, FString& OutReason)
{
	using namespace SpeedAuthoredImport;
	OutManifestJson.Reset(); OutReason.Reset();
	if (!IsInGameThread()) { OutReason = TEXT("Render staging requires the editor game thread."); return nullptr; }
	FString Text; FRenderInput Input;
	if (!ReadPinnedAscii(RenderPin, Text, OutReason) || !ParseRender(Text, Input, OutReason)) return nullptr;
	if (Input.Positions.Num() != RenderPositions || Input.Normals.Num() != RenderNormals || Input.Faces.Num() != RenderTriangles)
	{ OutReason = TEXT("Pinned derived render counts changed."); return nullptr; }
	FMeshDescription Description; double MaxPositionCastL1;
	if (!DescribeRender(Input, Description, MaxPositionCastL1, OutReason)) return nullptr;
	TStrongObjectPtr<UStaticMesh> Candidate(NewObject<UStaticMesh>(GetTransientPackage(), NAME_None, RF_Transient));
	Candidate->GetStaticMaterials().Add(FStaticMaterial(nullptr, FName(TEXT("Unassigned")), FName(TEXT("Unassigned"))));
	Candidate->MarkAsNotHavingNavigationData();
	Candidate->CreateBodySetup();
	Candidate->GetBodySetup()->CollisionTraceFlag = CTF_UseSimpleAsComplex;
	Candidate->GetBodySetup()->bNeverNeedsCookedCollisionData = true;
	Candidate->GetBodySetup()->DefaultInstance.SetCollisionEnabled(ECollisionEnabled::NoCollision);
	UStaticMesh::FBuildMeshDescriptionsParams Params;
	Params.bFastBuild = true; Params.bCommitMeshDescription = true; Params.bMarkPackageDirty = false;
	Params.bBuildSimpleCollision = false; Params.bAllowCpuAccess = true;
	UStaticMesh::FBuildMeshDescriptionsLODParams LODParams; LODParams.bUseHighPrecisionTangentBasis = true;
	Params.PerLODOverrides.Add(LODParams);
	if (!Candidate->BuildFromMeshDescriptions({ &Description }, Params))
	{ OutReason = TEXT("Could not build transient render data."); return nullptr; }
	// The Engine fast path sets section collision=true even without simple collision.
	for (int32 I = 0; I < Candidate->GetRenderData()->LODResources[0].Sections.Num(); ++I)
	{
		Candidate->GetRenderData()->LODResources[0].Sections[I].bEnableCollision = false;
		Candidate->GetRenderData()->LODResources[0].Sections[I].MaterialIndex = 0;
		FMeshSectionInfo Info = Candidate->GetSectionInfoMap().Get(0, I); Info.bEnableCollision = false; Info.MaterialIndex = 0;
		Candidate->GetSectionInfoMap().Set(0, I, Info); Candidate->GetOriginalSectionInfoMap().Set(0, I, Info);
	}
	auto& BuildSettings = Candidate->GetSourceModel(0).BuildSettings;
	BuildSettings.bRecomputeNormals = false; BuildSettings.bRecomputeTangents = false;
	BuildSettings.bGenerateLightmapUVs = false;
	if (Candidate->GetNumTriangles(0) != RenderTriangles || !Candidate->GetMeshDescription(0) ||
		Candidate->GetMeshDescription(0)->Vertices().Num() != RenderPositions || Candidate->GetBodySetup()->AggGeom.GetElementCount() != 0)
	{ OutReason = TEXT("Built render counts or empty collision contract changed."); return nullptr; }
	const auto Manifest = MakeShared<FJsonObject>();
	Manifest->SetStringField(TEXT("schema"), TEXT("skyleague.analytic-derived-render-staging/v1"));
	Manifest->SetStringField(TEXT("render_sha256"), RenderPin.Sha256); Manifest->SetStringField(TEXT("recipe_sha256"), RecipePin.Sha256);
	Manifest->SetStringField(TEXT("frame"), Frame); Manifest->SetStringField(TEXT("units"), TEXT("cm"));
	Manifest->SetNumberField(TEXT("shared_description_positions"), RenderPositions);
	Manifest->SetNumberField(TEXT("triangles"), RenderTriangles);
	Manifest->SetNumberField(TEXT("description_corner_instances"), Description.VertexInstances().Num());
	Manifest->SetNumberField(TEXT("maximum_binary64_to_float32_position_L1_cm"), MaxPositionCastL1);
	Manifest->SetStringField(TEXT("winding"), TEXT("input corners 0,2,1; supplied normals retained"));
	FValues Roles;
	for (int32 I = 0; I < Input.Roles.Num(); ++I)
	{
		const auto Row = MakeShared<FJsonObject>(); Row->SetNumberField(TEXT("polygon_group"), I);
		Row->SetStringField(TEXT("analytic_role"), Input.Roles[I]); Roles.Add(MakeShared<FJsonValueObject>(Row));
	}
	Manifest->SetArrayField(TEXT("roles"), Roles);
	Manifest->SetBoolField(TEXT("materials_assigned"), false); Manifest->SetBoolField(TEXT("UV_bound"), false);
	Manifest->SetBoolField(TEXT("collision_enabled"), false); Manifest->SetBoolField(TEXT("physics_source_modified"), false);
	Manifest->SetBoolField(TEXT("component_bound"), false); Manifest->SetBoolField(TEXT("saved"), false);
	Manifest->SetStringField(TEXT("render_normal_quantization_bound"), TEXT("NOT_ESTABLISHED"));
	Manifest->SetStringField(TEXT("tangent_basis"), TEXT("geometric placeholder; not UV tangent space"));
	if (!FJsonSerializer::Serialize(Manifest, TJsonWriterFactory<>::Create(&OutManifestJson)))
	{ OutManifestJson.Reset(); OutReason = TEXT("Could not serialize render staging manifest."); return nullptr; }
	OutReason = TEXT("Transient render candidate only; no materials, UV chart, source binding, save or physics admission.");
	return Candidate.Get();
}

#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSpeedRenderCornerContractTest,
	"IAmSpeed.AnalyticImport.RenderCornerContract", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FSpeedRenderCornerContractTest::RunTest(const FString&)
{
	using namespace SpeedAuthoredImport;
	const FString Fixture = TEXT("v 0 0 0\nv 1 0 0\nv 0 1 0\nv -1 0 0\nvn 0 0 1\nvn 0 0.6 0.8\ng floor\nf 1//1 2//1 3//2\ng collar\nf 1//1 3//2 4//1\n");
	FRenderInput Input; FString Reason; FMeshDescription Mesh; double Error;
	if (!TestTrue(TEXT("fixture parses"), ParseRender(Fixture, Input, Reason)) ||
		!TestTrue(TEXT("description forms"), DescribeRender(Input, Mesh, Error, Reason))) return false;
	FStaticMeshAttributes Attr(Mesh);
	TestEqual(TEXT("shared seam IDs survive"), Mesh.Vertices().Num(), 4);
	TestEqual(TEXT("corner attributes remain separate"), Mesh.VertexInstances().Num(), 6);
	const auto Corners = Mesh.GetTriangleVertexInstances(FTriangleID(0));
	TestEqual(TEXT("UE middle corner keeps paired position"), Mesh.GetVertexInstanceVertex(Corners[1]).GetValue(), 2);
	TestTrue(TEXT("UE middle corner keeps its analytic normal"), Attr.GetVertexInstanceNormals()[Corners[1]] == FVector3f(0, .6f, .8f));
	TestEqual(TEXT("role groups retained"), Mesh.PolygonGroups().Num(), 2);
	for (const FString& Bad : { Fixture + TEXT("usemtl imaginary\n"), Fixture + TEXT("f -1//1 2//1 3//1\n"),
		Fixture + TEXT("f 1//1 2//1 3//99\n"), Fixture + TEXT("f 1//1 1//1 3//1\n"), Fixture + TEXT("vt 0 0\n") })
		TestFalse(TEXT("unsupported or malformed additions fail"), ParseRender(Bad, Input, Reason));
	TestFalse(TEXT("nonfinite position rejected"), ParseRender(TEXT("v nan 0 0\n"), Input, Reason));
	double Number;
	for (const FString& BadNumber : { TEXT("1x"), TEXT("1e"), TEXT("1e+"), TEXT("."), TEXT("+"), TEXT("1,5"), TEXT("0x1p0"), TEXT("1e9999") })
		TestFalse(TEXT("number grammar and finite check"), RenderNumber(BadNumber, Number));
	TestTrue(TEXT("complete exponent supported"), RenderNumber(TEXT("-1.25e+2"), Number) && Number == -125.0);
	TestTrue(TEXT("fraction parses to pinned bits"), RenderNumber(TEXT("0.1"), Number) && DoubleHex(Number) == TEXT("3fb999999999999a"));
	TestTrue(TEXT("underflow fixture parses"), ParseRender(TEXT("v 0 0 0\nv 1e-46 0 0\nv 0 1e-46 0\nvn 0 0 1\ng floor\nf 1//1 2//1 3//1\n"), Input, Reason));
	FMeshDescription Collapsed;
	TestFalse(TEXT("float32 collapse is rejected"), DescribeRender(Input, Collapsed, Error, Reason));
	return !HasAnyErrors();
}

#if PLATFORM_WINDOWS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSpeedRenderPinnedStadiumTest,
	"IAmSpeed.AnalyticImport.PinnedStadiumRender", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FSpeedRenderPinnedStadiumTest::RunTest(const FString&)
{
	FString Manifest, Reason;
	TStrongObjectPtr<UStaticMesh> Mesh(USpeedAnalyticAuthoredImport::StageStadiumRenderV1(Manifest, Reason));
	if (!Mesh.IsValid()) { AddError(Reason); return false; }
	TestTrue(TEXT("transient only"), Mesh->HasAnyFlags(RF_Transient) && !Mesh->HasAnyFlags(RF_Public | RF_Standalone));
	TestEqual(TEXT("actual derived triangles"), Mesh->GetNumTriangles(0), SpeedAuthoredImport::RenderTriangles);
	TestEqual(TEXT("one structural unassigned slot"), Mesh->GetStaticMaterials().Num(), 1);
	TestNull(TEXT("no material asset fabricated"), Mesh->GetStaticMaterials()[0].MaterialInterface.Get());
	TestEqual(TEXT("no collision shapes"), Mesh->GetBodySetup()->AggGeom.GetElementCount(), 0);
	TestTrue(TEXT("default body collision disabled"), Mesh->GetBodySetup()->DefaultInstance.GetCollisionEnabled() == ECollisionEnabled::NoCollision);
	for (int32 I = 0; I < Mesh->GetRenderData()->LODResources[0].Sections.Num(); ++I)
	{
		TestFalse(TEXT("render section collision disabled"), Mesh->GetRenderData()->LODResources[0].Sections[I].bEnableCollision);
		TestFalse(TEXT("saved section collision disabled"), Mesh->GetSectionInfoMap().Get(0, I).bEnableCollision);
	}
	return !HasAnyErrors();
}
#endif
#endif
