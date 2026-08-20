#include "SpeedAnalyticBakeCommandlet.h"

#include "Algo/Sort.h"
#include "AssetRegistry/AssetRegistryModule.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/Level.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "Editor.h"
#include "FileHelpers.h"
#include "IAmSpeed/Actors/SpeedStaticActor.h"
#include "IAmSpeed/World/Analytic/AnalyticWorldData.h"
#include "IAmSpeed/World/Analytic/AnalyticLandscapeAdapter.h"
#include "IAmSpeed/World/Analytic/SpeedAnalyticCollisionAsset.h"
#include "IAmSpeed/World/Analytic/SpeedAnalyticSourceComponent.h"
#include "MeshDescription.h"
#include "Misc/PackageName.h"
#include "PhysicsEngine/BodySetup.h"
#include "LandscapeProxy.h"
#include "StaticMeshAttributes.h"
#include "UObject/Package.h"
#include "UObject/SavePackage.h"

namespace
{
	constexpr uint64 BakeFnvOffset = 14695981039346656037ull;
	constexpr uint64 BakeFnvPrime = 1099511628211ull;

	void HashBytes(uint64& Hash, const void* Data, const SIZE_T Size)
	{
		const uint8* Bytes = static_cast<const uint8*>(Data);
		for (SIZE_T Index = 0; Index < Size; ++Index)
		{
			Hash ^= Bytes[Index];
			Hash *= BakeFnvPrime;
		}
	}

	template <typename T>
	void HashValue(uint64& Hash, const T& Value)
	{
		HashBytes(Hash, &Value, sizeof(T));
	}

	void HashVector(uint64& Hash, const FVector3d& Value)
	{
		HashValue(Hash, Value.X);
		HashValue(Hash, Value.Y);
		HashValue(Hash, Value.Z);
	}

	void HashVector(uint64& Hash, const FVector3f& Value)
	{
		HashValue(Hash, Value.X);
		HashValue(Hash, Value.Y);
		HashValue(Hash, Value.Z);
	}

	bool VectorLess(const FVector3d& A, const FVector3d& B)
	{
		if (A.X != B.X) return A.X < B.X;
		if (A.Y != B.Y) return A.Y < B.Y;
		return A.Z < B.Z;
	}

	struct FImportedTriangle
	{
		FVector3d Vertices[3];
		FVector3f Normals[3];
		uint64 MaterialSlotId = 0;
	};

	void RotateCanonical(FImportedTriangle& Triangle)
	{
		int32 First = 0;
		if (VectorLess(Triangle.Vertices[1], Triangle.Vertices[First])) First = 1;
		if (VectorLess(Triangle.Vertices[2], Triangle.Vertices[First])) First = 2;
		if (First == 0)
		{
			return;
		}
		FImportedTriangle Copy = Triangle;
		for (int32 Index = 0; Index < 3; ++Index)
		{
			Triangle.Vertices[Index] = Copy.Vertices[(First + Index) % 3];
			Triangle.Normals[Index] = Copy.Normals[(First + Index) % 3];
		}
	}

	bool TriangleLess(const FImportedTriangle& A, const FImportedTriangle& B)
	{
		for (int32 Index = 0; Index < 3; ++Index)
		{
			if (VectorLess(A.Vertices[Index], B.Vertices[Index])) return true;
			if (VectorLess(B.Vertices[Index], A.Vertices[Index])) return false;
		}
		return A.MaterialSlotId < B.MaterialSlotId;
	}

	void CaptureCollisionPolicy(
		const UStaticMeshComponent& Component,
		FSpeedAnalyticMeshSourceRecord& Record)
	{
		Record.ObjectType = static_cast<uint32>(Component.GetCollisionObjectType());
		Record.CollisionEnabled = static_cast<uint8>(Component.GetCollisionEnabled());
		Record.bQueryCollisionEnabled = Component.IsQueryCollisionEnabled();
		Record.BlockingChannels = 0;
		Record.OverlapChannels = 0;
		for (int32 Channel = 0; Channel < static_cast<int32>(ECC_MAX) && Channel < 64;
			++Channel)
		{
			const ECollisionResponse Response = Component.GetCollisionResponseToChannel(
				static_cast<ECollisionChannel>(Channel));
			if (Response == ECR_Block) Record.BlockingChannels |= 1ull << Channel;
			if (Response == ECR_Overlap) Record.OverlapChannels |= 1ull << Channel;
		}
		if (const UBodySetup* BodySetup = Component.GetStaticMesh()->GetBodySetup())
		{
			Record.CollisionTraceFlag = static_cast<uint8>(BodySetup->CollisionTraceFlag);
		}
	}

	uint64 HashCollisionPolicy(const FSpeedAnalyticMeshSourceRecord& Record)
	{
		uint64 Hash = Speed::Analytic::CombineStableIds(0, Record.ObjectType);
		Hash = Speed::Analytic::CombineStableIds(Hash, Record.CollisionEnabled);
		Hash = Speed::Analytic::CombineStableIds(Hash, Record.BlockingChannels);
		Hash = Speed::Analytic::CombineStableIds(Hash, Record.OverlapChannels);
		Hash = Speed::Analytic::CombineStableIds(Hash, Record.CollisionTraceFlag);
		return Hash;
	}

	bool ImportMeshSource(
		const AActor& Actor,
		const UStaticMeshComponent& Component,
		FSpeedAnalyticMeshSourceRecord& OutRecord,
		TArray<FSpeedAnalyticTriangleRecord>& OutTriangles,
		FString& OutError)
	{
		const UStaticMesh* Mesh = Component.GetStaticMesh();
		const FMeshDescription* Description = Mesh ? Mesh->GetMeshDescription(0) : nullptr;
		if (!Description || Description->Triangles().Num() == 0)
		{
			OutError = FString::Printf(TEXT("Mesh %s has no editable LOD0 triangles."),
				Mesh ? *Mesh->GetPathName() : TEXT("<null>"));
			return false;
		}

		OutRecord.ActorPath = Actor.GetPathName();
		OutRecord.ComponentPath = Component.GetPathName();
		OutRecord.MeshPath = Mesh->GetPathName();
		OutRecord.SourceId = Speed::Analytic::StableStringId(OutRecord.ComponentPath);
		OutRecord.MeshId = Speed::Analytic::StableStringId(OutRecord.MeshPath);
		OutRecord.WorldTransform = Component.GetComponentTransform();
		CaptureCollisionPolicy(Component, OutRecord);

		for (int32 MaterialIndex = 0; MaterialIndex < Component.GetNumMaterials();
			++MaterialIndex)
		{
			const UMaterialInterface* Material = Component.GetMaterial(MaterialIndex);
			OutRecord.MaterialIds.Add(Speed::Analytic::StableStringId(
				Material ? Material->GetPathName() : FString::Printf(
					TEXT("<null:%d>"), MaterialIndex)));
		}
		OutRecord.MaterialHash = BakeFnvOffset;
		for (const uint64 MaterialId : OutRecord.MaterialIds)
		{
			HashValue(OutRecord.MaterialHash, MaterialId);
		}

		const FStaticMeshConstAttributes Attributes(*Description);
		const TVertexAttributesConstRef<FVector3f> Positions =
			Attributes.GetVertexPositions();
		const TVertexInstanceAttributesConstRef<FVector3f> Normals =
			Attributes.GetVertexInstanceNormals();
		const TPolygonGroupAttributesConstRef<FName> MaterialSlotNames =
			Attributes.GetPolygonGroupMaterialSlotNames();
		TArray<FImportedTriangle> Triangles;
		Triangles.Reserve(Description->Triangles().Num());
		const FTransform3d WorldTransform(Component.GetComponentTransform());
		for (const FTriangleID TriangleId : Description->Triangles().GetElementIDs())
		{
			FImportedTriangle& Triangle = Triangles.AddDefaulted_GetRef();
			const TArrayView<const FVertexInstanceID> Instances =
				Description->GetTriangleVertexInstances(TriangleId);
			for (int32 Corner = 0; Corner < 3; ++Corner)
			{
				const FVertexInstanceID InstanceId = Instances[Corner];
				const FVertexID VertexId =
					Description->GetVertexInstanceVertex(InstanceId);
				Triangle.Vertices[Corner] = WorldTransform.TransformPosition(
					FVector3d(Positions[VertexId]));
				Triangle.Normals[Corner] = FVector3f(
					WorldTransform.TransformVectorNoScale(
						FVector3d(Normals[InstanceId])).GetSafeNormal());
				OutRecord.WorldBounds += FVector(Triangle.Vertices[Corner]);
			}
			Triangle.MaterialSlotId = Speed::Analytic::StableStringId(
				MaterialSlotNames[Description->GetTrianglePolygonGroup(TriangleId)].ToString());
			const FVector3d Cross = FVector3d::CrossProduct(
				Triangle.Vertices[1] - Triangle.Vertices[0],
				Triangle.Vertices[2] - Triangle.Vertices[0]);
			if (Cross.SquaredLength() <= 1.0e-18)
			{
				++OutRecord.DegenerateTriangleCount;
			}
			RotateCanonical(Triangle);
		}
		Algo::Sort(Triangles, TriangleLess);
		OutRecord.TriangleCount = Triangles.Num();
		OutRecord.TriangleHash = BakeFnvOffset;
		OutRecord.NormalHash = BakeFnvOffset;
		for (const FImportedTriangle& Triangle : Triangles)
		{
			for (int32 Corner = 0; Corner < 3; ++Corner)
			{
				HashVector(OutRecord.TriangleHash, Triangle.Vertices[Corner]);
				HashVector(OutRecord.NormalHash, Triangle.Normals[Corner]);
			}
			HashValue(OutRecord.TriangleHash, Triangle.MaterialSlotId);
		}
		const uint64 SurfaceId = Speed::Analytic::StableStringId(Actor.GetPathName());
		const uint64 FeatureId = Speed::Analytic::CombineStableIds(
			SurfaceId, static_cast<uint64>(Speed::Analytic::EFeatureKind::Interior) + 1ull);
		OutTriangles.Reserve(OutTriangles.Num() + Triangles.Num());
		for (int32 TriangleIndex = 0; TriangleIndex < Triangles.Num(); ++TriangleIndex)
		{
			const FImportedTriangle& Imported = Triangles[TriangleIndex];
			FSpeedAnalyticTriangleRecord& Record = OutTriangles.AddDefaulted_GetRef();
			Record.SourceId = OutRecord.SourceId;
			Record.SurfaceId = SurfaceId;
			Record.FeatureId = FeatureId;
			Record.PrimitiveId = Speed::Analytic::CombineStableIds(
				OutRecord.SourceId, static_cast<uint64>(TriangleIndex) + 1ull);
			Record.MaterialId = static_cast<uint32>(
				Imported.MaterialSlotId ^ (Imported.MaterialSlotId >> 32));
			if (Record.MaterialId == 0) Record.MaterialId = 1;
			Record.Vertex0 = FVector(Imported.Vertices[0]);
			Record.Vertex1 = FVector(Imported.Vertices[1]);
			Record.Vertex2 = FVector(Imported.Vertices[2]);
			Record.Normal0 = FVector(Imported.Normals[0]);
			Record.Normal1 = FVector(Imported.Normals[1]);
			Record.Normal2 = FVector(Imported.Normals[2]);
		}
		OutRecord.CollisionPolicyHash = HashCollisionPolicy(OutRecord);
		return true;
	}

	FVector CanonicalPoolVector(FVector Value)
	{
		if (Value.X == 0.0) Value.X = 0.0;
		if (Value.Y == 0.0) Value.Y = 0.0;
		if (Value.Z == 0.0) Value.Z = 0.0;
		return Value;
	}

	int32 FindOrAddPoolValue(
		const FVector& Value, TArray<FVector>& Pool, TMap<FVector, int32>& Indices)
	{
		const FVector Canonical = CanonicalPoolVector(Value);
		if (const int32* Existing = Indices.Find(Canonical))
		{
			return *Existing;
		}
		const int32 Index = Pool.Add(Canonical);
		Indices.Add(Canonical, Index);
		return Index;
	}

	void BuildIndexedPayload(
		const TArray<FSpeedAnalyticTriangleRecord>& Expanded,
		TArray<FVector>& OutPositions,
		TArray<FVector>& OutNormals,
		TArray<FSpeedAnalyticIndexedTriangleRecord>& OutTriangles)
	{
		OutPositions.Reset();
		OutNormals.Reset();
		OutTriangles.Reset();
		OutTriangles.Reserve(Expanded.Num());
		TMap<FVector, int32> PositionIndices;
		TMap<FVector, int32> NormalIndices;
		PositionIndices.Reserve(Expanded.Num());
		NormalIndices.Reserve(Expanded.Num());
		for (const FSpeedAnalyticTriangleRecord& Source : Expanded)
		{
			FSpeedAnalyticIndexedTriangleRecord& Target =
				OutTriangles.AddDefaulted_GetRef();
			Target.SourceId = Source.SourceId;
			Target.SurfaceId = Source.SurfaceId;
			Target.FeatureId = Source.FeatureId;
			Target.PrimitiveId = Source.PrimitiveId;
			Target.MaterialId = Source.MaterialId;
			const FVector Positions[3] = {
				Source.Vertex0, Source.Vertex1, Source.Vertex2 };
			const FVector Normals[3] = {
				Source.Normal0, Source.Normal1, Source.Normal2 };
			for (int32 Corner = 0; Corner < 3; ++Corner)
			{
				Target.VertexIndices[Corner] = FindOrAddPoolValue(
					Positions[Corner], OutPositions, PositionIndices);
				Target.NormalIndices[Corner] = FindOrAddPoolValue(
					Normals[Corner], OutNormals, NormalIndices);
			}
		}
	}
}

USpeedAnalyticBakeCommandlet::USpeedAnalyticBakeCommandlet()
{
	IsClient = false;
	IsServer = false;
	LogToConsole = true;
}

int32 USpeedAnalyticBakeCommandlet::Main(const FString& Params)
{
	TArray<FString> Tokens;
	TArray<FString> Switches;
	TMap<FString, FString> Named;
	ParseCommandLine(*Params, Tokens, Switches, Named);
	const FString OutputPackageName = Named.FindRef(TEXT("Output"));
	if (!FPackageName::IsValidLongPackageName(OutputPackageName))
	{
		UE_LOG(LogTemp, Error, TEXT("-Output=/Game/... is required."));
		return 1;
	}

	if (Switches.Contains(TEXT("ValidateOnly")))
	{
		const FString AssetName = FPackageName::GetLongPackageAssetName(OutputPackageName);
		const FString ObjectPath = OutputPackageName + TEXT(".") + AssetName;
		const USpeedAnalyticCollisionAsset* Existing =
			LoadObject<USpeedAnalyticCollisionAsset>(nullptr, *ObjectPath);
		FString Reason;
		const TSharedPtr<const Speed::Analytic::FAnalyticWorldData> Runtime =
			Existing ? Existing->BuildRuntimeData(&Reason, true) : nullptr;
		if (!Existing || !Runtime)
		{
			UE_LOG(LogTemp, Error, TEXT("[AnalyticBakeValidation] Asset=%s Result=Invalid Detail=%s"),
				*ObjectPath, Existing ? *Reason : TEXT("Asset not found."));
			return 2;
		}
		int32 BoundaryEdgeCount = 0;
		int32 NonManifoldEdgeCount = 0;
		int32 SmoothEdgeCount = 0;
		int32 CreaseEdgeCount = 0;
		int32 PlanarCandidateCount = 0;
		int32 ValidShapeSampleCount = 0;
		int32 FlatShapeSampleCount = 0;
		int32 DevelopableShapeSampleCount = 0;
		int32 EllipticShapeSampleCount = 0;
		int32 SaddleShapeSampleCount = 0;
		int32 PoorShapeFitCount = 0;
		int32 ReciprocalPlanarMirrorCount = 0;
		int32 ArchitecturalPlanarConstraintCount = 0;
		int32 ReciprocalCurvatureMirrorCount = 0;
		int32 PlausibleCurvatureMirrorCount = 0;
		int32 PlausibleCurvatureSelfMirrorCount = 0;
		int32 PlanarConstraintTriangleCount = 0;
		int32 DevelopableTriangleCount = 0;
		int32 EllipticTriangleCount = 0;
		int32 SaddleTriangleCount = 0;
		int32 UnresolvedCurvatureTriangleCount = 0;
		int32 ExtrusionTriangleCandidateCount = 0;
		int32 PlausibleQuarterEllipseCount = 0;
		int32 ReciprocalExtrusionMirrorCount = 0;
		int32 PlausibleExtrusionMirrorCount = 0;
		int32 PlausibleExtrusionSelfMirrorCount = 0;
		int32 EllipseBoundaryC0Count = 0;
		int32 EllipseBoundaryG1Count = 0;
		int32 UsableC2TransitionBoundaryCount = 0;
		int32 UsableSharedC2PairCount = 0;
		int32 PlausibleSharedC2PairCount = 0;
		int32 PlausibleSharedC2FrameCount = 0;
		int32 CompatibleGlobalC2PlaneCount = 0;
		int32 PlausibleGlobalC2JoinCount = 0;
		int32 PlausibleSymmetrizedC2PlaneCount = 0;
		int32 ExactSymmetrizedC2PlaneCount = 0;
		int32 PlausibleCoupledC2SolutionCount = 0;
		int32 PlausibleCoupledC2FamilyCount = 0;
		int32 CoveredC2TransitionCount = 0;
		int32 SelfMirrorC2TransitionCount = 0;
		int32 DiagonalC2TransitionCount = 0;
		int32 UnpairedAxisC2TransitionCount = 0;
		int32 PlayableInnerC2TransitionCount = 0;
		int32 OuterBackingC2TransitionCount = 0;
		int32 CompletePlayableC2OrbitCount = 0;
		int32 TopologyPlausiblePlayableC2OrbitCount = 0;
		int32 SourcePlausiblePlayableC2OrbitCount = 0;
		int32 CompletePlayableC2TriangleSupportCount = 0;
		int32 PlausiblePlayableC2TriangleSupportCount = 0;
		int32 SourcePlausiblePlayableC2TriangleSupportCount = 0;
		int32 NetworkPlausiblePlayableC2TriangleSupportCount = 0;
		int32 CompatiblePlayableC2PlaneBindingCount = 0;
		int32 SameSourcePlayableC2PlaneBindingCount = 0;
		int32 CompatiblePlayableC2NetworkBindingCount = 0;
		int32 SyntheticPlayableC2NetworkPlaneCount = 0;
		int32 ClosedEndWallBoundaryComponentCount = 0;
		int32 CentralEndWallBoundaryComponentCount = 0;
		int32 PlayableOpenRimCount = 0;
		int32 OuterOpenRimCount = 0;
		int32 OpenCentralOpenRimCount = 0;
		int32 MirrorPlausibleOpenRimCount = 0;
		int32 SelfPlausibleOpenRimCount = 0;
		int32 PartitionedOpenRimCount = 0;
		int32 PlausibleCanonicalOpenArchCount = 0;
		int32 OpeningSurfaceBandCount[4] = { 0, 0, 0, 0 };
		int32 OpeningSurfaceBandSameSmoothCount[4] = { 0, 0, 0, 0 };
		int32 OpeningSurfaceBandG1Count[4] = { 0, 0, 0, 0 };
		double OpeningSurfaceBandMaximumDihedral[4] = { 0.0, 0.0, 0.0, 0.0 };
		double OpeningSurfaceBandMaximumNormalJump[4] = { 0.0, 0.0, 0.0, 0.0 };
		double OpeningSurfaceBandMaximumLongitudinalStep[4] = { 0.0, 0.0, 0.0, 0.0 };
		for (const Speed::Analytic::FTriangleMeshEdge& Edge : Runtime->MeshEdges)
		{
			BoundaryEdgeCount += Edge.IsBoundary() ? 1 : 0;
			NonManifoldEdgeCount += Edge.IsNonManifold() ? 1 : 0;
			SmoothEdgeCount += Edge.Continuity ==
				Speed::Analytic::EEdgeContinuity::Smooth ? 1 : 0;
			CreaseEdgeCount += Edge.Continuity ==
				Speed::Analytic::EEdgeContinuity::Crease ? 1 : 0;
		}
		for (const Speed::Analytic::FSurfacePatch& Patch : Runtime->SurfacePatches)
		{
			PlanarCandidateCount += Patch.Kind ==
				Speed::Analytic::ESurfacePatchKind::PlanarCandidate ? 1 : 0;
		}
		for (const Speed::Analytic::FVertexShapeSample& Sample : Runtime->VertexShapeSamples)
		{
			if (!Sample.bValid) continue;
			++ValidShapeSampleCount;
			const double MinimumAbs = FMath::Min(
				FMath::Abs(Sample.MinimumPrincipalCurvature),
				FMath::Abs(Sample.MaximumPrincipalCurvature));
			const double MaximumAbs = FMath::Max(
				FMath::Abs(Sample.MinimumPrincipalCurvature),
				FMath::Abs(Sample.MaximumPrincipalCurvature));
			if (MaximumAbs <= 1.0e-6)
			{
				++FlatShapeSampleCount;
			}
			else if (MinimumAbs <= 0.1 * MaximumAbs)
			{
				++DevelopableShapeSampleCount;
			}
			else if (Sample.GaussianCurvature > 0.0)
			{
				++EllipticShapeSampleCount;
			}
			else
			{
				++SaddleShapeSampleCount;
			}
			PoorShapeFitCount += Sample.RootMeanSquareResidual >
				0.25 * MaximumAbs + 1.0e-8 ? 1 : 0;
		}
		for (const Speed::Analytic::FPlanarGroupMirrorMatch& Match :
			Runtime->PlanarGroupMirrorMatches)
		{
			ReciprocalPlanarMirrorCount += Match.bReciprocal ? 1 : 0;
		}
		for (const Speed::Analytic::FPlanarSurfaceGroup& Group :
			Runtime->PlanarSurfaceGroups)
		{
			ArchitecturalPlanarConstraintCount +=
				Group.bArchitecturalConstraint ? 1 : 0;
		}
		for (const Speed::Analytic::FCurvatureRegionMirrorMatch& Match :
			Runtime->CurvatureRegionMirrorMatches)
		{
			ReciprocalCurvatureMirrorCount += Match.bReciprocal ? 1 : 0;
			PlausibleCurvatureMirrorCount += Match.bPlausible ? 1 : 0;
			PlausibleCurvatureSelfMirrorCount +=
				Match.bPlausible && Match.bSelfMirror ? 1 : 0;
		}
		for (const Speed::Analytic::FTriangleCurvatureEvidence& Evidence :
			Runtime->TriangleCurvatureEvidence)
		{
			switch (Evidence.Kind)
			{
			case Speed::Analytic::ECurvatureEvidenceKind::PlanarConstraint:
				++PlanarConstraintTriangleCount;
				break;
			case Speed::Analytic::ECurvatureEvidenceKind::DevelopableCandidate:
				++DevelopableTriangleCount;
				break;
			case Speed::Analytic::ECurvatureEvidenceKind::EllipticCandidate:
				++EllipticTriangleCount;
				break;
			case Speed::Analytic::ECurvatureEvidenceKind::SaddleCandidate:
				++SaddleTriangleCount;
				break;
			default:
				++UnresolvedCurvatureTriangleCount;
				break;
			}
		}
		for (const Speed::Analytic::FTriangleExtrusionEvidence& Evidence :
			Runtime->TriangleExtrusionEvidence)
		{
			ExtrusionTriangleCandidateCount += Evidence.bCandidate ? 1 : 0;
		}
		for (const Speed::Analytic::FExtrusionSurfaceRegion& Region :
			Runtime->ExtrusionSurfaceRegions)
		{
			PlausibleQuarterEllipseCount +=
				Region.bQuarterEllipsePlausible ? 1 : 0;
		}
		for (const Speed::Analytic::FExtrusionRegionMirrorMatch& Match :
			Runtime->ExtrusionRegionMirrorMatches)
		{
			ReciprocalExtrusionMirrorCount += Match.bReciprocal ? 1 : 0;
			PlausibleExtrusionMirrorCount += Match.bPlausible ? 1 : 0;
			PlausibleExtrusionSelfMirrorCount +=
				Match.bPlausible && Match.bSelfMirror ? 1 : 0;
		}
		for (const Speed::Analytic::FQuarterEllipseBoundaryMatch& Match :
			Runtime->QuarterEllipseBoundaryMatches)
		{
			EllipseBoundaryC0Count += Match.bC0Plausible ? 1 : 0;
			EllipseBoundaryG1Count += Match.bG1Plausible ? 1 : 0;
		}
		for (const Speed::Analytic::FC2TransitionSectionFit& Fit :
			Runtime->C2TransitionSectionFits)
		{
			UsableC2TransitionBoundaryCount +=
				Fit.bBoundaryEvidenceUsable ? 1 : 0;
		}
		for (const Speed::Analytic::FSharedC2TransitionPairFit& Pair :
			Runtime->SharedC2TransitionPairFits)
		{
			UsableSharedC2PairCount += Pair.bBothBoundaryEvidenceUsable ? 1 : 0;
			PlausibleSharedC2PairCount += Pair.bPlausible ? 1 : 0;
		}
		for (const Speed::Analytic::FSharedC2FrameLedgerEntry& Entry :
			Runtime->SharedC2FrameLedger)
		{
			PlausibleSharedC2FrameCount +=
				Entry.bSourceRegularizationPlausible ? 1 : 0;
		}
		for (const Speed::Analytic::FGlobalC2PlaneConstraint& Constraint :
			Runtime->GlobalC2PlaneConstraints)
		{
			CompatibleGlobalC2PlaneCount += Constraint.bConstraintCompatible ? 1 : 0;
		}
		for (const Speed::Analytic::FGlobalC2JoinCertificate& Certificate :
			Runtime->GlobalC2JoinCertificates)
		{
			PlausibleGlobalC2JoinCount += Certificate.bLocalC2Retained ? 1 : 0;
		}
		for (const Speed::Analytic::FSymmetrizedC2PlaneConstraint& Constraint :
			Runtime->SymmetrizedC2PlaneConstraints)
		{
			PlausibleSymmetrizedC2PlaneCount +=
				Constraint.bSourceFitPlausible ? 1 : 0;
			ExactSymmetrizedC2PlaneCount +=
				Constraint.bExactMirrorPlacement ? 1 : 0;
		}
		for (const Speed::Analytic::FCoupledC2TransitionPairSolution& Solution :
			Runtime->CoupledC2TransitionSolutions)
		{
			PlausibleCoupledC2SolutionCount += Solution.bSourceFitPlausible ? 1 : 0;
		}
		for (const Speed::Analytic::FCoupledC2TransitionFamilySolution& Family :
			Runtime->CoupledC2TransitionFamilies)
		{
			PlausibleCoupledC2FamilyCount += Family.bSourceFitPlausible ? 1 : 0;
		}
		for (const Speed::Analytic::FC2TransitionCoverageEntry& Entry :
			Runtime->C2TransitionCoverage)
		{
			switch (Entry.Kind)
			{
			case Speed::Analytic::EC2TransitionCoverageKind::CoveredFamily:
				++CoveredC2TransitionCount;
				break;
			case Speed::Analytic::EC2TransitionCoverageKind::AxisAlignedSelfMirror:
				++SelfMirrorC2TransitionCount;
				break;
			case Speed::Analytic::EC2TransitionCoverageKind::DiagonalCornerCandidate:
				++DiagonalC2TransitionCount;
				break;
			case Speed::Analytic::EC2TransitionCoverageKind::UnpairedAxisAligned:
				++UnpairedAxisC2TransitionCount;
				break;
			default:
				break;
			}
			PlayableInnerC2TransitionCount += Entry.SurfaceLayer ==
				Speed::Analytic::EC2TransitionSurfaceLayer::PlayableInner ? 1 : 0;
			OuterBackingC2TransitionCount += Entry.SurfaceLayer ==
				Speed::Analytic::EC2TransitionSurfaceLayer::OuterBacking ? 1 : 0;
		}
		for (const Speed::Analytic::FPlayableC2OrbitCandidate& Candidate :
			Runtime->PlayableC2OrbitCandidates)
		{
			CompletePlayableC2OrbitCount += Candidate.bOrbitComplete ? 1 : 0;
			TopologyPlausiblePlayableC2OrbitCount +=
				Candidate.bMirrorTopologyPlausible ? 1 : 0;
			SourcePlausiblePlayableC2OrbitCount +=
				Candidate.bSourceFitPlausible ? 1 : 0;
		}
		for (const Speed::Analytic::FPlayableC2TriangleSupportCandidate& Candidate :
			Runtime->PlayableC2TriangleSupportCandidates)
		{
			CompletePlayableC2TriangleSupportCount += Candidate.bComplete ? 1 : 0;
			PlausiblePlayableC2TriangleSupportCount +=
				Candidate.bSupportPlausible ? 1 : 0;
			SourcePlausiblePlayableC2TriangleSupportCount +=
				Candidate.bSourceFitPlausible ? 1 : 0;
			NetworkPlausiblePlayableC2TriangleSupportCount +=
				Candidate.bNetworkSourceFitPlausible ? 1 : 0;
		}
		for (const Speed::Analytic::FPlayableC2PlaneBinding& Binding :
			Runtime->PlayableC2PlaneBindings)
		{
			CompatiblePlayableC2PlaneBindingCount += Binding.bCompatible ? 1 : 0;
			SameSourcePlayableC2PlaneBindingCount +=
				Binding.bSameSourcePlanarGroup ? 1 : 0;
			CompatiblePlayableC2NetworkBindingCount +=
				Binding.bNetworkCompatible ? 1 : 0;
		}
		for (const Speed::Analytic::FPlayableC2NetworkPlaneConstraint& Plane :
			Runtime->PlayableC2NetworkPlaneConstraints)
		{
			SyntheticPlayableC2NetworkPlaneCount +=
				Plane.bInheritedFromSymmetrizedNetwork ? 0 : 1;
		}
		for (const Speed::Analytic::FEndWallBoundaryComponent& Component :
			Runtime->EndWallBoundaryComponents)
		{
			ClosedEndWallBoundaryComponentCount += Component.bClosedLoop ? 1 : 0;
			CentralEndWallBoundaryComponentCount +=
				Component.bCrossesXSymmetryPlane ? 1 : 0;
		}
		for (const Speed::Analytic::FOpenRimCandidate& Candidate :
			Runtime->OpenRimCandidates)
		{
			PlayableOpenRimCount += Candidate.SurfaceLayer ==
				Speed::Analytic::EC2TransitionSurfaceLayer::PlayableInner ? 1 : 0;
			OuterOpenRimCount += Candidate.SurfaceLayer ==
				Speed::Analytic::EC2TransitionSurfaceLayer::OuterBacking ? 1 : 0;
			OpenCentralOpenRimCount += Candidate.bOpenAtBaseline &&
				Candidate.bCrossesTransverseSymmetryPlane ? 1 : 0;
			MirrorPlausibleOpenRimCount +=
				Candidate.bLongitudinalMirrorPlausible ? 1 : 0;
			SelfPlausibleOpenRimCount +=
				Candidate.bTransverseSelfPlausible ? 1 : 0;
			PartitionedOpenRimCount +=
				Candidate.bFeaturePartitionComplete ? 1 : 0;
		}
		for (const Speed::Analytic::FCanonicalOpenArchSolution& Solution :
			Runtime->CanonicalOpenArchSolutions)
		{
			PlausibleCanonicalOpenArchCount +=
				Solution.bSourceRegularizationPlausible ? 1 : 0;
		}
		for (const Speed::Analytic::FOpenRimSurfaceBandObservation& Observation :
			Runtime->OpenRimSurfaceBandObservations)
		{
			const int32 Feature = static_cast<int32>(Observation.Feature);
			++OpeningSurfaceBandCount[Feature];
			OpeningSurfaceBandSameSmoothCount[Feature] +=
				Observation.bSameSmoothRegion ? 1 : 0;
			OpeningSurfaceBandG1Count[Feature] +=
				Observation.bSourceG1Plausible ? 1 : 0;
			OpeningSurfaceBandMaximumDihedral[Feature] = FMath::Max(
				OpeningSurfaceBandMaximumDihedral[Feature],
				Observation.FaceDihedralDegrees);
			OpeningSurfaceBandMaximumNormalJump[Feature] = FMath::Max(
				OpeningSurfaceBandMaximumNormalJump[Feature],
				Observation.ImportedNormalJumpDegrees);
			OpeningSurfaceBandMaximumLongitudinalStep[Feature] = FMath::Max(
				OpeningSurfaceBandMaximumLongitudinalStep[Feature],
				Observation.FirstLongitudinalStepCm);
		}
		UE_LOG(LogTemp, Display,
			TEXT("[AnalyticBakeValidation] Asset=%s Result=Valid Sources=%d SourceHash=%016llX Triangles=%d Positions=%d Normals=%d RuntimeVertices=%d Edges=%d BoundaryEdges=%d NonManifoldEdges=%d SmoothEdges=%d CreaseEdges=%d SmoothRegions=%d Patches=%d PlanarCandidates=%d PlanarGroups=%d ArchitecturalPlaneConstraints=%d PlanarMirrorReciprocal=%d/%d ShapeValid=%d ShapeFlat=%d ShapeDevelopable=%d ShapeElliptic=%d ShapeSaddle=%d ShapePoorFit=%d CurvatureTriangles=Planar:%d/Developable:%d/Elliptic:%d/Saddle:%d/Unresolved:%d CurvatureRegions=%d CurvatureMirrorReciprocal=%d/%d CurvatureMirrorPlausible=%d/Self:%d/DistinctDirected:%d ExtrusionTriangles=%d ExtrusionRegions=%d PlausibleQuarterEllipses=%d ExtrusionMirrorReciprocal=%d/%d ExtrusionMirrorPlausible=%d/Self:%d/DistinctDirected:%d EllipseBoundaries=%d/C0:%d/G1:%d ZeroCurvatureProfiles=%d/UsableBoundaries:%d SharedC2Pairs=%d/Usable:%d/Plausible:%d SharedC2Frames=%d/Plausible:%d GlobalC2Planes=%d/Compatible:%d GlobalC2Joins=%d/Plausible:%d SymmetrizedC2Planes=%d/SourcePlausible:%d/ExactMirror:%d CoupledC2Solutions=%d/Plausible:%d CoupledC2Families=%d/Plausible:%d C2Coverage=%d/Covered:%d/Self:%d/Diagonal:%d/UnpairedAxis:%d/Inner:%d/Outer:%d RuntimeHash=%016llX RecognitionHash=%016llX"),
			*ObjectPath, Existing->MeshSources.Num(), Existing->SourceHash,
			Existing->IndexedTriangles.Num(), Existing->VertexPositions.Num(),
			Existing->VertexNormals.Num(), Runtime->MeshVertices.Num(),
			Runtime->MeshEdges.Num(), BoundaryEdgeCount, NonManifoldEdgeCount,
			SmoothEdgeCount, CreaseEdgeCount, Runtime->SmoothSurfaceRegions.Num(),
			Runtime->SurfacePatches.Num(), PlanarCandidateCount,
			Runtime->PlanarSurfaceGroups.Num(),
			ArchitecturalPlanarConstraintCount,
			ReciprocalPlanarMirrorCount, Runtime->PlanarGroupMirrorMatches.Num(),
			ValidShapeSampleCount, FlatShapeSampleCount,
			DevelopableShapeSampleCount, EllipticShapeSampleCount,
			SaddleShapeSampleCount, PoorShapeFitCount,
			PlanarConstraintTriangleCount, DevelopableTriangleCount,
			EllipticTriangleCount, SaddleTriangleCount,
			UnresolvedCurvatureTriangleCount,
			Runtime->CurvatureSurfaceRegions.Num(),
			ReciprocalCurvatureMirrorCount,
			Runtime->CurvatureRegionMirrorMatches.Num(),
			PlausibleCurvatureMirrorCount,
			PlausibleCurvatureSelfMirrorCount,
			PlausibleCurvatureMirrorCount - PlausibleCurvatureSelfMirrorCount,
			ExtrusionTriangleCandidateCount, Runtime->ExtrusionSurfaceRegions.Num(),
			PlausibleQuarterEllipseCount,
			ReciprocalExtrusionMirrorCount,
			Runtime->ExtrusionRegionMirrorMatches.Num(),
			PlausibleExtrusionMirrorCount,
			PlausibleExtrusionSelfMirrorCount,
			PlausibleExtrusionMirrorCount - PlausibleExtrusionSelfMirrorCount,
			Runtime->QuarterEllipseBoundaryMatches.Num(), EllipseBoundaryC0Count,
			EllipseBoundaryG1Count,
			Runtime->C2TransitionSectionFits.Num(),
			UsableC2TransitionBoundaryCount,
			Runtime->SharedC2TransitionPairFits.Num(), UsableSharedC2PairCount,
			PlausibleSharedC2PairCount,
			Runtime->SharedC2FrameLedger.Num(), PlausibleSharedC2FrameCount,
			Runtime->GlobalC2PlaneConstraints.Num(), CompatibleGlobalC2PlaneCount,
			Runtime->GlobalC2JoinCertificates.Num(), PlausibleGlobalC2JoinCount,
			Runtime->SymmetrizedC2PlaneConstraints.Num(),
			PlausibleSymmetrizedC2PlaneCount, ExactSymmetrizedC2PlaneCount,
			Runtime->CoupledC2TransitionSolutions.Num(),
			PlausibleCoupledC2SolutionCount,
			Runtime->CoupledC2TransitionFamilies.Num(),
			PlausibleCoupledC2FamilyCount,
			Runtime->C2TransitionCoverage.Num(), CoveredC2TransitionCount,
			SelfMirrorC2TransitionCount, DiagonalC2TransitionCount,
			UnpairedAxisC2TransitionCount,
			PlayableInnerC2TransitionCount, OuterBackingC2TransitionCount,
			Runtime->StableHash(), Runtime->RecognitionDiagnosticsHash());
		UE_LOG(LogTemp, Display,
			TEXT("[AnalyticPlayableC2OrbitSummary] Candidates=%d Complete=%d TopologyPlausible=%d SourcePlausible=%d Members=%d"),
			Runtime->PlayableC2OrbitCandidates.Num(),
			CompletePlayableC2OrbitCount,
			TopologyPlausiblePlayableC2OrbitCount,
			SourcePlausiblePlayableC2OrbitCount,
			Runtime->PlayableC2OrbitMembers.Num());
		UE_LOG(LogTemp, Display,
			TEXT("[AnalyticPlayableC2TriangleSupportSummary] Candidates=%d Complete=%d Plausible=%d SourcePlausible=%d NetworkPlausible=%d Members=%d CanonicalPoints=%d"),
			Runtime->PlayableC2TriangleSupportCandidates.Num(),
			CompletePlayableC2TriangleSupportCount,
			PlausiblePlayableC2TriangleSupportCount,
			SourcePlausiblePlayableC2TriangleSupportCount,
			NetworkPlausiblePlayableC2TriangleSupportCount,
			Runtime->PlayableC2TriangleSupportMembers.Num(),
			Runtime->PlayableC2CanonicalSupportPoints.Num());
		UE_LOG(LogTemp, Display,
			TEXT("[AnalyticPlayableC2PlaneBindingSummary] Bindings=%d ExistingCompatible=%d SameSourceGroup=%d NetworkCompatible=%d NetworkPlanes=%d SyntheticPlanes=%d"),
			Runtime->PlayableC2PlaneBindings.Num(),
			CompatiblePlayableC2PlaneBindingCount,
			SameSourcePlayableC2PlaneBindingCount,
			CompatiblePlayableC2NetworkBindingCount,
			Runtime->PlayableC2NetworkPlaneConstraints.Num(),
			SyntheticPlayableC2NetworkPlaneCount);
		UE_LOG(LogTemp, Display,
			TEXT("[AnalyticEndWallBoundarySummary] Components=%d Closed=%d CrossX=%d Edges=%d"),
			Runtime->EndWallBoundaryComponents.Num(),
			ClosedEndWallBoundaryComponentCount,
			CentralEndWallBoundaryComponentCount,
			Runtime->EndWallBoundaryEdgeIndices.Num());
		UE_LOG(LogTemp, Display,
			TEXT("[AnalyticOpenRimSummary] Candidates=%d Playable=%d Outer=%d OpenCentral=%d MirrorPlausible=%d SelfPlausible=%d Partitioned=%d Edges=%d"),
			Runtime->OpenRimCandidates.Num(), PlayableOpenRimCount,
			OuterOpenRimCount, OpenCentralOpenRimCount,
			MirrorPlausibleOpenRimCount,
			SelfPlausibleOpenRimCount,
			PartitionedOpenRimCount,
			Runtime->OpenRimEdgeIndices.Num());
		UE_LOG(LogTemp, Display,
			TEXT("[AnalyticCanonicalOpenArchSummary] Solutions=%d Plausible=%d Members=%d"),
			Runtime->CanonicalOpenArchSolutions.Num(),
			PlausibleCanonicalOpenArchCount,
			Runtime->CanonicalOpenArchMemberFits.Num());
		for (int32 Feature = 0; Feature < 4; ++Feature)
		{
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticOpeningSurfaceBandSummary] Feature=%d Observations=%d SameSmooth=%d SourceG1=%d MaxDihedralDeg=%.9g MaxNormalJumpDeg=%.9g MaxLongitudinalStepCm=%.9g"),
				Feature, OpeningSurfaceBandCount[Feature],
				OpeningSurfaceBandSameSmoothCount[Feature],
				OpeningSurfaceBandG1Count[Feature],
				OpeningSurfaceBandMaximumDihedral[Feature],
				OpeningSurfaceBandMaximumNormalJump[Feature],
				OpeningSurfaceBandMaximumLongitudinalStep[Feature]);
		}
		for (int32 Layer = 1; Layer <= 2; ++Layer)
		{
			for (int32 Feature = 0; Feature < 4; ++Feature)
			{
				int32 Count = 0;
				int32 SameSmooth = 0;
				int32 SourceG1 = 0;
				double MaximumDihedral = 0.0;
				for (const Speed::Analytic::FOpenRimSurfaceBandObservation&
					Observation : Runtime->OpenRimSurfaceBandObservations)
				{
					const Speed::Analytic::FOpenRimCandidate& Rim =
						Runtime->OpenRimCandidates[
							Observation.OpenRimCandidateIndex];
					if (static_cast<int32>(Rim.SurfaceLayer) != Layer ||
						static_cast<int32>(Observation.Feature) != Feature)
						continue;
					++Count;
					SameSmooth += Observation.bSameSmoothRegion ? 1 : 0;
					SourceG1 += Observation.bSourceG1Plausible ? 1 : 0;
					MaximumDihedral = FMath::Max(MaximumDihedral,
						Observation.FaceDihedralDegrees);
				}
				UE_LOG(LogTemp, Display,
					TEXT("[AnalyticOpeningSurfaceBandLayerSummary] Layer=%d Feature=%d Observations=%d SameSmooth=%d SourceG1=%d MaxDihedralDeg=%.9g"),
					Layer, Feature, Count, SameSmooth, SourceG1,
					MaximumDihedral);
			}
		}
		int32 ConnectedOpeningSectionCount = 0;
		int32 BoundaryPlaneG1OpeningSectionCount = 0;
		int32 LongitudinalRunOpeningSectionCount = 0;
		int32 IndividualC2OpeningSectionCount = 0;
		double MaximumIndividualC2OpeningSectionRms = 0.0;
		double MaximumIndividualC2OpeningSectionResidual = 0.0;
		for (const Speed::Analytic::FOpenRimTransverseSection& Section :
			Runtime->OpenRimTransverseSections)
		{
			ConnectedOpeningSectionCount += Section.bConnectedToRim ? 1 : 0;
			BoundaryPlaneG1OpeningSectionCount +=
				Section.bSourceBoundaryPlaneG1Plausible ? 1 : 0;
			LongitudinalRunOpeningSectionCount +=
				Section.bLongitudinalRunPlausible ? 1 : 0;
			IndividualC2OpeningSectionCount += Section.bIndividualC2FitValid ? 1 : 0;
			MaximumIndividualC2OpeningSectionRms = FMath::Max(
				MaximumIndividualC2OpeningSectionRms,
				Section.IndividualC2RootMeanSquareResidualCm);
			MaximumIndividualC2OpeningSectionResidual = FMath::Max(
				MaximumIndividualC2OpeningSectionResidual,
				Section.IndividualC2MaximumResidualCm);
		}
		UE_LOG(LogTemp, Display,
			TEXT("[AnalyticOpeningTransverseSectionSummary] Sections=%d Connected=%d BoundaryPlaneG1=%d LongitudinalRun=%d IndividualC2=%d IndividualC2WorstRmsCm=%.9g IndividualC2MaxCm=%.9g Points=%d"),
			Runtime->OpenRimTransverseSections.Num(),
			ConnectedOpeningSectionCount, BoundaryPlaneG1OpeningSectionCount,
			LongitudinalRunOpeningSectionCount, IndividualC2OpeningSectionCount,
			MaximumIndividualC2OpeningSectionRms,
			MaximumIndividualC2OpeningSectionResidual,
			Runtime->OpenRimTransverseSectionPoints.Num());
		for (const Speed::Analytic::FOpenRimTransverseSection& Section :
			Runtime->OpenRimTransverseSections)
		{
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticOpeningTransverseSection] Id=%016llX Rim=%016llX RimEdge=%d Feature=%u SliceCm=%.9g CanonicalRimParameter=%.9g TopologyCanonicalRimParameter=%.9g TopologyCanonicalValid=%d SliceOrigin=%s SliceNormal=%s OpeningDirection=%s Segments=%d Points=%d TransitionSegments=%d TransitionPoints=%d ArcCm=%.9g RimDistanceCm=%.9g MaxDepthCm=%.9g MaxOpeningOffsetCm=%.9g BoundaryPlaneTangentDeg=%.9g LongitudinalTangentDeg=%.9g LongitudinalTangentDepthCm=%.9g LongitudinalTangentOffsetCm=%.9g TransitionHop=%d IndividualC2BoundaryPlaneTangentCm=%.9g IndividualC2LongitudinalTangentCm=%.9g IndividualC2RmsCm=%.9g IndividualC2MaxCm=%.9g BoundsMin=%s BoundsMax=%s Connected=%d BoundaryPlaneG1=%d LongitudinalRun=%d IndividualC2=%d"),
				Section.SectionId,
				Runtime->OpenRimCandidates[
					Section.OpenRimCandidateIndex].CandidateId,
				Section.RimEdgeIndex,
				static_cast<uint8>(Section.Feature), Section.SliceCoordinateCm,
				Section.CanonicalRimParameter,
				Section.TopologyCanonicalRimParameter,
				Section.bTopologyCanonicalRimParameterValid ? 1 : 0,
				*Section.SliceOrigin.ToString(),
				*Section.SlicePlaneNormal.ToString(),
				*Section.OpeningDirection.ToString(),
				Section.SegmentCount, Section.PointCount,
				Section.TransitionSegmentCount, Section.TransitionPointCount,
				Section.ArcLengthCm,
				Section.RimDistanceCm, Section.MaximumLongitudinalDepthCm,
				Section.MaximumOpeningOffsetCm,
				Section.BoundaryPlaneTangentResidualDegrees,
				Section.LongitudinalTangentResidualDegrees,
				Section.LongitudinalTangentDepthCm,
				Section.LongitudinalTangentOpeningOffsetCm,
				Section.LongitudinalTransitionHop,
				Section.IndividualC2BoundaryPlaneTangentMagnitudeCm,
				Section.IndividualC2LongitudinalTangentMagnitudeCm,
				Section.IndividualC2RootMeanSquareResidualCm,
				Section.IndividualC2MaximumResidualCm,
				*Section.Bounds.Min.ToString(), *Section.Bounds.Max.ToString(),
				Section.bConnectedToRim ? 1 : 0,
				Section.bSourceBoundaryPlaneG1Plausible ? 1 : 0,
				Section.bLongitudinalRunPlausible ? 1 : 0,
				Section.bIndividualC2FitValid ? 1 : 0);
			for (int32 PointOffset = 0;
				PointOffset < Section.TransitionPointCount; ++PointOffset)
			{
				const FVector2d& Point =
					Runtime->OpenRimTransitionSectionPoints[
						Section.FirstTransitionPointIndex + PointOffset];
				UE_LOG(LogTemp, Display,
					TEXT("[AnalyticOpeningTransitionPoint] Section=%016llX Offset=%d Point=%s"),
					Section.SectionId, PointOffset, *Point.ToString());
			}
		}
		for (const Speed::Analytic::FOpenRimC2LoftStation& Station :
			Runtime->OpenRimC2LoftStations)
		{
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticOpeningC2LoftStation] Id=%016llX Family=%u CanonicalRimParameter=%.9g Members=%d EndDepthMeanCm=%.9g EndOffsetMeanCm=%.9g BoundaryPlaneTangentMeanCm=%.9g LongitudinalTangentMeanCm=%.9g EndDepthMedianCm=%.9g EndOffsetMedianCm=%.9g BoundaryPlaneTangentMedianCm=%.9g LongitudinalTangentMedianCm=%.9g GeometricEndDepthCm=%.9g GeometricEndOffsetCm=%.9g GeometricBoundaryPlaneTangentCm=%.9g GeometricLongitudinalTangentCm=%.9g GeometricRmsCm=%.9g GeometricMaxCm=%.9g ParameterSpreadRmsCm=%.9g ParameterSpreadMaxCm=%.9g BothOpenings=%d BothSides=%d GeometricFit=%d"),
				Station.StationId, static_cast<uint8>(Station.Family),
				Station.CanonicalRimParameter, Station.MemberCount,
				Station.MeanEndDepthCm, Station.MeanEndOpeningOffsetCm,
				Station.MeanBoundaryPlaneTangentCm,
				Station.MeanLongitudinalTangentCm,
				Station.MedianEndDepthCm,
				Station.MedianEndOpeningOffsetCm,
				Station.MedianBoundaryPlaneTangentCm,
				Station.MedianLongitudinalTangentCm,
				Station.GeometricFitEndDepthCm,
				Station.GeometricFitEndOpeningOffsetCm,
				Station.GeometricFitBoundaryPlaneTangentCm,
				Station.GeometricFitLongitudinalTangentCm,
				Station.GeometricFitRootMeanSquareResidualCm,
				Station.GeometricFitMaximumResidualCm,
				Station.RootMeanSquareParameterSpreadCm,
				Station.MaximumParameterSpreadCm,
				Station.bHasBothOpenings ? 1 : 0,
				Station.bHasBothSides ? 1 : 0,
				Station.bGeometricFitValid ? 1 : 0);
		}
		for (const Speed::Analytic::FOpenRimC2LoftFit& Fit :
			Runtime->OpenRimC2LoftFits)
		{
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticOpeningC2LoftFit] Id=%016llX Model=%u Coefficients=%d Segments=%d Members=%d Samples=%d BalancedRmsCm=%.9g WorstMemberRmsCm=%.9g MaxCm=%.9g MinimumParameterCm=%.9g MaximumFirstDerivativeCm=%.9g MaximumSecondDerivativeCm=%.9g FirstDerivativeJoinResidualCm=%.9g SecondDerivativeJoinResidualCm=%.9g ExactXY=%d C2AlongRim=%d Regular=%d"),
				Fit.FitId, static_cast<uint8>(Fit.Model), Fit.CoefficientCount,
				Fit.SegmentCount, Fit.MemberCount, Fit.SampleCount,
				Fit.BalancedRootMeanSquareResidualCm,
				Fit.MaximumMemberRootMeanSquareResidualCm,
				Fit.MaximumResidualCm, Fit.MinimumSampledParameterValueCm,
				Fit.MaximumSampledFirstDerivativeMagnitudeCm,
				Fit.MaximumSampledSecondDerivativeMagnitudeCm,
				Fit.MaximumFirstDerivativeJoinResidualCm,
				Fit.MaximumSecondDerivativeJoinResidualCm,
				Fit.bExactXYMirrorPlacement ? 1 : 0,
				Fit.bC2AlongRimByPolynomialConstruction ? 1 : 0,
				Fit.bRegularPositiveParameterization ? 1 : 0);
			for (int32 Index = 0; Index < Fit.CoefficientCount; ++Index)
			{
				UE_LOG(LogTemp, Display,
					TEXT("[AnalyticOpeningC2LoftCoefficient] Fit=%016llX Index=%d EndDepth=%.9g EndOffset=%.9g BoundaryPlaneTangent=%.9g LongitudinalTangent=%.9g"),
					Fit.FitId, Index, Fit.EndDepthCoefficients[Index],
					Fit.EndOpeningOffsetCoefficients[Index],
					Fit.BoundaryPlaneTangentCoefficients[Index],
					Fit.LongitudinalTangentCoefficients[Index]);
			}
		}
		for (const Speed::Analytic::FOpenRimCanonicalSurfaceFit& Fit :
			Runtime->OpenRimCanonicalSurfaceFits)
		{
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticOpeningCanonicalSurfaceFit] Id=%016llX Samples=%d ParameterWidths=%.9g/%.9g/%.9g RmsCm=%.9g MaxCm=%.9g MinimumEndDepthCm=%.9g MinimumLongitudinalTangentCm=%.9g FirstDerivativeJoinResidualCm=%.9g SecondDerivativeJoinResidualCm=%.9g NormalJoinResidualDeg=%.9g CorrespondenceLanes=%d CorrespondenceSections=%d MaxCorrespondenceDisplacement=%.17g MinCorrespondenceSpacing=%.17g ExactXY=%d TransverseC2=%d LongitudinalC2=%d TransitionCorrectionC2=%d CorrespondenceMonotone=%d CorrespondenceBounded=%d CorrespondenceInterior=%d Regular=%d"),
				Fit.FitId, Fit.SampleCount, Fit.VerticalSegmentParameterWidth,
				Fit.TransitionParameterWidth, Fit.HorizontalSpanParameterWidth,
				Fit.RootMeanSquareResidualCm,
				Fit.MaximumResidualCm, Fit.MinimumEndDepthCm,
				Fit.MinimumLongitudinalTangentCm,
				Fit.MaximumFirstDerivativeJoinResidualCm,
				Fit.MaximumSecondDerivativeJoinResidualCm,
				Fit.MaximumNormalJoinResidualDegrees,
				Fit.CorrespondenceLaneCount,
				Fit.CorrespondenceSectionCount,
				Fit.MaximumCorrespondenceDisplacement,
				Fit.MinimumCorrespondenceSpacing,
				Fit.bExactXYMirrorPlacement ? 1 : 0,
				Fit.bExactTransverseC2ByConstruction ? 1 : 0,
				Fit.bLongitudinallyC2ByConstruction ? 1 : 0,
				Fit.bTransitionCorrectionC2ByConstruction ? 1 : 0,
				Fit.bCorrespondenceStrictlyMonotone ? 1 : 0,
				Fit.bCorrespondenceWithinQuantizedWitnessBounds ? 1 : 0,
				Fit.bCorrespondenceStrictlyInterior ? 1 : 0,
				Fit.bRegularPositiveParameterization ? 1 : 0);
			for (int32 Index = 0; Index < 4; ++Index)
			{
				UE_LOG(LogTemp, Display,
					TEXT("[AnalyticOpeningCanonicalTransitionCoefficient] Fit=%016llX Index=%d StartTangentY=%.9g StartTangentZ=%.9g EndDepth=%.9g EndY=%.9g EndZ=%.9g"),
					Fit.FitId, Index,
					Fit.TransitionStartTangentYCoefficients[Index],
					Fit.TransitionStartTangentZCoefficients[Index],
					Fit.TransitionEndDepthCoefficients[Index],
					Fit.TransitionEndYCoefficients[Index],
					Fit.TransitionEndZCoefficients[Index]);
			}
			for (int32 FeatureIndex = 0; FeatureIndex < 4; ++FeatureIndex)
			{
				UE_LOG(LogTemp, Display,
					TEXT("[AnalyticOpeningCanonicalSurfaceFeatureFit] Fit=%016llX Feature=%d Samples=%d RmsCm=%.9g MaxCm=%.9g"),
					Fit.FitId, FeatureIndex, Fit.FeatureSampleCounts[FeatureIndex],
					Fit.FeatureRootMeanSquareResidualsCm[FeatureIndex],
					Fit.FeatureMaximumResidualsCm[FeatureIndex]);
			}
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticOpeningCanonicalSurfaceMaximum] Fit=%016llX Section=%d PointOffset=%d RimParameter=%.17g TransitionParameter=%.17g ResidualCm=%.9g Source=(%.9g,%.9g,%.9g) Surface=(%.9g,%.9g,%.9g)"),
				Fit.FitId, Fit.MaximumResidualSectionIndex,
				Fit.MaximumResidualPointOffset, Fit.MaximumResidualRimParameter,
				Fit.MaximumResidualTransitionParameter, Fit.MaximumResidualCm,
				Fit.MaximumResidualSourcePositionCm.X,
				Fit.MaximumResidualSourcePositionCm.Y,
				Fit.MaximumResidualSourcePositionCm.Z,
				Fit.MaximumResidualFitPositionCm.X,
				Fit.MaximumResidualFitPositionCm.Y,
				Fit.MaximumResidualFitPositionCm.Z);
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticOpeningCanonicalTransitionResidualClasses] Fit=%016llX EndpointClampedSamples=%d EndpointClampedRmsCm=%.9g EndpointClampedMaxCm=%.9g InteriorSamples=%d InteriorRmsCm=%.9g InteriorMaxCm=%.9g"),
				Fit.FitId, Fit.TransitionEndpointClampedSampleCount,
				Fit.TransitionEndpointClampedRootMeanSquareResidualCm,
				Fit.TransitionEndpointClampedMaximumResidualCm,
				Fit.TransitionInteriorSampleCount,
				Fit.TransitionInteriorRootMeanSquareResidualCm,
				Fit.TransitionInteriorMaximumResidualCm);
		}
		for (const Speed::Analytic::FOpenRimCanonicalSectionCorrespondence&
			Correspondence : Runtime->OpenRimCanonicalSectionCorrespondences)
		{
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticOpeningCanonicalCorrespondence] Section=%d Lane=%016llX Rank=%d/%d Geometric=%.17g Topology=%.17g Optimized=%.17g RmsCm=%.9g Bounded=%d Interior=%d"),
				Correspondence.OpenRimTransverseSectionIndex,
				Correspondence.LaneId, Correspondence.LaneRank,
				Correspondence.LaneSectionCount,
				Correspondence.GeometricSurfaceParameter,
				Correspondence.TopologySurfaceParameter,
				Correspondence.OptimizedSurfaceParameter,
				Correspondence.RootMeanSquareResidualCm,
				Correspondence.bWithinQuantizedWitnessBounds ? 1 : 0,
				Correspondence.bStrictlyInterior ? 1 : 0);
		}
		UE_LOG(LogTemp, Display,
			TEXT("[AnalyticOpeningTransitionFitSummary] Fits=%d TransitionPoints=%d"),
			Runtime->OpenRimTransitionFamilyFits.Num(),
			Runtime->OpenRimTransitionSectionPoints.Num());
		for (const Speed::Analytic::FOpenRimTransitionFamilyFit& Fit :
			Runtime->OpenRimTransitionFamilyFits)
		{
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticOpeningTransitionFit] Id=%016llX Family=%u Members=%d Samples=%d EndDepthCm=%.9g EndOffsetCm=%.9g BoundaryPlaneTangentCm=%.9g LongitudinalTangentCm=%.9g BalancedRmsCm=%.9g WorstMemberRmsCm=%.9g MaxCm=%.9g ExactC2=%d"),
				Fit.FitId, static_cast<uint8>(Fit.Family), Fit.MemberCount,
				Fit.SampleCount, Fit.EndDepthCm, Fit.EndOpeningOffsetCm,
				Fit.BoundaryPlaneTangentMagnitudeCm,
				Fit.LongitudinalTangentMagnitudeCm,
				Fit.BalancedRootMeanSquareResidualCm,
				Fit.MaximumMemberRootMeanSquareResidualCm,
				Fit.MaximumResidualCm,
				Fit.bExactC0G1C2ByConstruction ? 1 : 0);
			for (int32 MemberOffset = 0;
				MemberOffset < Fit.MemberCount; ++MemberOffset)
			{
				const Speed::Analytic::FOpenRimTransitionMemberFit& Member =
					Runtime->OpenRimTransitionMemberFits[
						Fit.FirstMemberFitIndex + MemberOffset];
				const Speed::Analytic::FOpenRimTransverseSection& Section =
					Runtime->OpenRimTransverseSections[
						Member.OpenRimTransverseSectionIndex];
				UE_LOG(LogTemp, Display,
					TEXT("[AnalyticOpeningTransitionMemberFit] Fit=%016llX Section=%016llX Feature=%u SliceCm=%.9g Samples=%d RmsCm=%.9g MaxCm=%.9g"),
					Fit.FitId, Section.SectionId,
					static_cast<uint8>(Section.Feature),
					Section.SliceCoordinateCm, Member.SampleCount,
					Member.RootMeanSquareResidualCm,
					Member.MaximumResidualCm);
			}
		}
		UE_LOG(LogTemp, Display,
			TEXT("[AnalyticOpeningSupportTransitionSummary] Intents=%d"),
			Runtime->OpenRimSupportTransitionIntents.Num());
		for (const Speed::Analytic::FOpenRimSupportTransitionIntent& Intent :
			Runtime->OpenRimSupportTransitionIntents)
		{
			const Speed::Analytic::FOpenRimTransverseSection& Section =
				Runtime->OpenRimTransverseSections[
					Intent.OpenRimTransverseSectionIndex];
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticOpeningSupportTransition] Id=%016llX Section=%016llX Feature=%u Policy=%u GeometricC2Permitted=%d Bidirectional=%d ImmediateHandoffForbidden=%d FreshApproachMayAcquire=%d"),
				Intent.IntentId, Section.SectionId,
				static_cast<uint8>(Intent.Feature),
				static_cast<uint8>(Intent.Policy),
				Intent.bGeometricC2Permitted ? 1 : 0,
				Intent.bBidirectional ? 1 : 0,
				Intent.bImmediateAdjacentSurfaceHandoffForbidden ? 1 : 0,
				Intent.bFreshApproachMayAcquireAdjacentSurface ? 1 : 0);
		}
		auto Quantile = [](const TArray<double>& Sorted, const double Fraction)
		{
			if (Sorted.IsEmpty()) return 0.0;
			const int32 Index = FMath::Clamp(
				FMath::RoundToInt(Fraction * static_cast<double>(Sorted.Num() - 1)),
				0, Sorted.Num() - 1);
			return Sorted[Index];
		};
		TArray<double> EdgeLengths;
		EdgeLengths.Reserve(Runtime->MeshEdges.Num());
		for (const Speed::Analytic::FTriangleMeshEdge& Edge : Runtime->MeshEdges)
		{
			EdgeLengths.Add(FVector3d::Distance(
				Runtime->MeshVertices[Edge.VertexA], Runtime->MeshVertices[Edge.VertexB]));
		}
		Algo::Sort(EdgeLengths);
		UE_LOG(LogTemp, Display,
			TEXT("[AnalyticEdgeLengthQuantiles] Cm=P10:%.9g,P25:%.9g,P50:%.9g,P75:%.9g,P90:%.9g,P95:%.9g,P99:%.9g,Max:%.9g"),
			Quantile(EdgeLengths, 0.10), Quantile(EdgeLengths, 0.25),
			Quantile(EdgeLengths, 0.50), Quantile(EdgeLengths, 0.75),
			Quantile(EdgeLengths, 0.90), Quantile(EdgeLengths, 0.95),
			Quantile(EdgeLengths, 0.99), Quantile(EdgeLengths, 1.0));
		TArray<double> CurvatureMagnitudes;
		TArray<double> ShapeResiduals;
		TArray<double> ShapeSupportRadii;
		TArray<double> ShapeNeighborCounts;
		CurvatureMagnitudes.Reserve(ValidShapeSampleCount);
		ShapeResiduals.Reserve(ValidShapeSampleCount);
		ShapeSupportRadii.Reserve(ValidShapeSampleCount);
		ShapeNeighborCounts.Reserve(ValidShapeSampleCount);
		for (const Speed::Analytic::FVertexShapeSample& Sample : Runtime->VertexShapeSamples)
		{
			if (!Sample.bValid) continue;
			CurvatureMagnitudes.Add(FMath::Max(
				FMath::Abs(Sample.MinimumPrincipalCurvature),
				FMath::Abs(Sample.MaximumPrincipalCurvature)));
			ShapeResiduals.Add(Sample.RootMeanSquareResidual);
			ShapeSupportRadii.Add(Sample.SupportRadiusCm);
			ShapeNeighborCounts.Add(static_cast<double>(Sample.NeighborCount));
		}
		Algo::Sort(CurvatureMagnitudes);
		Algo::Sort(ShapeResiduals);
		Algo::Sort(ShapeSupportRadii);
		Algo::Sort(ShapeNeighborCounts);
		UE_LOG(LogTemp, Display,
			TEXT("[AnalyticShapeQuantiles] Rings=2 CurvatureAbsInvCm=P10:%.9g,P25:%.9g,P50:%.9g,P75:%.9g,P90:%.9g,P99:%.9g ResidualInvCm=P10:%.9g,P25:%.9g,P50:%.9g,P75:%.9g,P90:%.9g,P99:%.9g SupportRadiusCm=P50:%.9g,P90:%.9g,P99:%.9g Neighbors=P50:%.9g,P90:%.9g,P99:%.9g"),
			Quantile(CurvatureMagnitudes, 0.10), Quantile(CurvatureMagnitudes, 0.25),
			Quantile(CurvatureMagnitudes, 0.50), Quantile(CurvatureMagnitudes, 0.75),
			Quantile(CurvatureMagnitudes, 0.90), Quantile(CurvatureMagnitudes, 0.99),
			Quantile(ShapeResiduals, 0.10), Quantile(ShapeResiduals, 0.25),
			Quantile(ShapeResiduals, 0.50), Quantile(ShapeResiduals, 0.75),
			Quantile(ShapeResiduals, 0.90), Quantile(ShapeResiduals, 0.99),
			Quantile(ShapeSupportRadii, 0.50), Quantile(ShapeSupportRadii, 0.90),
			Quantile(ShapeSupportRadii, 0.99), Quantile(ShapeNeighborCounts, 0.50),
			Quantile(ShapeNeighborCounts, 0.90), Quantile(ShapeNeighborCounts, 0.99));
		constexpr double SymmetryMatchToleranceCm = 0.1;
		for (const Speed::Analytic::EPlanarSymmetryAxis Axis : {
			Speed::Analytic::EPlanarSymmetryAxis::X,
			Speed::Analytic::EPlanarSymmetryAxis::Y })
		{
			const Speed::Analytic::FPlanarSymmetryMetrics Symmetry =
				Runtime->MeasurePlanarSymmetry(Axis, SymmetryMatchToleranceCm);
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticSymmetry] Axis=%s CenterCm=%.9g ToleranceCm=%.9g MatchedVertices=%d UnmatchedVertices=%d Coverage=%.9g RMSResidualCm=%.9g MaxResidualCm=%.9g"),
				Axis == Speed::Analytic::EPlanarSymmetryAxis::X ? TEXT("X") : TEXT("Y"),
				Symmetry.CenterCoordinate, SymmetryMatchToleranceCm,
				Symmetry.MatchedVertexCount, Symmetry.UnmatchedVertexCount,
				Runtime->MeshVertices.IsEmpty() ? 0.0 :
					static_cast<double>(Symmetry.MatchedVertexCount) /
					static_cast<double>(Runtime->MeshVertices.Num()),
				Symmetry.RootMeanSquareResidual, Symmetry.MaximumResidual);
			const Speed::Analytic::FSurfaceSymmetryMetrics SurfaceSymmetry =
				Runtime->MeasurePlanarSurfaceSymmetry(
					Axis, SymmetryMatchToleranceCm, 0.0);
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticSurfaceSymmetry] Axis=%s CenterCm=%.9g ToleranceCm=%.9g WithinTolerance=%d OutsideTolerance=%d Coverage=%.9g RMSNearestResidualCm=%.9g MaxNearestResidualCm=%.9g"),
				Axis == Speed::Analytic::EPlanarSymmetryAxis::X ? TEXT("X") : TEXT("Y"),
				SurfaceSymmetry.CenterCoordinate, SymmetryMatchToleranceCm,
				SurfaceSymmetry.WithinToleranceVertexCount,
				SurfaceSymmetry.OutsideToleranceVertexCount,
				Runtime->MeshVertices.IsEmpty() ? 0.0 :
					static_cast<double>(SurfaceSymmetry.WithinToleranceVertexCount) /
					static_cast<double>(Runtime->MeshVertices.Num()),
				SurfaceSymmetry.RootMeanSquareNearestResidual,
				SurfaceSymmetry.MaximumNearestResidual);
		}
		TArray<int32> PatchOrder;
		PatchOrder.SetNumUninitialized(Runtime->SurfacePatches.Num());
		for (int32 Index = 0; Index < PatchOrder.Num(); ++Index) PatchOrder[Index] = Index;
		Algo::Sort(PatchOrder, [&Runtime](const int32 A, const int32 B)
		{
			const Speed::Analytic::FSurfacePatch& PatchA = Runtime->SurfacePatches[A];
			const Speed::Analytic::FSurfacePatch& PatchB = Runtime->SurfacePatches[B];
			return PatchA.Area != PatchB.Area
				? PatchA.Area > PatchB.Area
				: PatchA.PatchId < PatchB.PatchId;
		});
		for (int32 Rank = 0; Rank < FMath::Min(12, PatchOrder.Num()); ++Rank)
		{
			const Speed::Analytic::FSurfacePatch& Patch =
				Runtime->SurfacePatches[PatchOrder[Rank]];
			TSet<int32> PatchVertices;
			TArray<double> PatchCurvatures;
			TArray<double> PatchResiduals;
			for (int32 Offset = 0; Offset < Patch.TriangleCount; ++Offset)
			{
				const int32 TriangleIndex = Runtime->PatchTriangleIndices[
					Patch.FirstTriangleIndex + Offset];
				for (int32 Corner = 0; Corner < 3; ++Corner)
				{
					PatchVertices.Add(Runtime->TriangleVertexIndices[TriangleIndex][Corner]);
				}
			}
			for (const int32 VertexIndex : PatchVertices)
			{
				const Speed::Analytic::FVertexShapeSample& Sample =
					Runtime->VertexShapeSamples[VertexIndex];
				if (!Sample.bValid) continue;
				PatchCurvatures.Add(FMath::Max(
					FMath::Abs(Sample.MinimumPrincipalCurvature),
					FMath::Abs(Sample.MaximumPrincipalCurvature)));
				PatchResiduals.Add(Sample.RootMeanSquareResidual);
			}
			Algo::Sort(PatchCurvatures);
			Algo::Sort(PatchResiduals);
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticPatch] Rank=%d Id=%016llX Kind=%u Triangles=%d Area=%.9g ResidualCm=%.9g NormalErrorDeg=%.9g ShapeSamples=%d ShapeCurvatureP50=%.9g ShapeCurvatureP90=%.9g ShapeResidualP50=%.9g Origin=%s Normal=%s BoundsMin=%s BoundsMax=%s"),
				Rank, Patch.PatchId, static_cast<uint8>(Patch.Kind),
				Patch.TriangleCount, Patch.Area, Patch.MaximumPlaneResidual,
				Patch.MaximumNormalAngleDegrees, PatchCurvatures.Num(),
				Quantile(PatchCurvatures, 0.50), Quantile(PatchCurvatures, 0.90),
				Quantile(PatchResiduals, 0.50), *Patch.Origin.ToString(),
				*Patch.Normal.ToString(), *Patch.Bounds.Min.ToString(),
				*Patch.Bounds.Max.ToString());
		}
		TArray<int32> PlanarGroupOrder;
		PlanarGroupOrder.SetNumUninitialized(Runtime->PlanarSurfaceGroups.Num());
		for (int32 Index = 0; Index < PlanarGroupOrder.Num(); ++Index)
		{
			PlanarGroupOrder[Index] = Index;
		}
		Algo::Sort(PlanarGroupOrder, [&Runtime](const int32 A, const int32 B)
		{
			const Speed::Analytic::FPlanarSurfaceGroup& GroupA =
				Runtime->PlanarSurfaceGroups[A];
			const Speed::Analytic::FPlanarSurfaceGroup& GroupB =
				Runtime->PlanarSurfaceGroups[B];
			return GroupA.Area != GroupB.Area
				? GroupA.Area > GroupB.Area
				: GroupA.GroupId < GroupB.GroupId;
		});
		for (int32 Rank = 0; Rank < FMath::Min(20, PlanarGroupOrder.Num()); ++Rank)
		{
			const Speed::Analytic::FPlanarSurfaceGroup& Group =
				Runtime->PlanarSurfaceGroups[PlanarGroupOrder[Rank]];
			const int32 GroupIndex = PlanarGroupOrder[Rank];
			const int32 GroupCount = Runtime->PlanarSurfaceGroups.Num();
			const Speed::Analytic::FPlanarGroupMirrorMatch& MirrorX =
				Runtime->PlanarGroupMirrorMatches[GroupIndex];
			const Speed::Analytic::FPlanarGroupMirrorMatch& MirrorY =
				Runtime->PlanarGroupMirrorMatches[GroupCount + GroupIndex];
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticPlanarGroup] Rank=%d Id=%016llX Patches=%d Triangles=%d Area=%.9g PlaneOffsetCm=%.9g ResidualCm=%.9g NormalErrorDeg=%.9g Normal=%s BoundsMin=%s BoundsMax=%s MirrorX=%016llX/Score:%.9g/Plane:%.9g/Bounds:%.9g/Area:%.9g/Reciprocal:%d MirrorY=%016llX/Score:%.9g/Plane:%.9g/Bounds:%.9g/Area:%.9g/Reciprocal:%d"),
				Rank, Group.GroupId, Group.PatchCount, Group.TriangleCount,
				Group.Area, Group.PlaneOffset, Group.MaximumPlaneResidual,
				Group.MaximumNormalAngleDegrees, *Group.Normal.ToString(),
				*Group.Bounds.Min.ToString(), *Group.Bounds.Max.ToString(),
				Runtime->PlanarSurfaceGroups[MirrorX.TargetGroupIndex].GroupId,
				MirrorX.Score, MirrorX.PlaneOffsetResidual, MirrorX.BoundsResidual,
				MirrorX.RelativeAreaResidual, MirrorX.bReciprocal ? 1 : 0,
				Runtime->PlanarSurfaceGroups[MirrorY.TargetGroupIndex].GroupId,
				MirrorY.Score, MirrorY.PlaneOffsetResidual, MirrorY.BoundsResidual,
				MirrorY.RelativeAreaResidual, MirrorY.bReciprocal ? 1 : 0);
		}
		TArray<int32> CurvatureRegionOrder;
		CurvatureRegionOrder.SetNumUninitialized(
			Runtime->CurvatureSurfaceRegions.Num());
		for (int32 Index = 0; Index < CurvatureRegionOrder.Num(); ++Index)
		{
			CurvatureRegionOrder[Index] = Index;
		}
		Algo::Sort(CurvatureRegionOrder, [&Runtime](const int32 A, const int32 B)
		{
			const Speed::Analytic::FCurvatureSurfaceRegion& RegionA =
				Runtime->CurvatureSurfaceRegions[A];
			const Speed::Analytic::FCurvatureSurfaceRegion& RegionB =
				Runtime->CurvatureSurfaceRegions[B];
			return RegionA.Area != RegionB.Area
				? RegionA.Area > RegionB.Area
				: RegionA.RegionId < RegionB.RegionId;
		});
		for (const Speed::Analytic::ECurvatureEvidenceKind Kind : {
			Speed::Analytic::ECurvatureEvidenceKind::DevelopableCandidate,
			Speed::Analytic::ECurvatureEvidenceKind::EllipticCandidate,
			Speed::Analytic::ECurvatureEvidenceKind::SaddleCandidate })
		{
			int32 KindRank = 0;
			for (const int32 RegionIndex : CurvatureRegionOrder)
			{
				const Speed::Analytic::FCurvatureSurfaceRegion& Region =
					Runtime->CurvatureSurfaceRegions[RegionIndex];
				if (Region.Kind != Kind) continue;
				const int32 RegionCount = Runtime->CurvatureSurfaceRegions.Num();
				const Speed::Analytic::FCurvatureRegionMirrorMatch& MirrorX =
					Runtime->CurvatureRegionMirrorMatches[RegionIndex];
				const Speed::Analytic::FCurvatureRegionMirrorMatch& MirrorY =
					Runtime->CurvatureRegionMirrorMatches[RegionCount + RegionIndex];
				UE_LOG(LogTemp, Display,
					TEXT("[AnalyticCurvatureRegion] KindRank=%d Id=%016llX Kind=%u Triangles=%d Area=%.9g MeanKMin=%.9g MeanKMax=%.9g DominantMin=%.9g DominantMax=%.9g RelativeFitMax=%.9g LowDirection=%s BoundsMin=%s BoundsMax=%s MirrorX=%016llX/Score:%.9g/KMin:%.9g/KMax:%.9g/DirectionDeg:%.9g/Bounds:%.9g/Area:%.9g/Reciprocal:%d/Self:%d/Plausible:%d MirrorY=%016llX/Score:%.9g/KMin:%.9g/KMax:%.9g/DirectionDeg:%.9g/Bounds:%.9g/Area:%.9g/Reciprocal:%d/Self:%d/Plausible:%d"),
					KindRank, Region.RegionId, static_cast<uint8>(Region.Kind),
					Region.TriangleCount, Region.Area,
					Region.MeanMinimumPrincipalCurvature,
					Region.MeanMaximumPrincipalCurvature,
					Region.MinimumAbsoluteDominantCurvature,
					Region.MaximumAbsoluteDominantCurvature,
					Region.MaximumRelativeFitResidual,
					*Region.MeanLowCurvatureDirection.ToString(),
					*Region.Bounds.Min.ToString(), *Region.Bounds.Max.ToString(),
					Runtime->CurvatureSurfaceRegions[MirrorX.TargetRegionIndex].RegionId,
					MirrorX.Score, MirrorX.MinimumCurvatureResidual,
					MirrorX.MaximumCurvatureResidual,
					MirrorX.LowDirectionAngleResidualDegrees, MirrorX.BoundsResidual,
					MirrorX.RelativeAreaResidual, MirrorX.bReciprocal ? 1 : 0,
					MirrorX.bSelfMirror ? 1 : 0, MirrorX.bPlausible ? 1 : 0,
					Runtime->CurvatureSurfaceRegions[MirrorY.TargetRegionIndex].RegionId,
					MirrorY.Score, MirrorY.MinimumCurvatureResidual,
					MirrorY.MaximumCurvatureResidual,
					MirrorY.LowDirectionAngleResidualDegrees, MirrorY.BoundsResidual,
					MirrorY.RelativeAreaResidual, MirrorY.bReciprocal ? 1 : 0,
					MirrorY.bSelfMirror ? 1 : 0, MirrorY.bPlausible ? 1 : 0);
				if (++KindRank >= 8) break;
			}
		}
		for (const Speed::Analytic::EPlanarSymmetryAxis Axis : {
			Speed::Analytic::EPlanarSymmetryAxis::X,
			Speed::Analytic::EPlanarSymmetryAxis::Y })
		{
			const int32 RegionCount = Runtime->CurvatureSurfaceRegions.Num();
			const int32 MatchOffset = Axis == Speed::Analytic::EPlanarSymmetryAxis::X
				? 0 : RegionCount;
			int32 PairRank = 0;
			for (const int32 SourceIndex : CurvatureRegionOrder)
			{
				const Speed::Analytic::FCurvatureSurfaceRegion& Source =
					Runtime->CurvatureSurfaceRegions[SourceIndex];
				const Speed::Analytic::FCurvatureRegionMirrorMatch& Match =
					Runtime->CurvatureRegionMirrorMatches[MatchOffset + SourceIndex];
				if (Source.Kind !=
						Speed::Analytic::ECurvatureEvidenceKind::DevelopableCandidate ||
					!Match.bPlausible)
				{
					continue;
				}
				const Speed::Analytic::FCurvatureSurfaceRegion& Target =
					Runtime->CurvatureSurfaceRegions[Match.TargetRegionIndex];
				if (!Match.bSelfMirror && Source.RegionId > Target.RegionId) continue;
				UE_LOG(LogTemp, Display,
					TEXT("[AnalyticCurvatureMirror] Axis=%u PairRank=%d Self=%d Source=%016llX Target=%016llX SourceTriangles=%d TargetTriangles=%d SourceArea=%.9g TargetArea=%.9g SourceKMin=%.9g SourceKMax=%.9g TargetKMin=%.9g TargetKMax=%.9g DirectionDeg=%.9g BoundsResidual=%.9g AreaResidual=%.9g SourceBoundsMin=%s SourceBoundsMax=%s TargetBoundsMin=%s TargetBoundsMax=%s"),
					static_cast<uint8>(Axis), PairRank, Match.bSelfMirror ? 1 : 0,
					Source.RegionId, Target.RegionId, Source.TriangleCount,
					Target.TriangleCount, Source.Area, Target.Area,
					Source.MeanMinimumPrincipalCurvature,
					Source.MeanMaximumPrincipalCurvature,
					Target.MeanMinimumPrincipalCurvature,
					Target.MeanMaximumPrincipalCurvature,
					Match.LowDirectionAngleResidualDegrees, Match.BoundsResidual,
					Match.RelativeAreaResidual, *Source.Bounds.Min.ToString(),
					*Source.Bounds.Max.ToString(), *Target.Bounds.Min.ToString(),
					*Target.Bounds.Max.ToString());
				if (++PairRank >= 32) break;
			}
		}
		TArray<int32> ExtrusionRegionOrder;
		ExtrusionRegionOrder.SetNumUninitialized(Runtime->ExtrusionSurfaceRegions.Num());
		for (int32 Index = 0; Index < ExtrusionRegionOrder.Num(); ++Index)
		{
			ExtrusionRegionOrder[Index] = Index;
		}
		Algo::Sort(ExtrusionRegionOrder, [&Runtime](const int32 A, const int32 B)
		{
			const Speed::Analytic::FExtrusionSurfaceRegion& RegionA =
				Runtime->ExtrusionSurfaceRegions[A];
			const Speed::Analytic::FExtrusionSurfaceRegion& RegionB =
				Runtime->ExtrusionSurfaceRegions[B];
			return RegionA.Area != RegionB.Area
				? RegionA.Area > RegionB.Area : RegionA.RegionId < RegionB.RegionId;
		});
		for (int32 Rank = 0;
			Rank < FMath::Min(32, ExtrusionRegionOrder.Num()); ++Rank)
		{
			const Speed::Analytic::FExtrusionSurfaceRegion& Region =
				Runtime->ExtrusionSurfaceRegions[ExtrusionRegionOrder[Rank]];
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticExtrusionRegion] Rank=%d Id=%016llX Triangles=%d Area=%.9g Axis=%s MinAspect=%.9g AxisDeviationDeg=%.9g BoundsMin=%s BoundsMax=%s QuarterEllipse=%d Plausible=%d CenterMaxU=%d CenterMaxV=%d SectionU=%s SectionV=%s CenterUV=%s RadiusU=%.9g RadiusV=%.9g EllipseRmsCm=%.9g EllipseMaxCm=%.9g"),
				Rank, Region.RegionId, Region.TriangleCount, Region.Area,
				*Region.Axis.ToString(), Region.MinimumAspectRatio,
				Region.MaximumAxisDeviationDegrees, *Region.Bounds.Min.ToString(),
				*Region.Bounds.Max.ToString(), Region.bQuarterEllipseFitValid ? 1 : 0,
				Region.bQuarterEllipsePlausible ? 1 : 0,
				Region.bEllipseCenterAtMaximumU ? 1 : 0,
				Region.bEllipseCenterAtMaximumV ? 1 : 0,
				*Region.SectionAxisU.ToString(), *Region.SectionAxisV.ToString(),
				*Region.EllipseCenterCoordinates.ToString(), Region.EllipseRadiusU,
				Region.EllipseRadiusV, Region.EllipseRootMeanSquareResidualCm,
				Region.EllipseMaximumResidualCm);
		}
		for (const Speed::Analytic::EPlanarSymmetryAxis Axis : {
			Speed::Analytic::EPlanarSymmetryAxis::X,
			Speed::Analytic::EPlanarSymmetryAxis::Y })
		{
			const int32 RegionCount = Runtime->ExtrusionSurfaceRegions.Num();
			const int32 MatchOffset = Axis ==
				Speed::Analytic::EPlanarSymmetryAxis::X ? 0 : RegionCount;
			int32 PairRank = 0;
			for (const int32 SourceIndex : ExtrusionRegionOrder)
			{
				const Speed::Analytic::FExtrusionRegionMirrorMatch& Match =
					Runtime->ExtrusionRegionMirrorMatches[MatchOffset + SourceIndex];
				if (!Match.bPlausible) continue;
				const Speed::Analytic::FExtrusionSurfaceRegion& Source =
					Runtime->ExtrusionSurfaceRegions[SourceIndex];
				const Speed::Analytic::FExtrusionSurfaceRegion& Target =
					Runtime->ExtrusionSurfaceRegions[Match.TargetRegionIndex];
				if (!Match.bSelfMirror && Source.RegionId > Target.RegionId) continue;
				UE_LOG(LogTemp, Display,
					TEXT("[AnalyticExtrusionMirror] Axis=%u PairRank=%d Self=%d Source=%016llX Target=%016llX SourceTriangles=%d TargetTriangles=%d SourceArea=%.9g TargetArea=%.9g AxisDeg=%.9g CenterLineCm=%.9g RadiusURel=%.9g RadiusVRel=%.9g BoundsCm=%.9g AreaRel=%.9g"),
					static_cast<uint8>(Axis), PairRank, Match.bSelfMirror ? 1 : 0,
					Source.RegionId, Target.RegionId, Source.TriangleCount,
					Target.TriangleCount, Source.Area, Target.Area,
					Match.AxisAngleResidualDegrees, Match.CenterLineResidualCm,
					Match.RelativeRadiusUResidual,
					Match.RelativeRadiusVResidual, Match.BoundsResidualCm,
					Match.RelativeAreaResidual);
				if (++PairRank >= 32) break;
			}
		}
		int32 BoundaryRank = 0;
		for (const Speed::Analytic::FQuarterEllipseBoundaryMatch& Match :
			Runtime->QuarterEllipseBoundaryMatches)
		{
			const Speed::Analytic::FExtrusionSurfaceRegion& Region =
				Runtime->ExtrusionSurfaceRegions[Match.ExtrusionRegionIndex];
			if (!Runtime->PlanarSurfaceGroups.IsValidIndex(Match.PlanarGroupIndex))
			{
				UE_LOG(LogTemp, Display,
					TEXT("[AnalyticEllipseBoundary] Rank=%d Region=%016llX Endpoint=%u Plane=None Position=%s C0=0 G1=0"),
					BoundaryRank, Region.RegionId,
					static_cast<uint8>(Match.Endpoint),
					*Match.EndpointPosition.ToString());
				if (++BoundaryRank >= 64) break;
				continue;
			}
			const Speed::Analytic::FPlanarSurfaceGroup& Group =
				Runtime->PlanarSurfaceGroups[Match.PlanarGroupIndex];
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticEllipseBoundary] Rank=%d Region=%016llX Endpoint=%u Plane=%016llX Position=%s PositionResidualCm=%.9g BoundsResidualCm=%.9g NormalDeg=%.9g C0=%d G1=%d"),
				BoundaryRank, Region.RegionId, static_cast<uint8>(Match.Endpoint),
				Group.GroupId, *Match.EndpointPosition.ToString(),
				Match.PositionResidualCm, Match.BoundsResidualCm,
				Match.NormalAngleResidualDegrees,
				Match.bC0Plausible ? 1 : 0, Match.bG1Plausible ? 1 : 0);
			if (++BoundaryRank >= 64) break;
		}
		int32 TransitionRank = 0;
		for (const Speed::Analytic::FC2TransitionSectionFit& Fit :
			Runtime->C2TransitionSectionFits)
		{
			const Speed::Analytic::FExtrusionSurfaceRegion& Region =
				Runtime->ExtrusionSurfaceRegions[Fit.ExtrusionRegionIndex];
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticZeroCurvatureProfile] Rank=%d Region=%016llX BoundaryUsable=%d RadiusU=%.9g RadiusV=%.9g Flattening=%.9g ProfileRmsCm=%.9g ProfileMaxCm=%.9g EllipseRmsCm=%.9g EllipseMaxCm=%.9g EndpointPlaneCm=%.9g NormalAdjustmentDeg=%.9g ZeroEndpointCurvature=%d"),
				TransitionRank, Region.RegionId,
				Fit.bBoundaryEvidenceUsable ? 1 : 0, Fit.RadiusU, Fit.RadiusV,
				Fit.FlatteningFraction, Fit.RootMeanSquareResidualCm,
				Fit.MaximumResidualCm,
				Region.EllipseRootMeanSquareResidualCm,
				Region.EllipseMaximumResidualCm,
				Fit.MaximumEndpointPlaneResidualCm,
				Fit.MaximumEndpointNormalAdjustmentDegrees,
				Fit.bZeroEndpointCurvatureByConstruction ? 1 : 0);
			if (++TransitionRank >= 64) break;
		}
		int32 SharedPairRank = 0;
		for (const Speed::Analytic::FSharedC2TransitionPairFit& Pair :
			Runtime->SharedC2TransitionPairFits)
		{
			const Speed::Analytic::FExtrusionSurfaceRegion& RegionA =
				Runtime->ExtrusionSurfaceRegions[Pair.RegionAIndex];
			const Speed::Analytic::FExtrusionSurfaceRegion& RegionB =
				Runtime->ExtrusionSurfaceRegions[Pair.RegionBIndex];
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticSharedC2Transition] Rank=%d Axis=%u RegionA=%016llX RegionB=%016llX BoundariesUsable=%d Plausible=%d SharedRadiusU=%.9g SharedRadiusV=%.9g SharedFlattening=%.9g RegionARmsCm=%.9g RegionAMaxCm=%.9g RegionBRmsCm=%.9g RegionBMaxCm=%.9g BalancedRmsCm=%.9g NormalAdjustmentDeg=%.9g"),
				SharedPairRank, static_cast<uint8>(Pair.SymmetryAxis),
				RegionA.RegionId, RegionB.RegionId,
				Pair.bBothBoundaryEvidenceUsable ? 1 : 0,
				Pair.bPlausible ? 1 : 0, Pair.SharedRadiusU,
				Pair.SharedRadiusV, Pair.SharedFlatteningFraction,
				Pair.RegionARootMeanSquareResidualCm,
				Pair.RegionAMaximumResidualCm,
				Pair.RegionBRootMeanSquareResidualCm,
				Pair.RegionBMaximumResidualCm,
				Pair.BalancedRootMeanSquareResidualCm,
				Pair.MaximumEndpointNormalAdjustmentDegrees);
			if (++SharedPairRank >= 32) break;
		}
		int32 FrameRank = 0;
		for (const Speed::Analytic::FSharedC2FrameLedgerEntry& Entry :
			Runtime->SharedC2FrameLedger)
		{
			const Speed::Analytic::FSharedC2TransitionPairFit& Pair =
				Runtime->SharedC2TransitionPairFits[Entry.SharedPairFitIndex];
			const Speed::Analytic::FExtrusionSurfaceRegion& RegionA =
				Runtime->ExtrusionSurfaceRegions[Pair.RegionAIndex];
			const Speed::Analytic::FExtrusionSurfaceRegion& RegionB =
				Runtime->ExtrusionSurfaceRegions[Pair.RegionBIndex];
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticSharedC2Frame] Rank=%d RegionA=%016llX RegionB=%016llX AxisA=%s AxisB=%s AxisAdjustmentDeg=%.9g PlaneNormalAdjustmentDeg=%.9g PlaneSourceRmsCm=%.9g PlaneSourceMaxCm=%.9g OriginalEndpointPlaneCm=%.9g ConstructedEndpointCm=%.9g TangentDirectionResidual=%.9g MinimumTangentMagnitude=%.9g EndpointSecondDerivative=%.9g C0=%d G1=%d G2=%d C2=%d SourcePlausible=%d"),
				FrameRank, RegionA.RegionId, RegionB.RegionId,
				*Entry.RegularizedAxisA.ToString(),
				*Entry.RegularizedAxisB.ToString(),
				Entry.MaximumExtrusionAxisAdjustmentDegrees,
				Entry.MaximumPlaneNormalAdjustmentDegrees,
				Entry.PlanarSourceRootMeanSquareResidualCm,
				Entry.PlanarSourceMaximumResidualCm,
				Entry.MaximumOriginalPlaneEndpointResidualCm,
				Entry.MaximumConstructedEndpointPositionResidualCm,
				Entry.MaximumEndpointTangentDirectionResidual,
				Entry.MinimumEndpointTangentMagnitude,
				Entry.MaximumEndpointSecondDerivativeMagnitude,
				Entry.bC0ByConstruction ? 1 : 0,
				Entry.bG1ByConstruction ? 1 : 0,
				Entry.bG2ByConstruction ? 1 : 0,
				Entry.bParametricC2ByConstruction ? 1 : 0,
				Entry.bSourceRegularizationPlausible ? 1 : 0);
			if (++FrameRank >= 32) break;
		}
		for (const Speed::Analytic::FGlobalC2PlaneConstraint& Constraint :
			Runtime->GlobalC2PlaneConstraints)
		{
			const Speed::Analytic::FPlanarSurfaceGroup& Group =
				Runtime->PlanarSurfaceGroups[Constraint.PlanarGroupIndex];
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticGlobalC2Plane] Group=%016llX Requests=%d Normal=%s Offset=%.9g NormalConflictDeg=%.9g EndpointConflictCm=%.9g SourceNormalAdjustmentDeg=%.9g SourceRmsCm=%.9g SourceMaxCm=%.9g Compatible=%d"),
				Group.GroupId, Constraint.RequestCount,
				*Constraint.Normal.ToString(), Constraint.PlaneOffset,
				Constraint.MaximumRequestedNormalConflictDegrees,
				Constraint.MaximumRequestedEndpointResidualCm,
				Constraint.SourceNormalAdjustmentDegrees,
				Constraint.SourceRootMeanSquareResidualCm,
				Constraint.SourceMaximumResidualCm,
				Constraint.bConstraintCompatible ? 1 : 0);
		}
		for (const Speed::Analytic::FGlobalC2JoinCertificate& Certificate :
			Runtime->GlobalC2JoinCertificates)
		{
			const Speed::Analytic::FSharedC2TransitionPairFit& Pair =
				Runtime->SharedC2TransitionPairFits[
					Certificate.SharedPairFitIndex];
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticGlobalC2Join] RegionA=%016llX RegionB=%016llX Boundaries=%d EndpointPlaneCm=%.9g TangentNormalDeg=%.9g C0=%d G1=%d LocalC2Retained=%d"),
				Runtime->ExtrusionSurfaceRegions[Pair.RegionAIndex].RegionId,
				Runtime->ExtrusionSurfaceRegions[Pair.RegionBIndex].RegionId,
				Certificate.BoundaryCount,
				Certificate.MaximumEndpointPlaneResidualCm,
				Certificate.MaximumTangentNormalResidualDegrees,
				Certificate.bNetworkC0Plausible ? 1 : 0,
				Certificate.bNetworkG1Plausible ? 1 : 0,
				Certificate.bLocalC2Retained ? 1 : 0);
		}
		const Speed::Analytic::FC2SymmetryPlacementFrame& Placement =
			Runtime->C2SymmetryPlacementFrame;
		UE_LOG(LogTemp, Display,
			TEXT("[AnalyticC2SymmetryFrame] BoundsCenter=%s SourceInferredCenter=%s TargetCenter=%s XConstraints=%d XRmsCm=%.9g XMaxCm=%.9g YConstraints=%d YRmsCm=%.9g YMaxCm=%.9g Fitted=%d"),
			*Placement.BoundsCenter.ToString(),
			*Placement.SourceInferredCenter.ToString(),
			*Placement.TargetCenter.ToString(),
			Placement.XConstraintCount,
			Placement.XRootMeanSquareResidualCm,
			Placement.XMaximumResidualCm,
			Placement.YConstraintCount,
			Placement.YRootMeanSquareResidualCm,
			Placement.YMaximumResidualCm,
			Placement.bFittedFromMirrorPlanes ? 1 : 0);
		for (const Speed::Analytic::FSymmetrizedC2PlaneConstraint& Constraint :
			Runtime->SymmetrizedC2PlaneConstraints)
		{
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticSymmetrizedC2Plane] Orbit=%016llX Group=%016llX TransformMask=%u Normal=%s Offset=%.9g SourceNormalAdjustmentDeg=%.9g SourceRmsCm=%.9g SourceMaxCm=%.9g MirrorNormalDeg=%.9g MirrorOffsetCm=%.9g SourcePlausible=%d ExactMirror=%d"),
				Constraint.OrbitId,
				Runtime->PlanarSurfaceGroups[
					Constraint.PlanarGroupIndex].GroupId,
				Constraint.TransformMaskFromOrbitRoot,
				*Constraint.Normal.ToString(), Constraint.PlaneOffset,
				Constraint.SourceNormalAdjustmentDegrees,
				Constraint.SourceRootMeanSquareResidualCm,
				Constraint.SourceMaximumResidualCm,
				Constraint.MaximumMirrorNormalResidualDegrees,
				Constraint.MaximumMirrorOffsetResidualCm,
				Constraint.bSourceFitPlausible ? 1 : 0,
				Constraint.bExactMirrorPlacement ? 1 : 0);
		}
		for (const Speed::Analytic::FCoupledC2TransitionPairSolution& Solution :
			Runtime->CoupledC2TransitionSolutions)
		{
			const Speed::Analytic::FSharedC2TransitionPairFit& Pair =
				Runtime->SharedC2TransitionPairFits[Solution.SharedPairFitIndex];
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticCoupledC2Solution] RegionA=%016llX RegionB=%016llX RadiusU=%.9g RadiusV=%.9g Flattening=%.9g CenterA=%s CenterB=%s RegionARmsCm=%.9g RegionAMaxCm=%.9g RegionBRmsCm=%.9g RegionBMaxCm=%.9g BalancedRmsCm=%.9g PlaneOrthogonality=%.9g EndpointPlaneCm=%.9g TangentPlaneResidual=%.9g EndpointSecondDerivative=%.9g ExactC0G1C2=%d SourcePlausible=%d"),
				Runtime->ExtrusionSurfaceRegions[Pair.RegionAIndex].RegionId,
				Runtime->ExtrusionSurfaceRegions[Pair.RegionBIndex].RegionId,
				Solution.SharedRadiusU, Solution.SharedRadiusV,
				Solution.SharedFlatteningFraction,
				*Solution.RegionACenterCoordinates.ToString(),
				*Solution.RegionBCenterCoordinates.ToString(),
				Solution.RegionARootMeanSquareResidualCm,
				Solution.RegionAMaximumResidualCm,
				Solution.RegionBRootMeanSquareResidualCm,
				Solution.RegionBMaximumResidualCm,
				Solution.BalancedRootMeanSquareResidualCm,
				Solution.MaximumPlaneOrthogonalityResidual,
				Solution.MaximumEndpointPlaneResidualCm,
				Solution.MaximumEndpointTangentPlaneResidual,
				Solution.MaximumEndpointSecondDerivativeMagnitude,
				Solution.bExactC0G1C2ByConstruction ? 1 : 0,
				Solution.bSourceFitPlausible ? 1 : 0);
		}
		for (const Speed::Analytic::FCoupledC2TransitionFamilySolution& Family :
			Runtime->CoupledC2TransitionFamilies)
		{
			FString RegionIds;
			for (int32 Offset = 0; Offset < Family.RegionCount; ++Offset)
			{
				const int32 RegionIndex = Runtime->CoupledC2FamilyRegionIndices[
					Family.FirstRegionIndex + Offset];
				RegionIds += FString::Printf(TEXT("%s%016llX"),
					Offset == 0 ? TEXT("") : TEXT(","),
					Runtime->ExtrusionSurfaceRegions[RegionIndex].RegionId);
			}
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticCoupledC2Family] Family=%016llX Regions=%s RegionCount=%d PairConstraints=%d AxisMask=%u RadiusU=%.9g RadiusV=%.9g Flattening=%.9g BalancedRmsCm=%.9g WorstRegionRmsCm=%.9g MaximumCm=%.9g EndpointPlaneCm=%.9g TangentPlaneResidual=%.9g EndpointSecondDerivative=%.9g ExactC0G1C2=%d SourcePlausible=%d"),
				Family.FamilyId, *RegionIds, Family.RegionCount,
				Family.PairConstraintCount, Family.SymmetryAxisMask,
				Family.SharedRadiusU, Family.SharedRadiusV,
				Family.SharedFlatteningFraction,
				Family.BalancedRootMeanSquareResidualCm,
				Family.MaximumRegionRootMeanSquareResidualCm,
				Family.MaximumResidualCm,
				Family.MaximumEndpointPlaneResidualCm,
				Family.MaximumEndpointTangentPlaneResidual,
				Family.MaximumEndpointSecondDerivativeMagnitude,
				Family.bExactC0G1C2ByConstruction ? 1 : 0,
				Family.bSourceFitPlausible ? 1 : 0);
		}
		for (const Speed::Analytic::FC2TransitionCoverageEntry& Entry :
			Runtime->C2TransitionCoverage)
		{
			const Speed::Analytic::FExtrusionSurfaceRegion& Region =
				Runtime->ExtrusionSurfaceRegions[Entry.ExtrusionRegionIndex];
			auto RegionIdOrZero = [&Runtime](const int32 RegionIndex)
			{
				return Runtime->ExtrusionSurfaceRegions.IsValidIndex(RegionIndex)
					? Runtime->ExtrusionSurfaceRegions[RegionIndex].RegionId : 0ull;
			};
			const int32 RegionCount = Runtime->ExtrusionSurfaceRegions.Num();
			const Speed::Analytic::FExtrusionRegionMirrorMatch* XMatch =
				Runtime->ExtrusionRegionMirrorMatches.IsValidIndex(
					Entry.ExtrusionRegionIndex)
				? &Runtime->ExtrusionRegionMirrorMatches[
					Entry.ExtrusionRegionIndex] : nullptr;
			const Speed::Analytic::FExtrusionRegionMirrorMatch* YMatch =
				Runtime->ExtrusionRegionMirrorMatches.IsValidIndex(
					RegionCount + Entry.ExtrusionRegionIndex)
				? &Runtime->ExtrusionRegionMirrorMatches[
					RegionCount + Entry.ExtrusionRegionIndex] : nullptr;
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticC2Coverage] Region=%016llX Kind=%u Layer=%u Family=%d Axis=%s BoundsMin=%s BoundsMax=%s HorizontalPlane=%016llX HorizontalHeightCm=%.9g XTarget=%016llX XPlausible=%d XReciprocal=%d XSelf=%d XAxisDeg=%.9g XCenterLineCm=%.9g XRadiusURel=%.9g XRadiusVRel=%.9g XBoundsCm=%.9g XAreaRel=%.9g YTarget=%016llX YPlausible=%d YReciprocal=%d YSelf=%d YAxisDeg=%.9g YCenterLineCm=%.9g YRadiusURel=%.9g YRadiusVRel=%.9g YBoundsCm=%.9g YAreaRel=%.9g BoundaryPositionCm=%.9g BoundaryNormalDeg=%.9g"),
				Region.RegionId, static_cast<uint8>(Entry.Kind),
				static_cast<uint8>(Entry.SurfaceLayer), Entry.FamilyIndex,
				*Region.Axis.ToString(), *Region.Bounds.Min.ToString(),
				*Region.Bounds.Max.ToString(),
				Runtime->PlanarSurfaceGroups.IsValidIndex(
					Entry.HorizontalPlanarGroupIndex)
					? Runtime->PlanarSurfaceGroups[
						Entry.HorizontalPlanarGroupIndex].GroupId : 0ull,
				Entry.HorizontalPlaneHeightCm,
				RegionIdOrZero(Entry.XMirrorTargetRegionIndex),
				Entry.bXMirrorPlausible ? 1 : 0,
				XMatch && XMatch->bReciprocal ? 1 : 0,
				Entry.bXSelfMirror ? 1 : 0,
				XMatch ? XMatch->AxisAngleResidualDegrees : 0.0,
				XMatch ? XMatch->CenterLineResidualCm : 0.0,
				XMatch ? XMatch->RelativeRadiusUResidual : 0.0,
				XMatch ? XMatch->RelativeRadiusVResidual : 0.0,
				XMatch ? XMatch->BoundsResidualCm : 0.0,
				XMatch ? XMatch->RelativeAreaResidual : 0.0,
				RegionIdOrZero(Entry.YMirrorTargetRegionIndex),
				Entry.bYMirrorPlausible ? 1 : 0,
				YMatch && YMatch->bReciprocal ? 1 : 0,
				Entry.bYSelfMirror ? 1 : 0,
				YMatch ? YMatch->AxisAngleResidualDegrees : 0.0,
				YMatch ? YMatch->CenterLineResidualCm : 0.0,
				YMatch ? YMatch->RelativeRadiusUResidual : 0.0,
				YMatch ? YMatch->RelativeRadiusVResidual : 0.0,
				YMatch ? YMatch->BoundsResidualCm : 0.0,
				YMatch ? YMatch->RelativeAreaResidual : 0.0,
				Entry.MaximumBoundaryPositionResidualCm,
				Entry.MaximumBoundaryNormalResidualDegrees);
		}
		for (const Speed::Analytic::FPlayableC2OrbitCandidate& Candidate :
			Runtime->PlayableC2OrbitCandidates)
		{
			const Speed::Analytic::FC2TransitionCoverageEntry& Coverage =
				Runtime->C2TransitionCoverage[Candidate.SeedCoverageIndex];
			FString Members;
			for (int32 Offset = 0; Offset < Candidate.MemberCount; ++Offset)
			{
				const Speed::Analytic::FPlayableC2OrbitMember& Member =
					Runtime->PlayableC2OrbitMembers[
						Candidate.FirstMemberIndex + Offset];
				Members += FString::Printf(TEXT("%s%u:%016llX:%.9g:%.9g"),
					Offset == 0 ? TEXT("") : TEXT(","),
					Member.TransformMaskFromSeed,
					Runtime->ExtrusionSurfaceRegions[
						Member.ExtrusionRegionIndex].RegionId,
					Member.RootMeanSquareResidualCm,
					Member.MaximumResidualCm);
			}
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticPlayableC2Orbit] Orbit=%016llX Seed=%016llX Members=%s MemberCount=%d Expected=%d AxisMask=%u Complete=%d TopologyPlausible=%d RadiusU=%.9g RadiusV=%.9g Flattening=%.9g BalancedRmsCm=%.9g WorstRegionRmsCm=%.9g MaximumCm=%.9g MirrorAxisDeg=%.9g MirrorCenterLineCm=%.9g MirrorBoundsCm=%.9g MirrorAreaRel=%.9g EndpointPlaneCm=%.9g TangentPlaneResidual=%.9g EndpointSecondDerivative=%.9g ExactC0G1C2=%d SourcePlausible=%d"),
				Candidate.OrbitId,
				Runtime->ExtrusionSurfaceRegions[
					Coverage.ExtrusionRegionIndex].RegionId,
				*Members, Candidate.MemberCount,
				Candidate.ExpectedMemberCount, Candidate.SymmetryAxisMask,
				Candidate.bOrbitComplete ? 1 : 0,
				Candidate.bMirrorTopologyPlausible ? 1 : 0,
				Candidate.SharedRadiusU, Candidate.SharedRadiusV,
				Candidate.SharedFlatteningFraction,
				Candidate.BalancedRootMeanSquareResidualCm,
				Candidate.MaximumRegionRootMeanSquareResidualCm,
				Candidate.MaximumResidualCm,
				Candidate.MaximumMirrorAxisResidualDegrees,
				Candidate.MaximumMirrorCenterLineResidualCm,
				Candidate.MaximumMirrorBoundsResidualCm,
				Candidate.MaximumMirrorRelativeAreaResidual,
				Candidate.MaximumEndpointPlaneResidualCm,
				Candidate.MaximumEndpointTangentPlaneResidual,
				Candidate.MaximumEndpointSecondDerivativeMagnitude,
				Candidate.bExactC0G1C2ByConstruction ? 1 : 0,
				Candidate.bSourceFitPlausible ? 1 : 0);
		}
		for (const Speed::Analytic::FPlayableC2TriangleSupportCandidate& Candidate :
			Runtime->PlayableC2TriangleSupportCandidates)
		{
			const Speed::Analytic::FC2TransitionCoverageEntry& Coverage =
				Runtime->C2TransitionCoverage[Candidate.SeedCoverageIndex];
			FString Members;
			for (int32 Offset = 0; Offset < Candidate.MemberCount; ++Offset)
			{
				const Speed::Analytic::FPlayableC2TriangleSupportMember& Member =
					Runtime->PlayableC2TriangleSupportMembers[
						Candidate.FirstMemberIndex + Offset];
				Members += FString::Printf(
					TEXT("%s%u:%d/%d:%d:%.9g:%.9g:%.9g:%.9g:%.9g:%.9g"),
					Offset == 0 ? TEXT("") : TEXT(","),
					Member.TransformMaskFromSeed,
					Member.MatchedSampleCount, Member.SampleCount,
					Member.UniqueTargetTriangleCount,
					Member.RootMeanSquarePositionResidualCm,
					Member.MaximumPositionResidualCm,
					Member.RootMeanSquareNormalResidualDegrees,
					Member.MaximumNormalResidualDegrees,
					Member.ProfileRootMeanSquareResidualCm,
					Member.ProfileMaximumResidualCm);
			}
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticPlayableC2TriangleSupport] Orbit=%016llX Seed=%016llX SmoothRegion=%d Members=%s MemberCount=%d Expected=%d Complete=%d PositionRmsCm=%.9g PositionMaxCm=%.9g NormalRmsDeg=%.9g NormalMaxDeg=%.9g Plausible=%d RadiusU=%.9g RadiusV=%.9g Flattening=%.9g ProfileBalancedRmsCm=%.9g ProfileWorstRmsCm=%.9g ProfileMaxCm=%.9g EndpointPlaneCm=%.9g TangentPlaneResidual=%.9g EndpointSecondDerivative=%.9g ExactC0G1C2=%d SourcePlausible=%d NetworkRadiusU=%.9g NetworkRadiusV=%.9g NetworkFlattening=%.9g NetworkProfileBalancedRmsCm=%.9g NetworkProfileWorstRmsCm=%.9g NetworkProfileMaxCm=%.9g NetworkEndpointPlaneCm=%.9g NetworkTangentPlaneResidual=%.9g NetworkEndpointSecondDerivative=%.9g NetworkExactC0G1C2=%d NetworkSourcePlausible=%d"),
				Candidate.OrbitId,
				Runtime->ExtrusionSurfaceRegions[
					Coverage.ExtrusionRegionIndex].RegionId,
				Candidate.SmoothRegionIndex, *Members,
				Candidate.MemberCount, Candidate.ExpectedMemberCount,
				Candidate.bComplete ? 1 : 0,
				Candidate.MaximumMemberRootMeanSquarePositionResidualCm,
				Candidate.MaximumPositionResidualCm,
				Candidate.MaximumMemberRootMeanSquareNormalResidualDegrees,
				Candidate.MaximumNormalResidualDegrees,
				Candidate.bSupportPlausible ? 1 : 0,
				Candidate.SharedRadiusU, Candidate.SharedRadiusV,
				Candidate.SharedFlatteningFraction,
				Candidate.BalancedProfileRootMeanSquareResidualCm,
				Candidate.MaximumMemberProfileRootMeanSquareResidualCm,
				Candidate.MaximumProfileResidualCm,
				Candidate.MaximumEndpointPlaneResidualCm,
				Candidate.MaximumEndpointTangentPlaneResidual,
				Candidate.MaximumEndpointSecondDerivativeMagnitude,
				Candidate.bExactC0G1C2ByConstruction ? 1 : 0,
				Candidate.bSourceFitPlausible ? 1 : 0,
				Candidate.NetworkSharedRadiusU,
				Candidate.NetworkSharedRadiusV,
				Candidate.NetworkSharedFlatteningFraction,
				Candidate.NetworkBalancedProfileRootMeanSquareResidualCm,
				Candidate.NetworkMaximumMemberProfileRootMeanSquareResidualCm,
				Candidate.NetworkMaximumProfileResidualCm,
				Candidate.NetworkMaximumEndpointPlaneResidualCm,
				Candidate.NetworkMaximumEndpointTangentPlaneResidual,
				Candidate.NetworkMaximumEndpointSecondDerivativeMagnitude,
				Candidate.bNetworkExactC0G1C2ByConstruction ? 1 : 0,
				Candidate.bNetworkSourceFitPlausible ? 1 : 0);
		}
		for (const Speed::Analytic::FPlayableC2PlaneBinding& Binding :
			Runtime->PlayableC2PlaneBindings)
		{
			const Speed::Analytic::FPlayableC2TriangleSupportCandidate& Candidate =
				Runtime->PlayableC2TriangleSupportCandidates[
					Binding.TriangleSupportCandidateIndex];
			const Speed::Analytic::FC2TransitionCoverageEntry& Coverage =
				Runtime->C2TransitionCoverage[Candidate.SeedCoverageIndex];
			const Speed::Analytic::FSymmetrizedC2PlaneConstraint* Constraint =
				Runtime->SymmetrizedC2PlaneConstraints.IsValidIndex(
					Binding.SymmetrizedConstraintIndex)
				? &Runtime->SymmetrizedC2PlaneConstraints[
					Binding.SymmetrizedConstraintIndex] : nullptr;
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticPlayableC2PlaneBinding] Orbit=%016llX Seed=%016llX Endpoint=%u SourceGroup=%016llX SourceNormal=%s SourceOffset=%.9g TargetOrbit=%016llX TargetGroup=%016llX Position=%s NormalDeg=%.9g EndpointPlaneCm=%.9g OffsetCm=%.9g SameSource=%d ExistingCompatible=%d NetworkIndex=%d NetworkNormalDeg=%.9g NetworkEndpointPlaneCm=%.9g NetworkOffsetCm=%.9g NetworkCompatible=%d"),
				Candidate.OrbitId,
				Runtime->ExtrusionSurfaceRegions[
					Coverage.ExtrusionRegionIndex].RegionId,
				static_cast<uint8>(Binding.Endpoint),
				Runtime->PlanarSurfaceGroups.IsValidIndex(
					Binding.SourcePlanarGroupIndex)
					? Runtime->PlanarSurfaceGroups[
						Binding.SourcePlanarGroupIndex].GroupId : 0ull,
				Runtime->PlanarSurfaceGroups.IsValidIndex(
					Binding.SourcePlanarGroupIndex)
					? *Runtime->PlanarSurfaceGroups[
						Binding.SourcePlanarGroupIndex].Normal.ToString()
					: TEXT("invalid"),
				Runtime->PlanarSurfaceGroups.IsValidIndex(
					Binding.SourcePlanarGroupIndex)
					? Runtime->PlanarSurfaceGroups[
						Binding.SourcePlanarGroupIndex].PlaneOffset : 0.0,
				Constraint ? Constraint->OrbitId : 0ull,
				Constraint && Runtime->PlanarSurfaceGroups.IsValidIndex(
					Constraint->PlanarGroupIndex)
					? Runtime->PlanarSurfaceGroups[
						Constraint->PlanarGroupIndex].GroupId : 0ull,
				*Binding.EndpointPosition.ToString(),
				Binding.NormalResidualDegrees,
				Binding.EndpointPlaneResidualCm,
				Binding.OrientedOffsetResidualCm,
				Binding.bSameSourcePlanarGroup ? 1 : 0,
				Binding.bCompatible ? 1 : 0,
				Binding.NetworkPlaneConstraintIndex,
				Binding.NetworkNormalResidualDegrees,
				Binding.NetworkEndpointPlaneResidualCm,
				Binding.NetworkOrientedOffsetResidualCm,
				Binding.bNetworkCompatible ? 1 : 0);
		}
		for (const Speed::Analytic::FPlayableC2NetworkPlaneConstraint& Plane :
			Runtime->PlayableC2NetworkPlaneConstraints)
		{
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticPlayableC2NetworkPlane] Orbit=%016llX TransformMask=%u SourceGroup=%016llX Normal=%s Offset=%.9g Observations=%d SourceNormalDeg=%.9g SourceOffsetCm=%.9g Inherited=%d SourcePlausible=%d ExactMirror=%d"),
				Plane.OrbitId, Plane.TransformMaskFromOrbitRoot,
				Runtime->PlanarSurfaceGroups.IsValidIndex(
					Plane.SourcePlanarGroupIndex)
					? Runtime->PlanarSurfaceGroups[
						Plane.SourcePlanarGroupIndex].GroupId : 0ull,
				*Plane.Normal.ToString(), Plane.PlaneOffset,
				Plane.SourceObservationCount,
				Plane.MaximumSourceNormalResidualDegrees,
				Plane.MaximumSourceOffsetResidualCm,
				Plane.bInheritedFromSymmetrizedNetwork ? 1 : 0,
				Plane.bSourceFitPlausible ? 1 : 0,
				Plane.bExactMirrorPlacement ? 1 : 0);
		}
		for (const Speed::Analytic::FEndWallBoundaryComponent& Component :
			Runtime->EndWallBoundaryComponents)
		{
			const Speed::Analytic::FPlayableC2NetworkPlaneConstraint& Plane =
				Runtime->PlayableC2NetworkPlaneConstraints[
					Component.NetworkPlaneConstraintIndex];
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticEndWallBoundary] Id=%016llX PlaneOrbit=%016llX PlaneOffset=%.9g Group=%016llX Edges=%d Vertices=%d AdjacentTriangles=%d PerimeterCm=%.9g BoundsMin=%s BoundsMax=%s Layer=%u Closed=%d CrossX=%d"),
				Component.ComponentId, Plane.OrbitId, Plane.PlaneOffset,
				Runtime->PlanarSurfaceGroups[
					Component.PlanarGroupIndex].GroupId,
				Component.EdgeCount, Component.UniqueVertexCount,
				Component.AdjacentTriangleCount, Component.PerimeterCm,
				*Component.Bounds.Min.ToString(),
				*Component.Bounds.Max.ToString(),
				static_cast<uint8>(Component.SurfaceLayer),
				Component.bClosedLoop ? 1 : 0,
				Component.bCrossesXSymmetryPlane ? 1 : 0);
			for (int32 EdgeOffset = 0;
				EdgeOffset < Component.EdgeCount; ++EdgeOffset)
			{
				const int32 EdgeIndex = Runtime->EndWallBoundaryEdgeIndices[
					Component.FirstEdgeIndex + EdgeOffset];
				const Speed::Analytic::FTriangleMeshEdge& Edge =
					Runtime->MeshEdges[EdgeIndex];
				int32 AdjacentTriangleIndex = INDEX_NONE;
				for (int32 IncidentOffset = 0;
					IncidentOffset < Edge.IncidentTriangleCount; ++IncidentOffset)
				{
					const int32 TriangleIndex = Runtime->EdgeIncidentTriangleIndices[
						Edge.FirstIncidentTriangle + IncidentOffset];
					const Speed::Analytic::FTriangleSurface& Triangle =
						Runtime->Triangles[TriangleIndex];
					const double MaximumPlaneDistance = FMath::Max3(
						FMath::Abs(FVector3d::DotProduct(
							Runtime->PlanarSurfaceGroups[
								Component.PlanarGroupIndex].Normal,
							Triangle.Vertices[0]) -
							Runtime->PlanarSurfaceGroups[
								Component.PlanarGroupIndex].PlaneOffset),
						FMath::Abs(FVector3d::DotProduct(
							Runtime->PlanarSurfaceGroups[
								Component.PlanarGroupIndex].Normal,
							Triangle.Vertices[1]) -
							Runtime->PlanarSurfaceGroups[
								Component.PlanarGroupIndex].PlaneOffset),
						FMath::Abs(FVector3d::DotProduct(
							Runtime->PlanarSurfaceGroups[
								Component.PlanarGroupIndex].Normal,
							Triangle.Vertices[2]) -
							Runtime->PlanarSurfaceGroups[
								Component.PlanarGroupIndex].PlaneOffset));
					if (MaximumPlaneDistance > 0.01)
					{
						AdjacentTriangleIndex = TriangleIndex;
						break;
					}
				}
				const Speed::Analytic::FTriangleSurface* Adjacent =
					Runtime->Triangles.IsValidIndex(AdjacentTriangleIndex)
					? &Runtime->Triangles[AdjacentTriangleIndex] : nullptr;
				UE_LOG(LogTemp, Display,
					TEXT("[AnalyticEndWallBoundaryEdge] Component=%016llX Edge=%d A=%s B=%s LengthCm=%.9g AdjacentPrimitive=%016llX AdjacentNormal=%s AdjacentBoundsMin=%s AdjacentBoundsMax=%s"),
					Component.ComponentId, EdgeIndex,
					*Runtime->MeshVertices[Edge.VertexA].ToString(),
					*Runtime->MeshVertices[Edge.VertexB].ToString(),
					FVector3d::Distance(Runtime->MeshVertices[Edge.VertexA],
						Runtime->MeshVertices[Edge.VertexB]),
					Adjacent ? Adjacent->PrimitiveId : 0ull,
					Adjacent ? *Adjacent->FaceNormal.ToString() : TEXT("invalid"),
					Adjacent ? *Adjacent->Bounds.Min.ToString() : TEXT("invalid"),
					Adjacent ? *Adjacent->Bounds.Max.ToString() : TEXT("invalid"));
			}
		}
		for (const Speed::Analytic::FOpenRimCandidate& Candidate :
			Runtime->OpenRimCandidates)
		{
			const Speed::Analytic::FEndWallBoundaryComponent& Component =
				Runtime->EndWallBoundaryComponents[
					Candidate.WallBoundaryComponentIndex];
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticOpenRim] Id=%016llX Wall=%016llX Axis=%u Layer=%u Edges=%d Vertices=%d ArcLengthCm=%.9g BaselineZ=%.9g BoundsMin=%s BoundsMax=%s OpenAtBaseline=%d CrossTransverse=%d MirrorTarget=%016llX MirrorRmsCm=%.9g MirrorMaxCm=%.9g MirrorPlausible=%d SelfRmsCm=%.9g SelfMaxCm=%.9g SelfPlausible=%d NegativeVerticalEdges=%d NegativeVerticalHeightCm=%.9g PositiveVerticalEdges=%d PositiveVerticalHeightCm=%.9g HorizontalSpanEdges=%d HorizontalSpanHeightCm=%.9g HorizontalSpanSpanCm=%.9g CornerEdges=%d Partitioned=%d"),
				Candidate.CandidateId, Component.ComponentId,
				Candidate.WallAxis, static_cast<uint8>(Candidate.SurfaceLayer),
				Candidate.EdgeCount, Candidate.UniqueVertexCount,
				Candidate.ArcLengthCm, Candidate.BaselineHeightCm,
				*Candidate.Bounds.Min.ToString(),
				*Candidate.Bounds.Max.ToString(),
				Candidate.bOpenAtBaseline ? 1 : 0,
				Candidate.bCrossesTransverseSymmetryPlane ? 1 : 0,
				Runtime->OpenRimCandidates.IsValidIndex(
					Candidate.LongitudinalMirrorCandidateIndex)
					? Runtime->OpenRimCandidates[
						Candidate.LongitudinalMirrorCandidateIndex].CandidateId : 0ull,
				Candidate.LongitudinalMirrorRootMeanSquareResidualCm,
				Candidate.LongitudinalMirrorMaximumResidualCm,
				Candidate.bLongitudinalMirrorPlausible ? 1 : 0,
				Candidate.TransverseSelfRootMeanSquareResidualCm,
				Candidate.TransverseSelfMaximumResidualCm,
				Candidate.bTransverseSelfPlausible ? 1 : 0,
				Candidate.NegativeVerticalEdgeCount, Candidate.NegativeVerticalHeightCm,
				Candidate.PositiveVerticalEdgeCount, Candidate.PositiveVerticalHeightCm,
				Candidate.HorizontalSpanEdgeCount, Candidate.HorizontalSpanHeightCm,
				Candidate.HorizontalSpanSpanCm, Candidate.UpperTransitionEdgeCount,
				Candidate.bFeaturePartitionComplete ? 1 : 0);
		}
		for (const Speed::Analytic::FCanonicalOpenArchSolution& Solution :
			Runtime->CanonicalOpenArchSolutions)
		{
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticCanonicalOpenArch] Id=%016llX Members=%d BaselineZ=%.9g VerticalSegmentHalfWidth=%.9g TransitionZ=%.9g HorizontalSpanHalfSpan=%.9g HorizontalSpanZ=%.9g Flattening=%.9g BalancedRmsCm=%.9g WorstMemberRmsCm=%.9g MaxCm=%.9g EndpointPositionCm=%.9g EndpointTangentDeg=%.9g EndpointSecond=%.9g Complete=%d ExactXY=%d ExactC2=%d Plausible=%d"),
				Solution.SolutionId, Solution.MemberCount,
				Solution.BaselineHeightCm, Solution.VerticalSegmentHalfWidthCm,
				Solution.VerticalSegmentTransitionHeightCm, Solution.HorizontalSpanHalfSpanCm,
				Solution.HorizontalSpanHeightCm, Solution.FlatteningFraction,
				Solution.BalancedRootMeanSquareResidualCm,
				Solution.MaximumMemberRootMeanSquareResidualCm,
				Solution.MaximumResidualCm,
				Solution.MaximumEndpointPositionResidualCm,
				Solution.MaximumEndpointTangentResidualDegrees,
				Solution.MaximumEndpointSecondDerivativeMagnitude,
				Solution.bCompleteFourMemberOrbit ? 1 : 0,
				Solution.bExactXYMirrorPlacement ? 1 : 0,
				Solution.bExactC0G1C2ByConstruction ? 1 : 0,
				Solution.bSourceRegularizationPlausible ? 1 : 0);
			for (int32 MemberOffset = 0;
				MemberOffset < Solution.MemberCount; ++MemberOffset)
			{
				const Speed::Analytic::FCanonicalOpenArchMemberFit& Fit =
					Runtime->CanonicalOpenArchMemberFits[
						Solution.FirstMemberIndex + MemberOffset];
				UE_LOG(LogTemp, Display,
					TEXT("[AnalyticCanonicalOpenArchMember] Arch=%016llX Rim=%016llX Side=%u Samples=%d RmsCm=%.9g MaxCm=%.9g"),
					Solution.SolutionId,
					Runtime->OpenRimCandidates[
						Fit.OpenRimCandidateIndex].CandidateId,
					Fit.TransverseSide, Fit.SampleCount,
					Fit.RootMeanSquareResidualCm, Fit.MaximumResidualCm);
			}
		}
		for (const Speed::Analytic::FOpenRimSurfaceBandObservation& Observation :
			Runtime->OpenRimSurfaceBandObservations)
		{
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticOpeningSurfaceBand] Rim=%016llX Edge=%d Feature=%u BoundaryPlanePrimitive=%016llX OpeningPrimitive=%016llX EdgeLengthCm=%.9g DihedralDeg=%.9g NormalJumpDeg=%.9g LongitudinalStepCm=%.9g StepAxisResidualDeg=%.9g SameSmooth=%d SourceG1=%d"),
				Runtime->OpenRimCandidates[
					Observation.OpenRimCandidateIndex].CandidateId,
				Observation.RimEdgeIndex,
				static_cast<uint8>(Observation.Feature),
				Runtime->Triangles[Observation.BoundaryPlaneTriangleIndex].PrimitiveId,
				Runtime->Triangles[Observation.OpeningSurfaceTriangleIndex].PrimitiveId,
				Observation.EdgeLengthCm, Observation.FaceDihedralDegrees,
				Observation.ImportedNormalJumpDegrees,
				Observation.FirstLongitudinalStepCm,
				Observation.FirstStepAxisResidualDegrees,
				Observation.bSameSmoothRegion ? 1 : 0,
				Observation.bSourceG1Plausible ? 1 : 0);
		}
		TArray<int32> RegionOrder;
		RegionOrder.SetNumUninitialized(Runtime->SmoothSurfaceRegions.Num());
		for (int32 Index = 0; Index < RegionOrder.Num(); ++Index) RegionOrder[Index] = Index;
		Algo::Sort(RegionOrder, [&Runtime](const int32 A, const int32 B)
		{
			const Speed::Analytic::FSmoothSurfaceRegion& RegionA =
				Runtime->SmoothSurfaceRegions[A];
			const Speed::Analytic::FSmoothSurfaceRegion& RegionB =
				Runtime->SmoothSurfaceRegions[B];
			return RegionA.Area != RegionB.Area
				? RegionA.Area > RegionB.Area
				: RegionA.RegionId < RegionB.RegionId;
		});
		for (int32 Rank = 0; Rank < FMath::Min(12, RegionOrder.Num()); ++Rank)
		{
			const Speed::Analytic::FSmoothSurfaceRegion& Region =
				Runtime->SmoothSurfaceRegions[RegionOrder[Rank]];
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticSmoothRegion] Rank=%d Id=%016llX Triangles=%d Area=%.9g NormalSpanDeg=%.9g BoundsMin=%s BoundsMax=%s"),
				Rank, Region.RegionId, Region.TriangleCount, Region.Area,
				Region.MaximumFaceNormalAngleDegrees, *Region.Bounds.Min.ToString(),
				*Region.Bounds.Max.ToString());
		}
		TArray<int32> CreaseOrder;
		int32 SmoothEdgesAboveOneDegree = 0;
		int32 SmoothEdgesAboveTenDegrees = 0;
		for (int32 EdgeIndex = 0; EdgeIndex < Runtime->MeshEdges.Num(); ++EdgeIndex)
		{
			const Speed::Analytic::FTriangleMeshEdge& Edge = Runtime->MeshEdges[EdgeIndex];
			if (Edge.Continuity == Speed::Analytic::EEdgeContinuity::Crease)
			{
				CreaseOrder.Add(EdgeIndex);
			}
			if (Edge.Continuity == Speed::Analytic::EEdgeContinuity::Smooth)
			{
				SmoothEdgesAboveOneDegree += Edge.DihedralAngleDegrees > 1.0 ? 1 : 0;
				SmoothEdgesAboveTenDegrees += Edge.DihedralAngleDegrees > 10.0 ? 1 : 0;
			}
		}
		Algo::Sort(CreaseOrder, [&Runtime](const int32 A, const int32 B)
		{
			const Speed::Analytic::FTriangleMeshEdge& EdgeA = Runtime->MeshEdges[A];
			const Speed::Analytic::FTriangleMeshEdge& EdgeB = Runtime->MeshEdges[B];
			return EdgeA.MaximumNormalDiscontinuityDegrees !=
				EdgeB.MaximumNormalDiscontinuityDegrees
				? EdgeA.MaximumNormalDiscontinuityDegrees >
					EdgeB.MaximumNormalDiscontinuityDegrees
				: A < B;
		});
		UE_LOG(LogTemp, Display,
			TEXT("[AnalyticContinuitySummary] SmoothDihedralAbove1Deg=%d SmoothDihedralAbove10Deg=%d Creases=%d"),
			SmoothEdgesAboveOneDegree, SmoothEdgesAboveTenDegrees, CreaseOrder.Num());
		for (int32 Rank = 0; Rank < CreaseOrder.Num(); ++Rank)
		{
			const int32 EdgeIndex = CreaseOrder[Rank];
			const Speed::Analytic::FTriangleMeshEdge& Edge = Runtime->MeshEdges[EdgeIndex];
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticCrease] Rank=%d Edge=%d A=%s B=%s DihedralDeg=%.9g NormalJumpDeg=%.9g"),
				Rank, EdgeIndex, *Runtime->MeshVertices[Edge.VertexA].ToString(),
				*Runtime->MeshVertices[Edge.VertexB].ToString(),
				Edge.DihedralAngleDegrees,
				Edge.MaximumNormalDiscontinuityDegrees);
		}
		return 0;
	}

	const FString MapPackageName = Named.FindRef(TEXT("Map"));
	if (!FPackageName::IsValidLongPackageName(MapPackageName))
	{
		UE_LOG(LogTemp, Error, TEXT("-Map=/Game/... is required."));
		return 1;
	}
	const FString MapFilename = FPackageName::LongPackageNameToFilename(
		MapPackageName, FPackageName::GetMapPackageExtension());
	if (!FEditorFileUtils::LoadMap(MapFilename, false, false))
	{
		UE_LOG(LogTemp, Error, TEXT("Could not load world %s."), *MapPackageName);
		return 3;
	}
	UWorld* World = GEditor ? GEditor->GetEditorWorldContext().World() : nullptr;
	if (!World)
	{
		UE_LOG(LogTemp, Error, TEXT("Editor world context is unavailable for %s."),
			*MapPackageName);
		return 3;
	}

	TArray<FSpeedAnalyticMeshSourceRecord> Records;
	TArray<FSpeedAnalyticTriangleRecord> TriangleRecords;
	for (ULevel* Level : World->GetLevels())
	{
		if (!Level) continue;
		for (AActor* Actor : Level->Actors)
		{
			if (!Actor) continue;
			const USpeedAnalyticSourceComponent* Source =
				Actor->FindComponentByClass<USpeedAnalyticSourceComponent>();
			if (!Source) continue;
			TArray<UStaticMeshComponent*> Components;
			if (const ASpeedStaticActor* StaticActor = Cast<ASpeedStaticActor>(Actor))
			{
				StaticActor->GatherAnalyticStaticMeshes(Components);
			}
			else
			{
				Source->GatherOwnedStaticMeshes(Components);
			}
			for (const UStaticMeshComponent* Component : Components)
			{
				FSpeedAnalyticMeshSourceRecord Record;
				FString Error;
				if (!Component || !ImportMeshSource(
					*Actor, *Component, Record, TriangleRecords, Error))
				{
					UE_LOG(LogTemp, Error, TEXT("[AnalyticBake] %s"), *Error);
					return 4;
				}
				Records.Add(MoveTemp(Record));
			}
		}
	}
	Records.Sort([](const FSpeedAnalyticMeshSourceRecord& A,
		const FSpeedAnalyticMeshSourceRecord& B)
	{
		return A.SourceId < B.SourceId;
	});
	TriangleRecords.Sort([](const FSpeedAnalyticTriangleRecord& A,
		const FSpeedAnalyticTriangleRecord& B)
	{
		return A.PrimitiveId < B.PrimitiveId;
	});
	TArray<Speed::Analytic::FBoundedPlane> LandscapePlanes;
	TArray<ALandscapeProxy*> Landscapes;
	for (TActorIterator<ALandscapeProxy> It(World); It; ++It)
	{
		Landscapes.Add(*It);
	}
	Landscapes.Sort([](const ALandscapeProxy& A, const ALandscapeProxy& B)
	{
		return A.GetPathName() < B.GetPathName();
	});
	for (const ALandscapeProxy* Landscape : Landscapes)
	{
		const Speed::Analytic::FFlatLandscapeAdapterOutput Output =
			Speed::Analytic::BuildFlatLandscapePlane(*Landscape, 0.001);
		UE_LOG(LogTemp, Display,
			TEXT("[AnalyticBakeLandscape] Source=%s Result=%u ResidualCm=%.9g Detail=%s"),
			*Landscape->GetPathName(), static_cast<uint8>(Output.Result),
			Output.MaximumHeightResidual, *Output.Diagnostic);
		if (Output.Result != Speed::Analytic::EFlatLandscapeAdapterResult::
			SuccessAuthorityEligible)
		{
			UE_LOG(LogTemp, Error,
				TEXT("Landscape %s cannot be certified for analytical authority."),
				*Landscape->GetPathName());
			return 4;
		}
		LandscapePlanes.Add(Output.Plane);
	}
	TArray<FVector> VertexPositions;
	TArray<FVector> VertexNormals;
	TArray<FSpeedAnalyticIndexedTriangleRecord> IndexedTriangleRecords;
	BuildIndexedPayload(TriangleRecords, VertexPositions, VertexNormals,
		IndexedTriangleRecords);
	if (Records.IsEmpty())
	{
		UE_LOG(LogTemp, Error, TEXT("No analytical static-mesh sources found in %s."),
			*MapPackageName);
		return 5;
	}

	uint64 SourceHash = 0;
	for (const FSpeedAnalyticMeshSourceRecord& Record : Records)
	{
		SourceHash = Speed::Analytic::CombineStableIds(SourceHash, Record.SourceId);
		SourceHash = Speed::Analytic::CombineStableIds(SourceHash, Record.TriangleHash);
		SourceHash = Speed::Analytic::CombineStableIds(SourceHash, Record.NormalHash);
		SourceHash = Speed::Analytic::CombineStableIds(SourceHash, Record.MaterialHash);
		SourceHash = Speed::Analytic::CombineStableIds(
			SourceHash, Record.CollisionPolicyHash);
	}
	for (const Speed::Analytic::FBoundedPlane& Plane : LandscapePlanes)
	{
		SourceHash = Speed::Analytic::CombineStableIds(SourceHash, Plane.SourceId);
		SourceHash = Speed::Analytic::CombineStableIds(SourceHash, Plane.PrimitiveId);
		SourceHash = Speed::Analytic::CombineStableIds(SourceHash, Plane.MaterialId);
		SourceHash = Speed::Analytic::CombineStableIds(SourceHash, Plane.ObjectType);
		SourceHash = Speed::Analytic::CombineStableIds(
			SourceHash, Plane.BlockingChannels);
	}

	UPackage* OutputPackage = LoadPackage(nullptr, *OutputPackageName, LOAD_None);
	if (!OutputPackage)
	{
		OutputPackage = CreatePackage(*OutputPackageName);
	}
	const FString AssetName = FPackageName::GetLongPackageAssetName(OutputPackageName);
	USpeedAnalyticCollisionAsset* Asset = FindObject<USpeedAnalyticCollisionAsset>(
		OutputPackage, *AssetName);
	const bool bNewAsset = Asset == nullptr;
	if (!Asset)
	{
		Asset = NewObject<USpeedAnalyticCollisionAsset>(OutputPackage, *AssetName,
			RF_Public | RF_Standalone);
	}
	Asset->BakeSchemaVersion = 6;
	Asset->SchemaVersion = Speed::Analytic::AnalyticWorldSchemaVersion;
	Asset->SourceHash = SourceHash;
	Asset->MeshSources = MoveTemp(Records);
	Asset->Triangles.Reset();
	Asset->VertexPositions = MoveTemp(VertexPositions);
	Asset->VertexNormals = MoveTemp(VertexNormals);
	Asset->IndexedTriangles = MoveTemp(IndexedTriangleRecords);
	Asset->BoundedPlanes.Reset();
	Asset->ExtrudedQuinticPatches.Reset();
	FString ValidationReason;
	if (!Asset->ValidateGeneratedData(&ValidationReason))
	{
		UE_LOG(LogTemp, Error, TEXT("Generated asset is invalid: %s"),
			*ValidationReason);
		return 6;
	}
	const TSharedPtr<const Speed::Analytic::FAnalyticWorldData> RecognitionRuntime =
		Asset->BuildRuntimeData(&ValidationReason, true);
	if (!RecognitionRuntime)
	{
		UE_LOG(LogTemp, Error, TEXT("Could not synthesize compact patches: %s"),
			*ValidationReason);
		return 6;
	}
	for (const Speed::Analytic::FBoundedPlane& SourcePlane :
		RecognitionRuntime->Planes)
	{
		if (!SourcePlane.bRequiresCompactOptIn) continue;
		FSpeedAnalyticBoundedPlaneRecord& Record =
			Asset->BoundedPlanes.AddDefaulted_GetRef();
		Record.SourceId = SourcePlane.SourceId;
		Record.SurfaceId = SourcePlane.SurfaceId;
		Record.FeatureId = SourcePlane.FeatureId;
		Record.PrimitiveId = SourcePlane.PrimitiveId;
		Record.MaterialId = static_cast<int32>(SourcePlane.MaterialId);
		Record.ObjectType = static_cast<int32>(SourcePlane.ObjectType);
		Record.BlockingChannels = SourcePlane.BlockingChannels;
		Record.Origin = FVector(SourcePlane.Origin);
		Record.Normal = FVector(SourcePlane.Normal);
		Record.AxisU = FVector(SourcePlane.AxisU);
		Record.AxisV = FVector(SourcePlane.AxisV);
		Record.HalfExtents = FVector2D(SourcePlane.HalfExtents);
		Record.DomainVertices.Reserve(SourcePlane.DomainVertices.Num());
		for (const FVector2d& Vertex : SourcePlane.DomainVertices)
		{
			Record.DomainVertices.Add(FVector2D(Vertex));
		}
		Record.bQueryCollisionEnabled = SourcePlane.bQueryCollisionEnabled;
		Record.bRequiresCompactOptIn = SourcePlane.bRequiresCompactOptIn;
		Record.bAuthorityEligible = SourcePlane.bAuthorityEligible;
	}
	for (const Speed::Analytic::FBoundedPlane& SourcePlane : LandscapePlanes)
	{
		FSpeedAnalyticBoundedPlaneRecord& Record =
			Asset->BoundedPlanes.AddDefaulted_GetRef();
		Record.SourceId = SourcePlane.SourceId;
		Record.SurfaceId = SourcePlane.SurfaceId;
		Record.FeatureId = SourcePlane.FeatureId;
		Record.PrimitiveId = SourcePlane.PrimitiveId;
		Record.MaterialId = static_cast<int32>(SourcePlane.MaterialId);
		Record.ObjectType = static_cast<int32>(SourcePlane.ObjectType);
		Record.BlockingChannels = SourcePlane.BlockingChannels;
		Record.Origin = FVector(SourcePlane.Origin);
		Record.Normal = FVector(SourcePlane.Normal);
		Record.AxisU = FVector(SourcePlane.AxisU);
		Record.AxisV = FVector(SourcePlane.AxisV);
		Record.HalfExtents = FVector2D(SourcePlane.HalfExtents);
		Record.DomainVertices.Reserve(SourcePlane.DomainVertices.Num());
		for (const FVector2d& Vertex : SourcePlane.DomainVertices)
		{
			Record.DomainVertices.Add(FVector2D(Vertex));
		}
		Record.bQueryCollisionEnabled = SourcePlane.bQueryCollisionEnabled;
		Record.bRequiresCompactOptIn = false;
		Record.bAuthorityEligible = SourcePlane.bAuthorityEligible;
	}
	Asset->ExtrudedQuinticPatches.Reserve(
		RecognitionRuntime->ExtrudedQuinticPatches.Num());
	for (const Speed::Analytic::FExtrudedQuinticPatch& SourcePatch :
		RecognitionRuntime->ExtrudedQuinticPatches)
	{
		FSpeedAnalyticExtrudedQuinticPatchRecord& Record =
			Asset->ExtrudedQuinticPatches.AddDefaulted_GetRef();
		Record.SourceId = SourcePatch.SourceId;
		Record.SurfaceId = SourcePatch.SurfaceId;
		Record.FeatureId = SourcePatch.FeatureId;
		Record.PrimitiveId = SourcePatch.PrimitiveId;
		Record.CanonicalGroupId = SourcePatch.CanonicalGroupId;
		Record.CanonicalSymmetryAxisMask = SourcePatch.CanonicalSymmetryAxisMask;
		Record.MaterialId = SourcePatch.MaterialId;
		Record.ObjectType = SourcePatch.ObjectType;
		Record.BlockingChannels = SourcePatch.BlockingChannels;
		Record.SectionControlPoints.SetNumUninitialized(6);
		for (int32 ControlIndex = 0; ControlIndex < 6; ++ControlIndex)
		{
			Record.SectionControlPoints[ControlIndex] =
				FVector(SourcePatch.SectionControlPoints[ControlIndex]);
		}
		Record.InteriorCorrectionControlPoints.SetNumUninitialized(2);
		for (int32 CorrectionIndex = 0; CorrectionIndex < 2; ++CorrectionIndex)
		{
			Record.InteriorCorrectionControlPoints[CorrectionIndex] =
				FVector(SourcePatch.InteriorCorrectionControlPoints[CorrectionIndex]);
		}
		Record.BaseRootMeanSquareResidualCm =
			SourcePatch.BaseRootMeanSquareResidualCm;
		Record.BaseMaximumResidualCm = SourcePatch.BaseMaximumResidualCm;
		Record.CorrectedRootMeanSquareResidualCm =
			SourcePatch.CorrectedRootMeanSquareResidualCm;
		Record.CorrectedMaximumResidualCm =
			SourcePatch.CorrectedMaximumResidualCm;
		Record.ExtrusionAxis = FVector(SourcePatch.ExtrusionAxis);
		Record.MinimumExtrusionCoordinate =
			SourcePatch.MinimumExtrusionCoordinate;
		Record.MaximumExtrusionCoordinate =
			SourcePatch.MaximumExtrusionCoordinate;
		Record.Bounds = FBox(SourcePatch.Bounds);
		Record.bQueryCollisionEnabled = SourcePatch.bQueryCollisionEnabled;
		Record.bCanonicalC2ByConstruction =
			SourcePatch.bCanonicalC2ByConstruction;
		Record.bCanonicalSymmetryByConstruction =
			SourcePatch.bCanonicalSymmetryByConstruction;
		Record.bAuthorityEligible = SourcePatch.bAuthorityEligible;
		UE_LOG(LogTemp, Display,
			TEXT("[AnalyticCompactPatch] Primitive=%016llX Surface=%016llX Feature=%016llX BaseRmsCm=%.9g BaseMaxCm=%.9g CorrectedRmsCm=%.9g CorrectedMaxCm=%.9g CorrectionA=%s CorrectionB=%s"),
			SourcePatch.PrimitiveId, SourcePatch.SurfaceId, SourcePatch.FeatureId,
			SourcePatch.BaseRootMeanSquareResidualCm,
			SourcePatch.BaseMaximumResidualCm,
			SourcePatch.CorrectedRootMeanSquareResidualCm,
			SourcePatch.CorrectedMaximumResidualCm,
			*FVector(SourcePatch.InteriorCorrectionControlPoints[0]).ToString(),
			*FVector(SourcePatch.InteriorCorrectionControlPoints[1]).ToString());
	}
	if (!Asset->ValidateGeneratedData(&ValidationReason))
	{
		UE_LOG(LogTemp, Error, TEXT("Generated compact asset is invalid: %s"),
			*ValidationReason);
		return 6;
	}
	if (bNewAsset)
	{
		FAssetRegistryModule::AssetCreated(Asset);
	}
	OutputPackage->MarkPackageDirty();
	OutputPackage->GetMetaData();
	const FString Filename = FPackageName::LongPackageNameToFilename(
		OutputPackageName, FPackageName::GetAssetPackageExtension());
	FSavePackageArgs SaveArgs;
	SaveArgs.TopLevelFlags = RF_Public | RF_Standalone;
	SaveArgs.SaveFlags = SAVE_NoError;
	if (!UPackage::SavePackage(OutputPackage, Asset, *Filename, SaveArgs))
	{
		UE_LOG(LogTemp, Error, TEXT("Failed to save %s."), *Filename);
		return 7;
	}
	for (const FSpeedAnalyticMeshSourceRecord& Record : Asset->MeshSources)
	{
		UE_LOG(LogTemp, Display,
			TEXT("[AnalyticBakeSource] Component=%s Mesh=%s Triangles=%d Degenerate=%d TriangleHash=%016llX NormalHash=%016llX MaterialHash=%016llX CollisionHash=%016llX ObjectType=%u Enabled=%u Blocking=%016llX Overlap=%016llX Trace=%u"),
			*Record.ComponentPath, *Record.MeshPath, Record.TriangleCount,
			Record.DegenerateTriangleCount, Record.TriangleHash, Record.NormalHash,
			Record.MaterialHash, Record.CollisionPolicyHash, Record.ObjectType,
			Record.CollisionEnabled,
			Record.BlockingChannels, Record.OverlapChannels,
			Record.CollisionTraceFlag);
	}
	int32 AuthorityEligibleCount = 0;
	for (const FSpeedAnalyticBoundedPlaneRecord& Plane : Asset->BoundedPlanes)
	{
		AuthorityEligibleCount += Plane.bAuthorityEligible ? 1 : 0;
	}
	for (const FSpeedAnalyticExtrudedQuinticPatchRecord& Patch :
		Asset->ExtrudedQuinticPatches)
	{
		AuthorityEligibleCount += Patch.bAuthorityEligible ? 1 : 0;
	}
	UE_LOG(LogTemp, Display,
		TEXT("[AnalyticBake] Asset=%s Sources=%d SourceHash=%016llX RuntimeTriangles=%d Positions=%d Normals=%d RuntimePlanes=%d RuntimeExtrudedQuintics=%d AuthorityEligible=%d"),
		*OutputPackageName, Asset->MeshSources.Num(), Asset->SourceHash,
		Asset->IndexedTriangles.Num(), Asset->VertexPositions.Num(),
		Asset->VertexNormals.Num(),
		Asset->BoundedPlanes.Num(), Asset->ExtrudedQuinticPatches.Num(),
		AuthorityEligibleCount);
	return 0;
}
