// Fill out your copyright notice in the Description page of Project Settings.

// USpeedWorldSubsystem.cpp

#include "SpeedWorldSubsystem.h"
#include "IAmSpeed/Actors/SpeedStaticActor.h"
#include "IAmSpeed/Components/ISpeedComponent.h"
#include "IAmSpeed/World/Simulation/CanonicalFrameContext.h"
#include "IAmSpeed/World/Simulation/SimulationActorDiagnostics.h"
#include "IAmSpeed/World/Collision/ResolvedPairSet.h"
#include "IAmSpeed/World/Analytic/AnalyticLandscapeAdapter.h"
#include "IAmSpeed/World/Analytic/SpeedAnalyticCollisionAsset.h"
#include "IAmSpeed/World/Analytic/SpeedAnalyticSourceComponent.h"
#include "IAmSpeed/World/Analytic/StaticWorldQueryAudit.h"
#include "IAmSpeed/SubBodies/Solid/BoxSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SolidSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SphereSubBody.h"
#include "GameFramework/Actor.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "LandscapeProxy.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "TimerManager.h"
#include "Algo/Sort.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformTime.h"
#include "Misc/PackageName.h"

static TAutoConsoleVariable<int32> CVarIAmSpeedPersistentDynamicPairs(
	TEXT("p.IAmSpeed.Collision.PersistentDynamicPairs"),
	1,
	TEXT("Enables finite-mass persistent constraints for opted-in dynamic body pairs."));

static TAutoConsoleVariable<int32> CVarIAmSpeedUnilateralRollingPairs(
	TEXT("p.IAmSpeed.Collision.UnilateralRollingPairs"),
	0,
	TEXT("Experimental: maintains opted-in sphere/box contacts from current geometry as unilateral rolling manifolds."));

static TAutoConsoleVariable<float> CVarIAmSpeedRollingSeparationTolerance(
	TEXT("p.IAmSpeed.Collision.RollingSeparationToleranceCm"),
	0.25f,
	TEXT("Maximum positive sphere/box gap retained only to preserve experimental rolling-manifold identity."));

static TAutoConsoleVariable<float> CVarIAmSpeedRollingGentleAcquisitionSpeed(
	TEXT("p.IAmSpeed.Collision.RollingGentleAcquisitionSpeedCmS"),
	5.0f,
	TEXT("Maximum pre-impact normal closing speed that acquires the ordinary gentle rolling manifold without changing impact response."));

static TAutoConsoleVariable<float> CVarIAmSpeedRollingPenetrationSlop(
	TEXT("p.IAmSpeed.Collision.RollingPenetrationSlopCm"),
	0.02f,
	TEXT("Penetration ignored by rolling-manifold velocity stabilization."));

static TAutoConsoleVariable<float> CVarIAmSpeedAnalyticLandscapeFlatnessTolerance(
	TEXT("p.IAmSpeed.AnalyticWorld.Landscape.FlatnessToleranceCm"),
	0.001f,
	TEXT("Maximum full-source height residual accepted by the shadow-only flat Landscape adapter."));

namespace
{
	FString AuthoredObjectPath(const UObject& Object)
	{
		return UWorld::RemovePIEPrefix(Object.GetPathName());
	}
}

void USpeedWorldSubsystem::OnWorldBeginPlay(UWorld& InWorld)
{
	Super::OnWorldBeginPlay(InWorld);
	SimulationWorld.ResetStaticCollisionWorld();
	AnalyticWorldData.Reset();
	AnalyticSourceComponents.Reset();
	AnalyticWorldBuildAttempt = 0;
	if (!Speed::Analytic::FStaticWorldQueryAudit::ShouldBuildAnalyticWorld())
	{
		return;
	}
	BuildAnalyticWorldFromLoadedSources();
}

TWeakObjectPtr<UPrimitiveComponent> USpeedWorldSubsystem::FindAnalyticSourceComponent(
	const uint64 SourceId) const
{
	if (const TWeakObjectPtr<UPrimitiveComponent>* Component =
		AnalyticSourceComponents.Find(SourceId))
	{
		return *Component;
	}
	return nullptr;
}

void USpeedWorldSubsystem::BuildAnalyticWorldFromLoadedSources()
{
	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}
	++AnalyticWorldBuildAttempt;

	TArray<ALandscapeProxy*> Landscapes;
	for (TActorIterator<ALandscapeProxy> It(World); It; ++It)
	{
		Landscapes.Add(*It);
	}
	Landscapes.Sort([](const ALandscapeProxy& A, const ALandscapeProxy& B)
	{
		return A.GetPathName() < B.GetPathName();
	});

	TSharedPtr<Speed::Analytic::FAnalyticWorldData> Imported =
		MakeShared<Speed::Analytic::FAnalyticWorldData>();
	TSharedPtr<const Speed::Analytic::FAnalyticWorldData> BakedRuntimeWorld;
	bool bRuntimeWorldMutated = false;
	bool bRuntimeWorldIncrementallyFinalized = false;
	uint64 CombinedSourceHash = 0;
	bool bSourceReadinessPending = false;
	TArray<Speed::Analytic::FBoundedPlane> AdditionalLandscapePlanes;

	TArray<AActor*> AnalyticMeshSourceActors;
	for (TActorIterator<AActor> It(World); It; ++It)
	{
		if (It->FindComponentByClass<USpeedAnalyticSourceComponent>())
		{
			AnalyticMeshSourceActors.Add(*It);
		}
	}
	AnalyticMeshSourceActors.Sort([](const AActor& A, const AActor& B)
	{
		return A.GetPathName() < B.GetPathName();
	});
	uint64 MeshSourceInventoryHash = 0;
	int32 MeshSourceComponentCount = 0;
	TArray<UStaticMeshComponent*> AnalyticMeshComponents;
	const USpeedAnalyticCollisionAsset* ExplicitBakedAsset = nullptr;
	FString ExplicitBakedAssetPath;
	for (const AActor* SourceActor : AnalyticMeshSourceActors)
	{
		TArray<UStaticMeshComponent*> MeshComponents;
		const USpeedAnalyticSourceComponent* SourceComponent =
			SourceActor->FindComponentByClass<USpeedAnalyticSourceComponent>();
		if (const ASpeedStaticActor* SpeedStaticActor =
			Cast<ASpeedStaticActor>(SourceActor))
		{
			SpeedStaticActor->GatherAnalyticStaticMeshes(MeshComponents);
		}
		else if (SourceComponent)
		{
			SourceComponent->GatherOwnedStaticMeshes(MeshComponents);
		}
		if (SourceComponent && !SourceComponent->AnalyticCollisionAsset.IsNull())
		{
			const FString AssetPath =
				SourceComponent->AnalyticCollisionAsset.ToSoftObjectPath().ToString();
			const USpeedAnalyticCollisionAsset* CandidateAsset =
				SourceComponent->AnalyticCollisionAsset.LoadSynchronous();
			if (!CandidateAsset)
			{
				UE_LOG(LogTemp, Error,
					TEXT("[AnalyticExplicitBake] Asset=%s Result=Missing Owner=%s"),
					*AssetPath, *SourceActor->GetPathName());
				return;
			}
			if (ExplicitBakedAsset && ExplicitBakedAsset != CandidateAsset)
			{
				UE_LOG(LogTemp, Error,
					TEXT("[AnalyticExplicitBake] Result=ConflictingAssets First=%s Second=%s"),
					*ExplicitBakedAssetPath, *AssetPath);
				return;
			}
			ExplicitBakedAsset = CandidateAsset;
			ExplicitBakedAssetPath = AssetPath;
		}
		for (const UStaticMeshComponent* MeshComponent : MeshComponents)
		{
			const UStaticMesh* Mesh = MeshComponent
				? MeshComponent->GetStaticMesh() : nullptr;
			if (!MeshComponent || !Mesh)
			{
				continue;
			}
			++MeshSourceComponentCount;
			AnalyticMeshComponents.Add(
				const_cast<UStaticMeshComponent*>(MeshComponent));
			const FString ActorPath = AuthoredObjectPath(*SourceActor);
			const FString ComponentPath = AuthoredObjectPath(*MeshComponent);
			AnalyticSourceComponents.Add(
				Speed::Analytic::StableStringId(ComponentPath),
				const_cast<UStaticMeshComponent*>(MeshComponent));
			const FString StableIdentity = FString::Printf(
				TEXT("%s|%s|%s|%s"), *ActorPath,
				*ComponentPath, *Mesh->GetPathName(),
				*MeshComponent->GetComponentTransform().ToString());
			MeshSourceInventoryHash = Speed::Analytic::CombineStableIds(
				MeshSourceInventoryHash,
				Speed::Analytic::StableStringId(StableIdentity));
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticMeshSource] Owner=%s Component=%s Mesh=%s"),
				*SourceActor->GetPathName(), *MeshComponent->GetName(),
				*Mesh->GetPathName());
		}
	}

	bool bMeshBakeLoaded = false;
	// PIE duplicates a world under UEDPIE_<instance>_<map>. Generated collision
	// certificates belong to the authored map and must keep the same stable ids
	// in editor play, standalone and packaged execution.
	const FString WorldName = UWorld::RemovePIEPrefix(
		FPackageName::GetShortName(World->GetOutermost()));
	const FString BakedPackagePath = FString::Printf(
		TEXT("/Game/Generated/Analytic/%s_AnalyticWorld"), *WorldName);
	const FString BakedObjectPath = FString::Printf(
		TEXT("%s.%s_AnalyticWorld"), *BakedPackagePath, *WorldName);
	const bool bUsesExplicitBake = ExplicitBakedAsset != nullptr;
	const USpeedAnalyticCollisionAsset* BakedAsset = ExplicitBakedAsset;
	if (!BakedAsset)
	{
		BakedAsset = LoadObject<USpeedAnalyticCollisionAsset>(
			nullptr, *BakedObjectPath);
	}
	if (BakedAsset)
	{
		int32 SerializedPlaneAuthorityCount = 0;
		for (const FSpeedAnalyticBoundedPlaneRecord& Plane :
			BakedAsset->BoundedPlanes)
		{
			SerializedPlaneAuthorityCount += Plane.bAuthorityEligible ? 1 : 0;
		}
		int32 SerializedQuinticAuthorityCount = 0;
		for (const FSpeedAnalyticExtrudedQuinticPatchRecord& Patch :
			BakedAsset->ExtrudedQuinticPatches)
		{
			SerializedQuinticAuthorityCount += Patch.bAuthorityEligible ? 1 : 0;
		}
		int32 SerializedTensorAuthorityCount = 0;
		for (const FSpeedAnalyticTensorBezierPatchRecord& Patch :
			BakedAsset->TensorBezierPatches)
		{
			SerializedTensorAuthorityCount += Patch.bAuthorityEligible ? 1 : 0;
		}
		int32 SerializedPiecewiseAuthorityCount = 0;
		for (const FSpeedAnalyticPiecewiseTensorBezierPatchRecord& Patch :
			BakedAsset->PiecewiseTensorBezierPatches)
		{
			SerializedPiecewiseAuthorityCount += Patch.bAuthorityEligible ? 1 : 0;
		}
		uint64 BakedInventoryHash = 0;
		for (const FSpeedAnalyticMeshSourceRecord& Source : BakedAsset->MeshSources)
		{
			const FString StableIdentity = FString::Printf(
				TEXT("%s|%s|%s|%s"), *Source.ActorPath, *Source.ComponentPath,
				*Source.MeshPath, *Source.WorldTransform.ToString());
			BakedInventoryHash = Speed::Analytic::CombineStableIds(
				BakedInventoryHash,
				Speed::Analytic::StableStringId(StableIdentity));
		}
		FString BakedReason;
		const bool bAuthorityProvidersOnly =
			Speed::Analytic::FStaticWorldQueryAudit::IsSurfaceAnalyticBackend();
		const double RuntimeBuildStartSeconds = FPlatformTime::Seconds();
		const TSharedPtr<const Speed::Analytic::FAnalyticWorldData> BakedRuntime =
			BakedAsset->BuildRuntimeData(&BakedReason, false,
				bAuthorityProvidersOnly);
		UE_LOG(LogTemp, Display,
			TEXT("[AnalyticRuntimeLoadTiming] AuthorityOnly=%d Seconds=%.6f Success=%d"),
			bAuthorityProvidersOnly ? 1 : 0,
			FPlatformTime::Seconds() - RuntimeBuildStartSeconds,
			BakedRuntime ? 1 : 0);
		int32 RuntimePlaneAuthorityCount = 0;
		int32 RuntimeQuinticAuthorityCount = 0;
		int32 RuntimeTensorAuthorityCount = 0;
		int32 RuntimePiecewiseAuthorityCount = 0;
		int64 RuntimeTensorApproximationCellCount = 0;
		int64 RuntimePiecewiseCellCount = 0;
		int64 RuntimePiecewiseApproximationCellCount = 0;
		int64 RuntimePiecewiseApproximationBvhNodeCount = 0;
		if (BakedRuntime)
		{
			for (const Speed::Analytic::FBoundedPlane& Plane : BakedRuntime->Planes)
			{
				RuntimePlaneAuthorityCount += Plane.bAuthorityEligible ? 1 : 0;
			}
			for (const Speed::Analytic::FExtrudedQuinticPatch& Patch :
				BakedRuntime->ExtrudedQuinticPatches)
			{
				RuntimeQuinticAuthorityCount += Patch.bAuthorityEligible ? 1 : 0;
			}
			for (const Speed::Analytic::FTensorBezierPatch& Patch :
				BakedRuntime->TensorBezierPatches)
			{
				RuntimeTensorAuthorityCount += Patch.bAuthorityEligible ? 1 : 0;
				RuntimeTensorApproximationCellCount += Patch.ApproximationCells.Num();
			}
			for (const Speed::Analytic::FPiecewiseTensorBezierPatch& Patch :
				BakedRuntime->PiecewiseTensorBezierPatches)
			{
				RuntimePiecewiseAuthorityCount += Patch.bAuthorityEligible ? 1 : 0;
				RuntimePiecewiseCellCount += Patch.Cells.Num();
				for (const Speed::Analytic::FPiecewiseTensorBezierCell& Cell :
					Patch.Cells)
				{
					RuntimePiecewiseApproximationCellCount +=
						Cell.ApproximationCells.Num();
					RuntimePiecewiseApproximationBvhNodeCount +=
						Cell.ApproximationCellBvhNodes.Num();
				}
			}
		}
		const int32 RuntimePlaneCount = BakedRuntime ? BakedRuntime->Planes.Num() : 0;
		const int32 RuntimeQuinticCount = BakedRuntime ?
			BakedRuntime->ExtrudedQuinticPatches.Num() : 0;
		const int32 RuntimeTensorCount = BakedRuntime ?
			BakedRuntime->TensorBezierPatches.Num() : 0;
		const int32 RuntimePiecewiseCount = BakedRuntime ?
			BakedRuntime->PiecewiseTensorBezierPatches.Num() : 0;
		UE_LOG(LogTemp, Display,
			TEXT("[AnalyticMeshBakeAuthority] BakeSchema=%u SerializedPlanes=%d/%d SerializedQuintics=%d/%d SerializedTensors=%d/%d SerializedPiecewise=%d/%d RuntimePlanes=%d/%d RuntimeQuintics=%d/%d RuntimeTensors=%d/%d RuntimePiecewise=%d/%d"),
			BakedAsset->BakeSchemaVersion,
			SerializedPlaneAuthorityCount, BakedAsset->BoundedPlanes.Num(),
			SerializedQuinticAuthorityCount,
			BakedAsset->ExtrudedQuinticPatches.Num(),
			SerializedTensorAuthorityCount, BakedAsset->TensorBezierPatches.Num(),
			SerializedPiecewiseAuthorityCount,
			BakedAsset->PiecewiseTensorBezierPatches.Num(),
			RuntimePlaneAuthorityCount, RuntimePlaneCount,
			RuntimeQuinticAuthorityCount, RuntimeQuinticCount,
			RuntimeTensorAuthorityCount, RuntimeTensorCount,
			RuntimePiecewiseAuthorityCount, RuntimePiecewiseCount);
		UE_LOG(LogTemp, Display,
			TEXT("[AnalyticMeshBakeAcceleration] TensorApproximationCells=%lld PiecewiseCells=%lld PiecewiseApproximationCells=%lld PiecewiseApproximationBvhNodes=%lld"),
			RuntimeTensorApproximationCellCount, RuntimePiecewiseCellCount,
			RuntimePiecewiseApproximationCellCount,
			RuntimePiecewiseApproximationBvhNodeCount);
		bool bInventoryMatches =
			BakedInventoryHash == MeshSourceInventoryHash &&
			BakedAsset->MeshSources.Num() == MeshSourceComponentCount;
		if (bUsesExplicitBake)
		{
			bInventoryMatches =
				BakedAsset->MeshSources.Num() == MeshSourceComponentCount;
			TSet<const UStaticMeshComponent*> ClaimedComponents;
			for (const FSpeedAnalyticMeshSourceRecord& Source :
				BakedAsset->MeshSources)
			{
				const UStaticMeshComponent* MatchedComponent = nullptr;
				for (const UStaticMeshComponent* Candidate : AnalyticMeshComponents)
				{
					if (!Candidate || ClaimedComponents.Contains(Candidate) ||
						!Candidate->GetStaticMesh() ||
						Candidate->GetStaticMesh()->GetPathName() != Source.MeshPath ||
						!Candidate->GetComponentTransform().Equals(
							Source.WorldTransform, 0.01f))
					{
						continue;
					}
					MatchedComponent = Candidate;
					break;
				}
				if (!MatchedComponent)
				{
					bInventoryMatches = false;
					break;
				}
				ClaimedComponents.Add(MatchedComponent);
				AnalyticSourceComponents.Add(
					Source.SourceId,
					const_cast<UStaticMeshComponent*>(MatchedComponent));
			}
		}
		if (BakedRuntime && bInventoryMatches)
		{
			BakedRuntimeWorld = BakedRuntime;
			CombinedSourceHash = Speed::Analytic::CombineStableIds(
				CombinedSourceHash, BakedRuntime->SourceHash);
			bMeshBakeLoaded = true;
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticExplicitBake] Asset=%s Result=%s"),
				bUsesExplicitBake ? *ExplicitBakedAssetPath : *BakedObjectPath,
				bUsesExplicitBake ? TEXT("Accepted") : TEXT("MapFallbackAccepted"));
		}
		else
		{
			UE_LOG(LogTemp, Warning,
				TEXT("[AnalyticMeshBake] Asset=%s Result=Rejected LiveHash=%016llX BakedHash=%016llX Detail=%s"),
				bUsesExplicitBake ? *ExplicitBakedAssetPath : *BakedObjectPath,
				MeshSourceInventoryHash, BakedInventoryHash,
				BakedRuntime ? TEXT("Source inventory mismatch.") : *BakedReason);
		}
	}
	UE_LOG(LogTemp, Display,
		TEXT("[AnalyticMeshSourceInventory] Actors=%d Components=%d Hash=%016llX Baked=%d Triangles=%d"),
		AnalyticMeshSourceActors.Num(), MeshSourceComponentCount,
		MeshSourceInventoryHash, bMeshBakeLoaded ? 1 : 0,
		BakedRuntimeWorld ? BakedRuntimeWorld->Triangles.Num() : Imported->Triangles.Num());
	if (MeshSourceComponentCount > 0 && !bMeshBakeLoaded)
	{
		UE_LOG(LogTemp, Error,
			TEXT("[AnalyticAuthorityCoverageMissing] World=%s Components=%d Hash=%016llX Detail=The strict analytical world will remain unavailable until the authored source inventory has a matching immutable bake."),
			*WorldName, MeshSourceComponentCount, MeshSourceInventoryHash);
		return;
	}

	for (const ALandscapeProxy* Landscape : Landscapes)
	{
		if (!Landscape)
		{
			continue;
		}
		if (!Landscape->CollisionComponents.IsEmpty() &&
			Landscape->CollisionComponents[0])
		{
			const FString LandscapePath = AuthoredObjectPath(*Landscape);
			AnalyticSourceComponents.Add(
				Speed::Analytic::StableStringId(LandscapePath),
				Landscape->CollisionComponents[0]);
		}
		const FString LandscapePath = AuthoredObjectPath(*Landscape);
		const uint64 LandscapeSourceId =
			Speed::Analytic::StableStringId(LandscapePath);
		const Speed::Analytic::FAnalyticWorldData& CandidateWorld =
			BakedRuntimeWorld && !bRuntimeWorldMutated
				? *BakedRuntimeWorld : *Imported;
		if (CandidateWorld.Planes.ContainsByPredicate(
			[LandscapeSourceId](const Speed::Analytic::FBoundedPlane& Plane)
			{
				return Plane.SourceId == LandscapeSourceId &&
					!Plane.bRequiresCompactOptIn;
			}))
		{
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticLandscapeAdapter] Source=%s Result=BakedAuthority Detail=Immutable baked Landscape certificate loaded."),
				*Landscape->GetPathName());
			continue;
		}
		const Speed::Analytic::FFlatLandscapeAdapterOutput Output =
			Speed::Analytic::BuildFlatLandscapePlane(
				*Landscape,
				CVarIAmSpeedAnalyticLandscapeFlatnessTolerance.GetValueOnGameThread());
		UE_LOG(LogTemp, Display,
			TEXT("[AnalyticLandscapeAdapter] Source=%s Result=%u ResidualCm=%.9g Detail=%s"),
			*Landscape->GetPathName(), static_cast<uint8>(Output.Result),
			Output.MaximumHeightResidual, *Output.Diagnostic);
		if (Output.Result ==
			Speed::Analytic::EFlatLandscapeAdapterResult::SuccessShadowOnly ||
			Output.Result == Speed::Analytic::EFlatLandscapeAdapterResult::
				SuccessAuthorityEligible)
		{
			AdditionalLandscapePlanes.Add(Output.Plane);
			CombinedSourceHash = Speed::Analytic::CombineStableIds(
				CombinedSourceHash, Output.Plane.SurfaceId);
		}
		else if (Output.Result ==
			Speed::Analytic::EFlatLandscapeAdapterResult::NoSourceData)
		{
			bSourceReadinessPending = true;
		}
	}
	constexpr uint8 MaximumBuildAttempts = 8;
	if (bSourceReadinessPending && AnalyticWorldBuildAttempt < MaximumBuildAttempts)
	{
		World->GetTimerManager().SetTimerForNextTick(
			FTimerDelegate::CreateUObject(
				this, &USpeedWorldSubsystem::BuildAnalyticWorldFromLoadedSources));
		return;
	}
	if (!AdditionalLandscapePlanes.IsEmpty())
	{
		if (BakedRuntimeWorld)
		{
			Imported = MakeShared<Speed::Analytic::FAnalyticWorldData>(
				*BakedRuntimeWorld);
			Imported->SourceHash = CombinedSourceHash;
			FString ExtensionReason;
			if (!Imported->AppendFinalizedNonCompactPlanes(
				MoveTemp(AdditionalLandscapePlanes), &ExtensionReason))
			{
				UE_LOG(LogTemp, Warning,
					TEXT("[AnalyticWorldBuild] IncrementalPlaneExtensionRejected Detail=%s"),
					*ExtensionReason);
				return;
			}
			bRuntimeWorldMutated = true;
			bRuntimeWorldIncrementallyFinalized = true;
		}
		else
		{
			Imported->Planes.Append(MoveTemp(AdditionalLandscapePlanes));
		}
	}
	TSharedPtr<const Speed::Analytic::FAnalyticWorldData> FinalizedWorld;
	if (BakedRuntimeWorld && !bRuntimeWorldMutated)
	{
		FinalizedWorld = BakedRuntimeWorld;
	}
	else if (bRuntimeWorldIncrementallyFinalized)
	{
		FinalizedWorld = Imported;
	}
	else
	{
		Imported->SourceHash = CombinedSourceHash;
		FString ValidationReason;
		if (!Imported->FinalizeAndValidate(&ValidationReason))
		{
			UE_LOG(LogTemp, Warning,
				TEXT("[AnalyticWorldBuild] Rejected Detail=%s"), *ValidationReason);
			return;
		}
		FinalizedWorld = Imported;
	}
	const Speed::Analytic::FAnalyticWorldData& Finalized = *FinalizedWorld;
	int32 BoundaryEdgeCount = 0;
	int32 NonManifoldEdgeCount = 0;
	int32 SmoothEdgeCount = 0;
	int32 CreaseEdgeCount = 0;
	int32 PlanarCandidateCount = 0;
	int32 ValidShapeSampleCount = 0;
	for (const Speed::Analytic::FTriangleMeshEdge& Edge : Finalized.MeshEdges)
	{
		BoundaryEdgeCount += Edge.IsBoundary() ? 1 : 0;
		NonManifoldEdgeCount += Edge.IsNonManifold() ? 1 : 0;
		SmoothEdgeCount += Edge.Continuity ==
			Speed::Analytic::EEdgeContinuity::Smooth ? 1 : 0;
		CreaseEdgeCount += Edge.Continuity ==
			Speed::Analytic::EEdgeContinuity::Crease ? 1 : 0;
	}
	for (const Speed::Analytic::FSurfacePatch& Patch : Finalized.SurfacePatches)
	{
		PlanarCandidateCount += Patch.Kind ==
			Speed::Analytic::ESurfacePatchKind::PlanarCandidate ? 1 : 0;
	}
	for (const Speed::Analytic::FVertexShapeSample& Sample : Finalized.VertexShapeSamples)
	{
		ValidShapeSampleCount += Sample.bValid ? 1 : 0;
	}
	UE_LOG(LogTemp, Display,
		TEXT("[AnalyticWorldBuild] Attempt=%u Planes=%d ExtrudedQuintics=%d Triangles=%d Vertices=%d Edges=%d BoundaryEdges=%d NonManifoldEdges=%d SmoothEdges=%d CreaseEdges=%d SmoothRegions=%d Patches=%d PlanarCandidates=%d ShapeValid=%d BvhNodes=%d Hash=%016llX AuthorityEligible=%d PendingSource=%d"),
		AnalyticWorldBuildAttempt, Finalized.Planes.Num(),
		Finalized.ExtrudedQuinticPatches.Num(), Finalized.Triangles.Num(),
		Finalized.MeshVertices.Num(), Finalized.MeshEdges.Num(), BoundaryEdgeCount,
		NonManifoldEdgeCount, SmoothEdgeCount, CreaseEdgeCount,
		Finalized.SmoothSurfaceRegions.Num(), Finalized.SurfacePatches.Num(), PlanarCandidateCount,
		ValidShapeSampleCount, Finalized.TriangleBvh.Num(), Finalized.StableHash(),
		Finalized.IsAuthorityEligible() ? 1 : 0,
		bSourceReadinessPending ? 1 : 0);
#if !UE_BUILD_SHIPPING
	uint64 ExtrudedFacetCount = 0;
	uint64 ExtrudedFacetBoundsAllocatedBytes = 0;
	for (const Speed::Analytic::FExtrudedQuinticPatch& Patch : Finalized.ExtrudedQuinticPatches)
	{
		ExtrudedFacetCount += Patch.SectionSegmentBounds.Num();
		ExtrudedFacetBoundsAllocatedBytes += Patch.SectionSegmentBounds.GetAllocatedSize();
	}
	UE_LOG(LogTemp, Display,
		TEXT("[AnalyticQueryStorage] ExtrudedFacets=%llu FacetBoundsAllocatedBytes=%llu"),
		ExtrudedFacetCount, ExtrudedFacetBoundsAllocatedBytes);
#endif
	AnalyticWorldData = MoveTemp(FinalizedWorld);
	SimulationWorld.SetStaticCollisionWorld(
		MakeUnique<Speed::FAnalyticStaticCollisionWorld>(*AnalyticWorldData));
}

static TAutoConsoleVariable<float> CVarIAmSpeedRollingBaumgarte(
	TEXT("p.IAmSpeed.Collision.RollingBaumgarte"),
	0.20f,
	TEXT("Fraction of rolling-manifold penetration corrected per simulation step."));

static TAutoConsoleVariable<float> CVarIAmSpeedRollingMaxCorrectionSpeed(
	TEXT("p.IAmSpeed.Collision.RollingMaxCorrectionSpeed"),
	50.0f,
	TEXT("Maximum separating speed requested by rolling-manifold penetration correction, in cm/s."));

static TAutoConsoleVariable<float> CVarIAmSpeedRollingReleaseSpeed(
	TEXT("p.IAmSpeed.Collision.RollingReleaseSpeedCmS"),
	3.0f,
	TEXT("Separating normal speed above which a zero-gap rolling pair releases."));

static TAutoConsoleVariable<float> CVarIAmSpeedRollingTangentialFakePhysicsScale(
	TEXT("p.IAmSpeed.Collision.RollingTangentialFakePhysicsScale"),
	0.0f,
	TEXT("Amplitude of the fake-physics share projected into an active manifold's tangent plane. It never changes application frequency."));

static TAutoConsoleVariable<int32> CVarIAmSpeedDebugPersistentDynamicPairs(
	TEXT("p.IAmSpeed.Collision.DebugPersistentDynamicPairs"),
	0,
	TEXT("Logs registration, rejection and impulses for persistent dynamic pairs."));

bool USpeedWorldSubsystem::AreUnilateralRollingPairsEnabled()
{
	return CVarIAmSpeedUnilateralRollingPairs.GetValueOnAnyThread() != 0;
}

bool USpeedWorldSubsystem::ShouldAcquireUnilateralRollingPair(
	const float RelativeNormalSpeed)
{
	return AreUnilateralRollingPairsEnabled() &&
		RelativeNormalSpeed <= FMath::Max(
			0.0f, CVarIAmSpeedRollingGentleAcquisitionSpeed.GetValueOnAnyThread());
}

void USpeedWorldSubsystem::RegisterSpeedComponent(ISpeedComponent* Comp)
{
    if (!Comp) return;
    FScopeLock Lock(&PendingCS);
    PendingOps.Add({ true, Comp });
}

void USpeedWorldSubsystem::UnregisterSpeedComponent(ISpeedComponent* Comp)
{
    if (!Comp) return;
    FScopeLock Lock(&PendingCS);
    PendingOps.Add({ false, Comp });
}

void USpeedWorldSubsystem::ApplyPendingOps()
{
    TArray<FPendingOp> Local;
    {
        FScopeLock Lock(&PendingCS);
        Local = MoveTemp(PendingOps);
        PendingOps.Reset();
    }

    if (Local.Num() == 0) return;

    for (const FPendingOp& Op : Local)
    {
        if (!Op.Comp) continue;

        if (Op.bAdd)
        {
			if (!SimulationWorld.ContainsAdapter(*Op.Comp)) AddComponent(*Op.Comp);
        }
        else
        {
			RemoveComponent(*Op.Comp);
        }
    }
}

void USpeedWorldSubsystem::RebuildSortedIfNeeded()
{
	SimulationWorld.RebuildOrderedAdapters();
}

void USpeedWorldSubsystem::AddComponent(ISpeedComponent& Comp)
{
    // Add New component to all existing ones
    for (ISpeedComponent* Component : SimulationWorld.GetAdapters())
    {
		Component->AddExternalSubBodies(Comp.GetSubBodies());
    }

	// Add existing ones to the new component
    for (ISpeedComponent* Component : SimulationWorld.GetAdapters())
	{
        Comp.AddExternalSubBodies(Component->GetSubBodies());
	}

	// Finally add to list
    SimulationWorld.AddAdapter(Comp);
}

void USpeedWorldSubsystem::RemoveComponent(ISpeedComponent& Comp)
{
	// Remove component from list
	SimulationWorld.RemoveAdapter(Comp);

	// Remove sub-bodies of the removed component from all existing ones
	for (ISpeedComponent* Component : SimulationWorld.GetAdapters())
    {
        Component->RemoveExternalSubBodies(Comp.GetSubBodies());
	}
}

void USpeedWorldSubsystem::RegisterDynamicContactPair(
	USolidSubBody& BodyA,
	USolidSubBody& BodyB,
	const FVector& ContactPoint,
	const FVector& NormalBToA,
	const float ImpactRelativeNormalSpeed)
{
	unsigned int& CurrentStepFrame = SimulationWorld.GetCurrentStepFrame();
	if (CVarIAmSpeedPersistentDynamicPairs.GetValueOnAnyThread() == 0)
	{
		return;
	}

	ISpeedComponent* CompA = BodyA.GetParentComponent();
	ISpeedComponent* CompB = BodyB.GetParentComponent();
	if (!CompA || !CompB || CompA == CompB ||
		!BodyA.UsesPersistentBilateralContact() ||
		!BodyB.UsesPersistentBilateralContact())
	{
		return;
	}

	const uint32 IdA = BodyA.GetUniqueID();
	const uint32 IdB = BodyB.GetUniqueID();
	const uint32 Lo = FMath::Min(IdA, IdB);
	const uint32 Hi = FMath::Max(IdA, IdB);
	const uint64 PairKey = (static_cast<uint64>(Lo) << 32) | Hi;
	FDynamicContactPair* Pair = SimulationWorld.FindDynamicContactPair(PairKey);
	bool bPendingPair = false;
	if (!Pair && AreUnilateralRollingPairsEnabled())
	{
		// Collision resolution runs inside the TOI iteration. Queue structural
		// changes so the active pair array remains immutable until the iteration
		// has completed.
		bPendingPair = true;
		Pair = SimulationWorld.FindPendingRollingContactPair(PairKey);
	}
	if (!Pair)
	{
		Pair = bPendingPair
			? &SimulationWorld.AddPendingRollingContactPair(PairKey, BodyA, BodyB)
			: &SimulationWorld.AddDynamicContactPair(PairKey, BodyA, BodyB);
		Pair->FirstSeenFrame = CurrentStepFrame;
		Pair->AcquisitionNormalSpeed = ImpactRelativeNormalSpeed;
	}

	Pair->LastSeenFrame = CurrentStepFrame;
	Pair->LastCOMA = CompA->GetPhysCOM();
	Pair->LastCOMB = CompB->GetPhysCOM();
	Pair->LocalAnchorA = CompA->GetPhysRotation().UnrotateVector(ContactPoint - CompA->GetPhysCOM());
	Pair->LocalAnchorB = CompB->GetPhysRotation().UnrotateVector(ContactPoint - CompB->GetPhysCOM());
	Pair->LocalNormalB = CompB->GetPhysRotation().UnrotateVector(NormalBToA.GetSafeNormal());
	if (CVarIAmSpeedDebugPersistentDynamicPairs.GetValueOnAnyThread() != 0)
	{
		UE_LOG(LogTemp, Warning,
			TEXT("[PersistentPair][Register] Frame=%u A=%s B=%s Normal=%s Point=%s AcquisitionNormalSpeed=%.6f"),
			CurrentStepFrame, *BodyA.GetName(), *BodyB.GetName(),
			*NormalBToA.ToString(), *ContactPoint.ToString(),
			ImpactRelativeNormalSpeed);
	}
}

void USpeedWorldSubsystem::ActivatePendingRollingContactPairAtTOI(
	const USolidSubBody& BodyA,
	const USolidSubBody& BodyB,
	const float RemainingDt)
{
	if (!AreUnilateralRollingPairsEnabled())
	{
		return;
	}

	const uint32 IdA = BodyA.GetUniqueID();
	const uint32 IdB = BodyB.GetUniqueID();
	const uint32 Lo = FMath::Min(IdA, IdB);
	const uint32 Hi = FMath::Max(IdA, IdB);
	const uint64 PairKey = (static_cast<uint64>(Lo) << 32) | Hi;
	if (!SimulationWorld.ActivatePendingRollingContactPair(PairKey))
	{
		return;
	}

	if (RemainingDt > KINDA_SMALL_NUMBER)
	{
		SolveDynamicContactPairs(RemainingDt, PairKey, true, true);
	}
}

bool USpeedWorldSubsystem::IsDynamicContactPairOwnedByRollingManifold(
	const USolidSubBody& BodyA,
	const USolidSubBody& BodyB) const
{
	const auto& DynamicContactPairs = SimulationWorld.GetDynamicContactPairs();
	const unsigned int CurrentStepFrame = SimulationWorld.GetCurrentStepFrame();
	if (!bLoggedRollingOwnershipState && AreUnilateralRollingPairsEnabled())
	{
		bLoggedRollingOwnershipState = true;
		UE_LOG(LogTemp, Warning,
			TEXT("[PersistentPair][OwnershipQueryState] Frame=%u PairCount=%d A=%s B=%s"),
			CurrentStepFrame, DynamicContactPairs.Num(),
			*BodyA.GetName(), *BodyB.GetName());
	}
	if (DynamicContactPairs.IsEmpty() ||
		!AreUnilateralRollingPairsEnabled())
	{
		return false;
	}

	const uint32 IdA = BodyA.GetUniqueID();
	const uint32 IdB = BodyB.GetUniqueID();
	const uint32 Lo = FMath::Min(IdA, IdB);
	const uint32 Hi = FMath::Max(IdA, IdB);
	const uint64 PairKey = (static_cast<uint64>(Lo) << 32) | Hi;
	const FDynamicContactPair* Pair = SimulationWorld.FindDynamicContactPair(PairKey);
	// Once an acquired pair has been solved at its TOI, it owns the remaining
	// substep and later sweeps must yield to the manifold.
	return Pair && Pair->bRollingManifoldReady &&
		Pair->BodyA.IsValid() && Pair->BodyB.IsValid();
}

ERollingManifoldContactState USpeedWorldSubsystem::GetRollingManifoldContactState(
	const USolidSubBody& Body) const
{
	if (!AreUnilateralRollingPairsEnabled())
	{
		return ERollingManifoldContactState::Absent;
	}

	const TSet<uint64>* PairKeys = SimulationWorld.FindDynamicContactPairKeys(Body.GetUniqueID());
	if (!PairKeys)
	{
		return ERollingManifoldContactState::Absent;
	}

	ERollingManifoldContactState Result = ERollingManifoldContactState::Absent;
	for (const uint64 PairKey : *PairKeys)
	{
		const FDynamicContactPair* Pair = SimulationWorld.FindDynamicContactPair(PairKey);
		if (!Pair || !Pair->bRollingManifoldReady ||
			!Pair->BodyA.IsValid() || !Pair->BodyB.IsValid() ||
			(Pair->BodyA.Get() != &Body && Pair->BodyB.Get() != &Body))
		{
			continue;
		}

		if (Pair->bActiveContactLastSolve)
		{
			return ERollingManifoldContactState::ActiveContact;
		}
		Result = ERollingManifoldContactState::IdentityOnly;
	}
	return Result;
}

void USpeedWorldSubsystem::SolveDynamicContactPairs(
	const float Dt,
	const uint64 PairKeyFilter,
	const bool bFilterByPairKey,
	const bool bSolveFirstSeenFrame)
{
	auto& DynamicContactPairs = SimulationWorld.GetDynamicContactPairs();
	unsigned int& CurrentStepFrame = SimulationWorld.GetCurrentStepFrame();
	constexpr unsigned int MaxUnseenFrames = 2;
	const float SeparationToleranceCm = FMath::Max(
		0.0f, CVarIAmSpeedRollingSeparationTolerance.GetValueOnAnyThread());
	const auto LogRollingRelease = [&CurrentStepFrame](
		const FDynamicContactPair& Pair,
		const TCHAR* Reason,
		const float Separation = 0.0f,
		const float RelativeNormalVelocity = 0.0f)
	{
		if (!Pair.bRollingManifoldReady ||
			CVarIAmSpeedDebugPersistentDynamicPairs.GetValueOnAnyThread() == 0)
		{
			return;
		}

		UE_LOG(LogTemp, Warning,
			TEXT("[PersistentPair][RollingSummary] Frame=%u PairKey=%llu Reason=%s AcquisitionNormalSpeed=%.6f LifetimeFrames=%u IdentityFrames=%u ActiveContactFrames=%u ReactionFrames=%u FeatureTransitions=%u NormalImpulse=%.6f NormalActivity=%.6f NormalSupportActivity=%.6f PeakNormalActivity=%.6f PeakNormalSupportActivity=%.6f MinSeparation=%.6f MaxSeparation=%.6f Separation=%.6f RelN=%.6f"),
			CurrentStepFrame,
			Pair.PairKey,
			Reason,
			Pair.AcquisitionNormalSpeed,
			CurrentStepFrame >= Pair.FirstSeenFrame
				? CurrentStepFrame - Pair.FirstSeenFrame + 1u
				: 0u,
			Pair.SupportedFrameCount,
			Pair.ActiveContactFrameCount,
			Pair.ReactionFrameCount,
			Pair.FeatureTransitionCount,
			Pair.AccumulatedNormalImpulse,
			Pair.AccumulatedNormalActivity,
			Pair.AccumulatedNormalSupportActivity,
			Pair.PeakNormalActivity,
			Pair.PeakNormalSupportActivity,
			Pair.MinimumSeparation == TNumericLimits<float>::Max()
				? 0.0f : Pair.MinimumSeparation,
			Pair.MaximumSeparation == TNumericLimits<float>::Lowest()
				? 0.0f : Pair.MaximumSeparation,
			Separation,
			RelativeNormalVelocity);
	};

	int32 FirstPairIndex = DynamicContactPairs.Num() - 1;
	int32 LastPairIndex = 0;
	if (bFilterByPairKey)
	{
		FirstPairIndex = SimulationWorld.FindDynamicContactPairIndex(PairKeyFilter);
		if (FirstPairIndex == INDEX_NONE)
		{
			return;
		}
		LastPairIndex = FirstPairIndex;
	}
	for (int32 Index = FirstPairIndex; Index >= LastPairIndex; --Index)
	{
		FDynamicContactPair& Pair = DynamicContactPairs[Index];
		if ((bFilterByPairKey && Pair.PairKey != PairKeyFilter) ||
			Pair.LastSolvedFrame == CurrentStepFrame)
		{
			continue;
		}
		Pair.bActiveContactLastSolve = false;
		USolidSubBody* BodyA = Pair.BodyA.Get();
		USolidSubBody* BodyB = Pair.BodyB.Get();
		ISpeedComponent* CompA = BodyA ? BodyA->GetParentComponent() : nullptr;
		ISpeedComponent* CompB = BodyB ? BodyB->GetParentComponent() : nullptr;
		if (!BodyA || !BodyB || !CompA || !CompB)
		{
			LogRollingRelease(Pair, TEXT("InvalidBody"));
			SimulationWorld.RemoveDynamicContactPairAtSwap(Index);
			continue;
		}

		USphereSubBody* Sphere = Cast<USphereSubBody>(BodyA);
		UBoxSubBody* Box = Cast<UBoxSubBody>(BodyB);
		if (!Sphere || !Box)
		{
			Sphere = Cast<USphereSubBody>(BodyB);
			Box = Cast<UBoxSubBody>(BodyA);
		}
		const bool bUseRollingManifold =
			AreUnilateralRollingPairsEnabled() &&
			Sphere != nullptr && Box != nullptr;
		if (bUseRollingManifold && !bLoggedRollingSolveState)
		{
			bLoggedRollingSolveState = true;
			UE_LOG(LogTemp, Warning,
				TEXT("[PersistentPair][RollingSolveState] Frame=%u FirstSeen=%u LastSeen=%u PairCount=%d A=%s B=%s"),
				CurrentStepFrame, Pair.FirstSeenFrame, Pair.LastSeenFrame,
				DynamicContactPairs.Num(), *BodyA->GetName(), *BodyB->GetName());
		}
		if (!bUseRollingManifold &&
			CurrentStepFrame - Pair.LastSeenFrame > MaxUnseenFrames)
		{
			SimulationWorld.RemoveDynamicContactPairAtSwap(Index);
			continue;
		}

		// Scenario resets can rewind the component frame counter. A pair from the
		// previous timeline must never retain ownership of CCD in the new one.
		if (CurrentStepFrame < Pair.FirstSeenFrame)
		{
			if (CVarIAmSpeedDebugPersistentDynamicPairs.GetValueOnAnyThread() != 0)
			{
				UE_LOG(LogTemp, Warning,
					TEXT("[PersistentPair][ReleaseFrameRewind] Frame=%u FirstSeenFrame=%u"),
					CurrentStepFrame, Pair.FirstSeenFrame);
			}
			LogRollingRelease(Pair, TEXT("FrameRewind"));
			SimulationWorld.RemoveDynamicContactPairAtSwap(Index);
			continue;
		}

		// The ordinary end-of-frame pass must not resolve a newly registered pair
		// twice. The targeted TOI continuation is the only caller allowed to solve
		// it on its acquisition frame.
		if (CurrentStepFrame == Pair.FirstSeenFrame && !bSolveFirstSeenFrame)
		{
			Pair.bRollingManifoldReady = bUseRollingManifold;
			continue;
		}
		Pair.LastSolvedFrame = CurrentStepFrame;

		const FVector CurrentCOMA = CompA->GetPhysCOM();
		const FVector CurrentCOMB = CompB->GetPhysCOM();
		const float MaxExpectedMoveA = CompA->GetPhysCOMVelocity().Size() * Dt * 2.0f + 10.0f;
		const float MaxExpectedMoveB = CompB->GetPhysCOMVelocity().Size() * Dt * 2.0f + 10.0f;
		if (FVector::DistSquared(CurrentCOMA, Pair.LastCOMA) > FMath::Square(MaxExpectedMoveA) ||
			FVector::DistSquared(CurrentCOMB, Pair.LastCOMB) > FMath::Square(MaxExpectedMoveB))
		{
			LogRollingRelease(Pair, TEXT("DiscontinuousMotion"));
			SimulationWorld.RemoveDynamicContactPairAtSwap(Index);
			continue;
		}
		Pair.LastCOMA = CurrentCOMA;
		Pair.LastCOMB = CurrentCOMB;

		const FQuat RotA = CompA->GetPhysRotation();
		const FQuat RotB = CompB->GetPhysRotation();
		FVector N = RotB.RotateVector(Pair.LocalNormalB).GetSafeNormal();
		FVector AnchorA = CompA->GetPhysCOM() + RotA.RotateVector(Pair.LocalAnchorA);
		FVector AnchorB = CompB->GetPhysCOM() + RotB.RotateVector(Pair.LocalAnchorB);
		float ShapeSeparation = FVector::DotProduct(AnchorA - AnchorB, N);
		if (Sphere && Box)
		{
			const SSphere SphereShape = Sphere->MakeSphere();
			const SSBox BoxShape = Box->MakeBox();
			FVector ClosestPoint = FVector::ZeroVector;
			ShapeSeparation = BoxShape.SphereOBBSeparation(
				BoxShape.Rot,
				BoxShape.AbsoluteCenter(),
				SphereShape.Center,
				SphereShape.Radius,
				&ClosestPoint);
			const FVector CurrentBoxToSphereNormal =
				(SphereShape.Center - ClosestPoint).GetSafeNormal();
			const FVector StoredBoxToSphereNormal = Sphere == BodyA ? N : -N;
			if (ShapeSeparation > SeparationToleranceCm ||
				CurrentBoxToSphereNormal.IsNearlyZero() ||
				(!bUseRollingManifold &&
					FVector::DotProduct(CurrentBoxToSphereNormal, StoredBoxToSphereNormal) < 0.5f))
			{
				if (CVarIAmSpeedDebugPersistentDynamicPairs.GetValueOnAnyThread() != 0)
				{
					UE_LOG(LogTemp, Warning,
						TEXT("[PersistentPair][RejectGeometry] Frame=%u Separation=%.3f CurrentNormal=%s StoredNormal=%s"),
						CurrentStepFrame, ShapeSeparation,
						*CurrentBoxToSphereNormal.ToString(), *StoredBoxToSphereNormal.ToString());
				}
				LogRollingRelease(Pair, TEXT("Geometry"), ShapeSeparation);
				SimulationWorld.RemoveDynamicContactPairAtSwap(Index);
				continue;
			}

			if (bUseRollingManifold)
			{
				const FVector PreviousNormal = N;
				N = Sphere == BodyA
					? CurrentBoxToSphereNormal
					: -CurrentBoxToSphereNormal;
				const FVector SphereContactPoint =
					SphereShape.Center - CurrentBoxToSphereNormal * SphereShape.Radius;
				if (Sphere == BodyA)
				{
					AnchorA = SphereContactPoint;
					AnchorB = ClosestPoint;
				}
				else
				{
					AnchorA = ClosestPoint;
					AnchorB = SphereContactPoint;
				}

				Pair.LocalAnchorA = RotA.UnrotateVector(AnchorA - CompA->GetPhysCOM());
				Pair.LocalAnchorB = RotB.UnrotateVector(AnchorB - CompB->GetPhysCOM());
				Pair.LocalNormalB = RotB.UnrotateVector(N);
				Pair.LastSeenFrame = CurrentStepFrame;
				if (FVector::DotProduct(PreviousNormal, N) < 0.995f)
				{
					++Pair.FeatureTransitionCount;
					if (CVarIAmSpeedDebugPersistentDynamicPairs.GetValueOnAnyThread() != 0)
					{
						UE_LOG(LogTemp, Warning,
							TEXT("[PersistentPair][FeatureTransition] Frame=%u Separation=%.3f PreviousNormal=%s CurrentNormal=%s"),
							CurrentStepFrame, ShapeSeparation,
							*PreviousNormal.ToString(), *N.ToString());
					}
				}
			}
		}

		if (N.IsNearlyZero())
		{
			continue;
		}
		Pair.bRollingManifoldReady = bUseRollingManifold;

		const FVector OffsetA = AnchorA - CompA->GetPhysCOM();
		const FVector OffsetB = AnchorB - CompB->GetPhysCOM();
		const FVector VelocityA = CompA->GetPhysCOMVelocity() +
			FVector::CrossProduct(CompA->GetPhysAngularVelocity(), OffsetA);
		const FVector VelocityB = CompB->GetPhysCOMVelocity() +
			FVector::CrossProduct(CompB->GetPhysAngularVelocity(), OffsetB);
		const float Separation = bUseRollingManifold
			? ShapeSeparation
			: FVector::DotProduct(AnchorA - AnchorB, N);
		const float RelativeNormalVelocity = FVector::DotProduct(VelocityA - VelocityB, N);
		if (bUseRollingManifold)
		{
			++Pair.SupportedFrameCount;
			Pair.MinimumSeparation = FMath::Min(Pair.MinimumSeparation, Separation);
			Pair.MaximumSeparation = FMath::Max(Pair.MaximumSeparation, Separation);
		}
		const float PredictedRelativeNormalVelocity = RelativeNormalVelocity +
			Dt * FVector::DotProduct(
				CompA->GetPhysAccelerationAtPoint(AnchorA) -
				CompB->GetPhysAccelerationAtPoint(AnchorB), N);
		const float PenetrationSlop = FMath::Max(
			0.0f, CVarIAmSpeedRollingPenetrationSlop.GetValueOnAnyThread());
		if (bUseRollingManifold && Separation >= -PenetrationSlop &&
			PredictedRelativeNormalVelocity > FMath::Max(
				0.0f, CVarIAmSpeedRollingReleaseSpeed.GetValueOnAnyThread()))
		{
			LogRollingRelease(
				Pair, TEXT("SeparatingVelocity"), Separation,
				PredictedRelativeNormalVelocity);
			SimulationWorld.RemoveDynamicContactPairAtSwap(Index);
			continue;
		}

		// Retaining pair identity across a small positive gap must not create
		// speculative support. Activate the constraint only at contact or when
		// unconstrained motion would cross the contact plane during this tick.
		const bool bContactActive = !bUseRollingManifold ||
			Separation <= PenetrationSlop ||
			Separation + Dt * PredictedRelativeNormalVelocity <= PenetrationSlop;
		if (!bContactActive)
		{
			if (CVarIAmSpeedDebugPersistentDynamicPairs.GetValueOnAnyThread() != 0)
			{
				UE_LOG(LogTemp, Warning,
					TEXT("[PersistentPair][IdentityOnly] Frame=%u Separation=%.6f RelN=%.6f PredictedRelN=%.6f"),
					CurrentStepFrame, Separation, RelativeNormalVelocity,
					PredictedRelativeNormalVelocity);
			}
			continue;
		}
		if (bUseRollingManifold)
		{
			++Pair.ActiveContactFrameCount;
			Pair.bActiveContactLastSolve = true;
		}

		const float PenetrationDepth = FMath::Max(0.0f, -Separation);
		const float TargetRelativeNormalVelocity = bUseRollingManifold &&
			PenetrationDepth > PenetrationSlop
			? FMath::Min(
				FMath::Max(0.0f, CVarIAmSpeedRollingMaxCorrectionSpeed.GetValueOnAnyThread()),
				FMath::Max(0.0f, CVarIAmSpeedRollingBaumgarte.GetValueOnAnyThread()) *
					(PenetrationDepth - PenetrationSlop) / Dt)
			: 0.0f;
		const float InvMassA = 1.0f / FMath::Max(CompA->GetPhysMass(), 1.0f);
		const float InvMassB = 1.0f / FMath::Max(CompB->GetPhysMass(), 1.0f);
		const FVector AngularA = FVector::CrossProduct(
			BodyA->ComputeWorldInvInertiaTensor().TransformVector(
				FVector::CrossProduct(OffsetA, N)), OffsetA);
		const FVector AngularB = FVector::CrossProduct(
			BodyB->ComputeWorldInvInertiaTensor().TransformVector(
				FVector::CrossProduct(OffsetB, N)), OffsetB);
		const float Denominator = InvMassA + InvMassB +
			FVector::DotProduct(N, AngularA + AngularB);
		if (Denominator <= KINDA_SMALL_NUMBER)
		{
			continue;
		}

		const float NormalImpulseMagnitude = FMath::Max(
			0.0f,
			(TargetRelativeNormalVelocity - PredictedRelativeNormalVelocity) / Denominator);
		const float NormalActivity = NormalImpulseMagnitude * Denominator;
		// This is the velocity change required to prevent further normal closure.
		// Keep Baumgarte separation out of it: penetration correction is numerical
		// stabilization, not evidence of additional gameplay-shot activity.
		const float NormalSupportActivity = FMath::Max(
			0.0f, -PredictedRelativeNormalVelocity);
		if (bUseRollingManifold && NormalImpulseMagnitude > KINDA_SMALL_NUMBER)
		{
			++Pair.ReactionFrameCount;
			Pair.AccumulatedNormalImpulse += NormalImpulseMagnitude;
			Pair.AccumulatedNormalActivity += NormalActivity;
			Pair.AccumulatedNormalSupportActivity += NormalSupportActivity;
			Pair.PeakNormalActivity = FMath::Max(Pair.PeakNormalActivity, NormalActivity);
			Pair.PeakNormalSupportActivity = FMath::Max(
				Pair.PeakNormalSupportActivity, NormalSupportActivity);
			if (CVarIAmSpeedDebugPersistentDynamicPairs.GetValueOnAnyThread() != 0 &&
				(CurrentStepFrame - Pair.FirstSeenFrame) % 30u == 0u)
			{
				UE_LOG(LogTemp, Warning,
					TEXT("[PersistentPair][RollingTelemetry] Frame=%u PairKey=%llu LifetimeFrames=%u SupportedFrames=%u ReactionFrames=%u FeatureTransitions=%u NormalImpulse=%.6f NormalActivity=%.6f NormalSupportActivity=%.6f PeakNormalActivity=%.6f PeakNormalSupportActivity=%.6f Separation=%.6f RelN=%.6f"),
					CurrentStepFrame,
					Pair.PairKey,
					CurrentStepFrame - Pair.FirstSeenFrame + 1u,
					Pair.SupportedFrameCount,
					Pair.ReactionFrameCount,
					Pair.FeatureTransitionCount,
					Pair.AccumulatedNormalImpulse,
					Pair.AccumulatedNormalActivity,
					Pair.AccumulatedNormalSupportActivity,
					Pair.PeakNormalActivity,
					Pair.PeakNormalSupportActivity,
					Separation,
					RelativeNormalVelocity);
			}
		}
		const FVector Impulse = NormalImpulseMagnitude * N;
		const FVector ContactPoint = 0.5f * (AnchorA + AnchorB);
		FFakePhysicsImpactContext RollingFakePhysicsContext;
		if (bUseRollingManifold)
		{
			RollingFakePhysicsContext.SelfParentKinematics = Sphere == BodyA
				? CompA->GetKinematicState()
				: CompB->GetKinematicState();
			RollingFakePhysicsContext.OtherParentKinematics = Sphere == BodyA
				? CompB->GetKinematicState()
				: CompA->GetKinematicState();
			RollingFakePhysicsContext.FakeImpulseScale = FMath::Max(
				0.0f,
				CVarIAmSpeedRollingTangentialFakePhysicsScale.GetValueOnAnyThread());
			RollingFakePhysicsContext.bProjectFakeImpulseOntoContactPlane = true;
			RollingFakePhysicsContext.FakeTangentialVelocityScale = 1.0f;
			RollingFakePhysicsContext.bUseContactPointRelativeVelocity = true;
		}
		if (CVarIAmSpeedDebugPersistentDynamicPairs.GetValueOnAnyThread() != 0)
		{
			UE_LOG(LogTemp, Warning,
				TEXT("[PersistentPair][Impulse] Frame=%u A=%s B=%s J=%s Separation=%.3f RelN=%.3f NormalActivity=%.6f NormalSupportActivity=%.6f AccumulatedNormalActivity=%.6f AccumulatedNormalSupportActivity=%.6f"),
				CurrentStepFrame, *BodyA->GetName(), *BodyB->GetName(),
				*Impulse.ToString(), Separation, RelativeNormalVelocity,
				NormalActivity, NormalSupportActivity,
				Pair.AccumulatedNormalActivity,
				Pair.AccumulatedNormalSupportActivity);
		}
		if (NormalImpulseMagnitude > KINDA_SMALL_NUMBER)
		{
			BodyA->ApplyImpulse(Impulse, ContactPoint);
			BodyB->ApplyImpulse(-Impulse, ContactPoint);
		}

		if (bUseRollingManifold && NormalImpulseMagnitude > KINDA_SMALL_NUMBER)
		{
			const FVector PostVelocityA = CompA->GetPhysVelocityAtPoint(AnchorA);
			const FVector PostVelocityB = CompB->GetPhysVelocityAtPoint(AnchorB);
			const FVector RelativeVelocity = PostVelocityA - PostVelocityB;
			const FVector TangentialVelocity = RelativeVelocity -
				FVector::DotProduct(RelativeVelocity, N) * N;
			const float TangentialSpeed = TangentialVelocity.Size();
			if (TangentialSpeed > KINDA_SMALL_NUMBER)
			{
				const FVector Tangent = TangentialVelocity / TangentialSpeed;
				const FVector TangentialAngularA = FVector::CrossProduct(
					BodyA->ComputeWorldInvInertiaTensor().TransformVector(
						FVector::CrossProduct(OffsetA, Tangent)), OffsetA);
				const FVector TangentialAngularB = FVector::CrossProduct(
					BodyB->ComputeWorldInvInertiaTensor().TransformVector(
						FVector::CrossProduct(OffsetB, Tangent)), OffsetB);
				const float TangentialDenominator = InvMassA + InvMassB +
					FVector::DotProduct(Tangent, TangentialAngularA + TangentialAngularB);
				if (TangentialDenominator > KINDA_SMALL_NUMBER)
				{
					const float ManifoldFriction =
						USolidSubBody::ResolveSphereBoxManifoldFriction(
						*Sphere,
						*Box,
						USolidSubBody::MixFriction(
							Sphere->GetStaticFriction(), Box->GetStaticFriction(), EMixMode::E_Max));
					const float MaxFrictionImpulse =
						NormalImpulseMagnitude * FMath::Max(0.0f, ManifoldFriction);
					const float TangentialImpulseMagnitude = FMath::Min(
						TangentialSpeed / TangentialDenominator, MaxFrictionImpulse);
					const FVector TangentialImpulse = -TangentialImpulseMagnitude * Tangent;
					BodyA->ApplyImpulse(TangentialImpulse, ContactPoint);
					BodyB->ApplyImpulse(-TangentialImpulse, ContactPoint);
				}
			}

		}

		// A valid active manifold represents one contact during this simulation
		// frame. Evaluate fake physics exactly once, with the manifold owning the
		// normal response and the fake share projected into its tangent plane.
		if (bUseRollingManifold)
		{
			const FVector BoxToSphereNormal = Sphere == BodyA ? N : -N;
			SHitResult RollingHit(
				true,
				ContactPoint,
				BoxToSphereNormal,
				0.0f);
			RollingHit.Location = Sphere->GetKinematicState().Location;
			RollingHit.PenetrationDepth = PenetrationDepth;
			RollingHit.ContactPointThis = Sphere == BodyA ? AnchorA : AnchorB;
			RollingHit.ContactPointOther = Sphere == BodyA ? AnchorB : AnchorA;
			RollingHit.SubBody = Box;
			Sphere->ApplyFakePhysicsOn(
				*Box, RollingHit, Dt, &RollingFakePhysicsContext);
		}
	}
}

void USpeedWorldSubsystem::ProjectDynamicContactPairs()
{
	auto& DynamicContactPairs = SimulationWorld.GetDynamicContactPairs();
	auto& PendingRollingContactPairs = SimulationWorld.GetPendingRollingContactPairs();
	if (!USphereSubBody::IsSphereBoxProjectionEnabled())
	{
		return;
	}

	const auto ProjectPairs = [](TArray<FDynamicContactPair>& Pairs)
	{
		for (FDynamicContactPair& Pair : Pairs)
		{
			USolidSubBody* BodyA = Pair.BodyA.Get();
			USolidSubBody* BodyB = Pair.BodyB.Get();
			if (!BodyA || !BodyB)
			{
				continue;
			}

			USphereSubBody* Sphere = Cast<USphereSubBody>(BodyA);
			UBoxSubBody* Box = Cast<UBoxSubBody>(BodyB);
			if (!Sphere || !Box)
			{
				Sphere = Cast<USphereSubBody>(BodyB);
				Box = Cast<UBoxSubBody>(BodyA);
			}
			if (Sphere && Box)
			{
				Sphere->ProjectOutOfBox(*Box);
			}
		}
	};

	ProjectPairs(DynamicContactPairs);
	ProjectPairs(PendingRollingContactPairs);

	// New encounters and teleports do not necessarily own a persistent
	// manifold yet. Measure every independent sphere/box pair from refreshed
	// parent states so the next frame cannot inherit an overlap.
	for (USphereSubBody* Sphere : SimulationWorld.GetProjectionSpheres())
	{
		for (UBoxSubBody* Box : SimulationWorld.GetProjectionBoxes())
		{
			if (Sphere && Box &&
				Sphere->GetParentComponent() != Box->GetParentComponent())
			{
				Sphere->ProjectOutOfBox(*Box);
			}
		}
	}
}

void USpeedWorldSubsystem::PrepareCanonicalFrame(
	const FCanonicalFrameContext& Context)
{
	ApplyPendingOps();
	RebuildSortedIfNeeded();

	for (ISpeedComponent* Component : SimulationWorld.GetOrderedAdapters())
	{
		if (Component)
		{
			IAMSPEED_ACTOR_SCOPE(SimulationWorld, *Component, Prepare);
			Component->PrepareCanonicalFrame(Context);
		}
	}
}

ECanonicalRunControlState USpeedWorldSubsystem::GetCanonicalRunControlState()
{
	ApplyPendingOps();
	RebuildSortedIfNeeded();

	bool bFoundController = false;
	bool bAllReady = true;
	bool bAllComplete = true;
	for (const ISpeedComponent* Component : SimulationWorld.GetOrderedAdapters())
	{
		if (!Component || !Component->IsCanonicalRunController())
		{
			continue;
		}

		bFoundController = true;
		bAllReady &= Component->IsCanonicalRunReady();
		bAllComplete &= Component->IsCanonicalRunComplete();
	}

	if (!bFoundController)
	{
		return ECanonicalRunControlState::Uncontrolled;
	}
	if (!bAllReady)
	{
		return ECanonicalRunControlState::WaitingForScenario;
	}
	return bAllComplete
		? ECanonicalRunControlState::Complete
		: ECanonicalRunControlState::Ready;
}

FSimulationSnapshot USpeedWorldSubsystem::CaptureSimulationSnapshot(
	const uint64 NumFrame, const uint64 InputJournalHash)
{
	RebuildSortedIfNeeded();
	return SimulationWorld.CaptureSnapshot(NumFrame, InputJournalHash);
}

bool USpeedWorldSubsystem::RestoreSimulationSnapshot(
	const FSimulationSnapshot& Snapshot,
	const uint64 ExpectedInputJournalHash)
{
	ApplyPendingOps();
	RebuildSortedIfNeeded();
	return SimulationWorld.RestoreSnapshot(Snapshot, ExpectedInputJournalHash);
}

uint64 USpeedWorldSubsystem::GetSimulationStableId(const ISpeedComponent& Component)
{
	ApplyPendingOps();
	RebuildSortedIfNeeded();
	return SimulationWorld.FindStableId(Component);
}

bool USpeedWorldSubsystem::ApplySimulationInputs(
	const uint64 NumFrame, const Speed::SimulationBoundary::FInputJournal& Journal)
{
	RebuildSortedIfNeeded();
	return SimulationWorld.ApplyInputsForFrame(NumFrame, Journal);
}

void USpeedWorldSubsystem::Step(const float& Dt, const float& SimTime, const unsigned int& Frame)
{
	unsigned int& CurrentStepFrame = SimulationWorld.GetCurrentStepFrame();
	const double StepStartSeconds = FPlatformTime::Seconds();
	LastStepDiagnostics = FSpeedStepDiagnostics();
	LastStepDiagnostics.Serial = ++StepSerial;
	LastStepDiagnostics.Frame = Frame;
	LastStepDiagnostics.PhysicalDeltaTimeMilliseconds =
		static_cast<double>(Dt) * 1000.0;
	constexpr int32 MaxIter = 24;
	LastStepDiagnostics.MaximumIterationCount = MaxIter;
	CurrentStepFrame = Frame;
    ApplyPendingOps();
    RebuildSortedIfNeeded();

    if (Dt <= 0.f || SimulationWorld.GetOrderedAdapters().Num() == 0)
	{
		LastStepDiagnostics.TotalMilliseconds =
			(FPlatformTime::Seconds() - StepStartSeconds) * 1000.0;
        return;
	}

    // ------------------------------------------------------------
    // 1) Reset frame
    // ------------------------------------------------------------
	const double ResetStartSeconds = FPlatformTime::Seconds();
	for (ISpeedComponent* Comp : SimulationWorld.GetOrderedAdapters())
    {
        if (!Comp) continue;

		IAMSPEED_ACTOR_SCOPE(SimulationWorld, *Comp, Reset);
		Comp->UpdateSubBodiesKinematics();
        Comp->ResetForFrame(Dt);
    }

	LastStepDiagnostics.ResetMilliseconds =
		(FPlatformTime::Seconds() - ResetStartSeconds) * 1000.0;

    // Anti double-resolve (pair) on the frame
    Speed::Collision::FResolvedPairSet ResolvedPairs;

    float TimePassed = 0.f;
    float LastSubDelta = Dt;
	const float CompletionTolerance = FMath::Max(1.0e-8f, Dt * 1.0e-6f);
	const auto IntegrateAll = [&](const float SubDelta)
	{
		LastSubDelta = SubDelta;
		const double IntegrateStartSeconds = FPlatformTime::Seconds();
		for (ISpeedComponent* Comp : SimulationWorld.GetOrderedAdapters())
		{
			if (Comp)
			{
				IAMSPEED_ACTOR_SCOPE(SimulationWorld, *Comp, Integrate);
				Comp->IntegrateKinematics(SubDelta);
			}
		}
		LastStepDiagnostics.IntegrateMilliseconds +=
			(FPlatformTime::Seconds() - IntegrateStartSeconds) * 1000.0;
		TimePassed += SubDelta;
	};

    int32 Iter = 0;
	while (TimePassed < Dt)
    {
        const float Remaining = Dt - TimePassed;
		if (Remaining <= CompletionTolerance)
		{
			IntegrateAll(Remaining);
			break;
		}
		if (Iter >= MaxIter)
		{
			LastStepDiagnostics.bIterationLimitReached = true;
			IntegrateAll(Remaining);
			break;
		}
		++Iter;
		LastStepDiagnostics.IterationCount = Iter;

        // ------------------------------------------------------------
		// 2) Find every independent hit at the global earliest TOI. The
		// iteration budget bounds temporal substeps, not the number of objects
		// that happen to collide at the same instant.
        // ------------------------------------------------------------
		constexpr float SimultaneousTOITolerance = 1.0e-9f;
		TArray<SComponentTOI, TInlineAllocator<16>> EarliestHits;
		float EarliestTOI = Remaining;

        const double SweepStartSeconds = FPlatformTime::Seconds();
        for (ISpeedComponent* Comp : SimulationWorld.GetOrderedAdapters())
        {
            if (!Comp) continue;
			++LastStepDiagnostics.ComponentSweepCount;

            SComponentTOI Ctoi;
            {
                IAMSPEED_ACTOR_SCOPE(SimulationWorld, *Comp, Sweep);
                Ctoi = Comp->SweepTOISubBodies(Remaining, LastSubDelta);
            }

            // TOI sanity
            if (!Ctoi.bHit)
                continue;
			// Once an analytical provider has produced its event impulse during
			// this canonical frame, it is an established unilateral constraint.
			// Rediscovering the same time-zero event would only consume the bounded
			// CCD iteration budget; IntegrateKinematics now owns its continuation.
			if (Ctoi.Hit.SurfaceId != 0 &&
				ResolvedPairs.Contains(Ctoi.PairKey))
			{
				continue;
			}

            const float T = FMath::Clamp(Ctoi.TOI, 0.f, Remaining);
			if (EarliestHits.IsEmpty() ||
				T < EarliestTOI - SimultaneousTOITolerance)
            {
				EarliestTOI = T;
				EarliestHits.Reset();
				SComponentTOI& Earliest = EarliestHits.Add_GetRef(Ctoi);
				Earliest.TOI = T;
            }
			else if (FMath::IsNearlyEqual(
				T, EarliestTOI, SimultaneousTOITolerance))
            {
				SComponentTOI& Earliest = EarliestHits.Add_GetRef(Ctoi);
				Earliest.TOI = T;
            }
        }
		LastStepDiagnostics.SweepMilliseconds +=
			(FPlatformTime::Seconds() - SweepStartSeconds) * 1000.0;
		EarliestHits.Sort([](const SComponentTOI& A, const SComponentTOI& B)
		{
			if (A.PairKey != B.PairKey)
			{
				return A.PairKey < B.PairKey;
			}
			const USSubBody* ResolverA = A.Resolver.Get();
			const USSubBody* ResolverB = B.Resolver.Get();
			const uint32 ResolverIdA = ResolverA ? ResolverA->GetUniqueID() : 0u;
			const uint32 ResolverIdB = ResolverB ? ResolverB->GetUniqueID() : 0u;
			return ResolverIdA < ResolverIdB;
		});

        // ------------------------------------------------------------
		// 3) Integrate everyone once to the shared TOI, or through the full
		// remaining interval when there is no hit.
        // ------------------------------------------------------------
		const bool bWillResolve = !EarliestHits.IsEmpty();
		const float SubDelta = FMath::Clamp(
			bWillResolve ? EarliestTOI : Remaining, 0.0f, Remaining);
		IntegrateAll(SubDelta);

        // No hit => End
        if (!bWillResolve)
            break;

        // ------------------------------------------------------------
		// 4) Resolve the simultaneous batch once per pair. All candidates were
		// measured from the same pre-resolution state, so independent objects do
		// not consume additional temporal iterations.
        // ------------------------------------------------------------
		for (const SComponentTOI& Hit : EarliestHits)
		{
			USSubBody* Resolver = Hit.Resolver.Get();
			if (!Resolver || ResolvedPairs.Contains(Hit.PairKey))
			{
				continue;
			}

			Resolver->SetFutureHit(Hit.Hit);
			Resolver->AcceptHit();
			if (UPrimitiveComponent* HitComponent =
				Resolver->GetHit().Component.Get())
			{
				if (Resolver->ComponentHasBeenIgnored(*HitComponent))
				{
					ResolvedPairs.Add(Hit.PairKey);
				}
			}
			USolidSubBody* RollingBodyA = Cast<USolidSubBody>(Resolver);
			USolidSubBody* RollingBodyB =
				Cast<USolidSubBody>(Resolver->GetHit().SubBody.Get());

			const double ResolveStartSeconds = FPlatformTime::Seconds();
			Resolver->ResolveCurrentHit(SubDelta, SimTime);
			LastStepDiagnostics.ResolveMilliseconds +=
				(FPlatformTime::Seconds() - ResolveStartSeconds) * 1000.0;
			++LastStepDiagnostics.ResolvedEventCount;
			if (Hit.Hit.SurfaceId != 0)
			{
				ResolvedPairs.Add(Hit.PairKey);
			}
			if (RollingBodyA && RollingBodyB)
			{
				ActivatePendingRollingContactPairAtTOI(
					*RollingBodyA, *RollingBodyB, Dt - TimePassed);
			}
		}

        // Optional: post update at each substep (useful if certain gameplay sensors need to react "immediately")
        /*
        for (ISpeedComponent* Comp : SimulationWorld.GetOrderedAdapters())
        {
            if (!Comp) continue;
            Comp->PostPhysicsUpdate();
        }
        */
    }

	// A contact acquired at the last TOI has no remaining integration substep.
	// Project once at the frame boundary so PostPhysics observers never receive
	// an infeasible analytical pose. Moving the body here also avoids changing
	// the within-frame CCD trajectory from inside an impact callback.
	const double ProjectionStartSeconds = FPlatformTime::Seconds();
	for (ISpeedComponent* Comp : SimulationWorld.GetOrderedAdapters())
	{
		if (Comp)
		{
			IAMSPEED_ACTOR_SCOPE(SimulationWorld, *Comp, Projection);
			Comp->ProjectEstablishedStaticContacts(Dt);
		}
	}

    // Make persistent finite-mass contacts geometrically feasible before any
    // PostPhysics observer samples them.
	ProjectDynamicContactPairs();
	LastStepDiagnostics.ProjectionMilliseconds =
		(FPlatformTime::Seconds() - ProjectionStartSeconds) * 1000.0;

    // ------------------------------------------------------------
    // 5) Final post update
    // ------------------------------------------------------------
	const double PostStartSeconds = FPlatformTime::Seconds();
    for (ISpeedComponent* Comp : SimulationWorld.GetOrderedAdapters())
    {
        if (!Comp) continue;
		IAMSPEED_ACTOR_SCOPE(SimulationWorld, *Comp, Post);
		Comp->PostPhysicsUpdate(Dt);
	}

	if (!SimulationWorld.GetPendingRollingContactPairs().IsEmpty())
	{
		SimulationWorld.MergePendingRollingContactPairs();
	}

	SolveDynamicContactPairs(Dt);
	ProjectDynamicContactPairs();
	LastStepDiagnostics.PostMilliseconds =
		(FPlatformTime::Seconds() - PostStartSeconds) * 1000.0;
	LastStepDiagnostics.TotalMilliseconds =
		(FPlatformTime::Seconds() - StepStartSeconds) * 1000.0;
	const Speed::Analytic::FStaticWorldQueryCounters QueryCounters =
		Speed::Analytic::FStaticWorldQueryAudit::GetCurrentFrameCounters();
	LastStepDiagnostics.StaticQueryCount = QueryCounters.QueryCount;
	LastStepDiagnostics.LegacySweepCount = QueryCounters.LegacySweepCount;
	LastStepDiagnostics.AuthorityAttemptCount = QueryCounters.AuthorityAttemptCount;
	LastStepDiagnostics.StaticAuthorityMilliseconds =
		QueryCounters.AuthorityMilliseconds;
	LastStepDiagnostics.MaximumStaticAuthorityMilliseconds =
		QueryCounters.MaximumAuthorityMilliseconds;

	if (LastStepDiagnostics.bIterationLimitReached)
	{
		UE_LOG(LogTemp, Error,
			TEXT("[SpeedSolverIterationLimit] Frame=%u Iterations=%d MaxIterations=%d ComponentSweeps=%d StaticQueries=%u AuthorityAttempts=%u ResolvedEvents=%d TimePassed=%.9g Dt=%.9g StepMs=%.6f"),
			Frame, LastStepDiagnostics.IterationCount,
			LastStepDiagnostics.MaximumIterationCount,
			LastStepDiagnostics.ComponentSweepCount,
			LastStepDiagnostics.StaticQueryCount,
			LastStepDiagnostics.AuthorityAttemptCount,
			LastStepDiagnostics.ResolvedEventCount,
			TimePassed, Dt, LastStepDiagnostics.TotalMilliseconds);
	}
}
