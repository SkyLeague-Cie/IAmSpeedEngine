// Fill out your copyright notice in the Description page of Project Settings.

// USpeedWorldSubsystem.cpp

#include "SpeedWorldSubsystem.h"
#include "IAmSpeed/Actors/SpeedStaticActor.h"
#include "IAmSpeed/Components/ISpeedComponent.h"
#include "IAmSpeed/World/CanonicalFrameContext.h"
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

void USpeedWorldSubsystem::OnWorldBeginPlay(UWorld& InWorld)
{
	Super::OnWorldBeginPlay(InWorld);
	StaticCollisionWorld.Reset();
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

	TUniquePtr<Speed::Analytic::FAnalyticWorldData> Imported =
		MakeUnique<Speed::Analytic::FAnalyticWorldData>();
	uint64 CombinedSourceHash = 0;
	bool bSourceReadinessPending = false;

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
	for (const AActor* SourceActor : AnalyticMeshSourceActors)
	{
		TArray<UStaticMeshComponent*> MeshComponents;
		if (const ASpeedStaticActor* SpeedStaticActor =
			Cast<ASpeedStaticActor>(SourceActor))
		{
			SpeedStaticActor->GatherAnalyticStaticMeshes(MeshComponents);
		}
		else if (const USpeedAnalyticSourceComponent* SourceComponent =
			SourceActor->FindComponentByClass<USpeedAnalyticSourceComponent>())
		{
			SourceComponent->GatherOwnedStaticMeshes(MeshComponents);
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
			AnalyticSourceComponents.Add(
				Speed::Analytic::StableStringId(MeshComponent->GetPathName()),
				const_cast<UStaticMeshComponent*>(MeshComponent));
			const FString StableIdentity = FString::Printf(
				TEXT("%s|%s|%s|%s"), *SourceActor->GetPathName(),
				*MeshComponent->GetPathName(), *Mesh->GetPathName(),
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
	const FString WorldName = FPackageName::GetShortName(World->GetOutermost());
	const FString BakedPackagePath = FString::Printf(
		TEXT("/Game/Generated/Analytic/%s_AnalyticWorld"), *WorldName);
	const FString BakedObjectPath = FString::Printf(
		TEXT("%s.%s_AnalyticWorld"), *BakedPackagePath, *WorldName);
	if (const USpeedAnalyticCollisionAsset* BakedAsset =
		LoadObject<USpeedAnalyticCollisionAsset>(nullptr, *BakedObjectPath))
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
		const TSharedPtr<const Speed::Analytic::FAnalyticWorldData> BakedRuntime =
			BakedAsset->BuildRuntimeData(&BakedReason);
		int32 RuntimePlaneAuthorityCount = 0;
		int32 RuntimeQuinticAuthorityCount = 0;
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
		}
		const int32 RuntimePlaneCount = BakedRuntime ? BakedRuntime->Planes.Num() : 0;
		const int32 RuntimeQuinticCount = BakedRuntime ?
			BakedRuntime->ExtrudedQuinticPatches.Num() : 0;
		UE_LOG(LogTemp, Display,
			TEXT("[AnalyticMeshBakeAuthority] BakeSchema=%u SerializedPlanes=%d/%d SerializedQuintics=%d/%d RuntimePlanes=%d/%d RuntimeQuintics=%d/%d"),
			BakedAsset->BakeSchemaVersion,
			SerializedPlaneAuthorityCount, BakedAsset->BoundedPlanes.Num(),
			SerializedQuinticAuthorityCount,
			BakedAsset->ExtrudedQuinticPatches.Num(),
			RuntimePlaneAuthorityCount, RuntimePlaneCount,
			RuntimeQuinticAuthorityCount, RuntimeQuinticCount);
		if (BakedRuntime && BakedInventoryHash == MeshSourceInventoryHash &&
			BakedAsset->MeshSources.Num() == MeshSourceComponentCount)
		{
			Imported->Triangles = BakedRuntime->Triangles;
			Imported->Planes = BakedRuntime->Planes;
			Imported->ExtrudedQuinticPatches =
				BakedRuntime->ExtrudedQuinticPatches;
			CombinedSourceHash = Speed::Analytic::CombineStableIds(
				CombinedSourceHash, BakedRuntime->SourceHash);
			bMeshBakeLoaded = true;
		}
		else
		{
			UE_LOG(LogTemp, Warning,
				TEXT("[AnalyticMeshBake] Asset=%s Result=Rejected LiveHash=%016llX BakedHash=%016llX Detail=%s"),
				*BakedObjectPath, MeshSourceInventoryHash, BakedInventoryHash,
				BakedRuntime ? TEXT("Source inventory mismatch.") : *BakedReason);
		}
	}
	UE_LOG(LogTemp, Display,
		TEXT("[AnalyticMeshSourceInventory] Actors=%d Components=%d Hash=%016llX Baked=%d Triangles=%d"),
		AnalyticMeshSourceActors.Num(), MeshSourceComponentCount,
		MeshSourceInventoryHash, bMeshBakeLoaded ? 1 : 0, Imported->Triangles.Num());

	for (const ALandscapeProxy* Landscape : Landscapes)
	{
		if (!Landscape)
		{
			continue;
		}
		if (!Landscape->CollisionComponents.IsEmpty() &&
			Landscape->CollisionComponents[0])
		{
			AnalyticSourceComponents.Add(
				Speed::Analytic::StableStringId(Landscape->GetPathName()),
				Landscape->CollisionComponents[0]);
		}
		const uint64 LandscapeSourceId =
			Speed::Analytic::StableStringId(Landscape->GetPathName());
		if (Imported->Planes.ContainsByPredicate(
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
			Imported->Planes.Add(Output.Plane);
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
	Imported->SourceHash = CombinedSourceHash;
	FString ValidationReason;
	if (!Imported->FinalizeAndValidate(&ValidationReason))
	{
		UE_LOG(LogTemp, Warning,
			TEXT("[AnalyticWorldBuild] Rejected Detail=%s"), *ValidationReason);
		return;
	}
	int32 BoundaryEdgeCount = 0;
	int32 NonManifoldEdgeCount = 0;
	int32 SmoothEdgeCount = 0;
	int32 CreaseEdgeCount = 0;
	int32 PlanarCandidateCount = 0;
	int32 ValidShapeSampleCount = 0;
	for (const Speed::Analytic::FTriangleMeshEdge& Edge : Imported->MeshEdges)
	{
		BoundaryEdgeCount += Edge.IsBoundary() ? 1 : 0;
		NonManifoldEdgeCount += Edge.IsNonManifold() ? 1 : 0;
		SmoothEdgeCount += Edge.Continuity ==
			Speed::Analytic::EEdgeContinuity::Smooth ? 1 : 0;
		CreaseEdgeCount += Edge.Continuity ==
			Speed::Analytic::EEdgeContinuity::Crease ? 1 : 0;
	}
	for (const Speed::Analytic::FSurfacePatch& Patch : Imported->SurfacePatches)
	{
		PlanarCandidateCount += Patch.Kind ==
			Speed::Analytic::ESurfacePatchKind::PlanarCandidate ? 1 : 0;
	}
	for (const Speed::Analytic::FVertexShapeSample& Sample : Imported->VertexShapeSamples)
	{
		ValidShapeSampleCount += Sample.bValid ? 1 : 0;
	}
	UE_LOG(LogTemp, Display,
		TEXT("[AnalyticWorldBuild] Attempt=%u Planes=%d ExtrudedQuintics=%d Triangles=%d Vertices=%d Edges=%d BoundaryEdges=%d NonManifoldEdges=%d SmoothEdges=%d CreaseEdges=%d SmoothRegions=%d Patches=%d PlanarCandidates=%d ShapeValid=%d BvhNodes=%d Hash=%016llX AuthorityEligible=%d PendingSource=%d"),
		AnalyticWorldBuildAttempt, Imported->Planes.Num(),
		Imported->ExtrudedQuinticPatches.Num(), Imported->Triangles.Num(),
		Imported->MeshVertices.Num(), Imported->MeshEdges.Num(), BoundaryEdgeCount,
		NonManifoldEdgeCount, SmoothEdgeCount, CreaseEdgeCount,
		Imported->SmoothSurfaceRegions.Num(), Imported->SurfacePatches.Num(), PlanarCandidateCount,
		ValidShapeSampleCount, Imported->TriangleBvh.Num(), Imported->StableHash(),
		Imported->IsAuthorityEligible() ? 1 : 0,
		bSourceReadinessPending ? 1 : 0);
	AnalyticWorldData = MoveTemp(Imported);
	StaticCollisionWorld = MakeUnique<Speed::FAnalyticStaticCollisionWorld>(
		*AnalyticWorldData);
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
            bool bExists = false;
            for (auto& W : Components) { if (W == Op.Comp) { bExists = true; break; } }
			if (!bExists) AddComponent(*Op.Comp);
        }
        else
        {
			RemoveComponent(*Op.Comp);
        }
    }

    bDirtyOrder = true;
}

void USpeedWorldSubsystem::RebuildSortedIfNeeded()
{
    if (!bDirtyOrder)
        return;

	// Clean invalids
    Components.RemoveAll([](const ISpeedComponent* W)
        {
            return !W;
        });

    ComponentsSorted = Components;

    // Deterministic order:
    // - first Owner->GetUniqueID (stable during a run)
    // - then Object UniqueID of the component
    // Commented for the moment since there is a crash happening in the sort
    /*Algo::Sort(ComponentsSorted, [](const ISpeedComponent* A, const ISpeedComponent* B)
        {
            const UObject* OA = reinterpret_cast<const UObject*>(A);
            const UObject* OB = reinterpret_cast<const UObject*>(B);
            if (!OA || !OB) return OA != nullptr; // valid ones first

            AActor* AA = Cast<AActor>(OA->GetOuter());
            AActor* AB = Cast<AActor>(OB->GetOuter());

            const uint32 AOwnerId = AA ? (uint32)AA->GetUniqueID() : 0u;
            const uint32 BOwnerId = AB ? (uint32)AB->GetUniqueID() : 0u;

            if (AOwnerId != BOwnerId)
                return AOwnerId < BOwnerId;

            return OA->GetUniqueID() < OB->GetUniqueID();
        });*/

    bDirtyOrder = false;
}

void USpeedWorldSubsystem::AddComponent(ISpeedComponent& Comp)
{
    // Add New component to all existing ones
    for (ISpeedComponent* Component : Components)
    {
		Component->AddExternalSubBodies(Comp.GetSubBodies());
    }

	// Add existing ones to the new component
    for (ISpeedComponent* Component : Components)
	{
        Comp.AddExternalSubBodies(Component->GetSubBodies());
	}

	// Finally add to list
    Components.Add(&Comp);
}

void USpeedWorldSubsystem::RemoveComponent(ISpeedComponent& Comp)
{
	// Remove component from list
    Components.RemoveAll([&](const ISpeedComponent* W)
    {
        return !W || W == &Comp;
    });

	// Remove sub-bodies of the removed component from all existing ones
	for (ISpeedComponent* Component : Components)
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
	FDynamicContactPair* Pair = DynamicContactPairs.FindByPredicate(
		[PairKey](const FDynamicContactPair& Candidate)
		{
			return Candidate.PairKey == PairKey;
		});
	TArray<FDynamicContactPair>* DestinationPairs = &DynamicContactPairs;
	if (!Pair && AreUnilateralRollingPairsEnabled())
	{
		// Collision resolution runs inside the TOI iteration. Queue structural
		// changes so the active pair array remains immutable until the iteration
		// has completed.
		DestinationPairs = &PendingRollingContactPairs;
		Pair = PendingRollingContactPairs.FindByPredicate(
			[PairKey](const FDynamicContactPair& Candidate)
			{
				return Candidate.PairKey == PairKey;
			});
	}
	if (!Pair)
	{
		Pair = &DestinationPairs->AddDefaulted_GetRef();
		Pair->BodyA = &BodyA;
		Pair->BodyB = &BodyB;
		Pair->PairKey = PairKey;
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
	const int32 PendingIndex = PendingRollingContactPairs.IndexOfByPredicate(
		[PairKey](const FDynamicContactPair& Candidate)
		{
			return Candidate.PairKey == PairKey;
		});
	if (PendingIndex == INDEX_NONE)
	{
		return;
	}

	if (!DynamicContactPairs.ContainsByPredicate(
		[PairKey](const FDynamicContactPair& Candidate)
		{
			return Candidate.PairKey == PairKey;
		}))
	{
		DynamicContactPairs.Add(MoveTemp(PendingRollingContactPairs[PendingIndex]));
	}
	PendingRollingContactPairs.RemoveAtSwap(PendingIndex, 1, EAllowShrinking::No);

	if (RemainingDt > KINDA_SMALL_NUMBER)
	{
		SolveDynamicContactPairs(RemainingDt, PairKey, true, true);
	}
}

bool USpeedWorldSubsystem::IsDynamicContactPairOwnedByRollingManifold(
	const USolidSubBody& BodyA,
	const USolidSubBody& BodyB) const
{
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
	return DynamicContactPairs.ContainsByPredicate(
		[PairKey, this](const FDynamicContactPair& Pair)
		{
			// Once an acquired pair has been solved at its TOI, it owns the remaining
			// substep and later sweeps must yield to the manifold.
			return Pair.PairKey == PairKey &&
				Pair.bRollingManifoldReady &&
				Pair.BodyA.IsValid() && Pair.BodyB.IsValid();
		});
}

ERollingManifoldContactState USpeedWorldSubsystem::GetRollingManifoldContactState(
	const USolidSubBody& Body) const
{
	if (!AreUnilateralRollingPairsEnabled())
	{
		return ERollingManifoldContactState::Absent;
	}

	ERollingManifoldContactState Result = ERollingManifoldContactState::Absent;
	for (const FDynamicContactPair& Pair : DynamicContactPairs)
	{
		if (!Pair.bRollingManifoldReady ||
			!Pair.BodyA.IsValid() || !Pair.BodyB.IsValid() ||
			(Pair.BodyA.Get() != &Body && Pair.BodyB.Get() != &Body))
		{
			continue;
		}

		if (Pair.bActiveContactLastSolve)
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
	constexpr unsigned int MaxUnseenFrames = 2;
	const float SeparationToleranceCm = FMath::Max(
		0.0f, CVarIAmSpeedRollingSeparationTolerance.GetValueOnAnyThread());
	const auto LogRollingRelease = [this](
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

	for (int32 Index = DynamicContactPairs.Num() - 1; Index >= 0; --Index)
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
			DynamicContactPairs.RemoveAtSwap(Index, 1, EAllowShrinking::No);
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
			DynamicContactPairs.RemoveAtSwap(Index, 1, EAllowShrinking::No);
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
			DynamicContactPairs.RemoveAtSwap(Index, 1, EAllowShrinking::No);
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
			DynamicContactPairs.RemoveAtSwap(Index, 1, EAllowShrinking::No);
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
				DynamicContactPairs.RemoveAtSwap(Index, 1, EAllowShrinking::No);
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
			DynamicContactPairs.RemoveAtSwap(Index, 1, EAllowShrinking::No);
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
	TArray<USphereSubBody*> Spheres;
	TArray<UBoxSubBody*> Boxes;
	for (ISpeedComponent* Component : ComponentsSorted)
	{
		if (!Component)
		{
			continue;
		}
		for (USSubBody* SubBody : Component->GetSubBodies())
		{
			if (USphereSubBody* Sphere = Cast<USphereSubBody>(SubBody))
			{
				Spheres.Add(Sphere);
			}
			else if (UBoxSubBody* Box = Cast<UBoxSubBody>(SubBody))
			{
				Boxes.Add(Box);
			}
		}
	}
	for (USphereSubBody* Sphere : Spheres)
	{
		for (UBoxSubBody* Box : Boxes)
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

	for (ISpeedComponent* Component : ComponentsSorted)
	{
		if (Component)
		{
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
	for (const ISpeedComponent* Component : ComponentsSorted)
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

void USpeedWorldSubsystem::Step(const float& Dt, const float& SimTime, const unsigned int& Frame)
{
	CurrentStepFrame = Frame;
    ApplyPendingOps();
    RebuildSortedIfNeeded();

    if (Dt <= 0.f || ComponentsSorted.Num() == 0)
        return;

    // ------------------------------------------------------------
    // 1) Reset frame
    // ------------------------------------------------------------
	for (ISpeedComponent* Comp : ComponentsSorted)
    {
        if (!Comp) continue;

		Comp->UpdateSubBodiesKinematics();
        Comp->ResetForFrame(Dt);
    }

    // Anti double-resolve (pair) on the frame
    TSet<uint64> ResolvedPairs;
    ResolvedPairs.Reserve(128);

    float TimePassed = 0.f;

    // Very important: if you leave 0, you risk an infinite loop on TOI==0
	// -> No it should not, SubBody ignores component once it hit it during the frame, so it should just continue to the next hit.
    // const float MinStep = 1e-6f;
    const float MinStep = 0.0f;
    const int32 MaxIter = 24;
    float LastSubDelta = Dt;

    int32 Iter = 0;
    while (TimePassed < Dt && Iter++ < MaxIter)
    {
        const float Remaining = Dt - TimePassed;
        if (Remaining <= MinStep)
            break;

        // ------------------------------------------------------------
        // 2) Find global earliest TOI
        // ------------------------------------------------------------
        SComponentTOI Best;
        Best.bHit = false;
        Best.TOI = Remaining;

        for (ISpeedComponent* Comp : ComponentsSorted)
        {
            if (!Comp) continue;

            const SComponentTOI Ctoi = Comp->SweepTOISubBodies(Remaining, LastSubDelta);

            // TOI sanity
            if (!Ctoi.bHit)
                continue;

            const float T = FMath::Clamp(Ctoi.TOI, 0.f, Remaining);

            // Deterministic tie-break if equal
            // (super important: two hits can have very close TOI)
            if (T < Best.TOI - 1e-9f)
            {
                Best = Ctoi;
                Best.TOI = T;
            }
            else if (FMath::IsNearlyEqual(T, Best.TOI, 1e-9f))
            {
                // tie-break : PairKey then resolver unique id
                if (Ctoi.PairKey < Best.PairKey)
                {
                    Best = Ctoi;
                    Best.TOI = T;
                }
                else if (Ctoi.PairKey == Best.PairKey)
                {
                    USSubBody* R1 = Ctoi.Resolver.Get();
                    USSubBody* R2 = Best.Resolver.Get();
                    const uint32 Id1 = R1 ? (uint32)R1->GetUniqueID() : 0u;
                    const uint32 Id2 = R2 ? (uint32)R2->GetUniqueID() : 0u;
                    if (Id1 < Id2)
                    {
                        Best = Ctoi;
                        Best.TOI = T;
                    }
                }
            }
        }

        // ------------------------------------------------------------
        // 3) Integrate everyone up to TOI (or full remaining if no hit or Iter >= MaxIter)
        // ------------------------------------------------------------
        const bool bWillResolve = Best.bHit && Best.Resolver.IsValid() && Iter < MaxIter;

        float SubDelta = bWillResolve ? Best.TOI : Remaining;

        // clamp to avoid zero-step loop
        if (SubDelta < MinStep)
        {
            // If TOI ~ 0, advance a little, or force resolution without integrating (but this can explode)
            SubDelta = MinStep;
        }
        SubDelta = FMath::Clamp(SubDelta, 0.f, Remaining);
		LastSubDelta = SubDelta;

        // Advance all components to that time
        for (ISpeedComponent* Comp : ComponentsSorted)
        {
            if (!Comp) continue;
            Comp->IntegrateKinematics(SubDelta);
        }
        TimePassed += SubDelta;

        // No hit => End
        if (!bWillResolve)
            break;

        // ------------------------------------------------------------
        // 4) Resolve hit (single per pair)
        // ------------------------------------------------------------
        USSubBody* Resolver = Best.Resolver.Get();
        if (!Resolver)
            continue;

        // Anti-double resolve : if this pair has already been resolved in the frame, skip
        // and continue the loop (re-sweep on updated Remaining).
        if (ResolvedPairs.Contains(Best.PairKey))
            continue;

        // Handoff the chosen hit to the resolver (very important)
        // Resolver->SetFutureHit(Best.Hit);
        Resolver->AcceptHit();
        if (Resolver->ComponentHasBeenIgnored(*Resolver->GetHit().Component.Get()))
        {
            ResolvedPairs.Add(Best.PairKey);
        }
		USolidSubBody* RollingBodyA = Cast<USolidSubBody>(Resolver);
		USolidSubBody* RollingBodyB = Cast<USolidSubBody>(Resolver->GetHit().SubBody.Get());

		// Resolve at current substep time.
		Resolver->ResolveCurrentHit(SubDelta, SimTime);
		if (RollingBodyA && RollingBodyB)
		{
			ActivatePendingRollingContactPairAtTOI(
				*RollingBodyA, *RollingBodyB, Dt - TimePassed);
		}

        // Optional: post update at each substep (useful if certain gameplay sensors need to react "immediately")
        /*
        for (ISpeedComponent* Comp : ComponentsSorted)
        {
            if (!Comp) continue;
            Comp->PostPhysicsUpdate();
        }
        */
    }

    // Make persistent finite-mass contacts geometrically feasible before any
    // PostPhysics observer samples them.
	ProjectDynamicContactPairs();

    // ------------------------------------------------------------
    // 5) Final post update
    // ------------------------------------------------------------
    for (ISpeedComponent* Comp : ComponentsSorted)
    {
        if (!Comp) continue;
		Comp->PostPhysicsUpdate(Dt);
	}

	if (!PendingRollingContactPairs.IsEmpty())
	{
		for (FDynamicContactPair& PendingPair : PendingRollingContactPairs)
		{
			if (!DynamicContactPairs.ContainsByPredicate(
				[Key = PendingPair.PairKey](const FDynamicContactPair& ActivePair)
				{
					return ActivePair.PairKey == Key;
				}))
			{
				DynamicContactPairs.Add(MoveTemp(PendingPair));
			}
		}
		PendingRollingContactPairs.Reset();
	}

	SolveDynamicContactPairs(Dt);
	ProjectDynamicContactPairs();
}
