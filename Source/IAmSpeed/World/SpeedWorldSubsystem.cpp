// Fill out your copyright notice in the Description page of Project Settings.

// USpeedWorldSubsystem.cpp

#include "SpeedWorldSubsystem.h"
#include "IAmSpeed/Components/ISpeedComponent.h"
#include "IAmSpeed/SubBodies/Solid/BoxSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SolidSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SphereSubBody.h"
#include "GameFramework/Actor.h"
#include "Engine/World.h"
#include "Algo/Sort.h"
#include "HAL/IConsoleManager.h"

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
	TEXT("Maximum positive sphere/box gap retained by the experimental rolling manifold."));

static TAutoConsoleVariable<float> CVarIAmSpeedRollingPenetrationSlop(
	TEXT("p.IAmSpeed.Collision.RollingPenetrationSlopCm"),
	0.02f,
	TEXT("Penetration ignored by rolling-manifold velocity stabilization."));

static TAutoConsoleVariable<float> CVarIAmSpeedRollingBaumgarte(
	TEXT("p.IAmSpeed.Collision.RollingBaumgarte"),
	0.20f,
	TEXT("Fraction of rolling-manifold penetration corrected per simulation step."));

static TAutoConsoleVariable<float> CVarIAmSpeedRollingMaxCorrectionSpeed(
	TEXT("p.IAmSpeed.Collision.RollingMaxCorrectionSpeed"),
	50.0f,
	TEXT("Maximum separating speed requested by rolling-manifold penetration correction, in cm/s."));

static TAutoConsoleVariable<float> CVarIAmSpeedRollingFrictionScale(
	TEXT("p.IAmSpeed.Collision.RollingFrictionScale"),
	1.0f,
	TEXT("Scale applied to the existing mixed sphere/box friction in persistent rolling contact."));

static TAutoConsoleVariable<float> CVarIAmSpeedRollingReleaseSpeed(
	TEXT("p.IAmSpeed.Collision.RollingReleaseSpeedCmS"),
	1.0f,
	TEXT("Separating normal speed above which a zero-gap rolling pair releases."));

static TAutoConsoleVariable<float> CVarIAmSpeedRollingFakePhysicsRate(
	TEXT("p.IAmSpeed.Collision.RollingFakePhysicsRateHz"),
	60.0f,
	TEXT("Equivalent discrete fake-physics applications per second while a rolling manifold is supported."));

static TAutoConsoleVariable<float> CVarIAmSpeedRollingFakePhysicsScale(
	TEXT("p.IAmSpeed.Collision.RollingFakePhysicsScale"),
	1.0f,
	TEXT("Amplitude scale for the time-normalized fake-physics share of a rolling manifold."));

static TAutoConsoleVariable<float> CVarIAmSpeedRollingFakeTangentialVelocityScale(
	TEXT("p.IAmSpeed.Collision.RollingFakeTangentialVelocityScale"),
	1.0f,
	TEXT("Tangential relative-velocity fraction fed back into persistent fake physics."));

static TAutoConsoleVariable<int32> CVarIAmSpeedDebugPersistentDynamicPairs(
	TEXT("p.IAmSpeed.Collision.DebugPersistentDynamicPairs"),
	0,
	TEXT("Logs registration, rejection and impulses for persistent dynamic pairs."));

bool USpeedWorldSubsystem::AreUnilateralRollingPairsEnabled()
{
	return CVarIAmSpeedUnilateralRollingPairs.GetValueOnAnyThread() != 0;
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
	const FVector& NormalBToA)
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
			TEXT("[PersistentPair][Register] Frame=%u A=%s B=%s Normal=%s Point=%s"),
			CurrentStepFrame, *BodyA.GetName(), *BodyB.GetName(),
			*NormalBToA.ToString(), *ContactPoint.ToString());
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
			// Registration happens inside CCD resolution. Ownership may only change
			// at the following physics-frame boundary, never midway through a TOI
			// iteration that was built with the pair still collidable.
			return Pair.PairKey == PairKey &&
				Pair.bRollingManifoldReady &&
				Pair.BodyA.IsValid() && Pair.BodyB.IsValid();
		});
}

void USpeedWorldSubsystem::SolveDynamicContactPairs(const float Dt)
{
	constexpr unsigned int MaxUnseenFrames = 2;
	const float SeparationToleranceCm = FMath::Max(
		0.0f, CVarIAmSpeedRollingSeparationTolerance.GetValueOnAnyThread());

	for (int32 Index = DynamicContactPairs.Num() - 1; Index >= 0; --Index)
	{
		FDynamicContactPair& Pair = DynamicContactPairs[Index];
		USolidSubBody* BodyA = Pair.BodyA.Get();
		USolidSubBody* BodyB = Pair.BodyB.Get();
		ISpeedComponent* CompA = BodyA ? BodyA->GetParentComponent() : nullptr;
		ISpeedComponent* CompB = BodyB ? BodyB->GetParentComponent() : nullptr;
		if (!BodyA || !BodyB || !CompA || !CompB)
		{
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
			DynamicContactPairs.RemoveAtSwap(Index, 1, EAllowShrinking::No);
			continue;
		}

		// The CCD impact already resolved this pair on its first frame.
		if (CurrentStepFrame == Pair.FirstSeenFrame)
		{
			Pair.bRollingManifoldReady = bUseRollingManifold;
			continue;
		}

		const FVector CurrentCOMA = CompA->GetPhysCOM();
		const FVector CurrentCOMB = CompB->GetPhysCOM();
		const float MaxExpectedMoveA = CompA->GetPhysCOMVelocity().Size() * Dt * 2.0f + 10.0f;
		const float MaxExpectedMoveB = CompB->GetPhysCOMVelocity().Size() * Dt * 2.0f + 10.0f;
		if (FVector::DistSquared(CurrentCOMA, Pair.LastCOMA) > FMath::Square(MaxExpectedMoveA) ||
			FVector::DistSquared(CurrentCOMB, Pair.LastCOMB) > FMath::Square(MaxExpectedMoveB))
		{
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
				if (CVarIAmSpeedDebugPersistentDynamicPairs.GetValueOnAnyThread() != 0 &&
					FVector::DotProduct(PreviousNormal, N) < 0.995f)
				{
					UE_LOG(LogTemp, Warning,
						TEXT("[PersistentPair][FeatureTransition] Frame=%u Separation=%.3f PreviousNormal=%s CurrentNormal=%s"),
						CurrentStepFrame, ShapeSeparation,
						*PreviousNormal.ToString(), *N.ToString());
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
		if (Separation > SeparationToleranceCm && RelativeNormalVelocity >= 0.0f)
		{
			DynamicContactPairs.RemoveAtSwap(Index, 1, EAllowShrinking::No);
			continue;
		}

		const float PredictedRelativeNormalVelocity = RelativeNormalVelocity +
			Dt * FVector::DotProduct(
				CompA->GetPhysAccelerationAtPoint(AnchorA) -
				CompB->GetPhysAccelerationAtPoint(AnchorB), N);
		const float PenetrationDepth = FMath::Max(0.0f, -Separation);
		const float PenetrationSlop = FMath::Max(
			0.0f, CVarIAmSpeedRollingPenetrationSlop.GetValueOnAnyThread());
		const float TargetRelativeNormalVelocity = bUseRollingManifold &&
			PenetrationDepth > PenetrationSlop
			? FMath::Min(
				FMath::Max(0.0f, CVarIAmSpeedRollingMaxCorrectionSpeed.GetValueOnAnyThread()),
				FMath::Max(0.0f, CVarIAmSpeedRollingBaumgarte.GetValueOnAnyThread()) *
					(PenetrationDepth - PenetrationSlop) / Dt)
			: 0.0f;
		if (PredictedRelativeNormalVelocity >= TargetRelativeNormalVelocity)
		{
			if (bUseRollingManifold && Separation >= -PenetrationSlop &&
				PredictedRelativeNormalVelocity > FMath::Max(
					0.0f, CVarIAmSpeedRollingReleaseSpeed.GetValueOnAnyThread()))
			{
				DynamicContactPairs.RemoveAtSwap(Index, 1, EAllowShrinking::No);
				continue;
			}
			if (bUseRollingManifold &&
				CVarIAmSpeedDebugPersistentDynamicPairs.GetValueOnAnyThread() != 0)
			{
				UE_LOG(LogTemp, Warning,
					TEXT("[PersistentPair][SupportedNoImpulse] Frame=%u Separation=%.3f RelN=%.3f PredictedRelN=%.3f"),
					CurrentStepFrame, Separation, RelativeNormalVelocity,
					PredictedRelativeNormalVelocity);
			}
			continue;
		}

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

		const float NormalImpulseMagnitude =
			(TargetRelativeNormalVelocity - PredictedRelativeNormalVelocity) / Denominator;
		const FVector Impulse = NormalImpulseMagnitude * N;
		const FVector ContactPoint = 0.5f * (AnchorA + AnchorB);
		FFakePhysicsImpactContext RollingFakePhysicsContext;
		if (bUseRollingManifold)
		{
			const float RollingFakePhysicsRate = FMath::Max(
				0.0f, CVarIAmSpeedRollingFakePhysicsRate.GetValueOnAnyThread());
			RollingFakePhysicsContext.SelfParentKinematics = Sphere == BodyA
				? CompA->GetKinematicState()
				: CompB->GetKinematicState();
			RollingFakePhysicsContext.OtherParentKinematics = Sphere == BodyA
				? CompB->GetKinematicState()
				: CompA->GetKinematicState();
			RollingFakePhysicsContext.FakeImpulseScale = Dt * RollingFakePhysicsRate *
				FMath::Max(0.0f, CVarIAmSpeedRollingFakePhysicsScale.GetValueOnAnyThread());
			RollingFakePhysicsContext.bProjectFakeImpulseOntoContactPlane = true;
			RollingFakePhysicsContext.FakeTangentialVelocityScale = FMath::Max(
				0.0f,
				CVarIAmSpeedRollingFakeTangentialVelocityScale.GetValueOnAnyThread());
			RollingFakePhysicsContext.bUseContactPointRelativeVelocity = true;
			if (RollingFakePhysicsRate > KINDA_SMALL_NUMBER)
			{
				const FVector BoxToSphereNormal = Sphere == BodyA ? N : -N;
				const float IncidentRelativeNormalVelocity = FMath::Min(
					RelativeNormalVelocity, PredictedRelativeNormalVelocity);
				const float DiscreteIntervalScale = FMath::Max(
					1.0f, 1.0f / (RollingFakePhysicsRate * Dt));
				RollingFakePhysicsContext.FakeRelativeVelocityBias =
					BoxToSphereNormal * IncidentRelativeNormalVelocity *
					(DiscreteIntervalScale - 1.0f);
			}
		}
		if (CVarIAmSpeedDebugPersistentDynamicPairs.GetValueOnAnyThread() != 0)
		{
			UE_LOG(LogTemp, Warning,
				TEXT("[PersistentPair][Impulse] Frame=%u A=%s B=%s J=%s Separation=%.3f RelN=%.3f"),
				CurrentStepFrame, *BodyA->GetName(), *BodyB->GetName(),
				*Impulse.ToString(), Separation, RelativeNormalVelocity);
		}
		BodyA->ApplyImpulse(Impulse, ContactPoint);
		BodyB->ApplyImpulse(-Impulse, ContactPoint);

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
					const float MixedFriction = USolidSubBody::ResolveSphereBoxFriction(
						*Sphere,
						*Box,
						USolidSubBody::MixFriction(
							Sphere->GetStaticFriction(), Box->GetStaticFriction(), EMixMode::E_Max));
					const float MaxFrictionImpulse = NormalImpulseMagnitude * MixedFriction *
						FMath::Max(0.0f, CVarIAmSpeedRollingFrictionScale.GetValueOnAnyThread());
					const float TangentialImpulseMagnitude = FMath::Min(
						TangentialSpeed / TangentialDenominator, MaxFrictionImpulse);
					const FVector TangentialImpulse = -TangentialImpulseMagnitude * Tangent;
					BodyA->ApplyImpulse(TangentialImpulse, ContactPoint);
					BodyB->ApplyImpulse(-TangentialImpulse, ContactPoint);
				}
			}

			// RL repeatedly resolves a supported ball/car contact and therefore also
			// repeatedly contributes its fake shot response. A CCD manifold owns the
			// pair instead, so apply the same response as a time-normalized rate.
			if (RollingFakePhysicsContext.FakeImpulseScale > 0.0f)
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

        // Resolve at current substep time
        Resolver->ResolveCurrentHit(SubDelta, SimTime);

        // Optional: post update at each substep (useful if certain gameplay sensors need to react "immediately")
        /*
        for (ISpeedComponent* Comp : ComponentsSorted)
        {
            if (!Comp) continue;
            Comp->PostPhysicsUpdate();
        }
        */
    }

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
}
