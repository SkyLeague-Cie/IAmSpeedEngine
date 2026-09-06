#include "ISpeedComponent.h"
#include "IAmSpeed/SubBodies/Solid/SolidSubBody.h"
#include "IAmSpeed/World/Analytic/AnalyticWorldData.h"
#include "Misc/ScopeExit.h"
#include <cmath>

namespace
{
	constexpr uint8 MaxConstraintPersistenceFrames = 1;
	constexpr float ConstraintNormalDotThreshold = 0.995f;
	constexpr float ConstraintPointThresholdCm = 10.0f;
	constexpr int32 ConstraintProjectionPasses = 4;
	constexpr float ConstraintPersistSignedDistCm = 5.0f;
	constexpr float ConstraintPersistSeparatingSpeed = 50.0f;

	// A call-local view shares the origin formulas without reconstructing fields
	// that a single-field getter will discard. No state is cached across calls.
	struct FOriginKinematicView
	{
		const SKinematic& COM;
		const FVector OriginToCOM;
		FOriginKinematicView(const SKinematic& State, const FVector& LocalCOM)
			: COM(State), OriginToCOM(State.Rotation.RotateVector(LocalCOM)) {}
		FVector Location() const { return COM.Location - OriginToCOM; }
		FVector Velocity() const { return COM.Velocity - FVector::CrossProduct(COM.AngularVelocity, OriginToCOM); }
		FVector Acceleration() const
		{
			return COM.Acceleration - FVector::CrossProduct(COM.AngularAcceleration, OriginToCOM)
				- FVector::CrossProduct(COM.AngularVelocity, FVector::CrossProduct(COM.AngularVelocity, OriginToCOM));
		}
	};
}

void ISpeedComponent::RestoreSimulationSnapshot(
	const SKinematic& KinematicState,
	const bool bFrozen,
	const TConstArrayView<uint8> MechanicPayload)
{
	check(CanRestoreSimulationSnapshot(MechanicPayload));
	SleepState.Reset();
	SetStaticCollisionWorldForFrame(nullptr);
	KinematicQuantizationCache.Reset();
	SetKinematicState(KinematicState);
	SetIsFrozen(bFrozen);
	RestoreSimulationSnapshotPayload(MechanicPayload);
	UpdateSubBodiesKinematics();
}

void ISpeedComponent::QuantizeKinematicState(SKinematic& State, const FQuat& PreviousRotation)
{
	// IAMSPEED.PHYS.BOX_STANDING_STILL/BOX_EQUILIBRIUM.V1: independent world
	// and actor grids need not intersect at a valid contact pose. Preserve the
	// pose of an ALREADY geometrically certified, non-rotating face (including
	// its material's admissible tangent slide), never round motion into rest.
	bool bPreserveSupportedPose = HasExactStaticFaceSupport();
	if (!bPreserveSupportedPose && StaticRestingWorld)
	{
		for (const USSubBody* Body : GetSubBodies())
			if (Body && Body->RequiresUnquantizedContactPose()) { bPreserveSupportedPose = true; break; }
	}
	const FVector SupportedLocation = State.Location;
	const FQuat SupportedRotation = State.Rotation;
	const FVector SupportedVelocity = State.Velocity;
	const FVector SupportedAngularVelocity = State.AngularVelocity;
	KinematicQuantizationCache.Quantize(State, PreviousRotation);
	bool bRejectedQuantizedPose = false;
	if (!bPreserveSupportedPose && StaticRestingWorld &&
		(State.Location != SupportedLocation || State.Rotation != SupportedRotation))
	{
		for (const USSubBody* Body : GetSubBodies())
			if (Body && !Body->CanApplyQuantizedPose(*StaticRestingWorld, State))
			{
				bRejectedQuantizedPose = true;
				break;
			}
	}
	// An approaching body may have no contact yet. Rounding must not cross the
	// surface before CCD reaches it; retain its integrated exterior pose instead.
	if (bPreserveSupportedPose || bRejectedQuantizedPose)
	{
		State.Location = SupportedLocation;
		State.Rotation = SupportedRotation;
	}
	if (bPreserveSupportedPose)
	{
		// Edge support couples normal COM velocity and angular velocity. Rounding
		// them independently breaks the constraint even if pose precision is kept.
		State.Velocity = SupportedVelocity;
		State.AngularVelocity = SupportedAngularVelocity;
	}
}

void ISpeedComponent:: ApplyImpulse(const FVector& LinearImpulse, const FVector& WorldPoint, const USolidSubBody* SubBody)
{
	AddPhysImpulseAtPoint(LinearImpulse, WorldPoint, SubBody);
}

void ISpeedComponent::IntegrateKinematicsPrv(const float& SubDelta)
{
    // BasePhysicsState is the rigid body's COM state. No origin compensation
    // belongs in the integrator; origin data is derived at the geometry boundary.
    SetKinematicState(GetKinematicState().Integrate(SubDelta));
}

bool ISpeedComponent::IsPhysicsSleeping() const
{
	return !IsFrozen() && SleepState.CanSleep(GetKinematicState(), HasStableSleepSupport());
}

bool ISpeedComponent::HasExactStaticRestingSupport() const
{
	return GetKinematicState().Velocity.IsZero() && HasExactStaticFaceSupport();
}

bool ISpeedComponent::HasExactStaticFaceSupport() const
{
	if (!StaticRestingWorld || IsFrozen()) return false;
	const SKinematic& State = GetKinematicState();
	if (!State.AngularVelocity.IsZero() || !State.AngularAcceleration.IsZero()) return false;
	const FVector Load = GetNominalGravityAcceleration();
	if (Load.IsZero() || FVector::DotProduct(State.Velocity, Load) != 0) return false;
	for (const USSubBody* Body : GetSubBodies())
		if (Body && Body->HasExactStaticRestingSupport(*StaticRestingWorld, Load)) return true;
	return false;
}

float ISpeedComponent::GetMaximumCanonicalSupportInterval(float Remaining) const
{
	if (!StaticRestingWorld || IsFrozen()) return Remaining;
	double Candidate = TNumericLimits<double>::Max();
	for (const USSubBody* Body : GetSubBodies())
		if (Body) Candidate = FMath::Min(Candidate, Body->GetStaticSupportStopTimeCandidate());
	if (Candidate >= Remaining) return Remaining;
	double CertifiedTime = TNumericLimits<double>::Max();
	if (GetStaticRestingReaction(&CertifiedTime).IsZero() || CertifiedTime <= 0 || CertifiedTime >= Remaining) return Remaining;
	float Interval = static_cast<float>(CertifiedTime);
	if (Interval < CertifiedTime) Interval = std::nextafter(Interval, FLT_MAX);
	return FMath::Min(Interval, Remaining);
}

FVector ISpeedComponent::GetStaticRestingReaction(double* StopAfterSeconds) const
{
	if (StopAfterSeconds) *StopAfterSeconds = TNumericLimits<double>::Max();
#if !UE_BUILD_SHIPPING
	const auto Observe = [this](EStaticRestingReactionStatus Status)
	{
		if (bObserveStaticRestingReaction) ObserveStaticRestingReaction(Status);
	};
	if (!StaticRestingWorld) { Observe(EStaticRestingReactionStatus::NoWorld); return FVector::ZeroVector; }
#else
	if (!StaticRestingWorld) return FVector::ZeroVector;
#endif
	const SKinematic& State = GetKinematicState();
	// Shape-independent rejection is cheap in the ordinary moving-body path.
	// No closing/separating motion or spin is accepted. Each shape must also
	// certify its material law before admitting tangential motion; no damping
	// or sleep acquisition is performed here.
	if (IsFrozen() || FVector::DotProduct(State.Velocity, State.Acceleration) != 0 || !State.AngularVelocity.IsZero() ||
		!State.AngularAcceleration.IsZero() || State.Acceleration.IsZero())
	{
#if !UE_BUILD_SHIPPING
		Observe(IsFrozen() ? EStaticRestingReactionStatus::Frozen :
			!State.Velocity.IsZero() ? EStaticRestingReactionStatus::Moving :
			!State.AngularVelocity.IsZero() ? EStaticRestingReactionStatus::Spinning :
			!State.AngularAcceleration.IsZero() ? EStaticRestingReactionStatus::AngularLoad :
			EStaticRestingReactionStatus::NoLoad);
#endif
		return FVector::ZeroVector;
	}
	for (const USSubBody* SubBody : GetSubBodies())
	{
		if (SubBody)
		{
			const FVector Reaction = SubBody->GetStaticRestingReaction(*StaticRestingWorld, StopAfterSeconds);
			if (!Reaction.IsZero())
			{
#if !UE_BUILD_SHIPPING
				Observe(EStaticRestingReactionStatus::Supported);
#endif
				return Reaction;
			}
		}
	}
#if !UE_BUILD_SHIPPING
	Observe(EStaticRestingReactionStatus::NoSupport);
#endif
	return FVector::ZeroVector;
}

bool ISpeedComponent::IsSpeedLimitPreservedByExactSupport(float Delta) const
{
	if (!StaticRestingWorld || Delta <= 0 || StaticSupportFrameHorizon < Delta) return false;
	const SKinematic& State = GetKinematicState();
	const double MaxSpeedSquared = FMath::Square(double(GetPhysMaxSpeed()));
	// Squaring a normalized/clamped oblique vector can exceed the squared cap
	// by one ULP (2300 at 40 degrees is a regression). Bound only double
	// arithmetic in the norm comparison; this never changes the velocity.
	const double NormRoundoff = 8 * DBL_EPSILON * FMath::Max(1.0, MaxSpeedSquared);
	if (State.Velocity.SizeSquared() > MaxSpeedSquared + NormRoundoff) return false;
	double StopAfter = TNumericLimits<double>::Max();
	const FVector Reaction = GetStaticRestingReaction(&StopAfter);
	if (Reaction.IsZero()) return false;
	// The admitted law has a straight, constant-acceleration trajectory up to
	// its Coulomb stop, then rest. Its speed norm is convex on that interval,
	// so endpoints suffice. A boundary/hole or incompatible load rejects the
	// support certificate and retains ordinary unconstrained speed limiting.
	const FVector EndVelocity = State.Velocity + (State.Acceleration + Reaction) *
		FMath::Min(double(Delta), StopAfter);
	return EndVelocity.SizeSquared() <= MaxSpeedSquared + NormRoundoff;
}

void ISpeedComponent::GetStaticContactAcceleration(FVector& Linear, FVector& Angular) const
{
	Linear = Angular = FVector::ZeroVector;
	if (!StaticRestingWorld || IsFrozen()) return;
	for (const USSubBody* Body : GetSubBodies())
		if (Body && Body->TryGetStaticContactAcceleration(Linear, Angular)) return;
}

void ISpeedComponent::IntegrateKinematics(const float& SubDelta)
{
	if (IsFrozen())
		return;

	if (SubDelta <= 0.f)
		return;

	// Apply only during this segment. Restoring the external load afterwards
	// lets a later TOI impulse/torque release support immediately in this frame.
	double StopAfterSeconds = TNumericLimits<double>::Max();
	FVector RestingReaction = GetStaticRestingReaction(&StopAfterSeconds);
	FVector AngularReaction = FVector::ZeroVector;
	if (RestingReaction.IsZero()) GetStaticContactAcceleration(RestingReaction, AngularReaction);
	if (!RestingReaction.IsZero()) SetPhysAcceleration(GetPhysAcceleration() + RestingReaction);
	if (!AngularReaction.IsZero()) SetPhysAngularAcceleration(GetPhysAngularAcceleration() + AngularReaction);
	ON_SCOPE_EXIT
	{
		if (!RestingReaction.IsZero() || !AngularReaction.IsZero())
		{
			SetPhysAcceleration(GetPhysAcceleration() - RestingReaction);
			SetPhysAngularAcceleration(GetPhysAngularAcceleration() - AngularReaction);
			UpdateSubBodiesKinematics();
		}
	};

	if (!RestingReaction.IsZero() && StopAfterSeconds <= SubDelta)
	{
		// Integrate the exact Coulomb stop event. The remainder of this float
		// interval is rest, not reversed slip; no velocity threshold or sleep
		// is involved. The world split CCD at this event before reaching here.
		SKinematic State = GetKinematicState();
		State.Location += State.Velocity * (0.5 * StopAfterSeconds);
		State.Velocity = FVector::ZeroVector;
		SetKinematicState(State);
		UpdateSubBodiesKinematics();
		ProjectEstablishedStaticContacts(SubDelta);
		PostIntegrateKinematics(SubDelta);
		return;
	}

	if (IsPhysicsSleeping())
	{
#if !UE_BUILD_SHIPPING
		++Speed::FSimulationSleepState::ThreadIntegrationSkips();
#endif
		// Keep support bookkeeping and all collision detection alive. Only the
		// unchanged rigid pose and its redundant transport/projection are skipped.
		PostIntegrateKinematics(SubDelta);
		return;
	}

	bool bRequiresContactTransport = false;
	for (const USSubBody* SubBody : GetSubBodies())
	{
		if (SubBody && SubBody->RequiresEstablishedStaticContactTransport())
		{
			bRequiresContactTransport = true;
			break;
		}
	}

	// A curved established contact cannot be integrated as one straight chord
	// followed by a depenetration at its end: that freezes one tangent normal
	// over the whole interval. Transport it through a bounded deterministic set
	// of stages instead. Each stage samples the analytical provider again, so
	// pose feasibility and the unilateral velocity reaction follow the changing
	// surface normal during this very SubDelta.
	constexpr float ContactTransportStepSeconds = 1.0f / 1200.0f;
	constexpr int32 MaximumContactTransportSteps = 4;
	const int32 TransportSteps = bRequiresContactTransport
		? FMath::Clamp(
			FMath::CeilToInt(SubDelta / ContactTransportStepSeconds),
			1, MaximumContactTransportSteps)
		: 1;
	const float TransportDelta = SubDelta / static_cast<float>(TransportSteps);
	for (int32 Step = 0; Step < TransportSteps; ++Step)
	{
		IntegrateKinematicsPrv(TransportDelta);

		// Enforce limits directly on the canonical COM state.
		SetPhysCOMVelocity(GetPhysCOMVelocity());
		SetPhysAngularVelocity(GetPhysAngularVelocity());

		UpdateSubBodiesKinematics();
		ProjectEstablishedStaticContacts(TransportDelta);
	}

	// update physics state after integrating kinematics to the time of impact (e.g update suspension traces for wheels, update hit info for hitboxes, etc.)
	PostIntegrateKinematics(SubDelta);
    // UE_LOG(LogTemp, Log, TEXT("[IntegrateKinematics] NumFrame = %d, SubDelta = %f. Kinematics = %s"), NumFrame(), SubDelta, *GetKinematicState().ToString());
}

bool ISpeedComponent::ProjectEstablishedStaticContacts(const float& Delta)
{
	bool bProjected = false;
	for (USSubBody* SubBody : GetSubBodies())
	{
		if (SubBody && SubBody->ProjectEstablishedStaticContact(Delta))
		{
			bProjected = true;
		}
	}
	if (bProjected)
	{
		UpdateSubBodiesKinematics();
	}
	return bProjected;
}

void ISpeedComponent::UpdateSubBodiesKinematics()
{
    // Every child observes the same parent state; derive origin from COM once.
    const SKinematic OriginState = GetOriginKinematicState();
    for (USSubBody* SubBody : GetSubBodies())
    {
        SubBody->UpdateKinematicsFromOwner(OriginState);
    }
}

SKinematic ISpeedComponent::GetOriginKinematicState() const
{
	return GetOriginKinematicStateForFrame(NumFrame());
}

SKinematic ISpeedComponent::GetOriginKinematicStateForFrame(const unsigned int& NumFrameToRead) const
{
	const SKinematic& COMState = GetKinematicStateForFrame(NumFrameToRead);
	SKinematic OriginState = COMState;
	const FOriginKinematicView Origin(COMState, GetPhysCenterOfMassLocal());
	OriginState.Location = Origin.Location();
	OriginState.Velocity = Origin.Velocity();
	OriginState.Acceleration = Origin.Acceleration();
	return OriginState;
}

void ISpeedComponent::ResetForFrame(const float& Delta)
{
	for (int32 ConstraintIdx = PhysicalConstraints.Num() - 1; ConstraintIdx >= 0; --ConstraintIdx)
	{
		FPhysicalContactConstraint& Constraint = PhysicalConstraints[ConstraintIdx];

		if (!Constraint.bPersistent)
		{
			PhysicalConstraints.RemoveAtSwap(ConstraintIdx);
			continue;
		}

		Constraint.FramesSinceSeen++;
		if (Constraint.FramesSinceSeen > MaxConstraintPersistenceFrames ||
			!IsPhysicalConstraintStillRelevant(Constraint))
		{
			PhysicalConstraints.RemoveAtSwap(ConstraintIdx);
		}
	}

	for (USSubBody* SubBody : GetSubBodies())
	{
		SubBody->ResetForFrame(Delta);
	}
}

SComponentTOI ISpeedComponent::SweepTOISubBodies(const float& RemainingDelta, const float& LastSubDelta)
{
	// CCD predicts from the same constrained acceleration as integration, but
	// no reaction is cached across a collision, teleport or force change.
	FVector RestingReaction = GetStaticRestingReaction();
	FVector AngularReaction = FVector::ZeroVector;
	if (RestingReaction.IsZero()) GetStaticContactAcceleration(RestingReaction, AngularReaction);
	if (!RestingReaction.IsZero() || !AngularReaction.IsZero())
	{
		SetPhysAcceleration(GetPhysAcceleration() + RestingReaction);
		SetPhysAngularAcceleration(GetPhysAngularAcceleration() + AngularReaction);
		UpdateSubBodiesKinematics();
	}
	ON_SCOPE_EXIT
	{
		if (!RestingReaction.IsZero() || !AngularReaction.IsZero())
		{
			SetPhysAcceleration(GetPhysAcceleration() - RestingReaction);
			SetPhysAngularAcceleration(GetPhysAngularAcceleration() - AngularReaction);
			UpdateSubBodiesKinematics();
		}
	};
    SComponentTOI Best;
    Best.bHit = false;
    Best.TOI = RemainingDelta;

    const TArray<USSubBody*>& InSubBodies = GetSubBodies();

    for (USSubBody* Sweeper : InSubBodies)
    {
        if (!Sweeper) continue;

        float TOI = RemainingDelta;
        constexpr float MinSubDelta = 0.0f;
        // if (LastSubDelta > MinSubDelta)
        {
            if (!Sweeper->SweepTOI(RemainingDelta, TOI))
                continue;
        }
        /*else if (Sweeper->WillHit())
        {
			TOI = Sweeper->GetFutureTOI();
        }*/

        if (TOI >= Best.TOI)
            continue;

        const SHitResult& H = Sweeper->GetFutureHit();
        USSubBody* OtherSB = H.SubBody.Get();
        USSubBody* Resolver = USSubBody::PickResolver(Sweeper, OtherSB);

        SHitResult HR = H;

        if (Resolver && Resolver != Sweeper)
        {
            HR.ImpactNormal *= -1.f;
        }

        Best.bHit = true;
        Best.TOI = TOI;
        Best.Resolver = Resolver;
        Best.Hit = HR;
        Best.PairKey = USSubBody::MakePairKey(
            Resolver ? Resolver : Sweeper, HR.Component.Get());
        // One static component may own several adjacent analytical providers.
        // Suppress repeated event resolution only within the same immutable
        // provider; a genuine seam transition must remain independently visible.
        const uint64 ProviderId = HR.CanonicalGroupId != 0
            ? HR.CanonicalGroupId : HR.SurfaceId;
        if (ProviderId != 0)
        {
            Best.PairKey = Speed::Analytic::CombineStableIds(
                Best.PairKey, ProviderId);
        }
        if (StaticRestingWorld && HR.SurfaceId != 0 && !HR.bSurfaceNormalMayVary &&
            HR.ContactFeatureThis == Speed::EContactFeatureKind::Vertex && HR.ContactFeatureIndexThis >= 0)
        {
            // A newly arriving vertex is a new event on the same plane, not
            // a repeated resolution of its already-established old contact.
            Best.PairKey = Speed::Analytic::CombineStableIds(Best.PairKey,
                0x504c414e00000000ull | static_cast<uint64>(HR.ContactFeatureIndexThis + 1));
        }
    }

    return Best;
}

void ISpeedComponent::AddExternalSubBodies(const TArray<USSubBody*>& ExtSubBodies)
{
    const TArray<USSubBody*>& InSubBodies = GetSubBodies();
    for (USSubBody* SubBody : InSubBodies)
    {
        if (SubBody)
        {
            SubBody->AddExternalSubBodies(ExtSubBodies);
        }
    }
}

void ISpeedComponent::RemoveExternalSubBodies(const TArray<USSubBody*>& ExtSubBodies)
{
    const TArray<USSubBody*>& InSubBodies = GetSubBodies();
    for (USSubBody* SubBody : InSubBodies)
    {
        if (SubBody)
        {
            SubBody->RemoveExternalSubBodies(ExtSubBodies);
        }
    }
}

void ISpeedComponent::PostPhysicsUpdate(const float& delta)
{
	ON_SCOPE_EXIT { SetStaticCollisionWorldForFrame(nullptr); };
	// reset accelerations after physics update so that they can be set again during the next tick
	SetPhysAcceleration(FVector::ZeroVector);
	SetPhysAngularAcceleration(FVector::ZeroVector);

	const TArray<USSubBody*>& InSubBodies = GetSubBodies();
    for (USSubBody* SubBody : InSubBodies)
    {
        if (SubBody)
        {
            SubBody->PostPhysicsUpdate();
        }
	}

    PostPhysicsUpdatePrv(delta);
	SleepState.ObserveCompletedPose(GetKinematicState());
}

// ===================== Kinematics getters and setters =================
FVector ISpeedComponent::GetPhysLocation() const
{
	// Keep the historical-frame selection used by the complete origin getter.
	const SKinematic& COMState = GetKinematicStateForFrame(NumFrame());
	return FOriginKinematicView(COMState, GetPhysCenterOfMassLocal()).Location();
}

void ISpeedComponent::SetPhysCOMLocation(const FVector& NewCOMLocation)
{
	SKinematic K = GetKinematicState();
	K.Location = NewCOMLocation;
	SetKinematicState(K);
}

void ISpeedComponent::SetPhysLocation(const FVector& NewLocation)
{
	SetPhysCOMLocation(NewLocation + GetPhysRotation().RotateVector(GetPhysCenterOfMassLocal()));
}

const FQuat& ISpeedComponent::GetPhysRotation() const
{
    return GetKinematicState().Rotation;
}

void ISpeedComponent::SetPhysRotation(const FQuat& NewRotation)
{
    SKinematic K = GetKinematicState();
    K.Rotation = NewRotation;
	SetKinematicState(K);
}

void ISpeedComponent::SetPhysRotationPreserveCOM(const FQuat& NewRotation)
{
	// Rotation is stored at the COM, so it already preserves both COM position
	// and COM velocity. The origin is derived after the rotation changes.
	SetPhysRotation(NewRotation);
}

FVector ISpeedComponent::GetPhysVelocity() const
{
	const SKinematic& COMState = GetKinematicStateForFrame(NumFrame());
	return FOriginKinematicView(COMState, GetPhysCenterOfMassLocal()).Velocity();
}

void ISpeedComponent::SetPhysVelocity(const FVector& NewVelocity)
{
	if (IsFrozen())
	{
		return;
	}

	const FVector OriginToCOM = GetPhysRotation().RotateVector(GetPhysCenterOfMassLocal());
	SetPhysCOMVelocity(NewVelocity + FVector::CrossProduct(GetPhysAngularVelocity(), OriginToCOM));
}

void ISpeedComponent::SetPhysCOMVelocity(const FVector& NewCOMVelocity)
{
	if (IsFrozen())
	{
		return;
	}

	const FVector ClampedCOMVelocity = NewCOMVelocity.GetClampedToMaxSize(GetPhysMaxSpeed());
	SetPhysCOMVelocityRaw(ClampedCOMVelocity);
}

FVector ISpeedComponent::GetPhysVelocityAtPoint(const FVector& Point) const
{
    // v + w x r
    const FVector R = Point - GetPhysCOM();
    return GetPhysCOMVelocity() + FVector::CrossProduct(GetPhysAngularVelocity(), R);
}

const FVector& ISpeedComponent::GetPhysCOMVelocity() const
{
	return GetKinematicState().Velocity;
}

const FVector& ISpeedComponent::GetPhysAngularVelocity() const
{
	return GetKinematicState().AngularVelocity;
}

void ISpeedComponent::SetPhysAngularVelocity(const FVector& NewAngularVelocity)
{
	if (IsFrozen())
	{
		return;
	}
	SetPhysAngularVelocityRaw(NewAngularVelocity.GetClampedToMaxSize(GetPhysMaxAngularSpeed()));
}

void ISpeedComponent::SetPhysVelocityAtPoint(const FVector& NewVelocity, const FVector& Point)
{
    // v + w x r
    const FVector R = Point - GetPhysCOM();
    const FVector NewAngularVelocity = FVector::CrossProduct(R, NewVelocity - GetPhysCOMVelocity()).GetSafeNormal() * GetPhysAngularVelocity().Size();
    SetPhysAngularVelocity(NewAngularVelocity);
    SetPhysCOMVelocity(NewVelocity - FVector::CrossProduct(NewAngularVelocity, R));
}

const FVector& ISpeedComponent::GetPhysAcceleration() const
{
    return GetKinematicState().Acceleration;
}

const FVector& ISpeedComponent::GetPhysAngularAcceleration() const
{
	return GetKinematicState().AngularAcceleration;
}

void ISpeedComponent::SetPhysAcceleration(const FVector& NewAcceleration)
{
	if (IsFrozen())
	{
		return;
	}

    SKinematic K = GetKinematicState();
    K.Acceleration = NewAcceleration;
	SetKinematicState(K);
}

void ISpeedComponent::SetPhysAngularAcceleration(const FVector& NewAngularAcceleration)
{
	if (IsFrozen())
	{
		return;
	}

    SKinematic K = GetKinematicState();
    K.AngularAcceleration = NewAngularAcceleration;
	SetKinematicState(K);
}

FVector ISpeedComponent::GetPhysAccelerationAtPoint(const FVector& Point) const
{
    // a + alpha x r + w x (w x r)
    const FVector R = Point - GetPhysCOM();
    return GetPhysAcceleration() + FVector::CrossProduct(GetPhysAngularAcceleration(), R) + FVector::CrossProduct(GetPhysAngularVelocity(), FVector::CrossProduct(GetPhysAngularVelocity(), R));
}

void ISpeedComponent::AddPhysVelocity(const FVector& DeltaVelocity)
{
	FVector ProjectedDeltaVelocity = DeltaVelocity;
	ProjectLinearDeltaAgainstConstraints(ProjectedDeltaVelocity);
	SetPhysCOMVelocity(GetPhysCOMVelocity() + ProjectedDeltaVelocity);
}

FVector ISpeedComponent::AddPhysLocation(const FVector& DeltaLocation)
{
	FVector ProjectedDeltaLocation = DeltaLocation;
	ProjectLinearDeltaAgainstConstraints(ProjectedDeltaLocation);
	if (!ProjectedDeltaLocation.IsNearlyZero())
	{
		SetPhysCOMLocation(GetPhysCOM() + ProjectedDeltaLocation);
	}
	return ProjectedDeltaLocation;
}

FQuat ISpeedComponent::AddPhysRotation(const FQuat& DeltaRotation)
{
	FQuat NormalizedDeltaRotation = DeltaRotation.GetNormalized();
	if (NormalizedDeltaRotation.Equals(FQuat::Identity))
	{
		return FQuat::Identity;
	}

	FVector Axis = FVector::ZeroVector;
	float Angle = 0.f;
	NormalizedDeltaRotation.ToAxisAndAngle(Axis, Angle);
	if (!Axis.Normalize())
	{
		return FQuat::Identity;
	}

	if (Angle > PI)
	{
		Angle -= 2.f * PI;
	}

	FVector DeltaAngular = Axis * Angle;
	ProjectAngularDeltaAgainstConstraints(DeltaAngular);

	const float ProjectedAngle = DeltaAngular.Size();
	if (ProjectedAngle <= KINDA_SMALL_NUMBER)
	{
		return FQuat::Identity;
	}

	const FQuat AppliedDeltaRotation(DeltaAngular / ProjectedAngle, ProjectedAngle);
	SetPhysRotation((GetPhysRotation() * AppliedDeltaRotation).GetNormalized());
	return AppliedDeltaRotation;
}
void ISpeedComponent::AddPhysAngularVelocity(const FVector& DeltaAngularVelocity)
{
	FVector ProjectedDeltaAngularVelocity = DeltaAngularVelocity;
	ProjectAngularDeltaAgainstConstraints(ProjectedDeltaAngularVelocity);
	SetPhysAngularVelocity(GetPhysAngularVelocity() + ProjectedDeltaAngularVelocity);
}

void ISpeedComponent::AddPhysAcceleration(const FVector& DeltaAcceleration)
{
	FVector ProjectedDeltaAcceleration = DeltaAcceleration;
	ProjectLinearDeltaAgainstConstraints(ProjectedDeltaAcceleration);
    SetPhysAcceleration(GetPhysAcceleration() + ProjectedDeltaAcceleration);
}

void ISpeedComponent::AddPhysAngularAcceleration(const FVector& DeltaAngularAcceleration)
{
	FVector ProjectedDeltaAngularAcceleration = DeltaAngularAcceleration;
	ProjectAngularDeltaAgainstConstraints(ProjectedDeltaAngularAcceleration);
    SetPhysAngularAcceleration(GetPhysAngularAcceleration() + ProjectedDeltaAngularAcceleration);
}

void ISpeedComponent::AddPhysAngularAccelerationLocal(const FVector& LocalAngularAccel)
{
    // alpha_world = R * alpha_local
    const FQuat Rot = GetPhysRotation();
    const FVector DeltaAngularAccelWorld = Rot.RotateVector(LocalAngularAccel);
	AddPhysAngularAcceleration(DeltaAngularAccelWorld);
}

void ISpeedComponent::AddPhysImpulseAtPoint(const FVector& Impulse, const FVector& Point, const USolidSubBody* SubBody)
{
    // delta v = impulse / mass
    float PhysMass = GetPhysMass() > 0.f ? GetPhysMass() : 1.f; // avoid divide by zero
    FVector DeltaV = Impulse / PhysMass;
    // delta w = I^-1 * (r x impulse)
	const FVector COM = GetPhysCOM();
    const FVector R = Point - COM;
	const FMatrix WorldInvInertiaTensor = SubBody? SubBody->ComputeWorldInvInertiaTensor() : ComputeWorldInvInertiaTensor();
    FVector DeltaW = WorldInvInertiaTensor.TransformVector(FVector::CrossProduct(R, Impulse));

	ProjectPointDeltaAgainstConstraints(DeltaV, DeltaW);
	const FVector NewCOMVelocity = GetPhysCOMVelocity() + DeltaV;
	SetPhysAngularVelocityRaw((GetPhysAngularVelocity() + DeltaW).GetClampedToMaxSize(GetPhysMaxAngularSpeed()));
	SetPhysCOMVelocity(NewCOMVelocity);
}

void ISpeedComponent::AddPhysImpulseBatchAtPoints(
	const TArray<FVector>& Impulses, const TArray<FVector>& WorldPoints,
	const USolidSubBody* SubBody)
{
	if (Impulses.Num() == 0 || Impulses.Num() != WorldPoints.Num())
	{
		return;
	}

	const float PhysMass = GetPhysMass() > 0.f ? GetPhysMass() : 1.f;
	const FVector COM = GetPhysCOM();
	const FMatrix WorldInvInertiaTensor = SubBody
		? SubBody->ComputeWorldInvInertiaTensor() : ComputeWorldInvInertiaTensor();
	FVector DeltaV = FVector::ZeroVector;
	FVector DeltaW = FVector::ZeroVector;
	for (int32 Index = 0; Index < Impulses.Num(); ++Index)
	{
		DeltaV += Impulses[Index] / PhysMass;
		DeltaW += WorldInvInertiaTensor.TransformVector(FVector::CrossProduct(
			WorldPoints[Index] - COM, Impulses[Index]));
	}

	// The complete point-impulse set is one physical transaction. Projecting
	// each wheel separately makes the result depend on wheel iteration order.
	ProjectPointDeltaAgainstConstraints(DeltaV, DeltaW);
	SetPhysAngularVelocityRaw((GetPhysAngularVelocity() + DeltaW)
		.GetClampedToMaxSize(GetPhysMaxAngularSpeed()));
	SetPhysCOMVelocity(GetPhysCOMVelocity() + DeltaV);
}

void ISpeedComponent::AddPhysForceAtPoint(const FVector& Force, const FVector& WorldPoint, const USolidSubBody* SubBody)
{
    // F = m * a => a = F / m
    float PhysMass = GetPhysMass() > 0.f ? GetPhysMass() : 1.f; // avoid divide by zero
    FVector DeltaA = Force / PhysMass;
    // alpha = I^-1 * (r x F)
	const FVector COM = GetPhysCOM();
    const FVector R = WorldPoint - COM;
	const FMatrix WorldInvInertiaTensor = SubBody? SubBody->ComputeWorldInvInertiaTensor() : ComputeWorldInvInertiaTensor();
    FVector DeltaAlpha = WorldInvInertiaTensor.TransformVector(FVector::CrossProduct(R, Force));

	ProjectPointDeltaAgainstConstraints(DeltaA, DeltaAlpha);
    SetPhysAcceleration(GetPhysAcceleration() + DeltaA);
	SetPhysAngularAcceleration(GetPhysAngularAcceleration() + DeltaAlpha);
}

void ISpeedComponent::AddPhysForceBatchAtPoints(
	const TArray<FVector>& Forces, const TArray<FVector>& WorldPoints,
	const USolidSubBody* SubBody)
{
	if (Forces.Num() == 0 || Forces.Num() != WorldPoints.Num())
	{
		return;
	}

	const float PhysMass = GetPhysMass() > 0.f ? GetPhysMass() : 1.f;
	const FVector COM = GetPhysCOM();
	const FMatrix WorldInvInertiaTensor = SubBody
		? SubBody->ComputeWorldInvInertiaTensor() : ComputeWorldInvInertiaTensor();
	FVector DeltaA = FVector::ZeroVector;
	FVector DeltaAlpha = FVector::ZeroVector;
	for (int32 Index = 0; Index < Forces.Num(); ++Index)
	{
		DeltaA += Forces[Index] / PhysMass;
		DeltaAlpha += WorldInvInertiaTensor.TransformVector(FVector::CrossProduct(
			WorldPoints[Index] - COM, Forces[Index]));
	}

	ProjectPointDeltaAgainstConstraints(DeltaA, DeltaAlpha);
	SetPhysAcceleration(GetPhysAcceleration() + DeltaA);
	SetPhysAngularAcceleration(GetPhysAngularAcceleration() + DeltaAlpha);
}

void ISpeedComponent::SetPhysCOMVelocityRaw(const FVector& NewVelocity)
{
    SKinematic K = GetKinematicState();
    K.Velocity = NewVelocity;
	SetKinematicState(K);
}

void ISpeedComponent::SetPhysAngularVelocityRaw(const FVector& NewAngularVelocity)
{
    SKinematic K = GetKinematicState();
    K.AngularVelocity = NewAngularVelocity;
	SetKinematicState(K);
}

void ISpeedComponent::RegisterPhysicalConstraint(const FPhysicalContactConstraint& Constraint)
{
	if (!Constraint.IsValid())
	{
		return;
	}

	FPhysicalContactConstraint NewConstraint = Constraint;
	NewConstraint.Normal = NewConstraint.Normal.GetSafeNormal();
	NewConstraint.FramesSinceSeen = 0;
	if (NewConstraint.Normal.IsNearlyZero())
	{
		return;
	}

	for (FPhysicalContactConstraint& ExistingConstraint : PhysicalConstraints)
	{
		if (!AreSimilarPhysicalConstraints(ExistingConstraint, NewConstraint))
		{
			continue;
		}

		if (NewConstraint.PenetrationDepth >= ExistingConstraint.PenetrationDepth)
		{
			ExistingConstraint = NewConstraint;
		}
		else
		{
			ExistingConstraint.FramesSinceSeen = 0;
		}
		return;
	}

	PhysicalConstraints.Add(NewConstraint);
}

void ISpeedComponent::ClearPhysicalConstraints()
{
	PhysicalConstraints.Reset();
}

bool ISpeedComponent::HasActivePhysicalConstraintsOtherThan(const USSubBody* Source) const
{
	for (const FPhysicalContactConstraint& Constraint : PhysicalConstraints)
		if (Constraint.SourceSubBody.Get() != Source && IsPhysicalConstraintStillRelevant(Constraint))
			return true;
	return false;
}

bool ISpeedComponent::IsPhysicalConstraintStillRelevant(const FPhysicalContactConstraint& Constraint) const
{
	if (!Constraint.IsValid())
	{
		return false;
	}

	if (Constraint.FramesSinceSeen == 0)
	{
		return true;
	}

	const FVector Normal = Constraint.Normal.GetSafeNormal();
	if (Normal.IsNearlyZero())
	{
		return false;
	}

	const float SignedDist = FVector::DotProduct(GetPhysCOM() - Constraint.ContactPoint, Normal);
	if (SignedDist > ConstraintPersistSignedDistCm)
	{
		return false;
	}

	const float PointSpeedAlongNormal = FVector::DotProduct(GetPhysVelocityAtPoint(Constraint.ContactPoint), Normal);
	return PointSpeedAlongNormal <= ConstraintPersistSeparatingSpeed;
}

void ISpeedComponent::ProjectLinearDeltaAgainstConstraints(FVector& DeltaLinear) const
{
	if (DeltaLinear.IsNearlyZero())
	{
		return;
	}

	for (int32 PassIdx = 0; PassIdx < ConstraintProjectionPasses; ++PassIdx)
	{
		bool bChanged = false;
		for (const FPhysicalContactConstraint& Constraint : PhysicalConstraints)
		{
			if (!IsPhysicalConstraintStillRelevant(Constraint))
			{
				continue;
			}

			const FVector Normal = Constraint.Normal.GetSafeNormal();
			const float IntoSurface = FVector::DotProduct(DeltaLinear, Normal);
			if (IntoSurface < 0.0f)
			{
				DeltaLinear -= IntoSurface * Normal;
				bChanged = true;
			}
		}

		if (!bChanged)
		{
			break;
		}
	}
}

void ISpeedComponent::ProjectAngularDeltaAgainstConstraints(FVector& DeltaAngular) const
{
	if (DeltaAngular.IsNearlyZero())
	{
		return;
	}

	const FVector COM = GetPhysCOM();
	for (int32 PassIdx = 0; PassIdx < ConstraintProjectionPasses; ++PassIdx)
	{
		bool bChanged = false;
		for (const FPhysicalContactConstraint& Constraint : PhysicalConstraints)
		{
			if (!IsPhysicalConstraintStillRelevant(Constraint))
			{
				continue;
			}

			const FVector Normal = Constraint.Normal.GetSafeNormal();
			const FVector LeverArmAxis = FVector::CrossProduct(Constraint.ContactPoint - COM, Normal);
			const float AxisSizeSquared = LeverArmAxis.SizeSquared();
			if (AxisSizeSquared <= KINDA_SMALL_NUMBER)
			{
				continue;
			}

			const float IntoSurface = FVector::DotProduct(DeltaAngular, LeverArmAxis);
			if (IntoSurface < 0.0f)
			{
				DeltaAngular -= (IntoSurface / AxisSizeSquared) * LeverArmAxis;
				bChanged = true;
			}
		}

		if (!bChanged)
		{
			break;
		}
	}
}

void ISpeedComponent::ProjectPointDeltaAgainstConstraints(FVector& DeltaLinear, FVector& DeltaAngular) const
{
	if (DeltaLinear.IsNearlyZero() && DeltaAngular.IsNearlyZero())
	{
		return;
	}

	const FVector COM = GetPhysCOM();

	for (int32 PassIdx = 0; PassIdx < ConstraintProjectionPasses; ++PassIdx)
	{
		bool bChanged = false;

		for (const FPhysicalContactConstraint& Constraint : PhysicalConstraints)
		{
			if (!IsPhysicalConstraintStillRelevant(Constraint))
			{
				continue;
			}

			const FVector N = Constraint.Normal.GetSafeNormal();
			if (N.IsNearlyZero())
			{
				continue;
			}

			const FVector R = Constraint.ContactPoint - COM;
			const FVector DeltaAtPoint =
				DeltaLinear + FVector::CrossProduct(DeltaAngular, R);

			const float IntoSurface = FVector::DotProduct(DeltaAtPoint, N);
			if (IntoSurface >= 0.f)
			{
				continue;
			}

			const FVector LeverArmAxis = FVector::CrossProduct(R, N);
			const float AxisSizeSq = LeverArmAxis.SizeSquared();

			// Prefer angular correction, because this is a point constraint.
			if (AxisSizeSq > KINDA_SMALL_NUMBER && !DeltaAngular.IsNearlyZero())
			{
				DeltaAngular -= (IntoSurface / AxisSizeSq) * LeverArmAxis;
				bChanged = true;
			}
			else if (!DeltaLinear.IsNearlyZero())
			{
				DeltaLinear -= IntoSurface * N;
				bChanged = true;
			}
		}

		if (!bChanged)
		{
			break;
		}
	}
}

bool ISpeedComponent::AreSimilarPhysicalConstraints(const FPhysicalContactConstraint& A, const FPhysicalContactConstraint& B)
{
	if (!A.OtherComponent.IsValid() || !B.OtherComponent.IsValid())
	{
		return false;
	}

	if (A.OtherComponent != B.OtherComponent)
	{
		return false;
	}

	const FVector ANormal = A.Normal.GetSafeNormal();
	const FVector BNormal = B.Normal.GetSafeNormal();
	if (ANormal.IsNearlyZero() || BNormal.IsNearlyZero())
	{
		return false;
	}

	if (FVector::DotProduct(ANormal, BNormal) < ConstraintNormalDotThreshold)
	{
		return false;
	}

	return FVector::DistSquared(A.ContactPoint, B.ContactPoint) <= FMath::Square(ConstraintPointThresholdCm);
}

void ISpeedComponent::FreezeMovement()
{
	SetPhysCOMVelocityRaw(FVector::ZeroVector);
	SetPhysAngularVelocityRaw(FVector::ZeroVector);
	SetPhysAcceleration(FVector::ZeroVector);
	SetPhysAngularAcceleration(FVector::ZeroVector);

	SetIsFrozen(true);
}

void ISpeedComponent::UnFreezeMovement()
{
	SetIsFrozen(false);
}
