#include "ISpeedComponent.h"
#include "IAmSpeed/World/Simulation/Internal/BodyContactWrite.h"
#include "IAmSpeed/World/Simulation/Internal/ComponentContactWrite.h"
#include "IAmSpeed/Netcode/client-prediction/PrivateNativeStorageMap.h"
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadNativeObjects.h"
#include "IAmSpeed/World/Simulation/Internal/BodyKinematicRead.h"
#include "IAmSpeed/World/Simulation/Internal/ComponentKinematicRead.h"
#include "IAmSpeed/World/Simulation/Internal/OriginKinematicView.h"
using Speed::NativeKinematics::FOriginKinematicView;
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

}

void ISpeedComponent::RestoreSimulationSnapshot(
	const SKinematic& KinematicState,
	const bool bFrozen,
	const TConstArrayView<uint8> MechanicPayload)
{
    // This public entry may be reached without FSimulationWorld prevalidation.
    // A debug-only check cannot enforce the same refusal in Shipping. Refuse
    // BEFORE any common cache, pose, frozen flag or native sub-body mutation.
    if (!CanRestoreSimulationSnapshot(MechanicPayload)) return;
	SleepStateStorage().Reset();
	SetStaticCollisionWorldForFrame(nullptr);
	KinematicQuantizationCacheStorage().Reset();
	SetKinematicState(KinematicState);
	SetIsFrozen(bFrozen);
	RestoreSimulationSnapshotPayload(MechanicPayload);
	UpdateSubBodiesKinematics();
}

void ISpeedComponent::QuantizeKinematicState(SKinematic& State, const FQuat& PreviousRotation)
{
    // Prescribed bodies publish their immutable track sample, not a free-body
    // network quantization grid. Contact response remains owned by the peer.
    if(IsPrescribedMotion())return;
	// IAMSPEED.PHYS.BOX_STANDING_STILL/BOX_EQUILIBRIUM.V1: independent world
	// and actor grids need not intersect at a valid contact pose. Preserve the
	// pose of an ALREADY geometrically certified, non-rotating face (including
	// its material's admissible tangent slide), never round motion into rest.
	bool bPreserveSupportedPose = HasExactStaticFaceSupport();
	if (!bPreserveSupportedPose && StaticRestingWorldStorage())
	{
		for (const USSubBody* Body : GetSubBodies())
			if (Body && Body->RequiresUnquantizedContactPose()) { bPreserveSupportedPose = true; break; }
	}
	const FVector SupportedLocation = State.Location;
	const FQuat SupportedRotation = State.Rotation;
	const FVector SupportedVelocity = State.Velocity;
	const FVector SupportedAngularVelocity = State.AngularVelocity;
	KinematicQuantizationCacheStorage().Quantize(State, PreviousRotation);
	// Preserve the integrator's affine trajectory on unforced axes. Snapping
	// the position every frame changes an otherwise constant velocity by a
	// rate-dependent amount (80/1440 -> .06, but 80/60 -> 1.33 cm per frame).
	// This includes stationary axes and prevents a common fractional offset
	// from moving only one collision participant. A live acceleration revokes
	// the rule; velocity quantization and the inner codec/cache remain
	// unchanged. No residual, origin anchor or new published state is needed.
	for (int32 Axis = 0; Axis < 3; ++Axis)
		if (State.Acceleration[Axis] == 0.)
			State.Location[Axis] = SupportedLocation[Axis];
	// A packed Euler grid is not an angular integrator. Rounding after every
	// unforced step makes constant spin depend on the number of frames (and
	// pins sufficiently slow rotations). Preserve the integrated orientation
	// when angular acceleration is zero; the inner codec, cache and velocity
	// quantization remain unchanged. A live angular acceleration revokes this.
	if (State.AngularAcceleration.IsZero())
		State.Rotation = SupportedRotation;
	bool bRejectedQuantizedPose = false;
	if (!bPreserveSupportedPose && StaticRestingWorldStorage() &&
		(State.Location != SupportedLocation || State.Rotation != SupportedRotation))
	{
		for (const USSubBody* Body : GetSubBodies())
			if (Body && !Body->CanApplyQuantizedPose(*StaticRestingWorldStorage(), State))
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
	if (bPreserveSupportedPose || ShouldPreserveNativeVelocitiesForIntegration())
	{
		// Edge support couples the velocities; an explicit native-precision policy
		// also preserves their integrator values. Restore only after the unchanged
		// cache and pose checks; neither codec nor support admission is bypassed.
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
    SetKinematicState(ReadPhysicsKinematicState().Integrate(SubDelta));
#if !UE_BUILD_SHIPPING
	if (IsPhysicsPhaseObservationEnabled())
		ObservePhysicsPhase(ESpeedPhysicsObservationPhase::PostForcePredictor, SubDelta);
#endif
}

bool ISpeedComponent::IsPhysicsSleeping() const
{
	return !IsFrozen() && SleepStateStorage().CanSleep(ReadPhysicsKinematicState(), HasStableSleepSupport());
}

bool ISpeedComponent::HasExactStaticRestingSupport() const
{
	return ReadPhysicsKinematicState().Velocity.IsZero() && HasExactStaticFaceSupport();
}

bool ISpeedComponent::HasExactStaticFaceSupport() const
{
	if (!StaticRestingWorldStorage() || IsFrozen()) return false;
	const SKinematic& State = ReadPhysicsKinematicState();
	if (!State.AngularVelocity.IsZero() || !State.AngularAcceleration.IsZero()) return false;
	const FVector Load = GetNominalGravityAcceleration();
	if (Load.IsZero() || FVector::DotProduct(State.Velocity, Load) != 0) return false;
	for (const USSubBody* Body : GetSubBodies())
		if (Body && Body->HasExactStaticRestingSupport(*StaticRestingWorldStorage(), Load)) return true;
	return false;
}

float ISpeedComponent::GetMaximumCanonicalSupportInterval(float Remaining) const
{
    if(IsPrescribedMotion())return Remaining;
	if (!StaticRestingWorldStorage() || IsFrozen()) return Remaining;
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
	if (!StaticRestingWorldStorage()) { Observe(EStaticRestingReactionStatus::NoWorld); return FVector::ZeroVector; }
#else
	if (!StaticRestingWorldStorage()) return FVector::ZeroVector;
#endif
	const SKinematic& State = ReadPhysicsKinematicState();
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
			const FVector Reaction = SubBody->GetStaticRestingReaction(*StaticRestingWorldStorage(), StopAfterSeconds);
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
	if (!StaticRestingWorldStorage() || Delta <= 0 || StaticSupportFrameHorizonStorage() < Delta) return false;
	const SKinematic& State = ReadPhysicsKinematicState();
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
	if (!StaticRestingWorldStorage() || IsFrozen()) return;
	for (const USSubBody* Body : GetSubBodies())
		if (Body && Body->TryGetStaticContactAcceleration(Linear, Angular)) return;
}

void ISpeedComponent::IntegrateKinematics(const float& SubDelta)
{
	if (IsFrozen())
		return;

	if (SubDelta <= 0.f)
		return;

    // A mechanical track is kinematically imposed. Static support, friction
    // stop events, transport and depenetration cannot deflect it; evaluating
    // those free-body routines on large authored movers was pure overhead.
    if(IsPrescribedMotion())
    {
        PostIntegrateKinematics(SubDelta);
        return;
    }

	// Apply only during this segment. Restoring the external load afterwards
	// lets a later TOI impulse/torque release support immediately in this frame.
	double StopAfterSeconds = TNumericLimits<double>::Max();
	FVector RestingReaction = GetStaticRestingReaction(&StopAfterSeconds);
	FVector AngularReaction = FVector::ZeroVector;
	if (RestingReaction.IsZero()) GetStaticContactAcceleration(RestingReaction, AngularReaction);
	if (!RestingReaction.IsZero()) SetPhysAcceleration(Speed::NativeKinematics::FComponentRead::Of(*this).Acceleration() + RestingReaction);
	if (!AngularReaction.IsZero()) SetPhysAngularAcceleration(Speed::NativeKinematics::FComponentRead::Of(*this).AngularAcceleration() + AngularReaction);
	ON_SCOPE_EXIT
	{
		if (!RestingReaction.IsZero() || !AngularReaction.IsZero())
		{
			SetPhysAcceleration(Speed::NativeKinematics::FComponentRead::Of(*this).Acceleration() - RestingReaction);
			SetPhysAngularAcceleration(Speed::NativeKinematics::FComponentRead::Of(*this).AngularAcceleration() - AngularReaction);
			UpdateSubBodiesKinematics();
		}
	};

	if (!RestingReaction.IsZero() && StopAfterSeconds <= SubDelta)
	{
		// Integrate the exact Coulomb stop event. The remainder of this float
		// interval is rest, not reversed slip; no velocity threshold or sleep
		// is involved. The world split CCD at this event before reaching here.
		SKinematic State = ReadPhysicsKinematicState();
		State.Location += State.Velocity * (0.5 * StopAfterSeconds);
		State.Velocity = FVector::ZeroVector;
		SetKinematicState(State);
#if !UE_BUILD_SHIPPING
	if (IsPhysicsPhaseObservationEnabled())
		ObservePhysicsPhase(ESpeedPhysicsObservationPhase::PostForcePredictor, SubDelta);
#endif
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
#if !UE_BUILD_SHIPPING
	if (IsPhysicsPhaseObservationEnabled())
		ObservePhysicsPhase(ESpeedPhysicsObservationPhase::PostForcePredictor, SubDelta);
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
		SetPhysCOMVelocity(Speed::NativeKinematics::FComponentRead::Of(*this).COMVelocity());
		SetPhysAngularVelocity(Speed::NativeKinematics::FComponentRead::Of(*this).AngularVelocity());

		UpdateSubBodiesKinematics();
		ProjectEstablishedStaticContacts(TransportDelta);
	}

	// update physics state after integrating kinematics to the time of impact (e.g update suspension traces for wheels, update hit info for hitboxes, etc.)
	PostIntegrateKinematics(SubDelta);
    // UE_LOG(LogTemp, Log, TEXT("[IntegrateKinematics] NumFrame = %d, SubDelta = %f. Kinematics = %s"), NumFrame(), SubDelta, *ReadPhysicsKinematicState().ToString());
}

bool ISpeedComponent::ProjectEstablishedStaticContacts(const float& Delta)
{
    if(IsPrescribedMotion())return false;
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

class ISpeedComponent::FLegacyChildrenUpdateAccess final
{
    friend class ISpeedComponent;ISpeedComponent& Owner;
    explicit FLegacyChildrenUpdateAccess(ISpeedComponent& Value):Owner(Value){}
    FLegacyChildrenUpdateAccess(const FLegacyChildrenUpdateAccess&)=delete;
    FLegacyChildrenUpdateAccess& operator=(const FLegacyChildrenUpdateAccess&)=delete;
    FLegacyChildrenUpdateAccess(FLegacyChildrenUpdateAccess&&)=delete;
    FLegacyChildrenUpdateAccess& operator=(FLegacyChildrenUpdateAccess&&)=delete;
public:
    SKinematic Origin() const {return Speed::NativeKinematics::FComponentRead::Origin(Owner);}
    const TArray<USSubBody*>& SubBodies() const {return Speed::NativeKinematics::FComponentRead::SubBodies(Owner);}
    void Update(USSubBody* Body,const SKinematic& Parent) const {Body->UpdateKinematicsFromOwner(Parent);}
};
class ISpeedComponent::FNativeChildrenUpdateAccess final
{
    friend class ISpeedComponent;ISpeedComponent& Owner;
    const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation;
    explicit FNativeChildrenUpdateAccess(ISpeedComponent& Value,const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation):Owner(Value),Operation(Operation){(void)Speed::NativeKinematics::FComponentRead::Current(Operation,Owner);(void)SubBodies();}
    FNativeChildrenUpdateAccess(const FNativeChildrenUpdateAccess&)=delete;
    FNativeChildrenUpdateAccess& operator=(const FNativeChildrenUpdateAccess&)=delete;
    FNativeChildrenUpdateAccess(FNativeChildrenUpdateAccess&&)=delete;
    FNativeChildrenUpdateAccess& operator=(FNativeChildrenUpdateAccess&&)=delete;
public:
    SKinematic Origin() const {return Speed::NativeKinematics::FComponentRead::Origin(Operation,Owner);}
    const TArray<USSubBody*>& SubBodies() const {return Speed::NativeKinematics::FComponentRead::SubBodies(Operation,Owner);}
    void Update(USSubBody* Body,const SKinematic& Parent) const {Speed::NativeKinematics::FBodyWrite::UpdateFromOwner(Operation,Body,Parent);}
};
void ISpeedComponent::UpdateSubBodiesKinematics()
{
    FLegacyChildrenUpdateAccess Storage(*this);UpdateSubBodiesKinematicsBody(Storage);
}
void ISpeedComponent::UpdateSubBodiesKinematicsNative(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation)
{
    FNativeChildrenUpdateAccess Storage(*this,Operation);UpdateSubBodiesKinematicsBody(Storage);
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

class ISpeedComponent::FLegacyFrameResetAccess final
{
    ISpeedComponent& Owner;
public:
    explicit FLegacyFrameResetAccess(ISpeedComponent& Value):Owner(Value){}
    auto& Constraints() const {return Owner.PhysicalConstraintsStorage();}
    const TArray<USSubBody*>& Bodies() const {return Owner.GetSubBodies();}
    bool StillRelevant(const FPhysicalContactConstraint& Value) const {return Owner.IsPhysicalConstraintStillRelevant(Value);}
    void ResetChild(USSubBody& Body,const float& Delta) const {Body.ResetForFrame(Delta);}
};
class ISpeedComponent::FNativeFrameResetAccess final
{
    ISpeedComponent& Owner;const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation;
public:
    FNativeFrameResetAccess(ISpeedComponent& Value,const Speed::Netcode::Prediction::FPrivateNativeOperation& Op):Owner(Value),Operation(Op)
    {
        namespace P=Speed::Netcode::Prediction;
        P::FPrivateNativeStorageAccess::RequireStep(Operation);
        Speed::NativeKinematics::FComponentRead::ValidateCurrentAndSubBodies(Operation,&Owner);
        // Preflight the entire actual roster before constraints or an earlier
        // child can be written. Each dispatch revalidates its original subject.
        for(const USSubBody* Body:Bodies())
        {
            Speed::NativeKinematics::FBodyWrite::ValidateResetFrame(Operation,Body);
            if(Speed::NativeKinematics::FBodyRead::Parent(Operation,*Body)!=&Owner)
                throw P::FPrivateNativeStorageUnavailable{};
        }
    }
    FNativeFrameResetAccess(const FNativeFrameResetAccess&)=delete;
    FNativeFrameResetAccess& operator=(const FNativeFrameResetAccess&)=delete;
    auto& Constraints() const
    {return Speed::NativeKinematics::FComponentWrite::FrameConstraints(Operation,Owner);}
    const TArray<USSubBody*>& Bodies() const {return Speed::NativeKinematics::FComponentRead::SubBodies(Operation,Owner);}
    bool StillRelevant(const FPhysicalContactConstraint& Value) const {return Owner.IsPhysicalConstraintStillRelevantNative(Operation,Value);}
    void ResetChild(USSubBody& Body,const float& Delta) const {Speed::NativeKinematics::FBodyWrite::ResetFrame(Operation,&Body,Delta);}
};
template<class TAccess>
void ISpeedComponent::ResetForFrameBody(TAccess& Storage,const float& Delta)
{
	for (int32 ConstraintIdx = Storage.Constraints().Num() - 1; ConstraintIdx >= 0; --ConstraintIdx)
	{
		FPhysicalContactConstraint& Constraint = Storage.Constraints()[ConstraintIdx];

		if (!Constraint.bPersistent)
		{
			Storage.Constraints().RemoveAtSwap(ConstraintIdx);
			continue;
		}

		Constraint.FramesSinceSeen++;
		if (Constraint.FramesSinceSeen > MaxConstraintPersistenceFrames ||
			!Storage.StillRelevant(Constraint))
		{
			Storage.Constraints().RemoveAtSwap(ConstraintIdx);
		}
	}

	for (USSubBody* SubBody : Storage.Bodies())
	{
		Storage.ResetChild(*SubBody,Delta);
	}
}
void ISpeedComponent::ResetForFrame(const float& Delta)
{FLegacyFrameResetAccess Storage(*this);ResetForFrameBody(Storage,Delta);}
void ISpeedComponent::ResetNativeForFrame(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const float& Delta)
{FNativeFrameResetAccess Storage(*this,Operation);ResetForFrameBody(Storage,Delta);}


SComponentTOI ISpeedComponent::SweepTOISubBodies(
	const float& RemainingDelta, const float& LastSubDelta
#if !UE_BUILD_SHIPPING
	, const bool bCaptureDiagnostics
#endif
)
{
	// CCD predicts from the same constrained acceleration as integration, but
	// no reaction is cached across a collision, teleport or force change.
	FVector RestingReaction = GetStaticRestingReaction();
	FVector AngularReaction = FVector::ZeroVector;
	if (RestingReaction.IsZero()) GetStaticContactAcceleration(RestingReaction, AngularReaction);
	if (!RestingReaction.IsZero() || !AngularReaction.IsZero())
	{
		SetPhysAcceleration(Speed::NativeKinematics::FComponentRead::Of(*this).Acceleration() + RestingReaction);
		SetPhysAngularAcceleration(Speed::NativeKinematics::FComponentRead::Of(*this).AngularAcceleration() + AngularReaction);
		UpdateSubBodiesKinematics();
	}
	ON_SCOPE_EXIT
	{
		if (!RestingReaction.IsZero() || !AngularReaction.IsZero())
		{
			SetPhysAcceleration(Speed::NativeKinematics::FComponentRead::Of(*this).Acceleration() - RestingReaction);
			SetPhysAngularAcceleration(Speed::NativeKinematics::FComponentRead::Of(*this).AngularAcceleration() - AngularReaction);
			UpdateSubBodiesKinematics();
		}
	};
    // The hit names the other shape relative to Sweeper. Reorient all
    // body-owned fields if PickResolver selects that other shape.
    const auto ReorientForResolver = [](SHitResult& Hit, USSubBody* Sweeper, USSubBody* Resolver)
    {
        if (!Resolver || Resolver == Sweeper) return;
        Hit.ImpactNormal *= -1.f;
        Swap(Hit.ContactPointThis, Hit.ContactPointOther);
        Swap(Hit.ContactFeatureThis, Hit.ContactFeatureOther);
        Swap(Hit.ContactFeatureIndexThis, Hit.ContactFeatureIndexOther);
        Hit.Component = Sweeper;
        Hit.SubBody = Sweeper;
    };
    SComponentTOI Best;
    Best.bHit = false;
    Best.TOI = RemainingDelta;

    const TArray<USSubBody*>& InSubBodies = GetSubBodies();

#if !UE_BUILD_SHIPPING
	int32 DiagnosticSweepOrder = INDEX_NONE;
#endif
    for (USSubBody* Sweeper : InSubBodies)
    {
#if !UE_BUILD_SHIPPING
		if (bCaptureDiagnostics)
		{
			++DiagnosticSweepOrder;
		}
#endif
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

#if !UE_BUILD_SHIPPING
		if (bCaptureDiagnostics)
		{
			const SHitResult& DiagnosticHit = Speed::NativeKinematics::FBodyRead::FutureHit(*Sweeper);
			USSubBody* DiagnosticResolver = USSubBody::PickResolver(
				Sweeper, DiagnosticHit.SubBody.Get());
			SSubBodyTOIDiagnostic& Diagnostic = Best.DiagnosticCandidates.AddDefaulted_GetRef();
			Diagnostic.SweepOrder = DiagnosticSweepOrder;
			Diagnostic.TOI = TOI;
			Diagnostic.Sweeper = Sweeper->GetPhysicsWeakIdentity();
			Diagnostic.Resolver = DiagnosticResolver?DiagnosticResolver->GetPhysicsWeakIdentity():TWeakObjectPtr<USSubBody>{};
			Diagnostic.Hit = DiagnosticHit;
			ReorientForResolver(Diagnostic.Hit, Sweeper, DiagnosticResolver);
		}
#endif

        if (TOI >= Best.TOI)
            continue;

        const SHitResult& H = Speed::NativeKinematics::FBodyRead::FutureHit(*Sweeper);
        USSubBody* OtherSB = H.SubBody.Get();
        USSubBody* Resolver = USSubBody::PickResolver(Sweeper, OtherSB);

        SHitResult HR = H;

        ReorientForResolver(HR, Sweeper, Resolver);

        Best.bHit = true;
        Best.TOI = TOI;
        Best.Resolver = Resolver?Resolver->GetPhysicsWeakIdentity():TWeakObjectPtr<USSubBody>{};
        Best.Hit = HR;
#if !UE_BUILD_SHIPPING
		if (bCaptureDiagnostics)
		{
			Best.DiagnosticBestCandidate = Best.DiagnosticCandidates.Num() - 1;
		}
#endif
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
        if (StaticRestingWorldStorage() && HR.SurfaceId != 0 && !HR.bSurfaceNormalMayVary &&
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
	SleepStateStorage().ObserveCompletedPose(ReadPhysicsKinematicState());
}

// ===================== Kinematics getters and setters =================
FVector ISpeedComponent::GetPhysLocation() const
{
	// Keep the historical-frame selection used by the complete origin getter.
	const SKinematic& COMState = GetKinematicStateForFrame(NumFrame());
	return FOriginKinematicView(COMState, GetPhysCenterOfMassLocal()).Location();
}

class ISpeedComponent::FLegacyVelocityAccess final
{
    friend class ISpeedComponent;
    ISpeedComponent& Owner;
    explicit FLegacyVelocityAccess(ISpeedComponent& Value):Owner(Value){}
    FLegacyVelocityAccess(const FLegacyVelocityAccess&)=delete;
    FLegacyVelocityAccess& operator=(const FLegacyVelocityAccess&)=delete;
    FLegacyVelocityAccess(FLegacyVelocityAccess&&)=delete;
    FLegacyVelocityAccess& operator=(FLegacyVelocityAccess&&)=delete;
public:
    bool Frozen() const {return Owner.IsFrozen();}
    float MaxSpeed() const {return Owner.GetPhysMaxSpeed();}
    const SKinematic& Current() const {return Owner.ReadPhysicsKinematicState();}
    void SetState(const SKinematic& State) const {Owner.SetKinematicState(State);}
    void SetCOMRaw(const FVector& Value) const {Owner.SetPhysCOMVelocityRaw(Value);}
    const FQuat& Rotation() const {return Speed::NativeKinematics::FComponentRead::Rotation(Owner);}
    const FVector& LocalCOM() const {return Speed::NativeKinematics::FComponentRead::LocalCOM(Owner);}
    const FVector& COM() const {return Speed::NativeKinematics::FComponentRead::COM(Owner);}
    const FVector& COMVelocity() const {return Speed::NativeKinematics::FComponentRead::COMVelocity(Owner);}
    const FVector& AngularVelocity() const {return Speed::NativeKinematics::FComponentRead::AngularVelocity(Owner);}
    const FVector& Acceleration() const {return Speed::NativeKinematics::FComponentRead::Acceleration(Owner);}
    const FVector& AngularAcceleration() const {return Speed::NativeKinematics::FComponentRead::AngularAcceleration(Owner);}
    float MaxAngularSpeed() const {return Owner.GetPhysMaxAngularSpeed();}
    void SetCOMLocation(const FVector& Value) const {Owner.SetPhysCOMLocation(Value);}
    void SetRotation(const FQuat& Value) const {Owner.SetPhysRotation(Value);}
    void SetCOM(const FVector& Value) const {Owner.SetPhysCOMVelocity(Value);}
    void SetAngular(const FVector& Value) const {Owner.SetPhysAngularVelocity(Value);}
    void SetAngularRaw(const FVector& Value) const {Owner.SetPhysAngularVelocityRaw(Value);}
    void SetAcceleration(const FVector& Value) const {Owner.SetPhysAcceleration(Value);}
    void SetAngularAcceleration(const FVector& Value) const {Owner.SetPhysAngularAcceleration(Value);}
    void AddAngularAcceleration(const FVector& Value) const {Owner.AddPhysAngularAcceleration(Value);}
    void ProjectLinear(FVector& Value) const {Owner.ProjectLinearDeltaAgainstConstraints(Value);}
    void ProjectAngular(FVector& Value) const {Owner.ProjectAngularDeltaAgainstConstraints(Value);}
};
class ISpeedComponent::FNativeVelocityAccess final
{
    friend class ISpeedComponent;
    ISpeedComponent& Owner;
    const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation;
    explicit FNativeVelocityAccess(ISpeedComponent& Value,const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation):Owner(Value),Operation(Operation)
    {(void)Speed::NativeKinematics::FComponentRead::Current(Operation,Owner);}
    FNativeVelocityAccess(const FNativeVelocityAccess&)=delete;
    FNativeVelocityAccess& operator=(const FNativeVelocityAccess&)=delete;
    FNativeVelocityAccess(FNativeVelocityAccess&&)=delete;
    FNativeVelocityAccess& operator=(FNativeVelocityAccess&&)=delete;
public:
    bool Frozen() const {return Speed::NativeKinematics::FComponentRead::Frozen(Operation,Owner);}
    float MaxSpeed() const {return Speed::NativeKinematics::FComponentRead::MaxSpeed(Operation,Owner);}
    const SKinematic& Current() const {return Speed::NativeKinematics::FComponentRead::Current(Operation,Owner);}
    void SetState(const SKinematic& State) const {Speed::NativeKinematics::FComponentWrite::Kinematic(Operation,Owner,State);}
    void SetCOMRaw(const FVector& Value) const {Owner.SetPhysCOMVelocityRawNative(Operation,Value);}
    const FQuat& Rotation() const {return Speed::NativeKinematics::FComponentRead::Rotation(Operation,Owner);}
    const FVector& LocalCOM() const {return Speed::NativeKinematics::FComponentRead::LocalCOM(Operation,Owner);}
    const FVector& COM() const {return Speed::NativeKinematics::FComponentRead::COM(Operation,Owner);}
    const FVector& COMVelocity() const {return Speed::NativeKinematics::FComponentRead::COMVelocity(Operation,Owner);}
    const FVector& AngularVelocity() const {return Speed::NativeKinematics::FComponentRead::AngularVelocity(Operation,Owner);}
    const FVector& Acceleration() const {return Speed::NativeKinematics::FComponentRead::Acceleration(Operation,Owner);}
    const FVector& AngularAcceleration() const {return Speed::NativeKinematics::FComponentRead::AngularAcceleration(Operation,Owner);}
    float MaxAngularSpeed() const {return Speed::NativeKinematics::FComponentRead::MaxAngularSpeed(Operation,Owner);}
    void SetCOMLocation(const FVector& Value) const {Owner.SetPhysCOMLocationNative(Operation,Value);}
    void SetRotation(const FQuat& Value) const {Owner.SetPhysRotationNative(Operation,Value);}
    void SetCOM(const FVector& Value) const {Owner.SetPhysCOMVelocityNative(Operation,Value);}
    void SetAngular(const FVector& Value) const {Owner.SetPhysAngularVelocityNative(Operation,Value);}
    void SetAngularRaw(const FVector& Value) const {Owner.SetPhysAngularVelocityRawNative(Operation,Value);}
    void SetAcceleration(const FVector& Value) const {Owner.SetPhysAccelerationNative(Operation,Value);}
    void SetAngularAcceleration(const FVector& Value) const {Owner.SetPhysAngularAccelerationNative(Operation,Value);}
    void AddAngularAcceleration(const FVector& Value) const {Owner.AddPhysAngularAccelerationNative(Operation,Value);}
    void ProjectLinear(FVector& Value) const {Owner.ProjectLinearDeltaAgainstConstraintsNative(Operation,Value);}
    void ProjectAngular(FVector& Value) const {Owner.ProjectAngularDeltaAgainstConstraintsNative(Operation,Value);}
};


template<class TAccess>
void ISpeedComponent::SetPhysCOMLocationBody(TAccess& Storage,const FVector& NewCOMLocation)
{
	SKinematic K = Storage.Current();
	K.Location = NewCOMLocation;
	Storage.SetState(K);
}

void ISpeedComponent::SetPhysCOMLocation(const FVector& NewCOMLocation)
{
    FLegacyVelocityAccess Storage(*this);
    SetPhysCOMLocationBody(Storage,NewCOMLocation);
}
void ISpeedComponent::SetPhysCOMLocationNative(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const FVector& NewCOMLocation)
{
    FNativeVelocityAccess Storage(*this,Operation);
    SetPhysCOMLocationBody(Storage,NewCOMLocation);
}


template<class TAccess>
void ISpeedComponent::SetPhysLocationBody(TAccess& Storage,const FVector& NewLocation)
{
	Storage.SetCOMLocation(NewLocation + Storage.Rotation().RotateVector(Storage.LocalCOM()));
}

void ISpeedComponent::SetPhysLocation(const FVector& NewLocation)
{
    FLegacyVelocityAccess Storage(*this);
    SetPhysLocationBody(Storage,NewLocation);
}
void ISpeedComponent::SetPhysLocationNative(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const FVector& NewLocation)
{
    FNativeVelocityAccess Storage(*this,Operation);
    SetPhysLocationBody(Storage,NewLocation);
}


const FQuat& ISpeedComponent::GetPhysRotation() const
{
    return GetKinematicState().Rotation;
}

template<class TAccess>
void ISpeedComponent::SetPhysRotationBody(TAccess& Storage,const FQuat& NewRotation)
{
    SKinematic K = Storage.Current();
    K.Rotation = NewRotation;
	Storage.SetState(K);
}

void ISpeedComponent::SetPhysRotation(const FQuat& NewRotation)
{
    FLegacyVelocityAccess Storage(*this);
    SetPhysRotationBody(Storage,NewRotation);
}
void ISpeedComponent::SetPhysRotationNative(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const FQuat& NewRotation)
{
    FNativeVelocityAccess Storage(*this,Operation);
    SetPhysRotationBody(Storage,NewRotation);
}


template<class TAccess>
void ISpeedComponent::SetPhysRotationPreserveCOMBody(TAccess& Storage,const FQuat& NewRotation)
{
	// Rotation is stored at the COM, so it already preserves both COM position
	// and COM velocity. The origin is derived after the rotation changes.
	Storage.SetRotation(NewRotation);
}

void ISpeedComponent::SetPhysRotationPreserveCOM(const FQuat& NewRotation)
{
    FLegacyVelocityAccess Storage(*this);
    SetPhysRotationPreserveCOMBody(Storage,NewRotation);
}
void ISpeedComponent::SetPhysRotationPreserveCOMNative(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const FQuat& NewRotation)
{
    FNativeVelocityAccess Storage(*this,Operation);
    SetPhysRotationPreserveCOMBody(Storage,NewRotation);
}


FVector ISpeedComponent::GetPhysVelocity() const
{
	const SKinematic& COMState = GetKinematicStateForFrame(NumFrame());
	return FOriginKinematicView(COMState, GetPhysCenterOfMassLocal()).Velocity();
}

template<class TAccess>
void ISpeedComponent::SetPhysVelocityBody(TAccess& Storage,const FVector& NewVelocity)
{
	if (Storage.Frozen())
	{
		return;
	}

	const FVector OriginToCOM = Storage.Rotation().RotateVector(Storage.LocalCOM());
	Storage.SetCOM(NewVelocity + FVector::CrossProduct(Storage.AngularVelocity(), OriginToCOM));
}

void ISpeedComponent::SetPhysVelocity(const FVector& NewVelocity)
{
    FLegacyVelocityAccess Storage(*this);
    SetPhysVelocityBody(Storage,NewVelocity);
}
void ISpeedComponent::SetPhysVelocityNative(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const FVector& NewVelocity)
{
    FNativeVelocityAccess Storage(*this,Operation);
    SetPhysVelocityBody(Storage,NewVelocity);
}



template<class TAccess>
void ISpeedComponent::SetPhysCOMVelocityBody(TAccess& Storage,const FVector& NewCOMVelocity)
{
	if (Storage.Frozen())
	{
		return;
	}

	const FVector ClampedCOMVelocity = NewCOMVelocity.GetClampedToMaxSize(Storage.MaxSpeed());
	Storage.SetCOMRaw(ClampedCOMVelocity);
}

void ISpeedComponent::SetPhysCOMVelocity(const FVector& NewCOMVelocity)
{
    FLegacyVelocityAccess Storage(*this);
    SetPhysCOMVelocityBody(Storage,NewCOMVelocity);
}

void ISpeedComponent::SetPhysCOMVelocityNative(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const FVector& NewCOMVelocity)
{
    FNativeVelocityAccess Storage(*this,Operation);
    SetPhysCOMVelocityBody(Storage,NewCOMVelocity);
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

template<class TAccess>
void ISpeedComponent::SetPhysAngularVelocityBody(TAccess& Storage,const FVector& NewAngularVelocity)
{
	if (Storage.Frozen())
	{
		return;
	}
	Storage.SetAngularRaw(NewAngularVelocity.GetClampedToMaxSize(Storage.MaxAngularSpeed()));
}

void ISpeedComponent::SetPhysAngularVelocity(const FVector& NewAngularVelocity)
{
    FLegacyVelocityAccess Storage(*this);
    SetPhysAngularVelocityBody(Storage,NewAngularVelocity);
}
void ISpeedComponent::SetPhysAngularVelocityNative(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const FVector& NewAngularVelocity)
{
    FNativeVelocityAccess Storage(*this,Operation);
    SetPhysAngularVelocityBody(Storage,NewAngularVelocity);
}


template<class TAccess>
void ISpeedComponent::SetPhysVelocityAtPointBody(TAccess& Storage,const FVector& NewVelocity,const FVector& Point)
{
    // v + w x r
    const FVector R = Point - Storage.COM();
    const FVector NewAngularVelocity = FVector::CrossProduct(R, NewVelocity - Storage.COMVelocity()).GetSafeNormal() * Storage.AngularVelocity().Size();
    Storage.SetAngular(NewAngularVelocity);
    Storage.SetCOM(NewVelocity - FVector::CrossProduct(NewAngularVelocity, R));
}

void ISpeedComponent::SetPhysVelocityAtPoint(const FVector& NewVelocity, const FVector& Point)
{
    FLegacyVelocityAccess Storage(*this);
    SetPhysVelocityAtPointBody(Storage,NewVelocity,Point);
}
void ISpeedComponent::SetPhysVelocityAtPointNative(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const FVector& NewVelocity,const FVector& Point)
{
    FNativeVelocityAccess Storage(*this,Operation);
    SetPhysVelocityAtPointBody(Storage,NewVelocity,Point);
}


const FVector& ISpeedComponent::GetPhysAcceleration() const
{
    return GetKinematicState().Acceleration;
}

const FVector& ISpeedComponent::GetPhysAngularAcceleration() const
{
	return GetKinematicState().AngularAcceleration;
}

template<class TAccess>
void ISpeedComponent::SetPhysAccelerationBody(TAccess& Storage,const FVector& NewAcceleration)
{
	if (Storage.Frozen())
	{
		return;
	}

    SKinematic K = Storage.Current();
    K.Acceleration = NewAcceleration;
	Storage.SetState(K);
}

void ISpeedComponent::SetPhysAcceleration(const FVector& NewAcceleration)
{
    FLegacyVelocityAccess Storage(*this);
    SetPhysAccelerationBody(Storage,NewAcceleration);
}
void ISpeedComponent::SetPhysAccelerationNative(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const FVector& NewAcceleration)
{
    FNativeVelocityAccess Storage(*this,Operation);
    SetPhysAccelerationBody(Storage,NewAcceleration);
}


template<class TAccess>
void ISpeedComponent::SetPhysAngularAccelerationBody(TAccess& Storage,const FVector& NewAngularAcceleration)
{
	if (Storage.Frozen())
	{
		return;
	}

    SKinematic K = Storage.Current();
    K.AngularAcceleration = NewAngularAcceleration;
	Storage.SetState(K);
}

void ISpeedComponent::SetPhysAngularAcceleration(const FVector& NewAngularAcceleration)
{
    FLegacyVelocityAccess Storage(*this);
    SetPhysAngularAccelerationBody(Storage,NewAngularAcceleration);
}
void ISpeedComponent::SetPhysAngularAccelerationNative(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const FVector& NewAngularAcceleration)
{
    FNativeVelocityAccess Storage(*this,Operation);
    SetPhysAngularAccelerationBody(Storage,NewAngularAcceleration);
}


FVector ISpeedComponent::GetPhysAccelerationAtPoint(const FVector& Point) const
{
    // a + alpha x r + w x (w x r)
    const FVector R = Point - GetPhysCOM();
    return GetPhysAcceleration() + FVector::CrossProduct(GetPhysAngularAcceleration(), R) + FVector::CrossProduct(GetPhysAngularVelocity(), FVector::CrossProduct(GetPhysAngularVelocity(), R));
}

template<class TAccess>
void ISpeedComponent::AddPhysVelocityBody(TAccess& Storage,const FVector& DeltaVelocity)
{
	FVector ProjectedDeltaVelocity = DeltaVelocity;
	Storage.ProjectLinear(ProjectedDeltaVelocity);
	Storage.SetCOM(Storage.COMVelocity() + ProjectedDeltaVelocity);
}

void ISpeedComponent::AddPhysVelocity(const FVector& DeltaVelocity)
{
    FLegacyVelocityAccess Storage(*this);
    AddPhysVelocityBody(Storage,DeltaVelocity);
}
void ISpeedComponent::AddPhysVelocityNative(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const FVector& DeltaVelocity)
{
    FNativeVelocityAccess Storage(*this,Operation);
    AddPhysVelocityBody(Storage,DeltaVelocity);
}


template<class TAccess>
FVector ISpeedComponent::AddPhysLocationBody(TAccess& Storage,const FVector& DeltaLocation)
{
	FVector ProjectedDeltaLocation = DeltaLocation;
	Storage.ProjectLinear(ProjectedDeltaLocation);
	if (!ProjectedDeltaLocation.IsNearlyZero())
	{
		Storage.SetCOMLocation(Storage.COM() + ProjectedDeltaLocation);
	}
	return ProjectedDeltaLocation;
}

FVector ISpeedComponent::AddPhysLocation(const FVector& DeltaLocation)
{
    FLegacyVelocityAccess Storage(*this);
    return AddPhysLocationBody(Storage,DeltaLocation);
}
FVector ISpeedComponent::AddPhysLocationNative(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const FVector& DeltaLocation)
{
    FNativeVelocityAccess Storage(*this,Operation);
    return AddPhysLocationBody(Storage,DeltaLocation);
}


template<class TAccess>
FQuat ISpeedComponent::AddPhysRotationBody(TAccess& Storage,const FQuat& DeltaRotation)
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
	Storage.ProjectAngular(DeltaAngular);

	const float ProjectedAngle = DeltaAngular.Size();
	if (ProjectedAngle <= KINDA_SMALL_NUMBER)
	{
		return FQuat::Identity;
	}

	const FQuat AppliedDeltaRotation(DeltaAngular / ProjectedAngle, ProjectedAngle);
	Storage.SetRotation((Storage.Rotation() * AppliedDeltaRotation).GetNormalized());
	return AppliedDeltaRotation;
}

FQuat ISpeedComponent::AddPhysRotation(const FQuat& DeltaRotation)
{
    FLegacyVelocityAccess Storage(*this);
    return AddPhysRotationBody(Storage,DeltaRotation);
}
FQuat ISpeedComponent::AddPhysRotationNative(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const FQuat& DeltaRotation)
{
    FNativeVelocityAccess Storage(*this,Operation);
    return AddPhysRotationBody(Storage,DeltaRotation);
}

template<class TAccess>
void ISpeedComponent::AddPhysAngularVelocityBody(TAccess& Storage,const FVector& DeltaAngularVelocity)
{
	FVector ProjectedDeltaAngularVelocity = DeltaAngularVelocity;
	Storage.ProjectAngular(ProjectedDeltaAngularVelocity);
	Storage.SetAngular(Storage.AngularVelocity() + ProjectedDeltaAngularVelocity);
}

void ISpeedComponent::AddPhysAngularVelocity(const FVector& DeltaAngularVelocity)
{
    FLegacyVelocityAccess Storage(*this);
    AddPhysAngularVelocityBody(Storage,DeltaAngularVelocity);
}
void ISpeedComponent::AddPhysAngularVelocityNative(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const FVector& DeltaAngularVelocity)
{
    FNativeVelocityAccess Storage(*this,Operation);
    AddPhysAngularVelocityBody(Storage,DeltaAngularVelocity);
}


template<class TAccess>
void ISpeedComponent::AddPhysAccelerationBody(TAccess& Storage,const FVector& DeltaAcceleration)
{
	FVector ProjectedDeltaAcceleration = DeltaAcceleration;
	Storage.ProjectLinear(ProjectedDeltaAcceleration);
    Storage.SetAcceleration(Storage.Acceleration() + ProjectedDeltaAcceleration);
}

void ISpeedComponent::AddPhysAcceleration(const FVector& DeltaAcceleration)
{
    FLegacyVelocityAccess Storage(*this);
    AddPhysAccelerationBody(Storage,DeltaAcceleration);
}
void ISpeedComponent::AddPhysAccelerationNative(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const FVector& DeltaAcceleration)
{
    FNativeVelocityAccess Storage(*this,Operation);
    AddPhysAccelerationBody(Storage,DeltaAcceleration);
}


template<class TAccess>
void ISpeedComponent::AddPhysAngularAccelerationBody(TAccess& Storage,const FVector& DeltaAngularAcceleration)
{
	FVector ProjectedDeltaAngularAcceleration = DeltaAngularAcceleration;
	Storage.ProjectAngular(ProjectedDeltaAngularAcceleration);
    Storage.SetAngularAcceleration(Storage.AngularAcceleration() + ProjectedDeltaAngularAcceleration);
}

void ISpeedComponent::AddPhysAngularAcceleration(const FVector& DeltaAngularAcceleration)
{
    FLegacyVelocityAccess Storage(*this);
    AddPhysAngularAccelerationBody(Storage,DeltaAngularAcceleration);
}
void ISpeedComponent::AddPhysAngularAccelerationNative(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const FVector& DeltaAngularAcceleration)
{
    FNativeVelocityAccess Storage(*this,Operation);
    AddPhysAngularAccelerationBody(Storage,DeltaAngularAcceleration);
}


template<class TAccess>
void ISpeedComponent::AddPhysAngularAccelerationLocalBody(TAccess& Storage,const FVector& LocalAngularAccel)
{
    // alpha_world = R * alpha_local
    const FQuat Rot = Storage.Rotation();
    const FVector DeltaAngularAccelWorld = Rot.RotateVector(LocalAngularAccel);
	Storage.AddAngularAcceleration(DeltaAngularAccelWorld);
}

void ISpeedComponent::AddPhysAngularAccelerationLocal(const FVector& LocalAngularAccel)
{
    FLegacyVelocityAccess Storage(*this);
    AddPhysAngularAccelerationLocalBody(Storage,LocalAngularAccel);
}
void ISpeedComponent::AddPhysAngularAccelerationLocalNative(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const FVector& LocalAngularAccel)
{
    FNativeVelocityAccess Storage(*this,Operation);
    AddPhysAngularAccelerationLocalBody(Storage,LocalAngularAccel);
}


class ISpeedComponent::FLegacyImpulseAccess final
{
    friend class ISpeedComponent;
    ISpeedComponent& Owner;
    explicit FLegacyImpulseAccess(ISpeedComponent& Value):Owner(Value){}
    FLegacyImpulseAccess(const FLegacyImpulseAccess&)=delete;
    FLegacyImpulseAccess& operator=(const FLegacyImpulseAccess&)=delete;
    FLegacyImpulseAccess(FLegacyImpulseAccess&&)=delete;
    FLegacyImpulseAccess& operator=(FLegacyImpulseAccess&&)=delete;
public:
    float Mass() const {return Owner.GetPhysMass();}
    float MaxAngularSpeed() const {return Owner.GetPhysMaxAngularSpeed();}
    const FVector& COM() const {return Speed::NativeKinematics::FComponentRead::COM(Owner);}
    const FVector& COMVelocity() const {return Speed::NativeKinematics::FComponentRead::COMVelocity(Owner);}
    const FVector& AngularVelocity() const {return Speed::NativeKinematics::FComponentRead::AngularVelocity(Owner);}
    FMatrix BodyTensor(const USolidSubBody* Body) const {return Body->ComputeWorldInvInertiaTensor();}
    FMatrix ComponentTensor() const {return Owner.ComputeWorldInvInertiaTensor();}
    void Project(FVector& V,FVector& W) const {Owner.ProjectPointDeltaAgainstConstraints(V,W);}
    void SetAngularRaw(const FVector& V) const {Owner.SetPhysAngularVelocityRaw(V);}
    void SetCOM(const FVector& V) const {Owner.SetPhysCOMVelocity(V);}
    const FVector& Acceleration() const {return Speed::NativeKinematics::FComponentRead::Acceleration(Owner);}
    void SetAcceleration(const FVector& V) const {Owner.SetPhysAcceleration(V);}
    const FVector& AngularAcceleration() const {return Speed::NativeKinematics::FComponentRead::AngularAcceleration(Owner);}
    void SetAngularAcceleration(const FVector& V) const {Owner.SetPhysAngularAcceleration(V);}
};
class ISpeedComponent::FNativeImpulseAccess final
{
    friend class ISpeedComponent;
    ISpeedComponent& Owner;
    const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation;
    explicit FNativeImpulseAccess(ISpeedComponent& Value,const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation):Owner(Value),Operation(Operation)
    {(void)Speed::NativeKinematics::FComponentRead::Current(Operation,Owner);}
    FNativeImpulseAccess(const FNativeImpulseAccess&)=delete;
    FNativeImpulseAccess& operator=(const FNativeImpulseAccess&)=delete;
    FNativeImpulseAccess(FNativeImpulseAccess&&)=delete;
    FNativeImpulseAccess& operator=(FNativeImpulseAccess&&)=delete;
public:
    float Mass() const {return Speed::NativeKinematics::FComponentRead::Mass(Operation,Owner);}
    float MaxAngularSpeed() const {return Speed::NativeKinematics::FComponentRead::MaxAngularSpeed(Operation,Owner);}
    const FVector& COM() const {return Speed::NativeKinematics::FComponentRead::COM(Operation,Owner);}
    const FVector& COMVelocity() const {return Speed::NativeKinematics::FComponentRead::COMVelocity(Operation,Owner);}
    const FVector& AngularVelocity() const {return Speed::NativeKinematics::FComponentRead::AngularVelocity(Operation,Owner);}
    FMatrix BodyTensor(const USolidSubBody* Body) const {return Speed::NativeKinematics::FBodyRead::WorldInvInertia(Operation,Body);}
    FMatrix ComponentTensor() const {return Speed::NativeKinematics::FComponentRead::WorldInvInertia(Operation,Owner);}
    void Project(FVector& V,FVector& W) const {Owner.ProjectPointDeltaAgainstConstraintsNative(Operation,V,W);}
    void SetAngularRaw(const FVector& V) const {Owner.SetPhysAngularVelocityRawNative(Operation,V);}
    void SetCOM(const FVector& V) const {Owner.SetPhysCOMVelocityNative(Operation,V);}
    const FVector& Acceleration() const {return Speed::NativeKinematics::FComponentRead::Acceleration(Operation,Owner);}
    void SetAcceleration(const FVector& V) const {Owner.SetPhysAccelerationNative(Operation,V);}
    const FVector& AngularAcceleration() const {return Speed::NativeKinematics::FComponentRead::AngularAcceleration(Operation,Owner);}
    void SetAngularAcceleration(const FVector& V) const {Owner.SetPhysAngularAccelerationNative(Operation,V);}
};
template<class TAccess>
void ISpeedComponent::AddPhysImpulseAtPointBody(TAccess& Storage,const FVector& Impulse,const FVector& Point,const USolidSubBody* SubBody)
{
    // delta v = impulse / mass
    float PhysMass = Storage.Mass() > 0.f ? Storage.Mass() : 1.f; // avoid divide by zero
    FVector DeltaV = Impulse / PhysMass;
    // delta w = I^-1 * (r x impulse)
	const FVector COM = Storage.COM();
    const FVector R = Point - COM;
	const FMatrix WorldInvInertiaTensor = SubBody? Storage.BodyTensor(SubBody) : Storage.ComponentTensor();
    FVector DeltaW = WorldInvInertiaTensor.TransformVector(FVector::CrossProduct(R, Impulse));

	Storage.Project(DeltaV, DeltaW);
	const FVector NewCOMVelocity = Storage.COMVelocity() + DeltaV;
	Storage.SetAngularRaw((Storage.AngularVelocity() + DeltaW).GetClampedToMaxSize(Storage.MaxAngularSpeed()));
	Storage.SetCOM(NewCOMVelocity);
}

void ISpeedComponent::AddPhysImpulseAtPoint(const FVector& Impulse, const FVector& Point, const USolidSubBody* SubBody)
{
    FLegacyImpulseAccess Storage(*this);
    AddPhysImpulseAtPointBody(Storage,Impulse,Point,SubBody);
}
void ISpeedComponent::AddPhysImpulseAtPointNative(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const FVector& Impulse,const FVector& Point,const USolidSubBody* SubBody)
{
    FNativeImpulseAccess Storage(*this,Operation);
    AddPhysImpulseAtPointBody(Storage,Impulse,Point,SubBody);
}


template<class TAccess>
void ISpeedComponent::AddPhysImpulseBatchAtPointsBody(TAccess& Storage,const TArray<FVector>& Impulses,const TArray<FVector>& WorldPoints,const USolidSubBody* SubBody)
{
	if (Impulses.Num() == 0 || Impulses.Num() != WorldPoints.Num())
	{
		return;
	}

	const float PhysMass = Storage.Mass() > 0.f ? Storage.Mass() : 1.f;
	const FVector COM = Storage.COM();
	const FMatrix WorldInvInertiaTensor = SubBody
		? Storage.BodyTensor(SubBody) : Storage.ComponentTensor();
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
	Storage.Project(DeltaV, DeltaW);
	Storage.SetAngularRaw((Storage.AngularVelocity() + DeltaW)
		.GetClampedToMaxSize(Storage.MaxAngularSpeed()));
	Storage.SetCOM(Storage.COMVelocity() + DeltaV);
}

void ISpeedComponent::AddPhysImpulseBatchAtPoints(
	const TArray<FVector>& Impulses, const TArray<FVector>& WorldPoints,
	const USolidSubBody* SubBody)
{
    FLegacyImpulseAccess Storage(*this);
    AddPhysImpulseBatchAtPointsBody(Storage,Impulses,WorldPoints,SubBody);
}
void ISpeedComponent::AddPhysImpulseBatchAtPointsNative(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const TArray<FVector>& Impulses,const TArray<FVector>& WorldPoints,const USolidSubBody* SubBody)
{
    FNativeImpulseAccess Storage(*this,Operation);
    AddPhysImpulseBatchAtPointsBody(Storage,Impulses,WorldPoints,SubBody);
}


template<class TAccess>
void ISpeedComponent::AddPhysForceAtPointBody(TAccess& Storage,const FVector& Force,const FVector& WorldPoint,const USolidSubBody* SubBody)
{
    // F = m * a => a = F / m
    float PhysMass = Storage.Mass() > 0.f ? Storage.Mass() : 1.f; // avoid divide by zero
    FVector DeltaA = Force / PhysMass;
    // alpha = I^-1 * (r x F)
	const FVector COM = Storage.COM();
    const FVector R = WorldPoint - COM;
	const FMatrix WorldInvInertiaTensor = SubBody? Storage.BodyTensor(SubBody) : Storage.ComponentTensor();
    FVector DeltaAlpha = WorldInvInertiaTensor.TransformVector(FVector::CrossProduct(R, Force));

	Storage.Project(DeltaA, DeltaAlpha);
    Storage.SetAcceleration(Storage.Acceleration() + DeltaA);
	Storage.SetAngularAcceleration(Storage.AngularAcceleration() + DeltaAlpha);
}

void ISpeedComponent::AddPhysForceAtPoint(const FVector& Force, const FVector& WorldPoint, const USolidSubBody* SubBody)
{
    FLegacyImpulseAccess Storage(*this);
    AddPhysForceAtPointBody(Storage,Force,WorldPoint,SubBody);
}
void ISpeedComponent::AddPhysForceAtPointNative(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const FVector& Force,const FVector& WorldPoint,const USolidSubBody* SubBody)
{
    FNativeImpulseAccess Storage(*this,Operation);
    AddPhysForceAtPointBody(Storage,Force,WorldPoint,SubBody);
}


template<class TAccess>
void ISpeedComponent::AddPhysForceBatchAtPointsBody(TAccess& Storage,const TArray<FVector>& Forces,const TArray<FVector>& WorldPoints,const USolidSubBody* SubBody)
{
	if (Forces.Num() == 0 || Forces.Num() != WorldPoints.Num())
	{
		return;
	}

	const float PhysMass = Storage.Mass() > 0.f ? Storage.Mass() : 1.f;
	const FVector COM = Storage.COM();
	const FMatrix WorldInvInertiaTensor = SubBody
		? Storage.BodyTensor(SubBody) : Storage.ComponentTensor();
	FVector DeltaA = FVector::ZeroVector;
	FVector DeltaAlpha = FVector::ZeroVector;
	for (int32 Index = 0; Index < Forces.Num(); ++Index)
	{
		DeltaA += Forces[Index] / PhysMass;
		DeltaAlpha += WorldInvInertiaTensor.TransformVector(FVector::CrossProduct(
			WorldPoints[Index] - COM, Forces[Index]));
	}

	Storage.Project(DeltaA, DeltaAlpha);
	Storage.SetAcceleration(Storage.Acceleration() + DeltaA);
	Storage.SetAngularAcceleration(Storage.AngularAcceleration() + DeltaAlpha);
}

void ISpeedComponent::AddPhysForceBatchAtPoints(
	const TArray<FVector>& Forces, const TArray<FVector>& WorldPoints,
	const USolidSubBody* SubBody)
{
    FLegacyImpulseAccess Storage(*this);
    AddPhysForceBatchAtPointsBody(Storage,Forces,WorldPoints,SubBody);
}
void ISpeedComponent::AddPhysForceBatchAtPointsNative(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const TArray<FVector>& Forces,const TArray<FVector>& WorldPoints,const USolidSubBody* SubBody)
{
    FNativeImpulseAccess Storage(*this,Operation);
    AddPhysForceBatchAtPointsBody(Storage,Forces,WorldPoints,SubBody);
}


template<class TAccess>
void ISpeedComponent::SetPhysCOMVelocityRawBody(TAccess& Storage,const FVector& NewVelocity)
{
    SKinematic K = Storage.Current();
    K.Velocity = NewVelocity;
	Storage.SetState(K);
}

void ISpeedComponent::SetPhysCOMVelocityRaw(const FVector& NewVelocity)
{
    FLegacyVelocityAccess Storage(*this);
    SetPhysCOMVelocityRawBody(Storage,NewVelocity);
}

void ISpeedComponent::SetPhysCOMVelocityRawNative(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const FVector& NewVelocity)
{
    FNativeVelocityAccess Storage(*this,Operation);
    SetPhysCOMVelocityRawBody(Storage,NewVelocity);
}


template<class TAccess>
void ISpeedComponent::SetPhysAngularVelocityRawBody(TAccess& Storage,const FVector& NewAngularVelocity)
{
    SKinematic K = Storage.Current();
    K.AngularVelocity = NewAngularVelocity;
	Storage.SetState(K);
}

void ISpeedComponent::SetPhysAngularVelocityRaw(const FVector& NewAngularVelocity)
{
    FLegacyVelocityAccess Storage(*this);
    SetPhysAngularVelocityRawBody(Storage,NewAngularVelocity);
}

void ISpeedComponent::SetPhysAngularVelocityRawNative(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const FVector& NewAngularVelocity)
{
    FNativeVelocityAccess Storage(*this,Operation);
    SetPhysAngularVelocityRawBody(Storage,NewAngularVelocity);
}


void ISpeedComponent::RegisterPhysicalConstraintForPhysics(const FPhysicalContactConstraint& Constraint)
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

	for (FPhysicalContactConstraint& ExistingConstraint : PhysicalConstraintsStorage())
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

	PhysicalConstraintsStorage().Add(NewConstraint);
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

	for (FPhysicalContactConstraint& ExistingConstraint : PublicPhysicalConstraintsMutableView())
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

	PublicPhysicalConstraintsMutableView().Add(NewConstraint);
}

void ISpeedComponent::ClearPhysicalConstraintsForPhysics()
{
	PhysicalConstraintsStorage().Reset();
}
void ISpeedComponent::ClearPhysicalConstraints()
{
	PublicPhysicalConstraintsMutableView().Reset();
}

bool ISpeedComponent::HasActivePhysicalConstraintsOtherThan(const USSubBody* Source) const
{
	for (const FPhysicalContactConstraint& Constraint : PhysicalConstraintsStorage())
		if (Constraint.SourceSubBody.Get() != Source && IsPhysicalConstraintStillRelevant(Constraint))
			return true;
	return false;
}

class ISpeedComponent::FLegacyConstraintAccess final
{
    friend class ISpeedComponent;
    const ISpeedComponent& Owner;
    explicit FLegacyConstraintAccess(const ISpeedComponent& Value):Owner(Value){}
    FLegacyConstraintAccess(const FLegacyConstraintAccess&)=delete;
    FLegacyConstraintAccess& operator=(const FLegacyConstraintAccess&)=delete;
    FLegacyConstraintAccess(FLegacyConstraintAccess&&)=delete;
    FLegacyConstraintAccess& operator=(FLegacyConstraintAccess&&)=delete;
public:
    const TArray<FPhysicalContactConstraint>& Constraints() const {return Owner.PhysicalConstraintsStorage();}
    const FVector& COM() const {return Speed::NativeKinematics::FComponentRead::COM(Owner);}
    FVector VelocityAtPoint(const FVector& Point) const {return Speed::NativeKinematics::FComponentRead::VelocityAtPoint(Owner,Point);}
    bool Relevant(const FPhysicalContactConstraint& Constraint) const {return Owner.IsPhysicalConstraintStillRelevant(Constraint);}
};
class ISpeedComponent::FNativeConstraintAccess final
{
    friend class ISpeedComponent;
    const ISpeedComponent& Owner;
    const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation;
    explicit FNativeConstraintAccess(const ISpeedComponent& Value,const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation):Owner(Value),Operation(Operation)
    {
        // Real subject/root/type validation before even local delta mutation.
        (void)Speed::NativeKinematics::FComponentRead::Current(Operation,Owner);
        (void)Speed::NativeKinematics::FComponentRead::Constraints(Operation,Owner);
    }
    FNativeConstraintAccess(const FNativeConstraintAccess&)=delete;
    FNativeConstraintAccess& operator=(const FNativeConstraintAccess&)=delete;
    FNativeConstraintAccess(FNativeConstraintAccess&&)=delete;
    FNativeConstraintAccess& operator=(FNativeConstraintAccess&&)=delete;
public:
    const TArray<FPhysicalContactConstraint>& Constraints() const {return Speed::NativeKinematics::FComponentRead::Constraints(Operation,Owner);}
    const FVector& COM() const {return Speed::NativeKinematics::FComponentRead::COM(Operation,Owner);}
    FVector VelocityAtPoint(const FVector& Point) const {return Speed::NativeKinematics::FComponentRead::VelocityAtPoint(Operation,Owner,Point);}
    bool Relevant(const FPhysicalContactConstraint& Constraint) const {return Owner.IsPhysicalConstraintStillRelevantNative(Operation,Constraint);}
};

template<class TAccess>
bool ISpeedComponent::IsPhysicalConstraintStillRelevantBody(TAccess& Storage,const FPhysicalContactConstraint& Constraint) const
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

	const float SignedDist = FVector::DotProduct(Storage.COM() - Constraint.ContactPoint, Normal);
	if (SignedDist > ConstraintPersistSignedDistCm)
	{
		return false;
	}

	const float PointSpeedAlongNormal = FVector::DotProduct(Storage.VelocityAtPoint(Constraint.ContactPoint), Normal);
	return PointSpeedAlongNormal <= ConstraintPersistSeparatingSpeed;
}

bool ISpeedComponent::IsPhysicalConstraintStillRelevant(const FPhysicalContactConstraint& Constraint) const
{
    FLegacyConstraintAccess Storage(*this);
    return IsPhysicalConstraintStillRelevantBody(Storage,Constraint);
}

bool ISpeedComponent::IsPhysicalConstraintStillRelevantNative(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const FPhysicalContactConstraint& Constraint) const
{
    FNativeConstraintAccess Storage(*this,Operation);
    return IsPhysicalConstraintStillRelevantBody(Storage,Constraint);
}


template<class TAccess>
void ISpeedComponent::ProjectLinearDeltaAgainstConstraintsBody(TAccess& Storage,FVector& DeltaLinear) const
{
	if (DeltaLinear.IsNearlyZero())
	{
		return;
	}

	for (int32 PassIdx = 0; PassIdx < ConstraintProjectionPasses; ++PassIdx)
	{
		bool bChanged = false;
		for (const FPhysicalContactConstraint& Constraint : Storage.Constraints())
		{
			if (!Storage.Relevant(Constraint))
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

void ISpeedComponent::ProjectLinearDeltaAgainstConstraints(FVector& DeltaLinear) const
{
    FLegacyConstraintAccess Storage(*this);
    ProjectLinearDeltaAgainstConstraintsBody(Storage,DeltaLinear);
}

void ISpeedComponent::ProjectLinearDeltaAgainstConstraintsNative(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,FVector& DeltaLinear) const
{
    FNativeConstraintAccess Storage(*this,Operation);
    ProjectLinearDeltaAgainstConstraintsBody(Storage,DeltaLinear);
}


template<class TAccess>
void ISpeedComponent::ProjectAngularDeltaAgainstConstraintsBody(TAccess& Storage,FVector& DeltaAngular) const
{
	if (DeltaAngular.IsNearlyZero())
	{
		return;
	}

	const FVector COM = Storage.COM();
	for (int32 PassIdx = 0; PassIdx < ConstraintProjectionPasses; ++PassIdx)
	{
		bool bChanged = false;
		for (const FPhysicalContactConstraint& Constraint : Storage.Constraints())
		{
			if (!Storage.Relevant(Constraint))
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

void ISpeedComponent::ProjectAngularDeltaAgainstConstraints(FVector& DeltaAngular) const
{
    FLegacyConstraintAccess Storage(*this);
    ProjectAngularDeltaAgainstConstraintsBody(Storage,DeltaAngular);
}

void ISpeedComponent::ProjectAngularDeltaAgainstConstraintsNative(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,FVector& DeltaAngular) const
{
    FNativeConstraintAccess Storage(*this,Operation);
    ProjectAngularDeltaAgainstConstraintsBody(Storage,DeltaAngular);
}


template<class TAccess>
void ISpeedComponent::ProjectPointDeltaAgainstConstraintsBody(TAccess& Storage,FVector& DeltaLinear,FVector& DeltaAngular) const
{
	if (DeltaLinear.IsNearlyZero() && DeltaAngular.IsNearlyZero())
	{
		return;
	}

	const FVector COM = Storage.COM();

	for (int32 PassIdx = 0; PassIdx < ConstraintProjectionPasses; ++PassIdx)
	{
		bool bChanged = false;

		for (const FPhysicalContactConstraint& Constraint : Storage.Constraints())
		{
			if (!Storage.Relevant(Constraint))
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

void ISpeedComponent::ProjectPointDeltaAgainstConstraints(FVector& DeltaLinear, FVector& DeltaAngular) const
{
    FLegacyConstraintAccess Storage(*this);
    ProjectPointDeltaAgainstConstraintsBody(Storage,DeltaLinear,DeltaAngular);
}

void ISpeedComponent::ProjectPointDeltaAgainstConstraintsNative(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,FVector& DeltaLinear,FVector& DeltaAngular) const
{
    FNativeConstraintAccess Storage(*this,Operation);
    ProjectPointDeltaAgainstConstraintsBody(Storage,DeltaLinear,DeltaAngular);
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

// Private static bridge: this existing friend may call the private reader.
// The caller passes the actual raw interface address; validation itself uses
// the unchanged two typed Parts before any target dereference or mutation.
// No operation issuer, virtual callback or additional friendship is introduced.
void ISpeedComponent::ValidateNativeFrameOwner(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const ISpeedComponent* Component)
{Speed::NativeKinematics::FComponentRead::ValidateCurrentAndSubBodies(Operation,Component);}
