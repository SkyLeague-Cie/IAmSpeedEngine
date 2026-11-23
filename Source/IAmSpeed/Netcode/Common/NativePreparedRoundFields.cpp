#include "NativePreparedRoundFields.h"

#include "IAmSpeed/Base/SpeedConstant.h"
#include "IAmSpeed/Components/SpeedMovementComponent.h"
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "IAmSpeed/SubBodies/Sensor/SensorSubBody.h"
#include "SuspensionSystem.h"
#include "WheelSystem.h"
#include <functional>
#include <type_traits>

namespace Speed
{
namespace
{
PreparedAccess::FSpan Range(const void* Address, std::size_t Bytes)
{
    return {reinterpret_cast<std::uintptr_t>(Address), Bytes};
}

bool ValidTimeline(const FSimulationComponentRestoreRequest& Request,
    const FPreparedNativeRoundFields::FActorTimeline& Values)
{
    return Request.Address.Frame < uint64(MAX_int32)
        && Values.SinceCanMoveFrame >= INDEX_NONE
        && int64(Values.SinceCanMoveFrame) <= int64(Request.Address.Frame) + 1
        && (!Values.bBaseCountdownStarted || Values.BaseFramesBeforeCanMove != 0)
        && (Values.History == FPreparedNativeRoundFields::EHistory::Empty
            || Values.History == FPreparedNativeRoundFields::EHistory::SeedRestoredFrame);
}

FBasePhysicsState BaseFrame(const FSimulationComponentRestoreRequest& Request,
    const FPreparedNativeRoundFields::FActorTimeline& Values)
{
    FBasePhysicsState Result;
    Result.Kinematic = Request.Kinematic;
    Result.bIsFrozen = Request.bFrozen;
    Result.nbFramesbeforeCanMove = Values.BaseFramesBeforeCanMove;
    Result.bStartCountdown = Values.bBaseCountdownStarted;
    return Result;
}

bool ValidVehicle(const FPreparedNativeRoundFields::FVehicleState& Values)
{
    const auto& P = Values.Physics;
    const auto& Input = Values.PhysicalInput;
    if (!P.LastCarBumpVictim.IsExplicitlyNull() || P.CarBumpCooldownFrames != 0
        || P.AllowedSideVelocity.ContainsNaN() || P.AllowedAngularVelocity.ContainsNaN()
        || (P.bStartCountdown && P.nbFramesbeforeCanMove == 0)
        || Input.Throttle != P.PhysicalThrottleInput || Input.Brake != P.PhysicalBrakeInput
        || Input.Steer != P.PhysicalSteerInput || Input.Steer == -128 || !Input.Camera.IsValid()
        || Values.Game.NbWheelsOnGround > 4) return false;
    for (int32 I = 0; I < 4; ++I)
        if (!FMath::IsFinite(P.SuspensionLastDisplacement[I])
            || !FMath::IsFinite(Values.Game.WheelsOmega[I])
            || !FMath::IsFinite(Values.Game.WheelsAngularPosition[I])) return false;
    return true;
}

bool ValidWheel(const FPreparedNativeRoundFields::FWheelState& Values)
{
    const auto& Render = Values.Render;
    return FMath::IsFinite(Values.SuspensionLengthCm) && Values.SuspensionLengthCm >= 0.0f
        && FMath::IsFinite(Values.LastDisplacementCm) && FMath::IsFinite(Values.AngularVelocity)
        && FMath::IsFinite(Values.RollAngle) && FMath::IsFinite(Values.SteeringAngle)
        && FMath::IsFinite(Render.SpringOffset) && Render.RollRotation.IsNormalized()
        && Render.SteerRotation.IsNormalized() && !Render.WorldPosition.ContainsNaN()
        && Render.RightAxis.IsNormalized() && Render.UpAxis.IsNormalized() && Render.ForwardAxis.IsNormalized();
}
}

struct FPreparedNativeRoundFields::FPlan
{
    enum class EPhase { Prepared, Ready, Finalized, Released, Aborted };
    const FSimulationWorld* const World;
    const PreparedWorld::FAddress Address;
    const FPreparedRestoreLeaseView Lease;
    EPhase Phase = EPhase::Prepared;
    PreparedAccess::FParticipantAccess Access;
    std::vector<std::function<void()>> Exchanges;

    FPlan(const FSimulationWorld& InWorld, const FSimulationComponentRestoreRequest& Request)
        : World(&InWorld), Address(Request.Address), Lease(Request.OwnerLease)
    {
        Access.Participant = reinterpret_cast<std::uintptr_t>(this);
        Access.Writes.push_back(Range(&Phase, sizeof(Phase)));
        Access.ReadOnly = {Range(&World, sizeof(World)), Range(&Address, sizeof(Address)), Range(&Lease, sizeof(Lease))};
    }

    template<class T>
    void Stage(T& Live, T Value)
    {
        static_assert(TUseBitwiseSwap<T>::Value || std::is_nothrow_move_assignable_v<T>);
        auto Staged = std::make_shared<T>(std::move(Value));
        Access.Writes.push_back(Range(&Live, sizeof(Live)));
        Access.Writes.push_back(Range(Staged.get(), sizeof(T)));
        Exchanges.emplace_back([&Live, Staged]() noexcept { ::Swap<T>(Live, *Staged); });
    }

    template<class T>
    void StageArray(TArray<T>& Live, TArray<T> Value)
    {
        static_assert(TUseBitwiseSwap<TArray<T>>::Value);
        if (Live.GetAllocatedSize()) Access.ReadOnly.push_back(Range(Live.GetData(), Live.GetAllocatedSize()));
        if (Value.GetAllocatedSize()) Access.ReadOnly.push_back(Range(Value.GetData(), Value.GetAllocatedSize()));
        Stage(Live, std::move(Value));
    }

    template<class T>
    void StageHistory(TArray<T>& Live, int32 LocalFrame, bool Seed, const T& Frame)
    {
        TArray<T> Replacement;
        Replacement.SetNum(SpeedConstants::RecordedHistorySize);
        if (Seed) Replacement[LocalFrame % SpeedConstants::RecordedHistorySize] = Frame;
        StageArray(Live, std::move(Replacement));
    }

    void StageBase(SBaseGameState& Game, FBasePhysicsState& Physics, int32& SinceCanMove,
        TArray<int32>& RecordedFrames, TArray<FBasePhysicsState>& RecordedStates, const FActorTimeline& Values,
        const FSimulationComponentRestoreRequest& Request)
    {
        const int32 LocalFrame = static_cast<int32>(Address.Frame + 1);
        SBaseGameState BaseGame;
        BaseGame.NumFrame = static_cast<uint32>(LocalFrame);
        Stage(Game, BaseGame);
        Stage(Physics.nbFramesbeforeCanMove, Values.BaseFramesBeforeCanMove);
        Stage(Physics.bStartCountdown, Values.bBaseCountdownStarted);
        Stage(SinceCanMove, Values.SinceCanMoveFrame);
        const bool Seed = Values.History == EHistory::SeedRestoredFrame;
        TArray<int32> Frames;
        Frames.Init(INDEX_NONE, SpeedConstants::RecordedHistorySize);
        if (Seed) Frames[LocalFrame % SpeedConstants::RecordedHistorySize] = LocalFrame;
        StageArray(RecordedFrames, std::move(Frames));
        StageHistory(RecordedStates, LocalFrame, Seed, BaseFrame(Request, Values));
    }

    bool HoldsBoundary() const noexcept { return Lease.HoldsExactBoundary(*World, Address); }
};

FPreparedNativeRoundFields::FPreparedNativeRoundFields(const FSimulationWorld& World,
    const FSimulationComponentRestoreRequest& Request) : Plan(std::make_unique<FPlan>(World, Request)) {}

std::unique_ptr<FPreparedNativeRoundFields> FPreparedNativeRoundFields::Prepare(USpeedMovementComponent& Target,
    const FSimulationComponentRestoreRequest& Request, const FActorTimeline& Values)
{
    const auto* World = Request.OwnerLease.ResolvePreparingWorld(Request, Request.Address);
    if (!World || !FPreparedNativeKinematicFields::MatchesActor(*World, Request, Target)
        || Target.BaseGameState.bHasTestVelocity || Target.bNetCorrHasTarget || !ValidTimeline(Request, Values)) return {};

    auto Fragment = std::unique_ptr<FPreparedNativeRoundFields>(new FPreparedNativeRoundFields(*World, Request));
    Fragment->Plan->StageBase(Target.BaseGameState, Target.BasePhysicsState, Target.SinceCanMoveFrame,
        Target.RecordedBaseFrames, Target.RecordedBaseStates, Values, Request);
    return Fragment;
}

std::unique_ptr<FPreparedNativeRoundFields> FPreparedNativeRoundFields::Prepare(USpeedWheeledComponent& Target,
    const FSimulationComponentRestoreRequest& Request, const FVehicleState& Values)
{
    const auto* World = Request.OwnerLease.ResolvePreparingWorld(Request, Request.Address);
    if (!World || !FPreparedNativeKinematicFields::MatchesActor(*World, Request, Target)
        || Target.BaseGameState.bHasTestVelocity || Target.bNetCorrHasTarget || Target.NetCorrAccum.bActive
        || Target.bCanonicalInputPending || Target.InputReservationV2 || Target.PreparedWheeledNetworkInput
        || !ValidTimeline(Request, Values.Timeline) || !ValidVehicle(Values)) return {};

    auto Fragment = std::unique_ptr<FPreparedNativeRoundFields>(new FPreparedNativeRoundFields(*World, Request));
    auto& P = *Fragment->Plan;
    P.StageBase(Target.BaseGameState, Target.BasePhysicsState, Target.SinceCanMoveFrame,
        Target.RecordedBaseFrames, Target.RecordedBaseStates, Values.Timeline, Request);
    P.Stage(Target.WheeledGameState, Values.Game);
    P.Stage(Target.WheeledPhysicsState, Values.Physics);
    P.Stage(Target.WheeledPhysicalInput, Values.PhysicalInput);
    P.Stage(Target.bCanonicalSupportStartupInitializationRequested, Values.bSupportStartupRequested);
    P.Stage(Target.bCanonicalSupportStartupPoseConfirmed, Values.bSupportStartupConfirmed);
    P.Stage(Target.bSimTimelineWasCanMove, Values.bTimelineWasCanMove);
    P.Stage(Target.bSimTimelineHasGroundState, Values.bTimelineHasGroundState);
    P.Stage(Target.bSimTimelineWasGrounded, Values.bTimelineWasGrounded);
    P.StageHistory(Target.RecordedWheeledStates, static_cast<int32>(Request.Address.Frame + 1),
        Values.Timeline.History == EHistory::SeedRestoredFrame, Values.Physics);
    P.StageArray(Target.PendingWheelContacts, {});
    return Fragment;
}

void FPreparedNativeRoundFields::StageClearContacts(USSubBody& Target)
{
    auto& P = *Plan;
    for (auto* Hit : {&Target.CurrentHit, &Target.FutureHit, &Target.GroundHit, &Target.BoxHit, &Target.SphereHit, &Target.WheelHit})
        P.Stage(*Hit, SHitResult{});
    P.StageArray(Target.IgnoredComponents, Target.AlwaysIgnoredComponents);
}

std::unique_ptr<FPreparedNativeRoundFields> FPreparedNativeRoundFields::PrepareClearSolidContacts(USolidSubBody& Target,
    const FSimulationComponentRestoreRequest& Request, const PreparedWorld::FEndpointBinding& Endpoint)
{
    const auto* World = Request.OwnerLease.ResolvePreparingWorld(Request, Request.Address);
    if (!World || !FPreparedNativeKinematicFields::MatchesSolid(*World, Request, Target, Endpoint)) return {};
    auto Fragment = std::unique_ptr<FPreparedNativeRoundFields>(new FPreparedNativeRoundFields(*World, Request));
    Fragment->StageClearContacts(Target);
    return Fragment;
}

std::unique_ptr<FPreparedNativeRoundFields> FPreparedNativeRoundFields::PrepareClearSensorOverlaps(USensorSubBody& Target,
    const FSimulationComponentRestoreRequest& Request, const FPreparedNativeKinematicFields::FRoster::FSubBody& Sensor)
{
    const auto* World = Request.OwnerLease.ResolvePreparingWorld(Request, Request.Address);
    if (!World || !FPreparedNativeKinematicFields::MatchesSensor(*World, Request, Target, Sensor)
        || static_cast<std::size_t>(Target.AlwaysIgnoredComponents.Num()) > PreparedWorld::FLimits{}.MaxEndpoints)
        return {};

    auto Fragment = std::unique_ptr<FPreparedNativeRoundFields>(new FPreparedNativeRoundFields(*World, Request));
    Fragment->StageClearContacts(Target);
    auto& P = *Fragment->Plan;
    P.StageArray(Target.OverlapsOrdered, {});
    // Exchange the set headers only. Old entries and buckets remain owned by
    // the staged sets until the plan is released, without traversing weak refs.
    static_assert(TUseBitwiseSwap<decltype(Target.OverlapsSet)>::Value);
    P.Stage(Target.OverlapsSet, {});
    P.Stage(Target.TouchedThisFrame, {});

    auto& K = P.Access.ReadOnly;
    K.push_back(Range(&Target.OnBeginOverlap, sizeof(Target.OnBeginOverlap)));
    K.push_back(Range(&Target.OnEndOverlap, sizeof(Target.OnEndOverlap)));
    K.push_back(Range(&Target.OnContinuousOverlap, sizeof(Target.OnContinuousOverlap)));
    K.push_back(Range(&Target.BeginOverlapHandle, sizeof(Target.BeginOverlapHandle)));
    K.push_back(Range(&Target.EndOverlapHandle, sizeof(Target.EndOverlapHandle)));
    K.push_back(Range(&Target.ContinuousOverlapHandle, sizeof(Target.ContinuousOverlapHandle)));
    return Fragment;
}

std::unique_ptr<FPreparedNativeRoundFields> FPreparedNativeRoundFields::PrepareWheel(USWheelSubBody& Target,
    const FSimulationComponentRestoreRequest& Request, const PreparedWorld::FEndpointBinding& Endpoint, const FWheelState& Values)
{
    const auto* World = Request.OwnerLease.ResolvePreparingWorld(Request, Request.Address);
    if (!World || !FPreparedNativeKinematicFields::MatchesSolid(*World, Request, Target, Endpoint)
        || !Target.PWheel || !Target.PSuspension || Target.Idx() < 0 || !ValidWheel(Values)) return {};

    auto& LiveSuspension = *Target.PSuspension;
    auto& LiveWheel = *Target.PWheel;
    const float Radius = LiveWheel.GetEffectiveRadius();
    const float TraceLength = Values.SuspensionLengthCm + LiveSuspension.Setup().RaycastSafetyMargin + Radius;
    const float SteeringDegrees = FMath::RadiansToDegrees(Values.SteeringAngle);
    if (!FMath::IsFinite(Radius) || Radius <= 0.0f || !FMath::IsFinite(TraceLength)
        || !FMath::IsFinite(SteeringDegrees)) return {};

    Chaos::FSimpleSuspensionSim Suspension(&LiveSuspension.Setup());
    Suspension.SetLocalRestingPosition(LiveSuspension.GetLocalRestingPosition());
    Suspension.SetSpringIndex(static_cast<uint32>(Target.Idx()));
    Suspension.SetSuspensionLength(TraceLength, Radius);
    Suspension.SetLastDisplacement(Values.LastDisplacementCm);
    Suspension.SetLastSpringLength(-Values.SuspensionLengthCm);
    auto Wheel = LiveWheel;
    Wheel.SetOnGround(false);
    Wheel.SetAngularPosition(Values.RollAngle);
    Wheel.SetAngularVelocity(Values.AngularVelocity);
    Wheel.SetSteeringAngle(SteeringDegrees);
    Wheel.SetWheelLoadForce(0.0f);
    Wheel.SetVehicleGroundSpeed(FVector::ZeroVector);
    Wheel.SetDriveTorque(0.0f);
    Wheel.SetBrakeTorque(0.0f);
    Wheel.SetDriveTorqueOverride(0.0f);
    Wheel.SetBrakeTorqueOverride(0.0f);
    Wheel.SetSurfaceFriction(1.0f);
    Wheel.Sx = 0.0f;
    Wheel.SlipAngle = 0.0f;
    Wheel.SlipVelocity = 0.0f;
    Wheel.ForceFromFriction = FVector::ZeroVector;
    Wheel.AppliedLinearDriveForce = 0.0f;
    Wheel.AppliedLinearBrakeForce = 0.0f;
    Wheel.LongitudinalAdhesiveLimit = 0.0f;
    Wheel.LateralAdhesiveLimit = 0.0f;
    Wheel.Spin = 0.0f;
    Wheel.AvailableGrip = 0.0f;
    Wheel.InputForces = FVector::ZeroVector;
    Wheel.bClipping = false;
    Wheel.bABSActivated = false;

    auto Fragment = std::unique_ptr<FPreparedNativeRoundFields>(new FPreparedNativeRoundFields(*World, Request));
    Fragment->StageClearContacts(Target);
    auto& P = *Fragment->Plan;
    P.Stage(LiveSuspension, std::move(Suspension));
    P.Stage(LiveWheel, std::move(Wheel));
    P.Stage(Target.SuspensionForce, 0.0f);
    P.Stage(Target.bIsJumping, Values.JumpFrames);
    P.Stage(Target.bJumpUnilateralSupport, Values.bJumpUnilateralSupport);
    P.Stage(Target.ConsecutiveGroundFrames, uint8{0});
    P.Stage(Target.SteeringAngle, Values.SteeringAngle);
    P.Stage(Target.Omega, Values.AngularVelocity);
    P.Stage(Target.RollAngle, Values.RollAngle);
    P.Stage(Target.bWasOnGroundPrevFrame, false);
    P.Stage(Target.bCountedGroundFrame, false);
    P.Stage(Target.bContactVelocityLocked, false);
    P.Stage(Target.bCrossedIntoCompressionThisFrame, false);
    P.Stage(Target.ForwardAxis, Values.Render.ForwardAxis);
    P.Stage(Target.RightAxis, Values.Render.RightAxis);
    P.Stage(Target.UpAxis, Values.Render.UpAxis);
    P.Stage(Target.RenderData, Values.Render);
    P.Stage(Target.PrimarySuspensionMiss, {});
    P.Stage(Target.bObservePrimarySuspension, false);
    return Fragment;
}

FPreparedNativeRoundFields::~FPreparedNativeRoundFields() noexcept
{
    if (Plan->Phase == FPlan::EPhase::Finalized) std::terminate();
}

bool FPreparedNativeRoundFields::AppendPreparedAccess(std::vector<PreparedAccess::FParticipantAccess>& Out) const
{
    if ((Plan->Phase != FPlan::EPhase::Prepared && Plan->Phase != FPlan::EPhase::Ready) || !Plan->HoldsBoundary()) return false;
    Out.push_back(Plan->Access);
    return true;
}

bool FPreparedNativeRoundFields::PrepareFinalization() noexcept
{
    if ((Plan->Phase != FPlan::EPhase::Prepared && Plan->Phase != FPlan::EPhase::Ready) || !Plan->HoldsBoundary()) return false;
    Plan->Phase = FPlan::EPhase::Ready;
    return true;
}

void FPreparedNativeRoundFields::Finalize() noexcept
{
    if (Plan->Phase != FPlan::EPhase::Ready) std::terminate();
    for (const auto& Exchange : Plan->Exchanges) Exchange();
    Plan->Phase = FPlan::EPhase::Finalized;
}

void FPreparedNativeRoundFields::AbortBeforeFinalize() noexcept
{
    if (Plan->Phase == FPlan::EPhase::Aborted) return;
    if (Plan->Phase != FPlan::EPhase::Prepared && Plan->Phase != FPlan::EPhase::Ready) std::terminate();
    Plan->Phase = FPlan::EPhase::Aborted;
}

void FPreparedNativeRoundFields::ReleaseAfterPublication() noexcept
{
    if (Plan->Phase != FPlan::EPhase::Finalized) std::terminate();
    Plan->Phase = FPlan::EPhase::Released;
}
}
