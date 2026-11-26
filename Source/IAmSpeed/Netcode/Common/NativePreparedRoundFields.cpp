#include "NativePreparedRoundFields.h"

#include "IAmSpeed/Base/CarBumpSnapshot.h"
#include "IAmSpeed/Base/OriginKinematicStateValues.h"
#include "IAmSpeed/Base/SpeedConstant.h"
#include "IAmSpeed/Components/SpeedMovementComponent.h"
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "IAmSpeed/SubBodies/Sensor/SensorSubBody.h"
#include "IAmSpeed/SubBodies/Solid/BoxSubBody.h"
#include "IAmSpeed/World/Simulation/ExactRemoteRoundReset.h"
#include "IAmSpeed/World/Simulation/SimulationWorld.h"
#include "IAmSpeed/World/Simulation/SpeedPhysicsSimulation.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "SuspensionSystem.h"
#include "WheelSystem.h"
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

bool ValidPose(const SKinematic& Pose)
{
    return !Pose.Location.ContainsNaN() && !Pose.Velocity.ContainsNaN() && !Pose.Acceleration.ContainsNaN()
        && !Pose.AngularVelocity.ContainsNaN() && !Pose.AngularAcceleration.ContainsNaN()
        && !Pose.Rotation.ContainsNaN() && Pose.Rotation.IsNormalized();
}

class FSuspensionLengthSample final : public Chaos::FSimpleSuspensionSim
{
public:
    explicit FSuspensionLengthSample(const Chaos::FSimpleSuspensionSim& Source)
        : Chaos::FSimpleSuspensionSim(Source) {}
    float LengthCm() const { return DisplacementInput; }
};
}

bool FPreparedNativeRoundFields::MatchesParkedKickoffDriverGT(
    const ASpeedPhysicsSimulation& Driver, uint64 ExpectedGeneration, uint64 ExpectedFirstFrame)
{
    return IsInGameThread() && ExpectedGeneration && Driver.IsOwnedWorkerExecutionMode()
        && Driver.OwnedBoundarySuspendDepth && Driver.SimulationWorker
        && Driver.SimulationWorker->IsBoundaryServiceSuspended() && Driver.SimulationWorker->IsPaused()
        && Driver.bOwnedSimulationPaused.Load() && !Driver.bOwnedWorkerTerminal.Load()
        && !Driver.bCanonicalPublicationTerminal.Load() && !Driver.bInputOwnerRetired.Load()
        && Driver.InputWorkerGeneration == ExpectedGeneration && Driver.bCanonicalFrameInitialized
        && Driver.CanonicalNumFrame == ExpectedFirstFrame && Driver.SnapshotBuffer.PublishedSerial() == 0;
}

bool FPreparedNativeRoundFields::ReadWheelValues(const USWheelSubBody& Wheel, FCapturedKickoffWheel& Out)
{
    Out.Pose = Wheel.GetKinematicState();
    Out.LocalOffset = Wheel.GetLocalOffset();
    Out.LocalRotation = Wheel.GetLocalRotation();
    Out.MaxSuspensionLengthCm = Wheel.MaxLength();
    Out.SuspensionRestRaiseCm = Wheel.SuspensionRestLength();
    // GetSpringLength advances the smoothing history on the live suspension.
    Out.State = {FSuspensionLengthSample(*Wheel.PSuspension).LengthCm(), Wheel.GetLastDisplacement(),
        Wheel.Omega, Wheel.RollAngle, Wheel.SteeringAngle, Wheel.bIsJumping,
        Wheel.bJumpUnilateralSupport, Wheel.RenderData};
    return ValidPose(Out.Pose) && ValidWheel(Out.State);
}

std::optional<uint16> FPreparedNativeRoundFields::ReadMovementMinimumFramesAtParkedStartupGT(
    const ASpeedPhysicsSimulation& Driver, uint64 ExpectedGeneration, uint64 ExpectedFirstFrame,
    uint64 ExpectedActorStableId, const USpeedMovementComponent& Target)
{
    if (!MatchesParkedKickoffDriverGT(Driver, ExpectedGeneration, ExpectedFirstFrame)
        || !Driver.ExactRemoteRoundProvider || !ExpectedActorStableId || !IsValid(&Target)
        || !Target.IsRegistered() || Target.IsPresentationOnly() || !IsValid(Target.GetOwner())) return {};

    const auto* World = Driver.GetWorld();
    const auto* Subsystem = Driver.SpeedWorldSubsystem;
    if (!World || World->bIsTearingDown || Target.GetWorld() != World
        || !Subsystem || Subsystem->GetWorld() != World) return {};

    std::optional<uint16> Result;
    Subsystem->VisitBorrowedSimulationWorld([&](const FSimulationWorld& NativeWorld)
    {
        const auto Origin = NativeWorld.ReadExactRoundOrigin();
        if (!Origin.Valid() || Origin.Authority != ExpectedGeneration
            || NativeWorld.bOrderDirty || NativeWorld.IsRestoreBoundaryReserved()
            || Target.GetPublishedSimulationStableId() != ExpectedActorStableId
            || NativeWorld.AdaptersByStableId.FindRef(ExpectedActorStableId) != &Target
            || NativeWorld.FindStableId(Target) != ExpectedActorStableId) return;

        Result = Target.MinNbFramesBeforeCanMove;
    });
    return Result;
}

std::optional<FPreparedNativeRoundFields::FCapturedKickoffVehicle> FPreparedNativeRoundFields::CaptureKickoffVehicleGT(
    const ASpeedPhysicsSimulation& Driver, uint64 ExpectedGeneration, uint64 ExpectedFirstFrame,
    const USpeedWheeledComponent& Target)
{
    if (!MatchesParkedKickoffDriverGT(Driver, ExpectedGeneration, ExpectedFirstFrame)
        || !IsValid(&Target) || !Target.IsRegistered() || !Target.GetOwner() || Target.GetWorld() != Driver.GetWorld()
        || Target.WheelSubBodies.Num() != 4) return {};

    const auto* Subsystem = Driver.SpeedWorldSubsystem;
    if (!Subsystem || Subsystem->GetWorld() != Driver.GetWorld()) return {};

    std::optional<FCapturedKickoffVehicle> Result;
    Subsystem->VisitBorrowedSimulationWorld([&](const FSimulationWorld& World)
    {
        const uint64 ActorId = Target.GetPublishedSimulationStableId();
        if (!ActorId || World.bOrderDirty || World.IsRestoreBoundaryReserved()
            || World.AdaptersByStableId.FindRef(ActorId) != &Target || World.FindStableId(Target) != ActorId)
            return;

        FCapturedKickoffVehicle Capture;
        Capture.WorkerGeneration = ExpectedGeneration;
        Capture.FirstFrame = ExpectedFirstFrame;
        Capture.ActorStableId = ActorId;
        Capture.ActorNativeObjectId = Target.GetUniqueID();
        Capture.CenterOfMass = Target.BasePhysicsState.Kinematic;
        Capture.LocalCenterOfMass = Target.GetPhysCenterOfMassLocal();
        Capture.bFrozen = Target.BasePhysicsState.bIsFrozen;
        Capture.State.Timeline = {Target.SinceCanMoveFrame, Target.BasePhysicsState.nbFramesbeforeCanMove,
            Target.BasePhysicsState.bStartCountdown, EHistory::Empty};
        Capture.State.Game = Target.WheeledGameState;
        Capture.State.Physics = Target.WheeledPhysicsState;
        Capture.State.PhysicalInput = Target.WheeledPhysicalInput;
        Capture.State.bSupportStartupRequested = Target.bCanonicalSupportStartupInitializationRequested;
        Capture.State.bSupportStartupConfirmed = Target.bCanonicalSupportStartupPoseConfirmed;
        Capture.State.bTimelineWasCanMove = Target.bSimTimelineWasCanMove;
        Capture.State.bTimelineHasGroundState = Target.bSimTimelineHasGroundState;
        Capture.State.bTimelineWasGrounded = Target.bSimTimelineWasGrounded;
        if (!Capture.ActorNativeObjectId || !ValidPose(Capture.CenterOfMass)
            || Capture.LocalCenterOfMass.ContainsNaN() || !Capture.State.Physics.LastCarBumpVictim.IsExplicitlyNull())
            return;

        const auto& Children = Target.GetSubBodies();
        for (int32 Index = 0; Index < 4; ++Index)
        {
            const auto* Wheel = Target.WheelSubBodies[Index].Get();
            if (!IsValid(Wheel) || Wheel->Idx() != Index || Wheel->GetParentComponent() != &Target
                || !Children.Contains(Wheel) || !Wheel->IsRegistered() || Wheel->GetWorld() != Target.GetWorld()
                || Wheel->GetOwner() != Target.GetOwner() || !Wheel->PSuspension || !Wheel->PWheel || !Wheel->ChaosWheel)
                return;
            const uint64 StableId = World.StableSubBodyIds.FindRef(Wheel);
            const uint32 NativeId = Wheel->GetUniqueID();
            if (!StableId || !NativeId || World.SubBodyNativeObjectIds.FindRef(StableId) != NativeId
                || World.SubBodiesByStableId.FindRef(StableId) != Wheel
                || World.SolidSubBodiesByStableId.FindRef(StableId) != Wheel) return;

            auto& Out = Capture.Wheels[Index];
            Out.Binding = {StableId, ActorId, NativeId, true};
            Out.Binding.Incarnation = World.ReadExactRoundSubBodyIncarnation(StableId, *Wheel);
            if (World.ExactRoundWorkerGeneration && !Out.Binding.Incarnation) return;
            if (!ReadWheelValues(*Wheel, Out)) return;
        }
        Result = std::move(Capture);
    });
    return Result;
}

std::optional<FPreparedNativeRoundFields::FCapturedRoundWheels> FPreparedNativeRoundFields::CaptureRoundWheels(
    const FSimulationWorld& World, const FCompletedSimulationPublication& Current, uint64 ActorStableId,
    const std::array<FPreparedNativeKinematicFields::FRoster::FSubBody, 4>& ExpectedWheels)
{
    const auto Address = FExactRemoteRoundBuild::CaptureNativeAddress(World, Current);
    const uint64 StateHash = Current.World().StateHash;
    if (!Address || World.GetCurrentStepFrame() != Address->Frame || !StateHash
        || !ActorStableId || ActorStableId > (UINT64_MAX >> 16)) return {};

    auto* Adapter = World.AdaptersByStableId.FindRef(ActorStableId);
    const auto* Target = Adapter ? Adapter->AsWheeledPhysicsComponent() : nullptr;
    if (!IsValid(Target) || static_cast<const ISpeedComponent*>(Target) != Adapter || Target->IsPresentationOnly()
        || !Target->IsRegistered() || !IsValid(Target->GetOwner()) || !Target->GetWorld()
        || World.FindStableId(*Target) != ActorStableId || Target->GetPublishedSimulationStableId() != ActorStableId
        || !Target->GetUniqueID() || Target->WheelSubBodies.Num() != 4) return {};

    FString Reason;
    if (!Target->USpeedWheeledComponent::ValidateSimulationBindings(Reason)) return {};

    FCapturedRoundWheels Capture{*Address, StateHash, ActorStableId, Target->GetUniqueID(), {}};
    const auto& Children = Target->USpeedWheeledComponent::GetSubBodies();
    for (int32 Ordinal = 0; Ordinal < 4; ++Ordinal)
    {
        const auto& Expected = ExpectedWheels[Ordinal];
        const auto Slot = static_cast<uint16>(Expected.StableId & MAX_uint16);
        const auto* Wheel = Target->WheelSubBodies[Ordinal].Get();
        if (!IsValid(Wheel) || Wheel->GetSubBodyType() != USSubBody::ESubBodyType::Wheel
            || Wheel->IsPresentationOnly() || !Wheel->IsRegistered() || Wheel->GetWorld() != Target->GetWorld()
            || Wheel->GetOwner() != Target->GetOwner() || !Slot || !Children.IsValidIndex(Slot)
            || Children[Slot] != Wheel || !Expected.IsSolidEndpoint) return {};
        const auto Row = FPreparedNativeKinematicFields::ReadSubBodyRow(World, *Target, ActorStableId, Slot, *Wheel);
        if (!Row || !(*Row == Expected)) return {};
        auto& Out = Capture.Wheels[Ordinal];
        Out.Binding = *Row;
        if (!ReadWheelValues(*Wheel, Out) || Out.LocalOffset.ContainsNaN() || !Out.LocalRotation.IsNormalized()
            || !FMath::IsFinite(Out.MaxSuspensionLengthCm) || !FMath::IsFinite(Out.SuspensionRestRaiseCm)) return {};
    }
    const auto RetainedAddress = FExactRemoteRoundBuild::CaptureNativeAddress(World, Current);
    if (!RetainedAddress || !(*RetainedAddress == *Address)) return {};
    return Capture;
}

std::optional<FPreparedNativeRoundFields::FContactFreeKickoff> FPreparedNativeRoundFields::ProjectContactFreeKickoff(
    const FCapturedKickoffVehicle& Captured, const SKinematic& Target, bool bRequestStartupSupport)
{
    if (!Captured.WorkerGeneration || !Captured.ActorStableId || !Captured.ActorNativeObjectId
        || Captured.LocalCenterOfMass.ContainsNaN() || !ValidPose(Target)
        || !Target.Velocity.IsZero() || !Target.Acceleration.IsZero()
        || !Target.AngularVelocity.IsZero() || !Target.AngularAcceleration.IsZero()) return {};

    FContactFreeKickoff Result;
    auto& Vehicle = Result.Vehicle;
    Vehicle.Timeline = Captured.State.Timeline;
    Vehicle.Physics.nbFramesbeforeCanMove = Captured.State.Physics.nbFramesbeforeCanMove;
    Vehicle.Physics.bStartCountdown = Captured.State.Physics.bStartCountdown;
    Vehicle.bSupportStartupRequested = bRequestStartupSupport;
    const auto Origin = CalculateOriginKinematicStateValues(Target, Captured.LocalCenterOfMass);
    for (std::size_t Index = 0; Index < Result.Wheels.size(); ++Index)
    {
        const auto& Seed = Captured.Wheels[Index];
        auto& Wheel = Result.Wheels[Index];
        if (Seed.Binding.ActorStableId != Captured.ActorStableId || !Seed.Binding.StableId || !Seed.Binding.NativeObjectId
            || !Seed.Binding.IsSolidEndpoint || Seed.LocalOffset.ContainsNaN() || Seed.LocalRotation.ContainsNaN()
            || !Seed.LocalRotation.IsNormalized() || !FMath::IsFinite(Seed.MaxSuspensionLengthCm)
            || Seed.MaxSuspensionLengthCm <= 0.0f || !FMath::IsFinite(Seed.SuspensionRestRaiseCm)
            || Seed.SuspensionRestRaiseCm < 0.0f || !FMath::IsFinite(Seed.State.RollAngle)) return {};
        for (std::size_t Prior = 0; Prior < Index; ++Prior)
            if (Result.Wheels[Prior].Binding.StableId == Seed.Binding.StableId
                || Result.Wheels[Prior].Binding.NativeObjectId == Seed.Binding.NativeObjectId) return {};

        Wheel.Binding = Seed.Binding;
        Wheel.Pose.Location = Origin.Location + Origin.Rotation.RotateVector(Seed.LocalOffset);
        Wheel.Pose.Rotation = Origin.Rotation * Seed.LocalRotation;
        // Full extension is the no-contact default. The old slot's compression,
        // hit and velocity are not support samples for the target slot.
        Wheel.State.SuspensionLengthCm = Seed.MaxSuspensionLengthCm;
        Wheel.State.RollAngle = Seed.State.RollAngle;
        auto& Render = Wheel.State.Render;
        Render.WorldPosition = Wheel.Pose.Location;
        Render.UpAxis = Origin.Rotation.GetUpVector();
        Render.ForwardAxis = Origin.Rotation.GetForwardVector().GetSafeNormal();
        Render.RightAxis = FVector::CrossProduct(Render.UpAxis, Render.ForwardAxis).GetSafeNormal();
        Render.RollRotation = FQuat(Render.RightAxis, Wheel.State.RollAngle);
        Render.SpringOffset = Seed.SuspensionRestRaiseCm + Wheel.State.SuspensionLengthCm;
        Vehicle.Game.WheelsAngularPosition[Index] = Wheel.State.RollAngle;
        if (!ValidPose(Wheel.Pose) || !ValidWheel(Wheel.State)) return {};
    }
    return ValidVehicle(Vehicle) ? std::optional<FContactFreeKickoff>{std::move(Result)} : std::nullopt;
}

struct FPreparedNativeRoundFields::FPlan
{
    enum class EPhase { Prepared, Ready, Finalized, Released, Aborted };
    using FOwnedValue = std::unique_ptr<void, void(*)(void*) noexcept>;
    struct FExchange
    {
        void* Live;
        FOwnedValue Staged;
        void (*Swap)(void*, void*) noexcept;
        void Apply() const noexcept { Swap(Live, Staged.get()); }
    };
    using FOverlapToken = TWeakObjectPtr<UPrimitiveComponent>;
    struct FKeptSensorOverlaps
    {
        struct FTarget { FOverlapToken Token; PreparedWorld::FEndpointBinding Binding; };
        struct FSetCell { const FOverlapToken* Entry; std::size_t OrderedIndex; };
        const USensorSubBody* Sensor = nullptr;
        uint64 SensorStableId = 0, SensorIncarnation = 0;
        uint32 SensorNativeObjectId = 0;
        const FOverlapToken* OrderedData = nullptr;
        std::vector<FTarget> Ordered;
        std::vector<FSetCell> SetCells;
    };
    struct FArrayBacking
    {
        const void* Data = nullptr;
        std::size_t Bytes = 0;
        int32 Num = 0, Max = 0;
        bool operator==(const FArrayBacking&) const = default;
    };
    struct FColdHitbox
    {
        const USpeedWheeledComponent* Actor;
        const UBoxSubBody* Box;
        PreparedWorld::FEndpointBinding Binding;
        std::array<FArrayBacking, 8> Backings;
    };
    struct FWheelBinding
    {
        const USpeedWheeledComponent* Actor;
        const USWheelSubBody* Body;
        PreparedWorld::FEndpointBinding Endpoint;
        const USpeedSimulation* Simulation;
        const Chaos::FSimpleWheeledVehicle* Vehicle;
        int32 Ordinal;
        Chaos::FSimpleWheelSim* Wheel;
        Chaos::FSimpleSuspensionSim* Suspension;
        TWeakObjectPtr<UChaosVehicleWheel> Configuration;
        const Chaos::FSimpleWheelConfig* WheelSetup;
        const Chaos::FSimpleSuspensionConfig* SuspensionSetup;
        std::array<FArrayBacking, 5> Backings;
        bool operator==(const FWheelBinding&) const = default;
    };

    const FSimulationWorld* const World;
    const PreparedWorld::FAddress Address;
    const FPreparedRestoreLeaseView Lease;
    EPhase Phase = EPhase::Prepared;
    PreparedAccess::FParticipantAccess Access;
    std::vector<FExchange> Exchanges;
    std::optional<FKeptSensorOverlaps> KeptSensor;
    std::optional<FColdHitbox> ColdHitbox;
    std::optional<FWheelBinding> RetainedWheel;

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
        // The exchange touches only the two declared objects. A non-bitwise
        // swap may only use the built-in scalar moves.
        static_assert(TUseBitwiseSwap<T>::Value || std::is_scalar_v<T>);
        FOwnedValue Staged(new T(std::move(Value)), [](void* Object) noexcept { delete static_cast<T*>(Object); });
        Access.Writes.push_back(Range(&Live, sizeof(Live)));
        Access.Writes.push_back(Range(Staged.get(), sizeof(T)));
        Exchanges.push_back({&Live, std::move(Staged), [](void* A, void* B) noexcept
        {
            ::Swap<T>(*static_cast<T*>(A), *static_cast<T*>(B));
        }});
    }

    void DeclareExchangeReads(PreparedAccess::FParticipantAccess& Declaration) const
    {
        Declaration.ReadOnly.push_back(Range(&Exchanges, sizeof(Exchanges)));
        if (!Exchanges.empty())
            Declaration.ReadOnly.push_back(Range(Exchanges.data(), Exchanges.size() * sizeof(FExchange)));
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
    bool StageColdBoxArray(TArray<T>& Live)
    {
        TArray<T> Empty;
        Empty.Reserve(Live.Max());
        if (Empty.Max() != Live.Max() || Empty.GetAllocatedSize() != Live.GetAllocatedSize()) return false;
        StageArray(Live, std::move(Empty));
        return true;
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

    static bool HasBoundedOverlaps(const USensorSubBody& Sensor) noexcept
    {
        constexpr int32 MaxEntries = 256;
        // A sparse set can have a small Num and a large iteration range.
        constexpr SIZE_T MaxSetBytes = 64 * 1024;
        return Sensor.OverlapsOrdered.Num() <= MaxEntries
            && Sensor.OverlapsSet.Num() == Sensor.OverlapsOrdered.Num()
            && Sensor.OverlapsSet.GetAllocatedSize() <= MaxSetBytes;
    }

    // The original round has already reauthenticated the adapter and its
    // registered slots. Match those records without running adapter hooks.
    bool MatchesNativeSubBodySlot(const USSubBody& Body, uint64 StableId, uint32 NativeObjectId,
        uint64 Incarnation) const noexcept
    {
        const uint64 ActorId = StableId >> 16;
        auto* Actor = World->AdaptersByStableId.FindRef(ActorId);
        const auto* Slot = World->ExactRoundSubBodySlots.Find(StableId);
        return ActorId && Actor && Slot && Incarnation && NativeObjectId
            && World->StableIds.FindRef(Actor) == ActorId
            && Actor->GetPublishedSimulationStableId() == ActorId
            && World->SubBodiesByStableId.FindRef(StableId) == &Body
            && World->StableSubBodyIds.FindRef(&Body) == StableId
            && World->SubBodyNativeObjectIds.FindRef(StableId) == NativeObjectId
            && Slot->Incarnation == Incarnation && Slot->Object.Get() == &Body
            && Body.GetUniqueID() == NativeObjectId && Body.GetParentComponent() == Actor;
    }

    bool MatchesOverlapTarget(const PreparedWorld::FEndpointBinding& Binding,
        const FOverlapToken& Token) const noexcept
    {
        const auto* Body = World->SolidSubBodiesByStableId.FindRef(Binding.StableId);
        return Body && Binding.ActorIncarnation && (Binding.StableId >> 16) == Binding.Actor
            && Token.Get() == Body
            && MatchesNativeSubBodySlot(*Body, Binding.StableId, Binding.NativeObjectId, Binding.Incarnation);
    }

    bool ValidateKeptSensor() const noexcept
    {
        if (!KeptSensor) return true;
        const auto& Keep = *KeptSensor;
        const auto& Sensor = *Keep.Sensor;
        if (!MatchesNativeSubBodySlot(Sensor, Keep.SensorStableId, Keep.SensorNativeObjectId, Keep.SensorIncarnation)
            || !HasBoundedOverlaps(Sensor) || Sensor.OverlapsOrdered.GetData() != Keep.OrderedData
            || static_cast<std::size_t>(Sensor.OverlapsOrdered.Num()) != Keep.Ordered.size()) return false;
        for (std::size_t Index = 0; Index < Keep.Ordered.size(); ++Index)
        {
            const auto& Target = Keep.Ordered[Index];
            const auto& Live = Keep.OrderedData[Index];
            if (!Live.HasSameIndexAndSerialNumber(Target.Token)
                || !MatchesOverlapTarget(Target.Binding, Live)
                || !Sensor.OverlapsSet.Contains(Keep.OrderedData[Index])) return false;
        }
        std::size_t Index = 0;
        for (const auto& Entry : Sensor.OverlapsSet)
        {
            if (Index == Keep.SetCells.size()) return false;
            const auto& Cell = Keep.SetCells[Index++];
            if (Cell.Entry != &Entry || !Entry.HasSameIndexAndSerialNumber(Keep.Ordered[Cell.OrderedIndex].Token))
                return false;
        }
        return Index == Keep.SetCells.size();
    }

    template<class T>
    static FArrayBacking ReadBacking(const TArray<T>& Array)
    {
        return {Array.GetData(), Array.GetAllocatedSize(), Array.Num(), Array.Max()};
    }

    std::optional<FWheelBinding> ReadWheelBinding(const USpeedWheeledComponent* Actor, const USWheelSubBody* Body,
        const PreparedWorld::FEndpointBinding& Endpoint) const noexcept
    {
        const FExactRoundNativeOrigin Origin{Address.Authority, Address.WorldIncarnation, Address.TopologyVersion};
        const auto Slot = static_cast<uint16>(Endpoint.StableId & MAX_uint16);
        if (!Actor || !Body || !Slot || !Endpoint.ActorIncarnation || !Origin.Valid()
            || !(World->ReadExactRoundOrigin() == Origin) || !World->StorageAccess.IsReservationOwner()
            || World->AdaptersByStableId.FindRef(Endpoint.Actor) != Actor
            || World->SolidSubBodiesByStableId.FindRef(Endpoint.StableId) != Body
            || !IsValid(Actor) || !IsValid(Body)
            || !MatchesNativeSubBodySlot(*Body, Endpoint.StableId, Endpoint.NativeObjectId, Endpoint.Incarnation))
            return {};

        const auto Ordinal = Body->Idx();
        const auto& Children = Actor->USpeedWheeledComponent::GetSubBodies();
        const auto* Simulation = Actor->SkySimulation;
        if (!Simulation || Actor->VehicleSimulationPT.Get() != Simulation
            || !Children.IsValidIndex(Slot) || Children[Slot] != Body
            || !Actor->WheelSubBodies.IsValidIndex(Ordinal) || Actor->WheelSubBodies[Ordinal].Get() != Body
            || !Actor->Wheels.IsValidIndex(Ordinal)) return {};

        const auto* Vehicle = Simulation->PVehicle.Get();
        if (!Vehicle || !Vehicle->Wheels.IsValidIndex(Ordinal) || !Vehicle->Suspension.IsValidIndex(Ordinal))
            return {};
        auto* Configuration = Actor->Wheels[Ordinal].Get();
        auto* Wheel = Body->PWheel;
        auto* Suspension = Body->PSuspension;
        if (!IsValid(Configuration) || Body->ChaosWheel != Configuration
            || Wheel != &Vehicle->Wheels[Ordinal] || Suspension != &Vehicle->Suspension[Ordinal]
            || !Wheel->SetupPtr || !Suspension->SetupPtr) return {};

        return FWheelBinding{Actor, Body, Endpoint, Simulation, Vehicle, Ordinal, Wheel, Suspension,
            TWeakObjectPtr<UChaosVehicleWheel>(Configuration), Wheel->SetupPtr, Suspension->SetupPtr,
            {ReadBacking(Children), ReadBacking(Actor->WheelSubBodies), ReadBacking(Actor->Wheels),
                ReadBacking(Vehicle->Wheels), ReadBacking(Vehicle->Suspension)}};
    }

    // IAS teardown retires the worker before replacing this storage. The host
    // hold must also retain it and exclude direct Chaos writers such as SetWheelClass.
    bool ValidateRetainedWheel() const noexcept
    {
        if (!RetainedWheel) return true;
        const auto& Saved = *RetainedWheel;
        const auto Current = ReadWheelBinding(Saved.Actor, Saved.Body, Saved.Endpoint);
        return Current && *Current == Saved;
    }

    void DeclareWheelBindingReads()
    {
        const auto& Saved = *RetainedWheel;
        const auto& Actor = *Saved.Actor;
        const auto& Body = *Saved.Body;
        const auto& Children = Actor.USpeedWheeledComponent::GetSubBodies();
        const auto Slot = static_cast<uint16>(Saved.Endpoint.StableId & MAX_uint16);
        const auto Read = [this](const auto& Value)
        {
            Access.ReadOnly.push_back(Range(&Value, sizeof(Value)));
        };
        Read(Saved);
        Read(Actor.VehicleSimulationPT);
        Read(Actor.SkySimulation);
        Read(Saved.Simulation->PVehicle);
        Read(Saved.Vehicle->Wheels);
        Read(Saved.Vehicle->Suspension);
        Read(Children);
        Read(Children[Slot]);
        Read(Actor.WheelSubBodies);
        Read(Actor.WheelSubBodies[Saved.Ordinal]);
        Read(Actor.Wheels);
        Read(Actor.Wheels[Saved.Ordinal]);
        Read(Body.PWheel);
        Read(Body.PSuspension);
        Read(Body.ChaosWheel);
    }

    static std::array<FArrayBacking, 8> ReadHitboxBackings(const UBoxSubBody& Box)
    {
        return {ReadBacking(Box.CurrentGroundContactsWS), ReadBacking(Box.PreviousFrameGroundContactsWS),
            ReadBacking(Box.CurrentGroundNormalsWS), ReadBacking(Box.PrevGroundContactsLS),
            ReadBacking(Box.LatchedEdgeContactsLS), ReadBacking(Box.PrevLambdaN),
            ReadBacking(Box.IgnoredComponents), ReadBacking(Box.AlwaysIgnoredComponents)};
    }

    bool ValidateColdHitbox() const noexcept
    {
        if (!ColdHitbox) return true;
        const FExactRoundNativeOrigin Origin{Address.Authority, Address.WorldIncarnation, Address.TopologyVersion};
        const auto& [Actor, Box, Binding, Backings] = *ColdHitbox;
        if (!Origin.Valid() || !(World->ReadExactRoundOrigin() == Origin) || !World->StorageAccess.IsReservationOwner()
            || World->AdaptersByStableId.FindRef(Binding.Actor) != Actor
            || World->SolidSubBodiesByStableId.FindRef(Binding.StableId) != Box
            || !MatchesNativeSubBodySlot(*Box, Binding.StableId, Binding.NativeObjectId, Binding.Incarnation))
            return false;

        const auto& Children = Actor->USpeedWheeledComponent::GetSubBodies();
        return !Children.IsEmpty() && Children[0] == Box && Actor->GetHitboxSubBodyForConfiguration() == Box
            && Box->GetSubBodyType() == USSubBody::ESubBodyType::Hitbox
            && Box->HasColdExactRoundHitCounts()
            && Backings == ReadHitboxBackings(*Box);
    }
};

FPreparedNativeRoundFields::FPreparedNativeRoundFields(const FSimulationWorld& World,
    const FSimulationComponentRestoreRequest& Request) : Plan(std::make_unique<FPlan>(World, Request)) {}

std::optional<PreparedWorld::FColdCarBump> FPreparedNativeRoundFields::ReadCarBumpSource(
    const FSimulationWorld& World, uint64 ActorId)
{
    if (!ActorId || !World.ExactRoundWorkerGeneration || !World.StorageAccess.IsReservationOwner()) return {};
    auto* Adapter = World.AdaptersByStableId.FindRef(ActorId);
    const auto* Car = Adapter ? Adapter->AsWheeledPhysicsComponent() : nullptr;
    if (!IsValid(Car) || static_cast<const ISpeedComponent*>(Car) != Adapter || Car->IsPresentationOnly()
        || World.FindStableId(*Car) != ActorId || Car->GetPublishedSimulationStableId() != ActorId) return {};

    const auto& Physics = Car->WheeledPhysicsState;
    if (!Physics.CarBumpCooldownFrames)
    {
        if (!Physics.LastCarBumpVictim.IsExplicitlyNull()) return {};
        return PreparedWorld::FColdCarBump{ActorId, 0, 0};
    }
    const auto* Victim = Physics.LastCarBumpVictim.Get();
    if (!IsValid(Victim) || Victim == Car || Victim->IsPresentationOnly()) return {};
    const uint64 VictimId = Victim->GetPublishedSimulationStableId();
    if (!VictimId || VictimId == ActorId || World.AdaptersByStableId.FindRef(VictimId) != Victim
        || World.FindStableId(*Victim) != VictimId) return {};
    return PreparedWorld::FColdCarBump{ActorId, VictimId, Physics.CarBumpCooldownFrames};
}

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
    const auto Owner = Request.OwnerLease.ResolvePreparingState(Request, Request.Address);
    if (!Owner || !FPreparedNativeKinematicFields::MatchesActor(*Owner->World, Request, Target)
        || Target.BaseGameState.bHasTestVelocity || Target.bNetCorrHasTarget || Target.NetCorrAccum.bActive
        || Target.bCanonicalInputPending || Target.InputReservationV2 || Target.PreparedWheeledNetworkInput
        || !ValidTimeline(Request, Values.Timeline) || !ValidVehicle(Values)) return {};
    const auto* World = Owner->World;
    if (World->ExactRoundWorkerGeneration)
    {
        const auto ActorId = Request.Supplement.Binding.StableId;
        const auto* Intent = PreparedWorld::FindColdCarBump(Owner->Envelope->ColdCarBumps, ActorId);
        const auto Source = ReadCarBumpSource(*World, ActorId);
        const auto& Payload = Request.LegacyMechanicPayload;
        const auto Cold = Collision::EncodeCarBumpSnapshot(Values.Physics.CarBumpCooldownFrames, 0);
        if (!Source || (Intent ? !(*Intent == *Source) : Source->SourceFrames != 0)
            || static_cast<std::size_t>(Payload.Num()) != Cold.size()
            || !std::equal(Cold.begin(), Cold.end(), Payload.GetData())) return {};
    }

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

std::unique_ptr<FPreparedNativeRoundFields> FPreparedNativeRoundFields::PrepareColdCommon(ISpeedComponent& Target,
    const FSimulationComponentRestoreRequest& Request)
{
    const auto* World = Request.OwnerLease.ResolvePreparingWorld(Request, Request.Address);
    if (!World || !FPreparedNativeKinematicFields::MatchesActor(*World, Request, Target)
        || (Target.StaticRestingWorld && Target.StaticRestingWorld != World->StaticCollisionWorld.Get())) return {};

    auto Fragment = std::unique_ptr<FPreparedNativeRoundFields>(new FPreparedNativeRoundFields(*World, Request));
    auto& P = *Fragment->Plan;
    P.StageArray(Target.PhysicalConstraints, {});
    P.Stage(Target.SleepState, FSimulationSleepState{});
    P.Stage(Target.KinematicQuantizationCache, FIdentityKinematicQuantizationCache{});
    // The following physical step supplies a fresh horizon.
    P.Stage(Target.StaticSupportFrameHorizon, 0.0f);
    P.Access.ReadOnly.push_back(Range(&Target.StaticRestingWorld, sizeof(Target.StaticRestingWorld)));
    return Fragment;
}

std::unique_ptr<FPreparedNativeRoundFields> FPreparedNativeRoundFields::PrepareClearSolidContacts(USolidSubBody& Target,
    const FSimulationComponentRestoreRequest& Request, const PreparedWorld::FEndpointBinding& Endpoint)
{
    const auto* World = Request.OwnerLease.ResolvePreparingWorld(Request, Request.Address);
    if (!World || !FPreparedNativeKinematicFields::MatchesSolid(*World, Request, Target, Endpoint)) return {};
    if (World->ExactRoundWorkerGeneration && (Endpoint.StableId & MAX_uint16) == 0
        && Cast<UBoxSubBody>(&Target) && Target.GetParentComponent()->AsWheeledPhysicsComponent()) return {};
    auto Fragment = std::unique_ptr<FPreparedNativeRoundFields>(new FPreparedNativeRoundFields(*World, Request));
    Fragment->StageClearContacts(Target);
    return Fragment;
}

std::unique_ptr<FPreparedNativeRoundFields> FPreparedNativeRoundFields::PrepareClearSolidContacts(
    const FSimulationComponentRestoreRequest& Request, const PreparedWorld::FEndpointBinding& Endpoint)
{
    const auto* World = Request.OwnerLease.ResolvePreparingWorld(Request, Request.Address);
    auto* Target = World ? FPreparedNativeKinematicFields::ResolveBoundSolid(*World, Request, Endpoint) : nullptr;
    return Target ? PrepareClearSolidContacts(*Target, Request, Endpoint) : nullptr;
}

std::unique_ptr<FPreparedNativeRoundFields> FPreparedNativeRoundFields::PrepareColdWheeledHitbox(
    const FSimulationComponentRestoreRequest& Request, const PreparedWorld::FEndpointBinding& Endpoint)
{
    const auto* World = Request.OwnerLease.ResolvePreparingWorld(Request, Request.Address);
    const auto ActorId = Request.Supplement.Binding.StableId;
    if (!World || !ActorId || ActorId > (UINT64_MAX >> 16) || Endpoint.StableId != (ActorId << 16)) return {};
    auto* Box = Cast<UBoxSubBody>(FPreparedNativeKinematicFields::ResolveBoundSolid(*World, Request, Endpoint));
    if (!Box) return {};
    auto* Parent = Box->GetParentComponent();
    const auto* Actor = Parent->AsWheeledPhysicsComponent();
    if (!Actor || static_cast<const ISpeedComponent*>(Actor) != Parent) return {};

    auto Fragment = std::unique_ptr<FPreparedNativeRoundFields>(new FPreparedNativeRoundFields(*World, Request));
    auto& P = *Fragment->Plan;
    P.ColdHitbox.emplace(FPlan::FColdHitbox{Actor, Box, Endpoint, FPlan::ReadHitboxBackings(*Box)});
    // An empty count alone does not establish absence of sparse-map storage.
    // The hit-count map takes no part in the exchange until its backing can be declared.
    if (!P.ValidateColdHitbox()
        || static_cast<std::size_t>(Box->AlwaysIgnoredComponents.Num()) > PreparedWorld::FLimits{}.MaxEndpoints)
        return {};
    P.Access.ReadOnly.push_back(Range(&*P.ColdHitbox, sizeof(*P.ColdHitbox)));
    P.Access.ReadOnly.push_back(Range(&Box->HitCountThisFrame, sizeof(Box->HitCountThisFrame)));
    P.Access.ReadOnly.push_back(Range(&Box->AlwaysIgnoredComponents, sizeof(Box->AlwaysIgnoredComponents)));
    const auto& AlwaysIgnored = P.ColdHitbox->Backings.back();
    if (AlwaysIgnored.Bytes) P.Access.ReadOnly.push_back(Range(AlwaysIgnored.Data, AlwaysIgnored.Bytes));

    Fragment->StageClearContacts(*Box);
    if (!P.StageColdBoxArray(Box->CurrentGroundContactsWS) || !P.StageColdBoxArray(Box->PreviousFrameGroundContactsWS)
        || !P.StageColdBoxArray(Box->CurrentGroundNormalsWS) || !P.StageColdBoxArray(Box->PrevGroundContactsLS)
        || !P.StageColdBoxArray(Box->LatchedEdgeContactsLS) || !P.StageColdBoxArray(Box->PrevLambdaN))
        return {};
    constexpr int32 NoContactFrame = INDEX_NONE;
    P.Stage(Box->bEdgeSupportLatched, false);
    P.Stage(Box->EdgeSupportLatchFrame, NoContactFrame);
    P.Stage(Box->bRoofSurfaceTraversalLatched, false);
    P.Stage(Box->LastRoofSurfaceContactFrame, NoContactFrame);
    P.Stage(Box->RoofSurfaceComp, {});
    P.Stage(Box->CompositeGroundNormal, FVector::UpVector);
    P.Stage(Box->bGroundContactStable, false);
    P.Stage(Box->LastResolvedGroundHitFrame, NoContactFrame);
    P.Stage(Box->bHasGroundContact, false);
    P.Stage(Box->bFreshEdgeRecoverCandidate, false);
    P.Stage(Box->StableTime, 0.0f);
    P.Stage(Box->bGroundHitFromSweep, false);
    P.Stage(Box->bGroundPlaneValid, false);
    P.Stage(Box->GroundPlanePointWS, FVector::ZeroVector);
    P.Stage(Box->GroundPlaneN, FVector::UpVector);
    P.Stage(Box->GroundPlaneD, 0.0f);
    P.Stage(Box->GroundComp, {});
    P.Stage(Box->EstablishedSupportSourceId, uint64{0});
    P.Stage(Box->EstablishedSupportSurfaceId, uint64{0});
    P.Stage(Box->FreshSupportClampUntilFrame, NoContactFrame);
    P.Stage(Box->FreshSupportClampN, FVector::UpVector);
    P.Stage(Box->FreshSupportClampComp, {});
    P.Stage(Box->PrevGroundNormal, FVector::UpVector);
    P.Stage(Box->bHadGroundContactPrevFrame, false);
    P.Stage(Box->PrevContactNormal, FVector::UpVector);
    P.Stage(Box->bHadContactPrevFrame, false);
    P.Stage(Box->LastWallOrGutterFrame, NoContactFrame);
    P.Stage(Box->LastWallOrGutterN, FVector::ZeroVector);
    P.Stage(Box->LastWallOrGutterUpDot, 0.0f);
    P.Stage(Box->LastResolvedSphereHit, SHitResult{});
    P.Stage(Box->LastResolvedSphereHitFrame, NoContactFrame);
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
    if (!World || !FPreparedNativeKinematicFields::MatchesSolid(*World, Request, Target, Endpoint) || !ValidWheel(Values))
        return {};
    auto* Parent = Target.GetParentComponent();
    const auto* Actor = Parent->AsWheeledPhysicsComponent();
    if (static_cast<const ISpeedComponent*>(Actor) != Parent) return {};

    auto Fragment = std::unique_ptr<FPreparedNativeRoundFields>(new FPreparedNativeRoundFields(*World, Request));
    auto& P = *Fragment->Plan;
    P.RetainedWheel = P.ReadWheelBinding(Actor, &Target, Endpoint);
    if (!P.RetainedWheel) return {};
    P.DeclareWheelBindingReads();

    auto& LiveSuspension = *P.RetainedWheel->Suspension;
    auto& LiveWheel = *P.RetainedWheel->Wheel;
    const float Radius = LiveWheel.GetEffectiveRadius();
    const float TraceLength = Values.SuspensionLengthCm + LiveSuspension.Setup().RaycastSafetyMargin + Radius;
    const float SteeringDegrees = FMath::RadiansToDegrees(Values.SteeringAngle);
    if (!FMath::IsFinite(Radius) || Radius <= 0.0f || !FMath::IsFinite(TraceLength)
        || !FMath::IsFinite(SteeringDegrees)) return {};

    Chaos::FSimpleSuspensionSim Suspension(&LiveSuspension.Setup());
    Suspension.SetLocalRestingPosition(LiveSuspension.GetLocalRestingPosition());
    Suspension.SetSpringIndex(static_cast<uint32>(P.RetainedWheel->Ordinal));
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

    Fragment->StageClearContacts(Target);
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

std::unique_ptr<FPreparedNativeRoundFields> FPreparedNativeRoundFields::PrepareClearSensorOverlaps(
    const FSimulationComponentRestoreRequest& Request, const FPreparedNativeKinematicFields::FRoster::FSubBody& Row)
{
    const auto* World = Request.OwnerLease.ResolvePreparingWorld(Request, Request.Address);
    auto* Target = World ? FPreparedNativeKinematicFields::ResolveBoundSensor(*World, Request, Row) : nullptr;
    return Target ? PrepareClearSensorOverlaps(*Target, Request, Row) : nullptr;
}

std::unique_ptr<FPreparedNativeRoundFields> FPreparedNativeRoundFields::PrepareKeepSensorOverlaps(
    const FSimulationComponentRestoreRequest& Request, const FPreparedNativeKinematicFields::FRoster::FSubBody& Row,
    std::vector<PreparedWorld::FEndpointBinding>* RetainedOrdered)
{
    const auto State = Request.OwnerLease.ResolvePreparingState(Request, Request.Address);
    if (!State) return {};
    const auto& World = *State->World;
    const auto& Envelope = *State->Envelope;
    const FExactRoundNativeOrigin Origin{Request.Address.Authority, Request.Address.WorldIncarnation,
        Request.Address.TopologyVersion};
    const auto Limits = PreparedWorld::FLimits{};
    if (!Origin.Valid() || !(World.ReadExactRoundOrigin() == Origin) || !World.StorageAccess.IsReservationOwner()
        || !Row.Incarnation || Envelope.Endpoints.size() > Limits.MaxEndpoints
        || Envelope.Actors.size() > Limits.MaxBodies) return {};
    if (Row.IsSolidEndpoint || Row.ActorStableId != State->ComponentBinding->StableId
        || (Row.StableId >> 16) != Row.ActorStableId || World.SolidSubBodiesByStableId.Contains(Row.StableId))
        return {};
    const auto* Sensor = Cast<USensorSubBody>(World.SubBodiesByStableId.FindRef(Row.StableId));
    if (!Sensor || Sensor->GetParentComponent() != State->ComponentTarget || !FPlan::HasBoundedOverlaps(*Sensor))
        return {};

    auto Fragment = std::unique_ptr<FPreparedNativeRoundFields>(new FPreparedNativeRoundFields(World, Request));
    auto& P = *Fragment->Plan;
    if (!P.MatchesNativeSubBodySlot(*Sensor, Row.StableId, Row.NativeObjectId, Row.Incarnation)) return {};
    auto& Keep = P.KeptSensor.emplace();
    Keep.Sensor = Sensor;
    Keep.SensorStableId = Row.StableId;
    Keep.SensorNativeObjectId = Row.NativeObjectId;
    Keep.SensorIncarnation = Row.Incarnation;
    Keep.OrderedData = Sensor->OverlapsOrdered.GetData();
    Keep.Ordered.reserve(Sensor->OverlapsOrdered.Num());
    Keep.SetCells.reserve(Sensor->OverlapsSet.Num());

    // Resolve raw addresses for lookup only. Dereference a target only after
    // joining it to a solid in the sealed native registry.
    TMap<const UPrimitiveComponent*, std::size_t> Wanted;
    const auto& Ordered = Sensor->OverlapsOrdered;
    for (const auto& Entry : Ordered)
    {
        const auto* Target = Entry.Get();
        if (!Target || Wanted.Contains(Target)) return {};
        Wanted.Add(Target, Keep.Ordered.size());
        Keep.Ordered.push_back({Entry, {}});
    }
    if (!Wanted.IsEmpty())
    {
        std::size_t Found = 0;
        for (const auto& Endpoint : Envelope.Endpoints)
        {
            const auto* Solid = World.SolidSubBodiesByStableId.FindRef(Endpoint.StableId);
            const auto* Index = Solid ? Wanted.Find(Solid) : nullptr;
            if (!Index) continue;
            auto& Target = Keep.Ordered[*Index];
            const auto Actor = std::lower_bound(Envelope.Actors.begin(), Envelope.Actors.end(), Endpoint.Actor,
                [](const auto& Binding, uint64 Id) { return Binding.StableId < Id; });
            if (Target.Binding.StableId || Actor == Envelope.Actors.end() || Actor->StableId != Endpoint.Actor
                || Actor->Incarnation != Endpoint.ActorIncarnation || !P.MatchesOverlapTarget(Endpoint, Target.Token)) return {};
            Target.Binding = Endpoint;
            if (++Found == Keep.Ordered.size()) break;
        }
        if (Found != Keep.Ordered.size()) return {};
    }

    auto& Reads = P.Access.ReadOnly;
    Reads.push_back(Range(&Sensor->OverlapsOrdered, sizeof(Sensor->OverlapsOrdered)));
    Reads.push_back(Range(&Sensor->OverlapsSet, sizeof(Sensor->OverlapsSet)));
    if (!Ordered.IsEmpty()) Reads.push_back(Range(Keep.OrderedData, sizeof(Ordered[0]) * Ordered.Num()));
    for (const auto& Entry : Sensor->OverlapsSet)
    {
        const auto* Target = Entry.Get();
        const auto* Index = Wanted.Find(Target);
        if (!Index || !Entry.HasSameIndexAndSerialNumber(Keep.Ordered[*Index].Token)) return {};
        Reads.push_back(Range(&Entry, sizeof(Entry)));
        Keep.SetCells.push_back({&Entry, *Index});
        Wanted.Remove(Target);
    }
    if (!Wanted.IsEmpty() || !P.HoldsBoundary() || !P.ValidateKeptSensor()) return {};
    if (RetainedOrdered)
    {
        std::vector<PreparedWorld::FEndpointBinding> Values;
        Values.reserve(Keep.Ordered.size());
        for (const auto& Kept : Keep.Ordered) Values.push_back(Kept.Binding);
        *RetainedOrdered = std::move(Values);
    }
    return Fragment;
}

std::unique_ptr<FPreparedNativeRoundFields> FPreparedNativeRoundFields::PrepareWheel(
    const FSimulationComponentRestoreRequest& Request, const PreparedWorld::FEndpointBinding& Endpoint, const FWheelState& Values)
{
    const auto* World = Request.OwnerLease.ResolvePreparingWorld(Request, Request.Address);
    auto* Target = World ? Cast<USWheelSubBody>(FPreparedNativeKinematicFields::ResolveBoundSolid(*World, Request, Endpoint)) : nullptr;
    return Target ? PrepareWheel(*Target, Request, Endpoint, Values) : nullptr;
}

FPreparedNativeRoundFields::~FPreparedNativeRoundFields() noexcept
{
    if (Plan->Phase == FPlan::EPhase::Finalized) std::terminate();
}

bool FPreparedNativeRoundFields::AppendPreparedAccess(std::vector<PreparedAccess::FParticipantAccess>& Out) const
{
    if ((Plan->Phase != FPlan::EPhase::Prepared && Plan->Phase != FPlan::EPhase::Ready) || !Plan->HoldsBoundary()
        || !Plan->ValidateKeptSensor() || !Plan->ValidateColdHitbox() || !Plan->ValidateRetainedWheel()) return false;
    auto Declaration = Plan->Access;
    Declaration.ReadOnly.push_back(Range(&Plan, sizeof(Plan)));
    Plan->DeclareExchangeReads(Declaration);
    Out.push_back(std::move(Declaration));
    return true;
}

bool FPreparedNativeRoundFields::PrepareFinalization() noexcept
{
    if ((Plan->Phase != FPlan::EPhase::Prepared && Plan->Phase != FPlan::EPhase::Ready) || !Plan->HoldsBoundary()
        || !Plan->ValidateKeptSensor() || !Plan->ValidateColdHitbox() || !Plan->ValidateRetainedWheel()) return false;
    Plan->Phase = FPlan::EPhase::Ready;
    return true;
}

void FPreparedNativeRoundFields::Finalize() noexcept
{
    if (Plan->Phase != FPlan::EPhase::Ready) std::terminate();
    for (const auto& Exchange : Plan->Exchanges) Exchange.Apply();
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

#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "Components/BoxComponent.h"
#include "Misc/AutomationTest.h"
#include "UObject/StrongObjectPtr.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedExactRoundWheeledHitCountTest,
    "IAmSpeed.Netcode.RoundReset.WheeledHitCountRetirement",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedExactRoundWheeledHitCountTest::RunTest(const FString&)
{
    TStrongObjectPtr<UBoxSubBody> Box(NewObject<UBoxSubBody>());
    const SIZE_T FreshMapBytes = Box->HitCountThisFrame.GetAllocatedSize();
    TestTrue(TEXT("The fresh native count map needs no external backing"), Box->HasColdExactRoundHitCounts());
    TStrongObjectPtr<UBoxComponent> First(NewObject<UBoxComponent>());
    TStrongObjectPtr<UBoxComponent> Second(NewObject<UBoxComponent>());
    First->SetMobility(EComponentMobility::Static);
    Second->SetMobility(EComponentMobility::Static);
    const auto AcceptStaticHit = [&](UBoxComponent* Surface)
    {
        SHitResult Hit;
        Hit.Component = Surface;
        Hit.bHit = Hit.bBlockingHit = true;
        Box->SetFutureHit(Hit);
        Box->AcceptHit();
    };
    const auto* SubBody = static_cast<const USSubBody*>(Box.Get());
    const auto Limit = Box->MaxHitsPerComponentPerFrame;
    for (uint32 Count = 1; Count < Limit; ++Count) AcceptStaticHit(First.Get());
    TestFalse(TEXT("The cap has not been reached on F-1"), SubBody->ComponentHasBeenIgnored(*First.Get()));
    AcceptStaticHit(Second.Get());
    AcceptStaticHit(First.Get());
    TestTrue(TEXT("The cap still applies at the last accepted F-1 hit"), SubBody->ComponentHasBeenIgnored(*First.Get()));
    TestFalse(TEXT("The second surface keeps its own count"), SubBody->ComponentHasBeenIgnored(*Second.Get()));
    TestEqual(TEXT("A real AcceptHit populated both map entries"), Box->HitCountThisFrame.Num(), 2);
    TestTrue(TEXT("The occupied count map owns backing"), Box->HitCountThisFrame.GetAllocatedSize() > FreshMapBytes);
    TestFalse(TEXT("Round staging refuses the occupied map"), Box->HasColdExactRoundHitCounts());

    Box->RetireFrameHitCounts();
    TestTrue(TEXT("Retiring the completed step releases the count backing"), Box->HasColdExactRoundHitCounts());
    TestTrue(TEXT("The previous contact and ignore decisions survive retirement"),
        Box->GetHit().Component.Get() == First.Get() && SubBody->ComponentHasBeenIgnored(*First.Get()));

    // Cover both small reservations and overflow of the sparse map's
    // inline occupancy words. Neither is cold until the backing is retired.
    for (const int32 ReservedEntries : {4, 256})
    {
        Box->HitCountThisFrame.Reserve(ReservedEntries);
        TestTrue(TEXT("The empty fixture can still own backing"), Box->HitCountThisFrame.IsEmpty()
            && Box->HitCountThisFrame.GetAllocatedSize() > FreshMapBytes);
        TestFalse(TEXT("Staging refuses reserved backing even with zero entries"), Box->HasColdExactRoundHitCounts());
        Box->RetireFrameHitCounts();
        TestTrue(TEXT("Retirement also releases empty reserved backing"), Box->HasColdExactRoundHitCounts());
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedContactFreeKickoffValuesTest,
    "IAmSpeed.Netcode.RoundReset.ContactFreeKickoffValues",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedContactFreeKickoffValuesTest::RunTest(const FString&)
{
    using FRound = Speed::FPreparedNativeRoundFields;
    FRound::FCapturedKickoffVehicle Seed;
    Seed.WorkerGeneration = 2;
    Seed.ActorStableId = 3;
    Seed.ActorNativeObjectId = 9;
    Seed.LocalCenterOfMass = FVector(10, 0, 0);
    Seed.State.Game.bGroundState = true;
    Seed.State.Game.NbWheelsOnGround = 4;
    Seed.State.bSupportStartupConfirmed = true;
    Seed.State.PhysicalInput.Throttle = 100;
    for (uint32 Index = 0; Index < Seed.Wheels.size(); ++Index)
    {
        auto& Wheel = Seed.Wheels[Index];
        Wheel.Binding = {100 + Index, 3, 10 + Index, true};
        Wheel.LocalOffset = FVector(40, 20 * Index, 10);
        Wheel.LocalRotation = FQuat::Identity;
        Wheel.MaxSuspensionLengthCm = 30;
        Wheel.SuspensionRestRaiseCm = 8;
        Wheel.State.LastDisplacementCm = 14;
        Wheel.State.RollAngle = 0.25f;
        Wheel.State.AngularVelocity = 12;
        Wheel.State.SteeringAngle = 0.4f;
        Wheel.State.JumpFrames = 2;
    }

    SKinematic Target;
    Target.Location = FVector(50, 100, 200);
    Target.Rotation = FQuat(FVector::UpVector, UE_DOUBLE_PI / 2);
    const auto Result = FRound::ProjectContactFreeKickoff(Seed, Target, true);
    if (!TestTrue(TEXT("Stationary target projects"), Result.has_value())) return false;
    TestTrue(TEXT("Support is requested but not confirmed"), Result->Vehicle.bSupportStartupRequested
        && !Result->Vehicle.bSupportStartupConfirmed && !Result->Vehicle.Game.bGroundState
        && Result->Vehicle.Game.NbWheelsOnGround == 0);
    TestTrue(TEXT("Controls are neutral"), Result->Vehicle.PhysicalInput.Throttle == 0
        && !Result->Vehicle.PhysicalInput.bCanMove);
    for (uint32 Index = 0; Index < Seed.Wheels.size(); ++Index)
    {
        const auto& Wheel = Result->Wheels[Index];
        TestTrue(TEXT("Target COM-to-origin transform and wheel order"),
            Wheel.Binding == Seed.Wheels[Index].Binding
            && Wheel.Pose.Location.Equals(FVector(50.0 - 20.0 * Index, 130, 210))
            && Wheel.State.Render.WorldPosition.Equals(Wheel.Pose.Location));
        TestTrue(TEXT("Cold extension and no previous contact-derived displacement"),
            Wheel.State.SuspensionLengthCm == 30 && Wheel.State.LastDisplacementCm == 0
            && Result->Vehicle.Physics.SuspensionLastDisplacement[Index] == 0
            && Wheel.State.AngularVelocity == 0 && Wheel.State.SteeringAngle == 0 && Wheel.State.JumpFrames == 0);
    }
    Target.Velocity = FVector::ForwardVector;
    TestFalse(TEXT("Moving target rejected"), FRound::ProjectContactFreeKickoff(Seed, Target, false).has_value());
    Target.Velocity = FVector::ZeroVector;
    Seed.Wheels[1].Binding = Seed.Wheels[0].Binding;
    TestFalse(TEXT("Duplicate wheel rejected"), FRound::ProjectContactFreeKickoff(Seed, Target, false).has_value());
    return true;
}
#endif
