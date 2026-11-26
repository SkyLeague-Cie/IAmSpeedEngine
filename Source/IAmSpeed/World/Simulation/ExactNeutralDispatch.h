#pragma once

#include <cstdint>

class ASpeedPhysicsSimulation;
class ISpeedComponent;
class UPrimitiveComponent;
class USSubBody;
class USensorSubBody;
class USpeedMovementComponent;
class USpeedWorldSubsystem;

namespace Speed
{
class FExactRoundHostHold;

enum class EExactNeutralDispatch : std::uint8_t
{
    ActorPreGameplay,
    SensorBeginOverlap,
    ActorPostPhysics,
    SensorContinuousOverlap,
};

namespace ExactNeutralDispatchDetail
{
class FSelectionScope;
class FSubBodyPostScope;
class FCallbackScope;

// Driver phase, selected native dispatch and producer callback must all
// belong to the same synchronous stack. Only IAS call sites install them.
class FFrameScope final
{
    friend class ::ASpeedPhysicsSimulation;
    friend class FSelectionScope;
    friend class FSubBodyPostScope;
    friend class FCallbackScope;
    static thread_local const FFrameScope* Current;

    FFrameScope(const ASpeedPhysicsSimulation& InDriver, const USpeedWorldSubsystem& InBridge,
        const FExactRoundHostHold* InHold, std::uint64_t InGeneration, std::uint64_t InFrame,
        bool bInPreparing) noexcept
        : Previous(Current), Driver(InDriver), Bridge(InBridge), Hold(InHold),
          Generation(InGeneration), Frame(InFrame), bPreparing(bInPreparing), bActive(Hold && !Previous)
    {
        if (Hold || Previous) Current = this;
    }

    ~FFrameScope() noexcept { if (Current == this) Current = Previous; }
    FFrameScope(const FFrameScope&) = delete;
    FFrameScope& operator=(const FFrameScope&) = delete;

    const FFrameScope* const Previous;
    const ASpeedPhysicsSimulation& Driver;
    const USpeedWorldSubsystem& Bridge;
    const FExactRoundHostHold* const Hold;
    const std::uint64_t Generation, Frame;
    const bool bPreparing;
    const bool bActive;
};

class FSelectionScope final
{
    friend class ::USpeedWorldSubsystem;
    friend class ::ASpeedPhysicsSimulation;
    friend class ::ISpeedComponent;
    friend class ::USensorSubBody;
    friend class FSubBodyPostScope;
    friend class FCallbackScope;
    static thread_local const FSelectionScope* Current;

    FSelectionScope(const USpeedWorldSubsystem& Bridge, std::uint64_t NumFrame,
        EExactNeutralDispatch InKind, const void* InProducer) noexcept
        : Previous(Current), Frame(FFrameScope::Current), Kind(InKind), Producer(InProducer),
          bActive(!Previous && Frame && Frame->bActive && &Frame->Bridge == &Bridge
              && Frame->Frame == NumFrame && Frame->bPreparing == (Kind == EExactNeutralDispatch::ActorPreGameplay)
              && Kind != EExactNeutralDispatch::SensorContinuousOverlap && Producer)
    {
        if (Frame || Previous) Current = this;
    }

    FSelectionScope(const FSubBodyPostScope* Post, const USSubBody& Sensor, const UPrimitiveComponent& Other) noexcept;

    static const FSelectionScope* ClaimActorPost(const ISpeedComponent& Actor) noexcept
    {
        const auto* Selected = Current;
        if (!Selected || !Selected->bActive || Selected->Frame != FFrameScope::Current
            || Selected->Kind != EExactNeutralDispatch::ActorPostPhysics || Selected->Producer != &Actor
            || Selected->bActorPostClaimed) return nullptr;
        Selected->bActorPostClaimed = true;
        return Selected;
    }

    ~FSelectionScope() noexcept { if (Current == this) Current = Previous; }
    FSelectionScope(const FSelectionScope&) = delete;
    FSelectionScope& operator=(const FSelectionScope&) = delete;

    const FSelectionScope* const Previous;
    const FFrameScope* const Frame;
    const EExactNeutralDispatch Kind;
    const void* const Producer;
    const UPrimitiveComponent* const OverlapTarget = nullptr;
    const bool bActive;
    mutable bool bActorPostClaimed = false;
    mutable bool bContinuousBroadcastClaimed = false;
};

// Only the non-virtual actor post traversal installs this slot. A sensor
// claims its native post once, before any delegate can re-enter that method.
class FSubBodyPostScope final
{
    friend class ::ISpeedComponent;
    friend class ::USensorSubBody;
    friend class FSelectionScope;
    static thread_local FSubBodyPostScope* Current;

    FSubBodyPostScope(const FSelectionScope* ActorPost, const USSubBody& SubBody) noexcept
        : Previous(Current), Selection(ActorPost), Body(SubBody)
    {
        if (Selection || Previous) Current = this;
    }

    static const FSubBodyPostScope* ClaimSensorPost(const USSubBody& Sensor) noexcept
    {
        auto* Post = Current;
        if (!Post || !Post->Selection || Post->Selection != FSelectionScope::Current
            || Post->Selection->Frame != FFrameScope::Current || &Post->Body != &Sensor
            || Post->bClaimed) return nullptr;
        Post->bClaimed = true;
        return Post;
    }

    ~FSubBodyPostScope() noexcept { if (Current == this) Current = Previous; }
    FSubBodyPostScope(const FSubBodyPostScope&) = delete;
    FSubBodyPostScope& operator=(const FSubBodyPostScope&) = delete;

    FSubBodyPostScope* const Previous;
    const FSelectionScope* const Selection;
    const USSubBody& Body;
    bool bClaimed = false;
};

inline FSelectionScope::FSelectionScope(const FSubBodyPostScope* Post, const USSubBody& Sensor,
    const UPrimitiveComponent& Other) noexcept
    : Previous(Current), Frame(FFrameScope::Current), Kind(EExactNeutralDispatch::SensorContinuousOverlap),
      Producer(&Sensor), OverlapTarget(&Other),
      bActive(Post && Post == FSubBodyPostScope::Current && Post->bClaimed && &Post->Body == &Sensor
          && Previous && Post->Selection == Previous && Previous->Frame == Frame)
{
    if (Frame || Previous) Current = this;
}

class FCallbackScope final
{
    friend class ::ASpeedPhysicsSimulation;
    friend class ::ISpeedComponent;
    friend class ::USensorSubBody;
    friend class ::USpeedMovementComponent;
    static thread_local const FCallbackScope* Current;

    // A recursive native post starts outside the continuous broadcast that called it.
    FCallbackScope() noexcept : Previous(Current), Selection(nullptr), bActive(false)
    {
        if (Previous && Previous->Selection && Previous->Selection->Kind == EExactNeutralDispatch::SensorContinuousOverlap)
            Current = this;
    }

    FCallbackScope(EExactNeutralDispatch Kind, const void* Producer, const UPrimitiveComponent* Other = nullptr) noexcept
        : Previous(Current), Selection(FSelectionScope::Current),
          bActive(!Previous && Selection && Selection->bActive && Selection->Frame == FFrameScope::Current
              && Selection->Kind == Kind && Selection->Producer == Producer && Selection->OverlapTarget == Other
              && (Kind != EExactNeutralDispatch::SensorContinuousOverlap || !Selection->bContinuousBroadcastClaimed))
    {
        if (bActive && Kind == EExactNeutralDispatch::SensorContinuousOverlap)
            Selection->bContinuousBroadcastClaimed = true;
        if (Selection || Previous) Current = this;
    }

    ~FCallbackScope() noexcept { if (Current == this) Current = Previous; }
    FCallbackScope(const FCallbackScope&) = delete;
    FCallbackScope& operator=(const FCallbackScope&) = delete;

    const FCallbackScope* const Previous;
    const FSelectionScope* const Selection;
    const bool bActive;
};
}
}
