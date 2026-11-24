#pragma once

#include <cstdint>

class ASpeedPhysicsSimulation;
class ISpeedComponent;
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
};

namespace ExactNeutralDispatchDetail
{
class FSelectionScope;
class FCallbackScope;

// Driver phase, selected native dispatch and producer callback must all
// belong to the same synchronous stack. Only IAS call sites install them.
class FFrameScope final
{
    friend class ::ASpeedPhysicsSimulation;
    friend class FSelectionScope;
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
    friend class FCallbackScope;
    static thread_local const FSelectionScope* Current;

    FSelectionScope(const USpeedWorldSubsystem& Bridge, std::uint64_t NumFrame,
        EExactNeutralDispatch InKind, const void* InProducer) noexcept
        : Previous(Current), Frame(FFrameScope::Current), Kind(InKind), Producer(InProducer),
          bActive(!Previous && Frame && Frame->bActive && &Frame->Bridge == &Bridge
              && Frame->Frame == NumFrame && Frame->bPreparing == (Kind == EExactNeutralDispatch::ActorPreGameplay)
              && Producer)
    {
        if (Frame || Previous) Current = this;
    }

    ~FSelectionScope() noexcept { if (Current == this) Current = Previous; }
    FSelectionScope(const FSelectionScope&) = delete;
    FSelectionScope& operator=(const FSelectionScope&) = delete;

    const FSelectionScope* const Previous;
    const FFrameScope* const Frame;
    const EExactNeutralDispatch Kind;
    const void* const Producer;
    const bool bActive;
};

class FCallbackScope final
{
    friend class ::ASpeedPhysicsSimulation;
    friend class ::ISpeedComponent;
    friend class ::USensorSubBody;
    friend class ::USpeedMovementComponent;
    static thread_local const FCallbackScope* Current;

    FCallbackScope(EExactNeutralDispatch Kind, const void* Producer) noexcept
        : Previous(Current), Selection(FSelectionScope::Current),
          bActive(!Previous && Selection && Selection->bActive && Selection->Frame == FFrameScope::Current
              && Selection->Kind == Kind && Selection->Producer == Producer)
    {
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
