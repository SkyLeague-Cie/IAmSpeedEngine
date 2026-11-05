#pragma once
#include "CoreMinimal.h"
#include <memory>
#include "FrameDispatchObservation1075.h"

namespace Speed
{
enum class EFrameExecutorBackend : uint8 { PrivatePool = 1, SerialQueueFallback = 2 };
struct FFrameExecutorAvailability
{
    int32 TaskGraphWorkers = 0; // Legacy telemetry field; Unreal scheduler is never queried.
    int32 PrivateWorkers = 0;
    int32 SelectedWorkers = 0;
    bool bPrivateSelected = false;
    bool bReady = true;
};
// Configure before physical frames. Both0 (automatic) and1 (explicit) select
// IAmSpeed's bounded pool. Legacy backend2 is rejected, never silently reused.
IAMSPEED_API int32 RequestedFrameExecutorBackend();
IAMSPEED_API FFrameExecutorAvailability ConfigureFrameParallelExecutor(
    int32 HardwareLaneCapacity, int32 RequestedLanes);
// Only immutable/value work may enter this joined barrier. Callers retain
// private scratch and canonical result folding. Numeric guards belong to each
// lane's existing typed work function. No Unreal scheduler executes this work.
IAMSPEED_API EFrameExecutorBackend DispatchFrameParallelLanes(
    int32 Lanes, TFunctionRef<void(int32)> Work,
    FFrameDispatchObservation1075* Observation = nullptr);
// Explicit owned-pool entry point, also used by native executor tests.
IAMSPEED_API EFrameExecutorBackend DispatchPrivateFrameParallelLanes(
    int32 Lanes, TFunctionRef<void(int32)> Work,
    FFrameDispatchObservation1075* Observation = nullptr);
// Native-only admission: false guarantees that Work was never called.
// No speculative serial work is executed when nested or when the queue is full.
IAMSPEED_API bool TryDispatchPrivateFrameParallelLanes1075(
    int32 Lanes, TFunctionRef<void(int32)> Work,
    FFrameDispatchObservation1075* Observation = nullptr);
// Run a certified independent producer on one owned worker while OwnerWork
// stays on the caller. Join both before returning or rethrowing. When nested or
// the fixed queue cannot admit the pair, retain producer-before-owner ordering.
// Producer exceptions take precedence; serial producer failure skips OwnerWork.
IAMSPEED_API EFrameExecutorBackend DispatchPrivateFramePair(
    TFunctionRef<void()> ProducerWork, TFunctionRef<void()> OwnerWork);
IAMSPEED_API int32 EnsurePrivateFrameWorkers(int32 RequestedWorkers);
// Opaque retention of the ACTUAL already-configured service root. Acquisition
// refuses after shutdown admission closes and never creates/resizes a pool.
// The native owner releases it only after all dependent returns/finalizers.
// Host module retention through every call and this destructor is still required.
// This does not protect physical views, sources, configurations or a world.
class FFrameExecutorServiceRetention;
IAMSPEED_API std::unique_ptr<FFrameExecutorServiceRetention> RetainConfiguredFrameExecutorService();
class IAMSPEED_API FFrameExecutorServiceRetention final
{
    struct FImpl;
    std::unique_ptr<FImpl> Impl;
    FFrameExecutorServiceRetention();
    friend IAMSPEED_API std::unique_ptr<FFrameExecutorServiceRetention> RetainConfiguredFrameExecutorService();
public:
    ~FFrameExecutorServiceRetention();
    FFrameExecutorServiceRetention(const FFrameExecutorServiceRetention&)=delete;
    FFrameExecutorServiceRetention& operator=(const FFrameExecutorServiceRetention&)=delete;
};
// Data-only observation for native lifecycle controls, never a capability.
struct FFrameExecutorServiceObservation
{
    bool Configured=false,Stopping=false;
    int32 ActualRetentions=0,ActiveDispatches=0,Workers=0;
};
IAMSPEED_API FFrameExecutorServiceObservation ObserveFrameExecutorServiceForDiagnostics();
// Module shutdown, after simulation owners have drained. Stops admission,
// waits for all actual retained consumers and dispatches, then joins all owned
// threads; no thread kill. Never call on a worker or while retaining the same
// service on the calling stack: the original owner must release first.
IAMSPEED_API void ShutdownFrameParallelExecutor();
}
