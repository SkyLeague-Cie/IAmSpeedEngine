#pragma once
#include "CoreMinimal.h"
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
// Module shutdown, after simulation owners have drained. Stops admission,
// joins every accepted work group, then joins all owned threads; no thread kill.
IAMSPEED_API void ShutdownFrameParallelExecutor();
}
