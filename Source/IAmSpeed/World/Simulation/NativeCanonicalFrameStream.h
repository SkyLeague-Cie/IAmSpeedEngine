#pragma once
#include "CoreMinimal.h"
#include "PinnedCanonicalFrameWindow.h"
class ASpeedPhysicsSimulation;class USpeedWorldSubsystem;class FSimulationWorker;
struct FOnePausedWorkRequest;
enum class ESimulationWorkerResult : uint8;
namespace Speed {class FPreparedNativeFrameConsumer;}
namespace Speed::OnePausedWork
{
// Lexical native context around the existing RealTime pulse. This scope does
// not schedule Work, drive a frame, alter the pacer, or borrow a parked nonce.
class IAMSPEED_API FNativeFrameStreamScope final
{
    TUniquePtr<FOnePausedWorkRequest> Request;
    FCanonicalFrameWindowPtr Window;
    TSharedPtr<const FPreparedNativeFrameConsumer,ESPMode::ThreadSafe> Consumer;
    uint64 FrameBefore=0;
    bool Finished=false;
public:
    FNativeFrameStreamScope(ASpeedPhysicsSimulation*,USpeedWorldSubsystem*,FSimulationWorker*,
        uint64 Frame,float Delta,float SimTime);
    ~FNativeFrameStreamScope() noexcept;
    ESimulationWorkerResult Finish(ESimulationWorkerResult,uint64 ActualFrameAfter);
    FNativeFrameStreamScope(const FNativeFrameStreamScope&)=delete;
    FNativeFrameStreamScope& operator=(const FNativeFrameStreamScope&)=delete;
};
}
