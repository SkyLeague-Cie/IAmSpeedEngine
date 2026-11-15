#pragma once
#include "CoreMinimal.h"
#include "PinnedCanonicalFrameWindow.h"
class ASpeedPhysicsSimulation;
class USpeedWorldSubsystem;
class FSimulationWorker;
namespace Speed { class FHostedGTStaticWorldLease; class IStaticCollisionWorld; }
namespace Speed::OnePausedWork
{
// Scheduling evidence only, never contact/response admission.
enum class EQuiescence : uint8 { None, ParkedForThisNonce, ClosedAndJoined };
struct FResult
{
    uint64 Nonce=0;
    bool WorkCalled=false,WorkReturned=false,DeadlineExceeded=false,GuardsAccepted=false;
    bool WindowAcceptedBeforeWork=false;uint64 ReservedParkSerial=0;
    uint8 WorkResult=0;
    EQuiescence Quiescence=EQuiescence::None;
};
struct FCanonicalIdentity
{
    const ASpeedPhysicsSimulation* Host=nullptr;
    const USpeedWorldSubsystem* Bridge=nullptr;
    const IStaticCollisionWorld* Provider=nullptr;
    const FSimulationWorker* Worker=nullptr;
    uint64 Generation=0,Frame=0;
    float Delta=0,SimTime=0;
    FCanonicalFrameWindowPtr Window;
};
// TLS is installed only by the actual worker around its existing Work. A pure
// scheduler test has no Host identity and cannot authorize any Host call.
IAMSPEED_API bool IsActive() noexcept;
IAMSPEED_API bool IsPreparationPrefixFor(const ASpeedPhysicsSimulation*) noexcept;
IAMSPEED_API bool IsPostPrepareInitialPositionFor(const ASpeedPhysicsSimulation*) noexcept;
IAMSPEED_API bool IsTerminalStepSyncFor(const ASpeedPhysicsSimulation*) noexcept;
IAMSPEED_API bool HasTerminalStepSyncStop(const ASpeedPhysicsSimulation*) noexcept;
IAMSPEED_API bool SealTerminalStepSyncStop(const ASpeedPhysicsSimulation*) noexcept;
IAMSPEED_API bool BeginTerminalStepSync(const ASpeedPhysicsSimulation*,const USpeedWorldSubsystem*,
    const IStaticCollisionWorld*,uint64 Frame,float Delta,float SimTime) noexcept;
IAMSPEED_API bool IsTerminalSphereRsCaptureFor(const ASpeedPhysicsSimulation*) noexcept;
IAMSPEED_API bool HasTerminalSphereRsCaptureStop(const ASpeedPhysicsSimulation*) noexcept;
IAMSPEED_API bool SealTerminalSphereRsCaptureStop(const ASpeedPhysicsSimulation*) noexcept;
IAMSPEED_API bool BeginTerminalSphereRsCapture(const ASpeedPhysicsSimulation*,const USpeedWorldSubsystem*,
    const IStaticCollisionWorld*,uint64 Frame,float Delta,float SimTime) noexcept;
IAMSPEED_API bool IsTerminalConsumedPrefixAdvanceFor(const ASpeedPhysicsSimulation*) noexcept;
IAMSPEED_API bool IsTerminalRsContactAdvanceFor(const ASpeedPhysicsSimulation*) noexcept;
IAMSPEED_API bool IsTerminalRsResponseFor(const ASpeedPhysicsSimulation*) noexcept;
IAMSPEED_API bool IsTerminalRsResponseSyncFor(const ASpeedPhysicsSimulation*) noexcept;
IAMSPEED_API bool HasTerminalConsumedPrefixAdvanceStop(const ASpeedPhysicsSimulation*) noexcept;
IAMSPEED_API bool HasTerminalRsContactAdvanceStop(const ASpeedPhysicsSimulation*) noexcept;
IAMSPEED_API bool HasTerminalRsResponseStop(const ASpeedPhysicsSimulation*) noexcept;
IAMSPEED_API bool HasTerminalRsResponseSyncStop(const ASpeedPhysicsSimulation*) noexcept;
IAMSPEED_API bool SealTerminalConsumedPrefixAdvanceStop(const ASpeedPhysicsSimulation*) noexcept;
IAMSPEED_API bool SealTerminalRsContactAdvanceStop(const ASpeedPhysicsSimulation*) noexcept;
IAMSPEED_API bool SealTerminalRsResponseStop(const ASpeedPhysicsSimulation*) noexcept;
IAMSPEED_API bool SealTerminalRsResponseSyncStop(const ASpeedPhysicsSimulation*) noexcept;
IAMSPEED_API bool BeginTerminalConsumedPrefixAdvance(const ASpeedPhysicsSimulation*,const USpeedWorldSubsystem*,
    const IStaticCollisionWorld*,uint64 Frame,float Delta,float SimTime) noexcept;
IAMSPEED_API bool BeginTerminalRsContactAdvance(const ASpeedPhysicsSimulation*,const USpeedWorldSubsystem*,
    const IStaticCollisionWorld*,uint64 Frame,float Delta,float SimTime) noexcept;
IAMSPEED_API bool BeginTerminalRsResponse(const ASpeedPhysicsSimulation*,const USpeedWorldSubsystem*,
    const IStaticCollisionWorld*,uint64 Frame,float Delta,float SimTime) noexcept;
IAMSPEED_API bool BeginTerminalRsResponseSync(const ASpeedPhysicsSimulation*,const USpeedWorldSubsystem*,
    const IStaticCollisionWorld*,uint64 Frame,float Delta,float SimTime) noexcept;
// A distinct terminal query2 capture; never a complete frame or response2.
IAMSPEED_API bool IsTerminalPostSyncRsCaptureFor(const ASpeedPhysicsSimulation*) noexcept;
IAMSPEED_API bool HasTerminalPostSyncRsCaptureStop(const ASpeedPhysicsSimulation*) noexcept;
IAMSPEED_API bool SealTerminalPostSyncRsCaptureStop(const ASpeedPhysicsSimulation*) noexcept;
IAMSPEED_API bool BeginTerminalPostSyncRsCapture(const ASpeedPhysicsSimulation*,const USpeedWorldSubsystem*,
    const IStaticCollisionWorld*,uint64 Frame,float Delta,float SimTime) noexcept;
IAMSPEED_API bool HasConfirmedTerminalStepSerial(const USpeedWorldSubsystem*,uint64 Serial) noexcept;
IAMSPEED_API bool HasPostPrepareInitialPositionStop(const ASpeedPhysicsSimulation*) noexcept;
IAMSPEED_API bool SealPostPrepareInitialPositionStop(const ASpeedPhysicsSimulation*) noexcept;
IAMSPEED_API bool SealPreparationStop(const ASpeedPhysicsSimulation*) noexcept;
IAMSPEED_API bool HasPreparationStop(const ASpeedPhysicsSimulation*) noexcept;
IAMSPEED_API bool BeginRun(const ASpeedPhysicsSimulation*,const USpeedWorldSubsystem*,
    const FSimulationWorker*,uint64 Generation,uint64 Frame,uint32 Count,bool NoRollback) noexcept;
IAMSPEED_API bool BeginStep(const ASpeedPhysicsSimulation*,uint64 Frame,bool Resimulation,float Delta,float SimTime) noexcept;
IAMSPEED_API bool BeginWorldStep(const ASpeedPhysicsSimulation*,const USpeedWorldSubsystem*,
    const IStaticCollisionWorld*,uint64 Frame,float Delta,float SimTime) noexcept;
IAMSPEED_API bool ReadExecutionWindow(FCanonicalFrameWindowPtr& Out) noexcept;
IAMSPEED_API bool BeforeWorldEffects(const USpeedWorldSubsystem*,const IStaticCollisionWorld*,uint64 Serial,uint64 Frame,float Delta,float SimTime) noexcept;
IAMSPEED_API bool ConfirmWorldSerial(uint64 Serial) noexcept;
IAMSPEED_API bool RefuseLegacySimulate() noexcept;
}
