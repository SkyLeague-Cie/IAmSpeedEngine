#include "SimulationWorker.h"
#include "NativeCanonicalFrameStream.h"
#include "IAmSpeed/World/Collision/PreparedNativeFrameConsumer.h"
#include "IAmSpeed/World/Collision/NativePreparationPrefix.h"
#include "IAmSpeed/World/Collision/PreparedPhysicalExecution.h"
#include "OwnedFrameAudit.h"
#include "IAmSpeed/World/Collision/PreparedWorldSnapshot.h"
#include "IAmSpeed/World/Collision/StaticCollisionWorld.h"
#include "IAmSpeed/World/Simulation/SpeedPhysicsSimulation.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"

#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "HAL/RunnableThread.h"
#include "Misc/ScopeLock.h"
#include <cstring>
#include <stdexcept>

#if WITH_DEV_AUTOMATION_TESTS
namespace Speed::Testing
{
namespace
{
thread_local FSimulationWaitAuditObserver WaitAuditObserver = nullptr;
thread_local void* WaitAuditContext = nullptr;
}
bool SetSimulationWaitAuditObserverForCurrentThread(
    FSimulationWaitAuditObserver Observer, void* Context) noexcept
{
    if (!Observer || !Context || WaitAuditObserver) return false;
    WaitAuditObserver = Observer; WaitAuditContext = Context; return true;
}
bool ClearSimulationWaitAuditObserverForCurrentThread(
    FSimulationWaitAuditObserver Observer, void* Context) noexcept
{
    if (WaitAuditObserver != Observer || WaitAuditContext != Context) return false;
    WaitAuditObserver = nullptr; WaitAuditContext = nullptr; return true;
}
static void ObserveSimulationWait(ESimulationWaitAuditEvent Event,
    double Deadline, double Precision, double Remaining,
    uint32 RequestedMilliseconds = 0, int32 Outcome = -1) noexcept
{
    auto* const OwnedAtEntry = Speed::OwnedFrameAudit::ActiveForCurrentThread();
    if (!WaitAuditObserver && !OwnedAtEntry) return;
    FSimulationWaitAuditSample Sample;
    Sample.Event = Event;
    Sample.WallTicks = FPlatformTime::Cycles64();
    FMemory::Memcpy(&Sample.DeadlineBits, &Deadline, sizeof(Deadline));
    FMemory::Memcpy(&Sample.PrecisionBits, &Precision, sizeof(Precision));
    FMemory::Memcpy(&Sample.RemainingBits, &Remaining, sizeof(Remaining));
    Sample.RequestedMilliseconds = RequestedMilliseconds; Sample.Outcome = Outcome;
    if (WaitAuditObserver) WaitAuditObserver(WaitAuditContext, Sample);
    Speed::OwnedFrameAudit::RecordWaitSample(OwnedAtEntry, Sample);
}
}
#endif

FSimulationWorker::FSimulationWorker(
	FWork&& InWork, FWaitBetweenFrames&& InWaitBetweenFrames,
	FServiceBoundary InServiceBoundary, FCloseOnWorker InCloseOnWorker, FCloseOnWorker InStructuralBoundaryReleased)
	: StructuralBoundaryReleased(MoveTemp(InStructuralBoundaryReleased))
    , Work(MoveTemp(InWork))
	, WaitBetweenFrames(MoveTemp(InWaitBetweenFrames))
	, ServiceBoundary(MoveTemp(InServiceBoundary))
	, CloseOnWorker(MoveTemp(InCloseOnWorker))
	, WakeEvent(FPlatformProcess::GetSynchEventFromPool(false))
	, PauseAcknowledgedEvent(FPlatformProcess::GetSynchEventFromPool(true))
	, BoundarySuspendedEvent(FPlatformProcess::GetSynchEventFromPool(true))
{
}

ESimulationWaitResult FSimulationWorkerWaitContext::WaitUntil(
	const double DeadlineSeconds,
	const double PrecisionWindowSeconds) const
{
	const double ClampedPrecisionWindowSeconds = FMath::Clamp(
		PrecisionWindowSeconds, 0.0, 0.005);
#if WITH_DEV_AUTOMATION_TESTS
        Speed::Testing::ObserveSimulationWait(Speed::Testing::ESimulationWaitAuditEvent::Entry,
            DeadlineSeconds, ClampedPrecisionWindowSeconds, 0.0, 0, -1);
#endif
	while (!StopRequested.Load() && !Paused.Load() && !LiveBoundaryRequested.Load())
	{
		const double RemainingSeconds = DeadlineSeconds - FPlatformTime::Seconds();
#if WITH_DEV_AUTOMATION_TESTS
        Speed::Testing::ObserveSimulationWait(Speed::Testing::ESimulationWaitAuditEvent::RemainingClockSample,
            DeadlineSeconds, ClampedPrecisionWindowSeconds, RemainingSeconds, 0, -1);
#endif
		if (RemainingSeconds <= 0.0)
		{
#if WITH_DEV_AUTOMATION_TESTS
        Speed::Testing::ObserveSimulationWait(Speed::Testing::ESimulationWaitAuditEvent::Return,
            DeadlineSeconds, ClampedPrecisionWindowSeconds, RemainingSeconds, 0, int32(ESimulationWaitResult::DeadlineReached));
#endif
			return ESimulationWaitResult::DeadlineReached;
		}
		const double CoarseWaitSeconds =
			RemainingSeconds - ClampedPrecisionWindowSeconds;
		const int32 CoarseWaitMilliseconds = FMath::FloorToInt(
			CoarseWaitSeconds * 1000.0);
		if (CoarseWaitMilliseconds > 0)
		{
#if WITH_DEV_AUTOMATION_TESTS
            const uint32 RequestedMilliseconds = static_cast<uint32>(FMath::Min(CoarseWaitMilliseconds, 100));
        Speed::Testing::ObserveSimulationWait(Speed::Testing::ESimulationWaitAuditEvent::EventWaitEntry,
            DeadlineSeconds, ClampedPrecisionWindowSeconds, RemainingSeconds, RequestedMilliseconds, -1);
            const bool SignalObserved = WakeEvent.Wait(RequestedMilliseconds);
        Speed::Testing::ObserveSimulationWait(Speed::Testing::ESimulationWaitAuditEvent::EventWaitExit,
            DeadlineSeconds, ClampedPrecisionWindowSeconds, RemainingSeconds, RequestedMilliseconds, int32(SignalObserved));
#else
			WakeEvent.Wait(static_cast<uint32>(FMath::Min(
				CoarseWaitMilliseconds, 100)));
#endif
		}
		else if (ClampedPrecisionWindowSeconds > 0.0)
		{
			// Yielding keeps the short precision tail interruptible and avoids the
			// systematic oversleep caused by rounding a sub-millisecond tail up.
#if WITH_DEV_AUTOMATION_TESTS
        Speed::Testing::ObserveSimulationWait(Speed::Testing::ESimulationWaitAuditEvent::YieldEntry,
            DeadlineSeconds, ClampedPrecisionWindowSeconds, RemainingSeconds, 0, -1);
#endif
			// Processor pause is confined to the existing precision window.
			// The sub-millisecond coarse remainder still yields the thread.
			if (RemainingSeconds <= ClampedPrecisionWindowSeconds)
			{
				FPlatformProcess::Yield();
			}
			else
			{
				FPlatformProcess::YieldThread();
			}
#if WITH_DEV_AUTOMATION_TESTS
        Speed::Testing::ObserveSimulationWait(Speed::Testing::ESimulationWaitAuditEvent::YieldExit,
            DeadlineSeconds, ClampedPrecisionWindowSeconds, RemainingSeconds, 0, -1);
#endif

		}
		else
		{
#if WITH_DEV_AUTOMATION_TESTS
        Speed::Testing::ObserveSimulationWait(Speed::Testing::ESimulationWaitAuditEvent::EventWaitEntry,
            DeadlineSeconds, ClampedPrecisionWindowSeconds, RemainingSeconds, 1, -1);
            const bool SignalObserved = WakeEvent.Wait(1);
        Speed::Testing::ObserveSimulationWait(Speed::Testing::ESimulationWaitAuditEvent::EventWaitExit,
            DeadlineSeconds, ClampedPrecisionWindowSeconds, RemainingSeconds, 1, int32(SignalObserved));
#else
			WakeEvent.Wait(1);
#endif
		}
	}
#if WITH_DEV_AUTOMATION_TESTS
        Speed::Testing::ObserveSimulationWait(Speed::Testing::ESimulationWaitAuditEvent::Return,
            DeadlineSeconds, ClampedPrecisionWindowSeconds, 0.0, 0, int32(ESimulationWaitResult::Interrupted));
#endif
	return ESimulationWaitResult::Interrupted;
}


struct FOnePausedWorkRequest
{
    Speed::OnePausedWork::FCanonicalIdentity Identity;
    uint64 Nonce=0;
    bool WorkCalled=false,WorkReturned=false,GuardsAccepted=false;
    bool WindowAcceptedBeforeWork=false,WorldEffectsEntered=false;uint64 ReservedParkSerial=0;
    ESimulationWorkerResult Result=ESimulationWorkerResult::Failed;
    bool RunEntered=false,StepEntered=false,WorldStepEntered=false,GuardRejected=false;
    bool PreparationStop=false,TerminalStepSyncEntered=false,TerminalConsumedPrefixAdvanceEntered=false,TerminalRsContactAdvanceEntered=false,TerminalRsResponseEntered=false,TerminalRsResponseSyncEntered=false,TerminalPostSyncRsCaptureEntered=false,TerminalRsCaptureEntered=false,WorldSerialConfirmed=false;
    TAtomic<uint64> ParkedNonce{0};
};
namespace Speed::OnePausedWork
{
namespace
{
thread_local FOnePausedWorkRequest* Current=nullptr;
bool Reject() noexcept {Current->GuardRejected=true;if(Current->Identity.Window)Current->Identity.Window->Close();return false;}
bool SameFloat(float a,float b) noexcept
{
    uint32 x=0,y=0;std::memcpy(&x,&a,sizeof(x));std::memcpy(&y,&b,sizeof(y));return x==y;
}
}
uint64 FCanonicalFrameWindow::AllocateSerialForNativeWork()
{
    static std::atomic<uint64> next{0};uint64 before=next.load(std::memory_order_relaxed);
    for(;;){if(before==MAX_uint64)return 0;if(next.compare_exchange_weak(before,before+1,std::memory_order_relaxed))return before+1;}
}
uint64 FCanonicalFrameWindow::AllocateSerialOnGT()
{return IsInGameThread()?AllocateSerialForNativeWork():0;}
#include "NativeCanonicalFrameStream.inl"
bool IsActive() noexcept {return Current!=nullptr;}
bool FCanonicalFrameWindow::HasRequiredProviderReadiness(const Speed::IStaticCollisionWorld* provider) const noexcept
{
    if (!provider || !IsOpen()) return false;
    if (Purpose == EPurpose::FullFrame) return !PreparationContract && provider->IsReadyForPhysicsStep();
    return (Purpose==EPurpose::PreparationPrefixStop||HasInitialPosition(Purpose))&&!provider->IsReadyForPhysicsStep()&&
        PreparationContract&&PreparationContract->GetPurpose()==Purpose&&PreparationContract->IsFor(Snapshot,Serial);
}
bool IsPreparationPrefixFor(const ASpeedPhysicsSimulation* host) noexcept
{
    FCanonicalFrameWindowPtr w;
    return Current && host && Current->Identity.Host == host && ReadExecutionWindow(w) &&
        w->GetPurpose() == EPurpose::PreparationPrefixStop;
}
bool IsPostPrepareInitialPositionFor(const ASpeedPhysicsSimulation* host) noexcept
{
    FCanonicalFrameWindowPtr w;
    return Current&&host&&Current->Identity.Host==host&&ReadExecutionWindow(w)&&w->GetPurpose()==EPurpose::PostPrepareInitialPositionStop;
}
bool IsTerminalStepSyncFor(const ASpeedPhysicsSimulation* host) noexcept
{
    FCanonicalFrameWindowPtr w;
    return Current&&host&&Current->Identity.Host==host&&ReadExecutionWindow(w)&&w->GetPurpose()==EPurpose::TerminalStepSyncStop;
}
bool IsTerminalSphereRsCaptureFor(const ASpeedPhysicsSimulation* host) noexcept
{
    FCanonicalFrameWindowPtr w;
    return Current&&host&&Current->Identity.Host==host&&ReadExecutionWindow(w)&&w->GetPurpose()==EPurpose::TerminalSphereRsCaptureStop;
}
bool HasTerminalSphereRsCaptureStop(const ASpeedPhysicsSimulation* host) noexcept
{return Current&&IsTerminalSphereRsCaptureFor(host)&&Current->PreparationStop&&!Current->GuardRejected;}
bool SealTerminalSphereRsCaptureStop(const ASpeedPhysicsSimulation* host) noexcept
{
    if((Current&&Current->TerminalRsResponseEntered)||!Current||!IsTerminalSphereRsCaptureFor(host)||!Current->RunEntered||!Current->StepEntered||
        !Current->TerminalRsCaptureEntered||Current->TerminalStepSyncEntered||!Current->WorldStepEntered||!Current->WorldEffectsEntered||
        !Current->WorldSerialConfirmed||Current->PreparationStop||!PreparationPrefix::FCursor::IsStoppedForCurrentCall(host))return Current?Reject():false;
    const auto& contract=Current->Identity.Window->GetPreparationContract();
    if(!contract||!contract->HasClosedNativeRsCapture())return Reject();
    Current->PreparationStop=true;return true;
}
bool IsTerminalConsumedPrefixAdvanceFor(const ASpeedPhysicsSimulation* host) noexcept
{
    FCanonicalFrameWindowPtr w;
    return Current&&host&&Current->Identity.Host==host&&ReadExecutionWindow(w)&&w->GetPurpose()==EPurpose::TerminalConsumedPrefixAdvanceStop;
}
bool HasTerminalConsumedPrefixAdvanceStop(const ASpeedPhysicsSimulation* host) noexcept
{return Current&&IsTerminalConsumedPrefixAdvanceFor(host)&&Current->PreparationStop&&!Current->GuardRejected;}
bool SealTerminalConsumedPrefixAdvanceStop(const ASpeedPhysicsSimulation* host) noexcept
{
    if((Current&&Current->TerminalRsResponseEntered)||!Current||!IsTerminalConsumedPrefixAdvanceFor(host)||!Current->RunEntered||!Current->StepEntered||
        !Current->TerminalConsumedPrefixAdvanceEntered||Current->TerminalRsContactAdvanceEntered||Current->TerminalStepSyncEntered||Current->TerminalRsCaptureEntered||!Current->WorldStepEntered||!Current->WorldEffectsEntered||
        !Current->WorldSerialConfirmed||Current->PreparationStop||!PreparationPrefix::FCursor::IsStoppedForCurrentCall(host))return Current?Reject():false;
    const auto& contract=Current->Identity.Window->GetPreparationContract();
    if(!contract||!contract->HasCompletedNativeRsAdvance())return Reject();
    Current->PreparationStop=true;return true;
}
bool IsTerminalRsContactAdvanceFor(const ASpeedPhysicsSimulation* host) noexcept
{
    FCanonicalFrameWindowPtr w;
    return Current&&host&&Current->Identity.Host==host&&ReadExecutionWindow(w)&&w->GetPurpose()==EPurpose::TerminalRsContactAdvanceStop;
}
bool HasTerminalRsContactAdvanceStop(const ASpeedPhysicsSimulation* host) noexcept
{return Current&&IsTerminalRsContactAdvanceFor(host)&&Current->PreparationStop&&!Current->GuardRejected;}
bool SealTerminalRsContactAdvanceStop(const ASpeedPhysicsSimulation* host) noexcept
{
    if((Current&&Current->TerminalRsResponseEntered)||!Current||!IsTerminalRsContactAdvanceFor(host)||!Current->RunEntered||!Current->StepEntered||
        !Current->TerminalRsContactAdvanceEntered||Current->TerminalConsumedPrefixAdvanceEntered||Current->TerminalStepSyncEntered||Current->TerminalRsCaptureEntered||!Current->WorldStepEntered||!Current->WorldEffectsEntered||
        !Current->WorldSerialConfirmed||Current->PreparationStop||!PreparationPrefix::FCursor::IsStoppedForCurrentCall(host))return Current?Reject():false;
    const auto& contract=Current->Identity.Window->GetPreparationContract();
    if(!contract||!contract->HasCompletedNativeRsContactAdvance())return Reject();
    Current->PreparationStop=true;return true;
}
bool HasTerminalStepSyncStop(const ASpeedPhysicsSimulation* host) noexcept
{return Current&&IsTerminalStepSyncFor(host)&&Current->PreparationStop&&!Current->GuardRejected;}
bool SealTerminalStepSyncStop(const ASpeedPhysicsSimulation* host) noexcept
{
    if((Current&&Current->TerminalRsResponseEntered)||!Current||!IsTerminalStepSyncFor(host)||!Current->RunEntered||!Current->StepEntered||
        !Current->TerminalStepSyncEntered||!Current->WorldStepEntered||!Current->WorldEffectsEntered||
        !Current->WorldSerialConfirmed||Current->PreparationStop||!PreparationPrefix::FCursor::IsStoppedForCurrentCall(host))return Current?Reject():false;
    const auto& contract=Current->Identity.Window->GetPreparationContract();
    if(!contract||!contract->HasCompletedStepSync())return Reject();
    Current->PreparationStop=true;return true;
}
bool HasPostPrepareInitialPositionStop(const ASpeedPhysicsSimulation* host) noexcept
{return Current&&IsPostPrepareInitialPositionFor(host)&&Current->PreparationStop&&!Current->GuardRejected;}
bool SealPostPrepareInitialPositionStop(const ASpeedPhysicsSimulation* host) noexcept
{
    if(!Current||!IsPostPrepareInitialPositionFor(host)||!Current->RunEntered||!Current->StepEntered||
        Current->WorldStepEntered||Current->WorldEffectsEntered||Current->PreparationStop||
        !PreparationPrefix::FCursor::IsStoppedForCurrentCall(host))return Current?Reject():false;
    const auto& contract=Current->Identity.Window->GetPreparationContract();
    if(!contract||!contract->HasAcceptedPosition())return Reject();
    Current->PreparationStop=true;return true;
}
bool HasPreparationStop(const ASpeedPhysicsSimulation* host) noexcept
{ return Current && host && Current->Identity.Host == host && Current->PreparationStop && !Current->GuardRejected; }
bool SealPreparationStop(const ASpeedPhysicsSimulation* host) noexcept
{
    if (!Current || !IsPreparationPrefixFor(host) || !Current->RunEntered || !Current->StepEntered ||
        Current->WorldStepEntered || Current->WorldEffectsEntered || Current->PreparationStop ||
        !PreparationPrefix::FCursor::IsStoppedForCurrentCall(host)) return Current ? Reject() : false;
    Current->PreparationStop = true; return true;
}

bool ReadExecutionWindow(FCanonicalFrameWindowPtr& out) noexcept
{
    out.Reset();if(!Current||!Current->Identity.Host||!Current->WindowAcceptedBeforeWork)return false;
    const auto& i=Current->Identity;const auto& w=i.Window;
    // Check revocation before dereferencing any native/UObject identity.
    if(!w||!w->IsOpen()||w->GetNonce()!=Current->Nonce||!w->Matches(i.Host,i.Bridge,i.Provider,i.Worker,i.Generation,i.Frame,i.Delta,i.SimTime))return false;
    if(!w->HasRequiredProviderReadiness(i.Provider)||i.Provider->GetPreparedSnapshot()!=w->GetSnapshot()||i.Bridge->GetStaticCollisionWorld()!=i.Provider||i.Host->GetInputWorkerGeneration()!=i.Generation||
        i.Host->GetWorld()!=w->GetWorldIdentity()||i.Bridge->GetWorld()!=w->GetWorldIdentity()||w->GetSnapshot()->GetOrigin().World!=w->GetWorldIdentity())return false;
    out=w;return true;
}
bool WindowCurrent() noexcept {FCanonicalFrameWindowPtr w;return ReadExecutionWindow(w);}
bool BeforeWorldEffects(const USpeedWorldSubsystem* b,const IStaticCollisionWorld* p,uint64 serial,uint64 frame,float dt,float time) noexcept
{
    if(!Current)return true;
    const auto& i=Current->Identity;
    if(!WindowCurrent()||b!=i.Bridge||p!=i.Provider||frame!=i.Frame||!SameFloat(dt,i.Delta)||!SameFloat(time,i.SimTime)||
        !Current->WorldStepEntered||Current->WorldEffectsEntered||serial!=i.Window->GetStepBefore()||serial==MAX_uint64)return Reject();
    Current->WorldEffectsEntered=true;return true;
}
bool ConfirmWorldSerial(uint64 serial) noexcept
{
    if(!Current)return true;
    if(!WindowCurrent()||!Current->WorldEffectsEntered||Current->WorldSerialConfirmed||serial!=Current->Identity.Window->GetExpectedStep())return Reject();
    Current->WorldSerialConfirmed=true;return true;
}
bool HasConfirmedTerminalStepSerial(const USpeedWorldSubsystem* bridge,uint64 serial) noexcept
{
    return Current&&WindowCurrent()&&bridge==Current->Identity.Bridge&&
        ((Current->Identity.Window->GetPurpose()==EPurpose::TerminalStepSyncStop&&Current->TerminalStepSyncEntered&&!Current->TerminalRsResponseEntered&&!Current->TerminalRsCaptureEntered)||
         (Current->Identity.Window->GetPurpose()==EPurpose::TerminalSphereRsCaptureStop&&Current->TerminalRsCaptureEntered&&!Current->TerminalRsResponseEntered&&!Current->TerminalStepSyncEntered)||
         (Current->Identity.Window->GetPurpose()==EPurpose::TerminalConsumedPrefixAdvanceStop&&Current->TerminalConsumedPrefixAdvanceEntered&&!Current->TerminalRsResponseEntered&&!Current->TerminalRsContactAdvanceEntered&&!Current->TerminalStepSyncEntered&&!Current->TerminalRsCaptureEntered)||
         (Current->Identity.Window->GetPurpose()==EPurpose::TerminalRsContactAdvanceStop&&Current->TerminalRsContactAdvanceEntered&&!Current->TerminalRsResponseEntered&&!Current->TerminalConsumedPrefixAdvanceEntered&&!Current->TerminalStepSyncEntered&&!Current->TerminalRsCaptureEntered)||
         (Current->Identity.Window->GetPurpose()==EPurpose::TerminalRsResponseStop&&Current->TerminalRsResponseEntered&&!Current->TerminalRsResponseSyncEntered&&
           !Current->TerminalRsContactAdvanceEntered&&!Current->TerminalConsumedPrefixAdvanceEntered&&!Current->TerminalStepSyncEntered&&!Current->TerminalRsCaptureEntered)||
         (Current->Identity.Window->GetPurpose()==EPurpose::TerminalRsResponseSyncStop&&Current->TerminalRsResponseSyncEntered&&!Current->TerminalRsResponseEntered&&
           !Current->TerminalRsContactAdvanceEntered&&!Current->TerminalConsumedPrefixAdvanceEntered&&!Current->TerminalStepSyncEntered&&!Current->TerminalRsCaptureEntered)||
         (Current->Identity.Window->GetPurpose()==EPurpose::TerminalPostSyncRsCaptureStop&&Current->TerminalPostSyncRsCaptureEntered&&
           !Current->TerminalRsResponseSyncEntered&&!Current->TerminalRsResponseEntered&&!Current->TerminalRsContactAdvanceEntered&&
           !Current->TerminalConsumedPrefixAdvanceEntered&&!Current->TerminalStepSyncEntered&&!Current->TerminalRsCaptureEntered))&&
        Current->WorldStepEntered&&Current->WorldEffectsEntered&&Current->WorldSerialConfirmed&&
        serial==Current->Identity.Window->GetExpectedStep();
}

bool BeginRun(const ASpeedPhysicsSimulation* h,const USpeedWorldSubsystem* b,const FSimulationWorker* w,
    uint64 generation,uint64 frame,uint32 count,bool noRollback) noexcept
{
    if(!Current)return true;
    const auto& i=Current->Identity;
    if(!WindowCurrent()||!h||h!=i.Host||b!=i.Bridge||w!=i.Worker||generation!=i.Generation||frame!=i.Frame||
        count!=1||!noRollback||Current->RunEntered)return Reject();
    Current->RunEntered=true;return true;
}
bool BeginStep(const ASpeedPhysicsSimulation* h,uint64 frame,bool resimulation,float dt,float time) noexcept
{
    if(!Current)return true;
    if(!WindowCurrent()||!h||h!=Current->Identity.Host||frame!=Current->Identity.Frame||resimulation||
        !SameFloat(dt,Current->Identity.Delta)||!SameFloat(time,Current->Identity.SimTime)||
        !Current->RunEntered||Current->StepEntered)return Reject();
    Current->StepEntered=true;return true;
}
bool BeginWorldStep(const ASpeedPhysicsSimulation* h,const USpeedWorldSubsystem* b,
    const IStaticCollisionWorld* p,uint64 frame,float dt,float time) noexcept
{
    if(!Current)return true;
    const auto& i=Current->Identity;
    if(!WindowCurrent()||!h||h!=i.Host||b!=i.Bridge||!p||p!=i.Provider||frame!=i.Frame||
        !SameFloat(dt,i.Delta)||!SameFloat(time,i.SimTime)||!Current->StepEntered||Current->WorldStepEntered||
        i.Window->GetPurpose()!=EPurpose::FullFrame||Current->PreparationStop)return Reject();
    Current->WorldStepEntered=true;return true;
}
bool BeginTerminalStepSync(const ASpeedPhysicsSimulation* h,const USpeedWorldSubsystem* b,
    const IStaticCollisionWorld* p,uint64 frame,float dt,float time) noexcept
{
    if(!Current)return false;
    const auto& i=Current->Identity;
    if(Current->TerminalRsResponseEntered||!WindowCurrent()||!h||h!=i.Host||b!=i.Bridge||!p||p!=i.Provider||frame!=i.Frame||
        !SameFloat(dt,i.Delta)||!SameFloat(time,i.SimTime)||!Current->StepEntered||Current->WorldStepEntered||
        Current->TerminalStepSyncEntered||Current->WorldEffectsEntered||Current->WorldSerialConfirmed||Current->PreparationStop||
        i.Window->GetPurpose()!=EPurpose::TerminalStepSyncStop||!i.Window->GetPreparationContract()||
        !i.Window->GetPreparationContract()->HasAcceptedPosition())return Reject();
    Current->TerminalStepSyncEntered=true;Current->WorldStepEntered=true;return true;
}
bool BeginTerminalSphereRsCapture(const ASpeedPhysicsSimulation* h,const USpeedWorldSubsystem* b,
    const IStaticCollisionWorld* p,uint64 frame,float dt,float time) noexcept
{
    if(!Current)return false;
    const auto& i=Current->Identity;
    if(Current->TerminalRsResponseEntered||!WindowCurrent()||!h||h!=i.Host||b!=i.Bridge||!p||p!=i.Provider||frame!=i.Frame||
        !SameFloat(dt,i.Delta)||!SameFloat(time,i.SimTime)||!Current->StepEntered||Current->WorldStepEntered||
        Current->TerminalStepSyncEntered||Current->TerminalRsCaptureEntered||Current->WorldEffectsEntered||Current->WorldSerialConfirmed||Current->PreparationStop||
        i.Window->GetPurpose()!=EPurpose::TerminalSphereRsCaptureStop||!i.Window->GetPreparationContract()||
        !i.Window->GetPreparationContract()->HasAcceptedPosition())return Reject();
    Current->TerminalRsCaptureEntered=true;Current->WorldStepEntered=true;return true;
}
bool BeginTerminalConsumedPrefixAdvance(const ASpeedPhysicsSimulation* h,const USpeedWorldSubsystem* b,
    const IStaticCollisionWorld* p,uint64 frame,float dt,float time) noexcept
{
    if(!Current)return false;
    const auto& i=Current->Identity;
    if(Current->TerminalRsResponseEntered||!WindowCurrent()||!h||h!=i.Host||b!=i.Bridge||!p||p!=i.Provider||frame!=i.Frame||
        !SameFloat(dt,i.Delta)||!SameFloat(time,i.SimTime)||!Current->StepEntered||Current->WorldStepEntered||
        Current->TerminalStepSyncEntered||Current->TerminalRsCaptureEntered||Current->TerminalConsumedPrefixAdvanceEntered||Current->TerminalRsContactAdvanceEntered||Current->WorldEffectsEntered||Current->WorldSerialConfirmed||Current->PreparationStop||
        i.Window->GetPurpose()!=EPurpose::TerminalConsumedPrefixAdvanceStop||!i.Window->GetPreparationContract()||
        !i.Window->GetPreparationContract()->HasAcceptedPosition())return Reject();
    Current->TerminalConsumedPrefixAdvanceEntered=true;Current->WorldStepEntered=true;return true;
}
bool BeginTerminalRsContactAdvance(const ASpeedPhysicsSimulation* h,const USpeedWorldSubsystem* b,
    const IStaticCollisionWorld* p,uint64 frame,float dt,float time) noexcept
{
    if(!Current)return false;
    const auto& i=Current->Identity;
    if(Current->TerminalRsResponseEntered||!WindowCurrent()||!h||h!=i.Host||b!=i.Bridge||!p||p!=i.Provider||frame!=i.Frame||
        !SameFloat(dt,i.Delta)||!SameFloat(time,i.SimTime)||!Current->StepEntered||Current->WorldStepEntered||
        Current->TerminalStepSyncEntered||Current->TerminalRsCaptureEntered||Current->TerminalRsContactAdvanceEntered||Current->TerminalConsumedPrefixAdvanceEntered||Current->TerminalRsContactAdvanceEntered||Current->WorldEffectsEntered||Current->WorldSerialConfirmed||Current->PreparationStop||
        i.Window->GetPurpose()!=EPurpose::TerminalRsContactAdvanceStop||!i.Window->GetPreparationContract()||
        !i.Window->GetPreparationContract()->HasAcceptedPosition())return Reject();
    Current->TerminalRsContactAdvanceEntered=true;Current->WorldStepEntered=true;return true;
}
bool IsTerminalRsResponseFor(const ASpeedPhysicsSimulation* host) noexcept
{
    FCanonicalFrameWindowPtr w;
    return Current&&host&&Current->Identity.Host==host&&ReadExecutionWindow(w)&&w->GetPurpose()==EPurpose::TerminalRsResponseStop;
}
bool HasTerminalRsResponseStop(const ASpeedPhysicsSimulation* host) noexcept
{return Current&&IsTerminalRsResponseFor(host)&&Current->PreparationStop&&!Current->GuardRejected;}
bool SealTerminalRsResponseStop(const ASpeedPhysicsSimulation* host) noexcept
{
    if(!Current||!IsTerminalRsResponseFor(host)||!Current->RunEntered||!Current->StepEntered||
        !Current->TerminalRsResponseEntered||Current->TerminalRsContactAdvanceEntered||Current->TerminalConsumedPrefixAdvanceEntered||Current->TerminalStepSyncEntered||Current->TerminalRsCaptureEntered||!Current->WorldStepEntered||!Current->WorldEffectsEntered||
        !Current->WorldSerialConfirmed||Current->PreparationStop||!PreparationPrefix::FCursor::IsStoppedForCurrentCall(host))return Current?Reject():false;
    const auto& contract=Current->Identity.Window->GetPreparationContract();
    if(!contract||!contract->HasCompletedNativeRsResponse())return Reject();
    Current->PreparationStop=true;return true;
}
bool BeginTerminalRsResponse(const ASpeedPhysicsSimulation* h,const USpeedWorldSubsystem* b,
    const IStaticCollisionWorld* p,uint64 frame,float dt,float time) noexcept
{
    if(!Current)return false;
    const auto& i=Current->Identity;
    if(!WindowCurrent()||!h||h!=i.Host||b!=i.Bridge||!p||p!=i.Provider||frame!=i.Frame||
        !SameFloat(dt,i.Delta)||!SameFloat(time,i.SimTime)||!Current->StepEntered||Current->WorldStepEntered||
        Current->TerminalStepSyncEntered||Current->TerminalRsContactAdvanceEntered||Current->TerminalRsCaptureEntered||Current->TerminalRsResponseEntered||Current->TerminalConsumedPrefixAdvanceEntered||Current->TerminalRsResponseEntered||Current->WorldEffectsEntered||Current->WorldSerialConfirmed||Current->PreparationStop||
        i.Window->GetPurpose()!=EPurpose::TerminalRsResponseStop||!i.Window->GetPreparationContract()||
        !i.Window->GetPreparationContract()->HasAcceptedPosition())return Reject();
    Current->TerminalRsResponseEntered=true;Current->WorldStepEntered=true;return true;
}
bool IsTerminalRsResponseSyncFor(const ASpeedPhysicsSimulation* host) noexcept
{
    FCanonicalFrameWindowPtr w;
    return Current&&host&&Current->Identity.Host==host&&ReadExecutionWindow(w)&&w->GetPurpose()==EPurpose::TerminalRsResponseSyncStop;
}
bool HasTerminalRsResponseSyncStop(const ASpeedPhysicsSimulation* host) noexcept
{return Current&&IsTerminalRsResponseSyncFor(host)&&Current->PreparationStop&&!Current->GuardRejected;}
bool SealTerminalRsResponseSyncStop(const ASpeedPhysicsSimulation* host) noexcept
{
    if(!Current||!IsTerminalRsResponseSyncFor(host)||!Current->RunEntered||!Current->StepEntered||
        !Current->TerminalRsResponseSyncEntered||Current->TerminalRsResponseEntered||Current->TerminalRsContactAdvanceEntered||Current->TerminalConsumedPrefixAdvanceEntered||Current->TerminalStepSyncEntered||Current->TerminalRsCaptureEntered||!Current->WorldStepEntered||!Current->WorldEffectsEntered||
        !Current->WorldSerialConfirmed||Current->PreparationStop||!PreparationPrefix::FCursor::IsStoppedForCurrentCall(host))return Current?Reject():false;
    const auto& contract=Current->Identity.Window->GetPreparationContract();
    if(!contract||!contract->HasCompletedNativeRsResponseSync())return Reject();
    Current->PreparationStop=true;return true;
}
bool BeginTerminalRsResponseSync(const ASpeedPhysicsSimulation* h,const USpeedWorldSubsystem* b,
    const IStaticCollisionWorld* p,uint64 frame,float dt,float time) noexcept
{
    if(!Current)return false;
    const auto& i=Current->Identity;
    if(Current->TerminalRsResponseEntered||!WindowCurrent()||!h||h!=i.Host||b!=i.Bridge||!p||p!=i.Provider||frame!=i.Frame||
        !SameFloat(dt,i.Delta)||!SameFloat(time,i.SimTime)||!Current->StepEntered||Current->WorldStepEntered||
        Current->TerminalStepSyncEntered||Current->TerminalRsContactAdvanceEntered||Current->TerminalRsCaptureEntered||Current->TerminalRsResponseSyncEntered||Current->TerminalConsumedPrefixAdvanceEntered||Current->TerminalRsResponseSyncEntered||Current->WorldEffectsEntered||Current->WorldSerialConfirmed||Current->PreparationStop||
        i.Window->GetPurpose()!=EPurpose::TerminalRsResponseSyncStop||!i.Window->GetPreparationContract()||
        !i.Window->GetPreparationContract()->HasAcceptedPosition())return Reject();
    Current->TerminalRsResponseSyncEntered=true;Current->WorldStepEntered=true;return true;
}
bool IsTerminalPostSyncRsCaptureFor(const ASpeedPhysicsSimulation* host) noexcept
{
    FCanonicalFrameWindowPtr w;
    return Current&&host&&Current->Identity.Host==host&&ReadExecutionWindow(w)&&w->GetPurpose()==EPurpose::TerminalPostSyncRsCaptureStop;
}
bool HasTerminalPostSyncRsCaptureStop(const ASpeedPhysicsSimulation* host) noexcept
{return Current&&IsTerminalPostSyncRsCaptureFor(host)&&Current->PreparationStop&&!Current->GuardRejected;}
bool SealTerminalPostSyncRsCaptureStop(const ASpeedPhysicsSimulation* host) noexcept
{
    if(!Current||!IsTerminalPostSyncRsCaptureFor(host)||!Current->RunEntered||!Current->StepEntered||
        !Current->TerminalPostSyncRsCaptureEntered||Current->TerminalRsResponseSyncEntered||Current->TerminalRsResponseEntered||Current->TerminalRsContactAdvanceEntered||Current->TerminalConsumedPrefixAdvanceEntered||Current->TerminalStepSyncEntered||Current->TerminalRsCaptureEntered||!Current->WorldStepEntered||!Current->WorldEffectsEntered||
        !Current->WorldSerialConfirmed||Current->PreparationStop||!PreparationPrefix::FCursor::IsStoppedForCurrentCall(host))return Current?Reject():false;
    const auto& contract=Current->Identity.Window->GetPreparationContract();
    if(!contract||!contract->HasCompletedNativePostSyncRsCapture())return Reject();
    Current->PreparationStop=true;return true;
}
bool BeginTerminalPostSyncRsCapture(const ASpeedPhysicsSimulation* h,const USpeedWorldSubsystem* b,
    const IStaticCollisionWorld* p,uint64 frame,float dt,float time) noexcept
{
    if(!Current)return false;
    const auto& i=Current->Identity;
    if(Current->TerminalRsResponseSyncEntered||Current->TerminalRsResponseEntered||!WindowCurrent()||!h||h!=i.Host||b!=i.Bridge||!p||p!=i.Provider||frame!=i.Frame||
        !SameFloat(dt,i.Delta)||!SameFloat(time,i.SimTime)||!Current->StepEntered||Current->WorldStepEntered||
        Current->TerminalStepSyncEntered||Current->TerminalRsContactAdvanceEntered||Current->TerminalRsCaptureEntered||Current->TerminalPostSyncRsCaptureEntered||Current->TerminalConsumedPrefixAdvanceEntered||Current->TerminalPostSyncRsCaptureEntered||Current->WorldEffectsEntered||Current->WorldSerialConfirmed||Current->PreparationStop||
        i.Window->GetPurpose()!=EPurpose::TerminalPostSyncRsCaptureStop||!i.Window->GetPreparationContract()||
        !i.Window->GetPreparationContract()->HasAcceptedPosition())return Reject();
    Current->TerminalPostSyncRsCaptureEntered=true;Current->WorldStepEntered=true;return true;
}
bool RefuseLegacySimulate() noexcept {return Current?Reject():true;}
}
Speed::OnePausedWork::FResult FSimulationWorker::RunOnePausedWork(
    const Speed::OnePausedWork::FCanonicalIdentity& Identity,
    TFunctionRef<void()> ReleaseGTFence,uint32 TimeoutMilliseconds)
{
    using namespace Speed::OnePausedWork;
    FResult out;
    FOnePausedWorkRequest request;request.Identity=Identity;
    {
        FScopeLock control(&BoundaryControlMutex);
        if(!IsInGameThread()||!IsBoundaryServiceSuspended()||!bPaused.Load()||PendingOneWork.Load()||
            OneWorkReturnFence.Load()||NextOneWorkNonce==MAX_uint64)return out;
        const uint64 oldPark=BoundarySuspendRequestSerial.Load();
        if(oldPark>=MAX_uint64-1)return out; // reserve a new post-Work ACK without wrap
        request.ReservedParkSerial=oldPark+1;
        if(Identity.Host&&(!Identity.Window||!Identity.Window->IsOpen()||Identity.Window->GetNonce()))return out;

        request.Nonce=++NextOneWorkNonce;out.Nonce=request.Nonce;out.ReservedParkSerial=request.ReservedParkSerial;
        if(Identity.Host&&!Identity.Window->BindNonce(request.Nonce)){out.Nonce=0;return out;}
        OneWorkParkedNonce.Store(0);OneWorkReturnFence.Store(true);
        PendingOneWork.Store(&request);
    }
    // No callback is borrowed by the worker. Stack storage survives until the
    // matching post-Work park or actual thread join, including exceptions.
    try {ReleaseGTFence();}
    catch (...)
    {
        if(Identity.Window)Identity.Window->Close();
        StopAndJoin();PendingOneWork.Store(nullptr);
        out.Quiescence=EQuiescence::ClosedAndJoined;OneWorkReturnFence.Store(false);return out;
    }
    bBoundaryServiceSuspended.Store(false);WakeEvent->Trigger();
    const double deadline=FPlatformTime::Seconds()+double(TimeoutMilliseconds)/1000.;
    while(bRunning.Load()&&!bStopRequested.Load()&&request.ParkedNonce.Load()!=request.Nonce)
    {
        if(FPlatformTime::Seconds()>=deadline){out.DeadlineExceeded=true;break;}
        FPlatformProcess::SleepNoStats(0.001f);
    }
    if(!out.DeadlineExceeded&&request.ParkedNonce.Load()==request.Nonce&&IsBoundaryServiceSuspended())
        out.Quiescence=EQuiescence::ParkedForThisNonce;
    else
    {
        // Join may exceed the requested wait deadline. Never return to GT
        // dispatch or release storage while a callback is still in flight.
        if(Identity.Window)Identity.Window->Close();
        StopAndJoin();PendingOneWork.Store(nullptr);
        out.Quiescence=EQuiescence::ClosedAndJoined;
    }
    out.WindowAcceptedBeforeWork=request.WindowAcceptedBeforeWork;
    out.WorkCalled=request.WorkCalled;out.WorkReturned=request.WorkReturned;
    out.WorkResult=uint8(request.Result);out.GuardsAccepted=request.GuardsAccepted;
    if(out.Quiescence==EQuiescence::ClosedAndJoined)OneWorkReturnFence.Store(false);
    return out;
}
#if WITH_DEV_AUTOMATION_TESTS
Speed::OnePausedWork::FResult FSimulationWorker::RunOnePausedWorkForTesting(uint32 timeout)
{
    const auto result=RunOnePausedWork({},[]{},timeout);
    if(result.Nonce&&result.Quiescence==Speed::OnePausedWork::EQuiescence::ParkedForThisNonce)OneWorkReturnFence.Store(false);return result;
}
#endif

FSimulationWorker::~FSimulationWorker()
{
	StopAndJoin();
	if (WakeEvent)
	{
		FPlatformProcess::ReturnSynchEventToPool(WakeEvent);
		WakeEvent = nullptr;
	}
	if (PauseAcknowledgedEvent)
	{
		FPlatformProcess::ReturnSynchEventToPool(PauseAcknowledgedEvent);
		PauseAcknowledgedEvent = nullptr;
	}
	if (BoundarySuspendedEvent)
	{
		FPlatformProcess::ReturnSynchEventToPool(BoundarySuspendedEvent);
		BoundarySuspendedEvent = nullptr;
	}
}

bool FSimulationWorker::Start(bool bStartPaused)
{
	if (Thread || !Work || !WaitBetweenFrames || !WakeEvent ||
		!PauseAcknowledgedEvent || !BoundarySuspendedEvent)
	{
		return false;
	}
	bStopRequested.Store(false);
	bPaused.Store(bStartPaused);
	PauseRequestSerial.Store(bStartPaused ? 1 : 0);
	bBoundaryServiceSuspended.Store(false);
	BoundarySuspendRequestSerial.Store(0);
	BoundarySuspendAckSerial.Store(0);
	PauseAcknowledgedEvent->Reset();
	BoundarySuspendedEvent->Reset();
	bRunning.Store(true);
	Thread = FRunnableThread::Create(
		this, TEXT("IAmSpeedSimulation"), 0, TPri_AboveNormal);
	if (!Thread)
	{
		bRunning.Store(false);
	}
	return Thread != nullptr;
}

void FSimulationWorker::Pause()
{
	(void)TryPause(MAX_uint32);
}

void FSimulationWorker::RequestPause()
{
	if (!bPaused.Load() && PauseAcknowledgedEvent)
	{
		PauseAcknowledgedEvent->Reset();
		++PauseRequestSerial;
	}
	bPaused.Store(true);
	if (WakeEvent)
	{
		WakeEvent->Trigger();
	}

}

bool FSimulationWorker::TryPause(const uint32 TimeoutMilliseconds)
{
    uint64 RequestedSerial=0;
    {
        FScopeLock control(&BoundaryControlMutex);
        if(OneWorkReturnFence.Load())return OneWorkParkedNonce.Load()!=0&&IsBoundaryServiceSuspended();
        RequestPause();RequestedSerial=PauseRequestSerial.Load();
    }
	// Only a true return proves the canonical boundary. A timeout leaves the
	// request pending and never authorizes access to live simulation state.
	const double Deadline = FPlatformTime::Seconds() + double(TimeoutMilliseconds) / 1000.0;
	while (bRunning.Load() && PauseAcknowledgedEvent)
	{
        {
            FScopeLock control(&BoundaryControlMutex);
            if(OneWorkReturnFence.Load())return OneWorkParkedNonce.Load()!=0&&IsBoundaryServiceSuspended();
            if(PauseAckSerial.Load()>=RequestedSerial&&!bStopRequested.Load())return true;
        }
		const double Remaining = Deadline - FPlatformTime::Seconds();
		if (Remaining <= 0) return false;
		PauseAcknowledgedEvent->Wait(uint32(FMath::Max(1,
			FMath::CeilToInt(FMath::Min(Remaining * 1000.0, 100.0)))));
	}
	return false;
}

bool FSimulationWorker::TrySuspendBoundaryService(const uint32 TimeoutMilliseconds)
{
	// A paused physical lane still services commands. This second barrier parks
	// that callback as well. It can acknowledge even when a lifecycle command is
	// waiting for baseline, because the entire callback has returned and no
	// physical work can run before the next loop's suspension check.
    uint64 RequestedSerial=0;
    {
    FScopeLock control(&BoundaryControlMutex);
    if(OneWorkReturnFence.Load()&&!OneWorkParkedNonce.Load())return false;
	if (!bRunning.Load() || bStopRequested.Load()) return false;
	RequestPause();
	if (!bBoundaryServiceSuspended.Load())
	{
		BoundarySuspendedEvent->Reset();
		++BoundarySuspendRequestSerial;
		bBoundaryServiceSuspended.Store(true);
		WakeEvent->Trigger();
	}
    RequestedSerial=BoundarySuspendRequestSerial.Load();
    }
	const double Deadline = FPlatformTime::Seconds() + double(TimeoutMilliseconds) / 1000.0;
	while (bRunning.Load() && !bStopRequested.Load())
	{
        {
            FScopeLock control(&BoundaryControlMutex);
            if(OneWorkReturnFence.Load()&&!OneWorkParkedNonce.Load())return false;
            if(BoundarySuspendAckSerial.Load()>=RequestedSerial&&!bStopRequested.Load())return true;
        }
		const double Remaining = Deadline - FPlatformTime::Seconds();
		if (Remaining <= 0) return false;
		BoundarySuspendedEvent->Wait(uint32(FMath::Max(1,
			FMath::CeilToInt(FMath::Min(Remaining * 1000.0, 100.0)))));
	}
	return false;
}

void FSimulationWorker::ResumeBoundaryService()
{
    FScopeLock control(&BoundaryControlMutex);
    if(OneWorkReturnFence.Load())return;
	bBoundaryServiceSuspended.Store(false);
	if (WakeEvent) WakeEvent->Trigger();
}

void FSimulationWorker::Resume()
{
    FScopeLock control(&BoundaryControlMutex);
    if(OneWorkReturnFence.Load())return;
	// Structural access must be explicitly released before physical work resumes.
	if (bBoundaryServiceSuspended.Load()) return;
	bPaused.Store(false);
	if (WakeEvent)
	{
		WakeEvent->Trigger();
	}
}

void FSimulationWorker::Stop()
{
	bStopRequested.Store(true);
	if (WakeEvent)
	{
		WakeEvent->Trigger();
	}
}

void FSimulationWorker::StopAndJoin()
{
    FScopeLock JoinLock(&JoinMutex);
	Stop();
	if (Thread)
	{
		Thread->WaitForCompletion();
		delete Thread;
		Thread = nullptr;
	}
}

uint32 FSimulationWorker::Run()
{
	FSimulationWorkerWaitContext WaitContext(
		*WakeEvent, bStopRequested, bPaused, bLiveBoundaryRequested);
	uint32 ExitCode = 0;
#if !UE_BUILD_SHIPPING
	bool bNaturalCompletion = false;
#endif
	try
	{
		while (!bStopRequested.Load())
		{
            if (ServiceLiveBoundaryControl())
            {
                // The acknowledged structural park is also a real pause ACK,
                // without changing the user-owned pause request or its serial.
                if (bPaused.Load())
                {
                    PauseAckSerial.Store(PauseRequestSerial.Load());
                    PauseAcknowledgedEvent->Trigger();
                }
                WakeEvent->Wait(1);
                continue;
            }
			if (bBoundaryServiceSuspended.Load())
			{
				const uint64 Request = BoundarySuspendRequestSerial.Load();
				BoundarySuspendAckSerial.Store(Request);
				BoundarySuspendedEvent->Trigger();
				// Exclusive suspension is stronger than an ordinary pause: no
				// callback or physical work can run. A nested pause request may
				// therefore acknowledge the same parked boundary.
				if (bPaused.Load())
				{
					PauseAckSerial.Store(PauseRequestSerial.Load());
					PauseAcknowledgedEvent->Trigger();
				}
#if WITH_DEV_AUTOMATION_TESTS
				Speed::OwnedFrameAudit::ParkWait(*WakeEvent, 1);
#else
				WakeEvent->Wait(1);
#endif
				continue;
			}

            if(auto* one=PendingOneWork.Load())
            {
                // Exactly one claim, before ServiceBoundary can change inputs.
                PendingOneWork.Store(nullptr);
                const auto& id=one->Identity;
                if(id.Host)
                {
                    const auto& w=id.Window;
                    // No Work (including its preparation) runs before this gate.
                    if(!w||!w->IsOpen()||w->GetNonce()!=one->Nonce||id.Worker!=this||
                        !w->Matches(id.Host,id.Bridge,id.Provider,this,id.Generation,id.Frame,id.Delta,id.SimTime)||
                        !w->HasRequiredProviderReadiness(id.Provider)||id.Host->GetInputWorkerGeneration()!=id.Generation||id.Bridge->GetStaticCollisionWorld()!=id.Provider||
                        id.Provider->GetPreparedSnapshot()!=w->GetSnapshot()||
                        id.Host->GetWorld()!=w->GetWorldIdentity()||id.Bridge->GetWorld()!=w->GetWorldIdentity()||w->GetSnapshot()->GetOrigin().World!=w->GetWorldIdentity()||
                        id.Bridge->GetLastStepDiagnostics().Serial!=w->GetStepBefore())
                    {one->GuardRejected=true;if(w)w->Close();ExitCode=1;break;}
                    one->WindowAcceptedBeforeWork=true;
                }
                one->WorkCalled=true;
                if(Speed::OnePausedWork::Current)throw std::runtime_error("Nested single Work request");
                Speed::OnePausedWork::Current=one;
                try
                {
                    Speed::PreparedExecution::FWorkerCallScope owner(*this);
                    one->Result=Work();one->WorkReturned=true;
                }
                catch (...){Speed::OnePausedWork::Current=nullptr;throw;}
                Speed::OnePausedWork::Current=nullptr;
                const bool host=one->Identity.Host!=nullptr;
                const bool preparationResult=one->Result==ESimulationWorkerResult::PreparationPrefixStopped;
                const bool positionResult=one->Result==ESimulationWorkerResult::PostPrepareInitialPositionStopped;
                const bool syncResult=one->Result==ESimulationWorkerResult::TerminalStepSyncStopped;
                const bool prefixResult=preparationResult||positionResult;
                const bool captureResult=one->Result==ESimulationWorkerResult::TerminalSphereRsCaptureStopped;
                const bool consumedAdvanceResult=one->Result==ESimulationWorkerResult::TerminalConsumedPrefixAdvanceStopped;
                const bool contactAdvanceResult=one->Result==ESimulationWorkerResult::TerminalRsContactAdvanceStopped;
                const bool responseResult=one->Result==ESimulationWorkerResult::TerminalRsResponseStopped;
                const bool responseSyncResult=one->Result==ESimulationWorkerResult::TerminalRsResponseSyncStopped;
                const bool postSyncRsCaptureResult=one->Result==ESimulationWorkerResult::TerminalPostSyncRsCaptureStopped;
                const bool prefixAccepted=prefixResult&&host&&!one->GuardRejected&&one->WindowAcceptedBeforeWork&&
                    one->Identity.Window&&one->Identity.Window->IsOpen()&&one->Identity.Window->GetNonce()==one->Nonce&&
                    one->Identity.Window->Matches(id.Host,id.Bridge,id.Provider,this,id.Generation,id.Frame,id.Delta,id.SimTime)&&
                    one->Identity.Window->HasRequiredProviderReadiness(id.Provider)&&
                    id.Provider->GetPreparedSnapshot()==one->Identity.Window->GetSnapshot()&&
                    id.Bridge->GetLastStepDiagnostics().Serial==one->Identity.Window->GetStepBefore()&&
                    Speed::PreparedExecution::FCanonicalFramePreflight::HasUnchangedRawPreparationCensus(
                        *id.Bridge,*one->Identity.Window->GetSnapshot(),one->Identity.Window->GetStepBefore())&&
                    ((preparationResult&&one->Identity.Window->GetPurpose()==Speed::OnePausedWork::EPurpose::PreparationPrefixStop)||
                     (positionResult&&one->Identity.Window->GetPurpose()==Speed::OnePausedWork::EPurpose::PostPrepareInitialPositionStop&&
                      one->Identity.Window->GetPreparationContract()&&one->Identity.Window->GetPreparationContract()->HasAcceptedPosition()))&&
                    one->RunEntered&&one->StepEntered&&one->PreparationStop&&!one->WorldStepEntered&&!one->WorldEffectsEntered&&
                    one->Identity.Window->GetPreparationContract()&&one->Identity.Window->GetPreparationContract()->HasReturnedStop();
                const bool syncAccepted=syncResult&&host&&!one->GuardRejected&&one->WindowAcceptedBeforeWork&&
                    one->Identity.Window&&one->Identity.Window->IsOpen()&&one->Identity.Window->GetNonce()==one->Nonce&&
                    one->Identity.Window->GetPurpose()==Speed::OnePausedWork::EPurpose::TerminalStepSyncStop&&
                    one->Identity.Window->Matches(id.Host,id.Bridge,id.Provider,this,id.Generation,id.Frame,id.Delta,id.SimTime)&&
                    one->Identity.Window->HasRequiredProviderReadiness(id.Provider)&&
                    id.Provider->GetPreparedSnapshot()==one->Identity.Window->GetSnapshot()&&
                    id.Bridge->GetLastStepDiagnostics().Serial==one->Identity.Window->GetExpectedStep()&&
                    Speed::PreparedExecution::FCanonicalFramePreflight::HasUnchangedRawPreparationCensus(
                        *id.Bridge,*one->Identity.Window->GetSnapshot(),one->Identity.Window->GetExpectedStep())&&
                    one->RunEntered&&one->StepEntered&&one->WorldStepEntered&&one->WorldEffectsEntered&&
                    one->TerminalStepSyncEntered&&one->WorldSerialConfirmed&&one->PreparationStop&&
                    one->Identity.Window->GetPreparationContract()&&one->Identity.Window->GetPreparationContract()->HasCompletedStepSync()&&
                    one->Identity.Window->GetPreparationContract()->HasReturnedStop();
                const bool captureAccepted=captureResult&&host&&!one->GuardRejected&&one->WindowAcceptedBeforeWork&&
                    one->Identity.Window&&one->Identity.Window->IsOpen()&&one->Identity.Window->GetNonce()==one->Nonce&&
                    one->Identity.Window->GetPurpose()==Speed::OnePausedWork::EPurpose::TerminalSphereRsCaptureStop&&
                    one->Identity.Window->Matches(id.Host,id.Bridge,id.Provider,this,id.Generation,id.Frame,id.Delta,id.SimTime)&&
                    one->Identity.Window->HasRequiredProviderReadiness(id.Provider)&&
                    id.Provider->GetPreparedSnapshot()==one->Identity.Window->GetSnapshot()&&
                    id.Bridge->GetLastStepDiagnostics().Serial==one->Identity.Window->GetExpectedStep()&&
                    Speed::PreparedExecution::FCanonicalFramePreflight::HasUnchangedRawPreparationCensus(
                        *id.Bridge,*one->Identity.Window->GetSnapshot(),one->Identity.Window->GetExpectedStep())&&
                    one->RunEntered&&one->StepEntered&&one->WorldStepEntered&&one->WorldEffectsEntered&&
                    one->TerminalRsCaptureEntered&&!one->TerminalStepSyncEntered&&one->WorldSerialConfirmed&&one->PreparationStop&&
                    one->Identity.Window->GetPreparationContract()&&one->Identity.Window->GetPreparationContract()->HasClosedNativeRsCapture()&&
                    one->Identity.Window->GetPreparationContract()->HasReturnedStop();
                const bool consumedAdvanceAccepted=consumedAdvanceResult&&host&&!one->GuardRejected&&one->WindowAcceptedBeforeWork&&
                    one->Identity.Window&&one->Identity.Window->IsOpen()&&one->Identity.Window->GetNonce()==one->Nonce&&
                    one->Identity.Window->GetPurpose()==Speed::OnePausedWork::EPurpose::TerminalConsumedPrefixAdvanceStop&&
                    one->Identity.Window->Matches(id.Host,id.Bridge,id.Provider,this,id.Generation,id.Frame,id.Delta,id.SimTime)&&
                    one->Identity.Window->HasRequiredProviderReadiness(id.Provider)&&
                    id.Provider->GetPreparedSnapshot()==one->Identity.Window->GetSnapshot()&&
                    id.Bridge->GetLastStepDiagnostics().Serial==one->Identity.Window->GetExpectedStep()&&
                    Speed::PreparedExecution::FCanonicalFramePreflight::HasUnchangedRawPreparationCensus(
                        *id.Bridge,*one->Identity.Window->GetSnapshot(),one->Identity.Window->GetExpectedStep())&&
                    one->RunEntered&&one->StepEntered&&one->WorldStepEntered&&one->WorldEffectsEntered&&
                    one->TerminalConsumedPrefixAdvanceEntered&&!one->TerminalStepSyncEntered&&!one->TerminalRsCaptureEntered&&one->WorldSerialConfirmed&&one->PreparationStop&&
                    one->Identity.Window->GetPreparationContract()&&one->Identity.Window->GetPreparationContract()->HasCompletedNativeRsAdvance()&&
                    one->Identity.Window->GetPreparationContract()->HasReturnedStop();
                const bool contactAdvanceAccepted=contactAdvanceResult&&host&&!one->GuardRejected&&one->WindowAcceptedBeforeWork&&
                    one->Identity.Window&&one->Identity.Window->IsOpen()&&one->Identity.Window->GetNonce()==one->Nonce&&
                    one->Identity.Window->GetPurpose()==Speed::OnePausedWork::EPurpose::TerminalRsContactAdvanceStop&&
                    one->Identity.Window->Matches(id.Host,id.Bridge,id.Provider,this,id.Generation,id.Frame,id.Delta,id.SimTime)&&
                    one->Identity.Window->HasRequiredProviderReadiness(id.Provider)&&
                    id.Provider->GetPreparedSnapshot()==one->Identity.Window->GetSnapshot()&&
                    id.Bridge->GetLastStepDiagnostics().Serial==one->Identity.Window->GetExpectedStep()&&
                    Speed::PreparedExecution::FCanonicalFramePreflight::HasUnchangedRawPreparationCensus(
                        *id.Bridge,*one->Identity.Window->GetSnapshot(),one->Identity.Window->GetExpectedStep())&&
                    one->RunEntered&&one->StepEntered&&one->WorldStepEntered&&one->WorldEffectsEntered&&
                    one->TerminalRsContactAdvanceEntered&&!one->TerminalConsumedPrefixAdvanceEntered&&!one->TerminalStepSyncEntered&&!one->TerminalRsCaptureEntered&&one->WorldSerialConfirmed&&one->PreparationStop&&
                    one->Identity.Window->GetPreparationContract()&&one->Identity.Window->GetPreparationContract()->HasCompletedNativeRsContactAdvance()&&
                    one->Identity.Window->GetPreparationContract()->HasReturnedStop();
                const bool responseAccepted=responseResult&&host&&!one->GuardRejected&&one->WindowAcceptedBeforeWork&&
                    one->Identity.Window&&one->Identity.Window->IsOpen()&&one->Identity.Window->GetNonce()==one->Nonce&&
                    one->Identity.Window->GetPurpose()==Speed::OnePausedWork::EPurpose::TerminalRsResponseStop&&
                    one->Identity.Window->Matches(id.Host,id.Bridge,id.Provider,this,id.Generation,id.Frame,id.Delta,id.SimTime)&&
                    one->Identity.Window->HasRequiredProviderReadiness(id.Provider)&&
                    id.Provider->GetPreparedSnapshot()==one->Identity.Window->GetSnapshot()&&
                    id.Bridge->GetLastStepDiagnostics().Serial==one->Identity.Window->GetExpectedStep()&&
                    Speed::PreparedExecution::FCanonicalFramePreflight::HasUnchangedRawPreparationCensus(
                        *id.Bridge,*one->Identity.Window->GetSnapshot(),one->Identity.Window->GetExpectedStep())&&
                    one->RunEntered&&one->StepEntered&&one->WorldStepEntered&&one->WorldEffectsEntered&&
                    one->TerminalRsResponseEntered&&!one->TerminalRsContactAdvanceEntered&&!one->TerminalConsumedPrefixAdvanceEntered&&!one->TerminalStepSyncEntered&&!one->TerminalRsCaptureEntered&&one->WorldSerialConfirmed&&one->PreparationStop&&
                    one->Identity.Window->GetPreparationContract()&&one->Identity.Window->GetPreparationContract()->HasCompletedNativeRsResponse()&&
                    one->Identity.Window->GetPreparationContract()->HasReturnedStop();
                const bool responseSyncAccepted=responseSyncResult&&host&&!one->GuardRejected&&one->WindowAcceptedBeforeWork&&
                    one->Identity.Window&&one->Identity.Window->IsOpen()&&one->Identity.Window->GetNonce()==one->Nonce&&
                    one->Identity.Window->GetPurpose()==Speed::OnePausedWork::EPurpose::TerminalRsResponseSyncStop&&
                    one->Identity.Window->Matches(id.Host,id.Bridge,id.Provider,this,id.Generation,id.Frame,id.Delta,id.SimTime)&&
                    one->Identity.Window->HasRequiredProviderReadiness(id.Provider)&&
                    id.Provider->GetPreparedSnapshot()==one->Identity.Window->GetSnapshot()&&
                    id.Bridge->GetLastStepDiagnostics().Serial==one->Identity.Window->GetExpectedStep()&&
                    Speed::PreparedExecution::FCanonicalFramePreflight::HasUnchangedRawPreparationCensus(
                        *id.Bridge,*one->Identity.Window->GetSnapshot(),one->Identity.Window->GetExpectedStep())&&
                    one->RunEntered&&one->StepEntered&&one->WorldStepEntered&&one->WorldEffectsEntered&&
                    one->TerminalRsResponseSyncEntered&&!one->TerminalRsResponseEntered&&!one->TerminalRsContactAdvanceEntered&&!one->TerminalConsumedPrefixAdvanceEntered&&!one->TerminalStepSyncEntered&&!one->TerminalRsCaptureEntered&&one->WorldSerialConfirmed&&one->PreparationStop&&
                    one->Identity.Window->GetPreparationContract()&&one->Identity.Window->GetPreparationContract()->HasCompletedNativeRsResponseSync()&&
                    one->Identity.Window->GetPreparationContract()->HasReturnedStop();
                const bool postSyncRsCaptureAccepted=postSyncRsCaptureResult&&host&&!one->GuardRejected&&one->WindowAcceptedBeforeWork&&
                    one->Identity.Window&&one->Identity.Window->IsOpen()&&one->Identity.Window->GetNonce()==one->Nonce&&
                    one->Identity.Window->GetPurpose()==Speed::OnePausedWork::EPurpose::TerminalPostSyncRsCaptureStop&&
                    one->Identity.Window->Matches(id.Host,id.Bridge,id.Provider,this,id.Generation,id.Frame,id.Delta,id.SimTime)&&
                    one->Identity.Window->HasRequiredProviderReadiness(id.Provider)&&
                    id.Provider->GetPreparedSnapshot()==one->Identity.Window->GetSnapshot()&&
                    id.Bridge->GetLastStepDiagnostics().Serial==one->Identity.Window->GetExpectedStep()&&
                    Speed::PreparedExecution::FCanonicalFramePreflight::HasUnchangedRawPreparationCensus(
                        *id.Bridge,*one->Identity.Window->GetSnapshot(),one->Identity.Window->GetExpectedStep())&&
                    one->RunEntered&&one->StepEntered&&one->WorldStepEntered&&one->WorldEffectsEntered&&
                    one->TerminalPostSyncRsCaptureEntered&&!one->TerminalRsResponseSyncEntered&&!one->TerminalRsResponseEntered&&!one->TerminalRsContactAdvanceEntered&&!one->TerminalConsumedPrefixAdvanceEntered&&!one->TerminalStepSyncEntered&&!one->TerminalRsCaptureEntered&&one->WorldSerialConfirmed&&one->PreparationStop&&
                    one->Identity.Window->GetPreparationContract()&&one->Identity.Window->GetPreparationContract()->HasCompletedNativePostSyncRsCapture()&&
                    one->Identity.Window->GetPreparationContract()->HasReturnedStop();
                // Current TLS is already cleared after Work returned. Rejoin
                // the retained request directly; do not reinstall a live scope
                // or mistake entered effects for a confirmed World step.
                const auto& fullWindow=one->Identity.Window;
                const bool noTerminalState=!one->PreparationStop&&!one->TerminalStepSyncEntered&&
                    !one->TerminalConsumedPrefixAdvanceEntered&&!one->TerminalRsContactAdvanceEntered&&
                    !one->TerminalRsResponseEntered&&!one->TerminalRsResponseSyncEntered&&
                    !one->TerminalPostSyncRsCaptureEntered&&!one->TerminalRsCaptureEntered;
                const bool fullWindowCurrent=host&&!one->GuardRejected&&one->WorkCalled&&one->WorkReturned&&
                    one->WindowAcceptedBeforeWork&&noTerminalState&&fullWindow&&fullWindow->IsOpen()&&
                    fullWindow->GetPurpose()==Speed::OnePausedWork::EPurpose::FullFrame&&fullWindow->GetNonce()==one->Nonce&&
                    fullWindow->Matches(id.Host,id.Bridge,id.Provider,this,id.Generation,id.Frame,id.Delta,id.SimTime)&&
                    fullWindow->HasRequiredProviderReadiness(id.Provider)&&id.Host->GetInputWorkerGeneration()==id.Generation&&
                    id.Bridge->GetStaticCollisionWorld()==id.Provider&&id.Provider->GetPreparedSnapshot()==fullWindow->GetSnapshot()&&
                    id.Host->GetWorld()==fullWindow->GetWorldIdentity()&&id.Bridge->GetWorld()==fullWindow->GetWorldIdentity()&&
                    fullWindow->GetSnapshot()->GetOrigin().World==fullWindow->GetWorldIdentity();
                const bool fullFrameAdvanced=fullWindowCurrent&&one->Result==ESimulationWorkerResult::Advanced&&
                    one->RunEntered&&one->StepEntered&&one->WorldStepEntered&&one->WorldEffectsEntered&&one->WorldSerialConfirmed&&
                    id.Bridge->GetLastStepDiagnostics().Serial==fullWindow->GetExpectedStep();
                const bool fullFrameIdle=fullWindowCurrent&&one->Result==ESimulationWorkerResult::Idle&&
                    !one->RunEntered&&!one->StepEntered&&!one->WorldStepEntered&&!one->WorldEffectsEntered&&!one->WorldSerialConfirmed&&
                    id.Bridge->GetLastStepDiagnostics().Serial==fullWindow->GetStepBefore();
                // This closes only the hosted worker-return/World-serial join.
                // It does not supply missing owner Prepare/Post/Snapshot or
                // physical contracts. Hostless scheduling tests stay separate.
                one->GuardsAccepted=postSyncRsCaptureResult?postSyncRsCaptureAccepted:responseSyncResult?responseSyncAccepted:responseResult?responseAccepted:contactAdvanceResult?contactAdvanceAccepted:consumedAdvanceResult?consumedAdvanceAccepted:captureResult?captureAccepted:syncResult?syncAccepted:prefixResult?prefixAccepted:!one->GuardRejected&&
                    (!host||fullFrameAdvanced||fullFrameIdle);
                // This terminal result closes and exits BEFORE the ordinary new
                // park ACK. CloseOnWorker runs after Current and worker scope end.
                if(prefixAccepted||syncAccepted||captureAccepted||consumedAdvanceAccepted||contactAdvanceAccepted||responseAccepted||responseSyncAccepted||postSyncRsCaptureAccepted){one->Identity.Window->Close();ExitCode=0;break;}
                if(!one->GuardsAccepted||one->Result==ESimulationWorkerResult::Complete||one->Result==ESimulationWorkerResult::Failed)
                {if(one->Identity.Window)one->Identity.Window->Close();ExitCode=one->Result==ESimulationWorkerResult::Complete&&one->GuardsAccepted?0:1;break;}
                bPaused.Store(true);BoundarySuspendedEvent->Reset();
                // The new ACK serial is created only AFTER Work and its native
                // worker scope returned. No old pause/event is sufficient.
                uint64 serial=0;
                {
                    FScopeLock control(&BoundaryControlMutex);
                    if(!one->ReservedParkSerial||BoundarySuspendRequestSerial.Load()!=one->ReservedParkSerial-1)
                    {one->GuardRejected=true;one->GuardsAccepted=false;if(one->Identity.Window)one->Identity.Window->Close();ExitCode=1;break;}
                    serial=one->ReservedParkSerial;BoundarySuspendRequestSerial.Store(serial);
                }
                bBoundaryServiceSuspended.Store(true);BoundarySuspendAckSerial.Store(serial);
                PauseAckSerial.Store(PauseRequestSerial.Load());
                BoundarySuspendedEvent->Trigger();PauseAcknowledgedEvent->Trigger();
                OneWorkParkedNonce.Store(one->Nonce);
                // Last access to caller-owned request storage.
                one->ParkedNonce.Store(one->Nonce);
                continue;
            }

			// Capture the request before servicing it. A newer pause cannot borrow
			// an acknowledgment from work performed for an earlier boundary.
			const uint64 Request = PauseRequestSerial.Load();
			const bool bPauseAtBoundary = bPaused.Load();
			const ESimulationBoundaryResult Boundary = ServiceBoundary
				? ServiceBoundary(bPauseAtBoundary) : ESimulationBoundaryResult::Ready;
			if (Boundary == ESimulationBoundaryResult::Failed)
			{
				ExitCode = 1;
				break;
			}
			if (bStopRequested.Load()) break;
			if (Boundary == ESimulationBoundaryResult::Waiting)
			{
#if WITH_DEV_AUTOMATION_TESTS
				Speed::OwnedFrameAudit::ParkWait(*WakeEvent, 1);
#else
				WakeEvent->Wait(1);
#endif
				continue;
			}
			if (bPaused.Load())
			{
				if (bPauseAtBoundary && PauseRequestSerial.Load() == Request)
				{
					PauseAckSerial.Store(Request);
					PauseAcknowledgedEvent->Trigger();
				}
				// Continue servicing commands while paused; no producer polling or
				// physical frame runs from this branch.
#if WITH_DEV_AUTOMATION_TESTS
				Speed::OwnedFrameAudit::ParkWait(*WakeEvent, 1);
#else
				WakeEvent->Wait(1);
#endif
				continue;
			}
			PauseAcknowledgedEvent->Reset();
			const ESimulationWorkerResult Result = [&]()
            {
                Speed::PreparedExecution::FWorkerCallScope PhysicalWorker(*this);
                return Work();
            }();
			if (Result == ESimulationWorkerResult::Complete || Result == ESimulationWorkerResult::Failed ||
                Result == ESimulationWorkerResult::PreparationPrefixStopped || Result == ESimulationWorkerResult::PostPrepareInitialPositionStopped || Result == ESimulationWorkerResult::TerminalStepSyncStopped || Result == ESimulationWorkerResult::TerminalSphereRsCaptureStopped || Result == ESimulationWorkerResult::TerminalConsumedPrefixAdvanceStopped || Result == ESimulationWorkerResult::TerminalRsContactAdvanceStopped || Result == ESimulationWorkerResult::TerminalRsResponseStopped || Result == ESimulationWorkerResult::TerminalRsResponseSyncStopped || Result == ESimulationWorkerResult::TerminalPostSyncRsCaptureStopped)
			{
                // Prefix is accepted only by the nonce-bound one-Work branch.
                ExitCode = Result == ESimulationWorkerResult::Complete ? 0 : 1;
#if !UE_BUILD_SHIPPING
                bNaturalCompletion = Result == ESimulationWorkerResult::Complete;
#endif
				break;
			}
			if (Result == ESimulationWorkerResult::Idle)
			{
#if WITH_DEV_AUTOMATION_TESTS
				Speed::OwnedFrameAudit::ParkWait(*WakeEvent, 1);
#else
				WakeEvent->Wait(1);
#endif
				continue;
			}
			WaitBetweenFrames(WaitContext);
		}
	}
	catch (...) { ExitCode = 1; }
	// The callback retires and destroys thread-affine session owners before
	// join can let the GT release their storage, on every exit path.
	try { if (CloseOnWorker) CloseOnWorker(); }
	catch (...) { ExitCode = 1; }
    {
        std::lock_guard<std::mutex> Lock(LiveBoundaryMutex);
        if (LiveBoundaryControl)
        {
            LiveBoundaryControl->Phase.store(Speed::SimulationBoundary::Detail::ELiveBoundaryPhase::Retired);
            LiveBoundaryControl.reset();
        }
        bLiveBoundaryRequested.Store(false);
    }
#if !UE_BUILD_SHIPPING
    // CloseOnWorker has returned. A plain StopAndJoin also has ExitCode 0,
    // but it never acquires the natural completion witness.
    bClosedAfterNaturalCompletion.Store(bNaturalCompletion && ExitCode == 0);
#endif
	bStopRequested.Store(true);
	bRunning.Store(false);
	PauseAcknowledgedEvent->Trigger();
	BoundarySuspendedEvent->Trigger();
	return ExitCode;
}

bool FSimulationWorker::RequestLiveBoundaryControl(
    const std::shared_ptr<Speed::SimulationBoundary::Detail::FLiveBoundaryControl>& Control)
{
    std::lock_guard<std::mutex> Lock(LiveBoundaryMutex);
    if (!Control || LiveBoundaryControl || bBoundaryServiceSuspended.Load()
        || !bRunning.Load() || bStopRequested.Load() || LiveBoundaryRequestSerial == MAX_uint64) return false;
    Control->RequestSerial = ++LiveBoundaryRequestSerial;
    LiveBoundaryControl = Control;
    bLiveBoundaryRequested.Store(true);
    WakeEvent->Trigger();
    return true;
}

bool FSimulationWorker::OwnsAcknowledgedLiveBoundary(
    const std::shared_ptr<Speed::SimulationBoundary::Detail::FLiveBoundaryControl>& Control) const
{
    using namespace Speed::SimulationBoundary::Detail;
    std::lock_guard<std::mutex> Lock(LiveBoundaryMutex);
    if (!Control || LiveBoundaryControl != Control || !Control->RequestSerial
        || !bRunning.Load() || bStopRequested.Load()) return false;
    const auto Phase = Control->Phase.load();
    return Phase == ELiveBoundaryPhase::Acknowledged || Phase == ELiveBoundaryPhase::Executing;
}

void FSimulationWorker::WakeLiveBoundaryControl()
{
    if (WakeEvent) WakeEvent->Trigger();
}

bool FSimulationWorker::ServiceLiveBoundaryControl()
{
    using namespace Speed::SimulationBoundary::Detail;
    bool Parked = false, ResetPacing = false;
    std::shared_ptr<FLiveBoundaryControl> Releasing;
    {
        std::lock_guard<std::mutex> Lock(LiveBoundaryMutex);
        if (!LiveBoundaryControl) return false;
        const auto Control = LiveBoundaryControl;
        auto Phase = Control->Phase.load();
        if (Control->CancelRequested.load() && Phase != ELiveBoundaryPhase::Executing)
        {
            if (Phase == ELiveBoundaryPhase::Requested || Phase == ELiveBoundaryPhase::Acknowledged)
                Control->Phase.compare_exchange_strong(Phase, ELiveBoundaryPhase::ReleaseRequested);
            Phase = Control->Phase.load();
        }
        if (Phase == ELiveBoundaryPhase::ReleaseRequested)
        {
            ResetPacing = Control->WasAcknowledged.load();
            Releasing = Control;
        }
        else
        {
            if (Phase == ELiveBoundaryPhase::Requested)
            {
                if (Control->Phase.compare_exchange_strong(Phase, ELiveBoundaryPhase::Acknowledged))
                    Control->WasAcknowledged.store(true);
            }
            Phase = Control->Phase.load();
            Parked = Phase == ELiveBoundaryPhase::Acknowledged || Phase == ELiveBoundaryPhase::Executing;
        }
    }
    // Only an actually acknowledged structural interruption resets wall-clock
    // pacing. The hook changes no user pause flag or canonical frame.
    if (Releasing)
    {
        try { if (ResetPacing && StructuralBoundaryReleased) StructuralBoundaryReleased(); }
        catch (...) { Releasing->Phase.store(ELiveBoundaryPhase::Retired); throw; }
        std::lock_guard<std::mutex> Lock(LiveBoundaryMutex);
        if (LiveBoundaryControl == Releasing) LiveBoundaryControl.reset();
        bLiveBoundaryRequested.Store(false);
        Releasing->Phase.store(bStopRequested.Load() ? ELiveBoundaryPhase::Retired : ELiveBoundaryPhase::Disposed);
    }
    return Parked;
}
