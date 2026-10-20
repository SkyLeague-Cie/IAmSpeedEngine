#include "CaseCallerCoverage276.h"
#if !UE_BUILD_SHIPPING
#include "HAL/PlatformTLS.h"
#include "HAL/IConsoleManager.h"
namespace Speed::Diagnostics::CaseCoverage276
{
namespace
{
bool SameCase(const Analytic::CaseSatDiagnostics::FCaseIdentity& A,
    const Analytic::CaseSatDiagnostics::FCaseIdentity& B)
{
    return A.RunUUID==B.RunUUID && A.DriverKey==B.DriverKey && A.Epoch==B.Epoch &&
        A.NativeCaseId==B.NativeCaseId && A.BatchIndex==B.BatchIndex &&
        A.WorldGeneration==B.WorldGeneration;
}
int32 Read(const TCHAR* Name)
{
    const IConsoleVariable* V=IConsoleManager::Get().FindConsoleVariable(Name);
    return V ? V->GetInt() : INDEX_NONE;
}
}
bool FDriverLedger::IsOwner() const noexcept
{
    return bOpen && Thread==FPlatformTLS::GetCurrentThreadId();
}
bool FDriverLedger::Increment(uint64& Value) noexcept
{
    if(Value==MAX_uint64) { bInvalid=true; bPermanentInvalid=true; return false; }
    ++Value; return true;
}
void FDriverLedger::CheckConfiguration() noexcept
{
    try
    {
        if(Read(TEXT("p.IAmSpeed.AnalyticWorld.CaseSatDiagnostics"))!=Key.Mode ||
            Read(TEXT("p.IAmSpeed.AnalyticWorld.PhaseTiming"))!=(Key.Mode==1?0:2) ||
            Read(TEXT("p.IAmSpeed.AnalyticWorld.LazyExtrudedBoxSatAxes"))!=Lazy ||
            (bRealTime && (Read(TEXT("p.IAmSpeed.FastSimulation.Enabled"))!=0 ||
                Read(TEXT("p.IAmSpeed.Simulation.ExecutionMode"))!=2)))
            bInvalid=true;
    }
    catch(...) { bInvalid=true; bPermanentInvalid=true; }
}
void FDriverLedger::BeginFast(const Analytic::CaseSatDiagnostics::FCaseIdentity& Identity,
    uint64 Before, uint64 PerfBefore, bool bOwnedLane) noexcept
{
    Begin(Identity,Before,PerfBefore,bOwnedLane,false);
}
void FDriverLedger::BeginRealTime(const Analytic::CaseSatDiagnostics::FCaseIdentity& Identity,
    uint64 Before, uint64 PerfBefore, bool bOwnedLane) noexcept
{
    if(bOpen) return; // The existing owner and case remain authoritative until an actual End.
    Begin(Identity,Before,PerfBefore,bOwnedLane,true);
}
void FDriverLedger::Begin(const Analytic::CaseSatDiagnostics::FCaseIdentity& Identity,
    uint64 Before, uint64 PerfBefore, bool bOwnedLane, bool bInRealTime) noexcept
{
    if(!Identity.bEnabled) return; // Default0: no log, counter reset or configuration lookup.
    try
    {
        if(bOpen || bPermanentInvalid || !bOwnedLane || (Identity.Mode!=1 && Identity.Mode!=2) ||
            Identity.RunUUID.IsEmpty() || Identity.DriverKey==0 || Identity.Epoch==0 ||
            Identity.NativeCaseId<0 || Identity.BatchIndex<0 || Identity.WorldGeneration==0 ||
            (bHaveClosed && (SameCase(Identity,LastClosedKey) ||
                Identity.RunUUID!=LastClosedKey.RunUUID || Identity.DriverKey!=LastClosedKey.DriverKey ||
                Identity.Epoch<=LastClosedKey.Epoch)) || NextRun==MAX_uint64)
        { bPermanentInvalid=true; return; } // Missing Begin/End stays UNKNOWN; never clear old incomplete work.
        bRealTime=bInRealTime;
        Key=Identity; Thread=FPlatformTLS::GetCurrentThreadId();
        if(Thread==0) { bPermanentInvalid=true; return; }
        Run=++NextRun; CanonicalStart=Before; PerformanceStart=PerfBefore;
        PreloopCalls=PreloopTrue=PreloopFailed=RollbackRequests=0;
        Attempts=Committed=Partial=Gaps=0; First=Last=MAX_uint64;
        bPreloopOpen=bForwardOpen=false; bInvalid=false; bOpen=true;
        Lazy=Read(TEXT("p.IAmSpeed.AnalyticWorld.LazyExtrudedBoxSatAxes"));
        if(Lazy!=0 && Lazy!=1) bInvalid=true;
        CheckConfiguration();
        if(bRealTime)
        {
        UE_LOG(LogTemp,Display,TEXT("[CaseCallerBegin696] Run=%s Driver=%llu Epoch=%llu Case=%d Batch=%d WorldGeneration=%llu Thread=%u CoverageRun=%llu ExecutionMode=RealTime DiagnosticMode=%d Lazy=%d CanonicalBefore=%llu PerformanceBefore=%llu Invalid=%d Scope=ForwardOwnedStepCallerOnly"),
            *Key.RunUUID,Key.DriverKey,Key.Epoch,Key.NativeCaseId,Key.BatchIndex,Key.WorldGeneration,Thread,Run,Key.Mode,Lazy,CanonicalStart,PerformanceStart,bInvalid?1:0);
        }
        else
        {
        UE_LOG(LogTemp,Display,TEXT("[CaseCallerBegin276] Run=%s Driver=%llu Epoch=%llu Case=%d Batch=%d WorldGeneration=%llu Thread=%u CoverageRun=%llu ExecutionMode=Fast DiagnosticMode=%d Lazy=%d CanonicalBefore=%llu PerformanceBefore=%llu Invalid=%d Scope=ForwardOwnedStepCallerOnly"),
            *Key.RunUUID,Key.DriverKey,Key.Epoch,Key.NativeCaseId,Key.BatchIndex,Key.WorldGeneration,Thread,Run,Key.Mode,Lazy,CanonicalStart,PerformanceStart,bInvalid?1:0);
        }
    }
    catch(...) { bInvalid=true; bPermanentInvalid=true; } // Diagnostic failure never changes physics.
}
bool FDriverLedger::EnterPreloop() noexcept
{
    if(!IsOwner()) return false;
    CheckConfiguration();
    if(bPreloopOpen || bForwardOpen) { bInvalid=true; return false; }
    bPreloopOpen=true; Increment(PreloopCalls); return true;
}
void FDriverLedger::LeavePreloop(bool bReturnedTrue) noexcept
{
    if(!IsOwner()) return;
    if(!bPreloopOpen) { bInvalid=true; return; }
    bPreloopOpen=false;
    if(bReturnedTrue) Increment(PreloopTrue);
    else { Increment(PreloopFailed); bInvalid=true; }
    CheckConfiguration();
}
void FDriverLedger::RollbackDequeued() noexcept
{
    if(!IsOwner()) return;
    Increment(RollbackRequests); bInvalid=true; // Actual dequeue observed, even if restore/replay later rejects.
    if(!bPreloopOpen) bPermanentInvalid=true;
}
bool FDriverLedger::EnterForward(uint64 Frame) noexcept
{
    if(!IsOwner()) return false;
    CheckConfiguration();
    if(bPreloopOpen || bForwardOpen) { bInvalid=true; return false; }
    bForwardOpen=true; Increment(Attempts);
    if(Frame==MAX_uint64) bInvalid=true;
    if(First==MAX_uint64) First=Frame;
    else if(Last==MAX_uint64 || Frame!=Last+1) { Increment(Gaps); bInvalid=true; }
    Last=Frame; return true;
}
void FDriverLedger::LeaveForward(bool bCommitted) noexcept
{
    if(!IsOwner()) return;
    if(!bForwardOpen) { bInvalid=true; return; }
    bForwardOpen=false;
    if(bCommitted) Increment(Committed);
    else { Increment(Partial); bInvalid=true; }
    CheckConfiguration();
}
void FDriverLedger::EndRealTime(uint64 After, uint64 PerfAfter, bool bCompleted, const TCHAR* Reason) noexcept
{
    if(!bRealTime) return;
    EndFast(After,PerfAfter,bCompleted,Reason);
}
void FDriverLedger::EndFast(uint64 After, uint64 PerfAfter, bool bCompleted, const TCHAR* Reason) noexcept
{
    if(!IsOwner()) return; // Wrong lane cannot close/mutate another owner's ledger: missing End => UNKNOWN.
    bOpen=false; // Idempotent before any allocation/logging.
    try
    {
        CheckConfiguration();
        const bool CountRange=Attempts>0 && First!=MAX_uint64 && Last!=MAX_uint64 && Last>=First &&
            Last-First==Attempts-1;
        const bool Candidate=bCompleted && !bInvalid && !bPermanentInvalid && !bPreloopOpen && !bForwardOpen &&
            PreloopCalls==PreloopTrue && PreloopFailed==0 && RollbackRequests==0 &&
            PreloopCalls==Attempts && Attempts==Committed && Partial==0 && Gaps==0 && CountRange &&
            First==CanonicalStart && After>Last && After-Last==1 && After>=CanonicalStart &&
            After-CanonicalStart==Committed && PerfAfter>=PerformanceStart && PerfAfter-PerformanceStart==Committed;
        LastClosedKey=Key; bHaveClosed=true;
        if(bRealTime)
        {
        UE_LOG(LogTemp,Display,TEXT("[CaseCallerEnd696] Run=%s Driver=%llu Epoch=%llu Case=%d Batch=%d WorldGeneration=%llu Thread=%u CoverageRun=%llu ExecutionMode=RealTime DiagnosticMode=%d Lazy=%d CanonicalBefore=%llu CanonicalAfter=%llu PerformanceBefore=%llu PerformanceAfter=%llu PreloopCalls=%llu PreloopTrue=%llu PreloopFailed=%llu RollbackDequeued=%llu ActualForwardAttempts=%llu ActualCommitted=%llu ActualPartial=%llu FirstForwardFrame=%llu LastForwardFrame=%llu NonContiguous=%llu Invalid=%d PermanentInvalid=%d EndReason=%s CallerCompleted=%d ForwardScopeCandidate=%d RuntimeCoverageReconciled=0 WholeCaseCPUQualified=0 SeedCaptureInclusion=UNKNOWN RealTimeCoverage=UNRECONCILED"),
            *Key.RunUUID,Key.DriverKey,Key.Epoch,Key.NativeCaseId,Key.BatchIndex,Key.WorldGeneration,Thread,Run,Key.Mode,Lazy,
            CanonicalStart,After,PerformanceStart,PerfAfter,PreloopCalls,PreloopTrue,PreloopFailed,RollbackRequests,
            Attempts,Committed,Partial,First,Last,Gaps,bInvalid?1:0,bPermanentInvalid?1:0,Reason,bCompleted?1:0,Candidate?1:0);
        }
        else
        {
        UE_LOG(LogTemp,Display,TEXT("[CaseCallerEnd276] Run=%s Driver=%llu Epoch=%llu Case=%d Batch=%d WorldGeneration=%llu Thread=%u CoverageRun=%llu ExecutionMode=Fast DiagnosticMode=%d Lazy=%d CanonicalBefore=%llu CanonicalAfter=%llu PerformanceBefore=%llu PerformanceAfter=%llu PreloopCalls=%llu PreloopTrue=%llu PreloopFailed=%llu RollbackDequeued=%llu ActualForwardAttempts=%llu ActualCommitted=%llu ActualPartial=%llu FirstForwardFrame=%llu LastForwardFrame=%llu NonContiguous=%llu Invalid=%d PermanentInvalid=%d EndReason=%s CallerCompleted=%d ForwardScopeCandidate=%d RuntimeCoverageReconciled=0 WholeCaseCPUQualified=0 SeedCaptureInclusion=UNKNOWN RealTimeCoverage=UNKNOWN"),
            *Key.RunUUID,Key.DriverKey,Key.Epoch,Key.NativeCaseId,Key.BatchIndex,Key.WorldGeneration,Thread,Run,Key.Mode,Lazy,
            CanonicalStart,After,PerformanceStart,PerfAfter,PreloopCalls,PreloopTrue,PreloopFailed,RollbackRequests,
            Attempts,Committed,Partial,First,Last,Gaps,bInvalid?1:0,bPermanentInvalid?1:0,Reason,bCompleted?1:0,Candidate?1:0);
        }
    }
    catch(...) { bInvalid=true; bPermanentInvalid=true; } // Missing terminal marker => UNKNOWN.
}
}
#endif
