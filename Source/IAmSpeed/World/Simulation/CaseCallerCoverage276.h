#pragma once
#include "CoreMinimal.h"
#include "IAmSpeed/World/Analytic/AnalyticCaseSatDiagnostics.h"

// Draft276: independent Fast caller ledger. No Owner231/TLS reads or physics changes.
#if !UE_BUILD_SHIPPING
namespace Speed::Diagnostics::CaseCoverage276
{
class FDriverLedger final
{
public:
    FDriverLedger()=default;
    FDriverLedger(const FDriverLedger&)=delete;
    FDriverLedger& operator=(const FDriverLedger&)=delete;
    FDriverLedger(FDriverLedger&&)=delete;
    FDriverLedger& operator=(FDriverLedger&&)=delete;
    void BeginFast(const Analytic::CaseSatDiagnostics::FCaseIdentity& Identity,
        uint64 CanonicalBefore, uint64 PerformanceBefore, bool bOwnedLane) noexcept;
    // Distinct RealTime696 protocol; repeating Begin on an open case never resets counters.
    void BeginRealTime(const Analytic::CaseSatDiagnostics::FCaseIdentity& Identity,
        uint64 CanonicalBefore, uint64 PerformanceBefore, bool bOwnedLane) noexcept;
    void EndRealTime(uint64 CanonicalAfter, uint64 PerformanceAfter,
        bool bCompleted, const TCHAR* Reason) noexcept;
    void EndFast(uint64 CanonicalAfter, uint64 PerformanceAfter,
        bool bCompleted, const TCHAR* Reason) noexcept;
    bool EnterPreloop() noexcept;
    void LeavePreloop(bool bReturnedTrue) noexcept;
    void RollbackDequeued() noexcept;
    bool EnterForward(uint64 Frame) noexcept;
    void LeaveForward(bool bCommitted) noexcept;
private:
    void Begin(const Analytic::CaseSatDiagnostics::FCaseIdentity& Identity,
        uint64 CanonicalBefore, uint64 PerformanceBefore, bool bOwnedLane, bool bInRealTime) noexcept;
    bool IsOwner() const noexcept;
    void CheckConfiguration() noexcept;
    bool Increment(uint64& Value) noexcept;
    Analytic::CaseSatDiagnostics::FCaseIdentity Key, LastClosedKey;
    bool bOpen=false, bHaveClosed=false, bPermanentInvalid=false, bInvalid=false;
    bool bPreloopOpen=false, bForwardOpen=false;
    bool bRealTime=false; // Log family/provenance only; never changes the physical step.
    uint32 Thread=0;
    int32 Lazy=0;
    uint64 NextRun=0, Run=0, CanonicalStart=0, PerformanceStart=0;
    uint64 PreloopCalls=0, PreloopTrue=0, PreloopFailed=0, RollbackRequests=0;
    uint64 Attempts=0, Committed=0, Partial=0;
    uint64 First=MAX_uint64, Last=MAX_uint64, Gaps=0;
};
class FScopedPreloop final
{
public:
    explicit FScopedPreloop(FDriverLedger& In) noexcept : Ledger(In), bTracked(In.EnterPreloop()) {}
    ~FScopedPreloop() noexcept { Finish(false); }
    void Finish(bool bReturnedTrue) noexcept { if(bTracked && !bFinished) { bFinished=true; Ledger.LeavePreloop(bReturnedTrue); } }
    FScopedPreloop(const FScopedPreloop&)=delete;
    FScopedPreloop& operator=(const FScopedPreloop&)=delete;
private:
    FDriverLedger& Ledger; bool bTracked=false, bFinished=false;
};
class FScopedForward final
{
public:
    explicit FScopedForward(FDriverLedger& In, uint64 Frame) noexcept : Ledger(In), bTracked(In.EnterForward(Frame)) {}
    ~FScopedForward() noexcept { Finish(false); }
    void Finish(bool bCommitted) noexcept { if(bTracked && !bFinished) { bFinished=true; Ledger.LeaveForward(bCommitted); } }
    FScopedForward(const FScopedForward&)=delete;
    FScopedForward& operator=(const FScopedForward&)=delete;
private:
    FDriverLedger& Ledger; bool bTracked=false, bFinished=false;
};
}
#endif
