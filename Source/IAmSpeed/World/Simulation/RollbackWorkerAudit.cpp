#include "RollbackWorkerAudit.h"

#include "CoreGlobals.h"
#include "HAL/PlatformTLS.h"
#include "HAL/PlatformTime.h"
#include "Logging/LogMacros.h"

#include <algorithm>
#include <atomic>

namespace Speed::RollbackAudit
{
namespace
{
constexpr uint32 MaxSeconds = 600;
constexpr uint64 EventLimit = 128;
std::atomic<uint64> NextWindow{1};
}

void FWindow::Configure(uint32 InSeconds, uint64 WorkerGeneration, uint32 DriverId, bool bExactProvider) noexcept
{
    *this = {};
    if (InSeconds == 0 || InSeconds > MaxSeconds || WorkerGeneration == 0) return;
    Window = NextWindow.fetch_add(1, std::memory_order_relaxed);
    Seconds = InSeconds;
    Generation = WorkerGeneration;
    Host = DriverId;
    bExact = bExactProvider;
}

bool FWindow::IsOpenFor(uint64 WorkerGeneration) const noexcept
{
    return bOpen && Generation == WorkerGeneration && Thread == FPlatformTLS::GetCurrentThreadId();
}

void FWindow::AtInputBoundary(uint64 WorkerGeneration, uint64 NextFrame)
{
    if (!Enabled() || bClosed || Generation != WorkerGeneration) return;
    const double Now = FPlatformTime::Seconds();
    if (!bOpen)
    {
        Thread = FPlatformTLS::GetCurrentThreadId();
        Deadline = Now + Seconds;
        FirstNextFrame = NextFrame;
        bOpen = true;
        UE_LOG(LogTemp, Display, TEXT("[RollbackWorkerAuditBegin] Schema=1 Domain=DedicatedOwnedLegacyDequeues Window=%llu Driver=%u WorkerGeneration=%llu Thread=%u FirstNextFrame=%llu MaxSeconds=%u EventLimit=%llu ExactIngressDisabled=%d"),
            Window, Host, Generation, Thread, FirstNextFrame, Seconds, EventLimit, bExact ? 1 : 0);
    }
    else if (IsOpenFor(WorkerGeneration) && Now >= Deadline)
        Close(NextFrame, TEXT("Deadline"));
}

void FWindow::AfterWork(uint64 WorkerGeneration, uint64 NextFrame, bool bFailed, bool bNeedsOwnerBoundary)
{
    if (!IsOpenFor(WorkerGeneration)) return;
    ++Pulses;
    FailedPulses += bFailed;
    BoundaryPulses += bNeedsOwnerBoundary;
    if (FPlatformTime::Seconds() >= Deadline)
        Close(NextFrame, TEXT("Deadline"));
}

void FWindow::AtWorkerClose(uint64 WorkerGeneration, uint64 NextFrame)
{
    if (IsOpenFor(WorkerGeneration)) Close(NextFrame, TEXT("WorkerClose"));
}

bool FWindow::EmitEvent() noexcept
{
    if (Events < EventLimit)
    {
        ++Events;
        return true;
    }
    ++Omitted;
    return false;
}

void FWindow::RecordRequest(const FRequest& Request, uint64 AfterNextFrame)
{
    ++Dequeued;
    const TCHAR* Outcome = TEXT("Failed");
    switch (Request.Outcome)
    {
    case EOutcome::Refused: ++Refused; Outcome = TEXT("Refused"); break;
    case EOutcome::Failed: ++Failed; break;
    case EOutcome::Succeeded: ++Succeeded; Outcome = TEXT("Succeeded"); break;
    }
    const uint64 RequestedDepth = Request.TargetFrame >= Request.RestoreFrame
        ? Request.TargetFrame - Request.RestoreFrame : 0;
    ReplayAttempts += Request.ReplayAttempts;
    ReplayCompleted += Request.ReplayCompleted;
    MaxRequestedDepth = std::max(MaxRequestedDepth, RequestedDepth);
    MaxCompletedDepth = std::max(MaxCompletedDepth, Request.ReplayCompleted);
    if (EmitEvent())
        UE_LOG(LogTemp, Display, TEXT("[RollbackWorkerAuditRequest] Window=%llu WorkerGeneration=%llu Sequence=%llu Outcome=%s Stage=%s BeforeNextFrame=%llu RestoreFrame=%llu TargetFrame=%llu AfterNextFrame=%llu RequestedDepth=%llu ReplayAttempts=%llu ReplayCompleted=%llu"),
            Window, Generation, Dequeued, Outcome, Request.Stage, Request.BeforeNextFrame, Request.RestoreFrame,
            Request.TargetFrame, AfterNextFrame, RequestedDepth, Request.ReplayAttempts, Request.ReplayCompleted);
}

void FWindow::RecordRound(bool bNeutralPublished, uint64 NeutralFrame, uint64 RoundGeneration)
{
    if (bNeutralPublished) ++RoundNeutralPublications;
    else ++RoundCommits;
    if (EmitEvent())
        UE_LOG(LogTemp, Display, TEXT("[RollbackWorkerAuditRound] Window=%llu WorkerGeneration=%llu Event=%s NeutralFrame=%llu RoundGeneration=%llu"),
            Window, Generation, bNeutralPublished ? TEXT("NeutralPublished") : TEXT("Commit"), NeutralFrame, RoundGeneration);
}

void FWindow::Close(uint64 NextFrame, const TCHAR* Reason)
{
    bOpen = false;
    bClosed = true;
    UE_LOG(LogTemp, Display, TEXT("[RollbackWorkerAuditEnd] Schema=1 Domain=DedicatedOwnedLegacyDequeues Window=%llu Driver=%u WorkerGeneration=%llu Thread=%u Reason=%s Status=%s FirstNextFrame=%llu EndNextFrame=%llu WorkPulses=%llu FailedPulses=%llu NeedsOwnerBoundaryPulses=%llu SlotChecks=%llu ForwardAttempts=%llu ForwardCompleted=%llu Dequeued=%llu Refused=%llu Failed=%llu Succeeded=%llu ReplayAttempts=%llu ReplayCompleted=%llu MaxRequestedDepth=%llu MaxCompletedDepth=%llu RoundCommits=%llu RoundNeutralPublications=%llu Events=%llu EventsOmitted=%llu ExactIngressDisabled=%d"),
        Window, Host, Generation, Thread, Reason, SlotChecks ? TEXT("Observed") : TEXT("NoSlotChecks"),
        FirstNextFrame, NextFrame, Pulses, FailedPulses, BoundaryPulses, SlotChecks, ForwardAttempts, ForwardCompleted,
        Dequeued, Refused, Failed, Succeeded, ReplayAttempts, ReplayCompleted, MaxRequestedDepth, MaxCompletedDepth,
        RoundCommits, RoundNeutralPublications, Events, Omitted, bExact ? 1 : 0);
}
}
