#pragma once

#include "CoreTypes.h"

namespace Speed::RollbackAudit
{
enum class EOutcome : uint8 { Refused, Failed, Succeeded };

// One request taken from the legacy rollback slot. Endpoints are zero-based.
struct FRequest
{
    uint64 BeforeNextFrame, RestoreFrame, TargetFrame;
    uint64 ReplayAttempts = 0, ReplayCompleted = 0;
    EOutcome Outcome = EOutcome::Failed;
    const TCHAR* Stage = TEXT("Validation");
};

// Only the original dedicated worker mutates an open window. Configure runs
// before worker startup. The elapsed-time cutoff is sampled at worker edges
// so an in-flight pulse and its request outcome are kept together.
class FWindow
{
public:
    void Configure(uint32 Seconds, uint64 WorkerGeneration, uint32 DriverId, bool bExactProvider) noexcept;
    bool Enabled() const noexcept { return Seconds != 0; }
    bool IsOpenFor(uint64 WorkerGeneration) const noexcept;
    void AtInputBoundary(uint64 WorkerGeneration, uint64 NextFrame);
    void AfterWork(uint64 WorkerGeneration, uint64 NextFrame, bool bFailed, bool bNeedsOwnerBoundary);
    void AtWorkerClose(uint64 WorkerGeneration, uint64 NextFrame);
    void CheckRollbackSlot() noexcept { ++SlotChecks; }
    void AttemptForward() noexcept { ++ForwardAttempts; }
    void CompleteForward() noexcept { ++ForwardCompleted; }
    void RecordRequest(const FRequest& Request, uint64 AfterNextFrame);
    void RecordRound(bool bNeutralPublished, uint64 NeutralFrame, uint64 RoundGeneration);

private:
    void Close(uint64 NextFrame, const TCHAR* Reason);
    bool EmitEvent() noexcept;

    uint32 Seconds = 0, Host = 0, Thread = 0;
    uint64 Window = 0, Generation = 0, FirstNextFrame = 0;
    double Deadline = 0;
    bool bExact = false, bOpen = false, bClosed = false;

    uint64 Pulses = 0, FailedPulses = 0, BoundaryPulses = 0;
    uint64 SlotChecks = 0, ForwardAttempts = 0, ForwardCompleted = 0;
    uint64 Dequeued = 0, Refused = 0, Failed = 0, Succeeded = 0;
    uint64 ReplayAttempts = 0, ReplayCompleted = 0, MaxRequestedDepth = 0, MaxCompletedDepth = 0;
    uint64 RoundCommits = 0, RoundNeutralPublications = 0, Events = 0, Omitted = 0;
};
}
