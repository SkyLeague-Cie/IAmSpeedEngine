#pragma once
#include "CoreMinimal.h"
#include "SimulationWorker.h"
class ASpeedSimulation;
struct FSpeedStepDiagnostics;

// Diagnostic ABI only. No member, virtual method, canonical state or camera
// codec is added to a simulation/actor/producer class.
#if WITH_DEV_AUTOMATION_TESTS
namespace Speed::OwnedFrameAudit
{
// Explicit native-test transport scope. No simulation/diagnostic record ABI
// changes; false leaves both outputs unchanged. Used only at capture admission
// and frozen export, never by physical Step/Work/Wait.
IAMSPEED_API bool GetDeclaredMainNativeOutputRoot(FString& OutRoot,FString& OutSavedDir);
enum class EFreezeReason : uint32 { ControlledCutoff = 1, WorkerClose = 2 };
enum class EAuxKind : uint32 { Service = 1, Work, Wait, EventWait, ParkWait };
enum EClockFlags : uint32 { QpcValid = 1, ThreadCyclesValid = 2, ThreadCpuValid = 4 };
struct FClockEdge
{
    uint64 WallTicks = 0, ThreadCycles = 0, Kernel100ns = 0, User100ns = 0;
    uint32 Flags = 0;
};
struct FSpan { FClockEdge Begin, End; uint32 Present = 0, Returned = 0; };
struct FFrameRecord
{
    uint64 ExecutionOrdinal = 0, PulseOrdinal = 0, ServiceCycle = 0;
    uint64 CanonicalFrame = MAX_uint64, CameraFrame = MAX_uint64;
    FSpan Step, Camera;
    uint32 CameraCalls = 0, Replay = 0, StepSucceeded = 0;
};
struct FAuxRecord
{
    uint64 Ordinal = 0, PulseOrdinal = 0, ServiceCycle = 0;
    uint64 LastStepExecution = 0, CandidateFrame = MAX_uint64;
    FSpan Span;
    uint64 DeadlineBits = 0, PrecisionBits = 0, RemainingBits = 0;
    uint64 ClockSamples = 0, YieldCalls = 0, YieldTotalTicks = 0, YieldMaxTicks = 0;
    uint64 InteriorEntryTicks = 0, InteriorReturnTicks = 0, InteriorReturnCount = 0;
    uint32 Kind = 0, RequestedMilliseconds = 0, PauseRequested = 0;
    int32 Outcome = -1;
};
struct FSummary
{
    uint64 FrameCount = 0, AuxCount = 0, PulseCount = 0, ServiceCycles = 0;
    uint64 ClockReads = 0, DroppedFrames = 0, DroppedAux = 0;
    uint64 FirstExecution = 0, LastExecution = 0, FirstCanonicalFrame = MAX_uint64, LastCanonicalFrame = MAX_uint64;
    uint64 LastCameraFrame = MAX_uint64, LastCameraExecution = 0;
    uint64 QpcFrequency = 0, ProcessBirthTicks = 0;
    uint32 ProcessId = 0, OwnerThreadId = 0, FreezeReason = 0;
    uint32 WrongOwner = 0, HookAttachFailed = 0, HookClearFailed = 0;
    uint32 NestedStep = 0, AddressMismatch = 0, OpenSpanAtFreeze = 0;
    uint32 FrozenSafe = 0, CompletePrefix = 0, CloseReached = 0;
    uint32 ExportClaimed = 0, ExportSucceeded = 0;
};
class FSession;
using FSessionPtr = TSharedPtr<FSession, ESPMode::ThreadSafe>;
struct FStepHandle { FSession* Session = nullptr; FFrameRecord* Record = nullptr; };
// Separate opt-in QPC-only sidecar. The original frame/aux binary ABI is unchanged.
enum class ECanonicalEdge : uint32
{
    Entry, Admission, Registry, ScenarioInputs, InputPrepare, InputInstall, InputSnapshot, StaticAudit, InputJournal, CanonicalInputs, WorldPrepare, WorldStep, Snapshot, ProducerList, Presentation, InputValidate, Publication, InputComplete, PostCommit, CameraSample, FrameHash, Exit, Count
};
struct FCanonicalPhaseRecord
{
    uint64 ExecutionOrdinal = 0, CanonicalFrame = MAX_uint64, PresentMask = 0;
    uint64 WallTicks[uint32(ECanonicalEdge::Count)]{};
    uint32 DuplicateEdges = 0, InvalidClocks = 0;
};
// Separate optional copy of existing solver diagnostics. No new physical clocks.
// Complete detail requires ValidMask==31, CaptureCalls==1 and offline finite/nonnegative times.
enum EPhysicalDetailFlags : uint32
{
    BeforeSerialRecorded = 1, DiagnosticsCopied = 2, SerialAdvancedOnce = 4,
    DiagnosticFrameMatches = 8, IntegerCountsNonnegative = 16
};
struct FPhysicalDetailRecord
{
    uint64 ExecutionOrdinal = 0, CanonicalFrame = MAX_uint64;
    uint64 DiagnosticSerial = 0, SerialBeforeStep = 0;
    uint32 DiagnosticFrame = 0, ValidMask = 0;
    int32 IterationCount = 0, MaximumIterationCount = 0;
    int32 ComponentSweepCount = 0, ResolvedEventCount = 0;
    uint32 StaticQueryCount = 0, LegacySweepCount = 0, AuthorityAttemptCount = 0;
    uint32 IterationLimitReached = 0, CaptureCalls = 0, Reserved = 0;
    double PhysicalDeltaTimeMilliseconds = 0, TotalMilliseconds = 0;
    double ResetMilliseconds = 0, SweepMilliseconds = 0;
    double StaticAuthorityMilliseconds = 0, MaximumStaticAuthorityMilliseconds = 0;
    double IntegrateMilliseconds = 0, ResolveMilliseconds = 0;
    double ProjectionMilliseconds = 0, PostMilliseconds = 0;
};
class IAMSPEED_API FCanonicalPhaseScope
{
public:
    explicit FCanonicalPhaseScope(uint64 Frame) noexcept;
    ~FCanonicalPhaseScope();
    void Mark(ECanonicalEdge Edge) noexcept;
    bool CapturesPhysicalDetail() const noexcept { return PhysicalRecord != nullptr; }
    void CopyPhysicalDetail(const FSpeedStepDiagnostics& Diagnostics, uint64 SerialBefore) noexcept;
    FCanonicalPhaseScope(const FCanonicalPhaseScope&) = delete;
    FCanonicalPhaseScope& operator=(const FCanonicalPhaseScope&) = delete;
private:
    FCanonicalPhaseRecord* Record = nullptr;
    FSession* PhysicalSession = nullptr;
    FPhysicalDetailRecord* PhysicalRecord = nullptr;
};
IAMSPEED_API bool ReadFrozenCanonicalPhaseForTests(const FSessionPtr& Session,
    uint64 Index, FCanonicalPhaseRecord& Out);
IAMSPEED_API bool ReadFrozenPhysicalDetailForTests(const FSessionPtr& Session,
    uint64 Index, FPhysicalDetailRecord& Out);


// Production admission is opt-in, normal-owned RealTime only, with a private
// output path. Allocations/pre-touch happen on the GT before worker creation.
IAMSPEED_API FSessionPtr CreateForHost(ASpeedSimulation* Host, bool Enabled, const TCHAR* CommandLine);
// Development-only harness API; tiny bounds exercise the SAME implementation.
IAMSPEED_API FSessionPtr CreateForTests(uint64 FrameCapacity, uint64 AuxCapacity,
    const FString& BinaryPath = FString(), bool CaptureCpu = true, bool CaptureCanonicalPhases = false, bool CapturePhysicalDetails = false);
IAMSPEED_API void OwnerBoundary(const FSessionPtr& Session, bool AuditStillEnabled);
IAMSPEED_API void BeginService(const FSessionPtr& Session, bool Paused, uint64 CandidateFrame);
IAMSPEED_API void EndService(const FSessionPtr& Session, ESimulationBoundaryResult Result);
IAMSPEED_API void BeginWork(const FSessionPtr& Session, uint64 CandidateFrame);
IAMSPEED_API void EndWork(const FSessionPtr& Session, ESimulationWorkerResult Result);
// These brackets are OUTSIDE StepCanonicalFrame, including the replay loop.
IAMSPEED_API FStepHandle BeginStep(uint64 Frame, bool Replay);
IAMSPEED_API void EndStep(FStepHandle Handle, bool Returned, bool Succeeded);
// Only direct primitive reads/scalar fields in the already reserved record.
// No callback, collector append, allocation, formatter or I/O in camera Produce.
class IAMSPEED_API FCameraScope
{
public:
    FCameraScope(uint64 Frame, bool IsCamera);
    ~FCameraScope();
    void MarkReturned() noexcept { if (Record) Record->Camera.Returned = 1; }
    FCameraScope(const FCameraScope&) = delete;
    FCameraScope& operator=(const FCameraScope&) = delete;
private:
    FSession* Session = nullptr;
    FFrameRecord* Record = nullptr;
};
IAMSPEED_API void BeginWait(const FSessionPtr& Session);
IAMSPEED_API void EndWait(const FSessionPtr& Session);
// Separate TLS; NEVER installs/clears/replaces the existing WaitAudit observer.
IAMSPEED_API FSession* ActiveForCurrentThread() noexcept;
IAMSPEED_API void RecordWaitSample(FSession* Session,
    const Speed::Testing::FSimulationWaitAuditSample& Sample) noexcept;
IAMSPEED_API bool ParkWait(FEvent& Event, uint32 Milliseconds);
IAMSPEED_API void FreezeOnOwner(const FSessionPtr& Session, EFreezeReason Reason);
// No reading mutable data: all of these require an actual Frozen release ACK.
IAMSPEED_API bool CopyFrozenSummary(const FSessionPtr& Session, FSummary& Out);
IAMSPEED_API bool ReadFrozenFrameForTests(const FSessionPtr& Session, uint64 Index, FFrameRecord& Out);
IAMSPEED_API bool ReadFrozenAuxForTests(const FSessionPtr& Session, uint64 Index, FAuxRecord& Out);
IAMSPEED_API bool ReleaseOwnedHookForTesting(const FSessionPtr& Session);
IAMSPEED_API bool TryExportFrozen(const FSessionPtr& Session);
IAMSPEED_API void TryExportForHostOnGameThread(const void* Host);
}
#endif
