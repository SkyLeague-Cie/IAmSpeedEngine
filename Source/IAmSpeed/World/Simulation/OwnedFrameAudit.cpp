#include "OwnedFrameAudit.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "SpeedSimulation.h"
#include "RealTimeSimulation.h"
#include "../Subsystem/SpeedWorldSubsystem.h"
#include "HAL/Event.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTLS.h"
#include "HAL/PlatformTime.h"
#include "HAL/FileManager.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"
#include "Serialization/Archive.h"
#include <atomic>
#include <memory>
#include <type_traits>
#if PLATFORM_WINDOWS
#include "Windows/WindowsHWrapper.h"
#endif

namespace Speed::OwnedFrameAudit
{
namespace
{
constexpr uint64 MaxFrames = 1800ull * 300ull; // Existing player cap/rate, not a solver threshold.
constexpr uint64 MaxAux = MaxFrames * 8ull;
thread_local FSession* ActiveSession = nullptr;
struct FHostCapture { FSessionPtr Current; uint64 Generation = 0; };
TMap<const void*, FHostCapture> HostSessions; // GT only; never read by an owner worker.

bool PrivatePath(const FString& Path)
{
    if (Path.IsEmpty() || FPaths::IsRelative(Path)) return false;
    FString Full = FPaths::ConvertRelativePathToFull(Path);
    FPaths::NormalizeFilename(Full);
    if (!FPaths::CollapseRelativeDirectories(Full)) return false;
    return Full.StartsWith(TEXT("E:/UELeadPrivate/"), ESearchCase::IgnoreCase);
}
struct FBinaryHeader
{
    uint64 Magic = 0x314455414E574F53ull; // SOWN AUD1, native little-endian development schema.
    uint32 Version = 1, FrameBytes = sizeof(FFrameRecord), AuxBytes = sizeof(FAuxRecord);
    uint32 HeaderBytes = 0;
    uint64 FrameCapacity = 0, AuxCapacity = 0;
    FSummary Summary;
};
struct FCanonicalPhaseHeader
{
    uint64 Magic = 0x315348504E574F53ull; // SOWNPHS1, separate diagnostic sidecar.
    uint32 Version = 1, RecordBytes = sizeof(FCanonicalPhaseRecord), HeaderBytes = 0;
    uint32 EdgeCount = uint32(ECanonicalEdge::Count);
    uint64 FrameCapacity = 0;
    FSummary Summary;
};
struct FPhysicalDetailHeader
{
    uint64 Magic = 0x315948504E574F53ull; // SOWNPHY1; original and phase ABIs unchanged.
    uint32 Version = 1, RecordBytes = sizeof(FPhysicalDetailRecord), HeaderBytes = 0;
    uint32 Flags = 0; // reserved, must remain zero
    uint64 FrameCapacity = 0;
    FSummary Summary;
};
static_assert(std::is_trivially_copyable_v<FPhysicalDetailRecord>);
static_assert(sizeof(FPhysicalDetailRecord) == 160);
static_assert(sizeof(FPhysicalDetailHeader) == 208);
static_assert(MaxFrames * sizeof(FPhysicalDetailRecord) == 86400000ull);
static_assert(std::is_trivially_copyable_v<FCanonicalPhaseRecord>);
static_assert(sizeof(FCanonicalPhaseRecord) == 208);
static_assert(sizeof(FCanonicalPhaseHeader) == 208);
static_assert(MaxFrames * sizeof(FCanonicalPhaseRecord) == 112320000ull);
static_assert(std::is_trivially_copyable_v<FFrameRecord>);
static_assert(std::is_trivially_copyable_v<FAuxRecord>);
static_assert(std::is_trivially_copyable_v<FBinaryHeader>);
#if PLATFORM_WINDOWS
// Compile-time admission of the EXACT native diagnostic layout/bounds. A
// different ABI is rejected before a recorder or buffer can be allocated.
static_assert(sizeof(FFrameRecord) == 232, "Owned audit frame layout mismatch");
static_assert(sizeof(FAuxRecord) == 224, "Owned audit auxiliary layout mismatch");
static_assert(sizeof(FSummary) == 176, "Owned audit summary layout mismatch");
static_assert(sizeof(FBinaryHeader) == 216, "Owned audit header layout mismatch");
constexpr uint64 BoundedArrayBytes = MaxFrames * sizeof(FFrameRecord) + MaxAux * sizeof(FAuxRecord);
static_assert(BoundedArrayBytes == 1092960000ull, "Owned audit allocation budget mismatch");
#endif
}

class FSession
{
public:
    FSession(uint64 InFrames, uint64 InAux, const FString& InPath, bool InCpu, bool InPhases, bool InPhysical)
        : FrameCapacity(InFrames), AuxCapacity(InAux), Path(InPath), CaptureCpu(InCpu),
          Frames(std::make_unique<FFrameRecord[]>(InFrames)), Aux(std::make_unique<FAuxRecord[]>(InAux)),
          CanonicalPhases(InPhases ? std::make_unique<FCanonicalPhaseRecord[]>(InFrames) : nullptr),
          PhysicalDetails(InPhysical ? std::make_unique<FPhysicalDetailRecord[]>(InFrames) : nullptr)
    {
        Stats.ProcessId = FPlatformProcess::GetCurrentProcessId();
#if PLATFORM_WINDOWS
        LARGE_INTEGER Frequency;
        if (::QueryPerformanceFrequency(&Frequency) && Frequency.QuadPart > 0)
            Stats.QpcFrequency = uint64(Frequency.QuadPart);
        FILETIME Creation, Exit, Kernel, User;
        if (::GetProcessTimes(::GetCurrentProcess(), &Creation, &Exit, &Kernel, &User))
            Stats.ProcessBirthTicks = uint64(Creation.dwLowDateTime) | (uint64(Creation.dwHighDateTime) << 32);
#endif
        // Value-initialized fixed arrays pre-touch their actual storage on the
        // creator GT. No allocation/resize occurs in Work, Step, camera or Wait.
    }
    bool Owner() const { return OwnerId.load(std::memory_order_acquire) == FPlatformTLS::GetCurrentThreadId(); }
    bool Writable()
    {
        if (!Owner()) { WrongOwner.fetch_add(1, std::memory_order_relaxed); return false; }
        return Recording && !Frozen.load(std::memory_order_acquire);
    }
    FClockEdge Clock()
    {
        FClockEdge Edge;
        ++Stats.ClockReads;
#if PLATFORM_WINDOWS
        const DWORD LastError = ::GetLastError();
        LARGE_INTEGER Counter;
        if (::QueryPerformanceCounter(&Counter) && Counter.QuadPart >= 0 && Stats.QpcFrequency)
        { Edge.WallTicks = uint64(Counter.QuadPart); Edge.Flags |= QpcValid; }
        if (CaptureCpu)
        {
            ULONG64 Cycles = 0;
            if (::QueryThreadCycleTime(::GetCurrentThread(), &Cycles))
            { Edge.ThreadCycles = Cycles; Edge.Flags |= ThreadCyclesValid; }
            FILETIME Creation, Exit, Kernel, User;
            if (::GetThreadTimes(::GetCurrentThread(), &Creation, &Exit, &Kernel, &User))
            {
                Edge.Kernel100ns = uint64(Kernel.dwLowDateTime) | (uint64(Kernel.dwHighDateTime) << 32);
                Edge.User100ns = uint64(User.dwLowDateTime) | (uint64(User.dwHighDateTime) << 32);
                Edge.Flags |= ThreadCpuValid;
            }
        }
        ::SetLastError(LastError);
#endif
        return Edge;
    }
    FAuxRecord NewAux(EAuxKind Kind)
    {
        FAuxRecord Row;
        FMemory::Memzero(&Row, sizeof(Row));
        Row.Kind = uint32(Kind); Row.Outcome = -1;
        Row.PulseOrdinal = Stats.PulseCount; Row.ServiceCycle = Stats.ServiceCycles;
        Row.LastStepExecution = Stats.LastExecution; Row.CandidateFrame = CandidateFrame;
        return Row;
    }
    void CommitAux(FAuxRecord& Row)
    {
        if (!Writable()) return;
        if (Stats.AuxCount == AuxCapacity) { ++Stats.DroppedAux; return; }
        Row.Ordinal = Stats.AuxCount + 1;
        Aux[Stats.AuxCount++] = Row;
    }

    uint64 FrameCapacity, AuxCapacity;
    FString Path;
    bool CaptureCpu = true, Recording = false;
    std::atomic<uint32> OwnerId{0};
    std::atomic<uint32> WrongOwner{0};
    std::atomic<bool> Frozen{false}, ExportClaimed{false}, ExportSucceeded{false};
    std::unique_ptr<FFrameRecord[]> Frames;
    std::unique_ptr<FAuxRecord[]> Aux;
    std::unique_ptr<FCanonicalPhaseRecord[]> CanonicalPhases;
    std::unique_ptr<FPhysicalDetailRecord[]> PhysicalDetails;
    FSummary Stats;
    FFrameRecord* CurrentStep = nullptr;
    uint64 CandidateFrame = MAX_uint64, NextExecution = 0;
    FAuxRecord Service, Work, Wait, Event;
    bool ServiceOpen = false, WorkOpen = false, WaitOpen = false, EventOpen = false;
    uint64 YieldEntry = 0;
    bool YieldOpen = false;
};

FSessionPtr CreateForTests(uint64 FrameCapacity, uint64 AuxCapacity,
    const FString& BinaryPath, bool CaptureCpu, bool CaptureCanonicalPhases, bool CapturePhysicalDetails)
{
#if !PLATFORM_WINDOWS
    // This diagnostic authenticates Windows QPC/thread counters and explicit
    // private Windows outputs. Unsupported platforms do not allocate a session.
    return {};
#endif
    if ((CapturePhysicalDetails && !CaptureCanonicalPhases) ||
        !IsInGameThread() || !FrameCapacity || FrameCapacity > MaxFrames ||
        !AuxCapacity || AuxCapacity > MaxAux || (!BinaryPath.IsEmpty() && !PrivatePath(BinaryPath))) return {};
    try { return MakeShared<FSession, ESPMode::ThreadSafe>(FrameCapacity, AuxCapacity, BinaryPath, CaptureCpu, CaptureCanonicalPhases, CapturePhysicalDetails); }
    catch (...) { return {}; } // Diagnostic refusal never changes physical admission.
}

FSessionPtr CreateForHost(ASpeedSimulation* Host, bool Enabled, const TCHAR* CommandLine)
{
#if !PLATFORM_WINDOWS
    return {}; // No allocation or actor inspection for unsupported capture.
#endif
    // Enabled is the caller's ACTUAL normal-owned mode + CVar admission;
    // execution-mode accessor stays private, without a friend/layout change.
    if (!Enabled || !Host || !IsInGameThread() || !Host->IsA<ARealTimeSimulation>()) return {};
    FString Path;
    if (!FParse::Value(CommandLine, TEXT("SLOwnedFrameAuditBinary="), Path) || !PrivatePath(Path)) return {};
    FHostCapture& Capture = HostSessions.FindOrAdd(Host);
    if (Capture.Current)
    {
        // A new owned generation may not read/replace a live recorder. Export
        // the previous immutable prefix before releasing its GT reference.
        if (!Capture.Current->Frozen.load(std::memory_order_acquire)) return {};
        if (!Capture.Current->ExportSucceeded.load(std::memory_order_acquire)) TryExportFrozen(Capture.Current);
        if (!Capture.Current->ExportSucceeded.load(std::memory_order_acquire)) return {};
    }
    const uint64 Generation = Capture.Generation + 1;
    const FString GenerationPath = Path + FString::Printf(TEXT(".g%06llu.bin"), static_cast<unsigned long long>(Generation));
    FSessionPtr Session = CreateForTests(MaxFrames, MaxAux, GenerationPath, true,
        FParse::Param(CommandLine, TEXT("SLOwnedCanonicalPhases")),
        FParse::Param(CommandLine, TEXT("SLOwnedPhysicalDetails")));
    if (Session) { Capture.Current = Session; Capture.Generation = Generation; }
    return Session;
}

FSession* ActiveForCurrentThread() noexcept { return ActiveSession; }

void OwnerBoundary(const FSessionPtr& Session, bool AuditStillEnabled)
{
    if (!Session || Session->Frozen.load(std::memory_order_acquire)) return;
    const uint32 ThisThread = FPlatformTLS::GetCurrentThreadId();
    if (IsInGameThread()) { Session->WrongOwner.fetch_add(1, std::memory_order_relaxed); return; }
    uint32 ExpectedOwner = 0;
    if (Session->OwnerId.compare_exchange_strong(ExpectedOwner, ThisThread, std::memory_order_acq_rel))
    {
        Session->Stats.OwnerThreadId = ThisThread;
        if (ActiveSession) { ++Session->Stats.HookAttachFailed; return; }
        ActiveSession = Session.Get(); Session->Recording = true;
    }
    else if (ExpectedOwner != ThisThread)
    { Session->WrongOwner.fetch_add(1, std::memory_order_relaxed); return; }
    if (ActiveSession != Session.Get()) { ++Session->Stats.HookAttachFailed; return; }
    if (!AuditStillEnabled) FreezeOnOwner(Session, EFreezeReason::ControlledCutoff);
}

void BeginService(const FSessionPtr& Session, bool Paused, uint64 CandidateFrame)
{
    if (!Session || !Session->Writable() || ActiveSession != Session.Get()) return;
    ++Session->Stats.ServiceCycles; Session->CandidateFrame = CandidateFrame;
    Session->Service = Session->NewAux(EAuxKind::Service);
    Session->Service.PauseRequested = Paused; Session->Service.Span.Begin = Session->Clock();
    Session->Service.Span.Present = 1; Session->ServiceOpen = true;
}
void EndService(const FSessionPtr& Session, ESimulationBoundaryResult Result)
{
    if (!Session || !Session->Writable() || !Session->ServiceOpen) return;
    Session->Service.Span.End = Session->Clock(); Session->Service.Span.Returned = 1;
    Session->Service.Outcome = int32(Result); Session->ServiceOpen = false;
    Session->CommitAux(Session->Service);
}
void BeginWork(const FSessionPtr& Session, uint64 CandidateFrame)
{
    if (!Session || !Session->Writable() || ActiveSession != Session.Get()) return;
    ++Session->Stats.PulseCount; Session->CandidateFrame = CandidateFrame;
    Session->Work = Session->NewAux(EAuxKind::Work);
    Session->Work.Span.Begin = Session->Clock(); Session->Work.Span.Present = 1;
    Session->WorkOpen = true;
}
void EndWork(const FSessionPtr& Session, ESimulationWorkerResult Result)
{
    if (!Session || !Session->Writable() || !Session->WorkOpen) return;
    Session->Work.Span.End = Session->Clock(); Session->Work.Span.Returned = 1;
    Session->Work.Outcome = int32(Result); Session->WorkOpen = false;
    Session->Work.LastStepExecution = Session->Stats.LastExecution;
    Session->CommitAux(Session->Work);
}

FStepHandle BeginStep(uint64 Frame, bool Replay)
{
    FSession* Session = ActiveSession;
    if (!Session || !Session->Writable()) return {};
    if (Session->CurrentStep) { ++Session->Stats.NestedStep; return {}; }
    const uint64 Execution = ++Session->NextExecution;
    if (Session->Stats.FrameCount == Session->FrameCapacity) { ++Session->Stats.DroppedFrames; return {}; }
    FFrameRecord* Row = &Session->Frames[Session->Stats.FrameCount];
    FMemory::Memzero(Row, sizeof(*Row));
    Row->ExecutionOrdinal = Execution; Row->PulseOrdinal = Session->Stats.PulseCount;
    Row->ServiceCycle = Session->Stats.ServiceCycles; Row->CanonicalFrame = Frame;
    Row->CameraFrame = MAX_uint64; Row->Replay = Replay;
    Row->Step.Begin = Session->Clock(); Row->Step.Present = 1;
    if (Session->CanonicalPhases)
    {
        auto& Phases = Session->CanonicalPhases[Session->Stats.FrameCount];
        FMemory::Memzero(&Phases, sizeof(Phases));
        Phases.ExecutionOrdinal = Execution; Phases.CanonicalFrame = Frame;
    }
    Session->CurrentStep = Row;
    return {Session, Row};
}
void EndStep(FStepHandle Handle, bool Returned, bool Succeeded)
{
    FSession* Session = Handle.Session;
    if (!Session || !Handle.Record || !Session->Writable()) return;
    if (Session->CurrentStep != Handle.Record) { ++Session->Stats.AddressMismatch; return; }
    Handle.Record->Step.End = Session->Clock(); Handle.Record->Step.Returned = Returned;
    Handle.Record->StepSucceeded = Succeeded;
    if (!Session->Stats.FrameCount)
    { Session->Stats.FirstExecution = Handle.Record->ExecutionOrdinal; Session->Stats.FirstCanonicalFrame = Handle.Record->CanonicalFrame; }
    Session->Stats.LastExecution = Handle.Record->ExecutionOrdinal;
    Session->Stats.LastCanonicalFrame = Handle.Record->CanonicalFrame;
    if (Handle.Record->CameraCalls)
    { Session->Stats.LastCameraFrame = Handle.Record->CameraFrame; Session->Stats.LastCameraExecution = Handle.Record->ExecutionOrdinal; }
    ++Session->Stats.FrameCount; // OUTSIDE the complete StepCanonicalFrame function.
    Session->CurrentStep = nullptr;
}
FCanonicalPhaseScope::FCanonicalPhaseScope(uint64 Frame) noexcept
{
    FSession* Session = ActiveSession;
    if (!Session || !Session->CanonicalPhases || !Session->Writable() || !Session->CurrentStep) return;
    if (Session->CurrentStep->CanonicalFrame != Frame) { ++Session->Stats.AddressMismatch; return; }
    Record = &Session->CanonicalPhases[Session->Stats.FrameCount];
    if (Session->PhysicalDetails)
    {
        PhysicalSession = Session;
        PhysicalRecord = &Session->PhysicalDetails[Session->Stats.FrameCount];
        FMemory::Memzero(PhysicalRecord, sizeof(*PhysicalRecord));
        PhysicalRecord->ExecutionOrdinal = Session->CurrentStep->ExecutionOrdinal;
        PhysicalRecord->CanonicalFrame = Frame;
    }
    Mark(ECanonicalEdge::Entry);
}
FCanonicalPhaseScope::~FCanonicalPhaseScope() { Mark(ECanonicalEdge::Exit); }
void FCanonicalPhaseScope::Mark(ECanonicalEdge Edge) noexcept
{
    if (!Record) return;
    const uint32 Index = uint32(Edge);
    if (Index >= uint32(ECanonicalEdge::Count)) { ++Record->DuplicateEdges; return; }
    const uint64 Bit = uint64(1) << Index;
    if (Record->PresentMask & Bit) { ++Record->DuplicateEdges; return; }
#if PLATFORM_WINDOWS
    // Only scalar writes and a primitive clock read. Preserve the caller's
    // Win32 error state; no collector, allocation, format, I/O or callback.
    const DWORD LastError = ::GetLastError();
    LARGE_INTEGER Counter;
    const bool Valid = ::QueryPerformanceCounter(&Counter) && Counter.QuadPart >= 0;
    ::SetLastError(LastError);
    if (Valid) { Record->WallTicks[Index] = uint64(Counter.QuadPart); Record->PresentMask |= Bit; }
    else ++Record->InvalidClocks;
#else
    ++Record->InvalidClocks;
#endif
}
bool ReadFrozenCanonicalPhaseForTests(const FSessionPtr& Session, uint64 Index, FCanonicalPhaseRecord& Out)
{
    if (!Session || !Session->Frozen.load(std::memory_order_acquire) ||
        !Session->CanonicalPhases || Index >= Session->Stats.FrameCount) return false;
    Out = Session->CanonicalPhases[Index]; return true;
}

void FCanonicalPhaseScope::CopyPhysicalDetail(const FSpeedStepDiagnostics& D, uint64 Before) noexcept
{
    if (!PhysicalRecord || !PhysicalSession || !PhysicalSession->Writable()) return;
    if (!PhysicalSession->CurrentStep ||
        PhysicalSession->CurrentStep->ExecutionOrdinal != PhysicalRecord->ExecutionOrdinal ||
        PhysicalSession->CurrentStep->CanonicalFrame != PhysicalRecord->CanonicalFrame)
    { ++PhysicalSession->Stats.AddressMismatch; return; }
    if (++PhysicalRecord->CaptureCalls != 1) return; // never overwrite a duplicate
    auto& Out = *PhysicalRecord;
    Out.SerialBeforeStep = Before; Out.DiagnosticSerial = D.Serial;
    Out.DiagnosticFrame = D.Frame;
    Out.ValidMask = BeforeSerialRecorded | DiagnosticsCopied;
    if (Before != MAX_uint64 && D.Serial == Before + 1) Out.ValidMask |= SerialAdvancedOnce;
    if (Out.CanonicalFrame <= MAX_uint32 && D.Frame == uint32(Out.CanonicalFrame))
        Out.ValidMask |= DiagnosticFrameMatches;
    Out.IterationCount = D.IterationCount; Out.MaximumIterationCount = D.MaximumIterationCount;
    Out.ComponentSweepCount = D.ComponentSweepCount; Out.ResolvedEventCount = D.ResolvedEventCount;
    Out.StaticQueryCount = D.StaticQueryCount; Out.LegacySweepCount = D.LegacySweepCount;
    Out.AuthorityAttemptCount = D.AuthorityAttemptCount; Out.IterationLimitReached = D.bIterationLimitReached ? 1 : 0;
    if (D.IterationCount >= 0 && D.MaximumIterationCount >= 0 && D.ComponentSweepCount >= 0 && D.ResolvedEventCount >= 0)
        Out.ValidMask |= IntegerCountsNonnegative;
    // Copy only. No floating arithmetic, clock, formatter, allocation or I/O here.
    // Finite/nonnegative elapsed validation happens after drain, outside Step.
    Out.PhysicalDeltaTimeMilliseconds = D.PhysicalDeltaTimeMilliseconds;
    Out.TotalMilliseconds = D.TotalMilliseconds; Out.ResetMilliseconds = D.ResetMilliseconds;
    Out.SweepMilliseconds = D.SweepMilliseconds; Out.StaticAuthorityMilliseconds = D.StaticAuthorityMilliseconds;
    Out.MaximumStaticAuthorityMilliseconds = D.MaximumStaticAuthorityMilliseconds;
    Out.IntegrateMilliseconds = D.IntegrateMilliseconds; Out.ResolveMilliseconds = D.ResolveMilliseconds;
    Out.ProjectionMilliseconds = D.ProjectionMilliseconds; Out.PostMilliseconds = D.PostMilliseconds;
}
bool ReadFrozenPhysicalDetailForTests(const FSessionPtr& Session, uint64 Index, FPhysicalDetailRecord& Out)
{
    if (!Session || !Session->Frozen.load(std::memory_order_acquire) ||
        !Session->PhysicalDetails || Index >= Session->Stats.FrameCount) return false;
    Out = Session->PhysicalDetails[Index]; return true;
}

FCameraScope::FCameraScope(uint64 Frame, bool IsCamera)
{
    if (!IsCamera) return;
    Session = ActiveSession;
    if (!Session || !Session->Writable() || !Session->CurrentStep) { Session = nullptr; return; }
    Record = Session->CurrentStep;
    if (Record->CanonicalFrame != Frame) ++Session->Stats.AddressMismatch;
    Record->CameraFrame = Frame; ++Record->CameraCalls;
    if (Record->CameraCalls == 1) Record->Camera.Begin = Session->Clock();
    else ++Session->Stats.AddressMismatch; // A single span cannot qualify multiple camera producers.
    Record->Camera.Present = 1;
}
FCameraScope::~FCameraScope()
{
    // Retain the exact entry pointer; never re-read a CVar mid-Produce.
    if (!Session || !Record) return;
    Record->Camera.End = Session->Clock(); // Returned is set only after the actual Produce call returns.
}

void BeginWait(const FSessionPtr& Session)
{
    if (!Session || !Session->Writable() || ActiveSession != Session.Get()) return;
    Session->Wait = Session->NewAux(EAuxKind::Wait);
    Session->Wait.Span.Begin = Session->Clock(); Session->Wait.Span.Present = 1;
    Session->WaitOpen = true;
}
void EndWait(const FSessionPtr& Session)
{
    if (!Session || !Session->Writable() || !Session->WaitOpen) return;
    Session->Wait.Span.End = Session->Clock(); Session->Wait.Span.Returned = 1;
    Session->WaitOpen = false; Session->CommitAux(Session->Wait);
}
void RecordWaitSample(FSession* Session, const Speed::Testing::FSimulationWaitAuditSample& Sample) noexcept
{
    if (!Session || !Session->Writable() || !Session->WaitOpen) return;
    using E = Speed::Testing::ESimulationWaitAuditEvent;
    switch (Sample.Event)
    {
    case E::Entry:
        Session->Wait.DeadlineBits = Sample.DeadlineBits; Session->Wait.PrecisionBits = Sample.PrecisionBits;
        Session->Wait.InteriorEntryTicks = Sample.WallTicks;
        break;
    case E::RemainingClockSample:
        ++Session->Wait.ClockSamples; Session->Wait.RemainingBits = Sample.RemainingBits; break;
    case E::EventWaitEntry:
        Session->Event = Session->NewAux(EAuxKind::EventWait);
        Session->Event.DeadlineBits = Sample.DeadlineBits; Session->Event.PrecisionBits = Sample.PrecisionBits;
        Session->Event.RemainingBits = Sample.RemainingBits; Session->Event.RequestedMilliseconds = Sample.RequestedMilliseconds;
        Session->Event.Span.Begin.WallTicks = Sample.WallTicks; Session->Event.Span.Begin.Flags = 0; // Original sample has no availability BOOL.
        Session->Event.Span.Present = 1; Session->EventOpen = true; break;
    case E::EventWaitExit:
        if (!Session->EventOpen) { ++Session->Stats.AddressMismatch; break; }
        Session->Event.Span.End.WallTicks = Sample.WallTicks; Session->Event.Span.End.Flags = 0; // Original sample has no availability BOOL.
        Session->Event.Span.Returned = 1; Session->Event.Outcome = Sample.Outcome;
        Session->EventOpen = false; Session->CommitAux(Session->Event); break;
    case E::YieldEntry:
        if (Session->YieldOpen) ++Session->Stats.AddressMismatch;
        Session->YieldEntry = Sample.WallTicks; Session->YieldOpen = true; break;
    case E::YieldExit:
        if (!Session->YieldOpen || Sample.WallTicks < Session->YieldEntry) { ++Session->Stats.AddressMismatch; break; }
        Session->YieldOpen = false;
        ++Session->Wait.YieldCalls; Session->Wait.YieldTotalTicks += Sample.WallTicks - Session->YieldEntry;
        Session->Wait.YieldMaxTicks = FMath::Max(Session->Wait.YieldMaxTicks, Sample.WallTicks - Session->YieldEntry);
        break;
    case E::Return:
        Session->Wait.Outcome = Sample.Outcome; Session->Wait.InteriorReturnTicks = Sample.WallTicks;
        ++Session->Wait.InteriorReturnCount; break;
    }
}
bool ParkWait(FEvent& Event, uint32 Milliseconds)
{
    FSession* Session = ActiveSession;
    if (!Session || !Session->Writable()) return Event.Wait(Milliseconds);
    FAuxRecord Row = Session->NewAux(EAuxKind::ParkWait);
    Row.RequestedMilliseconds = Milliseconds; Row.Span.Begin = Session->Clock(); Row.Span.Present = 1;
    const bool Signaled = Event.Wait(Milliseconds); // EXACT original primitive/argument once.
    Row.Span.End = Session->Clock(); Row.Span.Returned = 1; Row.Outcome = Signaled;
    Session->CommitAux(Row); return Signaled;
}

bool ReleaseOwnedHookForTesting(const FSessionPtr& Session)
{
    if (!Session || !Session->Owner() || ActiveSession != Session.Get()) return false;
    ActiveSession = nullptr; return true; // Actual matching TLS operation, not a forced status.
}
void FreezeOnOwner(const FSessionPtr& Session, EFreezeReason Reason)
{
    if (!Session || Session->Frozen.load(std::memory_order_acquire)) return;
    if (!Session->Owner()) { Session->WrongOwner.fetch_add(1, std::memory_order_relaxed); return; }
    if (Session->CurrentStep || Session->ServiceOpen || Session->WorkOpen || Session->WaitOpen || Session->EventOpen || Session->YieldOpen)
    { ++Session->Stats.OpenSpanAtFreeze; return; }
    Session->Recording = false;
    if (ActiveSession != Session.Get()) { ++Session->Stats.HookClearFailed; return; }
    ActiveSession = nullptr; // Disable EVERY Step/camera/service/work/wait/park write before release.
    Session->Stats.WrongOwner = Session->WrongOwner.load(std::memory_order_relaxed);
    Session->Stats.FreezeReason = uint32(Reason); Session->Stats.CloseReached = Reason == EFreezeReason::WorkerClose;
    Session->Stats.FrozenSafe = 1;
    Session->Stats.CompletePrefix = Session->Stats.QpcFrequency && !Session->Stats.DroppedFrames && !Session->Stats.DroppedAux &&
        !Session->Stats.WrongOwner && !Session->Stats.HookAttachFailed && !Session->Stats.HookClearFailed &&
        !Session->Stats.NestedStep && !Session->Stats.AddressMismatch && !Session->Stats.OpenSpanAtFreeze;
    Session->Frozen.store(true, std::memory_order_release);
}
bool CopyFrozenSummary(const FSessionPtr& Session, FSummary& Out)
{
    if (!Session || !Session->Frozen.load(std::memory_order_acquire)) return false;
    Out = Session->Stats;
    Out.ExportClaimed = Session->ExportClaimed.load(std::memory_order_acquire);
    Out.ExportSucceeded = Session->ExportSucceeded.load(std::memory_order_acquire);
    return true;
}
bool ReadFrozenFrameForTests(const FSessionPtr& Session, uint64 Index, FFrameRecord& Out)
{
    if (!Session || !Session->Frozen.load(std::memory_order_acquire) || Index >= Session->Stats.FrameCount) return false;
    Out = Session->Frames[Index]; return true;
}
bool ReadFrozenAuxForTests(const FSessionPtr& Session, uint64 Index, FAuxRecord& Out)
{
    if (!Session || !Session->Frozen.load(std::memory_order_acquire) || Index >= Session->Stats.AuxCount) return false;
    Out = Session->Aux[Index]; return true;
}
bool TryExportFrozen(const FSessionPtr& Session)
{
    if (!Session || !Session->Frozen.load(std::memory_order_acquire) || !PrivatePath(Session->Path)) return false;
    bool Expected = false;
    if (!Session->ExportClaimed.compare_exchange_strong(Expected, true, std::memory_order_acq_rel)) return false;
    // All data is immutable after release. No pause flag or mutable prefix read.
    if (IFileManager::Get().FileExists(*Session->Path)) return false;
    TUniquePtr<FArchive> Writer(IFileManager::Get().CreateFileWriter(*Session->Path));
    if (!Writer) return false;
    FBinaryHeader Header;
    FMemory::Memzero(&Header, sizeof(Header)); // Explicit native-layout padding, not canonical data.
    Header.Magic = 0x314455414E574F53ull; Header.Version = 1;
    Header.FrameBytes = sizeof(FFrameRecord); Header.AuxBytes = sizeof(FAuxRecord);
    Header.HeaderBytes = sizeof(FBinaryHeader);
    Header.FrameCapacity = Session->FrameCapacity; Header.AuxCapacity = Session->AuxCapacity;
    Header.Summary = Session->Stats; Header.Summary.ExportClaimed = 1;
    Writer->Serialize(&Header, sizeof(Header));
    Writer->Serialize(Session->Frames.get(), int64(Session->Stats.FrameCount * sizeof(FFrameRecord)));
    Writer->Serialize(Session->Aux.get(), int64(Session->Stats.AuxCount * sizeof(FAuxRecord)));
    const bool Ok = !Writer->IsError() && Writer->Close();
    if (Ok && Session->CanonicalPhases)
    {
        // Separate optional file. Its presence/completeness must be checked
        // independently: the original audit's success flag does not certify it.
        const FString PhasePath = Session->Path + TEXT(".canonical-phases.bin");
        if (!IFileManager::Get().FileExists(*PhasePath))
        {
            TUniquePtr<FArchive> PhaseWriter(IFileManager::Get().CreateFileWriter(*PhasePath));
            if (PhaseWriter)
            {
                FCanonicalPhaseHeader PhaseHeader;
                FMemory::Memzero(&PhaseHeader, sizeof(PhaseHeader));
                PhaseHeader.Magic = 0x315348504E574F53ull; PhaseHeader.Version = 1;
                PhaseHeader.RecordBytes = sizeof(FCanonicalPhaseRecord);
                PhaseHeader.HeaderBytes = sizeof(FCanonicalPhaseHeader);
                PhaseHeader.EdgeCount = uint32(ECanonicalEdge::Count);
                PhaseHeader.FrameCapacity = Session->FrameCapacity;
                PhaseHeader.Summary = Session->Stats;
                PhaseWriter->Serialize(&PhaseHeader, sizeof(PhaseHeader));
                PhaseWriter->Serialize(Session->CanonicalPhases.get(),
                    int64(Session->Stats.FrameCount * sizeof(FCanonicalPhaseRecord)));
                PhaseWriter->Close(); // No retry; offline exact size/header checks are mandatory.
            }
        }
    }
    if (Ok && Session->PhysicalDetails)
    {
        // Independent sidecar qualification: the old success flag does not certify it.
        const FString DetailPath = Session->Path + TEXT(".physical-detail.bin");
        if (!IFileManager::Get().FileExists(*DetailPath))
        {
            TUniquePtr<FArchive> DetailWriter(IFileManager::Get().CreateFileWriter(*DetailPath));
            if (DetailWriter)
            {
                FPhysicalDetailHeader DetailHeader;
                FMemory::Memzero(&DetailHeader, sizeof(DetailHeader));
                DetailHeader.Magic = 0x315948504E574F53ull; DetailHeader.Version = 1;
                DetailHeader.RecordBytes = sizeof(FPhysicalDetailRecord);
                DetailHeader.HeaderBytes = sizeof(FPhysicalDetailHeader);
                DetailHeader.FrameCapacity = Session->FrameCapacity;
                DetailHeader.Summary = Header.Summary; // exact original serialized summary
                DetailWriter->Serialize(&DetailHeader, sizeof(DetailHeader));
                DetailWriter->Serialize(Session->PhysicalDetails.get(),
                    int64(Session->Stats.FrameCount * sizeof(FPhysicalDetailRecord)));
                DetailWriter->Close(); // no retry or forced success flag
            }
        }
    }
    Session->ExportSucceeded.store(Ok, std::memory_order_release);
    return Ok; // File sizes/checksums/source/DLL envelope are independently authenticated offline.
}
void TryExportForHostOnGameThread(const void* Host)
{
    if (!IsInGameThread()) return;
    if (const FHostCapture* Capture = HostSessions.Find(Host)) TryExportFrozen(Capture->Current);
}
}
#endif

// Append to OwnedFrameAudit.cpp, outside its implementation namespace.
// Diagnostic-only tests of the REAL API. They do not claim native camera/contacts
// fullbytes; the separate current 27000 fixture must exercise actual Step TLS.
#if WITH_DEV_AUTOMATION_TESTS && PLATFORM_WINDOWS
#include "Misc/AutomationTest.h"
#include "Misc/ScopeExit.h"
#include "Misc/Paths.h"
#include "HAL/PlatformTLS.h"
#include "HAL/PlatformTime.h"
#include "HAL/PlatformProcess.h"

namespace Speed::OwnedFrameAudit::NativeAuditTests
{
static constexpr EAutomationTestFlags TestFlags = EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter;
static FString UniqueOutput(const TCHAR* Case)
{
    // ProjectSavedDir is the existing private native output directory. The
    // exporter does not create directories; do not invent a missing subfolder.
    return FPaths::Combine(FPaths::ProjectSavedDir(),
        FString::Printf(TEXT("OwnedFrameAudit-%s-%u-%llu.bin"), Case, FPlatformTLS::GetCurrentThreadId(),
            static_cast<unsigned long long>(FPlatformTime::Cycles64())));
}
static bool SameClock(const FClockEdge& A, const FClockEdge& B)
{
    return A.WallTicks==B.WallTicks && A.ThreadCycles==B.ThreadCycles &&
        A.Kernel100ns==B.Kernel100ns && A.User100ns==B.User100ns && A.Flags==B.Flags;
}
static bool SameSpan(const FSpan& A, const FSpan& B)
{
    return SameClock(A.Begin,B.Begin) && SameClock(A.End,B.End) &&
        A.Present==B.Present && A.Returned==B.Returned;
}
static bool SameFrame(const FFrameRecord& A, const FFrameRecord& B)
{
    return A.ExecutionOrdinal==B.ExecutionOrdinal && A.PulseOrdinal==B.PulseOrdinal &&
        A.ServiceCycle==B.ServiceCycle && A.CanonicalFrame==B.CanonicalFrame &&
        A.CameraFrame==B.CameraFrame && SameSpan(A.Step,B.Step) && SameSpan(A.Camera,B.Camera) &&
        A.CameraCalls==B.CameraCalls && A.Replay==B.Replay && A.StepSucceeded==B.StepSucceeded;
}
struct FOldWaitWitness
{
    uint64 Calls=0;
    uint64 Events[7]{};
    static void Observe(void* Context, const Speed::Testing::FSimulationWaitAuditSample& Sample) noexcept
    {
        auto& Self=*static_cast<FOldWaitWitness*>(Context);
        ++Self.Calls;
        const uint32 Event=uint32(Sample.Event);
        if (Event<UE_ARRAY_COUNT(Self.Events)) ++Self.Events[Event];
    }
};
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSpeedOwnedFrameAuditDisabledTest,
    "IAmSpeed.Simulation.OwnedFrameAuditDisabledZero", Speed::OwnedFrameAudit::NativeAuditTests::TestFlags)
bool FSpeedOwnedFrameAuditDisabledTest::RunTest(const FString&)
{
    using namespace Speed::OwnedFrameAudit;
    // Production admission must return no context before allocating buffers.
    const auto NoSession=CreateForHost(nullptr,false,TEXT(""));
    TestFalse(TEXT("disabled admission creates no session"),NoSession.IsValid());
    TestNull(TEXT("disabled current thread has no recorder"),ActiveForCurrentThread());
    OwnerBoundary(NoSession,true);
    BeginService(NoSession,false,61); EndService(NoSession,ESimulationBoundaryResult::Ready);
    BeginWork(NoSession,61);
    const auto H=BeginStep(61,false);
    TestNull(TEXT("disabled Step has no session"),H.Session);
    TestNull(TEXT("disabled Step reserves no record"),H.Record);
    { FCameraScope Camera(61,true); Camera.MarkReturned(); }
    EndStep(H,true,true); EndWork(NoSession,ESimulationWorkerResult::Advanced);
    BeginWait(NoSession); EndWait(NoSession);
    FreezeOnOwner(NoSession,EFreezeReason::ControlledCutoff);
    FSummary Summary;
    TestFalse(TEXT("disabled path cannot fabricate a capture"),CopyFrozenSummary(NoSession,Summary));
    TestFalse(TEXT("disabled path cannot export an old result"),TryExportFrozen(NoSession));
    TestNull(TEXT("disabled operations install no TLS"),ActiveForCurrentThread());
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSpeedOwnedFrameAuditOrdinalTest,
    "IAmSpeed.Simulation.OwnedFrameAuditOwnerAndFrameOrdinal", Speed::OwnedFrameAudit::NativeAuditTests::TestFlags)
bool FSpeedOwnedFrameAuditOrdinalTest::RunTest(const FString&)
{
    using namespace Speed::OwnedFrameAudit;
    using namespace Speed::OwnedFrameAudit::NativeAuditTests;
    const auto Session=CreateForTests(16,128,FString(),true);
    if (!TestTrue(TEXT("real diagnostic allocates test bounds before Start"),Session.IsValid())) return false;
    FEvent* Closed=FPlatformProcess::GetSynchEventFromPool(true);
    FOldWaitWitness Old;
    bool OldInstalled=false,OldCleared=false;
    uint32 Owner=0;
    int32 Pulse=0;
    constexpr uint64 Logical[5]={61,62,62,60,61};
    FSimulationWorker Worker([&]()
    {
        const uint64 Candidate=Pulse==0?61:60;
        BeginWork(Session,Candidate);
        if (Pulse>=2) { EndWork(Session,ESimulationWorkerResult::Complete); return ESimulationWorkerResult::Complete; }
        const int32 First=Pulse==0?0:3;
        const int32 End=Pulse==0?3:5;
        for (int32 I=First;I<End;++I)
        {
            const auto H=BeginStep(Logical[I],I<3);
            { FCameraScope Camera(Logical[I],true); Camera.MarkReturned(); }
            EndStep(H,true,true);
        }
        ++Pulse;
        EndWork(Session,ESimulationWorkerResult::Advanced);
        return ESimulationWorkerResult::Advanced;
    },[&](FSimulationWorkerWaitContext& Wait)
    {
        BeginWait(Session);
        (void)Wait.WaitUntil(FPlatformTime::Seconds()+.001,.0001);
        EndWait(Session);
    },[&](bool Paused)
    {
        if (!Owner)
        {
            Owner=FPlatformTLS::GetCurrentThreadId();
            OldInstalled=Speed::Testing::SetSimulationWaitAuditObserverForCurrentThread(&FOldWaitWitness::Observe,&Old);
        }
        OwnerBoundary(Session,true);
        BeginService(Session,Paused,Pulse==0?61:60);
        EndService(Session,ESimulationBoundaryResult::Ready);
        return ESimulationBoundaryResult::Ready;
    },[&]()
    {
        FreezeOnOwner(Session,EFreezeReason::WorkerClose);
        // Owned clear must have left the old observer in its original slot.
        OldCleared=Speed::Testing::ClearSimulationWaitAuditObserverForCurrentThread(&FOldWaitWitness::Observe,&Old);
        Closed->Trigger();
    });
    ON_SCOPE_EXIT { Worker.StopAndJoin(); FPlatformProcess::ReturnSynchEventToPool(Closed); };
    if (!TestTrue(TEXT("actual worker starts"),Worker.Start())) return false;
    if (!TestTrue(TEXT("owner reaches Close"),Closed->Wait(1000))) return false;
    Worker.StopAndJoin();
    FSummary Summary;
    if (!TestTrue(TEXT("owner release freeze is acquired"),CopyFrozenSummary(Session,Summary))) return false;
    TestEqual(TEXT("all Steps survive multiple-Step pulses"),Summary.FrameCount,uint64(5));
    TestEqual(TEXT("owner is actual worker"),Summary.OwnerThreadId,Owner);
    TestTrue(TEXT("old observer installed and survives Owned clear"),OldInstalled && OldCleared);
    TestTrue(TEXT("old observer receives actual Wait events"),Old.Calls>0 && Old.Events[uint32(Speed::Testing::ESimulationWaitAuditEvent::Return)]==2);
    TestEqual(TEXT("no addresses lost or mismatched"),Summary.AddressMismatch,uint32(0));
    TestEqual(TEXT("no nested Step"),Summary.NestedStep,uint32(0));
    TestEqual(TEXT("whole captured prefix sealed"),Summary.CompletePrefix,uint32(1));
    FFrameRecord Prior;
    for (uint64 I=0;I<5;++I)
    {
        FFrameRecord Row;
        if (!TestTrue(TEXT("frozen row readable"),ReadFrozenFrameForTests(Session,I,Row))) return false;
        TestEqual(TEXT("actual logical frame preserved"),Row.CanonicalFrame,Logical[I]);
        TestEqual(TEXT("camera frame linked to same Step"),Row.CameraFrame,Logical[I]);
        TestEqual(TEXT("one camera bracket"),Row.CameraCalls,uint32(1));
        TestEqual(TEXT("Step actually returned"),Row.Step.Returned,uint32(1));
        TestEqual(TEXT("Step marked successful"),Row.StepSucceeded,uint32(1));
        if (I) TestEqual(TEXT("execution ordinal increments despite logical rollback"),Row.ExecutionOrdinal,Prior.ExecutionOrdinal+1);
        if (I==1 || I==2 || I==4) TestEqual(TEXT("Steps in same Work share pulse ordinal"),Row.PulseOrdinal,Prior.PulseOrdinal);
        if (I==3) TestTrue(TEXT("next pulse has distinct ordinal"),Row.PulseOrdinal>Prior.PulseOrdinal);
        Prior=Row;
    }
    // Actual matching hook-loss negative, never a manufactured failure bit.
    const auto HookLoss=CreateForTests(4,8,FString(),false);
    FEvent* HookClosed=FPlatformProcess::GetSynchEventFromPool(true);
    bool Released=false;
    FSimulationWorker HookWorker([]() { return ESimulationWorkerResult::Complete; },
        [](FSimulationWorkerWaitContext&) {}, [&](bool)
        {
            OwnerBoundary(HookLoss,true);
            Released=ReleaseOwnedHookForTesting(HookLoss);
            return ESimulationBoundaryResult::Ready;
        },[&]() { FreezeOnOwner(HookLoss,EFreezeReason::WorkerClose); HookClosed->Trigger(); });
    ON_SCOPE_EXIT { HookWorker.StopAndJoin(); FPlatformProcess::ReturnSynchEventToPool(HookClosed); };
    if (!TestTrue(TEXT("hook-loss worker starts"),HookWorker.Start())) return false;
    if (!TestTrue(TEXT("hook-loss Close reached"),HookClosed->Wait(1000))) return false;
    HookWorker.StopAndJoin();
    TestTrue(TEXT("matching Owned hook really released"),Released);
    FSummary Unsafe;
    TestFalse(TEXT("clear failure never becomes readable Frozen"),CopyFrozenSummary(HookLoss,Unsafe));
    TestFalse(TEXT("clear failure cannot export complete"),TryExportFrozen(HookLoss));

    // Wrong-thread operation after a REAL owner has attached. It may flag a
    // diagnostic error but must not reserve a Step or overwrite owner storage.
    const auto WrongLane=CreateForTests(4,8,FString(),false);
    FEvent* OwnerEntered=FPlatformProcess::GetSynchEventFromPool(true);
    FEvent* OwnerRelease=FPlatformProcess::GetSynchEventFromPool(true);
    FEvent* WrongClosed=FPlatformProcess::GetSynchEventFromPool(true);
    FSimulationWorker WrongWorker([&]()
        {
            BeginWork(WrongLane,91);
            const auto Handle=BeginStep(91,false); EndStep(Handle,true,true);
            EndWork(WrongLane,ESimulationWorkerResult::Complete);
            return ESimulationWorkerResult::Complete;
        },[](FSimulationWorkerWaitContext&) {},[&](bool)
        {
            OwnerBoundary(WrongLane,true); OwnerEntered->Trigger();
            (void)OwnerRelease->Wait(5000);
            return ESimulationBoundaryResult::Ready;
        },[&]() { FreezeOnOwner(WrongLane,EFreezeReason::WorkerClose); WrongClosed->Trigger(); });
    ON_SCOPE_EXIT
    {
        OwnerRelease->Trigger(); WrongWorker.StopAndJoin();
        FPlatformProcess::ReturnSynchEventToPool(OwnerEntered);
        FPlatformProcess::ReturnSynchEventToPool(OwnerRelease);
        FPlatformProcess::ReturnSynchEventToPool(WrongClosed);
    };
    if (!TestTrue(TEXT("wrong-lane witness worker starts"),WrongWorker.Start())) return false;
    if (!TestTrue(TEXT("actual owner attached before GT negative"),OwnerEntered->Wait(1000))) return false;
    BeginWork(WrongLane,999); // GT: actual owner mismatch, no write to Work POD.
    TestNull(TEXT("GT never inherits owner TLS"),ActiveForCurrentThread());
    OwnerRelease->Trigger();
    if (!TestTrue(TEXT("wrong-lane witness closes safely"),WrongClosed->Wait(1000))) return false;
    WrongWorker.StopAndJoin();
    FSummary Rejected;
    if (!TestTrue(TEXT("owner can seal safe incomplete diagnostic"),CopyFrozenSummary(WrongLane,Rejected))) return false;
    TestTrue(TEXT("actual wrong-owner counter retained"),Rejected.WrongOwner>0);
    TestEqual(TEXT("wrong-owner attempt did not fabricate frame"),Rejected.FrameCount,uint64(1));
    TestEqual(TEXT("wrong-owner scope not qualified complete"),Rejected.CompletePrefix,uint32(0));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSpeedOwnedFrameAuditFreezeTest,
    "IAmSpeed.Simulation.OwnedFrameAuditFreezeBeforeClose", Speed::OwnedFrameAudit::NativeAuditTests::TestFlags)
bool FSpeedOwnedFrameAuditFreezeTest::RunTest(const FString&)
{
    using namespace Speed::OwnedFrameAudit;
    using namespace Speed::OwnedFrameAudit::NativeAuditTests;
    const auto Session=CreateForTests(8,64,UniqueOutput(TEXT("FreezeBeforeClose")),true);
    if (!TestTrue(TEXT("bounded capture prepared"),Session.IsValid())) return false;
    FEvent* CloseEntered=FPlatformProcess::GetSynchEventFromPool(true);
    FEvent* CloseRelease=FPlatformProcess::GetSynchEventFromPool(true);
    int32 Frames=0;
    bool FrozenThenContinued=false;
    FSimulationWorker Worker([&]()
    {
        if (Frames>=3) return ESimulationWorkerResult::Complete;
        BeginWork(Session,uint64(61+Frames));
        const auto H=BeginStep(uint64(61+Frames),false);
        { FCameraScope Camera(uint64(61+Frames),true); Camera.MarkReturned(); }
        EndStep(H,true,true); ++Frames;
        EndWork(Session,ESimulationWorkerResult::Advanced);
        return ESimulationWorkerResult::Advanced;
    },[&](FSimulationWorkerWaitContext& Wait)
    {
        BeginWait(Session); (void)Wait.WaitUntil(FPlatformTime::Seconds()+.001,.0001); EndWait(Session);
    },[&](bool Paused)
    {
        OwnerBoundary(Session,Frames<3); // Corresponds exactly to CVar0 at a boundary.
        BeginService(Session,Paused,uint64(61+Frames));
        EndService(Session,ESimulationBoundaryResult::Ready);
        if (Frames>=3) FrozenThenContinued=ActiveForCurrentThread()==nullptr;
        return ESimulationBoundaryResult::Ready;
    },[&]()
    {
        FreezeOnOwner(Session,EFreezeReason::WorkerClose);
        CloseEntered->Trigger();
        (void)CloseRelease->Wait(5000); // Fault injection only, never a player timeout.
    });
    ON_SCOPE_EXIT
    {
        CloseRelease->Trigger(); Worker.StopAndJoin();
        FPlatformProcess::ReturnSynchEventToPool(CloseEntered);
        FPlatformProcess::ReturnSynchEventToPool(CloseRelease);
    };
    if (!TestTrue(TEXT("actual worker starts"),Worker.Start())) return false;
    if (!TestTrue(TEXT("Close is deliberately blocked"),CloseEntered->Wait(1000))) return false;
    FSummary Summary;
    if (!TestTrue(TEXT("GT acquire observes frozen prefix before join"),CopyFrozenSummary(Session,Summary))) return false;
    TestEqual(TEXT("three complete frames retained"),Summary.FrameCount,uint64(3));
    TestEqual(TEXT("full prefix is safe"),Summary.FrozenSafe,uint32(1));
    TestEqual(TEXT("no open phase at freeze"),Summary.OpenSpanAtFreeze,uint32(0));
    TestTrue(TEXT("ordinary service continues without recorder writes"),FrozenThenContinued);
    FFrameRecord Before,After;
    TestTrue(TEXT("GT reads sealed last frame"),ReadFrozenFrameForTests(Session,2,Before));
    TestTrue(TEXT("export succeeds while Close is blocked"),TryExportFrozen(Session));
    TestTrue(TEXT("GT rereads identical sealed last frame"),ReadFrozenFrameForTests(Session,2,After));
    TestTrue(TEXT("immutable row after export"),SameFrame(Before,After));
    FSummary Exported;
    TestTrue(TEXT("export summary remains accessible"),CopyFrozenSummary(Session,Exported));
    TestEqual(TEXT("single export claim"),Exported.ExportClaimed,uint32(1));
    TestEqual(TEXT("file export succeeds"),Exported.ExportSucceeded,uint32(1));
    // Second try may return already-completed success; it must never write again.
    (void)TryExportFrozen(Session);
    FSummary Again;
    TestTrue(TEXT("second attempt preserves summary"),CopyFrozenSummary(Session,Again));
    TestEqual(TEXT("no duplicate export claim"),Again.ExportClaimed,Exported.ExportClaimed);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSpeedOwnedFrameAuditOverflowTest,
    "IAmSpeed.Simulation.OwnedFrameAuditOverflowAndPartialExport", Speed::OwnedFrameAudit::NativeAuditTests::TestFlags)
bool FSpeedOwnedFrameAuditOverflowTest::RunTest(const FString&)
{
    using namespace Speed::OwnedFrameAudit;
    const auto Session=CreateForTests(2,3,FString(),true);
    if (!TestTrue(TEXT("tiny actual bounded recorder created"),Session.IsValid())) return false;
    FEvent* Closed=FPlatformProcess::GetSynchEventFromPool(true);
    int32 Frames=0;
    FSimulationWorker Worker([&]()
    {
        if (Frames==8) return ESimulationWorkerResult::Complete;
        BeginWork(Session,uint64(61+Frames));
        const auto H=BeginStep(uint64(61+Frames),false);
        { FCameraScope Camera(uint64(61+Frames),true); Camera.MarkReturned(); }
        EndStep(H,true,true); ++Frames;
        EndWork(Session,ESimulationWorkerResult::Advanced);
        return ESimulationWorkerResult::Advanced;
    },[&](FSimulationWorkerWaitContext& Wait)
    {
        BeginWait(Session); (void)Wait.WaitUntil(FPlatformTime::Seconds()+.001,.0001); EndWait(Session);
    },[&](bool Paused)
    {
        OwnerBoundary(Session,true);
        BeginService(Session,Paused,uint64(61+Frames));
        EndService(Session,ESimulationBoundaryResult::Ready);
        return ESimulationBoundaryResult::Ready;
    },[&]() { FreezeOnOwner(Session,EFreezeReason::WorkerClose); Closed->Trigger(); });
    ON_SCOPE_EXIT { Worker.StopAndJoin(); FPlatformProcess::ReturnSynchEventToPool(Closed); };
    if (!TestTrue(TEXT("actual worker starts"),Worker.Start())) return false;
    if (!TestTrue(TEXT("overflow never prevents Close"),Closed->Wait(1000))) return false;
    Worker.StopAndJoin();
    FSummary Summary;
    if (!TestTrue(TEXT("safe partial prefix still sealed"),CopyFrozenSummary(Session,Summary))) return false;
    TestEqual(TEXT("all eight physical callbacks continue"),Frames,8);
    TestTrue(TEXT("no frame reallocation beyond declared capacity"),Summary.FrameCount<=2);
    TestTrue(TEXT("no aux reallocation beyond declared capacity"),Summary.AuxCount<=3);
    TestTrue(TEXT("explicit dropped records retained"),Summary.DroppedFrames+Summary.DroppedAux>0);
    TestEqual(TEXT("overflow cannot qualify complete"),Summary.CompletePrefix,uint32(0));
    TestFalse(TEXT("empty output cannot fabricate file success"),TryExportFrozen(Session));
    return true;
}
#endif

#if WITH_DEV_AUTOMATION_TESTS && PLATFORM_WINDOWS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSpeedOwnedCanonicalPhaseDiagnosticTest,
    "IAmSpeed.Simulation.OwnedCanonicalPhaseSidecar", Speed::OwnedFrameAudit::NativeAuditTests::TestFlags)
bool FSpeedOwnedCanonicalPhaseDiagnosticTest::RunTest(const FString&)
{
    using namespace Speed::OwnedFrameAudit;
    using namespace Speed::OwnedFrameAudit::NativeAuditTests;
    { FCanonicalPhaseScope Disabled(61); Disabled.Mark(ECanonicalEdge::WorldStep); }
    TestNull(TEXT("disabled phases install no TLS"), ActiveForCurrentThread());
    const FString Path = UniqueOutput(TEXT("CanonicalPhases"));
    const auto Session = CreateForTests(4,16,Path,false,true);
    if (!TestTrue(TEXT("small phase arrays allocate on GT"),Session.IsValid())) return false;
    FCanonicalPhaseRecord Unfrozen;
    TestFalse(TEXT("live records cannot be read"),ReadFrozenCanonicalPhaseForTests(Session,0,Unfrozen));
    FEvent* Closed=FPlatformProcess::GetSynchEventFromPool(true);
    bool Emitted=false;
    FSimulationWorker Worker([&]()
    {
        BeginWork(Session,61);
        if (Emitted) { EndWork(Session,ESimulationWorkerResult::Complete); return ESimulationWorkerResult::Complete; }
        for (uint64 I=0;I<2;++I)
        {
            const uint64 Frame=I?60:61;
            const auto H=BeginStep(Frame,I!=0);
            {
                FCanonicalPhaseScope Phase(Frame);
                Phase.Mark(ECanonicalEdge::Admission);
                Phase.Mark(ECanonicalEdge::WorldStep);
                if (I) Phase.Mark(ECanonicalEdge::WorldStep); // Real duplicate, never silently overwritten.
            }
            EndStep(H,true,true);
        }
        Emitted=true;EndWork(Session,ESimulationWorkerResult::Advanced);
        return ESimulationWorkerResult::Advanced;
    },[](FSimulationWorkerWaitContext&) {},[&](bool Paused)
    {
        OwnerBoundary(Session,true);BeginService(Session,Paused,61);
        EndService(Session,ESimulationBoundaryResult::Ready);return ESimulationBoundaryResult::Ready;
    },[&]() { FreezeOnOwner(Session,EFreezeReason::WorkerClose);Closed->Trigger(); });
    ON_SCOPE_EXIT { Worker.StopAndJoin();FPlatformProcess::ReturnSynchEventToPool(Closed); };
    if (!TestTrue(TEXT("actual phase-test owner starts"),Worker.Start())) return false;
    if (!TestTrue(TEXT("actual phase-test owner closes"),Closed->Wait(1000))) return false;
    Worker.StopAndJoin();
    FSummary Summary;TestTrue(TEXT("summary frozen"),CopyFrozenSummary(Session,Summary));
    TestEqual(TEXT("two original frame records"),Summary.FrameCount,uint64(2));
    for (uint64 I=0;I<2;++I)
    {
        FCanonicalPhaseRecord Phases;FFrameRecord Frame;
        if (!TestTrue(TEXT("actual phase row readable"),ReadFrozenCanonicalPhaseForTests(Session,I,Phases))) return false;
        TestTrue(TEXT("original row readable"),ReadFrozenFrameForTests(Session,I,Frame));
        TestEqual(TEXT("execution ordinal joins exact row"),Phases.ExecutionOrdinal,Frame.ExecutionOrdinal);
        TestEqual(TEXT("logical rollback frame preserved"),Phases.CanonicalFrame,Frame.CanonicalFrame);
        const uint64 Mask=(1ull<<uint32(ECanonicalEdge::Entry))|(1ull<<uint32(ECanonicalEdge::Admission))|
            (1ull<<uint32(ECanonicalEdge::WorldStep))|(1ull<<uint32(ECanonicalEdge::Exit));
        TestEqual(TEXT("only actual edges present"),Phases.PresentMask,Mask);
        TestEqual(TEXT("clock reads valid"),Phases.InvalidClocks,uint32(0));
        TestEqual(TEXT("duplicate captured as invalid evidence"),Phases.DuplicateEdges,uint32(I));
        uint64 Prior=Frame.Step.Begin.WallTicks;
        for (uint32 E=0;E<uint32(ECanonicalEdge::Count);++E)
            if (Mask&(1ull<<E)) { TestTrue(TEXT("edges ordered in original Step"),Phases.WallTicks[E]>=Prior && Phases.WallTicks[E]<=Frame.Step.End.WallTicks);Prior=Phases.WallTicks[E]; }
    }
    TestTrue(TEXT("original immutable binary exported"),TryExportFrozen(Session));
    TestEqual(TEXT("separate phase sidecar exact size"),IFileManager::Get().FileSize(*(Path+TEXT(".canonical-phases.bin"))),int64(208+2*208));
    TestEqual(TEXT("old binary exact ABI unchanged"),IFileManager::Get().FileSize(*Path),int64(216+Summary.FrameCount*232+Summary.AuxCount*224));
    return true;
}
#endif

// Contract-only synthetic diagnostics through the REAL owned recorder API.
// This harness is not an observed physical Step, rendered trace or provenance proof.
#if WITH_DEV_AUTOMATION_TESTS && PLATFORM_WINDOWS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSpeedOwnedPhysicalDetailContractTest,
    "IAmSpeed.Simulation.OwnedPhysicalDetailSidecarContract", Speed::OwnedFrameAudit::NativeAuditTests::TestFlags)
bool FSpeedOwnedPhysicalDetailContractTest::RunTest(const FString&)
{
    using namespace Speed::OwnedFrameAudit;
    using namespace Speed::OwnedFrameAudit::NativeAuditTests;
    TestFalse(TEXT("detail cannot be admitted without canonical phases"),
        CreateForTests(1,8,FString(),false,false,true).IsValid());
    { FCanonicalPhaseScope Disabled(61);
      TestFalse(TEXT("disabled scope has no detail destination"),Disabled.CapturesPhysicalDetail()); }
    const FString Path=UniqueOutput(TEXT("PhysicalDetailContract"));
    const auto Session=CreateForTests(5,32,Path,false,true,true);
    if (!TestTrue(TEXT("bounded detail allocated on GT"),Session.IsValid())) return false;
    FPhysicalDetailRecord Live;
    TestFalse(TEXT("mutable detail cannot be read"),ReadFrozenPhysicalDetailForTests(Session,0,Live));
    FEvent* Closed=FPlatformProcess::GetSynchEventFromPool(true);
    bool Emitted=false,AllScopesCaptured=true;
    FSimulationWorker Worker([&]()
    {
        BeginWork(Session,61);
        if (Emitted) { EndWork(Session,ESimulationWorkerResult::Complete);return ESimulationWorkerResult::Complete; }
        for (uint64 I=0;I<5;++I)
        {
            const uint64 Frame=61+I,Before=100+I;
            const auto H=BeginStep(Frame,false);
            {
                FCanonicalPhaseScope Phase(Frame);
                AllScopesCaptured=AllScopesCaptured && Phase.CapturesPhysicalDetail();
                FSpeedStepDiagnostics D;
                D.Serial=Before+1;D.Frame=uint32(Frame);D.MaximumIterationCount=24;
                D.IterationCount=1;D.ComponentSweepCount=2;D.ResolvedEventCount=3;
                D.StaticQueryCount=4;D.LegacySweepCount=5;D.AuthorityAttemptCount=6;
                D.PhysicalDeltaTimeMilliseconds=3.25;D.TotalMilliseconds=1.25;
                if (I==1) D.Serial=Before; // stale source, retained as invalid
                if (I==2) ++D.Frame;      // wrong canonical address, retained
                if (I==4) { D.IterationCount=-1;D.TotalMilliseconds=-2.0; } // raw invalid payload
                Phase.Mark(ECanonicalEdge::WorldStep);
                Phase.CopyPhysicalDetail(D,Before);
                if (I==3) { D.TotalMilliseconds=999.0;Phase.CopyPhysicalDetail(D,Before); } // duplicate never overwrites
            }
            EndStep(H,true,true);
        }
        Emitted=true;EndWork(Session,ESimulationWorkerResult::Advanced);return ESimulationWorkerResult::Advanced;
    },[](FSimulationWorkerWaitContext&) {},[&](bool Paused)
    {
        OwnerBoundary(Session,true);BeginService(Session,Paused,61);
        EndService(Session,ESimulationBoundaryResult::Ready);return ESimulationBoundaryResult::Ready;
    },[&]() { FreezeOnOwner(Session,EFreezeReason::WorkerClose);Closed->Trigger(); });
    ON_SCOPE_EXIT { Worker.StopAndJoin();FPlatformProcess::ReturnSynchEventToPool(Closed); };
    if (!TestTrue(TEXT("real contract worker starts"),Worker.Start())) return false;
    if (!TestTrue(TEXT("contract worker closes"),Closed->Wait(1000))) return false;
    Worker.StopAndJoin();TestTrue(TEXT("all actual scopes captured"),AllScopesCaptured);
    FSummary Summary;TestTrue(TEXT("summary really frozen"),CopyFrozenSummary(Session,Summary));
    TestEqual(TEXT("five original records retained"),Summary.FrameCount,uint64(5));
    const uint32 ExpectedMask[5]={31,27,23,31,15};
    for (uint64 I=0;I<5;++I)
    {
        FPhysicalDetailRecord D;FFrameRecord F;
        if (!TestTrue(TEXT("frozen detail readable"),ReadFrozenPhysicalDetailForTests(Session,I,D))) return false;
        TestTrue(TEXT("original record readable"),ReadFrozenFrameForTests(Session,I,F));
        TestEqual(TEXT("exact execution join"),D.ExecutionOrdinal,F.ExecutionOrdinal);
        TestEqual(TEXT("exact frame join"),D.CanonicalFrame,F.CanonicalFrame);
        TestEqual(TEXT("before serial retained independently"),D.SerialBeforeStep,uint64(100+I));
        TestEqual(TEXT("freshness/mismatch flags remain explicit"),D.ValidMask,ExpectedMask[I]);
        TestEqual(TEXT("duplicate count retained"),D.CaptureCalls,uint32(I==3?2:1));
        TestEqual(TEXT("reserved bytes stay zero"),D.Reserved,uint32(0));
        if (I==3) TestEqual(TEXT("duplicate preserves first payload"),D.TotalMilliseconds,1.25);
        if (I==4) { TestEqual(TEXT("negative count is never clamped"),D.IterationCount,int32(-1));
                    TestEqual(TEXT("negative elapsed is never clamped"),D.TotalMilliseconds,-2.0); }
    }
    TestFalse(TEXT("past-count detail cannot be read"),ReadFrozenPhysicalDetailForTests(Session,5,Live));
    TestTrue(TEXT("original immutable binary exports"),TryExportFrozen(Session));
    TestEqual(TEXT("detail sidecar exact size"),IFileManager::Get().FileSize(*(Path+TEXT(".physical-detail.bin"))),int64(208+5*160));
    TestEqual(TEXT("phase sidecar ABI unchanged"),IFileManager::Get().FileSize(*(Path+TEXT(".canonical-phases.bin"))),int64(208+5*208));
    TestEqual(TEXT("original ABI unchanged"),IFileManager::Get().FileSize(*Path),int64(216+5*232+Summary.AuxCount*224));
    // Existing phase summary has claim0; the original header overlays claim1.
    // The new physical summary must copy that original serialized header exactly.
    uint8 OriginalSummary[176]{},PhaseSummary[176]{},PhysicalSummary[176]{};
    TUniquePtr<FArchive> OriginalReader(IFileManager::Get().CreateFileReader(*Path));
    TUniquePtr<FArchive> PhaseReader(IFileManager::Get().CreateFileReader(*(Path+TEXT(".canonical-phases.bin"))));
    TUniquePtr<FArchive> PhysicalReader(IFileManager::Get().CreateFileReader(*(Path+TEXT(".physical-detail.bin"))));
    if (!TestTrue(TEXT("all three actual exports readable"),OriginalReader.IsValid() && PhaseReader.IsValid() && PhysicalReader.IsValid())) return false;
    OriginalReader->Seek(40);OriginalReader->Serialize(OriginalSummary,176);
    PhaseReader->Seek(32);PhaseReader->Serialize(PhaseSummary,176);
    PhysicalReader->Seek(32);PhysicalReader->Serialize(PhysicalSummary,176);
    TestFalse(TEXT("headers not truncated"),OriginalReader->IsError() || PhaseReader->IsError() || PhysicalReader->IsError());
    TestEqual(TEXT("physical summary exact original bytes"),FMemory::Memcmp(OriginalSummary,PhysicalSummary,176),int32(0));
    TestEqual(TEXT("existing phase immutable summary fields equal"),FMemory::Memcmp(OriginalSummary,PhaseSummary,168),int32(0));
    uint32 OriginalExport[2]{},PhaseExport[2]{};
    FMemory::Memcpy(OriginalExport,OriginalSummary+168,8);FMemory::Memcpy(PhaseExport,PhaseSummary+168,8);
    TestEqual(TEXT("original serialized claim overlay"),OriginalExport[0],uint32(1));
    TestEqual(TEXT("original serialized API return remains unobservable"),OriginalExport[1],uint32(0));
    TestEqual(TEXT("old phase serialized claim is unchanged"),PhaseExport[0],uint32(0));
    TestEqual(TEXT("old phase serialized success is unchanged"),PhaseExport[1],uint32(0));
    return true;
}
#endif

// Additional recorder contract negatives. Synthetic values, real owner worker.
#if WITH_DEV_AUTOMATION_TESTS && PLATFORM_WINDOWS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSpeedOwnedPhysicalDetailOverflowTest,
    "IAmSpeed.Simulation.OwnedPhysicalDetailOverflow", Speed::OwnedFrameAudit::NativeAuditTests::TestFlags)
bool FSpeedOwnedPhysicalDetailOverflowTest::RunTest(const FString&)
{
    using namespace Speed::OwnedFrameAudit;
    using namespace Speed::OwnedFrameAudit::NativeAuditTests;
    const FString Path=UniqueOutput(TEXT("PhysicalDetailOverflow"));
    const auto Session=CreateForTests(2,32,Path,false,true,true);
    if (!TestTrue(TEXT("tiny physical detail recorder allocated"),Session.IsValid())) return false;
    FEvent* Closed=FPlatformProcess::GetSynchEventFromPool(true);
    bool Emitted=false;int32 PhysicalCallbacks=0,Captured=0;
    FSimulationWorker Worker([&]()
    {
        BeginWork(Session,61);
        if (Emitted) { EndWork(Session,ESimulationWorkerResult::Complete);return ESimulationWorkerResult::Complete; }
        for (uint64 I=0;I<8;++I)
        {
            const auto H=BeginStep(61+I,false);
            {
                FCanonicalPhaseScope Phase(61+I);
                Captured+=Phase.CapturesPhysicalDetail()?1:0;
                FSpeedStepDiagnostics D;D.Serial=1001+I;D.Frame=uint32(61+I);
                D.MaximumIterationCount=24;D.TotalMilliseconds=double(I+1);
                Phase.Mark(ECanonicalEdge::WorldStep);Phase.CopyPhysicalDetail(D,1000+I);
            }
            EndStep(H,true,true);++PhysicalCallbacks;
        }
        Emitted=true;EndWork(Session,ESimulationWorkerResult::Advanced);return ESimulationWorkerResult::Advanced;
    },[](FSimulationWorkerWaitContext&) {},[&](bool Paused)
    {
        OwnerBoundary(Session,true);BeginService(Session,Paused,61);
        EndService(Session,ESimulationBoundaryResult::Ready);return ESimulationBoundaryResult::Ready;
    },[&]() { FreezeOnOwner(Session,EFreezeReason::WorkerClose);Closed->Trigger(); });
    ON_SCOPE_EXIT { Worker.StopAndJoin();FPlatformProcess::ReturnSynchEventToPool(Closed); };
    if (!TestTrue(TEXT("overflow worker starts"),Worker.Start())) return false;
    if (!TestTrue(TEXT("overflow worker closes"),Closed->Wait(1000))) return false;
    Worker.StopAndJoin();FSummary S;
    if (!TestTrue(TEXT("partial prefix actually frozen"),CopyFrozenSummary(Session,S))) return false;
    TestEqual(TEXT("all physical callbacks still ran"),PhysicalCallbacks,8);
    TestEqual(TEXT("only declared detail capacity captured"),Captured,2);
    TestEqual(TEXT("stored frame count bounded"),S.FrameCount,uint64(2));
    TestEqual(TEXT("six dropped attempts remain visible"),S.DroppedFrames,uint64(6));
    TestEqual(TEXT("overflow does not qualify complete"),S.CompletePrefix,uint32(0));
    for (uint64 I=0;I<2;++I)
    {
        FPhysicalDetailRecord D;
        if (!TestTrue(TEXT("retained physical prefix readable"),ReadFrozenPhysicalDetailForTests(Session,I,D))) return false;
        TestEqual(TEXT("overflow never overwrites execution"),D.ExecutionOrdinal,I+1);
        TestEqual(TEXT("overflow never overwrites serial"),D.DiagnosticSerial,1001+I);
        TestEqual(TEXT("first copied payload retained"),D.TotalMilliseconds,double(I+1));
        TestEqual(TEXT("bounded retained payload captured once"),D.CaptureCalls,uint32(1));
    }
    FPhysicalDetailRecord Past;TestFalse(TEXT("no read beyond capacity"),ReadFrozenPhysicalDetailForTests(Session,2,Past));
    TestTrue(TEXT("safe partial original export remains available"),TryExportFrozen(Session));
    TestEqual(TEXT("partial physical file contains only two records"),IFileManager::Get().FileSize(*(Path+TEXT(".physical-detail.bin"))),int64(208+2*160));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSpeedOwnedPhysicalDetailWrongOwnerTest,
    "IAmSpeed.Simulation.OwnedPhysicalDetailWrongOwner", Speed::OwnedFrameAudit::NativeAuditTests::TestFlags)
bool FSpeedOwnedPhysicalDetailWrongOwnerTest::RunTest(const FString&)
{
    using namespace Speed::OwnedFrameAudit;
    using namespace Speed::OwnedFrameAudit::NativeAuditTests;
    const auto Session=CreateForTests(2,16,FString(),false,true,true);
    if (!TestTrue(TEXT("owner test recorder allocated"),Session.IsValid())) return false;
    FEvent* Ready=FPlatformProcess::GetSynchEventFromPool(true);
    FEvent* Release=FPlatformProcess::GetSynchEventFromPool(true);
    FEvent* Closed=FPlatformProcess::GetSynchEventFromPool(true);
    std::atomic<FCanonicalPhaseScope*> PublishedScope{nullptr};bool Emitted=false;
    FSimulationWorker Worker([&]()
    {
        BeginWork(Session,61);
        if (Emitted) { EndWork(Session,ESimulationWorkerResult::Complete);return ESimulationWorkerResult::Complete; }
        const auto H=BeginStep(61,false);
        {
            FCanonicalPhaseScope Phase(61);
            PublishedScope.store(&Phase,std::memory_order_release);Ready->Trigger();
            // Release is triggered on every GT exit path before StopAndJoin.
            // Keep the stack scope alive until the GT API call has returned.
            (void)Release->Wait();
            FSpeedStepDiagnostics D;D.Serial=101;D.Frame=61;D.MaximumIterationCount=24;D.TotalMilliseconds=1.25;
            Phase.Mark(ECanonicalEdge::WorldStep);Phase.CopyPhysicalDetail(D,100);
            PublishedScope.store(nullptr,std::memory_order_release);
        }
        EndStep(H,true,true);Emitted=true;
        EndWork(Session,ESimulationWorkerResult::Advanced);return ESimulationWorkerResult::Advanced;
    },[](FSimulationWorkerWaitContext&) {},[&](bool Paused)
    {
        OwnerBoundary(Session,true);BeginService(Session,Paused,61);
        EndService(Session,ESimulationBoundaryResult::Ready);return ESimulationBoundaryResult::Ready;
    },[&]() { FreezeOnOwner(Session,EFreezeReason::WorkerClose);Closed->Trigger(); });
    ON_SCOPE_EXIT
    {
        Release->Trigger();Worker.StopAndJoin();
        FPlatformProcess::ReturnSynchEventToPool(Ready);FPlatformProcess::ReturnSynchEventToPool(Release);FPlatformProcess::ReturnSynchEventToPool(Closed);
    };
    if (!TestTrue(TEXT("physical owner worker starts"),Worker.Start())) return false;
    if (!TestTrue(TEXT("actual scope remains alive for negative"),Ready->Wait(1000))) return false;
    auto* Scope=PublishedScope.load(std::memory_order_acquire);
    if (!TestTrue(TEXT("actual worker scope published"),Scope!=nullptr)) return false;
    FSpeedStepDiagnostics Bad;Bad.Serial=999;Bad.Frame=999;Bad.TotalMilliseconds=999.0;
    Scope->CopyPhysicalDetail(Bad,998); // GT calls the real scope API; owner check must refuse.
    Release->Trigger();
    if (!TestTrue(TEXT("owner worker closes after GT negative"),Closed->Wait(1000))) return false;
    Worker.StopAndJoin();FSummary S;FPhysicalDetailRecord D;
    if (!TestTrue(TEXT("wrong-owner prefix is frozen"),CopyFrozenSummary(Session,S))) return false;
    TestTrue(TEXT("actual wrong-owner attempt retained"),S.WrongOwner>0);
    TestEqual(TEXT("wrong owner cannot qualify complete"),S.CompletePrefix,uint32(0));
    if (!TestTrue(TEXT("owner's one row remains readable"),ReadFrozenPhysicalDetailForTests(Session,0,D))) return false;
    TestEqual(TEXT("wrong owner did not count as copy"),D.CaptureCalls,uint32(1));
    TestEqual(TEXT("only real owner serial published"),D.DiagnosticSerial,uint64(101));
    TestEqual(TEXT("only real owner payload published"),D.TotalMilliseconds,1.25);
    TestEqual(TEXT("owner's validity retained separately from session failure"),D.ValidMask,uint32(31));
    return true;
}
#endif
