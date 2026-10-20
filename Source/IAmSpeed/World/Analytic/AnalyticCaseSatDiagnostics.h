#pragma once

#include "CoreMinimal.h"

// Observational only: not serialized into simulation state or rollback snapshots.
// Draft231. No compilation, lifecycle or performance qualification is implied.
namespace Speed::Analytic::CaseSatDiagnostics
{
#if !UE_BUILD_SHIPPING
struct FCaseIdentity
{
    FString RunUUID;
    uint64 DriverKey = 0;
    uint64 Epoch = 0;
    int32 NativeCaseId = INDEX_NONE;
    int32 BatchIndex = INDEX_NONE;
    uint64 WorldGeneration = 0;
    int32 Mode = 0; //0off,1CPUonly,2CPU+SATwork; immutable for this joined case.
    bool bEnabled = false;
};

struct FSatCounters
{
    uint64 Calls = 0, CrossProducts = 0, NormalizedCrossAxes = 0, VisitedAxes = 0;
    uint64 ReachedFirstValidCross = 0, CompletedAllAxes = 0;
    uint64 PostSatShellRejected = 0, Contacts = 0;
    uint64 StationaryReject[13] = {};
    uint64 IntervalReject[13] = {};
    uint64 DegenerateCrossSlots[9] = {};
};

// Values-only copy from the existing same-thread active forward scope.
// No pointer/reference to TLS, allocation, clock call, or physical state mutation.
// Current frame is IN_PROGRESS; committed/partial status requires the existing close protocol.
struct FOwnedForwardIdentity917
{
    TCHAR RunUUID[65] = {};
    uint64 DriverKey=0, Epoch=0, WorldGeneration=0, CanonicalFrame=0;
    int32 NativeCaseId=INDEX_NONE, BatchIndex=INDEX_NONE, Mode=0;
    uint32 ThreadId=0;
};
IAMSPEED_API bool TryCopyOwnedForwardIdentity917(FOwnedForwardIdentity917& Out) noexcept;

IAMSPEED_API bool IsEnabled();
// Caller must supply a freshly joined owned lane. Returns disabled identity on
// missing runUUID or incompatible PhaseTiming (mode1 needs0,mode2 needs2).
// Never changes a physical admission decision or rewrites another cvar.
IAMSPEED_API bool BuildCaseIdentity(FCaseIdentity& Out, uint64 DriverKey,
    uint64 Epoch, int32 CaseId, int32 BatchIndex, uint64 WorldGeneration);

class IAMSPEED_API FScopedFrame final
{
public:
    FScopedFrame(const FCaseIdentity& Case, uint64 CanonicalFrame) noexcept;
    ~FScopedFrame() noexcept;
    void Finish(bool bCommitted) noexcept;
    FScopedFrame(const FScopedFrame&) = delete;
    FScopedFrame& operator=(const FScopedFrame&) = delete;
private:
    bool bActive = false;
    bool bFinished = false;
    bool bCpuStartValid = false;
    uint32 StartThreadId;
    uint64 Frame, CpuStartUser100ns, CpuStartKernel100ns;
    double WallStart;
};

// Draft400: select exactly one inclusive CPU-cycle phase per joined Mode1 case.
// Raw cycles, never elapsed seconds. Disabled by default; no physical state.
enum class ECpuPhase400 : uint8 { SubsystemStep=1, SweepDetailed=2, Extruded=3, BoxTriangle=4, BoxPlane=5, SweptBounds=6, PlaneTriangulation=7 };
class IAMSPEED_API FScopedCpuPhase400 final
{
public:
    explicit FScopedCpuPhase400(ECpuPhase400 Phase) noexcept;
    ~FScopedCpuPhase400() noexcept;
    FScopedCpuPhase400(const FScopedCpuPhase400&) = delete;
    FScopedCpuPhase400& operator=(const FScopedCpuPhase400&) = delete;
private:
    bool bEntered=false, bOuter=false, bStartValid=false;
    uint32 ThreadId=0;
    uint64 Start=0, Frame=0;
};

class IAMSPEED_API FScopedExtrudedProvider final
{
public:
    FScopedExtrudedProvider() noexcept;
    ~FScopedExtrudedProvider() noexcept;
    FScopedExtrudedProvider(const FScopedExtrudedProvider&) = delete;
    FScopedExtrudedProvider& operator=(const FScopedExtrudedProvider&) = delete;
private:
    bool bPrevious;
};

// Called only under the already-existing PhaseTiming2 guard in the SAT kernel.
// nullptr means unmeasured, NOT a certificate that other threads did no queries.
IAMSPEED_API FSatCounters* GetActiveSatCounters(bool bLazy) noexcept;
// Exactly one idempotent close on the owning worker; before input-owner teardown.
// Missing close marker means diagnostic UNKNOWN. It never prevents physical close.
IAMSPEED_API void FlushOnOwnerThread(const TCHAR* Reason) noexcept;
#endif
}
