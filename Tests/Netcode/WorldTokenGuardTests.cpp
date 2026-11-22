#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <future>
#include <new>
#include <string>
#include "NativePlanControlFlowTests.cpp"
#include <windows.h>

// Only our fixture allocation is protected. Its C++ object remains alive;
// access permission is restored before its destructor runs. This exposes ANY
// premature world-field load without pretending to be a race detector or UE.
class FProtectedWorldFixture final
{
    void* Allocation = nullptr;
    std::size_t Bytes = 0;
    bool Protected = false;
public:
    FWorldFixture* Fixture = nullptr;
    FProtectedWorldFixture()
    {
        SYSTEM_INFO Info{}; GetSystemInfo(&Info);
        Bytes = ((sizeof(FWorldFixture) + Info.dwPageSize - 1) / Info.dwPageSize) * Info.dwPageSize;
        Allocation = VirtualAlloc(nullptr,Bytes,MEM_COMMIT|MEM_RESERVE,PAGE_READWRITE);
        CHECK(Allocation); Fixture = new(Allocation) FWorldFixture();
    }
    void Protect()
    { DWORD Old = 0; CHECK(VirtualProtect(Allocation,Bytes,PAGE_NOACCESS,&Old)); Protected = true; }
    void Unprotect()
    { DWORD Old = 0; CHECK(VirtualProtect(Allocation,Bytes,PAGE_READWRITE,&Old)); Protected = false; }
    ~FProtectedWorldFixture()
    {
        if (Protected) Unprotect();
        if (Fixture) Fixture->~FWorldFixture();
        if (Allocation) VirtualFree(Allocation,0,MEM_RELEASE);
    }
};

static void Probe(const std::string& Mode)
{
    FProtectedWorldFixture P;
    auto* W = &P.Fixture->World;
    auto T = P.Fixture->Prepare(); CHECK(T);
    const auto Publication = W->ReadPreparedRestore(*T); CHECK(Publication);
    if (Mode == "--probe-worker")
    {
        P.Protect();
        bool Refused = false;
        std::thread Foreign([&] { Refused = !W->ReadPreparedRestore(*T)
            && !W->PrepareRestoreFinalization(*T) && !W->AbortPreparedRestore(*T); });
        Foreign.join(); P.Unprotect(); CHECK(Refused);
    }
    else if (Mode == "--probe-world")
    {
        FProtectedWorldFixture Other; auto* ForeignWorld = &Other.Fixture->World;
        Other.Protect();
        const bool Refused = !ForeignWorld->ReadPreparedRestore(*T)
            && !ForeignWorld->PrepareRestoreFinalization(*T) && !ForeignWorld->AbortPreparedRestore(*T);
        Other.Unprotect(); CHECK(Refused);
    }
    else if (Mode == "--probe-default")
    {
        FSimulationWorld::FPreparedRestoreToken Default;
        P.Protect();
        const bool Refused = !W->ReadPreparedRestore(Default)
            && !W->PrepareRestoreFinalization(Default) && !W->AbortPreparedRestore(Default);
        P.Unprotect(); CHECK(Refused);
    }
    else if (Mode == "--probe-finalize" || Mode == "--probe-release")
    {
        P.Protect();
        std::thread Foreign([&] {
            // Install on the thread that must fail closed; do not assume the
            // owner's CRT terminate-handler selection applies to this worker.
            std::set_terminate([] { std::_Exit(86); });
            if (Mode == "--probe-finalize") W->FinalizePreparedRestore(*T);
            else W->ReleasePreparedRestoreAfterPublication(*T,Publication);
        });
        Foreign.join(); P.Unprotect(); CHECK(false);
    }
    else CHECK(false);
    CHECK(W->ReadPreparedRestore(*T) == Publication && W->AbortPreparedRestore(*T));
    CHECK(P.Fixture->Untouched());
}

static void ActualTokenMoveAndOrigin()
{
    FWorldFixture F,Other;
    auto T=F.Prepare();CHECK(T);
    FSimulationWorld::FPreparedRestoreToken Moved(std::move(*T));
    CHECK(!F.World.ReadPreparedRestore(*T) && !F.World.AbortPreparedRestore(*T));
    CHECK(!Other.World.ReadPreparedRestore(Moved) && !Other.World.PrepareRestoreFinalization(Moved));
    auto Published=F.World.ReadPreparedRestore(Moved);CHECK(Published);
    CHECK(F.World.PrepareRestoreFinalization(Moved));
    F.World.FinalizePreparedRestore(Moved);
    CHECK(F.World.ReadPreparedRestore(Moved)==Published);
    F.World.ReleasePreparedRestoreAfterPublication(Moved,Published);
    CHECK(!F.World.ReadPreparedRestore(Moved) && !F.World.AbortPreparedRestore(Moved));
}

static void ReserveBeforeBuildingWithAnOutstandingBorrow()
{
    using FGate = Speed::FWorldBorrowGate;
    using namespace std::chrono_literals;
    FWorldFixture F;
    auto& Gate = F.World.StorageAccess;
    std::promise<bool> Held;
    std::promise<void> Release;
    auto Ready = Held.get_future();
    auto Released = Release.get_future();
    std::thread Reader([&]
    {
        const auto Scope = F.World.BorrowStorage();
        const auto Nested = F.World.BorrowStorage();
        const auto* Address = Gate.ReadPointer([&] { return &F.World.CurrentStepFrame; });
        Held.set_value(Scope && Nested && Address == &F.World.CurrentStepFrame && Gate.AdmitReference());
        if (Released.wait_for(4s) != std::future_status::ready) std::abort();
        if (*Address != 999) std::abort();
    });
    CHECK(Ready.wait_for(4s) == std::future_status::ready && Ready.get());
    {
        const FGate::FReservation Reservation(Gate);
        CHECK(!Reservation.Holds(Gate) && !Gate.HasUntrackedReference() && F.Untouched());
    }
    Release.set_value();
    Reader.join();

    {
        const FGate::FReservation Reservation(Gate);
        CHECK(Reservation.Holds(Gate) && Gate.IsReservationOwner());
        CHECK(!F.World.BorrowStorage() && !Gate.AdmitReference());
        bool LookupRan = false;
        const auto* Address = Gate.ReadPointer([&] { LookupRan = true; return &F.World.CurrentStepFrame; });
        CHECK(!Address && !LookupRan);
        bool OtherRefused = false;
        std::thread Other([&]
        { OtherRefused = !Gate.IsReservationOwner() && !F.World.BorrowStorage() && !F.Prepare(); });
        Other.join();
        CHECK(OtherRefused && F.Untouched());
        const auto Token = F.Prepare();
        CHECK(Token && F.World.AbortPreparedRestore(*Token) && F.Untouched());
    }
    CHECK(F.World.BorrowStorage() && !Gate.HasUntrackedReference());
}

static void BareLegacyReferencesStayVisibleToReservation()
{
    using FGate = Speed::FWorldBorrowGate;
    FGate PointerGate, ReferenceGate;
    int Record = 17;
    CHECK(!PointerGate.ReadPointer([]() -> int* { return nullptr; }));
    {
        const FGate::FReservation EmptyLookupAllowed(PointerGate);
        CHECK(EmptyLookupAllowed.Holds(PointerGate));
    }
    auto* Escaped = PointerGate.ReadPointer([&] { return &Record; });
    CHECK(Escaped == &Record && PointerGate.HasUntrackedReference());
    const FGate::FReservation PointerRefused(PointerGate);
    CHECK(!PointerRefused.Holds(PointerGate) && *Escaped == 17);
    CHECK(ReferenceGate.AdmitReference() && ReferenceGate.HasUntrackedReference());
    const FGate::FReservation ReferenceRefused(ReferenceGate);
    CHECK(!ReferenceRefused.Holds(ReferenceGate));
}

int main(int Argc,char* Argv[])
{
    SetErrorMode(SEM_FAILCRITICALERRORS|SEM_NOGPFAULTERRORBOX|SEM_NOOPENFILEERRORBOX);
    if(Argc==2){Probe(Argv[1]);std::printf("TOKEN_PROBE_PASS %s\n",Argv[1]);return 0;}
    RebasedWorldShimRegressionMain();
    ActualTokenMoveAndOrigin();
    ReserveBeforeBuildingWithAnOutstandingBorrow();
    BareLegacyReferencesStayVisibleToReservation();
    std::printf("WORLD_TOKEN_GUARD ordinary combined checks=%u; full UE class/native lease NOT_COMPILED/RUN/ADMITTED\n",Checks);
    return 0;
}
