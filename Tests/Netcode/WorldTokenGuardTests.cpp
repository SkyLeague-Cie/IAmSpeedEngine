#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
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

int main(int Argc,char* Argv[])
{
    SetErrorMode(SEM_FAILCRITICALERRORS|SEM_NOGPFAULTERRORBOX|SEM_NOOPENFILEERRORBOX);
    if(Argc==2){Probe(Argv[1]);std::printf("TOKEN_PROBE_PASS %s\n",Argv[1]);return 0;}
    RebasedWorldShimRegressionMain();
    ActualTokenMoveAndOrigin();
    std::printf("WORLD_TOKEN_GUARD ordinary combined checks=%u; full UE class/native lease NOT_COMPILED/RUN/ADMITTED\n",Checks);
    return 0;
}
