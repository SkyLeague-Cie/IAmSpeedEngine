// Compile the ACTUAL PreparedWorldRestore.inc against explicit ordinary shims.
// This tests transaction control flow, NOT UE containers/UObjects, native
// lifetime fences, complete mechanical/gameplay restoration or physics.
#define main FrozenCoreTestsMain
#include "PreparedWorldCoreTests.cpp"
#undef main
#include "IAmSpeed/World/Simulation/PreparedSimulationRestore.h"
#include <functional>
#include <thread>

class ISpeedComponent
{
public:
    uint64 StableId = 0;
    virtual ~ISpeedComponent() = default;
    virtual std::unique_ptr<Speed::IPreparedSimulationComponentRestore>
        PrepareCompleteSimulationRestore(const Speed::FSimulationComponentRestoreRequest&) { return {}; }
};
class USolidSubBody
{
public:
    uint64 StableId = 0; uint32 ObjectId = 0; ISpeedComponent* Parent = nullptr;
    uint32 GetUniqueID() const noexcept { return ObjectId; }
    ISpeedComponent* GetParentComponent() const noexcept { return Parent; }
};
struct FDynamicContactPair
{
    USolidSubBody* BodyA = nullptr; USolidSubBody* BodyB = nullptr;
    FVector LocalAnchorA, LocalAnchorB, LocalNormalB, LastCOMA, LastCOMB;
    uint64 PairKey = 0;
    uint32 FirstSeenFrame = 0, LastSeenFrame = 0, LastSolvedFrame = 0, SupportedFrameCount = 0;
    uint32 ActiveContactFrameCount = 0, ReactionFrameCount = 0, FeatureTransitionCount = 0;
    float AcquisitionNormalSpeed = 0, AccumulatedNormalImpulse = 0, AccumulatedNormalActivity = 0;
    float AccumulatedNormalSupportActivity = 0, PeakNormalActivity = 0, PeakNormalSupportActivity = 0;
    float MinimumSeparation = 0, MaximumSeparation = 0;
    bool bRollingManifoldReady = false, bActiveContactLastSolve = false;
};
struct FSimulationSnapshot
{
    uint64 NumFrame = 0, InputJournalHash = 0, StateHash = 0;
    TArray<uint8> Payload;
};
namespace Speed { struct FSimulationBodyRecord { uint64 StableId = 0; ISpeedComponent* Adapter = nullptr; }; }
#include "test-shim/WorldPlanDeclaration.inc"
uint64 Speed::FSimulationWorld::FindStableId(const ISpeedComponent& C) const { return C.StableId; }
uint64 Speed::FSimulationWorld::FindStableSubBodyId(const USolidSubBody* C) const { return C ? C->StableId : 0; }
#include "IAmSpeed/World/Simulation/PreparedWorldRestore.inc"
#include "IAmSpeed/World/Simulation/PreparedAccessComposition.inc"
using namespace Speed;

struct FCounters { unsigned Abort = 0, Release = 0, Finalize = 0; };
class FTestLease final : public ISimulationRestoreBoundaryLease
{
    FSimulationWorld* World; FAddress Expected; FCounters& Counters;
    std::thread::id Worker = std::this_thread::get_id(); bool Done = false;
public:
    bool Protect = true, Valid = true, Ranges = true;
    std::function<void()> OnQuery, OnProtect, OnAccess, OnAbort, OnRelease, OnDestruction;
    mutable bool SawReservation = false;
    FPreparedRestoreLeaseView RecursiveProbe;
    mutable bool RecursiveProbeRefused = false;
    FTestLease(FSimulationWorld& W, const FAddress& A, FCounters& C) : World(&W), Expected(A), Counters(C) {}
    ~FTestLease() override { if (!Done) AbortBeforeFinalize(); if (OnDestruction) OnDestruction(); }
    bool HoldsExactBoundary(const FSimulationWorld& W, const FAddress& A) const noexcept override
    { if (OnQuery) OnQuery(); SawReservation = W.IsRestoreBoundaryReserved(); RecursiveProbeRefused = !RecursiveProbe.ProtectsEscapedMutableViews();
      return SawReservation && Valid && !Done && &W == World && Expected == A && Worker == std::this_thread::get_id(); }
    bool ProtectsEscapedMutableViews() const noexcept override { if (OnProtect) OnProtect(); return Protect; }
    bool AppendPreparedAccess(std::vector<PreparedAccess::FParticipantAccess>& Out) const override
    {
        if (OnAccess) OnAccess();
        if (!Ranges) return ISimulationRestoreBoundaryLease::AppendPreparedAccess(Out);
        const auto S = [](const void* P, std::size_t N) { return PreparedAccess::FSpan{reinterpret_cast<std::uintptr_t>(P), N}; };
        Out.push_back({reinterpret_cast<std::uintptr_t>(this), {S(&Counters, sizeof(Counters)), S(&Done, sizeof(Done)), S(&SawReservation, sizeof(SawReservation)), S(&RecursiveProbeRefused, sizeof(RecursiveProbeRefused))},
            {S(&Expected, sizeof(Expected)), S(&World, sizeof(World)), S(&Worker, sizeof(Worker))}}); return true;
    }
    void AbortBeforeFinalize() noexcept override { if (!Done) { Done = true; ++Counters.Abort; if (OnAbort) OnAbort(); } }
    void ReleaseAfterPublication() noexcept override { if (!Done) { Done = true; ++Counters.Release; if (OnRelease) OnRelease(); } }
};
class FTestSupplement final : public IPreparedSimulationWorldSupplement
{
    FAddress At; FCounters& Counters; bool Done = false;
public:
    bool FailFence = false, RefuseLease = false;
    std::function<void()> Reenter, OnCleanup, OnDestruction;
    FPreparedRestoreLeaseView OwnerLease;
    explicit FTestSupplement(const FAddress& A, FCounters& C) : At(A), Counters(C) {}
    ~FTestSupplement() override { if (!Done) AbortBeforeFinalize(); if (OnDestruction) OnDestruction(); }
    const FAddress& Address() const noexcept override { if (Reenter) Reenter(); return At; }
    uint64 Schema() const noexcept override { return 92; }
    uint64 CoveredDomains() const noexcept override { return CompleteWorldCoverage; }
    bool BindOwnerLease(FPreparedRestoreLeaseView V) override
    { if (RefuseLease) return IPreparedSimulationWorldSupplement::BindOwnerLease(V); if (!V.IsExactAddress(At)) return false; OwnerLease = V; return true; }
    bool AppendPreparedAccess(std::vector<PreparedAccess::FParticipantAccess>& Out) const override
    {
        const auto S = [](const void* P, std::size_t N) { return PreparedAccess::FSpan{reinterpret_cast<std::uintptr_t>(P), N}; };
        Out.push_back({reinterpret_cast<std::uintptr_t>(this), {S(&Counters, sizeof(Counters)), S(&Done, sizeof(Done))}, {S(&At, sizeof(At))}}); return true;
    }
    bool PrepareFinalization() override { if (Reenter) Reenter(); return !FailFence; }
    void Finalize() noexcept override { if (Reenter) Reenter(); ++Counters.Finalize; }
    void AbortBeforeFinalize() noexcept override { if (!Done) { Done = true; ++Counters.Abort; if (OnCleanup) OnCleanup(); } }
    void ReleaseAfterPublication() noexcept override { if (!Done) { Done = true; ++Counters.Release; if (OnCleanup) OnCleanup(); } }
};
class FTestComponent;
class FTestComponentPlan final : public IPreparedSimulationComponentRestore
{
    FAddress At; FActorBinding Actor; FTestComponent& Component; SKinematic State;
    bool Frozen = false, Done = false;
    FPreparedRestoreLeaseView OwnerLease;
public:
    FTestComponentPlan(const FSimulationComponentRestoreRequest&, FTestComponent&);
    ~FTestComponentPlan() override { if (!Done) AbortBeforeFinalize(); }
    const FAddress& Address() const noexcept override { return At; }
    const FActorBinding& Binding() const noexcept override { return Actor; }
    uint64 CoveredDomains() const noexcept override { return CompleteActorCoverage; }
    bool AppendPreparedAccess(std::vector<PreparedAccess::FParticipantAccess>&) const override;
    bool PrepareFinalization() override;
    void Finalize() noexcept override;
    void AbortBeforeFinalize() noexcept override;
    void ReleaseAfterPublication() noexcept override;
};
class FTestComponent final : public ISpeedComponent
{
public:
    FCounters Counters; SKinematic Live; bool Frozen = false;
    bool Refuse = false, FailFence = false, CorruptBinding = false, NoRanges = false, DriftRanges = false;
    bool SawLeaseInFinalize = false, SawRevokedOnAbort = false, SawRevokedOnRelease = false;
    uint64 WheelValue = 0, SubbodyValue = 0;
    uint8 DriftTarget = 0;
    std::array<uint64, 4> CommonConfiguration{}, WheelConfiguration{}, SubbodyConfiguration{};
    std::uintptr_t ExtraWrite = 0;
    FPreparedRestoreLeaseView SavedLease;
    std::function<void()> Reenter;
    std::function<void()> OnCleanup;
    std::unique_ptr<IPreparedSimulationComponentRestore> PrepareCompleteSimulationRestore(const FSimulationComponentRestoreRequest& R) override
    { if (Refuse || !R.OwnerLease.IsExactAddress(R.Address)) return {}; SavedLease = R.OwnerLease; return std::make_unique<FTestComponentPlan>(R, *this); }
};
FTestComponentPlan::FTestComponentPlan(const FSimulationComponentRestoreRequest& R, FTestComponent& C)
    : At(R.Address), Actor(R.Supplement.Binding), Component(C), State(R.Kinematic), Frozen(R.bFrozen), OwnerLease(R.OwnerLease) {}
bool FTestComponentPlan::AppendPreparedAccess(std::vector<PreparedAccess::FParticipantAccess>& Out) const
{
    if (Component.Reenter) Component.Reenter();
    if (Component.NoRanges) return IPreparedSimulationComponentRestore::AppendPreparedAccess(Out);
    const auto S = [](const void* P, std::size_t N) { return PreparedAccess::FSpan{reinterpret_cast<std::uintptr_t>(P), N}; };
    std::vector<PreparedAccess::FSpan> CommonWrites = {S(&Component.Live, sizeof(Component.Live)), S(&Component.Frozen, sizeof(Component.Frozen)),
        S(&Component.Counters, sizeof(Component.Counters)), S(&Done, sizeof(Done)), S(&Component.SawLeaseInFinalize, sizeof(Component.SawLeaseInFinalize)),
        S(&Component.SawRevokedOnAbort, sizeof(Component.SawRevokedOnAbort)), S(&Component.SawRevokedOnRelease, sizeof(Component.SawRevokedOnRelease))};
    if (Component.ExtraWrite) CommonWrites.push_back({Component.ExtraWrite, 1});
    const std::array<std::vector<PreparedAccess::FSpan>, 1> CommonRead{{{S(&Component.CommonConfiguration, sizeof(Component.CommonConfiguration)),
        S(&At, sizeof(At)), S(&Actor, sizeof(Actor)), S(&State, sizeof(State)), S(&Frozen, sizeof(Frozen))}}};
    const std::array<PreparedAccess::FSpan, 1> WheelWrites{S(&Component.WheelValue, sizeof(Component.WheelValue))};
    const std::array<std::vector<PreparedAccess::FSpan>, 1> WheelRead{{{S(&Component.WheelConfiguration, sizeof(Component.WheelConfiguration))}}};
    const std::array<PreparedAccess::FSpan, 1> SubWrites{S(&Component.SubbodyValue, sizeof(Component.SubbodyValue))};
    const std::array<std::vector<PreparedAccess::FSpan>, 1> SubRead{{{S(&Component.SubbodyConfiguration, sizeof(Component.SubbodyConfiguration))}}};
    return PreparedAccess::AppendPlanAccess(Out, reinterpret_cast<std::uintptr_t>(this), CommonWrites, CommonRead)
        && PreparedAccess::AppendPlanAccess(Out, reinterpret_cast<std::uintptr_t>(&Component.WheelValue), WheelWrites, WheelRead)
        && PreparedAccess::AppendPlanAccess(Out, reinterpret_cast<std::uintptr_t>(&Component.SubbodyValue), SubWrites, SubRead);
}
bool FTestComponentPlan::PrepareFinalization()
{ if (!OwnerLease.IsExactAddress(At)) return false; if (Component.Reenter) Component.Reenter(); if (Component.CorruptBinding) ++Actor.Incarnation;
    if (Component.DriftRanges) Component.ExtraWrite = reinterpret_cast<std::uintptr_t>(&Component.DriftTarget); return !Component.FailFence; }
void FTestComponentPlan::Finalize() noexcept
{ Component.SawLeaseInFinalize = OwnerLease.IsExactAddress(At); Component.Live = State; Component.Frozen = Frozen;
    Component.WheelValue = 111; Component.SubbodyValue = 222; ++Component.Counters.Finalize; if (Component.Reenter) Component.Reenter(); }
void FTestComponentPlan::AbortBeforeFinalize() noexcept
{ if (!Done) { Done = true; Component.SawRevokedOnAbort = !OwnerLease.IsExactAddress(At); ++Component.Counters.Abort; if (Component.OnCleanup) Component.OnCleanup(); } }
void FTestComponentPlan::ReleaseAfterPublication() noexcept
{ if (!Done) { Done = true; Component.SawRevokedOnRelease = !OwnerLease.IsExactAddress(At); ++Component.Counters.Release; if (Component.OnCleanup) Component.OnCleanup(); } }
struct FWorldFixture
{
    FFixture Format;
    FTestComponent Components[3];
    USolidSubBody Endpoints[4];
    FCounters LeaseCounters, SupplementCounters; // must outlive World/plan destruction
    FSimulationWorld World;
    FSimulationSnapshot Snapshot;
    std::shared_ptr<const FRestoreEnvelope> Envelope;
    FWorldFixture()
    {
        Envelope = std::make_shared<const FRestoreEnvelope>(Format.Envelope);
        for (std::size_t I = 0; I < 3; ++I)
        {
            Components[I].StableId = Format.Envelope.Actors[I].StableId;
            World.Bodies.Add({Components[I].StableId, &Components[I]});
            World.AdaptersByStableId.Add(Components[I].StableId, &Components[I]);
        }
        for (std::size_t I = 0; I < 4; ++I)
        {
            const auto& E = Format.Envelope.Endpoints[I];
            Endpoints[I].StableId = E.StableId; Endpoints[I].ObjectId = E.NativeObjectId;
            Endpoints[I].Parent = World.AdaptersByStableId.FindRef(E.Actor);
            World.SolidSubBodiesByStableId.Add(E.StableId, &Endpoints[I]);
        }
        Snapshot.NumFrame = Format.Envelope.Address.Frame; Snapshot.InputJournalHash = Format.Envelope.Address.InputHash;
        Snapshot.StateHash = Format.Envelope.Address.StateHash; Snapshot.Payload.assign(Format.Bytes.begin(), Format.Bytes.end());
        World.CurrentStepFrame = 999; World.ContactStorage->DynamicContactPairIndices.Add(123, 9);
    }
    auto Prepare()
    { return World.PrepareSnapshotRestore(Snapshot, Envelope,
        std::make_unique<FTestLease>(World, Envelope->Address, LeaseCounters),
        std::make_unique<FTestSupplement>(Envelope->Address, SupplementCounters)); }
    bool Untouched() const
    {
        for (const auto& C : Components) if (C.Counters.Finalize) return false;
        return World.CurrentStepFrame == 999 && World.ContactStorage->DynamicContactPairIndices.FindRef(123) == 9;
    }
};
static void TestActualPlanControlFlowUnderShim()
{
    {
        FWorldFixture F; auto* Old = F.World.ContactStorage.get(); auto T = F.Prepare(); CHECK(T && F.Untouched());
        auto P = F.World.ReadPreparedRestore(*T); CHECK(P && P->Envelope.get() != F.Envelope.get());
        CHECK(P->Physical->Bodies.size() == 3 && F.World.IsRestoreBoundaryReserved());
        bool ReentrantAbort = true;
        F.Components[0].Reenter = [&]() { ReentrantAbort = F.World.AbortPreparedRestore(*T); };
        CHECK(F.World.PrepareRestoreFinalization(*T)); CHECK(!ReentrantAbort && F.Untouched());
        const auto Count = Allocations.load(); FailAfter = 0;
        F.World.FinalizePreparedRestore(*T);
        FailAfter = -1; CHECK(Allocations.load() == Count); CHECK(!ReentrantAbort);
        CHECK(F.World.ContactStorage.get() != Old && F.World.CurrentStepFrame == 42);
        CHECK(F.LeaseCounters.Release == 0 && F.SupplementCounters.Finalize == 1);
        CHECK(F.World.ContactStorage->DynamicContactPairs.Num() == 2 && F.World.ContactStorage->PendingRollingContactPairs.Num() == 1);
        CHECK(F.World.ContactStorage->DynamicContactPairs[0].BodyA->StableId == 8001);
        CHECK(F.World.ContactStorage->DynamicContactPairs[0].LastSolvedFrame == UINT32_MAX);
        CHECK(SameBits(F.World.ContactStorage->DynamicContactPairs[0].LocalAnchorB.X, -0.0));
        CHECK(F.Components[2].Frozen && SameBits(F.Components[0].Live.Location.X, -0.0));
        CHECK(!F.World.AbortPreparedRestore(*T)); CHECK(!F.World.PrepareRestoreFinalization(*T));
        // Fixture publication marker only; no common input/world publisher exists here.
        auto Published = P;
        const auto ReleaseCount = Allocations.load(); FailAfter = 0;
        F.World.ReleasePreparedRestoreAfterPublication(*T, Published);
        FailAfter = -1; CHECK(Allocations.load() == ReleaseCount);
        CHECK(!F.World.IsRestoreBoundaryReserved() && F.LeaseCounters.Release == 1 && F.SupplementCounters.Release == 1);
        CHECK(F.Components[0].Counters.Release == 1 && !F.World.ReadPreparedRestore(*T));
        CHECK(!F.Prepare()); // same generation/publication cannot be replayed by a new coordinator
    }
    for (unsigned Fail = 0; Fail < 3; ++Fail)
    {
        FWorldFixture F; F.Components[Fail].Refuse = true;
        CHECK(!F.Prepare()); CHECK(F.Untouched() && !F.World.IsRestoreBoundaryReserved());
        for (unsigned I = 0; I < Fail; ++I) CHECK(F.Components[I].Counters.Abort == 1);
        CHECK(F.LeaseCounters.Abort == 1 && F.SupplementCounters.Abort == 1);
    }
    for (unsigned Fail = 0; Fail < 3; ++Fail)
    {
        FWorldFixture F; F.Components[Fail].FailFence = true; auto T = F.Prepare(); CHECK(T);
        CHECK(!F.World.PrepareRestoreFinalization(*T)); CHECK(F.Untouched() && !F.World.IsRestoreBoundaryReserved());
        for (const auto& C : F.Components) CHECK(C.Counters.Abort == 1);
    }
    {
        FWorldFixture F; F.Components[1].CorruptBinding = true; auto T = F.Prepare(); CHECK(T);
        CHECK(!F.World.PrepareRestoreFinalization(*T)); CHECK(F.Untouched());
    }
    {
        FWorldFixture F; auto T = F.Prepare(); CHECK(T);
        bool Aborted = true;
        std::thread Foreign([&]() { Aborted = F.World.AbortPreparedRestore(*T); }); Foreign.join();
        CHECK(!Aborted && F.World.IsRestoreBoundaryReserved()); CHECK(F.World.AbortPreparedRestore(*T)); CHECK(F.Untouched());
    }
    {
        FWorldFixture F; auto T = F.Prepare(); CHECK(T); T.reset();
        CHECK(F.World.IsRestoreBoundaryReserved() && F.Untouched());
        // Dropping an opaque token does not accidentally unpause/finalize.
        // World destruction aborts the unfinalized plan while its counters and components are alive.
    }
    {
        FWorldFixture F;
        auto Lease = std::make_unique<FTestLease>(F.World, F.Envelope->Address, F.LeaseCounters); Lease->Protect = false;
        CHECK(!F.World.PrepareSnapshotRestore(F.Snapshot, F.Envelope, std::move(Lease),
            std::make_unique<FTestSupplement>(F.Envelope->Address, F.SupplementCounters)));
        CHECK(F.Untouched() && F.LeaseCounters.Abort == 1);
    }
    unsigned Failures = 0; bool Complete = false;
    for (long long Budget = 0; Budget < 512; ++Budget)
    {
        FWorldFixture F; std::optional<FSimulationWorld::FPreparedRestoreToken> T;
        FailAfter = Budget;
        try { T = F.Prepare(); }
        catch (const std::bad_alloc&) {} // argument allocation may fail before function entry
        FailAfter = -1;
        CHECK(F.Untouched());
        if (T) { CHECK(F.World.AbortPreparedRestore(*T)); Complete = true; break; }
        ++Failures; CHECK(!F.World.IsRestoreBoundaryReserved());
    }
    CHECK(Complete && Failures > 50);
    std::printf("actual plan INC under ORDINARY SHIMS: %u failure budgets; full UE/native world remains NOT_COMPILED/RUN/ADMITTED\n", Failures);
}
static void TestCompositionAndLeaseExpiry()
{
    FPreparedRestoreLeaseView Empty;
    FAddress A; CHECK(!Empty.IsExactAddress(A) && !Empty.ProtectsEscapedMutableViews());
    FPreparedRestoreLeaseView Escaped;
    {
        FWorldFixture F;
        auto Lease = std::make_unique<FTestLease>(F.World, F.Envelope->Address, F.LeaseCounters); auto* Ptr = Lease.get();
        auto T = F.World.PrepareSnapshotRestore(F.Snapshot, F.Envelope, std::move(Lease),
            std::make_unique<FTestSupplement>(F.Envelope->Address, F.SupplementCounters));
        CHECK(T && Ptr->SawReservation);
        Ptr->RecursiveProbe = F.Components[0].SavedLease;
        CHECK(Ptr->RecursiveProbe.IsExactAddress(F.Envelope->Address) && Ptr->RecursiveProbeRefused);
        CHECK(F.World.AbortPreparedRestore(*T));
    }
    {
        FWorldFixture F; auto T = F.Prepare(); CHECK(T);
        Escaped = F.Components[0].SavedLease;
        CHECK(Escaped.IsExactAddress(F.Envelope->Address));
        auto Wrong = F.Envelope->Address; ++Wrong.WorldIncarnation; CHECK(!Escaped.IsExactAddress(Wrong));
        FSimulationWorld OtherWorld; CHECK(!Escaped.HoldsExactBoundary(OtherWorld, F.Envelope->Address));
        bool Foreign = true; std::thread Worker([&] { Foreign = Escaped.IsExactAddress(F.Envelope->Address); }); Worker.join(); CHECK(!Foreign);
        CHECK(F.World.PrepareRestoreFinalization(*T));
        CHECK(Escaped.IsExactAddress(F.Envelope->Address));
        F.World.FinalizePreparedRestore(*T); CHECK(Escaped.IsExactAddress(F.Envelope->Address));
        CHECK(F.LeaseCounters.Release == 0);
        for (const auto& C : F.Components) CHECK(C.SawLeaseInFinalize);
        auto Published = F.World.ReadPreparedRestore(*T);
        F.World.ReleasePreparedRestoreAfterPublication(*T, Published);
        CHECK(!Escaped.IsExactAddress(F.Envelope->Address));
        for (const auto& C : F.Components) CHECK(C.SawRevokedOnRelease);
    }
    CHECK(!Escaped.ProtectsEscapedMutableViews()); // no dangling lease/world dereference after owner destruction
    {
        FWorldFixture F; auto T = F.Prepare(); CHECK(T); auto View = F.Components[0].SavedLease;
        std::atomic<bool> Entered{false}, Stop{false}, WrongThreadGranted{false};
        std::thread Reader([View, &Entered, &Stop, &WrongThreadGranted, At = F.Envelope->Address]()
        {
            Entered = true;
            while (!Stop.load()) if (View.IsExactAddress(At)) WrongThreadGranted = true;
        });
        while (!Entered.load()) std::this_thread::yield();
        CHECK(F.World.AbortPreparedRestore(*T)); Stop = true; Reader.join();
        CHECK(!WrongThreadGranted.load() && !View.IsExactAddress(F.Envelope->Address));
    }
    {
        FWorldFixture F; auto T = F.Prepare(); CHECK(T); auto View = F.Components[0].SavedLease;
        CHECK(F.World.AbortPreparedRestore(*T)); CHECK(!View.IsExactAddress(F.Envelope->Address));
        for (const auto& C : F.Components) CHECK(C.SawRevokedOnAbort);
        CHECK(F.LeaseCounters.Abort == 1 && F.Untouched());
    }
    {
        FWorldFixture F;
        F.Components[0].ExtraWrite = reinterpret_cast<std::uintptr_t>(&F.Components[1].WheelConfiguration[2]);
        CHECK(!F.Prepare() && F.Untouched()); // COMMON write versus ANOTHER wheel config/backing range
        for (const auto& C : F.Components) CHECK(!C.SavedLease.ProtectsEscapedMutableViews());
    }
    {
        FWorldFixture F;
        F.Components[1].ExtraWrite = reinterpret_cast<std::uintptr_t>(&F.Components[2].SubbodyValue);
        CHECK(!F.Prepare() && F.Untouched()); // write/write alias across retained subplans
    }
    {
        FWorldFixture F; F.Components[1].NoRanges = true;
        CHECK(!F.Prepare() && F.Untouched()); // old/default provider cannot infer a range grant
    }
    {
        FWorldFixture F; F.Components[1].DriftRanges = true;
        auto T = F.Prepare(); CHECK(T); CHECK(!F.World.PrepareRestoreFinalization(*T)); CHECK(F.Untouched());
    }
    {
        FWorldFixture F;
        auto Lease = std::make_unique<FTestLease>(F.World, F.Envelope->Address, F.LeaseCounters); Lease->Ranges = false;
        CHECK(!F.World.PrepareSnapshotRestore(F.Snapshot, F.Envelope, std::move(Lease),
            std::make_unique<FTestSupplement>(F.Envelope->Address, F.SupplementCounters)));
        CHECK(F.Untouched() && F.LeaseCounters.Abort == 1);
    }
    {
        FWorldFixture F;
        auto Supplement = std::make_unique<FTestSupplement>(F.Envelope->Address, F.SupplementCounters); Supplement->RefuseLease = true;
        CHECK(!F.World.PrepareSnapshotRestore(F.Snapshot, F.Envelope,
            std::make_unique<FTestLease>(F.World, F.Envelope->Address, F.LeaseCounters), std::move(Supplement)));
        CHECK(F.Untouched() && F.LeaseCounters.Abort == 1);
    }
}
static void TestLeaseCallbackAndCleanupReentry()
{
    // Real transaction INC and owner view code; these participants remain
    // explicit control-flow fixtures, never native lifetime qualifications.
    for (unsigned Phase = 0; Phase < 9; ++Phase)
    {
        FWorldFixture F;
        auto Lease = std::make_unique<FTestLease>(F.World, F.Envelope->Address, F.LeaseCounters);
        auto* L = Lease.get();
        auto T = F.World.PrepareSnapshotRestore(F.Snapshot, F.Envelope, std::move(Lease),
            std::make_unique<FTestSupplement>(F.Envelope->Address, F.SupplementCounters));
        CHECK(T); unsigned Calls = 0;
        const auto Probe = [&]()
        {
            ++Calls;
            CHECK(F.World.IsRestoreBoundaryReserved());
            CHECK(!F.World.AbortPreparedRestore(*T));
            CHECK(!F.World.PrepareRestoreFinalization(*T));
            CHECK(!F.World.ReadPreparedRestore(*T));
            CHECK(!F.Prepare()); // includes the old-plan-null cleanup window
        };
        if (Phase == 0 || Phase == 1 || Phase == 2) L->OnQuery = Probe;
        if (Phase == 3) L->OnProtect = Probe;
        if (Phase == 4) L->OnAccess = Probe;
        if (Phase == 5 || Phase == 6) F.Components[0].OnCleanup = Probe;
        if (Phase == 7) { L->OnAbort = Probe; L->OnDestruction = Probe; }
        if (Phase == 8) { L->OnRelease = Probe; L->OnDestruction = Probe; }
        if (Phase == 0)
        {
            // Escaped view outside a world entry: only the owner's private
            // Querying fence can reject this reentrant destruction attempt.
            CHECK(F.Components[0].SavedLease.IsExactAddress(F.Envelope->Address));
            CHECK(Calls > 0 && F.Untouched()); L->OnQuery = {};
            CHECK(F.World.AbortPreparedRestore(*T));
        }
        else if (Phase == 5 || Phase == 7)
            CHECK(F.World.AbortPreparedRestore(*T));
        else
        {
            CHECK(F.World.PrepareRestoreFinalization(*T));
            if (Phase == 1 || Phase == 3 || Phase == 4) CHECK(Calls > 0 && F.Untouched());
            // Reset count to require the preamble/tail or cleanup itself to
            // exercise the callback, not merely the earlier preparation.
            if (Phase == 2 || Phase == 6 || Phase == 8) Calls = 0;
            F.World.FinalizePreparedRestore(*T);
            auto Published = F.World.ReadPreparedRestore(*T); CHECK(Published);
            F.World.ReleasePreparedRestoreAfterPublication(*T, Published);
        }
        CHECK(Calls > 0 && !F.World.IsRestoreBoundaryReserved());
    }
    {
        FWorldFixture F; unsigned Calls = 0;
        auto Lease = std::make_unique<FTestLease>(F.World, F.Envelope->Address, F.LeaseCounters);
        Lease->OnDestruction = [&]()
        { ++Calls; CHECK(F.World.IsRestoreBoundaryReserved()); CHECK(!F.Prepare()); };
        CHECK(!F.World.PrepareSnapshotRestore(F.Snapshot, nullptr, std::move(Lease),
            std::make_unique<FTestSupplement>(F.Envelope->Address, F.SupplementCounters)));
        CHECK(Calls == 1 && !F.World.IsRestoreBoundaryReserved() && F.Untouched());
    }
    for (bool Commit : {false, true})
    {
        FWorldFixture F;
        auto Supplement = std::make_unique<FTestSupplement>(F.Envelope->Address, F.SupplementCounters);
        auto* S = Supplement.get();
        auto T = F.World.PrepareSnapshotRestore(F.Snapshot, F.Envelope,
            std::make_unique<FTestLease>(F.World, F.Envelope->Address, F.LeaseCounters), std::move(Supplement));
        CHECK(T); unsigned Calls = 0;
        const auto Probe = [&]()
        { ++Calls; CHECK(F.World.IsRestoreBoundaryReserved()); CHECK(!F.World.AbortPreparedRestore(*T));
          CHECK(!F.World.ReadPreparedRestore(*T)); CHECK(!F.World.PrepareRestoreFinalization(*T)); CHECK(!F.Prepare()); };
        S->Reenter = Probe; S->OnCleanup = Probe; S->OnDestruction = Probe;
        F.Components[0].Reenter = Probe;
        if (Commit)
        {
            CHECK(F.World.PrepareRestoreFinalization(*T)); F.World.FinalizePreparedRestore(*T);
            auto Published = F.World.ReadPreparedRestore(*T); CHECK(Published);
            F.World.ReleasePreparedRestoreAfterPublication(*T, Published);
        }
        else CHECK(F.World.AbortPreparedRestore(*T));
        CHECK(Calls >= 2 && !F.World.IsRestoreBoundaryReserved());
    }
    {
        // The world destructor marks cleanup before resetting its plan. Other
        // owners (components, endpoints, snapshot and token) outlive it here.
        FWorldFixture F;
        auto World = std::make_unique<FSimulationWorld>(); auto* W = World.get();
        W->Bodies = F.World.Bodies; W->AdaptersByStableId = F.World.AdaptersByStableId;
        W->SolidSubBodiesByStableId = F.World.SolidSubBodiesByStableId;
        auto Lease = std::make_unique<FTestLease>(*W, F.Envelope->Address, F.LeaseCounters);
        auto* L = Lease.get();
        auto T = W->PrepareSnapshotRestore(F.Snapshot, F.Envelope, std::move(Lease),
            std::make_unique<FTestSupplement>(F.Envelope->Address, F.SupplementCounters));
        CHECK(T); unsigned Calls = 0;
        L->OnDestruction = [&]()
        {
            ++Calls; CHECK(W->IsRestoreBoundaryReserved()); CHECK(!W->AbortPreparedRestore(*T));
            CHECK(!W->PrepareSnapshotRestore(F.Snapshot, F.Envelope,
                std::make_unique<FTestLease>(*W, F.Envelope->Address, F.LeaseCounters),
                std::make_unique<FTestSupplement>(F.Envelope->Address, F.SupplementCounters)));
        };
        World.reset(); CHECK(Calls == 1);
    }
}
int RebasedWorldShimRegressionMain()
{
    FrozenCoreTestsMain(); TestActualPlanControlFlowUnderShim(); TestCompositionAndLeaseExpiry(); TestLeaseCallbackAndCleanupReentry();
    std::printf("PreparedWorld combined ordinary checks: %u PASS; SHIM fence/actors are fixture-only, zero native qualification\n", Checks);
    return 0; // Test main may be included under another entry point.
}
