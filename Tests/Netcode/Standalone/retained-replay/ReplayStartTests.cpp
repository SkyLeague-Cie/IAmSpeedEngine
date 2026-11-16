#include "IAmSpeed/Input/SameFrameInputOwner.h"
#include <cstdlib>
#include <iostream>
#include <new>
#include <stdexcept>
#include <string>

static thread_local int Budget = -1, Allocations = 0;
void* operator new(std::size_t N)
{ if (Budget >= 0) { ++Allocations; if (Budget-- == 0) throw std::bad_alloc(); } if (void* P = std::malloc(N ? N : 1)) return P; throw std::bad_alloc(); }
void operator delete(void* P) noexcept { std::free(P); }
void operator delete(void* P, std::size_t) noexcept { std::free(P); }
void* operator new[](std::size_t N) { return ::operator new(N); }
void operator delete[](void* P) noexcept { ::operator delete(P); }
void operator delete[](void* P, std::size_t) noexcept { ::operator delete(P); }
namespace I = Speed::Input;
namespace V = Speed::Input::V2;
static int Checks = 0;
#define CHECK(E) do { ++Checks; if (!(E)) throw std::runtime_error(std::string(__func__) + ":" + std::to_string(__LINE__) + " " #E); } while (false)
static std::shared_ptr<const V::FInputActionContract> Contract()
{
    static const auto C = [] { V::FInputActionContractDescription D; D.Revision.Value = 1;
        D.Actions = V::FInputActionContract::BaseActions();
        D.Physical = {{0, V::EPhysicalDestination::Throttle}, {1, V::EPhysicalDestination::Brake}, {2, V::EPhysicalDestination::Steering}};
        return V::FInputActionContract::Create(std::move(D)); }();
    return C;
}
static V::FInputFrame MakeFrame(I::FFrameNumber N)
{
    V::FInputFrameData D; D.ConsumptionFrame = N; D.SourceSequence = N ? 2 : 1; D.DeviceGeneration = {1};
    D.Producer = {I::EProducerKind::Device, 99}; D.StreamEpoch = {7}; D.Reset = N < 2;
    if (N) { D.Values[0] = 220; D.ActiveMask = 1; }
    return V::FInputFrame(Contract(), std::move(D));
}
struct FControl
{
    const std::array<V::FInputFrame, 4> Sealed{MakeFrame(0), MakeFrame(1), MakeFrame(2), MakeFrame(3)};
    std::optional<V::FInputFrame> BadBaseline;
    I::FFrameNumber Next = 0;
    int Freeze = 0, Produce = 0, Close = 0, LegacyLifecycle = 0, Preparations = 0, Finalize = 0, Release = 0, Aborts = 0;
    int Destroyed = 0, DestroyedWhileReserved = 0;
    bool Paused = false, Reserved = false, Fenced = false, Frozen = false, Delivered = false;
    bool Supported = true, RejectFinalization = false, ThrowFinalization = false, ThrowPrepare = false;
    V::FSameFrameInputOwner* ReenterPrepare = nullptr;
};
// Authored, sealed LOCAL scenario fixture. It has no remote/network transport
// and is never used to relabel/admit an ExactRemote producer.
class FSealedScenario final : public V::IInputProducer, public V::IInputProducerPollFence
{
    class FResume final : public V::IPreparedInputResume
    {
    public:
        explicit FResume(std::shared_ptr<FControl> S) : State(std::move(S)) {}
        ~FResume() override { Abort(); }
        const V::FInputFrame& Baseline() const noexcept override
        { return State->BadBaseline ? *State->BadBaseline : State->Sealed[State->Next]; }
        bool PrepareFinalization() override
        {
            if (State->ThrowFinalization) throw std::runtime_error("before fence");
            if (State->RejectFinalization) return false;
            State->Fenced = true; return true;
        }
        void Finalize() noexcept override { ++State->Finalize; State->Paused = false; Finalized = true; }
        void Release() noexcept override
        { if (Done) return; ++State->Release; State->Fenced = false; State->Reserved = false; Done = true; }
        void Abort() noexcept override
        { if (Done) return; if (Finalized) { Release(); return; } ++State->Aborts; State->Fenced = false; State->Reserved = false; Done = true; }
    private:
        std::shared_ptr<FControl> State;
        bool Finalized = false, Done = false;
    };
public:
    explicit FSealedScenario(std::shared_ptr<FControl> S) : State(std::move(S)), C(Contract()) {}
    ~FSealedScenario() override { ++State->Destroyed; if (State->Reserved || State->Fenced) ++State->DestroyedWhileReserved; }
    V::IInputProducerPollFence* PollFence() noexcept override { return this; }
    bool SupportsPollFence() const noexcept override { return true; }
    V::EProducerContract GetProducerContract() const noexcept override { return V::EProducerContract::ExactScenario; }
    const std::shared_ptr<const V::FInputActionContract>& GetContract() const override { return C; }
    std::unique_ptr<V::IPreparedInputResume> PrepareResumeAtBoundary(I::FFrameNumber N) override
    {
        ++State->Preparations;
        if (State->ThrowPrepare) throw std::runtime_error("source prepare failure");
        if (!State->Supported || !State->Paused || State->Reserved || State->Frozen || N != State->Next || N >= 4) return {};
        if (State->ReenterPrepare) State->ReenterPrepare->Poll(N);
        auto R = std::make_unique<FResume>(State); // allocate BEFORE reserving
        State->Reserved = true; return R;
    }
    std::optional<V::FInputPollCutoff> FreezeForOwner(I::FFrameNumber N) override
    {
        ++State->Freeze;
        if (State->Reserved || State->Fenced || State->Paused || State->Frozen || N != State->Next || N >= 4) return {};
        State->Frozen = true; State->Delivered = false;
        const auto& D = State->Sealed[N].GetData(); return V::FInputPollCutoff{D.SourceSequence, D.DeviceGeneration.Value, N + 1};
    }
    std::optional<V::FInputFrame> Produce(I::FFrameNumber N) override
    { ++State->Produce; if (!State->Frozen || State->Delivered || N != State->Next) return {}; State->Delivered = true; return State->Sealed[N]; }
    bool CloseFrozenCutoff(const V::FInputPollCutoff&) noexcept override
    { ++State->Close; if (!State->Frozen || !State->Delivered) return false; State->Frozen = false; ++State->Next; return true; }
    V::ELifecycleResult SetLifecyclePaused(bool P) override
    { ++State->LegacyLifecycle; if (State->Reserved || State->Fenced) return V::ELifecycleResult::Rejected; State->Paused = P; return V::ELifecycleResult::Applied; }
private:
    std::shared_ptr<FControl> State;
    std::shared_ptr<const V::FInputActionContract> C;
};
static V::FOwnerInputBinding Binding()
{
    V::FOwnerInputBinding B; B.AdapterId = 1; B.Contract = Contract(); B.Epoch = {7};
    B.Producer = {I::EProducerKind::Device, 99}; B.ProducerContract = V::EProducerContract::ExactScenario; return B;
}
static void Complete(V::FSameFrameInputOwner& O, I::FFrameNumber N)
{
    auto P = O.Poll(N); CHECK(P.Status == V::EOwnerInputStatus::Ready && P.Token);
    CHECK(O.Install(*P.Token)); auto Installed = O.ReadInstalled(*P.Token); CHECK(Installed);
    CHECK(Installed->Input.GetData().ConsumptionFrame == N); CHECK(O.BeginStep(*P.Token, N));
    CHECK(O.CanComplete(*P.Token, N)); CHECK(O.CompleteStep(*P.Token, N));
}
struct FFixture
{
    std::shared_ptr<FControl> Source = std::make_shared<FControl>();
    std::unique_ptr<V::FSameFrameInputOwner> Owner = V::FSameFrameInputOwner::Create(std::make_unique<FSealedScenario>(Source), Binding());
    FFixture() { CHECK(Owner); Complete(*Owner, 0); CHECK(Owner->PauseAtBoundary()); }
};
static void PrepareAbortRetryAndHeldFinalization()
{
    FFixture F; const auto Poll = F.Owner->GetPollSerial(); const auto Freeze = F.Source->Freeze;
    auto Last = F.Owner->ReadCompleted(0); auto P = F.Owner->PrepareResumeAtBoundary(1, 1);
    CHECK(P.Status == V::EOwnerInputStatus::Ready && P.Token); CHECK(F.Owner->GetPhase() == V::EOwnerInputPhase::Paused);
    CHECK(F.Source->Next == 1 && F.Source->Freeze == Freeze && F.Owner->GetPollSerial() == Poll);
    CHECK(F.Owner->ReadCompleted(0) == Last && !F.Owner->ReadLatest());
    const auto Baseline = F.Owner->ReadPreparedResumeBaseline(*P.Token); CHECK(Baseline && Baseline->GetData().Reset && Baseline->GetData().SourceSequence == 2);
    CHECK(F.Owner->Poll(1).Status == V::EOwnerInputStatus::Busy); CHECK(!F.Owner->ResumeAtBoundary() && !F.Owner->PauseAtBoundary());
    CHECK(!F.Owner->RetireAtBoundary() && !F.Owner->BeginReplay(0, 0));
    auto NewB = Binding(); NewB.Epoch = {8};
    CHECK(!F.Owner->RebindAtBoundary(std::make_unique<FSealedScenario>(std::make_shared<FControl>()), NewB, 1));
    CHECK(F.Owner->PrepareResumeAtBoundary(1, 2).Status == V::EOwnerInputStatus::Busy);
    CHECK(F.Owner->AbortPreparedResume(*P.Token)); CHECK(!F.Owner->AbortPreparedResume(*P.Token)); CHECK(!F.Source->Reserved && F.Source->Paused);
    auto Next = F.Owner->PrepareResumeAtBoundary(1, 1); CHECK(Next.Token);
    CHECK(!F.Owner->AbortPreparedResume(*P.Token)); // stale token cannot release new reservation
    CHECK(F.Owner->PrepareResumeFinalization(*Next.Token)); CHECK(F.Owner->PrepareResumeFinalization(*Next.Token));
    CHECK(F.Owner->IsResumeFinalizationPrepared(*Next.Token) && F.Source->Fenced);
    Allocations = 0; Budget = 0;
    F.Owner->FinalizePreparedResume();
    Budget = -1; CHECK(Allocations == 0 && F.Source->Finalize == 1);
    CHECK(F.Source->Fenced && F.Source->Reserved && !F.Source->Paused && F.Source->Next == 1);
    CHECK(F.Owner->GetPhase() == V::EOwnerInputPhase::Idle && F.Owner->Poll(1).Status == V::EOwnerInputStatus::Busy);
    CHECK(!F.Owner->BeginReplay(0, 0) && !F.Owner->PauseAtBoundary() && !F.Owner->RetireAtBoundary());
    CHECK(!F.Owner->AbortPreparedResume(*Next.Token) && !F.Owner->ReadPreparedResumeBaseline(*Next.Token));
    // This fixture marks the publication point; it runs no physics/world restore.
    Allocations = 0; Budget = 0; F.Owner->ReleasePreparedResume(); Budget = -1;
    CHECK(Allocations == 0 && !F.Source->Fenced && !F.Source->Reserved && F.Source->Release == 1);
    CHECK(F.Owner->GetNextFrame() == 1 && F.Owner->GetPollSerial() == Poll && F.Source->Close == 1);
    Complete(*F.Owner, 1); CHECK(F.Source->Next == 2 && F.Owner->ReadLatest()->Epoch == 7);
    CHECK(F.Owner->ReadLatest()->Snapshot->Input.GetData().Reset && F.Owner->ReadLatest()->Snapshot->Applied[0] == 220);
    Complete(*F.Owner, 2); CHECK(F.Owner->ReadCompleted(0) == Last);
}
static void AddressContinuityAndGenerationFailures()
{
    for (int Case = 0; Case < 7; ++Case)
    {
        FFixture F; auto Data = MakeFrame(1).GetData();
        switch (Case) { case 0: Data.Reset = false; break; case 1: Data.SourceSequence = 1; break; case 2: ++Data.ConsumptionFrame; break;
                        case 3: ++Data.Producer.Id; break; case 4: ++Data.StreamEpoch.Value; break; case 5: Data.DeviceGeneration.Value = 0; break;
                        case 6: Data.Values[0] = 300; break; }
        F.Source->BadBaseline.emplace(Contract(), Data); const auto Before = F.Owner->GetPollSerial();
        auto P = F.Owner->PrepareResumeAtBoundary(1, 1); CHECK(!P.Token && P.Status == V::EOwnerInputStatus::ResyncRequired);
        CHECK(!F.Source->Reserved && F.Source->Paused && F.Source->Next == 1 && F.Owner->GetPollSerial() == Before);
        CHECK(F.Owner->GetPhase() == V::EOwnerInputPhase::Paused);
        F.Source->BadBaseline.reset(); P = F.Owner->PrepareResumeAtBoundary(1, 1); CHECK(P.Token); CHECK(F.Owner->AbortPreparedResume(*P.Token));
    }
    FFixture F; CHECK(F.Owner->PrepareResumeAtBoundary(2, 1).Status == V::EOwnerInputStatus::WrongFrame);
    CHECK(F.Owner->PrepareResumeAtBoundary(1, 0).Status == V::EOwnerInputStatus::InvalidToken);
    auto P = F.Owner->PrepareResumeAtBoundary(1, 1); CHECK(P.Token); CHECK(F.Owner->PrepareResumeFinalization(*P.Token));
    F.Owner->FinalizePreparedResume(); F.Owner->ReleasePreparedResume(); CHECK(F.Owner->PauseAtBoundary());
    CHECK(F.Owner->PrepareResumeAtBoundary(1, 1).Status == V::EOwnerInputStatus::InvalidToken);
    auto Fresh = F.Owner->PrepareResumeAtBoundary(1, 2); CHECK(Fresh.Token); CHECK(F.Owner->AbortPreparedResume(*Fresh.Token));
}
static void SourceAndAllocationFailuresRemainPrecommit()
{
    for (int Case = 0; Case < 4; ++Case)
    {
        FFixture F; if (Case == 0) F.Source->Supported = false; if (Case == 1) F.Source->ThrowPrepare = true;
        auto P = F.Owner->PrepareResumeAtBoundary(1, 1);
        if (Case < 2) CHECK(!P.Token);
        else { CHECK(P.Token); if (Case == 2) F.Source->RejectFinalization = true; else F.Source->ThrowFinalization = true;
               CHECK(!F.Owner->PrepareResumeFinalization(*P.Token)); CHECK(F.Owner->AbortPreparedResume(*P.Token)); }
        CHECK(F.Source->Next == 1 && F.Source->Paused && !F.Source->Reserved && !F.Source->Fenced && F.Source->Close == 1);
        CHECK(F.Owner->GetPhase() == V::EOwnerInputPhase::Paused);
    }
    bool Success = false; int Failed = 0;
    for (int N = 0; N < 16; ++N)
    {
        FFixture F; Budget = N;
        auto P = F.Owner->PrepareResumeAtBoundary(1, 1);
        Budget = -1;
        CHECK(F.Owner->GetPhase() == V::EOwnerInputPhase::Paused && F.Source->Next == 1 && F.Source->Close == 1);
        if (P.Token) { CHECK(F.Owner->AbortPreparedResume(*P.Token)); Success = true; break; }
        ++Failed; CHECK(!F.Source->Reserved && !F.Source->Fenced);
        auto Retry = F.Owner->PrepareResumeAtBoundary(1, 1); CHECK(Retry.Token); CHECK(F.Owner->AbortPreparedResume(*Retry.Token));
    }
    CHECK(Success && Failed >= 3);
    FFixture F; F.Source->ReenterPrepare = F.Owner.get(); auto P = F.Owner->PrepareResumeAtBoundary(1, 1);
    CHECK(!P.Token && !F.Source->Reserved && F.Owner->GetPhase() == V::EOwnerInputPhase::Quarantined);
}
static void OwnerThreadPresentationAndDestruction()
{
    FFixture F; auto P = F.Owner->PrepareResumeAtBoundary(1, 1); CHECK(P.Token);
    bool Abort = true, Finalize = true; V::EOwnerInputStatus Status{};
    std::thread Other([&] { Status = F.Owner->PrepareResumeAtBoundary(1, 2).Status;
                           Abort = F.Owner->AbortPreparedResume(*P.Token); Finalize = F.Owner->PrepareResumeFinalization(*P.Token); }); Other.join();
    CHECK(Status == V::EOwnerInputStatus::WrongOwner && !Abort && !Finalize);
    { I::FPresentationInputScope Scope; CHECK(!F.Owner->AbortPreparedResume(*P.Token) && !F.Owner->PrepareResumeFinalization(*P.Token)); }
    CHECK(F.Owner->PrepareResumeFinalization(*P.Token)); F.Owner.reset();
    CHECK(!F.Source->Reserved && !F.Source->Fenced && F.Source->Paused && F.Source->Destroyed == 1 && F.Source->DestroyedWhileReserved == 0);
}
class FRemoteAdmissionProbe final : public V::IInputProducer
{
public:
    bool SupportsPollFence() const noexcept override { return true; }
    V::EProducerContract GetProducerContract() const noexcept override { return V::EProducerContract::ExactRemote; }
    const std::shared_ptr<const V::FInputActionContract>& GetContract() const override { return C; }
    std::optional<V::FInputFrame> Produce(I::FFrameNumber) override { return {}; }
private:
    std::shared_ptr<const V::FInputActionContract> C = Contract();
};
static void NativeRemoteGuardAndLegacyPathStayClosedOrCompatible()
{
    auto B = Binding(); B.ProducerContract = V::EProducerContract::ExactRemote;
    CHECK(!V::FSameFrameInputOwner::Create(std::make_unique<FRemoteAdmissionProbe>(), B));
    FFixture F; F.Source->Supported = false;
    CHECK(!F.Owner->PrepareResumeAtBoundary(1, 1).Token); CHECK(F.Owner->ResumeAtBoundary()); Complete(*F.Owner, 1);
}
#include "ReplayStartTests.inc"
int main()
{
    try
    {
        PreparedReplayIsNotLifecycleResume(); ReplayPreparationAbortsWithoutConsumingHistoryOrSource(); RealDeviceMapperFilterReplayedWithoutRawAcquisition();
        PrepareAbortRetryAndHeldFinalization(); AddressContinuityAndGenerationFailures();
        SourceAndAllocationFailuresRemainPrecommit(); OwnerThreadPresentationAndDestruction(); NativeRemoteGuardAndLegacyPathStayClosedOrCompatible();
        std::cout << "PASS " << Checks << " checks; prepared historical replay start; INPUT-only; real owner derivative; ExactRemote guard unchanged; native world NOT_RUN\n"; return 0;
    }
    catch (const std::exception& E) { Budget = -1; std::cerr << "FAIL " << Checks << " " << E.what() << '\n'; return 1; }
}
