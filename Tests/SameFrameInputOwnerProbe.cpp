#include "IAmSpeed/Input/SameFrameInputOwner.h"
#include "IAmSpeed/Input/Testing/TestInputProducerV2.h"
#include <condition_variable>
#include <functional>
#include <iostream>
#include <cstdlib>
#include <new>

static bool FailAllocations = false;
void* operator new(std::size_t N) { if (FailAllocations) throw std::bad_alloc(); if (auto* P = std::malloc(N ? N : 1)) return P; throw std::bad_alloc(); }
void operator delete(void* P) noexcept { std::free(P); }
void operator delete(void* P, std::size_t) noexcept { std::free(P); }
void* operator new[](std::size_t N) { return ::operator new(N); }
void operator delete[](void* P) noexcept { ::operator delete(P); }
void operator delete[](void* P, std::size_t) noexcept { ::operator delete(P); }

using namespace Speed::Input::V2;
using namespace Speed::Input;
using VFrame = Speed::Input::V2::FInputFrame;
using VProducer = Speed::Input::V2::IInputProducer;
static unsigned Checks = 0;
static void Check(bool V, const char* Why) { ++Checks; if (!V) { std::cerr << "FAIL " << Why << '\n'; std::exit(1); } }
static constexpr FActionId Jump = 3, Powerslide = 4, SwitchCam = 5;
static std::shared_ptr<const FInputActionContract> MakeContract()
{
    FInputActionContractDescription D; D.Revision = {1}; D.Actions = FInputActionContract::BaseActions();
    for (FActionId I = Jump; I <= SwitchCam; ++I)
    { FActionDefinition A; A.Id = I; A.Owner = "SkyLeague"; A.Name = I == Jump ? "Jump" : I == Powerslide ? "Powerslide" : "SwitchCam"; A.Wiring = EActionWiring::Wired; D.Actions.push_back(A); }
    D.Physical = {{0, EPhysicalDestination::Throttle}, {1, EPhysicalDestination::Brake}, {2, EPhysicalDestination::Steering}};
    return FInputActionContract::Create(D);
}
static FOwnerInputBinding Bind(const std::shared_ptr<const FInputActionContract>& C, std::uint64_t Epoch = 1)
{ return {101, {EProducerKind::Device, 7}, {Epoch}, C, {}}; }
static std::vector<VFrame> Timeline(const std::shared_ptr<const FInputActionContract>& C,
    FFrameNumber First = 10, std::uint64_t Epoch = 1)
{
    std::vector<VFrame> Frames;
    for (FFrameNumber I = 0; I < 5; ++I)
    {
        FInputFrameData D; D.Producer = {EProducerKind::Device, 7}; D.StreamEpoch = {Epoch}; D.DeviceGeneration = {1};
        D.ConsumptionFrame = First + I; D.SourceSequence = 10 + I; D.Reset = I == 0;
        if (I >= 1) { D.Values[Throttle] = 255; D.ActiveMask = 1; }
        if (I == 1) D.Transitions.push_back({Throttle, ETransition::Started, 255, {11, 0}});
        if (I == 1 || I == 2)
        {
            for (FActionId A = Jump; A <= SwitchCam; ++A)
            {
                const auto O = static_cast<std::uint32_t>(D.Transitions.size());
                D.Transitions.push_back({A, ETransition::Started, 1, {D.SourceSequence, O}});
                D.Transitions.push_back({A, ETransition::Completed, 0, {D.SourceSequence, O + 1}});
            }
        }
        Frames.emplace_back(C, D);
    }
    return Frames;
}
// Test adapter: the existing sealed TestInputProducer is the sole source of
// action frames. The adapter provides the explicit immutable cutoff contract.
class FFencedFixture final : public VProducer, public IInputProducerPollFence
{
public:
    IInputProducerPollFence* PollFence() noexcept override { return this; }
    bool SupportsPollFence() const noexcept override { return true; }
    EProducerContract GetProducerContract() const noexcept override { return EProducerContract::Device; }
    FFencedFixture(std::shared_ptr<const FInputActionContract> C, std::vector<VFrame> F)
        : Contract(std::move(C)), Frames(std::move(F))
    {
        const auto& D = Frames.front().GetData();
        Source = Speed::Input::V2::FTestInputProducer::Create(Contract, D.StreamEpoch, D.Producer, D.ConsumptionFrame, Frames);
        Check(bool(Source), "sealed TestInputProducer valid");
    }
    mutable std::function<void()> OnContract;
    std::function<void()> OnProduce, OnClose;
    std::function<VFrame(VFrame)> Alter;
    unsigned Polls = 0, Freezes = 0;
    ELifecycleResult Lifecycle = ELifecycleResult::Unaffected;
    bool Closed = false;
    const std::shared_ptr<const FInputActionContract>& GetContract() const override
    { if (OnContract) OnContract(); return Contract; }
    std::optional<FInputPollCutoff> FreezeForOwner(FFrameNumber N) override
    {
        ++Freezes;
        for (const auto& F : Frames) if (F.GetData().ConsumptionFrame == N)
        { Frozen = FInputPollCutoff{F.GetData().SourceSequence, F.GetData().DeviceGeneration.Value, 1}; return Frozen; }
        return {};
    }
    std::optional<VFrame> Produce(FFrameNumber N) override
    { ++Polls; if (OnProduce) OnProduce(); auto F = Source->Produce(N); if (F && Alter) return Alter(*F); return F; }
    bool CloseFrozenCutoff(const FInputPollCutoff& C) noexcept override
    { if (OnClose) OnClose(); const bool Valid = Frozen && C.Sequence == Frozen->Sequence && !Closed; Frozen.reset(); return Valid; }
    ELifecycleResult SetLifecyclePaused(bool) override { return Lifecycle; }
    ELifecycleResult CancelLifecycle() override { return Lifecycle; }
private:
    std::shared_ptr<const FInputActionContract> Contract;
    std::vector<VFrame> Frames;
    std::unique_ptr<Speed::Input::V2::FTestInputProducer> Source;
    std::optional<FInputPollCutoff> Frozen;
};
static std::shared_ptr<const FOwnerInputReceipt> Step(FSameFrameInputOwner& Owner, FFrameNumber N)
{
    auto P = Owner.Poll(N); Check(P.Status == EOwnerInputStatus::Ready && P.Token.has_value(), "poll exact frame");
    const auto Old = Owner.ReadLatest();
    FailAllocations = true;
    const bool Installed = Owner.Install(*P.Token);
    auto Snapshot = Owner.ReadInstalled(*P.Token);
    const bool Begun = Owner.BeginStep(*P.Token, N);
    const bool Completed = Owner.CompleteStep(*P.Token, N);
    FailAllocations = false;
    Check(Installed && Snapshot && Begun && Completed, "install begin complete no allocations");
    auto R = Owner.ReadLatest(); if (!R || R == Old) R = Owner.ReadReplayReceipt();
    Check(R && R->AppliedBeforeStep == N && R->StepFrame == N, "same N witness");
    Check(!Owner.CompleteStep(*P.Token, N) && !Owner.Install(*P.Token), "token single use");
    return R;
}
static void ExactFramesAndReplay()
{
    const auto C = MakeContract(); auto B = Bind(C); B.Processing.Step[Throttle] = 16;
    auto Source = std::make_unique<FFencedFixture>(C, Timeline(C)); auto* Observe = Source.get();
    auto O = FSameFrameInputOwner::Create(std::move(Source), B, 10); Check(bool(O), "create bound owner");
    Check(O->Poll(9).Status == EOwnerInputStatus::WrongFrame && O->Poll(11).Status == EOwnerInputStatus::WrongFrame && Observe->Polls == 0, "N-1 N+1 no poll");
    Step(*O, 10);
    auto P = O->Poll(11); Check(P.Status == EOwnerInputStatus::Ready, "prepare N");
    Check(O->Poll(11).Status == EOwnerInputStatus::Busy && Observe->Polls == 2, "no double poll pending");
    Check(!O->CompleteStep(*P.Token, 11) && !O->BeginStep(*P.Token, 11), "cannot complete or step before install");
    Check(O->Install(*P.Token), "install before first reader");
    auto S = O->ReadInstalled(*P.Token);
    Check(S && S->Applied[Throttle] == 16 && S->Requested[Throttle] == 255, "slew first increment on N");
    Check(S->Input.GetData().Transitions.size() == 7 && S->Input.GetData().Values[Jump] == 0, "short tap edges retained despite neutral end");
    Check(O->ReadLatest()->StepFrame == 10, "no pre-step success publication");
    { FPresentationInputScope Scope; Check(!O->ReadInstalled(*P.Token) && !O->Install(*P.Token), "presentation cannot inspect installed unpublished input"); }
    Check(!O->Install(*P.Token) && !O->BeginStep(*P.Token, 12), "no rewrite and wrong target step refused");
    Check(O->BeginStep(*P.Token, 11) && O->ReadInstalled(*P.Token) == S && O->CompleteStep(*P.Token, 11), "same immutable installed object consumed");
    Step(*O, 12); Step(*O, 13); Step(*O, 14);
    Check(Observe->Polls == 5 && O->ReadLatest()->Snapshot->Applied[Throttle] == 64, "one increment each frame");
    const auto Live = O->ReadLatest(); const auto Fingerprint = O->GetBindingFingerprint();
    Check(O->BeginReplay(11, 14), "begin retained replay");
    for (FFrameNumber N = 11; N <= 14; ++N)
    {
        auto R = Step(*O, N);
        Check(R->Replay && R->Snapshot->Applied == O->ReadCompleted(N)->Snapshot->Applied, "replay applied identical internal receipt");
    }
    Check(Observe->Polls == 5 && O->ReadLatest() == Live && O->GetNextFrame() == 15, "replay no live poll/publication/cursor advance");
    Check(O->Poll(14).Status == EOwnerInputStatus::ReplayComplete && O->EndReplay(), "replay end boundary");
    Check(Fingerprint == O->GetBindingFingerprint(), "binding immutable across replay");
    bool ForeignRejected = false;
    std::thread Other([&] { ForeignRejected = O->Poll(15).Status == EOwnerInputStatus::WrongOwner && !O->ReadLatest() && O->GetPollSerial() == 0; });
    Other.join(); Check(ForeignRejected, "wrong owner cannot poll or observe mutable state");
}
static void FailuresAndLifecycle()
{
    const auto C = MakeContract();
    for (int Phase = 0; Phase < 3; ++Phase)
    {
        auto O = FSameFrameInputOwner::Create(std::make_unique<FFencedFixture>(C, Timeline(C)), Bind(C), 10);
        Step(*O, 10); auto P = O->Poll(11);
        if (Phase >= 1) Check(O->Install(*P.Token), "abort fixture installed");
        if (Phase >= 2) Check(O->BeginStep(*P.Token, 11), "abort fixture stepping");
        Check(O->Abort(*P.Token) && !O->ReadLatest() && !O->ReadInstalled(*P.Token) && !O->ReadCompleted(11), "quarantine no partial success");
        Check(O->GetNextFrame() == 11 && O->Poll(11).Status == EOwnerInputStatus::Quarantined && O->ReadCompleted(10), "quarantine retains only committed history");
        Check(O->RebindAtBoundary(std::make_unique<FFencedFixture>(C, Timeline(C, 11, 2)), Bind(C, 2), 11), "explicit new epoch recovery");
        Step(*O, 11);
    }
    auto F = Timeline(C); auto D = F[2].GetData(); D.Reset = true; D.Transitions.clear(); D.DeviceGeneration = {2};
    D.Values[Jump] = 1; D.ActiveMask |= (1u << Jump); F[2] = VFrame(C, D); F.erase(F.begin() + 3, F.end());
    auto B = Bind(C); B.Processing.Step[Throttle] = 16;
    auto O = FSameFrameInputOwner::Create(std::make_unique<FFencedFixture>(C, F), B, 10);
    Step(*O, 10); Step(*O, 11);
    Check(O->PauseAtBoundary() && O->ReadBoundaryApplied()[Throttle] == 0 && !O->ReadLatest(), "pause neutral immediate");
    Check(O->Poll(12).Status == EOwnerInputStatus::Paused && O->ResumeAtBoundary(), "paused cannot poll");
    auto R = Step(*O, 12); Check(R->Snapshot->Applied[Throttle] == 255 && R->Snapshot->Applied[Jump] == 1
        && R->Snapshot->Input.GetData().Transitions.empty(), "fresh held resume immediate without Started");
    auto Bad = std::make_unique<FFencedFixture>(C, Timeline(C)); auto* PBad = Bad.get();
    auto Q = FSameFrameInputOwner::Create(std::move(Bad), Bind(C), 10);
    PBad->Lifecycle = static_cast<ELifecycleResult>(255); Check(!Q->PauseAtBoundary() && Q->GetPhase() == EOwnerInputPhase::Quarantined, "invalid lifecycle enum refused");
    for (int Kind = 0; Kind < 5; ++Kind)
    {
        auto X = std::make_unique<FFencedFixture>(C, Timeline(C));
        X->Alter = [&, Kind](VFrame Frame)
        { auto Data = Frame.GetData(); if (Kind == 0) ++Data.ConsumptionFrame; if (Kind == 1) ++Data.StreamEpoch.Value;
          if (Kind == 2) ++Data.Producer.Id; if (Kind == 3) ++Data.SourceSequence; if (Kind == 4) Data.Reset = false; return VFrame(C, Data); };
        auto A = FSameFrameInputOwner::Create(std::move(X), Bind(C), 10);
        Check(A->Poll(10).Status == EOwnerInputStatus::ResyncRequired && !A->ReadLatest(), "bad address epoch producer cutoff baseline fail closed");
    }
    auto Policy = Bind(C); Policy.Processing.Step[Jump] = 1;
    Check(!FSameFrameInputOwner::Create(std::make_unique<FFencedFixture>(C, Timeline(C)), Policy, 10), "Boolean smoothing rejected");
    auto LegacyTest = Speed::Input::V2::FTestInputProducer::Create(C, {1}, {EProducerKind::Device,7}, 10, Timeline(C));
    Check(!FSameFrameInputOwner::Create(std::move(LegacyTest), Bind(C), 10), "Test cannot masquerade as Device contract");
}
static void ReentrancyAndRecovery()
{
    const auto C = MakeContract();
    auto S = std::make_unique<FFencedFixture>(C, Timeline(C)); auto* Raw = S.get();
    auto O = FSameFrameInputOwner::Create(std::move(S), Bind(C), 10);
    Raw->OnProduce = [&] { Check(O->Poll(10).Status == EOwnerInputStatus::Quarantined, "producer reentry quarantines"); };
    Check(O->Poll(10).Status == EOwnerInputStatus::Quarantined && Raw->Polls == 1, "no second reentrant producer call");
    auto New = std::make_unique<FFencedFixture>(C, Timeline(C,10,2));
    New->OnContract = [&] { O->Poll(10); };
    Check(!O->RebindAtBoundary(std::move(New), Bind(C,2),10), "reentrant contract cannot revive quarantined session");
    Raw->OnProduce = {};
    Check(O->RebindAtBoundary(std::make_unique<FFencedFixture>(C, Timeline(C,10,2)), Bind(C,2),10), "subsequent valid rebind");
    Step(*O,10); Check(O->BeginReplay(10,10), "replay abort fixture");
    auto T = O->Poll(10); Check(T.Status == EOwnerInputStatus::Ready && O->Abort(*T.Token), "replay abort quarantined");
    Check(O->RebindAtBoundary(std::make_unique<FFencedFixture>(C, Timeline(C,10,3)), Bind(C,3),10), "new epoch recovery after replay abort");
    auto BadClose = std::make_unique<FFencedFixture>(C,Timeline(C,10,4)); auto* BC = BadClose.get();
    Check(O->RebindAtBoundary(std::move(BadClose),Bind(C,4),10), "replace bound producer");
    BC->OnClose = [BC] { BC->Closed = true; };
    Check(O->Poll(10).Status == EOwnerInputStatus::ResyncRequired, "lifecycle invalidation at atomic cutoff close rejects");
}

// A source-local inbox with a real mutex-frozen evidence snapshot. Appending is
// NOT physical input admission. Only the owner Freeze/Produce reads it for N.
class FInbox final : public VProducer, public IInputProducerPollFence
{
public:
    IInputProducerPollFence* PollFence() noexcept override { return this; }
    bool SupportsPollFence() const noexcept override { return true; }
    EProducerContract GetProducerContract() const noexcept override { return EProducerContract::Device; }
    explicit FInbox(std::shared_ptr<const FInputActionContract> C) : Contract(std::move(C)) {}
    std::function<void()> AfterFreeze;
    unsigned Polls = 0;
    bool Overflow = false;
    void Append(FActionId Action, bool Held)
    {
        std::lock_guard<std::mutex> Lock(Gate); ++Sequence;
        Values[Action] = Held ? 1 : 0;
        if (Held) Active |= 1u << Action; else Active &= ~(1u << Action);
        Events.push_back({Action, Held ? ETransition::Started : ETransition::Completed, Values[Action], {Sequence,0}});
        if (Events.size() > MaxEdges) Overflow = true;
    }
    const std::shared_ptr<const FInputActionContract>& GetContract() const override { return Contract; }
    std::optional<FInputPollCutoff> FreezeForOwner(FFrameNumber N) override
    {
        std::lock_guard<std::mutex> Lock(Gate); if (Overflow || Frozen) return {};
        FInputFrameData D; D.SourceSequence = Sequence; D.ConsumptionFrame = N; D.Producer = {EProducerKind::Device,7};
        D.StreamEpoch = {1}; D.DeviceGeneration = {1}; D.Reset = First; D.Values = Values; D.ActiveMask = Active;
        if (!First) D.Transitions = Events;
        Frozen.emplace(Contract,D); FrozenCutoff = {Sequence,1,1};
        return FrozenCutoff;
    }
    std::optional<VFrame> Produce(FFrameNumber) override { ++Polls; if (AfterFreeze) AfterFreeze(); return Frozen; }
    bool CloseFrozenCutoff(const FInputPollCutoff& Cutoff) noexcept override
    {
        std::lock_guard<std::mutex> Lock(Gate);
        const bool Valid = Frozen && !Overflow && Cutoff.Sequence == FrozenCutoff.Sequence && Cutoff.Generation == 1 && Cutoff.LifecycleFence == 1;
        if (Valid)
        {
            Events.erase(std::remove_if(Events.begin(), Events.end(), [&](const auto& E) { return E.Order.Sequence <= Cutoff.Sequence; }),Events.end());
            First = false;
        }
        Frozen.reset(); return Valid;
    }
private:
    std::shared_ptr<const FInputActionContract> Contract;
    mutable std::mutex Gate;
    std::uint64_t Sequence = 1;
    FActionValues Values{};
    std::uint32_t Active = 0;
    bool First = true;
    std::vector<FActionTransition> Events;
    std::optional<VFrame> Frozen;
    FInputPollCutoff FrozenCutoff;
};
static void CutoffConcurrency()
{
    const auto C = MakeContract(); auto S = std::make_unique<FInbox>(C); auto* Inbox = S.get();
    auto O = FSameFrameInputOwner::Create(std::move(S),Bind(C),0); Step(*O,0);
    Inbox->Append(Jump,true);
    std::mutex Gate; std::condition_variable Wake; bool Frozen = false, Arrived = false;
    Inbox->AfterFreeze = [&]
    { std::unique_lock<std::mutex> Lock(Gate); Frozen = true; Wake.notify_all(); Wake.wait(Lock,[&]{return Arrived;}); };
    std::thread Arrival([&]
    { { std::unique_lock<std::mutex> Lock(Gate); Wake.wait(Lock,[&]{return Frozen;}); }
      Inbox->Append(Jump,false); { std::lock_guard<std::mutex> Lock(Gate); Arrived = true; } Wake.notify_all(); });
    auto R1 = Step(*O,1); Arrival.join(); Inbox->AfterFreeze = {};
    Check(R1->Snapshot->Input.GetData().Values[Jump] == 1 && R1->Snapshot->CutoffSequence == 2, "before cutoff included after cutoff excluded");
    auto R2 = Step(*O,2);
    Check(R2->Snapshot->Input.GetData().Values[Jump] == 0 && R2->Snapshot->Input.GetData().Transitions.size() == 1
        && R2->Snapshot->CutoffSequence == 3, "late release retained for N+1");
    Inbox->Append(Powerslide,true); Inbox->Append(Powerslide,false); Inbox->Append(SwitchCam,true); Inbox->Append(SwitchCam,false);
    auto Tap = Step(*O,3); Check(Tap->Snapshot->Input.GetData().Transitions.size() == 4, "sameframe two action taps not coalesced");
    for (std::size_t I = 0; I <= MaxEdges; ++I) Inbox->Append(Jump,(I % 2)==0);
    const auto Polls = Inbox->Polls;
    Check(O->Poll(4).Status == EOwnerInputStatus::ResyncRequired && Inbox->Polls == Polls && !O->ReadLatest(), "overflow fail closed before producer poll");
}
static void BoundariesAndMalformedEvidence()
{
    const auto C = MakeContract();
    auto B = Bind(C); B.Processing.Step[Steering] = 16;
    std::vector<VFrame> Frames;
    for (FFrameNumber N = 0; N < 3; ++N)
    {
        FInputFrameData D; D.ConsumptionFrame = N; D.SourceSequence = N + 1; D.Producer = B.Producer; D.StreamEpoch = B.Epoch;
        D.DeviceGeneration = {1}; D.Reset = N == 0; D.Values[Steering] = N == 1 ? -127 : 127; D.ActiveMask = 1u << Steering;
        Frames.emplace_back(C,D);
    }
    auto O = FSameFrameInputOwner::Create(std::make_unique<FFencedFixture>(C,Frames),B,0);
    B.Processing.Step[Steering] = 127; // caller mutation cannot change binding
    Check(Step(*O,0)->Snapshot->Applied[Steering] == 127, "initial held baseline bypasses slew");
    Check(Step(*O,1)->Snapshot->Applied[Steering] == 111, "negative reversal shaped before same step");
    Check(Step(*O,2)->Snapshot->Applied[Steering] == 127, "policy copied not mutable caller data");
    auto P = std::make_unique<FFencedFixture>(C,Timeline(C)); auto* Observe = P.get();
    auto Q = FSameFrameInputOwner::Create(std::move(P),Bind(C),10); Step(*Q,10);
    Check(Q->PauseAtBoundary() && Q->ResumeAtBoundary(), "fresh required fixture boundary");
    Check(Q->Poll(11).Status == EOwnerInputStatus::ResyncRequired && Observe->Polls == 2, "ordinary nonbaseline after resume refused");
    for (int Kind = 0; Kind < 4; ++Kind)
    {
        auto S = std::make_unique<FFencedFixture>(C,Timeline(C)); auto* Source = S.get();
        auto T = FSameFrameInputOwner::Create(std::move(S),Bind(C),10); Step(*T,10);
        Source->Alter = [C,Kind](VFrame F)
        {
            auto D = F.GetData();
            if (Kind == 0) D.Transitions[1].Order = D.Transitions[0].Order;
            if (Kind == 1) D.Values[Throttle] = -1;
            if (Kind == 2) { auto Description = C->GetDescription(); ++Description.Revision.Value; return VFrame(FInputActionContract::Create(Description),D); }
            if (Kind == 3) D.DeviceGeneration.Value = 999;
            return VFrame(C,D);
        };
        Check(T->Poll(11).Status == EOwnerInputStatus::ResyncRequired && !T->ReadCompleted(11), "ordinal value fingerprint generation invalid");
    }
    auto AllocSource = std::make_unique<FFencedFixture>(C,Timeline(C)); auto* ObserveAlloc = AllocSource.get();
    auto A = FSameFrameInputOwner::Create(std::move(AllocSource),Bind(C),10);
    FailAllocations = true; auto Failed = A->Poll(10); FailAllocations = false;
    Check(Failed.Status == EOwnerInputStatus::ResyncRequired && ObserveAlloc->Polls == 1
        && !A->ReadLatest() && A->GetNextFrame() == 10, "allocation failure after poll quarantines no rewind or publish");
    auto E = FSameFrameInputOwner::Create(std::make_unique<FFencedFixture>(C,Timeline(C)),Bind(C),10);
    auto Pending = E->Poll(10); FSameFrameInputOwner::FToken Forged;
    Check(!E->Install(Forged) && !E->PauseAtBoundary() && !E->BeginReplay(10,10), "forged capability and active boundary changes refused");
    Check(!E->RebindAtBoundary(std::make_unique<FFencedFixture>(C,Timeline(C,10,2)),Bind(C,2),10), "binding immutable during reservation");
    auto Moved = std::move(*Pending.Token); Check(!E->Install(*Pending.Token) && E->Install(Moved), "moved token sole capability");
    Check(E->Abort(Moved), "close moved capability");
    std::vector<VFrame> Long;
    for (FFrameNumber N = 0; N < HistoryCapacity + 2; ++N)
    {
        FInputFrameData D; D.ConsumptionFrame = N; D.SourceSequence = N + 1; D.Producer = {EProducerKind::Device,7};
        D.StreamEpoch = {1}; D.DeviceGeneration = {1}; D.Reset = N == 0; Long.emplace_back(C,D);
    }
    auto L = FSameFrameInputOwner::Create(std::make_unique<FFencedFixture>(C,Long),Bind(C),0);
    std::shared_ptr<const FOwnerInputReceipt> Retained;
    for (FFrameNumber N = 0; N < Long.size(); ++N) { auto R = Step(*L,N); if (N == 0) Retained = R; }
    Check(!L->ReadCompleted(0) && !L->BeginReplay(0,1) && Retained->StepFrame == 0, "history exhaustion refuses replay while external immutable receipt survives");
    FInputFrameData Max; Max.ConsumptionFrame = UINT64_MAX; Max.SourceSequence = 1; Max.Producer = {EProducerKind::Device,7};
    Max.StreamEpoch = {1}; Max.DeviceGeneration = {1}; Max.Reset = true;
    auto Z = FSameFrameInputOwner::Create(std::make_unique<FFencedFixture>(C,std::vector<VFrame>{VFrame(C,Max)}),Bind(C),UINT64_MAX);
    Step(*Z,UINT64_MAX); Check(Z->Poll(UINT64_MAX).Status == EOwnerInputStatus::Exhausted, "frame exhaustion never wraps");
}
int main()
{
    static_assert(!std::is_copy_constructible_v<FSameFrameInputOwner::FToken>);
    ExactFramesAndReplay(); FailuresAndLifecycle(); ReentrancyAndRecovery(); CutoffConcurrency(); BoundariesAndMalformedEvidence();
    std::cout << "PASS SameFrameInputOwnerProbe checks=" << Checks << '\n';
}
