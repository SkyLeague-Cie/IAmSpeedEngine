#include "IAmSpeed/Input/InputSessionRegistry.h"
#include "IAmSpeed/Input/InputObservationChannel.h"
#include "IAmSpeed/Input/ActionDispatch.h"
#include <iostream>
#include <cstdlib>
#include <atomic>
#include <thread>
#include "Car/Input/SkyGameActions.h"

using namespace Speed::Input;
using namespace Speed::Input::V2;
using VFrame = Speed::Input::V2::FInputFrame;
static unsigned Checks = 0;
static void Check(bool V, const char* Why) { ++Checks; if (!V) { std::cerr << "FAIL " << Why << '\n'; std::exit(1); } }
static auto Contract()
{
    FInputActionContractDescription D; D.Revision = {1}; D.Actions = FInputActionContract::BaseActions();
    for (FActionId I = 3; I < 6; ++I) { FActionDefinition A; A.Id = I; A.Owner = "SkyLeague"; A.Name = I == 3 ? "Jump" : I == 4 ? "Powerslide" : "SwitchCam"; A.Wiring = EActionWiring::Wired; D.Actions.push_back(A); }
    D.Physical = {{0,EPhysicalDestination::Throttle},{1,EPhysicalDestination::Brake},{2,EPhysicalDestination::Steering}};
    return FInputActionContract::Create(D);
}
static FSessionDescriptor Scenario(std::uint64_t Id = 1, std::uint64_t Epoch = 1)
{
    FSessionDescriptor D; D.Id = Id; D.Epoch = Epoch; D.Controller = Id; D.Producer = Id; D.Contract = Contract();
    D.Kind = EProducerContract::ExactScenario; D.Actors = {{Id*10+1,1},{Id*10,1}};
    for (unsigned N=0; N<4; ++N)
    {
        FInputFrameData F; F.ConsumptionFrame = N; F.SourceSequence = N+1; F.StreamEpoch = {Epoch}; F.Producer = {EProducerKind::Device,Id}; F.DeviceGeneration = {1}; F.Reset = N == 0 || N == 2;
        F.Values[Throttle] = N == 0 ? 0 : 255; F.ActiveMask = N == 0 ? 0 : 1;
        if (N == 1)
        {
            F.Transitions.push_back({Throttle,ETransition::Started,255,{2,0}});
            for (FActionId A=3; A<6; ++A) { const auto O=std::uint32_t(F.Transitions.size()); F.Transitions.push_back({A,ETransition::Started,1,{2,O}}); F.Transitions.push_back({A,ETransition::Completed,0,{2,O+1}}); }
        }
        D.Scenario.emplace_back(D.Contract,F);
    }
    return D;
}
static FBoundaryCommandDescriptor Command(std::uint64_t Id, std::uint64_t Version, EBoundaryOperation Op = EBoundaryOperation::Bind)
{ FBoundaryCommandDescriptor C; C.Id=Id; C.WorkerGeneration=42; C.RegistryVersion=Version; C.Operation=Op; return C; }
static FSessionDescriptor Device(std::uint64_t Id=1, std::uint64_t Journal=7)
{
    auto D=Scenario(Id); D.Scenario.clear(); D.Kind=EProducerContract::Device; D.Journal=Journal;
    auto C=D.Contract->GetDescription(); C.Mapping={{{ERawControlKind::KeyboardUsage,4},Throttle,1}};
    D.Contract=FInputActionContract::Create(C); D.Processing.Step[Throttle]=16; return D;
}
static bool Publish(const std::shared_ptr<FRawAcquisitionJournal>& J, std::uint64_t Sequence, bool Held, bool Baseline)
{
    FRawAcquisitionBatch B; B.Count=1; auto& R=B.Readings[0]; R.DeviceId=7; R.Sequence=Sequence; R.TimestampMicroseconds=Sequence;
    R.Generation={1}; R.FreshBaseline=Baseline; R.State.Count=1; R.State.Values[0]={{ERawControlKind::KeyboardUsage,4},Held?1.0f:0.0f};
    return J->Publish(J->BeginAcquisition(),B);
}
static std::shared_ptr<const FRegistryFrame> Step(FInputSessionRegistry& R, FFrameNumber N)
{
    Check(R.PrepareFrame(N),"prepare once"); Check(!R.PrepareFrame(N),"cannot repoll pending");
    Check(!R.BeginAll(),"cannot begin before install"); Check(R.InstallAll(),"install all");
    Check(R.ReadLastFrameFailure().Reason == ERegistryFrameFailure::None, "successful install resets prior admission diagnostic");
    const auto F=R.ReadInstalled(); Check(F && F->Frame==N,"same frame installed");
    Check(R.BeginAll() && R.ValidateComplete() && R.CompleteAll(),"group complete");
    return F;
}
int main()
{
    {
        FRegistryLimits Limits; Limits.Receipts=1;
        auto Q=std::make_shared<FInputSessionCommands>(Limits);
        FInputSessionRegistry R(Q,42);
        for (std::uint64_t Id=1; Id<600; ++Id)
        {
            auto C=Command(Id,0,EBoundaryOperation::PauseAll);
            Check(Q->Submit(C)==ECommandAdmission::Enqueued,"acknowledged receipts allow arbitrarily many lifecycle commands");
            Check(!Q->Acknowledge(Id),"pending receipt cannot be removed");
            Check(R.ServiceBoundary() && Q->Read(Id)->Status==EBoundaryStatus::Applied,"receipt published before acknowledgement");
            Check(Q->Acknowledge(Id) && !Q->Read(Id),"only final copied receipt released");
            Check(Q->Submit(C)==ECommandAdmission::Rejected,"acknowledgement never permits duplicate command execution");
        }
    }

    {
        auto C=FInputActionContract::Create(Sky::Input::V2::DescribeGameActions({1},{}));
        auto Inbox=std::make_shared<FAIInputCommands>(C);
        auto Queue=std::make_shared<FInputSessionCommands>();
        FInputSessionRegistry Registry(Queue,42,{{9,{},Inbox}});
        auto Bind=Command(1,0);
        auto& D=Bind.Binding; D.Id=D.Epoch=D.Controller=D.Producer=1;
        D.Journal=9; D.Kind=EProducerContract::AI; D.Contract=C; D.Actors={{10,1}};
        Check(Queue->Submit(Bind)==ECommandAdmission::Enqueued && Registry.ServiceBoundary(),"AI owner constructed at worker boundary");
        auto F=Step(Registry,0);
        Check(F->Inputs[0].Snapshot->Applied[Throttle]==0,"absent AI decision is neutral");
        FActionValues V{}; V[Throttle]=255; V[Steering]=-127; V[Sky::Input::V2::Boost]=255;
        Check(Inbox->Publish(V),"AI timer publishes complete inert decision");
        F=Step(Registry,1);
        Check(F->Frame==1 && F->Inputs[0].Snapshot->Applied==V,"AI throttle steering boost applied exactly at N, no shared slew");
        Check(Inbox->Publish(V,{Sky::Input::V2::Jump}),"AI jump pulse accepted");
        F=Step(Registry,2);
        Check(F->Inputs[0].Snapshot->Input.GetData().Transitions.size()==2,"AI jump has one start and completion");
        Check(Inbox->Publish(V,{},FAIInputCommands::FClock::now()-std::chrono::seconds(2)),"expired decision fixture accepted as data");
        F=Step(Registry,3);
        Check(F->Inputs[0].Snapshot->Applied==FActionValues{} && F->Inputs[0].Snapshot->Input.GetData().Reset,"stale AI decision neutralizes all axes");
        auto Pause=Command(2,1,EBoundaryOperation::PauseAll);
        Check(Queue->Submit(Pause)==ECommandAdmission::Enqueued && Registry.ServiceBoundary(),"AI pause clears pending decision");
        Check(Queue->Read(2)->Sessions[0].Phase==ESessionPhase::Paused,"AI PauseAll receipt reports genuinely paused owner");
        Check(!Registry.PrepareFrame(4),"paused AI cannot poll or step");
        Check(Registry.ReadLastFrameFailure().Reason == ERegistryFrameFailure::SessionInactive,
            "paused session reported before any source poll");
        auto ResumeAI=Command(3,1,EBoundaryOperation::Resume); ResumeAI.Session=ResumeAI.Epoch=ResumeAI.ResumeGeneration=1;
        Check(Queue->Submit(ResumeAI)==ECommandAdmission::Enqueued && Registry.ServiceBoundary(),"AI controller Resume command serviced");
        Check(Queue->Read(3)->Status==EBoundaryStatus::Applied,"AI Resume is applied rather than rejected");
        F=Step(Registry,4);
        Check(F->Inputs[0].Snapshot->Applied==FActionValues{} && F->Inputs[0].Snapshot->Input.GetData().Reset,"first AI frame after Resume is neutral reset");
        Check(Inbox->Publish(V),"fresh AI decision after pause");
        F=Step(Registry,5);
        Check(F->Inputs[0].Snapshot->Applied==V,"fresh held AI axes immediately restored");
        auto Detach=Command(4,1,EBoundaryOperation::Detach); Detach.Session=Detach.Epoch=1;
        Check(Queue->Submit(Detach)==ECommandAdmission::Enqueued && Registry.ServiceBoundary(),"AI detach closes producer on worker");
        Check(!Inbox->Publish(V),"retired AI mailbox rejects late timer callback");
    }

    {
        auto C=FInputActionContract::Create(Sky::Input::V2::DescribeGameActions({1},{}));
        auto Decisions=std::make_shared<FAIInputCommands>(C);
        unsigned Calls=0;
        Check(Decisions->SetPollEvaluator([&](FFrameNumber N)
        {
            ++Calls;
            FAIInputCommands::FDecision D; D.Present=true;
            D.Values[Throttle]=N==2 ? 128 : 255;
            if (N<2) D.Pulses.push_back(Sky::Input::V2::Jump);
            return D;
        }),"AI physics evaluator installed");
        auto P=FAIInputProducer::Create(Decisions,C,{1},{EProducerKind::AI,1},0);
        Check(bool(P),"directly polled AI producer constructed");
        auto Cut=P->FreezeForOwner(0);
        auto F=P->Produce(0);
        Check(Cut && F && F->GetData().Values[Throttle]==255 &&
            F->GetData().Transitions.empty() && Calls==1,
            "AI decision computed by the first physical poll");
        Check(!Decisions->SetPollEvaluator([](FFrameNumber){ return FAIInputCommands::FDecision{}; })
            && !Decisions->Publish(FActionValues{}),
            "active AI decision source and goal cannot be replaced by a timer write");
        Check(P->CloseFrozenCutoff(*Cut),"first AI cutoff closed");
        Cut=P->FreezeForOwner(1); F=P->Produce(1);
        Check(Cut && F && F->GetData().Values[Throttle]==255 &&
            F->GetData().Transitions.size()==2 && Calls==2,
            "next physical poll computes one pulse, without duplicated reset pulse");
        Check(P->CloseFrozenCutoff(*Cut),"second AI cutoff closed");
        Check(P->SetLifecyclePaused(true)==ELifecycleResult::Applied &&
            !P->FreezeForOwner(2) && Calls==2,"paused AI cannot compute");
        Check(P->SetLifecyclePaused(false)==ELifecycleResult::Applied,"AI resumed");
        Cut=P->FreezeForOwner(2); F=P->Produce(2);
        Check(Cut && F && F->GetData().Reset && F->GetData().Values[Throttle]==128 && Calls==3,
            "first fresh resumed physical poll computes held AI decision");
        Check(P->CloseFrozenCutoff(*Cut),"resumed AI cutoff closed");
        Check(P->CancelLifecycle()==ELifecycleResult::Applied &&
            !P->FreezeForOwner(3) && Calls==3,"retired AI cannot compute");
    }
    {
        auto C=FInputActionContract::Create(Sky::Input::V2::DescribeGameActions({1},{}));
        auto Decisions=std::make_shared<FAIInputCommands>(C);
        std::atomic<bool> Entered{false}, AllowFinish{false}, Polled{false};
        std::atomic<unsigned> Calls{0};
        Check(Decisions->SetPollEvaluator([&](FFrameNumber)
        {
            ++Calls; Entered.store(true);
            while (!AllowFinish.load()) std::this_thread::yield();
            FAIInputCommands::FDecision D; D.Present=true; return D;
        }),"blocking physical evaluator installed");
        auto P=FAIInputProducer::Create(Decisions,C,{2},{EProducerKind::AI,2},0);
        std::thread Poll([&]
        {
            auto Cut=P->FreezeForOwner(0);
            if (Cut) { auto F=P->Produce(0); Polled.store(bool(F) && P->CloseFrozenCutoff(*Cut)); }
        });
        const auto Deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
        while (!Entered.load() && std::chrono::steady_clock::now()<Deadline) std::this_thread::yield();
        if (!Entered.load()) AllowFinish.store(true);
        Check(Entered.load(),"physical evaluator entered");
        Check(!Decisions->CloseAndWait(std::chrono::milliseconds(10)),
            "source teardown cannot claim quiescence during an in-flight physical read");
        AllowFinish.store(true); Poll.join();
        Check(Polled.load() && Decisions->CloseAndWait(std::chrono::milliseconds(100)) && Calls.load()==1,
            "teardown joins evaluator before physical source may be freed");
        auto Cut=P->FreezeForOwner(1);
        Check(Cut && Calls.load()==1,"closed source never calls captured physical pointers again");
        auto F=P->Produce(1);
        Check(F && F->GetData().Reset && F->GetData().Values==FActionValues{}
            && F->GetData().ActiveMask==0 && P->CloseFrozenCutoff(*Cut),
            "closed source yields a neutral reset frame while the owner keeps polling");
    }

    {
        auto Queue=std::make_shared<FInputSessionCommands>();
        auto B=Command(1,0); B.Binding=Scenario(); Queue->Submit(B);
        bool ForeignAccepted=true;
        std::thread Foreign([&]{ForeignAccepted=Queue->CancelUnprocessedAfterJoin();}); Foreign.join();
        Check(!ForeignAccepted,"only bridge publisher can cancel never-started generation");
        Check(Queue->CancelUnprocessedAfterJoin(),"cancel inert Bind after joined zero-owner proof");
        Check(Queue->Read(1)->Status==EBoundaryStatus::TerminalFailure,"unstarted Bind receives terminal outcome");
        B.Id=2; Check(Queue->Submit(B)==ECommandAdmission::Rejected,"closed unstarted generation cannot reopen");
        auto Running=std::make_shared<FInputSessionCommands>(); B.Id=1; Running->Submit(B);
        FInputSessionRegistry Registry(Running,42); Check(Registry.ServiceBoundary(),"construct real session");
        Check(!Running->CancelUnprocessedAfterJoin(),"inert cancellation cannot replace retirement of serviced owners");
    }

    {
        FRegistryLimits Limits; Limits.Sessions=1;
        auto Queue=std::make_shared<FInputSessionCommands>(Limits);
        FInputSessionRegistry Registry(Queue,42);
        auto Journal=std::make_shared<FRawAcquisitionJournal>(1,0);
        Check(Registry.RegisterJournalAtBoundary({7,Journal}),"register journal for rejected generation");
        auto B=Command(1,0); B.WorkerGeneration=41; B.Binding=Device();
        Check(Queue->Submit(B)==ECommandAdmission::Enqueued && Registry.ServiceBoundary(),"process stale worker Bind");
        Check(Queue->Read(1)->Status==EBoundaryStatus::Rejected,"stale worker Bind rejected");
        auto Next=std::make_shared<FRawAcquisitionJournal>(1,0);
        Check(Registry.RegisterJournalAtBoundary({8,Next}),"early rejection releases bounded journal slot");
        B=Command(2,99); B.Binding=Device(1,8); Queue->Submit(B);
        B=Command(3,0); B.Binding=Device(1,8); Queue->Submit(B);
        Check(Registry.ServiceBoundary() && Queue->Read(2)->Status==EBoundaryStatus::Rejected
            && Queue->Read(3)->Status==EBoundaryStatus::Applied,"stale version keeps journal required by later valid Bind");
    }

    auto Q=std::make_shared<FInputSessionCommands>();
    auto C=Command(1,0); C.Binding=Scenario();
    Check(Q->Submit(C)==ECommandAdmission::Enqueued,"inert descriptor admitted");
    Check(Q->Submit(C)==ECommandAdmission::Duplicate,"same bytes duplicate");
    auto Alter=C; Alter.Binding.Epoch=2;
    Check(Q->Submit(Alter)==ECommandAdmission::Rejected,"same id altered rejected");
    Alter=C; Alter.Binding.ResumeRearmMask=std::uint32_t{1} << 3;
    Check(Q->Submit(Alter)==ECommandAdmission::Rejected,
        "same id with altered menu rearm policy is not a duplicate");
    FInputSessionRegistry R(Q,42);
    Check(R.ServiceBoundary() && Q->Read(1)->Status==EBoundaryStatus::Applied,"worker creates session");
    Check(R.ConstructionCount()==1,"one construction across retries");
    Check(!R.HasFilterState(1),"ExactScenario allocates no filter state");
    Check(R.ReadRegistry()->Bindings[0].Actors[0].Id==10,"actor fanout stable sort");
    auto F=Step(R,0); Check(F->Inputs.size()==1 && F->Inputs[0].Actors.size()==2,"one session two actor fanout");
    F=Step(R,1); const auto& V=F->Inputs[0].Snapshot;
    Check(V->Applied==V->Requested && V->Applied[Throttle]==255,"ExactScenario has no implicit ramp");
    Check(V->Input.GetData().Transitions.size()==7 && !V->Applied[3],"short jump powerslide camera edges retained");
    auto Pause=Command(2,1,EBoundaryOperation::PauseAll); Check(Q->Submit(Pause)==ECommandAdmission::Enqueued,"pause request");
    Check(Q->Read(2)->Status==EBoundaryStatus::Pending,"enqueue is not ACK");
    Check(R.ServiceBoundary() && Q->Read(2)->Status==EBoundaryStatus::Applied,"pause actually serviced");
    auto Resume=Command(3,1,EBoundaryOperation::Resume); Resume.Session=1; Resume.Epoch=1; Resume.ResumeGeneration=1;
    Check(Q->Submit(Resume)==ECommandAdmission::Enqueued && R.ServiceBoundary(),"resume baseline inspected");
    Check(Q->Read(3)->Status==EBoundaryStatus::Applied,"scenario ready plus session ready");
    F=Step(R,2); Check(F->Inputs[0].Snapshot->Applied[Throttle]==255,"baseline inspect did not consume frame");
    Check(R.ReadHistorical(1)->Inputs[0].Snapshot==V,"historical immutable snapshot identity");
    const auto Polls=R.PollCount(1);
    Check(!R.BeginRetainedReplay(0,2,{{10,2},{11,1}}),"replay refuses wrong actor generation");
    Check(R.BeginRetainedReplay(0,2,{{10,1},{11,1}}),"historical registry input replay admitted");
    Check(!R.PrepareFrame(3),"no live frame during replay");
    Check(R.ReadLastFrameFailure().Reason == ERegistryFrameFailure::PrepareAdmission
        && !R.ReadLastFrameFailure().OwnerPresent && !R.ReadLastFrameFailure().OwnerStatusPresent,
        "replay admission reason has no invented owner status");
    for (unsigned N=0;N<3;++N) { const auto Replay=R.NextRetainedReplay(); Check(Replay && Replay->Frame==N,"replay frame order"); Check(Replay->Inputs[0].Snapshot->Applied==Replay->Inputs[0].Snapshot->Requested,"replay exact"); }
    Check(R.EndRetainedReplay() && R.PollCount(1)==Polls,"replay never polls live producer");
    {
        auto Q2=std::make_shared<FInputSessionCommands>(); auto B=Command(1,0); B.Binding=Scenario();
        Check(Q2->Submit(B)==ECommandAdmission::Enqueued,"initial bind"); FInputSessionRegistry I(Q2,42); Check(I.ServiceBoundary(),"initial service");
        Check(I.PrepareFrame(0) && I.InstallAll() && I.CloseInitialBoundary(),"initial token closes without step");
        const auto Initial=I.ReadInitial(); Check(Initial && !I.ReadLatest(),"initial receipt not completed frame");
        F=Step(I,0); Check(F->Inputs[0].AlreadyAppliedAtInitialBoundary && F->Inputs[0].Snapshot==Initial->Inputs[0].Snapshot,"new step reuses initial evidence no repoll");
        Check(I.PollCount(1)==1,"initial plus step only one Produce");
    }
    {
        auto Q2=std::make_shared<FInputSessionCommands>(); auto B=Command(1,0); B.Binding=Scenario();
        B.Binding.ResumeRearmMask=std::uint32_t{1} << 3;
        Check(Q2->Submit(B)==ECommandAdmission::Enqueued, "scenario rearm request reaches worker");
        FInputSessionRegistry I(Q2,42); Check(I.ServiceBoundary()
            && Q2->Read(1)->Status==EBoundaryStatus::Rejected,
            "menu rearm policy is device-only");
    }
    {
        auto Q2=std::make_shared<FInputSessionCommands>(); auto B=Command(1,0); B.Binding=Scenario(); B.Binding.Processing.Step[Throttle]=16;
        Check(Q2->Submit(B)==ECommandAdmission::Enqueued,"policy rejected on worker admission"); FInputSessionRegistry I(Q2,42); I.ServiceBoundary();
        Check(Q2->Read(1)->Status==EBoundaryStatus::Rejected && I.ConstructionCount()==0,"Test cannot request Device filter");
        Check(FInputSessionCommands::ExactPolicy(EProducerContract::ExactRemote,{}) && !FInputSessionCommands::ExactPolicy(EProducerContract::ExactRemote,B.Binding.Processing),"remote contract forbids local slew without enabling transport");
    }
    {
        auto Q2=std::make_shared<FInputSessionCommands>(); auto J=std::make_shared<FRawAcquisitionJournal>(7);
        auto B=Command(1,0); B.Binding=Device(); B.Binding.ResumeRearmMask=std::uint32_t{1} << 3;
        Check(Q2->Submit(B)==ECommandAdmission::Enqueued,"Device inert bind with menu rearm policy");
        FInputSessionRegistry I(Q2,42,{{7,J}}); Check(I.ServiceBoundary(),"Device bound paused");
        Check(I.ReadRegistry()->Bindings[0].ResumeRearmMask==B.Binding.ResumeRearmMask,
            "worker publishes exact device menu rearm policy");
        Check(!I.PrepareFrame(0) && !I.IsTerminal() && I.PollCount(1)==0,"empty acquisition cannot trigger premature poll");
        auto ResumeDevice=Command(2,1,EBoundaryOperation::Resume); ResumeDevice.Session=1; ResumeDevice.Epoch=1; ResumeDevice.ResumeGeneration=1;
        Check(Q2->Submit(ResumeDevice)==ECommandAdmission::Enqueued && !I.ServiceBoundary(),"Device waits baseline ACK");
        Check(Q2->Read(2)->Status==EBoundaryStatus::WaitingForBaseline && I.PollCount(1)==0,"waiting not session resume ACK");
        auto PauseDevice=Command(3,1,EBoundaryOperation::PauseAll);
        Check(Q2->Submit(PauseDevice)==ECommandAdmission::Enqueued && I.ServiceBoundary(),"new pause serviced while waiting without device");
        Check(Q2->Read(2)->Status==EBoundaryStatus::Rejected && Q2->Read(3)->Status==EBoundaryStatus::Applied,"superseded resume cannot later apply");
        ResumeDevice.Id=4; ResumeDevice.ResumeGeneration=2; Check(Q2->Submit(ResumeDevice)==ECommandAdmission::Enqueued && !I.ServiceBoundary(),"fresh generation waits");
        Check(Publish(J,1,false,true),"fresh raw baseline");
        Check(I.ServiceBoundary() && Q2->Read(4)->Status==EBoundaryStatus::Applied,"matching raw and session ACK");
        Step(I,0); Check(I.HasFilterState(1),"Device owns processing state");
        Check(Publish(J,2,true,false),"raw throttle edge"); F=Step(I,1);
        Check(F->Inputs[0].Snapshot->Requested[Throttle]==255 && F->Inputs[0].Snapshot->Applied[Throttle]==16,"Device slew once on same N");
        Check(I.PollCount(1)==2,"fanout does not repoll");
        Check(I.BeginRetainedReplay(0,1,{{10,1},{11,1}}),"Device replay restores retained before state");
        I.NextRetainedReplay(); F=I.NextRetainedReplay(); Check(F->Inputs[0].Snapshot->Applied[Throttle]==16 && I.EndRetainedReplay(),"Device replay no second slew");
        auto Other=Command(5,1); Other.Binding=Device(2);
        Check(Q2->Submit(Other)==ECommandAdmission::Enqueued && I.ServiceBoundary(),"duplicate journal command serviced");
        Check(Q2->Read(5)->Status==EBoundaryStatus::Rejected && I.ConstructionCount()==1,"one raw journal cannot feed two mutable producer cursors");
        J->Invalidate();
        Check(!I.PrepareFrame(2) && !I.ReadLatest() && I.ReadRegistry()->Terminal,
            "invalidation before physical poll aborts without publishing frame two");
        const auto Failure = I.ReadLastFrameFailure();
        Check(Failure.Reason == ERegistryFrameFailure::OwnerPoll && Failure.Frame == 2
            && Failure.Session == 1 && Failure.Epoch == 1 && Failure.OwnerPresent && Failure.OwnerStatusPresent
            && Failure.OwnerStatus == EOwnerInputStatus::ResyncRequired
            && Failure.PollFailure == EOwnerPollFailure::FreezeCutoff,
            "owner-poll failure copied before abort/quarantine");
    }
    {
        auto Q2=std::make_shared<FInputSessionCommands>(); auto J=std::make_shared<FRawAcquisitionJournal>(7);
        auto B=Command(1,0); B.Binding=Device(); Q2->Submit(B); FInputSessionRegistry I(Q2,42,{{7,J}}); I.ServiceBoundary();
        auto ResumeDevice=Command(2,1,EBoundaryOperation::Resume); ResumeDevice.Session=1; ResumeDevice.Epoch=1; ResumeDevice.ResumeGeneration=1;
        Q2->Submit(ResumeDevice); I.ServiceBoundary(); Q2->RequestStop();
        Check(I.ServiceBoundary() && I.IsTerminal() && I.ReadRegistry()->Terminal,"stop retires at paused boundary");
        Check(Q2->Read(2)->Status==EBoundaryStatus::TerminalFailure && I.DestructionCount()==1,"stop closes waiting receipt and one destruction");
    }
    {
        auto Q2=std::make_shared<FInputSessionCommands>(); auto J=std::make_shared<FRawAcquisitionJournal>(7);
        auto B=Command(1,0); B.Binding=Device(); Q2->Submit(B); FInputSessionRegistry I(Q2,42,{{7,J}}); I.ServiceBoundary();
        auto ResumeDevice=Command(2,1,EBoundaryOperation::Resume); ResumeDevice.Session=1; ResumeDevice.Epoch=1; ResumeDevice.ResumeGeneration=1;
        Q2->Submit(ResumeDevice); I.ServiceBoundary();
        auto Stale=Command(3,1,EBoundaryOperation::Detach); Stale.Session=1; Stale.Epoch=99;
        Q2->Submit(Stale); Check(!I.ServiceBoundary(),"stale detach cannot supplant waiting resume");
        Check(Q2->Read(3)->Status==EBoundaryStatus::Rejected && Q2->Read(2)->Status==EBoundaryStatus::WaitingForBaseline,"current resume receipt unchanged by wrong epoch");
        Publish(J,1,true,true); Check(I.ServiceBoundary() && Q2->Read(2)->Status==EBoundaryStatus::Applied,"current resume still completes after stale detach"); Step(I,0);
    }
    {
        auto Q2=std::make_shared<FInputSessionCommands>(); auto B=Command(1,0); B.Binding=Scenario(); Q2->Submit(B);
        FInputSessionRegistry I(Q2,42); I.ServiceBoundary(); I.PrepareFrame(0); I.InstallAll(); I.CloseInitialBoundary();
        const auto Initial=I.ReadInitial(); I.PrepareFrame(0); I.InstallAll(); I.AbortFrame();
        Check(I.IsTerminal() && I.ReadRegistry()->Terminal && !I.ReadLatest(),"failed frame cannot present success");
        Check(I.ReadHistoricalInitial()==Initial,"failed step retains closed initial evidence");
    }
    {
        // Registry/producer/mediator construction and destruction really occur on another lane.
        auto Q2=std::make_shared<FInputSessionCommands>(); auto B=Command(1,0); B.Binding=Scenario(); Q2->Submit(B);
        std::thread Worker([&] { FInputSessionRegistry I(Q2,42); Check(I.ServiceBoundary(),"worker-thread construction"); Step(I,0); Check(Q2->Submit(B)==ECommandAdmission::Rejected,"worker cannot impersonate GT command publisher"); });
        Worker.join(); Check(Q2->Read(1)->Status==EBoundaryStatus::Applied,"thread-neutral receipt after owner destruction");
        B.Id=2; B.RegistryVersion=1; Check(Q2->Submit(B)==ECommandAdmission::Rejected,"retired worker bridge rejects new command");
    }
    {
        auto Q2=std::make_shared<FInputSessionCommands>(); auto B=Command(1,0); B.Binding=Scenario(); Q2->Submit(B);
        { FInputSessionRegistry Unserviced(Q2,42); }
        Check(Q2->Read(1)->Status==EBoundaryStatus::TerminalFailure,"destruction closes never serviced command receipt");
        B.Id=2; Check(Q2->Submit(B)==ECommandAdmission::Rejected,"no enqueue after worker destructor");
    }
    {
        FRegistryLimits L; L.Commands=1; auto Q2=std::make_shared<FInputSessionCommands>(L);
        auto B=Command(1,0); B.Binding=Scenario(); Check(Q2->Submit(B)==ECommandAdmission::Enqueued,"queue capacity first");
        B.Id=2; Check(Q2->Submit(B)==ECommandAdmission::Rejected,"queue saturation rejects without transfer");
        FInputSessionRegistry I(Q2,42); Q2->RequestStop(); Check(I.ServiceBoundary() && I.ConstructionCount()==0,"stop bypasses full queue without producer construction");
        Check(Q2->Read(1)->Status==EBoundaryStatus::TerminalFailure,"unqueued execution has terminal receipt");
    }
    {
        auto Q2=std::make_shared<FInputSessionCommands>(); auto J=std::make_shared<FRawAcquisitionJournal>(7);
        auto B=Command(1,0); B.Binding=Scenario(); Q2->Submit(B);
        B=Command(2,1); B.Binding=Device(2); Q2->Submit(B);
        FInputSessionRegistry I(Q2,42,{{7,J}}); Check(I.ServiceBoundary(),"two distinct sessions admitted");
        auto ResumeDevice=Command(3,2,EBoundaryOperation::Resume); ResumeDevice.Session=2; ResumeDevice.Epoch=1; ResumeDevice.ResumeGeneration=1;
        Q2->Submit(ResumeDevice); I.ServiceBoundary(); Publish(J,1,false,true); I.ServiceBoundary(); Step(I,0);
        J->CancelLifecycle(); auto P=Command(4,2,EBoundaryOperation::PauseAll); Q2->Submit(P);
        Check(!I.ServiceBoundary() && I.IsTerminal(),"partial pause failure globally terminal");
        const auto Receipt=Q2->Read(4);
        Check(Receipt->Status==EBoundaryStatus::TerminalFailure && Receipt->Sessions.size()==2
            && Receipt->Sessions[0].Phase==ESessionPhase::Paused && Receipt->Sessions[1].Phase==ESessionPhase::CancelPending,"partial pause ledger records reached phases");
        Check(!I.PrepareFrame(1) && !I.ReadLatest() && I.ReadRegistry()->Terminal,"no old active registry or gameplay after failure");
        Check(I.ReadLastFrameFailure().Reason == ERegistryFrameFailure::PrepareAdmission
            && !I.ReadLastFrameFailure().OwnerPresent && !I.ReadLastFrameFailure().OwnerStatusPresent,
            "preexisting terminal rejects admission without invented owner status");
    }
    {
        // Same next axes after different prior histories, no filter state or ramp.
        for (const std::int16_t Before : {std::int16_t(1),std::int16_t(250)})
        {
            auto D=Scenario(); D.Scenario.clear();
            for (unsigned N=0;N<3;++N)
            {
                FInputFrameData X; X.Producer={EProducerKind::Device,1}; X.StreamEpoch={1}; X.DeviceGeneration={1}; X.SourceSequence=N+1; X.ConsumptionFrame=N; X.Reset=N==0;
                X.Values[Throttle]=N==0?Before:std::int16_t(N==1?255:1);
                X.Values[Brake]=N==0?Before:std::int16_t(N==1?1:255);
                X.Values[Steering]=N==0?std::int16_t(5):std::int16_t(N==1?-127:127); X.ActiveMask=7;
                D.Scenario.emplace_back(D.Contract,X);
            }
            auto Q2=std::make_shared<FInputSessionCommands>(); auto B=Command(1,0); B.Binding=D; Q2->Submit(B);
            FInputSessionRegistry I(Q2,42); I.ServiceBoundary(); Step(I,0);
            for (unsigned N=1;N<3;++N)
            {
                F=Step(I,N); Check(F->Inputs[0].Snapshot->Applied==D.Scenario[N].GetData().Values,"all axes bit exact regardless of prior values");
                Check(!I.HasFilterState(1) && !F->Inputs[0].FilterBefore,"exact timeline has no processing history");
            }
        }
    }
    {
        auto Q2=std::make_shared<FInputSessionCommands>(); auto B=Command(1,0); B.Binding=Scenario(); Q2->Submit(B);
        FInputSessionRegistry I(Q2,42); I.ServiceBoundary();
        auto Detach=Command(2,1,EBoundaryOperation::Detach); Detach.Session=1; Detach.Epoch=1; Q2->Submit(Detach); I.ServiceBoundary();
        Check(I.DestructionCount()==1 && Q2->Read(2)->Sessions[0].Phase==ESessionPhase::Retired,"detach records retirement before ownership release");
        B.Id=3; B.RegistryVersion=2; Q2->Submit(B); I.ServiceBoundary();
        Check(Q2->Read(3)->Status==EBoundaryStatus::Rejected,"epoch cannot be reused after detach");
        B.Id=4; B.Binding=Scenario(1,2); Q2->Submit(B); Check(I.ServiceBoundary(),"new epoch rebind");
        Check(Q2->Read(4)->Status==EBoundaryStatus::Applied && I.ConstructionCount()==2,"new sealed scenario new epoch exact"); Step(I,0);
    }
    {
        // A second participant failing preparation cannot reach the portable
        // physics hooks. Engine integration must run this same gate before forces.
        auto Q2=std::make_shared<FInputSessionCommands>(); auto B=Command(1,0); B.Binding=Scenario(); Q2->Submit(B);
        B=Command(2,1); B.Binding=Scenario(2); B.Binding.First=7;
        std::vector<VFrame> Shifted;
        for (const auto& Old : B.Binding.Scenario) { auto X=Old.GetData(); X.ConsumptionFrame+=7; Shifted.emplace_back(B.Binding.Contract,X); }
        B.Binding.Scenario=std::move(Shifted); Q2->Submit(B); FInputSessionRegistry I(Q2,42); I.ServiceBoundary();
        unsigned Gravity=0,Gameplay=0,PhysicsStep=0;
        if (I.PrepareFrame(0) && I.InstallAll() && I.BeginAll()) { ++Gravity; ++Gameplay; ++PhysicsStep; }
        Check(Gravity==0 && Gameplay==0 && PhysicsStep==0 && I.IsTerminal(),"rejected participant blocks every force/gameplay/step hook");
        Check(!I.ReadLatest() && I.PollCount(1)==1 && I.PollCount(2)==0,"partial source preparation never publishes or retries");
    }

    {
        struct FReceiver
        {
            std::vector<EStateAction> States;
            void Receive(const FActionEvent& E) { States.push_back(E.State); }
        };
        auto Q2=std::make_shared<FInputSessionCommands>(); auto B=Command(1,0); B.Binding=Scenario(); Q2->Submit(B);
        FInputSessionRegistry I(Q2,42); Check(I.ServiceBoundary(),"observer scenario bound");
        auto Channel=std::make_shared<FInputObservationChannel>(B.Binding.Contract,FStreamEpoch{1});
        Speed::Input::V2::FInputPresentationBindings Bindings(Channel);
        auto Receiver=std::make_shared<FReceiver>();
        Check(Bindings.BindAction("jump-start",3,EStateAction::Started,std::weak_ptr<FReceiver>(Receiver),&FReceiver::Receive),"observer binds Started");
        Check(Bindings.BindAction("jump-end",3,EStateAction::Completed,std::weak_ptr<FReceiver>(Receiver),&FReceiver::Receive),"observer binds Completed");
        Check(Bindings.Seal() && Channel->Activate(),"observation activates without producer");
        const auto Zero=Step(I,0); Check(Channel->Publish(Zero->Inputs[0].Snapshot),"publish completed baseline");
        const auto One=Step(I,1);
        auto Applied=std::make_shared<FOwnerInputSnapshot>(*One->Inputs[0].Snapshot); Applied->Applied[Throttle]=16;
        Check(Channel->Publish(Applied),"publish completed applied values");
        Check(Channel->ReadLatest()->Frame.GetData().Values[Throttle]==16,"GT observes Applied not raw Requested");
        Check(Bindings.HandleInputs()==EDispatchStatus::Dispatched && Receiver->States.size()==2
            && Receiver->States[0]==EStateAction::Started && Receiver->States[1]==EStateAction::Completed,"slow GT sees both short-tap edges exactly once");
        Check(Bindings.HandleInputs()==EDispatchStatus::NoChange && Receiver->States.size()==2,"no duplicate callbacks");
        bool ForeignPublished=true;
        std::thread Foreign([&] { ForeignPublished=Channel->Publish(Applied); }); Foreign.join();
        Check(!ForeignPublished,"foreign lane cannot publish");
        Channel->Pause(); Check(!Channel->ReadLatest(),"pause hides old held values");
        const auto Two=Step(I,2); Check(Channel->Publish(Two->Inputs[0].Snapshot),"fresh completed baseline resumes observer");
        Check(Channel->ReadLatest()->Frame.GetData().ConsumptionFrame==2,"fresh resumed frame visible");
        Channel->Deactivate(); Check(!Channel->Publish(Two->Inputs[0].Snapshot),"closed observer cannot publish");
    }
    {
        auto Q2=std::make_shared<FInputSessionCommands>(); FInputSessionRegistry I(Q2,42);
        auto J=std::make_shared<FRawAcquisitionJournal>(1,0);
        Check(I.RegisterJournalAtBoundary({7,J}),"worker registers thread-neutral acquisition journal");
        Check(I.RegisterJournalAtBoundary({7,J}),"identical journal registration idempotent");
        Check(!I.RegisterJournalAtBoundary({8,J}),"journal alias rejected");
        bool Accepted=true; std::thread Foreign([&] { Accepted=I.RegisterJournalAtBoundary({9,J}); }); Foreign.join();
        Check(!Accepted,"foreign lane cannot register journal");
    }
    {
        auto Queue=std::make_shared<FInputSessionCommands>(); FInputSessionRegistry Registry(Queue,42);
        auto Bind=Command(1,0); Bind.Binding=Scenario(); Bind.Binding.AllowScenarioAppend=true;
        Bind.Binding.Scenario.erase(Bind.Binding.Scenario.begin()+1,Bind.Binding.Scenario.end());
        Check(Queue->Submit(Bind)==ECommandAdmission::Enqueued && Registry.ServiceBoundary(),"adaptive exact scenario binds from sealed baseline");
        const auto Baseline=Step(Registry,0)->Inputs[0].Snapshot;
        for (std::uint64_t N=1; N<HistoryCapacity+5; ++N)
        {
            FInputFrameData Data; Data.ConsumptionFrame=N; Data.SourceSequence=N+1;
            Data.Producer={EProducerKind::Device,1}; Data.StreamEpoch={1}; Data.DeviceGeneration={1};
            Data.Values[Throttle]=N%2 ? 255 : 0; Data.ActiveMask=N%2 ? 1 : 0;
            Data.Transitions.push_back({Throttle,N%2 ? ETransition::Started : ETransition::Completed,Data.Values[Throttle],{N+1,0}});
            VFrame Input(Bind.Binding.Contract,Data);
            if (N==1)
            {
                Check(!Registry.AppendExactScenarioFrame(1,2,1,Input),"wrong author epoch rejected");
                Check(!Registry.AppendExactScenarioFrame(1,1,2,Input),"wrong author controller rejected");
                bool Foreign=true; std::thread Other([&]{ Foreign=Registry.AppendExactScenarioFrame(1,1,1,Input); }); Other.join();
                Check(!Foreign,"foreign thread cannot author scenario");
                { FPresentationInputScope Scope; Check(!Registry.AppendExactScenarioFrame(1,1,1,Input),"presentation cannot author scenario"); }
            }
            Check(Registry.AppendExactScenarioFrame(1,1,1,Input),"next exact frame sealed before cutoff");
            Check(!Registry.AppendExactScenarioFrame(1,1,1,Input),"sealed future frame cannot be overwritten");
            Check(Registry.PrepareFrame(N),"adaptive frame polled by same owner");
            auto Future=Data; Future.ConsumptionFrame=N+1; Future.SourceSequence=N+2; Future.Reset=true; Future.Transitions.clear();
            Check(!Registry.AppendExactScenarioFrame(1,1,1,VFrame(Bind.Binding.Contract,Future)),"no authoring during physical transaction");
            Check(Registry.InstallAll(),"adaptive install");
            const auto Installed=Registry.ReadInstalled();
            Check(Installed && Installed->Inputs[0].Snapshot->Applied==Data.Values,"exact authored stimulus no hidden slew");
            Check(Registry.BeginAll() && Registry.ValidateComplete() && Registry.CompleteAll(),"adaptive grouped commit");
            Check(Baseline->Input.GetData().ConsumptionFrame==0 && Baseline->Applied[Throttle]==0,"retained baseline remains immutable");
        }
    }
    {
        auto Queue=std::make_shared<FInputSessionCommands>(); FInputSessionRegistry Registry(Queue,42);
        auto Bind=Command(1,0); Bind.Binding=Scenario();
        Check(Queue->Submit(Bind)==ECommandAdmission::Enqueued && Registry.ServiceBoundary(),"static scenario binds");
        Check(!Registry.AppendExactScenarioFrame(1,1,1,Bind.Binding.Scenario.back()),"static descriptor cannot opt into authoring later");
    }
    std::cout<<"PASS InputSessionRegistryProbe checks="<<Checks<<" native=none gameplay=none\n";
}
