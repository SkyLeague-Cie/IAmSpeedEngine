#include "IAmSpeed/Input/InputSessionRegistry.h"
#include <iostream>
#include <cstdlib>

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
    const auto F=R.ReadInstalled(); Check(F && F->Frame==N,"same frame installed");
    Check(R.BeginAll() && R.ValidateComplete() && R.CompleteAll(),"group complete");
    return F;
}
int main()
{
    auto Q=std::make_shared<FInputSessionCommands>();
    auto C=Command(1,0); C.Binding=Scenario();
    Check(Q->Submit(C)==ECommandAdmission::Enqueued,"inert descriptor admitted");
    Check(Q->Submit(C)==ECommandAdmission::Duplicate,"same bytes duplicate");
    auto Alter=C; Alter.Binding.Epoch=2;
    Check(Q->Submit(Alter)==ECommandAdmission::Rejected,"same id altered rejected");
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
        auto Q2=std::make_shared<FInputSessionCommands>(); auto B=Command(1,0); B.Binding=Scenario(); B.Binding.Processing.Step[Throttle]=16;
        Check(Q2->Submit(B)==ECommandAdmission::Enqueued,"policy rejected on worker admission"); FInputSessionRegistry I(Q2,42); I.ServiceBoundary();
        Check(Q2->Read(1)->Status==EBoundaryStatus::Rejected && I.ConstructionCount()==0,"Test cannot request Device filter");
        Check(FInputSessionCommands::ExactPolicy(EProducerContract::ExactRemote,{}) && !FInputSessionCommands::ExactPolicy(EProducerContract::ExactRemote,B.Binding.Processing),"remote contract forbids local slew without enabling transport");
    }
    {
        auto Q2=std::make_shared<FInputSessionCommands>(); auto J=std::make_shared<FRawAcquisitionJournal>(7);
        auto B=Command(1,0); B.Binding=Device(); Check(Q2->Submit(B)==ECommandAdmission::Enqueued,"Device inert bind");
        FInputSessionRegistry I(Q2,42,{{7,J}}); Check(I.ServiceBoundary(),"Device bound paused");
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
    std::cout<<"PASS InputSessionRegistryProbe checks="<<Checks<<" native=none gameplay=none\n";
}
