#include "WindowsDesktopSourceFakes.h"
#include "IAmSpeed/Input/Windows/GameInputRawAcquisition.h"
#include "IAmSpeed/Input/InputProducerV2.h"
using Speed::Input::Windows::FGameInputDesktopSource;
using namespace Speed::Input::V2;
struct Fixture
{
    GameInputMouseInfo MouseInfo{};
    ComPtr<DesktopApi> Api;
    ComPtr<FakeDevice> Keyboard,Mouse,Pad;
    std::unique_ptr<FGameInputDiscovery> Discovery;
    std::unique_ptr<FGameInputDesktopSource> Collector;
    Fixture(bool HaveKeyboard=true,bool HaveMouse=true)
    {
        Api.Attach(new DesktopApi);
        MouseInfo.supportedButtons=GameInputMouseLeftButton|GameInputMouseRightButton|GameInputMouseMiddleButton;
        if(HaveKeyboard)
        {Keyboard=MakeDevice(1,GameInputKindKeyboard);Api->Initial.push_back({Keyboard,1,true});Api->Push(Api->Keyboard(Keyboard.Get(),1,false));}
        if(HaveMouse)
        {Mouse=MakeDevice(2,GameInputKindMouse);Mouse->Info.mouseInfo=&MouseInfo;Api->Initial.push_back({Mouse,1,true});Api->Push(Api->Mouse(Mouse.Get(),1,GameInputMouseNone));}
        Pad=MakeDevice(3,GameInputKindGamepad);Api->Initial.push_back({Pad,1,true});
        Discovery=FGameInputDiscovery::Create(Api.Get(),6101,{});
        Check(bool(Discovery),"SDK discovery real callback path created");
        Collector=FGameInputDesktopSource::Create(Api.Get(),Discovery.get(),6101);
        Check(bool(Collector),"desktop collector created");
    }
    ~Fixture(){Check(Collector->Close(),"collector callbacks truly closed");Check(Discovery->Shutdown(),"discovery callback truly closed");}
    FGameInputDesktopSource::FPreparation Prepare(bool Fresh=false)
    {
        auto R=Collector->Prepare(Fresh);
        Check(R.Status==FGameInputDesktopSource::EStatus::Prepared&&bool(R.Prepared),"collector has typed prepared publication");
        Check(bool(R.Prepared->Inspect())&&ValidateDesktopPublication(*R.Prepared->Inspect()),"complete publication validates");return R;
    }
    std::shared_ptr<const FDesktopPublication> Accept(FGameInputDesktopSource::FPreparation& R)
    {
        const auto P=R.Prepared->Inspect();unsigned Installed=0;
        Check(Collector->Commit(*R.Prepared,[&]()noexcept{++Installed;return true;}),"prepared sink and source commit");
        Check(Installed==1,"prepared sink called once");
        Check(!Collector->Commit(*R.Prepared,[&]()noexcept{++Installed;return true;})&&Installed==1,"consumed ticket cannot invoke sink again");
        return P;
    }
};
static float Value(const FDesktopPublication& P,ERawControlKind Kind,std::uint16_t Code)
{
    for(const auto& V:P.Sample.FinalState)if(V.Control.Kind==Kind&&V.Control.Code==Code)return V.Value;
    return 0;
}
static FDesktopSubsource Member(const FDesktopPublication& P,EDesktopSubsourceKind Kind)
{
    for(const auto& M:P.Baseline.Members)if(M.Kind==Kind)return M;
    Check(false,"expected typed member exists");return {};
}
static void Composed(bool HaveKeyboard)
{
    using Speed::Input::Windows::FGameInputSelectedSource;
    using Speed::Input::Windows::FGameInputRawAcquisition;
    using Speed::Input::Windows::ERawPumpResult;
    ComPtr<DesktopApi> Api;Api.Attach(new DesktopApi);
    auto Mouse=MakeDevice(2,GameInputKindMouse),Pad=MakeDevice(3,GameInputKindGamepad);
    ComPtr<FakeDevice> Keyboard;
    GameInputMouseInfo MouseInfo{};MouseInfo.supportedButtons=GameInputMouseLeftButton|GameInputMouseRightButton|GameInputMouseMiddleButton;
    Mouse->Info.mouseInfo=&MouseInfo;
    Api->Initial={{Mouse,1,true},{Pad,1,true}};
    Api->Push(Api->Mouse(Mouse.Get(),10,GameInputMouseNone));Api->Push(Api->Gamepad(Pad.Get(),10,false));
    if(HaveKeyboard)
    {Keyboard=MakeDevice(1,GameInputKindKeyboard);Api->Initial.push_back({Keyboard,1,true});Api->Push(Api->Keyboard(Keyboard.Get(),10,false));}
    FActivityConfig Policy{{.20f,.15f,.10f},{.08f,.05f,.04f},0,8,EDeviceKind::Gamepad};
    Policy.StartupPreferredKind=EDeviceKind::Gamepad;Policy.GamepadPriority=true;Policy.RetainSameKindOwner=true;
    const std::uint64_t Session=HaveKeyboard?6202:6201;
    auto Journal=std::make_shared<FRawAcquisitionJournal>(Session);
    FGameInputRawAcquisition Acquisition(FGameInputSelectedSource::CreateRaw(Api.Get(),Session,Policy,true),Journal,true);
    FInputActionContractDescription D;D.Revision={1};D.Actions=FInputActionContract::BaseActions();
    FActionDefinition Jump;Jump.Id=3;Jump.Owner="DesktopSourceProbe";Jump.Name="Jump";Jump.Wiring=EActionWiring::Wired;
    D.Actions.push_back(Jump);
    D.Mapping={{{ERawControlKind::KeyboardUsage,0x1a},Throttle,1},
        {{ERawControlKind::PadAxis,std::uint16_t(EPadAxis::RightTrigger)},Throttle,1},
        {{ERawControlKind::MouseButton,std::uint16_t(EMouseButton::Left)},3,1},
        {{ERawControlKind::PadButton,std::uint16_t(EPadButton::South)},3,1}};
    D.Physical={{0,EPhysicalDestination::Throttle},{1,EPhysicalDestination::Brake},{2,EPhysicalDestination::Steering}};
    const auto Contract=FInputActionContract::Create(std::move(D));Check(bool(Contract),"composed actual KB/mouse/pad action contract");
    auto Producer=Speed::Input::V2::FDeviceInputProducer::CreateDesktop(Journal,Contract,{Session},{EProducerKind::Device,Session},0,std::uint32_t(1)<<3);
    Check(bool(Producer),"composed real desktop-aware producer and mapper");
    std::uint64_t FrameNumber=0;
    const auto Step=[&](ERawDeviceKind Expected,std::int16_t ThrottleValue,std::int16_t JumpValue)
    {
        // A handoff may require the NEXT genuinely fresh SDK snapshot. Never
        // relabel a private Poll publication as a lifecycle envelope here.
        for(unsigned Attempt=0;Attempt<3;++Attempt)
        {
            const auto R=Acquisition.Pump();
            Check(R==ERawPumpResult::Installed||R==ERawPumpResult::NoChange,"composed native acquisition remains healthy");
            const auto B=Journal->ReadControlBaseline();
            if(B&&B->State.Kind==Expected&&
               (Expected!=ERawDeviceKind::Desktop||!B->State.FreshBaseline||
                (B->State.DesktopPublication&&B->State.DesktopPublication->RequiresLifecycleEnvelope)))break;
        }
        const auto B=Journal->ReadControlBaseline();
        Check(B&&B->State.Kind==Expected&&B->State.HasSelectedDevice,"composed selected source genuinely publishes expected owner");
        if(Expected==ERawDeviceKind::Desktop)
            Check(HasValidDesktopProvenance(B->State)&&(!B->State.FreshBaseline||B->State.DesktopPublication->RequiresLifecycleEnvelope),
                "first desktop consumer receives an actual fresh lifecycle envelope");
        auto Frame=Producer->Produce(FrameNumber++);
        Check(bool(Frame),"actual typed journal publication is accepted by physical producer/mapper");
        Check(Frame->GetData().Values[Throttle]==ThrottleValue&&Frame->GetData().Values[3]==JumpValue,
            "composed mapped values match SDK keyboard/mouse/pad controls");
        Check(Journal->ReadInitialOwnerAdmission().DeliverySequence==FrameNumber,"real journal delivery clock advances once per produced frame");
        return *Frame;
    };
    const auto Startup=Step(ERawDeviceKind::Gamepad,0,0);
    Check(Startup.GetData().Reset,"actual neutral pad startup is a baseline");
    bool MouseSubscribed=false;
    for(const auto& C:Api->ReadingCallbacks)if(C.second.Device==Mouse.Get()&&C.second.Kind==GameInputKindMouse)MouseSubscribed=true;
    Check(MouseSubscribed,"desktop collector already observes mouse while pad owns gameplay");
    Api->Push(Api->Mouse(Mouse.Get(),11,GameInputMouseLeftButton));
    const auto First=Step(ERawDeviceKind::Desktop,0,1);
    Check(First.GetData().Reset,"first actual mouse click replaces provisional neutral pad through real baseline");
    if(!HaveKeyboard)Check(Journal->ReadControlBaseline()->State.DesktopPublication->Baseline.Members.size()==1,
        "first mouse takeover needs no fabricated keyboard member");
    Api->Push(Api->Mouse(Mouse.Get(),12,GameInputMouseNone));Step(ERawDeviceKind::Desktop,0,0);
    if(HaveKeyboard)
    {
        Api->Push(Api->Keyboard(Keyboard.Get(),13,true));Step(ERawDeviceKind::Desktop,255,0);
        Api->Push(Api->Mouse(Mouse.Get(),14,GameInputMouseLeftButton));const auto Mixed=Step(ERawDeviceKind::Desktop,255,1);
        Check(!Mixed.GetData().Reset,"mouse jump keeps actual held keyboard throttle in same desktop generation");
        unsigned Starts=0;for(const auto& E:Mixed.GetData().Transitions)if(E.Action==3&&E.State==ETransition::Started)++Starts;
        Check(Starts==1,"one observed mouse press produces exactly one genuine mapped Jump Started");
        Api->Push(Api->Mouse(Mouse.Get(),15,GameInputMouseNone));Step(ERawDeviceKind::Desktop,255,0);
        Api->Push(Api->Keyboard(Keyboard.Get(),16,false));Step(ERawDeviceKind::Desktop,0,0);
    }
    Api->Push(Api->Gamepad(Pad.Get(),20,true));Step(ERawDeviceKind::Gamepad,255,1);
    Api->Push(Api->Gamepad(Pad.Get(),21,false));Step(ERawDeviceKind::Gamepad,0,0);
    Api->Push(Api->Mouse(Mouse.Get(),22,GameInputMouseLeftButton));Step(ERawDeviceKind::Desktop,0,1);
    Check(Producer->SetLifecyclePaused(true)==ELifecycleResult::Applied&&Journal->RequestFreshResume(),
        "composed pause requests a genuinely fresh current reading");
    Check(Acquisition.Pump()==ERawPumpResult::Installed&&Journal->IsResumeReady(),"real SDK desktop snapshot acknowledges resume fence");
    Check(Producer->SetLifecyclePaused(false)==ELifecycleResult::Applied,"actual producer resumes only after acquisition acknowledgement");
    const auto Resumed=Producer->Produce(FrameNumber++);Check(bool(Resumed),"fresh typed desktop resume maps successfully");
    Check(Resumed->GetData().Reset&&Resumed->GetData().Values[3]==0,"held mouse cannot invent Jump during resume rearm");
    for(const auto& E:Resumed->GetData().Transitions)Check(E.Action!=3||E.State!=ETransition::Started,"resume contains no ghost mouse Started");
    Api->Push(Api->Mouse(Mouse.Get(),23,GameInputMouseNone));Step(ERawDeviceKind::Desktop,0,0);
    Api->Push(Api->Mouse(Mouse.Get(),24,GameInputMouseLeftButton));const auto Rearmed=Step(ERawDeviceKind::Desktop,0,1);
    unsigned Starts=0;for(const auto& E:Rearmed.GetData().Transitions)if(E.Action==3&&E.State==ETransition::Started)++Starts;
    Check(Starts==1,"new post-resume SDK press has exactly one mapped Started");
    Check(Acquisition.Close(),"composed native source/journal acquisition closes its real callback owners");
}
int main()
{
    Composed(false);Composed(true);
    {
        Fixture F;
        Check(!FGameInputDesktopSource::Create(nullptr,F.Discovery.get(),1)&&!FGameInputDesktopSource::Create(F.Api.Get(),nullptr,1)
            &&!FGameInputDesktopSource::Create(F.Api.Get(),F.Discovery.get(),0),"no implicit API/catalogue/session");
        const auto Catalogue=F.Discovery->CaptureDesktop();
        Check(Catalogue&&Catalogue->Devices.size()==2,"real keyboard/mouse catalogue excludes gamepad");
        const auto Legacy=F.Discovery->Snapshot();
        for(const auto& D:Legacy)if(D.Id==Key(2))Check(D.SupportedKinds==0,"mouse never aliases legacy keyboard or pad");
        auto A=F.Prepare(true);const auto Staged=A.Prepared->Inspect();
        Check(Staged->Baseline.ProducerId==6101&&Staged->Baseline.StreamEpoch==6101,"producer and epoch are the actual session");
        Check(Staged->Baseline.Members.size()==2&&!A.Prepared->MouseActivity(),"initial real complete baseline has no mouse activation");
        unsigned Refusals=0;
        Check(!F.Collector->Commit(*A.Prepared,[&]()noexcept{++Refusals;return false;})&&Refusals==1,"refused sink cannot commit source");
        auto B=F.Prepare(false);
        Check(B.Prepared->Inspect()->Baseline.BaselineRevision==Staged->Baseline.BaselineRevision&&
            B.Prepared->Inspect()->Baseline.AcceptedThrough==Staged->Baseline.AcceptedThrough,"sink refusal leaves portable source watermarks unchanged");
        F.Accept(B);
        Check(!F.Collector->Commit(*A.Prepared,[]()noexcept{return true;}),"older prepared source ticket remains stale");
        const auto K1=F.Api->Keyboard(F.Keyboard.Get(),100,true),K0=F.Api->Keyboard(F.Keyboard.Get(),101,false);
        const auto M1=F.Api->Mouse(F.Mouse.Get(),100,GameInputMouseLeftButton),M0=F.Api->Mouse(F.Mouse.Get(),101,GameInputMouseNone);
        F.Api->Push(K1);F.Api->Push(M1);F.Api->Push(K0);F.Api->Push(M0);
        auto Changes=F.Prepare();const auto P=Changes.Prepared->Inspect();
        Check(Changes.Prepared->MouseActivity(),"only observed hardware mouse press claims desktop activity");
        Check(P->Changes.size()==4,"short keyboard and mouse taps retained between polls");
        Check(P->Changes[0].Origin.Control.Kind==ERawControlKind::KeyboardUsage&&P->Changes[0].Value==1&&
            P->Changes[1].Origin.Control.Kind==ERawControlKind::MouseButton&&P->Changes[1].Value==1&&
            P->Changes[2].Origin.Control.Kind==ERawControlKind::KeyboardUsage&&P->Changes[2].Value==0&&
            P->Changes[3].Origin.Control.Kind==ERawControlKind::MouseButton&&P->Changes[3].Value==0,"actual callback observation order retained across kinds despite equal SDK timestamps");
        Check(P->Changes[0].GlobalSequence<P->Changes[1].GlobalSequence&&P->Changes[1].GlobalSequence<P->Changes[2].GlobalSequence,
            "accepted ingress sequence, not timestamp tie guessing");
        Check(K1->KeyReads&&M1->MouseReads,"collector decodes real SDK GetKeyState and GetMouseState methods");
        Check(Value(*P,ERawControlKind::KeyboardUsage,0x1a)==0&&Value(*P,ERawControlKind::MouseButton,0)==0,"both final releases retained");
        F.Accept(Changes);
        Check(F.Collector->Prepare(false).Status==FGameInputDesktopSource::EStatus::NoChange,"quiet source creates no duplicate edges");
        F.Api->Push(K0);F.Api->Push(M0);
        Check(F.Collector->Prepare(false).Status==FGameInputDesktopSource::EStatus::NoChange,"repeated accepted COM identities do not duplicate input");
    }
    {
        Fixture F(false,true);
        F.Api->DuringCurrent=[&]()
        {F.Api->Push(F.Api->Mouse(F.Mouse.Get(),1,GameInputMouseLeftButton));F.Api->Push(F.Api->Mouse(F.Mouse.Get(),2,GameInputMouseNone));};
        auto Base=F.Prepare(true);
        Check(Value(*Base.Prepared->Inspect(),ERawControlKind::MouseButton,0)==1&&!Base.Prepared->MouseActivity(),
            "earliest retained callback beats a later current snapshot; baseline hold never claims activity");
        F.Accept(Base);auto Release=F.Prepare();
        Check(Release.Prepared->Inspect()->Changes.size()==1&&Release.Prepared->Inspect()->Changes[0].Value==0&&!Release.Prepared->MouseActivity(),
            "post-baseline release not lost during SDK current query");F.Accept(Release);
        F.Api->Push(F.Api->Mouse(F.Mouse.Get(),3,GameInputMouseMiddleButton));auto Middle=F.Prepare();
        Check(Middle.Prepared->MouseActivity()&&Value(*Middle.Prepared->Inspect(),ERawControlKind::MouseButton,2)==1,
            "mouse-only source produces typed middle-button input and activity");F.Accept(Middle);
    }
    {
        Fixture F;auto Base=F.Prepare(true);F.Accept(Base);
        F.Api->Push(F.Api->Keyboard(F.Keyboard.Get(),2,true));F.Api->Push(F.Api->Mouse(F.Mouse.Get(),2,GameInputMouseRightButton));
        auto Held=F.Prepare();const auto P=F.Accept(Held);
        const auto K=Member(*P,EDesktopSubsourceKind::Keyboard),M=Member(*P,EDesktopSubsourceKind::Mouse);
        F.Api->FireDevice(F.Mouse.Get(),2,false);
        auto Lost=F.Prepare();const auto L=Lost.Prepared->Inspect();
        Check(L->Baseline.Members.size()==1&&Member(*L,EDesktopSubsourceKind::Keyboard)==K,"mouse loss retains exact keyboard source generation");
        Check(Value(*L,ERawControlKind::KeyboardUsage,0x1a)==1&&L->RawTerminals.size()==1&&
            L->RawTerminals[0].Origin.Source==M&&L->RawTerminals[0].Cause==EIngressCause::Disconnect,"disconnect terminates only actual mouse contribution, keeps held keyboard");
        F.Accept(Lost);
        auto Replacement=MakeDevice(2,GameInputKindMouse);Replacement->Info.mouseInfo=&F.MouseInfo;
        F.Api->Push(F.Api->Mouse(Replacement.Get(),4,GameInputMouseRightButton));F.Api->FireDevice(Replacement.Get(),3,true);
        auto Back=F.Prepare();const auto R=Back.Prepared->Inspect();
        Check(Member(*R,EDesktopSubsourceKind::Mouse).Generation>M.Generation&&!Back.Prepared->MouseActivity(),
            "real replacement gets new typed generation without an invented held press");
        F.Accept(Back);const auto Revision=F.Discovery->CaptureDesktop()->Revision;
        F.Api->FireDevice(F.Mouse.Get(),4,false);
        Check(F.Discovery->CaptureDesktop()->Revision==Revision,"obsolete object disconnect cannot remove its replacement");
    }
    {
        Fixture F;auto Base=F.Prepare(true);const auto B=F.Accept(Base);
        F.Api->Push(F.Api->Keyboard(F.Keyboard.Get(),2,true));auto Candidate=F.Prepare();
        F.Api->FireDevice(F.Mouse.Get(),2,false);unsigned Install=0;
        Check(!F.Collector->Commit(*Candidate.Prepared,[&]()noexcept{++Install;return true;})&&Install==0,
            "hotplug after Prepare rejects before any downstream installation");
        auto Recovered=F.Prepare();
        Check(Recovered.Prepared->Inspect()->Baseline.BaselineRevision==B->Baseline.BaselineRevision+1,
            "rejected catalogue receipt did not advance portable source revision");F.Accept(Recovered);
    }
    {
        Fixture F(false,true);auto Base=F.Prepare(true);F.Accept(Base);
        for(unsigned I=0;I<257;++I)F.Api->Push(F.Api->Mouse(F.Mouse.Get(),I+2,(I&1)?GameInputMouseLeftButton:GameInputMouseNone));
        auto Overflow=F.Collector->Prepare(false);
        Check(Overflow.Status==FGameInputDesktopSource::EStatus::Unavailable&&!Overflow.Prepared,"bounded callback overflow publishes no guessed state");
        F.Api->FailReadUnregister=true;Check(!F.Collector->Close()&&!F.Api->ReadingCallbacks.empty(),"failed unregister retains callback contexts");
        F.Api->Push(F.Api->Mouse(F.Mouse.Get(),999,GameInputMouseLeftButton));
        F.Api->FailReadUnregister=false;Check(F.Collector->Close()&&F.Api->ReadingCallbacks.empty(),"retry really fences all callbacks before release");
    }
    {
        Fixture F(false,true);auto Base=F.Prepare(true);F.Accept(Base);
        const auto Callback=*F.Api->ReadingCallbacks.begin();
        Callback.second.Function(Callback.first,Callback.second.Context,nullptr);
        const auto Bad=F.Collector->Prepare(false);
        Check(Bad.Status==FGameInputDesktopSource::EStatus::Rejected&&!Bad.Prepared,
            "malformed SDK callback remains fatal, never backend retry permission");
    }
    {
        Fixture F;F.Mouse->Info.mouseInfo=nullptr;
        const auto Bad=F.Collector->Prepare(true);
        Check(Bad.Status==FGameInputDesktopSource::EStatus::Rejected&&!Bad.Prepared,"malformed SDK mouse capability stays fatal");
        Check(F.Collector->Close()&&F.Api->ReadingCallbacks.empty(),"partial setup closes earlier successfully registered contexts");
    }
    {
        Fixture F(true,false);auto Base=F.Prepare(true);F.Accept(Base);
        std::atomic<bool> Entered{false},Release{false};
        F.Api->BeforeReading=[&](){Entered=true;Await([&](){return Release.load();});};
        std::thread Callback([&](){F.Api->Push(F.Api->Keyboard(F.Keyboard.Get(),2,true));});
        Await([&](){return Entered.load();});
        std::thread Releaser([&](){Await([&](){return F.Api->UnregisterWaiting.load();});Release=true;});
        Check(F.Collector->Close(),"Close returns only after actual SDK callback fence");
        Callback.join();Releaser.join();Check(F.Api->ReadingCallbacks.empty(),"no retained callback after joined close");
    }
    std::cout<<"PASS WindowsDesktopSourceProbe checks="<<Checks<<'\n';
}
