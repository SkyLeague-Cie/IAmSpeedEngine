#include "IAmSpeed/Input/InputProducerV2.h"
#include "IAmSpeed/Input/DesktopInputSource.h"
#include <cstdlib>
#include <deque>
#include <iostream>
using namespace Speed::Input;
using namespace Speed::Input::V2;
static unsigned Checks=0;
static void Check(bool V,const char* M){++Checks;if(!V){std::cerr<<"FAIL "<<M<<'\n';std::exit(1);}}
static std::shared_ptr<const FInputActionContract> Contract()
{
    FInputActionContractDescription D;D.Revision={1};D.Actions=FInputActionContract::BaseActions();
    for(FActionId Id=3;Id<=4;++Id){FActionDefinition A;A.Id=Id;A.Owner="Fixture";A.Name=Id==3?"Boost":"Other";A.Wiring=EActionWiring::Wired;D.Actions.push_back(A);}
    D.Mapping={{{ERawControlKind::KeyboardUsage,4},3,1},{{ERawControlKind::KeyboardUsage,5},Throttle,1},
        {{ERawControlKind::KeyboardUsage,6},4,1},{{ERawControlKind::MouseButton,0},3,1},{{ERawControlKind::MouseButton,1},4,1},
        {{ERawControlKind::PadButton,std::uint16_t(EPadButton::South)},3,1},
        {{ERawControlKind::PadAxis,std::uint16_t(EPadAxis::RightTrigger)},Throttle,1}};
    D.Physical={{0,EPhysicalDestination::Throttle},{1,EPhysicalDestination::Brake},{2,EPhysicalDestination::Steering}};
    return FInputActionContract::Create(D);
}
struct Fixture
{
    static constexpr std::uint64_t Session=77;
    FDesktopInputSource Source{Session,Session};
    std::shared_ptr<const FInputActionContract> C=Contract();
    std::unique_ptr<FActionMapper> Mapper=FActionMapper::CreateDesktop(C,{Session},{EProducerKind::Device,Session});
    FDesktopSubsource Keyboard{EDesktopSubsourceKind::Keyboard,1,1},Mouse{EDesktopSubsourceKind::Mouse,2,2};
    std::uint64_t K=0,M=0,Delivery=0,Generation=0;
    Fixture(){Check(bool(C)&&bool(Mapper),"typed mapper is explicitly constructed");}
    FDesktopFullObservation Key(unsigned Bits)
    {
        ++K;return {Keyboard,K,K,{{{ERawControlKind::KeyboardUsage,4},(Bits&1)?1.f:0.f},
            {{ERawControlKind::KeyboardUsage,5},(Bits&2)?1.f:0.f},{{ERawControlKind::KeyboardUsage,6},(Bits&4)?1.f:0.f}}};
    }
    FDesktopFullObservation MouseRead(unsigned Bits)
    {
        ++M;return {Mouse,M,M,{{{ERawControlKind::MouseButton,0},(Bits&1)?1.f:0.f},
            {{ERawControlKind::MouseButton,1},(Bits&2)?1.f:0.f},{{ERawControlKind::MouseButton,2},(Bits&4)?1.f:0.f}}};
    }
    std::shared_ptr<const FDesktopPublication> Publish(EDesktopBatchMode Mode,
        std::initializer_list<FDesktopFullObservation> Observations={})
    {
        FDesktopBatch B;B.Mode=Mode;B.Observations=Observations;
        const auto T=Source.CapturePump();Check(bool(T),"desktop transaction ticket exists");
        auto P=Source.Prepare(*T,B);Check(P.Status==EDesktopAdmission::Ready&&bool(P.Prepared),"real source prepares a typed publication");
        Check(Source.Commit(*P.Prepared)==EDesktopAdmission::Ready,"real source commits once");
        auto Pub=Source.ReadPublished();Check(Pub&&ValidateDesktopPublication(*Pub),"real source publication passes typed validation");return Pub;
    }
    FRawInputSample Packet(std::vector<std::shared_ptr<const FDesktopPublication>> Pubs,bool Reset=false)
    {
        auto Latest=Source.ReadPublished();Check(bool(Latest),"packet has an actual current publication");
        FRawInputSample S;S.DeviceId=Session;S.Generation={Generation+=(Reset?1:0)};S.Sequence=++Delivery;
        S.Kind=ERawDeviceKind::Desktop;S.Status=Reset?ERawSampleStatus::Resync:ERawSampleStatus::Valid;S.FinalState=Latest->Sample.FinalState;
        auto In=std::make_shared<FDesktopFrameIngress>();In->Publications=std::move(Pubs);In->Latest=Latest;
        if(!Reset)
        {
            std::uint32_t Ordinal=0;
            for(const auto& P:In->Publications)for(auto C:P->Sample.Changes)
            {C.Order={S.Sequence,Ordinal++};S.Changes.push_back(C);}
        }
        S.DesktopIngress=std::move(In);Check(S.IsValid(),"typed physical packet is structurally valid");return S;
    }
    Speed::Input::V2::FInputFrame Map(FRawInputSample S,FFrameNumber Frame)
    {auto R=Mapper->MapDesktop(S,Frame);Check(R.Status==EMappingStatus::Mapped&&R.Frame&&R.Frame->IsValidFor(*C),"typed physical frame maps");return *R.Frame;}
    Speed::Input::V2::FInputFrame Baseline(unsigned Keys=0,unsigned Buttons=0)
    {auto P=Publish(EDesktopBatchMode::FreshBaseline,{Key(Keys),MouseRead(Buttons)});return Map(Packet({P},true),0);}
    FRawInputSample Pad()
    {
        FRawInputSample S;S.DeviceId=100;S.Generation={100};S.Sequence=++Delivery;S.Kind=ERawDeviceKind::Gamepad;S.Status=ERawSampleStatus::Resync;
        S.FinalState={{{ERawControlKind::PadButton,std::uint16_t(EPadButton::South)},0},
            {{ERawControlKind::PadAxis,std::uint16_t(EPadAxis::RightTrigger)},0}};Check(S.IsValid(),"replacement is a real typed gamepad DTO");return S;
    }
};
static const FDesktopActionTerminal& Terminal(const std::vector<FDesktopActionTerminal>& Ts,EDesktopSubsourceKind Kind,FActionId Action)
{
    for(const auto& T:Ts)if(T.Origin.Source.Kind==Kind&&T.Action==Action){Check(T.IsValid(),"terminal has one valid clock domain");return T;}
    Check(false,"expected action-aware terminal exists");return Ts.front();
}
class QueueRaw final:public IRawInputSource
{
public:
    std::deque<FRawInputSample> Samples;
    std::optional<FRawInputSample> Poll(FFrameNumber)override
    {if(Samples.empty())return {};auto S=std::move(Samples.front());Samples.pop_front();return S;}
    ELifecycleResult SetLifecyclePaused(bool)override{return ELifecycleResult::Applied;}
    ELifecycleResult CancelLifecycle()override{return ELifecycleResult::Applied;}
};
int main()
{
    {
        Fixture F;Check(!FActionMapper::SupportsContract(*F.C)&&FActionMapper::SupportsDesktopContract(*F.C),"legacy and typed capability gates remain distinct");
        Check(!FActionMapper::Create(F.C,{77},{EProducerKind::Device,77}),"legacy factory still refuses a mouse contract");
        auto P=F.Publish(EDesktopBatchMode::FreshBaseline,{F.Key(0),F.MouseRead(0)});auto S=F.Packet({P},true);
        FActionMapper Legacy(F.C,{77},{EProducerKind::Device,77});Check(!Legacy.Map(S,0).Frame,"legacy mapper does not activate typed Desktop");
        auto Bare=S;Bare.DesktopIngress.reset();Check(!F.Mapper->MapDesktop(Bare,0).Frame,"raw Desktop without provenance is refused");
        auto Base=F.Map(S,0);Check(Base.GetData().Reset&&Base.GetData().Transitions.empty(),"rejection did not consume first frame or invent a baseline edge");
        P=F.Publish(EDesktopBatchMode::Poll,{F.MouseRead(1)});auto Press=F.Map(F.Packet({P}),1);
        Check(Press.GetData().Transitions.size()==1&&Press.GetData().Transitions[0].Action==3&&Press.GetData().Transitions[0].State==ETransition::Started,"real mouse press emits one Started");
        const auto Started=IdentifyEdge(Press,0);Check(bool(Started),"actual immutable physical Started identity is available");
        P=F.Publish(EDesktopBatchMode::Poll,{F.Key(1)});auto Joined=F.Map(F.Packet({P}),2);
        Check(Joined.GetData().Transitions.empty()&&Joined.GetData().Values[3]==1,"keyboard joins held mouse without a duplicate Started");
        P=F.Publish(EDesktopBatchMode::Poll,{F.MouseRead(0)});auto MouseRelease=F.Map(F.Packet({P}),3);
        Check(MouseRelease.GetData().Transitions.empty()&&MouseRelease.GetData().Values[3]==1,"mouse release does not complete action still held by keyboard");
        const auto MT=Terminal(F.Mapper->DesktopTerminals(),EDesktopSubsourceKind::Mouse,3);
        Check(MT.Cause==EIngressCause::Hardware&&MT.Clock==EDesktopTerminalClock::Ingress&&MT.StartedEdge&&*MT.StartedEdge==*Started,"mouse contribution preserves original action Started identity");
        P=F.Publish(EDesktopBatchMode::Poll,{F.Key(0)});auto Release=F.Map(F.Packet({P}),4);
        Check(Release.GetData().Transitions.size()==1&&Release.GetData().Transitions[0].State==ETransition::Completed,"last genuine contribution release completes once");
        const auto KT=Terminal(F.Mapper->DesktopTerminals(),EDesktopSubsourceKind::Keyboard,3);
        Check(KT.StartedEdge&&*KT.StartedEdge==*Started&&KT.ActionActivation==MT.ActionActivation,"both sources share one action activation identity");
        auto Hold=F.Map(F.Packet({}),5);Check(Hold.GetData().Transitions.empty()&&F.Mapper->DesktopTerminals().empty(),"no-change Latest is not replayed as a new publication or terminal");
    }
    {
        Fixture F;
        F.Publish(EDesktopBatchMode::FreshBaseline,{F.Key(0),F.MouseRead(0)}); // collector is already warm behind pad
        auto Pad=F.Map(F.Pad(),0);Check(Pad.GetData().Reset,"real pad owns the first physical frame");
        auto P=F.Publish(EDesktopBatchMode::Poll,{F.MouseRead(1)});
        Check(!P->RequiresLifecycleEnvelope,"first actual mouse press is an ordinary collector publication");
        auto Mouse=F.Map(F.Packet({P},true),1);
        Check(Mouse.GetData().Reset&&Mouse.GetData().Values[3]==1&&Mouse.GetData().Transitions.empty(),
            "explicit outer owner resync admits typed full current snapshot without relabelling its collector clocks or inventing Started");
        Check(!F.Mapper->MapDesktop(F.Packet({P},true),2).Frame,"cached collector publication cannot be relabelled as another fresh baseline");
        P=F.Publish(EDesktopBatchMode::FreshBaseline,{F.Key(0),F.MouseRead(1)});
        auto Fresh=F.Map(F.Packet({P},true),2);Check(Fresh.GetData().Transitions.empty(),"only a new real typed publication can recover fresh baseline");
    }
    for(auto Mode:{EDesktopBatchMode::Pause,EDesktopBatchMode::Reset,EDesktopBatchMode::Retire})
    {
        Fixture F;F.Baseline();auto P=F.Publish(EDesktopBatchMode::Poll,{F.MouseRead(1)});auto Press=F.Map(F.Packet({P}),1);
        const auto Started=IdentifyEdge(Press,0);P=F.Publish(Mode);auto End=F.Map(F.Packet({P},true),2);
        const auto& T=Terminal(F.Mapper->DesktopTerminals(),EDesktopSubsourceKind::Mouse,3);
        const auto Cause=Mode==EDesktopBatchMode::Pause?EIngressCause::Pause:Mode==EDesktopBatchMode::Reset?EIngressCause::Reset:EIngressCause::Replacement;
        Check(End.GetData().Reset&&End.GetData().Transitions.empty()&&T.Cause==Cause&&T.Clock==EDesktopTerminalClock::Ingress
            &&T.StartedEdge&&Started&&*T.StartedEdge==*Started,"typed lifecycle retires exact activation without a manufactured Completed");
    }
    {
        Fixture F;F.Baseline();auto P=F.Publish(EDesktopBatchMode::Poll,{F.MouseRead(1)});auto Press=F.Map(F.Packet({P}),1);const auto Started=IdentifyEdge(Press,0);
        Check(F.Source.EnqueueLifecycle({F.Mouse,EIngressCause::Disconnect}),"real source accepts disconnect notice");
        P=F.Publish(EDesktopBatchMode::FreshBaseline,{F.Key(0)});auto End=F.Map(F.Packet({P},true),2);
        const auto& T=Terminal(F.Mapper->DesktopTerminals(),EDesktopSubsourceKind::Mouse,3);
        Check(T.Cause==EIngressCause::Disconnect&&T.Clock==EDesktopTerminalClock::Ingress&&T.StartedEdge&&*T.StartedEdge==*Started
            &&End.GetData().Transitions.empty(),"disconnect provenance retires real mouse source without disabling the remaining keyboard");
    }
    {
        Fixture F;F.Baseline();auto P=F.Publish(EDesktopBatchMode::Poll,{F.MouseRead(1)});auto Press=F.Map(F.Packet({P}),1);const auto Started=IdentifyEdge(Press,0);
        auto Forged=F.Packet({});Forged.FinalState[0].Value=1;
        Check(!F.Mapper->MapDesktop(Forged,2).Frame&&F.Mapper->DesktopTerminals().empty(),"rejected forged union cannot mutate live ledger or retire its Started");
        auto Replacement=F.Map(F.Pad(),2);const auto& T=Terminal(F.Mapper->DesktopTerminals(),EDesktopSubsourceKind::Mouse,3);
        Check(Replacement.GetData().Reset&&Replacement.GetData().Transitions.empty()&&T.Cause==EIngressCause::Replacement
            &&T.Clock==EDesktopTerminalClock::PhysicalBoundary&&!T.IngressTerminalSequence&&T.PhysicalBoundary
            &&T.PhysicalBoundary->Epoch.Value==77&&T.PhysicalBoundary->Frame==2&&T.StartedEdge&&*T.StartedEdge==*Started,
            "real pad replacement has a physical boundary, never a pad sequence posing as a Desktop raw terminal");
        auto Mixed=T;Mixed.IngressTerminalSequence=1;Check(!Mixed.IsValid(),"mixed terminal clock domains are rejected");
    }
    {
        Fixture F;F.Baseline();auto P=F.Publish(EDesktopBatchMode::Poll,{F.MouseRead(1)});auto Press=F.Map(F.Packet({P}),1);const auto Started=IdentifyEdge(Press,0);
        P=F.Publish(EDesktopBatchMode::FreshBaseline,{F.Key(0),F.MouseRead(1)});auto Reset=F.Map(F.Packet({P},true),2);
        const auto& T=Terminal(F.Mapper->DesktopTerminals(),EDesktopSubsourceKind::Mouse,3);
        Check(Reset.GetData().Transitions.empty()&&T.Cause==EIngressCause::Resync&&T.Clock==EDesktopTerminalClock::PhysicalBoundary
            &&T.StartedEdge&&*T.StartedEdge==*Started,"fresh held resync retires previous identity without inventing raw lifecycle timestamps");
        P=F.Publish(EDesktopBatchMode::Poll,{F.MouseRead(0)});F.Map(F.Packet({P}),3);
        Check(!Terminal(F.Mapper->DesktopTerminals(),EDesktopSubsourceKind::Mouse,3).StartedEdge,"held baseline does not manufacture a new Started identity");
    }
    {
        Fixture F;auto Raw=std::make_shared<QueueRaw>();
        auto Producer=Speed::Input::V2::FDeviceInputProducer::CreateDesktop(Raw,F.C,{77},{EProducerKind::Device,77},0,std::uint32_t(1)<<3);
        Check(bool(Producer),"opt-in physical producer composes the Desktop participant");
        auto P=F.Publish(EDesktopBatchMode::FreshBaseline,{F.Key(0),F.MouseRead(0)});Raw->Samples.push_back(F.Packet({P},true));Check(bool(Producer->Produce(0)),"physical baseline produced");
        Check(Producer->SetLifecyclePaused(true)==ELifecycleResult::Applied&&Producer->SetLifecyclePaused(false)==ELifecycleResult::Applied,"physical producer retains its existing rearm lifecycle");
        P=F.Publish(EDesktopBatchMode::FreshBaseline,{F.Key(0),F.MouseRead(1)});Raw->Samples.push_back(F.Packet({P},true));auto Held=Producer->Produce(1);
        Check(Held&&Held->GetData().Transitions.empty()&&Held->GetData().Values[3]==0,"resume-held action is suppressed without a fake Started");
        auto A=F.Publish(EDesktopBatchMode::Poll,{F.MouseRead(2)});auto B=F.Publish(EDesktopBatchMode::Poll,{F.MouseRead(3)});
        Raw->Samples.push_back(F.Packet({A,B}));auto Repress=Producer->Produce(2);
        Check(Repress&&Repress->GetData().Transitions.size()==2&&Repress->GetData().Transitions[0].Action==4
            &&Repress->GetData().Transitions[1].Action==3,"release/repress rearms while preserving another action in the final frame");
        const auto OtherEdge=IdentifyEdge(*Repress,0),BoostEdge=IdentifyEdge(*Repress,1);
        P=F.Publish(EDesktopBatchMode::Poll,{F.MouseRead(0)});Raw->Samples.push_back(F.Packet({P}));Check(bool(Producer->Produce(3)),"release final physical frame produced");
        const auto& Boost=Terminal(Producer->DesktopTerminals(),EDesktopSubsourceKind::Mouse,3);
        const auto& Other=Terminal(Producer->DesktopTerminals(),EDesktopSubsourceKind::Mouse,4);
        Check(Boost.StartedEdge&&BoostEdge&&*Boost.StartedEdge==*BoostEdge&&Other.StartedEdge&&OtherEdge&&*Other.StartedEdge==*OtherEdge,
            "terminal identities name final immutable post-rearm ordinals, never pre-filter indices");
    }
    std::cout<<"PASS DesktopParticipantProbe checks="<<Checks<<'\n';
}
