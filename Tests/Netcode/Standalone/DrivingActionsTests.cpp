#include "IAmSpeed/Driving/DrivingInput.h"
#include "IAmSpeed/Input/ActionMapping.h"
#include "IAmSpeed/Input/ActionDispatch.h"
#include "IAmSpeed/Input/Testing/TestInputProducerV2.h"
#include "IAmSpeed/Netcode/transport/NetInputV2Bridge.h"
#include <iostream>
#include <stdexcept>

namespace V2 = Speed::Input::V2;
namespace Wire = Speed::Netcode;
using Speed::Input::FActionId;
static unsigned Checks = 0;
#define CHECK(X) do { ++Checks; if (!(X)) throw std::runtime_error(std::to_string(__LINE__) + ": " #X); } while (false)
static auto Contract() { return Speed::Driving::MakeContract(Speed::Driving::FBindings{}); }
struct FCell { V2::ERawDeviceKind Device; V2::FRawControl Control; float Value; FActionId Action; int Expected; };
static const FCell Cells[] = {
    {V2::ERawDeviceKind::Keyboard,{V2::ERawControlKind::KeyboardUsage,0x52},1,0,255},
    {V2::ERawDeviceKind::Keyboard,{V2::ERawControlKind::KeyboardUsage,0x51},1,1,255},
    {V2::ERawDeviceKind::Keyboard,{V2::ERawControlKind::KeyboardUsage,0x50},1,2,-127},
    {V2::ERawDeviceKind::Keyboard,{V2::ERawControlKind::KeyboardUsage,0x4f},1,2,127},
    {V2::ERawDeviceKind::Keyboard,{V2::ERawControlKind::KeyboardUsage,0x13},1,3,1},
    {V2::ERawDeviceKind::Keyboard,{V2::ERawControlKind::KeyboardUsage,0x29},1,3,1},
    {V2::ERawDeviceKind::Keyboard,{V2::ERawControlKind::KeyboardUsage,0x15},1,4,1},
    {V2::ERawDeviceKind::Keyboard,{V2::ERawControlKind::KeyboardUsage,0x0d},1,5,-127},
    {V2::ERawDeviceKind::Keyboard,{V2::ERawControlKind::KeyboardUsage,0x0f},1,5,127},
    {V2::ERawDeviceKind::Keyboard,{V2::ERawControlKind::KeyboardUsage,0x0c},1,6,-127},
    {V2::ERawDeviceKind::Keyboard,{V2::ERawControlKind::KeyboardUsage,0x0e},1,6,127},
    {V2::ERawDeviceKind::Gamepad,{V2::ERawControlKind::PadAxis,5},1,0,255},
    {V2::ERawDeviceKind::Gamepad,{V2::ERawControlKind::PadAxis,4},1,1,255},
    {V2::ERawDeviceKind::Gamepad,{V2::ERawControlKind::PadAxis,0},-1,2,-127},
    {V2::ERawDeviceKind::Gamepad,{V2::ERawControlKind::PadAxis,0},1,2,127},
    {V2::ERawDeviceKind::Gamepad,{V2::ERawControlKind::PadButton,12},1,3,1},
    {V2::ERawDeviceKind::Gamepad,{V2::ERawControlKind::PadButton,13},1,4,1},
    {V2::ERawDeviceKind::Gamepad,{V2::ERawControlKind::PadAxis,2},-1,5,-127},
    {V2::ERawDeviceKind::Gamepad,{V2::ERawControlKind::PadAxis,2},1,5,127},
    {V2::ERawDeviceKind::Gamepad,{V2::ERawControlKind::PadAxis,3},1,6,-127},
    {V2::ERawDeviceKind::Gamepad,{V2::ERawControlKind::PadAxis,3},-1,6,127}
};
static std::vector<V2::FInputFrame> Frames(const FCell& Cell)
{
    auto C=Contract(); CHECK(C && C->GetDescription().Actions.size()==7);
    auto Mapper=V2::FActionMapper::Create(C,{7},{Speed::Input::EProducerKind::Device,99}); CHECK(Mapper);
    std::vector<V2::FInputFrame> Result;
    for (unsigned N=0;N<5;++N)
    {
        V2::FRawInputSample S; S.DeviceId=10; S.Generation={1}; S.Kind=Cell.Device; S.Sequence=N+1;
        S.Status=(N==0 || N==4)?V2::ERawSampleStatus::Resync:V2::ERawSampleStatus::Valid;
        const float Value=(N==1 || N==2 || N==4)?Cell.Value:0.f;
        for(const auto& Binding:C->GetDescription().Mapping)
            if(V2::IsControlKindForDevice(Binding.Control.Kind,Cell.Device))
                S.FinalState.push_back({Binding.Control,Binding.Control==Cell.Control?Value:0.f});
        std::sort(S.FinalState.begin(),S.FinalState.end(),[](const auto& A,const auto& B){return A.Control<B.Control;});
        S.FinalState.erase(std::unique(S.FinalState.begin(),S.FinalState.end(),[](const auto& A,const auto& B){return A.Control==B.Control;}),S.FinalState.end());
        if(N==1 || N==3)S.Changes={{{Cell.Control,Value},{N+1,0}}};
        if(N==4)Mapper->RequireFreshBaseline();
        auto M=Mapper->Map(S,N); CHECK(M.Status==V2::EMappingStatus::Mapped && M.Frame);
        Result.push_back(*M.Frame);
    }
    return Result;
}
struct FReceiver
{
    unsigned Started=0,Triggered=0,Completed=0;
    void Action(const V2::FActionEvent& E)
    {
        if(E.State==V2::EStateAction::Started){++Started;CHECK(E.Identity);}
        if(E.State==V2::EStateAction::Triggered){++Triggered;CHECK(!E.Identity);}
        if(E.State==V2::EStateAction::Completed){++Completed;CHECK(E.Identity);}
    }
};
static void Atomic(const FCell& Cell,const std::vector<V2::FInputFrame>& F)
{
    const auto C=Contract();
    for(unsigned N=0;N<5;++N)
    {
        const auto& D=F[N].GetData();const bool Held=N==1 || N==2 || N==4;
        CHECK(D.Values[Cell.Action]==(Held?Cell.Expected:0));
        CHECK(D.ActiveMask==(Held?(uint32_t(1)<<Cell.Action):0));
        for(FActionId A=0;A<32;++A)if(A!=Cell.Action)CHECK(D.Values[A]==0);
        CHECK(D.Reset==(N==0 || N==4));
        CHECK(D.Transitions.size()==((N==1 || N==3)?1u:0u));
        if(!D.Transitions.empty())
        {
            const auto& E=D.Transitions.front();CHECK(E.Action==Cell.Action);
            CHECK(E.State==(N==1?V2::ETransition::Started:V2::ETransition::Completed));
            CHECK(E.ValueAtTransition==(N==1?Cell.Expected:0));
        }
        const auto T=V2::AssembleDrivingTargets(F[N],*C,{7},N);CHECK(T.Valid);
        CHECK(T.ThrottleValue==(Held && Cell.Action==0?255:0));
        CHECK(T.BrakeValue==(Held && Cell.Action==1?255:0));
        CHECK(T.SteeringValue==(Held && Cell.Action==2?Cell.Expected:0));
        CHECK(!V2::AssembleDrivingTargets(F[N],*C,{8},N).Valid);
        CHECK(!V2::AssembleDrivingTargets(F[N],*C,{7},N+1).Valid);
    }
    auto Producer=V2::FTestInputProducer::Create(C,{7},{Speed::Input::EProducerKind::Device,99},0,F);CHECK(Producer);
    auto Stream=std::make_shared<V2::FInputStream>(std::move(Producer),C,V2::FStreamEpoch{7},Speed::Input::FProducerIdentity{Speed::Input::EProducerKind::Device,99});
    V2::FInputPresentationBindings Bindings(Stream);auto R=std::make_shared<FReceiver>();
    for(auto State:{V2::EStateAction::Started,V2::EStateAction::Triggered,V2::EStateAction::Completed})
        CHECK(Bindings.BindAction(std::to_string(unsigned(State)),Cell.Action,State,std::weak_ptr<FReceiver>(R),&FReceiver::Action));
    CHECK(Bindings.Seal() && Stream->Activate());
    for(unsigned N=0;N<5;++N)
    {
        auto Consumed=Stream->Consume(N);CHECK(Consumed.Status==V2::EConsumeStatus::Ready && Consumed.Reservation);
        // This commits INPUT only. No physical world has been stepped.
        CHECK(Stream->ConfirmPhysicalCommit(*Consumed.Reservation,true) && Stream->PublishCompleted(*Consumed.Reservation));
        CHECK(Bindings.HandleInputs()==V2::EDispatchStatus::Dispatched);
        CHECK(Bindings.HandleInputs()==V2::EDispatchStatus::NoChange);
    }
    CHECK(R->Started==1 && R->Triggered==3 && R->Completed==1);
    Stream->Deactivate();CHECK(Bindings.HandleInputs()==V2::EDispatchStatus::Detached);
}
static Wire::FBinding Binding()
{
    Wire::FBinding B;B.Session=11;B.Epoch=7;B.Actor=23;B.Incarnation=2;B.OwnerPeer=101;B.ServerPeer=1;B.SourceProducer=99;
    B.Contract[0]=42;return B; // Authored schema ID: negotiation remains unqualified.
}
static bool Validate(const Wire::FFrame& I)
{
    Wire::FPacket P;auto B=Binding();P.Kind=Wire::EKind::OwnerInput;P.Session=B.Session;P.Epoch=B.Epoch;
    P.Actor=B.Actor;P.Incarnation=B.Incarnation;P.Contract=B.Contract;P.Sequence=1;P.Input=I;
    return bool(Wire::ToV2(P,Contract()));
}
static Wire::FPacket Across(const Wire::FPacket& P)
{auto Bytes=Wire::Encode(P);CHECK(Bytes);auto Out=Wire::Decode(Bytes->data(),Bytes->size());CHECK(Out);return *Out;}
static void Network(const FCell& Cell,const std::vector<V2::FInputFrame>& F)
{
    auto B=Binding();Wire::FExactInbox Server(B,Wire::EKind::OwnerInput,0,Validate),Owner(B,Wire::EKind::AuthorityInput,0,Validate),Remote(B,Wire::EKind::AuthorityInput,0,Validate);
    for(unsigned N=0;N<5;++N)
    {
        auto P=Wire::FromV2(F[N],B,Wire::EKind::OwnerInput,N+1);CHECK(P);
        CHECK(Server.Receive(999,*P)==Wire::EResult::WrongPeer);
        auto Stale=*P;++Stale.Incarnation;CHECK(Server.Receive(B.OwnerPeer,Stale)==Wire::EResult::WrongStream);
        CHECK(Server.Receive(B.OwnerPeer,Across(*P))==Wire::EResult::Accepted);
        CHECK(Server.Receive(B.OwnerPeer,*P)==Wire::EResult::Duplicate);
        auto S=Server.Consume(N);CHECK(S && S->FirstConsume);
        auto Authority=S->Packet;Authority.Kind=Wire::EKind::AuthorityInput;
        for(auto* Inbox:{&Owner,&Remote})
        {
            CHECK(Inbox->Receive(B.OwnerPeer,Authority)==Wire::EResult::WrongPeer);
            CHECK(Inbox->Receive(B.ServerPeer,Across(Authority))==Wire::EResult::Accepted);
            auto Read=Inbox->Consume(N);CHECK(Read && Read->FirstConsume);
            auto V=Wire::ToV2(Read->Packet,Contract());CHECK(V);
            const auto& A=V->GetData();const auto& E=F[N].GetData();
            CHECK(A.Values==E.Values && A.ActiveMask==E.ActiveMask && A.Reset==E.Reset);
            CHECK(A.ConsumptionFrame==N && A.SourceSequence==E.SourceSequence && A.DeviceGeneration.Value==E.DeviceGeneration.Value);
            CHECK(A.Transitions.size()==E.Transitions.size());
            for(size_t K=0;K<A.Transitions.size();++K)
                CHECK(A.Transitions[K].Action==Cell.Action && A.Transitions[K].State==E.Transitions[K].State
                    && A.Transitions[K].ValueAtTransition==E.Transitions[K].ValueAtTransition
                    && A.Transitions[K].Order.Sequence==E.Transitions[K].Order.Sequence
                    && A.Transitions[K].Order.WithinSequence==E.Transitions[K].Order.WithinSequence);
            auto Again=Inbox->Consume(N);CHECK(Again && !Again->FirstConsume);
            CHECK(!Inbox->Consume(N+1)); // No invented held/release on missing frame.
        }
    }
}
int main(int Argc,char**)
{
    try
    {
        for(const auto& Cell:Cells){auto F=Frames(Cell);if(Argc>1)Network(Cell,F);else Atomic(Cell,F);}
        std::cout<<"PASS cells=21 frames=105 checks="<<Checks<<" scope="<<(Argc>1?"portable-wire-no-sockets":"atomic-input-no-physics")<<'\n';return 0;
    }
    catch(const std::exception& E){std::cerr<<"FAIL "<<E.what()<<'\n';return 1;}
}
