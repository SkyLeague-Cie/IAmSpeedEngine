#include "IAmSpeed/Input/Testing/TestInputProducer.h"
#include "IAmSpeed/Input/DeviceInputSession.h"
#include "IAmSpeed/Input/InputStream.h"
#include "IAmSpeed/Input/DrivingInputTargets.h"
#include <cstdlib>
#include <iostream>
using namespace Speed::Input;
static unsigned Checks=0;
static void Check(bool V,const char* M) {++Checks; if(!V){std::cerr<<"FAIL "<<M<<'\n';std::exit(1);}}
constexpr FActionId Jump=3, Boost=4; // Example game-owned slots, not engine mappings.
static FActionValues Values(int T,int S,int J,int B)
{FActionValues V{};V[Throttle]=static_cast<std::int16_t>(T);V[Steering]=static_cast<std::int16_t>(S);V[Jump]=static_cast<std::int16_t>(J);V[Boost]=static_cast<std::int16_t>(B);return V;}
static std::uint64_t Hash(const FInputFrame& F)
{
    std::uint64_t H=14695981039346656037ull;
    const auto Add=[&](std::uint64_t V){for(unsigned I=0;I<8;++I){H^=V&255;H*=1099511628211ull;V>>=8;}};
    Add(F.GetSourceFrame());Add(F.GetConsumptionFrame());Add(static_cast<unsigned>(F.GetProducer().Kind));Add(F.GetProducer().Id);Add(F.RequiresReset());
    for(auto V:F.GetActions()) Add(static_cast<std::uint16_t>(V));
    Add(F.GetEdgeCount());for(const auto& E:F.GetEdges()){Add(E.Action);Add(static_cast<unsigned>(E.Kind));Add(E.SourceFrame);}return H;
}
static std::vector<FInputFrame> Scenario(FProducerIdentity Id={EProducerKind::Device,77})
{
    std::array<FActionEdge,MaxEdges> Press{},Cycle{},Release{};
    Press[0]={Jump,EEdgeKind::Start,3};Press[1]={Boost,EEdgeKind::Start,3};
    Cycle[0]={Jump,EEdgeKind::Stop,4};Cycle[1]={Jump,EEdgeKind::Start,5};
    Release[0]={Jump,EEdgeKind::Stop,9};
    return {FInputFrame(2,0,Id,Values(20,-20,0,0),{},0,true),FInputFrame(3,1,Id,Values(40,20,1,1),Press,2),
        FInputFrame(5,2,Id,Values(40,20,1,1),Cycle,2),FInputFrame(8,3,Id,Values(5,0,1,0),{},0,true),
        FInputFrame(9,4,Id,Values(5,0,0,0),Release,1)};
}
struct FResult {std::vector<std::uint64_t> History;std::vector<FFrameNumber> Presented;};
static FResult Run(bool Device,bool SlowPresentation)
{
    const auto Expected=Scenario();
    std::array<bool,ActionCount> Digital{};Digital[Jump]=Digital[Boost]=true;
    auto Live=std::make_shared<FDeviceInputSession>(77,Digital);
    auto Generation=Live->SetConnected(true);Check(Generation.has_value(),"device initial generation");
    std::shared_ptr<IInputProducer> Producer=Device ? std::static_pointer_cast<IInputProducer>(Live)
        : std::shared_ptr<IInputProducer>(FTestInputProducer::Create({EProducerKind::Device,77},0,Expected));
    Check(bool(Producer),"scenario producer factory");FInputStream Stream(Producer);FInputPresentationBindings Bindings;FResult Result;
    unsigned CallbackCount=0;
    Check(Bindings.BindAction("JumpPresentation",Jump,[&](const FInputFrame& F,FActionId A){
        Check(A==Jump,"game extension slot");++CallbackCount;Result.Presented.push_back(F.GetConsumptionFrame());
        Check(!Producer->Produce(F.GetConsumptionFrame()+1),"presentation cannot advance producer");
        Check(!Stream.Consume(F.GetConsumptionFrame()+1) && !Stream.PublishCompleted(F.GetConsumptionFrame()),"presentation cannot mutate stream");
        Check(!Live->Submit(*Generation,99,Values(255,127,1,1)),"presentation cannot inject physical device values");
    }),"presentation binding");
    for(FFrameNumber N=0;N<Expected.size();++N){
        if(Device){
            if(N==0) Check(Live->Submit(*Generation,1,Values(20,-20,0,0)),"initial baseline");
            if(N==1) Check(Live->Submit(*Generation,2,Values(40,20,1,1)),"press sample");
            if(N==2){Check(Live->Submit(*Generation,3,Values(40,20,0,1)),"inter-poll stop");Check(Live->Submit(*Generation,4,Values(40,20,1,1)),"inter-poll start");}
            if(N==3){Check(Live->SetPaused(true).has_value(),"pause");Generation=Live->SetPaused(false);Check(Generation && Live->Submit(*Generation,1,Values(5,0,1,0)),"fresh held resume");}
            if(N==4) Check(Live->Submit(*Generation,2,Values(5,0,0,0)),"release");
        }
        const auto Before=Stream.ReadLatest();const auto F=Stream.Consume(N);
        Check(F && Hash(*F)==Hash(Expected[N]),"exact values edges reset clocks identity");
        const auto Targets=ReadDrivingInputTargets(F,N);
        Check(Targets.Valid && Targets.ThrottleValue==Expected[N].GetActions()[Throttle] && Targets.BrakeValue==0
            && Targets.SteeringValue==Expected[N].GetActions()[Steering],"shared actual component target helper");
        Check(!Before || Stream.ReadLatest()->Serial==Before->Serial,"consume does not publish");
        Check(Before || !Stream.ReadLatest(),"initial consume remains unpublished");
        Result.History.push_back(Hash(*F));
        // Represents successful completion only; the UE simulation is not run here.
        Check(Stream.PublishCompleted(N),"publish after simulated successful completion");
        if(!SlowPresentation || N==0 || N==4){for(unsigned Tick=0;Tick<3;++Tick) Bindings.HandleInputs(*Stream.ReadLatest());}
        Check(Hash(*Stream.ReadRecorded(N))==Result.History.back(),"callbacks cannot alter recorded input");
    }
    Check(CallbackCount==(SlowPresentation?2u:5u),"one callback per observed serial");
    for(FFrameNumber N=0;N<Expected.size();++N)Check(Hash(*Stream.ReadRecorded(N))==Result.History[N],"history stable after subsequent frames");
    Stream.Deactivate();Check(!Stream.Consume(5) && !Stream.PublishCompleted(4),"teardown stops consume and publish");
    return Result;
}
int main()
{
    const auto Fast=Run(false,false),Slow=Run(false,true),LiveFast=Run(true,false),LiveSlow=Run(true,true);
    Check(Fast.History==Slow.History && Fast.History==LiveFast.History && Fast.History==LiveSlow.History,"device/test parity independent of presentation cadence");
    Check(Fast.Presented==std::vector<FFrameNumber>({0,1,2,3,4}) && Slow.Presented==std::vector<FFrameNumber>({0,4}),"coalesced presentation retains exact physical history");
    for(auto Kind:{EProducerKind::Device,EProducerKind::AI,EProducerKind::Network}){
        const FProducerIdentity Id{Kind,88};auto S=Scenario(Id);auto P=FTestInputProducer::Create(Id,0,S);
        Check(P && !P->Produce(1),"initial gap rejected");const auto H=Hash(S[0]);S[0]=FInputFrame(2,0,Id,Values(255,0,0,0));
        Check(Hash(*P->Produce(0))==H && Hash(*P->Produce(0))==H,"scenario copied and replay exact");
        Check(!P->Skip(1) && !P->Produce(2),"skip rejected without advancing cursor");
        for(FFrameNumber N=1;N<5;++N) Check(P->Produce(N).has_value(),"contiguous timeline");
        Check(!P->Produce(5) && Hash(*P->Produce(4))==Hash(S[4]),"missing terminal no hold or invention");
    }
    const FProducerIdentity Id{EProducerKind::Device,77};
    Check(!FTestInputProducer::Create(Id,0,{}),"empty scenario rejected");
    auto Invalid=Scenario();Invalid[4]=FInputFrame(9,4,Id,Values(256,0,0,0));Check(!FTestInputProducer::Create(Id,0,Invalid),"invalid tail rejects entire scenario");
    Invalid=Scenario();Invalid[2]=Invalid[1];Check(!FTestInputProducer::Create(Id,0,Invalid),"duplicate frame rejected");
    Check(!FTestInputProducer::Create(Id,1,Scenario()),"wrong embedded consumption frame");
    Check(!FTestInputProducer::Create({EProducerKind::AI,77},0,Scenario()),"wrong explicit identity");
    const auto Last=std::numeric_limits<FFrameNumber>::max();
    std::vector<FInputFrame> End{FInputFrame(1,Last,Id,{})};auto P=FTestInputProducer::Create(Id,Last,End);
    Check(P && P->Produce(Last) && P->Produce(Last) && !P->Produce(0),"terminal frame does not wrap");
    End.push_back(FInputFrame(2,0,Id,{}));Check(!FTestInputProducer::Create(Id,Last,End),"overflow timeline rejected");
    std::vector<FInputFrame> Long;for(FFrameNumber N=0;N<=HistoryCapacity;++N)Long.emplace_back(N,N,Id,FActionValues{});
    P=FTestInputProducer::Create(Id,0,Long);for(FFrameNumber N=0;N<Long.size();++N)Check(P->Produce(N).has_value(),"long contiguous scenario");
    Check(!P->Produce(0) && P->Produce(1),"bounded replay matches retained history window");
    Check(!ReadDrivingInputTargets({},0).Valid && !ReadDrivingInputTargets(Scenario()[0],1).Valid,"shared helper missing/wrong frame fails closed");
    auto Incomplete=std::shared_ptr<IInputProducer>(FTestInputProducer::Create(Id,0,Scenario()));FInputStream Stream(Incomplete);
    Check(Stream.Consume(0).has_value() && !Stream.ReadLatest(),"failed physical step not published");
    Stream.Deactivate();Check(!Stream.PublishCompleted(0),"cancelled step cannot publish later");
    std::cout<<"PASS TestInputProducerProbe checks="<<Checks<<" common_consumer=driving_targets UE_simulation=not_run\n";
}
