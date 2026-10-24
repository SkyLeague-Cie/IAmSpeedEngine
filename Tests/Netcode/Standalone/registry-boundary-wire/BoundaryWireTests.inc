#include "RegistryBoundaryWireAdapter.h"
#include "../state-authority-envelope-v2/StateAuthorityEnvelope.h"
namespace M=Speed::Netcode::BoundaryMetadata;
static void BoundaryWireActualRegistryViews()
{
    FFixture F;const auto C=F.Registry.CaptureCompletedBoundary(0);CHECK(C);
    auto T=F.Registry.ReserveCompletedBoundary(C);CHECK(T);
    const auto P=M::PrepareRegistryMetadata(F.Registry,*T);CHECK(P && P->OriginCapture()==C);
    SaveBoundaryWireSample("six-session-paused.irb",P->CanonicalBytes());
    CHECK(P->Metadata().Consumed.size()==6 && P->Metadata().Current.size()==6 && P->Metadata().Closed.size()==6);
    CHECK(P->Metadata().ConsumedRegistryVersion==P->Metadata().CurrentRegistryVersion);
    for(const auto& B:P->Metadata().Consumed)CHECK(B.Phase==M::EPhase::NotCaptured);
    for(const auto& B:P->Metadata().Current)CHECK(B.Phase==M::EPhase::Paused);
    CHECK(!Speed::Netcode::StateAuthorityV2::Decode(P->CanonicalBytes())); // Different explicit subformat; no full-state upgrade.
    for(int Role=0;Role<2;++Role)
    {
        const auto D=M::Decode(P->CanonicalBytes());CHECK(D && M::Encode(*D)==P->CanonicalBytes());
        CHECK(D->WorkerGeneration==23 && D->CompletedInputFrame==0 && D->Closed.front().SourceSequence==1);
        CHECK(D->Consumed.front().Actors[0].Id==1001 && D->Consumed.front().Actors[0].InputGeneration==5);
        CHECK(D->Closed.front().OwnerBindingFingerprint==C->Completed()->Inputs.front().Snapshot->BindingFingerprint);
    }
    F.Registry.ReleaseCompletedBoundaryReservation(*T);CHECK(!M::PrepareRegistryMetadata(F.Registry,*T));
    auto Detach=F.Command(V::EBoundaryOperation::Detach);Detach.Session=1;Detach.Epoch=7;
    CHECK(F.Commands->Submit(Detach)==V::ECommandAdmission::Enqueued && F.Registry.ServiceBoundary(1));
    const auto After=F.Registry.CaptureCompletedBoundary(0);T=F.Registry.ReserveCompletedBoundary(After);CHECK(T);
    const auto Removed=M::PrepareRegistryMetadata(F.Registry,*T);CHECK(Removed);
    SaveBoundaryWireSample("six-consumed-five-current.irb",Removed->CanonicalBytes());
    CHECK(Removed->Metadata().Consumed.size()==6 && Removed->Metadata().Current.size()==5);
    CHECK(Removed->Metadata().CurrentRegistryVersion>Removed->Metadata().ConsumedRegistryVersion);
    CHECK(Removed->Metadata().Consumed.front().Session==1 && Removed->Metadata().Current.front().Session==2);
    F.Registry.ReleaseCompletedBoundaryReservation(*T);
    for(std::uint64_t Id=2;Id<=6;++Id)
    {
        Detach=F.Command(V::EBoundaryOperation::Detach);Detach.Session=Id;Detach.Epoch=7;
        CHECK(F.Commands->Submit(Detach)==V::ECommandAdmission::Enqueued && F.Registry.ServiceBoundary(1));
    }
    T=F.Registry.ReserveCompletedBoundary(F.Registry.CaptureCompletedBoundary(0));CHECK(T);
    const auto Empty=M::PrepareRegistryMetadata(F.Registry,*T);CHECK(Empty);
    SaveBoundaryWireSample("six-consumed-empty-current.irb",Empty->CanonicalBytes());
    CHECK(Empty->Metadata().Current.empty() && Empty->Metadata().Consumed.size()==6 && Empty->Metadata().CompletedInputFrame==0);
    CHECK(M::Decode(Empty->CanonicalBytes()));F.Registry.ReleaseCompletedBoundaryReservation(*T);
    CHECK(P->Metadata().Current.size()==6 && Removed->Metadata().Current.size()==5); // Immutable independent views retained.
    // No input step1, native actor resurrection or empty-input world completion was produced.
    static_assert(!M::NativeWorldQualified);
}
static void BoundaryWireReplacement()
{
    FFixture F(1);auto Remove=F.Command(V::EBoundaryOperation::Detach);Remove.Session=1;Remove.Epoch=7;
    CHECK(F.Commands->Submit(Remove)==V::ECommandAdmission::Enqueued && F.Registry.ServiceBoundary(1));
    V::FSessionDescriptor D;D.Id=1;D.Epoch=8;D.Controller=200;D.Producer=100;D.First=1;
    D.Contract=Contract();D.Kind=V::EProducerContract::ExactScenario;D.Actors={{1001,6}};
    auto Data=MakeFrame(100,1).GetData();Data.StreamEpoch={8};D.Scenario.emplace_back(Contract(),std::move(Data));
    auto Bind=F.Command(V::EBoundaryOperation::Bind);Bind.Binding=D;
    CHECK(F.Commands->Submit(Bind)==V::ECommandAdmission::Enqueued && F.Registry.ServiceBoundary(1));
    CHECK(F.Commands->Read(Bind.Id)->Status==V::EBoundaryStatus::Applied);
    const auto C=F.Registry.CaptureCompletedBoundary(0);auto T=F.Registry.ReserveCompletedBoundary(C);CHECK(T);
    const auto P=M::PrepareRegistryMetadata(F.Registry,*T);CHECK(P);
    SaveBoundaryWireSample("same-session-new-epoch-input-generation.irb",P->CanonicalBytes());
    const auto W=M::Decode(P->CanonicalBytes());CHECK(W);
    CHECK(W->Consumed[0].Session==1 && W->Consumed[0].Epoch==7 && W->Consumed[0].Actors[0].InputGeneration==5);
    CHECK(W->Current[0].Session==1 && W->Current[0].Epoch==8 && W->Current[0].Actors[0].InputGeneration==6);
    CHECK(W->Closed[0].Epoch==7 && W->Current[0].First==1 && W->Current[0].Phase==M::EPhase::Active);
    CHECK(F.Registry.PollCount(1)==0); // Replacement was explicitly bound, not polled or reset by export.
    F.Registry.ReleaseCompletedBoundaryReservation(*T);
}
static void BoundaryWireMalformedAndBounds()
{
    FFixture F(1);auto T=F.Registry.ReserveCompletedBoundary(F.Registry.CaptureCompletedBoundary(0));CHECK(T);
    const auto P=M::PrepareRegistryMetadata(F.Registry,*T);CHECK(P);const auto Bytes=P->CanonicalBytes();
    for(std::size_t End=0;End<Bytes.size();++End)CHECK(!M::Decode({Bytes.begin(),Bytes.begin()+End}));
    auto Bad=Bytes;Bad.push_back(0);CHECK(!M::Decode(Bad));
    for(std::size_t Offset:{std::size_t(0),std::size_t(4),std::size_t(6),std::size_t(12),std::size_t(54),M::HeaderSize+61})
    {Bad=Bytes;Bad[Offset]^=1;CHECK(!M::Decode(Bad));}
    Bad=Bytes;Bad[48]=65;CHECK(!M::Decode(Bad));
    Bad=Bytes;Bad[M::HeaderSize+49]=2;CHECK(!M::Decode(Bad)); // Boolean append flag.
    Bad=Bytes;Bad[M::HeaderSize+60]=0;CHECK(!M::Decode(Bad)); // Consumed phase must remain explicitly uncaptured.
    const auto Reject=[&](M::FMetadata X){CHECK(!M::Encode(X));};
    auto X=P->Metadata();X.WorkerGeneration=0;Reject(X);
    X=P->Metadata();X.CompletedInputFrame=UINT64_MAX;Reject(X);
    X=P->Metadata();X.CurrentRegistryVersion=0;Reject(X);
    X=P->Metadata();X.Current.clear();Reject(X); // Same version cannot silently replace the binding set.
    X=P->Metadata();X.Closed.clear();Reject(X);
    X=P->Metadata();X.Consumed.clear();X.Closed.clear();Reject(X); // No fabricated empty world frame.
    X=P->Metadata();++X.Current[0].Epoch;Reject(X);
    X=P->Metadata();X.Current[0].Phase=M::EPhase::NotCaptured;Reject(X);
    X=P->Metadata();X.Consumed[0].Actors[0].InputGeneration=0;Reject(X);
    X=P->Metadata();X.Closed[0].CutoffSequence=0;Reject(X);
    X=P->Metadata();X.Closed[0].OwnerBindingFingerprint.clear();Reject(X);
    X=P->Metadata();X.Consumed[0].Origin=M::EOrigin::AI;X.Consumed[0].Step[0]=1;Reject(X);
    X=P->Metadata();X.Current.push_back(X.Current.front());++X.CurrentRegistryVersion;Reject(X);
    X=P->Metadata();X.Consumed[0].ContractFingerprint.resize(M::MaxFingerprint+1);Reject(X);
    X=P->Metadata();X.CurrentRegistryVersion=2;X.Consumed[0].ContractFingerprint.resize(M::MaxFingerprint);
    X.Current[0].ContractFingerprint.resize(M::MaxFingerprint);X.Closed[0].OwnerBindingFingerprint.resize(M::MaxFingerprint);
    CHECK(M::Encode(X));X.Consumed.push_back(X.Consumed[0]);X.Consumed.back().Session=2;X.Consumed.back().Controller=201;
    X.Consumed.back().Producer=101;X.Consumed.back().Actors[0].Id=1002;
    X.Closed.push_back(X.Closed[0]);X.Closed.back().Session=2;
    CHECK(M::Valid(X) && !M::Encode(X)); // Individually bounded records exceed the whole-block budget.
    F.Registry.ReleaseCompletedBoundaryReservation(*T);
}
static V::FInputSessionRegistry* WireHookRegistry=nullptr;
static const V::FInputSessionRegistry::FCompletedBoundaryReservation* WireHookToken=nullptr;
static bool WireAbortDone=false;
static void AbortReservationDuringExport()
{
    if(!WireAbortDone){WireAbortDone=WireHookRegistry->AbortCompletedBoundaryReservation(*WireHookToken);}
}
static void BoundaryWireFailureAndReentrancy()
{
    FFixture F(1);const auto C=F.Registry.CaptureCompletedBoundary(0);auto T=F.Registry.ReserveCompletedBoundary(C);CHECK(T);
    Budget=10000;Allocations=0;auto Probe=M::PrepareRegistryMetadata(F.Registry,*T);const auto Count=Allocations;Budget=-1;CHECK(Probe && Count>0);
    const auto Bytes=Probe->CanonicalBytes();
    for(int At=0;At<Count;++At)
    {
        Budget=At;Allocations=0;const auto Failed=M::PrepareRegistryMetadata(F.Registry,*T);Budget=-1;
        CHECK(!Failed && F.Registry.ReadReservedCompletedBoundary(*T)==C && F.Registry.PollCount(1)==1);
    }
    auto Foreign=std::async(std::launch::async,[&]{return !M::PrepareRegistryMetadata(F.Registry,*T);});CHECK(Foreign.get());
    {I::FPresentationInputScope Scope;CHECK(!M::PrepareRegistryMetadata(F.Registry,*T));}
    WireHookRegistry=&F.Registry;WireHookToken=&*T;WireAbortDone=false;AllocationHook=AbortReservationDuringExport;
    const auto Reentered=M::PrepareRegistryMetadata(F.Registry,*T);AllocationHook=nullptr;WireHookRegistry=nullptr;WireHookToken=nullptr;
    CHECK(WireAbortDone && !Reentered && F.Registry.ValidateCompletedBoundaryCapture(*C));
    T=F.Registry.ReserveCompletedBoundary(C);CHECK(T);const auto Retry=M::PrepareRegistryMetadata(F.Registry,*T);CHECK(Retry && Retry->CanonicalBytes()==Bytes);
    F.Registry.ReleaseCompletedBoundaryReservation(*T);
    CHECK(Probe->OriginCapture()==C && M::Decode(Probe->CanonicalBytes()));
    std::cout<<"BOUNDARY_METADATA_ALLOCATIONS "<<Count<<"; all preparation failures preserve actual consumed/current input state\n";
}
