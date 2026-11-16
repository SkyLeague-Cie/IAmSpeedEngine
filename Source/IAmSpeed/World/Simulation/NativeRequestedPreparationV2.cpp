#include "NativeRequestedPreparationV2.h"
#include "SpeedPhysicsSimulation.h"
#include "CanonicalFrameContext.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "IAmSpeed/Components/ISpeedComponent.h"

bool ASpeedPhysicsSimulation::ConfigureRequestedNetworkPreparationV2(
    std::shared_ptr<const Speed::Netcode::Agreement::FHostIdentity> Host)
{
    if(!IsInGameThread()||SimulationWorker||InputSessionRegistry||RequestedPreparationV2||!InputWorkerGeneration
        ||!NativeInputAdmissionOwner||!CompletedPublicationSource||!bPublishCompletedWorldInput
        ||bInputOwnerRetired.Load()||bOwnedWorkerTerminal.Load()||bCanonicalPublicationTerminal.Load()
        ||!Host||Host->ReadCloseFence())return false;
    try{RequestedPreparationV2=std::make_shared<Speed::FRequestedNetworkPreparationCommandsV2>(InputWorkerGeneration,std::move(Host));return true;}
    catch(...){return false;}
}
Speed::ERequestedPreparationAdmissionV2 ASpeedPhysicsSimulation::SubmitRequestedNetworkPreparationV2(
    Speed::FRequestedNetworkPreparationRequestV2 Request)
{
    using E=Speed::ERequestedPreparationAdmissionV2;
    if(!IsInGameThread())return E::WrongThread;
    if(!RequestedPreparationV2||GetActiveExecutionMode()!=ESimulationExecutionMode::IAmSpeedThread
        ||!bPublishCompletedWorldInput||!CompletedPublicationSource||bInputOwnerRetired.Load()
        ||bOwnedWorkerTerminal.Load()||bCanonicalPublicationTerminal.Load())return E::Closed;
    const auto Result=RequestedPreparationV2->Submit(std::move(Request));
    if(Result==E::Enqueued)bInputSessionAdmissionRequested.Store(true);
    return Result;
}
std::optional<Speed::FRequestedPreparationReceiptV2> ASpeedPhysicsSimulation::ReadRequestedNetworkPreparationReceiptV2(uint64 G,uint64 Id)const
{return IsInGameThread()&&RequestedPreparationV2?RequestedPreparationV2->Read(G,Id):std::nullopt;}

void ASpeedPhysicsSimulation::ServiceRequestedNetworkPreparationV2AtBoundary()
{
    using namespace Speed;using namespace Speed::Input::V2;using namespace Speed::Netcode;using namespace Speed::Netcode::Agreement;
    if(!RequestedPreparationV2)return;
    if(!SimulationWorker||!SimulationWorker->OwnsCurrentInputBoundary()||!InputSessionRegistry
        ||!SpeedWorldSubsystem||SpeedWorldSubsystem->GetWorld()!=GetWorld())throw std::runtime_error("V2 preparation outside native boundary");
    auto Batch=RequestedPreparationV2->CaptureOnNativeOwner();
    if(Batch.Requests.empty())return;
    for(const auto& P:Batch.Requests)
    {
        ERequestedPreparationReasonV2 Reason=ERequestedPreparationReasonV2::None;
        auto Context=BuildRequestedPreparationV2AtBoundary(*P.Request,Batch.Cutoff,P.Deadline,Reason);
        RequestedPreparationV2->CompleteOnNativeOwner(P.Index,
            Reason==ERequestedPreparationReasonV2::None?ERequestedPreparationStatusV2::ComparedOnly:ERequestedPreparationStatusV2::Rejected,
            Reason,std::move(Context));
    }
}

std::shared_ptr<const Speed::FNativeRequestedPreparationContextV2> ASpeedPhysicsSimulation::BuildRequestedPreparationV2AtBoundary(
    const Speed::FRequestedNetworkPreparationRequestV2& R,uint64 Cutoff,std::chrono::steady_clock::time_point Deadline,
    Speed::ERequestedPreparationReasonV2& Reason)
{
    using namespace Speed;using namespace Speed::Input::V2;using namespace Speed::Netcode;using namespace Speed::Netcode::Agreement;
    std::shared_ptr<const FNativeRequestedPreparationContextV2> Context;
    if(!SimulationWorker||!SimulationWorker->OwnsCurrentInputBoundary()||!InputSessionRegistry||!SpeedWorldSubsystem
        ||!RequestedPreparationV2||!R.Service||!R.Agreement||!R.Command.Binding.Network
        ||R.Command.Binding.Network->Version!=2||!R.Command.Binding.Network->IsValid())
    {Reason=ERequestedPreparationReasonV2::NativeBoundary;return {};}
    const bool MappingsPrepared=SpeedWorldSubsystem->PrepareRequestedNetworkAdmissionBoundary();
        try
        {
            const auto& C=R.Command;const auto& D=C.Binding;
            const auto& N=*D.Network;const auto& L=*N.Agreement;
            const auto Registry=InputSessionRegistry->ReadRegistry();
            // Capture queue first, fences second: deliberately NOT an atomic
            // queue/fence snapshot. A newer fence may conservatively reject.
            const auto Close=R.Agreement->ReadClosure();
            const auto& Record=*R.Agreement;const auto& O=Record.Offer();const auto& Received=*Record.Received();
            const auto& Config=R.Service->GetConfiguration();const auto& Contract=R.Service->GetContract();
            const auto& WireReply=Received.ReceivedCopy();
            const auto ActualClock=MakeCanonicalFrameContext(CanonicalNumFrame);
            const auto* Component=MappingsPrepared?SpeedWorldSubsystem->ResolveRequestedPreparationMappingV2(D):nullptr;
            if(std::chrono::steady_clock::now()>=Deadline)Reason=ERequestedPreparationReasonV2::Expired;
            else if(Close.Host||Close.Slot||Close.Socket)Reason=ERequestedPreparationReasonV2::Closed;
            else if(C.WorkerGeneration!=InputWorkerGeneration||N.WorldGeneration!=InputWorkerGeneration
                ||RequestedPreparationV2->WorkerGeneration()!=InputWorkerGeneration)Reason=ERequestedPreparationReasonV2::Generation;
            else if(!Registry||Registry->Terminal||Registry->Version!=C.RegistryVersion||Registry->WorkerGeneration!=InputWorkerGeneration)
                Reason=ERequestedPreparationReasonV2::Registry;
            else if(!Component)Reason=ERequestedPreparationReasonV2::Mapping;
            else if(Record.Host()!=RequestedPreparationV2->Host()||Record.Host()->Generation()!=L.HostGeneration
                ||Record.Host()->Match()!=N.Session||Record.Host()->MatchEpoch()!=N.Epoch)Reason=ERequestedPreparationReasonV2::HostIdentity;
            else if(Record.SlotId()!=L.Slot||Received.Connection()->Instance()!=L.Connection
                ||Received.Stage()!=EObservedStage::AcceptReceivedAtServer||O.Match!=N.Session||O.MatchEpoch!=N.Epoch
                ||O.Actor!=N.Actor||O.Incarnation!=N.Incarnation||O.Component!=N.Component
                ||O.OfferSequence!=L.OfferSequence||O.StreamEpoch!=L.StreamEpoch
                ||O.GameCodec!=L.GameCodec||O.GameCodecVersion!=L.GameCodecVersion||O.WireContract!=N.WireContract)
                Reason=ERequestedPreparationReasonV2::AgreementIdentity;
            else if(!Contract||Contract->GetFingerprint()!=D.Contract->GetFingerprint()||!Config.AllowLocalFallback
                ||Config.FirstFrame!=D.First||Config.Binding.Session!=N.Session||Config.Binding.Epoch!=N.Epoch
                ||Config.Binding.Actor!=N.Actor||Config.Binding.Incarnation!=N.Incarnation||Config.Binding.ContractId!=N.WireContract
                ||Config.AuthenticatedPeer!=Received.Connection()->Instance()||Config.Original.SourceSequence
                ||O.OriginalProducerKind!=(Config.Original.Kind==Input::EProducerKind::Device?1ull:Config.Original.Kind==Input::EProducerKind::AI?2ull:0ull)
                ||O.OriginalProducer!=Config.Original.Producer||O.OriginalStreamEpoch!=Config.Original.StreamEpoch
                ||O.OriginalDeviceGeneration!=Config.Original.DeviceGeneration
                ||L.ProfileRoute!=(Config.Consumer==ENetworkInputConsumer::DedicatedServer?1:2))
                Reason=ERequestedPreparationReasonV2::ServiceConfiguration;
            else if(!bCanonicalFrameInitialized||!ActualClock.LogicalRateNumerator||!ActualClock.LogicalRateDenominator
                ||ActualClock.FrameOrigin>CanonicalNumFrame||O.FirstFrame!=D.First||D.First!=CanonicalNumFrame
                ||O.LogicalRateNumerator!=ActualClock.LogicalRateNumerator||O.LogicalRateDenominator!=ActualClock.LogicalRateDenominator
                ||O.FrameOrigin!=ActualClock.FrameOrigin||ActualClock.NumFrame!=CanonicalNumFrame
                ||!FMath::IsFinite(ActualClock.PhysicalDeltaTime)||ActualClock.PhysicalDeltaTime<=0
                ||ActualClock.PhysicalDeltaTime!=float(ActualClock.LogicalRateDenominator)/float(ActualClock.LogicalRateNumerator)
                ||ActualClock.SimTime!=float(CanonicalNumFrame-ActualClock.FrameOrigin)*ActualClock.PhysicalDeltaTime)
                Reason=ERequestedPreparationReasonV2::Clock;
            else
            {
                const auto Encoded=FWireCodec::EncodeOffer(O);auto Reply=FWireCodec::DecodeOffer(WireReply.data(),WireReply.size());
                const bool RoutePresent=std::any_of(O.Profiles.begin(),O.Profiles.end(),[&](const auto& V){return std::uint8_t(V.Route)==L.ProfileRoute;});
                if(!Encoded||!Reply||*Encoded!=WireReply||*Encoded!=Received.SentOfferCopy()||!RoutePresent)
                    Reason=ERequestedPreparationReasonV2::Profile;
                else
                {
                    FNativeRequestedPreparationBoundaryV2 B;
                    B.WorkerGeneration=InputWorkerGeneration;B.RegistryVersion=Registry->Version;B.CanonicalFrame=CanonicalNumFrame;B.QueueCutoff=Cutoff;
                    B.LocalComponent=Component->GetPublishedSimulationStableId();B.ComponentGeneration=N.LocalComponentGeneration;B.BodyGeneration=Component->GetCanonicalRequestedBodyGeneration();
                    B.LogicalRateNumerator=ActualClock.LogicalRateNumerator;B.LogicalRateDenominator=ActualClock.LogicalRateDenominator;B.FrameOrigin=ActualClock.FrameOrigin;B.PhysicalDeltaTime=ActualClock.PhysicalDeltaTime;B.Closures=Close;
                    Context=std::shared_ptr<const FNativeRequestedPreparationContextV2>(new FNativeRequestedPreparationContextV2(C,Config,Contract,R.Agreement,std::move(*Reply),B));
                    // A true game result is comparison ONLY. No source factory,
                    // FNativeNetworkAdmission or readiness call follows here.
                    try{if(!Component->CompareCanonicalRequestedPreparationV2(*Context))Reason=ERequestedPreparationReasonV2::GameRejected;}
                    catch(...){Reason=ERequestedPreparationReasonV2::GameException;}
                    const auto After=Record.ReadClosure();if(After.Host||After.Slot||After.Socket)Reason=ERequestedPreparationReasonV2::Closed;
                    if(std::chrono::steady_clock::now()>=Deadline)Reason=ERequestedPreparationReasonV2::Expired;
                }
            }
        }
        catch(...){Reason=ERequestedPreparationReasonV2::AllocationFailure;}
    return Context;
}
