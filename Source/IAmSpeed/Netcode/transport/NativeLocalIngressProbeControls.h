#pragma once
// Real-process probe controls; never a physical/source/module admission token.
#include "NativePredictionProcessProbe.h"
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <future>
#include <limits>
#include <thread>

namespace Speed::Netcode::ProcessProbeDetail
{
class FOriginalIngressProbeControls final
{
    std::unique_ptr<FNativeLocalIngress> Ingress;
    std::shared_ptr<const FNativeLocalReceivedMessage> First;
    std::uint64_t ReadyCount=0;
public:
    bool Acquire(FNativeLocalSession& Source)
    {
        ELocalSession Wrong=ELocalSession::Invalid;bool NoExport=false;
        // The original owner stays alive and owns Source through this join.
        std::thread Other([&]{auto P=Source.AcquireIngress(Wrong);NoExport=!P;});Other.join();
        if(Wrong!=ELocalSession::WrongOwner||!NoExport)return false;
        ELocalSession Status=ELocalSession::Invalid;Ingress=Source.AcquireIngress(Status);
        if(!Ingress||Status!=ELocalSession::Ready)return false;
        auto Duplicate=Source.AcquireIngress(Status);
        if(Duplicate||Status!=ELocalSession::ConsumerClaimed)return false;
        FLocalSessionMessage Denied;Denied.Peer=99;Denied.Bytes={1};
        if(Source.Receive(Denied)!=ELocalSession::ConsumerClaimed||Denied.Peer||!Denied.Bytes.empty())return false;
        return true;
    }
    ELocalSession Receive(FLocalSessionMessage& Values)
    {
        Values={};std::shared_ptr<const FNativeLocalReceivedMessage> Original;
        const auto Status=Ingress->Receive(Original);
        if(Status==ELocalSession::Ready)
        {
            if(!Original||ReadyCount==std::numeric_limits<std::uint64_t>::max())return ELocalSession::Full;
            if(!First)First=Original;
            Values=Original->Read();++ReadyCount;
        }
        else if(Original)return ELocalSession::Invalid;
        return Status;
    }
    bool CloseAndRecord(std::unique_ptr<FNativeLocalSession>& Source,const FNativePredictionProbeOptions& O)
    {
        if(!Source||!First||!ReadyCount||Source->LocalEndpoint().Port==0)return false;
        const auto& Received=First->Read();
        if(std::none_of(O.Launcher.Peers.begin(),O.Launcher.Peers.end(),[&](const auto& P){return P.Peer==Received.Peer;}))return false;
        const auto RetainedBytes=Received.Bytes;
        ELocalSession Wrong=ELocalSession::Invalid;bool OutputCleared=false;
        std::thread Other([&]{auto Out=First;Wrong=Ingress->Receive(Out);OutputCleared=!Out;});Other.join();
        if(Wrong!=ELocalSession::WrongOwner||!OutputCleared)return false;
        bool ClosedAcquire=false,OpenFacadeDestroyed=false,RaceRequested=false;
        int RaceStatus=-1;
        if(O.Launcher.SelfPeer==5)
        {
            // Coordinate actual calls on two live threads. The observed result
            // is recorded; no particular overlap/order or Ready is invented.
            std::promise<void> Start;auto Go=Start.get_future();
            std::thread Closer([&]{Go.wait();Ingress->Close();});
            Start.set_value();auto Out=First;const auto Status=Ingress->Receive(Out);Closer.join();
            RaceRequested=true;RaceStatus=int(Status);
            switch(Status)
            {
                case ELocalSession::Ready:if(!Out)return false;break;
                case ELocalSession::WouldBlock:case ELocalSession::Closed:case ELocalSession::Pending:
                case ELocalSession::Duplicate:case ELocalSession::Stale:case ELocalSession::UnknownPeer:
                case ELocalSession::PeerUnavailable:case ELocalSession::PeerDead:if(Out)return false;break;
                default:return false;
            }
            ELocalSession StatusAfter=ELocalSession::Invalid;auto Refused=Source->AcquireIngress(StatusAfter);
            ClosedAcquire=!Refused&&StatusAfter==ELocalSession::Closed;if(!ClosedAcquire)return false;
            Source.reset();
        }
        else
        {
            // No explicit Close precedes destruction of this still-bound facade.
            Source.reset();OpenFacadeDestroyed=true;
        }
        auto Out=First;
        if(Ingress->Receive(Out)!=ELocalSession::Closed||Out)return false;
        Ingress->Close();Ingress->Close();Out=First;
        if(Ingress->Receive(Out)!=ELocalSession::Closed||Out||First->Read().Bytes!=RetainedBytes)return false;
        const auto Path=std::filesystem::path(O.ReportPath+".opaque-ingress.json");
        auto Temp=Path;Temp+=".tmp";
        if(std::filesystem::exists(Path)||std::filesystem::exists(Temp))return false;
        std::ofstream S(Temp,std::ios::binary|std::ios::trunc);if(!S)return false;
        S<<"{\"scope\":\"actual_original_same_host_ingress_controls_only\",\"actual_pid\":"<<O.Launcher.OriginalSelf.Pid
         <<",\"self_peer\":"<<O.Launcher.SelfPeer<<",\"actual_first_received_peer\":"<<Received.Peer
         <<",\"actual_first_received_sequence\":"<<Received.Sequence<<",\"actual_forward_ready_count\":"<<ReadyCount
         <<",\"wrong_owner_acquire_refused\":true,\"double_consumer_refused\":true,\"facade_receive_after_export_refused\":true"
         <<",\"wrong_owner_receive_cleared_original_output\":true,\"surviving_export_receive_closed\":true,\"retained_original_bytes_unchanged\":true"
         <<",\"closed_acquire_refused\":"<<(ClosedAcquire?"true":"false")<<",\"open_facade_destroyed\":"<<(OpenFacadeDestroyed?"true":"false")
         <<",\"race_start_coordinated\":"<<(RaceRequested?"true":"false")<<",\"observed_race_result\":"<<RaceStatus
         <<",\"specific_race_overlap_or_order_qualified\":false,\"legacy_first_mode_control_qualified\":false"
         <<",\"module_service_global_lifetime\":false,\"restoreK\":false,\"localized\":false}\n";
        S.close();if(!S)return false;std::error_code Error;std::filesystem::rename(Temp,Path,Error);return !Error;
    }
};
}
