#pragma once
#include "../transport/NetInputTransport.h"
#include <algorithm>

namespace Speed::Netcode::BoundaryMetadata
{
inline constexpr bool NativeWorldQualified=false;
inline constexpr std::size_t MaxBytes=262144,MaxSessions=64,MaxActorsPerView=128,MaxFingerprint=65536,HeaderSize=56;
enum class EOrigin : std::uint8_t { Device=1,ExactScenario=2,AI=4 };
enum class EPhase : std::uint8_t { Active=0,Paused=1,WaitingBaseline=2,CancelPending=3,Retired=4,NotCaptured=255 };
// These are INPUT registry keys, never automatically native world actor IDs or
// actor incarnations. The admitted native mapping remains a separate dependency.
struct FInputActor
{
    U64 Id=0,InputGeneration=0;
    bool operator==(const FInputActor& R)const noexcept{return Id==R.Id && InputGeneration==R.InputGeneration;}
};
struct FBinding
{
    U64 Session=0,Epoch=0,Controller=0,Producer=0,Journal=0,First=0;
    EOrigin Origin=EOrigin::Device;
    EPhase Phase=EPhase::NotCaptured;
    bool AllowScenarioAppend=false;
    std::uint32_t ResumeRearmMask=0;
    std::array<std::uint16_t,ActionCount> Step{};
    std::vector<std::uint8_t> ContractFingerprint; // opaque exact origin bytes, not an imported action contract
    std::vector<FInputActor> Actors;
    bool SameBinding(const FBinding& R)const
    {
        return Session==R.Session && Epoch==R.Epoch && Controller==R.Controller && Producer==R.Producer
            && Journal==R.Journal && First==R.First && Origin==R.Origin && AllowScenarioAppend==R.AllowScenarioAppend
            && ResumeRearmMask==R.ResumeRearmMask && Step==R.Step && ContractFingerprint==R.ContractFingerprint && Actors==R.Actors;
    }
};
struct FClosedIdentity
{
    U64 Session=0,Epoch=0,SourceSequence=0,CutoffSequence=0;
    std::vector<std::uint8_t> OwnerBindingFingerprint;
};
struct FMetadata
{
    U64 CompletedInputFrame=0,WorkerGeneration=0,ConsumedRegistryVersion=0,CurrentRegistryVersion=0;
    std::vector<FBinding> Consumed,Current;
    std::vector<FClosedIdentity> Closed;
};
inline bool ValidView(const std::vector<FBinding>& View,bool Current)
{
    if(View.size()>MaxSessions)return false;
    std::size_t Actors=0;
    for(std::size_t I=0;I<View.size();++I)
    {
        const auto& B=View[I];
        if(!B.Session || !B.Epoch || !B.Controller || !B.Producer || (I && View[I-1].Session>=B.Session)
            || (B.Origin!=EOrigin::Device && B.Origin!=EOrigin::ExactScenario && B.Origin!=EOrigin::AI)
            || (Current ? B.Phase>EPhase::Retired : B.Phase!=EPhase::NotCaptured)
            || B.ContractFingerprint.empty() || B.ContractFingerprint.size()>MaxFingerprint || B.Actors.empty()
            || B.Actors.size()>MaxActorsPerView || (B.AllowScenarioAppend && B.Origin!=EOrigin::ExactScenario)
            || (B.ResumeRearmMask && B.Origin!=EOrigin::Device)
            || (B.Origin!=EOrigin::Device && std::any_of(B.Step.begin(),B.Step.end(),[](auto S){return S!=0;})))return false;
        Actors+=B.Actors.size();if(Actors>MaxActorsPerView)return false;
        for(std::size_t A=0;A<B.Actors.size();++A)
        {
            if(!B.Actors[A].Id || !B.Actors[A].InputGeneration || (A && B.Actors[A-1].Id>=B.Actors[A].Id))return false;
            for(std::size_t J=0;J<I;++J)for(const auto& Old:View[J].Actors)if(Old.Id==B.Actors[A].Id)return false;
        }
        for(std::size_t J=0;J<I;++J)if(View[J].Controller==B.Controller || View[J].Producer==B.Producer)return false;
    }
    return true;
}
inline bool Valid(const FMetadata& M)
{
    // Only a genuine nonempty completed input cohort can produce this block.
    // Empty WORLD frames need another actual native-owner receipt; this format
    // does not invent one from an old nonempty input history observation.
    if(!M.WorkerGeneration || !M.ConsumedRegistryVersion || M.CurrentRegistryVersion<M.ConsumedRegistryVersion
        || M.CompletedInputFrame==std::numeric_limits<U64>::max() || M.Consumed.empty()
        || !ValidView(M.Consumed,false) || !ValidView(M.Current,true) || M.Closed.size()!=M.Consumed.size())return false;
    for(std::size_t I=0;I<M.Closed.size();++I)
    {
        const auto& C=M.Closed[I];const auto& B=M.Consumed[I];
        if(C.Session!=B.Session || C.Epoch!=B.Epoch || !C.SourceSequence || C.CutoffSequence!=C.SourceSequence
            || C.OwnerBindingFingerprint.empty() || C.OwnerBindingFingerprint.size()>MaxFingerprint)return false;
    }
    if(M.CurrentRegistryVersion==M.ConsumedRegistryVersion)
    {
        if(M.Current.size()!=M.Consumed.size())return false;
        for(std::size_t I=0;I<M.Current.size();++I)if(!M.Consumed[I].SameBinding(M.Current[I]))return false;
    }
    return true;
}
inline std::optional<std::vector<std::uint8_t>> Encode(const FMetadata& M)
{
    if(!Valid(M))return {};
    std::size_t Size=HeaderSize;
    const auto Add=[&](std::size_t N){if(N>MaxBytes-Size)return false;Size+=N;return true;};
    for(const auto* View:{&M.Consumed,&M.Current})for(const auto& B:*View)
        if(!Add(128+B.ContractFingerprint.size()+16*B.Actors.size()))return {};
    for(const auto& C:M.Closed)if(!Add(40+C.OwnerBindingFingerprint.size()))return {};
    std::vector<std::uint8_t> Bytes;Bytes.reserve(Size);
    Wire::Put(Bytes,0x31425249,4);Wire::Put(Bytes,1,2);Wire::Put(Bytes,0,2); // IRB1: input metadata, no world-state version implied.
    Wire::Put(Bytes,Size,4);Wire::Put(Bytes,0,4);
    for(auto V:{M.CompletedInputFrame,M.WorkerGeneration,M.ConsumedRegistryVersion,M.CurrentRegistryVersion})Wire::Put(Bytes,V,8);
    Wire::Put(Bytes,M.Consumed.size(),2);Wire::Put(Bytes,M.Current.size(),2);Wire::Put(Bytes,M.Closed.size(),2);Wire::Put(Bytes,0,2);
    for(const auto* View:{&M.Consumed,&M.Current})for(const auto& B:*View)
    {
        for(auto V:{B.Session,B.Epoch,B.Controller,B.Producer,B.Journal,B.First})Wire::Put(Bytes,V,8);
        Wire::Put(Bytes,std::uint8_t(B.Origin),1);Wire::Put(Bytes,B.AllowScenarioAppend?1:0,1);Wire::Put(Bytes,B.Actors.size(),2);
        Wire::Put(Bytes,B.ResumeRearmMask,4);Wire::Put(Bytes,B.ContractFingerprint.size(),4);
        Wire::Put(Bytes,std::uint8_t(B.Phase),1);Wire::Put(Bytes,0,3);
        for(auto V:B.Step)Wire::Put(Bytes,V,2);
        Bytes.insert(Bytes.end(),B.ContractFingerprint.begin(),B.ContractFingerprint.end());
        for(const auto& A:B.Actors){Wire::Put(Bytes,A.Id,8);Wire::Put(Bytes,A.InputGeneration,8);}
    }
    for(const auto& C:M.Closed)
    {
        for(auto V:{C.Session,C.Epoch,C.SourceSequence,C.CutoffSequence})Wire::Put(Bytes,V,8);
        Wire::Put(Bytes,C.OwnerBindingFingerprint.size(),4);Wire::Put(Bytes,0,4);
        Bytes.insert(Bytes.end(),C.OwnerBindingFingerprint.begin(),C.OwnerBindingFingerprint.end());
    }
    if(Bytes.size()!=Size)return {};
    return Bytes;
}
inline std::optional<FMetadata> Decode(const std::vector<std::uint8_t>& Bytes)
{
    if(Bytes.size()<HeaderSize || Bytes.size()>MaxBytes)return {};
    Wire::FReader R{Bytes.data(),Bytes.size()};
    if(R.Get(4)!=0x31425249 || R.Get(2)!=1 || R.Get(2) || R.Get(4)!=Bytes.size() || R.Get(4))return {};
    FMetadata M;M.CompletedInputFrame=R.Get(8);M.WorkerGeneration=R.Get(8);M.ConsumedRegistryVersion=R.Get(8);M.CurrentRegistryVersion=R.Get(8);
    const auto Consumed=R.Get(2),Current=R.Get(2),Closed=R.Get(2);
    if(R.Get(2) || !R.Good || !Consumed || Consumed>MaxSessions || Current>MaxSessions || Closed!=Consumed)return {};
    const auto ReadFingerprint=[&](std::vector<std::uint8_t>& F,U64 Count)
    {
        if(!R.Good || !Count || Count>MaxFingerprint || Count>Bytes.size()-R.At)return false;
        F.assign(Bytes.begin()+R.At,Bytes.begin()+R.At+std::size_t(Count));R.At+=std::size_t(Count);return true;
    };
    for(int Which=0;Which<2;++Which)
    {
        auto& View=Which?M.Current:M.Consumed;const auto Count=Which?Current:Consumed;View.reserve(std::size_t(Count));
        std::size_t ActorTotal=0;
        for(U64 I=0;I<Count;++I)
        {
            if(R.At>Bytes.size() || Bytes.size()-R.At<128)return {};
            FBinding B;B.Session=R.Get(8);B.Epoch=R.Get(8);B.Controller=R.Get(8);B.Producer=R.Get(8);B.Journal=R.Get(8);B.First=R.Get(8);
            B.Origin=EOrigin(R.Get(1));const auto Append=R.Get(1),Actors=R.Get(2);
            B.ResumeRearmMask=std::uint32_t(R.Get(4));const auto Fingerprint=R.Get(4);B.Phase=EPhase(R.Get(1));
            if(R.Get(3) || Append>1 || !Actors || Actors>MaxActorsPerView)return {};B.AllowScenarioAppend=Append!=0;
            ActorTotal+=std::size_t(Actors);if(ActorTotal>MaxActorsPerView)return {};
            for(auto& V:B.Step)V=std::uint16_t(R.Get(2));
            if(!ReadFingerprint(B.ContractFingerprint,Fingerprint) || 16*Actors>Bytes.size()-R.At)return {};
            B.Actors.reserve(std::size_t(Actors));for(U64 A=0;A<Actors;++A)B.Actors.push_back({R.Get(8),R.Get(8)});
            View.push_back(std::move(B));
        }
    }
    M.Closed.reserve(std::size_t(Closed));
    for(U64 I=0;I<Closed;++I)
    {
        if(R.At>Bytes.size() || Bytes.size()-R.At<40)return {};
        FClosedIdentity C;C.Session=R.Get(8);C.Epoch=R.Get(8);C.SourceSequence=R.Get(8);C.CutoffSequence=R.Get(8);
        const auto Fingerprint=R.Get(4);if(R.Get(4) || !ReadFingerprint(C.OwnerBindingFingerprint,Fingerprint))return {};
        M.Closed.push_back(std::move(C));
    }
    if(!R.Good || R.At!=Bytes.size() || !Valid(M))return {};
    return M; // Untrusted metadata only. No source import, owner admission or native/world proof.
}
}
