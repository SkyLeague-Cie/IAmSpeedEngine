#pragma once
#include "NetInputTransport.h"
#include <algorithm>
#include <optional>
namespace Speed::Netcode::Observation
{
// Observable VALUE records only. Distinct wire magic/schema from full-state
// transport; no full-roster flag, checkpoint installation, source authentication
// or native lifecycle/restore capability follows from retaining these bytes.
using FBytes=std::vector<std::uint8_t>;
constexpr std::size_t MaxPayload=1024,MaxFrames=4096,DatagramBudget=1232,HeaderBytes=56;
struct FRecord {U64 Session=0,Epoch=0,NativeLifetime=0,Sequence=0,Frame=0;FBytes Bytes;};
struct FAck {U64 Session=0,Epoch=0,NativeLifetime=0,Sequence=0,Frame=0;std::uint16_t Length=0;std::uint32_t Crc=0;};
inline std::uint32_t Crc(const FBytes& Bytes)
{
    std::uint32_t V=0xffffffffu;for(auto B:Bytes){V^=B;for(int I=0;I<8;++I)V=(V>>1)^(0xedb88320u&(0u-(V&1u)));}return ~V;
}
inline bool Valid(const FRecord& R)
{return R.Session&&R.Epoch&&R.NativeLifetime&&R.Frame<MaxFrames&&R.Sequence==R.Frame+1&&!R.Bytes.empty()&&R.Bytes.size()<=MaxPayload;}
inline std::optional<FBytes> Encode(const FRecord& R)
{
    if(!Valid(R))return {};FBytes B;Wire::Put(B,0x314f5249,4);Wire::Put(B,1,2);Wire::Put(B,0,2);
    for(auto V:{R.Session,R.Epoch,R.NativeLifetime,R.Sequence,R.Frame})Wire::Put(B,V,8);
    Wire::Put(B,R.Bytes.size(),2);Wire::Put(B,0,2);Wire::Put(B,Crc(R.Bytes),4);B.insert(B.end(),R.Bytes.begin(),R.Bytes.end());return B;
}
inline std::optional<FRecord> Decode(const FBytes& B)
{
    if(B.size()<=HeaderBytes||B.size()>HeaderBytes+MaxPayload)return {};Wire::FReader R{B.data(),B.size()};
    if(R.Get(4)!=0x314f5249||R.Get(2)!=1||R.Get(2))return {};FRecord V;
    V.Session=R.Get(8);V.Epoch=R.Get(8);V.NativeLifetime=R.Get(8);V.Sequence=R.Get(8);V.Frame=R.Get(8);
    const auto N=R.Get(2);if(R.Get(2))return {};const auto Check=R.Get(4);
    if(!R.Good||N!=B.size()-HeaderBytes)return {};V.Bytes.assign(B.begin()+HeaderBytes,B.end());
    if(!Valid(V)||Crc(V.Bytes)!=Check)return {};return V;
}
inline FBytes Ack(const FRecord& R)
{
    if(!Valid(R))return {};FBytes B;Wire::Put(B,0x314f4149,4);Wire::Put(B,1,2);Wire::Put(B,0,2);
    for(auto V:{R.Session,R.Epoch,R.NativeLifetime,R.Sequence,R.Frame})Wire::Put(B,V,8);
    Wire::Put(B,R.Bytes.size(),2);Wire::Put(B,0,2);Wire::Put(B,Crc(R.Bytes),4);return B;
}
inline std::optional<FAck> DecodeAck(const FBytes& B)
{
    if(B.size()!=HeaderBytes)return {};Wire::FReader R{B.data(),B.size()};
    if(R.Get(4)!=0x314f4149||R.Get(2)!=1||R.Get(2))return {};FAck V;
    V.Session=R.Get(8);V.Epoch=R.Get(8);V.NativeLifetime=R.Get(8);V.Sequence=R.Get(8);V.Frame=R.Get(8);
    V.Length=std::uint16_t(R.Get(2));if(R.Get(2))return {};V.Crc=std::uint32_t(R.Get(4));
    if(!R.Good||!V.Session||!V.Epoch||!V.NativeLifetime||V.Frame>=MaxFrames||V.Sequence!=V.Frame+1||!V.Length||V.Length>MaxPayload)return {};return V;
}
inline bool Matches(const FAck& A,const FRecord& R)
{return Valid(R)&&A.Session==R.Session&&A.Epoch==R.Epoch&&A.NativeLifetime==R.NativeLifetime&&A.Sequence==R.Sequence&&A.Frame==R.Frame&&A.Length==R.Bytes.size()&&A.Crc==Crc(R.Bytes);}
inline std::optional<std::vector<FBytes>> Pack(const std::vector<FRecord>& Records)
{
    if(Records.empty()||Records.size()>256)return {};std::vector<FBytes> E;for(const auto& R:Records){auto B=Encode(R);if(!B)return {};E.push_back(std::move(*B));}
    std::vector<FBytes> Out;
    for(std::size_t First=0;First<E.size();)
    {
        std::size_t Last=First,N=8;while(Last<E.size()&&Last-First<16&&N+2+E[Last].size()<=DatagramBudget){N+=2+E[Last].size();++Last;}
        if(Last==First)return {};FBytes B;Wire::Put(B,0x314f4749,4);Wire::Put(B,1,1);Wire::Put(B,Last-First,1);Wire::Put(B,N,2);
        for(auto I=First;I<Last;++I){Wire::Put(B,E[I].size(),2);B.insert(B.end(),E[I].begin(),E[I].end());}Out.push_back(std::move(B));First=Last;
    }
    return Out;
}
inline std::optional<std::vector<FRecord>> Unpack(const FBytes& B)
{
    if(B.size()<8||B.size()>DatagramBudget)return {};Wire::FReader R{B.data(),B.size()};
    if(R.Get(4)!=0x314f4749||R.Get(1)!=1)return {};const auto Count=R.Get(1);if(!Count||Count>16||R.Get(2)!=B.size())return {};
    std::vector<FRecord> Out;
    for(U64 I=0;I<Count;++I)
    {
        const auto N=R.Get(2);if(!R.Good||N>B.size()-R.At)return {};FBytes V(B.begin()+R.At,B.begin()+R.At+N);R.At+=N;
        auto Record=Decode(V);if(!Record)return {};Out.push_back(std::move(*Record));
    }
    if(!R.Good||R.At!=B.size())return {};return Out;
}
enum class EKeep {Accepted,Duplicate,Conflict,WrongStream,TooFar,Invalid};
class FJournal final
{
    U64 Session,Epoch,Server,Kernel=0,Contiguous=0;std::vector<std::optional<FRecord>> Records;
public:
    FJournal(U64 S,U64 E,U64 P,std::size_t Budget):Session(S),Epoch(E),Server(P)
    {if(S&&E&&P&&Budget&&Budget<=MaxFrames)Records.resize(Budget);}
    FJournal(const FJournal&)=delete;FJournal& operator=(const FJournal&)=delete;
    EKeep Retain(U64 Peer,const FRecord& R)
    {
        if(Records.empty()||!Valid(R))return EKeep::Invalid;
        if(Peer!=Server||R.Session!=Session||R.Epoch!=Epoch||(Kernel&&Kernel!=R.NativeLifetime))return EKeep::WrongStream;
        if(R.Frame>=Records.size())return EKeep::TooFar;auto& Slot=Records[std::size_t(R.Frame)];
        if(Slot)return Encode(*Slot)==Encode(R)?EKeep::Duplicate:EKeep::Conflict;
        Slot=R;Kernel=R.NativeLifetime;while(Contiguous<Records.size()&&Records[std::size_t(Contiguous)])++Contiguous;return EKeep::Accepted;
    }
    std::optional<FRecord> Read(U64 Frame) const {return Frame<Records.size()?Records[std::size_t(Frame)]:std::optional<FRecord>{};}
    U64 ContiguousCount() const noexcept{return Contiguous;}
    // Select actual frame order, independent of allocation/ACK slot reuse.
    std::optional<std::vector<FRecord>> Pending(const std::vector<bool>& Acked,std::size_t Limit) const
    {
        if(Acked.size()!=Records.size()||!Limit||Limit>16)return {};std::vector<FRecord> Out;
        for(std::size_t N=0;N<Records.size()&&Out.size()<Limit;++N)if(Records[N]&&!Acked[N])Out.push_back(*Records[N]);return Out;
    }
};
}
