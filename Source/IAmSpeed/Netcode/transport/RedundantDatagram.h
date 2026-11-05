#pragma once
#include "NetStateTransport.h"
#include <algorithm>

namespace Speed::Netcode
{
// IAS batching only; each nested original frame/sequence/edge stays unchanged.
// No source authentication, ACK, clock, input consumption or snapshot admission
// is fabricated by this codec. A session validates every decoded source record.
constexpr std::size_t RedundantDatagramBudget=1232,RedundantGroupHeader=12;
enum class ERedundantFamily : std::uint8_t { Input=1,StateFragment=2 };
using FRedundantRecords=std::vector<std::vector<std::uint8_t>>;
struct FRedundantGroup {ERedundantFamily Family;FRedundantRecords Records;};
inline bool ValidRedundantRecord(ERedundantFamily Family,const std::vector<std::uint8_t>& Bytes)
{
    if(Family==ERedundantFamily::Input)return bool(Decode(Bytes.data(),Bytes.size()));
    if(Family==ERedundantFamily::StateFragment)return bool(DecodeFragment(Bytes.data(),Bytes.size()));
    return false;
}
inline std::optional<FRedundantRecords> PackRedundantDatagrams(ERedundantFamily Family,const FRedundantRecords& Records)
{
    if(Records.empty()||Records.size()>256)return {};
    for(const auto& R:Records)if(R.size()+RedundantGroupHeader+2>RedundantDatagramBudget||!ValidRedundantRecord(Family,R))return {};
    FRedundantRecords Out;
    for(std::size_t First=0;First<Records.size();)
    {
        std::size_t Last=First,Size=RedundantGroupHeader;
        while(Last<Records.size()&&Last-First<16&&Size+2+Records[Last].size()<=RedundantDatagramBudget)
        {Size+=2+Records[Last].size();++Last;}
        std::vector<std::uint8_t> Bytes;Bytes.reserve(Size);
        Wire::Put(Bytes,0x31475249,4);Wire::Put(Bytes,1,1);Wire::Put(Bytes,std::uint8_t(Family),1);
        Wire::Put(Bytes,Last-First,2);Wire::Put(Bytes,Size,2);Wire::Put(Bytes,0,2);
        for(std::size_t N=First;N<Last;++N){Wire::Put(Bytes,Records[N].size(),2);Bytes.insert(Bytes.end(),Records[N].begin(),Records[N].end());}
        Out.push_back(std::move(Bytes));First=Last;
    }
    return Out;
}
inline std::optional<FRedundantGroup> DecodeRedundantDatagram(const std::uint8_t* Data,std::size_t Size)
{
    if(!Data||Size<RedundantGroupHeader||Size>RedundantDatagramBudget)return {};
    Wire::FReader R{Data,Size};if(R.Get(4)!=0x31475249||R.Get(1)!=1)return {};
    const auto Family=static_cast<ERedundantFamily>(R.Get(1));const auto Count=R.Get(2);
    if((Family!=ERedundantFamily::Input&&Family!=ERedundantFamily::StateFragment)||!Count||Count>16||R.Get(2)!=Size||R.Get(2))return {};
    FRedundantGroup Out{Family,{}};
    for(std::size_t N=0;N<Count;++N)
    {
        const auto Length=R.Get(2);if(!R.Good||!Length||Length>Size-R.At)return {};
        std::vector<std::uint8_t> Record(Data+R.At,Data+R.At+Length);R.At+=Length;
        if(!ValidRedundantRecord(Family,Record))return {};Out.Records.push_back(std::move(Record));
    }
    return R.Good&&R.At==Size?std::optional<FRedundantGroup>(std::move(Out)):std::nullopt;
}
}
