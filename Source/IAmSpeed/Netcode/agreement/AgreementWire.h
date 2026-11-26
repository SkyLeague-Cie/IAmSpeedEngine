#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
#include <optional>
#include <vector>

// Standalone transport probes have no UE module macro. A composed UE build
// exports the real implementation for game modules without redefining it.
#if defined(IAMSPEED_API)
#define IAMSPEED_AGREEMENT_API IAMSPEED_API
#else
#define IAMSPEED_AGREEMENT_API
#endif

namespace Speed::Netcode::Agreement
{
using FBytes=std::vector<std::uint8_t>;
enum class EProfileRoute:std::uint8_t { ClientInputsAtServer=1, AuthorityAtReplica=2 };
struct FProfile
{
    EProfileRoute Route=EProfileRoute::ClientInputsAtServer;
    FBytes Bytes;
};
// Public wire values, never connection evidence or a native permission.
// Native/local worker, service, body and socket identities are absent.
struct FOffer
{
    std::uint64_t GameCodec=0,GameCodecVersion=0;
    std::uint64_t Match=0,MatchEpoch=0,Actor=0,Incarnation=0,Component=0;
    std::uint64_t OfferSequence=0,StreamEpoch=0,FrameOrigin=0,FirstFrame=0;
    std::uint64_t LogicalRateNumerator=0,LogicalRateDenominator=0;
    // Explicit wire tags: 1 device, 2 AI. These are not
    // a cast of the implementation's EProducerKind or an original sample.
    std::uint64_t OriginalProducerKind=0,OriginalProducer=0,OriginalStreamEpoch=0,OriginalDeviceGeneration=0;
    std::array<std::uint8_t,32> WireContract{};
    std::vector<FProfile> Profiles;
};
enum class EMessageKind:std::uint8_t { Offer=1,Accept=2,Commit=3,Reject=4 };
struct FMessage { EMessageKind Kind=EMessageKind::Reject;FBytes OfferBytes; };

class FWireCodec final
{
public:
    static constexpr std::uint64_t OfferMarker=0x3146474341534149ull;
    static constexpr std::uint64_t OfferVersion=1;
    static constexpr std::size_t MaxProfiles=2,MaxProfileBytes=65536;
    static constexpr std::size_t FixedOfferBytes=21*8+32;
    static constexpr std::size_t MaxOfferBytes=FixedOfferBytes+MaxProfiles*(16+MaxProfileBytes);
    static constexpr std::size_t FrameHeaderBytes=12,MaxFrameBytes=FrameHeaderBytes+MaxOfferBytes;
    static bool Valid(const FOffer& O) noexcept
    {
        if(!O.GameCodec||!O.GameCodecVersion||!O.Match||!O.MatchEpoch||!O.Actor||!O.Incarnation||!O.Component
            ||!O.OfferSequence||!O.StreamEpoch||O.FrameOrigin>O.FirstFrame||O.FirstFrame==UINT64_MAX
            ||!O.LogicalRateNumerator||!O.LogicalRateDenominator
            ||std::gcd(O.LogicalRateNumerator,O.LogicalRateDenominator)!=1
            ||!O.OriginalProducerKind||O.OriginalProducerKind>2||!O.OriginalProducer||!O.OriginalStreamEpoch||!O.OriginalDeviceGeneration
            ||O.Profiles.empty()||O.Profiles.size()>MaxProfiles)return false;
        bool Nonzero=false;for(auto B:O.WireContract)Nonzero|=B!=0;if(!Nonzero)return false;
        unsigned Previous=0;
        for(const auto& P:O.Profiles)
        {
            const unsigned Route=static_cast<unsigned>(P.Route);
            if(Route<1||Route>2||Route<=Previous||P.Bytes.empty()||P.Bytes.size()>MaxProfileBytes)return false;
            Previous=Route;
        }
        return true;
    }
    static std::optional<FBytes> EncodeOffer(const FOffer& O)
    {
        if(!Valid(O))return {};
        FBytes B;B.reserve(FixedOfferBytes+32+O.Profiles[0].Bytes.size());
        Add64(B,OfferMarker);Add64(B,OfferVersion);Add64(B,0);
        for(auto V:{O.GameCodec,O.GameCodecVersion,O.Match,O.MatchEpoch,O.Actor,O.Incarnation,O.Component,
            O.OfferSequence,O.StreamEpoch,O.FrameOrigin,O.FirstFrame,O.LogicalRateNumerator,O.LogicalRateDenominator,
            O.OriginalProducerKind,O.OriginalProducer,O.OriginalStreamEpoch,O.OriginalDeviceGeneration})Add64(B,V);
        B.insert(B.end(),O.WireContract.begin(),O.WireContract.end());Add64(B,O.Profiles.size());
        for(const auto& P:O.Profiles){Add64(B,static_cast<std::uint8_t>(P.Route));Add64(B,P.Bytes.size());B.insert(B.end(),P.Bytes.begin(),P.Bytes.end());}
        for(unsigned I=0;I<8;++I)B[16+I]=std::uint8_t(std::uint64_t(B.size())>>(I*8));
        return B;
    }
    static std::optional<FOffer> DecodeOffer(const std::uint8_t* Data,std::size_t Size)
    {
        if(!Data||Size<FixedOfferBytes||Size>MaxOfferBytes)return {};
        FReader R{Data,Size};std::uint64_t Marker=0,Version=0,Declared=0;
        if(!R.Get(Marker)||!R.Get(Version)||!R.Get(Declared)||Marker!=OfferMarker||Version!=OfferVersion||Declared!=Size)return {};
        FOffer O;
        for(auto* V:{&O.GameCodec,&O.GameCodecVersion,&O.Match,&O.MatchEpoch,&O.Actor,&O.Incarnation,&O.Component,
            &O.OfferSequence,&O.StreamEpoch,&O.FrameOrigin,&O.FirstFrame,&O.LogicalRateNumerator,&O.LogicalRateDenominator,
            &O.OriginalProducerKind,&O.OriginalProducer,&O.OriginalStreamEpoch,&O.OriginalDeviceGeneration})if(!R.Get(*V))return {};
        if(R.Left()<O.WireContract.size())return {};
        for(auto& V:O.WireContract)V=R.Data[R.At++];
        std::uint64_t Count=0;if(!R.Get(Count)||!Count||Count>MaxProfiles)return {};
        // Check the entire variable-length framing before any profile allocation.
        auto Check=R;
        for(std::uint64_t I=0;I<Count;++I)
        {
            std::uint64_t Route=0,Bytes=0;if(!Check.Get(Route)||!Check.Get(Bytes)||Route<1||Route>2||!Bytes||Bytes>MaxProfileBytes||Bytes>Check.Left())return {};
            Check.At+=std::size_t(Bytes);
        }
        if(Check.Left())return {};
        O.Profiles.reserve(std::size_t(Count));
        for(std::uint64_t I=0;I<Count;++I)
        {
            std::uint64_t Route=0,Bytes=0;R.Get(Route);R.Get(Bytes);
            FProfile P;P.Route=static_cast<EProfileRoute>(Route);P.Bytes.assign(R.Data+R.At,R.Data+R.At+std::size_t(Bytes));R.At+=std::size_t(Bytes);O.Profiles.push_back(std::move(P));
        }
        return Valid(O)?std::optional<FOffer>(std::move(O)):std::nullopt;
    }
    static std::optional<FBytes> EncodeFrame(EMessageKind Kind,const FBytes& Offer)
    {
        if(Kind<EMessageKind::Offer||Kind>EMessageKind::Reject||!DecodeOffer(Offer.data(),Offer.size()))return {};
        FBytes B;B.reserve(FrameHeaderBytes+Offer.size());
        const auto Size=std::uint32_t(Offer.size());for(unsigned I=0;I<4;++I)B.push_back(std::uint8_t(Size>>(I*8)));
        B.insert(B.end(),{'I','A','S','A',1,std::uint8_t(Kind),0,0});B.insert(B.end(),Offer.begin(),Offer.end());return B;
    }
    static std::optional<std::size_t> PayloadSize(const std::uint8_t* H,std::size_t Size) noexcept
    {
        if(!H||Size!=FrameHeaderBytes||H[4]!='I'||H[5]!='A'||H[6]!='S'||H[7]!='A'||H[8]!=1||H[9]<1||H[9]>4||H[10]||H[11])return {};
        std::uint32_t N=0;for(unsigned I=0;I<4;++I)N|=std::uint32_t(H[I])<<(I*8);
        return N>=FixedOfferBytes&&N<=MaxOfferBytes?std::optional<std::size_t>(N):std::nullopt;
    }
    static std::optional<FMessage> DecodeFrame(const std::uint8_t* Data,std::size_t Size)
    {
        if(!Data||Size<FrameHeaderBytes)return {};
        const auto Count=PayloadSize(Data,FrameHeaderBytes);
        if(!Count||Size-FrameHeaderBytes!=*Count||!DecodeOffer(Data+FrameHeaderBytes,*Count))return {};
        return FMessage{static_cast<EMessageKind>(Data[9]),FBytes(Data+FrameHeaderBytes,Data+Size)};
    }
private:
    static void Add64(FBytes& B,std::uint64_t V){for(unsigned I=0;I<8;++I)B.push_back(std::uint8_t(V>>(I*8)));}
    struct FReader
    {
        const std::uint8_t* Data;std::size_t Size,At=0;
        std::size_t Left() const noexcept {return Size-At;}
        bool Get(std::uint64_t& V) noexcept {if(Left()<8)return false;V=0;for(unsigned I=0;I<8;++I)V|=std::uint64_t(Data[At++])<<(I*8);return true;}
    };
};
}
