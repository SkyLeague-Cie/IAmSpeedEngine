#pragma once
#include "RequestedInputOwner.h"

namespace Speed::Input::V2
{
// Inert local locator, never proof of a received agreement. V2 preparation
// resolves these values against the exact private host record on the owner.
struct FRequestedNetworkAgreementLocator
{
    std::uint64_t HostGeneration=0,Slot=0,Connection=0,OfferSequence=0,StreamEpoch=0;
    std::uint64_t GameCodec=0,GameCodecVersion=0;
    std::uint8_t ProfileRoute=0; // 1 client inputs at server, 2 authority at replica.
    std::uint32_t LeaseMilliseconds=0; // Explicit LOCAL finite native lease, not remote time.
    bool IsValid()const noexcept
    {return HostGeneration&&Slot&&Connection&&OfferSequence&&StreamEpoch&&GameCodec&&GameCodecVersion&&(ProfileRoute==1||ProfileRoute==2)&&LeaseMilliseconds&&LeaseMilliseconds<=3600000;}
    bool operator==(const FRequestedNetworkAgreementLocator& R)const noexcept
    {return HostGeneration==R.HostGeneration&&Slot==R.Slot&&Connection==R.Connection&&OfferSequence==R.OfferSequence&&StreamEpoch==R.StreamEpoch&&GameCodec==R.GameCodec&&GameCodecVersion==R.GameCodecVersion&&ProfileRoute==R.ProfileRoute&&LeaseMilliseconds==R.LeaseMilliseconds;}
};
// Inert versioned command values. Canonical network identities never double as
// local component/body locators. None of these values is native admission.
struct FRequestedNetworkSessionDescriptor
{
    std::uint16_t Version=1;
    std::uint64_t Service=0,SourceGeneration=0,WorldGeneration=0;
    // Match identity may be shared by many vehicles. FSessionDescriptor Id /
    // Epoch identify the separate local input binding and its lifecycle.
    std::uint64_t Session=0,Epoch=0;
    std::uint64_t Actor=0,Incarnation=0,Component=0;
    std::uint64_t LocalComponent=0,LocalComponentGeneration=0,LocalBodyGeneration=0;
    std::array<std::uint8_t,32> WireContract{};
    ERequestedDigitalPolicy DigitalPolicy=ERequestedDigitalPolicy::Unspecified;
    std::optional<FRequestedNetworkAgreementLocator> Agreement;
    bool IsValid() const noexcept
    {
        return ((Version==1&&!Agreement)||(Version==2&&Agreement&&Agreement->IsValid()))
            &&Service&&SourceGeneration&&WorldGeneration&&Session&&Epoch&&Actor&&Incarnation&&Component
            &&LocalComponent&&LocalComponentGeneration&&LocalBodyGeneration
            &&DigitalPolicy==ERequestedDigitalPolicy::HeldStateAndCancelInactive
            &&std::any_of(WireContract.begin(),WireContract.end(),[](auto B){return B!=0;});
    }
    bool operator==(const FRequestedNetworkSessionDescriptor& R) const noexcept
    {
        return Version==R.Version&&Service==R.Service&&SourceGeneration==R.SourceGeneration&&WorldGeneration==R.WorldGeneration
            &&Session==R.Session&&Epoch==R.Epoch
            &&Actor==R.Actor&&Incarnation==R.Incarnation&&Component==R.Component&&LocalComponent==R.LocalComponent
            &&LocalComponentGeneration==R.LocalComponentGeneration&&LocalBodyGeneration==R.LocalBodyGeneration
            &&WireContract==R.WireContract&&DigitalPolicy==R.DigitalPolicy&&Agreement==R.Agreement;
    }
    template<class FAdd> void AppendCanonical(FAdd&& Add) const
    {
        Add(Version==2?0x4e455452455132ULL:0x4e455452455131ULL);Add(Version);Add(Service);Add(SourceGeneration);Add(WorldGeneration);
        Add(Session);Add(Epoch);
        Add(Actor);Add(Incarnation);Add(Component);Add(LocalComponent);Add(LocalComponentGeneration);Add(LocalBodyGeneration);
        for(const auto B:WireContract)Add(B);
        Add(std::uint64_t(DigitalPolicy));
        if(Version==2&&Agreement)
        {
            const auto& A=*Agreement;Add(A.HostGeneration);Add(A.Slot);Add(A.Connection);Add(A.OfferSequence);Add(A.StreamEpoch);
            Add(A.GameCodec);Add(A.GameCodecVersion);Add(A.ProfileRoute);Add(A.LeaseMilliseconds);
        }
    }
};
}
