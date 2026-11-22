#include "IAmSpeed/Netcode/contract-offer/ActionContractOfferCodec.h"
#include <iostream>
#include <stdexcept>
#include <string>
namespace C = Speed::Netcode::ContractOffer;
namespace V = Speed::Input::V2;
static unsigned Checks = 0;
#define CHECK(E) do { ++Checks; if (!(E)) throw std::runtime_error(std::string(__func__) + ":" + std::to_string(__LINE__) + " " #E); } while (false)
using FBytes = std::vector<std::uint8_t>;
static V::FInputActionContractDescription Minimal()
{
    V::FInputActionContractDescription D; D.Revision.Value = 1; D.Actions = V::FInputActionContract::BaseActions();
    D.Physical = {{0,V::EPhysicalDestination::Throttle},{1,V::EPhysicalDestination::Brake},{2,V::EPhysicalDestination::Steering}}; return D;
}
static FBytes Golden()
{
    const std::string Hex = "4941433101000000f600000001010000000000000003000849416d5370656564085468726f74746c650100000000010100ff000000000000000000000000000000803f0000803f0000010849416d5370656564054272616b650100000000010100ff000000000000000000000000000000803f0000803f0000020849416d5370656564085374656572696e6701000000000101017f000000000000000000000000000000803f0000803f00000000030000010102022000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000"; FBytes B;
    const auto N = [](char C) { return unsigned(C <= '9' ? C - '0' : C - 'a' + 10); };
    for (std::size_t I = 0; I < Hex.size(); I += 2) B.push_back(std::uint8_t(16*N(Hex[I])+N(Hex[I+1])));
    return B;
}
static void U32(FBytes& B, std::size_t P, std::uint32_t X)
{ for (unsigned I = 0; I < 4; ++I) B[P+I] = std::uint8_t(X >> (8*I)); }
static std::optional<C::FOffer> Decode(const FBytes& B) { return C::Decode(B.data(), B.size()); }
static FBytes RoundTrip(const V::FInputActionContractDescription& D, V::FInputProcessingPolicy P = {},
    V::EExactRemoteAdmission Mode = V::EExactRemoteAdmission::OriginalDeviceExact)
{
    const auto Original = V::FInputActionContract::Create(D); CHECK(Original);
    const auto B = C::Encode(D,P,Mode); CHECK(B && B->size() <= C::MaxEnvelopeBytes);
    const auto O = Decode(*B); CHECK(O && O->Contract && O->Mode == Mode && O->Processing.Step == P.Step);
    CHECK(O->Contract->GetFingerprint() == Original->GetFingerprint());
    CHECK(C::Encode(O->Contract->GetDescription(),O->Processing,O->Mode) == B); return *B;
}
static void FixedVersion1VectorAndDirectMode()
{
    const auto B = RoundTrip(Minimal()); CHECK(B == Golden() && B.size() == 246);
    CHECK(B[0]=='I' && B[1]=='A' && B[2]=='C' && B[3]=='1' && B[4]==1 && B[12]==1);
    const auto Processed = RoundTrip(Minimal(),{},V::EExactRemoteAdmission::OriginalDeviceAuthoritativeProcessing);
    CHECK(B != Processed && Decode(B)->Contract->GetFingerprint() == Decode(Processed)->Contract->GetFingerprint());
    CHECK(Decode(B)->Mode != Decode(Processed)->Mode); // Host must approve mode, not just descriptive fingerprint.
}
static V::FInputActionContractDescription Complete()
{
    auto D = Minimal(); D.Revision.Value = UINT64_MAX;
    auto& A = D.Actions[0]; A.Version = UINT32_MAX; A.Aliases={"ZetaA","Alpha"};
    A.ActivateAbove=.25f; A.DeactivateAtOrBelow=.05f; A.Deadzone=.1f; A.Exponent=2; A.Sensitivity=1.5f;
    A.Accumulation=V::EActionAccumulation::HighestAbsolute;
    A.Modifiers={{V::EScalarModifier::Scale,.75f,0},{V::EScalarModifier::Clamp,-.8f,.9f}};
    V::FActionDefinition Button; Button.Id=3; Button.Owner="Game"; Button.Name="Pause"; Button.Wiring=V::EActionWiring::Wired; D.Actions.push_back(Button);
    V::FActionDefinition Axis; Axis.Id=4; Axis.Owner="Game"; Axis.Name="Camera"; Axis.Type=V::EActionType::Axis1D;
    Axis.Wiring=V::EActionWiring::Wired; Axis.Signed=true; Axis.Quantization=4096; D.Actions.push_back(Axis);
    Axis.Id=5; Axis.Name="Unused"; Axis.Wiring=V::EActionWiring::Unwired; D.Actions.push_back(Axis);
    D.Mapping={{{V::ERawControlKind::PadAxis,4},0,.5f,{{V::EScalarModifier::Exponent,2,0}}},
        {{V::ERawControlKind::PadAxis,4},0,-.25f,{{V::EScalarModifier::Deadzone,.05f,.9f}}},
        {{V::ERawControlKind::KeyboardUsage,4},2,-1,{}},{{V::ERawControlKind::PadButton,0},3,1,{}},
        {{V::ERawControlKind::MouseButton,0},3,1,{}}};
    std::swap(D.Physical[0],D.Physical[2]); return D;
}
static void FullDescriptorPolicyAndContributionOrder()
{
    auto D=Complete(); V::FInputProcessingPolicy P; P.Step[0]=P.Step[1]=P.Step[2]=16; P.Step[4]=7;
    const auto M=V::EExactRemoteAdmission::OriginalDeviceAuthoritativeProcessing; const auto B=RoundTrip(D,P,M);
    const auto O=Decode(B); const auto& R=O->Contract->GetDescription(); CHECK(R.Revision.Value==UINT64_MAX && R.Actions[0].Version==UINT32_MAX);
    CHECK(R.Actions[0].Aliases[0]=="Alpha" && R.Actions[0].Aliases[1]=="ZetaA");
    CHECK(R.Actions[0].Accumulation==V::EActionAccumulation::HighestAbsolute && R.Actions[0].Modifiers.size()==2);
    CHECK(R.Mapping.size()==5 && R.Mapping[0].Scale==.5f && R.Mapping[1].Scale==-.25f && R.Mapping[0].Modifiers[0].Kind==V::EScalarModifier::Exponent);
    CHECK(R.Physical[0].Destination==V::EPhysicalDestination::Throttle);
    auto Noncanonical=B; const FBytes Alpha{5,'A','l','p','h','a'}, Zeta{5,'Z','e','t','a','A'};
    const auto IA=std::search(Noncanonical.begin(),Noncanonical.end(),Alpha.begin(),Alpha.end());
    const auto IZ=std::search(Noncanonical.begin(),Noncanonical.end(),Zeta.begin(),Zeta.end());
    CHECK(IA!=Noncanonical.end() && IZ!=Noncanonical.end()); std::swap_ranges(IA,IA+6,IZ); CHECK(!Decode(Noncanonical));
    auto Reordered=D; std::swap(Reordered.Mapping[0],Reordered.Mapping[1]); CHECK(RoundTrip(Reordered,P,M)!=B);
    Reordered=D; std::swap(Reordered.Actions[0].Modifiers[0],Reordered.Actions[0].Modifiers[1]); CHECK(RoundTrip(Reordered,P,M)!=B);
    CHECK(!C::Encode(D,P,V::EExactRemoteAdmission::OriginalDeviceExact));
    CHECK(!C::Encode(D,P,V::EExactRemoteAdmission::Closed)); CHECK(!C::Encode(D,P,static_cast<V::EExactRemoteAdmission>(255)));
    P.Step[3]=1; CHECK(!C::Encode(D,P,M)); P.Step[3]=0; P.Step[5]=1; CHECK(!C::Encode(D,P,M));
    P.Step[5]=0; P.Step[4]=8193; CHECK(!C::Encode(D,P,M));
}
static void BoundariesAndUntrustedBytes()
{
    const auto G=Golden(); CHECK(!C::Decode(nullptr,0) && !C::Decode(nullptr,G.size()));
    for (std::size_t N=0;N<G.size();++N)
    {
        FBytes B(G.begin(),G.begin()+N); if(N>=12)U32(B,8,std::uint32_t(N)); CHECK(!Decode(B));
    }
    auto B=G; B.push_back(0); CHECK(!Decode(B)); U32(B,8,std::uint32_t(B.size())); CHECK(!Decode(B));
    B.assign(C::MaxEnvelopeBytes+1,0); CHECK(!Decode(B));
    for (const auto& Mutation : std::vector<std::pair<std::size_t,int>>{{0,0},{4,2},{6,1},{12,0},{12,255},{21,33},{22,255},{23,65},{24,'!'},{45,17},{46,2},{47,2},{48,2},{71,2},{72,17},{174,4},{176,3},{181,31},{182,1}})
    { B=G; B[Mutation.first]=std::uint8_t(Mutation.second); CHECK(!Decode(B)); }
    B=G; for(unsigned I=13;I<21;++I)B[I]=0; CHECK(!Decode(B));
    B=G; B[49]=B[50]=0; CHECK(!Decode(B)); B=G; B[49]=0; B[50]=128; CHECK(!Decode(B));
    for(auto Bits:{0x7fc00000u,0x7f800000u,0xff800000u,0x80000000u}){B=G;U32(B,51,Bits);CHECK(!Decode(B));}
    B=G; U32(B,63,0); CHECK(!Decode(B)); B=G; B[172]=1; B[173]=1; CHECK(!Decode(B));
    // Physical enumeration is canonical even though Create accepts permutations.
    B=G; std::swap(B[175],B[177]);std::swap(B[176],B[178]);CHECK(!Decode(B));
    auto D=Minimal();D.Mapping={{{V::ERawControlKind::KeyboardUsage,4},0,1,{}}};B=RoundTrip(D);CHECK(B.size()==255);
    auto Bad=B;Bad[174]=4;CHECK(!Decode(Bad));Bad=B;Bad[175]=0;CHECK(!Decode(Bad));
    D.Mapping[0].Modifiers={{V::EScalarModifier::Scale,1,0}};B=RoundTrip(D);Bad=B;Bad[183]=4;CHECK(!Decode(Bad));
    Bad=B;U32(Bad,184,0x7f800000u);CHECK(!Decode(Bad));Bad=B;U32(Bad,188,0x3f800000u);CHECK(!Decode(Bad));
    D=Minimal();D.Actions[0].Owner=std::string(65,'A');CHECK(!C::Encode(D,{},V::EExactRemoteAdmission::OriginalDeviceExact));
    D=Minimal();D.Actions[0].Aliases.assign(17,"A");CHECK(!C::Encode(D,{},V::EExactRemoteAdmission::OriginalDeviceExact));
    D=Minimal();D.Mapping.resize(257);CHECK(!C::Encode(D,{},V::EExactRemoteAdmission::OriginalDeviceExact));
    // Every accepted single-byte mutation must have one exact canonical encoding.
    for(std::size_t I=0;I<G.size();++I)for(auto X:{0u,1u,127u,255u})
    {B=G;B[I]=std::uint8_t(X);const auto O=Decode(B);if(O)CHECK(C::Encode(O->Contract->GetDescription(),O->Processing,O->Mode)==std::optional<FBytes>(B));else CHECK(true);}
}
static std::string Token(unsigned I,unsigned J,char Prefix)
{auto S=std::string(1,Prefix)+std::to_string(I)+"_"+std::to_string(J);S.append(64-S.size(),'A');return S;}
static void LargestNativeDescriptionIsBoundedAndLossless()
{
    auto D=Minimal();D.Revision.Value=UINT64_MAX;
    for(unsigned I=3;I<32;++I){V::FActionDefinition A;A.Id=std::uint8_t(I);A.Owner=Token(I,0,'O');A.Name=Token(I,0,'N');A.Type=V::EActionType::Axis1D;A.Wiring=V::EActionWiring::Wired;A.Quantization=1024;D.Actions.push_back(A);}
    for(unsigned I=0;I<32;++I){for(unsigned J=0;J<16;++J)D.Actions[I].Aliases.push_back(Token(I,J,'L'));D.Actions[I].Modifiers.assign(16,{V::EScalarModifier::Scale,1,0});}
    for(unsigned I=0;I<256;++I)D.Mapping.push_back({{V::ERawControlKind::PadAxis,4},std::uint8_t(I%32),1,std::vector<V::FScalarModifier>(16,{V::EScalarModifier::Scale,1,0})});
    const auto B=RoundTrip(D);CHECK(B.size()>65536 && B.size()<=C::MaxEnvelopeBytes);
}
int main()
{
    try{FixedVersion1VectorAndDirectMode();FullDescriptorPolicyAndContributionOrder();BoundariesAndUntrustedBytes();LargestNativeDescriptionIsBoundedAndLossless();
        std::cout<<"PASS checks="<<Checks<<" scope=portable_version1_control_descriptor_only_no_binding_no_UE_no_network_no_hardware\n";return 0;}
    catch(const std::exception& E){std::cerr<<"FAIL "<<E.what()<<'\n';return 1;}
}
