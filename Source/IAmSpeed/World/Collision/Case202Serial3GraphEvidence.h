#pragma once
#include "Case202Serial3RoundedModel.h"
#include <cstddef>

namespace Case202Serial3
{
struct ChartIdentity
{
    uint64_t Source=0,Surface=0,Feature=0,Primitive=0;
    const void* ImmutableProduct=nullptr;
    bool Matches(const ChartIdentity& other) const
    {
        return Source&&Surface&&Feature&&Primitive&&ImmutableProduct&&
            Source==other.Source&&Surface==other.Surface&&Feature==other.Feature&&
            Primitive==other.Primitive&&ImmutableProduct==other.ImmutableProduct;
    }
};

struct NativeModelIdentity
{
    std::array<uint8_t,32> Image{},CRT{},RoundedDAG{};
    const void* IASModule=nullptr;
    const void* CRTModule=nullptr;
    const void* Integrate=nullptr;
    const void* IATSlot=nullptr;
    const void* ResolvedCos=nullptr;
    uint64_t NumericControl=0;
    double CosineReturnOneBelow=0.;
    const void* FmaIATSlot=nullptr;
    const void* ResolvedFma=nullptr;
    const void* FmaHardwareFlag=nullptr;
    std::array<const void*,2> ClassifyIATSlots{};
    bool Matches(const NativeModelIdentity& live) const
    {
        const auto nonempty=[](const auto& bytes)
        {return std::any_of(bytes.begin(),bytes.end(),[](uint8_t b){return b!=0;});};
        return IASModule&&CRTModule&&Integrate&&IATSlot&&ResolvedCos&&NumericControl&&nonempty(Image)&&nonempty(CRT)&&nonempty(RoundedDAG)&&
            IASModule==live.IASModule&&CRTModule==live.CRTModule&&Integrate==live.Integrate&&
            IATSlot==live.IATSlot&&ResolvedCos==live.ResolvedCos&&
            FmaIATSlot==live.FmaIATSlot&&ResolvedFma==live.ResolvedFma&&FmaHardwareFlag==live.FmaHardwareFlag&&
            ClassifyIATSlots==live.ClassifyIATSlots&&
            NumericControl==live.NumericControl&&Image==live.Image&&CRT==live.CRT&&RoundedDAG==live.RoundedDAG&&
            std::isfinite(CosineReturnOneBelow)&&CosineReturnOneBelow>0.&&CosineReturnOneBelow==live.CosineReturnOneBelow;
    }
};

// Deliberately no public constructor or factory taking caller-supplied booleans.
// The reviewed loaded-image verifier has to be implemented and authenticated in
// the real host before the candidate producer can mint a runtime token.
class ReviewedLoadedImageVerifier;
class LoadedModelAttestation final
{
    friend class ReviewedLoadedImageVerifier;
    using FStillLoaded=bool(*)(const NativeModelIdentity&);
    NativeModelIdentity Captured;
    FStillLoaded StillLoaded=nullptr;
    LoadedModelAttestation(const NativeModelIdentity& captured,FStillLoaded stillLoaded)
        :Captured(captured),StillLoaded(stillLoaded){}
public:
    bool Matches(const NativeModelIdentity& live) const
    {return StillLoaded&&Captured.Matches(live)&&StillLoaded(Captured);}
};

struct RawInitialInside
{
    ChartIdentity Chart;
    int Corner=-1,Opposite=-1;
    Range InsideGap,OppositeGap;
};
struct NormalizedAdjacent
{
    ChartIdentity Chart;
    uint32_t Clear=0,Inside=0;
    int Corner=-1;
    Range GapDerivative,LastClearGap,FirstInsideGap;
    std::array<Range,8> OtherGaps;
};
struct NativeClearCell
{
    uint32_t First=0,Last=0;
    std::array<Range,8> Gaps;
};
struct NativeAdjacent
{
    ChartIdentity Chart;
    uint32_t Clear=0,Inside=0;
    int Corner=-1;
    std::array<NativeClearCell,32> Cover;
    std::size_t Cells=0;
    std::array<Range,8> FirstInsideGaps;
    // Actual native graph proof producer also verifies each corner's finite
    // chart or left-of-chart alternative, and the two real endpoint readbacks.
};

inline bool StrictPositive(Range a) {return a.Valid()&&a.Lo>0.;}
inline bool StrictNegative(Range a) {return a.Valid()&&a.Hi<0.;}
inline bool Adjacent(uint32_t lo,uint32_t hi) {return lo>0&&hi>lo&&hi-lo==1;}

// Final consistency check of certified inputs. These range records must come
// from paid graph proofs, never from an untrusted file or a supplied verdict.
// This verifier does not grant a hit or replace the actual World filter.
inline bool VerifyGraphFacts(const ChartIdentity& chart,const RawInitialInside& raw,
    const NormalizedAdjacent& normalized,const NativeAdjacent& native,uint32_t originalH)
{
    if(!chart.Matches(raw.Chart)||!chart.Matches(normalized.Chart)||!chart.Matches(native.Chart)||
       raw.Corner<0||raw.Corner>=8||raw.Opposite!=(raw.Corner^4)||
       raw.Corner!=normalized.Corner||raw.Corner!=native.Corner||
       !StrictNegative(raw.InsideGap)||!StrictPositive(raw.OppositeGap)||
       !Adjacent(native.Clear,native.Inside)||!Adjacent(normalized.Clear,normalized.Inside)||
       native.Inside>=normalized.Clear||normalized.Inside>originalH||
       !StrictNegative(normalized.GapDerivative)||!StrictPositive(normalized.LastClearGap)||
       !StrictNegative(normalized.FirstInsideGap)||native.Cells==0||native.Cells>native.Cover.size())return false;
    for(int k=0;k<8;++k)
    {
        if(k==native.Corner)
        {
            if(!StrictNegative(native.FirstInsideGaps[k]))return false;
        }
        else if(!StrictPositive(native.FirstInsideGaps[k])||!StrictPositive(normalized.OtherGaps[k]))return false;
    }
    uint32_t covered=0;
    for(std::size_t i=0;i<native.Cells;++i)
    {
        const auto& cell=native.Cover[i];
        if(cell.First!=covered||cell.Last<=covered||cell.Last>native.Clear||
           !std::all_of(cell.Gaps.begin(),cell.Gaps.end(),StrictPositive))return false;
        covered=cell.Last;
    }
    return covered==native.Clear;
}

inline bool VerifyAttestedGraphFacts(const LoadedModelAttestation* attestation,
    const NativeModelIdentity& live,const ChartIdentity& chart,const RawInitialInside& raw,
    const NormalizedAdjacent& normalized,const NativeAdjacent& native,uint32_t originalH)
{
    return attestation&&attestation->Matches(live)&&VerifyGraphFacts(chart,raw,normalized,native,originalH);
}
}
