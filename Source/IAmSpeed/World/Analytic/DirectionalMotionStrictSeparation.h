#pragma once
#include "GroupedBoxVolumeBounds.h"
#include "Templates/Function.h"

namespace Speed::Analytic::Certified
{
namespace DirectionalMotionControl
{
IAMSPEED_API bool RuntimeEnabled();
}
enum class EDirectionalProof:uint8 {Clear,Unavailable,Unresolved,Work,Invalid};
struct FDirectionalMotionObservation
{
    uint64 Providers=0,Nodes=0,AabbClears=0,DirectionalClears=0,SupportBuilds=0,SupportHits=0;
    int32 Directions=0;
};
struct FDirectionalSlab {FVector3d Axis;Interval Bounds;};
using FCompleteMotionSupport=TFunctionRef<bool(const FVector3d&,FDistanceBudget&,Interval&)>;
// Short-lived construction cache only. The callback is never retained in a
// certificate. The supplied slabs define a REGION, not an actual-motion claim.
class FDirectionalRegionBuilder final
{
    FIntervalPoint Broad;
    FCompleteMotionSupport Support;
    TArray<FDirectionalSlab,TInlineAllocator<32>> Slabs;
    uint64 Numeric=0;
    FDirectionalMotionObservation* Observed=nullptr;
    friend class FReferenceDirectionalMotionStrictSeparation;
public:
    static constexpr int32 MaximumDirections=32;
    FDirectionalRegionBuilder(const FIntervalPoint& hull,FCompleteMotionSupport support,uint64 numeric,FDirectionalMotionObservation* observed)
        :Broad(hull),Support(support),Numeric(numeric),Observed(observed){}
    const FIntervalPoint& GetBroadHull()const{return Broad;}
    IAMSPEED_API EDirectionalProof GetSlab(const FVector3d&,FDistanceBudget&,Interval&);
};
namespace StaticBodyHierarchy
{
IAMSPEED_API EDirectionalProof TryClearDirectionalProvider(const FIndexedIntervalUnion&,int32,
    FDirectionalRegionBuilder&,FDistanceBudget&,FDirectionalMotionObservation*);
}
class FReferenceDirectionalMotionStrictSeparation;
using FReferenceDirectionalMotionStrictSeparationPtr=TSharedPtr<const FReferenceDirectionalMotionStrictSeparation,ESPMode::ThreadSafe>;
// Separation of BroadAABB INTERSECT every retained directional slab from the
// full immutable source. Every candidate/domain must be discharged. This is
// not FReferenceBoxVolumeStrictSeparation and does not certify its old region.
class FReferenceDirectionalMotionStrictSeparation final
{
    FReferenceBoundsCandidateSetPtr Partition;
    TArray<FDirectionalSlab,TInlineAllocator<32>> Slabs;
    uint64 Numeric=0;
    FReferenceDirectionalMotionStrictSeparation()=default;
public:
    static IAMSPEED_API EDirectionalProof TryProve(FReferenceBoundsCandidateSetPtr,FCompleteMotionSupport,
        FDistanceBudget&,FReferenceDirectionalMotionStrictSeparationPtr&,FDirectionalMotionObservation* observed=nullptr);
    IAMSPEED_API bool Matches(const FIndexedIntervalUnionPtr&,const FIntervalPoint&)const;
    IAMSPEED_API bool ContainsCompleteEndpointGroup(TConstArrayView<FIntervalPoint>)const;
    const FIntervalPoint& GetBroadHullForInspection()const{return Partition->GetQueryHull();}
    int32 GetDirectionCountForInspection()const{return Slabs.Num();}
};
}
