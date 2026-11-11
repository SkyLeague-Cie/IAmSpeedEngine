#pragma once
#include "IndexedIntervalUnion.h"
#include "ScalarSphereContactBounds.h"
#include "IAmSpeed/Base/SUtils.h"

namespace Speed::Analytic::Prepared { class FReferenceSphereGeneralFirstArrivalRs; }
namespace Speed::Analytic::Certified
{
// Explicit caller budgets/accuracy for mathematical evaluation. Zero defaults
// refuse. These are not gameplay tolerances or response acceptance margins.
struct FReferenceSphereOptions
{
    double TimeGap=0,SpatialGap=0,PointWidth=0,NormalWidth=0;
    int32 SpatialNodesPerAttempt=0,PrefixRangeAttempts=0;
};
enum class EReferenceSphereStatus : uint8 {Unknown,InitialInsideTube,ReferenceNoContact,ReferenceContactBounded};
enum class EReferenceSphereEvaluationStage:uint8 {Inputs,Model,Candidates,Policy,Contact,TimeFloor,RoundedCenter,Scalar,Prefix,Complete};
struct FReferenceSphereEvaluation
{
    EReferenceSphereEvaluationStage Stage=EReferenceSphereEvaluationStage::Inputs;
    EReferenceSphereStatus Status=EReferenceSphereStatus::Unknown;
    FReferenceCandidateSetPtr Candidates;
    FTensorSphereContactEnclosure Contact;
    FScalarSphereContact Scalar;
    float ConsumedSeconds=0;
    bool ConsumedPrefixClear=false;
};
namespace ReferenceSphereDetail
{
inline bool OptionsValid(const FReferenceSphereOptions& o)
{
    return TimeDetail::NonnegativeFinite(o.TimeGap)&&TimeDetail::NonnegativeFinite(o.SpatialGap)&&
        TimeDetail::NonnegativeFinite(o.PointWidth)&&TimeDetail::NonnegativeFinite(o.NormalWidth)&&
        o.SpatialNodesPerAttempt>0&&o.PrefixRangeAttempts>0;
}
inline bool Contains(const FIntervalPoint& box,const FVector3d& point)
{
    if(!Detail::ValidPoint(box)||!ScalarContactDetail::Finite(point))return false;
    for(int32 d=0;d<3;++d)if(point[d]<box.Axis[d].Lower||point[d]>box.Axis[d].Upper)return false;
    return true;
}
// Universal clearance of the COMPLETE union for every rounded consumed time.
// A failed hull may be subdivided, with overlapping inclusive boundaries.
// Success is published only after every contiguous range is proved. Neither
// endpoint clearance nor a reference time floor substitutes for this proof.
inline bool PrefixClear(const FIndexedIntervalUnionPtr& scene,const FZeroOffsetKinematicCenterReference& model,
    float end,double radius,const FReferenceSphereOptions& options,FDistanceBudget& budget)
{
    struct FRange {float First,Last;};TArray<FRange> pending;pending.Add({0.f,end});int32 attempts=0;
    while(!pending.IsEmpty())
    {
        if(attempts>=options.PrefixRangeAttempts||!budget.Consume())return false;
        ++attempts;const auto range=pending.Pop(EAllowShrinking::No);
        if(ProveConsumedCenterFloatSecondsRangeClear(scene,model,range.First,range.Last,radius,budget,
            options.SpatialGap,options.SpatialNodesPerAttempt))continue;
        if(budget.Remaining<=0)return false;
        const uint32 a=KinematicCenterDetail::Bits(range.First),b=KinematicCenterDetail::Bits(range.Last);
        if(b<=a+1)return false;
        uint32 mid=a+(b-a)/2;
        if(mid<KinematicCenterDetail::FloatMinNormal)mid=KinematicCenterDetail::FloatMinNormal;
        if(mid<=a||mid>=b)return false;
        const float seconds=KinematicCenterDetail::FromBits(mid);
        pending.Add({seconds,range.Last});pending.Add({range.First,seconds});
    }
    return true;
}
}
// Values-only reference computation, NOT an authentic native request, World
// hit, advance permission, owner-selection theorem, MTD or response. Both the
// continuous quadratic reference and the consumed SBox DAG use these same
// captured P/V/A bits. Every candidate passes the caller's conservative policy
// BEFORE Contact; no post-contact leaf deletion can change the first contact.
// A private fixed-source producer returns false only before admission.
// An admitted failure returns true with Unknown and cannot restart generic.
enum class EReferenceRoundedPrefix : uint8 { NotApplicable, Clear, Refused };
class FReferenceContactProducer final
{
    friend class ::Speed::Analytic::Prepared::FReferenceSphereGeneralFirstArrivalRs;
    using FCallback = TFunctionRef<bool(const FReferenceCandidateSetPtr&,FDistanceBudget&,FTensorSphereContactEnclosure&)>;
    using FPrefixCallback = TFunctionRef<EReferenceRoundedPrefix(float,FDistanceBudget&)>;
    explicit FReferenceContactProducer(FCallback callback,const FPrefixCallback* prefix=nullptr)
        : Callback(callback),Prefix(prefix) {}
    const FCallback Callback;
    const FPrefixCallback* Prefix=nullptr;
public:
    FReferenceContactProducer(const FReferenceContactProducer&) = delete;
    FReferenceContactProducer& operator=(const FReferenceContactProducer&) = delete;
    bool operator()(const FReferenceCandidateSetPtr& candidates,FDistanceBudget& budget,FTensorSphereContactEnclosure& out) const
    { return Callback(candidates,budget,out); }
    EReferenceRoundedPrefix RoundedPrefix(float end,FDistanceBudget& budget) const
    { return Prefix?(*Prefix)(end,budget):EReferenceRoundedPrefix::NotApplicable; }
};
inline FReferenceSphereEvaluation EvaluateReferenceSphere(
    FIndexedIntervalUnionPtr scene,FVector3d position,FVector3d velocity,FVector3d acceleration,
    float horizon,double collisionRadius,double physicalRadius,const FReferenceSphereOptions& options,
    FDistanceBudget& budget,TFunctionRef<bool(int32)> acceptsCandidate,const FReferenceContactProducer* producer=nullptr)
{
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    FReferenceSphereEvaluation value;
    if(!scene||!ReferenceSphereDetail::OptionsValid(options)||!CandidateDetail::PositiveRadius(collisionRadius)||
        !CandidateDetail::PositiveRadius(physicalRadius)||collisionRadius<physicalRadius||!budget.Consume())return value;
    value.Stage=EReferenceSphereEvaluationStage::Model;
    FZeroOffsetKinematicCenterReference model;
    if(!BuildZeroOffsetKinematicCenterReference(position,velocity,acceleration,horizon,FVector3d::ZeroVector,FVector3d::ZeroVector,model))return value;
    value.Stage=EReferenceSphereEvaluationStage::Candidates;
    if(!FReferenceCandidateSet::Select(scene,model.GetReferenceCurve(),collisionRadius,budget,value.Candidates))return value;
    value.Stage=EReferenceSphereEvaluationStage::Policy;
    for(int32 index:value.Candidates->GetCandidateIndices())if(!budget.Consume()||!acceptsCandidate(index))return value;
    value.Stage=EReferenceSphereEvaluationStage::Contact;
    if(!producer||!(*producer)(value.Candidates,budget,value.Contact))
        value.Contact=value.Candidates->Contact(budget,options.TimeGap,options.SpatialGap,options.SpatialNodesPerAttempt,options.PointWidth,options.NormalWidth);
    const auto provePrefix=[&](float end)
    {
        const auto status=producer?producer->RoundedPrefix(end,budget):EReferenceRoundedPrefix::NotApplicable;
        return status==EReferenceRoundedPrefix::NotApplicable?
            ReferenceSphereDetail::PrefixClear(scene,model,end,collisionRadius,options,budget):
            status==EReferenceRoundedPrefix::Clear;
    };
    if(value.Contact.Status==ETensorSphereContactStatus::InitialInsideTube)
    {
        value.Stage=EReferenceSphereEvaluationStage::Complete;value.Status=EReferenceSphereStatus::InitialInsideTube;
        // Collision-radius tube presence is not physical-radius penetration.
        // No fabricated time-zero contact, release or translation is emitted.
        return value;
    }
    if(value.Contact.Status==ETensorSphereContactStatus::NoContactWithProvidedTensors)
    {
        value.Stage=EReferenceSphereEvaluationStage::Prefix;
        if(!provePrefix(horizon))return value;
        value.Status=EReferenceSphereStatus::ReferenceNoContact;value.ConsumedSeconds=horizon;value.ConsumedPrefixClear=true;
    }
    else if(value.Contact.Status==ETensorSphereContactStatus::FirstContactEnclosed)
    {
        if(value.Contact.CollisionRadiusBits!=AnalyticCertified::Bits(collisionRadius)||value.Contact.Leaves.IsEmpty())return value;
        value.Stage=EReferenceSphereEvaluationStage::TimeFloor;
        const auto floor=FloorReferenceTimeToFloatSeconds(model,value.Contact.LowerTime);if(!floor.Known)return value;
        value.Stage=EReferenceSphereEvaluationStage::RoundedCenter;
        const auto center=KinematicCenterDetail::Bits(floor.Seconds)?Speed::SBox::AdvancePosition(position,velocity,acceleration,floor.Seconds):position;
        if(!ReferenceSphereDetail::Contains(BoundConsumedCenterAtFloatSeconds(model,floor.Seconds),center))return value;
        const auto& leaf=value.Contact.Leaves[0];if(!Detail::ValidPoint(leaf.FixedParameterPoint))return value;
        // A deterministic representative of the certified evaluation box at
        // this exact dyadic UV, not the exact bits of the runtime evaluator.
        FVector3d point;for(int32 d=0;d<3;++d)point[d]=leaf.FixedParameterPoint.Axis[d].Lower;
        value.Stage=EReferenceSphereEvaluationStage::Scalar;
        if(ConvertSphereContact(value.Contact,0,point,center,collisionRadius,physicalRadius,value.Scalar)!=EScalarSphereContactStatus::Bounded)return value;
        value.Stage=EReferenceSphereEvaluationStage::Prefix;
        if(!provePrefix(floor.Seconds))return value;
        value.Status=EReferenceSphereStatus::ReferenceContactBounded;value.ConsumedSeconds=floor.Seconds;value.ConsumedPrefixClear=true;
    }
    else return value;
    value.Stage=EReferenceSphereEvaluationStage::Complete;
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);return value;
}
// Stable values-only DLL boundary. Actual consumed SBox arithmetic remains
// inside IAS; callers do not reimplement it or import its unexported methods.
// No authentic native request, World, authority or response is granted.
IAMSPEED_API FReferenceSphereEvaluation EvaluateReferenceSphereValues(
    FIndexedIntervalUnionPtr scene,FVector3d position,FVector3d velocity,FVector3d acceleration,
    float horizon,double collisionRadius,double physicalRadius,const FReferenceSphereOptions& options,
    FDistanceBudget& budget,TFunctionRef<bool(int32)> acceptsCandidate);

}
