#pragma once
#include "ReferenceAffineContact.h"
#include "HAL/PlatformTime.h"
#include "PreparedSphereStrictSeparation.h"
#include "IAmSpeed/World/Analytic/ReferenceSphereConsumer.h"

namespace Speed::Analytic::Prepared::SignedAffineSphereNativeClosure
{
namespace C = Certified;
namespace A = ::AnalyticCertified;
using EDecision = C::SignedAffineSphereResidual::EDecision;
enum class EClose : uint8 { NotApplicable, Clear, Refused };
enum class EPlan : uint8 { NotApplicable, Empty, Ready, Refused };
struct FPrefixObservation
{
    uint64 AdmissionWork=0;
    uint64 SeparationWork=0;
    bool ReusedCover=false;
};

struct FPrefixTiming
{
    enum EStage:uint8 {Produce,Prefix,Admission,WholeBox,Joint,Tail,Predecessor,RangeRetry,Fresh,Count};
    struct FRow {uint64 Calls=0,Ticks=0,Work=0;};
    FRow Rows[Count];
    uint64 EvaluateStart=0,EvaluateEnd=0,ProduceFirst=0,SelectWork=0,EvaluateUsed=0;
    uint32 CoverAttempts=0,ReselectAttempts=0;
    bool Integrity=true;
};
class FScopedPrefixTiming
{
    FPrefixTiming* Timing;
    FPrefixTiming::EStage Stage;
    const C::FDistanceBudget& Budget;
    uint64 Before=0,Used=0;
public:
    FScopedPrefixTiming(FPrefixTiming* timing,FPrefixTiming::EStage stage,const C::FDistanceBudget& budget)
        :Timing(timing),Stage(stage),Budget(budget)
    {
        if(!Timing)return;
        Before=FPlatformTime::Cycles64();Used=Budget.Used;
        if(Stage==FPrefixTiming::Produce&&!Timing->ProduceFirst)
        {
            Timing->ProduceFirst=Before;
            Timing->Integrity&=Used>=Timing->EvaluateUsed;
            if(Used>=Timing->EvaluateUsed)Timing->SelectWork=Used-Timing->EvaluateUsed;
        }
    }
    ~FScopedPrefixTiming()
    {
        if(!Timing)return;
        const auto now=FPlatformTime::Cycles64();
        auto& row=Timing->Rows[Stage];
        ++row.Calls;
        if(now<Before||Budget.Used<Used){Timing->Integrity=false;return;}
        row.Ticks+=now-Before;row.Work+=Budget.Used-Used;
    }
    FScopedPrefixTiming(const FScopedPrefixTiming&)=delete;
    FScopedPrefixTiming& operator=(const FScopedPrefixTiming&)=delete;
};
template<class F>
auto MeasurePrefixStage(FPrefixTiming* timing,FPrefixTiming::EStage stage,const C::FDistanceBudget& budget,F&& run)
{
    FScopedPrefixTiming sample(timing,stage,budget);
    return run();
}

inline bool RoundedZDirection(double velocity,double acceleration,int32& out)
{
    out=0;
    if(!A::Finite(velocity)||!A::Finite(acceleration))return false;
    const auto direction=[](double value)
    {
        const uint64 bits=A::Bits(value);
        if(!(bits&A::Magnitude))return 0;
        return bits&A::Sign?-1:1;
    };
    const int32 v=direction(velocity),a=direction(acceleration);
    if(v&&a&&v!=a)return false;
    out=v?v:a;
    return true;
}

// The caller supplies the original motion owned by the same prepared capture.
// On nonnegative seconds the rounded square is nondecreasing. With same-sign
// or zero Vz/Az, the rounded linear and quadratic terms move in the same
// direction. Their sum with the fixed position remains monotone, including
// either multiply/add contraction. Neither endpoint alone proves X/Y bounds.
inline bool TightenRoundedZ(const FReferenceSphereStrictSeparation& captured,
    float first,float last,C::FIntervalPoint& out)
{
    const auto before=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    const auto& model=captured.GetModel();
    const auto& state=captured.GetCapturedInput().State;
    const auto& position=state.Location;
    const auto& velocity=state.Velocity;
    const auto& acceleration=state.Acceleration;
    int32 direction=0;
    if(!RoundedZDirection(velocity.Z,acceleration.Z,direction))return false;
    auto range=C::BoundConsumedCenterBetweenFloatSeconds(model,first,last);
    if(!C::Detail::ValidPoint(range))return false;
    const auto point=[&](float seconds)->FVector3d
    {
        if(C::KinematicCenterDetail::Bits(seconds))
            return SBox::AdvancePosition(position,velocity,acceleration,seconds);
        return FVector3d(position);
    };
    const FVector3d a=point(first),b=point(last);
    if(!C::ReferenceSphereDetail::Contains(C::BoundConsumedCenterAtFloatSeconds(model,first),a)||
        !C::ReferenceSphereDetail::Contains(C::BoundConsumedCenterAtFloatSeconds(model,last),b)||
        (direction>0&&a.Z>b.Z)||(direction<0&&a.Z<b.Z)||
        (direction==0&&a.Z!=b.Z))return false;
    const auto z=C::Interval{FMath::Min(a.Z,b.Z),FMath::Max(a.Z,b.Z),true};
    if(!A::Valid(z)||z.Lower<range.Axis[2].Lower||z.Upper>range.Axis[2].Upper||
        RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=
        before.ControlFingerprint())return false;
    range.Axis[2]=z;
    out=range;
    return true;
}

inline EPlan BuildCompletePlan(const FPreparedGeometrySelectionPtr& geometry,
    const C::FIntervalPoint& center,double radius,C::FDistanceBudget& budget,FReferenceAffineContactPlanPtr& out,
    const C::FReferenceCandidateSetPtr& cover={})
{
    using E=C::AffineReferenceDetail::EOutcome;
    out.Reset();
    if(!geometry||!geometry->HasValidAliases()||!geometry->GetReference())return EPlan::NotApplicable;
    if(!C::Detail::ValidPoint(center))return EPlan::Refused;
    const auto& source=geometry->GetReference();
    if(source->GetExactAffineProofIdentity().IsEmpty())return EPlan::NotApplicable;
    auto candidates=cover;
    if(candidates)
    {
        const auto& path=candidates->GetPathForFixedSource();
        if(candidates->GetScene()!=geometry->GetSelectedUnion()||path.Degree!=0||path.ControlPoints.Num()!=1||
            !C::CandidateDetail::PositiveRadius(radius)||radius>candidates->GetRadiusForFixedSource())return EPlan::NotApplicable;
        const auto& hull=path.ControlPoints[0];
        if(!C::Detail::ValidPoint(hull))return EPlan::NotApplicable;
        for(int32 axis=0;axis<3;++axis)
            if(center.Axis[axis].Lower<hull.Axis[axis].Lower||center.Axis[axis].Upper>hull.Axis[axis].Upper)
                return EPlan::NotApplicable;
    }
    else
    {
        C::FIntervalBezierCenterPath domain;
        domain.ControlPoints.Add(center);
        if(!C::FReferenceCandidateSet::Select(geometry->GetSelectedUnion(),domain,radius,budget,candidates)||!candidates||
            candidates->GetScene()!=geometry->GetSelectedUnion())return EPlan::Refused;
    }
    if(candidates->GetCandidateIndices().IsEmpty())return EPlan::Empty;
    const auto status=FReferenceAffineContactPlan::Build(source,candidates,budget,out,nullptr,0,false);
    if(status==E::NotApplicable)return EPlan::NotApplicable;
    if(status!=E::Complete||!out)return EPlan::Refused;
    if(!out->AdmitsRoundedSignedSeparation())
    {
        out.Reset();
        return EPlan::NotApplicable;
    }
    return EPlan::Ready;
}

inline EClose TryPrefixWithCover(const FReferenceSphereStrictSeparation& captured,const FCapturedSignedSphereMotion& motion,
    float end,int32 rangeAttempts,C::FDistanceBudget& budget,FReferenceAffineContactPlanPtr& partition,
    const C::FReferenceCandidateSetPtr& cover,FPrefixObservation& observation,bool splitLastFloat,bool recenterVertical,
    FPrefixTiming* timing,bool timeMidpoint)
{
    partition.Reset();
    const auto& input=captured.GetCapturedInput();
    int32 direction=0;
    const bool monotone=RoundedZDirection(input.State.Velocity.Z,input.State.Acceleration.Z,direction);
    if(rangeAttempts<=0||
        C::KinematicCenterDetail::Bits(captured.GetSeconds())!=0)return EClose::NotApplicable;
    const auto whole=C::BoundConsumedCenterBetweenFloatSeconds(captured.GetModel(),0.f,end);
    FReferenceAffineContactPlanPtr plan;
    const uint64 before=budget.Used;
    const auto ready=MeasurePrefixStage(timing,FPrefixTiming::Admission,budget,[&]{
        return BuildCompletePlan(captured.GetGeometry(),whole,input.SupportRadius,budget,plan,cover);
    });
    observation.AdmissionWork+=budget.Used-before;
    if(ready==EPlan::NotApplicable)return EClose::NotApplicable;
    if(ready==EPlan::Empty)return EClose::Clear;
    if(ready!=EPlan::Ready||!plan)return EClose::Refused;
    if(monotone)
    {
        // Keep the cheaper existing whole-box proof for ordinary motion.
        C::FIntervalPoint center;
        if(!TightenRoundedZ(captured,0.f,end,center))return EClose::Refused;
        const auto result=MeasurePrefixStage(timing,FPrefixTiming::WholeBox,budget,[&]{
            return plan->ProveRoundedSignedSeparation(center,input.SupportRadius,96,budget);
        });
        if(result==EDecision::Clear){partition=MoveTemp(plan);return EClose::Clear;}
        if(result!=EDecision::Indeterminate)return EClose::Refused;
    }
    else
    {
        // Still authenticate the actual native endpoint when monotonicity
        // cannot provide the cheap interval proof.
        const auto& p=input.State;
        const auto endpoint=C::KinematicCenterDetail::Bits(end)?
            SBox::AdvancePosition(p.Location,p.Velocity,p.Acceleration,end):FVector3d(p.Location);
        if(!C::ReferenceSphereDetail::Contains(C::BoundConsumedCenterAtFloatSeconds(captured.GetModel(),end),endpoint))
            return EClose::Refused;
    }
    const auto joint=MeasurePrefixStage(timing,FPrefixTiming::Joint,budget,[&]{
        return plan->ProveRoundedCorrelatedSeparation(motion,whole,end,input.SupportRadius,budget,recenterVertical);
    });
    if(joint==EDecision::Clear){partition=MoveTemp(plan);return EClose::Clear;}
    if(joint!=EDecision::Indeterminate&&joint!=EDecision::NotApplicable)return EClose::Refused;
    const uint32 endBits=C::KinematicCenterDetail::Bits(end);
    if(splitLastFloat&&endBits>C::KinematicCenterDetail::FloatMinNormal&&endBits<C::KinematicCenterDetail::FloatInfinity)
    {
        // The contact solver already encloses the continuous first arrival.
        // This is the additional proof over actual binary32 integration times.
        // Exactly one supported time remains after the predecessor prefix.
        const float previous=C::KinematicCenterDetail::FromBits(endBits-1);
        const auto prefix=C::BoundConsumedCenterBetweenFloatSeconds(captured.GetModel(),0.f,previous);
        const auto& state=input.State;
        const auto last=SBox::AdvancePosition(state.Location,state.Velocity,state.Acceleration,end);
        if(!C::Detail::ValidPoint(prefix)||
            !C::ReferenceSphereDetail::Contains(C::BoundConsumedCenterAtFloatSeconds(captured.GetModel(),end),last))
            return EClose::Refused;
        const auto tail=MeasurePrefixStage(timing,FPrefixTiming::Tail,budget,[&]{
            return plan->ProveRoundedSignedSeparation(C::Detail::Point(last),input.SupportRadius,96,budget);
        });
        if(tail==EDecision::Clear)
        {
            const auto earlier=MeasurePrefixStage(timing,FPrefixTiming::Predecessor,budget,[&]{
                return plan->ProveRoundedCorrelatedSeparation(motion,prefix,previous,input.SupportRadius,budget,recenterVertical);
            });
            if(earlier==EDecision::Clear){partition=MoveTemp(plan);return EClose::Clear;}
            if(earlier!=EDecision::Indeterminate&&earlier!=EDecision::NotApplicable)return EClose::Refused;
        }
        else if(tail!=EDecision::Indeterminate&&tail!=EDecision::NotApplicable)return EClose::Refused;
    }
    if(!monotone)return EClose::NotApplicable;
    FScopedPrefixTiming retryTiming(timing,FPrefixTiming::RangeRetry,budget);
    struct FRange {float First,Last;};
    const int32 limit=FMath::Min(rangeAttempts,64);
    const auto run=[&](bool useTimeMidpoint,int32& attempts)
    {
        TArray<FRange> pending{{0.f,end}};
        while(!pending.IsEmpty())
        {
            if(attempts>=limit||!budget.Consume())return false;
            ++attempts;
            const auto range=pending.Pop(EAllowShrinking::No);
            if(attempts!=1) // The same first whole box was already indeterminate.
            {
                C::FIntervalPoint center;
                if(!TightenRoundedZ(captured,range.First,range.Last,center))return false;
                const auto proved=plan->ProveRoundedSignedSeparation(center,input.SupportRadius,96,budget);
                if(proved==EDecision::Clear)continue;
                if(proved!=EDecision::Indeterminate)return false;
            }
            const uint32 a=C::KinematicCenterDetail::Bits(range.First),b=C::KinematicCenterDetail::Bits(range.Last);
            if(b<=a+1)return false;
            uint32 mid=FMath::Max(a+(b-a)/2,C::KinematicCenterDetail::FloatMinNormal);
            if(useTimeMidpoint)
            {
                const float proposal=float(.5*double(range.First)+.5*double(range.Last));
                const uint32 bits=C::KinematicCenterDetail::Bits(proposal);
                if(a<bits&&bits<b&&bits>=C::KinematicCenterDetail::FloatMinNormal)mid=bits;
            }
            if(mid<=a||mid>=b)return false;
            const float seconds=C::KinematicCenterDetail::FromBits(mid);
            pending.Add({seconds,range.Last});
            pending.Add({range.First,seconds});
        }
        return true;
    };
    bool proved=false;
#if !UE_BUILD_SHIPPING
    if(timeMidpoint&&endBits==0x3b256913u&&cover)
    {
        const auto indices=cover->GetCandidateIndices();
        if(indices.Num()==3&&indices.Contains(1914)&&indices.Contains(1940)&&indices.Contains(2375))
        {
            // Original fall-through remains fully funded: one outer consume
            // per cell, and per tested cell one center consume plus at most
            // one root + 96 spatial consumes per original retained product.
            // BuildCompletePlan never adds a product outside this cover.
            const int32 reserve=limit+(limit-1)*(1+indices.Num()*(1+96));
            constexpr int32 OptionalWork=4096;
            if(budget.ConsumptionFloor>=0&&budget.Remaining>=budget.ConsumptionFloor&&
                budget.Remaining-budget.ConsumptionFloor>reserve)
            {
                const int32 floor=budget.ConsumptionFloor+reserve;
                const uint64 began=budget.Used;const uint64 tick=FPlatformTime::Cycles64();
                int32 attempts=0;
                {
                    C::FScopedDistanceBudgetFloor optional(budget,FMath::Max(floor,budget.Remaining-OptionalWork));
                    if(optional.IsValid())proved=run(true,attempts);
                }
                UE_LOG(LogTemp,Display,TEXT("[NativeSphere799TimeMidpoint] EndBits=%08x Admitted=1 Clear=%d Fallback=%d Attempts=%d Work=%llu Ticks=%llu OriginalReserve=%d Remaining=%d Floor=%d"),
                    endBits,int32(proved),int32(!proved),attempts,budget.Used-began,FPlatformTime::Cycles64()-tick,reserve,budget.Remaining,budget.ConsumptionFloor);
            }
            else
                UE_LOG(LogTemp,Display,TEXT("[NativeSphere799TimeMidpoint] EndBits=%08x Admitted=0 Clear=0 Fallback=1 Attempts=0 Work=0 Ticks=0 OriginalReserve=%d Remaining=%d Floor=%d Reason=reserve"),
                    endBits,reserve,budget.Remaining,budget.ConsumptionFloor);
        }
    }
#endif
    int32 originalAttempts=0;
    if(!proved&&!run(false,originalAttempts))return EClose::Refused;
    partition=MoveTemp(plan);
    return EClose::Clear;
}

inline EClose TryPrefix(const FReferenceSphereStrictSeparation& captured,const FCapturedSignedSphereMotion& motion,
    float end,int32 rangeAttempts,C::FDistanceBudget& budget,FReferenceAffineContactPlanPtr& partition,
    const C::FReferenceCandidateSetPtr& cover,FPrefixObservation& observation,bool splitLastFloat=false,
    bool recenterVertical=false,FPrefixTiming* timing=nullptr,bool timeMidpoint=false)
{
    observation={};
    const auto tryWith=[&](const C::FReferenceCandidateSetPtr& candidates)
    {
        const uint64 before=budget.Used,admission=observation.AdmissionWork;
        if(timing)
        {
            if(candidates)++timing->CoverAttempts;
            else ++timing->ReselectAttempts;
        }
        const auto result=TryPrefixWithCover(captured,motion,end,rangeAttempts,budget,partition,candidates,observation,
            splitLastFloat,recenterVertical,timing,timeMidpoint);
        observation.SeparationWork+=budget.Used-before-(observation.AdmissionWork-admission);
        return result;
    };
    if(cover)
    {
        const auto result=tryWith(cover);
        if(result==EClose::Clear)
        {
            observation.ReusedCover=true;
            return result;
        }
    }
    return tryWith({});
}

// Fresh strict proof at the exact actual native center. The prior partition
// can establish which providers are present, never the result of this proof.
// The caller still authenticates the native point by Contains before use.
inline EClose TryActualPoint(const FPreparedGeometrySelectionPtr& geometry,const FVector3d& actualCenter,
    double radius,C::FDistanceBudget& budget,const FReferenceAffineContactPlan* partition)
{
    const auto point=C::Detail::Point(actualCenter);
    if(partition)
    {
        if(!geometry||!partition->MatchesRoundedSeparationSource(geometry->GetReference(),radius))return EClose::Refused;
        return partition->ProveFreshRoundedSignedPoint(point,radius,96,budget)==EDecision::Clear?
            EClose::Clear:EClose::Refused;
    }
    FReferenceAffineContactPlanPtr plan;
    const auto ready=BuildCompletePlan(geometry,point,radius,budget,plan);
    if(ready==EPlan::NotApplicable)return EClose::NotApplicable;
    if(ready==EPlan::Empty)return EClose::Clear;
    if(ready!=EPlan::Ready||!plan)return EClose::Refused;
    return plan->ProveRoundedSignedSeparation(point,radius,96,budget)==EDecision::Clear?
        EClose::Clear:EClose::Refused;
}
}
