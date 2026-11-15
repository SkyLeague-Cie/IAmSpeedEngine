#pragma once
#include "ContinuousCenterPath.h"
#include "PlanarProjectionDomains.h"
#include <atomic>

namespace Speed::Analytic::Certified
{
namespace PlanarFamilyTimeShortcutControl
{
#if WITH_DEV_AUTOMATION_TESTS
inline thread_local int32 TestEnabled=-1;
struct FScopedTestEnable
{
    int32 Previous;
    explicit FScopedTestEnable(bool enabled):Previous(TestEnabled){TestEnabled=enabled?1:0;}
    ~FScopedTestEnable(){TestEnabled=Previous;}
};
#endif
IAMSPEED_API bool RuntimeEnabled();
inline bool Enabled()
{
#if WITH_DEV_AUTOMATION_TESTS
    if(TestEnabled>=0)return TestEnabled==1;
#endif
    return RuntimeEnabled();
}
}
enum class EPlanarFamilyBuild:uint8 {NotApplicable,Ready,Work,Arithmetic};
enum class EPlanarFamilySpatialProjection:uint8 {Complete,Work,Arithmetic};
struct FPlanarFamilySpatialProvider
{
    bool Proved=false;
    int32 NormalAxis=INDEX_NONE;
    FIntervalPoint Foot;
    ContactDetail::FProjectedProviderUV Domain;
};
// Bounded is an ordinary certified distance interval, not a threshold decision.
// Append it to preserve the existing diagnostic enum values.
enum class EPlanarFamilyShortcut:uint8 {NotApplicable,Undecided,Clear,Inside,Work,Arithmetic,Bounded};
struct FPlanarFamilyShortcutInspection
{
    uint64 Requests=0,Clear=0,Inside=0,Undecided=0,GenericAttempts=0,Work=0,Arithmetic=0,Bounded=0;
};
// Passive totals. Read only after the owning actor has drained; independent
// relaxed snapshots during active calls are not a coherent frame measurement.
struct FPlanarFamilyShortcutObservation
{
    std::atomic<uint64> Requests{0},Clear{0},Inside{0},Undecided{0},GenericAttempts{0},Work{0},Arithmetic{0},Bounded{0};
};
// Exactly one instance is owned by the IAmSpeed module, including Game callers.
IAMSPEED_API FPlanarFamilyShortcutObservation& PlanarFamilyShortcutObservation();
IAMSPEED_API void PublishPlanarFamilyShortcutObservation();
// Query-local numerical proof only. This plan neither selects owners nor emits
// native hits. Original surfaces, original path and one work account stay alive.
class FPlanarFamilyTimeShortcutPlan final
{
    struct FFamily
    {
        int32 Axis=INDEX_NONE;uint64 HeightBits=0;
        TArray<int32> Members;
        ContactDetail::FPlanarProjectionDomains WholeProof;
        // One completed pure projection for the literal current temporal box.
        // Family membership and the original surfaces are immutable in this
        // query-local plan. No fixed witness or threshold decision is cached.
        mutable bool ProjectionMemoValid=false;
        mutable FIntervalPoint ProjectionMemoCenter;
        mutable ContactDetail::EPlanarProjectionProof ProjectionMemoStatus=
            ContactDetail::EPlanarProjectionProof::NotApplicable;
        mutable ContactDetail::FPlanarProjectionDomains ProjectionMemo;
    };
    // Strictly monotone coordinate-line image. The quintic parameterization
    // remains ORIGINAL; no segment UV is substituted for the curve's UV.
    struct FCoordinateLine
    {
        int32 TangentAxis=INDEX_NONE,Sign=0;
        Interval Start,End;
        double Controls[6]{},Derivative[5]{};
        FIntervalPoint FixedMiddle; // Original polynomial at the fixed UV (0,.5).
    };
    struct FProvider {int32 Family=INDEX_NONE,FamilyLocal=INDEX_NONE,CoordinateLine=INDEX_NONE;};
    const FIntervalTensorSurface* OriginalSurfaces=nullptr;
    int32 OriginalCount=0;
    const FDistanceBudget* OriginalAccount=nullptr;
    uint64 NumericControl=0,MinimumUsed=0;
    int32 MaximumRemaining=0,MinimumFloor=0;
    TArray<FFamily> Families;
    TArray<FCoordinateLine> CoordinateLines;
    TArray<FProvider> Providers;
    mutable FPlanarFamilyShortcutInspection Inspection;
    FPlanarFamilyTimeShortcutPlan()=default;
    static void Observe(uint64& value){if(value<MAX_uint64)++value;}
    EPlanarFamilyShortcut Finish(EPlanarFamilyShortcut status) const
    {
        switch(status)
        {
        case EPlanarFamilyShortcut::Clear:Observe(Inspection.Clear);PlanarFamilyShortcutObservation().Clear.fetch_add(1,std::memory_order_relaxed);break;
        case EPlanarFamilyShortcut::Inside:Observe(Inspection.Inside);PlanarFamilyShortcutObservation().Inside.fetch_add(1,std::memory_order_relaxed);break;
        case EPlanarFamilyShortcut::Bounded:Observe(Inspection.Bounded);PlanarFamilyShortcutObservation().Bounded.fetch_add(1,std::memory_order_relaxed);break;
        case EPlanarFamilyShortcut::Undecided:Observe(Inspection.Undecided);PlanarFamilyShortcutObservation().Undecided.fetch_add(1,std::memory_order_relaxed);break;
        case EPlanarFamilyShortcut::Work:Observe(Inspection.Work);PlanarFamilyShortcutObservation().Work.fetch_add(1,std::memory_order_relaxed);break;
        case EPlanarFamilyShortcut::Arithmetic:Observe(Inspection.Arithmetic);PlanarFamilyShortcutObservation().Arithmetic.fetch_add(1,std::memory_order_relaxed);break;
        default:break;
        }
        return status;
    }
    static bool Pay(FDistanceBudget& budget,uint64 count)
    {
        if(budget.ConsumptionFloor<0||budget.Remaining<budget.ConsumptionFloor||
            count>uint64(int64(budget.Remaining)-budget.ConsumptionFloor)||count>MAX_uint64-budget.Used)return false;
        for(uint64 i=0;i<count;++i)if(!budget.Consume())return false;
        return true;
    }

    static bool SameProjectionCenter(const FIntervalPoint& a,const FIntervalPoint& b)
    {
        for(int32 d=0;d<3;++d)
            if(a.Axis[d].Known!=b.Axis[d].Known||
                AnalyticCertified::Bits(a.Axis[d].Lower)!=AnalyticCertified::Bits(b.Axis[d].Lower)||
                AnalyticCertified::Bits(a.Axis[d].Upper)!=AnalyticCertified::Bits(b.Axis[d].Upper))return false;
        return true;
    }

    static EPlanarFamilyBuild BuildCoordinateLine(const FIntervalTensorSurface& surface,
        FDistanceBudget& budget,FCoordinateLine& out)
    {
        namespace A=::AnalyticCertified;
        out={};
        if(surface.DegreeU!=0||surface.DegreeV!=5||surface.ControlPoints.Num()!=6)
            return EPlanarFamilyBuild::NotApplicable;
        // Fixed reservation covers validation, all five derivative intervals,
        // model coefficients and the one original-net midpoint evaluation.
        if(!Pay(budget,256))return EPlanarFamilyBuild::Work;
        for(const auto& cp:surface.ControlPoints)
            if(!Detail::ValidPoint(cp))return EPlanarFamilyBuild::Arithmetic;
        int32 tangent=INDEX_NONE,constants=0;
        for(int32 d=0;d<3;++d)
        {
            const auto first=surface.ControlPoints[0].Axis[d];
            bool constant=A::Bits(first.Lower)==A::Bits(first.Upper);
            for(const auto& cp:surface.ControlPoints)
                constant&=A::Bits(cp.Axis[d].Lower)==A::Bits(first.Lower)&&
                    A::Bits(cp.Axis[d].Upper)==A::Bits(first.Upper);
            if(constant)++constants;else tangent=d;
        }
        if(constants!=2||tangent==INDEX_NONE)return EPlanarFamilyBuild::NotApplicable;
        bool increasing=true,decreasing=true;
        for(int32 i=0;i<5;++i)
        {
            const auto derivative=A::Multiply(Interval::Exact(5),A::Subtract(
                surface.ControlPoints[i+1].Axis[tangent],surface.ControlPoints[i].Axis[tangent]));
            if(!A::Valid(derivative))return EPlanarFamilyBuild::Arithmetic;
            increasing&=derivative.Lower>0;decreasing&=derivative.Upper<0;
        }
        if(!increasing&&!decreasing)return EPlanarFamilyBuild::NotApplicable;
        FCoordinateLine line;line.TangentAxis=tangent;line.Sign=increasing?1:-1;
        line.Start=surface.ControlPoints[0].Axis[tangent];line.End=surface.ControlPoints[5].Axis[tangent];
        for(int32 i=0;i<6;++i)
        {
            const auto p=surface.ControlPoints[i].Axis[tangent];
            line.Controls[i]=.5*p.Lower+.5*p.Upper;
            if(!A::Finite(line.Controls[i]))return EPlanarFamilyBuild::Arithmetic;
        }
        // Floating coefficients only propose a parameter. They prove nothing.
        for(int32 i=0;i<5;++i)
        {
            line.Derivative[i]=5*(line.Controls[i+1]-line.Controls[i]);
            if(!A::Finite(line.Derivative[i]))return EPlanarFamilyBuild::NotApplicable;
        }
        if(!ProjectedDistanceDetail::EvaluateFixedUV(surface,FVector2d(0,.5),line.FixedMiddle))
            return EPlanarFamilyBuild::Arithmetic;
        out=MoveTemp(line);return EPlanarFamilyBuild::Ready;
    }

    static bool ProposeCoordinateLineParameter(const FCoordinateLine& line,double target,double& parameter)
    {
        namespace A=::AnalyticCertified;
        const auto evaluate=[](const double* coefficients,int32 count,double v)
        {
            double values[6]{};for(int32 i=0;i<count;++i)values[i]=coefficients[i];
            for(int32 n=count-1;n>0;--n)for(int32 i=0;i<n;++i)
                values[i]=(1-v)*values[i]+v*values[i+1];
            return values[0];
        };
        const double span=line.Controls[5]-line.Controls[0];
        if(!A::Finite(target)||!A::Finite(span)||!(line.Sign*span>0))return false;
        double v=(target-line.Controls[0])/span;
        if(!A::Finite(v))return false;
        v=FMath::Clamp(v,0.,1.);double lo=0,hi=1;
        for(int32 iteration=0;iteration<8;++iteration)
        {
            const double value=evaluate(line.Controls,6,v),slope=evaluate(line.Derivative,5,v);
            const double residual=value-target;
            if(!A::Finite(value)||!A::Finite(slope)||!A::Finite(residual)||!(line.Sign*slope>0))return false;
            if(residual==0)break;
            if(line.Sign*residual>0)hi=v;else lo=v;
            double next=v-residual/slope;
            if(next==v)break;
            if(!A::Finite(next)||!(next>lo&&next<hi))next=.5*lo+.5*hi;
            if(next==v)break;
            v=next;
        }
        if(!A::Finite(v)||v<0||v>1)return false;
        parameter=v;return true;
    }

    EPlanarFamilyShortcut TryCoordinateLine(const FIntervalTensorSurface& original,
        const FCoordinateLine& line,const FIntervalPoint& center,Interval radiusSquared,
        FDistanceBudget& budget,FTensorDistanceBounds& out) const
    {
        namespace A=::AnalyticCertified;
        out={};
        if(line.TangentAxis<0||line.TangentAxis>=3||(line.Sign!=1&&line.Sign!=-1))
            return Finish(EPlanarFamilyShortcut::Arithmetic);
        if(!Pay(budget,16))return Finish(EPlanarFamilyShortcut::Work);
        auto orthogonal=Interval::Exact(0);
        for(int32 d=0;d<3;++d)if(d!=line.TangentAxis)
            orthogonal=A::Add(orthogonal,A::Square(A::Subtract(center.Axis[d],original.ControlPoints[0].Axis[d])));
        if(!A::Valid(orthogonal))return Finish(EPlanarFamilyShortcut::Arithmetic);
        // This lower bound is universal even beyond the finite line ends.
        if(orthogonal.Lower>radiusSquared.Upper)return Finish(EPlanarFamilyShortcut::Clear);
        const auto z=center.Axis[line.TangentAxis];
        const auto low=line.Sign>0?line.Start:line.End,high=line.Sign>0?line.End:line.Start;
        if(!(z.Lower>low.Upper&&z.Upper<high.Lower))return Finish(EPlanarFamilyShortcut::Undecided);
        // For every represented fixed curve and every center in this box,
        // strict monotonicity + endpoint interior prove an exact matching
        // tangent coordinate. The minimum distance is therefore orthogonal.
        const bool unresolved=orthogonal.Upper>=radiusSquared.Lower;
        FVector2d uv(0,.5);FIntervalPoint point=line.FixedMiddle;
        if(!unresolved)
        {
            // Eight bounded floating inverse iterations are only a proposal.
            // The ORIGINAL interval polynomial supplies the sole upper witness.
            if(!Pay(budget,1024))return Finish(EPlanarFamilyShortcut::Work);
            double v=0;
            if(!ProposeCoordinateLineParameter(line,.5*z.Lower+.5*z.Upper,v))
                return Finish(EPlanarFamilyShortcut::Undecided);
            uv=FVector2d(0,v);
            if(!ProjectedDistanceDetail::EvaluateFixedUV(original,uv,point))
                return Finish(EPlanarFamilyShortcut::Arithmetic);
        }
        if(!Pay(budget,12))return Finish(EPlanarFamilyShortcut::Work);
        auto distance=Interval::Exact(0);
        for(int32 d=0;d<3;++d)
            distance=A::Add(distance,A::Square(A::Subtract(center.Axis[d],point.Axis[d])));
        const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(!A::Valid(distance)||numeric.Status!=EAnalyticNumericStatus::Complete||numeric.ControlFingerprint()!=NumericControl||
            FMath::Max(0.,orthogonal.Lower)>distance.Upper)
            return Finish(EPlanarFamilyShortcut::Arithmetic);
        // A poor inverse must not prevent finding a genuinely strict inside
        // witness. Retain the old minimum in that case; never widen a tolerance.
        if(!unresolved&&!(distance.Upper<radiusSquared.Lower))return Finish(EPlanarFamilyShortcut::Undecided);
        out.Status=EDistanceBoundStatus::Bounded;out.LowerSquared=FMath::Max(0.,orthogonal.Lower);
        out.UpperSquared=distance.Upper;out.SampleUV=uv;out.SamplePoint=MoveTemp(point);
        return Finish(distance.Upper<radiusSquared.Lower?EPlanarFamilyShortcut::Inside:EPlanarFamilyShortcut::Bounded);
    }
public:
    ~FPlanarFamilyTimeShortcutPlan()=default;
    FPlanarFamilyTimeShortcutPlan(const FPlanarFamilyTimeShortcutPlan&)=delete;
    FPlanarFamilyTimeShortcutPlan& operator=(const FPlanarFamilyTimeShortcutPlan&)=delete;
    const FPlanarFamilyShortcutInspection& GetCountersForInspection() const{return Inspection;}
    void ObserveGenericAttempt() const
    {
        Observe(Inspection.GenericAttempts);
        PlanarFamilyShortcutObservation().GenericAttempts.fetch_add(1,std::memory_order_relaxed);
    }
    int32 GetFamilyForProviderForInspection(int32 local) const
    {return Providers.IsValidIndex(local)?Providers[local].Family:INDEX_NONE;}
    int32 GetFamilyLocalForInspection(int32 local) const
    {return Providers.IsValidIndex(local)?Providers[local].FamilyLocal:INDEX_NONE;}

    static EPlanarFamilyBuild Build(TConstArrayView<FIntervalTensorSurface> surfaces,
        const FIntervalBezierCenterPath& path,FDistanceBudget& budget,
        TUniquePtr<FPlanarFamilyTimeShortcutPlan>& out,bool enableCoordinateLines=false,bool admittedQuadratic=false)
    {
        namespace A=::AnalyticCertified;
        out.Reset();
        const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(numeric.Status!=EAnalyticNumericStatus::Complete)return EPlanarFamilyBuild::Arithmetic;
        // The private quadratic lane has authenticated the exact two plane
        // triangles. The plan below only uses the original whole center hull.
        const bool linear=path.Degree==1&&path.ControlPoints.Num()==2;
        const bool quadratic=admittedQuadratic&&path.Degree==2&&path.ControlPoints.Num()==3&&surfaces.Num()==2;
        if(surfaces.IsEmpty()||(!linear&&!quadratic))return EPlanarFamilyBuild::NotApplicable;
        if(quadratic)
            for(const auto& surface:surfaces)
                if(surface.DegreeU!=1||surface.DegreeV!=1||surface.ControlPoints.Num()!=4)
                    return EPlanarFamilyBuild::NotApplicable;
        if(!CenterDetail::Valid(path))return EPlanarFamilyBuild::Arithmetic;
        if(!Pay(budget,uint64(surfaces.Num())+1))return EPlanarFamilyBuild::Work;
        TUniquePtr<FPlanarFamilyTimeShortcutPlan> plan(new FPlanarFamilyTimeShortcutPlan);
        plan->OriginalSurfaces=surfaces.GetData();plan->OriginalCount=surfaces.Num();
        plan->Providers.SetNum(surfaces.Num());
        constexpr int32 MaximumFamilies=32; // Optional setup work bound, no geometric tolerance.
        for(int32 local=0;local<surfaces.Num();++local)
        {
            if(!Pay(budget,1))return EPlanarFamilyBuild::Work;
            const auto& surface=surfaces[local];
            if(enableCoordinateLines&&surface.DegreeU==0&&surface.DegreeV==5&&surface.ControlPoints.Num()==6)
            {
                FCoordinateLine line;
                const auto status=BuildCoordinateLine(surface,budget,line);
                if(status==EPlanarFamilyBuild::Work||status==EPlanarFamilyBuild::Arithmetic)return status;
                if(status==EPlanarFamilyBuild::Ready)
                {
                    plan->Providers[local].CoordinateLine=plan->CoordinateLines.Add(MoveTemp(line));
                    continue;
                }
            }
            const bool tensorPlanar=TensorPlanarFamilyControl::RuntimeEnabled()&&
                surface.DegreeU==5&&surface.DegreeV==5&&surface.ControlPoints.Num()==36;
            if(!tensorPlanar&&(surface.DegreeU!=1||surface.DegreeV!=1||surface.ControlPoints.Num()!=4))continue;
            bool valid=true;
            for(const auto& p:surface.ControlPoints)
            {if(!Pay(budget,1))return EPlanarFamilyBuild::Work;valid&=Detail::ValidPoint(p);}
            if(!valid)return EPlanarFamilyBuild::Arithmetic;
            // Independent bilinear coefficients are handled by the expanded
            // foot/homotopy proof; no fixed-source correlation is assumed.
            int32 axis=INDEX_NONE;uint64 height=0;
            for(int32 d=0;d<3&&axis==INDEX_NONE;++d)
            {
                const auto h=surface.ControlPoints[0].Axis[d];
                bool same=A::Bits(h.Lower)==A::Bits(h.Upper);
                for(const auto& p:surface.ControlPoints)
                {
                    if(!Pay(budget,1))return EPlanarFamilyBuild::Work;
                    same&=A::Bits(p.Axis[d].Lower)==A::Bits(h.Lower)&&A::Bits(p.Axis[d].Upper)==A::Bits(h.Lower);
                }
                if(same){axis=d;height=A::Bits(h.Lower);}
            }
            if(axis==INDEX_NONE)continue;
            int32 group=INDEX_NONE;
            for(int32 i=0;i<plan->Families.Num();++i)
            {
                if(!Pay(budget,1))return EPlanarFamilyBuild::Work;
                if(plan->Families[i].Axis==axis&&plan->Families[i].HeightBits==height){group=i;break;}
            }
            if(!Pay(budget,1))return EPlanarFamilyBuild::Work;
            if(group==INDEX_NONE)
            {
                if(plan->Families.Num()==MaximumFamilies)return EPlanarFamilyBuild::NotApplicable;
                group=plan->Families.AddDefaulted();
                plan->Families[group].Axis=axis;plan->Families[group].HeightBits=height;
            }
            plan->Families[group].Members.Add(local);
        }
        int32 active=plan->CoordinateLines.Num();
        const auto hull=CenterDetail::Hull(path);
        if(!Detail::ValidPoint(hull))return EPlanarFamilyBuild::Arithmetic;
        for(int32 group=0;group<plan->Families.Num();++group)
        {
            auto& family=plan->Families[group];
            const auto status=ContactDetail::ProjectedPlanarDomainsIndexed(
                surfaces,MakeArrayView(family.Members),hull,budget,family.WholeProof,true);
            if(status==ContactDetail::EPlanarProjectionProof::Work)return EPlanarFamilyBuild::Work;
            if(status==ContactDetail::EPlanarProjectionProof::Arithmetic)return EPlanarFamilyBuild::Arithmetic;
            if(status==ContactDetail::EPlanarProjectionProof::NotApplicable)
            {
                if(!Pay(budget,uint64(family.Members.Num())))return EPlanarFamilyBuild::Work;
                for(int32 j=0;j<family.Members.Num();++j)plan->Providers[family.Members[j]]={group,j};
                ++active;continue;
            }
            if(family.WholeProof.NormalAxis!=family.Axis||
                family.WholeProof.Providers.Num()!=family.Members.Num())return EPlanarFamilyBuild::Arithmetic;
            if(!Pay(budget,uint64(family.Members.Num())))return EPlanarFamilyBuild::Work;
            for(int32 j=0;j<family.Members.Num();++j)
                plan->Providers[family.Members[j]]={group,j};
            ++active;
        }
        if(!active)return EPlanarFamilyBuild::NotApplicable;
        const auto after=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(after.Status!=EAnalyticNumericStatus::Complete||after.ControlFingerprint()!=numeric.ControlFingerprint())
            return EPlanarFamilyBuild::Arithmetic;
        plan->OriginalAccount=&budget;plan->MinimumUsed=budget.Used;plan->MaximumRemaining=budget.Remaining;
        plan->MinimumFloor=budget.ConsumptionFloor;plan->NumericControl=numeric.ControlFingerprint();
        out=MoveTemp(plan);return EPlanarFamilyBuild::Ready;
    }

    // Same original provider indices; every family uses the SAME global bracket
    // center. A first global closest point in a family is also a family minimum.
    EPlanarFamilySpatialProjection ProjectSpatialFamilies(
        TConstArrayView<FIntervalTensorSurface> surfaces,const FIntervalPoint& center,
        FDistanceBudget& budget,TArray<FPlanarFamilySpatialProvider>& out) const
    {
        out.Reset();
        const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(surfaces.GetData()!=OriginalSurfaces||surfaces.Num()!=OriginalCount||&budget!=OriginalAccount||
            numeric.Status!=EAnalyticNumericStatus::Complete||numeric.ControlFingerprint()!=NumericControl||
            budget.Used<MinimumUsed||budget.Remaining>MaximumRemaining||budget.ConsumptionFloor<MinimumFloor||
            budget.Used-MinimumUsed!=uint64(int64(MaximumRemaining)-int64(budget.Remaining))||
            !Detail::ValidPoint(center))return EPlanarFamilySpatialProjection::Arithmetic;
        if(!Pay(budget,uint64(OriginalCount)+1))return EPlanarFamilySpatialProjection::Work;
        TArray<FPlanarFamilySpatialProvider> draft;draft.SetNum(OriginalCount);
        for(int32 group=0;group<Families.Num();++group)
        {
            if(!Pay(budget,1))return EPlanarFamilySpatialProjection::Work;
            const auto& family=Families[group];
            // A smaller bracket may now be interior even across a whole-path seam.
            ContactDetail::FPlanarProjectionDomains proof;
            const auto status=ContactDetail::ProjectedPlanarDomainsIndexed(
                surfaces,MakeArrayView(family.Members),center,budget,proof,true);
            if(status==ContactDetail::EPlanarProjectionProof::Work)return EPlanarFamilySpatialProjection::Work;
            if(status==ContactDetail::EPlanarProjectionProof::Arithmetic)return EPlanarFamilySpatialProjection::Arithmetic;
            if(status==ContactDetail::EPlanarProjectionProof::NotApplicable)continue; // ONLY this group remains generic.
            if(proof.NormalAxis!=family.Axis||proof.Providers.Num()!=family.Members.Num())
                return EPlanarFamilySpatialProjection::Arithmetic;
            if(!Pay(budget,uint64(family.Members.Num())))return EPlanarFamilySpatialProjection::Work;
            for(int32 j=0;j<family.Members.Num();++j)
            {
                const int32 original=family.Members[j];
                if(!Providers.IsValidIndex(original)||Providers[original].Family!=group||Providers[original].FamilyLocal!=j)
                    return EPlanarFamilySpatialProjection::Arithmetic;
                auto& target=draft[original];target.Proved=true;
                target.NormalAxis=proof.NormalAxis;target.Foot=proof.Foot;target.Domain=proof.Providers[j];
            }
        }
        const auto after=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(after.Status!=EAnalyticNumericStatus::Complete||after.ControlFingerprint()!=NumericControl)
            return EPlanarFamilySpatialProjection::Arithmetic;
        out=MoveTemp(draft);return EPlanarFamilySpatialProjection::Complete;
    }

    EPlanarFamilyShortcut TryProvider(TConstArrayView<FIntervalTensorSurface> surfaces,int32 originalLocal,
        const FIntervalPoint& center,Interval radiusSquared,FDistanceBudget& budget,
        FTensorDistanceBounds& insideWitness) const
    {
        namespace A=::AnalyticCertified;
        insideWitness={};Observe(Inspection.Requests);
        PlanarFamilyShortcutObservation().Requests.fetch_add(1,std::memory_order_relaxed);
        const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(surfaces.GetData()!=OriginalSurfaces||surfaces.Num()!=OriginalCount||
            &budget!=OriginalAccount||!Providers.IsValidIndex(originalLocal)||
            numeric.Status!=EAnalyticNumericStatus::Complete||numeric.ControlFingerprint()!=NumericControl||
            budget.Used<MinimumUsed||budget.Remaining>MaximumRemaining||budget.ConsumptionFloor<MinimumFloor||
            budget.Used-MinimumUsed!=uint64(int64(MaximumRemaining)-int64(budget.Remaining))||
            !Detail::ValidPoint(center)||!A::Valid(radiusSquared)||radiusSquared.Lower<0||!(radiusSquared.Upper>0))
            return Finish(EPlanarFamilyShortcut::Arithmetic);
        const auto provider=Providers[originalLocal];
        if(provider.CoordinateLine!=INDEX_NONE)
        {
            if(!CoordinateLines.IsValidIndex(provider.CoordinateLine))return Finish(EPlanarFamilyShortcut::Arithmetic);
            return TryCoordinateLine(surfaces[originalLocal],CoordinateLines[provider.CoordinateLine],
                center,radiusSquared,budget,insideWitness);
        }
        if(provider.Family==INDEX_NONE)return EPlanarFamilyShortcut::NotApplicable;
        if(!Families.IsValidIndex(provider.Family))return Finish(EPlanarFamilyShortcut::Arithmetic);
        const auto& family=Families[provider.Family];
        if(!family.Members.IsValidIndex(provider.FamilyLocal))return Finish(EPlanarFamilyShortcut::Arithmetic);
        if(!Pay(budget,3))return Finish(EPlanarFamilyShortcut::Work);
        const auto axial=A::Square(A::Subtract(center.Axis[family.Axis],
            Interval::Exact(A::FromBits(family.HeightBits))));
        if(!A::Valid(axial))return Finish(EPlanarFamilyShortcut::Arithmetic);
        // Supporting-plane lower bound is universal even for a descendant hull
        // with outward roundoff outside the original whole-path hull.
        if(axial.Lower>radiusSquared.Upper)return Finish(EPlanarFamilyShortcut::Clear);
        // Reuse only the completed pure inverse of this SAME literal hull.
        // All source/account/numeric-context guards above precede the lookup.
        // A different hull always recomputes; whole-path UVs are never reused.
        if(!Pay(budget,1))return Finish(EPlanarFamilyShortcut::Work);
        if(!family.ProjectionMemoValid||!SameProjectionCenter(family.ProjectionMemoCenter,center))
        {
            ContactDetail::FPlanarProjectionDomains current;
            const auto projection=ContactDetail::ProjectedPlanarDomainsIndexed(
                surfaces,MakeArrayView(family.Members),center,budget,current,true);
            if(projection==ContactDetail::EPlanarProjectionProof::Work)return Finish(EPlanarFamilyShortcut::Work);
            if(projection==ContactDetail::EPlanarProjectionProof::Arithmetic)return Finish(EPlanarFamilyShortcut::Arithmetic);
            if(projection!=ContactDetail::EPlanarProjectionProof::Proved&&
                projection!=ContactDetail::EPlanarProjectionProof::NotApplicable)
                return Finish(EPlanarFamilyShortcut::Arithmetic);
            if(projection==ContactDetail::EPlanarProjectionProof::Proved&&
                (current.NormalAxis!=family.Axis||current.Providers.Num()!=family.Members.Num()))
                return Finish(EPlanarFamilyShortcut::Arithmetic);
            // Publish only after the complete projection and its publication
            // debit. A failed/partial attempt leaves the previous memo intact.
            if(!Pay(budget,1))return Finish(EPlanarFamilyShortcut::Work);
            family.ProjectionMemo=MoveTemp(current);
            family.ProjectionMemoCenter=center;
            family.ProjectionMemoStatus=projection;
            family.ProjectionMemoValid=true;
        }
        if(family.ProjectionMemoStatus==ContactDetail::EPlanarProjectionProof::NotApplicable)
            return Finish(EPlanarFamilyShortcut::Undecided);
        if(!family.ProjectionMemo.Providers.IsValidIndex(provider.FamilyLocal))return Finish(EPlanarFamilyShortcut::Arithmetic);
        const auto& domain=family.ProjectionMemo.Providers[provider.FamilyLocal];
        const bool completeBounds=TensorPlanarFamilyControl::RuntimeEnabled();
        const bool supportOnly=domain.Outside||!domain.Bounded;
        if(supportOnly&&!completeBounds)return Finish(EPlanarFamilyShortcut::Undecided);
        if(!Pay(budget,1))return Finish(EPlanarFamilyShortcut::Work);
        const FVector2d uv=supportOnly?FVector2d(0.,0.):FVector2d(
            FMath::Clamp(.5*domain.U.Lower+.5*domain.U.Upper,0.,1.),
            FMath::Clamp(.5*domain.V.Lower+.5*domain.V.Upper,0.,1.));
        if(!A::Finite(uv.X)||!A::Finite(uv.Y))return Finish(EPlanarFamilyShortcut::Arithmetic);
        const auto& original=surfaces[originalLocal];
        const uint64 controls=uint64(original.ControlPoints.Num());
        const uint64 blends=uint64(original.DegreeU+1)*uint64(original.DegreeV)*uint64(original.DegreeV+1)/2+
            uint64(original.DegreeU)*uint64(original.DegreeU+1)/2;
        // Same paid fixed-UV evaluation and actual squared-distance arithmetic
        // as DirectPlaneUVProposal95, independent of its flag/local UV allowance.
        if(!Pay(budget,supportOnly?4:controls+blends+3))return Finish(EPlanarFamilyShortcut::Work);
        FIntervalPoint point;
        if(supportOnly)point=original.ControlPoints[0]; // Original tensor at UV=(0,0).
        else if(!ProjectedDistanceDetail::EvaluateFixedUV(original,uv,point))return Finish(EPlanarFamilyShortcut::Arithmetic);
        auto distance=Interval::Exact(0);
        for(int32 d=0;d<3;++d)
            distance=A::Add(distance,A::Square(A::Subtract(center.Axis[d],point.Axis[d])));
        const auto after=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(!A::Valid(distance)||after.Status!=EAnalyticNumericStatus::Complete||after.ControlFingerprint()!=NumericControl)
            return Finish(EPlanarFamilyShortcut::Arithmetic);
        if(!completeBounds&&!(distance.Upper<radiusSquared.Lower))return Finish(EPlanarFamilyShortcut::Undecided);
        insideWitness.Status=EDistanceBoundStatus::Bounded;
        insideWitness.LowerSquared=FMath::Max(0.,axial.Lower);insideWitness.UpperSquared=distance.Upper;
        insideWitness.SampleUV=uv;insideWitness.SamplePoint=MoveTemp(point);
        if(insideWitness.LowerSquared>insideWitness.UpperSquared)
        {insideWitness={};return Finish(EPlanarFamilyShortcut::Arithmetic);}
        return Finish(distance.Upper<radiusSquared.Lower?EPlanarFamilyShortcut::Inside:EPlanarFamilyShortcut::Bounded);
    }
};
}
