#pragma once

#include "NativeRepeatedSphereFloorExclusion.h"
#include "ReferenceRepeatedSphereC2Edge.h"

namespace Speed::Analytic::Prepared
{
// Separate operation-local proof. Both full partitions remain owned by their
// original geometry. A floor disposition is eligibility evidence, never a
// claim of geometric separation. This plan cannot construct a native result.
class FReferenceRepeatedSphereFloorPlan final
{
    using Shared=FReferenceSphereC2Prefix;
    using Exclusion=PreparedExecution::FNativeRepeatedSphereFloorExclusion;
    using Request=PreparedExecution::FNativeSolidStaticRequestPtr;
    using I=C::Interval;
    using P=C::FIntervalPoint;
    enum class EDisposition:uint8 {RepeatedFloor,WholeGeometryClear,Target};
    struct FRow {int32 Index;EDisposition Disposition;};
    TArray<FRow,TInlineAllocator<8>> RsRows,RcRows;
    FPreparedGeometrySelectionPtr Geometry;
    C::FReferenceCandidateSetPtr Rs,Rc;
    C::FZeroOffsetKinematicCenterReference Model;
    FReferenceRepeatedSphereC2Edge Edge;
    const Exclusion* Repeated=nullptr;
    const Collision::FResolvedPairSet* Pairs=nullptr;
    Request CurrentRequest;
    int32 TargetRs=INDEX_NONE,TargetRc=INDEX_NONE;
    float H=0.f;
    bool Built=false;

    bool Classify(const C::FReferenceCandidateSet& partition,const P& centers,I radius,I z,
        C::FDistanceBudget& budget,TArray<FRow,TInlineAllocator<8>>& rows,int32& target) const
    {
        namespace A=::AnalyticCertified;
        const auto surfaces=partition.GetSurfacesForFixedSource();
        const auto indices=partition.GetCandidateIndices();
        if(surfaces.IsEmpty()||surfaces.Num()!=indices.Num())return false;
        const auto gap=A::Multiply(A::Subtract(z,radius),A::Add(z,radius));
        const auto squared=A::Square(radius);
        if(!A::Valid(gap)||!A::Valid(squared))return false;
        bool seenFloor=false;
        for(int32 i=0;i<indices.Num();++i)
        {
            const int32 index=indices[i];
            if(!budget.Consume())return false;
            if(Repeated->Excludes(Geometry,index))
            {
                if(seenFloor)return false;
                rows.Add({index,EDisposition::RepeatedFloor});seenFloor=true;
                continue;
            }
            const auto& surface=surfaces[i];
            if(Shared::ClearProjection(surface,centers,squared,budget))
            {
                rows.Add({index,EDisposition::WholeGeometryClear});continue;
            }
            Shared::FArc arc;
            if(!Shared::BuildArc(surface,budget,arc))return false;
            if(Shared::ClearTransverse(surface,centers,arc,z,gap))
            {
                rows.Add({index,EDisposition::WholeGeometryClear});continue;
            }
            if(target!=INDEX_NONE)return false;
            rows.Add({index,EDisposition::Target});target=i;
        }
        return seenFloor&&target!=INDEX_NONE;
    }
    bool IsCurrent() const
    {return Repeated&&Pairs&&Repeated->IsCurrent(CurrentRequest,*Pairs);}

public:
    FReferenceRepeatedSphereFloorPlan()=default;
    FReferenceRepeatedSphereFloorPlan(const FReferenceRepeatedSphereFloorPlan&)=delete;
    FReferenceRepeatedSphereFloorPlan& operator=(const FReferenceRepeatedSphereFloorPlan&)=delete;

    bool TryBuild(const Request& request,const Collision::FResolvedPairSet& actualPairs,const Exclusion& repeated,
        const C::FReferenceCandidateSetPtr& rs,const PreparedExecution::FSelectedWholeHullPartition& rc,
        C::FDistanceBudget& budget)
    {
        namespace A=::AnalyticCertified;
        if(CurrentRequest||!repeated.IsCurrent(request,actualPairs)||!rs)return false;
        CurrentRequest=request;Pairs=&actualPairs;Repeated=&repeated;
        const auto& v=request->GetValues();
        const auto& body=v.BodyState;
        const auto& state=v.OwnerState;
        const auto same=[](FVector3d a,FVector3d b)
        {for(int32 i=0;i<3;++i)if(A::Bits(a[i])!=A::Bits(b[i]))return false;return true;};
        if(!same(body.Location,state.Location)||!same(body.Velocity,state.Velocity)||!same(body.Acceleration,state.Acceleration)||
            !same(v.Start,state.Location)||!v.SphereMarginCaptured||!v.SphereMemberSerial||v.Frozen||v.Prescribed||
            !C::BuildZeroOffsetKinematicCenterReference(state.Location,state.Velocity,state.Acceleration,v.Horizon,
                v.LocalOffset,v.LocalCOM,Model))return false;
        H=v.Horizon;
        Geometry=request->GetSnapshot()->GetGeometrySelection();
        if(!rc.UsesGeometry(Geometry)||Geometry->GetModel()!=EPreparedGeometryModel::RetainedReference||
            rs->GetScene()!=Geometry->GetSelectedUnion()||!Shared::SamePath(rs->GetPathForFixedSource(),Model.GetReferenceCurve())||
            A::Bits(rs->GetRadiusForFixedSource())!=A::Bits(v.SupportRadius)||
            A::Bits(rc.Partition->GetRadiusForFixedSource())!=A::Bits(double(v.QueryRadius))||double(v.QueryRadius)<v.SupportRadius)return false;
        const auto& hullPath=rc.Partition->GetPathForFixedSource();
        const auto rounded=C::BoundConsumedCenterBetweenFloatSeconds(Model,0.f,H);
        if(hullPath.Degree!=0||hullPath.ControlPoints.Num()!=1||!Shared::SamePoint(hullPath.ControlPoints[0],rounded)||
            !C::ReferenceSphereDetail::Contains(rounded,v.End))return false;
        const auto ref=C::CenterDetail::Hull(Model.GetReferenceCurve());
        if(!C::Detail::ValidPoint(ref)||!C::Detail::ValidPoint(rounded))return false;
        auto centers=C::TimeDetail::Hull(ref,rounded);
        centers.Axis[2]=I::Exact(state.Location.Z);
        FStaticSphereInitialValues captured;
        captured.State=state;captured.PhysicalRadius=v.PhysicalRadius;captured.StaticContactOffset=v.StaticContactOffset;
        captured.ActualMargin=v.SphereMargin;captured.QueryRadius=v.QueryRadius;captured.SupportRadius=v.SupportRadius;
        const auto z=centers.Axis[2];
        const auto strongest=I::Exact(double(v.QueryRadius));
        Rs=rs;Rc=rc.Partition;
        if(!Classify(*Rs,centers,strongest,z,budget,RsRows,TargetRs)||!Classify(*Rc,centers,strongest,z,budget,RcRows,TargetRc)||
            RsRows[TargetRs].Index!=RcRows[TargetRc].Index||
            !FReferenceRepeatedSphereC2Edge::Build(Rs->GetSurfacesForFixedSource()[TargetRs],captured,H,budget,Edge)||
            !Edge.MatchesReference(Rs->GetPathForFixedSource())||!Edge.ClearConsumed(0.f,budget))return false;
        Built=IsCurrent();return Built;
    }

    bool TryReferenceContact(const C::FReferenceSphereOptions& options,C::FDistanceBudget& budget,
        C::FTensorSphereContactEnclosure& out) const
    {
        using D=C::TimeDetail::ESignedTimeDecision;
        out={};
        if(!Built||!IsCurrent()||!Rs||!RsRows.IsValidIndex(TargetRs)||!C::ReferenceSphereDetail::OptionsValid(options))return false;
        const auto& path=Rs->GetPathForFixedSource();
        const auto& surface=Rs->GetSurfacesForFixedSource()[TargetRs];
        const auto& v=CurrentRequest->GetValues();
        const auto inside=[&](double end,C::FDistanceBudget& paid)
        {
            auto result=Edge.InsideReferenceEdge(0,end,paid);
            if(result.Decision!=D::Inside)
                result=FReferenceSphereC2SignedEndpoint::EvaluateBoundSurface(surface,path,v.OwnerState.Location.Z,
                    v.SupportRadius,0,end,paid);
            return result;
        };
        const auto endpoint=inside(1.,budget);
        if(endpoint.Decision!=D::Inside)return false;
        const auto sign=[&](double,double end,C::FDistanceBudget& paid)->C::TimeDetail::FSignedTimeInterval
        {
            if(Edge.ClearReference(end,paid))return {D::Clear};
            const auto point=end==1.?endpoint:inside(end,paid);
            return point.Decision==D::Inside?point:C::TimeDetail::FSignedTimeInterval{D::Indeterminate};
        };
        const C::TimeDetail::FSignedTimeQuery signedQuery(sign);
        // The solver view is local. Every omitted row remains in Rs/Rc with
        // its separate exact disposition. KKT sees the original target tensor.
        const auto target=Rs->GetSurfacesForFixedSource().Slice(TargetRs,1);
        out=C::BoundTensorSphereFirstContactGeneric(target,path,v.SupportRadius,budget,options.TimeGap,options.SpatialGap,
            options.SpatialNodesPerAttempt,options.PointWidth,options.NormalWidth,24,nullptr,
            &C::ContactDetail::ThreadContactCellGeometryMemo(),&signedQuery);
        if(out.Status!=C::ETensorSphereContactStatus::FirstContactEnclosed||out.Leaves.IsEmpty()||!IsCurrent())return false;
        for(auto& leaf:out.Leaves)
        {
            if(!budget.Consume()||leaf.SurfaceIndex!=0){out={};return false;}
            leaf.SurfaceIndex=RsRows[TargetRs].Index;
        }
        return true;
    }

    bool ProveConsumed(float end,C::FDistanceBudget& budget) const
    {return Built&&IsCurrent()&&Edge.ClearConsumed(end,budget)&&IsCurrent();}

    bool ProveFresh(float end,const FVector3d& actual,C::FDistanceBudget& budget) const
    {
        if(!Built||!IsCurrent()||!Rc||!FMath::IsFinite(end)||end<=0.f||end>H)return false;
        const auto point=C::BoundConsumedCenterAtFloatSeconds(Model,end);
        const auto& hull=Rc->GetPathForFixedSource();
        return C::ReferenceSphereDetail::Contains(point,actual)&&hull.Degree==0&&hull.ControlPoints.Num()==1&&
            C::ReferenceSphereDetail::Contains(hull.ControlPoints[0],actual)&&Edge.ClearFreshPoint(actual,budget)&&IsCurrent();
    }
};
}
