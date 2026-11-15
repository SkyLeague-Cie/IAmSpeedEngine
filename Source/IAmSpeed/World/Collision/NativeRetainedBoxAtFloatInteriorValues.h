#pragma once
#include "NativeFixedBoxAtFloatTimeValues.h"
namespace Speed::PreparedExecution
{
class FNativeRetainedBoxAtFloatInteriorValues;
using FNativeRetainedBoxAtFloatInteriorValuesPtr=TSharedPtr<const FNativeRetainedBoxAtFloatInteriorValues,ESPMode::ThreadSafe>;
// Strict intersection of ONE retained provider with the convex hull of the
// EIGHT STORED native corners at ONE PROVIDED original float time. Not SAT/OBB
// predicate equivalence, earliest float/event, continuous crossing, point/normal
// response, current live pose/FP reproducibility, Frame, authority or readiness.
class FNativeRetainedBoxAtFloatInteriorValues final
{
    FNativeRetainedBoxReferenceFirstEntryValuesPtr First;
    FNativeFixedBoxAtFloatTimeValuesPtr Actual;
    Analytic::Certified::FIntervalPoint SurfacePoint;
    Analytic::Certified::FReferenceDyadicCell ProviderU,ProviderV;
    int32 Tetrahedron[4]={INDEX_NONE,INDEX_NONE,INDEX_NONE,INDEX_NONE};
    AnalyticCertified::Interval Determinants[5];
    FNativeRetainedBoxAtFloatInteriorValues()=default;
    static bool Det(const Analytic::Certified::FIntervalPoint& a,const Analytic::Certified::FIntervalPoint& b,
        const Analytic::Certified::FIntervalPoint& c,Analytic::Certified::FDistanceBudget& budget,AnalyticCertified::Interval& out)
    {
        out={};namespace C=Analytic::Certified;
        if(!C::Detail::ValidPoint(a)||!C::Detail::ValidPoint(b)||!C::Detail::ValidPoint(c))return false;
        C::FIntervalPoint cross;
        for(int32 d=0;d<3;++d)
        {
            if(!budget.Consume())return false;
            cross.Axis[d]=AnalyticCertified::Subtract(
                AnalyticCertified::Multiply(b.Axis[(d+1)%3],c.Axis[(d+2)%3]),
                AnalyticCertified::Multiply(b.Axis[(d+2)%3],c.Axis[(d+1)%3]));
        }
        if(!budget.Consume()||!C::Detail::ValidPoint(cross))return false;
        const auto result=C::ContactDetail::Dot(a,cross);if(!AnalyticCertified::Valid(result))return false;
        out=result;return true;
    }
    static bool Compose(const Analytic::Certified::FReferenceDyadicCell& original,
        const Analytic::Certified::FReferenceDyadicCell& local,Analytic::Certified::FReferenceDyadicCell& out)
    {
        const auto a=original,b=local;out={};
        if(!a.IsValid()||!b.IsValid()||a.Depth+b.Depth>52)return false;
        out.Depth=a.Depth+b.Depth;out.Index=(a.Index<<b.Depth)|b.Index;return out.IsValid();
    }
public:
    // Pure sufficient tetrahedron test. It cannot construct the owned type or
    // authenticate a surface/Box/pose. ALL point-interval values must be inside;
    // a midpoint scalar or any failed tetrahedron never proves hull exclusion.
    static bool BoundStrictTetrahedronInterior(const Analytic::Certified::FIntervalPoint (&corners)[4],
        const Analytic::Certified::FIntervalPoint& point,Analytic::Certified::FDistanceBudget& budget,
        AnalyticCertified::Interval (&out)[5])
    {
        Analytic::Certified::FIntervalPoint copy[4];for(int32 k=0;k<4;++k)copy[k]=corners[k];const auto p=point;
        for(auto& d:out)d={};namespace C=Analytic::Certified;
        Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(!C::Detail::ValidPoint(p))return false;
        for(const auto& q:copy)if(!budget.Consume()||!C::Detail::ValidPoint(q))return false;
        const auto ba=C::ContactDetail::Subtract(copy[1],copy[0]);
        const auto ca=C::ContactDetail::Subtract(copy[2],copy[0]);
        const auto da=C::ContactDetail::Subtract(copy[3],copy[0]);
        const auto pa=C::ContactDetail::Subtract(p,copy[0]);AnalyticCertified::Interval d[5];
        if(!Det(ba,ca,da,budget,d[0])||
            !Det(C::ContactDetail::Subtract(copy[1],p),C::ContactDetail::Subtract(copy[2],p),
                C::ContactDetail::Subtract(copy[3],p),budget,d[1])||
            !Det(pa,ca,da,budget,d[2])||!Det(ba,pa,da,budget,d[3])||!Det(ba,ca,pa,budget,d[4]))return false;
        const bool positive=d[0].Lower>0,negative=d[0].Upper<0;if(!positive&&!negative)return false;
        for(int32 k=1;k<5;++k)if(positive?!(d[k].Lower>0):!(d[k].Upper<0))return false;
        // Na+Nb+Nc+Nd=D for each FIXED realization. All four barycentric
        // coordinates are strictly positive, without division or epsilon.
        Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        for(int32 k=0;k<5;++k)out[k]=d[k];return true;
    }
    static bool TryFind(FNativeRetainedBoxReferenceFirstEntryValuesPtr first,FNativeFixedBoxAtFloatTimeValuesPtr actual,
        Analytic::Certified::FDistanceBudget& budget,FNativeRetainedBoxAtFloatInteriorValuesPtr& out)
    {
        const auto keepPrior=out;out.Reset();namespace C=Analytic::Certified;
        Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(!first||!actual||!budget.Consume()||!actual->SharesMotionWithReferenceEntry(first))return false;
        const auto retained=first->GetPrefixForInspection()->GetCornerForInspection();
        const auto motion=actual->GetMotionForInspection();const auto geometry=retained->GetGeometryForInspection();
        if(!motion||!first->MatchesInputs(geometry,motion->GetCapturedForInspection())||
            !actual->MatchesMotionAndSeconds(motion,motion->GetCapturedForInspection(),actual->GetFloatSeconds()))return false;
        const auto& tensor=retained->GetCornerReferenceForInspection()->GetRootForInspection()->GetTensorForInspection();
        const auto originalU=retained->GetUCellForInspection(),originalV=retained->GetVCellForInspection();
        if(!originalU.IsValid()||!originalV.IsValid())return false;
        C::FIntervalPoint corners[8],boxHull;
        for(int32 k=0;k<8;++k)
        {
            if(!budget.Consume())return false;corners[k]=C::Detail::Point(actual->GetPhysicalCornersForInspection()[k]);
            if(!C::Detail::ValidPoint(corners[k]))return false;boxHull=k?C::TimeDetail::Hull(boxHull,corners[k]):corners[k];
        }
        struct FCell {C::FReferenceDyadicCell U,V;};TArray<FCell> queue;
        if(!budget.Consume())return false;queue.Add(FCell{});
        for(int32 next=0;next<queue.Num();++next)
        {
            if(!budget.Consume())return false;const auto cell=queue[next];C::FReferenceDyadicCell providerU,providerV;
            if(!Compose(originalU,cell.U,providerU)||!Compose(originalV,cell.V,providerV))return false;
            C::FIntervalTensorSurface restricted;
            if(!C::RestrictReferenceTensorDyadic(tensor,cell.U,cell.V,budget,restricted))return false;
            C::Detail::FNode node;node.DegreeU=restricted.DegreeU;node.DegreeV=restricted.DegreeV;node.Net=restricted.ControlPoints;
            C::FIntervalPoint surfaceHull;if(!C::ContactDetail::NetHull(node,surfaceHull))return false;
            bool separate=false;
            for(int32 d=0;d<3;++d)separate|=surfaceHull.Axis[d].Upper<boxHull.Axis[d].Lower||boxHull.Axis[d].Upper<surfaceHull.Axis[d].Lower;
            if(separate)continue; // Only THIS parameter cell, never the union.
            if(!budget.Consume())return false;const auto point=C::Detail::Midpoint(node);
            if(!C::Detail::ValidPoint(point))return false;
            bool pointSeparate=false;
            for(int32 d=0;d<3;++d)pointSeparate|=point.Axis[d].Upper<boxHull.Axis[d].Lower||boxHull.Axis[d].Upper<point.Axis[d].Lower;
            if(!pointSeparate)for(int32 a=0;a<5;++a)for(int32 b=a+1;b<6;++b)
                for(int32 c=b+1;c<7;++c)for(int32 d=c+1;d<8;++d)
                {
                    if(!budget.Consume())return false;
                    const C::FIntervalPoint tetra[4]={corners[a],corners[b],corners[c],corners[d]};AnalyticCertified::Interval determinants[5];
                    if(!BoundStrictTetrahedronInterior(tetra,point,budget,determinants))continue;
                    TSharedPtr<FNativeRetainedBoxAtFloatInteriorValues,ESPMode::ThreadSafe> value(new FNativeRetainedBoxAtFloatInteriorValues);
                    value->First=MoveTemp(first);value->Actual=MoveTemp(actual);value->SurfacePoint=point;
                    value->ProviderU=providerU;value->ProviderV=providerV;
                    value->Tetrahedron[0]=a;value->Tetrahedron[1]=b;value->Tetrahedron[2]=c;value->Tetrahedron[3]=d;
                    for(int32 k=0;k<5;++k)value->Determinants[k]=determinants[k];
                    Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
                    out=MoveTemp(value);return true;
                }
            // A strict tetrahedron test is sufficient only. Even failure of
            // all70 choices does not prove absence from the complete hull.
            const bool splitU=providerU.Depth<52,splitV=providerV.Depth<52;
            if(!splitU&&!splitV)continue;
            for(uint64 sideU=0;sideU<(splitU?2u:1u);++sideU)for(uint64 sideV=0;sideV<(splitV?2u:1u);++sideV)
            {
                if(!budget.Consume())return false;FCell child=cell;
                if(splitU){++child.U.Depth;child.U.Index=cell.U.Index*2+sideU;}
                if(splitV){++child.V.Depth;child.V.Index=cell.V.Index*2+sideV;}
                queue.Add(child); // Charged before allocation; no duplicate route.
            }
        }
        return false; // Unknown, not a miss or first-time selection.
    }
    bool MatchesInputs(const FNativeRetainedBoxReferenceFirstEntryValuesPtr& first,const FNativeFixedBoxAtFloatTimeValuesPtr& actual,
        const FPreparedGeometrySelectionPtr& geometry,const FNativeSolidStaticValues& captured,float seconds) const
    {
        return First&&Actual&&First==first&&Actual==actual&&First->MatchesInputs(geometry,captured)&&
            Actual->SharesMotionWithReferenceEntry(First)&&Actual->MatchesMotionAndSeconds(Actual->GetMotionForInspection(),captured,seconds);
    }
    const auto& GetFirstReferenceForInspection() const{return First;}
    const auto& GetActualAtFloatForInspection() const{return Actual;}
    const auto& GetSurfacePointForInspection() const{return SurfacePoint;}
    const auto& GetTetrahedronForInspection() const{return Tetrahedron;}
    const auto& GetDeterminantsForInspection() const{return Determinants;}
    const auto& GetProviderCellUForInspection() const{return ProviderU;}
    const auto& GetProviderCellVForInspection() const{return ProviderV;}
    FVector2d GetProviderMidpointForInspection() const
    {
        const auto u=ProviderU.GetRange(),v=ProviderV.GetRange();
        // Exact dyadic POINT, potentially depth53, not a depth53 cell code.
        return FVector2d(.5*u.Lower+.5*u.Upper,.5*v.Lower+.5*v.Upper);
    }
};
}
