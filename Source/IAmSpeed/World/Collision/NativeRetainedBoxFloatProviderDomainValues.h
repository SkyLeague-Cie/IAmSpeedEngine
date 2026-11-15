#pragma once
#include "NativeRetainedBoxFloatIntersectionBracketValues.h"
namespace Speed::PreparedExecution
{
enum class EFloatProviderDomainPart:uint8 {Surface,UMin,UMax,VMin,VMax};
class FNativeRetainedBoxFloatProviderDomainValues;
using FNativeRetainedBoxFloatProviderDomainValuesPtr=TSharedPtr<const FNativeRetainedBoxFloatProviderDomainValues,ESPMode::ThreadSafe>;
// Over the ENTIRE native physical window[successor(C),U], every intersection
// belongs to ONE retained provider's OPEN UV cell. Complete other-provider,
// closed UV-complement and four true cell-boundary distances are recomputed
// against the NATIVE eight-corner hull. ReferenceDomain seals are not reused.
// No uniqueness of parameter/point, native support corner/cone/unitnormal,
// continuous CCD/TOI, Source/GT/event/response/effect/Frame or Ready follows.
class FNativeRetainedBoxFloatProviderDomainValues final
{
public:
    struct FFragment
    {
        int32 ProviderIndex=INDEX_NONE;
        Analytic::Certified::FReferenceDyadicCell U,V;
        EFloatProviderDomainPart Part=EFloatProviderDomainPart::Surface;
        Analytic::Certified::FIntervalTensorSurface Tensor;
        Analytic::Certified::FTensorDistanceBounds Distance;
    };
private:
    FNativeRetainedBoxFloatIntersectionBracketValuesPtr Bracket;
    FNativeRetainedBoxCornerReferenceValuesPtr Corner;
    Analytic::Certified::FIntervalPoint BoxHull;
    Analytic::FAnalyticNumericEvidence Numeric;
    float First=0,Last=0;TArray<FFragment> Fragments;
    FNativeRetainedBoxFloatProviderDomainValues()=default;
    static bool Boundary(const Analytic::Certified::FIntervalTensorSurface& tensor,EFloatProviderDomainPart part,
        Analytic::Certified::FDistanceBudget& budget,Analytic::Certified::FIntervalTensorSurface& out)
    {
        out={};const bool fixedU=part==EFloatProviderDomainPart::UMin||part==EFloatProviderDomainPart::UMax;
        if(part==EFloatProviderDomainPart::Surface)return false;
        Analytic::Certified::FIntervalTensorSurface edge;
        edge.DegreeU=fixedU?0:tensor.DegreeU;edge.DegreeV=fixedU?tensor.DegreeV:0;
        const int32 count=fixedU?tensor.DegreeV+1:tensor.DegreeU+1;
        for(int32 k=0;k<count;++k)
        {
            if(!budget.Consume())return false;
            const int32 u=fixedU?(part==EFloatProviderDomainPart::UMax?tensor.DegreeU:0):k;
            const int32 v=fixedU?k:(part==EFloatProviderDomainPart::VMax?tensor.DegreeV:0);
            edge.ControlPoints.Add(tensor.ControlPoints[u*(tensor.DegreeV+1)+v]);
        }
        out=MoveTemp(edge);return true;
    }
public:
    static bool TryProve(FNativeRetainedBoxFloatIntersectionBracketValuesPtr bracket,Analytic::Certified::FDistanceBudget& budget,
        FNativeRetainedBoxFloatProviderDomainValuesPtr& out)
    {
        const auto keepPrior=out;out.Reset();namespace C=Analytic::Certified;
        const auto numeric=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(!bracket||!budget.Consume())return false;
        const auto prefix=bracket->GetPrefixForInspection();const auto geometry=prefix->GetGeometryForInspection();
        const auto motion=prefix->GetMotionForInspection();if(!geometry||!motion||!bracket->MatchesInputs(geometry,motion,motion->GetCapturedForInspection()))return false;
        const auto corner=bracket->GetInsideForInspection()->GetFirstReferenceForInspection()->GetPrefixForInspection()->GetCornerForInspection();
        if(!corner||corner->GetGeometryForInspection()!=geometry||corner->GetCornerReferenceForInspection()->GetMotionForInspection()!=motion)return false;
        const auto scene=geometry->GetSelectedUnion();const int32 selected=corner->GetProviderIndex();
        const auto u=corner->GetUCellForInspection(),v=corner->GetVCellForInspection();
        const float first=bracket->GetFirstPossibleSeconds(),last=bracket->GetInsideSeconds();uint32 a=0,b=0;
        const int64 expected=int64(scene->Num())-1+u.Depth+v.Depth+4;
        if(selected<0||selected>=scene->Num()||!u.IsValid()||!v.IsValid()||
            !C::FloorNativeSweepDetail::Ordinal(first,a)||!C::FloorNativeSweepDetail::Ordinal(last,b)||a>b||
            expected>int64(budget.Remaining)-budget.ConsumptionFloor)return false;
        TSharedPtr<FNativeRetainedBoxFloatProviderDomainValues,ESPMode::ThreadSafe> value(new FNativeRetainedBoxFloatProviderDomainValues);
        value->Bracket=MoveTemp(bracket);value->Corner=corner;value->First=first;value->Last=last;
        // Not PhysicalRange::TryProve: this volume MUST allow the selected
        // provider to intersect. Bound all8corners directly using SAME Motion.
        for(int32 k=0;k<8;++k)
        {
            if(!budget.Consume())return false;const auto point=motion->BoundConsumedPhysicalCornerBetweenFloatSeconds(k,first,last);
            if(!C::Detail::ValidPoint(point))return false;value->BoxHull=k?C::TimeDetail::Hull(value->BoxHull,point):point;
        }
        if(!C::Detail::ValidPoint(value->BoxHull))return false;
        const auto zero=AnalyticCertified::Interval::Exact(0);
        const auto append=[&](int32 provider,const C::FIntervalTensorSurface& master,C::FReferenceDyadicCell uc,
            C::FReferenceDyadicCell vc,EFloatProviderDomainPart part)
        {
            if(!budget.Consume())return false;FFragment frag;frag.ProviderIndex=provider;frag.U=uc;frag.V=vc;frag.Part=part;
            C::FIntervalTensorSurface restricted;
            if(!C::RestrictReferenceTensorDyadic(master,uc,vc,budget,restricted))return false;
            if(part==EFloatProviderDomainPart::Surface)frag.Tensor=MoveTemp(restricted);
            else if(!Boundary(restricted,part,budget,frag.Tensor))return false;
            frag.Distance=C::BoundTensorDistanceSquaredIntervalCenter(frag.Tensor,value->BoxHull,budget,0,budget.Remaining,&zero);
            if(frag.Distance.Status==C::EDistanceBoundStatus::Unknown||!AnalyticCertified::Finite(frag.Distance.LowerSquared)||
                !(frag.Distance.LowerSquared>0)||!AnalyticCertified::Finite(frag.Distance.UpperSquared)||
                frag.Distance.UpperSquared<frag.Distance.LowerSquared)return false;
            value->Fragments.Add(MoveTemp(frag));return true;
        };
        for(int32 provider=0;provider<scene->Num();++provider)
        {
            if(!budget.Consume())return false;C::FIntervalTensorSurface master;
            if(!scene->CopySurfaceForQuery(provider,budget,master))return false;
            if(provider!=selected)
            {
                if(!append(provider,master,{},{},EFloatProviderDomainPart::Surface))return false;
                continue;
            }
            // Closed complement of selected OPEN rectangle: U-complement x
            // fullV, then selectedU x V-complement, plus all four boundaries.
            for(uint32 level=1;level<=u.Depth;++level)
            {
                C::FReferenceDyadicCell sibling;sibling.Depth=level;sibling.Index=(u.Index>>(u.Depth-level))^uint64(1);
                if(!append(provider,master,sibling,{},EFloatProviderDomainPart::Surface))return false;
            }
            for(uint32 level=1;level<=v.Depth;++level)
            {
                C::FReferenceDyadicCell sibling;sibling.Depth=level;sibling.Index=(v.Index>>(v.Depth-level))^uint64(1);
                if(!append(provider,master,u,sibling,EFloatProviderDomainPart::Surface))return false;
            }
            for(auto part:{EFloatProviderDomainPart::UMin,EFloatProviderDomainPart::UMax,EFloatProviderDomainPart::VMin,EFloatProviderDomainPart::VMax})
                if(!append(provider,master,u,v,part))return false;
        }
        if(int64(value->Fragments.Num())!=expected||!value->Bracket->MatchesInputs(geometry,motion,motion->GetCapturedForInspection()))return false;
        const auto finalNumeric=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(finalNumeric.ControlFingerprint()!=numeric.ControlFingerprint())return false;
        value->Numeric=finalNumeric;out=MoveTemp(value);return true;
    }
    bool MatchesInputs(const FNativeRetainedBoxFloatIntersectionBracketValuesPtr& bracket,const FPreparedGeometrySelectionPtr& geometry,
        const FNativeSolidStaticValues& captured) const
    {
        const auto numeric=Analytic::InspectCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        return numeric.Status==Analytic::EAnalyticNumericStatus::Complete&&numeric.ControlFingerprint()==Numeric.ControlFingerprint()&&
            Bracket&&Bracket==bracket&&Corner&&Corner->GetGeometryForInspection()==geometry&&
            Bracket->MatchesInputs(geometry,Corner->GetCornerReferenceForInspection()->GetMotionForInspection(),captured)&&
            Analytic::Certified::KinematicCenterDetail::Bits(First)==Analytic::Certified::KinematicCenterDetail::Bits(Bracket->GetFirstPossibleSeconds())&&
            Analytic::Certified::KinematicCenterDetail::Bits(Last)==Analytic::Certified::KinematicCenterDetail::Bits(Bracket->GetInsideSeconds());
    }
    const auto& GetBracketForInspection() const{return Bracket;}
    const auto& GetCornerForInspection() const{return Corner;}
    const auto& GetBoxHullForInspection() const{return BoxHull;}
    const auto& GetNumericForInspection() const{return Numeric;}
    TConstArrayView<FFragment> GetFragmentsForInspection() const{return Fragments;}
    int32 GetProviderIndex() const{return Corner->GetProviderIndex();}
    float GetFirstFloatSeconds() const{return First;}
    float GetLastFloatSeconds() const{return Last;}
};
}
