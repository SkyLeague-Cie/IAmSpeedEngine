#pragma once
#include "NativeRetainedBoxPreCellClearanceValues.h"
namespace Speed::PreparedExecution
{
enum class EReferenceRootDomainPart:uint8 {Surface,UMin,UMax,VMin,VMax};
class FNativeRetainedBoxRootDomainClearanceValues;
using FNativeRetainedBoxRootDomainClearanceValuesPtr=TSharedPtr<const FNativeRetainedBoxRootDomainClearanceValues,ESPMode::ThreadSafe>;
// The entire reference Box, over the root TIME CELL, is separated from every
// other provider, the selected provider's UV complement, and all four closed
// UV-cell boundaries. The OPEN UV CELL remains possible, not just root's K.
// No first-hit/edge-face closure, float/native CCD or response/Ready follows.
class FNativeRetainedBoxRootDomainClearanceValues final
{
public:
    struct FFragment
    {
        int32 ProviderIndex=INDEX_NONE;
        Analytic::Certified::FReferenceDyadicCell U,V;
        EReferenceRootDomainPart Part=EReferenceRootDomainPart::Surface;
        Analytic::Certified::FIntervalTensorSurface Tensor;
        Analytic::Certified::FTensorDistanceBounds Distance;
    };
private:
    FNativeRetainedBoxCornerReferenceValuesPtr Corner;
    Analytic::Certified::FIntervalPoint BoxHull;
    TArray<FFragment> Fragments;
    FNativeRetainedBoxRootDomainClearanceValues()=default;
    static bool Boundary(const Analytic::Certified::FIntervalTensorSurface& tensor,EReferenceRootDomainPart part,
        Analytic::Certified::FDistanceBudget& budget,Analytic::Certified::FIntervalTensorSurface& out)
    {
        out={};const bool fixedU=part==EReferenceRootDomainPart::UMin||part==EReferenceRootDomainPart::UMax;
        if(part==EReferenceRootDomainPart::Surface)return false;
        Analytic::Certified::FIntervalTensorSurface edge;
        edge.DegreeU=fixedU?0:tensor.DegreeU;edge.DegreeV=fixedU?tensor.DegreeV:0;
        const int32 count=fixedU?tensor.DegreeV+1:tensor.DegreeU+1;
        for(int32 k=0;k<count;++k)
        {
            if(!budget.Consume())return false;
            const int32 u=fixedU?(part==EReferenceRootDomainPart::UMax?tensor.DegreeU:0):k;
            const int32 v=fixedU?k:(part==EReferenceRootDomainPart::VMax?tensor.DegreeV:0);
            edge.ControlPoints.Add(tensor.ControlPoints[u*(tensor.DegreeV+1)+v]);
        }
        out=MoveTemp(edge);return true;
    }
public:
    static bool TryProve(FNativeRetainedBoxCornerReferenceValuesPtr corner,
        Analytic::Certified::FDistanceBudget& budget,FNativeRetainedBoxRootDomainClearanceValuesPtr& out)
    {
        const auto keepPrior=out;out.Reset();
        Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(!corner||!budget.Consume())return false;
        const auto geometry=corner->GetGeometryForInspection();const auto root=corner->GetCornerReferenceForInspection();
        if(!geometry||!geometry->HasValidAliases()||geometry->GetModel()!=EPreparedGeometryModel::RetainedReference||!root)return false;
        const auto scene=geometry->GetSelectedUnion();const int32 selected=corner->GetProviderIndex();
        const auto u=corner->GetUCellForInspection(),v=corner->GetVCellForInspection();
        const int64 expected=int64(scene->Num())-1+u.Depth+v.Depth+4;
        if(selected<0||selected>=scene->Num()||!u.IsValid()||!v.IsValid()||expected>budget.Remaining)return false;
        TSharedPtr<FNativeRetainedBoxRootDomainClearanceValues,ESPMode::ThreadSafe> value(new FNativeRetainedBoxRootDomainClearanceValues);
        value->Corner=MoveTemp(corner);
        if(!FNativeRetainedBoxPreCellClearanceValues::BoundWholeBoxReference(*root->GetMotionForInspection(),
            root->GetTimeCellForInspection(),false,budget,value->BoxHull))return false;
        const auto zero=AnalyticCertified::Interval::Exact(0);
        const auto append=[&](int32 provider,const Analytic::Certified::FIntervalTensorSurface& master,
            Analytic::Certified::FReferenceDyadicCell uc,Analytic::Certified::FReferenceDyadicCell vc,EReferenceRootDomainPart part)
        {
            if(!budget.Consume())return false;FFragment f;f.ProviderIndex=provider;f.U=uc;f.V=vc;f.Part=part;
            Analytic::Certified::FIntervalTensorSurface restricted;
            if(!Analytic::Certified::RestrictReferenceTensorDyadic(master,uc,vc,budget,restricted))return false;
            if(part==EReferenceRootDomainPart::Surface)f.Tensor=MoveTemp(restricted);
            else if(!Boundary(restricted,part,budget,f.Tensor))return false;
            f.Distance=Analytic::Certified::BoundTensorDistanceSquaredIntervalCenter(f.Tensor,value->BoxHull,budget,0,budget.Remaining,&zero);
            if(f.Distance.Status==Analytic::Certified::EDistanceBoundStatus::Unknown||
                !AnalyticCertified::Finite(f.Distance.LowerSquared)||!(f.Distance.LowerSquared>0)||
                !AnalyticCertified::Finite(f.Distance.UpperSquared)||f.Distance.UpperSquared<f.Distance.LowerSquared)return false;
            value->Fragments.Add(MoveTemp(f));return true;
        };
        for(int32 provider=0;provider<scene->Num();++provider)
        {
            if(!budget.Consume())return false;Analytic::Certified::FIntervalTensorSurface master;
            if(!scene->CopySurfaceForQuery(provider,budget,master))return false;
            if(provider!=selected)
            {
                if(!append(provider,master,{},{},EReferenceRootDomainPart::Surface))return false;
                continue;
            }
            // Complement is U-complement x FULL V, then U-cell x
            // V-complement. At each level use the opposite sibling on either
            // side, not only the siblings preceding the selected code.
            for(uint32 level=1;level<=u.Depth;++level)
            {
                Analytic::Certified::FReferenceDyadicCell sibling;sibling.Depth=level;
                sibling.Index=(u.Index>>(u.Depth-level))^uint64(1);
                if(!append(provider,master,sibling,{},EReferenceRootDomainPart::Surface))return false;
            }
            for(uint32 level=1;level<=v.Depth;++level)
            {
                Analytic::Certified::FReferenceDyadicCell sibling;sibling.Depth=level;
                sibling.Index=(v.Index>>(v.Depth-level))^uint64(1);
                if(!append(provider,master,u,sibling,EReferenceRootDomainPart::Surface))return false;
            }
            for(auto part:{EReferenceRootDomainPart::UMin,EReferenceRootDomainPart::UMax,
                EReferenceRootDomainPart::VMin,EReferenceRootDomainPart::VMax})
                if(!append(provider,master,u,v,part))return false;
        }
        if(int64(value->Fragments.Num())!=expected)return false;
        Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        out=MoveTemp(value);return true;
    }
    bool MatchesInputs(const FNativeRetainedBoxCornerReferenceValuesPtr& corner,const FNativeSolidStaticValues& captured) const
    {return Corner&&Corner==corner&&Corner->GetGeometryForInspection()->HasValidAliases()&&
        Corner->GetCornerReferenceForInspection()->GetMotionForInspection()->MatchesCapturedValues(captured);}
    const auto& GetCornerForInspection() const{return Corner;}
    const auto& GetBoxHullForInspection() const{return BoxHull;}
    TConstArrayView<FFragment> GetFragmentsForInspection() const{return Fragments;}
};
}
