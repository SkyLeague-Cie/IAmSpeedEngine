#pragma once
#include "PreparedNativeSolidRequest.h"
#include "IAmSpeed/World/Analytic/FloorNativeSolidSweepBounds.h"
#include "IAmSpeed/World/Analytic/TensorPointRootBounds.h"
#include "IAmSpeed/World/Analytic/ReferenceDyadicCellBounds.h"
namespace Speed::PreparedExecution
{
class FNativeFixedBoxReferenceMotionValues;
using FNativeFixedBoxReferenceMotionValuesPtr=TSharedPtr<const FNativeFixedBoxReferenceMotionValues,ESPMode::ThreadSafe>;
// Values-only binding of a fixed Box's captured PVA/COM/offset/Q/physical
// extents to ONE common owner-COM polynomial and eight constant corner bounds.
// Not GT/Step/Snapshot authentication, float Integrate CCD or World authority.
class FNativeFixedBoxReferenceMotionValues final
{
    FNativeSolidStaticValues Captured;Analytic::Certified::FFloorNativeSolidModel Fixed;
    Analytic::Certified::FIntervalPoint Offsets[8];Analytic::Certified::FIntervalBezierCenterPath Paths[8];
    FNativeFixedBoxReferenceMotionValues()=default;
public:
    static bool TryBuild(const FNativeSolidStaticValues& values,Analytic::Certified::FDistanceBudget& budget,
        FNativeFixedBoxReferenceMotionValuesPtr& out)
    {
        const auto keepPrior=out;out.Reset();
        Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(values.Shape!=ENativeSolidQueryShape::Box||values.SphereMarginCaptured||!budget.Consume())return false;
        Analytic::Certified::FFloorNativeSolidModel fixed;
        if(!Analytic::Certified::BuildFloorNativeSolidModel(values,fixed)||!fixed.IsFixedBox())return false;
        TSharedPtr<FNativeFixedBoxReferenceMotionValues,ESPMode::ThreadSafe> result(new FNativeFixedBoxReferenceMotionValues);
        result->Captured=values;result->Fixed=MoveTemp(fixed);
        for(int32 k=0;k<8;++k)
        {
            if(!budget.Consume()||!result->Fixed.BuildReferenceCornerPath(k,result->Paths[k]))return false;
            result->Offsets[k]=result->Fixed.BoundPhysicalCornerOffset(k);
            if(!Analytic::Certified::Detail::ValidPoint(result->Offsets[k]))return false;
        }
        Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        out=MoveTemp(result);return true;
    }
    bool MatchesCapturedValues(const FNativeSolidStaticValues& values) const
    {return SameNativeSolidStaticValues(Captured,values);}
    const auto& GetCapturedForInspection() const{return Captured;}
    const auto& GetOwnerReferenceCurveForInspection() const{return Fixed.GetOwnerReferenceCurveForInspection();}
    const auto& GetCornerPathsForInspection() const{return Paths;}
    const auto& GetPhysicalOffsetsForInspection() const{return Offsets;}
    // Separate enclosures at the PROVIDED original float seconds, never a
    // seconds/H/seconds round trip. Both retain this SAME immutable Fixed.
    Analytic::Certified::FIntervalPoint BoundPhysicalCornerAtFloatSeconds(int32 corner,float seconds,bool consumed) const
    {return Fixed.BoundPhysicalCorner(corner,seconds,seconds,consumed);}
    // Response COM is the owner COM, distinct from the body origin below.
    Analytic::Certified::FIntervalPoint BoundConsumedOwnerCOMBetweenFloatSeconds(float first,float last) const
    {return Fixed.BoundConsumedOwnerCOMBetweenFloatSeconds(first,last);}
    Analytic::Certified::FIntervalPoint BoundConsumedBodyOriginBetweenFloatSeconds(float first,float last) const
    {return Fixed.BoundCenter(first,last,true);}
    Analytic::Certified::FIntervalPoint BoundConsumedPhysicalCornerBetweenFloatSeconds(int32 corner,float first,float last) const
    {return Fixed.BoundPhysicalCorner(corner,first,last,true);}
    float GetOriginalHorizon() const{return Captured.Horizon;}
};
class FNativeFixedBoxCornerReferenceValues;
using FNativeFixedBoxCornerReferenceValuesPtr=TSharedPtr<const FNativeFixedBoxCornerReferenceValues,ESPMode::ThreadSafe>;
// Joins the local point/tensor root and strict support/approach to the SAME
// captured common motion. Independent CP boxes are only an overapproximation;
// they are not a replacement set of eight unrelated physical trajectories.
// Still no global earliest/prefix/other-feature/actual float/contact-error,
// native normal, response, constrained trajectory, Frame or readiness proof.
class FNativeFixedBoxCornerReferenceValues final
{
    FNativeFixedBoxReferenceMotionValuesPtr Motion;
    Analytic::Certified::FTensorPointRootBoundsPtr Root;
    Analytic::Certified::FIntervalPoint RawNormal;int32 Corner=INDEX_NONE,NormalSign=0;
    Analytic::Certified::FReferenceDyadicCell TimeCell;
    Analytic::Certified::FIntervalBezierCenterPath RestrictedPointPath;
    FNativeFixedBoxCornerReferenceValues()=default;
public:
    static bool TryProve(FNativeFixedBoxReferenceMotionValuesPtr motion,
        const Analytic::Certified::FIntervalTensorSurface& tensor,int32 corner,int32 normalSign,
        Analytic::Certified::FDistanceBudget& budget,FNativeFixedBoxCornerReferenceValuesPtr& out)
    {
        return TryProveInTimeCell(MoveTemp(motion),tensor,corner,normalSign,{},budget,out);
    }
    static bool TryProveInTimeCell(FNativeFixedBoxReferenceMotionValuesPtr motion,
        const Analytic::Certified::FIntervalTensorSurface& tensor,int32 corner,int32 normalSign,
        const Analytic::Certified::FReferenceDyadicCell& cell,Analytic::Certified::FDistanceBudget& budget,
        FNativeFixedBoxCornerReferenceValuesPtr& out)
    {
        const auto keepPrior=out;out.Reset();
        Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(!motion||corner<0||corner>=8||(normalSign!=1&&normalSign!=-1)||!cell.IsValid()||!budget.Consume())return false;
        Analytic::Certified::FIntervalBezierCenterPath point;
        if(!Analytic::Certified::RestrictReferencePathDyadic(motion->GetCornerPathsForInspection()[corner],cell,budget,point))return false;
        Analytic::Certified::FTensorPointRootBoundsPtr root;Analytic::Certified::FIntervalPoint normal;
        if(!Analytic::Certified::FTensorPointRootBounds::TryProve(tensor,point,budget,root)||
            !root||!root->ProveTranslatedCloudVertexApproach(motion->GetPhysicalOffsetsForInspection(),corner,normalSign,budget,normal))return false;
        TSharedPtr<FNativeFixedBoxCornerReferenceValues,ESPMode::ThreadSafe> value(new FNativeFixedBoxCornerReferenceValues);
        value->Motion=MoveTemp(motion);value->Root=MoveTemp(root);value->RawNormal=normal;value->Corner=corner;value->NormalSign=normalSign;
        value->TimeCell=cell;value->RestrictedPointPath=MoveTemp(point);
        Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        out=MoveTemp(value);return true;
    }
    bool MatchesInputs(const FNativeSolidStaticValues& values,const Analytic::Certified::FIntervalTensorSurface& tensor,
        int32 corner,int32 normalSign) const
    {
        return MatchesInputsInTimeCell(values,tensor,corner,normalSign,{});
    }
    bool MatchesInputsInTimeCell(const FNativeSolidStaticValues& values,const Analytic::Certified::FIntervalTensorSurface& tensor,
        int32 corner,int32 normalSign,const Analytic::Certified::FReferenceDyadicCell& cell) const
    {
        return Motion&&Root&&Corner==corner&&NormalSign==normalSign&&TimeCell.SameCode(cell)&&Motion->MatchesCapturedValues(values)&&
            Root->MatchesInputs(tensor,RestrictedPointPath);
    }
    AnalyticCertified::Interval GetOriginalNormalizedTimeBounds() const
    {return Root?TimeCell.MapLocal(Root->GetLocalRootBounds().Axis[2]):AnalyticCertified::Interval{};}
    AnalyticCertified::Interval GetOriginalReferenceSecondsBounds() const
    {
        const auto time=GetOriginalNormalizedTimeBounds();if(!Motion||!AnalyticCertified::Valid(time))return {};
        auto seconds=AnalyticCertified::Multiply(AnalyticCertified::Interval::Exact(double(Motion->GetOriginalHorizon())),time);
        if(!AnalyticCertified::Valid(seconds))return {};
        seconds.Lower=FMath::Max(0.,seconds.Lower);seconds.Upper=FMath::Min(double(Motion->GetOriginalHorizon()),seconds.Upper);
        return AnalyticCertified::Valid(seconds)?seconds:AnalyticCertified::Interval{};
    }
    const auto& GetTimeCellForInspection() const{return TimeCell;}
    const auto& GetRestrictedPointPathForInspection() const{return RestrictedPointPath;}
    const auto& GetMotionForInspection() const{return Motion;}
    const auto& GetRootForInspection() const{return Root;}
    const auto& GetRawNormalForInspection() const{return RawNormal;}
    int32 GetCornerIndex() const{return Corner;}
    int32 GetNormalSign() const{return NormalSign;}
};
}
