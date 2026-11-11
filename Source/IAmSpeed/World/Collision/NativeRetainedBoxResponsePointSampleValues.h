#pragma once
#include "NativeRetainedBoxChartSampleValues.h"

namespace Speed::PreparedExecution
{
class FNativeRetainedBoxResponsePointSampleValues;
using FNativeRetainedBoxResponsePointSampleValuesPtr=TSharedPtr<const FNativeRetainedBoxResponsePointSampleValues,ESPMode::ThreadSafe>;
// Informative response-geometry sample CONDITIONAL on the same model's
// already-provided inside float U. COM(U) and body Q(U) are derived internally.
// U is NOT selected as TOI, and this COM is NOT the live post-advance GetCOM.
// Both ideal branches cover ALL points and either regular unit chart normal
// at THIS fixed COM: P, or C+N*Dot(P-C,N). The actual sample calls the shared
// resolver operation and retains separate point/arm errors against that set.
// Separate window bounds use the SAME model's consumed owner COM over the
// domain's ENTIRE [successor(C),U]. Window errors compare this sample at U to
// every IDEAL geometry in that window, NOT to every other float resolver DAG.
// No branch stability, collision-normal selection, scalar TOI, live constrained
// COM, response-quality allowance, consume/effect/Frame or Ready follows.
class FNativeRetainedBoxResponsePointSampleValues final
{
    FNativeRetainedBoxChartSampleValuesPtr Chart;
    FNativeFixedBoxAtFloatTimeValuesPtr Frame;
    FVector ModelCOM=FVector::ZeroVector,SelectedPoint=FVector::ZeroVector,Arm=FVector::ZeroVector;
    FQuat ModelBodyRotation=FQuat::Identity;
    BoxWallResponseGeometry::FResponsePointValues Scalar;
    Analytic::Certified::FIntervalPoint IdealPoints,IdealArms,WindowCOM,WindowPoints,WindowArms;
    double PointErrorCm=0,ArmErrorCm=0,WindowPointErrorCm=0,WindowArmErrorCm=0;
    Analytic::FAnalyticNumericEvidence Numeric;
    FNativeRetainedBoxResponsePointSampleValues()=default;
public:
    static bool TryBuild(FNativeRetainedBoxChartSampleValuesPtr chart,Analytic::Certified::FDistanceBudget& budget,
        FNativeRetainedBoxResponsePointSampleValuesPtr& out)
    {
        const auto keepPrior=out;out.Reset();namespace C=Analytic::Certified;namespace A=AnalyticCertified;
        const auto numeric=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(!chart||!budget.Consume())return false;
        const auto domain=chart->GetDomainForInspection();if(!domain)return false;
        const auto corner=domain->GetCornerForInspection();if(!corner)return false;
        const auto motion=corner->GetCornerReferenceForInspection()->GetMotionForInspection();
        const auto frame=domain->GetBracketForInspection()->GetInsideForInspection()->GetActualAtFloatForInspection();
        if(!motion||!frame||!chart->MatchesInputs(domain,motion->GetCapturedForInspection())||
            !frame->MatchesMotionAndSeconds(motion,motion->GetCapturedForInspection(),domain->GetLastFloatSeconds()))return false;
        TSharedPtr<FNativeRetainedBoxResponsePointSampleValues,ESPMode::ThreadSafe> value(new FNativeRetainedBoxResponsePointSampleValues);
        value->Chart=MoveTemp(chart);value->Frame=frame;
        value->ModelCOM=frame->GetOwnerAtFloatTimeForInspection().Location;
        value->ModelBodyRotation=frame->GetBodyPoseAtFloatTimeForInspection().Rotation;
        const auto center=C::Detail::Point(value->ModelCOM);
        value->WindowCOM=motion->BoundConsumedOwnerCOMBetweenFloatSeconds(domain->GetFirstFloatSeconds(),domain->GetLastFloatSeconds());
        const auto& points=value->Chart->GetPointHullForInspection();
        const auto& normals=value->Chart->GetEitherChartNormalHullForInspection();
        if(!C::Detail::ValidPoint(center)||!C::Detail::ValidPoint(value->WindowCOM)||!C::Detail::ValidPoint(points)||!C::Detail::ValidPoint(normals)||
            value->ModelBodyRotation.ContainsNaN())return false;
        const auto boundIdeals=[&](const C::FIntervalPoint& com,C::FIntervalPoint& outPoints,C::FIntervalPoint& outArms)
        {
            for(int32 k=0;k<12;++k)if(!budget.Consume())return false;
            const auto relative=C::ContactDetail::Subtract(points,com);
            const auto distance=C::ContactDetail::Dot(normals,relative);
            if(!A::Valid(distance))return false;
            const auto face=C::ScalarContactDetail::AddScaled(com,normals,distance);
            if(!C::Detail::ValidPoint(face))return false;
            // Neither scalar FaceAlignment/.98f nor lost correlations may
            // remove either ideal branch, any COM, point or unit normal.
            outPoints=C::TimeDetail::Hull(points,face);
            outArms=C::ContactDetail::Subtract(outPoints,com);
            return C::Detail::ValidPoint(outPoints)&&C::Detail::ValidPoint(outArms);
        };
        if(!boundIdeals(center,value->IdealPoints,value->IdealArms)||
            !boundIdeals(value->WindowCOM,value->WindowPoints,value->WindowArms)||!budget.Consume())return false;
        value->SelectedPoint=BoxWallResponseGeometry::SelectPoint(true,value->Chart->GetPointForInspection());
        value->Scalar=BoxWallResponseGeometry::BuildResponsePoint(value->Chart->GetConsumedNormalForInspection(),
            value->SelectedPoint,value->ModelCOM,value->ModelBodyRotation);
        // Actual subtraction used for the wall impulse arm if response reaches
        // that statement. This does not assert that the resolver was called.
        value->Arm=value->Scalar.SolvePoint-value->ModelCOM;
        if(!C::ScalarContactDetail::Finite(value->SelectedPoint)||!C::ScalarContactDetail::Finite(value->Scalar.SolvePoint)||
            !FMath::IsFinite(value->Scalar.FaceAlignment)||!C::ScalarContactDetail::Finite(value->Arm)||
            !C::ScalarContactDetail::ErrorToEveryPoint(value->Scalar.SolvePoint,value->IdealPoints,value->PointErrorCm)||
            !C::ScalarContactDetail::ErrorToEveryPoint(value->Arm,value->IdealArms,value->ArmErrorCm)||
            !C::ScalarContactDetail::ErrorToEveryPoint(value->Scalar.SolvePoint,value->WindowPoints,value->WindowPointErrorCm)||
            !C::ScalarContactDetail::ErrorToEveryPoint(value->Arm,value->WindowArms,value->WindowArmErrorCm)||
            !value->Chart->MatchesInputs(domain,motion->GetCapturedForInspection())||
            !frame->MatchesMotionAndSeconds(motion,motion->GetCapturedForInspection(),domain->GetLastFloatSeconds()))return false;
        const auto finalNumeric=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(finalNumeric.ControlFingerprint()!=numeric.ControlFingerprint())return false;
        value->Numeric=finalNumeric;out=MoveTemp(value);return true;
    }
    bool MatchesInputs(const FNativeRetainedBoxChartSampleValuesPtr& chart,const FNativeSolidStaticValues& captured) const
    {
        const auto numeric=Analytic::InspectCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(numeric.Status!=Analytic::EAnalyticNumericStatus::Complete||numeric.ControlFingerprint()!=Numeric.ControlFingerprint()||
            !Chart||Chart!=chart||!Frame)return false;
        const auto domain=Chart->GetDomainForInspection();
        return domain&&Chart->MatchesInputs(domain,captured)&&
            Frame==domain->GetBracketForInspection()->GetInsideForInspection()->GetActualAtFloatForInspection()&&
            Frame->MatchesMotionAndSeconds(Frame->GetMotionForInspection(),captured,domain->GetLastFloatSeconds());
    }
    const auto& GetChartForInspection() const{return Chart;}
    const auto& GetModelFrameAtProvidedInsideFloatForInspection() const{return Frame;}
    const auto& GetModelCOMForInspection() const{return ModelCOM;}
    const auto& GetModelBodyRotationForInspection() const{return ModelBodyRotation;}
    const auto& GetSelectedPointForInspection() const{return SelectedPoint;}
    const auto& GetScalarForInspection() const{return Scalar;}
    const auto& GetActualSubtractedArmForInspection() const{return Arm;}
    const auto& GetIdealEitherBranchPointHullForInspection() const{return IdealPoints;}
    const auto& GetIdealEitherBranchArmHullForInspection() const{return IdealArms;}
    const auto& GetModelWindowOwnerCOMHullForInspection() const{return WindowCOM;}
    const auto& GetIdealWindowEitherBranchPointHullForInspection() const{return WindowPoints;}
    const auto& GetIdealWindowEitherBranchArmHullForInspection() const{return WindowArms;}
    // Errors FROM the scalar sample at U TO every ideal window geometry.
    // No bound on floating BuildResponsePoint outputs at other floats implied.
    double GetSampleToIdealWindowPointErrorCm() const{return WindowPointErrorCm;}
    double GetSampleToIdealWindowArmErrorCm() const{return WindowArmErrorCm;}
    double GetSamplePointErrorCm() const{return PointErrorCm;}
    double GetSampleArmErrorCm() const{return ArmErrorCm;}
};
}
