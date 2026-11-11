#include "PreparedSphereInitialEvaluation.h"
#include "PreparedPhysicalExecution.h"
namespace Speed::Analytic::Prepared
{
bool FReferenceSphereInitialEvaluation::TryEvaluate(FPreparedGeometrySelectionPtr geometry,FStaticSphereInitialValues values,
    C::FDistanceBudget& budget,FReferenceSphereInitialEvaluationPtr& out,FString& reason)
{
    out.Reset();reason.Reset();RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    auto refuse=[&](const TCHAR* why){reason=why;return false;};
    const auto finite=[](const FKinematicState& s)
    {return !s.Location.ContainsNaN()&&!s.Velocity.ContainsNaN()&&!s.Acceleration.ContainsNaN()&&!s.Rotation.ContainsNaN()&&!s.AngularVelocity.ContainsNaN()&&!s.AngularAcceleration.ContainsNaN();};
    uint32 offsetBits=0;FMemory::Memcpy(&offsetBits,&values.StaticContactOffset,4);
    if((offsetBits&0x80000000u)&&(offsetBits&0x7fffffffu))return refuse(TEXT("Negative nonzero static offset bits unsupported"));
    if(!budget.Consume()||!geometry||!geometry->HasValidAliases()||!finite(values.State)||
        !FMath::IsFinite(values.PhysicalRadius)||values.PhysicalRadius<=0||
        !FMath::IsFinite(values.StaticContactOffset)||values.StaticContactOffset<0||
        !FMath::IsFinite(values.SupportRadius)||values.SupportRadius<=0||
        !FMath::IsFinite(values.QueryRadius)||values.QueryRadius<=0||
        !PreparedExecution::IsSupportedSphereResponseMargin(values.ActualMargin))
        return refuse(TEXT("Finite actual sphere shape/state and bounded initial evaluation unavailable"));
    // Retain actual M; never infer it from rounded Rc-Rp. Match the native
    // float->double support sum and float query conversion in that order.
    const double support=double(values.PhysicalRadius)+double(values.StaticContactOffset);
    const float query=float(support+double(values.ActualMargin));
    uint32 queryBits=0,actualQueryBits=0;FMemory::Memcpy(&queryBits,&query,4);FMemory::Memcpy(&actualQueryBits,&values.QueryRadius,4);
    if(AnalyticCertified::Bits(support)!=AnalyticCertified::Bits(values.SupportRadius)||queryBits!=actualQueryBits)
        return refuse(TEXT("Actual Rp/offset/Rs/M/Rc dependency bits do not match"));
    FReferencePlaneInitialContactPtr initial;
    if(!FReferencePlaneInitialContact::TryBound(geometry,values.State.Location,support,support,budget,initial,reason))return false;
    int32 axis=INDEX_NONE;double sign=0;
    if(!OriginPlaneDetail::SignedAxis(initial->GetGeometricNormal(),axis,sign))
        return refuse(TEXT("Exact initial coordinate-plane normal unavailable"));
    // The Rs theorem proves the exact initial distance: origin is a closed
    // provider point, ALL candidates coplanar, all others excluded at Rs>=Rp.
    // A skin-only overlap may have physical Rp SEPARATED, not StartPenetrating.
    const uint64 height=AnalyticCertified::Bits(initial->GetCenter()[axis])&AnalyticCertified::Magnitude;
    const uint64 physical=AnalyticCertified::Bits(double(values.PhysicalRadius));
    const auto kind=height>physical?EPhysicalSphereInitialKind::Separated:
        height==physical?EPhysicalSphereInitialKind::Tangency:EPhysicalSphereInitialKind::Overlap;
    FKinematicState preview=values.State;preview.Location=initial->GetTangencyTarget();
    preview.Quantize(values.State.Rotation); // LOCAL value only; never a setter/sync.
    if(!finite(preview))return refuse(TEXT("Native local quantizer produced a nonfinite preview"));
    FReferencePlaneEndpointSeparationPtr endpoint;
    if(!FReferencePlaneEndpointSeparation::TryBound(initial,preview.Location,budget,endpoint,reason))return false;
    TSharedPtr<FReferenceSphereInitialEvaluation,ESPMode::ThreadSafe> result(new FReferenceSphereInitialEvaluation);
    result->Input=values;result->Preview=preview;result->PhysicalKind=kind;
    result->SupportInitial=MoveTemp(initial);result->Endpoint=MoveTemp(endpoint);
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);out=result;return true;
}
}
