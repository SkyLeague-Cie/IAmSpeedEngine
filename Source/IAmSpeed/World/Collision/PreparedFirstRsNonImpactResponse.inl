#include "PreparedFirstRsNonImpactResponse.h"
#include "SphereStaticResponsePoint.h"

bool Speed::PreparedExecution::BuildFirstRsNonImpactResponseExpectation(FFirstArrivalRsHitValues values,
    FFirstRsNonImpactResponseInput input,FFirstRsNonImpactResponseExpectation& out,FString& reason)
{
    out={};reason.Reset();
    const auto refuse=[&](const TCHAR* why){out={};reason=why;return false;};
    const auto finite=[](const FKinematicState& v)
    {return !v.Location.ContainsNaN()&&!v.Velocity.ContainsNaN()&&!v.Acceleration.ContainsNaN()&&
        !v.Rotation.ContainsNaN()&&!v.AngularVelocity.ContainsNaN()&&!v.AngularAcceleration.ContainsNaN();};
    const auto& hit=values.Hit;
    if(!values.Geometry||!values.Quality||values.Quality->GetStatus()!=ESphereResponseQualityStatus::BoundedWithinInputMargin||
        !values.Quality->UsesRadialPoint()||!hit.bHit||!hit.bBlockingHit||hit.bStartPenetrating||hit.PreparedGroundEvidence||hit.FirstRsResponseData||
        !hit.SourceId||!hit.SurfaceId||!hit.bSurfaceNormalMayVary||
        !hit.Component.IsExplicitlyNull()||!hit.SubBody.IsExplicitlyNull()||
        hit.ImpactPoint.ContainsNaN()||hit.ImpactNormal.ContainsNaN()||!finite(input.OwnerBefore)||!finite(input.BodyBefore)||
        input.ActualCOM.ContainsNaN()||!FMath::IsFinite(input.PhysicalRadius)||input.PhysicalRadius<=0.0f||
        !FMath::IsFinite(input.StaticContactOffsetCm)||input.StaticContactOffsetCm<0.0f||
        !FMath::IsFinite(input.ImpactThreshold)||input.ImpactThreshold<0.0f)
        return refuse(TEXT("Rs nonimpact prediction lacks closed finite static native inputs"));
    if(!values.Geometry->HasValidAliases()||!values.Quality->GetArrival()||
        !values.Quality->GetArrival()->GetInitial()||!values.Quality->GetArrival()->GetInitial()->GetClosestAtHeight()||
        values.Geometry!=values.Quality->GetArrival()->GetInitial()->GetClosestAtHeight()->GetGeometry()||
        !values.Quality->MatchesPolicy(values.Quality->GetArrival(),values.Quality->GetPolicy()))
        return refuse(TEXT("Rs nonimpact prediction changed its same immutable geometry/quality"));
    const auto& captured=values.Quality->GetArrival()->GetInitial()->GetInput();
    if(ResponseQualityDetail::FloatBits(input.PhysicalRadius)!=ResponseQualityDetail::FloatBits(captured.PhysicalRadius)||
        ResponseQualityDetail::FloatBits(input.StaticContactOffsetCm)!=ResponseQualityDetail::FloatBits(captured.StaticContactOffset)||
        !ResponseQualityDetail::SamePoint(hit.Location,FVector3d(input.BodyBefore.Location)))
        return refuse(TEXT("Rs nonimpact hit centre/Rp/offset differs from the same quality input"));
    FFirstRsNonImpactResponseExpectation prediction;
    prediction.Input=input;prediction.Hit=hit;prediction.OwnerAfter=input.OwnerBefore;
    prediction.Normal=hit.ImpactNormal.GetSafeNormal();
    prediction.Point=SphereStaticResponse::Point(FVector(input.BodyBefore.Location),input.PhysicalRadius,
        hit.ImpactPoint,hit.ImpactNormal,hit.SurfaceId,hit.bSurfaceNormalMayVary,hit.bStartPenetrating,input.StaticContactOffsetCm);
    if(!values.Quality->MatchesConsumedValues(hit.TOI,hit.Location,hit.ImpactPoint,
        hit.ImpactNormal,prediction.Normal,prediction.Point))
        return refuse(TEXT("Rs nonimpact payload differs from its same first-arrival quality proof"));
    // Match ISpeedComponent::GetPhysVelocityAtPoint, including native operation
    // order and narrowing to the float used by ResolveHitVsGround.
    const FVector arm=prediction.Point-input.ActualCOM;
    prediction.VelocityAtPoint=FVector(input.OwnerBefore.Velocity)+FVector::CrossProduct(input.OwnerBefore.AngularVelocity,arm);
    prediction.IncomingNormalSpeed=FVector::DotProduct(prediction.VelocityAtPoint,prediction.Normal);
    if(!FMath::IsFinite(prediction.IncomingNormalSpeed)||prediction.IncomingNormalSpeed < -input.ImpactThreshold)
        return refuse(TEXT("Rs first response is a real impact; nonimpact closure cannot authorize it"));
    // Explicit-null component/subbody means no reciprocal/constraint write and
    // HandleMicroOscillation returns. Thus the native post-point velocity uses
    // the same arithmetic, without an impulse or a P/V projection.
    const FVector postVelocityAtPoint=FVector(input.OwnerBefore.Velocity)+FVector::CrossProduct(input.OwnerBefore.AngularVelocity,arm);
    prediction.PostNormalSpeed=FVector::DotProduct(postVelocityAtPoint,prediction.Normal);
    prediction.NormalAcceleration=FVector::DotProduct(input.OwnerBefore.Acceleration,prediction.Normal);
    if(!FMath::IsFinite(prediction.PostNormalSpeed)||!FMath::IsFinite(prediction.NormalAcceleration))
        return refuse(TEXT("Rs native point-speed/acceleration arithmetic is nonfinite"));
    prediction.AddsAcceleration=prediction.PostNormalSpeed<=input.ImpactThreshold&&prediction.NormalAcceleration<0.0f;
    if(prediction.AddsAcceleration)
    {
        // Unary negation is FLOAT before vector multiplication. Preserve this
        // DAG, including tiny tangent components and signed zero.
        const FVector deltaAcceleration=-prediction.NormalAcceleration*prediction.Normal;
        prediction.OwnerAfter.Acceleration=FVector(input.OwnerBefore.Acceleration)+deltaAcceleration;
    }
    if(!finite(prediction.OwnerAfter))return refuse(TEXT("Rs nonimpact predicted owner is nonfinite"));
    out=MoveTemp(prediction);return true;
}
