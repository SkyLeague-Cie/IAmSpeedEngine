#include "PreparedSphereInitialTubeState.h"
#include "PreparedPhysicalExecution.h"
namespace Speed::Analytic::Prepared
{
bool FReferenceSphereInitialTubeState::TryObserve(FPreparedGeometrySelectionPtr geometry,FStaticSphereInitialValues values,
    C::FDistanceBudget& budget,FReferenceSphereInitialTubeStatePtr& out,FString& reason)
{
    out.Reset();reason.Reset();RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    const auto refuse=[&](const TCHAR* why){reason=why;return false;};
    const auto finite=[](const FKinematicState& s)
    {return !s.Location.ContainsNaN()&&!s.Velocity.ContainsNaN()&&!s.Acceleration.ContainsNaN()&&!s.Rotation.ContainsNaN()&&!s.AngularVelocity.ContainsNaN()&&!s.AngularAcceleration.ContainsNaN();};
    uint32 offset=0;FMemory::Memcpy(&offset,&values.StaticContactOffset,4);
    if((offset&0x80000000u)&&(offset&0x7fffffffu))return refuse(TEXT("Negative nonzero static offset bits unsupported"));
    if(!budget.Consume()||!geometry||!geometry->HasValidAliases()||!finite(values.State)||
        !FMath::IsFinite(values.PhysicalRadius)||values.PhysicalRadius<=0||
        !FMath::IsFinite(values.StaticContactOffset)||values.StaticContactOffset<0||
        !C::CandidateDetail::PositiveRadius(values.SupportRadius)||
        !FMath::IsFinite(values.QueryRadius)||values.QueryRadius<=0||
        !PreparedExecution::IsSupportedSphereResponseMargin(values.ActualMargin))
        return refuse(TEXT("Fresh Rs observation requires finite retained shape/state"));
    const double support=double(values.PhysicalRadius)+double(values.StaticContactOffset);
    const float query=float(support+double(values.ActualMargin));uint32 q=0,actual=0;
    FMemory::Memcpy(&q,&query,4);FMemory::Memcpy(&actual,&values.QueryRadius,4);
    if(AnalyticCertified::Bits(support)!=AnalyticCertified::Bits(values.SupportRadius)||q!=actual||double(query)<support)
        return refuse(TEXT("Fresh Rs observation radius dependency/coverage differs"));
    int32 axis=INDEX_NONE;double height=0;
    for(int32 d=0;d<3;++d)
    {
        const uint64 magnitude=AnalyticCertified::Bits(values.State.Location[d])&AnalyticCertified::Magnitude;
        if(!magnitude)continue;
        if(axis!=INDEX_NONE||magnitude<AnalyticCertified::MinNormal)return refuse(TEXT("Fresh initial tube route requires a normal axial center"));
        axis=d;height=AnalyticCertified::FromBits(magnitude);
    }
    if(axis==INDEX_NONE||height>double(query))return refuse(TEXT("Fresh initial point is not in the proved coordinate-plane Rc tube"));
    FReferencePlaneInitialContactPtr closest;
    // Exact distance=h, at the ACTUAL center; all providers outside Rc are
    // strictly farther than h. This proof does not say h equals Rs.
    if(!FReferencePlaneInitialContact::TryBound(geometry,values.State.Location,height,double(query),budget,closest,reason))return false;
    int32 actualAxis=INDEX_NONE;double sign=0;
    if(!closest||!OriginPlaneDetail::SignedAxis(closest->GetGeometricNormal(),actualAxis,sign)||actualAxis!=axis||
        closest->GetKind()!=EReferencePlaneInitialKind::Tangency||AnalyticCertified::Bits(closest->GetRadius())!=AnalyticCertified::Bits(height))
        return refuse(TEXT("Fresh exact closest-distance witness differs"));
    TSharedPtr<FReferenceSphereInitialTubeState,ESPMode::ThreadSafe> result(new FReferenceSphereInitialTubeState);
    result->Values=values;result->ClosestAtHeight=MoveTemp(closest);result->Axis=axis;result->Sign=sign;result->Height=height;
    result->Kind=height>support?EReferenceSphereRsState::SeparatedRs:height==support?EReferenceSphereRsState::TangencyRs:EReferenceSphereRsState::OverlapRs;
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);out=result;return true;
}
bool FReferenceSphereInitialTubeState::TryPlanarStationaryNormalLoad(FVector3d& reaction,FVector3d& effective) const
{
    reaction=effective=FVector3d::ZeroVector;
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(&reaction==&effective||Kind!=EReferenceSphereRsState::TangencyRs||!ClosestAtHeight||Axis<0||Axis>2)return false;
    for(int32 d=0;d<3;++d)
    {
        if(AnalyticCertified::Bits(Values.State.Velocity[d])&AnalyticCertified::Magnitude)return false;
        const uint64 a=AnalyticCertified::Bits(Values.State.Acceleration[d])&AnalyticCertified::Magnitude;
        // Reject before arithmetic/conversion: DAZ must not erase a load or
        // manufacture its sign. No new slop/threshold is introduced.
        if(a>=AnalyticCertified::Infinity||(a&&a<AnalyticCertified::MinNormal))return false;
    }
    effective=Values.State.Acceleration;
    const double a=Values.State.Acceleration[Axis];
    const double normal=Sign>0?a:AnalyticCertified::FromBits(AnalyticCertified::Bits(a)^AnalyticCertified::Sign);
    if(normal<0)
    {
        reaction[Axis]=AnalyticCertified::FromBits(AnalyticCertified::Bits(a)^AnalyticCertified::Sign);
        effective[Axis]=0.; // exact cancellation of two opposite normal values
    }
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);return true;
}
}
