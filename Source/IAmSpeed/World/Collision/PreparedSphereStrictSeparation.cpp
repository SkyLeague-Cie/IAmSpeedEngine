#include "PreparedSphereStrictSeparation.h"
#include "PreparedPhysicalExecution.h"
#include "SignedAffineSphereNativeClosure.h"
#include "ReferenceSphereC2ContactPlan.h"
namespace Speed::Analytic::Prepared
{
bool FReferenceSphereStrictSeparation::TryProveAt(FPreparedGeometrySelectionPtr geometry,FStaticSphereInitialValues values,
    FVector3d localOffset,FVector3d localCOM,float horizon,float seconds,
    const C::FReferenceSphereOptions& options,C::FDistanceBudget& budget,
    FReferenceSphereStrictSeparationPtr& out,FString& reason)
{
    return TryProveAtImpl(MoveTemp(geometry),values,localOffset,localCOM,horizon,seconds,options,budget,out,reason,false,nullptr);
}
bool FReferenceSphereStrictSeparation::TryProveAtSignedAffine(FPreparedGeometrySelectionPtr geometry,FStaticSphereInitialValues values,
    FVector3d localOffset,FVector3d localCOM,float horizon,float seconds,
    const C::FReferenceSphereOptions& options,C::FDistanceBudget& budget,
    FReferenceSphereStrictSeparationPtr& out,FString& reason,const FReferenceAffineContactPlan* partition)
{
    return TryProveAtImpl(MoveTemp(geometry),values,localOffset,localCOM,horizon,seconds,options,budget,out,reason,true,partition);
}
bool FReferenceSphereStrictSeparation::TryProveAtSignedC2(FPreparedGeometrySelectionPtr geometry,FStaticSphereInitialValues values,
    FVector3d localOffset,FVector3d localCOM,float horizon,float seconds,const C::FReferenceSphereOptions& options,
    C::FDistanceBudget& budget,FReferenceSphereStrictSeparationPtr& out,FString& reason,
    const FReferenceSphereC2Prefix& c2,bool& c2Closed)
{
    return TryProveAtImpl(MoveTemp(geometry),values,localOffset,localCOM,horizon,seconds,options,budget,out,reason,false,nullptr,&c2,&c2Closed);
}
bool FReferenceSphereStrictSeparation::TryProveAtImpl(FPreparedGeometrySelectionPtr geometry,FStaticSphereInitialValues values,
    FVector3d localOffset,FVector3d localCOM,float horizon,float seconds,
    const C::FReferenceSphereOptions& options,C::FDistanceBudget& budget,
    FReferenceSphereStrictSeparationPtr& out,FString& reason,bool signedAffine,const FReferenceAffineContactPlan* partition,
    const FReferenceSphereC2Prefix* c2,bool* c2Closed)
{
    if(c2Closed)*c2Closed=false;
    out.Reset();reason.Reset();RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    const auto refuse=[&](const TCHAR* why){reason=why;return false;};
    const auto finite=[](const FKinematicState& state)
    {return !state.Location.ContainsNaN()&&!state.Velocity.ContainsNaN()&&!state.Acceleration.ContainsNaN()&&!state.Rotation.ContainsNaN()&&!state.AngularVelocity.ContainsNaN()&&!state.AngularAcceleration.ContainsNaN();};
    uint32 offset=0;FMemory::Memcpy(&offset,&values.StaticContactOffset,4);
    if((offset&0x80000000u)&&(offset&0x7fffffffu))return refuse(TEXT("Negative nonzero static offset bits unsupported"));
    if(!budget.Consume()||!geometry||!geometry->HasValidAliases()||!finite(values.State)||
        !C::ReferenceSphereDetail::OptionsValid(options)||!FMath::IsFinite(values.PhysicalRadius)||values.PhysicalRadius<=0||
        !FMath::IsFinite(values.StaticContactOffset)||values.StaticContactOffset<0||!C::CandidateDetail::PositiveRadius(values.SupportRadius)||
        !FMath::IsFinite(values.QueryRadius)||values.QueryRadius<=0||!PreparedExecution::IsSupportedSphereResponseMargin(values.ActualMargin))
        return refuse(TEXT("Strict separation requires finite captured inputs and supported objectives"));
    const double support=double(values.PhysicalRadius)+double(values.StaticContactOffset);
    const float query=float(support+double(values.ActualMargin));uint32 q=0,actual=0;
    FMemory::Memcpy(&q,&query,4);FMemory::Memcpy(&actual,&values.QueryRadius,4);
    if(AnalyticCertified::Bits(support)!=AnalyticCertified::Bits(values.SupportRadius)||q!=actual||double(query)<support)
        return refuse(TEXT("Strict separation radius dependency/coverage differs"));
    const uint32 bits=C::KinematicCenterDetail::Bits(seconds);
    if((bits&0x80000000u)||bits>=0x7f800000u||(bits&&bits<0x00800000u)||seconds>horizon)
        return refuse(TEXT("Strict separation seconds are not supported +0/positive normal within horizon"));
    C::FZeroOffsetKinematicCenterReference model;
    // Pass the actual copied local offset/COM. The model must reject every
    // nonzero bit magnitude, including values DAZ could otherwise erase.
    if(!C::BuildZeroOffsetKinematicCenterReference(values.State.Location,values.State.Velocity,values.State.Acceleration,
        horizon,localOffset,localCOM,model))return refuse(TEXT("Actual zero-offset center model unavailable"));
    const FVector3d center=bits?Speed::SBox::AdvancePosition(values.State.Location,values.State.Velocity,values.State.Acceleration,seconds):values.State.Location;
    const auto bound=C::BoundConsumedCenterAtFloatSeconds(model,seconds);
    if(!C::ReferenceSphereDetail::Contains(bound,center))return refuse(TEXT("Actual center escaped its rounded DAG enclosure"));
    // A fresh singleton theorem is mandatory, even when a caller already owns
    // a complete prefix. No public bool can manufacture this private result.
    using E=SignedAffineSphereNativeClosure::EClose;
    const auto signedPoint=signedAffine?SignedAffineSphereNativeClosure::TryActualPoint(geometry,center,values.SupportRadius,budget,partition):
        E::NotApplicable;
    if(signedPoint==E::Refused)return refuse(TEXT("Whole union signed strict Rs separation at actual native point unavailable; Unknown"));
    const bool freshC2=c2&&c2->ProveFreshActualPoint(geometry,values,localOffset,localCOM,horizon,seconds,center,bound,budget);
    if(c2Closed)*c2Closed=freshC2;
    if(signedPoint==E::NotApplicable&&!freshC2&&
        !C::ProveConsumedCenterFloatSecondsRangeClear(geometry->GetSelectedUnion(),model,seconds,seconds,values.SupportRadius,
            budget,options.SpatialGap,options.SpatialNodesPerAttempt))
        return refuse(TEXT("Whole union strict Rs separation unavailable; Unknown"));
    TSharedPtr<FReferenceSphereStrictSeparation,ESPMode::ThreadSafe> result(new FReferenceSphereStrictSeparation);
    result->Geometry=MoveTemp(geometry);result->CapturedInput=values;result->LocalOffset=localOffset;result->LocalCOM=localCOM;
    result->Model=MoveTemp(model);result->Center=center;result->CenterBound=bound;result->Horizon=horizon;result->Seconds=seconds;
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);out=MoveTemp(result);return true;
}
}
