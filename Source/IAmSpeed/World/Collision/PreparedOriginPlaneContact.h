#pragma once
#include "PreparedIntervalScene.h"
#include "PreparedGeometrySelection.h"
#include "IAmSpeed/World/Analytic/AnalyticWorldQuery.h"
namespace Speed::Analytic::Prepared
{
enum class EOriginPlaneContactStatus:uint8 { Unknown,InitialFaceOverlap,InitialFaceTangency };
struct FOriginPlaneContact
{
    EOriginPlaneContactStatus Status=EOriginPlaneContactStatus::Unknown;
    Speed::Analytic::FWorldHit Hit;
    FVector3d TubeTangencyCenter=FVector3d::ZeroVector;
    int32 SamePlaneCandidates=0,OtherProvidersExcluded=0;
    // Mathematical exact face witness/MTD for the supplied radius and this
    // authenticated union only. Not a live World publication or solver ticket.
};
namespace OriginPlaneDetail
{
inline bool SignedAxis(const FVector3d& v,int32& axis,double& sign)
{
    axis=INDEX_NONE;sign=0;
    for(int32 d=0;d<3;++d)
    {
        const uint64 bits=AnalyticCertified::Bits(v[d]),mag=bits&AnalyticCertified::Magnitude;
        if(mag==0)continue;
        if(mag!=AnalyticCertified::Bits(1.)||axis!=INDEX_NONE)return false;
        axis=d;sign=(bits&AnalyticCertified::Sign)?-1.:1.;
    }
    return axis!=INDEX_NONE;
}
inline bool ShallowDepth(double height,double radius,double& depth)
{
    depth=0;
    const auto h=AnalyticCertified::Bits(height)&AnalyticCertified::Magnitude;
    if(!C::CandidateDetail::PositiveRadius(radius)||!AnalyticCertified::Finite(height)||
        (AnalyticCertified::Bits(radius)&AnalyticCertified::Magnitude)<0x0020000000000000ull||
        (h&&h<AnalyticCertified::MinNormal)||height<0.5*radius||height>radius)return false;
    volatile double difference=radius-height;
    const double value=difference;const auto bits=AnalyticCertified::Bits(value)&AnalyticCertified::Magnitude;
    // FTZ may erase an exact positive subnormal Sterbenz difference. Distinct
    // normal inputs must not become an invented zero-depth tangency.
    if((height<radius&&!(value>0))||(bits&&bits<AnalyticCertified::MinNormal))return false;
    depth=value;return true;
}
inline bool StrictInside(const TArray<FVector2d>& polygon,const FVector2d& p,C::FDistanceBudget& budget)
{
    if(polygon.Num()<3||polygon.Num()>budget.Remaining)return false;
    int32 winding=0;
    for(int32 i=0;i<polygon.Num();++i)
    {
        if(!budget.Consume())return false;
        const auto& a=polygon[i];const auto& b=polygon[(i+1)%polygon.Num()];
        const auto ax=C::Interval::Exact(a.X),ay=C::Interval::Exact(a.Y),bx=C::Interval::Exact(b.X),by=C::Interval::Exact(b.Y);
        if(!AnalyticCertified::Valid(ax)||!AnalyticCertified::Valid(ay)||!AnalyticCertified::Valid(bx)||!AnalyticCertified::Valid(by))return false;
        const auto orient=AnalyticCertified::Subtract(
            AnalyticCertified::Multiply(AnalyticCertified::Subtract(bx,ax),AnalyticCertified::Subtract(C::Interval::Exact(p.Y),ay)),
            AnalyticCertified::Multiply(AnalyticCertified::Subtract(by,ay),AnalyticCertified::Subtract(C::Interval::Exact(p.X),ax)));
        if(!AnalyticCertified::Valid(orient))return false;
        // Zero orientation may be a true finite-domain edge. Refuse when the
        // point also lies in its closed bounding box; never add a domain skin.
        if(orient.Lower<=0&&orient.Upper>=0&&p.X>=FMath::Min(a.X,b.X)&&p.X<=FMath::Max(a.X,b.X)&&
            p.Y>=FMath::Min(a.Y,b.Y)&&p.Y<=FMath::Max(a.Y,b.Y))return false;
        if(a.Y<=p.Y&&b.Y>p.Y){if(orient.Lower>0)++winding;else if(orient.Upper>=0)return false;}
        else if(a.Y>p.Y&&b.Y<=p.Y){if(orient.Upper<0)--winding;else if(orient.Lower<=0)return false;}
    }
    return winding!=0;
}
}
// Restricted exact case needed by the captured current floor. Only coordinate
// planes through the origin, signed-axis orthonormal frames, an interior face
// projection and shallow positive-side overlap/tangency are supported. Every
// other provider is excluded over the WHOLE normal correction segment. No
// curved provider is dropped, treated as a mesh, or silently granted authority.
IAMSPEED_API FOriginPlaneContact BoundOriginPlaneInitialContact(const FScenePtr& input,
    const FVector3d& center,double radius,C::FDistanceBudget& budget);

// Values-only theorem about a fixed SBox position family: physical tangency
// to this finite plane, NOT a clear Rc sweep. No impulse, force, TOI, correction,
// World hit or advance permission. Effective PVA/lifecycle/filter binding DUE.
enum class EOriginPlaneTangencyRangeStatus:uint8 {Unknown,PhysicalTangencyOverConsumedRange};
struct FOriginPlaneTangencyRange
{
    EOriginPlaneTangencyRangeStatus Status=EOriginPlaneTangencyRangeStatus::Unknown;
    FScenePtr Scene;C::FZeroOffsetKinematicCenterReference Reference;C::FReferenceCandidateSetPtr Selected;
    FOriginPlaneContact InitialPhysicalWitness;
    FVector3d Position=FVector3d::ZeroVector,Velocity=FVector3d::ZeroVector,Acceleration=FVector3d::ZeroVector;
    uint64 CollisionRadiusBits=0,PhysicalRadiusBits=0;float Horizon=0,LastSeconds=0;
};
IAMSPEED_API FOriginPlaneTangencyRange BoundOriginPlaneTangencyRange(const FScenePtr& scene,
    FVector3d position,FVector3d velocity,FVector3d acceleration,float horizon,double collisionRadius,double physicalRadius,
    float lastSeconds,C::FDistanceBudget& budget);
// Selection-aware entry points cannot reinterpret nominal plane/frame
// theorems as statements about a retained reference model.
inline FOriginPlaneContact BoundSelectedOriginPlaneInitialContact(const FPreparedGeometrySelectionPtr& geometry,
    const FVector3d& center,double radius,C::FDistanceBudget& budget)
{
    if(!geometry||!geometry->HasValidAliases()||geometry->GetModel()!=EPreparedGeometryModel::NominalInput)return {};
    return BoundOriginPlaneInitialContact(geometry->GetNominalScene(),center,radius,budget);
}
inline FOriginPlaneTangencyRange BoundSelectedOriginPlaneTangencyRange(const FPreparedGeometrySelectionPtr& geometry,
    FVector3d position,FVector3d velocity,FVector3d acceleration,float horizon,double collisionRadius,double physicalRadius,
    float lastSeconds,C::FDistanceBudget& budget)
{
    if(!geometry||!geometry->HasValidAliases()||geometry->GetModel()!=EPreparedGeometryModel::NominalInput)return {};
    return BoundOriginPlaneTangencyRange(geometry->GetNominalScene(),position,velocity,acceleration,horizon,collisionRadius,physicalRadius,lastSeconds,budget);
}
namespace OriginPlaneDetail
{
// One interior point plus NO polygon edge AABB meeting the CLOSED rectangle
// proves the connected rectangle remains interior. Conservative at diagonals.
// Four inside corners alone do NOT suffice for a concave polygon.
inline bool StrictRectangleInside(const TArray<FVector2d>& polygon,C::Interval x,C::Interval y,C::FDistanceBudget& budget)
{
    if(!AnalyticCertified::Valid(x)||!AnalyticCertified::Valid(y)||
        !StrictInside(polygon,FVector2d(x.Lower,y.Lower),budget))return false;
    for(int32 i=0;i<polygon.Num();++i)
    {
        if(!budget.Consume())return false;const auto& a=polygon[i];const auto& b=polygon[(i+1)%polygon.Num()];
        if(!(FMath::Max(a.X,b.X)<x.Lower||FMath::Min(a.X,b.X)>x.Upper||
             FMath::Max(a.Y,b.Y)<y.Lower||FMath::Min(a.Y,b.Y)>y.Upper))return false;
    }
    return true;
}
}
}
