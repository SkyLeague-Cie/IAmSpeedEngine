#pragma once
#include "PreparedSphereReferenceConsumer.h"
#include "SphereStaticResponsePoint.h"
namespace Speed::PreparedExecution
{
struct FSphereResponseBranchValues
{
    uint64 SurfaceId=0;bool NormalMayVary=false,StartPenetrating=false;
};
struct FSphereResponseQualityValues
{
    FVector3d Position=FVector3d::ZeroVector,Velocity=FVector3d::ZeroVector,Acceleration=FVector3d::ZeroVector;
    float Horizon=0,CollisionRadius=0,PhysicalRadius=0,ResponseMargin=0,StaticContactOffsetCm=0;
    Analytic::Certified::FReferenceSphereOptions Options;
    // Complete caller-supplied branch values, indexed by selected provider.
    // These are mathematical inputs, NOT authenticated live SHitResult bits.
    TArray<FSphereResponseBranchValues> Branches;
    TArray<uint8> AcceptedProviders; // Owned complete 0/1 policy values; no callback identity.
};
enum class ESphereResponseQualityStatus : uint8 {Unknown,ErrorBoundExceedsInputMargin,BoundedWithinInputMargin};
class FSelectedSphereResponseQuality;
using FSelectedSphereResponseQualityPtr=TSharedPtr<const FSelectedSphereResponseQuality,ESPMode::ThreadSafe>;
IAMSPEED_API ESphereResponseQualityStatus EvaluateSelectedSphereResponseQuality(
    FPreparedGeometrySelectionPtr geometry,const FSphereResponseQualityValues& values,
    Analytic::Certified::FDistanceBudget& budget,
    FSelectedSphereResponseQualityPtr& output,FString& reason);
// Immutable pure-values evidence. Neither success nor exact input identity is
// a native request, World hit, MTD, response/advance ticket or physical Ready.
class FSelectedSphereResponseQuality final
{
    FPreparedGeometrySelectionPtr Geometry;FSphereResponseQualityValues Values;
    Analytic::Certified::FReferenceSphereEvaluation Reference;
    Analytic::Certified::FScalarSphereContact ReboundedScalar;
    FVector3d ResponsePoint=FVector3d::ZeroVector;
    double PositionErrorCm=0;bool Radial=false;
    ESphereResponseQualityStatus Status=ESphereResponseQualityStatus::Unknown;
    FSelectedSphereResponseQuality()=default;
    friend IAMSPEED_API ESphereResponseQualityStatus EvaluateSelectedSphereResponseQuality(
        FPreparedGeometrySelectionPtr,const FSphereResponseQualityValues&,Analytic::Certified::FDistanceBudget&,
        FSelectedSphereResponseQualityPtr&,FString&);
public:
    const FPreparedGeometrySelectionPtr& GetGeometry() const {return Geometry;}
    const FSphereResponseQualityValues& GetValues() const {return Values;}
    const Analytic::Certified::FReferenceSphereEvaluation& GetReference() const {return Reference;}
    const Analytic::Certified::FScalarSphereContact& GetReboundedScalar() const {return ReboundedScalar;}
    const FVector3d& GetResponsePoint() const {return ResponsePoint;}
    double GetPositionErrorCm() const {return PositionErrorCm;}
    bool UsesRadialPoint() const {return Radial;}
    ESphereResponseQualityStatus GetStatus() const {return Status;}
    IAMSPEED_API bool MatchesConsumedContact(float seconds,float physicalRadius,FVector3d location,
        FVector3d impactPoint,FVector3d normalStored,FVector3d responsePoint) const;
    IAMSPEED_API bool MatchesConsumedContactAndNormal(float seconds,float physicalRadius,FVector3d location,
        FVector3d impactPoint,FVector3d normalStored,FVector3d normalConsumed,FVector3d responsePoint) const;
    IAMSPEED_API bool MatchesValues(const FPreparedGeometrySelectionPtr& geometry,const FSphereResponseQualityValues& values) const;
};
}
