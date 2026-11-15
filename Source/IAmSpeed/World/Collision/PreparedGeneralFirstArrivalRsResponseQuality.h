#pragma once
#include "PreparedSphereResponseQuality.h"
#include "PreparedSphereGeneralFirstArrivalRs.h"
namespace Speed::PreparedExecution
{
// Distinct GENERAL pure-values policy inputs. This object cannot establish that a
// native filter or Source actually permits a provider. A native consumer must
// join every bit to its separately sealed live request/physical Source plan.
struct FGeneralFirstArrivalRsResponsePolicy
{
    TArray<FSphereResponseBranchValues> Branches;
    TArray<uint8> AcceptedProviders;
};
class FGeneralFirstArrivalRsPolicyAdmission final
{
    FPreparedGeometrySelectionPtr Geometry;
    FGeneralFirstArrivalRsResponsePolicy Policy;
    uint64 NumericControl=0;
    FGeneralFirstArrivalRsPolicyAdmission()=default;
public:
    using FPtr=TSharedPtr<const FGeneralFirstArrivalRsPolicyAdmission,ESPMode::ThreadSafe>;
    FGeneralFirstArrivalRsPolicyAdmission(const FGeneralFirstArrivalRsPolicyAdmission&)=delete;
    FGeneralFirstArrivalRsPolicyAdmission& operator=(const FGeneralFirstArrivalRsPolicyAdmission&)=delete;
    // Values only. The caller supplies the existing live filter on the actual
    // query and the native service still owns the world and response gates.
    static bool TryBuild(FPreparedGeometrySelectionPtr,TFunctionRef<bool(const Analytic::Prepared::FOwner&)>,
        Analytic::Certified::FDistanceBudget&,FPtr&,FString&);
    bool Matches(const FPreparedGeometrySelectionPtr&,const FGeneralFirstArrivalRsResponsePolicy&) const;
    const auto& GetPolicy() const {return Policy;}
};
class FGeneralFirstArrivalRsResponseQuality;
using FGeneralFirstArrivalRsResponseQualityPtr=TSharedPtr<const FGeneralFirstArrivalRsResponseQuality,ESPMode::ThreadSafe>;
IAMSPEED_API ESphereResponseQualityStatus EvaluateGeneralFirstArrivalRsResponseQuality(
    Analytic::Prepared::FReferenceSphereGeneralFirstArrivalRsPtr Arrival,const FGeneralFirstArrivalRsResponsePolicy& Policy,
    Analytic::Certified::FDistanceBudget& Budget,FGeneralFirstArrivalRsResponseQualityPtr& Out,FString& Reason,
    const FGeneralFirstArrivalRsPolicyAdmission* Admission=nullptr);
// Distinct from both axial Rs and Rc first-contact types. Contact is at Rs BINARY64, coverage
// is the retained actual Rc FLOAT, and the torque arm is actual Rp FLOAT.
// Success is mathematical quality only, never a native hit/response/MTD grant.
class FGeneralFirstArrivalRsResponseQuality final
{
    Analytic::Prepared::FReferenceSphereGeneralFirstArrivalRsPtr Arrival;
    FGeneralFirstArrivalRsResponsePolicy Policy;
    Analytic::Certified::FScalarSphereContact Scalar;
    FVector3d ResponsePoint=FVector3d::ZeroVector;
    double PositionErrorCm=0;
    bool Radial=false;
    ESphereResponseQualityStatus Status=ESphereResponseQualityStatus::Unknown;
    FGeneralFirstArrivalRsResponseQuality()=default;
    friend IAMSPEED_API ESphereResponseQualityStatus EvaluateGeneralFirstArrivalRsResponseQuality(
        Analytic::Prepared::FReferenceSphereGeneralFirstArrivalRsPtr,const FGeneralFirstArrivalRsResponsePolicy&,
        Analytic::Certified::FDistanceBudget&,FGeneralFirstArrivalRsResponseQualityPtr&,FString&,
        const FGeneralFirstArrivalRsPolicyAdmission*);
public:
    const auto& GetArrival() const {return Arrival;}
    const auto& GetPolicy() const {return Policy;}
    const auto& GetReboundedScalar() const {return Scalar;}
    const auto& GetResponsePoint() const {return ResponsePoint;}
    double GetPositionErrorCm() const {return PositionErrorCm;}
    bool UsesRadialPoint() const {return Radial;}
    ESphereResponseQualityStatus GetStatus() const {return Status;}
    IAMSPEED_API bool MatchesPolicy(const Analytic::Prepared::FReferenceSphereGeneralFirstArrivalRsPtr&,
        const FGeneralFirstArrivalRsResponsePolicy&) const;
    IAMSPEED_API bool MatchesConsumedValues(float Seconds,FVector3d Location,FVector3d ImpactPoint,
        FVector3d NormalStored,FVector3d NormalConsumed,FVector3d ActualResponsePoint) const;
};
}
