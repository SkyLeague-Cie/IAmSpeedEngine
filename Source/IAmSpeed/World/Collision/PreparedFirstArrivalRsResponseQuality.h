#pragma once
#include "PreparedSphereResponseQuality.h"
#include "PreparedSphereFirstArrivalRs.h"
namespace Speed::PreparedExecution
{
// Complete pure-values policy inputs. This object cannot establish that a
// native filter or Source actually permits a provider. A native consumer must
// join every bit to its separately sealed live request/physical Source plan.
struct FFirstArrivalRsResponsePolicy
{
    TArray<FSphereResponseBranchValues> Branches;
    TArray<uint8> AcceptedProviders;
};
class FFirstArrivalRsResponseQuality;
using FFirstArrivalRsResponseQualityPtr=TSharedPtr<const FFirstArrivalRsResponseQuality,ESPMode::ThreadSafe>;
IAMSPEED_API ESphereResponseQualityStatus EvaluateFirstArrivalRsResponseQuality(
    Analytic::Prepared::FReferenceSphereFirstArrivalRsPtr Arrival,const FFirstArrivalRsResponsePolicy& Policy,
    Analytic::Certified::FDistanceBudget& Budget,FFirstArrivalRsResponseQualityPtr& Out,FString& Reason);
// Distinct from the Rc first-contact type. Contact is at Rs BINARY64, coverage
// is the retained actual Rc FLOAT, and the torque arm is actual Rp FLOAT.
// Success is mathematical quality only, never a native hit/response/MTD grant.
class FFirstArrivalRsResponseQuality final
{
    Analytic::Prepared::FReferenceSphereFirstArrivalRsPtr Arrival;
    FFirstArrivalRsResponsePolicy Policy;
    Analytic::Certified::FScalarSphereContact Scalar;
    FVector3d ResponsePoint=FVector3d::ZeroVector;
    double PositionErrorCm=0;
    bool Radial=false;
    ESphereResponseQualityStatus Status=ESphereResponseQualityStatus::Unknown;
    FFirstArrivalRsResponseQuality()=default;
    friend IAMSPEED_API ESphereResponseQualityStatus EvaluateFirstArrivalRsResponseQuality(
        Analytic::Prepared::FReferenceSphereFirstArrivalRsPtr,const FFirstArrivalRsResponsePolicy&,
        Analytic::Certified::FDistanceBudget&,FFirstArrivalRsResponseQualityPtr&,FString&);
public:
    const auto& GetArrival() const {return Arrival;}
    const auto& GetPolicy() const {return Policy;}
    const auto& GetReboundedScalar() const {return Scalar;}
    const auto& GetResponsePoint() const {return ResponsePoint;}
    double GetPositionErrorCm() const {return PositionErrorCm;}
    bool UsesRadialPoint() const {return Radial;}
    ESphereResponseQualityStatus GetStatus() const {return Status;}
    IAMSPEED_API bool MatchesPolicy(const Analytic::Prepared::FReferenceSphereFirstArrivalRsPtr&,
        const FFirstArrivalRsResponsePolicy&) const;
    IAMSPEED_API bool MatchesConsumedValues(float Seconds,FVector3d Location,FVector3d ImpactPoint,
        FVector3d NormalStored,FVector3d NormalConsumed,FVector3d ActualResponsePoint) const;
};
}
