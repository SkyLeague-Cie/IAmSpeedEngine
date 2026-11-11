#pragma once
#include "PreparedNativeSolidRequest.h"
#include "PreparedGeneralFirstArrivalRsHitValues.h"
#include "IAmSpeed/World/Analytic/NativeSolidSweepVolume.h"
namespace Speed::PreparedExecution
{
class FNativeGeneralSphereContactValues;
class FNativeRepeatedSphereFloorExclusion;
using FNativeGeneralSphereContactValuesPtr=TSharedPtr<const FNativeGeneralSphereContactValues,ESPMode::ThreadSafe>;
// Pure complete-union contact VALUES. Initial/tangent/penetrated, mixed filter,
// mixed all-leaf identity and no-contact results remain Unknown in this route.
IAMSPEED_API bool BuildNativeGeneralSphereContactValues(FPreparedGeometrySelectionPtr,const FNativeSolidStaticValues&,
    TFunctionRef<bool(const Analytic::Prepared::FOwner&)>,Analytic::Certified::FDistanceBudget&,FNativeGeneralSphereContactValuesPtr&,FString&,
    Analytic::Certified::FIntervalPoint* WholeRcClearEnvelope=nullptr);
#if !UE_BUILD_SHIPPING
IAMSPEED_API bool ProveNativeGeneralSphereDirectedShortMiss(FPreparedGeometrySelectionPtr,const FNativeSolidStaticValues&,
    TFunctionRef<bool(const Analytic::Prepared::FOwner&)>,Analytic::Certified::FDistanceBudget&,Analytic::Certified::FIntervalPoint&);
#endif
IAMSPEED_API bool BuildNativeNextEligibleSphereContactValues(const FNativeRepeatedSphereFloorExclusion&,
    TFunctionRef<bool(const Analytic::Prepared::FOwner&)>,Analytic::Certified::FDistanceBudget&,
    FNativeGeneralSphereContactValuesPtr&,FString&);
IAMSPEED_API bool BindNativeGeneralSphereContactValues(const FNativeSolidStaticValues&,
    const FNativeGeneralSphereContactValuesPtr&,const FPreparedSourceContactBinding&,SHitResult&);
class FNativeGeneralSphereContactValues final
{
    FPreparedGeometrySelectionPtr Geometry;FNativeSolidStaticValues Captured;
    Analytic::Certified::FIntervalPoint Envelope;
    Analytic::Certified::FReferenceBoundsCandidateSetPtr VolumePartition;
    Analytic::Certified::FReferenceCandidateSetPtr FullRs,FullRc;
    Analytic::Prepared::FReferenceSphereGeneralFirstArrivalRsPtr Arrival;
    FGeneralFirstArrivalRsHitValues Named;
    friend bool BuildNativeGeneralSphereContactValuesImpl(FPreparedGeometrySelectionPtr,const FNativeSolidStaticValues&,
        TFunctionRef<bool(const Analytic::Prepared::FOwner&)>,Analytic::Certified::FDistanceBudget&,FNativeGeneralSphereContactValuesPtr&,FString&,
        Analytic::Certified::FIntervalPoint*,const FNativeRepeatedSphereFloorExclusion*,bool);
    friend IAMSPEED_API bool BindNativeGeneralSphereContactValues(const FNativeSolidStaticValues&,
        const FNativeGeneralSphereContactValuesPtr&,const FPreparedSourceContactBinding&,SHitResult&);
    FNativeGeneralSphereContactValues()=default;
public:
    const auto& GetGeometryForInspection() const{return Geometry;}
    const auto& GetEnvelopeForInspection() const{return Envelope;}
    const SHitResult& GetNamedHitForInspection() const{return Named.Hit;}
    bool IsNextEligible() const {return Arrival&&Arrival->IsNextEligible();}
};
}
