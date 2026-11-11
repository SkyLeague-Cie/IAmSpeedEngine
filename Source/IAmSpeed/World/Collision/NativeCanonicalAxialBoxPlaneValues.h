#pragma once
#include "PreparedNativeSolidRequest.h"
#include "IAmSpeed/Base/SHitResult.h"
#include "NativeBoxPlanarRefinementValues.h"
#include "IAmSpeed/World/Analytic/CanonicalAxialPlaneBounds.h"
#include "IAmSpeed/World/Analytic/FixedAxialBoxFirstArrival.h"
#include "IAmSpeed/World/Analytic/NativeSolidSweepVolume.h"
namespace Speed::PreparedExecution
{
class FNativeCanonicalAxialBoxPlaneValues;
using FNativeCanonicalAxialBoxPlaneValuesPtr=TSharedPtr<const FNativeCanonicalAxialBoxPlaneValues,ESPMode::ThreadSafe>;
IAMSPEED_API bool BuildNativeCanonicalAxialBoxPlaneValues(FPreparedGeometrySelectionPtr,Analytic::Prepared::FScenePtr,
    const FNativeSolidStaticValues&,Analytic::Certified::FDistanceBudget&,FNativeCanonicalAxialBoxPlaneValuesPtr&);
IAMSPEED_API bool MatchesNativeCanonicalAxialBoxPlaneValues(const FNativeSolidStaticValues&,const FPreparedGeometrySelectionPtr&,
    const Analytic::Prepared::FScenePtr&,const FNativeCanonicalAxialBoxPlaneValuesPtr&,
    const Analytic::Certified::FIntervalPoint&);
IAMSPEED_API bool MatchNativeCanonicalAxialBoxRefinement(const FNativeSolidStaticValues&,
    const FNativeCanonicalAxialBoxPlaneValuesPtr&,const SHitResult&,const FNativeBoxPlanarRefinementValues&);
// Only a complete STATIC canonical prediction/finite-plane descriptor. The
// caller still invokes the REAL native refiner and consumes its own real ray.
// No current cache, initial overlap, dynamics, response or advance follows.
class FNativeCanonicalAxialBoxPlaneValues final
{
    FPreparedGeometrySelectionPtr Geometry;Analytic::Prepared::FScenePtr Nominal;FNativeSolidStaticValues Captured;
    Analytic::Certified::FCanonicalAxialPlaneBounds Finite;
    Analytic::Certified::FFloorNativeSweepBounds Arrival;SHitResult Descriptor;
    FNativeCanonicalAxialBoxPlaneValues()=default;
    friend IAMSPEED_API bool BuildNativeCanonicalAxialBoxPlaneValues(FPreparedGeometrySelectionPtr,Analytic::Prepared::FScenePtr,
        const FNativeSolidStaticValues&,Analytic::Certified::FDistanceBudget&,FNativeCanonicalAxialBoxPlaneValuesPtr&);
    friend IAMSPEED_API bool MatchesNativeCanonicalAxialBoxPlaneValues(const FNativeSolidStaticValues&,const FPreparedGeometrySelectionPtr&,
        const Analytic::Prepared::FScenePtr&,const FNativeCanonicalAxialBoxPlaneValuesPtr&,const Analytic::Certified::FIntervalPoint&);
public:
    const auto& GetFiniteForInspection() const{return Finite;}
    const auto& GetArrivalForInspection() const{return Arrival;}
    const auto& GetDescriptorForInspection() const{return Descriptor;}
};
}
