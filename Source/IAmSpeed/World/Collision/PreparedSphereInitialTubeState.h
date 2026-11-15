#pragma once
#include "PreparedSphereInitialEvaluation.h"
namespace Speed::Analytic::Prepared
{
enum class EReferenceSphereRsState:uint8 {SeparatedRs,TangencyRs,OverlapRs};
class FReferenceSphereInitialTubeState;
using FReferenceSphereInitialTubeStatePtr=TSharedPtr<const FReferenceSphereInitialTubeState,ESPMode::ThreadSafe>;
// Fresh mathematical POINT observation inside the captured Rc tube. A closest
// witness whose radius is the actual height h is deliberately NOT a tangency
// at Rs. The explicit state below compares that exact h to the retained Rs.
// No support interval, contact arrival, FirstContact/Rc-clear prefix, native
// phase, physical policy/force authority, mutation or World readiness follows.
class FReferenceSphereInitialTubeState final
{
    FStaticSphereInitialValues Values;
    FReferencePlaneInitialContactPtr ClosestAtHeight;
    EReferenceSphereRsState Kind=EReferenceSphereRsState::SeparatedRs;
    int32 Axis=INDEX_NONE;double Height=0,Sign=0;
    FReferenceSphereInitialTubeState()=default;
public:
    IAMSPEED_API static bool TryObserve(FPreparedGeometrySelectionPtr Geometry,FStaticSphereInitialValues Values,
        C::FDistanceBudget& Budget,FReferenceSphereInitialTubeStatePtr& Out,FString& Reason);
    const auto& GetInput() const {return Values;}
    const auto& GetClosestAtHeight() const {return ClosestAtHeight;}
    EReferenceSphereRsState GetKind() const {return Kind;}
    double GetDistance() const {return Height;}
    // Instantaneous algebra ONLY: exact planar tangency at Rs, stationary
    // center, finite normal/zero acceleration. Uses center acceleration,
    // never the acceleration of a rotating material point C-Rp*n. A fresh
    // whole-segment/domain/phase/load binding is still required before any
    // native support force or integration; this does not supply one.
    IAMSPEED_API bool TryPlanarStationaryNormalLoad(FVector3d& ReactionAcceleration,FVector3d& EffectiveAcceleration) const;
};
}
