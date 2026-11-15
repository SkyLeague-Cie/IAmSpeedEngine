#pragma once
#include "PreparedSphereStrictSeparation.h"
#include "IAmSpeed/World/Analytic/ReferenceSphereConsumer.h"
namespace Speed::PreparedExecution {struct FSelectedWholeHullPartition;struct FNativeSolidStaticValues;class FNativeRepeatedSphereFloorExclusion;}
namespace Speed::Analytic::Prepared
{
class FReferenceSphereGeneralFirstArrivalRs;
using FReferenceSphereGeneralFirstArrivalRsPtr=TSharedPtr<const FReferenceSphereGeneralFirstArrivalRs,ESPMode::ThreadSafe>;
// Values-only general zero-offset route from a freshly proved strict Rs point.
// The complete continuous/reference and rounded-prefix proofs use Rs, while
// a distinct complete rounded hull retains the real Rc. No native radius is
// replaced. No World hit, unique owner, force, phase or advance is admitted.
class FReferenceSphereGeneralFirstArrivalRs final
{
    FReferenceSphereStrictSeparationPtr Initial,ConsumedPoint;
    struct FFilteredPoint
    {
        FPreparedGeometrySelectionPtr Geometry;
        FStaticSphereInitialValues Input;
        FVector3d FreshCenter;
        int32 RepeatedProvider=INDEX_NONE;
    };
    TOptional<FFilteredPoint> Filtered;
    C::FReferenceSphereEvaluation Reference;
    C::FReferenceCandidateSetPtr RcPartition;
    float Horizon=0;
    FReferenceSphereGeneralFirstArrivalRs()=default;
    static bool TryEvaluateImpl(FReferenceSphereStrictSeparationPtr,float,const C::FReferenceSphereOptions&,
        C::FDistanceBudget&,FReferenceSphereGeneralFirstArrivalRsPtr&,FString&,C::FReferenceSphereEvaluation*,
        const PreparedExecution::FSelectedWholeHullPartition*,int32,
        const PreparedExecution::FNativeSolidStaticValues*,const C::FReferenceBoundsCandidateSet*,bool*,bool directedShortOnly=false);
public:
    // PartialReference is empty before evaluation and on success. A refusal
    // after evaluation MOVES its original raw reference here; success retains
    // the sole reference in Out->GetReference(). Neither result admits a World hit.
    IAMSPEED_API static bool TryEvaluate(FReferenceSphereStrictSeparationPtr Initial,float ActualHorizon,
        const C::FReferenceSphereOptions& Options,C::FDistanceBudget& Budget,
        FReferenceSphereGeneralFirstArrivalRsPtr& Out,FString& Reason,C::FReferenceSphereEvaluation* PartialReference=nullptr);
    IAMSPEED_API static bool TryEvaluateWithFullRc(FReferenceSphereStrictSeparationPtr Initial,float ActualHorizon,
        const C::FReferenceSphereOptions& Options,const PreparedExecution::FSelectedWholeHullPartition& FullRc,
        int32 TemporalReserve,C::FDistanceBudget& Budget,FReferenceSphereGeneralFirstArrivalRsPtr& Out,FString& Reason,
        C::FReferenceSphereEvaluation* PartialReference=nullptr,
        const PreparedExecution::FNativeSolidStaticValues* Native=nullptr,
        const C::FReferenceBoundsCandidateSet* NativeVolume=nullptr,bool* WholeRcClear=nullptr,bool DirectedShortOnly=false);
    // This alternative has a different typed origin from strict whole-union separation.
    // Only the lexical World rejection and complete fresh/rounded geometry factory can create it.
    static bool TryEvaluateNextEligible(const PreparedExecution::FNativeRepeatedSphereFloorExclusion&,
        const C::FReferenceSphereOptions&,const PreparedExecution::FSelectedWholeHullPartition&,int32,
        C::FDistanceBudget&,FReferenceSphereGeneralFirstArrivalRsPtr&,FString&);
    const auto& GetInitial() const {return Initial;}
    const FPreparedGeometrySelectionPtr& GetGeometry() const {return Filtered?Filtered->Geometry:Initial->GetGeometry();}
    const FStaticSphereInitialValues& GetCapturedInput() const {return Filtered?Filtered->Input:Initial->GetCapturedInput();}
    const FVector3d& GetFreshCenter() const {return Filtered?Filtered->FreshCenter:ConsumedPoint->GetCenter();}
    bool HasBoundInitialAndFresh() const
    {
        return Filtered?(!Initial&&!ConsumedPoint&&Filtered->Geometry&&Filtered->RepeatedProvider!=INDEX_NONE):
            (Initial&&ConsumedPoint&&ConsumedPoint->GetGeometry()==Initial->GetGeometry()&&
                C::KinematicCenterDetail::Bits(ConsumedPoint->GetSeconds())==C::KinematicCenterDetail::Bits(Reference.ConsumedSeconds));
    }
    bool IsNextEligible() const {return Filtered.IsSet();}
    bool IsRepeatedProvider(int32 provider) const {return Filtered&&Filtered->RepeatedProvider==provider;}
    const auto& GetReference() const {return Reference;}
    const auto& GetRcPartition() const {return RcPartition;}
    const auto& GetConsumedPoint() const {return ConsumedPoint;}
    float GetHorizon() const {return Horizon;}
};
}
