#pragma once
#include "PreparedSphereInitialEvaluation.h"
#include "IAmSpeed/World/Analytic/ReferenceSphereConsumer.h"
namespace Speed::Analytic::Prepared
{
class FReferenceAffineContactPlan;
class FReferenceSphereC2Prefix;
class FReferenceSphereStrictSeparation;
using FReferenceSphereStrictSeparationPtr=TSharedPtr<const FReferenceSphereStrictSeparation,ESPMode::ThreadSafe>;
// Values-only strict Rs separation of the actual rounded center at Seconds.
// CapturedInput always describes the ORIGINAL P/V/A, not an advanced state.
// No closest point, exact distance, tangency, overlap, support or native grant.
class FReferenceSphereStrictSeparation final
{
    FPreparedGeometrySelectionPtr Geometry;
    FStaticSphereInitialValues CapturedInput;
    FVector3d LocalOffset=FVector3d::ZeroVector,LocalCOM=FVector3d::ZeroVector,Center=FVector3d::ZeroVector;
    C::FZeroOffsetKinematicCenterReference Model;
    C::FIntervalPoint CenterBound;
    float Horizon=0,Seconds=0;
    FReferenceSphereStrictSeparation()=default;
    friend class FReferenceSphereGeneralFirstArrivalRs;
    static bool TryProveAtImpl(FPreparedGeometrySelectionPtr geometry,FStaticSphereInitialValues values,
        FVector3d localOffset,FVector3d localCOM,float horizon,float seconds,const C::FReferenceSphereOptions& options,
        C::FDistanceBudget& budget,FReferenceSphereStrictSeparationPtr& out,FString& reason,bool signedAffine,const FReferenceAffineContactPlan* partition,
        const FReferenceSphereC2Prefix* c2=nullptr,bool* c2Closed=nullptr);
    static bool TryProveAtSignedAffine(FPreparedGeometrySelectionPtr geometry,FStaticSphereInitialValues values,
        FVector3d localOffset,FVector3d localCOM,float horizon,float seconds,const C::FReferenceSphereOptions& options,
        C::FDistanceBudget& budget,FReferenceSphereStrictSeparationPtr& out,FString& reason,
        const FReferenceAffineContactPlan* partition);
    static bool TryProveAtSignedC2(FPreparedGeometrySelectionPtr geometry,FStaticSphereInitialValues values,
        FVector3d localOffset,FVector3d localCOM,float horizon,float seconds,const C::FReferenceSphereOptions& options,
        C::FDistanceBudget& budget,FReferenceSphereStrictSeparationPtr& out,FString& reason,
        const FReferenceSphereC2Prefix& c2,bool& c2Closed);
public:
    IAMSPEED_API static bool TryProveAt(FPreparedGeometrySelectionPtr Geometry,FStaticSphereInitialValues CapturedInput,
        FVector3d ActualLocalOffset,FVector3d ActualLocalCOM,float Horizon,float Seconds,
        const C::FReferenceSphereOptions& Options,C::FDistanceBudget& Budget,
        FReferenceSphereStrictSeparationPtr& Out,FString& Reason);
    const auto& GetGeometry() const {return Geometry;}
    const auto& GetCapturedInput() const {return CapturedInput;}
    const auto& GetLocalOffset() const {return LocalOffset;}
    const auto& GetLocalCOM() const {return LocalCOM;}
    const auto& GetModel() const {return Model;}
    const auto& GetCenter() const {return Center;}
    const auto& GetCenterBound() const {return CenterBound;}
    float GetHorizon() const {return Horizon;}
    float GetSeconds() const {return Seconds;}
};
}
