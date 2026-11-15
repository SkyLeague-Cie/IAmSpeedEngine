#pragma once
#include "CanonicalFixedSphereWheelProbe.h"
#include "PreparedNativeAnalyticContactValues.h"
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
namespace SkyLeague::CurrentIntervalBinding::LegacyQuinticShadow {class FAtlas;}
namespace Speed::Analytic {struct FExtrudedQuinticPatch;}
namespace Speed::PreparedExecution
{
// Geometry quality and runtime response are deliberately separate contracts.
// This token owns the exact immutable patch subsequently passed to Legacy.
class IAMSPEED_API FInitCertifiedLegacyRepresentation final
{
    friend class ::SkyLeague::CurrentIntervalBinding::LegacyQuinticShadow::FAtlas;
    friend struct FInitCertifiedLegacyWheelTests;
    FPreparedWorldSnapshotPtr Snapshot;
    TSharedPtr<const Analytic::FExtrudedQuinticPatch,ESPMode::ThreadSafe> Geometry;
    uint64 NumericControl=0,RetainedBytes=0;
    int32 Provider=INDEX_NONE;
    double ErrorCm=0;
    bool MatchesOwnedGeometry(int32,const Analytic::FWorldHit&,int32)const;
    FInitCertifiedLegacyRepresentation()=default;
public:
    bool Matches(const FPreparedWorldSnapshotPtr&,int32,const Analytic::FWorldHit&,int32)const;
    double GetRepresentationErrorCm()const{return ErrorCm;}
    uint64 GetOwnedGeometryBytes()const{return RetainedBytes;}
};
using FInitCertifiedLegacyRepresentationPtr=TSharedPtr<const FInitCertifiedLegacyRepresentation,ESPMode::ThreadSafe>;
class FInitCertifiedLegacyWheelProbeValues;
using FInitCertifiedLegacyWheelProbeValuesPtr=TSharedPtr<const FInitCertifiedLegacyWheelProbeValues,ESPMode::ThreadSafe>;
IAMSPEED_API bool InitCertifiedLegacyWheelRuntimeEnabled();

// A Legacy response on init-certified geometry. It certifies NO local-original
// radial, unique normal cell, position/slack test, first TOI or closest point.
class IAMSPEED_API FInitCertifiedLegacyWheelProbeValues final
{
    friend struct FBoundedWheelValueFactory;
    FPreparedWorldSnapshotPtr Snapshot;
    TSharedPtr<const FPreparedCanonicalStaticQueryService,ESPMode::ThreadSafe> Producer;
    Analytic::Certified::FReferenceCandidateSetPtr Partition;
    FInitCertifiedLegacyRepresentationPtr Representation;
    Analytic::FWorldQuery Query;
    FNativeSolidTraceValues Trace;
    double Delta=0,PhysicalRadius=0;
    uint64 QuerySerial=0,NumericControl=0;
    int32 OriginalProvider=INDEX_NONE,OriginalFacet=INDEX_NONE;
    Analytic::FWorldHit Hit;
    SHitResult ExpectedNativeValues;
    FInitCertifiedLegacyWheelProbeValues()=default;
public:
    const Analytic::FWorldHit& GetHit()const{return Hit;}
    static constexpr bool HasCertifiedFirstContactTime(){return false;}
    static constexpr bool HasCertifiedOriginalLocalSample(){return false;}
    bool Matches(const FPreparedWorldSnapshotPtr&,const Analytic::FWorldQuery&,
        const FNativeSolidTraceValues&,double,double,uint64,const FPreparedCanonicalStaticQueryService*)const;
    bool MatchesConsumedContact(const SHitResult&)const;
};
IAMSPEED_API void RecordInitCertifiedLegacyWheelCompleted(const FPreparedWorldSnapshotPtr&,const Analytic::FWorldQuery&,
    double,double,const TSharedPtr<const void,ESPMode::ThreadSafe>&,
    const TSharedPtr<const FPreparedCanonicalStaticQueryService,ESPMode::ThreadSafe>&,
    const FInitCertifiedLegacyWheelProbeValuesPtr&);

enum class EBoundedApproxWheelAttempt:uint8
{NotApplicable,BoundedApproxSpatialHit,WorkFailure,NumericFailure};
IAMSPEED_API bool BoundedLegacyWheelRuntimeEnabled();
IAMSPEED_API int32 BoundedLegacyWheelRuntimeMode();
IAMSPEED_API bool BoundedLegacyWheelFrontendEnabled();
IAMSPEED_API EBoundedApproxWheelAttempt EvaluateBoundedLegacyWheelFrontend(FPreparedWorldSnapshotPtr,
    const Analytic::FWorldQuery&,const FNativeSolidTraceValues&,double,double,uint64,
    Analytic::Certified::FDistanceBudget&,
    const TSharedPtr<const FPreparedCanonicalStaticQueryService,ESPMode::ThreadSafe>&,
    FBoundedApproxFixedWheelProbeValuesPtr&,FString&);
// Unsealed optional-service output. Only the guarded route below can close a
// physical probe value. A proposal itself is neither a hit nor an authority.
struct FBoundedApproxWheelProposal
{
    Analytic::FWorldHit Hit;
    FInitCertifiedLegacyRepresentationPtr InitRepresentation;
    Analytic::Certified::FReferenceCandidateSetPtr CompleteInflatedPartition;
    Analytic::Certified::FReferenceCandidateInventoryPtr CompleteInflatedInventory;
    int32 OriginalProvider=INDEX_NONE,OriginalFacet=INDEX_NONE;
    FVector2d ProposedOriginalUV=FVector2d::ZeroVector;
    bool Plane=false;
    int32 PlaneNormalAxis=INDEX_NONE;
    double PlaneHeight=0,RepresentationErrorCm=0;
    int32 ProductFreeWorldAxis=INDEX_NONE;
    bool ProductFreeAlongU=true;
};
// Called only by the real wheel scope; its exact counterpart remains intact.
IAMSPEED_API bool EvaluateFixedSphereWheelProbeRoute(FPreparedWorldSnapshotPtr,
    const Analytic::FWorldQuery&,const FNativeSolidTraceValues&,double,double,uint64,
    Analytic::Certified::FDistanceBudget&,FCanonicalFixedSphereWheelProbeValuesPtr&,FString&,
    FFixedSphereProbeDiagnostic89,const Analytic::Certified::FInheritedReachInput91&,
    const TSharedPtr<const FPreparedCanonicalStaticQueryService,ESPMode::ThreadSafe>&,
    FBoundedApproxFixedWheelProbeValuesPtr&,FInitCertifiedLegacyWheelProbeValuesPtr* = nullptr);
struct FBoundedWheelValueFactory;
class IAMSPEED_API FBoundedApproxFixedWheelProbeValues final
{
    friend struct FBoundedWheelValueFactory;
    friend IAMSPEED_API bool EvaluateFixedSphereWheelProbeRoute(FPreparedWorldSnapshotPtr,
        const Analytic::FWorldQuery&,const FNativeSolidTraceValues&,double,double,uint64,
        Analytic::Certified::FDistanceBudget&,FCanonicalFixedSphereWheelProbeValuesPtr&,FString&,
        FFixedSphereProbeDiagnostic89,const Analytic::Certified::FInheritedReachInput91&,
        const TSharedPtr<const FPreparedCanonicalStaticQueryService,ESPMode::ThreadSafe>&,
        FBoundedApproxFixedWheelProbeValuesPtr&,FInitCertifiedLegacyWheelProbeValuesPtr*);
    FPreparedWorldSnapshotPtr Snapshot;
    TSharedPtr<const FPreparedCanonicalStaticQueryService,ESPMode::ThreadSafe> Producer;
    Analytic::Certified::FReferenceCandidateSetPtr Partition;
    Analytic::Certified::FReferenceCandidateInventoryPtr Inventory;
    Analytic::FWorldQuery Query;
    FNativeSolidTraceValues Trace;
    double Delta=0,PhysicalRadius=0,RepresentationErrorCm=0;
    uint64 QuerySerial=0,NumericControl=0;
    int32 OriginalProvider=INDEX_NONE,OriginalFacet=INDEX_NONE;
    int32 ApproximationMode=0;
    // Private local-normal proposal policy, never an exact-contact claim.
    int32 NormalPolicy=0;
    FVector2d OriginalSampleUV=FVector2d::ZeroVector;
    AnalyticCertified::Interval LocalPositionError;
    Analytic::FWorldHit Hit;
    SHitResult ExpectedNativeValues;
    FBoundedApproxFixedWheelProbeValues()=default;
public:
    const Analytic::FWorldHit& GetHit()const{return Hit;}
    static constexpr bool HasCertifiedFirstContactTime(){return false;}
    static constexpr bool HasCertifiedGlobalClosestPoint(){return false;}
    bool Matches(const FPreparedWorldSnapshotPtr&,const Analytic::FWorldQuery&,
        const FNativeSolidTraceValues&,double,double,uint64,const FPreparedCanonicalStaticQueryService*)const;
    bool MatchesConsumedContact(const SHitResult&)const;
};
}
#endif
