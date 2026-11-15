#pragma once
#include "UObject/WeakObjectPtr.h"
#include "PreparedWorldSnapshot.h"
#include "PreparedNativeTraceValues.h"
#include "IAmSpeed/World/Analytic/AnalyticWorldQuery.h"
#include "IAmSpeed/World/Analytic/ScalarSphereContactBounds.h"
#include "IAmSpeed/World/Analytic/IndexedIntervalUnion.h"
#include "CanonicalFixedSphereWheelProbeMath.h"
#include "IAmSpeed/World/Analytic/InheritedMotionReachBounds91.h"

// DRAFT / NOT INSTALLED. A fixed suspension probe is a geometric operation,
// not a fabricated native solid sweep/response request or readiness grant.
namespace Speed
{
class FPreparedCanonicalStaticQueryService;
struct SHitResult;
namespace PreparedExecution
{
// Observation only; defaults UNKNOWN, no query or caller admission.
struct FFixedSphereProbeDiagnostic89 {uint8 Site=255,Phase=255,Attempt=0;};
enum class ECanonicalFixedSphereWheelProbeResult:uint8
{Unknown,CanonicalSeparated,CanonicalContact};
class FCanonicalFixedSphereWheelProbeValues;
using FCanonicalFixedSphereWheelProbeValuesPtr=
    TSharedPtr<const FCanonicalFixedSphereWheelProbeValues,ESPMode::ThreadSafe>;

#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
class FBoundedApproxFixedWheelProbeValues;
using FBoundedApproxFixedWheelProbeValuesPtr=TSharedPtr<const FBoundedApproxFixedWheelProbeValues,ESPMode::ThreadSafe>;
class FInitCertifiedLegacyWheelProbeValues;
using FInitCertifiedLegacyWheelProbeValuesPtr=TSharedPtr<const FInitCertifiedLegacyWheelProbeValues,ESPMode::ThreadSafe>;
IAMSPEED_API bool EvaluateFixedSphereWheelProbeRoute(FPreparedWorldSnapshotPtr,
    const Analytic::FWorldQuery&,const FNativeSolidTraceValues&,double,double,uint64,
    Analytic::Certified::FDistanceBudget&,FCanonicalFixedSphereWheelProbeValuesPtr&,FString&,
    FFixedSphereProbeDiagnostic89,const Analytic::Certified::FInheritedReachInput91&,
    const TSharedPtr<const FPreparedCanonicalStaticQueryService,ESPMode::ThreadSafe>&,
    FBoundedApproxFixedWheelProbeValuesPtr&,FInitCertifiedLegacyWheelProbeValuesPtr*);
#endif

// A diagnostic-only inspection scope distinguishes a rejected coverage
// observation from a failed executed probe. It never catches an exception,
// changes a result, selects another backend or changes physical permission.
IAMSPEED_API bool HasCanonicalFixedProbeCoverageForInspection(
    const FPreparedCanonicalStaticQueryService&,const Analytic::FWorldQuery&);
IAMSPEED_API bool IsCanonicalFixedProbeCoverageObservation();

IAMSPEED_API bool EvaluateCanonicalFixedSphereWheelProbeValues(
    FPreparedWorldSnapshotPtr,const Analytic::FWorldQuery&,const FNativeSolidTraceValues&,
    double actualDelta,double actualPhysicalRadius,uint64 actualQuerySerial,
    Analytic::Certified::FDistanceBudget&,FCanonicalFixedSphereWheelProbeValuesPtr&,FString&,FFixedSphereProbeDiagnostic89 diagnostic={},
    const Analytic::Certified::FInheritedReachInput91& inheritedReach={});

class IAMSPEED_API FCanonicalFixedSphereWheelProbeValues final
{
    friend IAMSPEED_API bool EvaluateCanonicalFixedSphereWheelProbeValues(FPreparedWorldSnapshotPtr,
        const Analytic::FWorldQuery&,const FNativeSolidTraceValues&,double,double,uint64,
        Analytic::Certified::FDistanceBudget&,FCanonicalFixedSphereWheelProbeValuesPtr&,FString&,FFixedSphereProbeDiagnostic89,const Analytic::Certified::FInheritedReachInput91&);
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
    friend IAMSPEED_API bool EvaluateFixedSphereWheelProbeRoute(FPreparedWorldSnapshotPtr,
    const Analytic::FWorldQuery&,const FNativeSolidTraceValues&,double,double,uint64,
    Analytic::Certified::FDistanceBudget&,FCanonicalFixedSphereWheelProbeValuesPtr&,FString&,
    FFixedSphereProbeDiagnostic89,const Analytic::Certified::FInheritedReachInput91&,
    const TSharedPtr<const FPreparedCanonicalStaticQueryService,ESPMode::ThreadSafe>&,
    FBoundedApproxFixedWheelProbeValuesPtr&,FInitCertifiedLegacyWheelProbeValuesPtr*);
#endif
#if WITH_DEV_AUTOMATION_TESTS
    friend class FCanonicalFixedProbeBoundedSampleTest;
#endif
    FPreparedWorldSnapshotPtr Snapshot;
    Analytic::FWorldQuery Query;
    FNativeSolidTraceValues Trace;
    double Delta=0,PhysicalRadius=0;
    uint64 QuerySerial=0,NumericControl=0;
    ECanonicalFixedSphereWheelProbeResult Result=ECanonicalFixedSphereWheelProbeResult::Unknown;
    Analytic::Certified::FIntervalPoint Envelope;
    Analytic::Certified::FReferenceCandidateSetPtr Candidates;
    Analytic::Certified::FTensorSphereContactEnclosure Contact;
    TArray<Analytic::Certified::FIntervalPoint> RegularLeafChartNormals;
    Analytic::Certified::FixedWheelProbeMath::FBoundedFixedProbeSample Time;
    Analytic::Certified::FScalarSphereContact Scalar;
    Analytic::Certified::FixedWheelProbeMath::FUniqueQuantizedNormal Normal;
    AnalyticCertified::Interval CapturedProbeRadiusSlack,ProbePositionError;
    Analytic::FWorldHit Hit;
    FCanonicalFixedSphereWheelProbeValues()=default;
public:
    ECanonicalFixedSphereWheelProbeResult GetResult() const{return Result;}
    const Analytic::FWorldHit& GetHit() const{return Hit;}
    // Values-only identity. The real FNativeWheelProbeScope retains its owner,
    // phase, body, GT contact and before/after state/serial guards.
    bool Matches(const FPreparedWorldSnapshotPtr&,const Analytic::FWorldQuery&,
        const FNativeSolidTraceValues&,double actualDelta,double actualPhysicalRadius,
        uint64 actualQuerySerial) const;
    // Compare the ACTUAL post-GT-conversion and post-Quantize values. An error
    // bound in the ordinary hit is not a replacement for this local proof.
    bool MatchesConsumedContact(const SHitResult&) const;
};
}
}
