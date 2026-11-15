#pragma once
#include <atomic>
#include "PreparedNativeSolidRequest.h"
#include "NativeCanonicalBoxSweepClearanceValues.h"
#include "NativeCanonicalAxialBoxPlaneValues.h"
#include "IAmSpeed/World/Analytic/InitialFloorSphereGeometry.h"
#include "IAmSpeed/World/Analytic/NativeFloorBoxSupportBounds.h"
namespace Speed::PreparedExecution { class FNativeGeneralSphereContactValues; }
namespace Speed
{
class FPreparedCanonicalStaticQueryService;
enum class ENativeSolidGeometryStatus:uint8
{Unknown,WholeIntervalSeparated,CapturedSphereTouch,CapturedSphereOverlap,FirstArrivalBracket,BoxInitialPlaneDescriptor,BoxFloorSupportDescriptor,GeneralSphereFirstContact,CanonicalReferenceBoxSweepPathClear,CanonicalAxialBoxPlaneDescriptor,GeneralBoxFloorRefinerDescriptor,GeneralBoxFloorMotionDescriptor,GeneralBoxFloorCachedArrivalDescriptor,GeneralBoxCurvedArrivalDescriptor,GeneralBoxCurvedActualMotionClear};
// Complete retained STATIC geometry evaluation, not an event/MTD/resolve or
// advance capability. In particular an initial overlap is not a correction,
// and a Box arrival bracket supplies no positive-gap response allowance.
struct FNativeCase202AtlasObservation
{
    Analytic::Certified::FNativeGeneralBoxFloorMotionDescriptorBounds Floor;
    Analytic::Certified::FGeneralBoxPhysicalFloorPrefixBoundsPtr FloorPrefix;
    Analytic::Certified::FReferenceBoundsCandidateSetPtr AtZeroPartition;
    // Separate complete partition over the whole original H and query shells.
    // An empty value grants no positive-prefix roster evidence.
    Analytic::Certified::FReferenceBoundsCandidateSetPtr PositivePrefixPartition;
    int32 NonflatProvider=INDEX_NONE;
};
using FNativeCase202AtlasObservationPtr=TSharedPtr<const FNativeCase202AtlasObservation,ESPMode::ThreadSafe>;
// Work receipt for the original scoped Fixed FLOOR support operation.
// No geometry, hit or transport authority. Only the service may claim it.
struct FNativeFixedBoxFloorWorkReceipt
{
    const PreparedExecution::FNativeSolidStaticRequest* Request=nullptr;
    uint64 NumericFingerprint=0,Used=0;
    int32 Limit=0,Remaining=0,ConsumptionFloor=0;
};
struct FNativeSolidGeometryValues
{
    ENativeSolidGeometryStatus Status=ENativeSolidGeometryStatus::Unknown;
    Analytic::Certified::FIntervalPoint QueryEnvelope;
    Analytic::Certified::FInitialFloorSphereGeometry InitialSphere;
    Analytic::Certified::FFloorNativeSweepBounds Sweep;
    // Positive only for an exact finite downward plane with a proved arrival.
    // The current sealed native geometry request owns this coordinate.
    double NativeCeilingSphereArrivalZ=0;
    Analytic::Certified::FYawFloorBoxSweepBounds YawBox;
    AnalyticCertified::Interval YawNormSquared;
    bool YawBoxFirstArrival=false;
    Analytic::Certified::FNativeBoxFloorSupportBounds BoxSupport;
    TOptional<FNativeFixedBoxFloorWorkReceipt> FixedBoxFloorWork;
    // Geometry-only distinct route; it grants no cached support or transport.
    Analytic::Certified::FNativeGeneralBoxFloorDescriptorBounds GeneralBoxFloorDescriptor;
    Analytic::Certified::FNativeGeneralBoxFloorMotionDescriptorBounds GeneralBoxFloorMotionDescriptor;
    Analytic::Certified::FNativeGeneralCachedFloorArrivalDescriptorBounds GeneralBoxCachedFloorArrivalDescriptor;
    AnalyticCertified::Interval BoxInitialNativeGap,BoxInitialReferenceGap;
    TSharedPtr<const PreparedExecution::FNativeGeneralSphereContactValues,ESPMode::ThreadSafe> GeneralSphere;
    PreparedExecution::FNativeCanonicalBoxSweepClearanceValuesPtr CanonicalBoxClearance;
    PreparedExecution::FNativeCanonicalAxialBoxPlaneValuesPtr CanonicalAxialBoxPlane;
    uint64 SourceId=0,SurfaceId=0,FeatureId=0,PrimitiveId=0,CanonicalGroupId=0;
    uint32 MaterialId=0;
    FNativeCase202AtlasObservationPtr Case202AtlasObservation;
};
class FPreparedNativeSolidGeometry;
using FPreparedNativeSolidGeometryPtr=TSharedPtr<const FPreparedNativeSolidGeometry,ESPMode::ThreadSafe>;
class FPreparedNativeSolidGeometry final
{
    friend class FPreparedCanonicalStaticQueryService;
    PreparedExecution::FNativeSolidStaticRequestPtr Request;
    FNativeSolidGeometryValues Values;
    mutable std::atomic<bool> FixedBoxFloorCompletionClaimed{false};
    bool ClaimFixedBoxFloorCompletion() const
    {return !FixedBoxFloorCompletionClaimed.exchange(true,std::memory_order_acq_rel);}
    FPreparedNativeSolidGeometry(PreparedExecution::FNativeSolidStaticRequestPtr request,FNativeSolidGeometryValues values)
        :Request(MoveTemp(request)),Values(MoveTemp(values)){}
public:
    const PreparedExecution::FNativeSolidStaticRequestPtr& GetRequest() const {return Request;}
    const FNativeSolidGeometryValues& GetValues() const {return Values;}
};
// Inside namespace Speed; <atomic> is included by the containing header.
// This is an immutable geometry/work prelude, not a hit or transport token.
// Only the prepared service can construct it or claim its ONE continuation.
class FNativeGeneralBoxFloorObservationPrelude;
using FNativeGeneralBoxFloorObservationPreludePtr=TSharedPtr<const FNativeGeneralBoxFloorObservationPrelude,ESPMode::ThreadSafe>;
class FNativeGeneralBoxFloorObservationPrelude final
{
    friend class FPreparedCanonicalStaticQueryService;
    const PreparedExecution::FNativeSolidStaticRequestPtr Request;
    const Analytic::Certified::FNativeGeneralBoxFloorDescriptorBounds Values;
    mutable std::atomic<bool> CompletionClaimed{false};
    FNativeGeneralBoxFloorObservationPrelude(PreparedExecution::FNativeSolidStaticRequestPtr request,
        Analytic::Certified::FNativeGeneralBoxFloorDescriptorBounds values):Request(MoveTemp(request)),Values(MoveTemp(values)){}
    bool ClaimCompletion() const{return !CompletionClaimed.exchange(true,std::memory_order_acq_rel);}
public:
    const Analytic::Certified::FNativeGeneralBoxFloorDescriptorBounds& GetValuesForInspection()const{return Values;}
};
}
