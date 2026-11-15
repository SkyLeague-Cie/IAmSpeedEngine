#pragma once
#include "PreparedNativeSolidRequest.h"
#include "IAmSpeed/World/Analytic/DirectionalMotionStrictSeparation.h"
#include "IAmSpeed/World/Analytic/ReferenceBoundsStrictSeparation.h"
#include "IAmSpeed/World/Analytic/NativeGroupedBoxVolumeBounds.h"
#include "IAmSpeed/World/Analytic/NativeFloorBoxSupportBounds.h"
namespace Speed {class FPreparedNativeSolidGeometry;struct FNativeCase202AtlasObservation;}
namespace Speed::PreparedExecution
{
class FNativeBoxPrimitiveGeometryValues;
using FNativeBoxPrimitiveGeometryValuesPtr=TSharedPtr<const FNativeBoxPrimitiveGeometryValues,ESPMode::ThreadSafe>;
class FNativeCanonicalBoxSweepClearanceValues;
using FNativeCanonicalBoxSweepClearanceValuesPtr=TSharedPtr<const FNativeCanonicalBoxSweepClearanceValues,ESPMode::ThreadSafe>;
IAMSPEED_API bool BuildNativeCanonicalBoxSweepClearanceValues(FPreparedGeometrySelectionPtr,const FNativeSolidStaticValues&,
    Analytic::Certified::FDistanceBudget&,FNativeCanonicalBoxSweepClearanceValuesPtr&,FString&);
// Explicit optional allocation and immutable external-geometry guard. The
// five-argument ABI remains the unmodified fine-only entry point.
IAMSPEED_API bool BuildNativeCanonicalBoxSweepClearanceValues(FPreparedGeometrySelectionPtr,const FNativeSolidStaticValues&,
    Analytic::Certified::FDistanceBudget&,FNativeCanonicalBoxSweepClearanceValuesPtr&,FString&,
    FPreparedWorldSnapshotPtr,int32 coarseWorkAllowance);
// The live request is consumed only by the scoped OFF-by-default 976
// remainder experiment and is revalidated before and after its proof.
IAMSPEED_API bool BuildNativeCanonicalBoxSweepClearanceValues(FPreparedGeometrySelectionPtr,const FNativeSolidStaticValues&,
    Analytic::Certified::FDistanceBudget&,FNativeCanonicalBoxSweepClearanceValuesPtr&,FString&,
    FPreparedWorldSnapshotPtr,int32 coarseWorkAllowance,const FNativeSolidStaticRequest* liveRemainderRequest);
class FNativeCurvedBoxArrivalValues;
using FNativeCurvedBoxArrivalValuesPtr=TSharedPtr<const FNativeCurvedBoxArrivalValues,ESPMode::ThreadSafe>;
class FNativeCurvedBoxWholeClearValues;
using FNativeCurvedBoxWholeClearValuesPtr=TSharedPtr<const FNativeCurvedBoxWholeClearValues,ESPMode::ThreadSafe>;
IAMSPEED_API bool ProveCanonicalTargetedStationaryCurvedBoxClear(const FPreparedWorldSnapshotPtr&,
    const Analytic::FWorldQuery&,const Analytic::Certified::FIntervalPoint&,Analytic::Certified::FDistanceBudget&);
// Lexical handoff from the actual owner projection gate. The arrival only
// selects a graph theorem for this query. It cannot supply a stationary result.
class FNativeCurvedBoxPhysicalProjectionContext final
{
    friend class FStepScope;
    const Analytic::FWorldQuery* Query;
    const FNativeCurvedBoxArrivalValues* Arrival;
    FNativeCurvedBoxPhysicalProjectionContext(const Analytic::FWorldQuery& query,const FNativeCurvedBoxArrivalValues* arrival)
        :Query(&query),Arrival(arrival){}
public:
    FNativeCurvedBoxPhysicalProjectionContext(const FNativeCurvedBoxPhysicalProjectionContext&)=delete;
    FNativeCurvedBoxPhysicalProjectionContext& operator=(const FNativeCurvedBoxPhysicalProjectionContext&)=delete;
    const FNativeCurvedBoxArrivalValues* ArrivalFor(const Analytic::FWorldQuery& query) const
    {return Query==&query?Arrival:nullptr;}
};
// Geometry only. The only caller is the authenticated native Box projection.
IAMSPEED_API bool ProveCanonicalTargetedCurvedBoxPhysicalProjectionClear(const FPreparedWorldSnapshotPtr&,
    const Analytic::FWorldQuery&,const Analytic::Certified::FIntervalPoint&,Analytic::Certified::FDistanceBudget&,
    const FNativeCurvedBoxPhysicalProjectionContext* context=nullptr);
IAMSPEED_API bool ReadNativeCurvedBoxArrivalValues(const FPreparedWorldSnapshotPtr&,const FNativeSolidStaticValues&,
    const FNativeCanonicalBoxSweepClearanceValuesPtr&,float&,Analytic::FWorldHit&);
IAMSPEED_API bool BuildCase202Serial3CurveArrival(const FNativeSolidStaticRequest&,
    const TSharedPtr<const FNativeCase202AtlasObservation,ESPMode::ThreadSafe>&,
    FNativeCanonicalBoxSweepClearanceValuesPtr&);
IAMSPEED_API bool BuildNativeEstablishedFloorCurveArrival(const FPreparedWorldSnapshotPtr&,const FNativeSolidStaticValues&,
    const Analytic::Certified::FNativeGeneralBoxFloorDescriptorBounds&,Analytic::Certified::FDistanceBudget&,
    FNativeCanonicalBoxSweepClearanceValuesPtr&);
IAMSPEED_API bool MatchesNativeCanonicalBoxSweepClearanceValues(const FNativeSolidStaticValues&,const FPreparedGeometrySelectionPtr&,
    const FNativeCanonicalBoxSweepClearanceValuesPtr&,const Analytic::Certified::FIntervalPoint&);
IAMSPEED_API bool MatchesNativeCurvedBoxActualMotionClearValues(const FPreparedWorldSnapshotPtr&,const FNativeSolidStaticValues&,
    const FNativeCanonicalBoxSweepClearanceValuesPtr&,const Analytic::Certified::FIntervalPoint&);
IAMSPEED_API bool TryBuildNativeBoxPrimitiveFromPrediction(FPreparedWorldSnapshotPtr,const FNativeSolidStaticValues&,
    const FKinematicState&,float,const FNativeCanonicalBoxSweepClearanceValuesPtr&,
    Analytic::Certified::FDistanceBudget&,FNativeBoxPrimitiveGeometryValuesPtr&);
// DISTINCT canonical-reference operation. It closes only the captured Box
// SWEEP PREDICTION path against the exact retained master. No legacy native
// facet query was executed/proved, no cache/+0, constrained trajectory,
// dynamic pair, contact, response, physical step or Ready follows.
class FNativeCanonicalBoxSweepClearanceValues final
{
    FPreparedGeometrySelectionPtr Geometry;FNativeSolidStaticValues Captured;
    Analytic::Certified::FIntervalPoint Envelope;
    Analytic::Certified::FReferenceBoundsStrictSeparationPtr Separation;
    Analytic::Certified::FGroupedBoxVolumeBounds GroupedVolume;
    Analytic::Certified::FReferenceBoxVolumeStrictSeparationPtr GroupedSeparation;
    Analytic::Certified::FReferenceDirectionalMotionStrictSeparationPtr DirectionalSeparation;
    FNativeCurvedBoxArrivalValuesPtr CurvedArrival;
    FNativeCurvedBoxWholeClearValuesPtr CurvedWholeClear;
    bool GeneralTransportRequired=false,CurvedActualMotionOnly=false,Case202Serial3=false;uint64 NumericFingerprint=0;
    friend IAMSPEED_API bool ReadNativeCurvedBoxArrivalValues(const FPreparedWorldSnapshotPtr&,const FNativeSolidStaticValues&,
        const FNativeCanonicalBoxSweepClearanceValuesPtr&,float&,Analytic::FWorldHit&);
    friend IAMSPEED_API bool BuildNativeEstablishedFloorCurveArrival(const FPreparedWorldSnapshotPtr&,const FNativeSolidStaticValues&,
        const Analytic::Certified::FNativeGeneralBoxFloorDescriptorBounds&,Analytic::Certified::FDistanceBudget&,
        FNativeCanonicalBoxSweepClearanceValuesPtr&);
    friend IAMSPEED_API bool BuildCase202Serial3CurveArrival(const FNativeSolidStaticRequest&,
    const TSharedPtr<const FNativeCase202AtlasObservation,ESPMode::ThreadSafe>&,
    FNativeCanonicalBoxSweepClearanceValuesPtr&);
    FNativeCanonicalBoxSweepClearanceValues()=default;
    friend IAMSPEED_API bool TryBuildNativeBoxPrimitiveFromPrediction(FPreparedWorldSnapshotPtr,const FNativeSolidStaticValues&,
    const FKinematicState&,float,const FNativeCanonicalBoxSweepClearanceValuesPtr&,
    Analytic::Certified::FDistanceBudget&,FNativeBoxPrimitiveGeometryValuesPtr&);
    friend IAMSPEED_API bool BuildNativeCanonicalBoxSweepClearanceValues(FPreparedGeometrySelectionPtr,const FNativeSolidStaticValues&,
        Analytic::Certified::FDistanceBudget&,FNativeCanonicalBoxSweepClearanceValuesPtr&,FString&);
    friend IAMSPEED_API bool BuildNativeCanonicalBoxSweepClearanceValues(FPreparedGeometrySelectionPtr,const FNativeSolidStaticValues&,
        Analytic::Certified::FDistanceBudget&,FNativeCanonicalBoxSweepClearanceValuesPtr&,FString&,FPreparedWorldSnapshotPtr,int32);
    friend IAMSPEED_API bool BuildNativeCanonicalBoxSweepClearanceValues(FPreparedGeometrySelectionPtr,const FNativeSolidStaticValues&,
        Analytic::Certified::FDistanceBudget&,FNativeCanonicalBoxSweepClearanceValuesPtr&,FString&,FPreparedWorldSnapshotPtr,int32,
        const FNativeSolidStaticRequest*);
    friend IAMSPEED_API bool MatchesNativeCanonicalBoxSweepClearanceValues(const FNativeSolidStaticValues&,const FPreparedGeometrySelectionPtr&,
        const FNativeCanonicalBoxSweepClearanceValuesPtr&,const Analytic::Certified::FIntervalPoint&);
    friend IAMSPEED_API bool MatchesNativeCurvedBoxActualMotionClearValues(const FPreparedWorldSnapshotPtr&,const FNativeSolidStaticValues&,
        const FNativeCanonicalBoxSweepClearanceValuesPtr&,const Analytic::Certified::FIntervalPoint&);
public:
    bool RequiresGeneralBoxTransport() const{return GeneralTransportRequired;}
    bool IsCurvedArrivalPrefix() const{return CurvedArrival.IsValid();}
    bool IsCase202Serial3Prefix() const{return Case202Serial3&&IsCurvedArrivalPrefix();}
    bool IsCurvedWholeClear() const{return CurvedWholeClear.IsValid()&&!CurvedActualMotionOnly;}
    bool IsCurvedActualMotionClear() const{return CurvedWholeClear.IsValid()&&CurvedActualMotionOnly;}
    const FNativeCurvedBoxArrivalValuesPtr& GetCurvedArrivalForInspection() const{return CurvedArrival;}
    const FNativeCurvedBoxWholeClearValuesPtr& GetCurvedWholeClearForInspection() const{return CurvedWholeClear;}
    const auto& GetGeometryForInspection() const{return Geometry;}
    const auto& GetEnvelopeForInspection() const{return Envelope;}
    const auto& GetSeparationForInspection() const{return Separation;}
};
enum class ENativeBoxPrimitiveGeometryProvenance:uint8 {WholeWorldStrict,Released59,Motion60Whole,Motion60First,Cached65First,CurvedArrivalPrefix,CurvedGraphWholeClear,CurvedGraphActualMotionClear};

IAMSPEED_API bool BuildNativeBoxPrimitiveGeometryValues(FPreparedWorldSnapshotPtr,const FNativeSolidStaticValues&,
    const FKinematicState&,float,Analytic::Certified::FDistanceBudget&,FNativeBoxPrimitiveGeometryValuesPtr&,int32 coarseWorkAllowance=0);
IAMSPEED_API bool BuildNativeReleasedBoxFloorPrimitiveGeometryValues(FPreparedWorldSnapshotPtr,const FNativeSolidStaticValues&,
    const FKinematicState&,float,const TSharedPtr<const FPreparedNativeSolidGeometry,ESPMode::ThreadSafe>&,
    Analytic::Certified::FDistanceBudget&,FNativeBoxPrimitiveGeometryValuesPtr&);
IAMSPEED_API bool BuildNativeGeneralBoxFloorMotionPrimitiveGeometryValues(FPreparedWorldSnapshotPtr,const FNativeSolidStaticValues&,
    const FKinematicState&,float,float,const TSharedPtr<const FPreparedNativeSolidGeometry,ESPMode::ThreadSafe>&,
    Analytic::Certified::FDistanceBudget&,FNativeBoxPrimitiveGeometryValuesPtr&);
// Facts about one actual restarted primitive, independent of the original
// World-time arrival. The two reference roles retain separate endpoints.
struct FNativeBoxRebasedRoof202
{
    uint32 NativeBulkBits=0,RawClearBits=0,RawInsideBits=0,NormalizedClearBits=0,NativeClearBits=0;
    bool Matches(uint32 localBits) const
    {
        return NativeBulkBits>0&&NativeBulkBits<RawClearBits&&RawInsideBits==RawClearBits+1&&
            RawInsideBits<NormalizedClearBits&&NormalizedClearBits==localBits&&NativeClearBits==localBits;
    }
};
// Immutable pure geometry, never a World request, advance token or readiness.
class FNativeBoxPrimitiveGeometryValues final
{
    FPreparedWorldSnapshotPtr Snapshot;FPreparedGeometrySelectionPtr Geometry;FNativeSolidStaticValues Source;
    FKinematicState Actual;float Seconds=0;Analytic::Certified::FIntervalPoint Envelope;
    Analytic::Certified::FGroupedBoxVolumeBounds GroupedVolume;
    Analytic::Certified::FReferenceBoxVolumeStrictSeparationPtr GroupedSeparation;
    Analytic::Certified::FReferenceDirectionalMotionStrictSeparationPtr DirectionalSeparation;
    TSharedPtr<const FPreparedNativeSolidGeometry,ESPMode::ThreadSafe> ReleasedDescriptor,MotionDescriptor;
    FNativeCurvedBoxArrivalValuesPtr CurvedArrival;
    FNativeCurvedBoxWholeClearValuesPtr CurvedWholeClear;
    float MotionWorldSeconds=0;ENativeBoxPrimitiveGeometryProvenance Provenance=ENativeBoxPrimitiveGeometryProvenance::WholeWorldStrict;
    TSharedPtr<const Analytic::Certified::FGeneralBoxPhysicalFloorClearBounds,ESPMode::ThreadSafe> PhysicalFloorClear;
    Analytic::Certified::FGeneralBoxPhysicalFloorPrefixBoundsPtr PhysicalFloorPrefix;
    TOptional<FNativeBoxRebasedRoof202> RebasedRoof202;
    FNativeBoxPrimitiveGeometryValues()=default;
    friend IAMSPEED_API bool TryBuildNativeBoxPrimitiveFromPrediction(FPreparedWorldSnapshotPtr,const FNativeSolidStaticValues&,
    const FKinematicState&,float,const FNativeCanonicalBoxSweepClearanceValuesPtr&,
    Analytic::Certified::FDistanceBudget&,FNativeBoxPrimitiveGeometryValuesPtr&);
    friend IAMSPEED_API bool BuildNativeGeneralBoxFloorMotionPrimitiveGeometryValues(FPreparedWorldSnapshotPtr,const FNativeSolidStaticValues&,
    const FKinematicState&,float,float,const TSharedPtr<const FPreparedNativeSolidGeometry,ESPMode::ThreadSafe>&,
    Analytic::Certified::FDistanceBudget&,FNativeBoxPrimitiveGeometryValuesPtr&);
    friend IAMSPEED_API bool BuildNativeReleasedBoxFloorPrimitiveGeometryValues(FPreparedWorldSnapshotPtr,const FNativeSolidStaticValues&,
        const FKinematicState&,float,const TSharedPtr<const FPreparedNativeSolidGeometry,ESPMode::ThreadSafe>&,
        Analytic::Certified::FDistanceBudget&,FNativeBoxPrimitiveGeometryValuesPtr&);
    friend IAMSPEED_API bool BuildNativeBoxPrimitiveGeometryValues(FPreparedWorldSnapshotPtr,const FNativeSolidStaticValues&,
        const FKinematicState&,float,Analytic::Certified::FDistanceBudget&,FNativeBoxPrimitiveGeometryValuesPtr&,int32);
public:
    const auto& GetEnvelopeForInspection() const{return Envelope;}
    IAMSPEED_API bool Matches(const FPreparedWorldSnapshotPtr&,const FNativeSolidStaticValues&,const FKinematicState&,float) const;
    IAMSPEED_API bool ContainsEndpoint(const Analytic::FWorldQuery&) const;
    IAMSPEED_API bool MatchesFloorMotion(const FPreparedWorldSnapshotPtr&,const FNativeSolidStaticValues&,const FKinematicState&,
        float,float,const TSharedPtr<const FPreparedNativeSolidGeometry,ESPMode::ThreadSafe>&) const;
    IAMSPEED_API bool ContainsFloorMotionEndpoint(const Analytic::FWorldQuery&,float,
        const TSharedPtr<const FPreparedNativeSolidGeometry,ESPMode::ThreadSafe>&) const;
};
}
