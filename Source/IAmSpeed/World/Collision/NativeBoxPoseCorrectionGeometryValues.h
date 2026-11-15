#pragma once
#include "NativeCanonicalBoxSweepClearanceValues.h"
#include "PreparedNativeSolidGeometry.h"
#include "IAmSpeed/World/Analytic/BoxPoseCorridorBounds.h"

namespace Speed::PreparedExecution
{
class FNativeBoxPoseCorrectionGeometryValues;
using FNativeBoxPoseCorrectionGeometryValuesPtr=TSharedPtr<const FNativeBoxPoseCorrectionGeometryValues,ESPMode::ThreadSafe>;
IAMSPEED_API bool BuildNativeBoxPoseCorrectionGeometryValues(FPreparedWorldSnapshotPtr,const FNativeSolidStaticValues&,
    const FKinematicState&,const FKinematicState&,Analytic::Certified::FDistanceBudget&,FNativeBoxPoseCorrectionGeometryValuesPtr&);
IAMSPEED_API bool BuildNativeCurvedBoxPoseCorrectionGeometryValues(FPreparedWorldSnapshotPtr,const FNativeSolidStaticValues&,
    const FKinematicState&,const FKinematicState&,const FNativeCanonicalBoxSweepClearanceValuesPtr&,
    Analytic::Certified::FDistanceBudget&,FNativeBoxPoseCorrectionGeometryValuesPtr&);
IAMSPEED_API bool BuildNativeEstablishedFloorPoseCorrectionGeometryValues(FPreparedWorldSnapshotPtr,const FNativeSolidStaticValues&,
    const FKinematicState&,const FKinematicState&,FPreparedNativeSolidGeometryPtr,const Analytic::Certified::FIntervalPoint&,
    Analytic::Certified::FDistanceBudget&,FNativeBoxPoseCorrectionGeometryValuesPtr&);

inline Analytic::Certified::FBoxPoseCorridorInput MakeNativeBoxPoseCorrectionCorridorInput(
    const FNativeSolidStaticValues& source,const FKinematicState& before,const FKinematicState& candidate)
{
    Analytic::Certified::FBoxPoseCorridorInput input;
    input.BeforeCOM=before.Location;input.AfterCOM=candidate.Location;
    input.BeforeRotation=before.Rotation;input.AfterRotation=candidate.Rotation;
    input.LocalCOM=source.LocalCOM;input.LocalOffset=source.LocalOffset;input.LocalRotation=source.LocalRotation;
    input.PhysicalHalfExtent=source.PhysicalHalfExtent;input.QueryHalfExtent=source.QueryHalfExtent;
    input.DomainShell=source.Query.DomainTolerance;input.OverlapShell=source.Query.InitialOverlapTolerance;
    return input;
}

// Distinct pure geometric description of actual correction operands. It is
// neither an Integrate(dt) primitive nor a World query/event/advance ticket.
class FNativeBoxPoseCorrectionGeometryValues final
{
    FPreparedWorldSnapshotPtr Snapshot;FPreparedGeometrySelectionPtr Geometry;
    FNativeSolidStaticValues Source;FKinematicState Before,Candidate;
    Analytic::Certified::FIntervalPoint Envelope;
    Analytic::Certified::FGroupedBoxVolumeBounds GroupedVolume;
    Analytic::Certified::FReferenceBoxVolumeStrictSeparationPtr GroupedSeparation;
    FPreparedNativeSolidGeometryPtr EstablishedFloor;
    Analytic::Certified::FClosedPhysicalFloorPoseCorridorBoundsPtr FloorCorridor;
    FNativeCanonicalBoxSweepClearanceValuesPtr CurvedPrediction,CurvedActualPrediction;
    Analytic::Certified::FReferenceBoundsCandidateSetPtr CurvedPartition;
    FNativeBoxPoseCorrectionGeometryValues()=default;
    friend IAMSPEED_API bool BuildNativeBoxPoseCorrectionGeometryValues(FPreparedWorldSnapshotPtr,const FNativeSolidStaticValues&,
        const FKinematicState&,const FKinematicState&,Analytic::Certified::FDistanceBudget&,FNativeBoxPoseCorrectionGeometryValuesPtr&);
    friend IAMSPEED_API bool BuildNativeCurvedBoxPoseCorrectionGeometryValues(FPreparedWorldSnapshotPtr,const FNativeSolidStaticValues&,
        const FKinematicState&,const FKinematicState&,const FNativeCanonicalBoxSweepClearanceValuesPtr&,
        Analytic::Certified::FDistanceBudget&,FNativeBoxPoseCorrectionGeometryValuesPtr&);
    friend IAMSPEED_API bool BuildNativeEstablishedFloorPoseCorrectionGeometryValues(FPreparedWorldSnapshotPtr,const FNativeSolidStaticValues&,
        const FKinematicState&,const FKinematicState&,FPreparedNativeSolidGeometryPtr,const Analytic::Certified::FIntervalPoint&,
        Analytic::Certified::FDistanceBudget&,FNativeBoxPoseCorrectionGeometryValuesPtr&);
public:
    const auto& GetEnvelopeForInspection() const{return Envelope;}
    bool MatchesEstablishedFloorBinding(const FPreparedNativeSolidGeometryPtr& binding) const
    {return EstablishedFloor==binding&&FloorCorridor.IsValid()==binding.IsValid();}
    bool MatchesCurvedArrivalBinding(const FNativeCanonicalBoxSweepClearanceValuesPtr& binding) const
    {return !CurvedPrediction||CurvedPrediction==binding;}
    bool HasCurvedActualBinding(const FNativeCanonicalBoxSweepClearanceValuesPtr& binding) const
    {return binding&&CurvedActualPrediction==binding;}
    bool MatchesCurvedActualBinding(const FNativeCanonicalBoxSweepClearanceValuesPtr& binding) const
    {return !CurvedActualPrediction||CurvedActualPrediction==binding;}
    IAMSPEED_API bool Matches(const FPreparedWorldSnapshotPtr&,const FNativeSolidStaticValues&,
        const FKinematicState&,const FKinematicState&) const;
    IAMSPEED_API bool MatchesEndpoint(const FPreparedWorldSnapshotPtr&,const FNativeSolidStaticValues&,
        const FKinematicState&) const;
    IAMSPEED_API bool ContainsEndpoint(const Analytic::FWorldQuery&) const;
};
}
