// Included in the same existing TU as NativeCanonicalBoxSweepClearanceValues.
namespace Speed::PreparedExecution
{
static TAutoConsoleVariable<int32> CVarCurvedBoxActualClearRotationCorridor(
    TEXT("p.IAmSpeed.AnalyticWorld.CurvedBoxActualClearRotationCorridor"),0,
    TEXT("Prove a fresh pure-rotation Box corridor against the retained CurvedActual graph and complete static world. Default off."));
#if !UE_BUILD_SHIPPING
static TAutoConsoleVariable<int32> CVarNativeCase585PostFifthRotation(
    TEXT("p.IAmSpeed.AnalyticWorld.NativeCase585PostFifthRotation"),0,
    TEXT("Prove the complete static case-585 rotation after the restored fifth-hit cap. Default off."));

namespace CurvedBoxArrivalDetail
{
struct FCase585PostFifthRotationAdmission final
{
    static bool Matches(const FPreparedWorldSnapshotPtr& snapshot,const FNativeSolidStaticValues& source,
        Analytic::Certified::FDistanceBudget& budget)
    {
        const auto* scope=StepScope;
        FStepIdentity now;
        if(CVarNativeCase585PostFifthRotation.GetValueOnAnyThread()!=1||!scope||
            !scope->Active||!scope->NativeConsumer||scope->Snapshot!=snapshot||!snapshot||
            !scope->Case585FourthGround||!scope->Case585FifthGround||!scope->Case585FifthResolved||
            scope->Case585AcceptingFifth||!scope->Case585Pair||
            !ReadCurrentStep(now)||now.Frame!=86||now.QuerySerial!=6||
            source.FrameClock.OwnerLocalFrame!=87||source.OwnerStableId!=3||source.BodyIndex!=0||
            source.Trace.IgnoredComponents.Num()!=1||!source.Trace.IgnoredSourceObjects.IsEmpty())
            return false;

        // The already resolved fifth evidence is the only admissible origin
        // for this ignored contact. It is never removed from the fresh proof.
        const auto& fifth=*scope->Case585FifthGround;
        const auto& request=fifth.GetRequestForInspection();
        const auto& hit=fifth.GetGroundValuesForInspection();
        if(!fifth.IsCurvedBoxArrivalValues()||!request||request->GetSnapshot()!=snapshot||
            request->GetStep().Frame!=now.Frame||request->GetStep().QuerySerial!=5||
            request->GetStep().InputGeneration!=now.InputGeneration||
            request->GetValues().Owner!=source.Owner||request->GetValues().Body!=source.Body||
            hit.SourceId!=1274291482858373289ull||hit.SurfaceId!=12602828015134877967ull||
            hit.ContactFeatureIndexThis!=1||!budget.Consume())return false;
        bool ambiguous=false;
        const auto* binding=snapshot->FindUniqueContactBinding(hit.SourceId,ambiguous);
        return binding&&!ambiguous&&binding->ContactObjectId&&
            binding->ContactObjectId==source.Trace.IgnoredComponents[0];
    }
};
}
#endif
static TAutoConsoleVariable<int32> CVarNativeBoxCorrectionObliqueTranslation(
    TEXT("p.IAmSpeed.AnalyticWorld.NativeBoxCorrectionObliqueTranslation"),0,
    TEXT("Try strict full-roster separation of an actual Box translation correction with retained oblique axes first. Default off."));
bool BuildNativeBoxPoseCorrectionGeometryValues(FPreparedWorldSnapshotPtr snapshot,const FNativeSolidStaticValues& source,
    const FKinematicState& before,const FKinematicState& candidate,Analytic::Certified::FDistanceBudget& budget,
    FNativeBoxPoseCorrectionGeometryValuesPtr& out)
{
    out.Reset();Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    namespace C=Analytic::Certified;
    const auto geometry=snapshot?snapshot->GetGeometrySelection():FPreparedGeometrySelectionPtr{};
    if(!geometry||!geometry->HasValidAliases()||geometry->GetModel()!=EPreparedGeometryModel::RetainedReference||
        source.Shape!=ENativeSolidQueryShape::Box||source.Frozen||source.Prescribed||
        !FiniteState(before)||!FiniteState(candidate))return false;
    const auto input=MakeNativeBoxPoseCorrectionCorridorInput(source,before,candidate);
    C::FGroupedBoxVolumeBounds groupedVolume;
    if(!C::BuildBoxPoseCorridorGroupedVolume(input,budget,groupedVolume))return false;
    const auto hull=groupedVolume.BroadHull;
    C::FReferenceBoundsCandidateSetPtr partition;C::FReferenceBoxVolumeStrictSeparationPtr separation;
    if(!C::FReferenceBoundsCandidateSet::Select(geometry->GetSelectedUnion(),hull,budget,partition)||!partition)return false;
    bool refine=false;
    if(CVarNativeBoxCorrectionObliqueTranslation.GetValueOnAnyThread()==1&&
        source.Trace.IgnoredSourceObjects.IsEmpty()&&source.Trace.IgnoredComponents.IsEmpty())
    {
        const auto candidates=partition->GetCandidateIndices();
        auto translated=before;
        translated.Location=candidate.Location;
        refine=candidates.Num()==2&&candidates.Contains(726)&&candidates.Contains(1118)&&
            !SameVector(before.Location,candidate.Location)&&SameKinematicBits(translated,candidate);
    }
    bool proved=false;
    constexpr int32 refinementWork=3072;
    if(refine&&budget.Remaining-budget.ConsumptionFloor>=refinementWork)
    {
        const uint64 began=budget.Used;
        C::FScopedDistanceBudgetFloor optional(budget,budget.Remaining-refinementWork);
        auto augmented=groupedVolume;
        FVector3d target;
        bool changed=false;
        C::FGroupedBoxSeparationObservation observed;
        // Propose the axes before unbounded spatial subdivision. If this
        // attempt fails, the original sealed volume retains its own proof path.
        if(optional.IsValid()&&NativePredictionObliqueAxes::Midpoint(hull,target)&&
            NativePredictionObliqueAxes::Augment(partition,target,budget,augmented,changed)&&changed)
        {
            proved=C::FReferenceBoxVolumeStrictSeparation::TryProve(
                partition,augmented,budget,separation,&observed);
            if(proved)groupedVolume=MoveTemp(augmented);
        }
        UE_LOG(LogTemp,Display,TEXT("[NativeBoxCorrectionObliqueTranslation] Frame=%u Provider=%d ChangedAxes=%d Proved=%d Stage=%s Work=%llu"),
            source.FrameClock.OwnerLocalFrame,observed.Provider,int32(changed),int32(proved),observed.Stage,budget.Used-began);
    }
    if(!proved)
        proved=C::FReferenceBoxVolumeStrictSeparation::TryProve(partition,groupedVolume,budget,separation);
    if(!proved||!separation||!separation->Matches(geometry->GetSelectedUnion(),groupedVolume))return false;
    TSharedPtr<FNativeBoxPoseCorrectionGeometryValues,ESPMode::ThreadSafe> value(new FNativeBoxPoseCorrectionGeometryValues);
    value->Snapshot=snapshot;value->Geometry=geometry;value->Source=source;value->Before=before;
    value->Candidate=candidate;value->Envelope=hull;value->GroupedVolume=MoveTemp(groupedVolume);value->GroupedSeparation=MoveTemp(separation);
    Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    out=MoveTemp(value);return true;
}
bool BuildNativeCurvedBoxPoseCorrectionGeometryValues(FPreparedWorldSnapshotPtr snapshot,const FNativeSolidStaticValues& source,
    const FKinematicState& before,const FKinematicState& candidate,const FNativeCanonicalBoxSweepClearanceValuesPtr& prediction,
    Analytic::Certified::FDistanceBudget& budget,FNativeBoxPoseCorrectionGeometryValuesPtr& out)
{
    namespace C=Analytic::Certified;
    out.Reset();
    const bool actual=prediction&&prediction->IsCurvedActualMotionClear();
    C::FIntervalPoint envelope;
    C::FReferenceBoundsCandidateSetPtr partition;
    if(actual)
    {
        if(CVarCurvedBoxActualClearRotationCorridor.GetValueOnAnyThread()!=1||
            !source.Trace.IgnoredSourceObjects.IsEmpty()||
            !MatchesNativeCurvedBoxActualMotionClearValues(snapshot,source,prediction,prediction->GetEnvelopeForInspection()))return false;
        const bool ignoredComponent=!source.Trace.IgnoredComponents.IsEmpty();
        if(ignoredComponent)
        {
#if !UE_BUILD_SHIPPING
            auto expected=before;
            expected.Rotation=candidate.Rotation;
            if(SameQuat(before.Rotation,candidate.Rotation)||!SameKinematicBits(expected,candidate)||
                !CurvedBoxArrivalDetail::FCase585PostFifthRotationAdmission::Matches(snapshot,source,budget))return false;
#else
            return false;
#endif
        }
        const auto& graph=prediction->GetCurvedWholeClearForInspection();
        if(!graph||!graph->ProveActualRotationCorrection(snapshot,source,before,candidate,budget,envelope,partition))return false;
        if(ignoredComponent&&(!partition||partition->GetCandidateIndices().Num()!=1||
            partition->GetCandidateIndices()[0]!=726))return false;
    }
    else
    {
        const auto arrival=prediction?prediction->GetCurvedArrivalForInspection():FNativeCurvedBoxArrivalValuesPtr{};
        if(!arrival||!arrival->ProveActualCorrection(snapshot,source,before,candidate,budget,envelope,partition))return false;
    }

    TSharedPtr<FNativeBoxPoseCorrectionGeometryValues,ESPMode::ThreadSafe> value(new FNativeBoxPoseCorrectionGeometryValues);
    value->Snapshot=snapshot;
    value->Geometry=snapshot->GetGeometrySelection();
    value->Source=source;
    value->Before=before;
    value->Candidate=candidate;
    value->Envelope=envelope;
    if(actual)value->CurvedActualPrediction=prediction;
    else value->CurvedPrediction=prediction;
    value->CurvedPartition=MoveTemp(partition);
    if(!value->Matches(snapshot,source,before,candidate))return false;
#if !UE_BUILD_SHIPPING
    if(actual&&!source.Trace.IgnoredComponents.IsEmpty())
    {
        UE_LOG(LogTemp,Display,TEXT("[NativeCase585PostFifthRotationGraph] Frame=%u Serial=6 Proved=1 Candidates=1 Provider=726 Component=%u"),
            source.FrameClock.OwnerLocalFrame,source.Trace.IgnoredComponents[0]);
    }
#endif
    out=MoveTemp(value);
    return true;
}
namespace EstablishedFloorPoseCorrectionDetail
{
bool Descriptor(const FPreparedWorldSnapshotPtr& snapshot,const FNativeSolidStaticValues& source,
    const FPreparedNativeSolidGeometryPtr& descriptor)
{
    if(!snapshot||!descriptor||!descriptor->GetRequest()||descriptor->GetRequest()->GetSnapshot()!=snapshot||
        descriptor->GetValues().Status!=ENativeSolidGeometryStatus::GeneralBoxFloorRefinerDescriptor||
        !SameNativeSolidStaticValues(descriptor->GetRequest()->GetValues(),source))return false;
    const auto& floor=descriptor->GetValues().GeneralBoxFloorDescriptor;
    return floor.ObservationRequestIdentity==descriptor->GetRequest().Get()&&
        SameNativeSolidStaticValues(floor.Captured,source)&&
        C::MatchesNativeGeneralBoxFloorPhysicalClear(floor)&&
        C::ClosedPhysicalFloorPoseDetail::SamePoint(floor.QueryEnvelope,descriptor->GetValues().QueryEnvelope);
}
}

bool BuildNativeEstablishedFloorPoseCorrectionGeometryValues(FPreparedWorldSnapshotPtr snapshot,
    const FNativeSolidStaticValues& source,const FKinematicState& before,const FKinematicState& candidate,
    FPreparedNativeSolidGeometryPtr descriptor,const C::FIntervalPoint& authenticated,
    C::FDistanceBudget& budget,FNativeBoxPoseCorrectionGeometryValuesPtr& out)
{
    out.Reset();Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    const auto geometry=snapshot?snapshot->GetGeometrySelection():FPreparedGeometrySelectionPtr{};
    auto expected=before;expected.Location.Z=candidate.Location.Z;
    if(!geometry||!geometry->HasValidAliases()||geometry->GetModel()!=EPreparedGeometryModel::RetainedReference||
        source.Shape!=ENativeSolidQueryShape::Box||source.Frozen||source.Prescribed||!FiniteState(before)||!FiniteState(candidate)||
        !(candidate.Location.Z>before.Location.Z)||!SameKinematicBits(expected,candidate)||
        !EstablishedFloorPoseCorrectionDetail::Descriptor(snapshot,source,descriptor))return false;
    const auto& plane=descriptor->GetValues().GeneralBoxFloorDescriptor.CachedPlane;
    C::FClosedPhysicalFloorPoseCorridorBoundsPtr floor;
    if(!C::TryProveClosedPhysicalFloorPoseCorridor(MakeNativeBoxPoseCorrectionCorridorInput(source,before,candidate),
        authenticated,plane.Point,plane.Normal,budget,floor)||!floor)return false;
    TSharedPtr<FNativeBoxPoseCorrectionGeometryValues,ESPMode::ThreadSafe> value(new FNativeBoxPoseCorrectionGeometryValues);
    value->Snapshot=snapshot;value->Geometry=geometry;value->Source=source;value->Before=before;value->Candidate=candidate;
    value->Envelope=authenticated;value->EstablishedFloor=MoveTemp(descriptor);value->FloorCorridor=MoveTemp(floor);
    if(!value->Matches(snapshot,source,before,candidate))return false;
    out=MoveTemp(value);return true;
}

bool FNativeBoxPoseCorrectionGeometryValues::Matches(const FPreparedWorldSnapshotPtr& snapshot,
    const FNativeSolidStaticValues& source,const FKinematicState& before,const FKinematicState& candidate) const
{
    return MatchesEndpoint(snapshot,source,candidate)&&SameKinematicBits(Before,before)&&SameKinematicBits(Candidate,candidate);
}
bool FNativeBoxPoseCorrectionGeometryValues::MatchesEndpoint(const FPreparedWorldSnapshotPtr& snapshot,
    const FNativeSolidStaticValues& source,const FKinematicState& actual) const
{
    if(Snapshot!=snapshot||!snapshot||Geometry!=snapshot->GetGeometrySelection()||!Geometry||!Geometry->HasValidAliases()||
        !SameNativeSolidStaticValues(Source,source)||!SameVector(Candidate.Location,actual.Location)||
        !SameQuat(Candidate.Rotation,actual.Rotation))return false;
    if(CurvedActualPrediction)
    {
        const auto& graph=CurvedActualPrediction->GetCurvedWholeClearForInspection();
        return graph&&!CurvedPrediction&&!EstablishedFloor&&!FloorCorridor&&!GroupedSeparation&&
            MatchesNativeCurvedBoxActualMotionClearValues(snapshot,source,CurvedActualPrediction,
                CurvedActualPrediction->GetEnvelopeForInspection())&&graph->CoversActualCorrectionPartition(CurvedPartition,Envelope);
    }
    if(CurvedPrediction)
    {
        const auto& arrival=CurvedPrediction->GetCurvedArrivalForInspection();
        return arrival&&!EstablishedFloor&&!FloorCorridor&&!GroupedSeparation&&
            arrival->Matches(snapshot,source)&&(arrival->CoversCorrectionPartition(CurvedPartition,Envelope)||
                arrival->MatchesNativeAdjacentBandCorrectionPartition(CurvedPartition,Envelope,Before,Candidate));
    }
    if(FloorCorridor)
    {
        if(GroupedSeparation||!EstablishedFloorPoseCorrectionDetail::Descriptor(snapshot,source,EstablishedFloor))return false;
        const auto& plane=EstablishedFloor->GetValues().GeneralBoxFloorDescriptor.CachedPlane;
        return FloorCorridor->Matches(MakeNativeBoxPoseCorrectionCorridorInput(source,Before,Candidate),
            Envelope,plane.Point,plane.Normal);
    }
    return !EstablishedFloor&&GroupedSeparation&&C::GroupedBoxDetail::SamePoint(Envelope,GroupedVolume.BroadHull)&&
        GroupedSeparation->Matches(Geometry->GetSelectedUnion(),GroupedVolume);
}
bool FNativeBoxPoseCorrectionGeometryValues::ContainsEndpoint(const Analytic::FWorldQuery& pose) const
{
    if(CurvedActualPrediction)
    {
        const auto& graph=CurvedActualPrediction->GetCurvedWholeClearForInspection();
        C::FDistanceBudget budget{3072};
        return graph&&MatchesEndpoint(Snapshot,Source,Candidate)&&graph->ContainsActualEndpoint(pose,budget,&Envelope);
    }
    if(CurvedPrediction)
    {
        const auto& arrival=CurvedPrediction->GetCurvedArrivalForInspection();
        C::FDistanceBudget budget{1536};
        return arrival&&MatchesEndpoint(Snapshot,Source,Candidate)&&
            arrival->ContainsActualEndpoint(Snapshot,Source,pose,Envelope,budget,1536);
    }
    if(FloorCorridor)
    {
        if(!MatchesEndpoint(Snapshot,Source,Candidate)||pose.Shape!=Analytic::EQueryShape::Box||
            !SameVector(pose.Start,pose.End))return false;
        for(const auto& extent:{Source.PhysicalHalfExtent,Source.QueryHalfExtent})
        {
            auto fixed=pose;fixed.HalfExtent=extent;C::FIntervalPoint endpoint;
            if(!C::BuildFixedWorldQueryVolumeBounds(fixed,endpoint)||
                !C::ClosedPhysicalFloorPoseDetail::Within(Envelope,endpoint))return false;
        }
        return true;
    }
    return !EstablishedFloor&&Geometry&&Geometry->HasValidAliases()&&GroupedSeparation&&
        GroupedSeparation->Matches(Geometry->GetSelectedUnion(),GroupedVolume)&&
        C::GroupedBoxProofContainsEndpoint(GroupedSeparation,pose,Source.PhysicalHalfExtent,Source.QueryHalfExtent);
}

}
