// Same IAS TU as SBox's native arithmetic and ResponseQualityDetail. No
// unexported cross-DLL SBox call, event serial, response/advance or Ready grant.
namespace Speed::PreparedExecution
{
namespace NativeSpherePhaseDetail
{
namespace K=Analytic::Certified::ContactKernelProfile72;
static TAutoConsoleVariable<int32> Enabled(TEXT("p.IAmSpeed.AnalyticWorld.NativeSpherePhaseTiming"),0,
    TEXT("Private native Sphere stage timing. At most 128 observations per worker thread. Default OFF."));
#if !UE_BUILD_SHIPPING
static TAutoConsoleVariable<int32> PrefixRosterReadback(
    TEXT("p.IAmSpeed.AnalyticWorld.NativeSphere632799PrefixRosterReadback"),0,
    TEXT("Private P202W1 request readback. Default OFF; no authority or solver effect."));
#endif
enum EStage : uint8 {Input,Policy,Volume,Initial,FullRc,Reference,Quality,BuildHit,Publish,StageCount};
class FSample final
{
    const FNativeSolidStaticValues& Request;
    const Analytic::Certified::FDistanceBudget& Budget;
    TOptional<K::FEnabledScope> KernelScope;
    K::FCounters Before;
    uint64 Ticks[StageCount]{},Work[StageCount]{};
    uint64 Mark=0,MarkWork=0,Start=0;
    EStage Stage=Input;
    bool Integrity=true;
    uint32 Result=0;
    uint64 TimeWork=0,SpatialCells=0;
    int32 TemporalRounds=0,ConsumerRounds=0;
public:
    FSample(const FNativeSolidStaticValues& request,const Analytic::Certified::FDistanceBudget& budget)
        :Request(request),Budget(budget)
    {
        if(Enabled.GetValueOnAnyThread()!=1)return;
        static thread_local uint32 Count=0;
        if(Count>=128)return;
        ++Count;
        KernelScope.Emplace(true);Before=K::Counters();
        MarkWork=Budget.Used;Start=Mark=FPlatformTime::Cycles64();
    }
    void Next(EStage stage)
    {
        if(!KernelScope.IsSet())return;
        const uint64 now=FPlatformTime::Cycles64(),used=Budget.Used;
        if(now<Mark||used<MarkWork)Integrity=false;
        else if(Stage<StageCount){Ticks[Stage]=now-Mark;Work[Stage]=used-MarkWork;}
        Mark=now;MarkWork=used;Stage=stage;
    }
    void Note(const Analytic::Certified::FTensorSphereContactEnclosure& contact)
    {
        if(!KernelScope.IsSet())return;
        TimeWork=contact.TimeWorkUsed;SpatialCells=contact.SpatialVisitedCells;
        TemporalRounds=contact.TemporalWidthRefinements;ConsumerRounds=contact.ConsumerWidthRefinements;
    }
    void SetResult(uint32 result){Result=result;}
    ~FSample()
    {
        if(!KernelScope.IsSet())return;
        Next(StageCount);
        const auto& after=K::Counters();
        uint64 kt[K::StageCount]{},kc[K::StageCount]{};
        for(uint32 i=0;i<K::StageCount;++i)
        {
            if(after.Ticks[i]<Before.Ticks[i]||after.Calls[i]<Before.Calls[i])Integrity=false;
            else {kt[i]=after.Ticks[i]-Before.Ticks[i];kc[i]=after.Calls[i]-Before.Calls[i];}
        }
        Integrity&=Before.Integrity&&after.Integrity&&Mark>=Start;
        UE_LOG(LogTemp,Display,TEXT("[NativeSpherePhaseTiming] Frame=%u H=%.17g Result=%u Integrity=%d SecPerCycle=%.17g Total=%llu Input=%llu Policy=%llu Volume=%llu Initial=%llu FullRc=%llu Reference=%llu Quality=%llu BuildHit=%llu Publish=%llu TimeKernel=%llu SpatialKernel=%llu KernelSetup=%llu LiftKernel=%llu TimeCalls=%llu SpatialPasses=%llu LiftCalls=%llu PolicyWork=%llu VolumeWork=%llu InitialWork=%llu FullRcWork=%llu ReferenceWork=%llu QualityWork=%llu BuildHitWork=%llu TimeWork=%llu SpatialCells=%llu TemporalRounds=%d ConsumerRounds=%d"),
            Request.FrameClock.OwnerLocalFrame,double(Request.Horizon),Result,int32(Integrity),FPlatformTime::GetSecondsPerCycle64(),Mark>=Start?Mark-Start:0,
            Ticks[Input],Ticks[Policy],Ticks[Volume],Ticks[Initial],Ticks[FullRc],Ticks[Reference],Ticks[Quality],Ticks[BuildHit],Ticks[Publish],
            kt[K::TimeSearch],kt[K::SpatialPass],kt[K::RootSetup],kt[K::AffineLift],kc[K::TimeSearch],kc[K::SpatialPass],kc[K::AffineLift],
            Work[Policy],Work[Volume],Work[Initial],Work[FullRc],Work[Reference],Work[Quality],Work[BuildHit],
            TimeWork,SpatialCells,TemporalRounds,ConsumerRounds);
    }
};
}

bool BuildNativeGeneralSphereContactValuesImpl(FPreparedGeometrySelectionPtr geometry,const FNativeSolidStaticValues& v,
    TFunctionRef<bool(const Analytic::Prepared::FOwner&)> accepts,C::FDistanceBudget& budget,FNativeGeneralSphereContactValuesPtr& out,FString& reason,
    C::FIntervalPoint* wholeRcClearEnvelope,const FNativeRepeatedSphereFloorExclusion* repeated,bool directedShortOnly)
{
    out.Reset();reason.Reset();const auto refuse=[&](const TCHAR* why){reason=why;return false;};
    namespace Phase=NativeSpherePhaseDetail;
    Phase::FSample phase(v,budget);
    if(wholeRcClearEnvelope)*wholeRcClearEnvelope={};
    Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(!geometry||!geometry->HasValidAliases()||geometry->GetModel()!=EPreparedGeometryModel::RetainedReference||
        v.Shape!=ENativeSolidQueryShape::Sphere||v.Frozen||v.Prescribed||!v.SphereMarginCaptured||
        !SameVector(v.OwnerState.Location,v.BodyState.Location)||!SameVector(v.OwnerState.Velocity,v.BodyState.Velocity)||
        !SameVector(v.OwnerState.Acceleration,v.BodyState.Acceleration))return refuse(TEXT("General Sphere actual zero-offset PVA unavailable"));
    phase.Next(Phase::Policy);
    const auto scene=geometry->GetSelectedUnion();
    FGeneralFirstArrivalRsPolicyAdmission::FPtr admission;
    if(!FGeneralFirstArrivalRsPolicyAdmission::TryBuild(geometry,accepts,budget,admission,reason)||!admission)
        return false;
    const auto& policy=admission->GetPolicy();
    phase.Next(Phase::Volume);
    C::FFloorNativeSolidModel envelopeModel;C::FIntervalPoint envelope;
    if(!C::BuildNativeSolidSweepVolume(v,envelopeModel,envelope))return refuse(TEXT("General native/reference/captured/query envelope unavailable"));
    C::FReferenceBoundsCandidateSetPtr volume;
    if(!C::FReferenceBoundsCandidateSet::Select(scene,envelope,budget,volume)||!volume||volume->GetScene()!=scene)
        return refuse(TEXT("General complete native World-query envelope partition unavailable"));
    phase.Next(Phase::Initial);
    FSphereResponseQueryValues q;q.Position=v.OwnerState.Location;q.Velocity=v.OwnerState.Velocity;q.Acceleration=v.OwnerState.Acceleration;
    q.Horizon=v.Horizon;q.CollisionRadius=v.QueryRadius;q.PhysicalRadius=v.PhysicalRadius;
    q.ResponseMargin=v.SphereMargin;q.StaticContactOffsetCm=v.StaticContactOffset;
    C::FReferenceSphereOptions options;
    // Keep the same temporal search allowance. The complete admission pass
    // replaces both previous policy scans. Reserve their original total
    // allowance outside the search for final partition/leaf/publication checks.
    const int32 responseReserve=policy.AcceptedProviders.Num();
    if(!BuildMarginDrivenSphereReferenceOptions(q,budget.Remaining-responseReserve,options))return refuse(TEXT("Captured native margin cannot choose existing mathematical goals"));
    // The spatial solver asks each leaf for one quarter of the total
    // normal width. Leave room for that leaf's point enclosure before
    // subdivision, by narrowing the time bracket for this quadratic path.
    // This changes the proof target only. The captured acceptance margin,
    // geometry, complete prefix and response quality checks stay in force.
    options.TimeGap*=.125;
    // The time search treats a bounded spatial enclosure that straddles
    // the radius as unresolved. Leave it enough work to split the original
    // time interval instead of exhausting the account at one endpoint.
    options.SpatialNodesPerAttempt=FMath::Min(options.SpatialNodesPerAttempt,512);
    Analytic::Prepared::FStaticSphereInitialValues input;input.State=v.OwnerState;input.PhysicalRadius=v.PhysicalRadius;
    input.StaticContactOffset=v.StaticContactOffset;input.ActualMargin=v.SphereMargin;input.QueryRadius=v.QueryRadius;input.SupportRadius=v.SupportRadius;
    Analytic::Prepared::FReferenceSphereStrictSeparationPtr initial;
    C::FZeroOffsetKinematicCenterReference nextModel;
    if(repeated)
    {
        if(wholeRcClearEnvelope||!C::BuildZeroOffsetKinematicCenterReference(v.OwnerState.Location,v.OwnerState.Velocity,
            v.OwnerState.Acceleration,v.Horizon,v.LocalOffset,v.LocalCOM,nextModel))return false;
    }
    else if(!Analytic::Prepared::FReferenceSphereStrictSeparation::TryProveAt(geometry,input,v.LocalOffset,v.LocalCOM,v.Horizon,0.f,options,budget,initial,reason)||!initial)
        return false; // no invented tangency/overlap/MTD path
    // Both checks cover the same rounded center path. Every strict
    // exclusion at Rc is also an exclusion at Rs<=Rc, so retain one
    // conservative complete partition. Contact and consumed-prefix checks
    // still use the captured Rs on their original exact paths.
    phase.Next(Phase::FullRc);
    if(double(v.QueryRadius)<v.SupportRadius)return refuse(TEXT("General support radius exceeds the captured query radius"));
    const auto fullRc=SelectSelectedWholeConsumedSphereHull(geometry,repeated?nextModel:initial->GetModel(),v.Horizon,double(v.QueryRadius),budget);
    if(!fullRc.UsesGeometry(geometry))return refuse(TEXT("General full-H Rs/Rc coverage unavailable"));
    phase.Next(Phase::Reference);
    Analytic::Prepared::FReferenceSphereGeneralFirstArrivalRsPtr arrival;
    C::FReferenceSphereEvaluation refused;
    bool wholeRcClear=false;
    const bool evaluated=repeated?Analytic::Prepared::FReferenceSphereGeneralFirstArrivalRs::TryEvaluateNextEligible(
        *repeated,options,fullRc,responseReserve,budget,arrival,reason):
        Analytic::Prepared::FReferenceSphereGeneralFirstArrivalRs::TryEvaluateWithFullRc(
        MoveTemp(initial),v.Horizon,options,fullRc,responseReserve,budget,arrival,reason,&refused,
        wholeRcClearEnvelope?&v:nullptr,wholeRcClearEnvelope?volume.Get():nullptr,wholeRcClearEnvelope?&wholeRcClear:nullptr,directedShortOnly);
    phase.Next(Phase::Quality);
    phase.Note(arrival?arrival->GetReference().Contact:refused.Contact);
    if(wholeRcClear)
    {
        if(!evaluated||!wholeRcClearEnvelope||arrival)return refuse(TEXT("Inconsistent native whole-Rc clearance result"));
        *wholeRcClearEnvelope=volume->GetQueryHull();
        phase.SetResult(1);
        return true;
    }
    if(directedShortOnly)return false; // A miss probe emits neither refusal telemetry nor a hit.
    if(!evaluated||!arrival)
    {
        const auto& contact=refused.Contact;
        const auto candidates=refused.Candidates?refused.Candidates->GetCandidateIndices():TConstArrayView<int32>{};
        reason+=FString::Printf(TEXT("; ContactFailure=%u Site=%s TimeFailure=%u DyadicSite=%u Depth=%d Dyadic=[%.17g,%.17g] WidthRounds=%d Bracket=[%.17g,%.17g] Candidates=%d First=%d Second=%d Third=%d"),
            uint32(contact.Failure),contact.WorkSite,uint32(contact.TimeFailure),uint32(contact.TimeDyadicSite),
            contact.TimeDyadicDepth,contact.TimeDyadicA,contact.TimeDyadicB,contact.TemporalWidthRefinements,contact.LowerTime,contact.UpperTime,
            candidates.Num(),candidates.Num()>0?candidates[0]:-1,candidates.Num()>1?candidates[1]:-1,candidates.Num()>2?candidates[2]:-1);
        reason+=FString::Printf(TEXT("; ConsumerRounds=%d TimeWork=%llu SpatialCells=%llu WidthDiagnostic=%d WidthRound=%d WidthProviderLocal=%d WidthV=[%.17g,%.17g] WidthPointTarget=%.17g WidthNormalTarget=%.17g"),
            contact.ConsumerWidthRefinements,contact.TimeWorkUsed,contact.SpatialVisitedCells,int32(contact.HasWidthDiagnostic),
            contact.WidthRefinement,contact.WidthSurface,contact.WidthMinV,contact.WidthMaxV,contact.WidthPointTarget,contact.WidthNormalTarget);
        if(contact.HasWidthDiagnostic)
        {
            for(int32 axis=0;axis<3;++axis)
                reason+=FString::Printf(TEXT(" W%d=(%.17g,%.17g;%.17g,%.17g)"),axis,
                    contact.WidthPointUnion.Axis[axis].Lower,contact.WidthPointUnion.Axis[axis].Upper,
                    contact.WidthNormalUnion.Axis[axis].Lower,contact.WidthNormalUnion.Axis[axis].Upper);
        }
        UE_LOG(LogTemp,Display,TEXT("[NativeGutterSphereCaptured] Frame=%u P=(%.17g,%.17g,%.17g) V=(%.17g,%.17g,%.17g) A=(%.17g,%.17g,%.17g) H=%.17g Rs=%.17g Rc=%.17g"),
            v.FrameClock.OwnerLocalFrame,q.Position.X,q.Position.Y,q.Position.Z,q.Velocity.X,q.Velocity.Y,q.Velocity.Z,
            q.Acceleration.X,q.Acceleration.Y,q.Acceleration.Z,double(q.Horizon),v.SupportRadius,double(q.CollisionRadius));
        return false;
    }
    if(arrival->GetReference().Status!=C::EReferenceSphereStatus::ReferenceContactBounded)
    {
#if !UE_BUILD_SHIPPING
        static const auto* wallReadback=IConsoleManager::Get().FindTConsoleVariableDataInt(
            TEXT("p.IAmSpeed.AnalyticWorld.NativeSphereFiniteWallQuadraticRs"));
        const auto padded=volume->GetCandidateIndices();
        if(wallReadback&&wallReadback->GetValueOnAnyThread()==1&&
            arrival->GetReference().Status==C::EReferenceSphereStatus::ReferenceNoContact&&
            v.Horizon>0.f&&v.Horizon<=1.f/300.f&&padded.Num()<=16&&padded.Contains(2693)&&padded.Contains(2694))
        {
            FStepIdentity step;
            const bool stepKnown=ReadCurrentStep(step);
            FString roster;
            for(const int32 index:padded)
            {
                if(!roster.IsEmpty())roster+=TEXT(",");
                roster+=FString::FromInt(index);
            }
            UE_LOG(LogTemp,Display,TEXT("[NativeSphereFiniteWallDepartureReadback] Stage=rs_no_contact_rc_unproved Frame=%u Canonical=%llu StepKnown=%d Step=%llu Iteration=%d Serial=%llu Epoch=%llu Numeric=%016llx Owner=%llu Body=%d H=%.17g HBits=%08x TimePassed=%.17g P=(%.17g,%.17g,%.17g) V=(%.17g,%.17g,%.17g) A=(%.17g,%.17g,%.17g) QStart=(%.17g,%.17g,%.17g) QEnd=(%.17g,%.17g,%.17g) Rs=%.17g Rc=%.17g Domain=%.17g InitialOverlap=%.17g Padded=%s Used=%llu Remaining=%d"),
                v.FrameClock.OwnerLocalFrame,v.FrameClock.CanonicalFrame,int32(stepKnown),step.StepSerial,step.Iteration,
                step.QuerySerial,step.QueryEpoch,step.NumericControl,v.OwnerStableId,v.BodyIndex,double(v.Horizon),
                C::KinematicCenterDetail::Bits(v.Horizon),double(step.TimePassed),q.Position.X,q.Position.Y,q.Position.Z,
                q.Velocity.X,q.Velocity.Y,q.Velocity.Z,q.Acceleration.X,q.Acceleration.Y,q.Acceleration.Z,
                v.Query.Start.X,v.Query.Start.Y,v.Query.Start.Z,v.Query.End.X,v.Query.End.Y,v.Query.End.Z,
                v.SupportRadius,double(v.QueryRadius),v.Query.DomainTolerance,v.Query.InitialOverlapTolerance,
                *roster,budget.Used,budget.Remaining);
        }
#endif
        return refuse(TEXT("General no-contact Rs result is not a native Rc World miss"));
    }
    FGeneralFirstArrivalRsResponseQualityPtr quality;
    if(EvaluateGeneralFirstArrivalRsResponseQuality(arrival,policy,budget,quality,reason,admission.Get())!=ESphereResponseQualityStatus::BoundedWithinInputMargin||!quality)
        return false;
    phase.Next(Phase::BuildHit);
    FGeneralFirstArrivalRsHitValues named;
    if(!BuildGeneralFirstArrivalRsHitValues(geometry,quality,v.FrameClock.OwnerLocalFrame,budget,named,reason))return false;
#if !UE_BUILD_SHIPPING
    if(NativeSpherePhaseDetail::PrefixRosterReadback.GetValueOnAnyThread()==1&&!repeated)
    {
        const uint32 hbits=C::KinematicCenterDetail::Bits(v.Horizon);
        const uint64 canonical=v.FrameClock.CanonicalFrame;
        const bool first632=canonical==4395&&hbits==0x3b59e45c&&q.Position.X==14.944408288634424&&q.Position.Y==3839.1291550372389;
        const bool second632=canonical==4395&&hbits==0x3b41e47f&&q.Position.X==14.998906262575835&&q.Position.Y==3840.0012947345808;
        const bool first799=canonical==5359&&hbits==0x3b5a740e&&q.Position.X==4956.6419417560528&&q.Position.Y==-2091.580078125;
        static thread_local uint32 reports=0;
        if(reports<3&&(first632||second632||first799))
        {
            ++reports;
            const auto roster=[](TConstArrayView<int32> indices)
            {
                FString row;
                for(int32 index:indices)
                {
                    if(!row.IsEmpty())row+=TEXT(",");
                    row+=FString::FromInt(index);
                }
                return row;
            };
            const auto& proof=arrival->GetReference();
            const FString rs=proof.Candidates?roster(proof.Candidates->GetCandidateIndices()):TEXT("-");
            const FString rc=fullRc.Partition?roster(fullRc.Partition->GetCandidateIndices()):TEXT("-");
            const FString padded=volume?roster(volume->GetCandidateIndices()):TEXT("-");
            FStepIdentity step;
            const bool stepKnown=ReadCurrentStep(step);
            UE_LOG(LogTemp,Display,TEXT("[NativeSphere632799PrefixRosterReadback] Case=%d Capture=%d Canonical=%llu OwnerFrame=%u Owner=%llu Body=%d StepKnown=%d Step=%llu Serial=%llu Epoch=%llu Iteration=%d Numeric=%016llx H=%.17g HBits=%08x Rs=%.17g Rc=%.17g Domain=%.17g RsRoster=%s RcRoster=%s PaddedRoster=%s Lower=%.17g Upper=%.17g Consumed=%.17g ConsumedBits=%08x WinnerSource=%016llx WinnerSurface=%016llx WinnerPrimitive=%016llx TimeWork=%llu"),
                first799?799:632,second632?2:1,canonical,v.FrameClock.OwnerLocalFrame,v.OwnerStableId,v.BodyIndex,
                int32(stepKnown),step.StepSerial,step.QuerySerial,step.QueryEpoch,step.Iteration,step.NumericControl,
                double(v.Horizon),hbits,v.SupportRadius,double(v.QueryRadius),v.Query.DomainTolerance,*rs,*rc,*padded,
                proof.Contact.LowerTime,proof.Contact.UpperTime,double(proof.ConsumedSeconds),
                C::KinematicCenterDetail::Bits(proof.ConsumedSeconds),named.Hit.SourceId,named.Hit.SurfaceId,
                named.Hit.PrimitiveId,proof.Contact.TimeWorkUsed);
        }
    }
#endif
    phase.Next(Phase::Publish);
    TSharedPtr<FNativeGeneralSphereContactValues,ESPMode::ThreadSafe> result(new FNativeGeneralSphereContactValues);
    result->Geometry=MoveTemp(geometry);result->Captured=v;result->Envelope=envelope;result->VolumePartition=MoveTemp(volume);
    result->FullRs=fullRc.Partition;result->FullRc=fullRc.Partition;result->Arrival=MoveTemp(arrival);result->Named=MoveTemp(named);
    Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);out=MoveTemp(result);phase.SetResult(2);return true;
}
bool BuildNativeGeneralSphereContactValues(FPreparedGeometrySelectionPtr geometry,const FNativeSolidStaticValues& v,
    TFunctionRef<bool(const Analytic::Prepared::FOwner&)> accepts,C::FDistanceBudget& budget,FNativeGeneralSphereContactValuesPtr& out,
    FString& reason,C::FIntervalPoint* wholeRcClearEnvelope)
{
    return BuildNativeGeneralSphereContactValuesImpl(MoveTemp(geometry),v,accepts,budget,out,reason,wholeRcClearEnvelope,nullptr,false);
}
#if !UE_BUILD_SHIPPING
bool ProveNativeGeneralSphereDirectedShortMiss(FPreparedGeometrySelectionPtr geometry,const FNativeSolidStaticValues& values,
    TFunctionRef<bool(const Analytic::Prepared::FOwner&)> accepts,C::FDistanceBudget& budget,C::FIntervalPoint& envelope)
{
    FNativeGeneralSphereContactValuesPtr contact;
    FString reason;
    return BuildNativeGeneralSphereContactValuesImpl(MoveTemp(geometry),values,accepts,budget,contact,reason,&envelope,nullptr,true)&&
        !contact&&C::Detail::ValidPoint(envelope);
}
#endif
bool BuildNativeNextEligibleSphereContactValues(const FNativeRepeatedSphereFloorExclusion& excluded,
    TFunctionRef<bool(const Analytic::Prepared::FOwner&)> accepts,C::FDistanceBudget& budget,FNativeGeneralSphereContactValuesPtr& out,FString& reason)
{
    out.Reset();const auto request=excluded.GetCurrentRequest();const auto* pairs=excluded.GetCurrentPairs();
    if(!request||!pairs||!excluded.IsCurrent(request,*pairs))return false;
    const bool built=BuildNativeGeneralSphereContactValuesImpl(request->GetSnapshot()->GetGeometrySelection(),
        request->GetValues(),accepts,budget,out,reason,nullptr,&excluded,false);
    if(!built||!out||!out->IsNextEligible()||!excluded.IsCurrent(request,*pairs)){out.Reset();return false;}
    return true;
}
bool BindNativeGeneralSphereContactValues(const FNativeSolidStaticValues& v,const FNativeGeneralSphereContactValuesPtr& values,
    const FPreparedSourceContactBinding& binding,SHitResult& out)
{
    out={};if(!values||!values->Geometry||!SameNativeSolidStaticValues(v,values->Captured)||
        values->Named.Geometry!=values->Geometry||!values->Named.Quality||!values->Arrival||
        values->Named.Quality->GetArrival()!=values->Arrival||!values->FullRs||!values->FullRc||!values->VolumePartition||
        values->FullRs!=values->FullRc||values->Arrival->GetRcPartition()!=values->FullRc||
        values->FullRs->GetScene()!=values->Geometry->GetSelectedUnion()||values->FullRc->GetScene()!=values->Geometry->GetSelectedUnion()||
        values->VolumePartition->GetScene()!=values->Geometry->GetSelectedUnion())return false;
    const auto& h=values->Named.Hit;
    if(!h.bHit||!h.bBlockingHit||h.bStartPenetrating||!h.SourceId||!h.SurfaceId||!h.PrimitiveId||
        h.FrameTag!=v.FrameClock.OwnerLocalFrame||h.SourceId!=binding.SourceId||!binding.SourceIdentity||!binding.ContactIdentity||
        binding.ContactToken.IsExplicitlyNull()||!h.Component.IsExplicitlyNull()||!h.SubBody.IsExplicitlyNull()||
        h.PreparedGroundEvidence||h.FirstRsResponseData)return false;
    // All scalar/normal/Rp-arm/TOI bits were validated by the same immutable
    // GENERAL quality. Copy the genuine GT weak token, never resolve it here.
    out=h;out.Component=binding.ContactToken;return true;
}
}
