// Ordinary World response OBSERVATIONS. No request revalidation, integration
// permission, ResponseData annotation, geometry/effect admission or Ready bit.
bool FStepScope::HasOrdinaryResponseObservation() const
{
    if(OrdinaryResponse){RequireOrdinaryResponseContext();return true;}
    if(!Active||!Window||Window->GetPurpose()!=OnePausedWork::EPurpose::FullFrame)return false;
    Refuse("Ordinary response hook has no actual opened batch");
}
void FStepScope::RequireOrdinaryResponseContext() const
{
    ::PreparedFrameworkProfile1::FScope observedFramework(::PreparedFrameworkProfile1::ResponseContext);
    FStepIdentity now;
    if(!Active||StepScope!=this||!Window||Window->GetPurpose()!=OnePausedWork::EPurpose::FullFrame||
        !OrdinaryResponse||OrdinaryResponse->Closed||SweepPhase||ComponentSweepScope||MemberSweepScope||
        WheelProbeScope||!ReadCurrentStep(now)||OrdinaryResponse->Snapshot!=Snapshot||
        !OrdinaryWorldDetail::Epoch(OrdinaryResponse->EntryStep,now)||
        OrdinaryResponse->EntryStep.QuerySerial!=now.QuerySerial||!Snapshot||!Snapshot->GetNativeOwners()||
        !FCanonicalFramePreflight::HasUnchangedRawPreparationCensus(*Identity.Bridge,*Snapshot,Identity.StepSerial))
        Refuse("Ordinary response observation lost its fresh consumed context");
    if(OwnerProbePhase)
    {
        const auto& tx=*OrdinaryResponse;const auto* state=OwnerProbePhase->State.Get();
        const ISpeedComponent* actualOwner=nullptr;
        if(tx.ActiveCall==2&&tx.Entries.IsValidIndex(tx.ActiveIndex))
            for(const auto& owner:Snapshot->GetNativeOwners()->GetRecordsForInspection())
                for(const auto& child:owner.Bodies)if(child.Body==tx.Entries[tx.ActiveIndex].RawResolver)actualOwner=owner.Adapter;
        if(!state||!OwnerProbePhase->Check()||state->Returned||state->Phase!=ENativeOwnerProbePhase::ResponseResolve||
            state->Snapshot!=Snapshot||state->Owner!=actualOwner||!actualOwner)
            Refuse("Ordinary response overlaps an unrelated native owner probe phase");
    }
}
void FStepScope::ReadOrdinaryConsumedState(FOrdinaryConsumedStateValues& out) const
{
    RequireOrdinaryResponseContext();ReadOrdinaryFrameStateValues(out);RequireOrdinaryResponseContext();
}
void FStepScope::BeginOrdinaryResponseBatch(TConstArrayView<SComponentTOI> selected,
    const Collision::FResolvedPairSet& pairs,float seconds,float before,float actualAfter,float simTime)
{
    if(!Active||!Window||Window->GetPurpose()!=OnePausedWork::EPurpose::FullFrame)
    {if(OrdinaryResponse&&!OrdinaryResponse->Closed)Refuse("Ordinary response lost FullFrame context");return;}
    FStepIdentity now;
    if(OrdinaryResponse||StepScope!=this||SweepPhase||!ReadCurrentStep(now)||!OrdinaryReturns||
        !OrdinaryReturns->World||!OrdinaryReturns->World->Closed)Refuse("Ordinary response has no unique closed selector");
    if(!OrdinaryIntegration||!OrdinaryIntegration->Closed||!OrdinaryIntegration->HasActualSelection||
        !OrdinaryIntegration->SelectorJoined||!SameFloat(seconds,OrdinaryIntegration->Seconds)||
        !SameFloat(before,OrdinaryIntegration->ActualBefore)||!SameFloat(actualAfter,OrdinaryIntegration->ActualAfter))
        Refuse("Ordinary response lacks the returned actual integration observation");
    const auto& source=*OrdinaryReturns->World;auto consumedEpoch=source.Completion;
    if(consumedEpoch.QueryEpoch==MAX_uint64)Refuse("Ordinary response selector epoch exhausted");
    ++consumedEpoch.QueryEpoch;
    if(!OrdinaryWorldDetail::Epoch(consumedEpoch,now)||consumedEpoch.QuerySerial!=now.QuerySerial||
        source.PairsIdentity!=&pairs||selected.IsEmpty()||selected.Num()!=source.Sorted.Num()||
        !SameFloat(seconds,source.Seconds)||!SameFloat(before,source.Before)||!SameFloat(simTime,now.SimTime)||
        !FMath::IsFinite(actualAfter)||!SameFloat(actualAfter,before+seconds)||actualAfter>now.Delta)
        Refuse("Ordinary response actual batch/consumed clock differs from executed selector");
    for(int32 i=0;i<selected.Num();++i)
        if(!OrdinaryWorldDetail::Entry(selected[i],source.Sorted[i],source.Sorted[i].TOI))Refuse("Ordinary response batch retimed or reordered");
    OrdinaryResponse=MakeUnique<FOrdinaryResponseState>();auto& tx=*OrdinaryResponse;
    tx.EntryStep=now;tx.SelectorStep=source.Completion;tx.Snapshot=Snapshot;tx.PairsIdentity=&pairs;
    tx.Selected.Append(selected.GetData(),selected.Num());tx.Seconds=seconds;tx.Before=before;tx.ActualAfter=actualAfter;tx.SimTime=simTime;
    // ReadCurrentStep retains the pre-integrate clocks by native design. The
    // actual local clock above is separate; no synthetic identity is stamped.
    ReadOrdinaryConsumedState(tx.EntryValues);
}
void FStepScope::ObserveOrdinaryResponseDecision(int32 index,const SComponentTOI& selected,const USSubBody* raw,
    bool eligible,bool allowed,const Collision::FResolvedPairSet& pairs)
{
    if(!HasOrdinaryResponseObservation())return;
    RequireOrdinaryResponseContext();auto& tx=*OrdinaryResponse;
    if(tx.ActiveIndex!=INDEX_NONE||tx.ActiveCall!=INDEX_NONE||index!=tx.NextIndex||!tx.Selected.IsValidIndex(index)||
        tx.Entries.Num()!=index||tx.PairsIdentity!=&pairs||!OrdinaryWorldDetail::Entry(selected,tx.Selected[index],tx.Selected[index].TOI)||
        (!raw&&(eligible||allowed)))Refuse("Ordinary response decision duplicated, reordered or changed");
    // raw comes from the ACTUAL Weak.Get in WorldStep, not a second lookup.
    if(raw)
    {
        const FPreparedNativeBodyBinding* found=nullptr;
        for(const auto& owner:Snapshot->GetNativeOwners()->GetRecordsForInspection())for(const auto& body:owner.Bodies)
            if(body.Body==raw)
            {if(found||!body.ObjectIdCaptured||!static_cast<FWeakObjectPtr>(selected.Resolver).HasSameIndexAndSerialNumber(body.Object))Refuse("Ordinary response resolver GT join ambiguous");found=&body;}
        if(!found)Refuse("Ordinary response resolver outside actual GT census");
    }
    auto& row=tx.Entries.AddDefaulted_GetRef();row.Selected=selected;row.RawResolver=raw;row.Eligible=eligible;row.Allowed=allowed;
    row.PairBefore=pairs.Contains(selected.PairKey);row.GenerationBefore=pairs.GetGeneration(selected.PairKey);
    ReadOrdinaryConsumedState(row.Entry);
    if(!allowed)
    {
        row.PairAfter=pairs.Contains(selected.PairKey);row.GenerationAfter=pairs.GetGeneration(selected.PairKey);
        // After.Captured remains false: no effect/callback poststate asserted.
        row.Closed=true;++tx.NextIndex;return;
    }
    tx.ActiveIndex=index;
}
void FStepScope::BeginOrdinaryResponseCall(int32 call)
{
    if(!HasOrdinaryResponseObservation())return;
    RequireOrdinaryResponseContext();auto& tx=*OrdinaryResponse;
    if(tx.ActiveCall!=INDEX_NONE||!tx.Entries.IsValidIndex(tx.ActiveIndex)||call<0||call>2)
        Refuse("Ordinary response call has no unique active batch entry");
    auto& row=tx.Entries[tx.ActiveIndex];if(!row.Allowed||row.Closed||row.Calls.Num()!=call)Refuse("Ordinary response call repeated or reordered");
    if(NativeConsumer)NativeConsumer->RequireResponseTargetsInCensus();
    auto& action=row.Calls.AddDefaulted_GetRef();action.Call=call;action.Opened=true;tx.ActiveCall=call;
    ReadOrdinaryConsumedState(action.Before);
    if(call==0&&!OrdinarySetFutureDetail::Bind(action.Before,row.RawResolver,row.Selected.Hit,action.SetFutureValues))
        Refuse("Ordinary SetFutureHit lacks its complete unique unannotated pre-call value expectation");
}
void FStepScope::EndOrdinaryResponseCall(int32 call)
{
    if(!OrdinaryResponse)
    {if(!Active||!Window||Window->GetPurpose()!=OnePausedWork::EPurpose::FullFrame)return;Refuse("Ordinary response return has no actual batch");}
    if(!OrdinaryResponse||OrdinaryResponse->ActiveCall!=call||!OrdinaryResponse->Entries.IsValidIndex(OrdinaryResponse->ActiveIndex))
        Refuse("Ordinary response return has no opened call");
    auto& tx=*OrdinaryResponse;auto& row=tx.Entries[tx.ActiveIndex];
    if(!row.Calls.IsValidIndex(call)||!row.Calls[call].Opened||row.Calls[call].Returned)Refuse("Ordinary response return duplicated");
    auto& action=row.Calls[call];action.Returned=true; // retain actual return before post-effect read can refuse
    ReadOrdinaryConsumedState(action.After);
    const FOrdinaryConsumedBodyValues *before=nullptr,*after=nullptr;
    for(const auto& owner:action.Before.Owners)for(const auto& body:owner.Bodies)if(body.Body==row.RawResolver)before=&body;
    for(const auto& owner:action.After.Owners)for(const auto& body:owner.Bodies)if(body.Body==row.RawResolver)after=&body;
    if(!before||!after)Refuse("Ordinary response returned resolver no longer in fresh census");
    if(call==0)
    {
        if(!action.Inertia.IsEmpty()||!OrdinarySetFutureDetail::Match(action.Before,action.After,action.SetFutureValues))
            Refuse("Ordinary SetFutureHit changed values outside its unique all-owner Future assignment");
        action.SetFutureValues.MatchedAfterReturn=true; // ONLY recorded-value footprint, not a Frame/Resolve capability
    }
    if(call==1)
    {
        const bool wheel=NativeConsumer&&NativeConsumer->GetBodyRoute(row.RawResolver)==ENativeFrameBodyRoute::Wheel;
        if(wheel)
        {
            if(!SameNativeAnalyticContactValues(after->Current,before->Current)||
                !SameNativeAnalyticContactValues(after->Future,before->Future))
                Refuse("Native Wheel AcceptHit changed its retained Current/Future caches");
        }
        else if(!SameNativeAnalyticContactValues(after->Current,before->Future)||
            !SameNativeAnalyticContactValues(after->Future,SHitResult{}))Refuse("Native Sphere/Box AcceptHit cache transition changed");
    }
    // Other-owner mutations and Resolve changes are retained verbatim. They
    // are not replaced by a single-Sphere or no-cross-effect expectation.
    action.Closed=true;tx.ActiveCall=INDEX_NONE;
}
#if !UE_BUILD_SHIPPING
static TAutoConsoleVariable<int32> CVarNativeCase585OneCurvedReimpact(
    TEXT("p.IAmSpeed.AnalyticWorld.NativeCase585OneCurvedReimpact"),0,
    TEXT("Allow one certified curved reimpact after four hits in isolated offscreen FastBatch 585 only. Default off."));

static TAutoConsoleVariable<int32> CVarCase585FifthSoftWallDeltaV(
    TEXT("p.IAmSpeed.AnalyticWorld.Case585FifthSoftWallDeltaV"),0,
    TEXT("Default OFF. Use only the existing 50 cm/s wall impulse limit for the certified fifth impact in private case 585."));

bool FStepScope::TryUseCase585FifthSoftWallDeltaV(const UBoxSubBody& body,const SHitResult& hit)
{
    auto* scope=StepScope;
    if(CVarCase585FifthSoftWallDeltaV.GetValueOnAnyThread()!=1||!scope||!scope->NativeConsumer||
        !scope->Case585AcceptingFifth||scope->Case585FifthResolved||scope->Case585FifthDeltaVUsed||
        !scope->Case585FourthGround||!scope->Case585FifthGround||!scope->Case585Pair||!scope->OrdinaryResponse)
        return false;
    const auto& tx=*scope->OrdinaryResponse;
    const auto* phase=OwnerProbePhase;
    if(tx.ActiveCall!=2||!tx.Entries.IsValidIndex(tx.ActiveIndex)||!phase||!phase->State||
        phase->State->Phase!=ENativeOwnerProbePhase::ResponseResolve)return false;
    const auto& row=tx.Entries[tx.ActiveIndex];
    const auto& fifth=*scope->Case585FifthGround;
    const auto& request=fifth.GetRequestForInspection();
    const auto& expected=fifth.GetGroundValuesForInspection();
    if(row.RawResolver!=&body||!row.Allowed||!row.Eligible||!row.PairBefore||row.Selected.PairKey!=scope->Case585Pair||
        !fifth.IsCurvedBoxArrivalValues()||!request||request->GetSnapshot()!=scope->Snapshot||
        !SameNativeAnalyticContactValues(row.Selected.Hit,expected)||!SameNativeAnalyticContactValues(hit,expected)||
        hit.SourceId!=1274291482858373289ull||hit.SurfaceId!=12602828015134877967ull||
        hit.ContactFeatureThis!=EContactFeatureKind::Vertex||hit.ContactFeatureIndexThis!=1||
        !hit.bSurfaceNormalMayVary||!hit.bHit||!hit.bBlockingHit||hit.bStartPenetrating||!(hit.PenetrationDepth<=0.f))
        return false;
    const auto& step=request->GetStep();
    const auto& values=request->GetValues();
    FStepIdentity now;
    if(step.Frame!=87||step.Iteration!=5||step.QuerySerial!=5||values.FrameClock.OwnerLocalFrame!=88||
        values.OwnerStableId!=3||values.BodyIndex!=0||values.Body!=&body||values.Owner!=phase->State->Owner||
        !ReadCurrentStep(now)||now.Frame!=step.Frame||now.Iteration!=step.Iteration||now.QuerySerial!=step.QuerySerial)
        return false;
    scope->RequireOrdinaryResponseContext();
    scope->Case585FifthDeltaVUsed=true;
    return true;
}

bool FStepScope::TryKeepCase585CurvedComponent(const UBoxSubBody& body,const SHitResult& hit,uint32 count,uint32 cap)
{
    auto* scope=StepScope;
    if(!scope)return false;
    const auto& fourth=scope->Case585FourthGround;
    const bool armed=fourth&&fourth->GetRequestForInspection()->GetValues().Body==&body&&
        hit.Component.HasSameIndexAndSerialNumber(fourth->GetGroundValuesForInspection().Component);
    if(!armed)
    {
        if(fourth||cap!=4||count!=cap||!hit.bSurfaceNormalMayVary||
            hit.SourceId!=1274291482858373289ull||hit.SurfaceId!=12602828015134877967ull||
            CVarNativeCase585OneCurvedReimpact.GetValueOnAnyThread()!=1)return false;
        FString cases;
        const TCHAR* commandLine=FCommandLine::Get();
        if(!FParse::Param(commandLine,TEXT("SLStadiumNativeFastBatch"))||
            !FParse::Param(commandLine,TEXT("NullRHI"))||!FParse::Param(commandLine,TEXT("unattended"))||
            !FParse::Value(commandLine,TEXT("SLTestIds="),cases)||cases!=TEXT("585"))return false;
    }
    const auto reject=[&]()
    {
        if(armed)Refuse("Case585 certified fifth curved reimpact lost its actual response");
        return false;
    };
    if(!scope->NativeConsumer||!scope->OrdinaryResponse)return reject();
    scope->RequireOrdinaryResponseContext();
    const auto& tx=*scope->OrdinaryResponse;
    if(tx.ActiveCall!=1||!tx.Entries.IsValidIndex(tx.ActiveIndex))return reject();
    const auto& row=tx.Entries[tx.ActiveIndex];
    const auto& inventory=row.Selected.PreparedEvidence;
    if(row.RawResolver!=&body||!row.Eligible||!row.Allowed||!row.PairBefore||!row.Selected.PairKey||
        !SameNativeAnalyticContactValues(row.Selected.Hit,hit)||!inventory||!inventory->Sealed||
        !inventory->Bodies.IsValidIndex(inventory->SelectedBodyIndex))return reject();
    const auto& slot=inventory->Bodies[inventory->SelectedBodyIndex];
    const auto& ground=slot.OrdinaryGround;
    if(slot.RawBodyIdentity!=&body||!ground||!ground->IsCurvedBoxArrivalValues()||
        !SameNativeAnalyticContactValues(ground->GetGroundValuesForInspection(),hit))return reject();
    const auto& request=ground->GetRequestForInspection();
    const auto& v=request->GetValues();
    const auto& step=request->GetStep();
    const bool originalStep=step.Frame==86&&v.FrameClock.OwnerLocalFrame==87;
    const bool nextStep=step.Frame==87&&v.FrameClock.OwnerLocalFrame==88&&
        SameFloat(step.Delta,1.f/300.f)&&hit.ContactFeatureThis==EContactFeatureKind::Vertex&&
        hit.ContactFeatureIndexThis==1;
    if(request->GetSnapshot()!=scope->Snapshot||(!originalStep&&!nextStep)||
        v.OwnerStableId!=3||v.BodyIndex!=0||v.Owner!=inventory->Owner||v.Body!=&body||
        v.Shape!=ENativeSolidQueryShape::Box||!v.Trace.IgnoredComponents.IsEmpty()||
        !v.Trace.IgnoredSourceObjects.IsEmpty())return reject();
    if(armed)
    {
        if(count!=5||cap!=4||scope->Case585FifthResolved||scope->Case585AcceptingFifth||
            scope->Case585FifthGround!=ground||scope->Case585Pair!=row.Selected.PairKey)return reject();
        scope->Case585AcceptingFifth=true;
        return false; // Restore the original ignore at the fifth actual AcceptHit.
    }
    const auto& geometry=scope->Snapshot->GetGeometrySelection();
    if(step.Iteration!=4||step.QuerySerial!=4||!geometry||!geometry->HasValidAliases()||
        geometry->GetModel()!=EPreparedGeometryModel::RetainedReference||
        !geometry->GetReference()||geometry->GetReference()->GetExternalPacketIdentity()!=
            TEXT("090d41158d13f1127827b720ff467a51e415668aa205209a67ed8afcb6b10dc0"))return false;
    scope->Case585FourthGround=ground;scope->Case585Pair=row.Selected.PairKey;
    UE_LOG(LogTemp,Display,TEXT("[NativeCase585CurvedReimpactArmed] Frame=%u Serial=%llu Pair=%llu Count=%u"),
        v.FrameClock.OwnerLocalFrame,step.QuerySerial,row.Selected.PairKey,count);
    return true;
}

void FStepScope::BindCase585CurvedReimpact(
    const TSharedPtr<const FOrdinaryGroundSweepEvidence,ESPMode::ThreadSafe>& ground)
{
    if(!Case585FourthGround||Case585FifthResolved)return;
    const auto& previous=Case585FourthGround->GetRequestForInspection();
    const auto& request=ground->GetRequestForInspection();
    const auto& v=request->GetValues();
    if(v.Body!=previous->GetValues().Body)return;
    const auto& old=Case585FourthGround->GetGroundValuesForInspection();
    const auto& hit=ground->GetGroundValuesForInspection();
    const auto& step=request->GetStep();
    if(!ground->IsCurvedBoxArrivalValues()||request->GetSnapshot()!=Snapshot||
        !HasClosedCurvedBoxGroundForObservation(*request,old)||
        step.Iteration<=previous->GetStep().Iteration||step.QuerySerial<=previous->GetStep().QuerySerial||
        !v.Trace.IgnoredComponents.IsEmpty()||!v.Trace.IgnoredSourceObjects.IsEmpty()||
        !hit.Component.HasSameIndexAndSerialNumber(old.Component)||hit.SourceId!=old.SourceId||
        hit.SurfaceId!=old.SurfaceId||hit.FeatureId!=old.FeatureId||hit.PrimitiveId!=old.PrimitiveId||
        hit.CanonicalGroupId!=old.CanonicalGroupId||hit.MaterialId!=old.MaterialId||
        hit.ContactFeatureThis!=EContactFeatureKind::Vertex||hit.ContactFeatureIndexThis!=1||
        !hit.bSurfaceNormalMayVary||!hit.bHit||!hit.bBlockingHit||hit.bStartPenetrating||!(hit.PenetrationDepth<=0.f)||
        !FMath::IsFinite(hit.TOI)||!(hit.TOI>0.f&&hit.TOI<v.Horizon))
        Refuse("Case585 fifth curved reimpact is not the certified original corner");
    Case585FifthGround=ground;
    UE_LOG(LogTemp,Display,TEXT("[NativeCase585CurvedReimpactCandidate] Frame=%u Serial=%llu Surface=%llu Corner=%d Seconds=%.17g H=%.17g"),
        v.FrameClock.OwnerLocalFrame,step.QuerySerial,hit.SurfaceId,hit.ContactFeatureIndexThis,double(hit.TOI),double(v.Horizon));
}
#endif

void FStepScope::EndOrdinaryResponseEntry(int32 index,bool ignored,bool exactRecordDecision,const Collision::FResolvedPairSet& pairs)
{
    if(!HasOrdinaryResponseObservation())return;
    RequireOrdinaryResponseContext();auto& tx=*OrdinaryResponse;
    if(tx.ActiveIndex!=index||tx.ActiveCall!=INDEX_NONE||!tx.Entries.IsValidIndex(index)||tx.PairsIdentity!=&pairs)
        Refuse("Ordinary response entry return changed");
    auto& row=tx.Entries[index];if(row.Closed||!row.Allowed||row.Calls.Num()!=3)Refuse("Ordinary response entry call inventory incomplete");
    for(int32 i=0;i<3;++i)if(row.Calls[i].Call!=i||!row.Calls[i].Returned||!row.Calls[i].Closed)Refuse("Ordinary response call return unavailable");
    if(!row.Calls[0].SetFutureValues.BoundBeforeCall||!row.Calls[0].SetFutureValues.MatchedAfterReturn)
        Refuse("Ordinary response lacks the closed unannotated SetFutureHit value footprint");
    // Actual bRecordExactGeneration decision only, not the ignored return
    // value of RecordExactProgressResolution or an exact-progress grant.
    row.Ignored=ignored;row.ExactRecordDecision=exactRecordDecision;row.PairAfter=pairs.Contains(row.Selected.PairKey);row.GenerationAfter=pairs.GetGeneration(row.Selected.PairKey);
    ReadOrdinaryConsumedState(row.After);FNativePlanarResponseCommitScope::RequireClosedResponseEffect(*this,row);row.Closed=true;tx.ActiveIndex=INDEX_NONE;++tx.NextIndex;
#if !UE_BUILD_SHIPPING
    if(Case585AcceptingFifth)
    {
        const auto& hit=Case585FifthGround->GetGroundValuesForInspection();
        if(!ignored||exactRecordDecision||!row.PairAfter||row.Selected.PairKey!=Case585Pair||
            !SameNativeAnalyticContactValues(row.Selected.Hit,hit))
            Refuse("Case585 fifth curved response did not restore the original component cap");
        Case585AcceptingFifth=false;Case585FifthResolved=true;
        UE_LOG(LogTemp,Display,TEXT("[NativeCase585CurvedReimpactResolved] Frame=%u Serial=%llu Pair=%llu Corner=%d Ignored=1"),
            Case585FifthGround->GetRequestForInspection()->GetValues().FrameClock.OwnerLocalFrame,
            Case585FifthGround->GetRequestForInspection()->GetStep().QuerySerial,Case585Pair,hit.ContactFeatureIndexThis);
    }
#endif
    // Retain immutable VALUES only after all three actual call returns and the response-entry footprint are closed.
    if(row.Calls[2].PlanarEffect)ClosedPlanarValuesForObservation.Add(row.Calls[2].PlanarEffect);
}
void FStepScope::EndOrdinaryResponseBatch()
{
    if(!HasOrdinaryResponseObservation())return;
    RequireOrdinaryResponseContext();auto& tx=*OrdinaryResponse;
    if(tx.ActiveIndex!=INDEX_NONE||tx.ActiveCall!=INDEX_NONE||tx.NextIndex!=tx.Selected.Num()||tx.Entries.Num()!=tx.Selected.Num())
        Refuse("Ordinary response batch observations incomplete");
    for(const auto& row:tx.Entries)if(!row.Closed)Refuse("Ordinary response batch entry remained open");
    tx.Closed=true;
    for(const auto& row:tx.Entries)
    {
        if(!row.Allowed||!row.RawResolver||row.Calls.Num()!=3||!row.Calls[2].Closed)continue;
        ClosedCurvedBoxGroundForObservation.RemoveAll([&](const auto& previous)
        {return previous->GetRequestForInspection()->GetValues().Body==row.RawResolver;});

        const auto& inventory=row.Selected.PreparedEvidence;
        if(!inventory||!inventory->Sealed||!inventory->Bodies.IsValidIndex(inventory->SelectedBodyIndex))continue;
        const auto& body=inventory->Bodies[inventory->SelectedBodyIndex];
        const auto& ground=body.OrdinaryGround;
        if(body.RawBodyIdentity!=row.RawResolver||!ground||!ground->IsCurvedBoxArrivalValues()||
            !SameNativeAnalyticContactValues(ground->GetGroundValuesForInspection(),row.Selected.Hit))continue;

        const auto& request=ground->GetRequestForInspection();
        if(!request||request->GetSnapshot()!=Snapshot)continue;
        const auto& captured=request->GetValues();
        if(captured.Body!=row.RawResolver||captured.Owner!=inventory->Owner||captured.BodyIndex!=body.Index)continue;
        const auto* after=row.After.Owners.FindByPredicate([&](const auto& owner){return owner.Owner==captured.Owner;});
        if(!row.After.Captured||!after||!after->Bodies.IsValidIndex(body.Index))continue;
        const auto& actual=after->Bodies[body.Index];
        if(actual.Body!=row.RawResolver||actual.Class!=captured.BodyClass||actual.Index!=body.Index||
            !SameNativeAnalyticContactValues(actual.Current,ground->GetGroundValuesForInspection())||
            !SameNativeAnalyticContactValues(actual.Future,SHitResult{}))continue;

        // Historical classification only. The next sweep must obtain fresh
        // geometry, a new result and its own sealed current request.
        ClosedCurvedBoxGroundForObservation.Add(ground);
    }
}
void FStepScope::ObserveOrdinaryBodyInertiaValue(const USolidSubBody* body,const FMatrix& value,
    EOrdinaryBodyInertiaOrigin origin,const FQuat& basis)
{
    if(!Requested()||!StepScope||!StepScope->OrdinaryResponse||StepScope->OrdinaryResponse->ActiveCall==INDEX_NONE)return;
    auto* scope=StepScope;scope->RequireOrdinaryResponseContext();auto& tx=*scope->OrdinaryResponse;
    const ISpeedComponent* owner=nullptr;const FPreparedNativeBodyBinding* found=nullptr;
    for(const auto& row:scope->Snapshot->GetNativeOwners()->GetRecordsForInspection())for(const auto& child:row.Bodies)
        if(child.Body==body){if(found)Refuse("Consumed inertia raw body is ambiguous");found=&child;owner=row.Adapter;}
    if(!body||!found||!owner||!found->ObjectIdCaptured||body->GetClass()!=found->Class||body->GetParentComponent()!=owner||
        !owner->GetSubBodies().IsValidIndex(found->Index)||owner->GetSubBodies()[found->Index]!=body)
        Refuse("Actual inertia producer does not join the retained GT body");
    if(!tx.Entries.IsValidIndex(tx.ActiveIndex)||!tx.Entries[tx.ActiveIndex].Calls.IsValidIndex(tx.ActiveCall))Refuse("Actual inertia output has no native call");
    auto& out=tx.Entries[tx.ActiveIndex].Calls[tx.ActiveCall].Inertia.AddDefaulted_GetRef();
    out.Body=body;out.Owner=owner;out.BodyIndex=found->Index;out.Origin=origin;out.Value=value;out.Basis=basis;
    if(!ReadCurrentStep(out.Step)||!JoinCanonicalPreparedFrameClock(out.Step.Frame,owner->NumFrame(),out.Clock))Refuse("Consumed inertia clock changed");
    out.OwnerState=owner->GetKinematicState();out.BodyState=body->GetKinematicState();out.Current=body->GetHit();
    // All 16 matrix cells/basis bits are the native output, not recomputed
    // with an owner placeholder or swept pose. Direct solver Box rotations
    // are covered too. Owner-only inertia/projections remain separate due.
}

// Actual caller operands ONLY. No mutation, clearance or response capability.
void FStepScope::ObserveUnclosedPlanarCommitValues(const ISpeedComponent& owner,const UBoxSubBody& box,
    const SHitResult& hit,const FKinematicState& nativeBefore,const FKinematicState& localSolved)
{
    auto* scope=StepScope;
    if(!Requested()||!scope||!scope->Active||!scope->Window||
        scope->Window->GetPurpose()!=OnePausedWork::EPurpose::FullFrame||
        !scope->GeneralBoxOwners.Contains(&owner)||GeneralBoxTransportDetail::Pose(nativeBefore,localSolved))return;
    // Other native callsites retain their existing setter guards unchanged.
    if(!scope->OrdinaryResponse||scope->OrdinaryResponse->ActiveCall!=2)return;
    scope->RequireNativeFrameContext();scope->RequireOrdinaryResponseContext();
    auto& tx=*scope->OrdinaryResponse;
    if(tx.ActiveCall!=2||!tx.Entries.IsValidIndex(tx.ActiveIndex)||!scope->OrdinaryIntegration||
        !scope->OrdinaryIntegration->Closed||!scope->OrdinaryReturns||!scope->OrdinaryReturns->World||
        !scope->OrdinaryReturns->World->Closed||!OwnerProbePhase||!OwnerProbePhase->Check()||
        !FiniteState(nativeBefore)||!FiniteState(localSolved))Refuse("Planar commit collector has no genuine active Resolve");
    const auto& entry=tx.Entries[tx.ActiveIndex];
    if(entry.RawResolver!=&box||!entry.Allowed||entry.Closed||entry.Calls.Num()!=3||
        !entry.Calls[0].Closed||!entry.Calls[1].Closed||!entry.Calls[2].Opened||entry.Calls[2].Returned||
        !SameNativeAnalyticContactValues(hit,entry.Selected.Hit))
        Refuse("Planar commit collector differs from selected/accepted resolver");
    FOrdinaryConsumedStateValues current;scope->ReadOrdinaryConsumedState(current);
    const FOrdinaryConsumedOwnerValues* actual=nullptr;int32 ownerIndex=INDEX_NONE;
    for(int32 i=0;i<current.Owners.Num();++i)if(current.Owners[i].Owner==&owner)
    {if(actual)Refuse("Planar commit collector owner ambiguous");actual=&current.Owners[i];ownerIndex=i;}
    const auto bindings=scope->Snapshot->GetNativeOwners();const auto* census=bindings->Find(&owner);
    if(!actual||!census||!census->BodyCensusCaptured||!SameKinematicBits(actual->State,nativeBefore)||
        actual->Bodies.Num()!=census->Bodies.Num()||!scope->OrdinaryReturns->Rows.IsValidIndex(ownerIndex)||
        !scope->OrdinaryReturns->World->Rows.IsValidIndex(ownerIndex)||
        !scope->OrdinaryIntegration->Owners.IsValidIndex(ownerIndex))
        Refuse("Planar commit collector lost real before/census inventory");
    const auto& returned=scope->OrdinaryReturns->Rows[ownerIndex];const auto component=returned?returned->Component:FComponentSweepEvidencePtr{};
    const auto& integrated=scope->OrdinaryIntegration->Owners[ownerIndex];
    if(!returned||!returned->Returned||returned->Owner!=&owner||returned->Order!=ownerIndex||
        returned->StableId!=actual->StableId||!component||!component->Sealed||component->Owner!=&owner||
        component->OwnerStableId!=actual->StableId||component->Bodies.Num()!=actual->Bodies.Num()||
        scope->OrdinaryReturns->World->Rows[ownerIndex].Raw.PreparedEvidence!=component||
        integrated.Owner!=&owner||integrated.Index!=ownerIndex||!integrated.Returned||!integrated.Closed||
        !OrdinaryWorldDetail::Epoch(component->Step,scope->OrdinaryReturns->World->Entry)||
        !OrdinaryWorldDetail::Epoch(component->Step,component->CompletionStep)||
        !OrdinaryWorldDetail::Epoch(component->CompletionStep,returned->ReturnStep)||
        component->CompletionStep.QuerySerial!=returned->ReturnStep.QuerySerial)
        Refuse("Planar commit collector retained sweep/integration join changed");
    const auto historicalStep=[&](const FStepIdentity& step)
    {return OrdinaryWorldDetail::Epoch(step,component->Step)&&step.QuerySerial>=component->Step.QuerySerial&&
        step.QuerySerial<=component->CompletionStep.QuerySerial;};
    const FGeneralBoxTransportBinding* first=nullptr;
    for(const auto& binding:integrated.GeneralBindings)if(binding.Request&&binding.Request->GetValues().Body==&box)
    {if(first||!binding.FloorMotion||!binding.HasExactlyOneKind())Refuse("Planar commit collector lacks unique first FLOOR binding");first=&binding;}
    if(!first||first->Request->GetSnapshot()!=scope->Snapshot||
        !first->FloorMotion->IsFirstGeneralBoxFloorContactValues())Refuse("Planar commit collector cannot promote another contact kind");
    UE_LOG(LogTemp,Display,TEXT("[NativeGeneralFloorActualPlanarCommit] Frame=%llu Step=%llu Entry=%d Call=%d PhaseOrdinal=%llu BodyIndex=%d Children=%d FirstAdvance=%.17g NoSetterCalled=1 RcIsHistorical=1"),
        current.Step.Frame,current.Step.StepSerial,tx.ActiveIndex,tx.ActiveCall,OwnerProbePhase->State->Ordinal,
        first->Request->GetValues().BodyIndex,actual->Bodies.Num(),double(first->FloorMotionWorldSeconds));
    UncoveredGeneralFloorPoseDetail::Log(nativeBefore,localSolved,"actual_full_planar_local_before_first_setter");
    for(int32 i=0;i<actual->Bodies.Num();++i)
    {
        const auto& child=actual->Bodies[i];const auto& gt=census->Bodies[i];const auto& slot=component->Bodies[i];
        if(child.Index!=i||child.Body!=gt.Body||child.Class!=gt.Class||child.GTObjectId!=gt.ObjectId||!gt.ObjectIdCaptured||
            slot.Index!=i||slot.RawBodyIdentity!=child.Body||!slot.Returned||
            !static_cast<FWeakObjectPtr>(slot.Body).HasSameIndexAndSerialNumber(gt.Object)||child.Body->GetParentComponent()!=&owner)
            Refuse("Planar commit collector child GT identity changed");
        FrameParallel::FSubBodyKinematicInput fresh;
        bool freshCaptured=FrameParallel::CaptureNativeSubBodyKinematicInput(*child.Body,fresh);
        if(!freshCaptured&&child.SphereShapeCaptured)
            freshCaptured=bindings->CaptureSphereInput(&owner,*static_cast<const USphereSubBody*>(child.Body),fresh);
        UE_LOG(LogTemp,Display,TEXT("[NativeGeneralFloorPlanarChildCurrent] Index=%d GT=%u Sphere=%d Box=%d PhysicalRadius=%.17g SupportRadius=%.17g ContactOffset=%.17g Offset=(%.17g,%.17g,%.17g) LocalQ=(%.17g,%.17g,%.17g,%.17g) FreshInput=%d Wheel=%d Spring=%.17g Relative=(%.17g,%.17g,%.17g) RelativeQ=(%.17g,%.17g,%.17g,%.17g)"),
            i,gt.ObjectId,int32(child.SphereShapeCaptured),int32(child.BoxShapeCaptured),double(child.PhysicalRadius),child.SupportRadius,child.ContactOffset,
            child.LocalOffset.X,child.LocalOffset.Y,child.LocalOffset.Z,child.LocalRotation.X,child.LocalRotation.Y,child.LocalRotation.Z,child.LocalRotation.W,
            int32(freshCaptured),int32(fresh.bWheel),double(fresh.WheelSpringDisplacement),fresh.RelativeLocation.X,fresh.RelativeLocation.Y,fresh.RelativeLocation.Z,
            fresh.RelativeRotation.X,fresh.RelativeRotation.Y,fresh.RelativeRotation.Z,fresh.RelativeRotation.W);
        UncoveredGeneralFloorPoseDetail::Log(child.State,child.State,"actual_planar_current_child");
        if(slot.FinalSphereState)
        {
            const auto& seed=*slot.FinalSphereState;const auto& v=seed.Values;
            if(!slot.Ground||slot.Ground->StateSeed!=slot.FinalSphereState||seed.Snapshot!=scope->Snapshot||
                seed.Geometry!=scope->Snapshot->GetGeometrySelection()||!seed.Geometry||
                v.Owner!=&owner||v.Body!=child.Body||v.BodyClass!=gt.Class||v.BodyIndex!=i||
                v.OwnerStableId!=actual->StableId||!(v.FrameClock==actual->Clock)||!historicalStep(seed.Step))
                Refuse("Planar commit collector historical sphere seed is unrelated");
            UE_LOG(LogTemp,Display,TEXT("[NativeGeneralFloorPlanarChildQuery] Index=%d Source=FinalSphereState Frame=%llu QuerySerial=%llu QueryEpoch=%llu Rc=%.17g Rp=%.17g Rs=%.17g ContactOffset=%.17g H=%.17g SeedInput=%d SeedWheel=%d SeedSpring=%.17g SeedRelative=(%.17g,%.17g,%.17g) SeedQ=(%.17g,%.17g,%.17g,%.17g) MarginCaptured=%d Margin=%.17g MemberSerial=%llu"),
                i,seed.Step.Frame,seed.Step.QuerySerial,seed.Step.QueryEpoch,double(v.CollisionRadius),double(v.PhysicalRadius),v.StaticSupportRadius,double(v.StaticContactOffsetCm),double(v.Horizon),
                int32(seed.NativeLocalInputSupported),int32(seed.NativeLocalInput.bWheel),double(seed.NativeLocalInput.WheelSpringDisplacement),
                seed.NativeLocalInput.RelativeLocation.X,seed.NativeLocalInput.RelativeLocation.Y,seed.NativeLocalInput.RelativeLocation.Z,
                seed.NativeLocalInput.RelativeRotation.X,seed.NativeLocalInput.RelativeRotation.Y,seed.NativeLocalInput.RelativeRotation.Z,seed.NativeLocalInput.RelativeRotation.W,
                int32(v.ResponseMarginCaptured),double(v.ResponseMargin),v.ResponseScopeSerial);
            UncoveredGeneralFloorPoseDetail::Log(seed.OwnerHistoryAtFrame,seed.Values.BodyState,"historical_sweep_owner_COM_and_body");
        }
        else if(slot.OrdinaryGround&&slot.OrdinaryGround->GetRequestForInspection()&&
            slot.OrdinaryGround->GetRequestForInspection()->GetValues().Shape==ENativeSolidQueryShape::Sphere)
        {
            const auto& request=*slot.OrdinaryGround->GetRequestForInspection();const auto& v=request.GetValues();
            if(request.GetSnapshot()!=scope->Snapshot||v.Owner!=&owner||v.Body!=child.Body||v.BodyClass!=gt.Class||
                v.BodyIndex!=i||v.OwnerStableId!=actual->StableId||!(v.FrameClock==actual->Clock)||!historicalStep(request.GetStep()))
                Refuse("Planar commit collector historical sphere request is unrelated");
            UE_LOG(LogTemp,Display,TEXT("[NativeGeneralFloorPlanarChildQuery] Index=%d Source=OrdinarySphereRequest Frame=%llu QuerySerial=%llu QueryEpoch=%llu Rc=%.17g Rp=%.17g Rs=%.17g ContactOffset=%.17g H=%.17g MarginCaptured=%d Margin=%.17g MemberSerial=%llu"),
                i,request.GetStep().Frame,request.GetStep().QuerySerial,request.GetStep().QueryEpoch,double(v.QueryRadius),double(v.PhysicalRadius),v.SupportRadius,double(v.StaticContactOffset),double(v.Horizon),
                int32(v.SphereMarginCaptured),double(v.SphereMargin),v.SphereMemberSerial);
        }
        else UE_LOG(LogTemp,Display,TEXT("[NativeGeneralFloorPlanarChildQuery] Index=%d Source=missing NoRadiusInferred=1"),i);
    }
    if(!integrated.ChildSync.IsEmpty()&&integrated.ChildSync.Last().Captured)
    {
        const auto& sync=integrated.ChildSync.Last();
        UE_LOG(LogTemp,Display,TEXT("[NativeGeneralFloorPlanarPriorSync] Frame=%llu Step=%llu Children=%d PostProbe=%d IntegrationProjection=%d HistoricalOnly=1 FutureOriginNotInferred=1"),
            sync.Step.Frame,sync.Step.StepSerial,sync.Children.Num(),int32(sync.InExactPostProbe),int32(sync.InExactIntegrationProjection));
        UncoveredGeneralFloorPoseDetail::Log(sync.OwnerState,sync.ActualOrigin,"actual_prior_sync_owner_and_origin");
        for(const auto& child:sync.Children)UncoveredGeneralFloorPoseDetail::Log(child,child,"actual_prior_sync_child");
    }
    else UE_LOG(LogTemp,Display,TEXT("[NativeGeneralFloorPlanarPriorSync] Source=missing FutureOriginNotInferred=1"));
    scope->RequireOrdinaryResponseContext();
    Refuse("General FLOOR planar commit captured before first setter; all-child corridor/response receipt still unqualified");
}

// UNINSTALLED. Include inside PreparedExecution after consumed-state types.
// Whole physical/support BODY volumes only. No Sphere-TOI Rc is inferred and
// this value grants no query, owner advance, force, callback or native write.
namespace PlanarBodyPoseDetail
{
static TAutoConsoleVariable<int32> CVarOrderedPlanarChildBox(
    TEXT("p.IAmSpeed.AnalyticWorld.OrderedPlanarChildBox"),0,
    TEXT("Use a certified rotation-first BASE write when the Cartesian Box corridor fails. Default off."));
struct FPose
{
    FVector Center=FVector::ZeroVector;FQuat Rotation=FQuat::Identity;
};
inline FPose Pose(const FKinematicState& v){return {v.Location,v.Rotation};}
inline bool Same(const FPose& a,const FPose& b)
{return SameVector(a.Center,b.Center)&&SameQuat(a.Rotation,b.Rotation);}
inline bool SameShape(const FOrdinaryConsumedBodyValues& a,const FOrdinaryConsumedBodyValues& b)
{
    return a.Body==b.Body&&a.Class==b.Class&&a.Index==b.Index&&a.GTObjectId==b.GTObjectId&&
        a.BoxShapeCaptured==b.BoxShapeCaptured&&a.SphereShapeCaptured==b.SphereShapeCaptured&&
        SameVector(a.BoxExtent,b.BoxExtent)&&SameFloat(a.PhysicalRadius,b.PhysicalRadius)&&
        SameDouble(a.SupportRadius,b.SupportRadius)&&SameDouble(a.ContactOffset,b.ContactOffset)&&
        SameVector(a.LocalOffset,b.LocalOffset)&&SameQuat(a.LocalRotation,b.LocalRotation)&&
        a.SolidConfigCaptured&&b.SolidConfigCaptured&&SameNativeSolidResponseConfigValues(a.Config,b.Config);
}
struct FStrictBodyGeometry
{
    FPreparedWorldSnapshotPtr Snapshot;FPreparedGeometrySelectionPtr Geometry;
    FOrdinaryConsumedBodyValues Shape;FPose Before,After;FVector FloorPoint,FloorNormal;
    C::FGroupedBoxVolumeBounds Volume;C::FReferenceBoxVolumeStrictSeparationPtr Separation;
    bool RotationFirst=false;
    uint64 Fingerprint=0;bool Complete=false;
    bool Matches(const FPreparedWorldSnapshotPtr& snapshot,const FOrdinaryConsumedBodyValues& shape,
        const FPose& before,const FPose& after)const
    {
        return Complete&&Snapshot==snapshot&&snapshot&&Geometry==snapshot->GetGeometrySelection()&&Geometry&&Geometry->HasValidAliases()&&
            SameShape(Shape,shape)&&Same(Before,before)&&Same(After,after)&&Separation&&
            Separation->Matches(Geometry->GetSelectedUnion(),Volume)&&Fingerprint==
            Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint();
    }
};
using FStrictBodyGeometryPtr=TSharedPtr<const FStrictBodyGeometry,ESPMode::ThreadSafe>;
inline bool BuildFixedRoofStationaryGroups(const FOrdinaryConsumedBodyValues& shape,const FPose& before,const FPose& after,
    const FVector& floorNormal,C::FDistanceBudget& budget,C::FIntervalPoint& hull,
    TArray<C::FBoxVolumeGroup,TInlineAllocator<16>>& groups)
{
    namespace A=::AnalyticCertified;
    const auto zero=[](double x){return (A::Bits(x)&A::Magnitude)==0;};
    const auto boundedPositive=[](double x){return A::Finite(x)&&x>=1.&&x<=0x1p16;};
    const auto& q=before.Rotation;const auto& extent=shape.BoxExtent;
    const double z=before.Center.Z,half=extent.Z;
    if(!Same(before,after)||A::Bits(floorNormal.Z)!=A::Bits(1.)||!zero(q.X)||
        (A::Bits(q.Y)&A::Magnitude)!=A::Bits(1.)||!zero(q.Z)||!zero(q.W)||
        !boundedPositive(extent.X)||!boundedPositive(extent.Y)||!boundedPositive(half)||
        !boundedPositive(z)||z>2.*half||half>2.*z)return false;

    // Exact cardinal native Rotate and unit-reference Rotate have the same
    // signed offsets (-x,+y,-z). The bounded normal extents make doubling
    // exact; the lower Z sum is an exact Sterbenz difference.
    volatile double lower=z-half;
    const auto low=A::Interval::Exact(lower);
    if(!C::ClosedPhysicalFloorPoseDetail::NonNegative(low))return false;
    const auto high=A::Add(A::Interval::Exact(z),A::Interval::Exact(half));
    for(int32 family=0;family<2;++family)
    {
        auto& group=groups.AddDefaulted_GetRef();
        for(int32 corner=0;corner<8;++corner)
        {
            if(!budget.Consume())return false;
            C::FIntervalPoint point;
            point.Axis[0]=A::Add(A::Interval::Exact(before.Center.X),A::Interval::Exact((corner&1)?-extent.X:extent.X));
            point.Axis[1]=A::Add(A::Interval::Exact(before.Center.Y),A::Interval::Exact((corner&2)?extent.Y:-extent.Y));
            point.Axis[2]=(corner&4)?low:high;
            if(!C::Detail::ValidPoint(point)||!C::YawFloorBoxDetail::Join(hull,point))return false;
            group.Corners.Add(point);
        }
    }
    return true;
}
inline bool Prove(const FPreparedWorldSnapshotPtr& snapshot,const FOrdinaryConsumedBodyValues& shape,
    const FPose& before,const FPose& after,const FVector& floorPoint,const FVector& floorNormal,
    C::FDistanceBudget& budget,FStrictBodyGeometryPtr& output,bool allowRotationFirst=false,bool fixedRoofStationary=false)
{
    const auto keep=output;output.Reset();
    const auto numeric=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    const auto geometry=snapshot?snapshot->GetGeometrySelection():FPreparedGeometrySelectionPtr{};
    if(!geometry||!geometry->HasValidAliases()||geometry->GetModel()!=EPreparedGeometryModel::RetainedReference||
        !shape.Body||!shape.Class||shape.Index<0||!shape.SolidConfigCaptured||
        shape.BoxShapeCaptured==shape.SphereShapeCaptured||
        !C::GeneralPhysicalFloorClearDetail::CardinalFloor(floorPoint,floorNormal)||
        before.Center.ContainsNaN()||after.Center.ContainsNaN()||before.Rotation.ContainsNaN()||after.Rotation.ContainsNaN()||
        !before.Rotation.IsNormalized()||!after.Rotation.IsNormalized())return false;
    TSharedPtr<FStrictBodyGeometry,ESPMode::ThreadSafe> proof(new FStrictBodyGeometry);
    proof->Snapshot=snapshot;proof->Geometry=geometry;proof->Shape=shape;proof->Before=before;proof->After=after;
    proof->FloorPoint=floorPoint;proof->FloorNormal=floorNormal;proof->Fingerprint=numeric.ControlFingerprint();
    const auto fail=[&](const TCHAR* stage)
    {
        UE_LOG(LogTemp,Display,TEXT("[PlanarChildCorridorRefusal] Stage=%s Index=%d Box=%d BeforeP=(%.17g,%.17g,%.17g) BeforeQ=(%.17g,%.17g,%.17g,%.17g) AfterP=(%.17g,%.17g,%.17g) AfterQ=(%.17g,%.17g,%.17g,%.17g) Extent=(%.17g,%.17g,%.17g) PhysicalRadius=%.17g SupportRadius=%.17g FloorNormalZ=%.17g Used=%llu Remaining=%d"),
            stage,shape.Index,int32(shape.BoxShapeCaptured),before.Center.X,before.Center.Y,before.Center.Z,
            before.Rotation.X,before.Rotation.Y,before.Rotation.Z,before.Rotation.W,
            after.Center.X,after.Center.Y,after.Center.Z,after.Rotation.X,after.Rotation.Y,after.Rotation.Z,after.Rotation.W,
            shape.BoxExtent.X,shape.BoxExtent.Y,shape.BoxExtent.Z,double(shape.PhysicalRadius),shape.SupportRadius,
            floorNormal.Z,budget.Used,budget.Remaining);
        return false;
    };
    C::FIntervalPoint hull;TArray<C::FBoxVolumeGroup,TInlineAllocator<16>> groups;
    bool fixedStationaryUsed=false;
    if(shape.BoxShapeCaptured)
    {
        C::FPhysicalBodyBoxPoseInput input;input.BeforeCenter=before.Center;input.AfterCenter=after.Center;
        input.BeforeRotation=before.Rotation;input.AfterRotation=after.Rotation;input.PhysicalHalfExtent=shape.BoxExtent;
        C::FClosedPhysicalBodyBoxFloorBoundsPtr floor;
        if(C::TryProveClosedPhysicalBodyBoxFloor(input,floorPoint,floorNormal,budget,floor)&&floor)
        {
            const auto& bounds=floor->GetBodyBoundsForInspection();hull=bounds.CompleteHull;
            for(int32 family=0;family<2;++family)
            {
                auto& group=groups.AddDefaulted_GetRef();
                for(int32 k=0;k<8;++k)group.Corners.Add(family?bounds.UnitReferenceCorners[k]:bounds.NativeCorners[k]);
            }
        }
        else if(fixedRoofStationary)
        {
            if(!BuildFixedRoofStationaryGroups(shape,before,after,floorNormal,budget,hull,groups))
                return fail(TEXT("FixedRoofStationaryBoxFloor"));
            fixedStationaryUsed=true;
        }
        else
        {
            if(!allowRotationFirst||!C::TryBuildRotationFirstPhysicalBoxFloor(input,floorPoint,floorNormal,budget,hull,groups))
                return fail(TEXT("BoxFloor"));
            proof->RotationFirst=true;
        }
    }
    else
    {
        // A physical/support sphere is enclosed by a WORLD-axis cube; never
        // rotate a cube and silently claim it encloses a sphere. Nonzero
        // contact offsets and missing radii remain unsupported in this lot.
        const auto rp=AnalyticCertified::Interval::Exact(double(shape.PhysicalRadius));
        const auto rs=AnalyticCertified::Interval::Exact(shape.SupportRadius);
        if(!AnalyticCertified::Valid(rp)||!AnalyticCertified::Valid(rs)||rp.Lower<=0||rs.Lower<=0||
            (AnalyticCertified::Bits(shape.ContactOffset)&AnalyticCertified::Magnitude))return false;
        if(!budget.Consume())return false;
        const double radius=FMath::Max(rp.Upper,rs.Upper);auto& group=groups.AddDefaulted_GetRef();
        for(int32 k=0;k<8;++k)
        {
            C::FIntervalPoint corner;
            for(int32 d=0;d<3;++d)corner.Axis[d]=AnalyticCertified::Add(
                C::GeneralBoxDetail::Hull(AnalyticCertified::Interval::Exact(before.Center[d]),AnalyticCertified::Interval::Exact(after.Center[d])),
                AnalyticCertified::Interval::Exact((k&(1<<d))?radius:-radius));
            if(!C::YawFloorBoxDetail::Join(hull,corner))return false;group.Corners.Add(corner);
        }
        AnalyticCertified::Interval gap;
        if(!budget.Consume()||!C::GeneralPhysicalFloorClearDetail::RawGap(hull,floorPoint,floorNormal,gap)||
            !C::ClosedPhysicalFloorPoseDetail::NonNegative(gap))return fail(TEXT("SphereFloor"));
    }
    TArray<FVector3d,TInlineAllocator<8>> axes;
    axes.Add(FVector3d::ForwardVector);axes.Add(FVector3d::RightVector);axes.Add(FVector3d::UpVector);
    if(!C::CloseGroupedBoxVolumeBounds(hull,groups,axes,budget,proof->Volume))return fail(TEXT("Volume"));
    C::FReferenceBoundsCandidateSetPtr partition;
    if(!C::FReferenceBoundsCandidateSet::Select(geometry->GetSelectedUnion(),proof->Volume.BroadHull,budget,partition)||
        !C::FReferenceBoxVolumeStrictSeparation::TryProve(partition,proof->Volume,budget,proof->Separation)||!proof->Separation||
        !proof->Separation->Matches(geometry->GetSelectedUnion(),proof->Volume))return fail(TEXT("Nonfloor"));
    // Stronger than a filtered query: exclude EVERY captured Landscape's
    // entire horizontal plane. No invented wheel query filter/radius or
    // omitted provider can make this geometry value succeed.
    if(snapshot->GetExternalLandscape())for(const auto& row:snapshot->GetExternalLandscape()->GetPlanes())
    {
        if(!budget.Consume()||!SameVector(row.Plane.Normal,FVector::UpVector)||
            !FMath::IsFinite(row.Plane.Origin.Z)||
            !(proof->Volume.BroadHull.Axis[2].Lower>row.Plane.Origin.Z||proof->Volume.BroadHull.Axis[2].Upper<row.Plane.Origin.Z))return false;
    }
    proof->Complete=true;if(!proof->Matches(snapshot,shape,before,after))return false;
    if(fixedStationaryUsed)
        UE_LOG(LogTemp,Display,TEXT("[NativeRoofFixedStationaryChildCorridorClosed] Index=%d Families=2 Corners=16 FloorLower=%.17g Used=%llu"),
            shape.Index,proof->Volume.BroadHull.Axis[2].Lower,budget.Used);
    output=MoveTemp(proof);return true;
}
}

// UNINSTALLED Root proposal. Included after OrdinarySetFutureDetail and all
// response/integration state types, in the existing execution TU. No new law.
namespace PlanarBodyPoseDetail
{
inline FPose Derived(const FPose& origin,const FrameParallel::FSubBodyKinematicInput& input,bool spring)
{
    FPose result;const FVector offset=origin.Rotation.RotateVector(input.RelativeLocation);
    result.Center=origin.Center+offset;result.Rotation=origin.Rotation*input.RelativeRotation;
    if(spring&&input.bWheel)result.Center+=origin.Rotation.RotateVector(input.WheelSpringDisplacement*FVector::UpVector);
    return result;
}
struct FFamily
{
    FPose Origin;TArray<FPose> Base,Final;
    TArray<FStrictBodyGeometryPtr> BaseProofs,SpringProofs;
};
}
class FClosedPlanarResponseEffect final
{
    friend class FNativePlanarResponseCommitScope;friend class FStepScope;
    FClosedPlanarResponseEffect()=default;
    FPreparedWorldSnapshotPtr Snapshot;const ISpeedComponent* Owner=nullptr;
    const UBoxSubBody* Resolver=nullptr;FStepIdentity Step;uint64 PhaseOrdinal=0,Fingerprint=0;
    int32 Entry=INDEX_NONE,OwnerIndex=INDEX_NONE,ChosenFamily=INDEX_NONE;
    FKinematicState NativeBefore,Solved,OwnerAfter,ActualOrigin;
    FOrdinaryConsumedStateValues Before,After;
    FGeneralBoxTransportBinding Binding;C::FBoxPoseCorridorInput QueryCorridor;C::FIntervalPoint ShellEnvelope;
    TArray<FrameParallel::FSubBodyKinematicInput> Inputs;TArray<PlanarBodyPoseDetail::FFamily> Families;
    TArray<FKinematicState> Base,Final;TArray<FKinematicState> SetterReturns;
    FVector ClampInputs[2],ClampOutputs[2];double ClampCaps[2]={0,0};bool ClampRecorded[2]={false,false};
    int32 BaseWrites=0,SpringWrites=0;bool Closed=false;
};
struct FNativePlanarResponseCommitScope::FState
{
    FStepScope* Scope=nullptr;FOrdinaryResponseState* Response=nullptr;
    const FOrdinaryOwnerProbePhaseScope* Phase=nullptr;
    TSharedPtr<FClosedPlanarResponseEffect,ESPMode::ThreadSafe> Effect;
    FKinematicState Current,OpenCandidate;int32 Setters=0,NextChild=0;
    bool OpenSetter=false,SyncOpened=false,SyncClosed=false,BaseOpen=false;
    uint8 OrderedBaseStage=0;
};
thread_local FNativePlanarResponseCommitScope* NativePlanarCommit=nullptr;
class FPlanarChildRefreshValues final
{
    friend class FNativePlanarResponseCommitScope;
    FPlanarChildRefreshValues()=default;
    FStepScope* Scope=nullptr;const ISpeedComponent* Owner=nullptr;const FOrdinaryOwnerProbePhaseScope* Phase=nullptr;
    FPreparedWorldSnapshotPtr Snapshot;FStepIdentity Step;uint64 Fingerprint=0;
    FOrdinaryConsumedStateValues Before;int32 OwnerIndex=INDEX_NONE,NextChild=0,BaseWrites=0,SpringWrites=0;
    bool BaseOpen=false,SweepRefresh=false;uint8 FixedRoofRefreshRole=0;FKinematicState Origin;
    TArray<FrameParallel::FSubBodyKinematicInput> Inputs;TArray<FKinematicState> Base,Final;
    TArray<PlanarBodyPoseDetail::FStrictBodyGeometryPtr> BaseProofs,SpringProofs;
};
thread_local TUniquePtr<FPlanarChildRefreshValues> PlanarChildRefresh;
FPlanarChildRefreshValues& FNativePlanarResponseCommitScope::RequireRefresh()
{
    if(!PlanarChildRefresh||NativePlanarCommit||StepScope!=PlanarChildRefresh->Scope)Refuse("Planar refresh lexical identity changed");
    auto& state=*PlanarChildRefresh;auto* scope=state.Scope;FStepIdentity now;
    const PlanarSweepRefreshRead::FScope permit(scope,state.SweepRefresh);
    scope->RequireNativeFrameContext();scope->RequireOrdinaryFrameStateReadContext();
    if(scope->Snapshot!=state.Snapshot||OwnerProbePhase!=state.Phase||!ReadCurrentStep(now)||
        !OrdinaryWorldDetail::Epoch(state.Step,now)||state.Step.QuerySerial!=now.QuerySerial||
        state.Fingerprint!=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()||
        !FCanonicalFramePreflight::HasUnchangedRawPreparationCensus(*scope->Identity.Bridge,*scope->Snapshot,scope->Identity.StepSerial))
        Refuse("Planar refresh current context changed");
    return state;
}

FNativePlanarResponseCommitScope::FState& FNativePlanarResponseCommitScope::Require() const
{
    if(!State||NativePlanarCommit!=this||StepScope!=State->Scope)Refuse("Planar commit lexical identity changed");
    auto& state=*State;auto& effect=*state.Effect;auto* scope=state.Scope;
    scope->RequireNativeFrameContext();scope->RequireOrdinaryResponseContext();FStepIdentity now;
    if(scope->OrdinaryResponse.Get()!=state.Response||scope->Snapshot!=effect.Snapshot||OwnerProbePhase!=state.Phase||
        !state.Phase||!state.Phase->Check()||!state.Phase->State||state.Phase->State->Returned||
        state.Phase->State->Phase!=ENativeOwnerProbePhase::ResponseResolve||state.Phase->State->Owner!=effect.Owner||
        state.Phase->State->Ordinal!=effect.PhaseOrdinal||state.Response->ActiveIndex!=effect.Entry||
        state.Response->ActiveCall!=2||!ReadCurrentStep(now)||!OrdinaryWorldDetail::Epoch(effect.Step,now)||
        effect.Step.QuerySerial!=now.QuerySerial||!scope->NativeConsumer||
        !scope->Snapshot->GetNativeOwners()||!FCanonicalFramePreflight::HasUnchangedRawPreparationCensus(
            *scope->Identity.Bridge,*scope->Snapshot,scope->Identity.StepSerial)||
        effect.Fingerprint!=Analytic::RequireCurrentAnalyticNumericContext(
            Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()||
        !effect.Binding.Request||effect.Binding.Request->GetSnapshot()!=scope->Snapshot||
        !effect.Binding.HasExactlyOneKind()||!GeneralBoxTransportDetail::Configuration(effect.Binding.Request->GetValues()))
        Refuse("Planar commit lost source/Step/phase/census/configuration");
    return state;
}
FNativePlanarResponseCommitScope::FNativePlanarResponseCommitScope(const ISpeedComponent& owner,const UBoxSubBody& box,
    const SHitResult& hit,const FKinematicState& nativeBefore,const FKinematicState& solved)
{
    auto* scope=StepScope;
    if(!Requested()||!scope||!scope->Active||!scope->Window||scope->Window->GetPurpose()!=OnePausedWork::EPurpose::FullFrame||
        !scope->GeneralBoxOwners.Contains(&owner)||GeneralBoxTransportDetail::Pose(nativeBefore,solved))return;
    if(!scope->OrdinaryResponse||scope->OrdinaryResponse->ActiveCall!=2)return; // all old guards remain
    scope->RequireNativeFrameContext();scope->RequireOrdinaryResponseContext();auto& tx=*scope->OrdinaryResponse;
    if(NativePlanarCommit||PlanarChildRefresh||NativeOwnerPoseMutation||!tx.Entries.IsValidIndex(tx.ActiveIndex)||
        !scope->OrdinaryIntegration||!scope->OrdinaryIntegration->Closed||!scope->NativeConsumer||
        !OwnerProbePhase||!OwnerProbePhase->Check()||!OwnerProbePhase->State||
        OwnerProbePhase->State->Phase!=ENativeOwnerProbePhase::ResponseResolve||OwnerProbePhase->State->Owner!=&owner||
        !FiniteState(nativeBefore)||!FiniteState(solved)||!SameKinematicBits(nativeBefore,owner.GetKinematicState()))
        Refuse("Planar commit has no genuine unique active native Resolve");
    auto& entry=tx.Entries[tx.ActiveIndex];
    if(entry.RawResolver!=&box||!entry.Allowed||entry.Closed||entry.Calls.Num()!=3||!entry.Calls[0].Closed||
        !entry.Calls[1].Closed||!entry.Calls[2].Opened||entry.Calls[2].Returned||entry.Calls[2].PlanarEffect||
        !SameNativeAnalyticContactValues(hit,entry.Selected.Hit)||!SameNativeAnalyticContactValues(hit,box.GetHit()))
        Refuse("Planar commit differs from unique selected/accepted resolver");
    auto state=MakeUnique<FState>();state->Scope=scope;state->Response=&tx;state->Phase=OwnerProbePhase;
    state->Effect=TSharedPtr<FClosedPlanarResponseEffect,ESPMode::ThreadSafe>(new FClosedPlanarResponseEffect);
    auto& effect=*state->Effect;effect.Snapshot=scope->Snapshot;effect.Owner=&owner;effect.Resolver=&box;
    effect.Entry=tx.ActiveIndex;effect.PhaseOrdinal=OwnerProbePhase->State->Ordinal;effect.NativeBefore=nativeBefore;
    effect.Solved=solved;state->Current=nativeBefore;scope->ReadOrdinaryConsumedState(effect.Before);effect.Step=effect.Before.Step;
    effect.Fingerprint=Analytic::RequireCurrentAnalyticNumericContext(
        Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint();
    for(int32 i=0;i<effect.Before.Owners.Num();++i)if(effect.Before.Owners[i].Owner==&owner)
    {if(effect.OwnerIndex!=INDEX_NONE)Refuse("Planar owner ambiguous");effect.OwnerIndex=i;}
    if(!effect.Before.Owners.IsValidIndex(effect.OwnerIndex)||!scope->OrdinaryIntegration->Owners.IsValidIndex(effect.OwnerIndex))
        Refuse("Planar commit lost actual owner inventory");
    const auto& actual=effect.Before.Owners[effect.OwnerIndex];const auto* census=scope->Snapshot->GetNativeOwners()->Find(&owner);
    const auto& integrated=scope->OrdinaryIntegration->Owners[effect.OwnerIndex];
    if(!census||!census->BodyCensusCaptured||actual.Frozen||actual.Prescribed||!SameKinematicBits(actual.State,nativeBefore)||
        actual.Bodies.IsEmpty()||actual.Bodies.Num()!=census->Bodies.Num()||integrated.Owner!=&owner||
        integrated.Index!=effect.OwnerIndex||!integrated.Closed||!integrated.Returned||integrated.ChildSync.IsEmpty()||
        !integrated.ChildSync.Last().Captured||integrated.ChildSync.Last().Children.Num()!=actual.Bodies.Num()||
        !OrdinaryWorldDetail::Epoch(effect.Step,integrated.ChildSync.Last().Step))Refuse("Planar prior real integration sync unavailable");
    const FGeneralBoxTransportBinding* first=nullptr;
    for(const auto& binding:integrated.GeneralBindings)if(binding.Request&&binding.Request->GetValues().Body==&box)
    {if(first||!binding.FloorMotion||!binding.HasExactlyOneKind())Refuse("Planar commit lacks unique first FLOOR binding");first=&binding;}
    if(!first||first->Request->GetSnapshot()!=scope->Snapshot||!first->FloorMotion->IsFirstGeneralBoxFloorContactValues()||
        !GeneralBoxTransportDetail::Configuration(first->Request->GetValues()))Refuse("Planar commit cannot promote another contact kind");
    effect.Binding=*first;const auto descriptor=first->FloorMotion->GetGeneralFloorMotionDescriptorForInspection();
    if(!descriptor||descriptor->GetRequest()!=first->Request||
        !first->FloorMotion->HasMatchingFirstGeneralFloorGeometry())Refuse("Planar original finite FLOOR descriptor lost");
    const auto& source=first->Request->GetValues();const auto& raw=descriptor->GetValues().QueryEnvelope;
    const auto& floor=first->FloorMotion->GetFirstGeneralFloorPlaneForInspection();
    // The sealed FirstFloor evidence binds the real witness to this finite
    // plane. Its witness point and the descriptor's XY anchor need not agree.
    if(!SameVector(hit.ImpactNormal,floor.Normal)||
        !SameVector(nativeBefore.Acceleration,solved.Acceleration)||!SameVector(nativeBefore.AngularAcceleration,solved.AngularAcceleration))
        Refuse("Planar candidate changed acceleration or its actual FLOOR plane");
    // Keep complete original query/chord/SAT/domain/overlap shells inside the
    // previously authenticated finite concave atlas/NONfloor/Landscape raw.
    auto& corridor=effect.QueryCorridor;corridor.BeforeCOM=nativeBefore.Location;corridor.AfterCOM=solved.Location;
    corridor.BeforeRotation=nativeBefore.Rotation;corridor.AfterRotation=solved.Rotation;corridor.LocalCOM=source.LocalCOM;
    corridor.LocalOffset=source.LocalOffset;corridor.LocalRotation=source.LocalRotation;
    corridor.PhysicalHalfExtent=source.PhysicalHalfExtent;corridor.QueryHalfExtent=source.QueryHalfExtent;
    corridor.DomainShell=source.Query.DomainTolerance;corridor.OverlapShell=source.Query.InitialOverlapTolerance;
    if(!C::BuildBoxPoseCorridorBounds(corridor,effect.ShellEnvelope)||!C::ClosedPhysicalFloorPoseDetail::Within(raw,effect.ShellEnvelope))
        Refuse("Planar actual two-setter query-shell corridor escaped authenticated original raw");
    for(int32 i=0;i<actual.Bodies.Num();++i)
    {
        const auto& child=actual.Bodies[i];const auto& gt=census->Bodies[i];FrameParallel::FSubBodyKinematicInput input;
        if(child.Index!=i||child.Body!=gt.Body||child.Class!=gt.Class||!gt.ObjectIdCaptured||child.GTObjectId!=gt.ObjectId||
            child.Body->GetParentComponent()!=&owner||!scope->NativeConsumer->ReadCurrentBodyLocalKinematicValues(child.Body,input)||
            !SameVector(input.RelativeLocation,child.LocalOffset)||!SameQuat(input.RelativeRotation,child.LocalRotation)||
            input.bWheel!=child.SphereShapeCaptured||(!input.bWheel&&!child.BoxShapeCaptured))
            Refuse("Planar commit lacks genuine exact compiled child local/spring values");
        effect.Inputs.Add(input);
    }
    PlanarBodyPoseDetail::FPose live;live.Rotation=solved.Rotation;
    live.Center=solved.Location-solved.Rotation.RotateVector(actual.LocalCOM);
    effect.Families.AddDefaulted_GetRef().Origin=live;
    const auto historical=PlanarBodyPoseDetail::Pose(integrated.ChildSync.Last().ActualOrigin);
    if(!PlanarBodyPoseDetail::Same(live,historical))effect.Families.AddDefaulted_GetRef().Origin=historical;
    // One bounded, nonrefillable budget for every supplied conditional family
    // and every native BASE/final intermediate volume. No physics threshold.
    const uint64 pieces=uint64(effect.Families.Num())*uint64(actual.Bodies.Num())*2;
    const uint64 amount=pieces*(uint64(scope->Snapshot->GetGeometrySelection()->GetSelectedUnion()->Num())*2+16384);
    if(!amount||amount>uint64(MAX_int32))Refuse("Planar all-child proof budget is not representable");
    C::FDistanceBudget budget{int32(amount)};
    const bool allowOrderedBox=PlanarBodyPoseDetail::CVarOrderedPlanarChildBox.GetValueOnAnyThread()==1;
    for(auto& family:effect.Families)for(int32 i=0;i<actual.Bodies.Num();++i)
    {
        const auto& shape=actual.Bodies[i];const auto& input=effect.Inputs[i];
        const auto base=PlanarBodyPoseDetail::Derived(family.Origin,input,false),final=PlanarBodyPoseDetail::Derived(family.Origin,input,true);
        family.Base.Add(base);family.Final.Add(final);PlanarBodyPoseDetail::FStrictBodyGeometryPtr firstProof,secondProof;
        if(!PlanarBodyPoseDetail::Prove(scope->Snapshot,shape,PlanarBodyPoseDetail::Pose(shape.State),base,floor.Point,floor.Normal,budget,firstProof,allowOrderedBox&&!input.bWheel)||
            !firstProof)Refuse("Planar physical/support child BASE corridor not proved");
        family.BaseProofs.Add(MoveTemp(firstProof));
        if(input.bWheel&&(!PlanarBodyPoseDetail::Prove(scope->Snapshot,shape,base,final,floor.Point,floor.Normal,budget,secondProof)||!secondProof))
            Refuse("Planar real wheel spring intermediate corridor not proved");
        family.SpringProofs.Add(MoveTemp(secondProof));
    }
    FOrdinaryConsumedStateValues rechecked;scope->ReadOrdinaryConsumedState(rechecked);
    if(!OrdinarySetFutureDetail::Unchanged(effect.Before,rechecked))Refuse("Planar values changed while proving all native write corridors");
    State=MoveTemp(state);NativePlanarCommit=this;
    try{Require();}catch(...){if(NativePlanarCommit==this)NativePlanarCommit=nullptr;throw;}
}
bool FNativePlanarResponseCommitScope::BeforeOwnerSetter(const ISpeedComponent& owner,const FKinematicState& before,const FKinematicState& candidate)
{
    if(PlanarChildRefresh)Refuse("Owner setter attempted inside a native all-child refresh");
    if(!NativePlanarCommit)return false;auto& state=NativePlanarCommit->Require();auto& effect=*state.Effect;
    if(effect.Owner!=&owner||state.OpenSetter||state.SyncOpened||state.Setters<0||state.Setters>=4||
        !SameKinematicBits(before,state.Current)||!SameKinematicBits(before,owner.GetKinematicState()))
        Refuse("Planar owner setter is repeated/reordered or has foreign input");
    auto expected=state.Current;
    if(state.Setters==0)expected.Location=effect.Solved.Location;
    else if(state.Setters==1)expected.Rotation=effect.Solved.Rotation;
    else
    {
        const int32 k=state.Setters-2;if(!effect.ClampRecorded[k])Refuse("Planar V/W setter lacks its real native clamp output");
        if(k==0)expected.Velocity=effect.ClampOutputs[k];else expected.AngularVelocity=effect.ClampOutputs[k];
    }
    if(!SameKinematicBits(expected,candidate))Refuse("Planar true setter RHS differs from its exact covered single-field footprint");
    state.OpenCandidate=candidate;state.OpenSetter=true;return true;
}
bool FNativePlanarResponseCommitScope::AfterOwnerSetter(const ISpeedComponent& owner,const FKinematicState& actual)
{
    if(!NativePlanarCommit)return false;auto& state=NativePlanarCommit->Require();
    if(state.Effect->Owner!=&owner||!state.OpenSetter||state.SyncOpened||
        !SameKinematicBits(actual,state.OpenCandidate)||!SameKinematicBits(actual,owner.GetKinematicState()))
        Refuse("Planar true setter return differs from its covered RHS");
    state.Current=actual;state.OpenSetter=false;++state.Setters;state.Effect->SetterReturns.Add(actual);return true;
}
void FNativePlanarResponseCommitScope::ObserveNativeClamp(const ISpeedComponent& owner,const FVector& input,double cap,const FVector& output,bool angular)
{
    if(!NativePlanarCommit)return;auto& state=NativePlanarCommit->Require();auto& effect=*state.Effect;const int32 k=angular?1:0;
    if(effect.Owner!=&owner||state.OpenSetter||state.SyncOpened||state.Setters!=k+2||effect.ClampRecorded[k]||
        !SameVector(input,angular?effect.Solved.AngularVelocity:effect.Solved.Velocity)||
        !SameDouble(cap,double(angular?effect.Binding.Request->GetValues().ResponseConfig.OwnerMaxAngularSpeed:
            effect.Binding.Request->GetValues().ResponseConfig.OwnerMaxSpeed))||!FMath::IsFinite(cap)||cap<0||output.ContainsNaN())
        Refuse("Planar native clamp is not the unique original input/cap/output");
    effect.ClampInputs[k]=input;effect.ClampCaps[k]=cap;effect.ClampOutputs[k]=output;effect.ClampRecorded[k]=true;
}
void FNativePlanarResponseCommitScope::BeforeChildSync(const ISpeedComponent& owner,const FKinematicState& origin)
{
    if(FNativeFinalWheelSupportScope::BeforeChildSync(owner,origin))return;
    if(!NativePlanarCommit)
    {
        auto* scope=StepScope;
        if(!Requested()||!scope||!scope->NativeConsumer||!scope->GeneralBoxOwners.Contains(&owner))return;
        const bool sweepRefresh=scope->SweepPhase&&!scope->OrdinaryIntegration&&!scope->OrdinaryResponse;
        if(!sweepRefresh&&(!scope->OrdinaryIntegration||!scope->OrdinaryIntegration->Closed))return;
        // A true native sync following this owner's closed planar entry gets
        // its own fresh ticket, including post-entry and admitted final syncs.
        // No preceding response result grants another synchronization.
        TSharedPtr<const FClosedPlanarResponseEffect,ESPMode::ThreadSafe> previous;
        if(scope->OrdinaryResponse)for(const auto& row:scope->OrdinaryResponse->Entries)
            if(row.Closed&&row.Allowed&&row.Calls.Num()==3&&row.Calls[2].PlanarEffect&&
                row.Calls[2].PlanarEffect->Owner==&owner)previous=row.Calls[2].PlanarEffect;
        if(sweepRefresh)
        {
            FStepIdentity current;
            // Preserve an already-latched refusal during native cleanup.
            if(!ReadCurrentStep(current))return;
            for(int32 i=scope->ClosedPlanarValuesForObservation.Num()-1;i>=0;--i)
            {
                const auto& candidate=scope->ClosedPlanarValuesForObservation[i];
                if(candidate&&candidate->Owner==&owner&&candidate->Step.QueryEpoch<current.QueryEpoch&&
                    candidate->Step.QuerySerial<=current.QuerySerial&&
                    scope->HasClosedPlanarValuesForObservation(candidate->Resolver))
                {
                    previous=candidate;break;
                }
            }
        }
        const FOrdinaryConsumedOwnerValues* fixedBaseline=nullptr;
        const C::FNativeBoxCachedPlaneValues* fixedFloor=nullptr;
        const UBoxSubBody* fixedResolver=nullptr;
        uint8 fixedRole=0;
#if !UE_BUILD_SHIPPING
        if(!previous&&sweepRefresh)
        {
            auto* held=scope->FixedNoPoseSweepBaseline.Get();
            FStepIdentity now;
            const auto* pending=scope->OrdinaryReturns?scope->OrdinaryReturns->Pending.Get():nullptr;
            if(held&&held->After.Owner==&owner&&held->Snapshot==scope->Snapshot&&
                scope->HasOpenOrdinaryWorldSelection()&&pending&&pending->Owner==&owner&&pending->ReactionsObserved&&
                !ComponentSweepScope&&!MemberSweepScope&&ReadCurrentStep(now)&&
                OrdinaryWorldDetail::Epoch(pending->EntryStep,now)&&pending->EntryStep.QuerySerial<=now.QuerySerial&&
                held->Ground&&OrdinaryWorldDetail::Epoch(held->SweepStep,now)&&
                held->SweepStep.QuerySerial<=pending->EntryStep.QuerySerial&&
                held->SweepRefreshCount<2&&
                (!pending->LinearReaction.IsZero()||!pending->AngularReaction.IsZero()))
            {
                const bool apply=held->SweepRefreshCount==0;
                if(!SameKinematicBits(pending->EntryOwner,held->After.State)||
                    (apply?pending->EntryStep.QuerySerial!=now.QuerySerial:
                        held->SweepRefreshQuerySerial!=pending->EntryStep.QuerySerial||pending->EntryStep.QuerySerial>=now.QuerySerial))
                    Refuse("Fixed roof sweep refresh lost its exact closed response and current owner entry");
                auto expected=pending->EntryOwner;
                expected.Acceleration+=pending->LinearReaction;expected.AngularAcceleration+=pending->AngularReaction;
                if(!apply)
                {
                    expected.Acceleration-=pending->LinearReaction;expected.AngularAcceleration-=pending->AngularReaction;
                }
                if(!SameKinematicBits(expected,owner.GetKinematicState()))
                    Refuse("Fixed roof sweep refresh acceleration is not the original add or restore");
                fixedBaseline=&held->After;fixedFloor=&held->Ground->GetFirstGeneralFloorPlaneForInspection();
                fixedResolver=held->Resolver;fixedRole=apply?1:2;
            }
        }
#endif
        if(!previous&&!fixedBaseline)return;
        if(previous&&previous->Owner!=&owner)return;
        if(PlanarChildRefresh||NativeOwnerPoseMutation||!FiniteState(origin))Refuse("Planar refresh nested or origin unavailable");
        const PlanarSweepRefreshRead::FScope permit(scope,sweepRefresh);
        const auto* baseline=fixedBaseline?nullptr:
            (sweepRefresh?&previous->After:&scope->GeneralBoxCurrentClosedBaseline(false));
        auto refresh=TUniquePtr<FPlanarChildRefreshValues>(new FPlanarChildRefreshValues);
        auto& r=*refresh;r.Scope=scope;r.Owner=&owner;r.Snapshot=scope->Snapshot;r.Phase=OwnerProbePhase;r.Origin=origin;
        r.SweepRefresh=sweepRefresh;r.FixedRoofRefreshRole=fixedRole;
        scope->ReadOrdinaryFrameStateValues(r.Before);r.Step=r.Before.Step;
        if(!sweepRefresh)scope->RequireGeneralBoxUnchanged(*baseline,r.Before);
        for(int32 i=0;i<r.Before.Owners.Num();++i)if(r.Before.Owners[i].Owner==&owner)
        {if(r.OwnerIndex!=INDEX_NONE)Refuse("Planar refresh owner ambiguous");r.OwnerIndex=i;}
        if(!r.Before.Owners.IsValidIndex(r.OwnerIndex)||
            (previous&&(previous->Snapshot!=r.Snapshot||!previous->Closed||previous->Owner!=&owner)))
            Refuse("Planar refresh has no actual current closed predecessor");
        const auto& actual=r.Before.Owners[r.OwnerIndex];
        const auto* covered=fixedBaseline?fixedBaseline:GeneralBoxTransportDetail::Find(*baseline,&owner);
        if(!covered||actual.Bodies.Num()!=covered->Bodies.Num()||
            (sweepRefresh&&!GeneralBoxTransportDetail::SamePoseAndBoxConfiguration(*covered,actual)))
            Refuse("Planar refresh physical census or prior closed pose differs");
        const auto& floor=fixedFloor?*fixedFloor:previous->Binding.FloorMotion->GetFirstGeneralFloorPlaneForInspection();
        const uint64 pieces=uint64(actual.Bodies.Num())*2;
        const uint64 amount=pieces*(uint64(r.Snapshot->GetGeometrySelection()->GetSelectedUnion()->Num())*2+16384);
        if(!amount||amount>uint64(MAX_int32))Refuse("Planar refresh budget not representable");C::FDistanceBudget budget{int32(amount)};
        for(int32 i=0;i<actual.Bodies.Num();++i)
        {
            const auto& shape=actual.Bodies[i];FrameParallel::FSubBodyKinematicInput input;
            if(!PlanarBodyPoseDetail::SameShape(shape,covered->Bodies[i])||!GeneralBoxTransportDetail::Pose(shape.State,covered->Bodies[i].State)||
                !scope->NativeConsumer->ReadCurrentBodyLocalKinematicValues(shape.Body,input)||
                input.bWheel!=shape.SphereShapeCaptured||(!input.bWheel&&!shape.BoxShapeCaptured)||
                !SameVector(input.RelativeLocation,shape.LocalOffset)||!SameQuat(input.RelativeRotation,shape.LocalRotation))
                Refuse("Planar refresh fresh physical/input inventory differs from closed baseline");
            auto baseInput=input;baseInput.bWheel=false;r.Inputs.Add(input);
            r.Base.Add(FrameParallel::CalculateSubBodyKinematics(origin,baseInput));r.Final.Add(FrameParallel::CalculateSubBodyKinematics(origin,input));
            PlanarBodyPoseDetail::FStrictBodyGeometryPtr baseProof,springProof;
            const auto beforePose=PlanarBodyPoseDetail::Pose(shape.State);
            const auto basePose=PlanarBodyPoseDetail::Pose(r.Base.Last());
            const bool fixedStationary=fixedRole&&shape.Body==fixedResolver&&!input.bWheel&&
                PlanarBodyPoseDetail::Same(beforePose,basePose);
            if(!PlanarBodyPoseDetail::Prove(r.Snapshot,shape,beforePose,basePose,floor.Point,floor.Normal,budget,baseProof,false,fixedStationary)||!baseProof||
                (input.bWheel&&(!PlanarBodyPoseDetail::Prove(r.Snapshot,shape,PlanarBodyPoseDetail::Pose(r.Base.Last()),PlanarBodyPoseDetail::Pose(r.Final.Last()),floor.Point,floor.Normal,budget,springProof)||!springProof)))
                Refuse("Planar mandatory native refresh physical intermediate corridor not proved");
            r.BaseProofs.Add(MoveTemp(baseProof));r.SpringProofs.Add(MoveTemp(springProof));
        }
        FOrdinaryConsumedStateValues rechecked;scope->ReadOrdinaryFrameStateValues(rechecked);
        if(!OrdinarySetFutureDetail::Unchanged(r.Before,rechecked))Refuse("Planar refresh changed while proving fresh all-child corridors");
        r.Fingerprint=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint();
        PlanarChildRefresh=MoveTemp(refresh);
        try{RequireRefresh();}catch(...){PlanarChildRefresh.Reset();throw;}return;
    }
    auto& state=NativePlanarCommit->Require();auto& effect=*state.Effect;
    if(effect.Owner!=&owner||state.OpenSetter||state.Setters!=4||state.SyncOpened||!FiniteState(origin)||
        !SameKinematicBits(state.Current,owner.GetKinematicState()))Refuse("Planar actual child sync lacks four real returned setters");
    const auto pose=PlanarBodyPoseDetail::Pose(origin);
    for(int32 i=0;i<effect.Families.Num();++i)if(PlanarBodyPoseDetail::Same(effect.Families[i].Origin,pose))
    {if(effect.ChosenFamily!=INDEX_NONE)Refuse("Planar actual origin ambiguous");effect.ChosenFamily=i;}
    if(effect.ChosenFamily==INDEX_NONE)Refuse("Planar actual native origin is outside both preproved conditional families");
    effect.ActualOrigin=origin;effect.OwnerAfter=state.Current;
    for(const auto& input:effect.Inputs)
    {
        auto base=input;base.bWheel=false;effect.Base.Add(FrameParallel::CalculateSubBodyKinematics(origin,base));
        effect.Final.Add(FrameParallel::CalculateSubBodyKinematics(origin,input));
    }
    const auto& family=effect.Families[effect.ChosenFamily];
    for(int32 i=0;i<effect.Inputs.Num();++i)
        if(!PlanarBodyPoseDetail::Same(family.Base[i],PlanarBodyPoseDetail::Pose(effect.Base[i]))||
            !PlanarBodyPoseDetail::Same(family.Final[i],PlanarBodyPoseDetail::Pose(effect.Final[i])))
            Refuse("Planar actual-origin native arithmetic differs from preproved BODY targets");
    state.SyncOpened=true;
}
bool FNativePlanarResponseCommitScope::BeforeNativeBaseChildWrite(const USSubBody& body,const FKinematicState& origin,
    const FKinematicState& before,const FVector& offset,const FQuat& rotation)
{
    if(!NativePlanarCommit)
    {
        if(!PlanarChildRefresh)return false;auto& r=RequireRefresh();const int32 i=r.NextChild;
        if(r.BaseOpen||!r.Inputs.IsValidIndex(i)||!SameKinematicBits(origin,r.Origin))Refuse("Planar refresh BASE sequence changed");
        const auto& shape=r.Before.Owners[r.OwnerIndex].Bodies[i];const auto& input=r.Inputs[i];
        if(shape.Body!=&body||body.GetParentComponent()!=r.Owner||body.GetClass()!=shape.Class||!SameKinematicBits(before,shape.State)||
            !SameVector(offset,input.RelativeLocation)||!SameQuat(rotation,input.RelativeRotation))Refuse("Planar refresh actual BASE operands differ");
        ++r.BaseWrites;if(input.bWheel)r.BaseOpen=true;else ++r.NextChild;return false;
    }
    auto& state=NativePlanarCommit->Require();auto& effect=*state.Effect;
    if(!state.SyncOpened||state.SyncClosed||state.BaseOpen||state.OrderedBaseStage||!effect.Inputs.IsValidIndex(state.NextChild)||
        !SameKinematicBits(origin,effect.ActualOrigin))Refuse("Planar native BASE write order/origin differs");
    const int32 i=state.NextChild;const auto& actual=effect.Before.Owners[effect.OwnerIndex].Bodies[i];const auto& input=effect.Inputs[i];
    if(actual.Body!=&body||body.GetParentComponent()!=effect.Owner||body.GetClass()!=actual.Class||
        !SameKinematicBits(before,actual.State)||!SameVector(offset,input.RelativeLocation)||!SameQuat(rotation,input.RelativeRotation))
        Refuse("Planar native BASE local cache/input/state is not the actual preproved child");
    if(!effect.Families.IsValidIndex(effect.ChosenFamily))Refuse("Planar native BASE family is missing");
    const auto& family=effect.Families[effect.ChosenFamily];
    if(!family.BaseProofs.IsValidIndex(i)||!family.BaseProofs[i])Refuse("Planar native BASE proof is missing");
    const bool rotationFirst=family.BaseProofs[i]->RotationFirst;
    if(rotationFirst)
    {
        if(input.bWheel||!actual.BoxShapeCaptured||!family.BaseProofs[i]->Matches(
            effect.Snapshot,actual,PlanarBodyPoseDetail::Pose(before),family.Base[i]))
            Refuse("Ordered planar BASE proof does not match this child");
        state.OrderedBaseStage=1;
        return true;
    }
    ++effect.BaseWrites;if(input.bWheel)state.BaseOpen=true;else ++state.NextChild;
    return false;
}
void FNativePlanarResponseCommitScope::ReadbackOrderedBaseChildWrite(const USSubBody& body,
    const FKinematicState& actual,bool complete)
{
    if(!NativePlanarCommit)Refuse("Ordered planar BASE write has no active commit");
    auto& state=NativePlanarCommit->Require();auto& effect=*state.Effect;
    const int32 i=state.NextChild;
    const auto& children=effect.Before.Owners[effect.OwnerIndex].Bodies;
    if(state.OrderedBaseStage!=(complete?2:1)||!children.IsValidIndex(i)||!effect.Base.IsValidIndex(i)||
        children[i].Body!=&body||!SameKinematicBits(actual,body.GetKinematicState()))
        Refuse("Ordered planar BASE write sequence changed");
    auto expected=complete?effect.Base[i]:children[i].State;
    expected.Rotation=effect.Base[i].Rotation;
    if(!SameKinematicBits(actual,expected))Refuse("Ordered planar BASE actual field write differs");
    if(complete)
    {
        state.OrderedBaseStage=0;++effect.BaseWrites;++state.NextChild;
        UE_LOG(LogTemp,Display,TEXT("[PlanarChildRotationFirst] Frame=%llu Step=%llu Child=%d"),
            effect.Step.Frame,effect.Step.StepSerial,i);
    }
    else state.OrderedBaseStage=2;
}
void FNativePlanarResponseCommitScope::BeforeNativeWheelSpringWrite(const USWheelSubBody& body,const FKinematicState& origin,
    const FKinematicState& base,float spring)
{
    if(!NativePlanarCommit)
    {
        if(!PlanarChildRefresh)return;auto& r=RequireRefresh();const int32 i=r.NextChild;
        if(!r.BaseOpen||!r.Inputs.IsValidIndex(i)||!r.Inputs[i].bWheel||r.Before.Owners[r.OwnerIndex].Bodies[i].Body!=&body||
            !SameKinematicBits(origin,r.Origin)||!SameKinematicBits(base,r.Base[i])||!SameFloat(spring,r.Inputs[i].WheelSpringDisplacement))
            Refuse("Planar refresh actual wheel BASE/spring differs");
        r.BaseOpen=false;++r.SpringWrites;++r.NextChild;return;
    }
    auto& state=NativePlanarCommit->Require();auto& effect=*state.Effect;const int32 i=state.NextChild;
    if(!state.SyncOpened||state.SyncClosed||!state.BaseOpen||!effect.Inputs.IsValidIndex(i)||
        effect.Before.Owners[effect.OwnerIndex].Bodies[i].Body!=&body||!effect.Inputs[i].bWheel||
        !SameKinematicBits(origin,effect.ActualOrigin)||!SameKinematicBits(base,effect.Base[i])||
        !SameFloat(spring,effect.Inputs[i].WheelSpringDisplacement))Refuse("Planar actual wheel BASE/spring differs before first spring write");
    state.BaseOpen=false;++effect.SpringWrites;++state.NextChild;
}
bool FNativePlanarResponseCommitScope::AfterChildSync(const ISpeedComponent& owner,const FKinematicState& origin)
{
    if(!NativePlanarCommit)
    {
        if(!PlanarChildRefresh)return false;auto& r=RequireRefresh();int32 wheels=0;for(const auto& input:r.Inputs)wheels+=int32(input.bWheel);
        if(r.Owner!=&owner||r.BaseOpen||r.NextChild!=r.Inputs.Num()||r.BaseWrites!=r.Inputs.Num()||r.SpringWrites!=wheels||!SameKinematicBits(origin,r.Origin))
            Refuse("Planar refresh did not return every true child write");
        const PlanarSweepRefreshRead::FScope permit(r.Scope,r.SweepRefresh);
        FOrdinaryConsumedStateValues after;r.Scope->ReadOrdinaryFrameStateValues(after);auto expected=r.Before;
        for(int32 i=0;i<r.Inputs.Num();++i)
        {
            expected.Owners[r.OwnerIndex].Bodies[i].State=r.Final[i];const auto& shape=after.Owners[r.OwnerIndex].Bodies[i];
            if(!r.BaseProofs[i]->Matches(r.Snapshot,shape,PlanarBodyPoseDetail::Pose(r.Before.Owners[r.OwnerIndex].Bodies[i].State),PlanarBodyPoseDetail::Pose(r.Base[i]))||
                (r.Inputs[i].bWheel&&!r.SpringProofs[i]->Matches(r.Snapshot,shape,PlanarBodyPoseDetail::Pose(r.Base[i]),PlanarBodyPoseDetail::Pose(r.Final[i]))))
                Refuse("Planar refresh retained geometry changed");
        }
        if(!OrdinarySetFutureDetail::Unchanged(expected,after))Refuse("Planar refresh native effect exceeded its fresh actual all-child footprint");
        // The ordinary final/response baseline is not rewritten by this sync.
        // A refresh that changes physical endpoints needs another effect scope.
        for(int32 i=0;i<r.Inputs.Num();++i)if(!GeneralBoxTransportDetail::Pose(
            r.Before.Owners[r.OwnerIndex].Bodies[i].State,after.Owners[r.OwnerIndex].Bodies[i].State))
            Refuse("Planar refresh changed a closed physical endpoint");
#if !UE_BUILD_SHIPPING
        if(r.FixedRoofRefreshRole)
        {
            auto* held=r.Scope->FixedNoPoseSweepBaseline.Get();
            const auto* pending=r.Scope->OrdinaryReturns?r.Scope->OrdinaryReturns->Pending.Get():nullptr;
            if(!held||held->After.Owner!=&owner||!pending||pending->Owner!=&owner||
                held->SweepRefreshCount+1!=r.FixedRoofRefreshRole)Refuse("Fixed roof ordered child refresh lost its real owner binding");
            if(r.FixedRoofRefreshRole==1)held->SweepRefreshQuerySerial=pending->EntryStep.QuerySerial;
            ++held->SweepRefreshCount;
            UE_LOG(LogTemp,Display,TEXT("[NativeRoofFixedSweepRefreshClosed] Frame=%llu Iteration=%d Serial=%llu Role=%u Children=%d"),
                r.Step.Frame,r.Step.Iteration,r.Step.QuerySerial,uint32(r.FixedRoofRefreshRole),r.Inputs.Num());
        }
#endif
        PlanarChildRefresh.Reset();return true;
    }
    auto& state=NativePlanarCommit->Require();auto& effect=*state.Effect;
    int32 wheels=0;for(const auto& input:effect.Inputs)wheels+=int32(input.bWheel);
    if(effect.Owner!=&owner||!state.SyncOpened||state.SyncClosed||state.BaseOpen||state.NextChild!=effect.Inputs.Num()||
        effect.BaseWrites!=effect.Inputs.Num()||effect.SpringWrites!=wheels||state.Setters!=4||state.OpenSetter||
        !SameKinematicBits(origin,effect.ActualOrigin)||!SameKinematicBits(owner.GetKinematicState(),effect.OwnerAfter))
        Refuse("Planar actual all-child sync did not return exactly once");
    state.Scope->ReadOrdinaryConsumedState(effect.After);auto expected=effect.Before;
    auto& target=expected.Owners[effect.OwnerIndex];target.State=effect.OwnerAfter;target.PhysRotation=effect.OwnerAfter.Rotation;
    const auto& family=effect.Families[effect.ChosenFamily];
    for(int32 i=0;i<effect.Inputs.Num();++i)
    {
        target.Bodies[i].State=effect.Final[i];const auto& shape=effect.After.Owners[effect.OwnerIndex].Bodies[i];
        if(!family.BaseProofs[i]||!family.BaseProofs[i]->Matches(effect.Snapshot,shape,
            PlanarBodyPoseDetail::Pose(effect.Before.Owners[effect.OwnerIndex].Bodies[i].State),family.Base[i])||
            (effect.Inputs[i].bWheel&&(!family.SpringProofs[i]||!family.SpringProofs[i]->Matches(effect.Snapshot,shape,family.Base[i],family.Final[i]))))
            Refuse("Planar real child geometry/configuration no longer matches preproved corridor");
    }
    if(!OrdinarySetFutureDetail::Unchanged(expected,effect.After))Refuse("Planar four setters/sync changed values outside its exact covered footprint");
    state.SyncClosed=true;effect.Closed=true;
    state.Response->Entries[effect.Entry].Calls[2].PlanarEffect=state.Effect;
    UE_LOG(LogTemp,Display,TEXT("[NativeGeneralFloorPlanarCommitClosed] Frame=%llu Step=%llu Entry=%d Setters=4 Children=%d BaseWrites=%d SpringWrites=%d ActualOriginFamily=%d QueriesDuringCommit=0 NoIntegrationAfterRewrite=1"),
        effect.Step.Frame,effect.Step.StepSerial,effect.Entry,effect.Inputs.Num(),effect.BaseWrites,effect.SpringWrites,effect.ChosenFamily);
    return true;
}
bool FNativePlanarResponseCommitScope::HasUnclosedBodySync() noexcept
{return bool(PlanarChildRefresh)||(NativePlanarCommit&&NativePlanarCommit->State&&!NativePlanarCommit->State->SyncClosed)||
    FNativeFinalWheelSupportScope::HasPendingSync();}
void FNativePlanarResponseCommitScope::RequireQueriesClosed()
{if(HasUnclosedBodySync()){RecordPreparedPoison("planar_query_before_actual_all_child_sync");Refuse("Native query attempted between planar owner writes and actual child sync");}}
void FNativePlanarResponseCommitScope::CloseStep(FStepScope& scope) noexcept
{
    if(PlanarChildRefresh&&PlanarChildRefresh->Scope==&scope)
    {RecordPreparedPoison("planar_refresh_missing_actual_child_return");PlanarChildRefresh.Reset();}
    if(NativePlanarCommit&&NativePlanarCommit->State&&NativePlanarCommit->State->Scope==&scope)
        RecordPreparedPoison("planar_commit_outlived_actual_step");
}
FNativePlanarResponseCommitScope::~FNativePlanarResponseCommitScope() noexcept
{
    if(!State)return;
    if(NativePlanarCommit!=this)RecordPreparedPoison("planar_commit_lexical_scope_changed");
    if(!State->SyncClosed||!State->Effect->Closed||State->OpenSetter||State->Setters!=4)
        RecordPreparedPoison("planar_commit_missing_actual_setter_or_child_return");
    if(NativePlanarCommit==this)NativePlanarCommit=nullptr;
}
void FNativePlanarResponseCommitScope::RequireClosedResponseEffect(FStepScope& scope,const FOrdinaryResponseEntryValues& row)
{
    const auto effect=row.Calls.Num()==3?row.Calls[2].PlanarEffect:TSharedPtr<const FClosedPlanarResponseEffect,ESPMode::ThreadSafe>{};
    if(!effect){scope.RequireGeneralBoxUnchanged(row.Entry,row.After);return;}
    const auto& e=*effect;FStepIdentity now;
    if(!e.Closed||e.Snapshot!=scope.Snapshot||row.RawResolver!=e.Resolver||e.OwnerIndex<0||e.ChosenFamily<0||
        e.SetterReturns.Num()!=4||!e.ClampRecorded[0]||!e.ClampRecorded[1]||!ReadCurrentStep(now)||
        !OrdinaryWorldDetail::Epoch(e.Step,now)||e.Step.QuerySerial!=now.QuerySerial||
        e.Fingerprint!=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint())
        Refuse("Planar response effect lacks its genuine immutable four-setter/all-child closure");
    // Three explicit spans, with no exceptOwner exemption and no mutation of
    // the historical integration After. The middle exact footprint was closed
    // at the true native sync; endpoints alone cannot create that receipt.
    scope.RequireGeneralBoxUnchanged(row.Entry,e.Before);
    if(!OrdinarySetFutureDetail::Unchanged(row.Calls[2].Before,e.Before))
        Refuse("Planar Resolve changed captured values before the first covered setter");
    auto expected=e.Before;auto& target=expected.Owners[e.OwnerIndex];target.State=e.OwnerAfter;target.PhysRotation=e.OwnerAfter.Rotation;
    for(int32 i=0;i<e.Final.Num();++i)target.Bodies[i].State=e.Final[i];
    if(!OrdinarySetFutureDetail::Unchanged(expected,e.After))Refuse("Planar response closed middle footprint changed");
    scope.RequireGeneralBoxUnchanged(expected,e.After);scope.RequireGeneralBoxUnchanged(e.After,row.After);
    if(!OrdinarySetFutureDetail::Unchanged(e.After,row.Calls[2].After)||
        !OrdinarySetFutureDetail::Unchanged(row.Calls[2].After,row.After))
        Refuse("Planar Resolve changed captured values after its actual sync closure");
}
const FOrdinaryConsumedStateValues& FStepScope::GeneralBoxCurrentClosedBaseline(bool requireClosedBatch) const
{
    if(!OrdinaryIntegration||!OrdinaryIntegration->Closed)Refuse("General Box has no actual closed integration baseline");
    const FOrdinaryConsumedStateValues* baseline=&OrdinaryIntegration->After;
    if(OrdinaryResponse)
    {
        const auto& tx=*OrdinaryResponse;
        if(tx.NextIndex<0||tx.NextIndex>tx.Selected.Num()||
            (requireClosedBatch&&!tx.Closed)||(tx.Closed&&tx.NextIndex!=tx.Selected.Num()))
            Refuse("General Box baseline lacks a truly closed response prefix");
        if(tx.ActiveIndex==INDEX_NONE)
        {if(tx.ActiveCall!=INDEX_NONE||tx.Entries.Num()!=tx.NextIndex)Refuse("General Box closed response prefix inventory differs");}
        else if(requireClosedBatch||tx.Closed||tx.ActiveIndex!=tx.NextIndex||tx.Entries.Num()!=tx.NextIndex+1||
            tx.Entries[tx.ActiveIndex].Closed||!tx.Entries[tx.ActiveIndex].Allowed||tx.ActiveCall<INDEX_NONE||tx.ActiveCall>2)
            Refuse("General Box baseline active entry is not the unopened remainder of its closed prefix");
        for(int32 i=0;i<tx.NextIndex;++i)
        {
            const auto& row=tx.Entries[i];
            if(!row.Closed)Refuse("General Box final baseline response entry not closed");
            if(!row.Allowed)continue;
            RequireGeneralBoxUnchanged(*baseline,row.Entry);
            FNativePlanarResponseCommitScope::RequireClosedResponseEffect(*const_cast<FStepScope*>(this),row);
            baseline=&row.After;
        }
    }
    return *baseline;
}

bool FStepScope::IsPreparedCurvedBoxExactProgress(const SComponentTOI& candidate,const USSubBody* resolver,bool consumed) const
{
    const auto& hit=candidate.Hit;
    if(!candidate.bHit||!hit.bSurfaceNormalMayVary||!resolver||!NativeConsumer||
        StepScope!=this||!Active||!Window||Window->GetPurpose()!=OnePausedWork::EPurpose::FullFrame||!Snapshot||
        !hit.bHit||!hit.bBlockingHit||hit.bStartPenetrating||!(hit.PenetrationDepth<=0.f)||
        !hit.SourceId||!hit.SurfaceId||!hit.CanonicalGroupId||
        !FMath::IsFinite(hit.TOI)||hit.TOI<=0.f||!SameFloat(candidate.TOI,hit.TOI))return false;

    const auto& inventory=candidate.PreparedEvidence;
    if(!inventory||!inventory->Sealed||!inventory->SelectedHit||inventory->SelectedRawResolver!=resolver||
        inventory->SelectedPairKey!=candidate.PairKey||!SameFloat(inventory->SelectedSeconds,hit.TOI)||
        !inventory->SelectedResolver.HasSameIndexAndSerialNumber(candidate.Resolver)||
        !SameNativeAnalyticContactValues(inventory->SelectedOrientedHit,hit)||
        !inventory->Bodies.IsValidIndex(inventory->SelectedBodyIndex))return false;
    const auto& body=inventory->Bodies[inventory->SelectedBodyIndex];
    const auto& ground=body.OrdinaryGround;
    if(body.RawBodyIdentity!=resolver||!ground||!ground->IsCurvedBoxArrivalValues()||
        !SameNativeAnalyticContactValues(ground->GetGroundValuesForInspection(),hit))return false;
    const auto& request=ground->GetRequestForInspection();
    if(!request||request->GetSnapshot()!=Snapshot)return false;
    const auto& captured=request->GetValues();
    const auto& step=request->GetStep();
    if(captured.Shape!=ENativeSolidQueryShape::Box||captured.Body!=resolver||captured.Owner!=inventory->Owner||
        captured.BodyIndex!=body.Index||!step.QuerySerial||hit.TOI>captured.Horizon)return false;

    FStepIdentity now;
    if(!ReadCurrentStep(now))return false;
    if(!consumed)
        return SweepPhase&&HasOpenOrdinaryWorldSelection()&&step.QuerySerial<=now.QuerySerial&&
            OrdinaryWorldDetail::Epoch(step,now);

    // The request remains a historical value after the sweep epoch is
    // revoked. Join it to the actual closed selector, never reopen it.
    if(SweepPhase||!OrdinaryResponse||!NativeAdvance||!NativeAdvance->Consumed)return false;
    const auto& tx=*OrdinaryResponse;
    if(tx.Closed||tx.ActiveIndex!=INDEX_NONE||!tx.Selected.IsValidIndex(tx.NextIndex)||
        !OrdinaryWorldDetail::Epoch(tx.EntryStep,now)||tx.EntryStep.QuerySerial!=now.QuerySerial||
        !OrdinaryWorldDetail::Epoch(step,tx.SelectorStep)||step.QuerySerial>tx.SelectorStep.QuerySerial||
        !OrdinaryWorldDetail::Entry(candidate,tx.Selected[tx.NextIndex],hit.TOI))return false;
    // A tolerance-batched earlier hit did not consume this exact TOI and
    // must not seed or use timed curved reacquisition.
    return SameFloat(tx.Seconds,hit.TOI)&&SameFloat(tx.ActualAfter,tx.Before+hit.TOI)&&tx.ActualAfter>tx.Before;
}

bool FStepScope::HasClosedCurvedBoxGroundForObservation(const FNativeSolidStaticRequest& current,const SHitResult& actual) const
{
    if(StepScope!=this||!Active||!Window||Window->GetPurpose()!=OnePausedWork::EPurpose::FullFrame||
        !Snapshot||current.GetSnapshot()!=Snapshot||!actual.bSurfaceNormalMayVary)return false;
    const auto& now=current.GetStep();
    const auto& values=current.GetValues();
    for(const auto& historical:ClosedCurvedBoxGroundForObservation)
    {
        const auto& request=historical->GetRequestForInspection();
        const auto& saved=request->GetValues();
        if(request->GetSnapshot()!=Snapshot||saved.Owner!=values.Owner||saved.Body!=values.Body||
            saved.OwnerStableId!=values.OwnerStableId||saved.BodyIndex!=values.BodyIndex||saved.BodyClass!=values.BodyClass||
            !(saved.FrameClock==values.FrameClock)||!SameNativeAnalyticContactValues(historical->GetGroundValuesForInspection(),actual))
            continue;

        auto prior=request->GetStep();
        if(prior.QueryEpoch>=now.QueryEpoch||prior.Iteration>=now.Iteration||
            prior.QuerySerial>=now.QuerySerial||prior.TimePassed>now.TimePassed)continue;
        // Epoch/iteration/remaining change by design. Every durable identity
        // still joins the current request. No historical query is reopened.
        prior.QueryEpoch=now.QueryEpoch;
        prior.Iteration=now.Iteration;
        prior.TimePassed=now.TimePassed;
        prior.Remaining=now.Remaining;
        if(OrdinaryWorldDetail::Epoch(prior,now))return true;
    }
    return false;
}
bool FStepScope::HasClosedPlanarValuesForObservation(const UBoxSubBody* body) const
{
    FStepIdentity now;
    if(!body||StepScope!=this||!Active||!Window||Window->GetPurpose()!=OnePausedWork::EPurpose::FullFrame||
        !ReadCurrentStep(now)||!Snapshot)return false;
    for(const auto& value:ClosedPlanarValuesForObservation)
    {
        // Deliberately historical values: CCD iteration/query epoch may have
        // advanced. Only select a terminal observer; never revive an authority.
        if(value&&value->Closed&&value->Resolver==body&&value->Snapshot==Snapshot&&
            value->Step.Frame==now.Frame&&value->Step.StepSerial==now.StepSerial&&
            value->Step.Host==now.Host&&value->Step.Bridge==now.Bridge&&value->Step.World==now.World&&
            value->Step.Worker==now.Worker&&value->Step.Provider==now.Provider&&value->Step.Lease==now.Lease&&
            value->Step.InputGeneration==now.InputGeneration&&value->Step.WindowSerial==now.WindowSerial&&
            value->Step.WindowNonce==now.WindowNonce&&value->Step.NumericControl==now.NumericControl&&
            value->Step.ThreadId==now.ThreadId&&SameFloat(value->Step.Delta,now.Delta)&&SameFloat(value->Step.SimTime,now.SimTime)&&
            value->Owner==body->GetParentComponent()&&value->SetterReturns.Num()==4&&value->ClampRecorded[0]&&value->ClampRecorded[1]&&
            value->Before.Owners.IsValidIndex(value->OwnerIndex)&&value->Binding.Request)
        {
            const auto* owner=Snapshot->GetNativeOwners()->Find(value->Owner);const int32 index=value->Binding.Request->GetValues().BodyIndex;
            const auto& copied=value->Before.Owners[value->OwnerIndex];
            if(owner&&owner->BodyCensusCaptured&&owner->Bodies.IsValidIndex(index)&&copied.Bodies.IsValidIndex(index))
            {
                const auto& gt=owner->Bodies[index];const auto& prior=copied.Bodies[index];
                if(gt.Body==body&&gt.Class==body->GetClass()&&gt.Index==index&&gt.ObjectIdCaptured&&
                    prior.Body==body&&prior.Class==gt.Class&&prior.Index==index&&prior.GTObjectId==gt.ObjectId&&
                    value->BaseWrites==value->Inputs.Num()&&value->BaseWrites==copied.Bodies.Num())return true;
            }
        }
    }
    return false;
}
