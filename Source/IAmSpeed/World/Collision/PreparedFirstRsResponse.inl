// Native execution TU only. The request retained by Data stays revoked; this
// distinct response ticket is minted solely at the p7 global native site.
namespace {std::atomic<uint64> NextFirstRsResponseSerial{0};}

bool FStepScope::HasCurrentFirstRsResponseIdentity(bool syncSite) const
{
    if(!Active||StepScope!=this||!Window||!OnePausedWork::HasFirstRsResponsePrefix(Window->GetPurpose())||
        !Window->IsOpen()||Poisoned||MemberSweepScope||ComponentSweepScope||!HostScope||SweepPhase||
        !RsCapture||!RsCapture->IsClosed()||!RsCapture->HasBoundedFirstArrivalRsQuality()||
        !(syncSite?PreparationPrefix::FCursor::IsCurrentRsResponseSyncNativeWindow(*Identity.Bridge):
            PreparationPrefix::FCursor::IsCurrentRsResponseNativeWindow(*Identity.Bridge))||
        !IsCurrentWorker(Identity.Worker)||Identity.ThreadId!=FPlatformTLS::GetCurrentThreadId())return false;
    OnePausedWork::FCanonicalFrameWindowPtr current;
    if(!OnePausedWork::ReadExecutionWindow(current)||current!=Window||current->GetSerial()!=Identity.WindowSerial||
        current->GetNonce()!=Identity.WindowNonce||current->GetSnapshot()!=Snapshot||
        Identity.Provider->GetPreparedSnapshot()!=Snapshot||Identity.Host->GetInputWorkerGeneration()!=Identity.InputGeneration||
        Identity.Bridge->GetStaticCollisionWorld()!=Identity.Provider||Identity.Host->GetWorld()!=Identity.World||
        Identity.Bridge->GetWorld()!=Identity.World||
        !FCanonicalFramePreflight::HasUnchangedRawPreparationCensus(*Identity.Bridge,*Snapshot,Identity.StepSerial))return false;
    const auto& contract=Window->GetPreparationContract();const auto& data=contract->FirstRsResponseData;
    if(!data||data->HasSynchronizationExpectation!=OnePausedWork::HasFirstRsResponseSyncPrefix(Window->GetPurpose())||!data->Origin||!data->AdvanceSource||!data->AdvanceSource->SealedDuringMember||
        data->Origin!=RsCapture->Request||data->AdvanceSource!=RsCapture->AdvanceSource||
        !data->Values.Quality||data->Values.Quality!=RsCapture->RsQuality||
        data->Values.Quality->GetArrival()!=RsCapture->FirstArrival||data->Values.Geometry!=data->Origin->GetGeometrySelection()||
        data->Values.Hit.PreparedGroundEvidence||data->Values.Hit.FirstRsResponseData||
        data->Physical.Hit.PreparedGroundEvidence||data->Physical.Hit.FirstRsResponseData||
        !contract->RsAdvance.Complete||contract->RsAdvance.Serial!=data->AdvanceSerial||!data->AdvanceSerial||
        contract->RsAdvance.CompletedOwners!=3||contract->RsAdvance.VerifiedMask!=7||contract->RsAdvance.ReturnedMask!=7||
        contract->RsAdvance.StateCapturedMask!=7||contract->RsAdvance.ActualStates.Num()!=3)return false;
    const auto& origin=data->Origin->GetStep();
    const auto fp=Analytic::InspectCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    return origin.Lease==this&&origin.Host==Identity.Host&&origin.Bridge==Identity.Bridge&&origin.Worker==Identity.Worker&&
        origin.World==Identity.World&&origin.Provider==Identity.Provider&&origin.InputGeneration==Identity.InputGeneration&&
        origin.Frame==Identity.Frame&&origin.StepSerial==Identity.StepSerial&&origin.WindowSerial==Identity.WindowSerial&&
        origin.WindowNonce==Identity.WindowNonce&&origin.QuerySerial==1&&Identity.QuerySerial==1&&origin.QueryEpoch==2&&Identity.QueryEpoch==3&&
        fp.Status==Analytic::EAnalyticNumericStatus::Complete&&fp.ControlFingerprint()==Identity.NumericControl&&
        data->NumericControl==Identity.NumericControl&&data->AdvanceSource->NumericControl==Identity.NumericControl;
}

void FStepScope::CheckFirstRsResponseOwners(int32 completed,bool syncSite,bool synchronized) const
{
    if(!HasCurrentFirstRsResponseIdentity(syncSite)||completed<0||completed>3||
        (synchronized&&(!syncSite||!OnePausedWork::HasFirstRsResponseSyncPrefix(Window->GetPurpose()))))Refuse("Rs response native identity is stale");
    const auto& data=*Window->GetPreparationContract()->FirstRsResponseData;
    const auto& proof=*data.AdvanceSource;const auto& owners=Registry->GetOrderedAdapters();
    if(owners.Num()!=3||proof.Owners.Num()!=3)Refuse("Rs response owner inventory differs");
    for(int32 i=0;i<3;++i)
    {
        const auto& row=proof.Owners[i];const auto* owner=owners[i];const auto* retained=Snapshot->GetNativeOwners()->Find(owner);
        const FKinematicState& expected=(i==1&&completed==3)?data.Physical.OwnerAfter:row.After;
        if(!owner||owner!=row.Owner||!retained||!retained->BodyCensusCaptured||owner->GetPublishedSimulationStableId()!=row.StableId||
            owner->NumFrame()!=row.Frame||owner->IsFrozen()!=row.Frozen||&owner->GetKinematicState()!=row.Stored||
            &owner->GetKinematicStateForFrame(row.Frame)!=row.Selected||(!row.Frozen&&row.Stored!=row.Selected)||
            !SameVector(owner->GetPhysCenterOfMassLocal(),row.LocalCOM)||!SameFloat(owner->GetPhysMaxSpeed(),row.MaximumSpeed)||
            !SameFloat(owner->GetPhysMaxAngularSpeed(),row.MaximumAngularSpeed)||!owner->GetPhysicalConstraints().IsEmpty()||
            !SameKinematicBits(owner->GetKinematicState(),expected))
            Refuse("Rs response actual owner/caps/history/COM/state changed");
        if(!row.Frozen&&(owner->IsPrescribedMotion()||owner->IsPhysicsSleeping()))Refuse("Rs response free-owner branch changed");
        const auto& bodies=owner->GetSubBodies();
        if(bodies.Num()!=row.BodiesAfter.Num()||retained->Bodies.Num()!=row.BodiesAfter.Num())Refuse("Rs response child inventory changed");
        for(int32 j=0;j<bodies.Num();++j)
            if(!bodies[j]||retained->Bodies[j].Index!=j||retained->Bodies[j].Body!=bodies[j]||bodies[j]->GetParentComponent()!=owner||
                !SameKinematicBits(bodies[j]->GetKinematicState(),synchronized&&i==1&&j==0?data.BodyAfterSynchronization:row.BodiesAfter[j]))Refuse("Rs response child state changed");
    }
    const auto* binding=Snapshot->GetNativeOwners()->Find(data.Owner);
    if(!binding||binding->NativeObject->GetClass()!=data.OwnerClass||binding->Class!=data.OwnerClass||
        binding->Bodies.Num()!=1||binding->Bodies[0].Body!=data.Body||binding->Bodies[0].Class!=data.BodyClass||
        data.Body->GetClass()!=data.BodyClass||data.Body->GetParentComponent()!=data.Owner||
        data.Owner!=owners[1]||data.Body!=owners[1]->GetSubBodies()[0]||data.Owner->IsFrozen()||
        !SameFloat(data.Body->GetRadius(),data.Physical.Input.PhysicalRadius)||
        !SameFloat(data.Body->GetWorldStaticContactOffsetCm(),data.Physical.Input.StaticContactOffsetCm)||
        !SameFloat(data.Body->GetImpactThreshold(),data.Physical.Input.ImpactThreshold)||
        !SameVector(data.Owner->GetPhysCOM(),data.Physical.Input.ActualCOM))
        Refuse("Rs response exact native body/class/COM/physical branch changed");
}

const USphereSubBody* FStepScope::BeginTerminalFirstRsResponse(float seconds,float timePassed,float simTime)
{
    if(!HasCurrentFirstRsResponseIdentity()||!PreparationPrefix::FCursor::IsCurrentRsResponseSite(INDEX_NONE))
        Refuse("Rs response is outside its distinct native global site");
    const auto& contract=Window->GetPreparationContract();const auto& data=*contract->FirstRsResponseData;auto& actual=contract->FirstRsResponse;
    if(actual.Started||actual.Serial||actual.Failed||!SameFloat(seconds,data.Seconds)||!SameFloat(timePassed,data.ActualTimePassed)||
        !SameFloat(simTime,data.ActualResponseSimTime)||!SameFloat(seconds,contract->RsAdvance.Seconds)||
        !SameFloat(data.ActualTimePassed,data.Origin->GetStep().TimePassed+seconds)||
        !SameFloat(data.ActualResponseSimTime,data.Origin->GetStep().SimTime))
        Refuse("Rs response was repeated or its actual native clocks differ");
    for(int32 i=38;i<=41;++i)if(!contract->Records[i].Attempted||!contract->Records[i].Returned||!contract->Records[i].Accepted)
        Refuse("Rs response precedes complete advance Source closure");
    CheckFirstRsResponseOwners(0);
    uint64 serial=NextFirstRsResponseSerial.load();
    do{if(serial==MAX_uint64)Refuse("Rs response serial exhausted");}
    while(!NextFirstRsResponseSerial.compare_exchange_weak(serial,serial+1));
    actual.Serial=serial+1;actual.AdvanceSerial=data.AdvanceSerial;actual.NumericControl=Identity.NumericControl;
    actual.Data=contract->FirstRsResponseData;actual.Started=true;return data.Body;
}

SHitResult FStepScope::BeginTerminalFirstRsResponseCall(const USSubBody* body,int32 call)
{
    if(!HasCurrentFirstRsResponseIdentity()||call<0||call>2||!PreparationPrefix::FCursor::IsCurrentRsResponseSite(call))
        Refuse("Rs response call escaped its distinct native site");
    const auto& contract=Window->GetPreparationContract();const auto& data=*contract->FirstRsResponseData;auto& actual=contract->FirstRsResponse;
    if(!actual.Started||actual.Complete||actual.Failed||actual.ActiveCall!=INDEX_NONE||actual.CompletedCalls!=call||
        actual.Data!=contract->FirstRsResponseData||body!=data.Body||actual.AttemptedMask!=(uint8(1u<<call)-1))
        Refuse("Rs response call is stale, repeated or reordered");
    CheckFirstRsResponseOwners(call);
    SHitResult hit=data.Values.Hit;hit.FirstRsResponseData=actual.Data;
    if(call==0)
    {if(!IsEmptyNativeInitialHit(body->GetFutureHit())||!IsEmptyNativeInitialHit(body->GetHit()))Refuse("Rs response initial hit caches are not empty");}
    else if(call==1)
    {if(!SamePreparedHitValues(body->GetFutureHit(),hit)||!IsEmptyNativeInitialHit(body->GetHit()))Refuse("Rs response accepted hit cache differs");}
    else
    {
        if(!IsEmptyNativeInitialHit(body->GetFutureHit())||!SamePreparedHitValues(body->GetHit(),hit))Refuse("Rs response resolved hit cache differs");
        FFirstRsNonImpactResponseInput input=data.Physical.Input;input.OwnerBefore=data.Owner->GetKinematicState();
        input.BodyBefore=body->GetKinematicState();input.ActualCOM=data.Owner->GetPhysCOM();
        input.PhysicalRadius=data.Body->GetRadius();input.StaticContactOffsetCm=data.Body->GetWorldStaticContactOffsetCm();
        input.ImpactThreshold=data.Body->GetImpactThreshold();
        FFirstRsNonImpactResponseExpectation expected;FString reason;
        if(!BuildFirstRsNonImpactResponseExpectation(data.Values,input,expected,reason)||
            !SameKinematicBits(expected.OwnerAfter,data.Physical.OwnerAfter)||expected.AddsAcceleration!=data.Physical.AddsAcceleration||
            !SameVector(expected.Normal,data.Physical.Normal)||!SameVector(expected.Point,data.Physical.Point)||
            !SameFloat(expected.IncomingNormalSpeed,data.Physical.IncomingNormalSpeed)||
            !SameFloat(expected.PostNormalSpeed,data.Physical.PostNormalSpeed)||!SameFloat(expected.NormalAcceleration,data.Physical.NormalAcceleration))
            Refuse("Rs response actual nonimpact arithmetic differs before effect");
    }
    actual.AttemptedMask|=uint8(1u<<call);actual.ActiveCall=call;
    return hit;
}

void FStepScope::RequireTerminalFirstRsResponseHit(const USSubBody* body,const SHitResult& hit,
    EFirstRsResponseCall call,float delta,float simTime)
{
    const int32 index=int32(call);auto* s=StepScope;
    if(!hit.FirstRsResponseData||hit.PreparedGroundEvidence||!s||!s->HasCurrentFirstRsResponseIdentity()||
        !PreparationPrefix::FCursor::IsCurrentRsResponseSite(index))Refuse("Rs annotated hit has no current private response capability");
    const auto& contract=s->Window->GetPreparationContract();const auto& data=*contract->FirstRsResponseData;const auto& actual=contract->FirstRsResponse;
    if(index<0||index>2||!actual.Serial||!actual.Started||actual.Complete||actual.Failed||actual.ActiveCall!=index||actual.CompletedCalls!=index||
        actual.Data!=hit.FirstRsResponseData||actual.Data!=contract->FirstRsResponseData||actual.AdvanceSerial!=data.AdvanceSerial||
        body!=data.Body||actual.AttemptedMask!=(uint8(1u<<(index+1))-1))Refuse("Rs response action capability is stale or belongs to another body");
    SHitResult expected=data.Values.Hit;expected.FirstRsResponseData=actual.Data;
    if(!SamePreparedHitValues(hit,expected)||
        (call==EFirstRsResponseCall::Resolve&&(!SameFloat(delta,data.Seconds)||!SameFloat(simTime,data.ActualResponseSimTime))))
        Refuse("Rs response consumed hit or native clock differs");
    s->CheckFirstRsResponseOwners(index);
}

void RequireFirstRsResponseHit(const USSubBody* body,const SHitResult& hit,EFirstRsResponseCall call,float delta,float simTime)
{FStepScope::RequireTerminalFirstRsResponseHit(body,hit,call,delta,simTime);}

void FStepScope::RecordTerminalFirstRsResponseStates(const USSubBody* body,int32 call,bool returned)
{
    // Raw return/cache/kinematic facts survive post-effect refusal. A raw copy
    // never counts as verification and cannot create a second attempt.
    if(!Active||StepScope!=this||!Window||!OnePausedWork::HasFirstRsResponsePrefix(Window->GetPurpose())||
        call<0||call>2)return;
    const auto& contract=Window->GetPreparationContract();if(!contract||!contract->FirstRsResponseData)return;
    auto& actual=contract->FirstRsResponse;const auto& data=*contract->FirstRsResponseData;
    if(body!=data.Body||!actual.Started||actual.ActiveCall!=call||actual.CompletedCalls!=call||
        (actual.StateCaptureAttemptedMask&(uint8(1u<<call))))return;
    if(returned)actual.ReturnedMask|=uint8(1u<<call);
    actual.StateCaptureAttemptedMask|=uint8(1u<<call);
    FFirstRsResponseActualCall observed;observed.Call=EFirstRsResponseCall(call);observed.Returned=returned;
    observed.Future=body->GetFutureHit();observed.Current=body->GetHit();
    observed.SourceFieldsRefusal=TEXT("Partial Core fields unavailable before its bounded Source reader");
    if(!Registry||!Snapshot||!Snapshot->GetNativeOwners()||
        !FCanonicalFramePreflight::HasUnchangedRawPreparationCensus(*Identity.Bridge,*Snapshot,Identity.StepSerial))
    {actual.ActualCalls.Add(MoveTemp(observed));actual.Failed=true;return;}
    const auto& owners=Registry->GetOrderedAdapters();
    if(owners.Num()!=3){actual.ActualCalls.Add(MoveTemp(observed));actual.Failed=true;return;}
    for(const auto* owner:owners)
    {
        const auto* binding=Snapshot->GetNativeOwners()->Find(owner);
        if(!owner||!binding||owner->GetSubBodies().Num()!=binding->Bodies.Num())
        {actual.ActualCalls.Add(MoveTemp(observed));actual.Failed=true;return;}
        observed.Owners.Add(owner->GetKinematicState());auto& bodies=observed.Bodies.AddDefaulted_GetRef();
        for(int32 j=0;j<binding->Bodies.Num();++j)
        {
            const auto* child=binding->Bodies[j].Body;
            if(!child||binding->Bodies[j].Index!=j||owner->GetSubBodies()[j]!=child)
            {actual.ActualCalls.Add(MoveTemp(observed));actual.Failed=true;return;}
            bodies.Add(child->GetKinematicState());
        }
    }
    observed.RawStateCaptured=true;actual.StateCapturedMask|=uint8(1u<<call);
    observed.SourceFieldsCaptured=PreparationPrefix::FCursor::CaptureFirstRsResponsePartial(call,observed.SourceBallFields,observed.SourceFieldsRefusal);
    if(!observed.SourceFieldsCaptured)actual.Failed=true;
    actual.ActualCalls.Add(MoveTemp(observed));if(!returned)actual.Failed=true;
}

void FStepScope::EndTerminalFirstRsResponseCall(const USSubBody* body,int32 call)
{
    RecordTerminalFirstRsResponseStates(body,call,true);
    if(!HasCurrentFirstRsResponseIdentity()||call<0||call>2||!PreparationPrefix::FCursor::IsCurrentRsResponseSite(call))
        Refuse("Rs response actual call return escaped its native site");
    auto& actual=Window->GetPreparationContract()->FirstRsResponse;
    if(actual.Failed||actual.ActiveCall!=call||actual.CompletedCalls!=call||
        !(actual.ReturnedMask&(uint8(1u<<call)))||!(actual.StateCapturedMask&(uint8(1u<<call)))||
        actual.ActualCalls.Num()!=call+1||!actual.ActualCalls.Last().SourceFieldsCaptured||!actual.ActualCalls.Last().SourceBallFields)
        Refuse("Rs response actual return/state observation unavailable");
    CheckFirstRsResponseOwners(call+1);
    const auto& data=*actual.Data;SHitResult expected=data.Values.Hit;expected.FirstRsResponseData=actual.Data;
    if(call==0?(!SamePreparedHitValues(body->GetFutureHit(),expected)||!IsEmptyNativeInitialHit(body->GetHit())):
        (!IsEmptyNativeInitialHit(body->GetFutureHit())||!SamePreparedHitValues(body->GetHit(),expected)))
        Refuse("Rs response actual hit-cache transition differs");
    actual.VerifiedMask|=uint8(1u<<call);actual.CompletedCalls=call+1;actual.ActiveCall=INDEX_NONE;
}

void FStepScope::EndTerminalFirstRsResponse()
{
    if(!HasCurrentFirstRsResponseIdentity()||!PreparationPrefix::FCursor::IsCurrentRsResponseSite(INDEX_NONE))
        Refuse("Rs response completion is outside the distinct global native site");
    auto& actual=Window->GetPreparationContract()->FirstRsResponse;
    if(!actual.Started||actual.Complete||actual.Failed||actual.CompletedCalls!=3||actual.ActiveCall!=INDEX_NONE||
        actual.AttemptedMask!=7||actual.ReturnedMask!=7||actual.VerifiedMask!=7||actual.StateCapturedMask!=7||actual.ActualCalls.Num()!=3)
        Refuse("Rs response actual native call inventory incomplete");
    CheckFirstRsResponseOwners(3);actual.Complete=true;
}

bool FStepScope::ReadTerminalFirstRsResponseObservation(FTerminalFirstRsResponseObservation& out)
{
    out={};const auto* s=StepScope;
    if(!s||!s->HasCurrentFirstRsResponseIdentity()||!PreparationPrefix::FCursor::IsCurrentRsResponseObservation())return false;
    out=s->Window->GetPreparationContract()->FirstRsResponse;return true;
}

bool AppendFirstRsResponseDataFields(FNativePrepareGTFields& out,const FString& prefix,
    FFirstRsResponseDataPtr data,Analytic::Certified::FDistanceBudget& budget)
{
    bool ok=out.SharedIdentity(prefix+TEXT(".first_rs_data"),TSharedPtr<const void,ESPMode::ThreadSafe>(data,static_cast<const void*>(data.Get())),budget);
    const auto add=[&](const TCHAR* key,uint64 value){ok=out.Add(prefix+key,value,budget)&&ok;};
    add(TEXT(".first_rs_origin"),data?reinterpret_cast<UPTRINT>(data->GetOriginAddress()):0);
    add(TEXT(".first_rs_advance_source"),data?reinterpret_cast<UPTRINT>(data->GetAdvanceSourceAddress()):0);
    add(TEXT(".first_rs_arrival"),data?reinterpret_cast<UPTRINT>(data->GetArrivalAddress()):0);
    add(TEXT(".first_rs_quality"),data?reinterpret_cast<UPTRINT>(data->GetQualityAddress()):0);
    add(TEXT(".first_rs_advance_serial"),data?data->GetAdvanceSerial():0);
    add(TEXT(".first_rs_numeric_control"),data?data->GetNumericControl():0);
    add(TEXT(".first_rs_generation"),data?data->GetOriginIdentity().InputGeneration:0);
    add(TEXT(".first_rs_frame"),data?data->GetOriginIdentity().Frame:0);
    add(TEXT(".first_rs_step"),data?data->GetOriginIdentity().StepSerial:0);
    add(TEXT(".first_rs_window"),data?data->GetOriginIdentity().WindowSerial:0);
    add(TEXT(".first_rs_nonce"),data?data->GetOriginIdentity().WindowNonce:0);
    ok=out.Float(prefix+TEXT(".first_rs_seconds"),data?data->GetSeconds():0.0f,budget)&&ok;
    return ok;
}

bool AppendNativeHitIdentityFields(FNativePrepareGTFields& out,const FString& prefix,
    const SHitResult& hit,Analytic::Certified::FDistanceBudget& budget)
{
    // Defined where USSubBody is complete. Weak conversion retains index and
    // serial without resolving either UObject; all aliases share the codec.
    return out.WeakIdentity(prefix+TEXT(".component_token"),TWeakObjectPtr<const UObject>(hit.Component),budget)&&
        out.WeakIdentity(prefix+TEXT(".body_token"),TWeakObjectPtr<const UObject>(hit.SubBody),budget)&&
        out.SharedIdentity(prefix+TEXT(".ground_data"),TSharedPtr<const void,ESPMode::ThreadSafe>(
            hit.PreparedGroundEvidence,static_cast<const void*>(hit.PreparedGroundEvidence.Get())),budget)&&
        AppendFirstRsResponseDataFields(out,prefix,hit.FirstRsResponseData,budget);
}
