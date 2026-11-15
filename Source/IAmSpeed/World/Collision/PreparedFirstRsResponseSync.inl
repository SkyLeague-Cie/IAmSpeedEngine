// p8 native call. Prediction comes only from immutable pre-effect Source Data.
ISpeedComponent* FStepScope::BeginTerminalFirstRsResponseSynchronization()
{
    if(!HasCurrentFirstRsResponseIdentity(true)||!PreparationPrefix::FCursor::IsCurrentRsResponseSyncSite())
        Refuse("Rs response synchronization is outside its distinct native site");
    const auto& contract=Window->GetPreparationContract();const auto& data=*contract->FirstRsResponseData;auto& sync=contract->FirstRsResponseSync;
    const auto& response=contract->FirstRsResponse;const auto& proof=*data.AdvanceSource;
    if(!data.HasSynchronizationExpectation||!contract->HasCompletedNativeRsResponsePrefix()||sync.Attempted||sync.ResponseSerial||sync.Data||
        sync.Returned||sync.CaptureAttempted||sync.RawCaptured||sync.SourceCaptured||sync.Verified||sync.Complete||sync.Failed||!sync.Owners.IsEmpty()||!sync.Bodies.IsEmpty()||
        !FiniteState(data.BodyAfterSynchronization)||FrameParallel::IsJoinedActorBaseOwnerScope1430())
        Refuse("Rs response synchronization is repeated, unsealed or has an active joined-base scope");
    CheckFirstRsResponseOwners(3,true,false);
    const auto& local=proof.BallValues.Local;const FRotator euler=data.Body->GetRelativeRotation();
    if(proof.Owners.Num()!=3||local.bWheel||!SameVector(data.Owner->GetPhysCenterOfMassLocal(),proof.BallValues.LocalCOM)||
        !SameVector(proof.Owners[1].LocalCOM,proof.BallValues.LocalCOM)||
        !SameVector(data.Body->GetRelativeLocation(),local.RelativeLocation)||!SameVector(data.Body->GetRelativeLocation(),data.SynchronizationRelativeLocation)||
        !SameDouble(euler.Pitch,data.SynchronizationRelativeEuler.Pitch)||!SameDouble(euler.Yaw,data.SynchronizationRelativeEuler.Yaw)||
        !SameDouble(euler.Roll,data.SynchronizationRelativeEuler.Roll)||!SameQuat(euler.Quaternion(),local.RelativeRotation))
        Refuse("Rs synchronization actual COM/local/Euler differs from its sealed Source inputs");
    const auto origin=Speed::CalculateOriginKinematicStateValues(data.Physical.OwnerAfter,proof.BallValues.LocalCOM);
    const auto expected=FrameParallel::CalculateSubBodyKinematics(origin,local);
    if(!FiniteState(expected)||!SameKinematicBits(expected,data.BodyAfterSynchronization))Refuse("Rs synchronization pure body expectation differs");
    sync.ResponseSerial=response.Serial;sync.NumericControl=Identity.NumericControl;sync.Data=contract->FirstRsResponseData;sync.Attempted=true;
    return const_cast<ISpeedComponent*>(data.Owner);
}
void FStepScope::RecordTerminalFirstRsResponseSynchronizationStates(const ISpeedComponent* owner,bool returned)
{
    if(!Active||StepScope!=this||!Window||!OnePausedWork::HasFirstRsResponseSyncPrefix(Window->GetPurpose()))return;
    const auto& contract=Window->GetPreparationContract();if(!contract||!contract->FirstRsResponseData)return;
    auto& sync=contract->FirstRsResponseSync;const auto& data=*contract->FirstRsResponseData;
    if(owner!=data.Owner||!sync.Attempted||sync.CaptureAttempted||sync.Complete||sync.Data!=contract->FirstRsResponseData)return;
    sync.Returned=sync.Returned||returned;sync.CaptureAttempted=true;sync.SourceFieldsRefusal=TEXT("Partial Core sync fields unavailable before bounded Source read");
    if(!Registry||!Snapshot||!Snapshot->GetNativeOwners()||
        !FCanonicalFramePreflight::HasUnchangedRawPreparationCensus(*Identity.Bridge,*Snapshot,Identity.StepSerial))
    {sync.Failed=true;return;}
    const auto& owners=Registry->GetOrderedAdapters();if(owners.Num()!=3){sync.Failed=true;return;}
    for(const auto* actual:owners)
    {
        const auto* binding=Snapshot->GetNativeOwners()->Find(actual);
        if(!actual||!binding||actual->GetSubBodies().Num()!=binding->Bodies.Num()){sync.Failed=true;return;}
        sync.Owners.Add(actual->GetKinematicState());auto& bodies=sync.Bodies.AddDefaulted_GetRef();
        for(int32 j=0;j<binding->Bodies.Num();++j)
        {
            const auto* child=binding->Bodies[j].Body;
            if(!child||binding->Bodies[j].Index!=j||actual->GetSubBodies()[j]!=child){sync.Failed=true;return;}
            bodies.Add(child->GetKinematicState());
        }
    }
    sync.RawCaptured=true;
    sync.SourceCaptured=PreparationPrefix::FCursor::CaptureFirstRsResponseSyncPartial(sync.SourceBallFields,sync.SourceFieldsRefusal);
    if(!returned||!sync.SourceCaptured)sync.Failed=true;
}
void FStepScope::EndTerminalFirstRsResponseSynchronization(const ISpeedComponent* owner)
{
    // Actual return and partial facts are retained BEFORE any postcheck.
    RecordTerminalFirstRsResponseSynchronizationStates(owner,true);
    if(!HasCurrentFirstRsResponseIdentity(true)||!PreparationPrefix::FCursor::IsCurrentRsResponseSyncSite())
        Refuse("Rs synchronization return escaped its native site");
    const auto& contract=Window->GetPreparationContract();const auto& data=*contract->FirstRsResponseData;auto& sync=contract->FirstRsResponseSync;
    if(owner!=data.Owner||!sync.Attempted||!sync.Returned||!sync.CaptureAttempted||!sync.RawCaptured||!sync.SourceCaptured||!sync.SourceBallFields||sync.Failed||sync.Complete||sync.Verified||
        sync.Data!=contract->FirstRsResponseData||sync.ResponseSerial!=contract->FirstRsResponse.Serial||sync.NumericControl!=Identity.NumericControl||
        sync.Owners.Num()!=3||sync.Bodies.Num()!=3||FrameParallel::IsJoinedActorBaseOwnerScope1430())
        Refuse("Rs synchronization actual partial/return/serial closure unavailable");
    CheckFirstRsResponseOwners(3,true,true);sync.Verified=true;sync.Complete=true;
}
bool FStepScope::ReadTerminalFirstRsResponseSyncObservation(FTerminalFirstRsResponseSyncObservation& out)
{
    out={};const auto* s=StepScope;
    if(!s||!s->HasCurrentFirstRsResponseIdentity(true)||!PreparationPrefix::FCursor::IsCurrentRsResponseSyncNativeWindow(*s->Identity.Bridge))return false;
    out=s->Window->GetPreparationContract()->FirstRsResponseSync;return true;
}
