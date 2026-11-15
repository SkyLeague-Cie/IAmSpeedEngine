// Included in the native execution TU. This is a distinct, closed private
// consumed-prefix transition. FullFrame RequireAdvanceEvidence is unchanged.
namespace {std::atomic<uint64> NextConsumedPrefixAdvanceSerial{0};}
bool FStepScope::HasCurrentConsumedPrefixIdentity() const
{
    if(!Active||StepScope!=this||!Window||!OnePausedWork::HasConsumedSphereAdvancePrefix(Window->GetPurpose())||
        !Window->IsOpen()||Poisoned||MemberSweepScope||ComponentSweepScope||!HostScope||!RsCapture||!RsCapture->IsClosed()||
        !RsCapture->AdvanceSource||!RsCapture->AdvanceSource->SealedDuringMember||SweepPhase||
        !PreparationPrefix::FCursor::IsCurrentConsumedPrefixNativeWindow(*Identity.Bridge)||
        !IsCurrentWorker(Identity.Worker)||Identity.ThreadId!=FPlatformTLS::GetCurrentThreadId())return false;
    OnePausedWork::FCanonicalFrameWindowPtr current;
    if(!OnePausedWork::ReadExecutionWindow(current)||current!=Window||current->GetSerial()!=Identity.WindowSerial||
        current->GetNonce()!=Identity.WindowNonce||current->GetSnapshot()!=Snapshot||Identity.Provider->GetPreparedSnapshot()!=Snapshot||
        Identity.Host->GetInputWorkerGeneration()!=Identity.InputGeneration||Identity.Bridge->GetStaticCollisionWorld()!=Identity.Provider||
        Identity.Host->GetWorld()!=Identity.World||Identity.Bridge->GetWorld()!=Identity.World||
        !FCanonicalFramePreflight::HasUnchangedRawPreparationCensus(*Identity.Bridge,*Snapshot,Identity.StepSerial))return false;
    if(OnePausedWork::HasBoundedRsContactPrefix(Window->GetPurpose())&&!RsCapture->HasBoundedFirstArrivalRsQuality())return false;
    const auto& proof=*RsCapture->AdvanceSource;const auto& origin=proof.Request->GetStep();
    const auto fp=Analytic::InspectCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    return proof.Request==RsCapture->Request&&proof.Prefix==RsCapture->TemporalPrefix&&proof.PhysicalSourcePlan&&
        origin.Lease==this&&origin.Host==Identity.Host&&origin.Bridge==Identity.Bridge&&origin.Worker==Identity.Worker&&
        origin.World==Identity.World&&origin.Provider==Identity.Provider&&origin.InputGeneration==Identity.InputGeneration&&
        origin.Frame==Identity.Frame&&origin.StepSerial==Identity.StepSerial&&origin.WindowSerial==Identity.WindowSerial&&
        origin.WindowNonce==Identity.WindowNonce&&origin.QuerySerial==1&&Identity.QuerySerial==1&&origin.QueryEpoch==2&&Identity.QueryEpoch==3&&
        fp.Status==Analytic::EAnalyticNumericStatus::Complete&&fp.ControlFingerprint()==Identity.NumericControl&&
        proof.NumericControl==Identity.NumericControl;
}
void FStepScope::CheckConsumedPrefixOwners(int32 completed) const
{
    if(!HasCurrentConsumedPrefixIdentity()||completed<0||completed>3)Refuse("Consumed-prefix native identity is stale");
    const auto& proof=*RsCapture->AdvanceSource;const auto& owners=Registry->GetOrderedAdapters();
    if(owners.Num()!=3||proof.Owners.Num()!=3)Refuse("Consumed-prefix owner inventory differs");
    for(int32 i=0;i<3;++i)
    {
        const auto& row=proof.Owners[i];const auto* owner=owners[i];const auto* retained=Snapshot->GetNativeOwners()->Find(owner);
        if(!owner||owner!=row.Owner||!retained||!retained->BodyCensusCaptured||owner->GetPublishedSimulationStableId()!=row.StableId||
            owner->NumFrame()!=row.Frame||owner->IsFrozen()!=row.Frozen||
            &owner->GetKinematicState()!=row.Stored||&owner->GetKinematicStateForFrame(row.Frame)!=row.Selected||
            (!row.Frozen&&row.Stored!=row.Selected)||!SameVector(owner->GetPhysCenterOfMassLocal(),row.LocalCOM)||
            !SameFloat(owner->GetPhysMaxSpeed(),row.MaximumSpeed)||!SameFloat(owner->GetPhysMaxAngularSpeed(),row.MaximumAngularSpeed)||
            !owner->GetPhysicalConstraints().IsEmpty()||
            !SameKinematicBits(owner->GetKinematicState(),i<completed?row.After:row.Before))
            Refuse("Consumed-prefix actual owner/caps/history/COM/state changed");
        if(!row.Frozen&&(owner->IsPrescribedMotion()||owner->IsPhysicsSleeping()))Refuse("Consumed-prefix free branch changed");
#if WITH_DEV_AUTOMATION_TESTS
        if(!row.Frozen&&Speed::Testing::IsCcdPrimitiveCaptureEnabledForCurrentThread(owner))
            Refuse("Consumed-prefix CCD observer has unmodelled pre-integrate effects");
#endif
        const auto& bodies=owner->GetSubBodies();const auto& expected=i<completed?row.BodiesAfter:row.BodiesBefore;
        if(bodies.Num()!=expected.Num()||retained->Bodies.Num()!=expected.Num())Refuse("Consumed-prefix actual child inventory changed");
        for(int32 j=0;j<bodies.Num();++j)
            if(!bodies[j]||retained->Bodies[j].Index!=j||retained->Bodies[j].Body!=bodies[j]||bodies[j]->GetParentComponent()!=owner||
                !SameKinematicBits(bodies[j]->GetKinematicState(),expected[j]))Refuse("Consumed-prefix actual child poststate differs");
    }
}
float FStepScope::BeginTerminalConsumedPrefixAdvance()
{
    if(!HasCurrentConsumedPrefixIdentity()||!PreparationPrefix::FCursor::IsCurrentConsumedPrefixSite(INDEX_NONE))
        Refuse("Consumed-prefix advance is outside its distinct native global site");
    auto* cursor=PreparationPrefix::FCursor::Current;auto& actual=cursor->Contract->RsAdvance;
    if(actual.Started||actual.Serial||!RsCapture->HasPositiveConsumedPrefix()||!RsCapture->TemporalPrefix)
        Refuse("Consumed-prefix advance cannot be repeated or use a zero/unknown prefix");
    const float seconds=RsCapture->TemporalPrefix->GetConsumedSeconds();uint32 bits=0;FMemory::Memcpy(&bits,&seconds,4);
    if(bits<0x00800000u||bits>=0x7f800000u||seconds>Identity.Remaining)Refuse("Consumed-prefix seconds are not a positive captured float");
    CheckConsumedPrefixOwners(0);
    uint64 serial=NextConsumedPrefixAdvanceSerial.load();
    do {if(serial==MAX_uint64)Refuse("Consumed-prefix serial exhausted");}
    while(!NextConsumedPrefixAdvanceSerial.compare_exchange_weak(serial,serial+1));
    actual.Serial=serial+1;actual.Seconds=seconds;actual.NumericControl=Identity.NumericControl;actual.Started=true;
    return seconds;
}
void FStepScope::BeginTerminalConsumedPrefixAdvanceOwner(const ISpeedComponent* owner,int32 index)
{
    if(!HasCurrentConsumedPrefixIdentity()||index<0||index>2||!PreparationPrefix::FCursor::IsCurrentConsumedPrefixSite(index))
        Refuse("Consumed-prefix owner is outside its distinct actual native site");
    auto& actual=PreparationPrefix::FCursor::Current->Contract->RsAdvance;
    if(!actual.Started||actual.Complete||actual.ActiveIndex!=INDEX_NONE||actual.CompletedOwners!=index||
        owner!=RsCapture->AdvanceSource->Owners[index].Owner||actual.AttemptedMask!=(uint8(1u<<index)-1))
        Refuse("Consumed-prefix owner call is repeated, reordered or stale");
    CheckConsumedPrefixOwners(index);actual.AttemptedMask|=uint8(1u<<index);actual.ActiveIndex=index;
}
void FStepScope::RecordTerminalConsumedPrefixAdvanceStates(const ISpeedComponent* owner,int32 index,bool returned)
{
    // The original native call site records return/partial values BEFORE any
    // post-effect identity/FP/state comparison can refuse. Copying is not proof.
    if(!Active||StepScope!=this||!Window||!OnePausedWork::HasConsumedSphereAdvancePrefix(Window->GetPurpose())||
        !RsCapture||!RsCapture->AdvanceSource||index<0||index>2||RsCapture->AdvanceSource->Owners.Num()!=3||
        owner!=RsCapture->AdvanceSource->Owners[index].Owner)return;
    const auto contract=Window->GetPreparationContract();if(!contract)return;
    auto& actual=contract->RsAdvance;
    if(!actual.Started||actual.ActiveIndex!=index||actual.CompletedOwners!=index||(actual.StateCaptureAttemptedMask&(uint8(1u<<index))))return;
    if(returned)actual.ReturnedMask|=uint8(1u<<index);
    actual.StateCaptureAttemptedMask|=uint8(1u<<index);
    FConsumedPrefixActualStates observed;observed.AfterOwnerIndex=index;observed.CallbackReturned=returned;
    if(!Registry||!Snapshot||!Snapshot->GetNativeOwners()||
        !FCanonicalFramePreflight::HasUnchangedRawPreparationCensus(*Identity.Bridge,*Snapshot,Identity.StepSerial))
    {actual.ActualStates.Add(MoveTemp(observed));return;}
    const auto& owners=Registry->GetOrderedAdapters();
    if(owners.Num()!=3){actual.ActualStates.Add(MoveTemp(observed));return;}
    for(const auto* native:owners)
    {
        const auto* retained=Snapshot->GetNativeOwners()->Find(native);
        if(!native||!retained||native->GetSubBodies().Num()!=retained->Bodies.Num())
        {actual.ActualStates.Add(MoveTemp(observed));return;}
        observed.Owners.Add(native->GetKinematicState());auto& bodies=observed.Bodies.AddDefaulted_GetRef();
        for(int32 j=0;j<retained->Bodies.Num();++j)
        {
            const auto* body=retained->Bodies[j].Body;
            if(!body||retained->Bodies[j].Index!=j||native->GetSubBodies()[j]!=body)
            {actual.ActualStates.Add(MoveTemp(observed));return;}
            bodies.Add(body->GetKinematicState());
        }
    }
    actual.StateCapturedMask|=uint8(1u<<index);actual.ActualStates.Add(MoveTemp(observed));
}
void FStepScope::EndTerminalConsumedPrefixAdvanceOwner(const ISpeedComponent* owner,int32 index)
{
    RecordTerminalConsumedPrefixAdvanceStates(owner,index,true);
    if(!HasCurrentConsumedPrefixIdentity()||index<0||index>2||!PreparationPrefix::FCursor::IsCurrentConsumedPrefixSite(index))
        Refuse("Consumed-prefix actual owner return escaped its native site");
    auto& actual=PreparationPrefix::FCursor::Current->Contract->RsAdvance;
    if(actual.ActiveIndex!=index||actual.CompletedOwners!=index||owner!=RsCapture->AdvanceSource->Owners[index].Owner||
        !(actual.ReturnedMask&(uint8(1u<<index)))||!(actual.StateCapturedMask&(uint8(1u<<index))))
        Refuse("Consumed-prefix actual return/state observation unavailable");
    CheckConsumedPrefixOwners(index+1);actual.VerifiedMask|=uint8(1u<<index);actual.CompletedOwners=index+1;actual.ActiveIndex=INDEX_NONE;
}
void FStepScope::EndTerminalConsumedPrefixAdvance()
{
    if(!HasCurrentConsumedPrefixIdentity()||!PreparationPrefix::FCursor::IsCurrentConsumedPrefixSite(INDEX_NONE))
        Refuse("Consumed-prefix completion is outside the global native site");
    auto& actual=PreparationPrefix::FCursor::Current->Contract->RsAdvance;
    if(!actual.Started||actual.Complete||actual.CompletedOwners!=3||actual.ActiveIndex!=INDEX_NONE||
        actual.AttemptedMask!=7||actual.ReturnedMask!=7||actual.VerifiedMask!=7||actual.StateCapturedMask!=7||actual.ActualStates.Num()!=3)Refuse("Consumed-prefix actual call inventory incomplete");
    CheckConsumedPrefixOwners(3);actual.Complete=true;
}
bool FStepScope::ReadTerminalConsumedPrefixAdvanceObservation(FTerminalConsumedPrefixAdvanceObservation& out)
{
    out={};const auto* s=StepScope;auto* c=PreparationPrefix::FCursor::Current;
    if(!s||!c||!s->HasCurrentConsumedPrefixIdentity()||!PreparationPrefix::FCursor::IsCurrentConsumedPrefixObservation())return false;
    out=c->Contract->RsAdvance;return true;
}
