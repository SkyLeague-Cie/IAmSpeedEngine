// Included inside PreparedExecution in the existing native execution TU.
// This state belongs to ONE actual ordinary Step/epoch. It seals real return
// values; no dynamic/cache clearance or permission to integrate follows.
struct FOrdinaryOwnerSweepReturn
{
    const ISpeedComponent* Owner=nullptr;uint64 StableId=0;int32 Order=INDEX_NONE;
    FStepIdentity EntryStep,ReturnStep;float LastSubDelta=0;
    FKinematicState EntryOwner,EffectiveOwner,ReturnedOwner;
    TArray<const USSubBody*> Bodies;
    TArray<FKinematicState> EntryBodies,ReturnedBodies;
    FVector LinearReaction=FVector::ZeroVector,AngularReaction=FVector::ZeroVector;
    bool ReactionsObserved=false,Returned=false;
    FComponentSweepEvidencePtr Component;
};
struct FOrdinarySweepReturnState
{
    TArray<TSharedPtr<const FOrdinaryOwnerSweepReturn,ESPMode::ThreadSafe>> Rows;
    TSharedPtr<FOrdinaryOwnerSweepReturn,ESPMode::ThreadSafe> Pending;
    TSharedPtr<FOrdinaryWorldSelectionState,ESPMode::ThreadSafe> World;
};
void FStepScope::ResetOrdinaryOwnerSweepReturns()
{
    if(OrdinaryReturns&&(OrdinaryReturns->Pending||HasOpenOrdinaryWorldSelection()))Refuse("Ordinary owner/World selector did not close before next epoch");
    OrdinaryReturns.Reset();
}
void FStepScope::BeginOrdinaryOwnerSweep(const ISpeedComponent* owner,float lastSubDelta)
{
    if(!Active||!Window||Window->GetPurpose()!=OnePausedWork::EPurpose::FullFrame)return;
    FStepIdentity now;
    if(StepScope!=this||!SweepPhase||ComponentSweepScope||MemberSweepScope||!ReadCurrentStep(now)||
        !Registry||!owner||!FMath::IsFinite(lastSubDelta)||lastSubDelta<0||lastSubDelta>now.Delta)
        Refuse("Ordinary owner sweep entry is outside actual current Step");
    if(!OrdinaryReturns)OrdinaryReturns=MakeUnique<FOrdinarySweepReturnState>();
    const int32 index=OrdinaryReturns->Rows.Num();const auto& ordered=Registry->GetOrderedAdapters();
    if(OrdinaryReturns->Pending||!ordered.IsValidIndex(index)||ordered[index]!=owner)
        Refuse("Ordinary owner sweep entry is duplicated or reordered");
    auto value=MakeShared<FOrdinaryOwnerSweepReturn,ESPMode::ThreadSafe>();
    value->Owner=owner;value->StableId=Registry->FindStableId(*owner);value->Order=index;value->EntryStep=now;
    value->LastSubDelta=lastSubDelta;value->EntryOwner=owner->GetKinematicState();
    if(!value->StableId||!FiniteState(value->EntryOwner))Refuse("Ordinary owner entry values unavailable");
    const auto bindings=Snapshot?Snapshot->GetNativeOwners():FPreparedNativeOwnerBindingsPtr{};
    const auto* native=bindings?bindings->Find(owner):nullptr;
    if(!native||!native->BodyCensusCaptured||native->Bodies.Num()!=owner->GetSubBodies().Num()||
        bindings->GetGeneration()!=now.InputGeneration)Refuse("Ordinary owner entry has no matching GT raw census");
    for(int32 i=0;i<owner->GetSubBodies().Num();++i)
    {
        const auto* body=owner->GetSubBodies()[i];
        if(!body||native->Bodies[i].Index!=i||native->Bodies[i].Body!=body||
            body->GetParentComponent()!=owner||!FiniteState(body->GetKinematicState()))
            Refuse("Ordinary owner entry body state or census unavailable");
        value->Bodies.Add(body);value->EntryBodies.Add(body->GetKinematicState());
    }
    OrdinaryReturns->Pending=MoveTemp(value);
}
void FStepScope::ObserveOrdinarySweepReactions(const ISpeedComponent& owner,const FVector& linear,const FVector& angular)
{
    auto* scope=StepScope;
    if(!scope||!scope->Active||!scope->Window||scope->Window->GetPurpose()!=OnePausedWork::EPurpose::FullFrame)return;
    FStepIdentity now;
    if(!ReadCurrentStep(now)||!scope->SweepPhase||ComponentSweepScope||MemberSweepScope||
        !scope->OrdinaryReturns||!scope->OrdinaryReturns->Pending)
        Refuse("Ordinary reaction values have no current owner sweep entry");
    auto& row=*scope->OrdinaryReturns->Pending;
    if(row.Owner!=&owner||row.ReactionsObserved||row.EntryStep.Lease!=now.Lease||row.EntryStep.QueryEpoch!=now.QueryEpoch||
        row.EntryStep.QuerySerial!=now.QuerySerial||row.EntryStep.StepSerial!=now.StepSerial||
        row.EntryStep.WindowSerial!=now.WindowSerial||row.EntryStep.WindowNonce!=now.WindowNonce||
        !SameKinematicBits(row.EntryOwner,owner.GetKinematicState())||linear.ContainsNaN()||angular.ContainsNaN()||
        row.Bodies.Num()!=owner.GetSubBodies().Num())Refuse("Ordinary reaction entry or owner values changed");
    for(int32 i=0;i<row.Bodies.Num();++i)
        if(row.Bodies[i]!=owner.GetSubBodies()[i]||!SameKinematicBits(row.EntryBodies[i],row.Bodies[i]->GetKinematicState()))
            Refuse("Ordinary reaction input body state changed");
    row.LinearReaction=linear;row.AngularReaction=angular;row.ReactionsObserved=true;
}
void FStepScope::EndOrdinaryOwnerSweep(const ISpeedComponent* owner,const ::SComponentTOI& actual)
{
    if(!Active||!Window||Window->GetPurpose()!=OnePausedWork::EPurpose::FullFrame)return;
    FStepIdentity now;
    if(StepScope!=this||!SweepPhase||ComponentSweepScope||MemberSweepScope||!ReadCurrentStep(now)||
        !OrdinaryReturns||!OrdinaryReturns->Pending)Refuse("Ordinary owner return has no current entry");
    auto row=OrdinaryReturns->Pending;const auto& bundle=actual.PreparedEvidence;
    if(row->Owner!=owner||!row->ReactionsObserved||!bundle||!bundle->Sealed||!bundle->SelectionObserved||
        bundle->Owner!=owner||bundle->OwnerStableId!=row->StableId||bundle->Bodies.Num()!=row->Bodies.Num()||
        row->EntryStep.Lease!=now.Lease||row->EntryStep.QueryEpoch!=now.QueryEpoch||
        row->EntryStep.StepSerial!=now.StepSerial||row->EntryStep.WindowSerial!=now.WindowSerial||row->EntryStep.WindowNonce!=now.WindowNonce||
        bundle->CompletionStep.QuerySerial!=now.QuerySerial||bundle->CompletionStep.QueryEpoch!=now.QueryEpoch||
        bundle->CompletionStep.StepSerial!=now.StepSerial||bundle->CompletionStep.Lease!=now.Lease||
        bundle->SelectedHit!=actual.bHit||!SameFloat(bundle->SelectedSeconds,actual.TOI)||owner->GetSubBodies().Num()!=row->Bodies.Num())
        Refuse("Ordinary owner return or component selection is stale or incomplete");
    if(actual.bHit&&(!actual.Resolver.HasSameIndexAndSerialNumber(bundle->SelectedResolver)||
        actual.PairKey!=bundle->SelectedPairKey||!SameNativeAnalyticContactValues(actual.Hit,bundle->SelectedOrientedHit)))
        Refuse("Ordinary owner returned winner values changed");
    // At this outer callsite ON_SCOPE_EXIT has ALREADY restored A/alpha and
    // resynchronized bodies. Preserve it separately from effective sweep PVA;
    // this is not an assertion that either is a qualified advance trajectory.
    row->EffectiveOwner=bundle->SweepOwnerState;row->ReturnedOwner=owner->GetKinematicState();
    if(!FiniteState(row->ReturnedOwner))Refuse("Ordinary restored owner state is nonfinite");
    for(int32 i=0;i<row->Bodies.Num();++i)
    {
        if(row->Bodies[i]!=owner->GetSubBodies()[i]||bundle->Bodies[i].RawBodyIdentity!=row->Bodies[i]||!FiniteState(row->Bodies[i]->GetKinematicState()))
            Refuse("Ordinary restored body census or state changed");
        row->ReturnedBodies.Add(row->Bodies[i]->GetKinematicState());
    }
    row->Component=bundle;row->ReturnStep=now;row->Returned=true;
    OrdinaryReturns->Rows.Add(row);OrdinaryReturns->Pending.Reset();
}
void FStepScope::CheckOrdinaryOwnerSweepReturns(TConstArrayView<FComponentSweepEvidencePtr> bundles) const
{
    if(!Active||!Window||Window->GetPurpose()!=OnePausedWork::EPurpose::FullFrame)return;
    FStepIdentity now;
    if(!ReadCurrentStep(now)||!OrdinaryReturns||OrdinaryReturns->Pending||
        OrdinaryReturns->Rows.Num()!=Registry->GetOrderedAdapters().Num()||bundles.Num()!=OrdinaryReturns->Rows.Num())
        Refuse("Ordinary all-owner returned sweep inventory is incomplete");
    for(int32 i=0;i<bundles.Num();++i)
    {
        const auto& row=OrdinaryReturns->Rows[i];const auto* owner=Registry->GetOrderedAdapters()[i];
        if(!row||!row->Returned||row->Order!=i||row->Owner!=owner||row->StableId!=Registry->FindStableId(*owner)||
            row->Component!=bundles[i]||row->ReturnStep.QueryEpoch!=now.QueryEpoch||row->ReturnStep.StepSerial!=now.StepSerial||
            row->ReturnStep.Lease!=now.Lease||row->ReturnStep.WindowSerial!=now.WindowSerial||row->ReturnStep.WindowNonce!=now.WindowNonce||
            !SameKinematicBits(row->ReturnedOwner,owner->GetKinematicState())||owner->GetSubBodies().Num()!=row->Bodies.Num())
            Refuse("Ordinary all-owner restored return state or identity changed");
        for(int32 j=0;j<row->Bodies.Num();++j)
            if(row->Bodies[j]!=owner->GetSubBodies()[j]||!SameKinematicBits(row->ReturnedBodies[j],row->Bodies[j]->GetKinematicState()))
                Refuse("Ordinary all-owner restored body state changed");
    }
}
