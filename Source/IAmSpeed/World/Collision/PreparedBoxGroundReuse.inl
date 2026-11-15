// Same IAS TU, after NativeBoxCacheDetail; no World query/serial is minted.
namespace BoxGroundReuseDetail
{
bool SameStep(const FStepIdentity& a,const FStepIdentity& b,bool exactSerial)
{
    return a.Host==b.Host&&a.Bridge==b.Bridge&&a.World==b.World&&a.Worker==b.Worker&&a.Provider==b.Provider&&
        a.Lease==b.Lease&&a.InputGeneration==b.InputGeneration&&a.Frame==b.Frame&&a.WindowSerial==b.WindowSerial&&
        a.WindowNonce==b.WindowNonce&&a.StepSerial==b.StepSerial&&a.QueryEpoch==b.QueryEpoch&&a.Iteration==b.Iteration&&
        a.NumericControl==b.NumericControl&&a.ThreadId==b.ThreadId&&SameFloat(a.Delta,b.Delta)&&SameFloat(a.SimTime,b.SimTime)&&
        SameFloat(a.Remaining,b.Remaining)&&SameFloat(a.TimePassed,b.TimePassed)&&
        (exactSerial?a.QuerySerial==b.QuerySerial:a.QuerySerial<=b.QuerySerial);
}
bool IsFullFrame()
{
    if(!Requested())return false;OnePausedWork::FCanonicalFrameWindowPtr w;
    if(!OnePausedWork::ReadExecutionWindow(w)||!w)Refuse("Box outer observation has no native window");
    return w->GetPurpose()==OnePausedWork::EPurpose::FullFrame;
}
bool SameIgnored(const TArray<const UPrimitiveComponent*>& a,const TArray<const UPrimitiveComponent*>& b)
{if(a.Num()!=b.Num())return false;for(int32 i=0;i<a.Num();++i)if(a[i]!=b[i])return false;return true;}
}
void FComponentSweepScope::ReadBoxOuterState(const UBoxSubBody& body,FNativeBoxGroundCacheValues& cache,
    SHitResult& future,TArray<const UPrimitiveComponent*>& ignored)
{
    ReadBoxGroundCacheValues(body,cache);future=body.FutureHit;ignored.Reset();
    // Raw pointers are copied, never dereferenced. The decision is the actual
    // native Contains result, NOT a worker Weak.Get/index-only substitute.
    for(const auto* p:body.IgnoredComponents)ignored.Add(p);
}
TSharedPtr<FOrdinaryBoxGroundReuseObservation,ESPMode::ThreadSafe>
FComponentSweepScope::ReadNativeBoxOuterSite(const UBoxSubBody* body,FStepIdentity& now)
{
    const ISpeedComponent* owner=nullptr;int32 index=INDEX_NONE;
    if(!body||MemberSweepScope||!ReadCurrentStep(now)||!StepScope||!StepScope->Snapshot||
        !ReadNativeSolidSite(static_cast<const ISolidSweeper*>(body),body,owner,index)||!ComponentSweepScope)
        Refuse("Box outer observation has no real current GT/body callback");
    auto* scope=ComponentSweepScope;auto observed=scope->Pending->Bodies[index].BoxGroundReuse;
    FCanonicalPreparedFrameClock clock;
    if(!JoinCanonicalPreparedFrameClock(now.Frame,owner->NumFrame(),clock))Refuse("Box outer native frame clock changed");
    if(!observed||observed->CallbackClosed||observed->Body!=body||observed->Owner!=owner||observed->BodyIndex!=index||
        observed->OwnerStableId!=scope->Pending->OwnerStableId||observed->ScopeSerial!=scope->Pending->ScopeSerial||
        observed->Snapshot!=StepScope->Snapshot||!(observed->FrameClock==clock)||!BoxGroundReuseDetail::SameStep(observed->EntryStep,now,false)||
        !SameKinematicBits(observed->OwnerState,owner->GetKinematicState())||
        !SameKinematicBits(observed->BodyState,body->GetKinematicState()))
        Refuse("Box outer observation identity or native state changed");
    return observed;
}
void FComponentSweepScope::BeginNativeBoxOuterSweep(const UBoxSubBody* body,const SHitResult& old,float horizon)
{
        PreparedExecution::FNativePlanarResponseCommitScope::RequireQueriesClosed();
    if(!BoxGroundReuseDetail::IsFullFrame())return;
    const ISpeedComponent* owner=nullptr;int32 index=INDEX_NONE;FStepIdentity now;
    if(!body||MemberSweepScope||!ReadCurrentStep(now)||!StepScope||!StepScope->Snapshot||
        !ReadNativeSolidSite(static_cast<const ISolidSweeper*>(body),body,owner,index)||
        !FMath::IsFinite(horizon)||horizon<0||!SameFloat(horizon,now.Remaining))
        Refuse("Box outer entry has no current ordinary callback/horizon");
    auto* scope=ComponentSweepScope;auto& slot=scope->Pending->Bodies[index];
    if(slot.BoxGroundReuse)Refuse("Box outer entry duplicated");
    TSharedPtr<FOrdinaryBoxGroundReuseObservation,ESPMode::ThreadSafe> v(new FOrdinaryBoxGroundReuseObservation);
    v->EntryStep=now;v->Snapshot=StepScope->Snapshot;v->Body=body;v->Owner=owner;v->BodyIndex=index;
    v->OwnerStableId=scope->Pending->OwnerStableId;v->ScopeSerial=scope->Pending->ScopeSerial;
    if(!JoinCanonicalPreparedFrameClock(now.Frame,owner->NumFrame(),v->FrameClock))Refuse("Box entry native frame clock unavailable");
    v->OwnerState=owner->GetKinematicState();v->BodyState=body->GetKinematicState();v->Horizon=horizon;v->OldGround=old;
    ReadBoxOuterState(*body,v->EntryCache,v->EntryFuture,v->EntryIgnored);
    if(!SameNativeAnalyticContactValues(old,v->EntryCache.GroundHit)||
        !SameNativeAnalyticContactValues(v->EntryFuture,SHitResult{}))Refuse("Box entry differs from native old-cache/reset values");
    slot.BoxGroundReuse=MoveTemp(v);
}
void FComponentSweepScope::BeginNativeBoxGroundReuse(const UBoxSubBody* body,const SHitResult& old,
    float horizon,const float& seconds,uint8 candidateFlags)
{
        PreparedExecution::FNativePlanarResponseCommitScope::RequireQueriesClosed();
    if(!BoxGroundReuseDetail::IsFullFrame())return;FStepIdentity now;auto v=ReadNativeBoxOuterSite(body,now);
    if(v->Entered||v->NativeReturnObserved||candidateFlags||!SameFloat(horizon,v->Horizon)||
        !SameNativeAnalyticContactValues(old,v->OldGround))Refuse("Box cache fallback has no unique actual four-false branch");
    v->Entered=true;v->DecisionStep=now;v->BeforeSeconds=seconds;v->CandidateFlags=candidateFlags;
    ReadBoxOuterState(*body,v->BeforeCache,v->BeforeFuture,v->BeforeIgnored);
}
void FComponentSweepScope::FinishNativeBoxGroundReuse(const UBoxSubBody* body,
    const FNativeBoxGroundReuseDecisionValues& decision,bool returned,const float& seconds)
{
    if(!BoxGroundReuseDetail::IsFullFrame())return;FStepIdentity now;auto v=ReadNativeBoxOuterSite(body,now);
    const auto outcome=ClassifyNativeBoxGroundReuseDecision(decision);using E=ENativeBoxGroundReuseOutcome;
    if(!v->Entered||v->NativeReturnObserved||v->CandidateFlags||!BoxGroundReuseDetail::SameStep(v->DecisionStep,now,true)||
        outcome==E::Incomplete||outcome==E::NotEntered||returned!=(outcome==E::Reused))
        Refuse("Box cache fallback decisions or native branch return missing");
    ReadBoxOuterState(*body,v->AfterCache,v->AfterFuture,v->AfterIgnored);
    auto expectedCache=v->BeforeCache;SHitResult expectedFuture=v->BeforeFuture;
    if(outcome==E::Reused||outcome==E::Ignored)expectedCache.GroundHit=v->OldGround;
    if(outcome==E::Reused)expectedFuture=v->OldGround;
    if(outcome==E::Reused)expectedFuture.TOI=0.0f; // expected COPY, never delivered/mutated native hit
    if(!NativeBoxCacheDetail::Same(expectedCache,v->AfterCache)||
        !SameNativeAnalyticContactValues(expectedFuture,v->AfterFuture)||
        !BoxGroundReuseDetail::SameIgnored(v->BeforeIgnored,v->AfterIgnored)||
        !(outcome==E::Reused?SameFloat(seconds,0.0f):SameFloat(seconds,v->BeforeSeconds)))
        Refuse("Box cache fallback native assignments changed");
    // No current FrameTag, bHit flag, source ID, plane shape, zero gap or
    // resolved-frame freshness is manufactured from this heuristic cache.
    v->Decision=decision;v->Outcome=outcome;v->ReturnedHit=returned;v->AfterSeconds=seconds;
    v->ReturnStep=now;v->NativeReturnObserved=true;
}
void FComponentSweepScope::CloseNativeBoxOuterReturn(const UBoxSubBody* body,bool hit,float seconds)
{
    // Called only for an existing FullFrame observation. A different purpose
    // cannot silently skip its closure and still finish the body inventory.
    if(!BoxGroundReuseDetail::IsFullFrame())Refuse("Box outer observation left its native FullFrame window");
    FStepIdentity now;auto v=ReadNativeBoxOuterSite(body,now);
    if(v->Entered)
    {
        FNativeBoxGroundCacheValues cache;SHitResult future;TArray<const UPrimitiveComponent*> ignored;
        ReadBoxOuterState(*body,cache,future,ignored);
        if(!v->NativeReturnObserved||!BoxGroundReuseDetail::SameStep(v->ReturnStep,now,true)||hit!=v->ReturnedHit||
            !SameFloat(seconds,v->AfterSeconds)||!NativeBoxCacheDetail::Same(cache,v->AfterCache)||
            !SameNativeAnalyticContactValues(future,v->AfterFuture)||!BoxGroundReuseDetail::SameIgnored(ignored,v->AfterIgnored))
            Refuse("Box cache observation differs from actual final callback values");
    }
    // A non-entered branch remains only an entry observation; no claim about
    // its World, persistent sphere or other dynamic winner is introduced.
    v->CallbackClosed=true;
}
