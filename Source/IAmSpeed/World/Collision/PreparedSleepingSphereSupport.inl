// Included after ordinary native ground helpers in the same IAS TU.
namespace SleepingSupportDetail
{
bool Same(const FNativeSleepingSupportDecisionValues& a,const FNativeSleepingSupportDecisionValues& b)
{
    return a.MovementOwner==b.MovementOwner&&a.HasSupport==b.HasSupport&&a.Equilibrium==b.Equilibrium&&a.ForceSleeping==b.ForceSleeping&&
        a.Decision==b.Decision&&a.StableFrames==b.StableFrames&&SameVector(a.StoredNormal,b.StoredNormal)&&SameVector(a.StoredPoint,b.StoredPoint)&&
        SameVector(a.BranchNormal,b.BranchNormal)&&SameVector(a.COMVelocity,b.COMVelocity)&&SameVector(a.TangentialVelocity,b.TangentialVelocity)&&
        SameFloat(a.NormalSpeed,b.NormalSpeed)&&SameFloat(a.NativeWakeSpeed,b.NativeWakeSpeed);
}
bool Read(const USphereSubBody* body,float horizon,FNativeSolidStaticValues& value)
{
    value={};const auto* sweeper=static_cast<const ISolidSweeper*>(body);const ISpeedComponent* owner=nullptr;int32 index=INDEX_NONE;
    FStepIdentity step;uint64 member=0;float margin=0;
    if(!FComponentSweepScope::ReadNativeSolidSite(sweeper,body,owner,index)||!ReadCurrentStep(step)||!owner||
        !SameFloat(horizon,step.Remaining)||!ReadSphereMemberSweep(body,owner,member,margin))return false;
    FNativeSolidStaticValues v;v.Shape=ENativeSolidQueryShape::Sphere;v.Body=body;v.Sweeper=sweeper;v.Owner=owner;v.BodyIndex=index;
    v.BodyClass=body->GetClass();v.OwnerStableId=FStepScope::ReadOwnerStableId(owner);v.Horizon=horizon;
    if(!v.OwnerStableId||!JoinCanonicalPreparedFrameClock(step.Frame,owner->NumFrame(),v.FrameClock))return false;
    v.OwnerState=owner->GetKinematicState();v.BodyState=body->GetKinematicState();v.LocalOffset=body->GetLocalOffset();
    v.LocalCOM=owner->GetPhysCenterOfMassLocal();v.LocalRotation=body->GetLocalRotation();v.Frozen=owner->IsFrozen();v.Prescribed=owner->IsPrescribedMotion();
    v.PhysicalRadius=body->GetRadius();v.SupportRadius=body->GetWorldStaticSupportRadius();v.StaticContactOffset=body->GetWorldStaticContactOffsetCm();
    v.SphereMarginCaptured=true;v.SphereMargin=margin;v.SphereMemberSerial=member;
    const auto shape=sweeper->GetWorldStaticCollisionShape();const auto params=sweeper->BuildTraceParams();
    if(!shape.IsSphere()||!CaptureNativeSolidTraceValues(*sweeper,params,v.Trace)||!FiniteState(v.OwnerState)||!FiniteState(v.BodyState))return false;
    v.QueryRadius=shape.GetSphereRadius();
    // Stationary descriptor is FILTER/configuration data only. No native
    // World-call endpoint, CaptureNativeSolidCCD or query serial is fabricated.
    v.Start=v.End=v.BodyState.Location;
    v.Query=Analytic::FStaticWorldQueryAudit::MakeStrictSingleQuery1075(v.Start,v.End,v.BodyState.Rotation,shape,
        uint8(sweeper->GetCollisionChannel()),sweeper->GetResponseParams());
    value=MoveTemp(v);return true;
}
}
void FSphereMemberSweepScope::ObserveOrdinarySleepingSupport(const USphereSubBody* body,float horizon,
    TFunctionRef<bool(FNativeSleepingSupportDecisionValues&)> read)
{
    if(!Requested())return;OnePausedWork::FCanonicalFrameWindowPtr window;
    if(!OnePausedWork::ReadExecutionWindow(window)||!window)Refuse("Sleeping support branch has no current native window");
    if(window->GetPurpose()!=OnePausedWork::EPurpose::FullFrame)return;
    auto* member=MemberSweepScope;FStepIdentity step;const USphereSubBody* actual=nullptr;const void* owner=nullptr;
    if(!member||member->Completed||member->Interrupted||member->PendingGround||member->PendingOrdinaryGround||member->PendingSleepingSupport||
        !ReadBody(body,actual,owner)||actual!=body||!ReadCurrentStep(step)||!StepScope||!StepScope->Snapshot||!ComponentSweepScope)
        Refuse("Sleeping support branch lacks unique real member/callback binding");
    FNativeSolidStaticValues intrinsic;if(!SleepingSupportDetail::Read(body,horizon,intrinsic))Refuse("Sleeping support intrinsic values unavailable");
    FNativeSleepingSupportDecisionValues decision;
    if(!read(decision)||decision.MovementOwner!=intrinsic.Owner||!decision.Decision||!decision.ForceSleeping||
        !decision.HasSupport||!decision.Equilibrium||decision.StoredPoint.ContainsNaN()||decision.StoredNormal.ContainsNaN()||
        decision.BranchNormal.ContainsNaN()||decision.COMVelocity.ContainsNaN()||decision.TangentialVelocity.ContainsNaN()||
        !FMath::IsFinite(decision.NormalSpeed)||!FMath::IsFinite(decision.NativeWakeSpeed)||
        !SameVector(decision.COMVelocity,intrinsic.OwnerState.Velocity))Refuse("Sleeping support real native decision unavailable");
    const auto service=step.Provider->GetPreparedQueryService();FSerializedSphereSupportGeometryValues geometry;
    const FSerializedSphereSupportManifold manifold{decision.StoredNormal,decision.StoredPoint};
    if(!service||!service->MatchesPreparedSnapshot(StepScope->Snapshot)||!service->DescribeSerializedSphereSupportValues(intrinsic,manifold,geometry))
        Refuse("Sleeping serialized support finite-floor geometry/other-provider exclusion unknown");
    // Geometry did NOT execute a native query. Recheck identity before every
    // callback/getter reread, retaining the unchanged real query serial.
    FStepIdentity now;
    if(!ReadCurrentStep(now)||now.QueryEpoch!=step.QueryEpoch||now.QuerySerial!=step.QuerySerial||now.Lease!=step.Lease||
        now.WindowSerial!=step.WindowSerial||now.WindowNonce!=step.WindowNonce||!ReadBody(body,actual,owner)||actual!=body)
        Refuse("Sleeping support native context changed before reread");
    FNativeSolidStaticValues after;FNativeSleepingSupportDecisionValues afterDecision;
    if(!SleepingSupportDetail::Read(body,horizon,after)||!SameNativeSolidStaticValues(intrinsic,after)||!read(afterDecision)||
        !SleepingSupportDetail::Same(decision,afterDecision))Refuse("Sleeping support intrinsic/manifold/decision changed");
    TSharedPtr<FOrdinarySleepingSupportObservation,ESPMode::ThreadSafe> result(new FOrdinarySleepingSupportObservation);
    result->Step=step;result->Snapshot=StepScope->Snapshot;result->Intrinsic=MoveTemp(intrinsic);result->Decision=MoveTemp(decision);
    result->Geometry=MoveTemp(geometry);result->MemberSerial=member->Serial;member->PendingSleepingSupport=MoveTemp(result);
}
void FSphereMemberSweepScope::FinishSleepingSupport(bool deliveredHit,const SHitResult& delivered,float seconds)
{
    const auto result=PendingSleepingSupport;FStepIdentity now;FNativeSolidStaticValues current;
    if(!result||result->Closed||result->MemberSerial!=Serial||PendingGround||PendingOrdinaryGround||!ComponentSweepScope||
        !ReadCurrentStep(now)||now.Lease!=result->Step.Lease||now.QueryEpoch!=result->Step.QueryEpoch||now.QuerySerial!=result->Step.QuerySerial||
        now.StepSerial!=result->Step.StepSerial||now.WindowSerial!=result->Step.WindowSerial||now.WindowNonce!=result->Step.WindowNonce||
        result->Snapshot!=StepScope->Snapshot||deliveredHit||!SameNativeAnalyticContactValues(delivered,SHitResult{})||
        !SameFloat(seconds,result->Intrinsic.Horizon)||!SleepingSupportDetail::Read(Body,seconds,current)||
        !SameNativeSolidStaticValues(result->Intrinsic,current))Refuse("Sleeping support native no-query return changed");
    result->Closed=true;ComponentSweepScope->SealSleepingSupport(result);PendingSleepingSupport.Reset();
}
void FComponentSweepScope::SealSleepingSupport(TSharedPtr<const FOrdinarySleepingSupportObservation,ESPMode::ThreadSafe> observed)
{
    FStepIdentity now;
    if(!Active||Completed||ComponentSweepScope!=this||!observed||!observed->IsClosedObservation()||!Pending||
        !Pending->Bodies.IsValidIndex(CurrentIndex)||!ReadCurrentStep(now)||observed->Snapshot!=StepScope->Snapshot||
        observed->Step.Lease!=now.Lease||observed->Step.QueryEpoch!=now.QueryEpoch||observed->Step.QuerySerial!=now.QuerySerial||
        observed->Intrinsic.Owner!=Pending->Owner||observed->Intrinsic.Body!=Pending->Bodies[CurrentIndex].RawBodyIdentity||
        observed->Intrinsic.BodyIndex!=CurrentIndex||!SameKinematicBits(observed->Intrinsic.OwnerState,Pending->SweepOwnerState))
        Refuse("Sleeping support observation does not join actual body slot");
    auto& slot=Pending->Bodies[CurrentIndex];
    if(slot.SleepingSupportBypass||slot.Ground||slot.OrdinaryGround)Refuse("Sleeping support no-query branch duplicated or mixed with query");
    slot.SleepingSupportBypass=MoveTemp(observed);
}
