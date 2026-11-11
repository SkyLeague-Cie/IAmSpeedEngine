#include "BoundedApproxFixedWheelProbe.h"
// Included in the existing IAS TU after native query value helpers.
// Pure routing hint only; the complete producer and all live guards remain.
bool HistoricalCacheBeforeCoverageEnabled();
bool PreferHistoricalCacheBeforeCoverage(const FPreparedWorldSnapshotPtr&,const Analytic::FWorldQuery&,
    double,double,const TSharedPtr<const void,ESPMode::ThreadSafe>&);
void RecordHistoricalCompletedWheelFallback(const FPreparedWorldSnapshotPtr&,const Analytic::FWorldQuery&,
    double,double,const TSharedPtr<const void,ESPMode::ThreadSafe>&,const Analytic::FWorldHit&);
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
bool BoundedLegacyWheelMemoEnabled();
bool PreferBoundedLegacyWheelBeforeCoverage(const FPreparedWorldSnapshotPtr&,const Analytic::FWorldQuery&,
    double,double,const TSharedPtr<const void,ESPMode::ThreadSafe>&,
    const TSharedPtr<const FPreparedCanonicalStaticQueryService,ESPMode::ThreadSafe>&);
void RecordBoundedLegacyWheelCompleted(const FPreparedWorldSnapshotPtr&,const Analytic::FWorldQuery&,
    double,double,const TSharedPtr<const void,ESPMode::ThreadSafe>&,
    const TSharedPtr<const FPreparedCanonicalStaticQueryService,ESPMode::ThreadSafe>&,const FBoundedApproxFixedWheelProbeValuesPtr&);
#endif
namespace PreparedProbeImmutableRowsDetail
{
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
TAutoConsoleVariable<int32> CVarEnabled(TEXT("p.IAmSpeed.AnalyticWorld.PreparedProbeImmutableRows"),0,
    TEXT("Private experiment: retain immutable owner/body row addresses inside existing probe scopes; all live checks remain."));
bool Enabled(){return CVarEnabled.GetValueOnAnyThread()!=0;}
#else
bool Enabled(){return false;}
#endif
// Only a lookup cache, never a successful Check or an execution permission.
// The enclosing owner State strongly owns Snapshot, which owns the const
// NativeOwners and their private, fully built Rows/Bodies arrays. No row can
// move during this scope. A wheel cannot outlive its owning phase scope.
struct FOwnerRow
{
    bool Enabled=false;
    mutable const FPreparedWorldSnapshot* Snapshot=nullptr;
    mutable const FPreparedNativeOwnerBindings* Source=nullptr;
    mutable const ISpeedComponent* Owner=nullptr;
    mutable const FPreparedNativeOwnerBinding* Row=nullptr;
    template<class FFind>
    const FPreparedNativeOwnerBinding* Resolve(const FPreparedWorldSnapshot* snapshot,
        const FPreparedNativeOwnerBindings* source,const ISpeedComponent* owner,FFind&& find)const
    {
        if(!Enabled||!snapshot||!source||!owner)return find();
        if(Row&&Snapshot==snapshot&&Source==source&&Owner==owner)return Row;
        const auto* result=find();Snapshot=snapshot;Source=source;Owner=owner;Row=result;return result;
    }
};
struct FChildRow
{
    bool Enabled=false;
    mutable const FPreparedNativeOwnerBinding* Owner=nullptr;
    mutable const FPreparedNativeBodyBinding* Row=nullptr;
    mutable int32 Index=INDEX_NONE;
    const FPreparedNativeBodyBinding* Resolve(const FPreparedNativeOwnerBinding* owner,int32 index)const
    {
        if(Enabled&&Row&&Owner==owner&&Index==index)return Row;
        const auto* result=owner&&owner->Bodies.IsValidIndex(index)?&owner->Bodies[index]:nullptr;
        if(Enabled){Owner=owner;Index=index;Row=result;}return result;
    }
};
}
struct FOrdinaryOwnerProbePhaseScope::FState
{
    OnePausedWork::FCanonicalFrameWindowPtr Window;FPreparedWorldSnapshotPtr Snapshot;
    PreparedProbeImmutableRowsDetail::FOwnerRow ImmutableRow;
    const FHostCallScope* Host=nullptr;const ISpeedComponent* Owner=nullptr;
    ENativeOwnerProbePhase Phase=ENativeOwnerProbePhase::Prepare;FStepIdentity Step;
    uint64 Ordinal=0,QuerySerial=0,NumericControl=0;float Delta=0;bool Returned=false;const char* LastCheckFailure="not_checked";
};
bool FOrdinaryOwnerProbePhaseScope::Check() const
{
        ::StadiumProbeProfile70::FScope queryProfile70(::StadiumProbeProfile70::OwnerCheck);
    const auto fail=[&](const char* why){if(State)State->LastCheckFailure=why;return false;};
    if(!State||Poisoned||OwnerProbePhase!=this||!HostScope||HostScope!=State->Host||!HostScope->Active||
        HostScope->OwnerProbePhaseOrdinal!=State->Ordinal||HostScope->ThreadId!=FPlatformTLS::GetCurrentThreadId()||
        !IsCurrentWorker(HostScope->Worker)||!State->Window||!State->Window->IsOpen())return fail("owner_check_1");
    OnePausedWork::FCanonicalFrameWindowPtr window;
    if(!OnePausedWork::ReadExecutionWindow(window)||window!=State->Window||window->GetPurpose()!=OnePausedWork::EPurpose::FullFrame||
        !window->Matches(HostScope->Host,HostScope->Bridge,HostScope->Bridge->GetStaticCollisionWorld(),HostScope->Worker,
            HostScope->Generation,HostScope->Frame,HostScope->Delta,HostScope->SimTime)||!window->GetNonce()||
        window->GetSnapshot()!=State->Snapshot||!State->Snapshot||!State->Snapshot->GetNativeOwners()||
        HostScope->Host->GetInputWorkerGeneration()!=HostScope->Generation||HostScope->Host->GetWorld()!=HostScope->World||
        HostScope->Bridge->GetWorld()!=HostScope->World||HostScope->World!=window->GetWorldIdentity()||HostScope->World->bIsTearingDown)return fail("owner_check_2");
    const auto* provider=HostScope->Bridge->GetStaticCollisionWorld();const auto& origin=State->Snapshot->GetOrigin();
    if(!provider||provider->GetPreparedSnapshot()!=State->Snapshot||origin.Host!=HostScope->Host||origin.Bridge!=HostScope->Bridge||
        origin.World!=HostScope->World||origin.Generation!=HostScope->Generation||
        State->Snapshot->GetNativeOwners()->GetGeneration()!=HostScope->Generation||
        !State->Snapshot->GetGeometrySelection()||!State->Snapshot->GetGeometrySelection()->HasValidAliases())return fail("owner_check_3");
    const auto fp=Analytic::InspectCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(fp.Status!=Analytic::EAnalyticNumericStatus::Complete||fp.ControlFingerprint()!=State->NumericControl)return fail("owner_check_4");
    uint64 step=window->GetStepBefore();
    if(State->Phase==ENativeOwnerProbePhase::Prepare){if(StepScope)return fail("owner_check_5");}
    else
    {
        FStepIdentity now;if(!ReadCurrentStep(now)||!StepScope||StepScope->SweepPhase||
            now.StepSerial!=window->GetExpectedStep()||now.Lease!=State->Step.Lease||now.QueryEpoch!=State->Step.QueryEpoch||
            now.QuerySerial!=State->Step.QuerySerial||now.StepSerial!=State->Step.StepSerial||now.Provider!=provider||
            now.WindowSerial!=window->GetSerial()||now.WindowNonce!=window->GetNonce())return fail("owner_check_6");
        step=now.StepSerial;
    }
    // Shape of census only. This deliberately does not qualify Wheeled/Wheel
    // virtual implementations or replace the still-refusing FullFrame gate.
    if(!FCanonicalFramePreflight::HasUnchangedRawPreparationCensus(*HostScope->Bridge,*State->Snapshot,step))return fail("owner_check_7");
    const auto* row=State->ImmutableRow.Resolve(State->Snapshot.Get(),State->Snapshot->GetNativeOwners().Get(),State->Owner,
        [&]{return State->Snapshot->GetNativeOwners()->Find(State->Owner);});
    if(!row||!row->Adapter||row->Adapter!=State->Owner||!row->NativeObject||!row->Class||!row->BodyCensusCaptured)return fail("owner_check_8");
    if(row->NativeObject->GetClass()!=row->Class)return fail("owner_class");State->LastCheckFailure=nullptr;return true;
}
FOrdinaryOwnerProbePhaseScope::FOrdinaryOwnerProbePhaseScope(ENativeOwnerProbePhase phase,const ISpeedComponent& owner,float delta)
{
        PreparedExecution::FNativePlanarResponseCommitScope::RequireQueriesClosed();
    if(!Requested())return;OnePausedWork::FCanonicalFrameWindowPtr window;
    if(!OnePausedWork::ReadExecutionWindow(window)||!window)Refuse("Native owner probe phase lacks its execution window");
    if(window->GetPurpose()!=OnePausedWork::EPurpose::FullFrame)return; // historical terminal programs unchanged
    if(Poisoned||OwnerProbePhase||WheelProbeScope||!HostScope||!HostScope->Active||!IsSupportedSphereResponseMargin(delta)||
        HostScope->OwnerProbePhaseOrdinal==MAX_uint64)Refuse("Native owner probe phase identity unavailable");
    auto state=MakeUnique<FState>();state->Window=window;state->Snapshot=window->GetSnapshot();state->Host=HostScope;
    state->Owner=&owner;state->Phase=phase;state->Delta=delta;state->Ordinal=++HostScope->OwnerProbePhaseOrdinal;
    state->ImmutableRow.Enabled=PreparedProbeImmutableRowsDetail::Enabled();
    state->NumericControl=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint();
    if(phase!=ENativeOwnerProbePhase::Prepare&&!ReadCurrentStep(state->Step))Refuse("Post-prepare probe phase has no actual Step");
    State=MoveTemp(state);OwnerProbePhase=this;if(!Check()){OwnerProbePhase=nullptr;Refuse("Native owner probe phase join refused");}
}
void FOrdinaryOwnerProbePhaseScope::Returned()
{
    if(!State)return;if(State->Returned||WheelProbeScope||!Check())Refuse("Native owner probe phase return changed");State->Returned=true;
}
FOrdinaryOwnerProbePhaseScope::~FOrdinaryOwnerProbePhaseScope() noexcept
{
    if(!State)return;
    const auto poison=[&](const char* why,const char* detail=nullptr)
    {RecordPreparedPoison(why,detail,State->Ordinal,State->QuerySerial,INDEX_NONE,uint8(State->Phase));};
    try
    {
        if(!State->Returned)poison("owner_probe_dtor_not_returned");
        else if(WheelProbeScope)poison("owner_probe_dtor_open_wheel");
        else if(!Check())poison("owner_probe_dtor_check",State->LastCheckFailure);
    }
    catch(const std::exception& error){poison("owner_probe_dtor_exception",error.what());}
    catch(...){poison("owner_probe_dtor_exception","non_std_exception");}
    if(OwnerProbePhase==this)OwnerProbePhase=nullptr;else poison("owner_probe_dtor_scope_identity");
}
namespace NativeSphereProjectionDetail
{
TAutoConsoleVariable<int32> Enabled(TEXT("p.IAmSpeed.AnalyticWorld.NativeExactSphereProjection"),0,
    TEXT("Default OFF. Certify the actual stationary sphere projection after canonical coverage refusal."));
}
FNativeSphereProjectionScope::FNativeSphereProjectionScope(const USphereSubBody& body,const ISpeedComponent& owner,
    const FVector& center,float radius,int32 pass)
{
    if(NativeSphereProjectionDetail::Enabled.GetValueOnAnyThread()!=1||!Requested()||!StepScope||!OwnerProbePhase)return;
    Body=&body;Owner=&owner;Step=StepScope;Phase=OwnerProbePhase;Snapshot=Step->Snapshot.Get();
    Center=center;Radius=radius;Pass=pass;
}
bool FNativeSphereProjectionScope::Matches(const Analytic::FWorldQuery& query,const FPreparedCanonicalStaticQueryService& service) const
{
    FNativePlanarResponseCommitScope::RequireQueriesClosed();
    if(NativeSphereProjectionDetail::Enabled.GetValueOnAnyThread()!=1||!IsActive()||StepScope!=Step||
        OwnerProbePhase!=Phase||!Phase->State||Phase->State->Returned||!Phase->Check()||
        Phase->State->Owner!=Owner||Phase->State->Snapshot.Get()!=Snapshot||Step->Snapshot.Get()!=Snapshot||
        !Snapshot||service.GetPreparedSnapshot().Get()!=Snapshot||!Step->Identity.Provider||
        Step->Identity.Provider->GetPreparedQueryService().Get()!=&service||!Body||!Owner||
        Pass<0||!FMath::IsFinite(Radius)||Radius<=0||query.Shape!=Analytic::EQueryShape::Sphere||
        !query.bObjectQuery||!query.bApplyCollisionFilter||!query.bAuthorityOnly||
        query.ObjectTypes!=(1ull<<uint8(ECC_WorldStatic))||query.Rotation!=FQuat::Identity||
        !SameVector(query.Start,Center)||!SameVector(query.End,Center)||
        AnalyticCertified::Bits(query.Radius)!=AnalyticCertified::Bits(double(Radius)))return false;
    Step->RequireNativeFrameContext();
    const auto* row=Snapshot->GetNativeOwners()->Find(Owner);
    if(!row||!row->BodyCensusCaptured||row->Adapter!=Owner||!Step->NativeConsumer->ContainsBody(Body)||
        Step->NativeConsumer->GetBodyRoute(Body)!=ENativeFrameBodyRoute::Sphere)return false;
    const auto* child=row->Bodies.FindByPredicate([&](const auto& candidate){return candidate.Body==Body;});
    if(!child||!child->ObjectIdCaptured||child->Class!=Body->GetClass()||
        Body->GetParentComponent()!=Owner||!Body->IsMainSubBody()||Body->GetWorld()!=Snapshot->GetOrigin().World||
        !Owner->GetSubBodies().IsValidIndex(child->Index)||Owner->GetSubBodies()[child->Index]!=Body||
        Owner->GetStaticCollisionWorldForFrame()!=Step->Identity.Provider||
        (Pass==0&&!SameVector(Owner->GetPhysLocation(),Center)))return false;
    switch(Phase->State->Phase)
    {
        case ENativeOwnerProbePhase::ResponseResolve:
        {
            Step->RequireOrdinaryResponseContext();
            const auto& tx=*Step->OrdinaryResponse;
            return tx.ActiveCall==2&&tx.Entries.IsValidIndex(tx.ActiveIndex)&&
                tx.Entries[tx.ActiveIndex].RawResolver==Body&&tx.Entries[tx.ActiveIndex].Allowed&&
                !tx.Entries[tx.ActiveIndex].Closed;
        }
        case ENativeOwnerProbePhase::PostIntegrate:
        case ENativeOwnerProbePhase::IntegrationProjection:
            Step->RequireOrdinaryIntegrationContext(true);
            return true;
        case ENativeOwnerProbePhase::FinalProjection:
        case ENativeOwnerProbePhase::FinalPost:
        {
            const auto* final=Step->OrdinaryFinal.Get();
            FStepIdentity now;
            if(!final||final->Closed||!final->CallOpen||final->Operations.IsEmpty()||
                !ReadCurrentStep(now)||!OrdinaryWorldDetail::Epoch(final->Entry,now)||
                final->Entry.QuerySerial!=now.QuerySerial||
                (Step->OrdinaryIntegration&&!Step->OrdinaryIntegration->Closed)||
                (Step->OrdinaryResponse&&!Step->OrdinaryResponse->Closed))return false;
            const auto& call=final->Operations.Last();
            const auto kind=Phase->State->Phase==ENativeOwnerProbePhase::FinalProjection?
                ENativeFrameFinalOperation::StaticProjectionOwner:ENativeFrameFinalOperation::PostOwner;
            return call.Owner==Owner&&call.Kind==kind&&call.Called&&!call.Returned;
        }
        default:return false;
    }
}
static TAutoConsoleVariable<int32> CVarCurvedBoxProjectionTrace(
    TEXT("p.IAmSpeed.AnalyticWorld.CurvedBoxProjectionTrace"),0,TEXT("Trace admission of the physical curved Box projection. Default OFF."));

bool FStepScope::TryProveNativeCurvedBoxPhysicalProjectionClear(const UBoxSubBody& box,
    const SHitResult& seed,const IStaticCollisionWorld& world,const Analytic::FWorldQuery& query)
{
    const auto fail=[](const TCHAR* reason)
    {
        if(CVarCurvedBoxProjectionTrace.GetValueOnAnyThread()==1)
            UE_LOG(LogTemp,Display,TEXT("[CurvedBoxPhysicalProjectionGate] Reason=%s"),reason);
        return false;
    };
    const auto* step=StepScope;
    const auto* phase=OwnerProbePhase;
    if(!step||!phase||!phase->State||phase->State->Phase!=ENativeOwnerProbePhase::IntegrationProjection||
        phase->State->Returned||!seed.bSurfaceNormalMayVary||!seed.SourceId||!seed.SurfaceId||
        !seed.CanonicalGroupId||query.RequiredSourceId!=seed.SourceId||
        query.RequiredSurfaceId!=seed.SurfaceId||query.RequiredCanonicalGroupId!=seed.CanonicalGroupId||
        step->Identity.Provider!=&world)return fail(TEXT("scope_or_seed"));
    step->RequireOrdinaryIntegrationContext(true);
    const auto& tx=*step->OrdinaryIntegration;
    const auto& row=tx.Owners[tx.ActiveIndex];
    if(!row.Owner||box.GetParentComponent()!=row.Owner||phase->State->Owner!=row.Owner||
        !row.GeneralOpen||!row.GeneralReturned||row.OpenPoseSetter||
        query.Shape!=Analytic::EQueryShape::Box||
        query.TraceChannel!=uint8(static_cast<const USSubBody&>(box).GetCollisionChannel())||
        !SameVector(query.HalfExtent,box.GetBoxExtent())||!SameVector(query.Start,query.End))return fail(TEXT("owner_or_row"));
    const auto livePose=box.GetKinematicState();
    if(!SameVector(query.Start,livePose.Location)||!SameQuat(query.Rotation,livePose.Rotation))return fail(TEXT("live_pose"));

    // Join the actually advancing curved prediction to the same target as the
    // previous projection seed. The prediction neither supplies the old normal
    // nor proves the new stationary query. That proof uses the live pose below.
    const auto* binding=row.GeneralBindings.FindByPredicate([&](const auto& item)
    {return item.Request&&item.Request->GetValues().Body==&box;});
    if(!binding||!binding->HasExactlyOneKind()||binding->Request->GetSnapshot()!=step->Snapshot||
        binding->Request->GetValues().Owner!=row.Owner||!binding->Prediction||
        !binding->Prediction->IsCurvedArrivalPrefix())return fail(TEXT("binding"));
    Analytic::FWorldHit arrival;float arrivalSeconds=0;
    if(!ReadNativeCurvedBoxArrivalValues(step->Snapshot,binding->Request->GetValues(),
            binding->Prediction,arrivalSeconds,arrival)||!arrival.bHit||
        arrival.SourceId!=query.RequiredSourceId||arrival.SurfaceId!=query.RequiredSurfaceId||
        arrival.CanonicalGroupId!=query.RequiredCanonicalGroupId||
        !(FVector::DotProduct(arrival.Normal,query.ReferenceNormal)>query.MinimumReferenceNormalDot))
    {
        if(CVarCurvedBoxProjectionTrace.GetValueOnAnyThread()==1)
            UE_LOG(LogTemp,Display,TEXT("[CurvedBoxPhysicalProjectionArrival] Hit=%d Src=%llu Surf=%llu Group=%llu Dot=%.17g Threshold=%.17g"),
                int32(arrival.bHit),arrival.SourceId,arrival.SurfaceId,arrival.CanonicalGroupId,
                FVector::DotProduct(arrival.Normal,query.ReferenceNormal),query.MinimumReferenceNormalDot);
        return fail(TEXT("arrival"));
    }

    const auto service=world.GetPreparedQueryService();
    const bool exactSeed=arrival.FeatureId==seed.FeatureId&&arrival.PrimitiveId==seed.PrimitiveId;
    const FNativeCurvedBoxPhysicalProjectionContext context(query,
        exactSeed?binding->Prediction->GetCurvedArrivalForInspection().Get():nullptr);
    if(!service||!service->MatchesPreparedSnapshot(step->Snapshot)||
        !service->EvaluateNativeCurvedBoxPhysicalProjectionClear(query,context))return fail(TEXT("service"));
    step->RequireOrdinaryIntegrationContext(true);
    const auto after=box.GetKinematicState();
    if(!SameVector(query.Start,after.Location)||!SameQuat(query.Rotation,after.Rotation))return fail(TEXT("post_pose"));
    if(CVarCurvedBoxProjectionTrace.GetValueOnAnyThread()==1)
        UE_LOG(LogTemp,Display,TEXT("[CurvedBoxPhysicalProjectionGate] Reason=clear"));
    return true;
}
struct FNativeWheelProbeScope::FState
{
    FOrdinaryOwnerProbePhaseScope* Phase=nullptr;const USWheelSubBody* Wheel=nullptr;
    PreparedProbeImmutableRowsDetail::FChildRow ImmutableRow;
    FInheritedMotionPolicyValues91 InheritedMotionPolicy91;
    int32 Index=INDEX_NONE;const void* Class=nullptr;float Delta=0;int32 Uncaught=0;bool QueryOpen=false,Used=false;
    const char* LastCheckFailure="not_checked";uint64 CapturedOrdinal=0,CapturedSerial=0;uint8 CapturedPhase=0xff;
};
bool FNativeWheelProbeScope::Check() const
{
        ::StadiumProbeProfile70::FScope queryProfile70(::StadiumProbeProfile70::WheelCheck);
    const auto fail=[&](const char* why){if(State)State->LastCheckFailure=why;return false;};
    if(!State||WheelProbeScope!=this||!State->Phase)return fail("wheel_check_1");
    if(!State->Phase->Check())return fail(State->Phase->State?State->Phase->State->LastCheckFailure:"wheel_owner_state_absent");
    if(State->Phase->State->Returned)return fail("wheel_owner_returned");
    const auto& parent=*State->Phase->State;
    const auto* row=parent.ImmutableRow.Resolve(parent.Snapshot.Get(),parent.Snapshot->GetNativeOwners().Get(),parent.Owner,
        [&]{return parent.Snapshot->GetNativeOwners()->Find(parent.Owner);});
    const auto* childRow=State->ImmutableRow.Resolve(row,State->Index);
    if(!row||!childRow)return fail("wheel_check_2");const auto& child=*childRow;
    if(child.Index!=State->Index||child.Body!=static_cast<const USSubBody*>(State->Wheel)||child.Class!=State->Class||
        !child.ObjectIdCaptured||!child.Body||!SameInheritedMotionPolicy91(child.InheritedMotionPolicy91,State->InheritedMotionPolicy91))return fail("wheel_check_3");
    // Immutable GT association precedes wheel/owner virtual or UObject reads.
    if(child.Body->GetClass()!=State->Class||child.Body->GetParentComponent()!=parent.Owner||child.Body->GetWorld()!=HostScope->World||
        parent.Owner->GetSubBodies().Num()!=row->Bodies.Num()||parent.Owner->GetSubBodies()[State->Index]!=child.Body||
        parent.Owner->GetStaticCollisionWorldForFrame()!=HostScope->Bridge->GetStaticCollisionWorld())return fail("wheel_check_4");
    FCanonicalPreparedFrameClock clock;if(!JoinCanonicalPreparedFrameClock(HostScope->Frame,parent.Owner->NumFrame(),clock))return fail("wheel_clock");State->LastCheckFailure=nullptr;return true;
}
FNativeWheelProbeScope::FNativeWheelProbeScope(const USWheelSubBody& wheel,float delta)
{
    FNativePlanarResponseCommitScope::RequireQueriesClosed();
    if(!Requested())return;OnePausedWork::FCanonicalFrameWindowPtr window;
    if(!OnePausedWork::ReadExecutionWindow(window)||!window)Refuse("Wheel probe lacks its actual execution window");
    // The historical terminal path keeps its own previously closed contracts.
    // This new adapter provides no query/phase permission to those programs.
    if(window->GetPurpose()!=OnePausedWork::EPurpose::FullFrame)return;
    if(Poisoned)Refuse("Wheel probe FullFrame context is already poisoned");
    if(WheelProbeScope)Refuse("Wheel probe FullFrame scope is nested");
    if(!OwnerProbePhase)Refuse("Wheel probe lacks its real FullFrame owner phase");
    if(!OwnerProbePhase->Check())Refuse("Wheel probe real FullFrame owner phase join changed");
    if(!IsSupportedSphereResponseMargin(delta))Refuse("Wheel probe actual FullFrame delta unsupported");
    const auto& parent=*OwnerProbePhase->State;
    const auto* row=parent.ImmutableRow.Resolve(parent.Snapshot.Get(),parent.Snapshot->GetNativeOwners().Get(),parent.Owner,
        [&]{return parent.Snapshot->GetNativeOwners()->Find(parent.Owner);});
    if(!row)Refuse("Wheel probe owner absent from retained GT census");
    auto state=MakeUnique<FState>();state->Phase=OwnerProbePhase;state->Wheel=&wheel;state->Delta=delta;state->Uncaught=std::uncaught_exceptions();
    state->CapturedOrdinal=parent.Ordinal;state->CapturedSerial=parent.QuerySerial;state->CapturedPhase=uint8(parent.Phase);
    state->ImmutableRow.Enabled=parent.ImmutableRow.Enabled;
    for(const auto& child:row->Bodies)if(child.Body==static_cast<const USSubBody*>(&wheel))
    {if(state->Index!=INDEX_NONE)Refuse("Wheel probe GT association ambiguous");state->Index=child.Index;state->Class=child.Class;state->InheritedMotionPolicy91=child.InheritedMotionPolicy91;}
    State=MoveTemp(state);WheelProbeScope=this;if(!Check()){WheelProbeScope=nullptr;Refuse("Wheel probe GT association changed");}
}
FNativeWheelProbeScope::~FNativeWheelProbeScope() noexcept
{
    if(!State)return;
    const auto poison=[&](const char* why,const char* detail=nullptr)
    {RecordPreparedPoison(why,detail,State->CapturedOrdinal,State->CapturedSerial,State->Index,State->CapturedPhase,State->QueryOpen,State->Used,State->Uncaught);};
    try
    {
        if(State->QueryOpen)poison("wheel_dtor_query_open");
        else if(std::uncaught_exceptions()!=State->Uncaught)poison("wheel_dtor_unwinding");
        else if(!Check())poison("wheel_dtor_check",State->LastCheckFailure);
    }
    catch(const std::exception& error){poison("wheel_dtor_check_exception",error.what());}
    catch(...){poison("wheel_dtor_check_exception","non_std_exception");}
    if(WheelProbeScope==this)WheelProbeScope=nullptr;else poison("wheel_dtor_scope_identity");
}
bool FNativeWheelProbeScope::Sweep(const FVector& start,const FVector& end,const FQuat& rotation,const FCollisionShape& shape,
    const FCollisionQueryParams& params,Analytic::EStaticQuerySite site,SHitResult& out)
{
        ::StadiumProbeProfile70::FScope queryProfile70(::StadiumProbeProfile70::WholeSweep);
    FNativePlanarResponseCommitScope::RequireQueriesClosed();
    out={};if(!Check()||State->Used)Refuse("Wheel probe query scope invalid or reused");State->Used=true;
    auto& parent=*State->Phase->State;const auto* wheel=State->Wheel;const auto* owner=parent.Owner;
    const auto service=HostScope->Bridge->GetStaticCollisionWorld()->GetPreparedQueryService();
    if(!service||!service->MatchesPreparedSnapshot(parent.Snapshot))Refuse("Wheel probe canonical service differs");
    auto query=Analytic::FStaticWorldQueryAudit::MakeStrictSingleQuery1075(start,end,rotation,shape,
        uint8(wheel->GetCollisionChannel()),wheel->GetResponseParams());
    bool first=true;
    FCanonicalFixedSphereWheelProbeValuesPtr canonicalProbe;
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
    FBoundedApproxFixedWheelProbeValuesPtr approximateProbe;
    FInitCertifiedLegacyWheelProbeValuesPtr initialProbe;
#endif
    uint64 canonicalProbeSerial=0;
    double canonicalProbeRadius=0;
    const auto evaluate=[&](const Analytic::FWorldQuery& q)
    {
        ::StadiumProbeProfile70::FScope evaluateProfile70(::StadiumProbeProfile70::WholeEvaluate);
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
        const bool legacyShadow=service->HasPreparedLegacyQuinticShadow();
        const uint64 legacyExactBegan=legacyShadow?FPlatformTime::Cycles64():0;
#endif
        if(!Check()||State->QueryOpen||parent.QuerySerial==MAX_uint64)Refuse("Wheel probe phase/query serial unavailable");
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
        approximateProbe.Reset();initialProbe.Reset();
#endif
        canonicalProbe.Reset();canonicalProbeSerial=0;canonicalProbeRadius=0;
        const uint64 serial=++parent.QuerySerial;FNativeSolidTraceValues trace;
        const double actualProbeRadius=double(wheel->Radius());
        if(!CaptureNativeSolidTraceValues(*wheel,params,trace)||!parent.Snapshot->GetFilters()||
            !parent.Snapshot->GetFilters()->CoversPhysicalFilter(true,HostScope->Generation,owner,wheel,q))Refuse("Wheel probe actual trace/filter unsupported");
        const auto beforeOwner=owner->GetKinematicState(),beforeBody=wheel->GetKinematicState();
        const auto actual=Analytic::FStaticWorldQueryAudit::MakeStrictSingleQuery1075(start,end,beforeBody.Rotation,wheel->GetCollisionShape(),
            uint8(wheel->GetCollisionChannel()),wheel->GetResponseParams());
        auto base=q;base.RequiredSourceId=0;base.RequiredSurfaceId=0;base.RequiredCanonicalGroupId=0;
        base.ReferenceNormal=FVector::ZeroVector;base.MinimumReferenceNormalDot=-1.;base.bAllowEstablishedFaceContactAtBoundary=false;
        if(!FiniteState(beforeOwner)||!FiniteState(beforeBody)||!NativeSolidDetail::SameQueryBits(base,actual))Refuse("Wheel probe shape/rotation values changed");
        const uint8 diagnosticAttempt89=first?1:2;
        const uint32 attempt=Analytic::FStaticWorldQueryAudit::BeginPreparedProbeQuery(first);first=false;
        const double began=FPlatformTime::Seconds();Analytic::FWorldHit hit;bool complete=false;State->QueryOpen=true;
        ON_SCOPE_EXIT{Analytic::FStaticWorldQueryAudit::EndPreparedProbeQuery(site,q,hit,attempt,
            (FPlatformTime::Seconds()-began)*1000.,complete);State->QueryOpen=false;};
        bool needCoverageDomain=HistoricalCacheBeforeCoverageEnabled();
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
        needCoverageDomain|=BoundedLegacyWheelMemoEnabled();
#endif
        const auto coverageDomain=needCoverageDomain?service->GetHistoricalFixedCoverageDomain():TSharedPtr<const void,ESPMode::ThreadSafe>{};
        const bool preferHistorical=PreferHistoricalCacheBeforeCoverage(parent.Snapshot,q,double(State->Delta),actualProbeRadius,coverageDomain);
        bool preferBounded=false;
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
        if(!preferHistorical)preferBounded=PreferBoundedLegacyWheelBeforeCoverage(parent.Snapshot,q,double(State->Delta),actualProbeRadius,coverageDomain,service);
#endif
        // One account survives a paid optional frontend decline and the
        // original exact fallback. OFF initializes it at the old location.
        Analytic::Certified::FDistanceBudget queryBudget;
        bool queryBudgetPrepared=false;
        const auto prepareQueryBudget=[&]()
        {
            if(queryBudgetPrepared)return;
            const int64 work=2ll*parent.Snapshot->GetGeometrySelection()->GetSelectedUnion()->Num()+
                parent.Snapshot->GetAllContactBindings().Num()+parent.Snapshot->GetFilters()->GetRecords().Num()+64+128ll*8192;
            if(work<=0||work>MAX_int32)Refuse("Canonical fixed Sphere wheel work bound unavailable");
            queryBudget=Analytic::Certified::FDistanceBudget{int32(work)};queryBudgetPrepared=true;
        };
        bool coverage70=false,coverageTried=false;
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
        if(!preferHistorical&&!preferBounded&&BoundedLegacyWheelFrontendEnabled()&&service->PreferPreparedFixedProbeCoverage(q))
        {
            coverageTried=true;::StadiumProbeProfile70::FScope scope70(::StadiumProbeProfile70::Coverage);
            coverage70=service->TrySweepNativeFixedProbeWithCoverage(q,trace,hit);
        }
#endif
        bool frontendHit=false;
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
        bool frontendAttempted=false;
        if(!coverage70&&!preferHistorical&&!preferBounded&&BoundedLegacyWheelFrontendEnabled())
        {
            frontendAttempted=true;prepareQueryBudget();FString frontendReason;
            const auto status=EvaluateBoundedLegacyWheelFrontend(parent.Snapshot,q,trace,double(State->Delta),
                actualProbeRadius,serial,queryBudget,service,approximateProbe,frontendReason);
            if(status==EBoundedApproxWheelAttempt::WorkFailure||status==EBoundedApproxWheelAttempt::NumericFailure)
                Refuse("Bounded wheel frontend work/numeric failure");
            if(status==EBoundedApproxWheelAttempt::BoundedApproxSpatialHit)
            {
                if(!approximateProbe||!approximateProbe->Matches(parent.Snapshot,q,trace,double(State->Delta),actualProbeRadius,serial,service.Get()))
                    Refuse("Bounded wheel frontend differs from actual query");
                hit=approximateProbe->GetHit();canonicalProbeSerial=serial;canonicalProbeRadius=actualProbeRadius;frontendHit=true;
            }
        }
#endif
        if(frontendHit||preferHistorical||preferBounded)
        {FNativePlanarResponseCommitScope::RequireQueriesClosed();Analytic::RequireCurrentAnalyticNumericContext(service->GetNumericContract());}
        else if(!coverageTried)
        {::StadiumProbeProfile70::FScope scope70(::StadiumProbeProfile70::Coverage);coverage70=service->TrySweepNativeFixedProbeWithCoverage(q,trace,hit);}
        if(!coverage70&&!frontendHit)
        {
            prepareQueryBudget();auto& budget=queryBudget;FString reason;
            Analytic::Certified::FInheritedReachInput91 inheritedReach91;
            inheritedReach91.Available=true;inheritedReach91.Policy=State->InheritedMotionPolicy91;
            inheritedReach91.Owner=beforeOwner;inheritedReach91.Body=beforeBody;
            bool sphere70;
            {::StadiumProbeProfile70::FScope scope70(::StadiumProbeProfile70::SphereFallback);
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
                if(frontendAttempted)
                    sphere70=EvaluateCanonicalFixedSphereWheelProbeValues(parent.Snapshot,q,trace,double(State->Delta),
                        actualProbeRadius,serial,budget,canonicalProbe,reason,FFixedSphereProbeDiagnostic89{uint8(site),uint8(parent.Phase),diagnosticAttempt89},inheritedReach91);
                else sphere70=EvaluateFixedSphereWheelProbeRoute(parent.Snapshot,q,trace,double(State->Delta),
                    actualProbeRadius,serial,budget,canonicalProbe,reason,FFixedSphereProbeDiagnostic89{uint8(site),uint8(parent.Phase),diagnosticAttempt89},inheritedReach91,service,approximateProbe,&initialProbe);
#else
                sphere70=EvaluateCanonicalFixedSphereWheelProbeValues(parent.Snapshot,q,trace,double(State->Delta),
                    actualProbeRadius,serial,budget,canonicalProbe,reason,FFixedSphereProbeDiagnostic89{uint8(site),uint8(parent.Phase),diagnosticAttempt89},inheritedReach91);
#endif
            }
            if(sphere70)
            {
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
                if(initialProbe)
                {
                    if(canonicalProbe||approximateProbe||!initialProbe->Matches(parent.Snapshot,q,trace,double(State->Delta),actualProbeRadius,serial,service.Get()))
                        Refuse("Init-certified Legacy response differs from actual query");
                    hit=initialProbe->GetHit();
                }
                else if(approximateProbe)
                {
                    if(canonicalProbe||!approximateProbe->Matches(parent.Snapshot,q,trace,double(State->Delta),actualProbeRadius,serial,service.Get()))
                        Refuse("Bounded spatial wheel proposal differs from actual query");
                    hit=approximateProbe->GetHit();
                }
                else
#endif
                {
                    if(!canonicalProbe||!canonicalProbe->Matches(parent.Snapshot,q,trace,double(State->Delta),actualProbeRadius,serial))
                        Refuse("Canonical fixed Sphere wheel proof differs from actual query");
                    hit=canonicalProbe->GetHit();
                }
                canonicalProbeSerial=serial;canonicalProbeRadius=actualProbeRadius;
            }
            else
            {
                UE_LOG(LogTemp,Warning,TEXT("[CanonicalFixedSphereWheelProbeUnknown] Serial=%llu Reason=%s"),serial,*reason);
                // Preserve the existing exact-query hard refusal. No miss,
                // Unreal fallback or response is inferred from a failed proof.
                hit=service->SweepNativeFixedProbeValues(q,trace);
            }
        }
        {
        ::StadiumProbeProfile70::FScope suffix70(::StadiumProbeProfile70::Suffix);
        if(!Check()||parent.QuerySerial!=serial)Refuse("Wheel probe phase revoked before any result getter");
        FNativeSolidTraceValues afterTrace;
        const auto afterQuery=Analytic::FStaticWorldQueryAudit::MakeStrictSingleQuery1075(start,end,wheel->GetKinematicState().Rotation,
            wheel->GetCollisionShape(),uint8(wheel->GetCollisionChannel()),wheel->GetResponseParams());
        if(!Check()||parent.QuerySerial!=serial||!CaptureNativeSolidTraceValues(*wheel,params,afterTrace)||!trace.Same(afterTrace)||
            !NativeSolidDetail::SameQueryBits(actual,afterQuery)||!SameKinematicBits(beforeOwner,owner->GetKinematicState())||
            !SameKinematicBits(beforeBody,wheel->GetKinematicState())||
            AnalyticCertified::Bits(actualProbeRadius)!=AnalyticCertified::Bits(double(wheel->Radius()))||
            (canonicalProbe&&!canonicalProbe->Matches(parent.Snapshot,q,afterTrace,double(State->Delta),double(wheel->Radius()),serial)))
            Refuse("Wheel probe phase/query/body state changed during fixed query");
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
        if(initialProbe&&!initialProbe->Matches(parent.Snapshot,q,afterTrace,double(State->Delta),double(wheel->Radius()),serial,service.Get()))
            Refuse("Init-certified Legacy response changed during fresh suffix");
        if(approximateProbe&&!approximateProbe->Matches(parent.Snapshot,q,afterTrace,double(State->Delta),double(wheel->Radius()),serial,service.Get()))
            Refuse("Bounded spatial wheel proposal changed during fresh suffix");
#endif
        complete=true;
        }
        // Publish route provenance only AFTER the actual producer, Matches,
        // live before/after values, trace and numeric/scope checks completed.
        // A direct producer call cannot manufacture this successful route.
        if(frontendHit||preferHistorical||preferBounded)Analytic::RequireCurrentAnalyticNumericContext(service->GetNumericContract());
        if(!preferHistorical&&!preferBounded&&!coverage70&&canonicalProbe&&coverageDomain&&
            canonicalProbe->GetResult()==ECanonicalFixedSphereWheelProbeResult::CanonicalContact&&
            coverageDomain==service->GetHistoricalFixedCoverageDomain())
            RecordHistoricalCompletedWheelFallback(parent.Snapshot,q,double(State->Delta),actualProbeRadius,coverageDomain,hit);
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
        if(!coverage70&&approximateProbe&&coverageDomain&&coverageDomain==service->GetHistoricalFixedCoverageDomain())
            RecordBoundedLegacyWheelCompleted(parent.Snapshot,q,double(State->Delta),actualProbeRadius,coverageDomain,service,approximateProbe);
        if(!coverage70&&initialProbe&&coverageDomain&&coverageDomain==service->GetHistoricalFixedCoverageDomain())
            RecordInitCertifiedLegacyWheelCompleted(parent.Snapshot,q,double(State->Delta),actualProbeRadius,coverageDomain,service,initialProbe);
        if(legacyShadow&&!approximateProbe&&!initialProbe)service->ObserveLegacyQuinticShadow(q,hit,FPlatformTime::Cycles64()-legacyExactBegan,coverage70);
#endif
        return hit;
    };
    auto hit=evaluate(query);
    if(!hit.bHit&&wheel->IsOnGround())
    {
        const SHitResult& previous=wheel->GetHit();const FVector normal=previous.ImpactNormal.GetSafeNormal();
        const float inward=FVector::DotProduct(owner->GetPhysCOMVelocity(),normal);
        if(!normal.IsNearlyZero()&&inward<0.f&&previous.SourceId!=0)
        {
            query.RequiredSourceId=previous.SourceId;query.ReferenceNormal=normal;
            query.MinimumReferenceNormalDot=double(0.9f);query.bAllowEstablishedFaceContactAtBoundary=true;
            hit=evaluate(query);
        }
    }
    if(!Check())Refuse("Wheel probe result lost its phase binding");if(!hit.bHit)return false;
    ::StadiumProbeProfile70::FScope conversion70(::StadiumProbeProfile70::Conversion);
    bool bindingAmbiguous=false;
    const FPreparedSourceContactBinding* binding=parent.Snapshot->FindUniqueContactBinding(hit.SourceId,bindingAmbiguous);
    if(bindingAmbiguous)Refuse("Wheel probe Contact SourceId ambiguous");
    SHitResult value;
    // Preserve the real probe conversion: float(normalized Time * Delta),
    // old FrameTag=0 and old low-31-bit FaceIndex. Neither is a CCD event ID.
    const float seconds=static_cast<float>(hit.Time*State->Delta);
    if(!binding||!BuildNativeAnalyticContactValues(hit,seconds,0,*binding,value))Refuse("Wheel probe typed GT Contact conversion unsupported");
    value.FaceIndex=static_cast<int32>(hit.PrimitiveId&0x7fffffffull);
    value.ImpactNormal=QuantizeUnitNormal(value.ImpactNormal);
    if(canonicalProbe&&(canonicalProbe->GetResult()!=ECanonicalFixedSphereWheelProbeResult::CanonicalContact||
        parent.QuerySerial!=canonicalProbeSerial||
        AnalyticCertified::Bits(canonicalProbeRadius)!=AnalyticCertified::Bits(double(wheel->Radius()))||
        !canonicalProbe->MatchesConsumedContact(value)))Refuse("Canonical fixed Sphere wheel consumed values differ");
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
    if(initialProbe&&(canonicalProbe||approximateProbe||parent.QuerySerial!=canonicalProbeSerial||
        AnalyticCertified::Bits(canonicalProbeRadius)!=AnalyticCertified::Bits(double(wheel->Radius()))||
        !initialProbe->MatchesConsumedContact(value)))Refuse("Init-certified Legacy consumed values differ");
    if(approximateProbe&&(canonicalProbe||parent.QuerySerial!=canonicalProbeSerial||
        AnalyticCertified::Bits(canonicalProbeRadius)!=AnalyticCertified::Bits(double(wheel->Radius()))||
        !approximateProbe->MatchesConsumedContact(value)))Refuse("Bounded spatial wheel consumed values differ");
#endif
    if(!Check())Refuse("Wheel probe changed before native return");out=MoveTemp(value);return true;
}

#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPreparedProbeImmutableRowsTest,
    "IAmSpeed.AnalyticWorld.Prepared.ProbeImmutableRows",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FPreparedProbeImmutableRowsTest::RunTest(const FString&)
{
    // Like RawCensusValueMemo, these are pure metadata identity sentinels.
    // No fake UObject/snapshot is dereferenced and no execution scope is made.
    auto* ownerA=reinterpret_cast<ISpeedComponent*>(UPTRINT(0x1000));
    auto* ownerB=reinterpret_cast<ISpeedComponent*>(UPTRINT(0x2000));
    auto* bodyA=reinterpret_cast<const USSubBody*>(UPTRINT(0x3000));
    auto* bodyB=reinterpret_cast<const USSubBody*>(UPTRINT(0x4000));
    const auto* snapshotA=reinterpret_cast<const FPreparedWorldSnapshot*>(UPTRINT(0x5000));
    const auto* snapshotB=reinterpret_cast<const FPreparedWorldSnapshot*>(UPTRINT(0x6000));
    const auto* sourceA=reinterpret_cast<const FPreparedNativeOwnerBindings*>(UPTRINT(0x7000));
    const auto* sourceB=reinterpret_cast<const FPreparedNativeOwnerBindings*>(UPTRINT(0x8000));
    TArray<FPreparedNativeOwnerBinding> rows;
    for(auto* owner:{ownerA,ownerB})
    {
        FPreparedNativeOwnerBinding row;row.Adapter=owner;
        row.NativeObject=reinterpret_cast<const UActorComponent*>(UPTRINT(owner));
        row.Class=reinterpret_cast<const void*>(UPTRINT(0x9000));row.BodyCensusCaptured=true;
        auto& child=row.Bodies.AddDefaulted_GetRef();child.Index=0;child.Body=owner==ownerA?bodyA:bodyB;
        child.Class=row.Class;child.ObjectId=owner==ownerA?1u:2u;child.ObjectIdCaptured=true;rows.Add(MoveTemp(row));
    }
    // Arrays are complete before any binding and remain unchanged thereafter.
    for(int32 mode:{0,1})
    {
        FScopedCVar<int32> setting(TEXT("p.IAmSpeed.AnalyticWorld.PreparedProbeImmutableRows"),mode,
            FName(TEXT("PreparedProbeImmutableRowsTestMode")));
        PreparedProbeImmutableRowsDetail::FOwnerRow ownerBinding;
        ownerBinding.Enabled=PreparedProbeImmutableRowsDetail::Enabled();
        PreparedProbeImmutableRowsDetail::FChildRow childBinding;childBinding.Enabled=ownerBinding.Enabled;
        int32 searches=0;
        auto resolve=[&](const FPreparedWorldSnapshot* snapshot,const FPreparedNativeOwnerBindings* source,
            const ISpeedComponent* expected)
        {
            return ownerBinding.Resolve(snapshot,source,expected,[&]() -> const FPreparedNativeOwnerBinding*
                {++searches;return rows.FindByPredicate([&](const auto& row){return row.Adapter==expected;});});
        };
        for(int32 repeat=0;repeat<12;++repeat)
        {
            const auto* row=resolve(snapshotA,sourceA,ownerA);
            if(!TestTrue(TEXT("same real immutable metadata row and body"),row==&rows[0]&&
                childBinding.Resolve(row,0)==&rows[0].Bodies[0]))return false;
        }
        if(!TestEqual(TEXT("only immutable lookup work is removed"),searches,mode?1:12))return false;
        int32 before=searches;resolve(snapshotB,sourceA,ownerA);
        if(!TestEqual(TEXT("foreign snapshot performs original lookup"),searches,before+1))return false;
        before=searches;resolve(snapshotB,sourceB,ownerA);
        if(!TestEqual(TEXT("foreign collection performs original lookup"),searches,before+1))return false;
        const auto* other=resolve(snapshotB,sourceB,ownerB);
        if(!TestTrue(TEXT("foreign owner never reuses previous body"),other==&rows[1]&&
            childBinding.Resolve(other,0)==&rows[1].Bodies[0]&&
            childBinding.Resolve(other,1)==nullptr&&childBinding.Resolve(nullptr,0)==nullptr))return false;
        before=searches;
        if(!TestNull(TEXT("missing owner remains missing"),resolve(snapshotB,sourceB,nullptr))||
            !TestEqual(TEXT("missing key does not reuse"),searches,before+1))return false;
        TArray<ISpeedComponent*> adapters{ownerA,ownerB},ordered{ownerB,ownerA};
        const auto accepted=InspectCanonicalCensusForRefusal(rows,adapters,ordered,false,false);
        if(!TestTrue(TEXT("structural census reaches the existing phase-coverage boundary"),
            accepted==ECanonicalCensusRefusal::NoCompiledPhaseCoverage))return false;
        if(!TestTrue(TEXT("actual live census classifier still rejects pending and dirty registry"),
            accepted!=InspectCanonicalCensusForRefusal(rows,adapters,ordered,true,false)&&
            accepted!=InspectCanonicalCensusForRefusal(rows,adapters,ordered,false,true)))return false;
        ordered[0]=ownerA;
        if(!TestTrue(TEXT("actual live census classifier still rejects changed current order"),
            accepted!=InspectCanonicalCensusForRefusal(rows,adapters,ordered,false,false)))return false;
    }
    return true;
}
#endif
