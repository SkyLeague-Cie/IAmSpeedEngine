// Included after GeneralBoxTransportDetail, inside Speed::PreparedExecution.
namespace FinalGeneralBoxCaptureDetail
{
TAutoConsoleVariable<int32> Enabled(TEXT("p.IAmSpeed.AnalyticWorld.FinalGeneralBoxCapture"),0,
    TEXT("Default OFF. Fresh compact GeneralBox-only values at final-operation boundaries; preserves live census/configuration validation and all native callbacks."));

template<class TValues> const auto* Find(const TValues& values,const ISpeedComponent* owner)
{
    using FOwner=typename std::remove_reference_t<decltype(values.Owners)>::ElementType;
    for(const auto& row:values.Owners)if(row.Owner==owner)return &row;
    return static_cast<const FOwner*>(nullptr);
}
template<class TA,class TB> bool SameOwner(const TA& a,const TB& b)
{
    // Literal field contract of GeneralBoxTransportDetail::SamePoseAndBoxConfiguration.
    if(a.Owner!=b.Owner||a.StableId!=b.StableId||a.Frozen!=b.Frozen||a.Prescribed!=b.Prescribed||
        !GeneralBoxTransportDetail::Pose(a.State,b.State)||!SameVector(a.LocalCOM,b.LocalCOM)||
        !SameQuat(a.PhysRotation,b.PhysRotation)||a.Bodies.Num()!=b.Bodies.Num())return false;
    for(int32 i=0;i<a.Bodies.Num();++i)
    {
        const auto& x=a.Bodies[i];const auto& y=b.Bodies[i];
        if(x.BoxShapeCaptured!=y.BoxShapeCaptured)return false;if(!x.BoxShapeCaptured)continue;
        if(x.Body!=y.Body||x.Index!=y.Index||x.Class!=y.Class||x.GTObjectId!=y.GTObjectId||
            !GeneralBoxTransportDetail::Pose(x.State,y.State)||!SameVector(x.LocalOffset,y.LocalOffset)||
            !SameQuat(x.LocalRotation,y.LocalRotation)||!SameVector(x.BoxExtent,y.BoxExtent)||
            !x.SolidConfigCaptured||!y.SolidConfigCaptured||!SameNativeSolidResponseConfigValues(x.Config,y.Config))return false;
    }
    return true;
}
template<class TA,class TB> bool Same(const TA& before,const TB& after,TConstArrayView<const ISpeedComponent*> owners)
{
    if(owners.IsEmpty())return true;
    if(!before.Captured||!after.Captured)return false;
    for(const auto* owner:owners)
    {
        const auto* a=Find(before,owner);const auto* b=Find(after,owner);
        if(!a||!b||!SameOwner(*a,*b))return false;
    }
    return true;
}
template<class TA,class TB> void RequireUnchanged(const TA& before,const TB& after,TConstArrayView<const ISpeedComponent*> owners)
{
    if(owners.IsEmpty())return;
    if(!before.Captured||!after.Captured)Refuse("General Box operation has no complete actual before/after values");
    if(!Same(before,after,owners))Refuse("General Box final/other-owner operation changed an uncovered pose or shape");
}

static void Read(const FSimulationWorld& registry,const FPreparedWorldSnapshot& snapshot,
    TConstArrayView<const ISpeedComponent*> generalOwners,TFunctionRef<void()> requireContext,
    FFinalGeneralBoxStateValues& out)
{
    ::PreparedFrameworkProfile1::FScope observedFramework(::PreparedFrameworkProfile1::FrameStateRead);
    out={};requireContext();if(!ReadCurrentStep(out.Step))Refuse("Consumed response Step unavailable");
    const auto& ordered=registry.GetOrderedAdapters();const auto bindings=snapshot.GetNativeOwners();
    if(bindings->GetGeneration()!=out.Step.InputGeneration||bindings->Num()!=ordered.Num())Refuse("Consumed owner census changed");
    const int32 ownerReserve=observedFramework.Active?out.Owners.Max():0;
    out.Owners.Reserve(generalOwners.Num());observedFramework.Capacity(ownerReserve,out.Owners);
    const auto alignedRows=OrdinaryFrameOwnerRowLookup::CandidateRows(*bindings);
    for(int32 index=0;index<ordered.Num();++index)
    {
        const auto* owner=ordered[index];
        const auto* row=OrdinaryFrameOwnerRowLookup::Find(*bindings,alignedRows,index,owner);
        if(!owner||!row||!row->NativeObject||!row->Class||row->NativeObject->GetClass()!=row->Class||!row->BodyCensusCaptured)
            Refuse("Consumed owner raw GT identity unavailable");
        FFinalGeneralBoxOwnerValues* stored=nullptr;
        if(generalOwners.Contains(owner))
        {
            const int32 previous=observedFramework.Active?out.Owners.Max():0;
            stored=&out.Owners.AddDefaulted_GetRef();observedFramework.Capacity(previous,out.Owners);
            stored->Owner=owner;if(observedFramework.Active)++observedFramework.Owners;
        }
        const uint64 stableId=registry.FindStableId(*owner);FCanonicalPreparedFrameClock clock;
        if(!stableId||owner->GetPublishedSimulationStableId()!=stableId||
            !JoinCanonicalPreparedFrameClock(out.Step.Frame,owner->NumFrame(),clock))Refuse("Consumed owner stable identity/clock changed");
        if(stored)stored->StableId=stableId;
        // Preserve the actual owner getter order, including virtual calls and
        // the nested virtual reads in IsPhysicsSleeping/GetPhysRotation.
        if(stored)stored->State=owner->GetKinematicState();else (void)owner->GetKinematicState();
        const bool frozen=owner->IsFrozen();if(stored)stored->Frozen=frozen;
        const bool prescribed=owner->IsPrescribedMotion();if(stored)stored->Prescribed=prescribed;
        (void)owner->IsPhysicsSleeping();
        const FVector localCOM=owner->GetPhysCenterOfMassLocal();if(stored)stored->LocalCOM=localCOM;
        const FQuat physRotation=owner->GetPhysRotation();if(stored)stored->PhysRotation=physRotation;
        // GetPhysicalConstraints is a nonvirtual const-ref accessor. Its
        // payload has no final consumer, so no TArray copy is made here.
        const auto& bodies=owner->GetSubBodies();if(bodies.Num()!=row->Bodies.Num())Refuse("Consumed body census changed");
        if(stored)
        {
            const int32 previous=observedFramework.Active?stored->Bodies.Max():0;
            stored->Bodies.Reserve(bodies.Num());observedFramework.Capacity(previous,stored->Bodies);
        }
        for(int32 i=0;i<bodies.Num();++i)
        {
            const auto& binding=row->Bodies[i];const auto* body=binding.Body;
            if(!body||binding.Index!=i||bodies[i]!=body||!binding.Class||!binding.ObjectIdCaptured||
                body->GetClass()!=binding.Class||body->GetParentComponent()!=owner)Refuse("Consumed body raw GT identity changed");
            FFinalGeneralBoxBodyValues* keptBody=nullptr;
            if(stored)
            {
                const int32 previous=observedFramework.Active?stored->Bodies.Max():0;
                keptBody=&stored->Bodies.AddDefaulted_GetRef();observedFramework.Capacity(previous,stored->Bodies);
                if(observedFramework.Active)++observedFramework.Bodies;
            }
            // Body state/hit/ignored/local accessors are nonvirtual. Only the
            // box fields compared by the unchanged contract are retained.
            const auto* box=Cast<UBoxSubBody>(body);
            if(keptBody&&box)
            {
                keptBody->Body=body;keptBody->Class=binding.Class;keptBody->Index=i;keptBody->GTObjectId=binding.ObjectId;
                keptBody->State=body->GetKinematicState();keptBody->LocalOffset=body->GetLocalOffset();
                keptBody->LocalRotation=body->GetLocalRotation();
            }
            if(const auto* solid=Cast<USolidSubBody>(body))
            {
                // Preserve the old validity check and its virtual owner reads
                // even for solids whose payload the final comparator ignores.
                FNativeSolidResponseConfigValues ignoredConfig;
                auto& config=keptBody&&box?keptBody->Config:ignoredConfig;
                const bool captured=NativeSolidDetail::ReadResponseConfig(*solid,*owner,config);
                if(!captured)Refuse("Consumed native response configuration unavailable");
                // SupportRadius calls the virtual ContactOffset getter, and
                // the old reader calls that getter again separately. Keep
                // both calls in their original order, without retaining the
                // unused sphere payload.
                if(const auto* sphere=Cast<USphereSubBody>(body))
                {(void)sphere->GetRadius();(void)sphere->GetWorldStaticSupportRadius();(void)sphere->GetWorldStaticContactOffsetCm();}
                // GetBoxExtent overrides the virtual IBoxSweeper getter even
                // though UBoxSubBody omits the virtual keyword. Keep this call
                // for every box, including owners whose payload is not kept.
                if(box)
                {
                    const FVector extent=box->GetBoxExtent();
                    if(keptBody)
                    {keptBody->SolidConfigCaptured=true;keptBody->BoxShapeCaptured=true;keptBody->BoxExtent=extent;}
                }
            }
        }
    }
    requireContext();out.Captured=true;
}
}

void PoisonNativeRotationFailure() noexcept {RecordPreparedPoison("PreparedNativeFrameExecution.inl:1");}
bool FCanonicalFramePreflight::DispatchNativePrepare(ISpeedComponent& owner,const FCanonicalFrameContext& context)
{
    auto* frame=NativeFramePreflight;
    if(!frame||!frame->NativeConsumer)return false;
    if(Poisoned||!HostScope||!HostScope->Active||StepScope||!frame->Window||!frame->Window->IsOpen()||
        !frame->Window->GetNonce()||!IsCurrentWorker(frame->Worker)||frame->ThreadId!=FPlatformTLS::GetCurrentThreadId()||
        frame->Host!=HostScope->Host||frame->Bridge!=HostScope->Bridge||frame->Generation!=HostScope->Generation||
        frame->Frame!=context.NumFrame||!SameFloat(context.PhysicalDeltaTime,HostScope->Delta)||
        !SameFloat(context.SimTime,HostScope->SimTime)||frame->PreparedOwners.Contains(&owner)||
        !frame->NativeConsumer->Matches(frame->Snapshot,frame->Bridge->GetStaticCollisionWorld()->GetPreparedQueryService())||
        !HasUnchangedRawPreparationCensus(*frame->Bridge,*frame->Snapshot,frame->StepBefore))
        Refuse("Native preparation lost its current retained frame/census");
    Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    frame->NativeConsumer->PrepareOwner(owner,context);
    if(!HasUnchangedRawPreparationCensus(*frame->Bridge,*frame->Snapshot,frame->StepBefore)||
        !frame->NativeConsumer->Matches(frame->Snapshot,frame->Bridge->GetStaticCollisionWorld()->GetPreparedQueryService()))
        Refuse("Native preparation changed retained frame inventory");
    frame->PreparedOwners.Add(&owner);return true;
}
void FCanonicalFramePreflight::RequireClosedForSnapshot()
{
    if(!NativeConsumer)return;
    if(Poisoned||NativeFramePreflight!=this||!NativeStepClosed||SnapshotClosureConsumed||
        !NativeConsumer->Matches(Snapshot,Bridge->GetStaticCollisionWorld()->GetPreparedQueryService())||
        !HasUnchangedRawPreparationCensus(*Bridge,*Snapshot,Window->GetExpectedStep()))
        Refuse("Native FullFrame has no unique closed physical Step before snapshot");
    const uint64 previous=NativeConsumer->LastSnapshotClosure.load(std::memory_order_acquire);
    if(previous!=MAX_uint64&&(previous==MAX_uint64-1||Frame!=previous+1))
        Refuse("Native snapshot closure skipped or replayed a canonical frame");
    NativeConsumer->LastSnapshotClosure.store(Frame,std::memory_order_release);
    SnapshotClosureConsumed=true;
}
void FStepScope::RequireNativeFrameContext() const
{
    FStepIdentity current;
    if(!NativeConsumer||!Active||StepScope!=this||!ReadCurrentStep(current)||!Window||
        Window->GetPurpose()!=OnePausedWork::EPurpose::FullFrame||!NativeFramePreflight||
        NativeFramePreflight->NativeConsumer!=NativeConsumer||NativeFramePreflight->Snapshot!=Snapshot||
        NativeFramePreflight->NativeStepClosed||
        !NativeConsumer->Matches(Snapshot,Identity.Provider->GetPreparedQueryService())||
        !FCanonicalFramePreflight::HasUnchangedRawPreparationCensus(*Identity.Bridge,*Snapshot,Identity.StepSerial))
        Refuse("Native frame operation lost its actual worker/window/census");
}
void FStepScope::ConsumeNativeAdvanceTicket(float seconds,bool selection,TConstArrayView<SComponentTOI> selected,
    float horizon,float before)
{
    if(!NativeConsumer)return;
    RequireNativeFrameContext();FStepIdentity current;if(!ReadCurrentStep(current))Refuse("Native advance context unavailable");
    if(!NativeAdvance||NativeAdvance->Consumed||SweepPhase||NativeAdvance->Queried.QueryEpoch==MAX_uint64)
        Refuse("Native advance has no unconsumed private ticket");
    auto revoked=NativeAdvance->Queried;++revoked.QueryEpoch;
    if(!OrdinaryWorldDetail::Epoch(revoked,current)||revoked.QuerySerial!=current.QuerySerial||
        selection!=NativeAdvance->HasSelection||!SameFloat(seconds,NativeAdvance->Seconds)||
        !SameFloat(horizon,NativeAdvance->Horizon)||!SameFloat(before,NativeAdvance->Before)||
        selected.Num()!=NativeAdvance->Selected.Num())Refuse("Native advance ticket was retimed, revoked twice or substituted");
    for(int32 i=0;i<selected.Num();++i)
        if(!OrdinaryWorldDetail::Entry(selected[i],NativeAdvance->Selected[i],NativeAdvance->Selected[i].TOI))
            Refuse("Native advance ticket selected batch changed");
    NativeAdvance->Consumed=true;
}
bool FStepScope::DispatchNativeIntegration(ISpeedComponent& owner,float seconds)
{
    if(!NativeConsumer)return false;
    RequireNativeFrameContext();RequireOrdinaryIntegrationContext();
    const auto& tx=*OrdinaryIntegration;
    if(!NativeAdvance||!NativeAdvance->Consumed||!tx.Owners.IsValidIndex(tx.ActiveIndex)||
        tx.Owners[tx.ActiveIndex].Owner!=&owner||!SameFloat(seconds,tx.Seconds))
        Refuse("Native integration call is outside its consumed ticket/owner entry");
    NativeConsumer->IntegrateOwner(owner,seconds);return true;
}
bool FStepScope::DispatchNativeResponse(int32 call,USSubBody& body,const SHitResult& hit,float seconds,float simTime)
{
    if(!NativeConsumer)return false;
    RequireNativeFrameContext();RequireOrdinaryResponseContext();const auto& tx=*OrdinaryResponse;
    if(tx.ActiveCall!=call||!tx.Entries.IsValidIndex(tx.ActiveIndex)||
        tx.Entries[tx.ActiveIndex].RawResolver!=&body||!SameFloat(seconds,tx.Seconds)||!SameFloat(simTime,tx.SimTime))
        Refuse("Native response call is outside its actual selected entry");
    switch(call)
    {
        case 0:NativeConsumer->SetFutureHit(body,hit);break;
        case 1:NativeConsumer->AcceptHit(body);break;
        case 2:
        {
            const ISpeedComponent* owner=nullptr;
            for(const auto& row:Snapshot->GetNativeOwners()->GetRecordsForInspection())
                for(const auto& child:row.Bodies)if(child.Body==&body)owner=row.Adapter;
            if(!owner)Refuse("Native Resolve owner is outside the actual retained census");
            FOrdinaryOwnerProbePhaseScope probes(ENativeOwnerProbePhase::ResponseResolve,*owner,seconds);
            NativeConsumer->ResolveHit(body,seconds,simTime);probes.Returned();break;
        }
        default:Refuse("Unknown native response operation");
    }
    return true;
}
void FStepScope::RequireNativeContactPairTargets(const FSimulationWorld& registry,const USSubBody* a,const USSubBody* b)
{
    const auto* step=StepScope;
    const auto consumer=step?step->NativeConsumer:(NativeFramePreflight?NativeFramePreflight->NativeConsumer:TSharedPtr<const FPreparedNativeFrameConsumer,ESPMode::ThreadSafe>{});
    if(!consumer)return;
    FPreparedWorldSnapshotPtr snapshot;
    if(step){step->RequireNativeFrameContext();if(step->Registry!=&registry)Refuse("Native pairs belong to another registry");snapshot=step->Snapshot;}
    else
    {
        const auto* frame=NativeFramePreflight;
        if(!frame||&frame->Bridge->SimulationWorld!=&registry||
            !consumer->Matches(frame->Snapshot,frame->Bridge->GetStaticCollisionWorld()->GetPreparedQueryService())||
            !FCanonicalFramePreflight::HasUnchangedRawPreparationCensus(*frame->Bridge,*frame->Snapshot,frame->StepBefore))
            Refuse("Native prepare pair operation lost its retained registry");
        snapshot=frame->Snapshot;
    }
    if((a&&!consumer->ContainsBody(a))||(b&&!consumer->ContainsBody(b)))
        Refuse("Native pending-pair activation targets an uncaptured body");
    const auto inCensus=[&](const FWeakObjectPtr& token)
    {
        if(token.IsExplicitlyNull())return true;
        for(const auto& owner:snapshot->GetNativeOwners()->GetRecordsForInspection())for(const auto& child:owner.Bodies)
            if(token.HasSameIndexAndSerialNumber(child.Object))return true;
        return false;
    };
    for(const auto* pairs:{&registry.GetDynamicContactPairs(),&registry.GetPendingRollingContactPairs()})
        for(const auto& pair:*pairs)
            if(!inCensus(static_cast<FWeakObjectPtr>(pair.BodyA))||!inCensus(static_cast<FWeakObjectPtr>(pair.BodyB)))
                Refuse("Native persistent-pair target lies outside retained weak index/serial census");
}
void FStepScope::RequireOrdinaryFinalContext() const
{
    ::PreparedFrameworkProfile1::FScope observedFramework(::PreparedFrameworkProfile1::FinalContext);
    RequireNativeFrameContext();FStepIdentity current;
    if(!OrdinaryFinal||OrdinaryFinal->Closed||!OrdinaryFinal->CallOpen||!ReadCurrentStep(current)||
        !OrdinaryWorldDetail::Epoch(OrdinaryFinal->Entry,current)||
        OrdinaryFinal->Entry.QuerySerial!=current.QuerySerial||SweepPhase||ComponentSweepScope||MemberSweepScope||
        OwnerProbePhase||WheelProbeScope||(OrdinaryIntegration&&!OrdinaryIntegration->Closed)||
        (OrdinaryResponse&&!OrdinaryResponse->Closed))Refuse("Native final operation lost its distinct lexical context");
}
void FStepScope::BeginNativeFinalOperation(ENativeFrameFinalOperation kind,const ISpeedComponent* owner,int32 index,bool called)
{
    if(!NativeConsumer)return;
    RequireNativeFrameContext();FStepIdentity current;
    if(!ReadCurrentStep(current)||SweepPhase||ComponentSweepScope||MemberSweepScope||OwnerProbePhase||WheelProbeScope||
        (NativeAdvance&&!NativeAdvance->Consumed)||(OrdinaryIntegration&&!OrdinaryIntegration->Closed)||
        (OrdinaryResponse&&!OrdinaryResponse->Closed))Refuse("Native final operation preceded an incomplete physical phase");
    if(!OrdinaryFinal){OrdinaryFinal=MakeUnique<FOrdinaryFinalState>();OrdinaryFinal->Entry=current;}
    auto& state=*OrdinaryFinal;const auto& ordered=Registry->GetOrderedAdapters();
    if(state.Closed||state.CallOpen||int32(kind)!=state.Stage)Refuse("Native final stage repeated or reordered");
    const bool ownerStage=kind==ENativeFrameFinalOperation::StaticProjectionOwner||kind==ENativeFrameFinalOperation::PostOwner;
    if(ownerStage)
    {
        if(!called||index!=state.NextIndex||!ordered.IsValidIndex(index)||ordered[index]!=owner||
            !owner||!NativeConsumer->ContainsOwner(owner))Refuse("Native final owner omitted, reordered or substituted");
    }
    else if(owner||index!=INDEX_NONE||(!called&&kind!=ENativeFrameFinalOperation::MergeRollingPairs))
        Refuse("Native final system operation has a different target or branch");
    NativeConsumer->RequireResponseTargetsInCensus();
    RequireNativeContactPairTargets(*Registry);
    auto& operation=state.Operations.AddDefaulted_GetRef();operation.Kind=kind;operation.Owner=owner;
    operation.Index=index;operation.Called=called;state.CallOpen=true;
    if(FinalGeneralBoxCaptureDetail::Enabled.GetValueOnAnyThread()==1)
    {
        operation.GeneralBoxOnly=MakeUnique<FFinalGeneralBoxObservation>();
        FinalGeneralBoxCaptureDetail::Read(*Registry,*Snapshot,GeneralBoxOwners,
            [&](){RequireOrdinaryFrameStateReadContext();},operation.GeneralBoxOnly->Before);
    }
    else ReadOrdinaryFrameStateValues(operation.Before);
    if(!GeneralBoxOwners.IsEmpty())
    {
        if(!OrdinaryIntegration||!OrdinaryIntegration->Closed)Refuse("General Box final operation lacks a closed actual integration");
        if(const auto* baseline=CurrentFinalWheelSupportBaseline(state))
        {
            if(operation.GeneralBoxOnly)
                FinalGeneralBoxCaptureDetail::RequireUnchanged(*baseline,operation.GeneralBoxOnly->Before,GeneralBoxOwners);
            else FinalGeneralBoxCaptureDetail::RequireUnchanged(*baseline,operation.Before,GeneralBoxOwners);
        }
        else if(operation.GeneralBoxOnly)
            FinalGeneralBoxCaptureDetail::RequireUnchanged(GeneralBoxCurrentClosedBaseline(),operation.GeneralBoxOnly->Before,GeneralBoxOwners);
        else RequireGeneralBoxUnchanged(GeneralBoxCurrentClosedBaseline(),operation.Before);
    }
}
void FStepScope::EndNativeFinalOperation()
{
    if(!NativeConsumer)return;
    RequireOrdinaryFinalContext();auto& state=*OrdinaryFinal;
    if(state.Operations.IsEmpty()||state.Operations.Last().Returned)Refuse("Native final operation has no unique actual return");
    auto& operation=state.Operations.Last();
    if(operation.WheelSupportExpected&&(operation.Kind!=ENativeFrameFinalOperation::PostOwner||
        !operation.GeneralBoxOnly||state.WheelSupportOperation!=INDEX_NONE||FNativeFinalWheelSupportScope::HasPendingSync()))
        Refuse("585 final wheel support has no unique closed post effect");
    operation.Returned=true;
    if(operation.GeneralBoxOnly)
    {
        FinalGeneralBoxCaptureDetail::Read(*Registry,*Snapshot,GeneralBoxOwners,
            [&](){RequireOrdinaryFrameStateReadContext();},operation.GeneralBoxOnly->After);
        const auto& expected=operation.WheelSupportExpected?*operation.WheelSupportExpected:operation.GeneralBoxOnly->Before;
        FinalGeneralBoxCaptureDetail::RequireUnchanged(expected,operation.GeneralBoxOnly->After,GeneralBoxOwners);
        if(operation.WheelSupportExpected)
        {
            state.WheelSupportOperation=state.Operations.Num()-1;
            UE_LOG(LogTemp,Display,TEXT("[NativeCase585FinalWheelSupportClosed] Frame=%llu Serial=%llu"),
                state.Entry.Frame,state.Entry.QuerySerial);
        }
    }
    else
    {
        ReadOrdinaryFrameStateValues(operation.After);
        RequireGeneralBoxUnchanged(operation.Before,operation.After);
    }
    state.CallOpen=false;
    if(operation.Kind==ENativeFrameFinalOperation::StaticProjectionOwner||operation.Kind==ENativeFrameFinalOperation::PostOwner)
    {if(++state.NextIndex==Registry->GetOrderedAdapters().Num()){state.NextIndex=0;++state.Stage;}}
    else ++state.Stage;
}
bool FStepScope::DispatchNativeFinalOwner(ISpeedComponent& owner,float seconds,bool post)
{
    if(!NativeConsumer)return false;
    RequireNativeFrameContext();
    if(!OrdinaryFinal||!OrdinaryFinal->CallOpen||OrdinaryFinal->Operations.IsEmpty()||
        OrdinaryFinal->Operations.Last().Owner!=&owner||!SameFloat(seconds,Identity.Delta)||
        OrdinaryFinal->Operations.Last().Kind!=(post?ENativeFrameFinalOperation::PostOwner:ENativeFrameFinalOperation::StaticProjectionOwner))
        Refuse("Native final owner dispatch is outside its actual operation");
    if(post)NativeConsumer->PostOwner(owner,seconds);
    else {NativeConsumer->RequireOwner(&owner);owner.ProjectEstablishedStaticContacts(seconds);}
    return true;
}
void FStepScope::CloseNativeFrameStep(float actualTime)
{
    if(!NativeConsumer)return;
    RequireNativeFrameContext();
    if(!OrdinaryFinal||OrdinaryFinal->Closed||OrdinaryFinal->CallOpen||OrdinaryFinal->Stage!=6||
        OrdinaryFinal->NextIndex||!SameFloat(actualTime,Identity.Delta)||
        (NativeAdvance&&!NativeAdvance->Consumed)||(OrdinaryIntegration&&!OrdinaryIntegration->Closed)||
        (OrdinaryResponse&&!OrdinaryResponse->Closed))Refuse("Native physical Step closure is incomplete before snapshot");
    for(const auto& operation:OrdinaryFinal->Operations)if(!operation.Returned)Refuse("Native final operation did not return");
#if !UE_BUILD_SHIPPING
    if(Case585FourthGround&&!Case585FifthResolved)
        Refuse("Case585 frame cannot publish an unclosed fifth curved reimpact");
#endif
    OrdinaryFinal->ActualEndTime=actualTime;OrdinaryFinal->Closed=true;NativeFramePreflight->NativeStepClosed=true;
}

#if WITH_DEV_AUTOMATION_TESTS
namespace FinalGeneralBoxCaptureDetail
{
static FFinalGeneralBoxOwnerValues ProjectForTesting(const FOrdinaryConsumedOwnerValues& source)
{
    FFinalGeneralBoxOwnerValues out;
    out.Owner=source.Owner;out.StableId=source.StableId;out.State=source.State;
    out.Frozen=source.Frozen;out.Prescribed=source.Prescribed;out.LocalCOM=source.LocalCOM;out.PhysRotation=source.PhysRotation;
    for(const auto& body:source.Bodies)
    {
        auto& row=out.Bodies.AddDefaulted_GetRef();row.BoxShapeCaptured=body.BoxShapeCaptured;
        if(!body.BoxShapeCaptured)continue;
        row.Body=body.Body;row.Class=body.Class;row.Index=body.Index;row.GTObjectId=body.GTObjectId;
        row.State=body.State;row.LocalOffset=body.LocalOffset;row.LocalRotation=body.LocalRotation;
        row.BoxExtent=body.BoxExtent;row.Config=body.Config;row.SolidConfigCaptured=body.SolidConfigCaptured;
    }
    return out;
}
static FOrdinaryConsumedOwnerValues Fixture()
{
    FOrdinaryConsumedOwnerValues owner;owner.Owner=reinterpret_cast<const ISpeedComponent*>(UPTRINT(0x1000));
    owner.StableId=42;owner.State.Location=FVector(1,2,3);owner.LocalCOM=FVector(4,5,6);
    auto& box=owner.Bodies.AddDefaulted_GetRef();
    box.Body=reinterpret_cast<const USSubBody*>(UPTRINT(0x2000));box.Class=reinterpret_cast<const void*>(UPTRINT(0x3000));
    box.Index=0;box.GTObjectId=13;box.State.Location=FVector(7,8,9);box.LocalOffset=FVector(10,11,12);
    box.BoxExtent=FVector(2,3,4);box.BoxShapeCaptured=true;box.SolidConfigCaptured=true;box.Config.BodyMass=7;
    auto& nonbox=owner.Bodies.AddDefaulted_GetRef();
    nonbox.Body=reinterpret_cast<const USSubBody*>(UPTRINT(0x4000));nonbox.Index=1;
    nonbox.SphereShapeCaptured=true;nonbox.PhysicalRadius=15;nonbox.Ignored.Add(nullptr);
    owner.Constraints.AddDefaulted();return owner;
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCompactFinalGeneralBoxFieldsTest,
 "IAmSpeed.AnalyticWorld.NativeSolid.FinalGeneralBoxCapture.FieldsAndOracle",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCompactFinalGeneralBoxFieldsTest::RunTest(const FString&)
{
    using namespace FinalGeneralBoxCaptureDetail;
    const auto original=Fixture();const auto compact=ProjectForTesting(original);
    TestTrue(TEXT("unchanged compact/full compare identically"),SameOwner(original,compact)&&SameOwner(compact,original));
    int32 checked=0;
    const auto checkCase=[&](const TCHAR* name,auto change,bool expected)
    {
        auto changed=original;change(changed);const auto projected=ProjectForTesting(changed);
        const bool oracle=GeneralBoxTransportDetail::SamePoseAndBoxConfiguration(original,changed);
        TestEqual(FString(name)+TEXT(" old oracle"),oracle,expected);
        TestEqual(FString(name)+TEXT(" full/compact"),SameOwner(original,projected),oracle);
        TestEqual(FString(name)+TEXT(" compact/full"),SameOwner(compact,changed),oracle);
        TestEqual(FString(name)+TEXT(" compact/compact"),SameOwner(compact,projected),oracle);++checked;
    };
    checkCase(TEXT("owner identity"),[](auto& x){x.Owner=nullptr;},false);
    checkCase(TEXT("stable ID"),[](auto& x){++x.StableId;},false);
    checkCase(TEXT("frozen"),[](auto& x){x.Frozen=!x.Frozen;},false);
    checkCase(TEXT("prescribed"),[](auto& x){x.Prescribed=!x.Prescribed;},false);
    checkCase(TEXT("owner location"),[](auto& x){x.State.Location.X+=1;},false);
    checkCase(TEXT("owner rotation"),[](auto& x){x.State.Rotation.W=-x.State.Rotation.W;},false);
    checkCase(TEXT("COM"),[](auto& x){x.LocalCOM.Y+=1;},false);
    checkCase(TEXT("physical rotation"),[](auto& x){x.PhysRotation.X+=1;},false);
    checkCase(TEXT("body inventory"),[](auto& x){x.Bodies.AddDefaulted();},false);
    checkCase(TEXT("box capture"),[](auto& x){x.Bodies[0].BoxShapeCaptured=false;},false);
    checkCase(TEXT("new box capture"),[](auto& x){x.Bodies[1].BoxShapeCaptured=true;},false);
    checkCase(TEXT("body identity"),[](auto& x){x.Bodies[0].Body=nullptr;},false);
    checkCase(TEXT("body class"),[](auto& x){x.Bodies[0].Class=nullptr;},false);
    checkCase(TEXT("body index"),[](auto& x){++x.Bodies[0].Index;},false);
    checkCase(TEXT("GT object ID"),[](auto& x){++x.Bodies[0].GTObjectId;},false);
    checkCase(TEXT("body location"),[](auto& x){x.Bodies[0].State.Location.Z+=1;},false);
    checkCase(TEXT("body rotation"),[](auto& x){x.Bodies[0].State.Rotation.W=-x.Bodies[0].State.Rotation.W;},false);
    checkCase(TEXT("local offset"),[](auto& x){x.Bodies[0].LocalOffset.X+=1;},false);
    checkCase(TEXT("local rotation"),[](auto& x){x.Bodies[0].LocalRotation.Y+=1;},false);
    checkCase(TEXT("extent"),[](auto& x){x.Bodies[0].BoxExtent.Z+=1;},false);
    checkCase(TEXT("missing config"),[](auto& x){x.Bodies[0].SolidConfigCaptured=false;},false);
    checkCase(TEXT("body mass"),[](auto& x){x.Bodies[0].Config.BodyMass+=1;},false);
    checkCase(TEXT("owner mass"),[](auto& x){x.Bodies[0].Config.OwnerPhysMass+=1;},false);
    checkCase(TEXT("restitution"),[](auto& x){x.Bodies[0].Config.Restitution+=1;},false);
    checkCase(TEXT("impact"),[](auto& x){x.Bodies[0].Config.ImpactThreshold+=1;},false);
    checkCase(TEXT("static friction"),[](auto& x){x.Bodies[0].Config.StaticFriction+=1;},false);
    checkCase(TEXT("dynamic friction"),[](auto& x){x.Bodies[0].Config.DynamicFriction+=1;},false);
    checkCase(TEXT("damping"),[](auto& x){x.Bodies[0].Config.HitDamping+=1;},false);
    checkCase(TEXT("max speed"),[](auto& x){x.Bodies[0].Config.OwnerMaxSpeed+=1;},false);
    checkCase(TEXT("max angular speed"),[](auto& x){x.Bodies[0].Config.OwnerMaxAngularSpeed+=1;},false);
    checkCase(TEXT("off-diagonal inertia"),[](auto& x){x.Bodies[0].Config.CachedBodyLocalInvInertia.M[1][2]+=1;},false);
    checkCase(TEXT("main body"),[](auto& x){x.Bodies[0].Config.MainSubBody=true;},false);
    checkCase(TEXT("rest force"),[](auto& x){x.Bodies[0].Config.ApplyRestForce=true;},false);
    checkCase(TEXT("aerial model"),[](auto& x){x.Bodies[0].Config.AerialShotContactModel=true;},false);
    checkCase(TEXT("projection captured"),[](auto& x){x.Bodies[0].Config.SphereProjectionPolicyCaptured=true;},false);
    checkCase(TEXT("projection enabled"),[](auto& x){x.Bodies[0].Config.SphereWorldStaticProjectionEnabled=true;},false);
    checkCase(TEXT("unconsumed owner velocity"),[](auto& x){x.State.Velocity.X+=1;},true);
    checkCase(TEXT("unconsumed sleeping eligibility"),[](auto& x){x.SleepingEligibility=!x.SleepingEligibility;},true);
    checkCase(TEXT("unconsumed constraints"),[](auto& x){x.Constraints.Reset();},true);
    checkCase(TEXT("unconsumed body velocity"),[](auto& x){x.Bodies[0].State.Velocity.Y+=1;},true);
    checkCase(TEXT("unconsumed hit"),[](auto& x){x.Bodies[0].Current.TOI+=1;x.Bodies[0].Future.TOI+=1;},true);
    checkCase(TEXT("unconsumed ignored"),[](auto& x){x.Bodies[0].Ignored.Add(nullptr);},true);
    checkCase(TEXT("unconsumed nonbox"),[](auto& x){x.Bodies[1].State.Location.X+=1;x.Bodies[1].PhysicalRadius+=1;},true);
    TestEqual(TEXT("all required and deliberately absent field cases exercised"),checked,43);
    return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCompactFinalGeneralBoxOwnershipTest,
 "IAmSpeed.AnalyticWorld.NativeSolid.FinalGeneralBoxCapture.OwnershipAndMode",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCompactFinalGeneralBoxOwnershipTest::RunTest(const FString&)
{
    using namespace FinalGeneralBoxCaptureDetail;
    const auto source=Fixture();TArray<const ISpeedComponent*> owners{source.Owner};
    FOrdinaryConsumedStateValues full;full.Captured=true;full.Owners.Add(source);
    FFinalGeneralBoxStateValues first;first.Captured=true;first.Owners.Add(ProjectForTesting(source));
    auto second=first;
    TestTrue(TEXT("full baseline and independent compact capture match"),Same(full,first,owners)&&Same(first,second,owners));
    second.Owners[0].Bodies[0].State.Location.X+=1;
    TestTrue(TEXT("later callback mutation is detected and never rewrites prior capture"),
        !Same(first,second,owners)&&Same(full,first,owners)&&first.Owners[0].Bodies.GetData()!=second.Owners[0].Bodies.GetData());
    second=first;second.Captured=false;TestFalse(TEXT("unclosed capture refused"),Same(first,second,owners));
    second=first;second.Owners.Reset();TestFalse(TEXT("missing required owner refused"),Same(first,second,owners));
    TArray<const ISpeedComponent*> empty;
    TestTrue(TEXT("empty GeneralBox set keeps the old empty-comparison contract"),Same(first,second,empty));
    FOrdinaryFinalOperation operation;
    {
        FScopedCVar<int32> on(TEXT("p.IAmSpeed.AnalyticWorld.FinalGeneralBoxCapture"),1,FName(TEXT("FinalCaptureOwnershipOn")));
        if(Enabled.GetValueOnAnyThread()==1)operation.GeneralBoxOnly=MakeUnique<FFinalGeneralBoxObservation>();
        {
            FScopedCVar<int32> off(TEXT("p.IAmSpeed.AnalyticWorld.FinalGeneralBoxCapture"),0,FName(TEXT("FinalCaptureOwnershipOff")));
            TestTrue(TEXT("operation format remains latched across a callback flag change"),operation.GeneralBoxOnly.IsValid());
        }
        TestEqual(TEXT("nested test tags preserve outer mode"),Enabled.GetValueOnAnyThread(),1);
    }
    operation.GeneralBoxOnly->Before=MoveTemp(first);operation.GeneralBoxOnly->After=operation.GeneralBoxOnly->Before;
    TestTrue(TEXT("separate boundaries retain separate immutable payloads"),
        Same(operation.GeneralBoxOnly->Before,operation.GeneralBoxOnly->After,owners));
    TestTrue(TEXT("compact payload is structurally smaller"),sizeof(FFinalGeneralBoxBodyValues)<sizeof(FOrdinaryConsumedBodyValues)&&
        sizeof(FFinalGeneralBoxOwnerValues)<sizeof(FOrdinaryConsumedOwnerValues));
    static_assert(!std::is_copy_constructible_v<FOrdinaryFinalOperation>);
    static_assert(std::is_move_constructible_v<FOrdinaryFinalOperation>);
    TArray<FOrdinaryFinalOperation> operations;
    TArray<const FFinalGeneralBoxObservation*> payloads;
    for(int32 i=0;i<24;++i)
    {
        auto& next=operations.AddDefaulted_GetRef();next.Index=i;
        next.GeneralBoxOnly=MakeUnique<FFinalGeneralBoxObservation>();
        next.GeneralBoxOnly->Before.Captured=true;
        next.GeneralBoxOnly->Before.Owners.Add(ProjectForTesting(source));
        payloads.Add(next.GeneralBoxOnly.Get());
    }
    for(int32 i=0;i<operations.Num();++i)
        TestTrue(TEXT("actual AddDefaulted growth moves ownership without copying or losing earlier boundaries"),
            operations[i].Index==i&&operations[i].GeneralBoxOnly.Get()==payloads[i]&&
            Same(full,operations[i].GeneralBoxOnly->Before,owners));
    return !HasAnyErrors();
}
#endif
