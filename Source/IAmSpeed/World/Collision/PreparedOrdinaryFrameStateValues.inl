namespace OrdinaryFrameOwnerRowLookup
{
TAutoConsoleVariable<int32> Aligned(TEXT("p.IAmSpeed.AnalyticWorld.AlignedNativeOwnerRows"),0,
    TEXT("Default OFF. Match immutable native owner rows by checked ordinal before using the original lookup."));

static TConstArrayView<FPreparedNativeOwnerBinding> CandidateRows(const FPreparedNativeOwnerBindings& bindings)
{
    return Aligned.GetValueOnAnyThread()==1?bindings.GetRecordsForInspection():TConstArrayView<FPreparedNativeOwnerBinding>{};
}
static const FPreparedNativeOwnerBinding* Find(const FPreparedNativeOwnerBindings& bindings,
    TConstArrayView<FPreparedNativeOwnerBinding> rows,int32 index,const ISpeedComponent* owner)
{
    // SelectGT rejects duplicate adapters and publishes immutable Rows.
    // Matching the actual pointer therefore selects the same unique row as
    // Find. A different order stays on the original first-match search.
    if(rows.IsValidIndex(index)&&rows[index].Adapter==owner)return &rows[index];
    return bindings.Find(owner);
}
}

namespace PlanarSweepRefreshRead
{
thread_local const FStepScope* OwnerScope=nullptr;
struct FScope
{
    const FStepScope* Before;
    FScope(const FStepScope* scope,bool enabled):Before(OwnerScope)
    {
        if(enabled)OwnerScope=scope;
    }
    ~FScope(){OwnerScope=Before;}
};
}

// Shared value copy, still bound to exactly ONE authenticated lexical consumer.
void FStepScope::RequireOrdinaryFrameStateReadContext() const
{
    if(PlanarSweepRefreshRead::OwnerScope==this)
    {
        RequireNativeFrameContext();
        if(!SweepPhase||OrdinaryResponse||OrdinaryIntegration||OrdinaryFinal||
            MemberSweepScope||WheelProbeScope||OwnerProbePhase)
            Refuse("Planar refresh read is outside the next sweep preparation");
        return;
    }
    const bool response=OrdinaryResponse&&!OrdinaryResponse->Closed;
    const bool integration=OrdinaryIntegration&&!OrdinaryIntegration->Closed;
    const bool final=OrdinaryFinal&&!OrdinaryFinal->Closed&&OrdinaryFinal->CallOpen;
    if(int32(response)+int32(integration)+int32(final)!=1)Refuse("Native frame state read lacks one unique lexical consumer");
    if(response)RequireOrdinaryResponseContext();else if(integration)RequireOrdinaryIntegrationContext();else RequireOrdinaryFinalContext();
}
void FStepScope::ReadOrdinaryFrameStateValues(FOrdinaryConsumedStateValues& out) const
{
    ::PreparedFrameworkProfile1::FScope observedFramework(::PreparedFrameworkProfile1::FrameStateRead);
    bool recycled=false;
    if(OrdinaryIntegration&&!OrdinaryIntegration->Closed&&
        OrdinaryIntegration->Owners.IsValidIndex(OrdinaryIntegration->ActiveIndex))
    {
        const auto& active=OrdinaryIntegration->Owners[OrdinaryIntegration->ActiveIndex];
        recycled=active.RecycledStateBuffers&&(&out==&active.Before||&out==&active.After);
    }
    if(recycled){out.Captured=false;out.Step={};}
    else out={};
    RequireOrdinaryFrameStateReadContext();if(!ReadCurrentStep(out.Step))Refuse("Consumed response Step unavailable");
    const auto& ordered=Registry->GetOrderedAdapters();const auto bindings=Snapshot->GetNativeOwners();
    if(bindings->GetGeneration()!=out.Step.InputGeneration||bindings->Num()!=ordered.Num())Refuse("Consumed owner census changed");
    // Only positive ordinary integration rows admit the compact payload. It
    // keeps the active owner's complete values and every current General Box
    // comparison owner. All live census and getter validations still run.
    const ISpeedComponent* integrationTarget=nullptr;
    if(OrdinaryIntegration&&!OrdinaryIntegration->Closed&&
        OrdinaryIntegration->Owners.IsValidIndex(OrdinaryIntegration->ActiveIndex))
    {
        const auto& tx=*OrdinaryIntegration;const auto& row=tx.Owners[tx.ActiveIndex];
        if(row.CompactStateCapture)
        {
            if(!row.Owner||!row.Opened||row.Closed||row.Index!=tx.ActiveIndex||!(tx.Seconds>0.f))
                Refuse("Compact integration state lacks its live positive owner call");
            integrationTarget=row.Owner;
        }
    }
    const int32 retainedCount=integrationTarget?GeneralBoxOwners.Num()+int32(!GeneralBoxOwners.Contains(integrationTarget)):ordered.Num();
    const int32 observedOwnerReserve=observedFramework.Active?out.Owners.Max():0;
    out.Owners.Reserve(retainedCount);observedFramework.Capacity(observedOwnerReserve,out.Owners);
    if(recycled)out.Owners.SetNum(retainedCount,EAllowShrinking::No);
    int32 retainedIndex=0;
    const auto alignedRows=OrdinaryFrameOwnerRowLookup::CandidateRows(*bindings);
    for(int32 index=0;index<ordered.Num();++index)
    {
        const auto* owner=ordered[index];
        const auto* row=OrdinaryFrameOwnerRowLookup::Find(*bindings,alignedRows,index,owner);
        if(!owner||!row||!row->NativeObject||!row->Class||row->NativeObject->GetClass()!=row->Class||!row->BodyCensusCaptured)
            Refuse("Consumed owner raw GT identity unavailable");
        const bool retain=!integrationTarget||owner==integrationTarget||GeneralBoxOwners.Contains(owner);
        const int32 observedOwnerCapacity=observedFramework.Active?out.Owners.Max():0;
        if(recycled&&retain&&!out.Owners.IsValidIndex(retainedIndex))Refuse("Recycled integration owner exceeds retained roster");
        auto* v=retain?(recycled?&out.Owners[retainedIndex]:&out.Owners.AddDefaulted_GetRef()):nullptr;
        if(v)++retainedIndex;
        if(recycled&&v)
        {
            auto constraints=MoveTemp(v->Constraints);auto bodies=MoveTemp(v->Bodies);
            *v={};v->Constraints=MoveTemp(constraints);v->Bodies=MoveTemp(bodies);
        }
        observedFramework.Capacity(observedOwnerCapacity,out.Owners);
        if(observedFramework.Active&&v)++observedFramework.Owners;
        if(v)v->Owner=owner;
        const uint64 stableId=Registry->FindStableId(*owner);
        if(v)v->StableId=stableId;
        FCanonicalPreparedFrameClock unusedClock;
        if(!stableId||owner->GetPublishedSimulationStableId()!=stableId||
            !JoinCanonicalPreparedFrameClock(out.Step.Frame,owner->NumFrame(),v?v->Clock:unusedClock))
            Refuse("Consumed owner stable identity/clock changed");
        if(v)
        {
            v->State=owner->GetKinematicState();v->Frozen=owner->IsFrozen();v->Prescribed=owner->IsPrescribedMotion();v->SleepingEligibility=owner->IsPhysicsSleeping();
            v->LocalCOM=owner->GetPhysCenterOfMassLocal();v->PhysRotation=owner->GetPhysRotation();
        }
        else
        {
            // Compact omission suppresses value storage only, not live reads.
            (void)owner->GetKinematicState();(void)owner->IsFrozen();(void)owner->IsPrescribedMotion();(void)owner->IsPhysicsSleeping();
            (void)owner->GetPhysCenterOfMassLocal();(void)owner->GetPhysRotation();
        }
        const auto& constraints=owner->GetPhysicalConstraints();
        if(v)
        {
            const int32 previous=observedFramework.Active?v->Constraints.Max():0;
            if(recycled){v->Constraints.Reset();v->Constraints.Append(constraints);}
            else v->Constraints=constraints;
            observedFramework.Capacity(previous,v->Constraints);
            if(observedFramework.Active)observedFramework.Constraints+=uint64(v->Constraints.Num());
        }
        const auto& bodies=owner->GetSubBodies();if(bodies.Num()!=row->Bodies.Num())Refuse("Consumed body census changed");
        if(v)
        {
            const int32 observedBodyReserve=observedFramework.Active?v->Bodies.Max():0;
            v->Bodies.Reserve(bodies.Num());observedFramework.Capacity(observedBodyReserve,v->Bodies);
            if(recycled)v->Bodies.SetNum(bodies.Num(),EAllowShrinking::No);
        }
        for(int32 i=0;i<bodies.Num();++i)
        {
            const auto& binding=row->Bodies[i];const auto* body=binding.Body;
            if(!body||binding.Index!=i||bodies[i]!=body||!binding.Class||!binding.ObjectIdCaptured||
                body->GetClass()!=binding.Class||body->GetParentComponent()!=owner)Refuse("Consumed body raw GT identity changed");
            FOrdinaryConsumedBodyValues* b=nullptr;
            if(v)
            {
                const int32 observedBodyCapacity=observedFramework.Active?v->Bodies.Max():0;
                b=recycled?&v->Bodies[i]:&v->Bodies.AddDefaulted_GetRef();
                if(recycled)
                {
                    auto ignored=MoveTemp(b->Ignored);
                    *b={};b->Ignored=MoveTemp(ignored);b->Ignored.Reset();
                }
                observedFramework.Capacity(observedBodyCapacity,v->Bodies);
                if(observedFramework.Active)++observedFramework.Bodies;
                b->Body=body;b->Class=binding.Class;b->Index=i;b->GTObjectId=binding.ObjectId;
                b->State=body->GetKinematicState();b->Current=body->GetHit();b->Future=body->GetFutureHit();
                const int32 observedIgnoredReserve=observedFramework.Active?b->Ignored.Max():0;
                b->Ignored.Reserve(body->IgnoredComponents.Num());observedFramework.Capacity(observedIgnoredReserve,b->Ignored);
                for(const auto* p:body->IgnoredComponents)
                {const int32 observedIgnoredCapacity=observedFramework.Active?b->Ignored.Max():0;b->Ignored.Add(p);
                    observedFramework.Capacity(observedIgnoredCapacity,b->Ignored);if(observedFramework.Active)++observedFramework.Ignored;} // identity copy, never dereference
                b->LocalOffset=body->GetLocalOffset();b->LocalRotation=body->GetLocalRotation();
            }
            else
            {
                (void)body->GetKinematicState();(void)body->GetHit();(void)body->GetFutureHit();
                (void)body->GetLocalOffset();(void)body->GetLocalRotation();
            }
            if(const auto* solid=Cast<USolidSubBody>(body))
            {
                if(b)
                {
                    b->SolidConfigCaptured=NativeSolidDetail::ReadResponseConfig(*solid,*owner,b->Config);
                    if(!b->SolidConfigCaptured)Refuse("Consumed native response configuration unavailable");
                }
                else
                {
                    FNativeSolidResponseConfigValues unusedConfig;
                    if(!NativeSolidDetail::ReadResponseConfig(*solid,*owner,unusedConfig))
                        Refuse("Consumed native response configuration unavailable");
                }
                if(const auto* sphere=Cast<USphereSubBody>(body))
                {
                    if(b)
                    {b->SphereShapeCaptured=true;b->PhysicalRadius=sphere->GetRadius();b->SupportRadius=sphere->GetWorldStaticSupportRadius();b->ContactOffset=sphere->GetWorldStaticContactOffsetCm();}
                    else
                    {(void)sphere->GetRadius();(void)sphere->GetWorldStaticSupportRadius();(void)sphere->GetWorldStaticContactOffsetCm();}
                }
                if(const auto* box=Cast<UBoxSubBody>(body))
                {
                    if(b){b->BoxShapeCaptured=true;b->BoxExtent=box->GetBoxExtent();}
                    else (void)box->GetBoxExtent();
                }
            }
            // No virtual inertia query is added here: actual operator outputs
            // are captured ONLY where native functions already produce them.
        }
    }
    if(out.Owners.Num()!=retainedCount||(recycled&&retainedIndex!=retainedCount))Refuse("Consumed owner payload census differs");
    RequireOrdinaryFrameStateReadContext();out.Captured=true;
}

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FObservedFrameStorageIsolationTest,
 "IAmSpeed.AnalyticWorld.NativeSolid.FrameStateStorageIsolation",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FObservedFrameStorageIsolationTest::RunTest(const FString&)
{
    // Opaque identities only; never construct, resolve or dereference UObject.
    uint8 opaque[10]{};
    const auto pointer=[&](int32 i){return reinterpret_cast<const UPrimitiveComponent*>(&opaque[i]);};
    FOrdinaryConsumedStateValues before;before.Owners.Reserve(3);
    for(int32 i=0;i<3;++i)before.Owners.AddDefaulted();
    auto& owner=before.Owners[2];owner.Bodies.Reserve(5);
    for(int32 i=0;i<5;++i)owner.Bodies.AddDefaulted();
    auto& body=owner.Bodies[4];body.State.Location=FVector(1,2,3);
    for(int32 i=0;i<8;++i)body.Ignored.Add(pointer(i));
    if(!TestTrue(TEXT("eight ignored identities require no secondary heap allocation"),
        body.Ignored.Num()==8&&body.Ignored.GetAllocatedSize()==0))return false;
    auto inlineCopy=before;inlineCopy.Owners[2].Bodies[4].Ignored[0]=pointer(9);
    inlineCopy.Owners[2].Bodies[4].State.Location.X=42;
    TestTrue(TEXT("nested observation copy owns independent inline values"),
        before.Owners[2].Bodies[4].Ignored[0]==pointer(0)&&before.Owners[2].Bodies[4].State.Location.X==1&&
        inlineCopy.Owners[2].Bodies[4].Ignored.GetData()!=before.Owners[2].Bodies[4].Ignored.GetData());
    body.Ignored.Add(pointer(8));
    if(!TestTrue(TEXT("larger lists retain ordinary complete heap fallback"),
        body.Ignored.Num()==9&&body.Ignored.GetAllocatedSize()>0&&body.Ignored[8]==pointer(8)))return false;
    auto heapCopy=before;heapCopy.Owners[2].Bodies[4].Ignored[0]=pointer(9);
    TestTrue(TEXT("heap fallback copy never aliases retained earlier observation"),
        body.Ignored[0]==pointer(0)&&heapCopy.Owners[2].Bodies[4].Ignored.GetData()!=body.Ignored.GetData());
    auto moved=MoveTemp(heapCopy);heapCopy={};inlineCopy={};
    TestTrue(TEXT("move and release of neighboring observations preserve owned values"),
        moved.Owners.Num()==3&&moved.Owners[2].Bodies.Num()==5&&
        moved.Owners[2].Bodies[4].Ignored.Num()==9&&moved.Owners[2].Bodies[4].Ignored[0]==pointer(9)&&body.Ignored[0]==pointer(0));
    // Lexically nested/reentrant value capture uses a different destination.
    {auto nested=before;nested.Owners[2].Bodies[4].Ignored.Reset();nested.Captured=true;}
    TestTrue(TEXT("nested destruction leaves earlier storage and capture state unchanged"),
        !before.Captured&&body.Ignored.Num()==9&&body.State.Location==FVector(1,2,3));
    before={};
    TestTrue(TEXT("destination reset retains no previous identity or captured value"),before.Owners.IsEmpty()&&!before.Captured);
    return !HasAnyErrors();
}
#endif