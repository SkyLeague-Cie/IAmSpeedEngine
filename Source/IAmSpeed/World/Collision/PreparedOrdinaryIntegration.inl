namespace OrdinaryIntegrationStateCapture
{
TAutoConsoleVariable<int32> CompactOwnerRows(TEXT("p.IAmSpeed.AnalyticWorld.CompactIntegrationOwnerState"),0,
    TEXT("Default OFF. Fresh positive integration row snapshots retain the active owner and General Boxes; all live checks and getters run."));
#if !UE_BUILD_SHIPPING
TAutoConsoleVariable<int32> RecycleOwnerStateBuffers(TEXT("p.IAmSpeed.AnalyticWorld.RecycleIntegrationOwnerStateBuffers"),0,
    TEXT("Default OFF. Reuse the immediately preceding closed owner's two private value buffers in the current integration transaction."));
#endif
}
// Value observations ONLY. The real RequireAdvanceEvidence stays before all
// these hooks and remains closed. No consumed sweep request is revived.
void FStepScope::RequireOrdinaryIntegrationContext(bool allowPost) const
{
    ::PreparedFrameworkProfile1::FScope observedFramework(::PreparedFrameworkProfile1::IntegrationContext);
    FStepIdentity now;
    if(!Active||StepScope!=this||!Window||Window->GetPurpose()!=OnePausedWork::EPurpose::FullFrame||
        !OrdinaryIntegration||OrdinaryIntegration->Closed||(OrdinaryResponse&&!OrdinaryResponse->Closed)||
        SweepPhase||ComponentSweepScope||MemberSweepScope||WheelProbeScope||!ReadCurrentStep(now)||
        OrdinaryIntegration->Snapshot!=Snapshot||!OrdinaryWorldDetail::Epoch(OrdinaryIntegration->EntryStep,now)||
        OrdinaryIntegration->EntryStep.QuerySerial!=now.QuerySerial||!Snapshot||!Snapshot->GetNativeOwners()||
        !FCanonicalFramePreflight::HasUnchangedRawPreparationCensus(*Identity.Bridge,*Snapshot,Identity.StepSerial))
        Refuse("Ordinary integration observation lost current native context");
    if(OwnerProbePhase)
    {
        // Child-origin observations only, inside this exact current owner's
        // Post or internal integration projection. The shared state reader
        // never requests this route and gains no permission from this join.
        const auto& tx=*OrdinaryIntegration;
        if(!allowPost||!tx.Owners.IsValidIndex(tx.ActiveIndex)||!OwnerProbePhase->Check()||!OwnerProbePhase->State||
            OwnerProbePhase->State->Returned||
            (OwnerProbePhase->State->Phase!=ENativeOwnerProbePhase::PostIntegrate&&
             OwnerProbePhase->State->Phase!=ENativeOwnerProbePhase::IntegrationProjection)||
            OwnerProbePhase->State->Owner!=tx.Owners[tx.ActiveIndex].Owner||OwnerProbePhase->State->Snapshot!=Snapshot)
            Refuse("Ordinary integration observation overlaps an unrelated native probe phase");
    }
}
bool FStepScope::HasOrdinaryIntegrationObservation() const
{
    if(OrdinaryIntegration){RequireOrdinaryIntegrationContext();return true;}
    if(!Active||!Window||Window->GetPurpose()!=OnePausedWork::EPurpose::FullFrame)return false;
    Refuse("Ordinary integration hook has no opened actual call");
}
void FStepScope::BeginOrdinaryIntegration(float seconds,bool hasSelection,TConstArrayView<SComponentTOI> selected,
    float horizon,float actualBefore)
{
    if(!Active||!Window||Window->GetPurpose()!=OnePausedWork::EPurpose::FullFrame)
    {if(OrdinaryIntegration&&!OrdinaryIntegration->Closed)Refuse("Ordinary integration left FullFrame");return;}
    FStepIdentity now;
    if(StepScope!=this||SweepPhase||!ReadCurrentStep(now)||(OrdinaryIntegration&&(!OrdinaryIntegration->Closed||
        OrdinaryIntegration->EntryStep.QueryEpoch>=now.QueryEpoch))||(OrdinaryResponse&&!OrdinaryResponse->Closed)||
        !FMath::IsFinite(seconds)||seconds<0||!FMath::IsFinite(actualBefore)||actualBefore<0||
        actualBefore>now.Delta||seconds>now.Delta-actualBefore)
        Refuse("Ordinary integration actual call/clock duplicated or unavailable");
    ConsumeNativeAdvanceTicket(seconds,hasSelection,selected,horizon,actualBefore);
    bool joined=false;
    if(hasSelection)
    {
        if(!OrdinaryReturns||!OrdinaryReturns->World||!OrdinaryReturns->World->Closed)Refuse("Integration lacks actual selected values");
        const auto& source=*OrdinaryReturns->World;auto revoked=source.Completion;
        if(revoked.QueryEpoch==MAX_uint64)Refuse("Integration selector epoch exhausted");++revoked.QueryEpoch;
        if(!OrdinaryWorldDetail::Epoch(revoked,now)||revoked.QuerySerial!=now.QuerySerial||
            !SameFloat(seconds,source.Seconds)||!SameFloat(horizon,source.Horizon)||!SameFloat(actualBefore,source.Before)||
            selected.Num()!=source.Sorted.Num())Refuse("Integration actual selection/clock changed");
        for(int32 i=0;i<selected.Num();++i)if(!OrdinaryWorldDetail::Entry(selected[i],source.Sorted[i],source.Sorted[i].TOI))
            Refuse("Integration actual selected batch retimed or reordered");
        joined=true;
    }
    OrdinaryIntegration=MakeUnique<FOrdinaryIntegrationState>();auto& tx=*OrdinaryIntegration;
    tx.EntryStep=now;tx.Snapshot=Snapshot;tx.Seconds=seconds;tx.HasActualSelection=hasSelection;tx.SelectorJoined=joined;
    tx.Horizon=horizon;tx.ActualBefore=actualBefore;tx.Selected.Append(selected.GetData(),selected.Num());
#if !UE_BUILD_SHIPPING
    tx.RecycleOwnerStateBuffers=OrdinaryIntegrationStateCapture::RecycleOwnerStateBuffers.GetValueOnAnyThread()==1;
#endif
    // Completion/max-iteration calls without actual selection keep joined=false;
    // neither a selector, a clear interval nor an advance grant is invented.
    ReadOrdinaryFrameStateValues(tx.Before);
    if(tx.RecycleOwnerStateBuffers)tx.Owners.Reserve(tx.Before.Owners.Num());
}
void FStepScope::BeginOrdinaryIntegrationOwner(const ISpeedComponent* owner,int32 index)
{
    if(!HasOrdinaryIntegrationObservation())return;auto& tx=*OrdinaryIntegration;const auto& ordered=Registry->GetOrderedAdapters();
    if(tx.Restoring||tx.ActiveIndex!=INDEX_NONE||index!=tx.NextIndex||tx.Owners.Num()!=index||
        !ordered.IsValidIndex(index)||ordered[index]!=owner)Refuse("Ordinary integrate owner reordered or duplicated");
    auto& row=tx.Owners.AddDefaulted_GetRef();row.Owner=owner;row.Index=index;row.Opened=true;tx.ActiveIndex=index;
    // Previous owner closure already consumed both captures. Later response and
    // final operations retain its ledger/proofs and use tx.After as the baseline.
    if(tx.RecycleOwnerStateBuffers&&index>0)
    {
        auto& previous=tx.Owners[index-1];
        if(!previous.Closed||!previous.Returned||!previous.Before.Captured||!previous.After.Captured)
            Refuse("Integration owner buffers have no closed producer");
        row.Before.Owners=MoveTemp(previous.Before.Owners);
        row.After.Owners=MoveTemp(previous.After.Owners);
        previous.Before={};previous.After={};
        row.RecycledStateBuffers=true;
    }
    row.CompactStateCapture=owner&&tx.Seconds>0.f&&
        OrdinaryIntegrationStateCapture::CompactOwnerRows.GetValueOnAnyThread()==1;
    ReadOrdinaryFrameStateValues(row.Before);
    if(NativeAdvance&&NativeAdvance->Consumed)for(const auto& binding:NativeAdvance->GeneralBindings)
        if(binding.Request->GetValues().Owner==owner)row.GeneralBindings.Add(binding);
}
void FStepScope::EndOrdinaryIntegrationOwner(const ISpeedComponent* owner,int32 index,bool called)
{
    if(!OrdinaryIntegration)
    {if(!Active||!Window||Window->GetPurpose()!=OnePausedWork::EPurpose::FullFrame)return;Refuse("Integration owner return lacks actual observation");}
    auto& tx=*OrdinaryIntegration;
    if(tx.ActiveIndex!=index||!tx.Owners.IsValidIndex(index)||tx.Owners[index].Owner!=owner||
        tx.Owners[index].Returned||called!=(owner!=nullptr))Refuse("Integration owner return repeated or different");
    auto& row=tx.Owners[index];row.Called=called;row.Returned=true; // actual return survives a later refusal
    RequireOrdinaryIntegrationContext();if(tx.Restoring||row.RestorationAttempts!=row.RestorationReturns)
        Refuse("Native reaction restoration did not return before owner integration");
    // This read is OUTSIDE the native call: Post scope has been destroyed and
    // ON_SCOPE_EXIT has restored temporary loads. No equality with entry is assumed.
    ReadOrdinaryFrameStateValues(row.After);
    FStepIdentity actual;if(!ReadCurrentStep(actual))Refuse("Primitive value closure lost current Step");
    OrdinaryPrimitiveDetail::CloseValues(row,tx.Seconds,actual);
    RequireOrdinaryIntegrationContext();FStepIdentity after;
    if(!ReadCurrentStep(after)||!OrdinaryWorldDetail::Epoch(actual,after)||actual.QuerySerial!=after.QuerySerial)
        Refuse("Primitive value closure changed current Step");
    bool exactNoMotion=false;
    if(!row.GeneralBindings.IsEmpty())
    {
        GeneralBoxTransportDetail::ClosedSetters(row,true);
        // The nonvirtual native function returns before all effects at exact +/-0.
        // Do not invent a transport primitive or evaluate Integrate(0) for this call.
        exactNoMotion=OrdinarySetFutureDetail::ExactZeroDelta(tx.Seconds)&&
            row.Paths.Num()==1&&row.Paths[0].Path==ENativeIntegrationPath::NonpositiveDelta&&
            SameFloat(row.Paths[0].Delta,tx.Seconds)&&row.PrimitiveStatus==EOrdinaryPrimitiveValuesStatus::OutsideTransport&&
            NativeAdvance&&NativeAdvance->Consumed&&tx.SelectorJoined&&row.Stages.IsEmpty()&&row.ChildSync.IsEmpty()&&
            row.RestorationAttempts==0&&row.RestorationReturns==0&&row.PrimitiveComparisons.IsEmpty()&&
            !row.GeneralOpen&&!row.GeneralReturned&&row.GeneralCount==0&&row.GeneralNext==0&&row.GeneralSyncs==0&&
            row.GeneralProofs.IsEmpty()&&OrdinarySetFutureDetail::Unchanged(row.Before,row.After);
        if(!exactNoMotion)
        {
        if(row.GeneralOpen||row.GeneralCount<1||row.GeneralNext!=row.GeneralCount||
            row.PrimitiveStatus!=EOrdinaryPrimitiveValuesStatus::ValuesMatch||row.Paths.Num()!=1||
            row.Paths[0].Path!=ENativeIntegrationPath::Transport)
        {
            uint32 secondsBits=0;FMemory::Memcpy(&secondsBits,&tx.Seconds,sizeof(secondsBits));
            Refuse(TCHAR_TO_UTF8(*FString::Printf(TEXT("General Box actual primitive/child/projection closure incomplete: secondsBits=%08x path=%d status=%d count=%d next=%d open=%d stages=%d syncs=%d"),
                secondsBits,row.Paths.Num()==1?int32(row.Paths[0].Path):-1,int32(row.PrimitiveStatus),row.GeneralCount,row.GeneralNext,int32(row.GeneralOpen),row.Stages.Num(),row.ChildSync.Num())));
        }
        const auto* actualOwner=GeneralBoxTransportDetail::Find(row.After,owner);
        if(!actualOwner||!GeneralBoxTransportDetail::Pose(actualOwner->State,row.GeneralExpected))Refuse("General Box owner return changed its covered endpoint");
        for(int32 i=0;i<row.GeneralBindings.Num();++i)
            if(!GeneralBoxTransportDetail::Endpoint(Snapshot,row,i))
                Refuse("General Box owner return has an uncovered actual endpoint");
        }
    }
    RequireGeneralBoxUnchanged(row.Before,row.After,exactNoMotion?nullptr:owner);
    row.Closed=true;tx.ActiveIndex=INDEX_NONE;++tx.NextIndex;
}
void FStepScope::EndOrdinaryIntegration(float actualAfter)
{
    if(!HasOrdinaryIntegrationObservation())return;auto& tx=*OrdinaryIntegration;
    if(tx.Restoring||tx.ActiveIndex!=INDEX_NONE||tx.NextIndex!=Registry->GetOrderedAdapters().Num()||
        tx.Owners.Num()!=tx.NextIndex||!FMath::IsFinite(actualAfter)||
        !SameFloat(actualAfter,tx.ActualBefore+tx.Seconds)||actualAfter>Identity.Delta)
        Refuse("Ordinary actual integration owner inventory or post-clock incomplete");
    for(const auto& row:tx.Owners)if(!row.Closed||!row.Returned)Refuse("Ordinary integration call remained open");
    tx.ActualAfter=actualAfter;ReadOrdinaryFrameStateValues(tx.After);tx.Closed=true;
}
void FStepScope::ObserveOrdinaryIntegrationPath(const ISpeedComponent& owner,const FNativeIntegrationPathValues& path)
{
    if(!Requested()||!StepScope||!StepScope->OrdinaryIntegration||StepScope->OrdinaryIntegration->ActiveIndex==INDEX_NONE)return;
    auto* scope=StepScope;scope->RequireOrdinaryIntegrationContext();auto& tx=*scope->OrdinaryIntegration;
    if(!tx.Owners.IsValidIndex(tx.ActiveIndex)||tx.Owners[tx.ActiveIndex].Owner!=&owner)
        Refuse("Native integration branch belongs to another actual owner");
    tx.Owners[tx.ActiveIndex].Paths.Add(path); // no IsSleeping/support/transport predicate replay
}
void FStepScope::ObserveOrdinaryIntegrationStage(const ISpeedComponent& owner,ENativeIntegrationStage stage,
    float delta,int32 index,int32 count,bool projected)
{
    if(!Requested()||!StepScope||!StepScope->OrdinaryIntegration||StepScope->OrdinaryIntegration->ActiveIndex==INDEX_NONE)return;
    auto* scope=StepScope;scope->RequireOrdinaryIntegrationContext();auto& tx=*scope->OrdinaryIntegration;
    if(!tx.Owners.IsValidIndex(tx.ActiveIndex)||tx.Owners[tx.ActiveIndex].Owner!=&owner)
        Refuse("Native integration stage owner changed");
    auto& v=tx.Owners[tx.ActiveIndex].Stages.AddDefaulted_GetRef();v.Stage=stage;v.Delta=delta;v.Index=index;v.Count=count;
    v.Projected=projected;v.OwnerState=owner.GetKinematicState();if(!ReadCurrentStep(v.Step))Refuse("Native integration stage epoch changed");
    scope->RequireOrdinaryIntegrationContext();v.Captured=true;scope->ObserveGeneralBoxStage(tx.Owners[tx.ActiveIndex],v);
}
void FStepScope::ObserveOrdinaryIntegrationChildSync(const ISpeedComponent& owner,const FKinematicState& origin) noexcept
{
    try{if(FNativePlanarResponseCommitScope::AfterChildSync(owner,origin)||
        FNativeFinalWheelSupportScope::AfterChildSync(owner,origin))return;}
    catch(const std::exception& error){RecordPreparedPoison("planar_actual_child_sync_failed",error.what());return;}
    catch(...){RecordPreparedPoison("planar_actual_child_sync_failed","non_std_exception");return;}
    // Final Post and response can tentatively move then roll back. Observe
    // their FIRST real sync too; endpoint equality after rollback is not a
    // proof of that excursion. Copy/check only, no geometry or new actor.
    auto* observed=StepScope;
    if(Requested()&&observed&&observed->GeneralBoxOwners.Contains(&owner)&&
        (!observed->OrdinaryIntegration||observed->OrdinaryIntegration->ActiveIndex==INDEX_NONE))
    {
        try
        {
            const bool final=observed->OrdinaryFinal&&!observed->OrdinaryFinal->Closed&&observed->OrdinaryFinal->CallOpen;
            const bool response=observed->OrdinaryResponse&&!observed->OrdinaryResponse->Closed;
            if(int32(final)+int32(response)!=1||!observed->OrdinaryIntegration||!observed->OrdinaryIntegration->Closed)
                Refuse("General Box synchronized outside its unique covered final/response context");
            const auto validate=[&](const auto* covered)
            {
                if(!covered||!GeneralBoxTransportDetail::Pose(covered->State,owner.GetKinematicState())||
                    !SameVector(covered->LocalCOM,owner.GetPhysCenterOfMassLocal())||owner.GetSubBodies().Num()!=covered->Bodies.Num())
                    Refuse("General Box final/response tentative pose escaped its covered endpoint");
                for(const auto& child:covered->Bodies)if(child.BoxShapeCaptured)
                {
                    if(!owner.GetSubBodies().IsValidIndex(child.Index)||owner.GetSubBodies()[child.Index]!=child.Body||!child.Body||
                        !GeneralBoxTransportDetail::Pose(child.State,child.Body->GetKinematicState())||
                        !SameVector(child.LocalOffset,child.Body->GetLocalOffset())||!SameQuat(child.LocalRotation,child.Body->GetLocalRotation()))
                        Refuse("General Box final/response tentative child sync changed covered pose/configuration");
                }
            };
            if(final)
                if(const auto* baseline=CurrentFinalWheelSupportBaseline(*observed->OrdinaryFinal))
                {
                    validate(baseline->Owners.FindByPredicate([&](const auto& item){return item.Owner==&owner;}));
                    return;
                }
            validate(GeneralBoxTransportDetail::Find(observed->GeneralBoxCurrentClosedBaseline(false),&owner));
        }
        catch(const std::exception& error){RecordPreparedPoison("PreparedOrdinaryIntegration.inl:181",error.what());}catch(...){RecordPreparedPoison("PreparedOrdinaryIntegration.inl:181","non_std_exception");}
    }
    if(!Requested()||!StepScope||!StepScope->OrdinaryIntegration||StepScope->OrdinaryIntegration->ActiveIndex==INDEX_NONE)return;
    auto* scope=StepScope;auto& tx=*scope->OrdinaryIntegration;
    const auto capture=[&]()
    {
        scope->RequireOrdinaryIntegrationContext(true);
        if(!tx.Owners.IsValidIndex(tx.ActiveIndex)||tx.Owners[tx.ActiveIndex].Owner!=&owner)Refuse("Native child-sync owner changed");
        const auto* row=scope->Snapshot->GetNativeOwners()->Find(&owner);
        if(!row||!row->BodyCensusCaptured||owner.GetSubBodies().Num()!=row->Bodies.Num())Refuse("Native child-sync GT census changed");
        auto& v=tx.Owners[tx.ActiveIndex].ChildSync.AddDefaulted_GetRef();v.ActualOrigin=origin;v.InExactPostProbe=OwnerProbePhase&&OwnerProbePhase->State->Phase==ENativeOwnerProbePhase::PostIntegrate;
        v.InExactIntegrationProjection=OwnerProbePhase&&OwnerProbePhase->State->Phase==ENativeOwnerProbePhase::IntegrationProjection;
        v.OwnerState=owner.GetKinematicState();if(!ReadCurrentStep(v.Step))Refuse("Native child-sync Step changed");
        for(int32 i=0;i<row->Bodies.Num();++i)
        {
            const auto& child=row->Bodies[i];if(!child.Body||child.Index!=i||owner.GetSubBodies()[i]!=child.Body||
                child.Body->GetClass()!=child.Class||child.Body->GetParentComponent()!=&owner)Refuse("Native child-sync association changed");
            v.Children.Add(child.Body->GetKinematicState());
        }
        scope->RequireOrdinaryIntegrationContext(true);v.Captured=true;scope->ObserveGeneralBoxChildSync(tx.Owners[tx.ActiveIndex]);
    };
    // Native restoration is a scope-exit cleanup. Never throw an observation
    // refusal from it or replace its subtraction/sync by early termination.
    try{capture();}catch(const std::exception& error){RecordPreparedPoison("PreparedOrdinaryIntegration.inl:204",error.what());}catch(...){RecordPreparedPoison("PreparedOrdinaryIntegration.inl:204","non_std_exception");}
}
void FStepScope::BeginOrdinaryIntegrationRestoration(const ISpeedComponent& owner) noexcept
{
    auto* scope=StepScope;if(!scope||!scope->OrdinaryIntegration||scope->OrdinaryIntegration->ActiveIndex==INDEX_NONE)return;
    auto& tx=*scope->OrdinaryIntegration;
    try
    {
        if(!tx.Owners.IsValidIndex(tx.ActiveIndex)||tx.Owners[tx.ActiveIndex].Owner!=&owner||tx.Restoring){RecordPreparedPoison("PreparedOrdinaryIntegration.inl:212");return;}
        tx.Restoring=true;++tx.Owners[tx.ActiveIndex].RestorationAttempts;scope->RequireOrdinaryIntegrationContext();
    }catch(const std::exception& error){RecordPreparedPoison("PreparedOrdinaryIntegration.inl:214",error.what());}catch(...){RecordPreparedPoison("PreparedOrdinaryIntegration.inl:214","non_std_exception");}
}
void FStepScope::EndOrdinaryIntegrationRestoration(const ISpeedComponent& owner,float delta) noexcept
{
    auto* scope=StepScope;if(!scope||!scope->OrdinaryIntegration||scope->OrdinaryIntegration->ActiveIndex==INDEX_NONE)return;
    auto& tx=*scope->OrdinaryIntegration;
    try
    {
        if(!tx.Owners.IsValidIndex(tx.ActiveIndex)||tx.Owners[tx.ActiveIndex].Owner!=&owner||!tx.Restoring){RecordPreparedPoison("PreparedOrdinaryIntegration.inl:222");return;}
        ++tx.Owners[tx.ActiveIndex].RestorationReturns; // true native cleanup return, not a verified effect
        ObserveOrdinaryIntegrationStage(owner,ENativeIntegrationStage::ReactionRestored,delta);
    }catch(const std::exception& error){RecordPreparedPoison("PreparedOrdinaryIntegration.inl:225",error.what());}catch(...){RecordPreparedPoison("PreparedOrdinaryIntegration.inl:225","non_std_exception");}
    tx.Restoring=false;
}
