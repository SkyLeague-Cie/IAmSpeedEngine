// Included in the native execution TU. No hit, integration, support or response2.
bool FStepScope::IsPostSyncRsPurpose() const
{
    return Active&&StepScope==this&&Window&&Window->GetPurpose()==OnePausedWork::EPurpose::TerminalPostSyncRsCaptureStop;
}
bool FStepScope::HasPostSyncRsIdentity() const
{
    if(!IsPostSyncRsPurpose()||!Window->IsOpen()||Poisoned||!HostScope||!IsCurrentWorker(Identity.Worker)||
        Identity.ThreadId!=FPlatformTLS::GetCurrentThreadId()||
        !PreparationPrefix::FCursor::IsCurrentPostSyncRsNativeWindow(*Identity.Bridge)||
        !RsCapture||!RsCapture->IsClosed()||!RsCapture->HasBoundedFirstArrivalRsQuality())return false;
    OnePausedWork::FCanonicalFrameWindowPtr current;
    if(!OnePausedWork::ReadExecutionWindow(current)||current!=Window||current->GetSerial()!=Identity.WindowSerial||
        current->GetNonce()!=Identity.WindowNonce||current->GetSnapshot()!=Snapshot||
        Identity.Provider->GetPreparedSnapshot()!=Snapshot||Identity.Host->GetInputWorkerGeneration()!=Identity.InputGeneration||
        Identity.Host->GetWorld()!=Identity.World||Identity.Bridge->GetWorld()!=Identity.World||
        Identity.Bridge->GetStaticCollisionWorld()!=Identity.Provider||Registry->GetStaticCollisionWorld()!=Identity.Provider||
        !FCanonicalFramePreflight::HasUnchangedRawPreparationCensus(*Identity.Bridge,*Snapshot,Identity.StepSerial))return false;
    const auto& contract=Window->GetPreparationContract();
    if(!contract||!contract->HasCompletedNativeRsResponseSyncPrefix())return false;
    const auto& data=contract->FirstRsResponseData;const auto& sync=contract->FirstRsResponseSync;
    if(!data||data!=sync.Data||!data->HasSynchronizationExpectation||!data->Origin||
        data->Origin!=RsCapture->Request||data->AdvanceSource!=RsCapture->AdvanceSource||
        data->Values.Quality!=RsCapture->RsQuality||!contract->FirstRsResponse.Serial||
        sync.ResponseSerial!=contract->FirstRsResponse.Serial||sync.NumericControl!=Identity.NumericControl)return false;
    const auto& origin=data->Origin->GetStep();
    const auto fp=Analytic::InspectCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(origin.Lease!=this||origin.Host!=Identity.Host||origin.Bridge!=Identity.Bridge||origin.Worker!=Identity.Worker||
        origin.World!=Identity.World||origin.Provider!=Identity.Provider||origin.InputGeneration!=Identity.InputGeneration||
        origin.Frame!=Identity.Frame||origin.StepSerial!=Identity.StepSerial||origin.WindowSerial!=Identity.WindowSerial||
        origin.WindowNonce!=Identity.WindowNonce||origin.QuerySerial!=1||origin.QueryEpoch!=2||
        data->NumericControl!=Identity.NumericControl||fp.Status!=Analytic::EAnalyticNumericStatus::Complete||
        fp.ControlFingerprint()!=Identity.NumericControl)return false;
    if(!PostSyncRs)return !SweepPhase&&Identity.Iteration==1&&Identity.QueryEpoch==3&&Identity.QuerySerial==1;
    const auto& post=*PostSyncRs;
    if(!post.Started||post.Data!=data||post.ResponseSerial!=sync.ResponseSerial||
        !SameFloat(post.TimePassed,data->ActualTimePassed)||!SameFloat(post.LastSubDelta,data->Seconds)||
        !SameFloat(post.RawRemaining,Identity.Delta-post.TimePassed))return false;
    if(Identity.Iteration==1)return !SweepPhase&&Identity.QueryEpoch==3&&Identity.QuerySerial==1;
    if(Identity.Iteration!=2||!SameFloat(Identity.TimePassed,post.TimePassed)||!SameFloat(Identity.Remaining,post.Horizon))return false;
    if(SweepPhase)return Identity.QueryEpoch==4&&(Identity.QuerySerial==1||Identity.QuerySerial==2);
    return Identity.QueryEpoch==5&&Identity.QuerySerial==2&&post.Capture&&post.Capture->IsClosed();
}
void FStepScope::CheckPostSyncRsOwners() const
{
    if(!HasPostSyncRsIdentity())Refuse("Post-sync Rs native identity unavailable");
    const auto& data=*Window->GetPreparationContract()->FirstRsResponseData;
    const auto& proof=*data.AdvanceSource;const auto& owners=Registry->GetOrderedAdapters();
    if(owners.Num()!=3||proof.Owners.Num()!=3)Refuse("Post-sync Rs owner census differs");
    for(int32 i=0;i<3;++i)
    {
        const auto& row=proof.Owners[i];const auto* owner=owners[i];const auto* retained=Snapshot->GetNativeOwners()->Find(owner);
        const FKinematicState& expected=i==1?data.Physical.OwnerAfter:row.After;
        if(!owner||owner!=row.Owner||!retained||!retained->BodyCensusCaptured||owner->GetPublishedSimulationStableId()!=row.StableId||
            owner->NumFrame()!=row.Frame||owner->IsFrozen()!=row.Frozen||&owner->GetKinematicState()!=row.Stored||
            &owner->GetKinematicStateForFrame(row.Frame)!=row.Selected||(!row.Frozen&&row.Stored!=row.Selected)||
            !SameVector(owner->GetPhysCenterOfMassLocal(),row.LocalCOM)||!SameFloat(owner->GetPhysMaxSpeed(),row.MaximumSpeed)||
            !SameFloat(owner->GetPhysMaxAngularSpeed(),row.MaximumAngularSpeed)||!owner->GetPhysicalConstraints().IsEmpty()||
            !SameKinematicBits(owner->GetKinematicState(),expected))Refuse("Post-sync Rs owner/history/caps/COM/state changed");
        if(!row.Frozen&&(owner->IsPrescribedMotion()||owner->IsPhysicsSleeping()))Refuse("Post-sync Rs free-owner branch changed");
        const auto& bodies=owner->GetSubBodies();
        if(bodies.Num()!=row.BodiesAfter.Num()||retained->Bodies.Num()!=row.BodiesAfter.Num())Refuse("Post-sync Rs child inventory changed");
        for(int32 j=0;j<bodies.Num();++j)
            if(!bodies[j]||retained->Bodies[j].Index!=j||retained->Bodies[j].Body!=bodies[j]||bodies[j]->GetParentComponent()!=owner||
                !SameKinematicBits(bodies[j]->GetKinematicState(),i==1&&j==0?data.BodyAfterSynchronization:row.BodiesAfter[j]))
                Refuse("Post-sync Rs actual body state changed");
    }
    const auto* binding=Snapshot->GetNativeOwners()->Find(data.Owner);
    if(!binding||binding->NativeObject->GetClass()!=data.OwnerClass||binding->Class!=data.OwnerClass||
        binding->Bodies.Num()!=1||binding->Bodies[0].Body!=data.Body||binding->Bodies[0].Class!=data.BodyClass||
        data.Body->GetClass()!=data.BodyClass||data.Body->GetParentComponent()!=data.Owner||
        data.Owner!=owners[1]||data.Body!=owners[1]->GetSubBodies()[0]||data.Owner->IsFrozen()||
        !SameFloat(data.Body->GetRadius(),data.Physical.Input.PhysicalRadius)||
        !SameFloat(data.Body->GetWorldStaticContactOffsetCm(),data.Physical.Input.StaticContactOffsetCm)||
        !SameFloat(data.Body->GetImpactThreshold(),data.Physical.Input.ImpactThreshold)||
        !SameVector(data.Owner->GetPhysCOM(),data.Physical.Input.ActualCOM))Refuse("Post-sync Rs exact class/radius/physical branch changed");
}
void FStepScope::BeginPostSyncRsPrelude(float rawRemaining,float timePassed,float lastSubDelta)
{
    if(!HasPostSyncRsIdentity()||PostSyncRs||MemberSweepScope||ComponentSweepScope||SweepPhase||
        !PreparationPrefix::FCursor::IsCurrentPostSyncRsPrelude(*Identity.Bridge))Refuse("Post-sync Rs prelude is not one-use after closed sync");
    CheckPostSyncRsOwners();const auto& contract=Window->GetPreparationContract();const auto& data=*contract->FirstRsResponseData;
    if(!SameFloat(rawRemaining,Identity.Delta-timePassed)||!SameFloat(timePassed,data.ActualTimePassed)||
        !SameFloat(lastSubDelta,data.Seconds)||!FMath::IsFinite(rawRemaining)||rawRemaining<=0||
        !SameFloat(Identity.SimTime,data.ActualResponseSimTime))Refuse("Post-sync Rs actual native clocks differ");
    PostSyncRs=TUniquePtr<FPostSyncRsNativeState>(new FPostSyncRsNativeState);
    auto& post=*PostSyncRs;post.Data=contract->FirstRsResponseData;post.ResponseSerial=contract->FirstRsResponse.Serial;
    post.RawRemaining=rawRemaining;post.TimePassed=timePassed;post.LastSubDelta=lastSubDelta;post.Horizon=rawRemaining;post.Started=true;
}
void FStepScope::ObservePostSyncRsSupport(const ISpeedComponent* owner,float before,float actual)
{
    if(!HasPostSyncRsIdentity()||!PostSyncRs||MemberSweepScope||ComponentSweepScope||SweepPhase||
        !PreparationPrefix::FCursor::IsCurrentPostSyncRsPrelude(*Identity.Bridge))Refuse("Post-sync Rs support read escaped prelude");
    auto& post=*PostSyncRs;CheckPostSyncRsOwners();
    if(post.SupportCount>=3||Registry->GetOrderedAdapters().Num()!=3||Registry->GetOrderedAdapters()[post.SupportCount]!=owner||
        !owner||!SameFloat(before,post.Horizon)||!FMath::IsFinite(actual)||actual<=0)
        Refuse("Post-sync Rs actual support return or owner order invalid");
    post.SupportBefore.Add(before);post.SupportAfter.Add(actual);post.Horizon=FMath::Min(before,actual);++post.SupportCount;
}
void FStepScope::BeginPostSyncRsOwner(const ISpeedComponent* owner,float lastSubDelta)
{
    if(!HasPostSyncRsIdentity()||!PostSyncRs||!SweepPhase||MemberSweepScope||ComponentSweepScope)
        Refuse("Post-sync Rs owner outside actual second iteration");
    auto& post=*PostSyncRs;CheckPostSyncRsOwners();
    if(post.SupportCount!=3||post.OwnerActive||post.Capture||post.SweepOwners>=2||Identity.Iteration!=2||
        !SameFloat(lastSubDelta,post.LastSubDelta)||!owner||Registry->GetOrderedAdapters()[post.SweepOwners]!=owner||
        !PreparationPrefix::FCursor::IsCurrentPostSyncRsSweepSite(*owner,post.SweepOwners))Refuse("Post-sync Rs second owner dispatch differs");
    const auto& bodies=owner->GetSubBodies();
    if(bodies.Num()!=(post.SweepOwners==0?0:1)||!owner->GetPhysicalConstraints().IsEmpty())Refuse("Post-sync Rs runner/ball census differs");
    post.Owner=owner;post.OwnerBefore=owner->GetKinematicState();post.BodiesBefore.Reset();
    for(const auto* body:bodies)
    {if(!body||body->GetParentComponent()!=owner)Refuse("Post-sync Rs body parent differs");post.BodiesBefore.Add(body->GetKinematicState());}
    post.ReactionObserved=false;post.OwnerActive=true;
}
void FStepScope::ValidatePostSyncRsReactions(const ISpeedComponent& owner,const FVector& linear,const FVector& angular)
{
    if(!HasPostSyncRsIdentity()||!PostSyncRs)Refuse("Post-sync Rs reactions lack current identity");
    auto& post=*PostSyncRs;
    if(!post.OwnerActive||post.Owner!=&owner||post.ReactionObserved||MemberSweepScope||ComponentSweepScope||
        !PreparationPrefix::FCursor::IsCurrentPostSyncRsSweepSite(owner,post.SweepOwners)||
        !SameVector(linear,FVector::ZeroVector)||!SameVector(angular,FVector::ZeroVector)||
        !SameKinematicBits(owner.GetKinematicState(),post.OwnerBefore))Refuse("Post-sync Rs rejects actual reactions before native setters");
    CheckPostSyncRsOwners();post.LinearReaction=linear;post.AngularReaction=angular;post.ReactionObserved=true;
}
void FStepScope::CompletePostSyncRsRunner(const ISpeedComponent* owner,const SComponentTOI& actual)
{
    if(!HasPostSyncRsIdentity()||!PostSyncRs)Refuse("Post-sync Rs runner identity unavailable");
    auto& post=*PostSyncRs;
    if(!post.OwnerActive||post.Owner!=owner||post.SweepOwners!=0||!post.ReactionObserved||post.Capture||
        MemberSweepScope||ComponentSweepScope||!actual.PreparedEvidence||!actual.PreparedEvidence->IsSealedInventory()||
        actual.PreparedEvidence->Owner!=owner||!actual.PreparedEvidence->Bodies.IsEmpty()||actual.bHit||
        !SameFloat(actual.TOI,Identity.Remaining)||!owner->GetSubBodies().IsEmpty()||
        !SameKinematicBits(post.OwnerBefore,owner->GetKinematicState()))Refuse("Post-sync Rs runner did not return its real complete zero-body inventory");
    const auto& evidence=*actual.PreparedEvidence;const auto& step=evidence.Step;
    if(actual.PreparedEvidence==RsRunnerEvidence||!evidence.ScopeSerial||evidence.OwnerStableId!=Registry->FindStableId(*owner)||
        step.Lease!=this||step.Host!=Identity.Host||step.Bridge!=Identity.Bridge||step.World!=Identity.World||
        step.Worker!=Identity.Worker||step.Provider!=Identity.Provider||step.InputGeneration!=Identity.InputGeneration||
        step.Frame!=Identity.Frame||step.StepSerial!=Identity.StepSerial||step.WindowSerial!=Identity.WindowSerial||
        step.WindowNonce!=Identity.WindowNonce||step.NumericControl!=Identity.NumericControl||step.ThreadId!=Identity.ThreadId||
        step.Iteration!=2||step.QueryEpoch!=4||step.QuerySerial!=1||!SameFloat(step.Remaining,Identity.Remaining)||
        !SameFloat(step.TimePassed,post.TimePassed)||!SameFloat(step.Delta,Identity.Delta)||!SameFloat(step.SimTime,Identity.SimTime)||
        !SameKinematicBits(evidence.SweepOwnerState,post.OwnerBefore))Refuse("Post-sync Rs runner evidence is stale or from slot1");
    post.RunnerEvidence=actual.PreparedEvidence;post.OwnerActive=false;post.Owner=nullptr;++post.SweepOwners;
}
bool FStepScope::ReadPostSyncRsObservation(FPostSyncRsStepObservation& out)
{
    out={};const auto* s=StepScope;FStepIdentity now;
    if(!s||!s->PostSyncRs||!ReadCurrentStep(now)||!s->HasPostSyncRsIdentity()||
        !PreparationPrefix::FCursor::IsCurrentPostSyncRsObservation())return false;
    const auto& post=*s->PostSyncRs;out.Step=now;out.ResponseSerial=post.ResponseSerial;
    out.RawRemaining=post.RawRemaining;out.TimePassed=post.TimePassed;out.LastSubDelta=post.LastSubDelta;out.Horizon=post.Horizon;
    out.SupportBefore=post.SupportBefore;out.SupportAfter=post.SupportAfter;out.SupportCount=post.SupportCount;out.SweepOwners=post.SweepOwners;
    out.OwnerActive=post.OwnerActive;out.ReactionObserved=post.ReactionObserved;out.MemberPresent=MemberSweepScope!=nullptr;
    out.ComponentPresent=ComponentSweepScope!=nullptr;out.SweepPhase=s->SweepPhase;out.CaptureClosed=post.Capture&&post.Capture->IsClosed();
    out.RunnerInventorySealed=post.RunnerEvidence&&post.RunnerEvidence->IsSealedInventory();out.LinearReaction=post.LinearReaction;out.AngularReaction=post.AngularReaction;
    const auto& diagnostic=now.Bridge->GetLastStepDiagnostics();out.DiagnosticIteration=diagnostic.IterationCount;out.DiagnosticComponentSweeps=diagnostic.ComponentSweepCount;return true;
}
bool FSphereMemberSweepScope::IsPostSyncRsCaptureRequested()
{
    auto* s=StepScope;
    if(!s||!s->IsPostSyncRsPurpose()||!s->PostSyncRs||!s->PostSyncRs->Started)return false;
    const auto& post=*s->PostSyncRs;FStepIdentity now;
    if(!ReadCurrentStep(now)||!s->HasPostSyncRsIdentity()||!post.OwnerActive||post.SweepOwners!=1||
        !post.ReactionObserved||post.SupportCount!=3||post.Capture||now.Iteration!=2||now.QueryEpoch!=4||
        now.QuerySerial!=1||!SameFloat(now.TimePassed,post.TimePassed)||!SameFloat(now.Remaining,post.Horizon)||
        !PreparationPrefix::FCursor::IsCurrentPostSyncRsSweepSite(*post.Owner,1))
        Refuse("Post-sync Rs member requested outside its one actual second-query site");
    return true;
}
[[noreturn]] void FSphereMemberSweepScope::CapturePostSyncRsAndStop(FNativeSphereRequestPtr request)
{
    auto* s=StepScope;auto* member=MemberSweepScope;auto* component=ComponentSweepScope;
    if(!s||!s->HasPostSyncRsIdentity()||!s->PostSyncRs||!member||!component||!member->Active||
        member->Completed||member->Interrupted||!member->Trace||!component->Active||component->Completed||component->Interrupted||
        !request||!RevalidateSphereCCD(*request))Refuse("Post-sync Rs lacks a live native member/component/request");
    auto& post=*s->PostSyncRs;const auto& v=request->GetValues();
    if(post.Capture||member->PendingGround||!post.OwnerActive||post.Owner!=v.Owner||member->Body!=v.Body||
        v.BodyIndex!=0||post.SweepOwners!=1||!post.ReactionObserved||component->CurrentIndex!=0||component->NextIndex!=0||
        component->Pending->Bodies.Num()!=1||component->Pending->Bodies[0].Returned||component->Pending->Bodies[0].Ground||
        component->Pending->Owner!=post.Owner||request->GetStep().Iteration!=2||request->GetStep().QuerySerial!=2||
        request->GetStep().QueryEpoch!=4||!component->Pending->ScopeSerial)Refuse("Post-sync Rs capture is not the fresh query2 inventory");
    s->CheckPostSyncRsOwners();FPostSyncRsMathResult math;
    const auto diagnosticContract=s->Window->GetPreparationContract();
    bool mathMovedIntoCapture=false;
    ON_SCOPE_EXIT
    {
        // Every refusal/exception after the actual callback retains its sole
        // raw buffer by move. This is not a completed native capture.
        if(!mathMovedIntoCapture&&diagnosticContract&&diagnosticContract->PostSyncRsMathAttempted)
        {
            diagnosticContract->PostSyncRsUnqualifiedMath=MoveTemp(math);
            diagnosticContract->PostSyncRsUnqualifiedMathPresent=true;
        }
    };
    if(!PreparationPrefix::FCursor::EvaluateCapturedPostSyncRs(request,*member->Trace,math)||!RevalidateSphereCCD(*request))
        Refuse("Post-sync Rs callback or actual request revalidation refused");
    if(math.BudgetBefore<math.BudgetAfter||math.BudgetAfter<0||math.Attempts<0||math.ClosureReserve<0||math.BudgetAfter<math.ClosureReserve)
        Refuse("Post-sync Rs shared math budget counters differ");
    for(const auto& part:{TPair<bool,Analytic::Certified::FReferenceCandidateSetPtr>(math.FullHorizonRsAttempted,math.FullHorizonRsPartition),
                          TPair<bool,Analytic::Certified::FReferenceCandidateSetPtr>(math.FullHorizonRcAttempted,math.FullHorizonRcPartition)})
        if(part.Value&&(!part.Key||part.Value->GetScene()!=request->GetGeometrySelection()->GetSelectedUnion()))
            Refuse("Post-sync full-H partition changed selected geometry or attempt identity");
    if((math.FullHorizonDynamicSource&&(!math.DynamicDomainAttempted||math.FullHorizonDynamicBodies<=0))||
        (!math.FullHorizonDynamicSource&&math.FullHorizonDynamicBodies))
        Refuse("Post-sync dynamic source/count does not join its actual attempt");
    if(math.Initial)
    {
        const auto& input=math.Initial->GetCapturedInput();
        if(!SameKinematicBits(input.State,v.BodyState)||!SameFloat(input.PhysicalRadius,v.PhysicalRadius)||
            !SameFloat(input.StaticContactOffset,v.StaticContactOffsetCm)||!SameFloat(input.ActualMargin,v.ResponseMargin)||
            !SameDouble(input.SupportRadius,v.StaticSupportRadius)||!SameFloat(input.QueryRadius,v.CollisionRadius)||
            !SameVector(math.Initial->GetLocalOffset(),v.LocalOffset)||!SameVector(math.Initial->GetLocalCOM(),v.LocalCOM)||
            !SameFloat(math.Initial->GetHorizon(),v.Horizon)||!SameFloat(math.Initial->GetSeconds(),0.f)||
            math.Initial->GetGeometry()!=request->GetGeometrySelection())Refuse("Post-sync Rs strict point changed actual captured input");
    }
    if(math.TemporalPrefix&&(!math.Initial||math.TemporalPrefix->GetInitial()!=math.Initial||
        !SameFloat(math.TemporalPrefix->GetHorizon(),v.Horizon)||!math.TemporalPrefix->HasWholeRoundedPrefixProof()||
        !math.TemporalPrefix->GetRsCandidates()||!math.TemporalPrefix->GetRcPartition()||
        math.TemporalPrefix->GetRsCandidates()->GetScene()!=request->GetGeometrySelection()->GetSelectedUnion()||
        math.TemporalPrefix->GetRcPartition()->GetScene()!=request->GetGeometrySelection()->GetSelectedUnion()))
        Refuse("Post-sync Rs temporal result changed geometry/initial/horizon");
    if(math.FirstArrival&&(!math.Initial||math.FirstArrival->GetInitial()!=math.Initial||
        !SameFloat(math.FirstArrival->GetHorizon(),v.Horizon)||!math.FirstArrival->GetRcPartition()||
        math.FirstArrival->GetRcPartition()->GetScene()!=request->GetGeometrySelection()->GetSelectedUnion()))
        Refuse("Post-sync Rs first-arrival result changed geometry/initial/horizon");
    // Unknown is retained as Unknown. Neither independent positive time is a
    // physical selection, and the two mathematical algorithms need not share c.
    TSharedPtr<FNativeSpherePostSyncRsCapture,ESPMode::ThreadSafe> capture(new FNativeSpherePostSyncRsCapture);
    capture->Request=request;capture->Trace=*member->Trace;capture->ComponentSerial=component->Pending->ScopeSerial;
    capture->ResponseSerial=post.ResponseSerial;capture->OriginatingResponseData=post.Data;
    capture->Math=MoveTemp(math);capture->MathAttempted=true;post.Capture=MoveTemp(capture);
    mathMovedIntoCapture=true;
    // Retain even if member/component/Step closure subsequently refuses.
    // Only AfterInterruptedPostSyncRsCapture may populate the qualified slot.
    diagnosticContract->PostSyncRsDiagnosticCapture=post.Capture;
    throw FTerminalRsCaptureStop(s,request->GetStep().QuerySerial,member->Serial,FTerminalRsCaptureStop::ESlot::PostSync);
}
void FSphereMemberSweepScope::ClosePostSyncRsCapture(const FTerminalRsCaptureStop& stop,float actualMargin,const FNativeRsTraceValues& trace)
{
    auto* s=StepScope;
    if(!Active||Completed||Interrupted||MemberSweepScope!=this||!s||!s->HasPostSyncRsIdentity()||!s->PostSyncRs||
        stop.Slot!=FTerminalRsCaptureStop::ESlot::PostSync||stop.Lease!=s||stop.MemberSerial!=Serial||PendingGround)
        Refuse("Post-sync Rs member stop belongs to another native scope");
    auto& post=*s->PostSyncRs;auto& capture=post.Capture;
    if(!capture||capture->Stage!=FNativeSpherePostSyncRsCapture::EStage::Captured||!Trace||!Trace->Same(trace)||
        !capture->Trace.Same(trace)||!SameFloat(actualMargin,Margin)||!IsSupportedSphereResponseMargin(actualMargin)||
        !capture->Request||capture->Request->GetStep().QuerySerial!=2||stop.QuerySerial!=2||
        capture->Request->GetValues().Body!=Body||capture->Request->GetValues().Owner!=Owner||
        capture->ResponseSerial!=post.ResponseSerial||capture->OriginatingResponseData!=post.Data||
        !RevalidateSphereCCD(*capture->Request))Refuse("Post-sync Rs member did not revalidate actual trace/margin/request2");
    Interrupted=true;capture->Stage=FNativeSpherePostSyncRsCapture::EStage::MemberClosed;
}
bool FSphereMemberSweepScope::HasClosedInterruptedPostSyncRsMember() const noexcept
{
    const auto* s=StepScope;
    return Interrupted&&!Completed&&s&&s->IsPostSyncRsPurpose()&&s->PostSyncRs&&s->PostSyncRs->Capture&&
        s->PostSyncRs->Capture->Stage==FNativeSpherePostSyncRsCapture::EStage::MemberClosed&&
        s->PostSyncRs->Capture->Request&&s->PostSyncRs->Capture->Request->GetValues().Body==Body&&
        s->PostSyncRs->Capture->Request->GetValues().Owner==Owner&&s->PostSyncRs->Capture->Request->GetValues().ResponseScopeSerial==Serial&&
        s->PostSyncRs->Capture->Request->GetStep().QuerySerial==2&&Step.QuerySerial==1&&Step.QueryEpoch==4&&Step.Iteration==2;
}
bool FComponentSweepScope::CloseInterruptedPostSyncRsCapture() noexcept
{
    auto* s=StepScope;FStepIdentity now;
    if(!s||!ReadCurrentStep(now)||!s->HasPostSyncRsIdentity()||!s->PostSyncRs||!Active||Completed||Interrupted||
        ComponentSweepScope!=this||MemberSweepScope||!Pending)return false;
    auto& post=*s->PostSyncRs;auto& capture=post.Capture;
    if(!capture||capture->Stage!=FNativeSpherePostSyncRsCapture::EStage::MemberClosed||!capture->Request)return false;
    const auto& request=*capture->Request;const auto& step=request.GetStep();
    if(now.Lease!=step.Lease||now.WindowSerial!=step.WindowSerial||now.WindowNonce!=step.WindowNonce||
        now.StepSerial!=step.StepSerial||now.QueryEpoch!=4||step.QueryEpoch!=4||now.Iteration!=2||step.Iteration!=2||
        now.QuerySerial!=2||step.QuerySerial!=2||Pending->Step.QuerySerial!=1||Pending->Step.QueryEpoch!=4||
        Pending->Step.Lease!=now.Lease||Pending->Step.WindowSerial!=now.WindowSerial||Pending->Step.WindowNonce!=now.WindowNonce||
        Pending->Step.StepSerial!=now.StepSerial||Pending->ScopeSerial!=capture->ComponentSerial||Pending->Owner!=post.Owner||
        CurrentIndex!=0||NextIndex!=0||Pending->Bodies.Num()!=1||Pending->Bodies[0].Index!=0||Pending->Bodies[0].Returned||Pending->Bodies[0].Ground||
        TerminalRsRawBody!=request.GetValues().Body||!SameKinematicBits(Pending->SweepOwnerState,Pending->Owner->GetKinematicState()))return false;
    Interrupted=true;capture->Stage=FNativeSpherePostSyncRsCapture::EStage::ComponentClosed;return true;
}
FNativeSpherePostSyncRsCapturePtr FStepScope::ClosePostSyncRsCapture(const FTerminalRsCaptureStop& stop)
{
    FStepIdentity now;
    if(!ReadCurrentStep(now)||!HasPostSyncRsIdentity()||!PostSyncRs||stop.Slot!=FTerminalRsCaptureStop::ESlot::PostSync||
        stop.Lease!=this||MemberSweepScope||ComponentSweepScope)Refuse("Post-sync Rs Step stop escaped native identity");
    auto& post=*PostSyncRs;auto& capture=post.Capture;
    if(!capture||!capture->Request||capture->Stage!=FNativeSpherePostSyncRsCapture::EStage::ComponentClosed||
        !post.OwnerActive||post.SweepOwners!=1||post.SupportCount!=3||!post.ReactionObserved||!post.Owner||
        !post.RunnerEvidence||!post.RunnerEvidence->IsSealedInventory()||now.QuerySerial!=2||now.QueryEpoch!=4||now.Iteration!=2||
        stop.QuerySerial!=2||stop.MemberSerial!=capture->Request->GetValues().ResponseScopeSerial||
        capture->Request->GetStep().Lease!=this||capture->Request->GetStep().WindowNonce!=Identity.WindowNonce||
        capture->Request->GetStep().WindowSerial!=Identity.WindowSerial||capture->Request->GetStep().StepSerial!=Identity.StepSerial||
        capture->Request->GetStep().QueryEpoch!=4||capture->Request->GetStep().QuerySerial!=2||
        !PreparationPrefix::FCursor::IsCurrentPostSyncRsSweepSite(*post.Owner,1)||
        !SameKinematicBits(post.OwnerBefore,post.Owner->GetKinematicState())||post.BodiesBefore.Num()!=1||
        post.Owner->GetSubBodies().Num()!=1||post.Owner->GetSubBodies()[0]!=capture->Request->GetValues().Body||
        !SameKinematicBits(post.BodiesBefore[0],post.Owner->GetSubBodies()[0]->GetKinematicState()))
        Refuse("Post-sync Rs Step lacks authentic ordered scope closure or unchanged PVA");
    CheckPostSyncRsOwners();InvalidateQueries();post.OwnerActive=false;post.Owner=nullptr;++post.SweepOwners;
    capture->Stage=FNativeSpherePostSyncRsCapture::EStage::StepClosed;return capture;
}
