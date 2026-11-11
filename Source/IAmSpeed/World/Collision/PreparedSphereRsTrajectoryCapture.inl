// Included inside Speed::PreparedExecution, in the actual native execution TU.
// The new terminal purpose is deliberately unable to deliver a hit or advance.
bool FStepScope::IsTerminalRsPurpose() const
{
    return Active&&StepScope==this&&Window&&
        OnePausedWork::HasSphereRsCapturePrefix(Window->GetPurpose());
}
void FStepScope::ObserveTerminalRsSupportInterval(const ISpeedComponent* owner,float before,float after)
{
    if(!IsTerminalRsPurpose())return;
    if(MemberSweepScope||ComponentSweepScope||RsCapture||SweepPhase||RsSupportCount>=3||
        !PreparationPrefix::FCursor::IsCurrentRsPrelude(*Identity.Bridge)||
        Registry->GetOrderedAdapters()[RsSupportCount]!=owner||!owner||
        !SameFloat(before,Identity.Delta)||!SameFloat(after,before))
        Refuse("Terminal Rs support interval changed or escaped its exact native prelude");
    RsSupportBefore.Add(before);RsSupportAfter.Add(after);++RsSupportCount;
}
void FStepScope::BeginTerminalRsSweepOwner(const ISpeedComponent* owner,float lastSubDelta)
{
    if(!IsTerminalRsPurpose())return;
    FStepIdentity now;
    if(!ReadCurrentStep(now)||!SweepPhase||RsSupportCount!=3||RsOwnerActive||RsCapture||
        MemberSweepScope||ComponentSweepScope||RsSweepOwners>=2||now.Iteration!=1||
        !SameFloat(now.TimePassed,0.f)||!SameFloat(now.Remaining,now.Delta)||!SameFloat(lastSubDelta,now.Delta)||
        !owner||Registry->GetOrderedAdapters().Num()!=3||Registry->GetOrderedAdapters()[RsSweepOwners]!=owner||
        !PreparationPrefix::FCursor::IsCurrentRsSweepSite(*owner,RsSweepOwners))
        Refuse("Terminal Rs owner dispatch is not the exact first native sweep");
    const auto& bodies=owner->GetSubBodies();
    if(bodies.Num()!=(RsSweepOwners==0?0:1)||!owner->GetPhysicalConstraints().IsEmpty())
        Refuse("Terminal Rs owner/body/constraint census differs before native sweep");
    RsOwner=owner;RsOwnerBefore=owner->GetKinematicState();RsBodiesBefore.Reset();
    for(const auto* body:bodies)
    {
        if(!body||body->GetParentComponent()!=owner)Refuse("Terminal Rs body owner changed");
        RsBodiesBefore.Add(body->GetKinematicState());
    }
    RsReactionObserved=false;RsOwnerActive=true;
    RsLastSubDelta=lastSubDelta;RsLastSubDeltaObserved=true;
}
bool FStepScope::ReadTerminalRsObservation(FTerminalRsStepObservation& out)
{
    out={};auto* s=StepScope;FStepIdentity now;
    if(!s||!ReadCurrentStep(now)||!s->IsTerminalRsPurpose()||
        !PreparationPrefix::FCursor::IsCurrentRsObservation(*now.Bridge))return false;
    out.Step=now;out.SupportCount=s->RsSupportCount;out.SweepOwners=s->RsSweepOwners;
    out.SupportBefore=s->RsSupportBefore;out.SupportAfter=s->RsSupportAfter;
    out.OwnerActive=s->RsOwnerActive;out.ReactionObserved=s->RsReactionObserved;
    out.MemberPresent=MemberSweepScope!=nullptr;out.ComponentPresent=ComponentSweepScope!=nullptr;
    out.SweepPhase=s->SweepPhase;out.CaptureClosed=s->RsCapture&&s->RsCapture->IsClosed();
    out.RunnerInventorySealed=s->RsRunnerEvidence&&s->RsRunnerEvidence->IsSealedInventory();
    out.RunnerBodies=s->RsRunnerEvidence?s->RsRunnerEvidence->GetBodies().Num():0;
    out.LastSubDeltaObserved=s->RsLastSubDeltaObserved;out.LastSubDelta=s->RsLastSubDelta;
    out.LinearReaction=s->RsLinearReaction;out.AngularReaction=s->RsAngularReaction;
    const auto& diagnostic=now.Bridge->GetLastStepDiagnostics();
    out.DiagnosticIteration=diagnostic.IterationCount;out.DiagnosticComponentSweeps=diagnostic.ComponentSweepCount;
    out.ResetMilliseconds=diagnostic.ResetMilliseconds;out.SweepMilliseconds=diagnostic.SweepMilliseconds;
    return true;
}
void FStepScope::ValidateTerminalRsReactions(const ISpeedComponent& owner,const FVector& linear,const FVector& angular)
{
    auto* s=StepScope;if(!s||!s->IsTerminalRsPurpose())return;
    if(s->PostSyncRs&&s->PostSyncRs->Started){s->ValidatePostSyncRsReactions(owner,linear,angular);return;}
    if(!s->RsOwnerActive||s->RsOwner!=&owner||s->RsReactionObserved||MemberSweepScope||ComponentSweepScope||
        !PreparationPrefix::FCursor::IsCurrentRsSweepSite(owner,s->RsSweepOwners)||
        !SameVector(linear,FVector::ZeroVector)||!SameVector(angular,FVector::ZeroVector)||
        !SameKinematicBits(owner.GetKinematicState(),s->RsOwnerBefore))
        Refuse("Terminal Rs initial route rejects native reactions before acceleration setters");
    s->RsLinearReaction=linear;s->RsAngularReaction=angular;s->RsReactionObserved=true;
}
void FStepScope::CompleteTerminalRsRunner(const ISpeedComponent* owner,const SComponentTOI& actual)
{
    if(!IsTerminalRsPurpose())return;
    if(!RsOwnerActive||RsOwner!=owner||RsSweepOwners!=0||!RsReactionObserved||RsCapture||
        MemberSweepScope||ComponentSweepScope||!actual.PreparedEvidence||!actual.PreparedEvidence->IsSealedInventory()||
        actual.PreparedEvidence->Owner!=owner||!actual.PreparedEvidence->Bodies.IsEmpty()||
        actual.bHit||!SameFloat(actual.TOI,Identity.Remaining)||!owner->GetSubBodies().IsEmpty()||
        !SameKinematicBits(RsOwnerBefore,owner->GetKinematicState()))
        Refuse("Terminal Rs zero-body runner did not return its exact actual result");
    RsRunnerEvidence=actual.PreparedEvidence;RsOwnerActive=false;RsOwner=nullptr;++RsSweepOwners;
}
bool FSphereMemberSweepScope::IsTerminalRsCaptureRequested()
{
    auto* s=StepScope;
    if(!s||!s->IsTerminalRsPurpose())return false;
    if(s->PostSyncRs&&s->PostSyncRs->Started)return IsPostSyncRsCaptureRequested();
    FStepIdentity now;
    if(!ReadCurrentStep(now)||!s->RsOwnerActive||s->RsSweepOwners!=1||!s->RsReactionObserved||
        s->RsSupportCount!=3||s->RsCapture||now.Iteration!=1||now.QuerySerial||
        !SameFloat(now.TimePassed,0.f)||!SameFloat(now.Remaining,now.Delta)||
        !PreparationPrefix::FCursor::IsCurrentRsSweepSite(*s->RsOwner,1))
        Refuse("Terminal Rs member requested outside its exact native ball site");
    return true;
}
bool FSphereMemberSweepScope::IsTerminalRsCapture()
{
    return MemberSweepScope&&MemberSweepScope->Active&&MemberSweepScope->Trace&&
        !MemberSweepScope->Completed&&!MemberSweepScope->Interrupted&&StepScope&&StepScope->IsTerminalRsPurpose();
}
[[noreturn]] void FSphereMemberSweepScope::CaptureRsAndStop(FNativeSphereRequestPtr request)
{
    auto* member=MemberSweepScope;auto* s=StepScope;auto* component=ComponentSweepScope;
    if(s&&s->PostSyncRs&&s->PostSyncRs->Started)CapturePostSyncRsAndStop(MoveTemp(request));
    if(!IsTerminalRsCapture()||!s||!component||!component->Active||component->Completed||component->Interrupted||
        !request||!RevalidateSphereCCD(*request)||s->RsCapture||member->PendingGround||
        s->RsOwner!=request->GetValues().Owner||member->Body!=request->GetValues().Body||
        request->GetValues().BodyIndex!=0||s->RsSweepOwners!=1||!s->RsReactionObserved||
        component->CurrentIndex!=0||component->NextIndex!=0||component->Pending->Bodies.Num()!=1||
        component->Pending->Bodies[0].Returned||component->Pending->Bodies[0].Ground||
        component->Pending->Owner!=s->RsOwner||request->GetStep().QuerySerial!=1||
        component->Pending->ScopeSerial==0)
        Refuse("Terminal Rs capture lacks its unique authentic member/component request");
    FCapturedRsMathResult math;
    if(!PreparationPrefix::FCursor::EvaluateCapturedRs(request,*member->Trace,math)||!RevalidateSphereCCD(*request))
        Refuse("Terminal Rs private source callback or native request revalidation refused");
    if(math.BudgetBefore<math.BudgetAfter||math.BudgetAfter<0||math.Attempts<0||math.ClosureReserve<0||math.BudgetAfter<math.ClosureReserve)
        Refuse("Terminal Rs Source math budget counters are inconsistent");
    // Mathematical objects have private constructors. Bind their exact input
    // bits to this native request before retaining even an observation.
    const auto& v=request->GetValues();
    if(math.Initial)
    {
        const auto& input=math.Initial->GetInput();const auto closest=math.Initial->GetClosestAtHeight();
        if(!SameKinematicBits(input.State,v.BodyState)||!SameFloat(input.PhysicalRadius,v.PhysicalRadius)||
            !SameFloat(input.StaticContactOffset,v.StaticContactOffsetCm)||!SameFloat(input.ActualMargin,v.ResponseMargin)||
            !SameDouble(input.SupportRadius,v.StaticSupportRadius)||!SameFloat(input.QueryRadius,v.CollisionRadius)||
            !closest||closest->GetGeometry()!=request->GetGeometrySelection())
            Refuse("Terminal Rs mathematical classification is not the captured request");
    }
    if(math.FirstArrival&&(!math.Initial||math.FirstArrival->GetInitial()!=math.Initial||
        !SameFloat(math.FirstArrival->GetHorizon(),v.Horizon)))
        Refuse("Terminal Rs first-arrival observation changed initial object or horizon");
    const bool contactPurpose=OnePausedWork::HasBoundedRsContactPrefix(s->Window->GetPurpose());
    if((s->Window->GetPurpose()==OnePausedWork::EPurpose::TerminalConsumedPrefixAdvanceStop&&math.FirstArrival)||
        (!contactPurpose&&math.RsQuality))
        Refuse("Original free-prefix purpose cannot retain a contact/quality object");
    if(math.RsQuality&&(!math.FirstArrival||math.RsQuality->GetArrival()!=math.FirstArrival))
        Refuse("Rs quality changed its actual retained first-arrival object");
    if(math.TemporalPrefix&&(!math.Initial||(!contactPurpose&&math.FirstArrival)||math.TemporalPrefix->GetInitial()!=math.Initial||
        !SameFloat(math.TemporalPrefix->GetHorizon(),v.Horizon)||!math.TemporalPrefix->HasWholeRoundedPrefixProof()||
        !math.TemporalPrefix->GetConsumedPoint()||math.TemporalPrefix->GetConsumedPoint()->GetKind()!=Analytic::Prepared::EReferenceSphereRsState::SeparatedRs||
        math.RetainedTemporalAttempt<=0||math.RetainedTemporalAttempt>math.Attempts))
        Refuse("Terminal Rs temporal prefix changed its captured input or masquerades as a full contact");
    if(OnePausedWork::HasConsumedSphereAdvancePrefix(s->Window->GetPurpose())&&math.AdvanceSource)
    {
        const auto& proof=math.AdvanceSource;
        if(contactPurpose)
        {
            if(!math.FirstArrival||!math.RsQuality||!math.TemporalPrefix||
                math.RsQuality->GetStatus()!=ESphereResponseQualityStatus::BoundedWithinInputMargin||
                math.FirstArrival->GetReference().Status!=Analytic::Certified::EReferenceSphereStatus::ReferenceContactBounded||
                !SameFloat(math.FirstArrival->GetReference().ConsumedSeconds,math.TemporalPrefix->GetConsumedSeconds())||
                !SameVector(math.FirstArrival->GetConsumedPoint()->GetInput().State.Location,math.TemporalPrefix->GetConsumedPoint()->GetInput().State.Location))
                Refuse("Rs contact-advance Source proposition lacks the same bounded contact/quality/c");
        }
        if(!proof||proof->SealedDuringMember||proof->Request!=request||proof->Prefix!=math.TemporalPrefix||
            !proof->PhysicalSourcePlan||!proof->Trace||!proof->Trace->Same(*member->Trace)||proof->Owners.Num()!=3||
            proof->NumericControl!=request->GetStep().NumericControl)
            Refuse("Consumed-prefix Source proposition was not made during this live native member");
        proof->SealedDuringMember=true; // set once by the ACTUAL member before revocation
    }
    else if(math.AdvanceSource)Refuse("Observation-only Rs capture cannot retain an advance Source proposition");
    else if(s->Window->GetPurpose()==OnePausedWork::EPurpose::TerminalConsumedPrefixAdvanceStop)
        Refuse("Original consumed-prefix purpose requires its positive Source proposition");
    // New contact-purpose Unknown/error diagnostics remain real copied facts.
    // Without a sealed positive Source proposition the advance gate refuses
    // before integration; retaining these values grants no mutation.
    TSharedPtr<FNativeSphereRsTrajectoryCapture,ESPMode::ThreadSafe> capture(new FNativeSphereRsTrajectoryCapture);
    capture->Request=request;capture->Trace=*member->Trace;capture->ComponentSerial=component->Pending->ScopeSerial;
    capture->AdvanceSource=MoveTemp(math.AdvanceSource);
    capture->RsQuality=MoveTemp(math.RsQuality);capture->FirstArrivalPartial=MoveTemp(math.FirstArrivalPartial);
    capture->Initial=MoveTemp(math.Initial);capture->FirstArrival=MoveTemp(math.FirstArrival);capture->MathReason=MoveTemp(math.Reason);
    capture->MathBudgetBefore=math.BudgetBefore;capture->MathBudgetAfter=math.BudgetAfter;capture->MathAttempts=math.Attempts;
    capture->TemporalPrefix=MoveTemp(math.TemporalPrefix);capture->TemporalDiagnostic=MoveTemp(math.TemporalDiagnostic);
    capture->MathClosureReserve=math.ClosureReserve;capture->MaxObservedSourceUnits=math.MaxObservedSourceUnits;capture->RetainedTemporalAttempt=math.RetainedTemporalAttempt;
    capture->MathAttempted=true;capture->InitialKnown=bool(capture->Initial);capture->FirstArrivalKnown=bool(capture->FirstArrival);
    if(capture->FirstArrival)
    {
        const auto& reference=capture->FirstArrival->GetReference();uint32 bits=0;
        FMemory::Memcpy(&bits,&reference.ConsumedSeconds,4);
        capture->PositiveConsumedPrefix=bits>=0x00800000u&&bits<0x7f800000u&&reference.ConsumedPrefixClear&&
            capture->FirstArrival->GetConsumedPoint()&&
            capture->FirstArrival->GetConsumedPoint()->GetKind()==Analytic::Prepared::EReferenceSphereRsState::SeparatedRs;
    }
    if(capture->TemporalPrefix)
    {
        const float seconds=capture->TemporalPrefix->GetConsumedSeconds();uint32 bits=0;FMemory::Memcpy(&bits,&seconds,4);
        capture->PositiveConsumedPrefix=bits>=0x00800000u&&bits<0x7f800000u&&capture->TemporalPrefix->HasWholeRoundedPrefixProof();
    }
    s->RsCapture=MoveTemp(capture);
    throw FTerminalRsCaptureStop(s,request->GetStep().QuerySerial,member->Serial);
}
void FSphereMemberSweepScope::CloseTerminalRsCapture(const FTerminalRsCaptureStop& stop,float actualMargin,const FNativeRsTraceValues& trace)
{
    if(stop.Slot==FTerminalRsCaptureStop::ESlot::PostSync){ClosePostSyncRsCapture(stop,actualMargin,trace);return;}
    if(stop.Slot!=FTerminalRsCaptureStop::ESlot::Initial)Refuse("Unsupported Rs capture stop slot");
    auto* s=StepScope;
    if(!Active||Completed||Interrupted||MemberSweepScope!=this||!s||!s->IsTerminalRsPurpose()||
        stop.Lease!=s||stop.MemberSerial!=Serial||!s->RsCapture||
        s->RsCapture->Stage!=FNativeSphereRsTrajectoryCapture::EStage::Captured||!Trace||!Trace->Same(trace)||
        !s->RsCapture->Trace.Same(trace)||!SameFloat(actualMargin,Margin)||!IsSupportedSphereResponseMargin(actualMargin)||
        !s->RsCapture->Request||s->RsCapture->Request->GetStep().QuerySerial!=stop.QuerySerial||
        s->RsCapture->Request->GetValues().Body!=Body||s->RsCapture->Request->GetValues().Owner!=Owner||
        !RevalidateSphereCCD(*s->RsCapture->Request)||PendingGround)
        Refuse("Terminal Rs member interruption did not revalidate actual margin/trace/request");
    Interrupted=true;s->RsCapture->Stage=FNativeSphereRsTrajectoryCapture::EStage::MemberClosed;
}
bool FComponentSweepScope::CloseInterruptedTerminalRsCapture() noexcept
{
    auto* s=StepScope;FStepIdentity now;
    if(s&&s->PostSyncRs&&s->PostSyncRs->Started)return CloseInterruptedPostSyncRsCapture();
    if(!s||!ReadCurrentStep(now)||!s->IsTerminalRsPurpose()||!Active||Completed||Interrupted||ComponentSweepScope!=this||MemberSweepScope||
        !s->RsCapture||s->RsCapture->Stage!=FNativeSphereRsTrajectoryCapture::EStage::MemberClosed||!Pending||
        !s->RsCapture->Request)return false;
    const auto& request=*s->RsCapture->Request;const auto& step=request.GetStep();
    if(now.Lease!=step.Lease||now.WindowSerial!=step.WindowSerial||now.WindowNonce!=step.WindowNonce||
        now.StepSerial!=step.StepSerial||now.QueryEpoch!=step.QueryEpoch||now.QuerySerial!=step.QuerySerial||
        Pending->Step.Lease!=now.Lease||Pending->Step.WindowSerial!=now.WindowSerial||
        Pending->Step.WindowNonce!=now.WindowNonce||Pending->Step.StepSerial!=now.StepSerial||
        Pending->Step.QueryEpoch!=now.QueryEpoch||Pending->Step.QuerySerial!=0||now.QuerySerial!=1||
        Pending->ScopeSerial!=s->RsCapture->ComponentSerial||Pending->Owner!=s->RsOwner||
        CurrentIndex!=0||NextIndex!=0||Pending->Bodies.Num()!=1||Pending->Bodies[0].Returned||Pending->Bodies[0].Ground||
        Pending->Bodies[0].Index!=0||TerminalRsRawBody!=request.GetValues().Body||
        !SameKinematicBits(Pending->SweepOwnerState,Pending->Owner->GetKinematicState()))return false;
    Interrupted=true;s->RsCapture->Stage=FNativeSphereRsTrajectoryCapture::EStage::ComponentClosed;
    return true; // Pending remains unsealed, body remains not Returned.
}
FNativeSphereRsTrajectoryCapturePtr FStepScope::CloseTerminalRsCapture(const FTerminalRsCaptureStop& stop)
{
    if(stop.Slot!=FTerminalRsCaptureStop::ESlot::Initial)Refuse("Original Rs Step cannot consume post-sync stop");
    FStepIdentity now;
    if(!ReadCurrentStep(now)||!IsTerminalRsPurpose()||stop.Lease!=this||MemberSweepScope||ComponentSweepScope||!RsCapture||
        !RsCapture->Request)Refuse("Terminal Rs Step capture is not in the same live native window");
    const auto& step=RsCapture->Request->GetStep();
    if(now.Lease!=step.Lease||now.WindowSerial!=step.WindowSerial||now.WindowNonce!=step.WindowNonce||
        now.StepSerial!=step.StepSerial||now.QueryEpoch!=step.QueryEpoch||now.QuerySerial!=step.QuerySerial||
        RsCapture->Stage!=FNativeSphereRsTrajectoryCapture::EStage::ComponentClosed||!RsOwnerActive||RsSweepOwners!=1||
        !RsReactionObserved||!RsOwner||!RsRunnerEvidence||!RsRunnerEvidence->IsSealedInventory()||
        stop.QuerySerial!=RsCapture->Request->GetStep().QuerySerial||
        stop.MemberSerial!=RsCapture->Request->GetValues().ResponseScopeSerial||
        !PreparationPrefix::FCursor::IsCurrentRsSweepSite(*RsOwner,1)||
        !SameKinematicBits(RsOwnerBefore,RsOwner->GetKinematicState())||RsBodiesBefore.Num()!=1||
        RsOwner->GetSubBodies().Num()!=1||RsOwner->GetSubBodies()[0]!=RsCapture->Request->GetValues().Body||
        !SameKinematicBits(RsBodiesBefore[0],RsOwner->GetSubBodies()[0]->GetKinematicState())||
        !FCanonicalFramePreflight::HasUnchangedRawPreparationCensus(*Identity.Bridge,*Snapshot,Identity.StepSerial))
        Refuse("Terminal Rs Step interruption lacks real closed scopes or exact restored states");
    InvalidateQueries();RsOwnerActive=false;RsOwner=nullptr;++RsSweepOwners;
    RsCapture->Stage=FNativeSphereRsTrajectoryCapture::EStage::StepClosed;return RsCapture;
}
