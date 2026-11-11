FNativeFrameStreamScope::FNativeFrameStreamScope(ASpeedPhysicsSimulation* host,USpeedWorldSubsystem* bridge,
    FSimulationWorker* worker,uint64 frame,float delta,float simTime)
{
    // A genuine manual one-paused request already owns Current. Preserve its
    // complete historical route, including all terminal programs.
    if(Current||!PreparedExecution::Requested())return;
    try
    {
    if(!host||!bridge||!worker||!PreparedExecution::IsCurrentWorker(worker))
        throw std::runtime_error("Native frame stream has no actual owning worker");
    const auto* provider=bridge->GetStaticCollisionWorld();
    const auto snapshot=provider?provider->GetPreparedSnapshot():FPreparedWorldSnapshotPtr{};
    Consumer=provider?provider->GetPreparedNativeFrameConsumer():FPreparedNativeFrameConsumerPtr{};
    if(!Consumer||!Consumer->ContinuousEnabled.load(std::memory_order_acquire)||Consumer->StreamWorker!=worker||
        !Consumer->Matches(snapshot,provider->GetPreparedQueryService())||!provider->IsReadyForPhysicsStep()||
        !snapshot||snapshot->GetOrigin().Host!=host||snapshot->GetOrigin().Bridge!=bridge||
        snapshot->GetOrigin().World!=host->GetWorld()||bridge->GetWorld()!=host->GetWorld()||
        snapshot->GetOrigin().Generation!=host->GetInputWorkerGeneration()||
        frame>=MAX_uint32||!SameFloat(delta,FCanonicalFrameContext::CanonicalPhysicalDeltaTime)||
        !FMath::IsFinite(simTime))
        throw std::runtime_error("Native frame stream was not armed for this retained current generation");
    const uint64 step=bridge->GetLastStepDiagnostics().Serial;
    if(step==MAX_uint64||!PreparedExecution::FCanonicalFramePreflight::HasUnchangedRawPreparationCensus(*bridge,*snapshot,step))
        throw std::runtime_error("Native frame stream registry or StepBefore changed");
    const uint64 serial=FCanonicalFrameWindow::AllocateSerialForNativeWork();
    if(!serial)throw std::runtime_error("Native frame stream window serial exhausted");
    uint64 nonce=0;
    {
        FScopeLock lock(&worker->BoundaryControlMutex);
        if(worker->NextOneWorkNonce==MAX_uint64||worker->PendingOneWork.Load()||worker->OneWorkReturnFence.Load())
            throw std::runtime_error("Native frame stream conflicts with an actual parked request");
        nonce=++worker->NextOneWorkNonce;
    }
    Window=MakeShareable(new FCanonicalFrameWindow(snapshot,host->GetWorld(),host,bridge,provider,worker,
        host->GetInputWorkerGeneration(),frame,step,serial,delta,simTime));
    if(!Window->BindNonce(nonce))throw std::runtime_error("Native frame stream nonce binding refused");
    Request=MakeUnique<FOnePausedWorkRequest>();auto& current=*Request;
    current.Identity={host,bridge,provider,worker,host->GetInputWorkerGeneration(),frame,delta,simTime,Window};
    current.Nonce=nonce;current.WindowAcceptedBeforeWork=true;current.WorkCalled=true;
    FrameBefore=frame;Current=Request.Get();
    FCanonicalFrameWindowPtr read;
    if(!ReadExecutionWindow(read)||read!=Window)
    {Current=nullptr;Window->Close();throw std::runtime_error("Native frame stream live input join refused");}
    }
    catch(...)
    {
        if(Request&&Current==Request.Get())Current=nullptr;
        if(Window)Window->Close();
        if(Consumer)Consumer->ContinuousEnabled.store(false,std::memory_order_release);
        throw;
    }
}
FNativeFrameStreamScope::~FNativeFrameStreamScope() noexcept
{
    if(!Request)return;
    if(Current==Request.Get())Current=nullptr;
    if(Window)Window->Close();
    if(!Finished&&Consumer)Consumer->ContinuousEnabled.store(false,std::memory_order_release);
}
ESimulationWorkerResult FNativeFrameStreamScope::Finish(ESimulationWorkerResult result,uint64 frameAfter)
{
    if(!Request)return result;
    if(Finished||Current!=Request.Get())
    {if(Window)Window->Close();Consumer->ContinuousEnabled.store(false,std::memory_order_release);return ESimulationWorkerResult::Failed;}
    auto& current=*Request;current.WorkReturned=true;current.Result=result;
    const auto& id=current.Identity;
    FCanonicalFrameWindowPtr read;
    const bool context=!current.GuardRejected&&ReadExecutionWindow(read)&&read==Window&&
        Consumer->Matches(Window->GetSnapshot(),id.Provider->GetPreparedQueryService())&&
        id.Host->GetInputWorkerGeneration()==id.Generation&&
        !current.PreparationStop&&!current.TerminalStepSyncEntered&&!current.TerminalRsCaptureEntered&&
        !current.TerminalConsumedPrefixAdvanceEntered&&!current.TerminalRsContactAdvanceEntered&&
        !current.TerminalRsResponseEntered&&!current.TerminalRsResponseSyncEntered&&!current.TerminalPostSyncRsCaptureEntered;
    const bool advanced=context&&result==ESimulationWorkerResult::Advanced&&frameAfter==FrameBefore+1&&
        current.RunEntered&&current.StepEntered&&current.WorldStepEntered&&current.WorldEffectsEntered&&
        current.WorldSerialConfirmed&&id.Bridge->GetLastStepDiagnostics().Serial==Window->GetExpectedStep()&&
        Consumer->LastSnapshotClosure.load(std::memory_order_acquire)==FrameBefore;
    const bool noEffects=context&&frameAfter==FrameBefore&&
        !current.RunEntered&&!current.StepEntered&&!current.WorldStepEntered&&!current.WorldEffectsEntered&&
        !current.WorldSerialConfirmed&&id.Bridge->GetLastStepDiagnostics().Serial==Window->GetStepBefore();
    Current=nullptr;Window->Close();Finished=true;
    const bool idle=noEffects&&result==ESimulationWorkerResult::Idle;
    const bool complete=noEffects&&result==ESimulationWorkerResult::Complete;
    if(complete)Consumer->ContinuousEnabled.store(false,std::memory_order_release);
    if(!advanced&&!idle&&!complete)
    {Consumer->ContinuousEnabled.store(false,std::memory_order_release);return ESimulationWorkerResult::Failed;}
    return result;
}
