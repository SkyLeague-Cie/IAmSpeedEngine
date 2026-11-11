// Included after native owner-probe and integration values.
#if !UE_BUILD_SHIPPING
static TAutoConsoleVariable<int32> CVarNativeCase585FinalWheelSupport(
    TEXT("p.IAmSpeed.AnalyticWorld.NativeCase585FinalWheelSupport"),0,
    TEXT("Certify one original case-585 FinalPost wheel-support translation after the proven fifth response and rotation. Default off."));
static thread_local FNativeFinalWheelSupportScope* NativeFinalWheelSupport=nullptr;

struct FNativeFinalWheelSupportScope::FState
{
    FStepScope* Scope=nullptr;
    FOrdinaryFinalOperation* Operation=nullptr;
    const ISpeedComponent* Owner=nullptr;
    const FOrdinaryOwnerProbePhaseScope* Phase=nullptr;
    FStepIdentity Step;
    FNativeSolidStaticRequestPtr Request;
    FNativeCanonicalBoxSweepClearanceValuesPtr Prediction;
    FNativeBoxPoseCorrectionGeometryValuesPtr Proof;
    FKinematicState Before,Candidate,Origin;
    TArray<FrameParallel::FSubBodyKinematicInput,TInlineAllocator<5>> Inputs;
    bool SetterOpen=false,SetterReturned=false,SyncOpen=false,Synchronized=false;
};

FNativeFinalWheelSupportScope::FState& FNativeFinalWheelSupportScope::Require()
{
    if(!NativeFinalWheelSupport||!NativeFinalWheelSupport->State)
        Refuse("585 final wheel support lost its lexical caller");
    auto& state=*NativeFinalWheelSupport->State;
    auto* scope=StepScope;
    auto* phase=OwnerProbePhase;
    FStepIdentity now;
    if(!scope||scope!=state.Scope||!scope->Active||!scope->NativeConsumer||!scope->Snapshot||!scope->Identity.Provider||
        !phase||phase!=state.Phase||!phase->State||phase->State->Returned||!phase->Check()||
        phase->State->Phase!=ENativeOwnerProbePhase::FinalPost||phase->State->Owner!=state.Owner||
        phase->State->Snapshot!=scope->Snapshot||WheelProbeScope||scope->SweepPhase||
        !scope->OrdinaryFinal||scope->OrdinaryFinal->Closed||!scope->OrdinaryFinal->CallOpen||
        scope->OrdinaryFinal->Operations.IsEmpty()||&scope->OrdinaryFinal->Operations.Last()!=state.Operation||
        state.Operation->Kind!=ENativeFrameFinalOperation::PostOwner||state.Operation->Owner!=state.Owner||
        !state.Operation->Called||state.Operation->Returned||!state.Operation->GeneralBoxOnly||
        !scope->OrdinaryIntegration||!scope->OrdinaryIntegration->Closed||
        scope->OrdinaryIntegration->ActiveIndex!=INDEX_NONE||
        (scope->OrdinaryResponse&&!scope->OrdinaryResponse->Closed)||
        !ReadCurrentStep(now)||!OrdinaryWorldDetail::Epoch(state.Step,now)||state.Step.QuerySerial!=now.QuerySerial||
        state.Request->GetSnapshot()!=scope->Snapshot||
        !GeneralBoxTransportDetail::Configuration(state.Request->GetValues()))
        Refuse("585 final wheel support context or actual shape changed");
    scope->RequireNativeFrameContext();
    return state;
}

FNativeOwnerPoseMutationScope::FFinalWheelSupport::FFinalWheelSupport(const ISpeedComponent& owner,int32 pass)
{
    if(pass!=0||CVarNativeCase585FinalWheelSupport.GetValueOnAnyThread()!=1||!Requested())return;
    auto* scope=StepScope;
    const auto* phase=OwnerProbePhase;
    FStepIdentity step;
    if(!scope||!scope->Active||!scope->NativeConsumer||!scope->Snapshot||
        !scope->Case585FourthGround||!scope->Case585FifthGround||!scope->Case585FifthResolved||
        scope->Case585AcceptingFifth||!scope->Case585Pair||
        !phase||!phase->State||phase->State->Phase!=ENativeOwnerProbePhase::FinalPost||
        phase->State->Owner!=&owner||!ReadCurrentStep(step)||step.Frame!=86||step.QuerySerial!=6||
        !scope->OrdinaryFinal||scope->OrdinaryFinal->Operations.IsEmpty()||
        !scope->OrdinaryIntegration||!scope->OrdinaryIntegration->Closed)return;
    auto& operation=scope->OrdinaryFinal->Operations.Last();
    if(!operation.GeneralBoxOnly||operation.WheelSupportExpected||scope->OrdinaryFinal->WheelSupportOperation!=INDEX_NONE)return;
    const auto* row=scope->OrdinaryIntegration->Owners.FindByPredicate([&](const auto& item){return item.Owner==&owner;});
    if(!row||!row->Closed||row->OpenPoseSetter||row->PoseAssignmentsBegun!=row->PoseAssignmentsCompleted||
        row->SyncVersion!=row->PoseVersion||row->GeneralBindings.Num()!=1||row->GeneralProofs.Num()!=1||
        !row->GeneralProofs[0]||row->LatestPoseCorrections.Num()!=1||!row->LatestPoseCorrections[0])return;
    const auto& binding=row->GeneralBindings[0];
    if(!binding.HasExactlyOneKind()||!binding.Request||binding.Request->GetSnapshot()!=scope->Snapshot||
        binding.Request->GetStep().Frame!=step.Frame||binding.Request->GetStep().StepSerial!=step.StepSerial||
        binding.Request->GetStep().InputGeneration!=step.InputGeneration||binding.Request->GetStep().QuerySerial!=6||
        !binding.Prediction||!binding.Prediction->IsCurvedActualMotionClear())return;
    const auto& source=binding.Request->GetValues();
    const auto& fifth=*scope->Case585FifthGround;
    const auto& fifthRequest=fifth.GetRequestForInspection();
    const auto& hit=fifth.GetGroundValuesForInspection();
    if(source.Owner!=&owner||source.OwnerStableId!=3||source.FrameClock.OwnerLocalFrame!=87||source.BodyIndex!=0||
        !fifth.IsCurvedBoxArrivalValues()||!fifthRequest||fifthRequest->GetSnapshot()!=scope->Snapshot||
        fifthRequest->GetStep().Frame!=step.Frame||fifthRequest->GetStep().StepSerial!=step.StepSerial||
        fifthRequest->GetStep().InputGeneration!=step.InputGeneration||fifthRequest->GetStep().QuerySerial!=5||
        fifthRequest->GetValues().Owner!=source.Owner||fifthRequest->GetValues().Body!=source.Body||
        hit.SourceId!=1274291482858373289ull||hit.SurfaceId!=12602828015134877967ull||hit.ContactFeatureIndexThis!=1||
        !row->LatestPoseCorrections[0]->HasCurvedActualBinding(binding.Prediction)||
        !row->LatestPoseCorrections[0]->MatchesEndpoint(scope->Snapshot,source,owner.GetKinematicState()))return;
    if(NativeFinalWheelSupport||NativeOwnerPoseMutation)Refuse("585 final wheel support caller nested");
    State=MakeUnique<FState>();State->Scope=scope;State->Operation=&operation;State->Owner=&owner;
    State->Phase=phase;State->Step=step;State->Request=binding.Request;State->Prediction=binding.Prediction;
    NativeFinalWheelSupport=this;
    try{Require();}catch(...){NativeFinalWheelSupport=nullptr;throw;}
}

FNativeOwnerPoseMutationScope::FFinalWheelSupport::~FFinalWheelSupport() noexcept
{
    if(!State)return;
    if(NativeFinalWheelSupport!=this)RecordPreparedPoison("case585_final_wheel_support_scope_identity");
    else NativeFinalWheelSupport=nullptr;
    if((State->SetterOpen||State->SetterReturned)&&!State->Synchronized)RecordPreparedPoison("case585_final_wheel_support_missing_actual_setter_or_sync");
}

bool FNativeFinalWheelSupportScope::BeginSetter(const ISpeedComponent& owner,
    const FKinematicState& before,const FKinematicState& candidate)
{
    if(!NativeFinalWheelSupport)return false;
    auto& state=Require();
    auto translated=before;translated.Location=candidate.Location;
    if(state.Owner!=&owner||state.SetterOpen||state.SetterReturned||state.Synchronized||
        !FiniteState(before)||!FiniteState(candidate)||!SameKinematicBits(before,owner.GetKinematicState())||
        !SameKinematicBits(translated,candidate)||SameVector(before.Location,candidate.Location))
        Refuse("585 final wheel support is not one real pure translation");
    const auto& captured=state.Operation->GeneralBoxOnly->Before;
    const auto* covered=captured.Owners.FindByPredicate([&](const auto& item){return item.Owner==&owner;});
    if(!captured.Captured||!covered||!GeneralBoxTransportDetail::Pose(covered->State,before)||
        owner.GetSubBodies().Num()!=covered->Bodies.Num()||owner.GetSubBodies().Num()!=5)
        Refuse("585 final wheel support lost its original covered endpoint");
    const auto& source=state.Request->GetValues();
    const auto service=state.Scope->Identity.Provider->GetPreparedQueryService();
    FNativeBoxPoseCorrectionGeometryValuesPtr proof;
    if(!service||service->GetPreparedSnapshot()!=state.Scope->Snapshot||
        !service->DescribeNativeBoxPoseCorrectionWithCompleteClear(state.Request,before,candidate,proof,state.Prediction)||
        !proof||!proof->Matches(state.Scope->Snapshot,source,before,candidate)||
        !proof->MatchesCurvedActualBinding(state.Prediction))
        Refuse("585 final wheel support is not strictly clear of the complete static world");
    state.Origin=Speed::CalculateOriginKinematicStateValues(candidate,source.LocalCOM);
    if(!FiniteState(state.Origin))Refuse("585 final wheel support expected origin is invalid");
    for(int32 i=0;i<covered->Bodies.Num();++i)
    {
        const auto* body=owner.GetSubBodies()[i];
        const auto& original=covered->Bodies[i];
        FrameParallel::FSubBodyKinematicInput input;
        if(!body||!state.Scope->NativeConsumer->ReadCurrentBodyLocalKinematicValues(body,input)||
            input.bWheel!=(i!=source.BodyIndex)||
            (i==source.BodyIndex&&(!original.BoxShapeCaptured||body!=source.Body||original.Body!=body||
                !GeneralBoxTransportDetail::Pose(original.State,body->GetKinematicState())||
                !SameVector(input.RelativeLocation,source.LocalOffset)||!SameQuat(input.RelativeRotation,source.LocalRotation))))
            Refuse("585 final wheel support lost its exact native child inputs");
        state.Inputs.Add(input);
    }
    Require();
    if(!SameKinematicBits(before,owner.GetKinematicState()))
        Refuse("585 final wheel support changed during corridor qualification");
    state.Before=before;state.Candidate=candidate;state.Proof=MoveTemp(proof);state.SetterOpen=true;
    return true;
}

void FNativeFinalWheelSupportScope::EndSetter(const ISpeedComponent& owner,const FKinematicState& actual)
{
    auto& state=Require();
    if(state.Owner!=&owner||!state.SetterOpen||state.SetterReturned||
        !SameKinematicBits(actual,state.Candidate)||!state.Proof||
        !state.Proof->Matches(state.Scope->Snapshot,state.Request->GetValues(),state.Before,actual))
        Refuse("585 final wheel support actual setter return differs");
    state.SetterOpen=false;state.SetterReturned=true;
}

bool FNativeFinalWheelSupportScope::BeforeChildSync(const ISpeedComponent& owner,const FKinematicState& origin)
{
    if(!HasPendingSync())return false;
    auto& state=Require();
    if(state.Owner!=&owner||state.SetterOpen||!state.SetterReturned||state.SyncOpen||
        !SameKinematicBits(state.Candidate,owner.GetKinematicState())||!SameKinematicBits(state.Origin,origin))
        Refuse("585 final wheel support actual child sync opened without its exact setter return");
    state.SyncOpen=true;
    return true;
}

bool FNativeFinalWheelSupportScope::AfterChildSync(const ISpeedComponent& owner,const FKinematicState& origin)
{
    if(!HasPendingSync())return false;
    auto& state=Require();
    if(state.Owner!=&owner||state.SetterOpen||!state.SetterReturned||!state.SyncOpen||state.Synchronized||
        !SameKinematicBits(state.Candidate,owner.GetKinematicState())||!SameKinematicBits(state.Origin,origin)||
        owner.GetSubBodies().Num()!=state.Inputs.Num())
        Refuse("585 final wheel support actual child sync has no matching returned setter");
    auto expected=MakeUnique<FFinalGeneralBoxStateValues>(state.Operation->GeneralBoxOnly->Before);
    auto* covered=expected->Owners.FindByPredicate([&](const auto& item){return item.Owner==&owner;});
    if(!covered||covered->Bodies.Num()!=state.Inputs.Num())
        Refuse("585 final wheel support child inventory changed");
    covered->State=state.Candidate;
    for(int32 i=0;i<state.Inputs.Num();++i)
    {
        const auto* child=owner.GetSubBodies()[i];
        const auto& input=state.Inputs[i];
        FrameParallel::FSubBodyKinematicInput actual;
        const auto target=FrameParallel::CalculateSubBodyKinematics(state.Origin,input);
        if(!child||!state.Scope->NativeConsumer->ReadCurrentBodyLocalKinematicValues(child,actual)||
            input.bWheel!=actual.bWheel||!SameFloat(input.WheelSpringDisplacement,actual.WheelSpringDisplacement)||
            !SameVector(input.RelativeLocation,actual.RelativeLocation)||!SameQuat(input.RelativeRotation,actual.RelativeRotation)||
            !FiniteState(target)||!SameKinematicBits(target,child->GetKinematicState()))
            Refuse("585 final wheel support actual synchronized child differs");
        if(covered->Bodies[i].BoxShapeCaptured)covered->Bodies[i].State=target;
    }
    Require();
    state.Operation->WheelSupportExpected=MoveTemp(expected);state.Synchronized=true;
    UE_LOG(LogTemp,Display,TEXT("[NativeCase585FinalWheelSupportSync] Frame=%llu Serial=%llu Children=%d"),
        state.Step.Frame,state.Step.QuerySerial,state.Inputs.Num());
    return true;
}

bool FNativeFinalWheelSupportScope::HasPendingSync() noexcept
{
    if(!NativeFinalWheelSupport||!NativeFinalWheelSupport->State)return false;
    const auto& state=*NativeFinalWheelSupport->State;
    return (state.SetterOpen||state.SetterReturned)&&!state.Synchronized;
}
#else
struct FNativeFinalWheelSupportScope::FState {};
FNativeOwnerPoseMutationScope::FFinalWheelSupport::FFinalWheelSupport(const ISpeedComponent&,int32) {}
FNativeOwnerPoseMutationScope::FFinalWheelSupport::~FFinalWheelSupport() noexcept=default;
bool FNativeFinalWheelSupportScope::BeginSetter(const ISpeedComponent&,const FKinematicState&,const FKinematicState&){return false;}
void FNativeFinalWheelSupportScope::EndSetter(const ISpeedComponent&,const FKinematicState&){}
bool FNativeFinalWheelSupportScope::BeforeChildSync(const ISpeedComponent&,const FKinematicState&){return false;}
bool FNativeFinalWheelSupportScope::AfterChildSync(const ISpeedComponent&,const FKinematicState&){return false;}
bool FNativeFinalWheelSupportScope::HasPendingSync() noexcept{return false;}
#endif

static const FFinalGeneralBoxStateValues* CurrentFinalWheelSupportBaseline(const FOrdinaryFinalState& state)
{
    if(!state.Operations.IsEmpty()&&state.Operations.Last().WheelSupportExpected)
        return state.Operations.Last().WheelSupportExpected.Get();
    if(state.WheelSupportOperation==INDEX_NONE)return nullptr;
    if(!state.Operations.IsValidIndex(state.WheelSupportOperation))
        Refuse("585 final wheel support baseline lost its closed operation");
    const auto& correction=state.Operations[state.WheelSupportOperation];
    if(!correction.Returned||!correction.GeneralBoxOnly||!correction.GeneralBoxOnly->After.Captured||
        !correction.WheelSupportExpected)Refuse("585 final wheel support baseline is incomplete");
    return &correction.GeneralBoxOnly->After;
}
