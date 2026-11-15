bool FCursor::IsClosedPlannedRow(int32 i) const
{
    if(!Contract||!Contract->Records.IsValidIndex(i))return false;
    const auto& row=Contract->Records[i];
    if(!row.Attempted||!row.Accepted)return false;
    if(OnePausedWork::HasSphereRsCapturePrefix(Contract->GetPurpose())&&i==37)
        return !row.Returned&&row.InterruptedAtRsCapture&&Contract->HasClosedNativeRsCapture();
    if(Contract->GetPurpose()==OnePausedWork::EPurpose::TerminalPostSyncRsCaptureStop&&i==49)
        return !row.Returned&&!row.InterruptedAtRsCapture&&row.InterruptedAtPostSyncRsCapture&&Contract->HasCompletedNativePostSyncRsCapture();
    return row.Returned&&!row.InterruptedAtRsCapture&&!row.InterruptedAtPostSyncRsCapture;
}
bool FCursor::IsCurrentRsNativeWindow(const USpeedWorldSubsystem& bridge) noexcept
{
    try
    {
        const auto* c=Current;
        if(!c||c->Context.Bridge!=&bridge||!c->CurrentIdentity()||c->PositionDispatch||
            !OnePausedWork::HasSphereRsCapturePrefix(c->Contract->GetPurpose())||
            c->Contract->Calls.Num()!=OnePausedWork::ExpectedNativeCallCount(c->Contract->GetPurpose())||c->Contract->Records.Num()!=c->Contract->Calls.Num()||
            !c->Contract->StepSerialConfirmed||!c->Contract->StepSyncReached||!c->Contract->HasAcceptedPosition()||
            c->Contract->Reached||c->CurrentStep!=c->Window->GetExpectedStep()||c->NextCall<35||c->NextCall>38)return false;
        if(c->Observing)
        {
            const auto* a=c->ActiveObservation;
            return a&&a->Epoch==c->Epoch&&a->Ordinal>=36&&a->Ordinal<=38&&
                (a->Effect==EEffect::BeginRsCaptureSweep||a->Effect==EEffect::SweepOwnerForRsCapture);
        }
        return true;
    }
    catch(...){return false;}
}
bool FCursor::IsCurrentRsPrelude(const USpeedWorldSubsystem& bridge) noexcept
{
    const auto* c=Current;
    return IsCurrentRsNativeWindow(bridge)&&c&&!c->Observing&&c->NextCall==36&&
        c->Stack.Num()==1&&c->Stack[0]==35&&c->Epoch==71&&
        c->Contract->Records[35].Attempted&&!c->Contract->Records[35].Returned;
}
bool FCursor::IsCurrentRsObservation(const USpeedWorldSubsystem& bridge) noexcept
{
    const auto* c=Current;
    return IsCurrentRsNativeWindow(bridge)&&c&&c->Observing&&c->ActiveObservation&&
        c->ActiveObservation->Epoch==c->Epoch&&c->ActiveObservation->Ordinal>=36&&c->ActiveObservation->Ordinal<=38;
}
bool FCursor::IsCurrentRsSweepSite(const ISpeedComponent& owner,int32 index) noexcept
{
    const auto* c=Current;if(!c||index<0||index>1)return false;
    const int32 row=36+index;
    return c->Context.Bridge&&IsCurrentRsNativeWindow(*c->Context.Bridge)&&!c->Observing&&
        c->NextCall==row+1&&c->Stack.Num()==1&&c->Stack[0]==row&&c->Epoch==uint64(row*2+1)&&
        c->Contract->Calls[row].Effect==EEffect::SweepOwnerForRsCapture&&c->Contract->Calls[row].Owner==&owner&&
        c->Contract->Records[row].Attempted&&!c->Contract->Records[row].Returned&&
        !c->Contract->Records[row].InterruptedAtRsCapture;
}
bool FCursor::EvaluateCapturedRs(const PreparedExecution::FNativeSphereRequestPtr& request,
    const PreparedExecution::FNativeRsTraceValues& trace,PreparedExecution::FCapturedRsMathResult& out)
{
    out={};auto* c=Current;
    if(!c||!request||!request->GetValues().Owner||
        !IsCurrentRsSweepSite(*static_cast<const ISpeedComponent*>(request->GetValues().Owner),1)||
        !c->Contract->EvaluateCapturedRsAction||c->Contract->RsMathAttempted||c->Contract->RsMathReturned||
        c->Contract->RsCapture||request->GetSnapshot()!=c->Window->GetSnapshot()||
        request->GetValues().BodyIndex!=0||request->GetStep().QuerySerial!=1||
        request->GetStep().WindowSerial!=c->Context.WindowSerial||request->GetStep().WindowNonce!=c->Context.Nonce||
        request->GetStep().StepSerial!=c->CurrentStep||request->GetStep().Frame!=c->Context.Frame||
        !PreparedExecution::RevalidateSphereCCD(*request))return false;
    const auto& context=c->Contract->Records[37].BeforeContext;
    bool accepted=false;
    {
        TGuardValue<bool> observing(c->Observing,true);
        TGuardValue<const FContext*> active(c->ActiveObservation,&context);
        c->Contract->RsRequestCaptured=request; // authentic partial fact survives a later math/closure refusal
        c->Contract->RsMathAttempted=true;
        accepted=c->Contract->EvaluateCapturedRsAction(c->Contract->SourceState.get(),context,request,trace,out,c->Contract->RsMathActionRefusal);
        c->Contract->RsMathReturned=true;
    }
    return accepted&&c->CurrentIdentity()&&PreparedExecution::RevalidateSphereCCD(*request);
}
void FCursor::AfterInterruptedRsCapture(int32 i,PreparedExecution::FNativeSphereRsTrajectoryCapturePtr capture)
{
    if(Current!=this||Observing||!CurrentIdentity()||
        !OnePausedWork::HasSphereRsCapturePrefix(Contract->GetPurpose())||
        Contract->Records.Num()!=OnePausedWork::ExpectedNativeCallCount(Contract->GetPurpose())||Contract->Calls.Num()!=Contract->Records.Num()||i!=37||NextCall!=38||
        Stack.Num()!=1||Stack[0]!=37||Epoch!=75||Contract->RsCapture||
        !capture||!capture->IsClosed()||!capture->GetRequest()||!Contract->RsMathAttempted||!Contract->RsMathReturned)
        Refuse(TEXT("Rs interruption is not the one actual closed native capture"));
    auto& row=Contract->Records[i];const auto& call=Contract->Calls[i];
    const auto& request=*capture->GetRequest();const auto& step=request.GetStep();
    if(!row.Attempted||row.Returned||row.Accepted||row.InterruptedAtRsCapture||
        call.Effect!=EEffect::SweepOwnerForRsCapture||!call.Owner||call.Owner!=request.GetValues().Owner||
        request.GetValues().BodyIndex!=0||request.GetSnapshot()!=Window->GetSnapshot()||
        step.Host!=Context.Host||step.Bridge!=Context.Bridge||step.Worker!=Context.Worker||step.World!=Context.World||
        step.InputGeneration!=Context.Generation||step.Frame!=Context.Frame||step.StepSerial!=CurrentStep||
        step.WindowSerial!=Context.WindowSerial||step.WindowNonce!=Context.Nonce||step.QuerySerial!=1||step.Iteration!=1)
        Refuse(TEXT("Rs closed capture belongs to a different native source/window/member"));
    Contract->RsCapture=MoveTemp(capture);
    row.InterruptedAtRsCapture=true; // Actual interruption remains recorded even if Source refuses below.
    row.AfterContext=MakeContext(call,38,false);row.AfterContext.InterruptedAtRsCapture=true;
    row.AfterContext.NativeReturnValue=0; // This is not a normal native callback return.
    try
    {
        row.Accepted=Observe(EObservation::InterruptedAtRsCapture,row.AfterContext,row.After,row.Refusal)&&CurrentIdentity();
        Stack.Pop(EAllowShrinking::No);
        if(!row.Accepted)Refuse(TEXT("Rs interrupted capture Source/value closure refused"));
    }
    catch(...){Contract->Failed=true;Window->Close();throw;}
}
void FEffectScope::InterruptedAtRsCapture(PreparedExecution::FNativeSphereRsTrajectoryCapturePtr capture)
{
    if(!Cursor)throw std::runtime_error("Rs interruption has no native source effect");
    auto* cursor=Cursor;const int32 i=Record;
    Cursor=nullptr;Record=INDEX_NONE;cursor->AfterInterruptedRsCapture(i,MoveTemp(capture));
}

bool FCursor::IsCurrentConsumedPrefixNativeWindow(const USpeedWorldSubsystem& bridge) noexcept
{
    try
    {
        const auto* c=Current;
        if(!c||c->Context.Bridge!=&bridge||!c->CurrentIdentity()||c->PositionDispatch||
            !OnePausedWork::HasConsumedSphereAdvancePrefix(c->Contract->GetPurpose())||
            c->Contract->Calls.Num()!=OnePausedWork::ExpectedNativeCallCount(c->Contract->GetPurpose())||c->Contract->Records.Num()!=OnePausedWork::ExpectedNativeCallCount(c->Contract->GetPurpose())||!c->Contract->HasClosedNativeRsCapture()||
            c->Contract->Reached||c->NextCall<38||c->NextCall>42)return false;
        if(c->Observing)
        {
            const auto* a=c->ActiveObservation;
            return a&&a->Epoch==c->Epoch&&a->Ordinal>=39&&a->Ordinal<=42&&
                (a->Effect==EEffect::AdvanceRsPrefix||a->Effect==EEffect::AdvanceRsPrefixOwner);
        }
        return !c->Stack.IsEmpty()&&c->Stack[0]==38;
    }
    catch(...){return false;}
}
bool FCursor::IsCurrentConsumedPrefixSite(int32 owner) noexcept
{
    const auto* c=Current;
    if(!c||!c->Context.Bridge||!IsCurrentConsumedPrefixNativeWindow(*c->Context.Bridge)||c->Observing)return false;
    if(owner==INDEX_NONE)
        return c->Stack.Num()==1&&c->Stack[0]==38&&c->NextCall>=39&&c->NextCall<=42&&
            c->Epoch==uint64(77+2*(c->NextCall-39))&&c->Contract->Records[38].Attempted&&!c->Contract->Records[38].Returned;
    const int32 row=39+owner;
    return owner>=0&&owner<3&&c->NextCall==row+1&&c->Stack.Num()==2&&c->Stack[0]==38&&c->Stack[1]==row&&
        c->Epoch==uint64(78+2*owner)&&c->Contract->Calls[row].Effect==EEffect::AdvanceRsPrefixOwner&&
        c->Contract->Calls[row].Owner==c->Contract->Calls[32+owner].Owner&&
        c->Contract->Records[row].Attempted&&!c->Contract->Records[row].Returned;
}
bool FCursor::IsCurrentConsumedPrefixObservation() noexcept
{
    const auto* c=Current;
    return c&&c->Observing&&c->Context.Bridge&&IsCurrentConsumedPrefixNativeWindow(*c->Context.Bridge)&&
        c->ActiveObservation&&c->ActiveObservation->Epoch==c->Epoch;
}
