// Distinct native cursor sites after the first response/synchronization prefix.
bool FCursor::IsCurrentPostSyncRsNativeWindow(const USpeedWorldSubsystem& bridge) noexcept
{
    try
    {
        const auto* c=Current;
        if(!c||c->Context.Bridge!=&bridge||!c->CurrentIdentity()||c->PositionDispatch||
            c->Contract->GetPurpose()!=OnePausedWork::EPurpose::TerminalPostSyncRsCaptureStop||
            c->Contract->Calls.Num()!=53||c->Contract->Records.Num()!=53||c->Contract->Reached||
            !c->Contract->HasCompletedNativeRsResponseSyncPrefix()||c->NextCall<47||c->NextCall>50)return false;
        if(c->Observing)
        {
            const auto* a=c->ActiveObservation;
            if(!a||a->Epoch!=c->Epoch||a->Ordinal<48||a->Ordinal>50)return false;
            const int32 row=int32(a->Ordinal)-1;
            return a->Effect==c->Contract->Calls[row].Effect&&a->Owner==c->Contract->Calls[row].Owner&&
                (a->Epoch==uint64(row*2+1)||a->Epoch==uint64(row*2+2))&&
                (row==47?a->Effect==EEffect::BeginPostSyncRsCaptureSweep:a->Effect==EEffect::SweepOwnerForPostSyncRsCapture);
        }
        if(c->Stack.Num()!=1)return false;const int32 row=c->Stack[0];
        return row>=47&&row<=49&&c->NextCall==row+1&&c->Epoch==uint64(row*2+1)&&
            c->Contract->Records[row].Attempted&&!c->Contract->Records[row].Returned;
    }
    catch(...){return false;}
}
bool FCursor::IsCurrentPostSyncRsPrelude(const USpeedWorldSubsystem& bridge) noexcept
{
    const auto* c=Current;
    return c&&IsCurrentPostSyncRsNativeWindow(bridge)&&!c->Observing&&c->NextCall==48&&
        c->Stack.Num()==1&&c->Stack[0]==47&&c->Epoch==95;
}
bool FCursor::IsCurrentPostSyncRsSweepSite(const ISpeedComponent& owner,int32 index) noexcept
{
    const auto* c=Current;if(!c||index<0||index>1||!c->Context.Bridge)return false;const int32 row=48+index;
    return IsCurrentPostSyncRsNativeWindow(*c->Context.Bridge)&&!c->Observing&&c->NextCall==row+1&&
        c->Stack.Num()==1&&c->Stack[0]==row&&c->Epoch==uint64(row*2+1)&&
        c->Contract->Calls[row].Effect==EEffect::SweepOwnerForPostSyncRsCapture&&c->Contract->Calls[row].Owner==&owner&&
        c->Contract->Records[row].Attempted&&!c->Contract->Records[row].Returned&&
        !c->Contract->Records[row].InterruptedAtPostSyncRsCapture;
}
bool FCursor::IsCurrentPostSyncRsObservation() noexcept
{
    const auto* c=Current;
    return c&&c->Observing&&c->Context.Bridge&&IsCurrentPostSyncRsNativeWindow(*c->Context.Bridge);
}
bool FCursor::EvaluateCapturedPostSyncRs(const PreparedExecution::FNativeSphereRequestPtr& request,
    const PreparedExecution::FNativeRsTraceValues& trace,PreparedExecution::FPostSyncRsMathResult& out)
{
    out={};auto* c=Current;
    if(!c||!request||!request->GetValues().Owner||
        !IsCurrentPostSyncRsSweepSite(*static_cast<const ISpeedComponent*>(request->GetValues().Owner),1)||
        !c->Contract->EvaluateCapturedPostSyncRsAction||c->Contract->PostSyncRsMathAttempted||c->Contract->PostSyncRsMathReturned||
        c->Contract->PostSyncRsCapture||request->GetSnapshot()!=c->Window->GetSnapshot()||request->GetValues().BodyIndex!=0||
        request->GetStep().QuerySerial!=2||request->GetStep().QueryEpoch!=4||request->GetStep().Iteration!=2||
        request->GetStep().WindowSerial!=c->Context.WindowSerial||request->GetStep().WindowNonce!=c->Context.Nonce||
        request->GetStep().StepSerial!=c->CurrentStep||request->GetStep().Frame!=c->Context.Frame||
        !PreparedExecution::RevalidateSphereCCD(*request))return false;
    const auto& context=c->Contract->Records[49].BeforeContext;bool accepted=false;
    {
        TGuardValue<bool> observing(c->Observing,true);TGuardValue<const FContext*> active(c->ActiveObservation,&context);
        c->Contract->PostSyncRsRequestCaptured=request;c->Contract->PostSyncRsMathAttempted=true;
        accepted=c->Contract->EvaluateCapturedPostSyncRsAction(c->Contract->SourceState.get(),context,request,trace,out,c->Contract->PostSyncRsMathActionRefusal);
        c->Contract->PostSyncRsMathReturned=true;
    }
    return accepted&&c->CurrentIdentity()&&PreparedExecution::RevalidateSphereCCD(*request);
}
void FCursor::AfterInterruptedPostSyncRsCapture(int32 i,PreparedExecution::FNativeSpherePostSyncRsCapturePtr capture)
{
    if(Current!=this||Observing||!CurrentIdentity()||Contract->GetPurpose()!=OnePausedWork::EPurpose::TerminalPostSyncRsCaptureStop||
        Contract->Calls.Num()!=53||Contract->Records.Num()!=53||i!=49||NextCall!=50||Stack.Num()!=1||Stack[0]!=49||Epoch!=99||
        !Contract->HasCompletedNativeRsResponseSyncPrefix()||Contract->PostSyncRsCapture||
        !capture||!capture->IsClosed()||!capture->GetRequest()||!Contract->PostSyncRsMathAttempted||!Contract->PostSyncRsMathReturned)
        Refuse(TEXT("Post-sync Rs interruption is not the authentic closed query2"));
    auto& row=Contract->Records[i];const auto& call=Contract->Calls[i];const auto& request=*capture->GetRequest();const auto& step=request.GetStep();
    if(!row.Attempted||row.Returned||row.Accepted||row.InterruptedAtRsCapture||row.InterruptedAtPostSyncRsCapture||
        call.Effect!=EEffect::SweepOwnerForPostSyncRsCapture||!call.Owner||call.Owner!=request.GetValues().Owner||
        request.GetValues().BodyIndex!=0||request.GetSnapshot()!=Window->GetSnapshot()||
        step.Host!=Context.Host||step.Bridge!=Context.Bridge||step.Worker!=Context.Worker||step.World!=Context.World||
        step.InputGeneration!=Context.Generation||step.Frame!=Context.Frame||step.StepSerial!=CurrentStep||
        step.WindowSerial!=Context.WindowSerial||step.WindowNonce!=Context.Nonce||step.QuerySerial!=2||step.QueryEpoch!=4||step.Iteration!=2||
        capture->GetResponseSerial()!=Contract->FirstRsResponse.Serial||capture->GetResponseData()!=Contract->FirstRsResponseData)
        Refuse(TEXT("Post-sync Rs capture changed native source/window/member/response identity"));
    Contract->PostSyncRsCapture=MoveTemp(capture);row.InterruptedAtPostSyncRsCapture=true;
    row.AfterContext=MakeContext(call,50,false);row.AfterContext.InterruptedAtPostSyncRsCapture=true;row.AfterContext.NativeReturnValue=0;
    try
    {
        row.Accepted=Observe(EObservation::InterruptedAtPostSyncRsCapture,row.AfterContext,row.After,row.Refusal)&&CurrentIdentity();
        Stack.Pop(EAllowShrinking::No);if(!row.Accepted)Refuse(TEXT("Post-sync Rs real interruption Source closure refused"));
    }
    catch(...){Contract->Failed=true;Window->Close();throw;}
}
void FEffectScope::InterruptedAtPostSyncRsCapture(PreparedExecution::FNativeSpherePostSyncRsCapturePtr capture)
{
    if(!Cursor)throw std::runtime_error("Post-sync Rs interruption has no native source effect");
    auto* cursor=Cursor;const int32 i=Record;Cursor=nullptr;Record=INDEX_NONE;cursor->AfterInterruptedPostSyncRsCapture(i,MoveTemp(capture));
}
