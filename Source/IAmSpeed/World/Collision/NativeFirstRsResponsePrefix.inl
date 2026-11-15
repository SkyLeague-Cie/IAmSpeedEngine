// Included in NativePreparationPrefix.cpp's namespace. The p7 mutation sites
// remain disjoint from capture, ordinary ReadCurrentStep and p5/p6 advance.
bool FCursor::IsCurrentRsResponseNativeWindow(const USpeedWorldSubsystem& bridge) noexcept
{
    try
    {
        const auto* c=Current;
        if(!c||c->Context.Bridge!=&bridge||!c->CurrentIdentity()||c->PositionDispatch||
            !OnePausedWork::HasFirstRsResponsePrefix(c->Contract->GetPurpose())||
            c->Contract->Calls.Num()!=OnePausedWork::ExpectedNativeCallCount(c->Contract->GetPurpose())||
            c->Contract->Records.Num()!=c->Contract->Calls.Num()||!c->Contract->HasClosedNativeRsCapture()||
            !c->Contract->RsCapture->HasBoundedFirstArrivalRsQuality()||c->Contract->Reached||c->NextCall<42||c->NextCall>46)return false;
        if(c->Observing)
        {
            const auto* a=c->ActiveObservation;
            return a&&a->Epoch==c->Epoch&&a->Ordinal>=43&&a->Ordinal<=46&&
                (a->Effect==EEffect::ResponseRsContact||a->Effect==EEffect::SetRsFutureHit||
                 a->Effect==EEffect::AcceptRsHit||a->Effect==EEffect::ResolveRsHit);
        }
        return !c->Stack.IsEmpty()&&c->Stack[0]==42;
    }
    catch(...){return false;}
}
bool FCursor::IsCurrentRsResponseSite(int32 call) noexcept
{
    const auto* c=Current;
    if(!c||!c->Context.Bridge||!IsCurrentRsResponseNativeWindow(*c->Context.Bridge)||c->Observing)return false;
    if(call==INDEX_NONE)
        return c->Stack.Num()==1&&c->Stack[0]==42&&c->NextCall>=43&&c->NextCall<=46&&
            c->Epoch==uint64(85+2*(c->NextCall-43))&&c->Contract->Records[42].Attempted&&!c->Contract->Records[42].Returned;
    const int32 row=43+call;
    if(call<0||call>2||c->NextCall!=row+1||c->Stack.Num()!=2||c->Stack[0]!=42||c->Stack[1]!=row||
        c->Epoch!=uint64(86+2*call)||c->Contract->Calls[row].Owner!=c->Contract->Calls[33].Owner||
        !c->Contract->Records[row].Attempted||c->Contract->Records[row].Returned)return false;
    const EEffect effects[3]={EEffect::SetRsFutureHit,EEffect::AcceptRsHit,EEffect::ResolveRsHit};
    return c->Contract->Calls[row].Effect==effects[call];
}
bool FCursor::IsCurrentRsResponseObservation() noexcept
{
    const auto* c=Current;
    return c&&c->Observing&&c->Context.Bridge&&IsCurrentRsResponseNativeWindow(*c->Context.Bridge)&&
        c->ActiveObservation&&c->ActiveObservation->Epoch==c->Epoch;
}


bool FCursor::CaptureFirstRsResponsePartial(int32 call,TSharedPtr<const FNativePrepareGTFields,ESPMode::ThreadSafe>& out,FString& reason) noexcept
{
    out.Reset();reason.Reset();
    try
    {
        auto* c=Current;
        if(!c||call<0||call>2||!IsCurrentRsResponseSite(call)||!c->Contract->ReadRsResponsePartialAction||
            c->Contract->FirstRsResponse.ActiveCall!=call)
        {reason=TEXT("Partial Core Source context unavailable; no state inferred");return false;}
        const auto& context=c->Contract->Records[43+call].BeforeContext;
        FNativePrepareGTFields fields;bool captured=false;
        {
            TGuardValue<bool> observing(c->Observing,true);
            TGuardValue<const FContext*> active(c->ActiveObservation,&context);
            captured=c->Contract->ReadRsResponsePartialAction(c->Contract->SourceState.get(),context,fields,reason);
        }
        if(!captured)
        {
            if(reason.IsEmpty())reason=TEXT("Bounded partial Core Source reader refused; no values inferred");
            return false;
        }
        auto retained=MakeShared<FNativePrepareGTFields,ESPMode::ThreadSafe>(MoveTemp(fields));out=MoveTemp(retained);return true;
    }
    catch(...){reason=TEXT("Partial Core Source read threw; no state inferred");return false;}
}

// The p8 site is disjoint from the already closed three-call response ticket.
bool FCursor::IsCurrentRsResponseSyncNativeWindow(const USpeedWorldSubsystem& bridge) noexcept
{
    try
    {
        const auto* c=Current;
        if(!c||c->Context.Bridge!=&bridge||!c->CurrentIdentity()||c->PositionDispatch||
            !OnePausedWork::HasFirstRsResponseSyncPrefix(c->Contract->GetPurpose())||
            c->Contract->Calls.Num()!=OnePausedWork::ExpectedNativeCallCount(c->Contract->GetPurpose())||c->Contract->Records.Num()!=c->Contract->Calls.Num()||c->Contract->Reached||
            !c->Contract->HasCompletedNativeRsResponsePrefix()||c->NextCall<46||c->NextCall>47)return false;
        if(c->Observing)
        {
            const auto* a=c->ActiveObservation;
            return a&&a->Epoch==c->Epoch&&a->Ordinal==47&&(a->Epoch==93||a->Epoch==94)&&
                a->Effect==EEffect::SyncRsResponseOwner&&a->Owner==c->Contract->Calls[33].Owner;
        }
        return c->NextCall==47&&c->Stack.Num()==1&&c->Stack[0]==46&&c->Epoch==93&&
            c->Contract->Records[46].Attempted&&!c->Contract->Records[46].Returned;
    }
    catch(...){return false;}
}
bool FCursor::IsCurrentRsResponseSyncSite() noexcept
{
    const auto* c=Current;
    return c&&!c->Observing&&c->Context.Bridge&&IsCurrentRsResponseSyncNativeWindow(*c->Context.Bridge);
}
bool FCursor::CaptureFirstRsResponseSyncPartial(TSharedPtr<const FNativePrepareGTFields,ESPMode::ThreadSafe>& out,FString& reason) noexcept
{
    out.Reset();reason.Reset();
    try
    {
        auto* c=Current;
        if(!c||!IsCurrentRsResponseSyncSite()||!c->Contract->ReadRsResponsePartialAction||!c->Contract->FirstRsResponseSync.Attempted)
        {reason=TEXT("Partial Core sync Source context unavailable");return false;}
        const auto& context=c->Contract->Records[46].BeforeContext;FNativePrepareGTFields fields;bool captured=false;
        {
            TGuardValue<bool> observing(c->Observing,true);TGuardValue<const FContext*> active(c->ActiveObservation,&context);
            captured=c->Contract->ReadRsResponsePartialAction(c->Contract->SourceState.get(),context,fields,reason);
        }
        if(!captured)return false;
        out=MakeShared<FNativePrepareGTFields,ESPMode::ThreadSafe>(MoveTemp(fields));return true;
    }
    catch(...){reason=TEXT("Partial Core sync Source read threw");return false;}
}
