#include "NativePreparationPrefix.h"
#include "PreparedPhysicalExecution.h"
#include "StaticCollisionWorld.h"
#include "HostedGTStaticWorldLease.h"
#include "IAmSpeed/World/Simulation/OnePausedCanonicalWork.h"
#include "IAmSpeed/World/Simulation/SpeedPhysicsSimulation.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "HAL/PlatformTLS.h"
#include <stdexcept>
namespace Speed::PreparationPrefix
{
thread_local FCursor* FCursor::Current = nullptr;
const FPreparedNativeOwnerBinding* FindCurrentPrivateSourceObservation(const ISpeedComponent* adapter) noexcept
{
    try
    {
        const auto* c=FCursor::Current;
        if(!c||(!c->Observing&&!c->PositionDispatch)||!c->ActiveObservation||!adapter||
            c->ActiveObservation->ThreadId!=FPlatformTLS::GetCurrentThreadId()||
            c->ActiveObservation->Epoch!=c->Epoch||!c->CurrentIdentity()||
            !c->Window->GetSnapshot()->GetNativeOwners())return nullptr;
        const auto* actual=c->Window->GetSnapshot()->GetNativeOwners()->Find(adapter);
        return actual&&actual->BodyCensusCaptured&&actual->Adapter==adapter&&actual->NativeObject&&actual->Class?actual:nullptr;
    }
    catch(...){return nullptr;}
}
bool IsCurrentPrivateSourceObservation(const UActorComponent* object,
    const ISpeedComponent* adapter,const FPreparedNativeOwnerBinding& binding) noexcept
{
    const auto* actual=FindCurrentPrivateSourceObservation(adapter);
    return actual==&binding&&object&&binding.NativeObject==object;
}
[[noreturn]] void FCursor::Refuse(const TCHAR* reason)
{
    if (Contract) { Contract->Failed = true; Contract->Refusal = reason; }
    if (Window) Window->Close();
    throw std::runtime_error("Native preparation prefix refused; partial effects are not rolled back");
}
bool FCursor::CurrentIdentity() const
{
    OnePausedWork::FCanonicalFrameWindowPtr actual;
    if (!Window || !Window->IsOpen() || !Contract || !Contract->IsFor(Window->GetSnapshot(), Window->GetSerial()) ||
        Contract->Failed || !PreparedExecution::IsCurrentWorker(Context.Worker) ||
        FPlatformTLS::GetCurrentThreadId() != Context.ThreadId || !OnePausedWork::ReadExecutionWindow(actual) ||
        actual != Window || Window->GetPurpose()!=Contract->GetPurpose() ||
        (Window->GetPurpose()!=OnePausedWork::EPurpose::PreparationPrefixStop&&!OnePausedWork::HasInitialPosition(Window->GetPurpose())) ||
        Window->GetPreparationContract() != Contract || Window->GetNonce() != Context.Nonce ||
        Context.Host->GetInputWorkerGeneration() != Context.Generation ||
        Context.Host->GetWorld() != Context.World || Context.Bridge->GetWorld() != Context.World ||
        Context.Bridge->GetLastStepDiagnostics().Serial != CurrentStep ||
        !(FHostedGTStaticWorldLease::ReadPreparationHostCounters(*Context.Host)==Counters)) return false;
    return PreparedExecution::FCanonicalFramePreflight::HasUnchangedRawPreparationCensus(
        *Context.Bridge, *Window->GetSnapshot(), CurrentStep);
}
bool FCursor::Observe(EObservation kind,const FContext& context,FNativePrepareGTFields& fields,FString& reason)
{
    if(Observing)Refuse(TEXT("Preparation source observation reentrance"));
    TGuardValue<bool> observing(Observing,true);
    TGuardValue<const FContext*> observation(ActiveObservation,&context);
    return Contract->Observe(Contract->SourceState.get(),kind,context,fields,reason);
}
FCursor::FCursor(const ASpeedPhysicsSimulation& host, const USpeedWorldSubsystem& bridge,
    const FSimulationWorker* worker, uint64 generation, uint64 frame, float delta, float time)
{
    if (Current || !OnePausedWork::ReadExecutionWindow(Window) || !Window ||
        (Window->GetPurpose()!=OnePausedWork::EPurpose::PreparationPrefixStop&&!OnePausedWork::HasInitialPosition(Window->GetPurpose())))
        Refuse(TEXT("Preparation requires its actual native window and a non-nested cursor"));
    Contract = Window->GetPreparationContract();
    Context.Host = &host; Context.Bridge = &bridge; Context.Worker = worker;
    Context.World = Window->GetWorldIdentity(); Context.Generation = generation; Context.Frame = frame;
    Context.StepBefore = Window->GetStepBefore(); CurrentStep=Context.StepBefore; Context.StepCurrent=CurrentStep; Context.WindowSerial = Window->GetSerial();
    Context.Nonce = Window->GetNonce(); Context.ThreadId = FPlatformTLS::GetCurrentThreadId();
    Counters=FHostedGTStaticWorldLease::ReadPreparationHostCounters(host);
    if(Counters.Frame!=frame)Refuse(TEXT("Preparation actual canonical N differs from the immutable window"));
    Context.PublicationSerial=Counters.PublicationSerial;Context.FrameHashCount=Counters.FrameHashCount;
    FMemory::Memcpy(&Context.DeltaBits, &delta, 4); FMemory::Memcpy(&Context.TimeBits, &time, 4);
    if (!Window->Matches(&host, &bridge, bridge.GetStaticCollisionWorld(), worker, generation, frame, delta, time) ||
        !CurrentIdentity() || Contract->Started || Contract->AllPrepareReached || Contract->Reached || Contract->ScopeReturned ||
        Contract->Calls.IsEmpty() || Contract->Calls.Num() > 128) Refuse(TEXT("Preparation initial identity or one-use contract differs"));
    Contract->Started = true;
    try
    {
        // Fixed record cardinality and stack capacity are allocated before the
        // first callback. A partial/failed row is never erased or reused.
        Contract->Records.SetNum(Contract->Calls.Num()); Stack.Reserve(Contract->Calls.Num());
        FNativePrepareGTFields observed; FString reason;
        Current=this; // Initial observation is also inside the native reentrance guard.
        if (!Observe(EObservation::Initial, Context, observed, reason))
        {
            const FString failure=TEXT("Preparation initial source observation refused: ")+reason;
            Refuse(*failure);
        }
        if (!observed.Same(Contract->Initial))
        {
            const auto& expected=Contract->Initial;
            FString detail=FString::Printf(TEXT("Preparation initial GT/worker values differ: names=%d/%d words=%d/%d sections=%d/%d retained=%d/%d cameras=%d/%d"),
                observed.Names.Num(),expected.Names.Num(),observed.Words.Num(),expected.Words.Num(),
                observed.Sections.Num(),expected.Sections.Num(),observed.RetainedValues.Num(),expected.RetainedValues.Num(),
                observed.RetainedCameraEndpoints.Num(),expected.RetainedCameraEndpoints.Num());
            const int32 count=FMath::Min(FMath::Min(observed.Names.Num(),expected.Names.Num()),FMath::Min(observed.Words.Num(),expected.Words.Num()));
            for(int32 i=0;i<count;++i)
                if(observed.Names[i]!=expected.Names[i]||observed.Words[i]!=expected.Words[i])
                {detail+=FString::Printf(TEXT(" first[%d]=%s:%016llx expected=%s:%016llx"),i,*observed.Names[i],observed.Words[i],*expected.Names[i],expected.Words[i]);break;}
            Refuse(*detail);
        }
        if (!CurrentIdentity()) Refuse(TEXT("Preparation initial source values accepted but final native identity changed"));
    }
    catch (...) { if(Current==this)Current=nullptr;Contract->Failed = true; Window->Close(); throw; }
}
FContext FCursor::MakeContext(const FCall& call, uint64 ordinal, bool succeeded)
{
    if (Epoch == MAX_uint64) Refuse(TEXT("Preparation epoch overflow"));
    FContext out = Context; out.StepCurrent=CurrentStep; out.Owner = call.Owner; out.Effect = call.Effect; out.Pass = call.Pass;
    out.Ordinal = ordinal; out.Epoch = ++Epoch; out.CallbackSucceeded = succeeded; return out;
}
int32 FCursor::Before(EEffect effect, const ISpeedComponent* owner, int32 pass)
{
    if (Current != this || Observing || !CurrentIdentity() || Contract->Reached || NextCall >= Contract->Calls.Num())
        Refuse(TEXT("Preparation effect outside its current native cursor"));
    const int32 index = NextCall; const auto& expected = Contract->Calls[index];
    if (expected.Effect != effect || expected.Owner != owner || expected.Pass != pass)
        Refuse(TEXT("Preparation effect order/owner/pass differs from the closed source plan"));
    auto& row = Contract->Records[index]; row.BeforeContext = MakeContext(expected, uint64(index) + 1, false);
    try
    {
        if (!Observe(EObservation::Before, row.BeforeContext, row.Before, row.Refusal) || !CurrentIdentity())
            Refuse(TEXT("Preparation before-effect values refused"));
        // Attempted is recorded before actual dispatch, Returned only after it.
        row.Attempted = true; ++NextCall; Stack.Add(index); return index;
    }
    catch (...) { Contract->Failed = true; Window->Close(); throw; }
}
void FCursor::After(int32 index, bool succeeded, int64 nativeReturnValue)
{
    if (Current != this || Observing || Stack.IsEmpty() || Stack.Last() != index ||
        !Contract->Records.IsValidIndex(index)) Refuse(TEXT("Preparation return is not the current nested effect"));
    auto& row = Contract->Records[index]; row.Returned = true;
    // A returned callback remains Returned even if its post-effect identity is
    // invalid. Refusal is not evidence that dispatch never happened.
    if(!CurrentIdentity())Refuse(TEXT("Preparation callback returned with changed native identity/census"));
    row.AfterContext = MakeContext(Contract->Calls[index], uint64(index) + 1, succeeded);
    row.AfterContext.NativeReturnValue = nativeReturnValue;
    try
    {
        row.Accepted = Observe(EObservation::After, row.AfterContext, row.After, row.Refusal) && succeeded && CurrentIdentity();
        Stack.Pop(EAllowShrinking::No);
        if (!row.Accepted) Refuse(TEXT("Preparation returned effect/transition refused"));
    }
    catch (...) { Contract->Failed = true; Window->Close(); throw; }
}
void FCursor::Abandon(int32 index) noexcept
{
    if (!Contract) return;
    Contract->Failed = true; Contract->Refusal = TEXT("Preparation effect did not return normally; no rollback inferred");
    if (Contract->Records.IsValidIndex(index)) Contract->Records[index].Refusal = Contract->Refusal;
    if (Window) Window->Close();
}
bool ReadInitialPositionObservationContext(const ISpeedComponent* owner,const USSubBody* body,FContext& out) noexcept
{
    out={};try
    {
        const auto* c=FCursor::Current;
        if(!c||!c->Observing||c->PositionDispatch||!c->ActiveObservation||!owner||!body||!c->CurrentIdentity()||
            !OnePausedWork::HasInitialPosition(c->Contract->GetPurpose())||
            c->ActiveObservation->Effect!=EEffect::ApplyInitialPosition||c->ActiveObservation->Owner!=owner||
            c->ActiveObservation->Ordinal!=31||c->ActiveObservation->Epoch!=c->Epoch)return false;
        const auto* binding=FindCurrentPrivateSourceObservation(owner);
        if(!binding||binding->Bodies.Num()!=1||binding->Bodies[0].Index!=0||binding->Bodies[0].Body!=body)return false;
        out=*c->ActiveObservation;return true;
    }
    catch(...){return false;}
}
bool ReadInitialPositionMutationContext(const ISpeedComponent* owner,const USSubBody* body,FContext& out) noexcept
{
    out={};try
    {
        const auto* c=FCursor::Current;
        if(!c||c->Observing||!c->PositionDispatch||!c->ActiveObservation||!owner||!body||!c->CurrentIdentity()||
            !OnePausedWork::HasInitialPosition(c->Contract->GetPurpose())||
            c->NextCall!=31||c->Stack.Num()!=1||c->Stack[0]!=30||c->Contract->Records.Num()!=OnePausedWork::ExpectedNativeCallCount(c->Contract->GetPurpose())||
            !c->Contract->PositionAttempted||c->Contract->PositionReturned||
            !c->Contract->Records[30].Attempted||c->Contract->Records[30].Returned||
            c->ActiveObservation!=&c->Contract->Records[30].BeforeContext||
            c->ActiveObservation->Effect!=EEffect::ApplyInitialPosition||c->ActiveObservation->Owner!=owner||
            c->ActiveObservation->Ordinal!=31||c->ActiveObservation->Epoch!=c->Epoch)return false;
        const auto* binding=FindCurrentPrivateSourceObservation(owner);
        if(!binding||binding->Bodies.Num()!=1||binding->Bodies[0].Index!=0||binding->Bodies[0].Body!=body)return false;
        out=*c->ActiveObservation;return true;
    }
    catch(...){return false;}
}
bool FCursor::ApplyPostPrepareInitialPosition(const ASpeedPhysicsSimulation& host)
{
    auto* c=Current;
    if(!c||c->Context.Host!=&host||c->Observing||c->PositionDispatch||!c->CurrentIdentity()||
        !OnePausedWork::HasInitialPosition(c->Contract->GetPurpose())||
        !c->Contract->PositionAction||c->Contract->PositionAttempted||c->Contract->PositionReturned||c->Contract->PositionAccepted||
        c->NextCall!=30||!c->Stack.IsEmpty()||
        c->Contract->Calls.Num()!=OnePausedWork::ExpectedNativeCallCount(c->Contract->GetPurpose())||
        c->Contract->Records.Num()!=c->Contract->Calls.Num()||
        c->Contract->Reached||c->Contract->AllPrepareReached)return false;
    for(int32 i=0;i<30;++i)if(!c->Contract->Records[i].Attempted||!c->Contract->Records[i].Returned||!c->Contract->Records[i].Accepted)return false;
    c->Contract->PositionBoundaryReached=true;
    const auto& call=c->Contract->Calls[30];
    if(call.Effect!=EEffect::ApplyInitialPosition||!call.Owner||call.Pass!=INDEX_NONE)return false;
    const auto* binding=c->Window->GetSnapshot()->GetNativeOwners()->Find(call.Owner);
    if(!binding||!binding->BodyCensusCaptured||binding->Bodies.Num()!=1||binding->Bodies[0].Index!=0||!binding->Bodies[0].Body)return false;
    // Before installs ordinal31/epoch61 and the Source expectation first.
    const int32 index=c->Before(call.Effect,call.Owner,call.Pass);
    if(index!=30)c->Refuse(TEXT("Position dispatch is not the sealed native ordinal"));
    FString actionReason;
    try
    {
        bool returned=false;
        {
            TGuardValue<bool> dispatch(c->PositionDispatch,true);
            TGuardValue<const FContext*> observation(c->ActiveObservation,&c->Contract->Records[30].BeforeContext);
            c->Contract->PositionAttempted=true;
            FContext actual;
            if(!ReadInitialPositionMutationContext(call.Owner,binding->Bodies[0].Body,actual))c->Refuse(TEXT("Position mutation context unavailable before action"));
            returned=c->Contract->PositionAction(c->Contract->SourceState.get(),actual,actionReason);
            c->Contract->PositionReturned=true;
        }
        if(!returned)c->Contract->PositionActionRefusal=actionReason;
        c->After(index,returned,returned?1:0);
        c->Contract->PositionAccepted=true;return true;
    }
    catch(...)
    {
        c->Contract->Failed=true;c->Window->Close();
        if(!c->Contract->Records[30].Returned)c->Abandon(index);
        if(!actionReason.IsEmpty())
        {
            c->Contract->PositionActionRefusal=actionReason;
            auto& row=c->Contract->Records[30];
            if(!row.Refusal.IsEmpty())row.Refusal+=TEXT("; ");row.Refusal+=actionReason;
            if(!c->Contract->Refusal.IsEmpty())c->Contract->Refusal+=TEXT("; ");c->Contract->Refusal+=actionReason;
        }
        throw;
    }
}
bool FCursor::IsTerminalStepSyncEntry(const USpeedWorldSubsystem& bridge) noexcept
{
    try
    {
        const auto* c=Current;
        if(!c||c->Context.Bridge!=&bridge||!c->CurrentIdentity()||c->Observing||c->PositionDispatch||
            !OnePausedWork::HasNativeStepPrefix(c->Contract->GetPurpose())||!c->Contract->HasAcceptedPosition()||
            c->Contract->StepSerialConfirmed||c->Contract->StepSyncReached||c->CurrentStep!=c->Context.StepBefore||
            c->NextCall!=32||c->Stack.Num()!=1||c->Stack[0]!=31||c->Contract->Records.Num()!=OnePausedWork::ExpectedNativeCallCount(c->Contract->GetPurpose()))return false;
        for(int32 i=0;i<31;++i)if(!c->Contract->Records[i].Accepted)return false;
        const auto& row=c->Contract->Records[31];
        return row.Attempted&&!row.Returned&&row.BeforeContext.StepCurrent==c->Context.StepBefore;
    }
    catch(...){return false;}
}
bool FCursor::ConfirmTerminalStepSerial(const USpeedWorldSubsystem& bridge) noexcept
{
    // Called ONLY immediately after the real ++StepSerial and the worker's
    // confirmation. The old identity deliberately cannot accept either serial.
    auto* c=Current;
    if(!c||!OnePausedWork::HasNativeStepPrefix(c->Contract->GetPurpose()))return true;
    try
    {
        OnePausedWork::FCanonicalFrameWindowPtr actual;
        if(c->Context.Bridge!=&bridge||c->Observing||c->PositionDispatch||c->Contract->StepSerialConfirmed||
            c->Contract->StepSyncReached||c->CurrentStep!=c->Context.StepBefore||c->NextCall!=32||
            c->Stack.Num()!=1||c->Stack[0]!=31||!c->Contract->Records[31].Attempted||c->Contract->Records[31].Returned||
            !OnePausedWork::ReadExecutionWindow(actual)||actual!=c->Window||
            !OnePausedWork::HasConfirmedTerminalStepSerial(&bridge,c->Window->GetExpectedStep())||
            bridge.GetLastStepDiagnostics().Serial!=c->Window->GetExpectedStep()||
            !PreparedExecution::FCanonicalFramePreflight::HasUnchangedRawPreparationCensus(bridge,*c->Window->GetSnapshot(),c->Window->GetExpectedStep()))return false;
        c->CurrentStep=c->Window->GetExpectedStep();c->Contract->StepSerialConfirmed=true;
        return c->CurrentIdentity();
    }
    catch(...){return false;}
}
bool FCursor::FinishTerminalStepSync(const USpeedWorldSubsystem& bridge)
{
    auto* c=Current;
    if(!c||!OnePausedWork::HasNativeStepPrefix(c->Contract->GetPurpose()))return false;
    if(c->Context.Bridge!=&bridge||!c->CurrentIdentity()||c->Observing||c->PositionDispatch||
        !c->Contract->HasAcceptedPosition()||!c->Contract->StepSerialConfirmed||c->Contract->StepSyncReached||
        c->CurrentStep!=c->Window->GetExpectedStep()||c->NextCall!=35||!c->Stack.IsEmpty()||c->Contract->Records.Num()!=OnePausedWork::ExpectedNativeCallCount(c->Contract->GetPurpose()))
        c->Refuse(TEXT("Native terminal Step sync did not reach its declared boundary"));
    for(int32 i=0;i<35;++i)if(!c->Contract->Records[i].Attempted||!c->Contract->Records[i].Returned||!c->Contract->Records[i].Accepted)
        c->Refuse(TEXT("Native terminal Step sync has incomplete actual effect evidence"));
    c->Contract->StepSyncReached=true;return c->Contract->GetPurpose()==OnePausedWork::EPurpose::TerminalStepSyncStop;
}
bool FCursor::Finish(const ASpeedPhysicsSimulation& host)
{
    auto* cursor = Current;
    if (!cursor || cursor->Context.Host != &host) return false;
    auto& c = *cursor;
    if (!c.CurrentIdentity() || !c.Stack.IsEmpty() || c.NextCall != c.Contract->Calls.Num() || !c.Contract->AllPrepareReached || c.Contract->Reached)
        c.Refuse(TEXT("Preparation stop before all planned native effects returned"));
    if(OnePausedWork::HasInitialPosition(c.Contract->GetPurpose())&&!c.Contract->HasAcceptedPosition())
        c.Refuse(TEXT("Position stop lacks an accepted native action"));
    for(int32 i=0;i<c.Contract->Records.Num();++i)
        if(!c.IsClosedPlannedRow(i))c.Refuse(TEXT("Preparation stop has incomplete actual effect closure evidence"));
    if(c.Contract->GetPurpose()==OnePausedWork::EPurpose::TerminalStepSyncStop&&!c.Contract->HasCompletedStepSync())
        c.Refuse(TEXT("Step sync stop lacks the real serial/reset boundary"));
    if(c.Contract->GetPurpose()==OnePausedWork::EPurpose::TerminalSphereRsCaptureStop&&!c.Contract->HasClosedNativeRsCapture())
        c.Refuse(TEXT("Rs capture stop lacks its authentic closed native interruption"));
    if(c.Contract->GetPurpose()==OnePausedWork::EPurpose::TerminalConsumedPrefixAdvanceStop&&!c.Contract->HasCompletedNativeRsAdvance())
        c.Refuse(TEXT("Consumed-prefix stop lacks complete real owner integration and Source closure"));
    if(c.Contract->GetPurpose()==OnePausedWork::EPurpose::TerminalRsContactAdvanceStop&&!c.Contract->HasCompletedNativeRsContactAdvance())
        c.Refuse(TEXT("Rs contact-advance stop lacks complete real owner integration and Source closure"));
    if(c.Contract->GetPurpose()==OnePausedWork::EPurpose::TerminalRsResponseStop&&!c.Contract->HasCompletedNativeRsResponse())
        c.Refuse(TEXT("Rs response stop lacks complete real response and Source closure"));
    if(c.Contract->GetPurpose()==OnePausedWork::EPurpose::TerminalRsResponseSyncStop&&!c.Contract->HasCompletedNativeRsResponseSync())
        c.Refuse(TEXT("Rs response-sync stop lacks complete real synchronization and Source closure"));
    if(c.Contract->GetPurpose()==OnePausedWork::EPurpose::TerminalPostSyncRsCaptureStop&&!c.Contract->HasCompletedNativePostSyncRsCapture())
        c.Refuse(TEXT("Post-sync Rs stop lacks authentic second capture/response-prefix closure"));
    FContext final = c.Context; final.StepCurrent=c.CurrentStep; final.Ordinal = uint64(c.NextCall) + 1;
    if (c.Epoch == MAX_uint64) c.Refuse(TEXT("Preparation final epoch overflow"));
    final.Epoch = ++c.Epoch;
    try
    {
        // The closed game observer must reread ALL owners here, after ALL
        // Prepare calls and restoration of subsystem diagnostic scope values.
        if (!c.Observe(EObservation::Stop, final, c.Contract->Final, c.Contract->Refusal) || !c.CurrentIdentity())
            c.Refuse(TEXT("Preparation complete all-owner final values refused"));
        c.Contract->Reached = true;
        if (!(c.Contract->GetPurpose()==OnePausedWork::EPurpose::PostPrepareInitialPositionStop?
            OnePausedWork::SealPostPrepareInitialPositionStop(&host):
            c.Contract->GetPurpose()==OnePausedWork::EPurpose::TerminalPostSyncRsCaptureStop?
            OnePausedWork::SealTerminalPostSyncRsCaptureStop(&host):
            c.Contract->GetPurpose()==OnePausedWork::EPurpose::TerminalRsResponseSyncStop?
            OnePausedWork::SealTerminalRsResponseSyncStop(&host):
            c.Contract->GetPurpose()==OnePausedWork::EPurpose::TerminalRsResponseStop?
            OnePausedWork::SealTerminalRsResponseStop(&host):
            c.Contract->GetPurpose()==OnePausedWork::EPurpose::TerminalStepSyncStop?
            OnePausedWork::SealTerminalStepSyncStop(&host):
            c.Contract->GetPurpose()==OnePausedWork::EPurpose::TerminalConsumedPrefixAdvanceStop?
            OnePausedWork::SealTerminalConsumedPrefixAdvanceStop(&host):
            c.Contract->GetPurpose()==OnePausedWork::EPurpose::TerminalRsContactAdvanceStop?
            OnePausedWork::SealTerminalRsContactAdvanceStop(&host):
            c.Contract->GetPurpose()==OnePausedWork::EPurpose::TerminalSphereRsCaptureStop?
            OnePausedWork::SealTerminalSphereRsCaptureStop(&host):OnePausedWork::SealPreparationStop(&host))) c.Refuse(TEXT("Preparation native terminal marker refused"));
        return true;
    }
    catch (...) { c.Contract->Failed = true; c.Window->Close(); throw; }
}
bool FCursor::IsStoppedForCurrentCall(const ASpeedPhysicsSimulation* host) noexcept
{
    return Current && Current->Context.Host == host && Current->Contract &&
        Current->Contract->Reached && !Current->Contract->Failed && Current->Stack.IsEmpty() &&
        Current->NextCall == Current->Contract->Calls.Num() && Current->CurrentIdentity();
}
FCursor::~FCursor() noexcept
{
    if (!Contract) return;
    if (Current != this || !Contract->Reached || Contract->Failed || !Stack.IsEmpty() || !CurrentIdentity() ||
        FPlatformTLS::GetCurrentThreadId() != Context.ThreadId)
    { Contract->Failed = true; if (Window) Window->Close(); }
    else Contract->ScopeReturned = true;
    if (Current == this) Current = nullptr;
}
void FEffectScope::VerifyStructuralBoundary()
{
    if (FCursor::Current && (FCursor::Current->Observing || !FCursor::Current->CurrentIdentity()))
        FCursor::Current->Refuse(TEXT("Preparation pending/order/census changed before structural effects"));
}
FEffectScope::FEffectScope(EEffect effect, const ISpeedComponent* owner, int32 pass)
{
    Cursor = FCursor::Current;
    if (Cursor) Record = Cursor->Before(effect, owner, pass);
}
void FEffectScope::Returned(bool succeeded, int64 nativeReturnValue)
{
    if (!Cursor) return;
    auto* cursor = Cursor; const int32 index = Record;
    // After may throw; it has already recorded the actual callback's return.
    Cursor = nullptr; Record = INDEX_NONE; cursor->After(index, succeeded, nativeReturnValue);
}
FEffectScope::~FEffectScope() noexcept { if (Cursor) Cursor->Abandon(Record); }
#include "NativeRsCapturePrefix.inl"
#include "NativeFirstRsResponsePrefix.inl"
#include "NativePostSyncRsCapturePrefix.inl"

}
