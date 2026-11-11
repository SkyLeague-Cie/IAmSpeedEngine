#pragma once
#include "CoreMinimal.h"
#include "IAmSpeed/World/Simulation/NativePreparationPurpose.h"
#include "IAmSpeed/World/Collision/NativePrepareGTInspection.h"
#include "IAmSpeed/World/Collision/PreparedWorldSnapshot.h"
#include "PreparedSphereRsTrajectoryCapture.h"
#include "PreparedSpherePostSyncRsCapture.h"
#include <atomic>
#include <memory>
class ASpeedPhysicsSimulation;
class USpeedWorldSubsystem;
class FSimulationWorker;
class USSubBody;
namespace Speed
{
class FHostedGTStaticWorldLease;
namespace PreparedExecution { class FCanonicalFramePreflight; class FStepScope; class FSphereMemberSweepScope; }
namespace OnePausedWork
{
class FCanonicalFrameWindow;

}
namespace PreparationPrefix
{
enum class EEffect : uint8
{
    AdmitCanonicalFrame, ValidateOwnerBinding, StageScenarioInput,
    PrepareInputRegistry, InstallInputRegistry, InstallOwnerInput, BeginInputRegistry,
    BeginStaticAudit, ApplyInputJournal, PrepareOwnerInput, ClearInputError,
    PrepareWorld, ReadPrescribedMotion, BindStaticWorld, PrepareOwner, RetireOwnerInput,
    ApplyInitialPosition, EnterTerminalStep, SyncStepOwner, BeginRsCaptureSweep, SweepOwnerForRsCapture, AdvanceRsPrefix, AdvanceRsPrefixOwner, ResponseRsContact, SetRsFutureHit, AcceptRsHit, ResolveRsHit, SyncRsResponseOwner, BeginPostSyncRsCaptureSweep, SweepOwnerForPostSyncRsCapture
};
static_assert(uint8(EEffect::RetireOwnerInput)==15&&uint8(EEffect::ApplyInitialPosition)==16&&uint8(EEffect::EnterTerminalStep)==17&&uint8(EEffect::SyncStepOwner)==18,"Native effect wire values must stay stable");
static_assert(uint8(EEffect::SyncRsResponseOwner)==27,"Response synchronization effect wire value must stay27");
static_assert(uint8(EEffect::BeginPostSyncRsCaptureSweep)==28&&uint8(EEffect::SweepOwnerForPostSyncRsCapture)==29,"Post-sync effect wire values must stay28/29");
enum class EObservation : uint8 { Initial, Before, After, Stop, InterruptedAtRsCapture, InterruptedAtPostSyncRsCapture };
struct FCall
{
    EEffect Effect;
    const ISpeedComponent* Owner = nullptr;
    int32 Pass = INDEX_NONE;
};
struct FHostCounters
{
    uint64 Frame=0,PublicationSerial=0;
    int32 FrameHashCount=0;
    bool operator==(const FHostCounters& x) const
    {return Frame==x.Frame&&PublicationSerial==x.PublicationSerial&&FrameHashCount==x.FrameHashCount;}
};
struct FContext
{
    const ASpeedPhysicsSimulation* Host = nullptr;
    const USpeedWorldSubsystem* Bridge = nullptr;
    const FSimulationWorker* Worker = nullptr;
    const void* World = nullptr;
    const ISpeedComponent* Owner = nullptr;
    uint64 Generation = 0, Frame = 0, StepBefore = 0, WindowSerial = 0, Nonce = 0;
    uint64 Ordinal = 0, Epoch = 0, StepCurrent = 0;
    uint32 ThreadId = 0, DeltaBits = 0, TimeBits = 0;
    EEffect Effect = EEffect::AdmitCanonicalFrame;
    int32 Pass = INDEX_NONE;
    uint64 PublicationSerial = 0;
    int32 FrameHashCount = 0;
    bool CallbackSucceeded = false;
    bool InterruptedAtRsCapture = false, InterruptedAtPostSyncRsCapture = false;
    int64 NativeReturnValue = 0;
};
struct FEffectRecord
{
    FContext BeforeContext, AfterContext;
    FNativePrepareGTFields Before, After;
    bool Attempted = false, Returned = false, Accepted = false, InterruptedAtRsCapture = false, InterruptedAtPostSyncRsCapture = false;
    FString Refusal;
};
class FCursor;
// The actual game translation unit must define this ONE closed producer after
// authenticating every reached compiled callback, native owner, input and value.
// There is deliberately no public factory or subclassable always-true program.
class FClosedSourceProducer;
class FContract final
{
    friend class FClosedSourceProducer;
    friend class Speed::PreparedExecution::FStepScope;
    friend class Speed::PreparedExecution::FSphereMemberSweepScope;
    friend class FCursor;
    friend IAMSPEED_API bool ReadInitialPositionMutationContext(const ISpeedComponent*,const USSubBody*,FContext&) noexcept;
    friend class Speed::FHostedGTStaticWorldLease;
    friend class ::USpeedWorldSubsystem;
    using FObserve = bool (*)(const void*, EObservation, const FContext&,
        FNativePrepareGTFields&, FString&);
    const FPreparedWorldSnapshotPtr Snapshot;
    const std::shared_ptr<const void> SourceState;
    using FPositionAction = bool (*)(const void*,const FContext&,FString&);
    const FObserve Observe;
    const FPositionAction PositionAction;
    using FEvaluateCapturedRs=bool (*)(const void*,const FContext&,const PreparedExecution::FNativeSphereRequestPtr&,
        const PreparedExecution::FNativeRsTraceValues&,PreparedExecution::FCapturedRsMathResult&,FString&);
    const FEvaluateCapturedRs EvaluateCapturedRsAction;
    using FEvaluateCapturedPostSyncRs=bool (*)(const void*,const FContext&,const PreparedExecution::FNativeSphereRequestPtr&,
        const PreparedExecution::FNativeRsTraceValues&,PreparedExecution::FPostSyncRsMathResult&,FString&);
    const FEvaluateCapturedPostSyncRs EvaluateCapturedPostSyncRsAction;
    using FReadPostSyncRsMathPartial=const PreparedExecution::FPostSyncRsMathResult* (*)(const void*) noexcept;
    const FReadPostSyncRsMathPartial ReadPostSyncRsMathPartialAction;
    using FReadRsResponsePartial=bool (*)(const void*,const FContext&,FNativePrepareGTFields&,FString&);
    const FReadRsResponsePartial ReadRsResponsePartialAction;
    const OnePausedWork::EPurpose Purpose;
    const FNativePrepareGTFields Initial;
    const TArray<FCall> Calls;
    mutable std::atomic<uint64> ClaimedWindow{0};
    mutable TArray<FEffectRecord> Records;
    mutable FNativePrepareGTFields Final;
    mutable FString Refusal;
    mutable std::atomic<bool> Started{false}, AllPrepareReached{false}, Reached{false}, Failed{false}, ScopeReturned{false};
    mutable bool Joined = false, OwnersRetired = false;
    mutable bool PositionBoundaryReached=false,PositionAttempted=false,PositionReturned=false,PositionAccepted=false;
    mutable FString PositionActionRefusal;
    mutable bool StepSerialConfirmed=false,StepSyncReached=false;
    mutable bool RsMathAttempted=false,RsMathReturned=false;
    mutable FString RsMathActionRefusal;
    mutable PreparedExecution::FNativeSphereRsTrajectoryCapturePtr RsCapture;
    mutable PreparedExecution::FNativeSphereRequestPtr RsRequestCaptured;
    mutable PreparedExecution::FTerminalConsumedPrefixAdvanceObservation RsAdvance;
    mutable PreparedExecution::FFirstRsResponseDataPtr FirstRsResponseData;
    mutable PreparedExecution::FTerminalFirstRsResponseObservation FirstRsResponse;
    mutable PreparedExecution::FTerminalFirstRsResponseSyncObservation FirstRsResponseSync;
    mutable bool PostSyncRsMathAttempted=false,PostSyncRsMathReturned=false;
    // Raw diagnostic ownership only. Neither slot qualifies a closed capture.
    mutable PreparedExecution::FPostSyncRsMathResult PostSyncRsUnqualifiedMath;
    mutable bool PostSyncRsUnqualifiedMathPresent=false;
    mutable PreparedExecution::FNativeSpherePostSyncRsCapturePtr PostSyncRsDiagnosticCapture;
    mutable FString PostSyncRsMathActionRefusal;
    mutable PreparedExecution::FNativeSpherePostSyncRsCapturePtr PostSyncRsCapture;
    mutable PreparedExecution::FNativeSphereRequestPtr PostSyncRsRequestCaptured;
    bool HasCompletedNativeRsResponseSyncPrefix() const noexcept
    {
        const auto& sync=FirstRsResponseSync;
        return OnePausedWork::HasFirstRsResponseSyncPrefix(Purpose)&&Records.Num()==OnePausedWork::ExpectedNativeCallCount(Purpose)&&HasCompletedNativeRsResponsePrefix()&&
            FirstRsResponseData&&sync.Data==FirstRsResponseData&&
            sync.ResponseSerial==FirstRsResponse.Serial&&sync.NumericControl==FirstRsResponse.NumericControl&&
            sync.Attempted&&sync.Returned&&sync.CaptureAttempted&&sync.RawCaptured&&sync.SourceCaptured&&sync.SourceBallFields&&sync.Verified&&sync.Complete&&!sync.Failed&&
            sync.Owners.Num()==3&&sync.Bodies.Num()==3&&Records[46].Attempted&&Records[46].Returned&&Records[46].Accepted;
    }
    FContract(FPreparedWorldSnapshotPtr snapshot, std::shared_ptr<const void> source,
        FObserve observe, FNativePrepareGTFields&& initial, TArray<FCall>&& calls,
        OnePausedWork::EPurpose purpose=OnePausedWork::EPurpose::PreparationPrefixStop,FPositionAction positionAction=nullptr,FEvaluateCapturedRs rsAction=nullptr,FReadRsResponsePartial responsePartial=nullptr,FEvaluateCapturedPostSyncRs postSyncAction=nullptr,FReadPostSyncRsMathPartial postSyncPartial=nullptr)
        : Snapshot(MoveTemp(snapshot)), SourceState(std::move(source)), Observe(observe),
          PositionAction(positionAction), EvaluateCapturedRsAction(rsAction), EvaluateCapturedPostSyncRsAction(postSyncAction), ReadPostSyncRsMathPartialAction(postSyncPartial), ReadRsResponsePartialAction(responsePartial), Purpose(purpose), Initial(MoveTemp(initial)), Calls(MoveTemp(calls)) {}
    bool ClaimWindow(const FPreparedWorldSnapshotPtr& snapshot, uint64 serial) const
    {
        if (!IsInGameThread() || !Snapshot || Snapshot != snapshot || !SourceState || !Observe ||
            Calls.IsEmpty() || Calls.Num() > 128 || !serial) return false;
        if(OnePausedWork::HasSphereRsCapturePrefix(Purpose)!=bool(EvaluateCapturedRsAction))return false;
        if(OnePausedWork::HasFirstRsResponsePrefix(Purpose)!=bool(ReadRsResponsePartialAction))return false;
        const bool postSync=Purpose==OnePausedWork::EPurpose::TerminalPostSyncRsCaptureStop;
        if(postSync!=bool(EvaluateCapturedPostSyncRsAction)||postSync!=bool(ReadPostSyncRsMathPartialAction))return false;
        for(int32 i=0;i<Calls.Num();++i)
        {
            const bool prelude=Calls[i].Effect==EEffect::BeginPostSyncRsCaptureSweep;
            const bool sweep=Calls[i].Effect==EEffect::SweepOwnerForPostSyncRsCapture;
            if((prelude||sweep)&&!postSync)return false;
            if(postSync)
            {
                if(i==47&&(Calls[i].Effect!=EEffect::BeginPostSyncRsCaptureSweep||Calls[i].Owner||Calls[i].Pass!=INDEX_NONE))return false;
                if((i==48||i==49)&&(Calls[i].Effect!=EEffect::SweepOwnerForPostSyncRsCapture||Calls[i].Owner!=Calls[i-16].Owner||Calls[i].Pass!=INDEX_NONE))return false;
                if((prelude&&i!=47)||(sweep&&i!=48&&i!=49))return false;
            }
        }
        if(Purpose==OnePausedWork::EPurpose::PreparationPrefixStop)
        {if(PositionAction)return false;}
        else if(Purpose==OnePausedWork::EPurpose::PostPrepareInitialPositionStop)
        {
            if(!PositionAction||Calls.Num()!=34||Calls[30].Effect!=EEffect::ApplyInitialPosition||
                !Calls[30].Owner||Calls[30].Pass!=INDEX_NONE)return false;
            for(int32 i=0;i<Calls.Num();++i)if(i!=30&&Calls[i].Effect==EEffect::ApplyInitialPosition)return false;
        }
        else if(OnePausedWork::HasNativeStepPrefix(Purpose))
        {
            if(!PositionAction||Calls.Num()!=OnePausedWork::ExpectedNativeCallCount(Purpose)||Calls[30].Effect!=EEffect::ApplyInitialPosition||!Calls[30].Owner||
                Calls[30].Pass!=INDEX_NONE||Calls[31].Effect!=EEffect::EnterTerminalStep||Calls[31].Owner||Calls[31].Pass!=INDEX_NONE)return false;
            for(int32 i=0;i<Calls.Num();++i)
            {
                const bool rsResponse=OnePausedWork::HasFirstRsResponsePrefix(Purpose);
                const bool rsSync=OnePausedWork::HasFirstRsResponseSyncPrefix(Purpose);
                if(Calls[i].Effect==EEffect::SyncRsResponseOwner&&(!rsSync||i!=46||Calls[i].Owner!=Calls[33].Owner||Calls[i].Pass!=INDEX_NONE))return false;
                if(rsSync&&i==46&&Calls[i].Effect!=EEffect::SyncRsResponseOwner)return false;
                const bool newEffect=Calls[i].Effect==EEffect::ResponseRsContact||Calls[i].Effect==EEffect::SetRsFutureHit||
                    Calls[i].Effect==EEffect::AcceptRsHit||Calls[i].Effect==EEffect::ResolveRsHit;
                if(newEffect&&!rsResponse)return false;
                if(rsResponse)
                {
                    if(i==42&&(Calls[i].Effect!=EEffect::ResponseRsContact||Calls[i].Owner||Calls[i].Pass!=INDEX_NONE))return false;
                    if(i>=43&&i<=45)
                    {
                        const EEffect effects[3]={EEffect::SetRsFutureHit,EEffect::AcceptRsHit,EEffect::ResolveRsHit};
                        if(Calls[i].Effect!=effects[i-43]||Calls[i].Owner!=Calls[33].Owner||Calls[i].Pass!=INDEX_NONE)return false;
                    }
                    if((i<42||i>45)&&newEffect)return false;
                }
                if(i!=30&&Calls[i].Effect==EEffect::ApplyInitialPosition)return false;
                if(i!=31&&Calls[i].Effect==EEffect::EnterTerminalStep)return false;
                if(i>=32&&i<35)
                {
                    if(Calls[i].Effect!=EEffect::SyncStepOwner||!Calls[i].Owner||Calls[i].Pass!=INDEX_NONE)return false;
                    for(int32 j=32;j<i;++j)if(Calls[j].Owner==Calls[i].Owner)return false;
                }
                else if(Calls[i].Effect==EEffect::SyncStepOwner)return false;
                if(OnePausedWork::HasSphereRsCapturePrefix(Purpose))
                {
                    const int32 retire=OnePausedWork::ExpectedNativeCallCount(Purpose)-3;
                    if(i==35&&(Calls[i].Effect!=EEffect::BeginRsCaptureSweep||Calls[i].Owner||Calls[i].Pass!=INDEX_NONE))return false;
                    if((i==36||i==37)&&(Calls[i].Effect!=EEffect::SweepOwnerForRsCapture||Calls[i].Owner!=Calls[i-4].Owner||Calls[i].Pass!=INDEX_NONE))return false;
                    if(i>=retire&&(Calls[i].Effect!=EEffect::RetireOwnerInput||Calls[i].Owner!=Calls[32+i-retire].Owner||Calls[i].Pass!=INDEX_NONE))return false;
                    if(i!=35&&Calls[i].Effect==EEffect::BeginRsCaptureSweep)return false;
                    if(i!=36&&i!=37&&Calls[i].Effect==EEffect::SweepOwnerForRsCapture)return false;
                    if(OnePausedWork::HasConsumedSphereAdvancePrefix(Purpose))
                    {
                        if(i==38&&(Calls[i].Effect!=EEffect::AdvanceRsPrefix||Calls[i].Owner||Calls[i].Pass!=INDEX_NONE))return false;
                        if(i>=39&&i<=41&&(Calls[i].Effect!=EEffect::AdvanceRsPrefixOwner||Calls[i].Owner!=Calls[i-7].Owner||Calls[i].Pass!=INDEX_NONE))return false;
                        if(i!=38&&Calls[i].Effect==EEffect::AdvanceRsPrefix)return false;
                        if((i<39||i>41)&&Calls[i].Effect==EEffect::AdvanceRsPrefixOwner)return false;
                    }
                    else if(Calls[i].Effect==EEffect::AdvanceRsPrefix||Calls[i].Effect==EEffect::AdvanceRsPrefixOwner)return false;
                }
                else if(Calls[i].Effect==EEffect::BeginRsCaptureSweep||Calls[i].Effect==EEffect::SweepOwnerForRsCapture||
                    Calls[i].Effect==EEffect::AdvanceRsPrefix||Calls[i].Effect==EEffect::AdvanceRsPrefixOwner)return false;
            }
        }
        else return false;
        uint64 empty = 0; return ClaimedWindow.compare_exchange_strong(empty, serial);
    }
public:
    FContract(const FContract&) = delete;
    FContract& operator=(const FContract&) = delete;
    bool IsFor(const FPreparedWorldSnapshotPtr& snapshot, uint64 serial) const noexcept
    { return Snapshot && Snapshot == snapshot && SourceState && Observe && serial && ClaimedWindow.load() == serial; }
    OnePausedWork::EPurpose GetPurpose() const noexcept {return Purpose;}
    // Native boundary fact only; it grants no physical mutation or full-frame coverage.
    bool HasReachedPositionBoundary() const noexcept {return OnePausedWork::HasInitialPosition(Purpose)&&PositionBoundaryReached;}
    bool HasAcceptedPosition() const noexcept {return PositionAttempted&&PositionReturned&&PositionAccepted;}
    bool HasCompletedStepSync() const noexcept
    {return Purpose==OnePausedWork::EPurpose::TerminalStepSyncStop&&StepSerialConfirmed&&StepSyncReached&&HasAcceptedPosition()&&!Failed;}
    bool HasClosedNativeRsCapture() const noexcept
    {return OnePausedWork::HasSphereRsCapturePrefix(Purpose)&&StepSerialConfirmed&&StepSyncReached&&
        HasAcceptedPosition()&&RsMathAttempted&&RsMathReturned&&RsCapture&&RsCapture->IsClosed()&&!Failed;}
    bool HasCompletedNativeRsAdvance() const noexcept
    {
        if(Purpose!=OnePausedWork::EPurpose::TerminalConsumedPrefixAdvanceStop||!HasClosedNativeRsCapture()||
            !RsAdvance.Started||!RsAdvance.Complete||!RsAdvance.Serial||RsAdvance.CompletedOwners!=3||
            RsAdvance.ActiveIndex!=INDEX_NONE||RsAdvance.AttemptedMask!=7||RsAdvance.ReturnedMask!=7||RsAdvance.VerifiedMask!=7||RsAdvance.StateCapturedMask!=7||RsAdvance.ActualStates.Num()!=3||
            Records.Num()!=45)return false;
        for(int32 i=38;i<=41;++i)if(!Records[i].Attempted||!Records[i].Returned||!Records[i].Accepted)return false;
        return true;
    }
    bool HasCompletedNativeRsContactAdvance() const noexcept
    {
        if(Purpose!=OnePausedWork::EPurpose::TerminalRsContactAdvanceStop||!HasClosedNativeRsCapture()||!RsCapture->HasBoundedFirstArrivalRsQuality()||
            !RsAdvance.Started||!RsAdvance.Complete||!RsAdvance.Serial||RsAdvance.CompletedOwners!=3||
            RsAdvance.ActiveIndex!=INDEX_NONE||RsAdvance.AttemptedMask!=7||RsAdvance.ReturnedMask!=7||RsAdvance.VerifiedMask!=7||RsAdvance.StateCapturedMask!=7||RsAdvance.ActualStates.Num()!=3||
            Records.Num()!=45)return false;
        for(int32 i=38;i<=41;++i)if(!Records[i].Attempted||!Records[i].Returned||!Records[i].Accepted)return false;
        return true;
    }
    bool HasCompletedNativeRsResponsePrefix() const noexcept
    {
        if(!OnePausedWork::HasFirstRsResponsePrefix(Purpose)||Records.Num()!=OnePausedWork::ExpectedNativeCallCount(Purpose)||!HasClosedNativeRsCapture()||
            !RsCapture->HasBoundedFirstArrivalRsQuality()||!RsAdvance.Started||!RsAdvance.Complete||!RsAdvance.Serial||
            RsAdvance.CompletedOwners!=3||RsAdvance.ActiveIndex!=INDEX_NONE||RsAdvance.AttemptedMask!=7||RsAdvance.ReturnedMask!=7||
            RsAdvance.VerifiedMask!=7||RsAdvance.StateCapturedMask!=7||RsAdvance.ActualStates.Num()!=3||!FirstRsResponseData||
            !FirstRsResponse.Started||!FirstRsResponse.Complete||FirstRsResponse.Failed||!FirstRsResponse.Serial||
            FirstRsResponse.Data!=FirstRsResponseData||FirstRsResponse.AdvanceSerial!=RsAdvance.Serial||
            FirstRsResponse.CompletedCalls!=3||FirstRsResponse.ActiveCall!=INDEX_NONE||FirstRsResponse.AttemptedMask!=7||
            FirstRsResponse.ReturnedMask!=7||FirstRsResponse.VerifiedMask!=7||FirstRsResponse.StateCapturedMask!=7||
            FirstRsResponse.ActualCalls.Num()!=3)return false;
        for(int32 i=38;i<=45;++i)if(!Records[i].Attempted||!Records[i].Returned||!Records[i].Accepted)return false;
        return true;
    }
    bool HasCompletedNativeRsResponse() const noexcept
    {return Purpose==OnePausedWork::EPurpose::TerminalRsResponseStop&&Records.Num()==49&&HasCompletedNativeRsResponsePrefix();}
    bool HasCompletedNativeRsResponseSync() const noexcept
    {return Purpose==OnePausedWork::EPurpose::TerminalRsResponseSyncStop&&Records.Num()==50&&HasCompletedNativeRsResponseSyncPrefix();}
    bool HasCompletedNativePostSyncRsCapture() const noexcept
    {
        if(Purpose!=OnePausedWork::EPurpose::TerminalPostSyncRsCaptureStop||Records.Num()!=53||!HasCompletedNativeRsResponseSyncPrefix()||
            !PostSyncRsMathAttempted||!PostSyncRsMathReturned||!PostSyncRsCapture||!PostSyncRsCapture->IsClosed()||
            !PostSyncRsRequestCaptured||PostSyncRsCapture->GetRequest()!=PostSyncRsRequestCaptured||
            PostSyncRsCapture->GetResponseData()!=FirstRsResponseData||PostSyncRsCapture->GetResponseSerial()!=FirstRsResponse.Serial||
            !Records[49].Attempted||Records[49].Returned||!Records[49].Accepted||!Records[49].InterruptedAtPostSyncRsCapture||Records[49].InterruptedAtRsCapture)return false;
        for(int32 i=47;i<49;++i)if(!Records[i].Attempted||!Records[i].Returned||!Records[i].Accepted)return false;
        return true;
    }
    PreparedExecution::FNativeSpherePostSyncRsCapturePtr ReadPostSyncRsCaptureAfterJoin() const
    {return IsInGameThread()&&Joined&&Purpose==OnePausedWork::EPurpose::TerminalPostSyncRsCaptureStop&&Records.Num()==53?PostSyncRsCapture:PreparedExecution::FNativeSpherePostSyncRsCapturePtr{};}
    PreparedExecution::FNativeSphereRequestPtr ReadPostSyncRsRequestAfterJoin() const
    {return IsInGameThread()&&Joined&&Purpose==OnePausedWork::EPurpose::TerminalPostSyncRsCaptureStop&&Records.Num()==53?PostSyncRsRequestCaptured:PreparedExecution::FNativeSphereRequestPtr{};}
    PreparedExecution::FNativeSpherePostSyncRsCapturePtr ReadPostSyncRsDiagnosticCaptureAfterJoin() const
    {return IsInGameThread()&&Joined&&Purpose==OnePausedWork::EPurpose::TerminalPostSyncRsCaptureStop&&Records.Num()==53?PostSyncRsDiagnosticCapture:PreparedExecution::FNativeSpherePostSyncRsCapturePtr{};}
    bool DidAttemptPostSyncRsMathAfterJoin() const noexcept
    {return IsInGameThread()&&Joined&&Purpose==OnePausedWork::EPurpose::TerminalPostSyncRsCaptureStop&&Records.Num()==53&&PostSyncRsMathAttempted;}
    bool DidReturnPostSyncRsMathAfterJoin() const noexcept
    {return IsInGameThread()&&Joined&&Purpose==OnePausedWork::EPurpose::TerminalPostSyncRsCaptureStop&&Records.Num()==53&&PostSyncRsMathReturned;}
    const PreparedExecution::FPostSyncRsMathResult* ReadPostSyncRsMathPartialAfterJoin() const noexcept
    {
        if(!IsInGameThread()||!Joined||Purpose!=OnePausedWork::EPurpose::TerminalPostSyncRsCaptureStop||Records.Num()!=53)return nullptr;
        if(PostSyncRsCapture)return &PostSyncRsCapture->GetMath();
        if(ReadPostSyncRsMathPartialAction)
            if(const auto* sourceFailure=ReadPostSyncRsMathPartialAction(SourceState.get()))return sourceFailure;
        if(PostSyncRsDiagnosticCapture)return &PostSyncRsDiagnosticCapture->GetMath();
        return PostSyncRsUnqualifiedMathPresent?&PostSyncRsUnqualifiedMath:nullptr;
    }
    const FString* ReadPostSyncRsMathRefusalAfterJoin() const noexcept
    {return IsInGameThread()&&Joined&&Purpose==OnePausedWork::EPurpose::TerminalPostSyncRsCaptureStop&&Records.Num()==53?&PostSyncRsMathActionRefusal:nullptr;}
    const PreparedExecution::FTerminalFirstRsResponseObservation* ReadFirstRsResponsePrefixForPostSyncAfterJoin() const
    {return IsInGameThread()&&Joined&&Purpose==OnePausedWork::EPurpose::TerminalPostSyncRsCaptureStop&&Records.Num()==53?&FirstRsResponse:nullptr;}
    const PreparedExecution::FTerminalFirstRsResponseSyncObservation* ReadFirstRsResponseSyncForPostSyncAfterJoin() const
    {return IsInGameThread()&&Joined&&Purpose==OnePausedWork::EPurpose::TerminalPostSyncRsCaptureStop&&Records.Num()==53?&FirstRsResponseSync:nullptr;}
    const PreparedExecution::FTerminalFirstRsResponseObservation* ReadFirstRsResponsePrefixForSyncAfterJoin() const
    {return IsInGameThread()&&Joined&&Purpose==OnePausedWork::EPurpose::TerminalRsResponseSyncStop?&FirstRsResponse:nullptr;}
    const PreparedExecution::FTerminalFirstRsResponseSyncObservation* ReadFirstRsResponseSyncAfterJoin() const
    {return IsInGameThread()&&Joined&&Purpose==OnePausedWork::EPurpose::TerminalRsResponseSyncStop?&FirstRsResponseSync:nullptr;}
    const PreparedExecution::FTerminalFirstRsResponseObservation* ReadFirstRsResponseAfterJoin() const
    {return IsInGameThread()&&Joined&&Purpose==OnePausedWork::EPurpose::TerminalRsResponseStop?&FirstRsResponse:nullptr;}
    const PreparedExecution::FTerminalConsumedPrefixAdvanceObservation* ReadConsumedPrefixAdvanceAfterJoin() const
    {return IsInGameThread()&&Joined&&OnePausedWork::HasConsumedSphereAdvancePrefix(Purpose)?&RsAdvance:nullptr;}
    PreparedExecution::FNativeSphereRsTrajectoryCapturePtr ReadRsCaptureAfterJoin() const
    {return IsInGameThread()&&Joined?RsCapture:PreparedExecution::FNativeSphereRsTrajectoryCapturePtr{};}
    PreparedExecution::FNativeSphereRequestPtr ReadRsRequestAfterJoin() const
    {return IsInGameThread()&&Joined?RsRequestCaptured:PreparedExecution::FNativeSphereRequestPtr{};}
    bool HasReachedAllPrepare() const noexcept { return AllPrepareReached.load(); }
    bool HasReturnedStop() const noexcept { return Reached && ScopeReturned && !Failed&&
        (Purpose==OnePausedWork::EPurpose::PreparationPrefixStop||
         (Purpose==OnePausedWork::EPurpose::PostPrepareInitialPositionStop&&HasAcceptedPosition())||HasCompletedStepSync()||
         (Purpose==OnePausedWork::EPurpose::TerminalSphereRsCaptureStop&&HasClosedNativeRsCapture())||HasCompletedNativeRsAdvance()||HasCompletedNativeRsContactAdvance()||HasCompletedNativeRsResponse()||HasCompletedNativeRsResponseSync()||HasCompletedNativePostSyncRsCapture()); }
    // The GT lease sets Joined only after the matching native worker really
    // joins. Reached is separate from successful owner retirement/neutralization.
    const TArray<FEffectRecord>* ReadJournalAfterJoin() const
    { return IsInGameThread() && Joined ? &Records : nullptr; }
    bool DidRetireOwnersAfterJoin() const { return IsInGameThread() && Joined && OwnersRetired; }
    const FString* ReadPositionActionRefusalAfterJoin() const
    { return IsInGameThread() && Joined ? &PositionActionRefusal : nullptr; }
    const FString* ReadRefusalAfterJoin() const
    { return IsInGameThread() && Joined ? &Refusal : nullptr; }
};

// Readback of the real current context, never a caller-supplied permission.
// Observation is read-only; mutation context exists only inside the native
// one-use dispatch of the new purpose. Every Module entry must reread it.
IAMSPEED_API bool ReadInitialPositionObservationContext(const ISpeedComponent*,const USSubBody*,FContext&) noexcept;
IAMSPEED_API bool ReadInitialPositionMutationContext(const ISpeedComponent*,const USSubBody*,FContext&) noexcept;
// Only native Host/subsystem sites construct effects. Callers cannot manufacture
// ordinal/epoch/context, publish journal rows, or seal a terminal prefix marker.
class IAMSPEED_API FEffectScope final
{
    friend class ::ASpeedPhysicsSimulation;
    friend class ::USpeedWorldSubsystem;
    FCursor* Cursor = nullptr;
    int32 Record = INDEX_NONE;
    FEffectScope(EEffect effect, const ISpeedComponent* owner = nullptr, int32 pass = INDEX_NONE);
    static void VerifyStructuralBoundary();
    void InterruptedAtRsCapture(PreparedExecution::FNativeSphereRsTrajectoryCapturePtr);
    void InterruptedAtPostSyncRsCapture(PreparedExecution::FNativeSpherePostSyncRsCapturePtr);
public:
    ~FEffectScope() noexcept;
    void Returned(bool succeeded = true, int64 nativeReturnValue = 0);
    FEffectScope(const FEffectScope&) = delete;
    FEffectScope& operator=(const FEffectScope&) = delete;
};
class FCursor final
{
    friend class Speed::PreparedExecution::FCanonicalFramePreflight;
    friend class Speed::PreparedExecution::FStepScope;
    friend class Speed::PreparedExecution::FSphereMemberSweepScope;
    friend IAMSPEED_API bool Speed::PreparedExecution::ReadCurrentStep(Speed::PreparedExecution::FStepIdentity&) noexcept;
    friend IAMSPEED_API bool Speed::PreparedExecution::NextSphereQuery(Speed::PreparedExecution::FStepIdentity&) noexcept;
    friend class FEffectScope;
    friend class ::ASpeedPhysicsSimulation;
    friend class ::USpeedWorldSubsystem;
    TSharedPtr<const OnePausedWork::FCanonicalFrameWindow, ESPMode::ThreadSafe> Window;
    FContractPtr Contract;
    FContext Context;
    FHostCounters Counters;
    bool Observing=false,PositionDispatch=false;
    const FContext* ActiveObservation=nullptr;
    friend IAMSPEED_API bool ReadInitialPositionMutationContext(const ISpeedComponent*,const USSubBody*,FContext&) noexcept;
    friend IAMSPEED_API bool ReadInitialPositionObservationContext(const ISpeedComponent*,const USSubBody*,FContext&) noexcept;
    friend IAMSPEED_API const FPreparedNativeOwnerBinding* FindCurrentPrivateSourceObservation(const ISpeedComponent*) noexcept;
    friend IAMSPEED_API bool IsCurrentPrivateSourceObservation(const UActorComponent*,
        const ISpeedComponent*,const FPreparedNativeOwnerBinding&) noexcept;
    TArray<int32, TInlineAllocator<16>> Stack;
    int32 NextCall = 0;
    uint64 Epoch = 0, CurrentStep = 0;
    static thread_local FCursor* Current;
    FCursor(const ASpeedPhysicsSimulation&, const USpeedWorldSubsystem&, const FSimulationWorker*,
        uint64 generation, uint64 frame, float delta, float time);
    [[noreturn]] void Refuse(const TCHAR* reason);
    bool CurrentIdentity() const;
    bool Observe(EObservation,const FContext&,FNativePrepareGTFields&,FString&);
    FContext MakeContext(const FCall&, uint64 ordinal, bool succeeded);
    int32 Before(EEffect, const ISpeedComponent*, int32 pass);
    void After(int32 record, bool succeeded, int64 nativeReturnValue);
    void Abandon(int32 record) noexcept;
    bool IsClosedPlannedRow(int32 Index) const;
    void AfterInterruptedRsCapture(int32,PreparedExecution::FNativeSphereRsTrajectoryCapturePtr);
    void AfterInterruptedPostSyncRsCapture(int32,PreparedExecution::FNativeSpherePostSyncRsCapturePtr);
    static bool IsCurrentPostSyncRsNativeWindow(const USpeedWorldSubsystem&) noexcept;
    static bool IsCurrentPostSyncRsPrelude(const USpeedWorldSubsystem&) noexcept;
    static bool IsCurrentPostSyncRsSweepSite(const ISpeedComponent&,int32) noexcept;
    static bool IsCurrentPostSyncRsObservation() noexcept;
    static bool EvaluateCapturedPostSyncRs(const PreparedExecution::FNativeSphereRequestPtr&,const PreparedExecution::FNativeRsTraceValues&,PreparedExecution::FPostSyncRsMathResult&);
    static bool IsCurrentRsNativeWindow(const USpeedWorldSubsystem&) noexcept;
    static bool IsCurrentRsObservation(const USpeedWorldSubsystem&) noexcept;
    static bool IsCurrentRsPrelude(const USpeedWorldSubsystem&) noexcept;
    static bool IsCurrentRsSweepSite(const ISpeedComponent&,int32 OwnerIndex) noexcept;
    static bool EvaluateCapturedRs(const PreparedExecution::FNativeSphereRequestPtr&,const PreparedExecution::FNativeRsTraceValues&,PreparedExecution::FCapturedRsMathResult&);
    static bool IsCurrentConsumedPrefixNativeWindow(const USpeedWorldSubsystem&) noexcept;
    static bool IsCurrentConsumedPrefixSite(int32 OwnerIndex) noexcept;
    static bool IsCurrentConsumedPrefixObservation() noexcept;
    static bool IsCurrentRsResponseNativeWindow(const USpeedWorldSubsystem&) noexcept;
    static bool IsCurrentRsResponseSite(int32 Call) noexcept;
    static bool IsCurrentRsResponseObservation() noexcept;
    static bool IsCurrentRsResponseSyncNativeWindow(const USpeedWorldSubsystem&) noexcept;
    static bool IsCurrentRsResponseSyncSite() noexcept;
    static bool CaptureFirstRsResponseSyncPartial(TSharedPtr<const FNativePrepareGTFields,ESPMode::ThreadSafe>&,FString&) noexcept;
    static bool CaptureFirstRsResponsePartial(int32 Call,TSharedPtr<const FNativePrepareGTFields,ESPMode::ThreadSafe>&,FString&) noexcept;
    static bool ApplyPostPrepareInitialPosition(const ASpeedPhysicsSimulation&);
    // Private native-site transitions; Source callbacks cannot supply a serial
    // or synthesize a reset completion. The StepBefore descriptor is immutable.
    static bool IsTerminalStepSyncEntry(const USpeedWorldSubsystem&) noexcept;
    static bool ConfirmTerminalStepSerial(const USpeedWorldSubsystem&) noexcept;
    static bool FinishTerminalStepSync(const USpeedWorldSubsystem&);
    static bool Finish(const ASpeedPhysicsSimulation&);
public:
    ~FCursor() noexcept;
    FCursor(const FCursor&) = delete;
    FCursor& operator=(const FCursor&) = delete;
    static bool IsStoppedForCurrentCall(const ASpeedPhysicsSimulation*) noexcept;
};
}
}
