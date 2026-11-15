#pragma once
#include "NativePreparationPrefix.h"
#include "PreparedNativeFrameConsumer.h"
#include "IAmSpeed/World/Simulation/SpeedPhysicsSimulation.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "IAmSpeed/World/Simulation/CanonicalFrameDriver.h"
#include "IAmSpeed/World/Simulation/CanonicalFrameContext.h"
#include "Misc/ScopeLock.h"
#include "IAmSpeed/World/Simulation/OnePausedCanonicalWork.h"
#include "IAmSpeed/World/Collision/StaticCollisionWorld.h"
#include "IAmSpeed/World/Collision/PreparedWorldSnapshot.h"
#include "IAmSpeed/World/Collision/PreparedCanonicalStaticQueryService.h"
#include <optional>
#include <utility>
namespace Speed
{
// Lexical GT fence: no yield, user callback or physical restart is implicit.
// Timeout preserves the underlying pause request. Only an acknowledged service
// suspension or EXACT private worker absence can mint a lease.
class FHostedGTStaticWorldLease final
{
    friend class PreparationPrefix::FCursor;
    static PreparationPrefix::FHostCounters ReadPreparationHostCounters(const ASpeedPhysicsSimulation& h)
    {return {h.CanonicalNumFrame,h.SnapshotBuffer.PublishedSerial(),h.FrameHashes.Num()};}

    TWeakObjectPtr<ASpeedPhysicsSimulation> Host;
    TWeakObjectPtr<USpeedWorldSubsystem> Bridge;
    TWeakObjectPtr<UWorld> World;
    FSimulationWorker* Worker=nullptr; // Opaque identity only.
    uint64 Generation=0;
    bool OwnDepth=false;
    bool NativeGenerationReplacement=false;
    OnePausedWork::FCanonicalFrameWindowPtr ActiveWindow;
    FHostedGTStaticWorldLease(ASpeedPhysicsSimulation& h,USpeedWorldSubsystem& s,FSimulationWorker* worker,uint64 generation)
        :Host(&h),Bridge(&s),World(h.GetWorld()),Worker(worker),Generation(generation),OwnDepth(true){s.StaticWorldPublicationDepth=1;h.bStaticWorldPublicationLease=true;}
public:
    FHostedGTStaticWorldLease(const FHostedGTStaticWorldLease&)=delete;
    FHostedGTStaticWorldLease& operator=(const FHostedGTStaticWorldLease&)=delete;
    FHostedGTStaticWorldLease& operator=(FHostedGTStaticWorldLease&&)=delete;
    FHostedGTStaticWorldLease(FHostedGTStaticWorldLease&& other) noexcept
        :Host(other.Host),Bridge(other.Bridge),World(other.World),Worker(other.Worker),Generation(other.Generation),OwnDepth(std::exchange(other.OwnDepth,false)),NativeGenerationReplacement(other.NativeGenerationReplacement),ActiveWindow(MoveTemp(other.ActiveWindow)){}
    ~FHostedGTStaticWorldLease()
    {
        if(ActiveWindow)ActiveWindow->Close();
        if(!OwnDepth)return;
        if(!IsInGameThread()){ensureAlwaysMsgf(false,TEXT("HostedGT static World lease destroyed off GT; fence retained"));return;}
        // A stale/moved lease cannot resume a replacement generation/worker.
        if(auto* h=Host.GetEvenIfUnreachable())
            if(h->InputWorkerGeneration!=Generation||(Worker&&h->SimulationWorker.Get()!=Worker)){OwnDepth=false;return;}
        OwnDepth=false;if(auto* s=Bridge.Get())s->StaticWorldPublicationDepth=0;
        if(auto* h=Host.Get()){h->bStaticWorldPublicationLease=false;h->ResumeOwnedBoundaryService();}
        // Do NOT resume physical simulation here.
    }
    static std::optional<FHostedGTStaticWorldLease> TryAcquire(ASpeedPhysicsSimulation& h,USpeedWorldSubsystem& s,uint32 timeoutMs=1000)
    {
        if(!IsInGameThread())return std::nullopt;
        auto* w=h.GetWorld();
        if(!IsValid(&h)||h.IsTemplate()||!w||w->bIsTearingDown||s.GetWorld()!=w||w->GetSubsystem<USpeedWorldSubsystem>()!=&s||
            !h.IsOwnedWorkerExecutionMode()||h.OwnedBoundarySuspendDepth||h.bStaticWorldPublicationLease||s.StaticWorldPublicationDepth||!h.InputWorkerGeneration||
            h.bOwnedWorkerTerminal.Load()||h.bCanonicalPublicationTerminal.Load()||h.bInputOwnerRetired.Load())return std::nullopt;
        int32 hosts=0;for(TActorIterator<ASpeedPhysicsSimulation> it(w);it;++it){if(*it!=&h)return std::nullopt;++hosts;}
        if(hosts!=1)return std::nullopt;
        const uint64 generation=h.InputWorkerGeneration;auto* const worker=h.SimulationWorker.Get();
        if(!worker){h.bOwnedSimulationPaused.Store(true);h.OwnedBoundarySuspendDepth=1;}
        else if(h.TrySuspendOwnedBoundaryService(timeoutMs)!=ESimulationQuiescence::BoundaryAcknowledged)return std::nullopt;
        FHostedGTStaticWorldLease lease(h,s,worker,generation);
        if(!lease.IsFor(s))return std::nullopt;
        return std::move(lease);
    }
    bool IsFor(const USpeedWorldSubsystem& s) const
    {
        if(!IsInGameThread()||!OwnDepth||s.StaticWorldPublicationDepth!=1||s.bCanonicalFrameActive)return false;
        auto* h=Host.Get();auto* w=World.Get();
        if(!h||!w||w->bIsTearingDown||Bridge.Get()!=&s||h->GetWorld()!=w||s.GetWorld()!=w||w->GetSubsystem<USpeedWorldSubsystem>()!=&s||
            !h->IsOwnedWorkerExecutionMode()||!h->bStaticWorldPublicationLease||h->InputWorkerGeneration!=Generation||h->OwnedBoundarySuspendDepth!=1||
            h->bInputOwnerRetired.Load()||h->bOwnedWorkerTerminal.Load()||h->bCanonicalPublicationTerminal.Load()||
            h->SimulationWorker.Get()!=Worker)return false;
        return Worker?h->IsOwnedBoundaryServiceSuspended():!h->SimulationWorker;
    }

    bool HasNativeWorkerAtBoundary() const
    {auto* bridge=Bridge.Get();return bridge&&IsFor(*bridge)&&Worker;}
    // A distinct runtime generation replaces only its own joined/revoked
    // provider. Step/frame serials are deliberately never reset or refunded.
    bool BeginNativeGenerationReplacement(const FPreparedWorldSnapshotPtr& previous)
    {
        auto* bridge=Bridge.Get();auto* host=Host.Get();
        if(!bridge||!host||!IsFor(*bridge)||Worker||ActiveWindow||NativeGenerationReplacement||!previous)return false;
        const auto* provider=bridge->GetStaticCollisionWorld();
        const auto consumer=provider?provider->GetPreparedNativeFrameConsumer():FPreparedNativeFrameConsumerPtr{};
        const auto& origin=previous->GetOrigin();
        if(!provider||provider->GetPreparedSnapshot()!=previous||!consumer||consumer->Snapshot!=previous||
            !consumer->Retention||consumer->Retention->IsRetained()||
            origin.Host!=host||origin.Bridge!=bridge||origin.World!=World.Get()||origin.Generation==Generation||
            !origin.Generation||bridge->PreparedIntervalScene!=previous->GetNominalScene()||
            bridge->PreparedIntervalAsset.Get()!=origin.Asset)return false;
        bridge->SimulationWorld.ResetStaticCollisionWorld();bridge->PreparedIntervalScene.Reset();
        bridge->PreparedIntervalAsset.Reset();bridge->AnalyticSourceComponents.Reset();
        NativeGenerationReplacement=true;return true;
    }

    // Initial structural registration under the actual suspended GT lease.
    // No Prepare/Step/validation callback or physical time is executed here.
    bool FlushInitialNativeOwnerRegistry() const
    {
        auto* bridge=Bridge.Get();auto* host=Host.Get();
        if(!bridge||!host||!IsFor(*bridge)||ActiveWindow||bridge->bCanonicalFrameActive||
            (bridge->StepSerial!=0&&!NativeGenerationReplacement)||!bridge->bPreparedIntervalRouteRequested||bridge->PreparedIntervalScene||
            bridge->SimulationWorld.GetStaticCollisionWorld())return false;
        bridge->ApplyPendingOps();bridge->RebuildSortedIfNeeded();
        bool pending=false;{FScopeLock lock(&bridge->PendingCS);pending=!bridge->PendingOps.IsEmpty();}
        return IsFor(*bridge)&&!bridge->bCanonicalFrameActive&&(bridge->StepSerial==0||NativeGenerationReplacement)&&
            !pending&&!bridge->SimulationWorld.bOrderDirty;
    }

    // First actual frame must have closed through snapshot and advanced the
    // native published counter. Arming never accepts a caller-supplied PASS.
    bool EnableNativeFrameStream(const FPreparedNativeFrameConsumerPtr& consumer)
    {
        auto* bridge=Bridge.Get();auto* host=Host.Get();
        const auto* provider=bridge?bridge->GetStaticCollisionWorld():nullptr;
        if(!bridge||!host||!Worker||!IsFor(*bridge)||!consumer||!provider||
            provider->GetPreparedNativeFrameConsumer()!=consumer||
            !consumer->Matches(provider->GetPreparedSnapshot(),provider->GetPreparedQueryService()))return false;
        const uint64 closed=consumer->LastSnapshotClosure.load(std::memory_order_acquire);
        if(closed==MAX_uint64||closed==MAX_uint64-1||host->CanonicalNumFrame!=closed+1||!bridge->StepSerial)return false;
        if(ActiveWindow)
        {
            if(ActiveWindow->GetPurpose()!=OnePausedWork::EPurpose::FullFrame||ActiveWindow->Frame!=closed||
                ActiveWindow->GetExpectedStep()!=bridge->StepSerial||ActiveWindow->GetSnapshot()!=consumer->Snapshot)return false;
            ActiveWindow->Close();ActiveWindow.Reset();
        }
        consumer->StreamWorker=Worker;consumer->ContinuousEnabled.store(true,std::memory_order_release);return true;
    }

    struct FOneFrameResult
    {
        OnePausedWork::FResult Work;
        uint64 Generation=0,FrameBefore=0,FrameAfter=0,StepBefore=0,StepAfter=0;
        bool ExactCanonicalDelta=false,LeaseRecaptured=false,CountersObserved=false,HostRetirementClosed=false;
    };
    // Synchronous GT operation: no GT dispatch/pump, worker creation or Ready
    // mutation. Success retains THIS lease at the matching NEW post-Work park.
    FOneFrameResult RunOneCanonicalFrame(OnePausedWork::FCanonicalFrameWindowPtr window,uint32 timeoutMs=1000)
    {
        FOneFrameResult out;auto* h=Host.Get();auto* s=Bridge.Get();
        if(!h||!s||!IsFor(*s)||!Worker||!CanonicalFrameDriver::IsEnabled()||
            !h->bCanonicalFrameInitialized||h->CanonicalNumFrame>=MAX_uint32||
            !s->GetStaticCollisionWorld()||!s->GetStaticCollisionWorld()->IsReadyForPhysicsStep()||!window||window->GetPurpose()!=OnePausedWork::EPurpose::FullFrame||!window->IsOpen()||window->GetNonce()||ActiveWindow)return out;
        {FScopeLock lock(&h->RollbackRequestMutex);if(h->PendingRollbackRequest.IsSet())return out;}
        const auto context=h->MakeCanonicalFrameContext(h->CanonicalNumFrame);
        OnePausedWork::FCanonicalIdentity identity;
        identity.Host=h;identity.Bridge=s;identity.Provider=s->GetStaticCollisionWorld();
        identity.Worker=Worker;identity.Generation=Generation;identity.Frame=h->CanonicalNumFrame;
        identity.Delta=context.PhysicalDeltaTime;identity.SimTime=context.SimTime;
        identity.Window=window;
        if(!window->Matches(h,s,identity.Provider,Worker,Generation,identity.Frame,identity.Delta,identity.SimTime)||
            window->GetSnapshot()!=identity.Provider->GetPreparedSnapshot()||window->GetStepBefore()!=s->StepSerial)return out;
        out.Generation=Generation;out.FrameBefore=h->CanonicalNumFrame;out.StepBefore=s->StepSerial;
        ActiveWindow=window;
        out.Work=Worker->RunOnePausedWork(identity,[&]
        {
            OwnDepth=false;s->StaticWorldPublicationDepth=0;
            h->bStaticWorldPublicationLease=false;h->OwnedBoundarySuspendDepth=0;
        },timeoutMs);
        if(!out.Work.Nonce)return out; // Not armed: original lease remains.
        if(out.Work.Quiescence==OnePausedWork::EQuiescence::ParkedForThisNonce&&h->InputWorkerGeneration==Generation&&h->SimulationWorker.Get()==Worker)
        {
            auto fresh=TryAcquire(*h,*s,timeoutMs);
            if(fresh&&fresh->Worker==Worker&&fresh->Generation==Generation)
            {
                OwnDepth=std::exchange(fresh->OwnDepth,false);out.LeaseRecaptured=true;
                out.FrameAfter=h->CanonicalNumFrame;out.StepAfter=s->StepSerial;out.CountersObserved=true;
                const bool advanced=out.Work.WorkResult==uint8(ESimulationWorkerResult::Advanced);
                const bool idle=out.Work.WorkResult==uint8(ESimulationWorkerResult::Idle);
                out.ExactCanonicalDelta=out.Work.GuardsAccepted&&out.Work.WorkReturned&&
                    s->GetStaticCollisionWorld()==identity.Provider&&
                    ((advanced&&out.FrameAfter==out.FrameBefore+1&&out.StepBefore!=MAX_uint64&&out.StepAfter==out.StepBefore+1)||
                     (idle&&out.FrameAfter==out.FrameBefore&&out.StepAfter==out.StepBefore));
                Worker->OneWorkReturnFence.Store(false);
                if(out.ExactCanonicalDelta)return out;
            }
        }
        // Partial failure is never rolled back/resumed. Close the real owner.
        window->Close();
        if(h->InputWorkerGeneration==Generation&&h->SimulationWorker.Get()==Worker)
        {
            if(OwnDepth){OwnDepth=false;s->StaticWorldPublicationDepth=0;h->bStaticWorldPublicationLease=false;h->OwnedBoundarySuspendDepth=0;}
            out.HostRetirementClosed=h->JoinOwnedSimulationForInputTeardown();
        }
        else out.HostRetirementClosed=h->InputOwnersRetiredAfterJoin(Generation);
        if(h->InputWorkerGeneration==Generation&&!h->SimulationWorker){out.FrameAfter=h->CanonicalNumFrame;out.StepAfter=s->StepSerial;out.CountersObserved=true;}
        if(h->InputWorkerGeneration==Generation&&!h->SimulationWorker)Worker=nullptr;
        // A fallback attempt is not a retirement receipt. Preserve UNKNOWN
        // unless the same actual generation and all native owners are closed.
        out.Work.Quiescence=out.HostRetirementClosed&&h->InputWorkerGeneration==Generation&&
            !h->SimulationWorker&&h->bInputOwnerRetired.Load()&&h->bInputOwnersClosedOnWorker.Load()
            ?OnePausedWork::EQuiescence::ClosedAndJoined:OnePausedWork::EQuiescence::None;
        out.LeaseRecaptured=false;return out;
    }
    // Native structure/liveness publication. The game producer separately
    // authenticates the full actual asset+packet inside THIS same GT lease.
    IAMSPEED_API bool PublishCanonicalReferenceSnapshot(const USpeedAnalyticCollisionAsset& Asset,
        Analytic::Prepared::FReferenceTransportPtr Reference,FPreparedWorldSnapshotPtr& Out,FString& Reason,
        FPreparedNativeOwnerInputPtr NativeInput={}) const;
    // New overload preserves the existing export and all closed callers. A
    // factory installs only the query consumer, never provider readiness.
    IAMSPEED_API bool PublishCanonicalReferenceSnapshot(const USpeedAnalyticCollisionAsset& Asset,
        Analytic::Prepared::FReferenceTransportPtr Reference,FPreparedWorldSnapshotPtr& Out,FString& Reason,
        FPreparedNativeOwnerInputPtr NativeInput,const FPreparedCanonicalQueryServiceFactoryGT* Factory) const;
    // Distinct compiled physical-attempt factory; old diagnostic exports keep
    // their null consumer and remain non-Ready.
    IAMSPEED_API bool PublishCanonicalReferenceSnapshot(const USpeedAnalyticCollisionAsset& Asset,
        Analytic::Prepared::FReferenceTransportPtr Reference,FPreparedWorldSnapshotPtr& Out,FString& Reason,
        FPreparedNativeOwnerInputPtr NativeInput,const FPreparedCanonicalQueryServiceFactoryGT* Factory,
        const FPreparedNativeFrameConsumerFactoryGT* NativeFactory) const;

    IAMSPEED_API FPreparedWorldSnapshotPtr ReadCanonicalReferenceSnapshot(const USpeedAnalyticCollisionAsset& Asset,
        FPreparedNativeOwnerInputPtr NativeInput={}) const;

    IAMSPEED_API FPreparedWorldSnapshotPtr ReadCanonicalReferenceSnapshotForExecution(const USpeedAnalyticCollisionAsset& Asset,FPreparedNativeOwnerInputPtr NativeInput={}) const;
    OnePausedWork::FCanonicalFrameWindowPtr CreateCanonicalFrameWindow(const USpeedAnalyticCollisionAsset& asset,
        FPreparedNativeOwnerInputPtr nativeInput,const FPreparedWorldSnapshotPtr& expected) const
    {
        auto* h=Host.Get();auto* s=Bridge.Get();auto* world=World.Get();
        if(!h||!s||!world||!IsFor(*s)||!Worker||!expected||!expected->GetNativeOwners()||
            ReadCanonicalReferenceSnapshotForExecution(asset,MoveTemp(nativeInput))!=expected||!h->bCanonicalFrameInitialized||
            h->CanonicalNumFrame>=MAX_uint32||s->StepSerial==MAX_uint64)return {};
        const uint64 serial=OnePausedWork::FCanonicalFrameWindow::AllocateSerialOnGT();if(!serial)return {};
        const auto context=h->MakeCanonicalFrameContext(h->CanonicalNumFrame);
        return MakeShareable(new OnePausedWork::FCanonicalFrameWindow(expected,world,h,s,s->GetStaticCollisionWorld(),Worker,
            Generation,context.NumFrame,s->StepSerial,serial,context.PhysicalDeltaTime,context.SimTime));
    }
    OnePausedWork::FCanonicalFrameWindowPtr CreateCanonicalPreparationWindow(const USpeedAnalyticCollisionAsset& asset,
        FPreparedNativeOwnerInputPtr nativeInput,const FPreparedWorldSnapshotPtr& expected,
        PreparationPrefix::FContractPtr contract) const
    {
        auto* h=Host.Get();auto* s=Bridge.Get();auto* world=World.Get();
        if(!h||!s||!world||!IsFor(*s)||!Worker||!expected||!expected->GetNativeOwners()||!contract||
            ReadCanonicalReferenceSnapshotAtReadiness(asset,MoveTemp(nativeInput),false)!=expected||
            !h->bCanonicalFrameInitialized||h->CanonicalNumFrame>=MAX_uint32||s->StepSerial==MAX_uint64)return {};
        const uint64 serial=OnePausedWork::FCanonicalFrameWindow::AllocateSerialOnGT();
        if(!serial||!contract->ClaimWindow(expected,serial))return {};
        const auto context=h->MakeCanonicalFrameContext(h->CanonicalNumFrame);
        return MakeShareable(new OnePausedWork::FCanonicalFrameWindow(expected,world,h,s,s->GetStaticCollisionWorld(),Worker,
            Generation,context.NumFrame,s->StepSerial,serial,context.PhysicalDeltaTime,context.SimTime,
            OnePausedWork::EPurpose::PreparationPrefixStop,MoveTemp(contract)));
    }
    OnePausedWork::FCanonicalFrameWindowPtr CreateCanonicalPostPreparePositionWindow(const USpeedAnalyticCollisionAsset& asset,
        FPreparedNativeOwnerInputPtr nativeInput,const FPreparedWorldSnapshotPtr& expected,
        PreparationPrefix::FContractPtr contract) const
    {
        auto* h=Host.Get();auto* s=Bridge.Get();auto* world=World.Get();
        if(!h||!s||!world||!IsFor(*s)||!Worker||!expected||!expected->GetNativeOwners()||!contract||contract->GetPurpose()!=OnePausedWork::EPurpose::PostPrepareInitialPositionStop||
            ReadCanonicalReferenceSnapshotAtReadiness(asset,MoveTemp(nativeInput),false)!=expected||
            !h->bCanonicalFrameInitialized||h->CanonicalNumFrame>=MAX_uint32||s->StepSerial==MAX_uint64)return {};
        const uint64 serial=OnePausedWork::FCanonicalFrameWindow::AllocateSerialOnGT();
        if(!serial||!contract->ClaimWindow(expected,serial))return {};
        const auto context=h->MakeCanonicalFrameContext(h->CanonicalNumFrame);
        return MakeShareable(new OnePausedWork::FCanonicalFrameWindow(expected,world,h,s,s->GetStaticCollisionWorld(),Worker,
            Generation,context.NumFrame,s->StepSerial,serial,context.PhysicalDeltaTime,context.SimTime,
            OnePausedWork::EPurpose::PostPrepareInitialPositionStop,MoveTemp(contract)));
    }
    OnePausedWork::FCanonicalFrameWindowPtr CreateCanonicalTerminalStepSyncWindow(const USpeedAnalyticCollisionAsset& asset,
        FPreparedNativeOwnerInputPtr nativeInput,const FPreparedWorldSnapshotPtr& expected,
        PreparationPrefix::FContractPtr contract) const
    {
        auto* h=Host.Get();auto* s=Bridge.Get();auto* world=World.Get();
        if(!h||!s||!world||!IsFor(*s)||!Worker||!expected||!expected->GetNativeOwners()||!contract||contract->GetPurpose()!=OnePausedWork::EPurpose::TerminalStepSyncStop||
            ReadCanonicalReferenceSnapshotAtReadiness(asset,MoveTemp(nativeInput),false)!=expected||
            !h->bCanonicalFrameInitialized||h->CanonicalNumFrame>=MAX_uint32||s->StepSerial==MAX_uint64)return {};
        const uint64 serial=OnePausedWork::FCanonicalFrameWindow::AllocateSerialOnGT();
        if(!serial||!contract->ClaimWindow(expected,serial))return {};
        const auto context=h->MakeCanonicalFrameContext(h->CanonicalNumFrame);
        return MakeShareable(new OnePausedWork::FCanonicalFrameWindow(expected,world,h,s,s->GetStaticCollisionWorld(),Worker,
            Generation,context.NumFrame,s->StepSerial,serial,context.PhysicalDeltaTime,context.SimTime,
            OnePausedWork::EPurpose::TerminalStepSyncStop,MoveTemp(contract)));
    }
    OnePausedWork::FCanonicalFrameWindowPtr CreateCanonicalTerminalSphereRsCaptureWindow(const USpeedAnalyticCollisionAsset& asset,
        FPreparedNativeOwnerInputPtr nativeInput,const FPreparedWorldSnapshotPtr& expected,
        PreparationPrefix::FContractPtr contract) const
    {
        auto* h=Host.Get();auto* s=Bridge.Get();auto* world=World.Get();
        if(!h||!s||!world||!IsFor(*s)||!Worker||!expected||!expected->GetNativeOwners()||!contract||contract->GetPurpose()!=OnePausedWork::EPurpose::TerminalSphereRsCaptureStop||
            ReadCanonicalReferenceSnapshotAtReadiness(asset,MoveTemp(nativeInput),false)!=expected||
            !h->bCanonicalFrameInitialized||h->CanonicalNumFrame>=MAX_uint32||s->StepSerial==MAX_uint64)return {};
        const uint64 serial=OnePausedWork::FCanonicalFrameWindow::AllocateSerialOnGT();
        if(!serial||!contract->ClaimWindow(expected,serial))return {};
        const auto context=h->MakeCanonicalFrameContext(h->CanonicalNumFrame);
        return MakeShareable(new OnePausedWork::FCanonicalFrameWindow(expected,world,h,s,s->GetStaticCollisionWorld(),Worker,
            Generation,context.NumFrame,s->StepSerial,serial,context.PhysicalDeltaTime,context.SimTime,
            OnePausedWork::EPurpose::TerminalSphereRsCaptureStop,MoveTemp(contract)));
    }
    OnePausedWork::FCanonicalFrameWindowPtr CreateCanonicalTerminalConsumedPrefixAdvanceWindow(const USpeedAnalyticCollisionAsset& asset,
        FPreparedNativeOwnerInputPtr nativeInput,const FPreparedWorldSnapshotPtr& expected,
        PreparationPrefix::FContractPtr contract) const
    {
        auto* h=Host.Get();auto* s=Bridge.Get();auto* world=World.Get();
        if(!h||!s||!world||!IsFor(*s)||!Worker||!expected||!expected->GetNativeOwners()||!contract||contract->GetPurpose()!=OnePausedWork::EPurpose::TerminalConsumedPrefixAdvanceStop||
            ReadCanonicalReferenceSnapshotAtReadiness(asset,MoveTemp(nativeInput),false)!=expected||
            !h->bCanonicalFrameInitialized||h->CanonicalNumFrame>=MAX_uint32||s->StepSerial==MAX_uint64)return {};
        const uint64 serial=OnePausedWork::FCanonicalFrameWindow::AllocateSerialOnGT();
        if(!serial||!contract->ClaimWindow(expected,serial))return {};
        const auto context=h->MakeCanonicalFrameContext(h->CanonicalNumFrame);
        return MakeShareable(new OnePausedWork::FCanonicalFrameWindow(expected,world,h,s,s->GetStaticCollisionWorld(),Worker,
            Generation,context.NumFrame,s->StepSerial,serial,context.PhysicalDeltaTime,context.SimTime,
            OnePausedWork::EPurpose::TerminalConsumedPrefixAdvanceStop,MoveTemp(contract)));
    }
    OnePausedWork::FCanonicalFrameWindowPtr CreateCanonicalTerminalRsContactAdvanceWindow(const USpeedAnalyticCollisionAsset& asset,
        FPreparedNativeOwnerInputPtr nativeInput,const FPreparedWorldSnapshotPtr& expected,
        PreparationPrefix::FContractPtr contract) const
    {
        auto* h=Host.Get();auto* s=Bridge.Get();auto* world=World.Get();
        if(!h||!s||!world||!IsFor(*s)||!Worker||!expected||!expected->GetNativeOwners()||!contract||contract->GetPurpose()!=OnePausedWork::EPurpose::TerminalRsContactAdvanceStop||
            ReadCanonicalReferenceSnapshotAtReadiness(asset,MoveTemp(nativeInput),false)!=expected||
            !h->bCanonicalFrameInitialized||h->CanonicalNumFrame>=MAX_uint32||s->StepSerial==MAX_uint64)return {};
        const uint64 serial=OnePausedWork::FCanonicalFrameWindow::AllocateSerialOnGT();
        if(!serial||!contract->ClaimWindow(expected,serial))return {};
        const auto context=h->MakeCanonicalFrameContext(h->CanonicalNumFrame);
        return MakeShareable(new OnePausedWork::FCanonicalFrameWindow(expected,world,h,s,s->GetStaticCollisionWorld(),Worker,
            Generation,context.NumFrame,s->StepSerial,serial,context.PhysicalDeltaTime,context.SimTime,
            OnePausedWork::EPurpose::TerminalRsContactAdvanceStop,MoveTemp(contract)));
    }
    OnePausedWork::FCanonicalFrameWindowPtr CreateCanonicalTerminalRsResponseWindow(const USpeedAnalyticCollisionAsset& asset,
        FPreparedNativeOwnerInputPtr nativeInput,const FPreparedWorldSnapshotPtr& expected,
        PreparationPrefix::FContractPtr contract) const
    {
        auto* h=Host.Get();auto* s=Bridge.Get();auto* world=World.Get();
        if(!h||!s||!world||!IsFor(*s)||!Worker||!expected||!expected->GetNativeOwners()||!contract||contract->GetPurpose()!=OnePausedWork::EPurpose::TerminalRsResponseStop||
            ReadCanonicalReferenceSnapshotAtReadiness(asset,MoveTemp(nativeInput),false)!=expected||
            !h->bCanonicalFrameInitialized||h->CanonicalNumFrame>=MAX_uint32||s->StepSerial==MAX_uint64)return {};
        const uint64 serial=OnePausedWork::FCanonicalFrameWindow::AllocateSerialOnGT();
        if(!serial||!contract->ClaimWindow(expected,serial))return {};
        const auto context=h->MakeCanonicalFrameContext(h->CanonicalNumFrame);
        return MakeShareable(new OnePausedWork::FCanonicalFrameWindow(expected,world,h,s,s->GetStaticCollisionWorld(),Worker,
            Generation,context.NumFrame,s->StepSerial,serial,context.PhysicalDeltaTime,context.SimTime,
            OnePausedWork::EPurpose::TerminalRsResponseStop,MoveTemp(contract)));
    }
    OnePausedWork::FCanonicalFrameWindowPtr CreateCanonicalTerminalRsResponseSyncWindow(const USpeedAnalyticCollisionAsset& asset,
        FPreparedNativeOwnerInputPtr nativeInput,const FPreparedWorldSnapshotPtr& expected,
        PreparationPrefix::FContractPtr contract) const
    {
        auto* h=Host.Get();auto* s=Bridge.Get();auto* world=World.Get();
        if(!h||!s||!world||!IsFor(*s)||!Worker||!expected||!expected->GetNativeOwners()||!contract||contract->GetPurpose()!=OnePausedWork::EPurpose::TerminalRsResponseSyncStop||
            ReadCanonicalReferenceSnapshotAtReadiness(asset,MoveTemp(nativeInput),false)!=expected||
            !h->bCanonicalFrameInitialized||h->CanonicalNumFrame>=MAX_uint32||s->StepSerial==MAX_uint64)return {};
        const uint64 serial=OnePausedWork::FCanonicalFrameWindow::AllocateSerialOnGT();
        if(!serial||!contract->ClaimWindow(expected,serial))return {};
        const auto context=h->MakeCanonicalFrameContext(h->CanonicalNumFrame);
        return MakeShareable(new OnePausedWork::FCanonicalFrameWindow(expected,world,h,s,s->GetStaticCollisionWorld(),Worker,
            Generation,context.NumFrame,s->StepSerial,serial,context.PhysicalDeltaTime,context.SimTime,
            OnePausedWork::EPurpose::TerminalRsResponseSyncStop,MoveTemp(contract)));
    }
    OnePausedWork::FCanonicalFrameWindowPtr CreateCanonicalTerminalPostSyncRsCaptureWindow(const USpeedAnalyticCollisionAsset& asset,
        FPreparedNativeOwnerInputPtr nativeInput,const FPreparedWorldSnapshotPtr& expected,
        PreparationPrefix::FContractPtr contract) const
    {
        auto* h=Host.Get();auto* s=Bridge.Get();auto* world=World.Get();
        if(!h||!s||!world||!IsFor(*s)||!Worker||!expected||!expected->GetNativeOwners()||!contract||contract->GetPurpose()!=OnePausedWork::EPurpose::TerminalPostSyncRsCaptureStop||
            ReadCanonicalReferenceSnapshotAtReadiness(asset,MoveTemp(nativeInput),false)!=expected||
            !h->bCanonicalFrameInitialized||h->CanonicalNumFrame>=MAX_uint32||s->StepSerial==MAX_uint64)return {};
        const uint64 serial=OnePausedWork::FCanonicalFrameWindow::AllocateSerialOnGT();
        if(!serial||!contract->ClaimWindow(expected,serial))return {};
        const auto context=h->MakeCanonicalFrameContext(h->CanonicalNumFrame);
        return MakeShareable(new OnePausedWork::FCanonicalFrameWindow(expected,world,h,s,s->GetStaticCollisionWorld(),Worker,
            Generation,context.NumFrame,s->StepSerial,serial,context.PhysicalDeltaTime,context.SimTime,
            OnePausedWork::EPurpose::TerminalPostSyncRsCaptureStop,MoveTemp(contract)));
    }
    struct FPreparationResult
    {
        OnePausedWork::FResult Work;
        uint64 Generation=0,FrameBefore=0,FrameAfter=0,StepBefore=0,StepAfter=0;
        uint64 PublicationBefore=0,PublicationAfter=0;int32 HashesBefore=0,HashesAfter=0;
        bool AllPrepareReached=false,PreparationReached=false,PreparationAccepted=false,CountersObserved=false,CountersUnchanged=false,HostRetirementClosed=false;
        bool HostCountersUnchanged=false,StepSerialAdvancedOnce=false;
    };
    FPreparationResult RunOneCanonicalPreparation(OnePausedWork::FCanonicalFrameWindowPtr window,uint32 timeoutMs=1000)
    {
        FPreparationResult out;auto* h=Host.Get();auto* s=Bridge.Get();
        if(!h||!s||!IsFor(*s)||!Worker||!CanonicalFrameDriver::IsEnabled()||
            !h->bCanonicalFrameInitialized||h->CanonicalNumFrame>=MAX_uint32||!s->GetStaticCollisionWorld()||
            !window||window->GetPurpose()!=OnePausedWork::EPurpose::PreparationPrefixStop||
            !window->HasRequiredProviderReadiness(s->GetStaticCollisionWorld())||window->GetNonce()||ActiveWindow)return out;
        {FScopeLock lock(&h->RollbackRequestMutex);if(h->PendingRollbackRequest.IsSet())return out;}
        const auto context=h->MakeCanonicalFrameContext(h->CanonicalNumFrame);
        OnePausedWork::FCanonicalIdentity identity;
        identity.Host=h;identity.Bridge=s;identity.Provider=s->GetStaticCollisionWorld();identity.Worker=Worker;
        identity.Generation=Generation;identity.Frame=h->CanonicalNumFrame;identity.Delta=context.PhysicalDeltaTime;
        identity.SimTime=context.SimTime;identity.Window=window;
        if(!window->Matches(h,s,identity.Provider,Worker,Generation,identity.Frame,identity.Delta,identity.SimTime)||
            window->GetSnapshot()!=identity.Provider->GetPreparedSnapshot()||window->GetStepBefore()!=s->StepSerial)return out;
        const auto before=ReadPreparationHostCounters(*h);
        out.PublicationBefore=before.PublicationSerial;out.HashesBefore=before.FrameHashCount;
        out.Generation=Generation;out.FrameBefore=h->CanonicalNumFrame;out.StepBefore=s->StepSerial;ActiveWindow=window;
        out.Work=Worker->RunOnePausedWork(identity,[&]
        {
            OwnDepth=false;s->StaticWorldPublicationDepth=0;h->bStaticWorldPublicationLease=false;h->OwnedBoundarySuspendDepth=0;
        },timeoutMs);
        if(!out.Work.Nonce)return out;
        // No lease recapture and no success park/ACK: this path is terminal.
        window->Close();const auto contract=window->GetPreparationContract();
        out.PreparationReached=out.Work.GuardsAccepted&&out.Work.WorkReturned&&
            out.Work.WorkResult==uint8(ESimulationWorkerResult::PreparationPrefixStopped)&&contract&&contract->HasReturnedStop();
        if(h->InputWorkerGeneration==Generation&&h->SimulationWorker.Get()==Worker)
            out.HostRetirementClosed=h->JoinOwnedSimulationForInputTeardown();
        else out.HostRetirementClosed=h->InputOwnersRetiredAfterJoin(Generation);
        if(h->InputWorkerGeneration==Generation&&!h->SimulationWorker)
        {
            out.FrameAfter=h->CanonicalNumFrame;out.StepAfter=s->StepSerial;out.CountersObserved=true;Worker=nullptr;
            const auto after=ReadPreparationHostCounters(*h);
            out.PublicationAfter=after.PublicationSerial;out.HashesAfter=after.FrameHashCount;
            out.CountersUnchanged=after==before&&out.StepAfter==out.StepBefore;
        }
        // Closing the registry alone is insufficient if real adapter retirement
        // or neutralization poisoned canonical publication during cleanup.
        out.HostRetirementClosed=out.HostRetirementClosed&&!h->bCanonicalPublicationTerminal.Load();
        out.PreparationAccepted=out.PreparationReached&&!out.Work.DeadlineExceeded&&out.CountersUnchanged&&out.HostRetirementClosed;
        out.AllPrepareReached=contract&&contract->HasReachedAllPrepare(); // actual pre-retirement boundary
        out.PreparationReached=contract&&contract->Reached.load(); // whole prefix stop, separate from acceptance
        const bool joined=out.Work.Quiescence==OnePausedWork::EQuiescence::ClosedAndJoined;
        if(contract){contract->Joined=joined;contract->OwnersRetired=joined&&out.HostRetirementClosed;}
        out.Work.Quiescence=joined&&out.HostRetirementClosed&&h->InputWorkerGeneration==Generation&&
            !h->SimulationWorker&&h->bInputOwnerRetired.Load()&&h->bInputOwnersClosedOnWorker.Load()
            ?OnePausedWork::EQuiescence::ClosedAndJoined:OnePausedWork::EQuiescence::None;
        return out;
    }
    FPreparationResult RunOneCanonicalPostPreparePosition(OnePausedWork::FCanonicalFrameWindowPtr window,uint32 timeoutMs=1000)
    {
        FPreparationResult out;auto* h=Host.Get();auto* s=Bridge.Get();
        if(!h||!s||!IsFor(*s)||!Worker||!CanonicalFrameDriver::IsEnabled()||
            !h->bCanonicalFrameInitialized||h->CanonicalNumFrame>=MAX_uint32||!s->GetStaticCollisionWorld()||
            !window||window->GetPurpose()!=OnePausedWork::EPurpose::PostPrepareInitialPositionStop||
            !window->HasRequiredProviderReadiness(s->GetStaticCollisionWorld())||window->GetNonce()||ActiveWindow)return out;
        {FScopeLock lock(&h->RollbackRequestMutex);if(h->PendingRollbackRequest.IsSet())return out;}
        const auto context=h->MakeCanonicalFrameContext(h->CanonicalNumFrame);
        OnePausedWork::FCanonicalIdentity identity;
        identity.Host=h;identity.Bridge=s;identity.Provider=s->GetStaticCollisionWorld();identity.Worker=Worker;
        identity.Generation=Generation;identity.Frame=h->CanonicalNumFrame;identity.Delta=context.PhysicalDeltaTime;
        identity.SimTime=context.SimTime;identity.Window=window;
        if(!window->Matches(h,s,identity.Provider,Worker,Generation,identity.Frame,identity.Delta,identity.SimTime)||
            window->GetSnapshot()!=identity.Provider->GetPreparedSnapshot()||window->GetStepBefore()!=s->StepSerial)return out;
        const auto before=ReadPreparationHostCounters(*h);
        out.PublicationBefore=before.PublicationSerial;out.HashesBefore=before.FrameHashCount;
        out.Generation=Generation;out.FrameBefore=h->CanonicalNumFrame;out.StepBefore=s->StepSerial;ActiveWindow=window;
        out.Work=Worker->RunOnePausedWork(identity,[&]
        {
            OwnDepth=false;s->StaticWorldPublicationDepth=0;h->bStaticWorldPublicationLease=false;h->OwnedBoundarySuspendDepth=0;
        },timeoutMs);
        if(!out.Work.Nonce)return out;
        // No lease recapture and no success park/ACK: this path is terminal.
        window->Close();const auto contract=window->GetPreparationContract();
        out.PreparationReached=out.Work.GuardsAccepted&&out.Work.WorkReturned&&
            out.Work.WorkResult==uint8(ESimulationWorkerResult::PostPrepareInitialPositionStopped)&&contract&&contract->GetPurpose()==OnePausedWork::EPurpose::PostPrepareInitialPositionStop&&contract->HasAcceptedPosition()&&contract->HasReturnedStop();
        if(h->InputWorkerGeneration==Generation&&h->SimulationWorker.Get()==Worker)
            out.HostRetirementClosed=h->JoinOwnedSimulationForInputTeardown();
        else out.HostRetirementClosed=h->InputOwnersRetiredAfterJoin(Generation);
        if(h->InputWorkerGeneration==Generation&&!h->SimulationWorker)
        {
            out.FrameAfter=h->CanonicalNumFrame;out.StepAfter=s->StepSerial;out.CountersObserved=true;Worker=nullptr;
            const auto after=ReadPreparationHostCounters(*h);
            out.PublicationAfter=after.PublicationSerial;out.HashesAfter=after.FrameHashCount;
            out.CountersUnchanged=after==before&&out.StepAfter==out.StepBefore;
        }
        // Closing the registry alone is insufficient if real adapter retirement
        // or neutralization poisoned canonical publication during cleanup.
        out.HostRetirementClosed=out.HostRetirementClosed&&!h->bCanonicalPublicationTerminal.Load();
        out.PreparationAccepted=out.PreparationReached&&!out.Work.DeadlineExceeded&&out.CountersUnchanged&&out.HostRetirementClosed;
        out.AllPrepareReached=contract&&contract->HasReachedAllPrepare(); // actual pre-retirement boundary
        out.PreparationReached=contract&&contract->Reached.load(); // whole prefix stop, separate from acceptance
        const bool joined=out.Work.Quiescence==OnePausedWork::EQuiescence::ClosedAndJoined;
        if(contract){contract->Joined=joined;contract->OwnersRetired=joined&&out.HostRetirementClosed;}
        out.Work.Quiescence=joined&&out.HostRetirementClosed&&h->InputWorkerGeneration==Generation&&
            !h->SimulationWorker&&h->bInputOwnerRetired.Load()&&h->bInputOwnersClosedOnWorker.Load()
            ?OnePausedWork::EQuiescence::ClosedAndJoined:OnePausedWork::EQuiescence::None;
        return out;
    }
    FPreparationResult RunOneCanonicalTerminalStepSync(OnePausedWork::FCanonicalFrameWindowPtr window,uint32 timeoutMs=1000)
    {
        FPreparationResult out;auto* h=Host.Get();auto* s=Bridge.Get();
        if(!h||!s||!IsFor(*s)||!Worker||!CanonicalFrameDriver::IsEnabled()||
            !h->bCanonicalFrameInitialized||h->CanonicalNumFrame>=MAX_uint32||!s->GetStaticCollisionWorld()||
            !window||window->GetPurpose()!=OnePausedWork::EPurpose::TerminalStepSyncStop||
            !window->HasRequiredProviderReadiness(s->GetStaticCollisionWorld())||window->GetNonce()||ActiveWindow)return out;
        {FScopeLock lock(&h->RollbackRequestMutex);if(h->PendingRollbackRequest.IsSet())return out;}
        const auto context=h->MakeCanonicalFrameContext(h->CanonicalNumFrame);
        OnePausedWork::FCanonicalIdentity identity;
        identity.Host=h;identity.Bridge=s;identity.Provider=s->GetStaticCollisionWorld();identity.Worker=Worker;
        identity.Generation=Generation;identity.Frame=h->CanonicalNumFrame;identity.Delta=context.PhysicalDeltaTime;
        identity.SimTime=context.SimTime;identity.Window=window;
        if(!window->Matches(h,s,identity.Provider,Worker,Generation,identity.Frame,identity.Delta,identity.SimTime)||
            window->GetSnapshot()!=identity.Provider->GetPreparedSnapshot()||window->GetStepBefore()!=s->StepSerial)return out;
        const auto before=ReadPreparationHostCounters(*h);
        out.PublicationBefore=before.PublicationSerial;out.HashesBefore=before.FrameHashCount;
        out.Generation=Generation;out.FrameBefore=h->CanonicalNumFrame;out.StepBefore=s->StepSerial;ActiveWindow=window;
        out.Work=Worker->RunOnePausedWork(identity,[&]
        {
            OwnDepth=false;s->StaticWorldPublicationDepth=0;h->bStaticWorldPublicationLease=false;h->OwnedBoundarySuspendDepth=0;
        },timeoutMs);
        if(!out.Work.Nonce)return out;
        // No lease recapture and no success park/ACK: this path is terminal.
        window->Close();const auto contract=window->GetPreparationContract();
        out.PreparationReached=out.Work.GuardsAccepted&&out.Work.WorkReturned&&
            out.Work.WorkResult==uint8(ESimulationWorkerResult::TerminalStepSyncStopped)&&contract&&contract->GetPurpose()==OnePausedWork::EPurpose::TerminalStepSyncStop&&contract->HasCompletedStepSync()&&contract->HasReturnedStop();
        if(h->InputWorkerGeneration==Generation&&h->SimulationWorker.Get()==Worker)
            out.HostRetirementClosed=h->JoinOwnedSimulationForInputTeardown();
        else out.HostRetirementClosed=h->InputOwnersRetiredAfterJoin(Generation);
        if(h->InputWorkerGeneration==Generation&&!h->SimulationWorker)
        {
            out.FrameAfter=h->CanonicalNumFrame;out.StepAfter=s->StepSerial;out.CountersObserved=true;Worker=nullptr;
            const auto after=ReadPreparationHostCounters(*h);
            out.PublicationAfter=after.PublicationSerial;out.HashesAfter=after.FrameHashCount;
            out.HostCountersUnchanged=after==before;
            out.StepSerialAdvancedOnce=out.StepBefore<MAX_uint64&&out.StepAfter==out.StepBefore+1;
            // Preserve the old meaning: this is false when a real Step occurred.
            out.CountersUnchanged=out.HostCountersUnchanged&&out.StepAfter==out.StepBefore;
        }
        // Closing the registry alone is insufficient if real adapter retirement
        // or neutralization poisoned canonical publication during cleanup.
        out.HostRetirementClosed=out.HostRetirementClosed&&!h->bCanonicalPublicationTerminal.Load();
        out.PreparationAccepted=out.PreparationReached&&!out.Work.DeadlineExceeded&&out.CountersObserved&&out.HostCountersUnchanged&&out.StepSerialAdvancedOnce&&out.HostRetirementClosed;
        out.AllPrepareReached=contract&&contract->HasReachedAllPrepare(); // actual pre-retirement boundary
        out.PreparationReached=contract&&contract->Reached.load(); // whole prefix stop, separate from acceptance
        const bool joined=out.Work.Quiescence==OnePausedWork::EQuiescence::ClosedAndJoined;
        if(contract){contract->Joined=joined;contract->OwnersRetired=joined&&out.HostRetirementClosed;}
        out.Work.Quiescence=joined&&out.HostRetirementClosed&&h->InputWorkerGeneration==Generation&&
            !h->SimulationWorker&&h->bInputOwnerRetired.Load()&&h->bInputOwnersClosedOnWorker.Load()
            ?OnePausedWork::EQuiescence::ClosedAndJoined:OnePausedWork::EQuiescence::None;
        return out;
    }
    FPreparationResult RunOneCanonicalTerminalSphereRsCapture(OnePausedWork::FCanonicalFrameWindowPtr window,uint32 timeoutMs=1000)
    {
        FPreparationResult out;auto* h=Host.Get();auto* s=Bridge.Get();
        if(!h||!s||!IsFor(*s)||!Worker||!CanonicalFrameDriver::IsEnabled()||
            !h->bCanonicalFrameInitialized||h->CanonicalNumFrame>=MAX_uint32||!s->GetStaticCollisionWorld()||
            !window||window->GetPurpose()!=OnePausedWork::EPurpose::TerminalSphereRsCaptureStop||
            !window->HasRequiredProviderReadiness(s->GetStaticCollisionWorld())||window->GetNonce()||ActiveWindow)return out;
        {FScopeLock lock(&h->RollbackRequestMutex);if(h->PendingRollbackRequest.IsSet())return out;}
        const auto context=h->MakeCanonicalFrameContext(h->CanonicalNumFrame);
        OnePausedWork::FCanonicalIdentity identity;
        identity.Host=h;identity.Bridge=s;identity.Provider=s->GetStaticCollisionWorld();identity.Worker=Worker;
        identity.Generation=Generation;identity.Frame=h->CanonicalNumFrame;identity.Delta=context.PhysicalDeltaTime;
        identity.SimTime=context.SimTime;identity.Window=window;
        if(!window->Matches(h,s,identity.Provider,Worker,Generation,identity.Frame,identity.Delta,identity.SimTime)||
            window->GetSnapshot()!=identity.Provider->GetPreparedSnapshot()||window->GetStepBefore()!=s->StepSerial)return out;
        const auto before=ReadPreparationHostCounters(*h);
        out.PublicationBefore=before.PublicationSerial;out.HashesBefore=before.FrameHashCount;
        out.Generation=Generation;out.FrameBefore=h->CanonicalNumFrame;out.StepBefore=s->StepSerial;ActiveWindow=window;
        out.Work=Worker->RunOnePausedWork(identity,[&]
        {
            OwnDepth=false;s->StaticWorldPublicationDepth=0;h->bStaticWorldPublicationLease=false;h->OwnedBoundarySuspendDepth=0;
        },timeoutMs);
        if(!out.Work.Nonce)return out;
        // No lease recapture and no success park/ACK: this path is terminal.
        window->Close();const auto contract=window->GetPreparationContract();
        out.PreparationReached=out.Work.GuardsAccepted&&out.Work.WorkReturned&&
            out.Work.WorkResult==uint8(ESimulationWorkerResult::TerminalSphereRsCaptureStopped)&&contract&&contract->GetPurpose()==OnePausedWork::EPurpose::TerminalSphereRsCaptureStop&&contract->HasClosedNativeRsCapture()&&contract->HasReturnedStop();
        if(h->InputWorkerGeneration==Generation&&h->SimulationWorker.Get()==Worker)
            out.HostRetirementClosed=h->JoinOwnedSimulationForInputTeardown();
        else out.HostRetirementClosed=h->InputOwnersRetiredAfterJoin(Generation);
        if(h->InputWorkerGeneration==Generation&&!h->SimulationWorker)
        {
            out.FrameAfter=h->CanonicalNumFrame;out.StepAfter=s->StepSerial;out.CountersObserved=true;Worker=nullptr;
            const auto after=ReadPreparationHostCounters(*h);
            out.PublicationAfter=after.PublicationSerial;out.HashesAfter=after.FrameHashCount;
            out.HostCountersUnchanged=after==before;
            out.StepSerialAdvancedOnce=out.StepBefore<MAX_uint64&&out.StepAfter==out.StepBefore+1;
            // Preserve the old meaning: this is false when a real Step occurred.
            out.CountersUnchanged=out.HostCountersUnchanged&&out.StepAfter==out.StepBefore;
        }
        // Closing the registry alone is insufficient if real adapter retirement
        // or neutralization poisoned canonical publication during cleanup.
        out.HostRetirementClosed=out.HostRetirementClosed&&!h->bCanonicalPublicationTerminal.Load();
        out.PreparationAccepted=out.PreparationReached&&!out.Work.DeadlineExceeded&&out.CountersObserved&&out.HostCountersUnchanged&&out.StepSerialAdvancedOnce&&out.HostRetirementClosed;
        out.AllPrepareReached=contract&&contract->HasReachedAllPrepare(); // actual pre-retirement boundary
        out.PreparationReached=contract&&contract->Reached.load(); // whole prefix stop, separate from acceptance
        const bool joined=out.Work.Quiescence==OnePausedWork::EQuiescence::ClosedAndJoined;
        if(contract){contract->Joined=joined;contract->OwnersRetired=joined&&out.HostRetirementClosed;}
        out.Work.Quiescence=joined&&out.HostRetirementClosed&&h->InputWorkerGeneration==Generation&&
            !h->SimulationWorker&&h->bInputOwnerRetired.Load()&&h->bInputOwnersClosedOnWorker.Load()
            ?OnePausedWork::EQuiescence::ClosedAndJoined:OnePausedWork::EQuiescence::None;
        return out;
    }
    FPreparationResult RunOneCanonicalTerminalConsumedPrefixAdvance(OnePausedWork::FCanonicalFrameWindowPtr window,uint32 timeoutMs=1000)
    {
        FPreparationResult out;auto* h=Host.Get();auto* s=Bridge.Get();
        if(!h||!s||!IsFor(*s)||!Worker||!CanonicalFrameDriver::IsEnabled()||
            !h->bCanonicalFrameInitialized||h->CanonicalNumFrame>=MAX_uint32||!s->GetStaticCollisionWorld()||
            !window||window->GetPurpose()!=OnePausedWork::EPurpose::TerminalConsumedPrefixAdvanceStop||
            !window->HasRequiredProviderReadiness(s->GetStaticCollisionWorld())||window->GetNonce()||ActiveWindow)return out;
        {FScopeLock lock(&h->RollbackRequestMutex);if(h->PendingRollbackRequest.IsSet())return out;}
        const auto context=h->MakeCanonicalFrameContext(h->CanonicalNumFrame);
        OnePausedWork::FCanonicalIdentity identity;
        identity.Host=h;identity.Bridge=s;identity.Provider=s->GetStaticCollisionWorld();identity.Worker=Worker;
        identity.Generation=Generation;identity.Frame=h->CanonicalNumFrame;identity.Delta=context.PhysicalDeltaTime;
        identity.SimTime=context.SimTime;identity.Window=window;
        if(!window->Matches(h,s,identity.Provider,Worker,Generation,identity.Frame,identity.Delta,identity.SimTime)||
            window->GetSnapshot()!=identity.Provider->GetPreparedSnapshot()||window->GetStepBefore()!=s->StepSerial)return out;
        const auto before=ReadPreparationHostCounters(*h);
        out.PublicationBefore=before.PublicationSerial;out.HashesBefore=before.FrameHashCount;
        out.Generation=Generation;out.FrameBefore=h->CanonicalNumFrame;out.StepBefore=s->StepSerial;ActiveWindow=window;
        out.Work=Worker->RunOnePausedWork(identity,[&]
        {
            OwnDepth=false;s->StaticWorldPublicationDepth=0;h->bStaticWorldPublicationLease=false;h->OwnedBoundarySuspendDepth=0;
        },timeoutMs);
        if(!out.Work.Nonce)return out;
        // No lease recapture and no success park/ACK: this path is terminal.
        window->Close();const auto contract=window->GetPreparationContract();
        out.PreparationReached=out.Work.GuardsAccepted&&out.Work.WorkReturned&&
            out.Work.WorkResult==uint8(ESimulationWorkerResult::TerminalConsumedPrefixAdvanceStopped)&&contract&&contract->GetPurpose()==OnePausedWork::EPurpose::TerminalConsumedPrefixAdvanceStop&&contract->HasCompletedNativeRsAdvance()&&contract->HasReturnedStop();
        if(h->InputWorkerGeneration==Generation&&h->SimulationWorker.Get()==Worker)
            out.HostRetirementClosed=h->JoinOwnedSimulationForInputTeardown();
        else out.HostRetirementClosed=h->InputOwnersRetiredAfterJoin(Generation);
        if(h->InputWorkerGeneration==Generation&&!h->SimulationWorker)
        {
            out.FrameAfter=h->CanonicalNumFrame;out.StepAfter=s->StepSerial;out.CountersObserved=true;Worker=nullptr;
            const auto after=ReadPreparationHostCounters(*h);
            out.PublicationAfter=after.PublicationSerial;out.HashesAfter=after.FrameHashCount;
            out.HostCountersUnchanged=after==before;
            out.StepSerialAdvancedOnce=out.StepBefore<MAX_uint64&&out.StepAfter==out.StepBefore+1;
            // Preserve the old meaning: this is false when a real Step occurred.
            out.CountersUnchanged=out.HostCountersUnchanged&&out.StepAfter==out.StepBefore;
        }
        // Closing the registry alone is insufficient if real adapter retirement
        // or neutralization poisoned canonical publication during cleanup.
        out.HostRetirementClosed=out.HostRetirementClosed&&!h->bCanonicalPublicationTerminal.Load();
        out.PreparationAccepted=out.PreparationReached&&!out.Work.DeadlineExceeded&&out.CountersObserved&&out.HostCountersUnchanged&&out.StepSerialAdvancedOnce&&out.HostRetirementClosed;
        out.AllPrepareReached=contract&&contract->HasReachedAllPrepare(); // actual pre-retirement boundary
        out.PreparationReached=contract&&contract->Reached.load(); // whole prefix stop, separate from acceptance
        const bool joined=out.Work.Quiescence==OnePausedWork::EQuiescence::ClosedAndJoined;
        if(contract){contract->Joined=joined;contract->OwnersRetired=joined&&out.HostRetirementClosed;}
        out.Work.Quiescence=joined&&out.HostRetirementClosed&&h->InputWorkerGeneration==Generation&&
            !h->SimulationWorker&&h->bInputOwnerRetired.Load()&&h->bInputOwnersClosedOnWorker.Load()
            ?OnePausedWork::EQuiescence::ClosedAndJoined:OnePausedWork::EQuiescence::None;
        return out;
    }
    FPreparationResult RunOneCanonicalTerminalRsContactAdvance(OnePausedWork::FCanonicalFrameWindowPtr window,uint32 timeoutMs=1000)
    {
        FPreparationResult out;auto* h=Host.Get();auto* s=Bridge.Get();
        if(!h||!s||!IsFor(*s)||!Worker||!CanonicalFrameDriver::IsEnabled()||
            !h->bCanonicalFrameInitialized||h->CanonicalNumFrame>=MAX_uint32||!s->GetStaticCollisionWorld()||
            !window||window->GetPurpose()!=OnePausedWork::EPurpose::TerminalRsContactAdvanceStop||
            !window->HasRequiredProviderReadiness(s->GetStaticCollisionWorld())||window->GetNonce()||ActiveWindow)return out;
        {FScopeLock lock(&h->RollbackRequestMutex);if(h->PendingRollbackRequest.IsSet())return out;}
        const auto context=h->MakeCanonicalFrameContext(h->CanonicalNumFrame);
        OnePausedWork::FCanonicalIdentity identity;
        identity.Host=h;identity.Bridge=s;identity.Provider=s->GetStaticCollisionWorld();identity.Worker=Worker;
        identity.Generation=Generation;identity.Frame=h->CanonicalNumFrame;identity.Delta=context.PhysicalDeltaTime;
        identity.SimTime=context.SimTime;identity.Window=window;
        if(!window->Matches(h,s,identity.Provider,Worker,Generation,identity.Frame,identity.Delta,identity.SimTime)||
            window->GetSnapshot()!=identity.Provider->GetPreparedSnapshot()||window->GetStepBefore()!=s->StepSerial)return out;
        const auto before=ReadPreparationHostCounters(*h);
        out.PublicationBefore=before.PublicationSerial;out.HashesBefore=before.FrameHashCount;
        out.Generation=Generation;out.FrameBefore=h->CanonicalNumFrame;out.StepBefore=s->StepSerial;ActiveWindow=window;
        out.Work=Worker->RunOnePausedWork(identity,[&]
        {
            OwnDepth=false;s->StaticWorldPublicationDepth=0;h->bStaticWorldPublicationLease=false;h->OwnedBoundarySuspendDepth=0;
        },timeoutMs);
        if(!out.Work.Nonce)return out;
        // No lease recapture and no success park/ACK: this path is terminal.
        window->Close();const auto contract=window->GetPreparationContract();
        out.PreparationReached=out.Work.GuardsAccepted&&out.Work.WorkReturned&&
            out.Work.WorkResult==uint8(ESimulationWorkerResult::TerminalRsContactAdvanceStopped)&&contract&&contract->GetPurpose()==OnePausedWork::EPurpose::TerminalRsContactAdvanceStop&&contract->HasCompletedNativeRsContactAdvance()&&contract->HasReturnedStop();
        if(h->InputWorkerGeneration==Generation&&h->SimulationWorker.Get()==Worker)
            out.HostRetirementClosed=h->JoinOwnedSimulationForInputTeardown();
        else out.HostRetirementClosed=h->InputOwnersRetiredAfterJoin(Generation);
        if(h->InputWorkerGeneration==Generation&&!h->SimulationWorker)
        {
            out.FrameAfter=h->CanonicalNumFrame;out.StepAfter=s->StepSerial;out.CountersObserved=true;Worker=nullptr;
            const auto after=ReadPreparationHostCounters(*h);
            out.PublicationAfter=after.PublicationSerial;out.HashesAfter=after.FrameHashCount;
            out.HostCountersUnchanged=after==before;
            out.StepSerialAdvancedOnce=out.StepBefore<MAX_uint64&&out.StepAfter==out.StepBefore+1;
            // Preserve the old meaning: this is false when a real Step occurred.
            out.CountersUnchanged=out.HostCountersUnchanged&&out.StepAfter==out.StepBefore;
        }
        // Closing the registry alone is insufficient if real adapter retirement
        // or neutralization poisoned canonical publication during cleanup.
        out.HostRetirementClosed=out.HostRetirementClosed&&!h->bCanonicalPublicationTerminal.Load();
        out.PreparationAccepted=out.PreparationReached&&!out.Work.DeadlineExceeded&&out.CountersObserved&&out.HostCountersUnchanged&&out.StepSerialAdvancedOnce&&out.HostRetirementClosed;
        out.AllPrepareReached=contract&&contract->HasReachedAllPrepare(); // actual pre-retirement boundary
        out.PreparationReached=contract&&contract->Reached.load(); // whole prefix stop, separate from acceptance
        const bool joined=out.Work.Quiescence==OnePausedWork::EQuiescence::ClosedAndJoined;
        if(contract){contract->Joined=joined;contract->OwnersRetired=joined&&out.HostRetirementClosed;}
        out.Work.Quiescence=joined&&out.HostRetirementClosed&&h->InputWorkerGeneration==Generation&&
            !h->SimulationWorker&&h->bInputOwnerRetired.Load()&&h->bInputOwnersClosedOnWorker.Load()
            ?OnePausedWork::EQuiescence::ClosedAndJoined:OnePausedWork::EQuiescence::None;
        return out;
    }
    FPreparationResult RunOneCanonicalTerminalRsResponse(OnePausedWork::FCanonicalFrameWindowPtr window,uint32 timeoutMs=1000)
    {
        FPreparationResult out;auto* h=Host.Get();auto* s=Bridge.Get();
        if(!h||!s||!IsFor(*s)||!Worker||!CanonicalFrameDriver::IsEnabled()||
            !h->bCanonicalFrameInitialized||h->CanonicalNumFrame>=MAX_uint32||!s->GetStaticCollisionWorld()||
            !window||window->GetPurpose()!=OnePausedWork::EPurpose::TerminalRsResponseStop||
            !window->HasRequiredProviderReadiness(s->GetStaticCollisionWorld())||window->GetNonce()||ActiveWindow)return out;
        {FScopeLock lock(&h->RollbackRequestMutex);if(h->PendingRollbackRequest.IsSet())return out;}
        const auto context=h->MakeCanonicalFrameContext(h->CanonicalNumFrame);
        OnePausedWork::FCanonicalIdentity identity;
        identity.Host=h;identity.Bridge=s;identity.Provider=s->GetStaticCollisionWorld();identity.Worker=Worker;
        identity.Generation=Generation;identity.Frame=h->CanonicalNumFrame;identity.Delta=context.PhysicalDeltaTime;
        identity.SimTime=context.SimTime;identity.Window=window;
        if(!window->Matches(h,s,identity.Provider,Worker,Generation,identity.Frame,identity.Delta,identity.SimTime)||
            window->GetSnapshot()!=identity.Provider->GetPreparedSnapshot()||window->GetStepBefore()!=s->StepSerial)return out;
        const auto before=ReadPreparationHostCounters(*h);
        out.PublicationBefore=before.PublicationSerial;out.HashesBefore=before.FrameHashCount;
        out.Generation=Generation;out.FrameBefore=h->CanonicalNumFrame;out.StepBefore=s->StepSerial;ActiveWindow=window;
        out.Work=Worker->RunOnePausedWork(identity,[&]
        {
            OwnDepth=false;s->StaticWorldPublicationDepth=0;h->bStaticWorldPublicationLease=false;h->OwnedBoundarySuspendDepth=0;
        },timeoutMs);
        if(!out.Work.Nonce)return out;
        // No lease recapture and no success park/ACK: this path is terminal.
        window->Close();const auto contract=window->GetPreparationContract();
        out.PreparationReached=out.Work.GuardsAccepted&&out.Work.WorkReturned&&
            out.Work.WorkResult==uint8(ESimulationWorkerResult::TerminalRsResponseStopped)&&contract&&contract->GetPurpose()==OnePausedWork::EPurpose::TerminalRsResponseStop&&contract->HasCompletedNativeRsResponse()&&contract->HasReturnedStop();
        if(h->InputWorkerGeneration==Generation&&h->SimulationWorker.Get()==Worker)
            out.HostRetirementClosed=h->JoinOwnedSimulationForInputTeardown();
        else out.HostRetirementClosed=h->InputOwnersRetiredAfterJoin(Generation);
        if(h->InputWorkerGeneration==Generation&&!h->SimulationWorker)
        {
            out.FrameAfter=h->CanonicalNumFrame;out.StepAfter=s->StepSerial;out.CountersObserved=true;Worker=nullptr;
            const auto after=ReadPreparationHostCounters(*h);
            out.PublicationAfter=after.PublicationSerial;out.HashesAfter=after.FrameHashCount;
            out.HostCountersUnchanged=after==before;
            out.StepSerialAdvancedOnce=out.StepBefore<MAX_uint64&&out.StepAfter==out.StepBefore+1;
            // Preserve the old meaning: this is false when a real Step occurred.
            out.CountersUnchanged=out.HostCountersUnchanged&&out.StepAfter==out.StepBefore;
        }
        // Closing the registry alone is insufficient if real adapter retirement
        // or neutralization poisoned canonical publication during cleanup.
        out.HostRetirementClosed=out.HostRetirementClosed&&!h->bCanonicalPublicationTerminal.Load();
        out.PreparationAccepted=out.PreparationReached&&!out.Work.DeadlineExceeded&&out.CountersObserved&&out.HostCountersUnchanged&&out.StepSerialAdvancedOnce&&out.HostRetirementClosed;
        out.AllPrepareReached=contract&&contract->HasReachedAllPrepare(); // actual pre-retirement boundary
        out.PreparationReached=contract&&contract->Reached.load(); // whole prefix stop, separate from acceptance
        const bool joined=out.Work.Quiescence==OnePausedWork::EQuiescence::ClosedAndJoined;
        if(contract){contract->Joined=joined;contract->OwnersRetired=joined&&out.HostRetirementClosed;}
        out.Work.Quiescence=joined&&out.HostRetirementClosed&&h->InputWorkerGeneration==Generation&&
            !h->SimulationWorker&&h->bInputOwnerRetired.Load()&&h->bInputOwnersClosedOnWorker.Load()
            ?OnePausedWork::EQuiescence::ClosedAndJoined:OnePausedWork::EQuiescence::None;
        return out;
    }
    FPreparationResult RunOneCanonicalTerminalRsResponseSync(OnePausedWork::FCanonicalFrameWindowPtr window,uint32 timeoutMs=1000)
    {
        FPreparationResult out;auto* h=Host.Get();auto* s=Bridge.Get();
        if(!h||!s||!IsFor(*s)||!Worker||!CanonicalFrameDriver::IsEnabled()||
            !h->bCanonicalFrameInitialized||h->CanonicalNumFrame>=MAX_uint32||!s->GetStaticCollisionWorld()||
            !window||window->GetPurpose()!=OnePausedWork::EPurpose::TerminalRsResponseSyncStop||
            !window->HasRequiredProviderReadiness(s->GetStaticCollisionWorld())||window->GetNonce()||ActiveWindow)return out;
        {FScopeLock lock(&h->RollbackRequestMutex);if(h->PendingRollbackRequest.IsSet())return out;}
        const auto context=h->MakeCanonicalFrameContext(h->CanonicalNumFrame);
        OnePausedWork::FCanonicalIdentity identity;
        identity.Host=h;identity.Bridge=s;identity.Provider=s->GetStaticCollisionWorld();identity.Worker=Worker;
        identity.Generation=Generation;identity.Frame=h->CanonicalNumFrame;identity.Delta=context.PhysicalDeltaTime;
        identity.SimTime=context.SimTime;identity.Window=window;
        if(!window->Matches(h,s,identity.Provider,Worker,Generation,identity.Frame,identity.Delta,identity.SimTime)||
            window->GetSnapshot()!=identity.Provider->GetPreparedSnapshot()||window->GetStepBefore()!=s->StepSerial)return out;
        const auto before=ReadPreparationHostCounters(*h);
        out.PublicationBefore=before.PublicationSerial;out.HashesBefore=before.FrameHashCount;
        out.Generation=Generation;out.FrameBefore=h->CanonicalNumFrame;out.StepBefore=s->StepSerial;ActiveWindow=window;
        out.Work=Worker->RunOnePausedWork(identity,[&]
        {
            OwnDepth=false;s->StaticWorldPublicationDepth=0;h->bStaticWorldPublicationLease=false;h->OwnedBoundarySuspendDepth=0;
        },timeoutMs);
        if(!out.Work.Nonce)return out;
        // No lease recapture and no success park/ACK: this path is terminal.
        window->Close();const auto contract=window->GetPreparationContract();
        out.PreparationReached=out.Work.GuardsAccepted&&out.Work.WorkReturned&&
            out.Work.WorkResult==uint8(ESimulationWorkerResult::TerminalRsResponseSyncStopped)&&contract&&contract->GetPurpose()==OnePausedWork::EPurpose::TerminalRsResponseSyncStop&&contract->HasCompletedNativeRsResponseSync()&&contract->HasReturnedStop();
        if(h->InputWorkerGeneration==Generation&&h->SimulationWorker.Get()==Worker)
            out.HostRetirementClosed=h->JoinOwnedSimulationForInputTeardown();
        else out.HostRetirementClosed=h->InputOwnersRetiredAfterJoin(Generation);
        if(h->InputWorkerGeneration==Generation&&!h->SimulationWorker)
        {
            out.FrameAfter=h->CanonicalNumFrame;out.StepAfter=s->StepSerial;out.CountersObserved=true;Worker=nullptr;
            const auto after=ReadPreparationHostCounters(*h);
            out.PublicationAfter=after.PublicationSerial;out.HashesAfter=after.FrameHashCount;
            out.HostCountersUnchanged=after==before;
            out.StepSerialAdvancedOnce=out.StepBefore<MAX_uint64&&out.StepAfter==out.StepBefore+1;
            // Preserve the old meaning: this is false when a real Step occurred.
            out.CountersUnchanged=out.HostCountersUnchanged&&out.StepAfter==out.StepBefore;
        }
        // Closing the registry alone is insufficient if real adapter retirement
        // or neutralization poisoned canonical publication during cleanup.
        out.HostRetirementClosed=out.HostRetirementClosed&&!h->bCanonicalPublicationTerminal.Load();
        out.PreparationAccepted=out.PreparationReached&&!out.Work.DeadlineExceeded&&out.CountersObserved&&out.HostCountersUnchanged&&out.StepSerialAdvancedOnce&&out.HostRetirementClosed;
        out.AllPrepareReached=contract&&contract->HasReachedAllPrepare(); // actual pre-retirement boundary
        out.PreparationReached=contract&&contract->Reached.load(); // whole prefix stop, separate from acceptance
        const bool joined=out.Work.Quiescence==OnePausedWork::EQuiescence::ClosedAndJoined;
        if(contract){contract->Joined=joined;contract->OwnersRetired=joined&&out.HostRetirementClosed;}
        out.Work.Quiescence=joined&&out.HostRetirementClosed&&h->InputWorkerGeneration==Generation&&
            !h->SimulationWorker&&h->bInputOwnerRetired.Load()&&h->bInputOwnersClosedOnWorker.Load()
            ?OnePausedWork::EQuiescence::ClosedAndJoined:OnePausedWork::EQuiescence::None;
        return out;
    }
    FPreparationResult RunOneCanonicalTerminalPostSyncRsCapture(OnePausedWork::FCanonicalFrameWindowPtr window,uint32 timeoutMs=1000)
    {
        FPreparationResult out;auto* h=Host.Get();auto* s=Bridge.Get();
        if(!h||!s||!IsFor(*s)||!Worker||!CanonicalFrameDriver::IsEnabled()||
            !h->bCanonicalFrameInitialized||h->CanonicalNumFrame>=MAX_uint32||!s->GetStaticCollisionWorld()||
            !window||window->GetPurpose()!=OnePausedWork::EPurpose::TerminalPostSyncRsCaptureStop||
            !window->HasRequiredProviderReadiness(s->GetStaticCollisionWorld())||window->GetNonce()||ActiveWindow)return out;
        {FScopeLock lock(&h->RollbackRequestMutex);if(h->PendingRollbackRequest.IsSet())return out;}
        const auto context=h->MakeCanonicalFrameContext(h->CanonicalNumFrame);
        OnePausedWork::FCanonicalIdentity identity;
        identity.Host=h;identity.Bridge=s;identity.Provider=s->GetStaticCollisionWorld();identity.Worker=Worker;
        identity.Generation=Generation;identity.Frame=h->CanonicalNumFrame;identity.Delta=context.PhysicalDeltaTime;
        identity.SimTime=context.SimTime;identity.Window=window;
        if(!window->Matches(h,s,identity.Provider,Worker,Generation,identity.Frame,identity.Delta,identity.SimTime)||
            window->GetSnapshot()!=identity.Provider->GetPreparedSnapshot()||window->GetStepBefore()!=s->StepSerial)return out;
        const auto before=ReadPreparationHostCounters(*h);
        out.PublicationBefore=before.PublicationSerial;out.HashesBefore=before.FrameHashCount;
        out.Generation=Generation;out.FrameBefore=h->CanonicalNumFrame;out.StepBefore=s->StepSerial;ActiveWindow=window;
        out.Work=Worker->RunOnePausedWork(identity,[&]
        {
            OwnDepth=false;s->StaticWorldPublicationDepth=0;h->bStaticWorldPublicationLease=false;h->OwnedBoundarySuspendDepth=0;
        },timeoutMs);
        if(!out.Work.Nonce)return out;
        // No lease recapture and no success park/ACK: this path is terminal.
        window->Close();const auto contract=window->GetPreparationContract();
        out.PreparationReached=out.Work.GuardsAccepted&&out.Work.WorkReturned&&
            out.Work.WorkResult==uint8(ESimulationWorkerResult::TerminalPostSyncRsCaptureStopped)&&contract&&contract->GetPurpose()==OnePausedWork::EPurpose::TerminalPostSyncRsCaptureStop&&contract->HasCompletedNativePostSyncRsCapture()&&contract->HasReturnedStop();
        if(h->InputWorkerGeneration==Generation&&h->SimulationWorker.Get()==Worker)
            out.HostRetirementClosed=h->JoinOwnedSimulationForInputTeardown();
        else out.HostRetirementClosed=h->InputOwnersRetiredAfterJoin(Generation);
        if(h->InputWorkerGeneration==Generation&&!h->SimulationWorker)
        {
            out.FrameAfter=h->CanonicalNumFrame;out.StepAfter=s->StepSerial;out.CountersObserved=true;Worker=nullptr;
            const auto after=ReadPreparationHostCounters(*h);
            out.PublicationAfter=after.PublicationSerial;out.HashesAfter=after.FrameHashCount;
            out.HostCountersUnchanged=after==before;
            out.StepSerialAdvancedOnce=out.StepBefore<MAX_uint64&&out.StepAfter==out.StepBefore+1;
            // Preserve the old meaning: this is false when a real Step occurred.
            out.CountersUnchanged=out.HostCountersUnchanged&&out.StepAfter==out.StepBefore;
        }
        // Closing the registry alone is insufficient if real adapter retirement
        // or neutralization poisoned canonical publication during cleanup.
        out.HostRetirementClosed=out.HostRetirementClosed&&!h->bCanonicalPublicationTerminal.Load();
        out.PreparationAccepted=out.PreparationReached&&!out.Work.DeadlineExceeded&&out.CountersObserved&&out.HostCountersUnchanged&&out.StepSerialAdvancedOnce&&out.HostRetirementClosed;
        out.AllPrepareReached=contract&&contract->HasReachedAllPrepare(); // actual pre-retirement boundary
        out.PreparationReached=contract&&contract->Reached.load(); // whole prefix stop, separate from acceptance
        const bool joined=out.Work.Quiescence==OnePausedWork::EQuiescence::ClosedAndJoined;
        if(contract){contract->Joined=joined;contract->OwnersRetired=joined&&out.HostRetirementClosed;}
        out.Work.Quiescence=joined&&out.HostRetirementClosed&&h->InputWorkerGeneration==Generation&&
            !h->SimulationWorker&&h->bInputOwnerRetired.Load()&&h->bInputOwnersClosedOnWorker.Load()
            ?OnePausedWork::EQuiescence::ClosedAndJoined:OnePausedWork::EQuiescence::None;
        return out;
    }
    struct FRetiredPinnedExit {bool WorkerAbsent=false,FullGenerationRetirement=false;};
    FRetiredPinnedExit RetirePinnedWindowAndJoin(ASpeedPhysicsSimulation& h,USpeedWorldSubsystem& s)
    {
        // Only the same actual GT caller may pass its still-pinned objects.
        // No implicit Resume on an aborted window, no fabricated owner receipt.
        FRetiredPinnedExit out;if(ActiveWindow)ActiveWindow->Close();
        if(!IsInGameThread()||(!OwnDepth&&!ActiveWindow)||&h!=Host.GetEvenIfUnreachable()||&s!=Bridge.GetEvenIfUnreachable()||h.InputWorkerGeneration!=Generation)return out;
        // Never retire a replacement worker from an old or moved-from lease.
        if(h.SimulationWorker&&(!Worker||h.SimulationWorker.Get()!=Worker))return out;
        OwnDepth=false;s.StaticWorldPublicationDepth=0;h.bStaticWorldPublicationLease=false;h.OwnedBoundarySuspendDepth=0;
        if(h.SimulationWorker)
        {
            const bool joined=h.JoinOwnedSimulationForInputTeardown();
            out.FullGenerationRetirement=joined&&h.InputOwnersRetiredAfterJoin(Generation);
            if(h.InputWorkerGeneration==Generation&&h.SimulationWorker.Get()==Worker&&h.SimulationWorker)
            {h.bOwnedSimulationPaused.Store(true);h.bOwnedWorkerTerminal.Store(true);h.bCanonicalPublicationTerminal.Store(true);h.StopOwnedWorker();}
        }
        else out.FullGenerationRetirement=h.InputOwnersRetiredAfterJoin(Generation);
        out.WorkerAbsent=h.InputWorkerGeneration==Generation&&!h.SimulationWorker;
        if(out.WorkerAbsent)Worker=nullptr;return out;
    }
private:
    IAMSPEED_API FPreparedWorldSnapshotPtr ReadCanonicalReferenceSnapshotAtReadiness(const USpeedAnalyticCollisionAsset& Asset,FPreparedNativeOwnerInputPtr NativeInput,bool ExpectedReady) const;
public:
    // Lifetime/registry observation ONLY, not physical authority. The mutex
    // protects PendingOps; the actual parked-worker lease protects pair reads.
    bool HasNoPendingOrContactPairsForInitialPosition() const
    {
        auto* s=Bridge.Get();if(!s||!IsFor(*s))return false;
        FScopeLock lock(&s->PendingCS);
        return s->PendingOps.IsEmpty()&&s->SimulationWorld.GetDynamicContactPairs().IsEmpty()&&
            s->SimulationWorld.GetPendingRollingContactPairs().IsEmpty()&&IsFor(*s);
    }
    // Holds the registry queue lock across the final two scalar-copy setters.
    // This guard adds no physical authority; callers must establish their
    // operation-specific geometry/policy/native scope before acquiring it.
    class FInitialPositionRegistryCommit final
    {
        const FHostedGTStaticWorldLease& Lease;const USpeedWorldSubsystem& Subsystem;
        FScopeLock Lock;
        friend class FHostedGTStaticWorldLease;
        FInitialPositionRegistryCommit(const FHostedGTStaticWorldLease& lease,const USpeedWorldSubsystem& s,FCriticalSection& cs)
            :Lease(lease),Subsystem(s),Lock(&cs){}
    public:
        FInitialPositionRegistryCommit(const FInitialPositionRegistryCommit&)=delete;
        FInitialPositionRegistryCommit& operator=(const FInitialPositionRegistryCommit&)=delete;
        // PendingCS is recursive on the supported native Windows platform.
        // The existing inspection also checks the actual parked-worker pair
        // containers while the same outer registry lock remains held.
        bool IsCurrent() const {return Lease.IsFor(Subsystem)&&Lease.HasNoPendingOrContactPairsForInitialPosition();}
    };
    TUniquePtr<FInitialPositionRegistryCommit> TryBeginInitialPositionRegistryCommit() const
    {
        auto* s=Bridge.Get();if(!s||!IsFor(*s))return {};
        auto held=TUniquePtr<FInitialPositionRegistryCommit>(new FInitialPositionRegistryCommit(*this,*s,s->PendingCS));
        if(!held->IsCurrent())return {};return held;
    }
    bool ReadCanonicalCounters(uint64& frame,uint64& serial) const
    {auto* h=Host.Get();auto* s=Bridge.Get();if(!h||!s||!IsFor(*s))return false;frame=h->CanonicalNumFrame;serial=s->StepSerial;return true;}
    uint64 GetGeneration() const {return Generation;}
};
}
