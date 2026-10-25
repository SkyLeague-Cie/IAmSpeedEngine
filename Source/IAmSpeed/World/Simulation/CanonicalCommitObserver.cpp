#include "CanonicalCommitObserver.h"
#include "CanonicalCommitObserverInternal.h"
#include "SpeedPhysicsSimulation.h"
#include "Engine/World.h"
#include "HAL/PlatformTLS.h"
#include "Misc/ScopeExit.h"
#include <array>

namespace Speed::CanonicalObservers
{
using FRegistry=CanonicalObserver20::TRegistry<FCommittedValues>;
namespace
{
struct FHeldState
{
    FRegistry Registry;std::atomic<uint32> WorkerThread{0};
    FHeldState(uint64 Id,uint64 Generation):Registry(Id,Generation){}
};
struct FHostSlot
{
    std::atomic<const ASpeedPhysicsSimulation*> Host{nullptr};
    std::atomic<FHeldState*> Published{nullptr};
    // GT-owned root. Never replaced/cleared until actual join of this generation.
    TWeakObjectPtr<ASpeedPhysicsSimulation> Incarnation;
    std::shared_ptr<FHeldState> Retained;
};
std::array<FHostSlot,64> Hosts;
std::atomic<uint32> LiveHosts{0};
uint64 NextRegistry=1; // GT-only unique native identity, never from archive bytes.
bool Refuse(FString& F,const TCHAR* Reason){F=Reason;return false;}
FHostSlot* Slot(const ASpeedPhysicsSimulation& H)
{for(auto& S:Hosts)if(S.Host.load(std::memory_order_acquire)==&H)return &S;return nullptr;}
FHeldState* Borrow(const ASpeedPhysicsSimulation& H)
{if(!LiveHosts.load(std::memory_order_acquire))return nullptr;auto* S=Slot(H);return S?S->Published.load(std::memory_order_acquire):nullptr;}
bool OwnerLane(const FHeldState& S)
{return !IsInGameThread()&&S.WorkerThread.load(std::memory_order_acquire)==FPlatformTLS::GetCurrentThreadId();}
void ClearAfterJoin(FHostSlot& S)
{
    S.Published.store(nullptr,std::memory_order_release);
    S.Host.store(nullptr,std::memory_order_release);
    LiveHosts.fetch_sub(1,std::memory_order_release);
    S.Incarnation.Reset();S.Retained.reset(); // GT destruction after true join only.
}
}

// Friendship grants access, never storage/layout fields in the UCLASS. This
// lock also serializes observer registration against an external rollback request.
struct FAccess20
{
    static bool CanRegister(const ASpeedPhysicsSimulation& H)
    {
        FSimulationSnapshot Snapshot;
        return H.CanonicalNumFrame==0&&!H.bCanonicalPublicationTerminal.Load()&&
            !H.PendingRollbackRequest.IsSet()&&
            (!H.ReadLatestSimulationSnapshot(Snapshot)||!Snapshot.PublicationSerial);
    }
    static bool NoWorker(const ASpeedPhysicsSimulation& H){return !H.SimulationWorker;}
    static FCriticalSection& RollbackMutex(ASpeedPhysicsSimulation& H){return H.RollbackRequestMutex;}
};

bool Register(ASpeedPhysicsSimulation& H,const FConfig& Config,std::shared_ptr<IObserver> Observer,
    FHandle& Out,FString& Failure)
{
    Out={};Failure.Empty();
    if(!IsInGameThread())return Refuse(Failure,TEXT("Canonical observer registration requires GT"));
    if(!IsValid(&H)||!H.GetWorld()||H.GetWorld()->bIsTearingDown||!H.IsOwnedWorkerExecutionMode()||
       !Config.OwnerToken||Config.Generation!=H.GetInputWorkerGeneration()||!Config.Generation||!Observer||
       !Config.MaximumCommittedFrames||Config.MaximumCommittedFrames>CanonicalObserver20::MaxBoundedFrames)
        return Refuse(Failure,TEXT("Canonical observer owner/generation/configuration invalid"));
    const bool WasPaused=H.IsOwnedSimulationPaused();
    const auto Q=H.TrySuspendOwnedBoundaryService(1000);
    const bool Lease=Q==ESimulationQuiescence::BoundaryAcknowledged&&H.IsOwnedBoundaryServiceSuspended();
    const bool NoWorker=Q==ESimulationQuiescence::AlreadyStopped&&FAccess20::NoWorker(H);
    if(!Lease&&!NoWorker)return Refuse(Failure,TEXT("Canonical observer requires actual boundary ACK or no worker"));
    ON_SCOPE_EXIT {if(Lease)H.ResumeOwnedBoundaryService();if(!WasPaused)H.ResumeOwnedSimulation();};
    FScopeLock Lock(&FAccess20::RollbackMutex(H));
    if(!FAccess20::CanRegister(H))return Refuse(Failure,TEXT("Canonical observer registration is after frame0 or rollback admission"));
    auto* S=Slot(H);
    if(S)
    {
        if(S->Incarnation.Get()!=&H||!S->Retained||S->Retained->Registry.InputGeneration()!=Config.Generation)
            return Refuse(Failure,TEXT("Canonical observer cannot supersede retained generation"));
        if(!S->Retained->Registry.Add(Config,std::move(Observer),Out))
            return Refuse(Failure,TEXT("Canonical observer slot set sealed, duplicate owner or capacity reached"));
        return true;
    }
    if(!NextRegistry||NextRegistry==MAX_uint64)return Refuse(Failure,TEXT("Canonical observer identity exhausted"));
    for(auto& Candidate:Hosts)if(!Candidate.Host.load())
    {
        std::shared_ptr<FHeldState> Prepared;
        try{Prepared=std::make_shared<FHeldState>(NextRegistry++,Config.Generation);}
        catch(...){return Refuse(Failure,TEXT("Canonical observer bounded state allocation failed"));}
        if(!Prepared->Registry.Add(Config,std::move(Observer),Out))return Refuse(Failure,TEXT("Canonical observer admission refused"));
        Candidate.Incarnation=&H;Candidate.Retained=std::move(Prepared);
        Candidate.Published.store(Candidate.Retained.get(),std::memory_order_release);
        Candidate.Host.store(&H,std::memory_order_release);LiveHosts.fetch_add(1,std::memory_order_release);return true;
    }
    return Refuse(Failure,TEXT("Canonical observer fixed host capacity reached"));
}
bool ReadStatus(ASpeedPhysicsSimulation& H,const FHandle& Handle,FStatus& Out,FString& Failure)
{
    Out={};Failure.Empty();if(!IsInGameThread())return Refuse(Failure,TEXT("Canonical observer status requires GT"));
    auto* S=Slot(H);
    if(!S||S->Incarnation.Get()!=&H||!S->Retained||!S->Retained->Registry.Contains(Handle))
        return Refuse(Failure,TEXT("Canonical observer stale handle"));
    Out=S->Retained->Registry.Status();return true;
}
bool RetireAfterJoin(ASpeedPhysicsSimulation& H,const FHandle& Handle,FString& Failure)
{
    Failure.Empty();if(!IsInGameThread())return Refuse(Failure,TEXT("Canonical observer retirement requires GT"));
    auto* S=Slot(H);
    if(!S||S->Incarnation.Get()!=&H||!S->Retained||!S->Retained->Registry.Contains(Handle)||
       !H.InputOwnersRetiredAfterJoin(Handle.Generation))
        return Refuse(Failure,TEXT("Canonical observer retirement requires matching actual join/input owners"));
    auto& R=S->Retained->Registry;
    if(!R.Retire(Handle,{R.Id(),Handle.Generation,true,true}))return Refuse(Failure,TEXT("Canonical observer active callback or stale removal"));
    if(R.Status().Phase==CanonicalObserver20::EPhase::Retired)ClearAfterJoin(*S);
    return true;
}
bool HasRegistration(const ASpeedPhysicsSimulation& H){return Borrow(H)!=nullptr;}
bool EnterActualWorkerPulse(ASpeedPhysicsSimulation& H)
{
    auto* S=Borrow(H);if(!S)return true;
    if(IsInGameThread()||S->Registry.InputGeneration()!=H.GetInputWorkerGeneration())
    {S->Registry.DriverFailed();return false;}
    const auto Current=FPlatformTLS::GetCurrentThreadId();uint32 Empty=0;
    S->WorkerThread.compare_exchange_strong(Empty,Current,std::memory_order_acq_rel);
    if(S->WorkerThread.load()!=Current){S->Registry.DriverFailed();return false;}
    return true;
}
EReadiness DriverDecision(ASpeedPhysicsSimulation& H)
{
    auto* S=Borrow(H);return S?S->Registry.Decision(OwnerLane(*S),H.GetInputWorkerGeneration()):EReadiness::None;
}
EReadiness TerminalDecision(const ASpeedPhysicsSimulation& H)
{
    auto* S=Borrow(H);if(!S)return EReadiness::None;
    const auto P=S->Registry.Status().Phase;
    return P==CanonicalObserver20::EPhase::Complete?EReadiness::Complete:
        P==CanonicalObserver20::EPhase::Failed||P==CanonicalObserver20::EPhase::Retired?EReadiness::Failed:EReadiness::Ready;
}
bool AllowsPhysicalStep(ASpeedPhysicsSimulation& H,uint64 Frame)
{auto* S=Borrow(H);return !S||S->Registry.AllowsStep(OwnerLane(*S),H.GetInputWorkerGeneration(),Frame);}
EReadiness PostCommitted(ASpeedPhysicsSimulation& H,const FSimulationSnapshot& Snapshot,uint64 Serial)
{
    auto* S=Borrow(H);if(!S)return EReadiness::None;
    const FCommitReceipt R{H.GetInputWorkerGeneration(),Snapshot.NumFrame,Serial,Snapshot.StateHash,Snapshot.InputJournalHash,true,true,true};
    const FCommittedValues Values{Snapshot.Payload,Snapshot.PresentationBodies,Snapshot.PresentationOutputs};
    return S->Registry.Observe(OwnerLane(*S),R,Values);
}
bool HasCommittedAt(const ASpeedPhysicsSimulation& H,uint64 Frame)
{auto* S=Borrow(H);return S&&S->Registry.HasCommittedAt(Frame);}
void DriverFailed(ASpeedPhysicsSimulation& H){if(auto* S=Borrow(H))S->Registry.DriverFailed();}
bool RetireAllAfterActualJoin(ASpeedPhysicsSimulation& H)
{
    if(!IsInGameThread())return false;auto* S=Slot(H);if(!S)return true;
    if(S->Incarnation.Get()!=&H||!S->Retained)return false;
    auto& R=S->Retained->Registry;
    if(!H.InputOwnersRetiredAfterJoin(R.InputGeneration())||!R.RetireAll({R.Id(),R.InputGeneration(),true,true}))return false;
    ClearAfterJoin(*S);return true;
}
}
