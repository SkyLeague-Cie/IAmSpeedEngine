#include "SpeedPhysicsSimulation.h"
#include "IAmSpeed/World/Collision/NativePreparationPrefix.h"
#include "NativeCanonicalFrameStream.h"
#include "ReplayPhysicalFrameScope.h"
#include "ReplayLifecycleBoundary.h"
#include "Engine/World.h"
#include <array>
// Fill out your copyright notice in the Description page of Project Settings.


#include "SpeedPhysicsSimulation.h"
#include "IAmSpeed/Controllers/PrivateInputFailureFile.h"
#if !UE_BUILD_SHIPPING
#include "NativeFrameWitnessRecorder.h"
#include "IAmSpeed/World/Analytic/AnalyticCausalStats1075.h"
#include "IAmSpeed/World/Analytic/JoinedPiecewiseProviderWave1075.h"
#include "IAmSpeed/World/Analytic/JoinedTensorCellWave1213.h"
#include "IAmSpeed/World/Analytic/JoinedAnalyticQueryPrefetch1075.h"
#include "FrameAppliedCommandWitness.h"
#include "IAmSpeed/Base/EpisodeStateBytesForTesting.h"
#endif
#include "IAmSpeed/World/Simulation/FrameParallelSettings.h"
#include "IAmSpeed/World/Collision/StaticCollisionWorld.h"
#include "IAmSpeed/World/Collision/PreparedNativeFrameConsumer.h"
#include "IAmSpeed/World/Simulation/FramePresentationTasks.h"
#include "CanonicalFrameContext.h"
#include "IAmSpeed/World/Collision/PreparedPhysicalExecution.h"
#include "CanonicalEpisodeTestingScope.h"
#include "SimulationActorDiagnostics.h"
#include "OwnedFrameAudit.h"
#include "RealTimeSimulation.h"
#include "FastSimulation.h"
#include "CanonicalFrameDriver.h"
#include "OnePausedCanonicalWork.h"
#include "IAmSpeed/World/Analytic/StaticWorldQueryAudit.h"
#include "IAmSpeed/Base/SUtils.h"
#include "IAmSpeed/Components/ISpeedComponent.h"
#include "Net/UnrealNetwork.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "PhysicsEngine/PhysicsSettings.h"
#include "HAL/PlatformTime.h"
#include "HAL/IConsoleManager.h"
#include "Misc/ScopeExit.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/FileManager.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"
#include <exception>
#include <stdexcept>

namespace Speed::ReplayLifecycle
{
namespace
{
struct FFence
{
    std::atomic<const ASpeedPhysicsSimulation*> Host{nullptr};
    std::atomic<bool> ResumeRequested{false};
};
std::array<FFence,64> Fences;
struct FHold
{
    Token Id=0;TWeakObjectPtr<UWorld> World;uint64 Frame=0,Serial=0,Generation=0;
    uint32 Slot=0;bool InitiallyPaused=false,ServiceLease=false,Ready=false,RenewedWithoutFrame=false;
};
TMap<TWeakObjectPtr<ASpeedPhysicsSimulation>,FHold> Holds;
Token Next=1;
TWeakObjectPtr<ASpeedPhysicsSimulation> Key(ASpeedPhysicsSimulation& H){return TWeakObjectPtr<ASpeedPhysicsSimulation>(&H);}
bool Refuse(FString& F,const TCHAR* Why){F=Why;return false;}
FHold* Find(ASpeedPhysicsSimulation& H,Token T)
{auto* V=Holds.Find(Key(H));return V&&T&&V->Id==T?V:nullptr;}
bool Identity(ASpeedPhysicsSimulation& H,const FHold& V,FString& F)
{
    if(!V.Ready||!IsValid(&H)||!V.World.IsValid()||H.GetWorld()!=V.World.Get()||V.World->bIsTearingDown||
       !H.IsOwnedWorkerExecutionMode()||H.GetInputWorkerGeneration()!=V.Generation||
       !H.IsOwnedSimulationPaused()||Fences[V.Slot].Host.load()!=&H)
        return Refuse(F,TEXT("Replay lifecycle native owner/world/input generation/fence changed"));
    FSimulationSnapshot S;
    if(!H.ReadLatestSimulationSnapshot(S)||S.NumFrame!=V.Frame||S.PublicationSerial!=V.Serial)
        return Refuse(F,TEXT("Replay lifecycle physical publication advanced or unavailable under hold"));
    return true;
}
}
// Called only on GT by the existing resume path after its original guards.
// Public resume rejects non-GT calls before it can observe this ownership.
bool DeferResume(const ASpeedPhysicsSimulation* H)
{
    for(auto& V:Fences)if(V.Host.load()==H){V.ResumeRequested.store(true);return true;}
    return false;
}
void RetireHost(ASpeedPhysicsSimulation& H)
{
    check(IsInGameThread());auto* V=Holds.Find(Key(H));if(!V)return;
    Fences[V->Slot].Host.store(nullptr);Fences[V->Slot].ResumeRequested.store(false);Holds.Remove(Key(H));
}
Token Acquire(ASpeedPhysicsSimulation& H,FString& F)
{
    if(!IsInGameThread()){Refuse(F,TEXT("Replay lifecycle requires game thread"));return 0;}F.Empty();
    if(!IsValid(&H)||!H.HasActorBegunPlay()||!H.GetWorld()||H.GetWorld()->bIsTearingDown||
       !H.IsOwnedWorkerExecutionMode()||H.IsOwnedBoundaryServiceSuspended()||Holds.Contains(Key(H))||!Next||Next==MAX_uint64)
    {Refuse(F,TEXT("Replay lifecycle acquisition owner/boundary unavailable"));return 0;}
    uint32 Slot=uint32(Fences.size());
    for(uint32 I=0;I<uint32(Fences.size());++I)if(!Fences[I].Host.load()){Slot=I;break;}
    if(Slot==uint32(Fences.size())){Refuse(F,TEXT("Replay lifecycle finite fence capacity"));return 0;}
    FHold V;V.Id=Next++;V.Slot=Slot;V.World=H.GetWorld();V.InitiallyPaused=H.IsOwnedSimulationPaused();
    Holds.Add(Key(H),V); // allocation completes before publishing the fence
    Fences[Slot].ResumeRequested.store(false);Fences[Slot].Host.store(&H);
    const auto Q=H.TrySuspendOwnedBoundaryService(1000);auto* Current=Find(H,V.Id);
    const bool Ack=Q==ESimulationQuiescence::BoundaryAcknowledged&&H.IsOwnedBoundaryServiceSuspended();
    const bool Joined=Q==ESimulationQuiescence::AlreadyStopped&&H.InputOwnersRetiredAfterJoin(H.GetInputWorkerGeneration());
    FSimulationSnapshot S;
    if(!Current||(!Ack&&!Joined)||!H.ReadLatestSimulationSnapshot(S)||!S.PublicationSerial)
    {
        const bool Requested=Fences[Slot].ResumeRequested.load();
        if(Ack)H.ResumeOwnedBoundaryService();RetireHost(H);
        auto* W=H.GetWorld();
        if(Ack&&IsValid(&H)&&W&&V.World.IsValid()&&W==V.World.Get()&&!W->bIsTearingDown&&
           (!V.InitiallyPaused||Requested))H.ResumeOwnedSimulation();
        Refuse(F,TEXT("Replay lifecycle actual boundary ACK or joined input owner/publication missing"));return 0;
    }
    Current->Frame=S.NumFrame;Current->Serial=S.PublicationSerial;Current->Generation=H.GetInputWorkerGeneration();
    Current->ServiceLease=Ack;Current->Ready=true;
    if(!Identity(H,*Current,F)){FString Ignore;Release(H,V.Id,Ignore);return 0;}
    return V.Id;
}
bool HasActiveHold(ASpeedPhysicsSimulation& H)
{return IsInGameThread()&&Holds.Contains(Key(H));}
bool ReadActiveHoldResumeIntent(ASpeedPhysicsSimulation& H,bool& Intent)
{
    Intent=false;if(!IsInGameThread())return false;
    const auto* hold=Holds.Find(Key(H));if(!hold||!hold->Ready)return false;
    Intent=!hold->InitiallyPaused||Fences[hold->Slot].ResumeRequested.load();return true;
}
bool Validate(ASpeedPhysicsSimulation& H,Token T,FString& F)
{if(!IsInGameThread())return Refuse(F,TEXT("Replay lifecycle requires game thread"));F.Empty();auto* V=Find(H,T);return V?Identity(H,*V,F):Refuse(F,TEXT("Replay lifecycle token is not current owner"));}
bool SuspendService(ASpeedPhysicsSimulation& H,Token T,FString& F)
{
    if(!IsInGameThread())return Refuse(F,TEXT("Replay lifecycle requires game thread"));if(!Validate(H,T,F))return false;auto* V=Find(H,T);
    if(V->ServiceLease)return H.IsOwnedBoundaryServiceSuspended()||Refuse(F,TEXT("Replay lifecycle owned service ACK lost"));
    const auto Q=H.TrySuspendOwnedBoundaryService(1000);V=Find(H,T);
    if(!V)return Refuse(F,TEXT("Replay lifecycle owner changed during suspension"));
    if(Q==ESimulationQuiescence::BoundaryAcknowledged&&H.IsOwnedBoundaryServiceSuspended())V->ServiceLease=true;
    else if(Q!=ESimulationQuiescence::AlreadyStopped||
            (!H.InputOwnersRetiredAfterJoin(V->Generation)&&!V->RenewedWithoutFrame))
        return Refuse(F,TEXT("Replay lifecycle service needs ACK or proved joined/new paused generation"));
    return Identity(H,*V,F);
}
bool ResumeService(ASpeedPhysicsSimulation& H,Token T,FString& F)
{
    if(!IsInGameThread())return Refuse(F,TEXT("Replay lifecycle requires game thread"));if(!Validate(H,T,F))return false;auto* V=Find(H,T);
    if(V->ServiceLease){V->ServiceLease=false;H.ResumeOwnedBoundaryService();}
    V=Find(H,T);if(!V)return Refuse(F,TEXT("Replay lifecycle owner changed while releasing service"));
    return Identity(H,*V,F); // physical pause/fence remains active
}
bool RenewJoinedInputs(ASpeedPhysicsSimulation& H,Token T,FString& F)
{
    if(!IsInGameThread())return Refuse(F,TEXT("Replay lifecycle requires game thread"));if(!Validate(H,T,F))return false;auto* V=Find(H,T);
    if(V->ServiceLease||!H.InputOwnersRetiredAfterJoin(V->Generation))
        return Refuse(F,TEXT("Replay lifecycle input renewal requires the actual old generation join with service released"));
    const uint64 Previous=V->Generation;
    if(!H.PrepareControlledInputRun()||H.GetInputWorkerGeneration()==Previous)
        return Refuse(F,TEXT("Replay lifecycle controlled input generation renewal failed"));
    V=Find(H,T);if(!V)return Refuse(F,TEXT("Replay lifecycle owner changed during input renewal"));
    V->Generation=H.GetInputWorkerGeneration();V->RenewedWithoutFrame=true;
    return Identity(H,*V,F);
}
bool Release(ASpeedPhysicsSimulation& H,Token T,FString& F)
{
    if(!IsInGameThread())return Refuse(F,TEXT("Replay lifecycle requires game thread"));F.Empty();auto* V=Find(H,T);
    if(!V)return Refuse(F,TEXT("Replay lifecycle release is not the matching live owner"));
    const bool Verified=Identity(H,*V,F);const auto Copy=*V;
    const bool Requested=Fences[Copy.Slot].ResumeRequested.load();
    if(Copy.ServiceLease)H.ResumeOwnedBoundaryService();
    RetireHost(H);
    // Restoring the game's original resume intent is not capture admission.
    // Original native registry/teardown guards still decide whether it resumes.
    auto* W=H.GetWorld();
    if(Verified&&IsValid(&H)&&W&&Copy.World.IsValid()&&W==Copy.World.Get()&&!W->bIsTearingDown&&
       (!Copy.InitiallyPaused||Requested))H.ResumeOwnedSimulation();
    return Verified;
}
}


namespace
{
    // GT-only runtime ownership, never an administrative lease or file token.
    // Weak actor keys plus globally unique values prevent stale-incarnation
    // release; no per-actor fields/ABI change and no lookup from physics lane.
    TMap<TWeakObjectPtr<ASpeedPhysicsSimulation>,uint64> ReplayStartupHolds;
    uint64 NextReplayStartupToken=1;
    TMap<TWeakObjectPtr<ASpeedPhysicsSimulation>,uint64> NativeFrameStartupHolds;
    uint64 NextNativeFrameStartupToken=1;
    TWeakObjectPtr<ASpeedPhysicsSimulation> ReplayStartupKeyFor(const ASpeedPhysicsSimulation* Host)
    {return TWeakObjectPtr<ASpeedPhysicsSimulation>(const_cast<ASpeedPhysicsSimulation*>(Host));}
    bool HasReplayStartupHoldOnGameThread(const ASpeedPhysicsSimulation* Host)
    {check(IsInGameThread());const auto key=ReplayStartupKeyFor(Host);
        return ReplayStartupHolds.Contains(key)||NativeFrameStartupHolds.Contains(key);}
	std::atomic<uint64> NextInputWorkerGeneration{1};

	TAutoConsoleVariable<int32> CVarSimulationPerformanceAudit(
		TEXT("p.IAmSpeed.Simulation.PerformanceAudit"), 0,
		TEXT("Logs the wall-time and solver phase breakdown of every real-time IAmSpeed physical frame."));
	TAutoConsoleVariable<int32> CVarSimulationSlowFrameWarnings(
		TEXT("p.IAmSpeed.Simulation.SlowFrameWarnings"), 1,
		TEXT("Reports real-time IAmSpeed physical frames that exceed the configured wall-time budget."));
	TAutoConsoleVariable<float> CVarSimulationWarningFrameFraction(
		TEXT("p.IAmSpeed.Simulation.WarningFrameFraction"), 0.04f,
		TEXT("Warns when one physical step exceeds this fraction of its simulated delta time."));
}

#include "SpeedPhysicsSimulationPrivateInputStartup.inl"

unsigned int ASpeedPhysicsSimulation::EngineFPS = 120; // Default value, can be overridden by config or in-game

// Sets default values

#if WITH_DEV_AUTOMATION_TESTS
namespace Speed::Testing
{
namespace
{
thread_local FPresentationDispatchObserver PresentationDispatchObserver = nullptr;
thread_local void* PresentationDispatchContext = nullptr;
}
bool SetPresentationDispatchObserverForCurrentThread(FPresentationDispatchObserver Observer, void* Context) noexcept
{
    if (!Observer || !Context || PresentationDispatchObserver) return false;
    PresentationDispatchObserver = Observer; PresentationDispatchContext = Context; return true;
}
bool ClearPresentationDispatchObserverForCurrentThread(FPresentationDispatchObserver Observer, void* Context) noexcept
{
    if (PresentationDispatchObserver != Observer || PresentationDispatchContext != Context) return false;
    PresentationDispatchObserver = nullptr; PresentationDispatchContext = nullptr; return true;
}
bool HasPresentationDispatchObserverForCurrentThread(FPresentationDispatchObserver Observer, const void* Context) noexcept
{
    return PresentationDispatchObserver == Observer && PresentationDispatchContext == Context;
}
static void ObservePresentationDispatch(const ISimulationPresentationProducer* Producer,
    uint64 Frame) noexcept
{
    if (PresentationDispatchObserver)
        PresentationDispatchObserver(PresentationDispatchContext, EPresentationAuditEvent::ProducerDispatch, Producer, Producer->OwnerStableId(),
            Producer->Channel(), Producer->IsCameraPresentation(), Frame);
}
static void ObserveCanonicalCameraSample(uint64 Frame) noexcept
{
    if (PresentationDispatchObserver)
        PresentationDispatchObserver(PresentationDispatchContext, EPresentationAuditEvent::CanonicalCameraSample,
            nullptr, 0, 0, true, Frame);
}
}
#endif
ASpeedPhysicsSimulation::ASpeedPhysicsSimulation()
{
    bReplicates = true;
    bAlwaysRelevant = true;
    SetNetUpdateFrequency(100.f);
    SetReplicateMovement(false);
	bAsyncPhysicsTickEnabled = true;
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bStartWithTickEnabled = true;
	PrimaryActorTick.bTickEvenWhenPaused = true;
	PrimaryActorTick.TickGroup = TG_PrePhysics;
	EngineFPS = Speed::SimUtils::ComputePhysicsFPS(UPhysicsSettings::Get()->AsyncFixedTimeStepSize);
	RealDeltaTime = 1.0f / static_cast<float>(EngineFPS);
	InputWorkerGeneration = NextInputWorkerGeneration.fetch_add(1);
	check(InputWorkerGeneration && InputWorkerGeneration != MAX_uint64);
	InputSessionCommands = std::make_shared<Speed::Input::V2::FInputSessionCommands>();
}

void ASpeedPhysicsSimulation::BeginPlay()
{
	Super::BeginPlay();
	// Fixed before the worker starts; diagnostics never change the canonical payload.
	bPublishPresentation = FParse::Param(FCommandLine::Get(), TEXT("SpeedPublishPresentation"));
	bPublishCompletedWorldInput = FParse::Param(FCommandLine::Get(), TEXT("SpeedPublishCompletedWorldInput"));
	if (bPublishCompletedWorldInput)
		CompletedPublicationSource.reset(new Speed::FCompletedSimulationSource(InputWorkerGeneration));
	RefreshExecutionMode();
}

bool ASpeedPhysicsSimulation::ReadPresentationPose(const uint64 StableId, FSimulationPoseConsumption& Out)
{
	check(IsInGameThread());
	if (!bPublishPresentation) { Out = FSimulationPoseConsumption(); return false; }
	return PresentationLatch.ReadBody(GFrameCounter, SnapshotBuffer, StableId, Out);
}

bool ASpeedPhysicsSimulation::ReadCanonicalCameraSample(const uint64 NumFrame,
	FCameraCanonicalSample& Out) const
{
	check(IsInGameThread());
	return CameraSampleBuffer.ReadFrame(NumFrame, Out);
}

bool ASpeedPhysicsSimulation::ReadPresentationOutput(const uint64 StableId, const uint32 Channel,
	FSimulationPresentationOutput& Out)
{
	check(IsInGameThread());
	if (!bPublishPresentation) { Out = FSimulationPresentationOutput(); return false; }
	return PresentationLatch.ReadOutput(GFrameCounter, SnapshotBuffer, StableId, Channel, Out);
}

bool ASpeedPhysicsSimulation::RegisterPresentationProducer(
	TSharedRef<ISimulationPresentationProducer, ESPMode::ThreadSafe> Producer)
{
	check(IsInGameThread());
	FScopeLock Lock(&PresentationProducerMutex);
	if (bPresentationBindingClosed || !Producer->OwnerStableId() || !Producer->Channel() || PresentationProducers.Num() >= 64 ||
		(Producer->IsCameraPresentation() && !AllowsCameraPresentation())) return false;
	for (const auto& Existing : PresentationProducers)
		if (Existing->OwnerStableId() == Producer->OwnerStableId() && Existing->Channel() == Producer->Channel()) return false;
	PresentationProducers.Add(Producer);
	return true;
}

void ASpeedPhysicsSimulation::UnregisterPresentationProducer(
	const TSharedRef<ISimulationPresentationProducer, ESPMode::ThreadSafe>& Producer)
{
	check(IsInGameThread());
	FScopeLock Lock(&PresentationProducerMutex);
	PresentationProducers.Remove(Producer);
}

bool ASpeedPhysicsSimulation::UnbindPresentationAtFrameBoundary(
	const TSharedRef<ISimulationPresentationProducer, ESPMode::ThreadSafe>& Producer,
	TFunctionRef<void()> ReleaseAtBoundary)
{
	check(IsInGameThread());
	if (GetActiveExecutionMode() == ESimulationExecutionMode::UnrealAsyncCallback) return false;
	const bool bWasPaused = IsOwnedSimulationPaused();
	const bool bOwnedLane = GetActiveExecutionMode() == ESimulationExecutionMode::IAmSpeedThread;
	const ESimulationQuiescence Boundary = bOwnedLane ? TrySuspendOwnedBoundaryService() : TryPauseOwnedSimulation();
	if (Boundary != ESimulationQuiescence::BoundaryAcknowledged && Boundary != ESimulationQuiescence::AlreadyStopped)
	{
		UE_LOG(LogTemp, Warning, TEXT("[PresentationUnbindingPauseUnacknowledged] Removal rejected; simulation remains pause-requested."));
		return false;
	}
	const bool bSuspensionLease = bOwnedLane && Boundary == ESimulationQuiescence::BoundaryAcknowledged;
	ON_SCOPE_EXIT { if (bSuspensionLease) ResumeOwnedBoundaryService(); if (!bWasPaused && !(LiveBoundaryControl && LiveBoundaryControl->ScopeActive)) ResumeOwnedSimulation(); };
	FScopeLock Lock(&PresentationProducerMutex);
	const int32 Index = PresentationProducers.IndexOfByKey(Producer);
	ReleaseAtBoundary();
	if (Index != INDEX_NONE) PresentationProducers.RemoveAt(Index);
	return true;
}

TSharedPtr<ISimulationPresentationProducer, ESPMode::ThreadSafe> ASpeedPhysicsSimulation::BindPresentationAtFrameBoundary(
	ISpeedComponent& OwnerComponent, ISpeedComponent& TargetComponent,
	TFunctionRef<TSharedPtr<ISimulationPresentationProducer, ESPMode::ThreadSafe>(
		const FSimulationPresentationBinding&)> Factory,
	TSharedPtr<ISimulationPresentationProducer, ESPMode::ThreadSafe> Previous,
	ISimulationPresentationAdapterCommit* AdapterCommit)
{
	return BindPresentationAtFrameBoundary(OwnerComponent, &TargetComponent, Factory, Previous, AdapterCommit);
}

TSharedPtr<ISimulationPresentationProducer, ESPMode::ThreadSafe> ASpeedPhysicsSimulation::BindPresentationAtFrameBoundary(
	ISpeedComponent& OwnerComponent, ISpeedComponent* TargetComponent,
	TFunctionRef<TSharedPtr<ISimulationPresentationProducer, ESPMode::ThreadSafe>(
		const FSimulationPresentationBinding&)> Factory,
	TSharedPtr<ISimulationPresentationProducer, ESPMode::ThreadSafe> Previous,
	ISimulationPresentationAdapterCommit* AdapterCommit)
{
	check(IsInGameThread());
    int32 ProbeBoundary=-1; uint64 ProbeOwner=0,ProbeTarget=0;
    const auto Refuse=[&](int32 Line)->TSharedPtr<ISimulationPresentationProducer,ESPMode::ThreadSafe>
    {
        if(FParse::Param(FCommandLine::Get(),TEXT("ReplayDeliveryProbe")))
        {
            const FString File=FPaths::ProjectSavedDir()/TEXT("ReplayBoundaryDiagnostics.txt");
            const FString Row=FString::Printf(TEXT("Refuse line=%d closed=%d mode=%d terminal=%d subsystem=%d collision=%d startup=%d worker=%d paused=%d frame=%llu boundary=%d owner=%llu target=%llu\n"),Line,bPresentationBindingClosed?1:0,(int32)GetActiveExecutionMode(),bOwnedWorkerTerminal.Load()?1:0,SpeedWorldSubsystem?1:0,(SpeedWorldSubsystem&&SpeedWorldSubsystem->GetStaticCollisionWorld())?1:0,HasReplayStartupHoldOnGameThread(this)?1:0,SimulationWorker?1:0,IsOwnedSimulationPaused()?1:0,(unsigned long long)CanonicalNumFrame,ProbeBoundary,(unsigned long long)ProbeOwner,(unsigned long long)ProbeTarget);
            if(IFileManager::Get().FileSize(*File)<65536)FFileHelper::SaveStringToFile(Row,*File,FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM,&IFileManager::Get(),FILEWRITE_Append);
        }
        return nullptr;
    };
	if (bPresentationBindingClosed || GetActiveExecutionMode() == ESimulationExecutionMode::UnrealAsyncCallback ||
		bOwnedWorkerTerminal.Load() || !EnsureSimulationWorldReady()) return Refuse(__LINE__);
	const bool bWasPaused = IsOwnedSimulationPaused();
	const bool bOwnedLane = GetActiveExecutionMode() == ESimulationExecutionMode::IAmSpeedThread;
	const ESimulationQuiescence Boundary = bOwnedLane ? TrySuspendOwnedBoundaryService() : TryPauseOwnedSimulation();
    ProbeBoundary=(int32)Boundary;
	if (Boundary != ESimulationQuiescence::BoundaryAcknowledged && Boundary != ESimulationQuiescence::AlreadyStopped)
	{
		UE_LOG(LogTemp, Warning, TEXT("[PresentationBindingPauseUnacknowledged] Registration rejected; simulation remains pause-requested."));
		return Refuse(__LINE__);
	}
	const bool bSuspensionLease = bOwnedLane && Boundary == ESimulationQuiescence::BoundaryAcknowledged;
	ON_SCOPE_EXIT { if (bSuspensionLease) ResumeOwnedBoundaryService(); if (!bWasPaused && !(LiveBoundaryControl && LiveBoundaryControl->ScopeActive)) ResumeOwnedSimulation(); };
	InitializeCanonicalFrame(0);
	FSimulationPresentationBinding Binding;
	Binding.OwnerStableId = SpeedWorldSubsystem->GetSimulationStableId(OwnerComponent);
	Binding.TargetStableId = TargetComponent ? SpeedWorldSubsystem->GetSimulationStableId(*TargetComponent) : 0;
	Binding.FirstFrame = CanonicalNumFrame;
    ProbeOwner=Binding.OwnerStableId;ProbeTarget=Binding.TargetStableId;
	if (!Binding.OwnerStableId || (TargetComponent && !Binding.TargetStableId) || Binding.OwnerStableId == Binding.TargetStableId)
		return Refuse(__LINE__);
	auto Producer = Factory(Binding);
	if (!Producer || Producer->OwnerStableId() != Binding.OwnerStableId || !Producer->Channel() ||
		(Producer->IsCameraPresentation() && !AllowsCameraPresentation()))
		return Refuse(__LINE__);
	if (AdapterCommit && !AdapterCommit->Prepare(Producer)) return Refuse(__LINE__);
	if (Previous)
	{
		FScopeLock Lock(&PresentationProducerMutex);
		const int32 Index = PresentationProducers.IndexOfByKey(Previous.ToSharedRef());
		if (Index == INDEX_NONE || Previous->OwnerStableId() != Producer->OwnerStableId() ||
			Previous->Channel() != Producer->Channel()) return Refuse(__LINE__);
		// The acknowledged boundary makes replacement atomic with respect to Produce.
		PresentationProducers[Index] = Producer.ToSharedRef();
	}
	else if (!RegisterPresentationProducer(Producer.ToSharedRef())) return Refuse(__LINE__);
	// Successful binding opts into its body/output sidecars. The owned worker
	// is pause-acknowledged above and resumes only after this boundary returns.
	// Failed factories/registration must leave publication policy unchanged.
	if (AdapterCommit) AdapterCommit->Finalize(Producer);
	bPublishPresentation = true;
	return Producer;
}

bool ASpeedPhysicsSimulation::ReadCanonicalCameraSamples(const uint64 FirstFrame,
	const uint64 LastFrame, TArray<FCameraCanonicalSample>& Out) const
{
	check(IsInGameThread());
	return CameraSampleBuffer.ReadRange(FirstFrame, LastFrame, Out);
}

uint64 ASpeedPhysicsSimulation::TryAcquireNativeFrameStartupHoldBeforeBeginPlay()
{
    check(IsInGameThread());
    if(!::IsValid(this)||IsTemplate()||HasActorBegunPlay()||SimulationWorker||CanonicalNumFrame!=0||
        bOwnedWorkerTerminal.Load()||bInputOwnerRetired.Load()||!Speed::CanonicalFrameDriver::IsEnabled()||
        Speed::CanonicalFrameDriver::GetExecutionMode()!=ESimulationExecutionMode::IAmSpeedThread)return 0;
    for(auto it=NativeFrameStartupHolds.CreateIterator();it;++it)if(!it.Key().IsValid())it.RemoveCurrent();
    const auto key=ReplayStartupKeyFor(this);
    if(NativeFrameStartupHolds.Num()>=64||NativeFrameStartupHolds.Contains(key)||
        !NextNativeFrameStartupToken||NextNativeFrameStartupToken==MAX_uint64)return 0;
    const uint64 token=NextNativeFrameStartupToken++;NativeFrameStartupHolds.Add(key,token);return token;
}
bool ASpeedPhysicsSimulation::ReleaseNativeFrameStartupHold(uint64 token)
{
    check(IsInGameThread());const auto key=ReplayStartupKeyFor(this);const auto* held=NativeFrameStartupHolds.Find(key);
    if(!token||!held||*held!=token)return false;NativeFrameStartupHolds.Remove(key);return true;
}
bool ASpeedPhysicsSimulation::IsNativeFrameStartupHeld(uint64 token) const
{check(IsInGameThread());const auto* held=NativeFrameStartupHolds.Find(ReplayStartupKeyFor(this));return token&&held&&*held==token;}
uint64 ASpeedPhysicsSimulation::TryAcquireReplayStartupHoldBeforeBeginPlay()
{
    check(IsInGameThread());
    if(!::IsValid(this)||IsTemplate()||HasActorBegunPlay()||SimulationWorker||CanonicalNumFrame!=0||
        bOwnedWorkerTerminal.Load()||bInputOwnerRetired.Load()||!Speed::CanonicalFrameDriver::IsEnabled()||
        Speed::CanonicalFrameDriver::GetExecutionMode()!=ESimulationExecutionMode::IAmSpeedThread)return 0;
    for(auto It=ReplayStartupHolds.CreateIterator();It;++It)if(!It.Key().IsValid())It.RemoveCurrent();
    const auto Key=ReplayStartupKeyFor(this);
    if(ReplayStartupHolds.Num()>=64||ReplayStartupHolds.Contains(Key)||!NextReplayStartupToken||NextReplayStartupToken==MAX_uint64)return 0;
    const uint64 Token=NextReplayStartupToken++;ReplayStartupHolds.Add(Key,Token);return Token;
}
bool ASpeedPhysicsSimulation::ReleaseReplayStartupHold(uint64 Token)
{
    check(IsInGameThread());const auto Key=ReplayStartupKeyFor(this);const auto* Held=ReplayStartupHolds.Find(Key);
    if(!Token||!Held||*Held!=Token)return false;
    ReplayStartupHolds.Remove(Key);return true; // User pause is deliberately untouched.
}
bool ASpeedPhysicsSimulation::IsReplayStartupHeld(uint64 Token)const
{
    check(IsInGameThread());const auto* Held=ReplayStartupHolds.Find(ReplayStartupKeyFor(this));
    return Token&&Held&&*Held==Token;
}

void ASpeedPhysicsSimulation::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    if(!ReplayStartupHolds.IsEmpty())ReplayStartupHolds.Remove(ReplayStartupKeyFor(this));
    NativeFrameStartupHolds.Remove(ReplayStartupKeyFor(this));
	bPresentationBindingClosed = true;
#if WITH_DEV_AUTOMATION_TESTS
	Speed::OwnedFrameAudit::TryExportForHostOnGameThread(this);
#endif
	StopOwnedWorker();
#if !UE_BUILD_SHIPPING
    // Passive, cumulative closed-frame counters. The owned worker is joined;
    // this does not request a further pulse or change run completion state.
    if (IsOwnedWorkerExecutionMode() && PerformanceFrameCount &&
        FParse::Param(FCommandLine::Get(), TEXT("SLStadiumNativeFastBatch")))
        UE_LOG(LogTemp, Display, TEXT("[JoinedPhysicalFramePerformance] Boundary=EndPlay Host=%u Cumulative=1 Frames=%llu TotalStepMs=%.9f AverageStepMs=%.9f MaximumStepMs=%.9f WarningFrames=%llu IterationLimits=%llu"),
            GetUniqueID(), PerformanceFrameCount, PerformanceTotalStepMilliseconds,
            PerformanceTotalStepMilliseconds / double(PerformanceFrameCount), PerformanceMaximumStepMilliseconds,
            PerformanceWarningFrameCount, PerformanceIterationLimitCount);
#endif

    Speed::ReplayLifecycle::RetireHost(*this);
	{
		FScopeLock Lock(&PresentationProducerMutex);
		PresentationProducers.Reset();
	}
	if (GetActiveExecutionMode() != ESimulationExecutionMode::UnrealAsyncCallback) PresentationRetiredAfterJoin->Store(true);
	bAsyncPhysicsTickEnabled = false;
	Super::EndPlay(EndPlayReason);
}

void ASpeedPhysicsSimulation::Tick(const float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
#if WITH_DEV_AUTOMATION_TESTS
	Speed::OwnedFrameAudit::TryExportForHostOnGameThread(this);
#endif
	RefreshExecutionMode();
    ObservePrivateInputStartup(EPrivateInputStartupObservation::Tick);
    if(HasReplayStartupHoldOnGameThread(this))return;
	if (HostTransitionDelayTicks > 0)
	{
		--HostTransitionDelayTicks;
		return;
	}
	const bool bExecutionPaused = bOwnedSimulationPaused.Load() ||
		Speed::CanonicalFrameDriver::IsOwnedThreadPaused();

	switch (GetActiveExecutionMode())
	{
	case ESimulationExecutionMode::UnrealAsyncCallback:
		return;
	case ESimulationExecutionMode::GameThread:
	{
		if (bExecutionPaused || bOwnedWorkerTerminal.Load())
		{
			return;
		}
		GameThreadAccumulatorSeconds += FMath::Max(0.0f, DeltaSeconds);
		const double FrameIntervalSeconds =
			static_cast<double>(FCanonicalFrameContext::CanonicalPhysicalDeltaTime);
		while (GameThreadAccumulatorSeconds + UE_DOUBLE_SMALL_NUMBER >= FrameIntervalSeconds)
		{
			const ESimulationWorkerResult Result = ExecuteCheckedSimulationPulse([this]()
            { return DriveSimulation(FCanonicalFrameContext::CanonicalPhysicalDeltaTime, GetCanonicalPulseSimTime()); });
			if (Result == ESimulationWorkerResult::Failed ||
				Result == ESimulationWorkerResult::Complete)
			{
				if (Result == ESimulationWorkerResult::Failed)
				{
					bOwnedWorkerTerminal.Store(true);
					bOwnedSimulationPaused.Store(true);
				}
				GameThreadAccumulatorSeconds = 0.0;
				return;
			}
			GameThreadAccumulatorSeconds -= FrameIntervalSeconds;
			if (Result == ESimulationWorkerResult::Idle)
			{
				// Readiness delay is wall-clock time, not simulation time. Keep at
				// most one frame of debt and retry on the next game-thread tick.
				GameThreadAccumulatorSeconds = FMath::Min(
					GameThreadAccumulatorSeconds, FrameIntervalSeconds);
				return;
			}
		}
		return;
	}
	case ESimulationExecutionMode::IAmSpeedThread:
		StartOwnedWorkerIfReady();
		if (SimulationWorker)
		{
			if (bExecutionPaused && !SimulationWorker->IsPaused())
			{
				SimulationWorker->Pause();
			}
			else if (!bExecutionPaused && SimulationWorker->IsPaused())
			{
				SimulationWorker->Resume();
			}
		}
		return;
	default:
		return;
	}
}

void ASpeedPhysicsSimulation::RefreshExecutionMode()
{
	const ESimulationExecutionMode DesiredMode =
		Speed::CanonicalFrameDriver::IsEnabled()
		? Speed::CanonicalFrameDriver::GetExecutionMode()
		: ESimulationExecutionMode::UnrealAsyncCallback;
	if (!bExecutionModeInitialized || DesiredMode != GetActiveExecutionMode())
	{
		TransitionExecutionMode(DesiredMode);
	}
}

void ASpeedPhysicsSimulation::TransitionExecutionMode(
	const ESimulationExecutionMode NewMode)
{
    if (IsInGameThread() && bStaticWorldPublicationLease) return;
	if (bInputOwnerRetired.Load()) return;
    bAsyncPhysicsTickEnabled = false;
    StopOwnedWorker();
    Speed::ReplayLifecycle::RetireHost(*this);
    if (!ValidateAnalyticNumericOwner()) return;
    // A manual unsupported mode change invalidates startup admission; the
    // recording owner must refuse rather than override the requested mode.
    if(NewMode!=ESimulationExecutionMode::IAmSpeedThread)
    {ReplayStartupHolds.Remove(ReplayStartupKeyFor(this));NativeFrameStartupHolds.Remove(ReplayStartupKeyFor(this));}

	ResetCanonicalFrame();
	GameThreadAccumulatorSeconds = 0.0;
	bOwnedWorkerTerminal.Store(bCanonicalPublicationTerminal.Load());
	ActiveExecutionModeValue.Store(static_cast<uint8>(NewMode));
	bExecutionModeInitialized = true;
	HostTransitionDelayTicks = NewMode == ESimulationExecutionMode::UnrealAsyncCallback ? 0 : 1;
	if (NewMode == ESimulationExecutionMode::UnrealAsyncCallback)
	{
		bAsyncPhysicsTickEnabled = true;
	}
}

void ASpeedPhysicsSimulation::StartOwnedWorkerIfReady()
{
    if (IsInGameThread() && bStaticWorldPublicationLease) return;
    // Never validate GT as numeric owner of an already running worker.
    // Retain common startup observations around the same terminal guards.
    if (SimulationWorker)
    { ObservePrivateInputStartup(EPrivateInputStartupObservation::WorkerPresent); return; }
    if (bOwnedWorkerTerminal.Load())
    { ObservePrivateInputStartup(EPrivateInputStartupObservation::WorkerTerminal); return; }
    if (bCanonicalPublicationTerminal.Load() || !ValidateAnalyticNumericOwner()) return;
    if (HasReplayStartupHoldOnGameThread(this))
    { ObservePrivateInputStartup(EPrivateInputStartupObservation::ReplayHold); return; }
    if (!EnsureSimulationWorldReady())
    { ObservePrivateInputStartup(EPrivateInputStartupObservation::WorldNotReady); return; }
    if (!ValidateAnalyticNumericOwner()) return;
	InitializeCanonicalFrame(0.0f);
	OnOwnedWorkerStarting();
#if WITH_DEV_AUTOMATION_TESTS
	// Admission is checked from the class that owns the private execution mode.
	const auto OwnedAudit = Speed::OwnedFrameAudit::CreateForHost(this,
		GetActiveExecutionMode() == ESimulationExecutionMode::IAmSpeedThread &&
		IsA<ARealTimeSimulation>() && CVarSimulationPerformanceAudit.GetValueOnGameThread() != 0,
		FCommandLine::Get());
#endif
	SimulationWorker = MakeUnique<FSimulationWorker>(
		[this
#if WITH_DEV_AUTOMATION_TESTS
		, OwnedAudit
#endif
#if !UE_BUILD_SHIPPING
        , bActorProfileStarted=false
#endif
        ]() mutable
        {
#if !UE_BUILD_SHIPPING
            // Reuse the existing default-off aggregate profile on its owning
            // lane. It covers normal controlled-run worker generations.
            if(!bActorProfileStarted)
            {
                if(IsA<ARealTimeSimulation>())Speed::ActorDiagnostics::BeginRun();
                bActorProfileStarted=true;
            }
#endif
#if WITH_DEV_AUTOMATION_TESTS
			Speed::OwnedFrameAudit::BeginWork(OwnedAudit, CanonicalNumFrame);
#endif
			const ESimulationWorkerResult Result = ExecuteCheckedSimulationPulse([this]()
            {
                if (Speed::OnePausedWork::IsPreparationPrefixFor(this)||Speed::OnePausedWork::IsPostPrepareInitialPositionFor(this)||Speed::OnePausedWork::IsTerminalStepSyncFor(this)||Speed::OnePausedWork::IsTerminalSphereRsCaptureFor(this)||Speed::OnePausedWork::IsTerminalConsumedPrefixAdvanceFor(this)||Speed::OnePausedWork::IsTerminalRsContactAdvanceFor(this)||Speed::OnePausedWork::IsTerminalRsResponseFor(this)||Speed::OnePausedWork::IsTerminalRsResponseSyncFor(this)||Speed::OnePausedWork::IsTerminalPostSyncRsCaptureFor(this))
                {
                    bool noRollback=false;
                    {FScopeLock lock(&RollbackRequestMutex);noRollback=!PendingRollbackRequest.IsSet();}
                    if (!Speed::OnePausedWork::BeginRun(this,SpeedWorldSubsystem,SimulationWorker.Get(),
                        InputWorkerGeneration,CanonicalNumFrame,1,noRollback)) return ESimulationWorkerResult::Failed;
                    const bool produced=StepCanonicalFrame(MakeCanonicalFrameContext(CanonicalNumFrame));
                    if(!produced&&Speed::OnePausedWork::HasTerminalConsumedPrefixAdvanceStop(this))return ESimulationWorkerResult::TerminalConsumedPrefixAdvanceStopped;
                    if(!produced&&Speed::OnePausedWork::HasTerminalRsContactAdvanceStop(this))return ESimulationWorkerResult::TerminalRsContactAdvanceStopped;
                    if(!produced&&Speed::OnePausedWork::HasTerminalRsResponseStop(this))return ESimulationWorkerResult::TerminalRsResponseStopped;
                    if(!produced&&Speed::OnePausedWork::HasTerminalPostSyncRsCaptureStop(this))return ESimulationWorkerResult::TerminalPostSyncRsCaptureStopped;
                    if(!produced&&Speed::OnePausedWork::HasTerminalRsResponseSyncStop(this))return ESimulationWorkerResult::TerminalRsResponseSyncStopped;
                    if(!produced&&Speed::OnePausedWork::HasTerminalSphereRsCaptureStop(this))return ESimulationWorkerResult::TerminalSphereRsCaptureStopped;
                    if(!produced&&Speed::OnePausedWork::HasTerminalStepSyncStop(this))return ESimulationWorkerResult::TerminalStepSyncStopped;
                    if(!produced&&Speed::OnePausedWork::HasPostPrepareInitialPositionStop(this))return ESimulationWorkerResult::PostPrepareInitialPositionStopped;
                    return !produced&&Speed::OnePausedWork::HasPreparationStop(this)
                        ? ESimulationWorkerResult::PreparationPrefixStopped : ESimulationWorkerResult::Failed;
                }
                const auto nativeContext=MakeCanonicalFrameContext(CanonicalNumFrame);
                Speed::OnePausedWork::FNativeFrameStreamScope nativeStream(this,SpeedWorldSubsystem,SimulationWorker.Get(),
                    nativeContext.NumFrame,nativeContext.PhysicalDeltaTime,nativeContext.SimTime);
                const auto pulse=DriveOwnedWorkerPulse(FCanonicalFrameContext::CanonicalPhysicalDeltaTime, GetCanonicalPulseSimTime());
                return nativeStream.Finish(pulse,CanonicalNumFrame);
            });
#if WITH_DEV_AUTOMATION_TESTS
			Speed::OwnedFrameAudit::EndWork(OwnedAudit, Result);
#endif
			if (Result == ESimulationWorkerResult::Complete ||
				Result == ESimulationWorkerResult::Failed || Result == ESimulationWorkerResult::PreparationPrefixStopped || Result == ESimulationWorkerResult::PostPrepareInitialPositionStopped || Result == ESimulationWorkerResult::TerminalStepSyncStopped || Result == ESimulationWorkerResult::TerminalSphereRsCaptureStopped || Result == ESimulationWorkerResult::TerminalConsumedPrefixAdvanceStopped || Result == ESimulationWorkerResult::TerminalRsContactAdvanceStopped || Result == ESimulationWorkerResult::TerminalRsResponseStopped || Result == ESimulationWorkerResult::TerminalRsResponseSyncStopped || Result == ESimulationWorkerResult::TerminalPostSyncRsCaptureStopped)
			{
				bOwnedWorkerTerminal.Store(true);
			}
			return Result;
		},
		[this
#if WITH_DEV_AUTOMATION_TESTS
		, OwnedAudit
#endif
		](FSimulationWorkerWaitContext& WaitContext)
		{
#if WITH_DEV_AUTOMATION_TESTS
			Speed::OwnedFrameAudit::BeginWait(OwnedAudit);
#endif
			WaitBetweenFrames(WaitContext);
#if WITH_DEV_AUTOMATION_TESTS
			Speed::OwnedFrameAudit::EndWait(OwnedAudit);
#endif
		},
		[this
#if WITH_DEV_AUTOMATION_TESTS
		, OwnedAudit
#endif
		](bool bPauseRequested)
		{
#if WITH_DEV_AUTOMATION_TESTS
			if (OwnedAudit) Speed::OwnedFrameAudit::OwnerBoundary(OwnedAudit,
				CVarSimulationPerformanceAudit.GetValueOnAnyThread() != 0);
			Speed::OwnedFrameAudit::BeginService(OwnedAudit, bPauseRequested, CanonicalNumFrame);
#endif
			const auto Result = ServiceInputSessionBoundary(bPauseRequested);
#if WITH_DEV_AUTOMATION_TESTS
			Speed::OwnedFrameAudit::EndService(OwnedAudit, Result);
#endif
			return Result;
		},
		[this
#if WITH_DEV_AUTOMATION_TESTS
		, OwnedAudit
#endif
		]()
		{
#if WITH_DEV_AUTOMATION_TESTS
			Speed::OwnedFrameAudit::FreezeOnOwner(OwnedAudit, Speed::OwnedFrameAudit::EFreezeReason::WorkerClose);
			Speed::OwnedFrameAudit::TryExportFrozen(OwnedAudit);
#endif
#if !UE_BUILD_SHIPPING
        EndIndependentFastCoverage276(false,TEXT("WorkerCloseIncompleteUnlessAlreadyEnded"));
        Speed::Analytic::CaseSatDiagnostics::FlushOnOwnerThread(TEXT("WorkerCloseBeforeInputOwners"));
            if(IsA<ARealTimeSimulation>())Speed::ActorDiagnostics::EndRun();
#endif
			CloseInputSessionsOnWorker();
		}, [this]()
        {
            // Real structural ACK/release only: discard pacing debt through the
            // existing pacing hook, without changing the user's pause intent.
            OnOwnedSimulationPaused();
        });
	if (!SimulationWorker->Start(bOwnedSimulationPaused.Load() || Speed::CanonicalFrameDriver::IsOwnedThreadPaused()))
	{
		SimulationWorker.Reset();
		bOwnedWorkerTerminal.Store(true);
		UE_LOG(LogTemp, Error, TEXT("[SimulationWorkerStartFailed]"));
        ObservePrivateInputStartup(EPrivateInputStartupObservation::StartFailed);
	}
    else ObservePrivateInputStartup(EPrivateInputStartupObservation::Started);
}

void ASpeedPhysicsSimulation::StopOwnedWorker()
{
	if (SimulationWorker)
	{
		SimulationWorker->StopAndJoin();
		SimulationWorker.Reset();
	}
    RetireLiveBoundaryRequest();
}

bool ASpeedPhysicsSimulation::PrepareControlledInputRun()
{
    if (IsInGameThread() && bStaticWorldPublicationLease) return false;
	check(IsInGameThread());
	if (GetActiveExecutionMode() != ESimulationExecutionMode::IAmSpeedThread
		|| bCanonicalPublicationTerminal.Load()) return false;
	const auto Boundary = TryPauseOwnedSimulation();
	if (Boundary != ESimulationQuiescence::BoundaryAcknowledged
		&& Boundary != ESimulationQuiescence::AlreadyStopped) return false;
	StopOwnedWorker(); // CloseInputSessionsOnWorker retires producers on their lane.
	FScopeLock Lock(&InputSessionAdmissionMutex);
	if (InputSessionRegistry
		|| (bInputSessionAdmissionRequested.Load() && !bInputOwnersClosedOnWorker.Load())
		|| !PendingInputJournals.empty() || !PendingInputObservations.empty()
		|| !InputObservations.empty() || bCanonicalPublicationTerminal.Load()) return false;
	try
	{
		auto NextCommands = std::make_shared<Speed::Input::V2::FInputSessionCommands>();
		const auto Generation = NextInputWorkerGeneration.fetch_add(1);
		if (!Generation || Generation == MAX_uint64) return false;
		std::shared_ptr<const Speed::FCompletedSimulationSource> NextSource;
		if (bPublishCompletedWorldInput) NextSource.reset(new Speed::FCompletedSimulationSource(Generation));
		InputSessionCommands = std::move(NextCommands);
		CompletedPublicationSource = std::move(NextSource);
		InputWorkerGeneration = Generation; NextInputCommandId = 1;
	}
	catch (...) { return false; }
	PublishedInputRegistry.store(std::shared_ptr<const Speed::Input::V2::FInputRegistryView>{}, std::memory_order_seq_cst);
	PublishedInputFrame.store(std::shared_ptr<const Speed::Input::V2::FRegistryFrame>{}, std::memory_order_seq_cst);
	PublishedCompletedWorldInput.store({}, std::memory_order_seq_cst);
	bInputSessionAdmissionRequested.Store(false); bInputOwnersClosedOnWorker.Store(false);
	bInputNeutralizedDuringPause = false; NeutralizedInputRegistryVersion = MAX_uint64;
	bInputOwnerRetired.Store(false); bOwnedWorkerTerminal.Store(false);
	CanonicalReadyDelayPulsesObserved = 0;
	return true; // Remains paused until the new scenario is sealed.
}

void ASpeedPhysicsSimulation::RestartControlledRun()
{
	check(IsInGameThread());
	if (bInputOwnerRetired.Load() || bOwnedWorkerTerminal.Load()
		|| bCanonicalPublicationTerminal.Load() || !IsOwnedSimulationPaused())
	{
		UE_LOG(LogTemp, Error, TEXT("[ControlledRunNotPrepared] Prepare the new generation before admitting scenario inputs."));
		return;
	}
	// Never join here: replacement descriptors already belong to this generation.
	CanonicalReadyDelayPulsesObserved = 0;
	ResumeOwnedSimulation();
}

#if !UE_BUILD_SHIPPING
ESimulationWorkerResult ASpeedPhysicsSimulation::ReadOwnedFastBatchCompletion() const
{
    check(IsInGameThread());
    if(!SimulationWorker)return ESimulationWorkerResult::Failed;
    // Run publishes the outcome after its worker-side close callback, before
    // clearing bRunning. A boundary pause ACK is a different observation.
    if(SimulationWorker->IsRunning())return ESimulationWorkerResult::Idle;
    if(!SimulationWorker->bClosedAfterNaturalCompletion.Load()||bCanonicalPublicationTerminal.Load()||
        (bInputSessionAdmissionRequested.Load()&&!bInputOwnersClosedOnWorker.Load()))
        return ESimulationWorkerResult::Failed;
    return ESimulationWorkerResult::Complete;
}
#endif

ESimulationQuiescence ASpeedPhysicsSimulation::TryPauseOwnedSimulation(const uint32 TimeoutMilliseconds)
{
	check(IsInGameThread());
	bOwnedSimulationPaused.Store(true);
	if (!SimulationWorker && HasActorBegunPlay()
		&& GetActiveExecutionMode() == ESimulationExecutionMode::UnrealAsyncCallback)
		return ESimulationQuiescence::Failed; // No owned boundary for legacy async callbacks.
	if (!SimulationWorker || !SimulationWorker->IsRunning())
	{
		OnOwnedSimulationPaused();
		return ESimulationQuiescence::AlreadyStopped;
	}
	if (!SimulationWorker->TryPause(TimeoutMilliseconds)) return ESimulationQuiescence::TimedOut;
	OnOwnedSimulationPaused();
	return ESimulationQuiescence::BoundaryAcknowledged;
}

ESimulationQuiescence ASpeedPhysicsSimulation::TrySuspendOwnedBoundaryService(const uint32 TimeoutMilliseconds)
{
	check(IsInGameThread());
	if (GetActiveExecutionMode() != ESimulationExecutionMode::IAmSpeedThread)
		return ESimulationQuiescence::Failed;
    if (LiveBoundaryControl && LiveBoundaryControl->ScopeActive)
    {
        if (!HasLiveBoundaryScope() || OwnedBoundarySuspendDepth == MAX_uint32) return ESimulationQuiescence::Failed;
        ++OwnedBoundarySuspendDepth;
        return ESimulationQuiescence::BoundaryAcknowledged;
    }
    // A GT lifecycle/actor operation outside the callback may need the original
    // legacy barrier. Cancel this driver's pending live request first, so it
    // cannot park ahead of that legacy ACK and turn an ordinary bind/cleanup
    // into a timeout. The worker disposes it before servicing the legacy park.
    if (LiveBoundaryControl)
    {
        Speed::SimulationBoundary::Detail::RequestLiveBoundaryCancellation(LiveBoundaryControl);
        if (SimulationWorker) SimulationWorker->WakeLiveBoundaryControl();
    }
	if (OwnedBoundarySuspendDepth)
	{
		// Joining while a recreation lease is held supersedes the parked
		// boundary. The GT may finish teardown after the worker is gone; the
		// outstanding lease still has to be released exactly once.
		if (!SimulationWorker || !SimulationWorker->IsRunning())
			return ESimulationQuiescence::AlreadyStopped;
		if (!SimulationWorker->IsBoundaryServiceSuspended())
			return ESimulationQuiescence::Failed;
		++OwnedBoundarySuspendDepth;
		return ESimulationQuiescence::BoundaryAcknowledged;
	}
	if (!SimulationWorker || !SimulationWorker->IsRunning())
		return ESimulationQuiescence::AlreadyStopped;
	bOwnedSimulationPaused.Store(true);
	if (!SimulationWorker->TrySuspendBoundaryService(TimeoutMilliseconds))
		return ESimulationQuiescence::TimedOut;
	OwnedBoundarySuspendDepth = 1;
	return ESimulationQuiescence::BoundaryAcknowledged;
}

void ASpeedPhysicsSimulation::ResumeOwnedBoundaryService()
{
	check(IsInGameThread());
	if (!OwnedBoundarySuspendDepth) return;
    if (--OwnedBoundarySuspendDepth == 0 && SimulationWorker && !HasLiveBoundaryScope())
    {
        using namespace Speed::SimulationBoundary::Detail;
        if (LiveBoundaryControl && LiveBoundaryControl->CallbackRan && !LiveBoundaryControl->ScopeActive
            && LiveBoundaryControl->Phase.load() == ELiveBoundaryPhase::Executing)
        {
            LiveBoundaryControl->Phase.store(ELiveBoundaryPhase::ReleaseRequested);
            SimulationWorker->WakeLiveBoundaryControl();
        }
        else SimulationWorker->ResumeBoundaryService();
    }
}

bool ASpeedPhysicsSimulation::IsOwnedBoundaryServiceSuspended() const
{
	return HasLiveBoundaryScope() || (OwnedBoundarySuspendDepth && SimulationWorker && SimulationWorker->IsBoundaryServiceSuspended());
}

void ASpeedPhysicsSimulation::PauseOwnedSimulation()
{
	bOwnedSimulationPaused.Store(true);
	if (SimulationWorker)
	{
		SimulationWorker->Pause();
	}
	OnOwnedSimulationPaused();
}

bool ASpeedPhysicsSimulation::ReadInputFirstFrameAtPausedBoundary(uint64& OutFrame)
{
	check(IsInGameThread());
	if (GetActiveExecutionMode() != ESimulationExecutionMode::IAmSpeedThread
		|| bInputOwnerRetired.Load() || !bOwnedSimulationPaused.Load() || bCanonicalPublicationTerminal.Load()
		|| (SimulationWorker && SimulationWorker->IsRunning() && !SimulationWorker->IsPaused()))
	{
		// Current atomic observations after the existing rejection, not a new ACK/readiness predicate.
		RecordPrivateInputFailure(TEXT("SpeedPhysicsSimulation.ReadFirstFramePausedBoundaryRejected"), FString::Printf(
			TEXT("mode=%u retired=%d paused=%d publication_terminal=%d worker_present=%d worker_running=%d worker_paused=%d worker_terminal=%d driver_uid=%u generation=%llu"),
			uint32(GetActiveExecutionMode()), int32(bInputOwnerRetired.Load()), int32(bOwnedSimulationPaused.Load()),
			int32(bCanonicalPublicationTerminal.Load()), int32(bool(SimulationWorker)),
			SimulationWorker ? int32(SimulationWorker->IsRunning()) : -1,
			SimulationWorker ? int32(SimulationWorker->IsPaused()) : -1,
			int32(bOwnedWorkerTerminal.Load()), GetUniqueID(), GetInputWorkerGeneration()));
		return false;
	}
	InitializeCanonicalFrame(0.0f); OutFrame = CanonicalNumFrame; return true;
}

bool ASpeedPhysicsSimulation::JoinOwnedSimulationForInputTeardown()
{
	check(IsInGameThread());
	// An async callback (or absent driver) cannot grant an owned-thread fence.
	if (GetActiveExecutionMode() != ESimulationExecutionMode::IAmSpeedThread) return false;
	bInputOwnerRetired.Store(true);
	bOwnedSimulationPaused.Store(true); bOwnedWorkerTerminal.Store(true);
	StopOwnedWorker();
#if !UE_BUILD_SHIPPING
    // Passive, cumulative closed-frame counters. The owned worker is joined;
    // this does not request a further pulse or change run completion state.
    if (IsOwnedWorkerExecutionMode() && PerformanceFrameCount &&
        FParse::Param(FCommandLine::Get(), TEXT("SLStadiumNativeFastBatch")))
        UE_LOG(LogTemp, Display, TEXT("[JoinedPhysicalFramePerformance] Boundary=InputTeardown Host=%u Cumulative=1 Frames=%llu TotalStepMs=%.9f AverageStepMs=%.9f MaximumStepMs=%.9f WarningFrames=%llu IterationLimits=%llu"),
            GetUniqueID(), PerformanceFrameCount, PerformanceTotalStepMilliseconds,
            PerformanceTotalStepMilliseconds / double(PerformanceFrameCount), PerformanceMaximumStepMilliseconds,
            PerformanceWarningFrameCount, PerformanceIterationLimitCount);
#endif

	if (SimulationWorker) return false;
	if (!bInputOwnersClosedOnWorker.Load())
	{
		FScopeLock Lock(&InputSessionAdmissionMutex);
		// No published registry plus no surviving worker-owned registry means
		// only inert, unserviced descriptors may remain. Do not manufacture a
		// worker-owner retirement receipt for a registry that actually ran.
		if (InputSessionRegistry || ReadInputRegistryView() || !InputObservations.empty()
			|| !InputSessionCommands->CancelUnprocessedAfterJoin()) return false;
		for (const auto& Observation : PendingInputObservations) Observation.second->Deactivate();
		PendingInputObservations.clear(); PendingInputJournals.clear();
		bInputOwnersClosedOnWorker.Store(true); // Joined proof: zero owners were constructed.
	}
	return true;
}

void ASpeedPhysicsSimulation::ResumeOwnedSimulation()
{
    if (IsInGameThread() && bStaticWorldPublicationLease) return;
    // Native input/gameplay callers are GT owners. Refuse a foreign-thread
    // resume before consulting GT lifecycle ownership or changing pause state.
    if(!IsInGameThread())return;
	if (bInputOwnerRetired.Load()) return;
    // A resumed device does not prove the other owners (bots included) have
    // acknowledged Resume. Keep servicing boundaries, but do not step a
    // partially paused registry: PrepareFrame correctly rejects that state.
    const auto View = ReadInputRegistryView();
    if (View)
    {
        if (View->Terminal || View->Phases.size() != View->Bindings.size()) return;
        for (const auto& InputOwnerPhase : View->Phases)
            if (InputOwnerPhase.Phase != Speed::Input::V2::ESessionPhase::Active) return;
    }

	// A timed-out structural request remains fail-closed until a later caller
	// claims its ACK or joins the worker. Do not publish a false resumed state.
	if (SimulationWorker && SimulationWorker->IsBoundaryServiceSuspendRequested()) return;
    // Publication readiness does not authorize an ordinary worker pulse.
    // Input resumes can arrive during the paused native bootstrap. Keep them
    // paused until the actual first FullFrame has closed and armed this worker.
    if(Speed::PreparedExecution::Requested())
    {
        const auto* provider=SpeedWorldSubsystem?SpeedWorldSubsystem->GetStaticCollisionWorld():nullptr;
        const auto snapshot=provider?provider->GetPreparedSnapshot():Speed::FPreparedWorldSnapshotPtr{};
        const auto consumer=provider?provider->GetPreparedNativeFrameConsumer():Speed::FPreparedNativeFrameConsumerPtr{};
        if(!provider||!provider->IsReadyForPhysicsStep()||!snapshot||!consumer||
            snapshot->GetOrigin().Host!=this||snapshot->GetOrigin().Bridge!=SpeedWorldSubsystem||
            snapshot->GetOrigin().World!=GetWorld()||snapshot->GetOrigin().Generation!=GetInputWorkerGeneration()||
            !consumer->Matches(snapshot,provider->GetPreparedQueryService())||
            !consumer->IsContinuousStreamArmedFor(SimulationWorker.Get()))return;
    }
    if (Speed::ReplayLifecycle::DeferResume(this)) return;
	bOwnedSimulationPaused.Store(false);
	OnOwnedSimulationResumed();
	if (SimulationWorker &&
		!Speed::CanonicalFrameDriver::IsOwnedThreadPaused())
	{
		SimulationWorker->Resume();
	}
}

ESimulationExecutionMode ASpeedPhysicsSimulation::GetActiveExecutionMode() const
{
	return static_cast<ESimulationExecutionMode>(ActiveExecutionModeValue.Load());
}

bool ASpeedPhysicsSimulation::IsOwnedWorkerExecutionMode() const
{
	return GetActiveExecutionMode() == ESimulationExecutionMode::IAmSpeedThread;
}

float ASpeedPhysicsSimulation::GetCanonicalPulseSimTime() const
{
	return static_cast<float>(CanonicalNumFrame) *
		FCanonicalFrameContext::CanonicalPhysicalDeltaTime;
}

void ASpeedPhysicsSimulation::UpdateNumFrame(const float& SimTime)
{
	_NumFrame = Speed::SimUtils::ComputeNumFrameFromSimTime(EngineFPS, SimTime);
}

unsigned int ASpeedPhysicsSimulation::NumFrame() const
{
    return _NumFrame;
}

USpeedWorldSubsystem* ASpeedPhysicsSimulation::GetSpeedWorldSubsystem(UWorld* World)
{
    if (!World) return nullptr;
    USpeedWorldSubsystem* SS = World->GetSubsystem<USpeedWorldSubsystem>();
	return SS;
}

void ASpeedPhysicsSimulation::AsyncPhysicsTickActor(float Dt, float SimTime)
{
    if (bCanonicalPublicationTerminal.Load() || bOwnedWorkerTerminal.Load()) return;

	if (GetActiveExecutionMode() != ESimulationExecutionMode::UnrealAsyncCallback)
	{
		return;
	}
	if (bOwnedSimulationPaused.Load() ||
		Speed::CanonicalFrameDriver::IsOwnedThreadPaused())
	{
		return;
	}
	QUICK_SCOPE_CYCLE_COUNTER(STAT_IAmSpeed_AsyncPhysicsTickActor);
	TRACE_CPUPROFILER_EVENT_SCOPE(IAmSpeed_AsyncPhysicsTickActor);
	const double CallbackStartSeconds = FPlatformTime::Seconds();
	const uint64 StepSerialAtEntry = SpeedWorldSubsystem
		? SpeedWorldSubsystem->GetLastStepDiagnostics().Serial : 0;
	ON_SCOPE_EXIT
	{
		if (!SpeedWorldSubsystem || !ShouldMeasureCallback())
		{
			return;
		}

		const FSpeedStepDiagnostics& Diagnostics =
			SpeedWorldSubsystem->GetLastStepDiagnostics();
		if (Diagnostics.Serial == StepSerialAtEntry)
		{
			return;
		}

		const double CallbackMilliseconds =
			(FPlatformTime::Seconds() - CallbackStartSeconds) * 1000.0;
		const double BudgetMilliseconds =
			Diagnostics.PhysicalDeltaTimeMilliseconds * FMath::Max(
				0.0, static_cast<double>(
					CVarSimulationWarningFrameFraction.GetValueOnAnyThread()));
		const bool bSlow = CallbackMilliseconds > BudgetMilliseconds;
		const bool bDetailed =
			CVarSimulationPerformanceAudit.GetValueOnAnyThread() != 0;
		const bool bReportSlow = bSlow &&
			CVarSimulationSlowFrameWarnings.GetValueOnAnyThread() != 0 &&
			(SlowFrameReportCount == 0 ||
				Diagnostics.Frame >= LastSlowFrameReport + 300u);
		if (!bDetailed && !bReportSlow)
		{
			return;
		}

		if (bReportSlow)
		{
			++SlowFrameReportCount;
			LastSlowFrameReport = Diagnostics.Frame;
		}
		const FString Message = FString::Printf(
			TEXT("[SpeedPhysicalFrame] Frame=%u CallbackMs=%.6f BudgetMs=%.6f StepMs=%.6f ResetMs=%.6f SweepMs=%.6f StaticAuthorityMs=%.6f StaticAuthorityMaxMs=%.6f IntegrateMs=%.6f ResolveMs=%.6f ProjectionMs=%.6f PostMs=%.6f Iterations=%d MaxIterations=%d ComponentSweeps=%d StaticQueries=%u LegacySweeps=%u AuthorityAttempts=%u ResolvedEvents=%d IterationLimit=%d Slow=%d"),
			Diagnostics.Frame, CallbackMilliseconds, BudgetMilliseconds,
			Diagnostics.TotalMilliseconds, Diagnostics.ResetMilliseconds,
			Diagnostics.SweepMilliseconds, Diagnostics.StaticAuthorityMilliseconds,
			Diagnostics.MaximumStaticAuthorityMilliseconds,
			Diagnostics.IntegrateMilliseconds,
			Diagnostics.ResolveMilliseconds,
			Diagnostics.ProjectionMilliseconds, Diagnostics.PostMilliseconds,
			Diagnostics.IterationCount, Diagnostics.MaximumIterationCount,
			Diagnostics.ComponentSweepCount, Diagnostics.StaticQueryCount,
			Diagnostics.LegacySweepCount, Diagnostics.AuthorityAttemptCount,
			Diagnostics.ResolvedEventCount,
			Diagnostics.bIterationLimitReached ? 1 : 0, bSlow ? 1 : 0);
		if (bSlow)
		{
			UE_LOG(LogTemp, Warning, TEXT("%s"), *Message);
		}
		else
		{
			UE_LOG(LogTemp, Display, TEXT("%s"), *Message);
		}
	};

	const auto Result=ExecuteCheckedSimulationPulse([&]() { return DriveSimulation(Dt, SimTime); });
    if (Result == ESimulationWorkerResult::Failed)
    { bCanonicalPublicationTerminal.Store(true); bOwnedWorkerTerminal.Store(true); }
}

bool ASpeedPhysicsSimulation::ValidateAnalyticNumericOwner()
{
    if (bCanonicalPublicationTerminal.Load()) return false;
    const auto* Subsystem=SpeedWorldSubsystem ? SpeedWorldSubsystem : GetSpeedWorldSubsystem(GetWorld());
    const auto* Data=Subsystem ? Subsystem->GetAnalyticWorldData() : nullptr;
    const auto Contract=Data ? Data->NumericContract : Speed::Analytic::EAnalyticNumericContract::Unspecified;
    try
    {
        Speed::Analytic::RequireCurrentAnalyticNumericContext(Contract);
        if(const auto* Provider=Subsystem?Subsystem->GetStaticCollisionWorld():nullptr)
            Speed::Analytic::RequireCurrentAnalyticNumericContext(Provider->GetNumericContract());
        return true;
    }
    catch (const Speed::Analytic::FAnalyticNumericUnknown& Unknown)
    {
        bCanonicalPublicationTerminal.Store(true); bOwnedWorkerTerminal.Store(true);
        const auto& E=Unknown.Evidence;
        UE_LOG(LogTemp, Error, TEXT("[AnalyticNumericUnknown] Phase=OwnerPreflight Reason=%u MXCSR=%08X FCW=%04X NewStep=0 Terminal=1"),
            uint32(E.Failure),E.VectorControl,E.ScalarControl);
        return false;
    }
}

ESimulationWorkerResult ASpeedPhysicsSimulation::ExecuteCheckedSimulationPulse(
    TFunctionRef<ESimulationWorkerResult()> Work)
{
    if (!ValidateAnalyticNumericOwner()) return ESimulationWorkerResult::Failed;
    try { return Work(); }
    catch (const Speed::Analytic::FAnalyticNumericUnknown& Unknown)
    {
        const auto& E=Unknown.Evidence;
        UE_LOG(LogTemp, Error, TEXT("[AnalyticNumericUnknown] Phase=Pulse Reason=%u MXCSR=%08X FCW=%04X MutationPossible=1 Terminal=1"),
            uint32(E.Failure),E.VectorControl,E.ScalarControl);
    }
    catch (const std::exception& Error)
    { UE_LOG(LogTemp, Error, TEXT("[SimulationPulseUnknown] %hs MutationPossible=1 Terminal=1"),Error.what()); }
    catch (...)
    { UE_LOG(LogTemp, Error, TEXT("[SimulationPulseUnknown] Unclassified MutationPossible=1 Terminal=1")); }
    // Terminal failure is not a kinematic rollback. Teardown may still neutralize inputs.
    bCanonicalPublicationTerminal.Store(true); bOwnedWorkerTerminal.Store(true);
    return ESimulationWorkerResult::Failed;
}

bool ASpeedPhysicsSimulation::EnsureSimulationWorldReady()
{
	if (!SpeedWorldSubsystem)
	{
		SpeedWorldSubsystem = GetSpeedWorldSubsystem(GetWorld());
	}
	if (!SpeedWorldSubsystem) return false;
	if (Speed::Analytic::FStaticWorldQueryAudit::IsSurfaceAnalyticBackend() &&
		(!SpeedWorldSubsystem->GetStaticCollisionWorld() || !SpeedWorldSubsystem->GetStaticCollisionWorld()->IsReadyForPhysicsStep()))
	{
		if (!bStaticCollisionReadinessErrorReported)
		{
			bStaticCollisionReadinessErrorReported = true;
			UE_LOG(LogTemp, Error,
				TEXT("[StaticCollisionAuthorityNotReady] Canonical simulation is waiting for a valid analytical static collision world."));
		}
		return false;
	}
	bStaticCollisionReadinessErrorReported = false;
	return true;
}

ESimulationWorkerResult ASpeedPhysicsSimulation::CheckCanonicalRunReadiness()
{
    const bool nativeRequested=Speed::PreparedExecution::Requested();
    const auto nativeContext=MakeCanonicalFrameContext(CanonicalNumFrame);
    const auto nativeReady=[&]()
    {
        const bool supportedNativeHost=IsA<ARealTimeSimulation>()||
            (GetClass()==AFastSimulation::StaticClass()&&Speed::CanonicalFrameDriver::IsFastSimulationEnabled()&&
                GetActiveExecutionMode()==ESimulationExecutionMode::IAmSpeedThread);
        if(!supportedNativeHost||!SpeedWorldSubsystem)return false;
        {FScopeLock lock(&RollbackRequestMutex);if(PendingRollbackRequest.IsSet())return false;}
        return Speed::PreparedExecution::FCanonicalFramePreflight::HasCurrentNativeRunControlContext(
            *this,*SpeedWorldSubsystem,SimulationWorker.Get(),InputWorkerGeneration,CanonicalNumFrame,
            nativeContext.PhysicalDeltaTime,nativeContext.SimTime);
    };
    ECanonicalRunControlState RunState;
    try
    {
        if(nativeRequested&&!nativeReady())throw std::runtime_error("Native run-control context refused");
        // Actual complete ordered aggregation, exactly once. The factory has
        // joined these real virtual implementations to the retained census.
        RunState=SpeedWorldSubsystem->GetCanonicalRunControlState();
        if(nativeRequested&&!nativeReady())throw std::runtime_error("Native run-control census changed");
    }
    catch(...)
    {
        if(!nativeRequested)throw;
        bCanonicalPublicationTerminal.Store(true);bOwnedWorkerTerminal.Store(true);
        UE_LOG(LogTemp, Error, TEXT("[PreparedRunControlRefused] Current native context/callback closure unavailable"));
        return ESimulationWorkerResult::Failed;
    }
	if (RunState == ECanonicalRunControlState::WaitingForScenario ||
		(RunState == ECanonicalRunControlState::Uncontrolled &&
			RequiresCanonicalRunController()))
	{
		CanonicalReadyDelayPulsesObserved = 0;
		return ESimulationWorkerResult::Idle;
	}
	if (RunState == ECanonicalRunControlState::Complete)
	{
		return ESimulationWorkerResult::Complete;
	}
	if (RunState == ECanonicalRunControlState::Ready &&
		CanonicalReadyDelayPulsesObserved <
			Speed::CanonicalFrameDriver::GetFastReadyDelayCallbacks())
	{
		++CanonicalReadyDelayPulsesObserved;
		return ESimulationWorkerResult::Idle;
	}
	return ESimulationWorkerResult::Advanced;
}

void ASpeedPhysicsSimulation::InitializeCanonicalFrame(const float SimTime)
{
	if (bCanonicalFrameInitialized) return;
	UpdateNumFrame(SimTime);
	CanonicalNumFrame = NumFrame() > 0 ? uint64(NumFrame() - 1u) : 0u;
	bCanonicalFrameInitialized = true;
}

void ASpeedPhysicsSimulation::ResetCanonicalFrame()
{
	bCanonicalFrameInitialized = false;
	CanonicalReadyDelayPulsesObserved = 0;
}

#if !UE_BUILD_SHIPPING
// Case-window totals may contain nested or simultaneous intervals. They are
// not an exclusive phase sum and are never added to the frame QPC partition.
static bool ObserveJoinedAnalytic1075()
{
    for(const TCHAR* Name : { TEXT("p.IAmSpeed.Parallel.AnalyticCausal1075"),
        TEXT("p.IAmSpeed.Parallel.PiecewiseProviderObserve1075"),
        TEXT("p.IAmSpeed.Parallel.TensorCellObserve1213"),
        TEXT("p.IAmSpeed.Parallel.WheelQueryPrefetch1075") })
    {
        const IConsoleVariable* CVar=IConsoleManager::Get().FindConsoleVariable(Name);
        if(CVar && CVar->GetInt()!=0)return true;
    }
    return false;
}
static void LogJoinedAnalytic1075(const FString& Context)
{
    if(!ObserveJoinedAnalytic1075() || Context.IsEmpty())return;
    Speed::Analytic::LogAnalyticCausalStats1075(*Context);
    Speed::Analytic::LogPiecewiseProviderWaveStats1075(*Context);
    Speed::Analytic::LogTensorCellWaveStats1213(*Context);
    Speed::Analytic::LogAnalyticPrefetchStats1075(*Context);
}
bool ASpeedPhysicsSimulation::ConfigureCaseSatDiagnosticsAfterJoin(
    const int32 NativeCaseId, const int32 BatchIndex, const uint64 WorldGeneration)
{
    using namespace Speed::Analytic::CaseSatDiagnostics;
    // No new pause/join or physical rejection: observe the fence already required by SLTest.
    if (!IsInGameThread() || SimulationWorker || !IsOwnedWorkerExecutionMode() ||
        !bInputOwnerRetired.Load() || !bInputOwnersClosedOnWorker.Load() || CaseSatDiagnosticEpoch==MAX_uint64)
    {
        // Never mutate identity potentially being read by an active owner, even
        // if the live cvar changed to0. This rejected diagnostic call is inert.
        try
        {
            if (IsEnabled()) UE_LOG(LogTemp, Warning, TEXT("[CaseSatUnavailable] Case=%d Reason=NoJoinedOwnedLane DiagnosticOnly=1"),NativeCaseId);
        }
        catch (...) { /* Diagnostics cannot interrupt ResetActors or mutate a live identity. */ }
        return false;
    }
    ParallelFrameTiming.ConfigureAfterJoin(uint64(reinterpret_cast<UPTRINT>(this)),
        NativeCaseId, BatchIndex, WorldGeneration);
    // Existing joined-owner fence above, outside the canonical frame body.
    if(!ParallelFrameTiming.IsActive() && ObserveJoinedAnalytic1075())
    {
        Speed::Analytic::ResetAnalyticCausalStats1075();
        Speed::Analytic::ResetPiecewiseProviderWaveStats1075();
        Speed::Analytic::ResetTensorCellWaveStats1213();
        Speed::Analytic::ResetAnalyticPrefetchStats1075();
    }
    // The already-joined owned lane is the authority. This case identity is
    // independent of CaseSat diagnostics, so real parallel kernels stay on.
    NativeWitnessCase=Speed::FNativeFrameWitnessCaseIdentity{};
    FString WitnessUUID;
    if(NativeWitnessEpoch!=MAX_uint64 && NativeCaseId>=0 &&
       FParse::Value(FCommandLine::Get(),TEXT("SLFrameWitnessRunUUID="),WitnessUUID) &&
       !WitnessUUID.IsEmpty() && WitnessUUID.Len()<=64)
    {
        NativeWitnessCase.RunUUID=MoveTemp(WitnessUUID);
        NativeWitnessCase.DriverKey=uint64(reinterpret_cast<UPTRINT>(this));
        NativeWitnessCase.Epoch=++NativeWitnessEpoch;NativeWitnessCase.WorldGeneration=WorldGeneration;
        NativeWitnessCase.NativeCaseId=NativeCaseId;NativeWitnessCase.BatchIndex=BatchIndex;
        NativeWitnessCase.bConfigured=true;
    }
    // Read the already-established joined owner fence, after mode initialization.
    // V2 is per-case and must match the real CaseBegin identity; it is not a
    // replacement for consumed input or callback/source closure.
    if(ParallelFrameTiming.IsActive() || NativeWitnessCase.bConfigured)
        UE_LOG(LogTemp,Display,TEXT("[IAmSpeedOwnedCasePolicyV2] Driver=%llu Case=%d Batch=%d WorldGeneration=%llu ExecutionHost=%u OwnedInputFence=1 Scope=ExistingJoinedOwnerFence"),
            uint64(reinterpret_cast<UPTRINT>(this)),NativeCaseId,BatchIndex,WorldGeneration,uint32(GetActiveExecutionMode()));
    if (!IsEnabled()) { CaseSatDiagnosticIdentity=FCaseIdentity{}; return true; }
    try
    {
        return BuildCaseIdentity(CaseSatDiagnosticIdentity,
            uint64(reinterpret_cast<UPTRINT>(this)),++CaseSatDiagnosticEpoch,
            NativeCaseId,BatchIndex,WorldGeneration);
    }
    catch (...) { CaseSatDiagnosticIdentity=FCaseIdentity{}; return false; }
}
#endif

#if !UE_BUILD_SHIPPING
void ASpeedPhysicsSimulation::BeginIndependentRealTimeCoverage696() noexcept
{
    ParallelFrameTiming.BeginDriver(false, CanonicalNumFrame, IsOwnedWorkerExecutionMode());
    IndependentFastCoverage276.BeginRealTime(CaseSatDiagnosticIdentity,CanonicalNumFrame,
        PerformanceFrameCount,IsOwnedWorkerExecutionMode());
}
void ASpeedPhysicsSimulation::EndIndependentRealTimeCoverage696(bool bCompleted, const TCHAR* Reason) noexcept
{
    ParallelFrameTiming.Close(bCompleted, Reason);
    if(ParallelFrameTiming.ClaimJoinedCaseLog())
        LogJoinedAnalytic1075(ParallelFrameTiming.JoinedCaseContextForLogging());
    IndependentFastCoverage276.EndRealTime(CanonicalNumFrame,PerformanceFrameCount,bCompleted,Reason);
}
void ASpeedPhysicsSimulation::BeginIndependentFastCoverage276() noexcept
{
    ParallelFrameTiming.BeginDriver(true, CanonicalNumFrame, IsOwnedWorkerExecutionMode());
    IndependentFastCoverage276.BeginFast(CaseSatDiagnosticIdentity,CanonicalNumFrame,
        PerformanceFrameCount,IsOwnedWorkerExecutionMode());
}
void ASpeedPhysicsSimulation::EndIndependentFastCoverage276(bool bCompleted, const TCHAR* Reason) noexcept
{
    ParallelFrameTiming.Close(bCompleted, Reason);
    if(ParallelFrameTiming.ClaimJoinedCaseLog())
        LogJoinedAnalytic1075(ParallelFrameTiming.JoinedCaseContextForLogging());
    IndependentFastCoverage276.EndFast(CanonicalNumFrame,PerformanceFrameCount,bCompleted,Reason);
}
#endif

FCanonicalFrameContext ASpeedPhysicsSimulation::MakeCanonicalFrameContext(const uint64 Frame) const
{
    FCanonicalFrameContext Context(Frame);
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
    if (CollisionMatrixTestHz != 0)
    {
        check(Frame >= CollisionMatrixTestFirstFrame);
        Context.PhysicalDeltaTime = 1.0f / float(CollisionMatrixTestHz);
        Context.SimTime = float(Frame - CollisionMatrixTestFirstFrame) * Context.PhysicalDeltaTime;
    }
#endif
    return Context;
}

bool ASpeedPhysicsSimulation::RunCanonicalFrames(const uint32 FrameCount)
{
    if(Speed::OnePausedWork::IsActive())
    {
        bool noRollback=false;
        {FScopeLock lock(&RollbackRequestMutex);noRollback=!PendingRollbackRequest.IsSet();}
        if(!Speed::OnePausedWork::BeginRun(this,SpeedWorldSubsystem,SimulationWorker.Get(),
            InputWorkerGeneration,CanonicalNumFrame,FrameCount,noRollback))return false;
    }
    if (IsInGameThread() && bStaticWorldPublicationLease) return false;
    if (!ValidateAnalyticNumericOwner()) return false;

#if !UE_BUILD_SHIPPING
    Speed::Diagnostics::CaseCoverage276::FScopedPreloop CallerPreloop276(IndependentFastCoverage276);
#endif
	if (!Speed::OnePausedWork::IsActive() && !ProcessPendingRollbackRequest())
	{
#if !UE_BUILD_SHIPPING
        CallerPreloop276.Finish(false);
#endif
		return false;
	}
#if !UE_BUILD_SHIPPING
    CallerPreloop276.Finish(true);
#endif
	for (uint32 FrameIndex = 0; FrameIndex < FrameCount; ++FrameIndex)
	{
#if !UE_BUILD_SHIPPING
        Speed::Diagnostics::CaseCoverage276::FScopedForward CallerForward276(IndependentFastCoverage276,CanonicalNumFrame);
        Speed::Analytic::CaseSatDiagnostics::FScopedFrame DiagnosticFrame(CaseSatDiagnosticIdentity,CanonicalNumFrame);
#endif
		const double PhysicalFrameStartSeconds = FPlatformTime::Seconds();
#if WITH_DEV_AUTOMATION_TESTS
		const auto OwnedStep = Speed::OwnedFrameAudit::BeginStep(CanonicalNumFrame, false);
#endif
		const bool StepSucceeded = StepCanonicalFrame(MakeCanonicalFrameContext(CanonicalNumFrame));
#if WITH_DEV_AUTOMATION_TESTS
		Speed::OwnedFrameAudit::EndStep(OwnedStep, true, StepSucceeded);
#endif
		if (!StepSucceeded)
		{
#if !UE_BUILD_SHIPPING
            DiagnosticFrame.Finish(false);
            CallerForward276.Finish(false);
#endif
			return false;
		}
#if !UE_BUILD_SHIPPING
        DiagnosticFrame.Finish(true);
        CallerForward276.Finish(true);
#endif
		RecordStepPerformance(
			(FPlatformTime::Seconds() - PhysicalFrameStartSeconds) * 1000.0);
		++CanonicalNumFrame;
	}
	return true;
}

void ASpeedPhysicsSimulation::Simulate(const float& DeltaTime, const float& SimTime)
{
    if(!Speed::OnePausedWork::RefuseLegacySimulate())throw std::runtime_error("Legacy Simulate forbidden in single canonical Work");
    if (IsInGameThread() && bStaticWorldPublicationLease) throw std::runtime_error("HostedGT static World publication active");
    if (!ValidateAnalyticNumericOwner()) throw std::runtime_error("Terminal or incompatible authored analytic simulation");

    if (!SpeedWorldSubsystem)
    {
		SpeedWorldSubsystem = GetSpeedWorldSubsystem(GetWorld());
		if (!SpeedWorldSubsystem)
		{
			return;
		}
	}

	const double PhysicalFrameStartSeconds = FPlatformTime::Seconds();
    {
        Speed::PreparedExecution::FHostCallScope PhysicalOwner(Speed::PreparedExecution::Requested(),*this,*SpeedWorldSubsystem,SimulationWorker.Get(),InputWorkerGeneration,
            uint64(NumFrame()),DeltaTime,SimTime,IsOwnedWorkerExecutionMode()&&!bOwnedWorkerTerminal.Load()&&!bCanonicalPublicationTerminal.Load()&&!bInputOwnerRetired.Load()&&!bStaticWorldPublicationLease);
	SpeedWorldSubsystem->Step(DeltaTime, SimTime, NumFrame());
    }
	RecordStepPerformance(
		(FPlatformTime::Seconds() - PhysicalFrameStartSeconds) * 1000.0);
}

bool ASpeedPhysicsSimulation::QueueSimulationInput(const uint64 ActivationFrame,
	const uint64 TargetStableId, const TConstArrayView<uint8> Payload)
{
	return InputJournal.Append(ActivationFrame, TargetStableId, Payload);
}

void ASpeedPhysicsSimulation::SealSimulationInputs()
{
	InputJournal.Seal();
}

void ASpeedPhysicsSimulation::RecordStepPerformance(
	const double PhysicalFrameMilliseconds)
{
	if (!SpeedWorldSubsystem)
	{
		return;
	}
	const FSpeedStepDiagnostics& Diagnostics =
		SpeedWorldSubsystem->GetLastStepDiagnostics();
	if (Diagnostics.Serial == 0 || Diagnostics.Serial == LastRecordedStepSerial)
	{
		return;
	}
	LastRecordedStepSerial = Diagnostics.Serial;
	LastPhysicalFrameMilliseconds = PhysicalFrameMilliseconds;
	++PerformanceFrameCount;
	PerformanceTotalStepMilliseconds += PhysicalFrameMilliseconds;
	PerformanceMaximumStepMilliseconds = FMath::Max(
		PerformanceMaximumStepMilliseconds, PhysicalFrameMilliseconds);
	const double WarningThresholdMilliseconds =
		Diagnostics.PhysicalDeltaTimeMilliseconds * FMath::Max(
			0.0, static_cast<double>(
				CVarSimulationWarningFrameFraction.GetValueOnAnyThread()));
	PerformanceWarningFrameCount +=
		PhysicalFrameMilliseconds > WarningThresholdMilliseconds ? 1u : 0u;
	PerformanceIterationLimitCount +=
		Diagnostics.bIterationLimitReached ? 1u : 0u;
}

float ASpeedPhysicsSimulation::GetWarningFrameFraction() const
{
	return CVarSimulationWarningFrameFraction.GetValueOnAnyThread();
}

bool ASpeedPhysicsSimulation::StepCanonicalFrame(const FCanonicalFrameContext& Context)
{
    bool resimulation=false;
#if !UE_BUILD_SHIPPING
    resimulation=Context.bResimulationForTesting;
#endif
    if(!Speed::OnePausedWork::BeginStep(this,Context.NumFrame,resimulation,Context.PhysicalDeltaTime,Context.SimTime))return false;
    if (IsInGameThread() && bStaticWorldPublicationLease) return false;
    if (!ValidateAnalyticNumericOwner()) return false;

#if WITH_DEV_AUTOMATION_TESTS
    Speed::OwnedFrameAudit::FCanonicalPhaseScope OwnedPhases(Context.NumFrame);
#endif
#if !UE_BUILD_SHIPPING
    Speed::ParallelTiming::FFrameScope ParallelTimingScope(ParallelFrameTiming, Context.NumFrame, Context.PhysicalDeltaTime);
#endif
	if (bCanonicalPublicationTerminal.Load()) return false;
	if (!SpeedWorldSubsystem)
	{
		SpeedWorldSubsystem = GetSpeedWorldSubsystem(GetWorld());
		if (!SpeedWorldSubsystem)
		{
			return false;
		}
	}

	FString BindingFailure;
	bool bAdmitted = false;
    // Lives through Prepare, Step and EndCanonicalFrame; never a second nested Host.
    TUniquePtr<Speed::PreparedExecution::FHostCallScope> CanonicalHostOwner;
    TUniquePtr<Speed::PreparedExecution::FCanonicalFramePreflight> PreparedPreflight;
    try
    {
        if(Speed::PreparedExecution::Requested())CanonicalHostOwner.Reset(new Speed::PreparedExecution::FHostCallScope(
            Speed::PreparedExecution::Requested(),*this,*SpeedWorldSubsystem,SimulationWorker.Get(),InputWorkerGeneration,
            Context.NumFrame,Context.PhysicalDeltaTime,Context.SimTime,IsOwnedWorkerExecutionMode()&&!bOwnedWorkerTerminal.Load()&&!bCanonicalPublicationTerminal.Load()&&!bInputOwnerRetired.Load()&&!bStaticWorldPublicationLease));
        PreparedPreflight.Reset(new Speed::PreparedExecution::FCanonicalFramePreflight(
            Speed::PreparedExecution::Requested(),*this,*SpeedWorldSubsystem,SimulationWorker.Get(),
            InputWorkerGeneration,Context.NumFrame,Context.PhysicalDeltaTime,Context.SimTime));
        bAdmitted = SpeedWorldSubsystem->BeginCanonicalFrame(BindingFailure);
    }
	catch (...)
	{
		bCanonicalPublicationTerminal.Store(true);
		bOwnedWorkerTerminal.Store(true);
		UE_LOG(LogTemp, Error, TEXT("[CanonicalAdmissionExceptionTerminal] Frame=%llu"), Context.NumFrame);
		return false; // BeginCanonicalFrame sets its active flag only after validation returns.
	}
	if (!bAdmitted)
	{
		UE_LOG(LogTemp, Error, TEXT("[SimulationBindingRejected] frame=%llu %s"),
			Context.NumFrame, *BindingFailure);
		// Both drivers convert a rejected frame to Failed; the owned worker
		// exits at this boundary without advancing time or publishing a pose.
		return false;
	}
#if WITH_DEV_AUTOMATION_TESTS
    OwnedPhases.Mark(Speed::OwnedFrameAudit::ECanonicalEdge::Admission);
#endif
#if !UE_BUILD_SHIPPING
    // All three stage counters are owner TLS; facet totals belong to this world's
    // canonical service. Reset only while the independent timing recorder is on,
    // after admission freezes the backend, before any frame phase dispatch.
    // Default timing mode 0 pays none
    // of this observer's per-frame settings/counter work.
    Speed::ParallelTiming::FFrameWorkEvidence ParallelWork;
    const auto* ParallelTimingWorld = ParallelFrameTiming.IsActive()
        ? SpeedWorldSubsystem->GetStaticCollisionWorld() : nullptr;
    if (ParallelFrameTiming.IsActive())
    {
        const auto Settings = Speed::CaptureFrameParallelSettings();
        ParallelWork.Requested = Settings.RequestedLanes;
        ParallelWork.PolicyLanes = Settings.LaneLimit;
        Speed::ResetFrameParallelStageTotals(Speed::EFrameParallelStage::Camera);
        Speed::ResetFrameParallelStageTotals(Speed::EFrameParallelStage::Prepare);
        Speed::ResetFrameParallelStageTotals(Speed::EFrameParallelStage::QueryBatch);
        if (ParallelTimingWorld) ParallelTimingWorld->ResetFacetParallelTotals();
        ParallelWork.bKnown = true;
    }
#endif
    Speed::Replay::FPhysicalFrameScope ReplayPhysicalOwner(*this,Context.NumFrame);
	bool bAuditStarted = false;
	bool bGlobalPublished = false;
	bool bOutcomeHandled = false;
	ON_SCOPE_EXIT
	{
		if (bAuditStarted) Speed::Analytic::FStaticWorldQueryAudit::EndFrame();
		SpeedWorldSubsystem->EndCanonicalFrame();
	};
	const auto MarkTerminal = [this]()
	{
		if (InputSessionRegistry) InputSessionRegistry->AbortFrame();
		PublishedCompletedWorldInput.store({}, std::memory_order_seq_cst);
		PublishedInputFrame.store(std::shared_ptr<const Speed::Input::V2::FRegistryFrame>{}, std::memory_order_seq_cst);
		bCanonicalPublicationTerminal.Store(true);
		bOwnedWorkerTerminal.Store(true);
	};

#if !UE_BUILD_SHIPPING
        ParallelTimingScope.NextPhase(Speed::ParallelTiming::EFrameCostPhase::Input);
#endif
	const TCHAR* ExceptionStage = TEXT("InputRegistryRead");
	try
	{
		IAMSPEED_FRAME_SCOPE(Initialize);
		const auto InputView = InputSessionRegistry ? InputSessionRegistry->ReadRegistry() : nullptr;
#if WITH_DEV_AUTOMATION_TESTS
    OwnedPhases.Mark(Speed::OwnedFrameAudit::ECanonicalEdge::Registry);
#endif
		const bool bHasInputSessions = InputView && !InputView->Bindings.empty();
		if (bHasInputSessions)
		{
			ExceptionStage = TEXT("ScenarioInputs");
			if (!SpeedWorldSubsystem->StageCanonicalScenarioInputs(Context, *InputSessionRegistry))
				throw std::runtime_error("canonical scenario authoring rejected");
#if WITH_DEV_AUTOMATION_TESTS
    OwnedPhases.Mark(Speed::OwnedFrameAudit::ECanonicalEdge::ScenarioInputs);
#endif
			ExceptionStage = TEXT("InputPrepareFrame");
            bool registryPrepared=false;
            {
                Speed::PreparationPrefix::FEffectScope effect(Speed::PreparationPrefix::EEffect::PrepareInputRegistry);
                registryPrepared=InputSessionRegistry->PrepareFrame(Context.NumFrame);
                effect.Returned(registryPrepared,registryPrepared);
            }
			if (!registryPrepared)
			{
				const auto Failure = InputSessionRegistry->ReadLastFrameFailure();
                UE_LOG(LogTemp, Error, TEXT("[CanonicalInputPollRejected] Frame=%llu Reason=%u Session=%llu Epoch=%llu OwnerPresent=%u OwnerStatusPresent=%u OwnerStatus=%u OwnerPhase=%u PollFailure=%u PollSerial=%llu RegistryVersion=%llu"),
                    Context.NumFrame, unsigned(Failure.Reason), Failure.Session, Failure.Epoch,
                    unsigned(Failure.OwnerPresent), unsigned(Failure.OwnerStatusPresent),
                    unsigned(Failure.OwnerStatus), unsigned(Failure.OwnerPhase), unsigned(Failure.PollFailure),
					Failure.PollSerial, Failure.RegistryVersion);
				throw std::runtime_error("canonical input preparation rejected");
			}
#if WITH_DEV_AUTOMATION_TESTS
    OwnedPhases.Mark(Speed::OwnedFrameAudit::ECanonicalEdge::InputPrepare);
#endif
			ExceptionStage = TEXT("InputInstallAll");
            bool registryInstalled=false;
            {
                Speed::PreparationPrefix::FEffectScope effect(Speed::PreparationPrefix::EEffect::InstallInputRegistry);
                registryInstalled=InputSessionRegistry->InstallAll();
                effect.Returned(registryInstalled,registryInstalled);
            }
			if (!registryInstalled)
			{
				const auto Failure = InputSessionRegistry->ReadLastFrameFailure();
                UE_LOG(LogTemp, Error, TEXT("[CanonicalInputInstallRejected] Frame=%llu Reason=%u Session=%llu Epoch=%llu OwnerPresent=%u OwnerStatusPresent=%u OwnerStatus=%u OwnerPhase=%u PollFailure=%u PollSerial=%llu RegistryVersion=%llu"),
                    Context.NumFrame, unsigned(Failure.Reason), Failure.Session, Failure.Epoch,
                    unsigned(Failure.OwnerPresent), unsigned(Failure.OwnerStatusPresent),
                    unsigned(Failure.OwnerStatus), unsigned(Failure.OwnerPhase), unsigned(Failure.PollFailure),
					Failure.PollSerial, Failure.RegistryVersion);
				throw std::runtime_error("canonical input installation rejected before world binding");
			}
#if WITH_DEV_AUTOMATION_TESTS
    OwnedPhases.Mark(Speed::OwnedFrameAudit::ECanonicalEdge::InputInstall);
#endif
			ExceptionStage = TEXT("InputSnapshotInstall");
			const auto Installed = InputSessionRegistry->ReadInstalled();
            if (!Installed || !SpeedWorldSubsystem->InstallCanonicalInputs(*Installed))
                throw std::runtime_error("canonical input installation rejected");
            {
                Speed::PreparationPrefix::FEffectScope effect(Speed::PreparationPrefix::EEffect::BeginInputRegistry);
                const bool begun=InputSessionRegistry->BeginAll();effect.Returned(begun,begun);
                if(!begun)throw std::runtime_error("canonical input beginning rejected");
            }
		}
#if WITH_DEV_AUTOMATION_TESTS
    OwnedPhases.Mark(Speed::OwnedFrameAudit::ECanonicalEdge::InputSnapshot);
#endif
#if !UE_BUILD_SHIPPING
        const auto WitnessInstalled=bHasInputSessions && InputSessionRegistry?InputSessionRegistry->ReadInstalled():nullptr;
        TUniquePtr<Speed::FFrameAppliedCommandWitnessScope> AppliedCommandWitness;
        TUniquePtr<Speed::FNativeFrameWitnessRecorder> FrameWitness;
        try { FrameWitness=Speed::FNativeFrameWitnessRecorder::Begin(
            Context.NumFrame,Context.PhysicalDeltaTime,WitnessInstalled.get(),*SpeedWorldSubsystem,NativeWitnessCase,!bHasInputSessions); }
        catch(...) { UE_LOG(LogTemp,Error,TEXT("[FrameWitnessBeginUnknown] Frame=%llu"),Context.NumFrame); }
        if(FrameWitness)try { AppliedCommandWitness=MakeUnique<Speed::FFrameAppliedCommandWitnessScope>(
            Context.NumFrame,InputJournal.GetCommandsForFrame(Context.NumFrame).Num());
            SpeedWorldSubsystem->DeclareExpectedFrameAnalogWitnessForTesting(); }
        catch(...) { UE_LOG(LogTemp,Error,TEXT("[FrameAppliedCommandBeginUnknown] Frame=%llu"),Context.NumFrame); }
#endif
		ExceptionStage = TEXT("StaticWorldAudit");
        {
            Speed::PreparationPrefix::FEffectScope effect(Speed::PreparationPrefix::EEffect::BeginStaticAudit);
		Speed::Analytic::FStaticWorldQueryAudit::BeginFrame(
			Context.NumFrame, SpeedWorldSubsystem->GetAnalyticWorldData(),
			SpeedWorldSubsystem);
            bAuditStarted=true; // cleanup must survive a refused After observer
            effect.Returned();
        }
#if WITH_DEV_AUTOMATION_TESTS
    OwnedPhases.Mark(Speed::OwnedFrameAudit::ECanonicalEdge::StaticAudit);
#endif
		ExceptionStage = TEXT("SimulationInputJournal");
        bool journalApplied=true;
        {
            Speed::PreparationPrefix::FEffectScope effect(Speed::PreparationPrefix::EEffect::ApplyInputJournal);
            journalApplied=!InputJournal.IsSealed() || SpeedWorldSubsystem->ApplySimulationInputs(Context.NumFrame,InputJournal);
            effect.Returned(journalApplied,journalApplied);
        }
		if (!journalApplied)
		{
			if (!bInputConsumptionErrorReported)
			{
				bInputConsumptionErrorReported = true;
				UE_LOG(LogTemp, Error,
					TEXT("[SimulationInputRejected] Frame=%llu JournalHash=%016llX"),
					Context.NumFrame, InputJournal.StableHash());
			}
			SpeedWorldSubsystem->AbortCanonicalFrame(Context.NumFrame, ECanonicalFrameAbortReason::PreparationFailed);
			bOutcomeHandled = true;
			MarkTerminal();
			return false;
		}
#if WITH_DEV_AUTOMATION_TESTS
    OwnedPhases.Mark(Speed::OwnedFrameAudit::ECanonicalEdge::InputJournal);
#endif
		ExceptionStage = TEXT("CanonicalInputPrepare");
		if (!SpeedWorldSubsystem->PrepareCanonicalInputs(Context))
		{
			SpeedWorldSubsystem->AbortCanonicalFrame(Context.NumFrame, ECanonicalFrameAbortReason::PreparationFailed);
			bOutcomeHandled = true;
			MarkTerminal();
			UE_LOG(LogTemp, Error, TEXT("[CanonicalInputPreparationRejected] Frame=%llu"), Context.NumFrame);
			return false;
		}
#if WITH_DEV_AUTOMATION_TESTS
    OwnedPhases.Mark(Speed::OwnedFrameAudit::ECanonicalEdge::CanonicalInputs);
#endif
        {
            Speed::PreparationPrefix::FEffectScope effect(Speed::PreparationPrefix::EEffect::ClearInputError);
            bInputConsumptionErrorReported=false;effect.Returned();
        }

#if !UE_BUILD_SHIPPING
        ParallelTimingScope.NextPhase(Speed::ParallelTiming::EFrameCostPhase::Prepare);
#endif
		IAMSPEED_FRAME_PHASE(Prepare);
		ExceptionStage = TEXT("WorldPrepare");
        {
            Speed::PreparationPrefix::FEffectScope effect(Speed::PreparationPrefix::EEffect::PrepareWorld);
#if !UE_BUILD_SHIPPING
        {
            Speed::FCanonicalEpisodeTestingScope PrepareOwnerScope(Context);
            SpeedWorldSubsystem->PrepareCanonicalFrame(Context);
        }
#else
        SpeedWorldSubsystem->PrepareCanonicalFrame(Context);
#endif
            effect.Returned();
        }
#if WITH_DEV_AUTOMATION_TESTS
    OwnedPhases.Mark(Speed::OwnedFrameAudit::ECanonicalEdge::WorldPrepare);
#endif

        if((Speed::OnePausedWork::IsPostPrepareInitialPositionFor(this)||Speed::OnePausedWork::IsTerminalStepSyncFor(this)||Speed::OnePausedWork::IsTerminalSphereRsCaptureFor(this)||Speed::OnePausedWork::IsTerminalConsumedPrefixAdvanceFor(this)||Speed::OnePausedWork::IsTerminalRsContactAdvanceFor(this)||Speed::OnePausedWork::IsTerminalRsResponseFor(this)||Speed::OnePausedWork::IsTerminalRsResponseSyncFor(this)||Speed::OnePausedWork::IsTerminalPostSyncRsCaptureFor(this))&&
            !SpeedWorldSubsystem->DispatchPostPrepareInitialPosition(*this))
            throw std::runtime_error("Native post-Prepare position action refused before retirement");
        if (Speed::OnePausedWork::IsPreparationPrefixFor(this)||Speed::OnePausedWork::IsPostPrepareInitialPositionFor(this))
        {
            if (!SpeedWorldSubsystem->RetirePreparationInputTransactions(Context)||
                !Speed::PreparationPrefix::FCursor::Finish(*this))
                throw std::runtime_error("Native preparation prefix did not reach its closed all-owner stop");
            // No snapshot, hash, presentation, CompleteAll, Step or ++N. Existing
            // lexical cleanup ends the audit/canonical-active flag; real worker
            // retirement later cancels input tokens without physical rollback.
            bOutcomeHandled=true;
            return false;
        }
#if !UE_BUILD_SHIPPING
        ParallelTimingScope.NextPhase(Speed::ParallelTiming::EFrameCostPhase::WorldStep);
#endif
		IAMSPEED_FRAME_PHASE(Core);
		ExceptionStage = TEXT("WorldStep");
        const bool terminalAdvance=Speed::OnePausedWork::IsTerminalConsumedPrefixAdvanceFor(this);
        const bool terminalContactAdvance=Speed::OnePausedWork::IsTerminalRsContactAdvanceFor(this);
        const bool terminalResponse=Speed::OnePausedWork::IsTerminalRsResponseFor(this);
        const bool terminalResponseSync=Speed::OnePausedWork::IsTerminalRsResponseSyncFor(this);
        const bool terminalPostSyncRsCapture=Speed::OnePausedWork::IsTerminalPostSyncRsCaptureFor(this);
        const bool terminalCapture=Speed::OnePausedWork::IsTerminalSphereRsCaptureFor(this);
        const bool terminalSync=Speed::OnePausedWork::IsTerminalStepSyncFor(this);
        if(!(terminalPostSyncRsCapture?Speed::OnePausedWork::BeginTerminalPostSyncRsCapture(this,SpeedWorldSubsystem,
            SpeedWorldSubsystem->GetStaticCollisionWorld(),Context.NumFrame,Context.PhysicalDeltaTime,Context.SimTime):
            terminalResponseSync?Speed::OnePausedWork::BeginTerminalRsResponseSync(this,SpeedWorldSubsystem,
            SpeedWorldSubsystem->GetStaticCollisionWorld(),Context.NumFrame,Context.PhysicalDeltaTime,Context.SimTime):
            terminalResponse?Speed::OnePausedWork::BeginTerminalRsResponse(this,SpeedWorldSubsystem,
            SpeedWorldSubsystem->GetStaticCollisionWorld(),Context.NumFrame,Context.PhysicalDeltaTime,Context.SimTime):
            terminalContactAdvance?Speed::OnePausedWork::BeginTerminalRsContactAdvance(this,SpeedWorldSubsystem,
            SpeedWorldSubsystem->GetStaticCollisionWorld(),Context.NumFrame,Context.PhysicalDeltaTime,Context.SimTime):
            terminalAdvance?Speed::OnePausedWork::BeginTerminalConsumedPrefixAdvance(this,SpeedWorldSubsystem,
            SpeedWorldSubsystem->GetStaticCollisionWorld(),Context.NumFrame,Context.PhysicalDeltaTime,Context.SimTime):
            terminalCapture?Speed::OnePausedWork::BeginTerminalSphereRsCapture(this,SpeedWorldSubsystem,
            SpeedWorldSubsystem->GetStaticCollisionWorld(),Context.NumFrame,Context.PhysicalDeltaTime,Context.SimTime):
            terminalSync?Speed::OnePausedWork::BeginTerminalStepSync(this,SpeedWorldSubsystem,
            SpeedWorldSubsystem->GetStaticCollisionWorld(),Context.NumFrame,Context.PhysicalDeltaTime,Context.SimTime):
            Speed::OnePausedWork::BeginWorldStep(this,SpeedWorldSubsystem,
            SpeedWorldSubsystem->GetStaticCollisionWorld(),Context.NumFrame,Context.PhysicalDeltaTime,Context.SimTime)))
            throw std::runtime_error("Single canonical physical Step identity refused");
#if WITH_DEV_AUTOMATION_TESTS
    const uint64 OwnedDiagnosticSerialBefore = OwnedPhases.CapturesPhysicalDetail()
        ? SpeedWorldSubsystem->GetLastStepDiagnostics().Serial : 0;
#endif
		SpeedWorldSubsystem->Step(
			Context.PhysicalDeltaTime,
			Context.SimTime,
			static_cast<unsigned int>(Context.NumFrame));
#if WITH_DEV_AUTOMATION_TESTS
    OwnedPhases.Mark(Speed::OwnedFrameAudit::ECanonicalEdge::WorldStep);
    if (OwnedPhases.CapturesPhysicalDetail())
        OwnedPhases.CopyPhysicalDetail(SpeedWorldSubsystem->GetLastStepDiagnostics(), OwnedDiagnosticSerialBefore);
#endif

        if(terminalSync||terminalCapture||terminalAdvance||terminalContactAdvance||terminalResponse||terminalResponseSync||terminalPostSyncRsCapture)
        {
            if(!SpeedWorldSubsystem->RetirePreparationInputTransactions(Context)||!Speed::PreparationPrefix::FCursor::Finish(*this))
                throw std::runtime_error("Native terminal Step sync failed its all-owner retirement");
            bOutcomeHandled=true;return false;
        }
#if !UE_BUILD_SHIPPING
        ParallelTimingScope.NextPhase(Speed::ParallelTiming::EFrameCostPhase::Snapshot);
#endif
		IAMSPEED_FRAME_PHASE(Snapshot);
		ExceptionStage = TEXT("SnapshotCapture");
        PreparedPreflight->RequireClosedForSnapshot();
		FSimulationSnapshot Snapshot = SpeedWorldSubsystem->CaptureSimulationSnapshot(
			Context.NumFrame, InputJournal.StableHash(), bPublishPresentation);
#if WITH_DEV_AUTOMATION_TESTS
    OwnedPhases.Mark(Speed::OwnedFrameAudit::ECanonicalEdge::Snapshot);
#endif

#if !UE_BUILD_SHIPPING
        ParallelTimingScope.NextPhase(Speed::ParallelTiming::EFrameCostPhase::Presentation);
#endif
		ExceptionStage = TEXT("PresentationProducerList");
		TArray<TSharedRef<ISimulationPresentationProducer, ESPMode::ThreadSafe>> Producers;
		{
			FScopeLock Lock(&PresentationProducerMutex);
			Producers = PresentationProducers;
		}
#if WITH_DEV_AUTOMATION_TESTS
    OwnedPhases.Mark(Speed::OwnedFrameAudit::ECanonicalEdge::ProducerList);
#endif
		const Speed::FFrameParallelSettings ParallelPresentationSettings = Speed::CaptureFrameParallelSettings();
		ExceptionStage = TEXT("PresentationProduce");
        Producers.RemoveAll([this](const auto& Producer)
        { return Producer->IsCameraPresentation() && !AllowsCameraPresentation(); });
#if WITH_DEV_AUTOMATION_TESTS
        TOptional<Speed::OwnedFrameAudit::FCameraScope> OwnedCamera;
        const bool bInputComplete = Speed::FrameParallel::ProduceFramePresentationWithOwnerValidationAndObservation(
            Snapshot, Producers, ParallelPresentationSettings.bParallelCamera ? ParallelPresentationSettings.LaneLimit : 1,
            bHasInputSessions, [&] { return !bHasInputSessions || InputSessionRegistry->ValidateComplete(); },
            [&](const ISimulationPresentationProducer& Producer)
            {
                Speed::Testing::ObservePresentationDispatch(&Producer, Snapshot.NumFrame);
                OwnedCamera.Emplace(Snapshot.NumFrame, Producer.IsCameraPresentation());
            },
            [&](bool bProducerReturned)
            {
                if (bProducerReturned) OwnedCamera->MarkReturned();
                OwnedCamera.Reset();
            });
        OwnedPhases.Mark(Speed::OwnedFrameAudit::ECanonicalEdge::Presentation);
#else
        const bool bInputComplete = Speed::FrameParallel::ProduceFramePresentationWithOwnerValidation(
            Snapshot, Producers, ParallelPresentationSettings.bParallelCamera ? ParallelPresentationSettings.LaneLimit : 1,
            bHasInputSessions, [&] { return !bHasInputSessions || InputSessionRegistry->ValidateComplete(); });
#endif
		ExceptionStage = TEXT("InputValidateComplete");
		if (!bInputComplete)
			throw std::runtime_error("canonical input completion rejected");
#if WITH_DEV_AUTOMATION_TESTS
    OwnedPhases.Mark(Speed::OwnedFrameAudit::ECanonicalEdge::InputValidate);
#endif

#if !UE_BUILD_SHIPPING
        ParallelTimingScope.NextPhase(Speed::ParallelTiming::EFrameCostPhase::Publish);
#endif
        // Allocate/copy before either irreversible commit; retain this exact endpoint.
        std::shared_ptr<Speed::FCompletedSimulationPublication> CompletedDraft;
        if (bPublishCompletedWorldInput)
        {
            ExceptionStage = TEXT("CompletedWorldInputPrepare");
            const auto Installed = bHasInputSessions ? InputSessionRegistry->ReadInstalled() : nullptr;
            // ReadRegistry produces an observation copy. PrepareFrame retains its
            // own registry identity, which must accompany the installed endpoint.
            const auto CapturedRegistry = Installed ? Installed->Registry : InputView;
            if (!Speed::FCompletedSimulationPublication::CanPrepare(
                Snapshot, CompletedPublicationSource, CapturedRegistry, Installed))
                throw std::runtime_error("completed world/input preparation rejected");
            CompletedDraft.reset(new Speed::FCompletedSimulationPublication(
                Snapshot, CompletedPublicationSource, CapturedRegistry, Installed));
        }
        const auto PublishCompletedWorldInput =
            [&](const std::shared_ptr<const Speed::Input::V2::FRegistryFrame>& Completed)
        {
            if (!CompletedDraft) return true;
            if (!CompletedDraft->MatchesCompleted(Completed)
                || !CompletedDraft->BindPublicationSerial(SnapshotBuffer.PublishedSerial())) return false;
            // No allocation or callbacks here. Consume the only mutable alias.
            std::shared_ptr<const Speed::FCompletedSimulationPublication> Sealed = std::move(CompletedDraft);
            PublishedCompletedWorldInput.store(std::move(Sealed), std::memory_order_seq_cst);
            return true;
        };
		IAMSPEED_FRAME_PHASE(Publish);
		ExceptionStage = TEXT("CanonicalPublish");
		const ECanonicalPublicationResult Publication = SpeedWorldSubsystem->PublishCanonicalFrame(Context.NumFrame, [&]()
		{
			// One simulation owner. Avoid serial wrap before touching the inactive slot.
			if (SnapshotBuffer.PublishedSerial() == MAX_uint64) return false;
#if !UE_BUILD_SHIPPING
            // Replay publications do not represent paced real-time starts.
            return Context.bResimulationForTesting ? SnapshotBuffer.Publish(Snapshot) :
                SnapshotBuffer.PublishWithCadenceTiming(Snapshot, GetCanonicalPublicationTiming(Context));
#else
            return SnapshotBuffer.Publish(Snapshot);
#endif
		});
#if WITH_DEV_AUTOMATION_TESTS
    OwnedPhases.Mark(Speed::OwnedFrameAudit::ECanonicalEdge::Publication);
#endif
		bOutcomeHandled = true;
		bGlobalPublished = Publication == ECanonicalPublicationResult::Completed || Publication == ECanonicalPublicationResult::CommitInvariantFailed;
		if (Publication != ECanonicalPublicationResult::Completed)
		{
			MarkTerminal();
			UE_LOG(LogTemp, Error, TEXT("[CanonicalPublicationTerminal] Frame=%llu Result=%u GlobalPublished=%d"),
				Context.NumFrame, static_cast<uint32>(Publication), bGlobalPublished ? 1 : 0);
			return false;
		}

#if !UE_BUILD_SHIPPING
        ParallelTimingScope.NextPhase(Speed::ParallelTiming::EFrameCostPhase::PostCommit);
#endif
		if (bHasInputSessions)
		{
			ExceptionStage = TEXT("InputComplete");
			if (!InputSessionRegistry->CompleteAll())
			{
				MarkTerminal();
				return false;
			}
			const auto Completed = InputSessionRegistry->ReadLatest();
			if (!Completed || Completed->Frame != Context.NumFrame || !PublishCompletedWorldInput(Completed))
			{
				MarkTerminal();
				return false;
			}
			PublishedInputFrame.store(Completed, std::memory_order_seq_cst);
			for (const auto& Input : Completed->Inputs)
			{
				const auto Observation = InputObservations.find(Input.Session);
				if (Observation != InputObservations.end())
				{
					try { if (!Observation->second->Publish(Input.Snapshot)) Observation->second->Deactivate(); }
					catch (...) { UE_LOG(LogTemp, Error, TEXT("[InputObservationPublicationFailed] Frame=%llu"), Context.NumFrame); }
				}
			}
		}
        if (!bHasInputSessions && !PublishCompletedWorldInput({}))
        {
            MarkTerminal();
            return false;
        }
#if WITH_DEV_AUTOMATION_TESTS
    OwnedPhases.Mark(Speed::OwnedFrameAudit::ECanonicalEdge::InputComplete);
#endif
		// V2 authoritative commit is finished before any optional presentation work.
		ExceptionStage = TEXT("PostCommitNotify");
		SpeedWorldSubsystem->NotifyCanonicalFramePublished(Context.NumFrame);
#if !UE_BUILD_SHIPPING
        static const auto* OrdinaryContactTraceFrame = IConsoleManager::Get().FindTConsoleVariableDataInt(
            TEXT("p.IAmSpeed.Collision.SolverEventTraceFrame"));
        const int32 TracedFrame = OrdinaryContactTraceFrame ? OrdinaryContactTraceFrame->GetValueOnAnyThread() : -1;
        if (((TracedFrame >= 0 && Context.NumFrame == static_cast<uint64>(TracedFrame)) ||
             Speed::Analytic::FStaticWorldQueryAudit::IsQualificationContactTraceFrame(Context.NumFrame)) &&
            !Context.bResimulationForTesting && Speed::Analytic::FStaticWorldQueryAudit::IsSurfaceAnalyticBackend())
        {
            const auto* Provider = SpeedWorldSubsystem->GetStaticCollisionWorld();
            const uint64 SceneGeneration = Provider ? Provider->GetImmutableQueryIdentity1075().Generation : 0;
            const uint64 Serial = SnapshotBuffer.PublishedSerial();
            for (const auto& Body : Snapshot.PresentationBodies)
            {
                UE_LOG(LogTemp, Display,
                    TEXT("[OrdinaryPhysicsPublication] Frame=%llu SceneGeneration=%llu Body=%llu Serial=%llu StateHash=%016llx"),
                    Context.NumFrame, SceneGeneration, Body.StableId, Serial, Snapshot.StateHash);
            }
        }
#endif
		// Optional replay observes only the successful full commit. Failure retires
		// recording, never the live physics step or its canonical hash.
		ServiceReplayCapture(Snapshot);
#if WITH_DEV_AUTOMATION_TESTS
    OwnedPhases.Mark(Speed::OwnedFrameAudit::ECanonicalEdge::PostCommit);
#endif
		FCameraCanonicalSample CameraSample;
		ExceptionStage = TEXT("CameraSample");
#if WITH_DEV_AUTOMATION_TESTS
        const bool bHasCanonicalCameraSample = AllowsCameraPresentation() &&
            (Speed::Testing::ObserveCanonicalCameraSample(Snapshot.NumFrame), BuildCanonicalCameraSample(Snapshot, CameraSample));
#else
        const bool bHasCanonicalCameraSample = AllowsCameraPresentation() && BuildCanonicalCameraSample(Snapshot, CameraSample);
#endif
        if (bHasCanonicalCameraSample)
		{
			CameraSample.NumFrame = Snapshot.NumFrame;
			CameraSample.PublicationSerial = SnapshotBuffer.PublishedSerial();
			CameraSample.StateHash = Snapshot.StateHash;
			CameraSample.InputJournalHash = Snapshot.InputJournalHash;
			CameraSampleBuffer.Publish(CameraSample);
		}
#if WITH_DEV_AUTOMATION_TESTS
    OwnedPhases.Mark(Speed::OwnedFrameAudit::ECanonicalEdge::CameraSample);
#endif
#if !UE_BUILD_SHIPPING
        if(FrameWitness)try
        {
            // Every output payload and internal camera checkpoint stays in its
            // publication order; no float rounding, sorting or hash-only oracle.
            TArray<uint8> WitnessCamera;using WitnessBytes=Speed::FEpisodeStateBytesForTesting;
            WitnessBytes::Add(WitnessCamera,uint32(1));WitnessBytes::Add(WitnessCamera,Snapshot.NumFrame);
            WitnessBytes::Add(WitnessCamera,Snapshot.PresentationOutputs.Num());
            for(const auto& Output:Snapshot.PresentationOutputs)
            {
                WitnessBytes::Add(WitnessCamera,Output.OwnerStableId);WitnessBytes::Add(WitnessCamera,Output.Channel);
                WitnessBytes::Add(WitnessCamera,Output.NumFrame);WitnessBytes::Add(WitnessCamera,Output.PublicationSerial);
                WitnessBytes::Add(WitnessCamera,Output.Payload.Num());WitnessCamera.Append(Output.Payload);
                WitnessBytes::Add(WitnessCamera,Output.StatePayload.Num());WitnessCamera.Append(Output.StatePayload);
            }
            WitnessBytes::Add(WitnessCamera,bHasCanonicalCameraSample);
            if(bHasCanonicalCameraSample)
            {
                WitnessBytes::Add(WitnessCamera,CameraSample.NumFrame);WitnessBytes::Add(WitnessCamera,CameraSample.PublicationSerial);
                WitnessBytes::Add(WitnessCamera,CameraSample.StateHash);WitnessBytes::Add(WitnessCamera,CameraSample.InputJournalHash);
                WitnessBytes::Add(WitnessCamera,CameraSample.CarBodyStableId);WitnessBytes::Add(WitnessCamera,CameraSample.BallBodyStableId);
                WitnessBytes::Add(WitnessCamera,CameraSample.EvaluatorKind);
                WitnessBytes::Add(WitnessCamera,CameraSample.Position.X);WitnessBytes::Add(WitnessCamera,CameraSample.Position.Y);WitnessBytes::Add(WitnessCamera,CameraSample.Position.Z);
                WitnessBytes::Add(WitnessCamera,CameraSample.Rotation.X);WitnessBytes::Add(WitnessCamera,CameraSample.Rotation.Y);WitnessBytes::Add(WitnessCamera,CameraSample.Rotation.Z);WitnessBytes::Add(WitnessCamera,CameraSample.Rotation.W);
            }
            if(AppliedCommandWitness && !AppliedCommandWitness->SaveCommittedFrame())
                UE_LOG(LogTemp,Error,TEXT("[FrameAppliedCommandSaveUnknown] Frame=%llu"),Context.NumFrame);
            const auto WitnessCompleted=bHasInputSessions && InputSessionRegistry?InputSessionRegistry->ReadLatest():nullptr;
            FrameWitness->Finish(WitnessCompleted.get(),Snapshot.Payload,
                SpeedWorldSubsystem->GetLastStepDiagnostics().ResolvedEventCount,WitnessCamera,true,!bHasInputSessions);
        }
        catch(...) { UE_LOG(LogTemp,Error,TEXT("[FrameWitnessFinishUnknown] Frame=%llu"),Context.NumFrame); }
#endif

#if !UE_BUILD_SHIPPING
        ParallelTimingScope.NextPhase(Speed::ParallelTiming::EFrameCostPhase::Finalize);
#endif
		IAMSPEED_FRAME_PHASE(Journal);
		ExceptionStage = TEXT("FrameHash");
		FrameHashes.Append(Context.NumFrame, Snapshot.StateHash);
#if WITH_DEV_AUTOMATION_TESTS
    OwnedPhases.Mark(Speed::OwnedFrameAudit::ECanonicalEdge::FrameHash);
#endif
#if !UE_BUILD_SHIPPING
        if (ParallelWork.bKnown)
        {
            // Prepare, world queries and presentation have all joined before
            // reaching committed publication. The owner itself is one actual
            // participant even on an empty/plane-only frame with no dispatch.
            ParallelWork.LogicalMax = 1;
            ParallelWork.NativeMax = 1;
            for (const auto Stage : {Speed::EFrameParallelStage::Camera,
                 Speed::EFrameParallelStage::Prepare, Speed::EFrameParallelStage::QueryBatch})
            {
                const auto Totals = Speed::GetFrameParallelStageTotals(Stage);
                ParallelWork.LogicalMax = FMath::Max(ParallelWork.LogicalMax, Totals.MaximumLogicalLanes);
                ParallelWork.NativeMax = FMath::Max(ParallelWork.NativeMax, Totals.MaximumNativeThreads);
                ParallelWork.FallbackMask |= static_cast<uint32>(Totals.FallbackMask);
                if (Stage == Speed::EFrameParallelStage::Prepare)
                { ParallelWork.ParallelPrepareCalls = Totals.ParallelInvocations; ParallelWork.PrepareNativeMax = Totals.MaximumNativeThreads; }
                else if (Stage == Speed::EFrameParallelStage::QueryBatch)
                { ParallelWork.ParallelQueryCalls = Totals.ParallelInvocations; ParallelWork.QueryNativeMax = Totals.MaximumNativeThreads; }
                else if (Stage == Speed::EFrameParallelStage::Camera)
                { ParallelWork.ParallelCameraCalls = Totals.ParallelInvocations; ParallelWork.CameraNativeMax = Totals.MaximumNativeThreads; }

            }
            if (ParallelTimingWorld)
            {
                const auto Facets = ParallelTimingWorld->GetFacetParallelTotals();
                ParallelWork.FacetProviders = Facets.Providers;
                ParallelWork.ParallelFacetProviders = Facets.ParallelProviders;
                ParallelWork.EvaluatedFacets = Facets.EvaluatedFacets;
                ParallelWork.FacetNativeMax = Facets.MaximumNativeThreads;

                ParallelWork.LogicalMax = FMath::Max(ParallelWork.LogicalMax, Facets.MaximumLogicalLanes);
                ParallelWork.NativeMax = FMath::Max(ParallelWork.NativeMax, Facets.MaximumNativeThreads);
                if (Facets.InstrumentationFallbackProviders) ParallelWork.FallbackMask |= 1u << 16;
                if (Facets.NestedFallbackProviders) ParallelWork.FallbackMask |= 1u << 17;
                if (Facets.CapacityFallbackProviders) ParallelWork.FallbackMask |= 1u << 18;
                if (Facets.NumericFallbackProviders) ParallelWork.FallbackMask |= 1u << 19;
            }
            const auto EndSettings = Speed::CaptureFrameParallelSettings();
            ParallelWork.bKnown = EndSettings.RequestedLanes == ParallelWork.Requested &&
                EndSettings.LaneLimit == ParallelWork.PolicyLanes &&
                ParallelWork.LogicalMax <= ParallelWork.PolicyLanes;
        }
        ParallelTimingScope.MarkCommitted(Snapshot.StateHash, Snapshot.InputJournalHash, ParallelWork);
#endif
		IAMSPEED_FRAME_PHASE(Finalize);
		return true;
	}
	catch (const Speed::Analytic::FAnalyticNumericUnknown& Unknown)
    {
        MarkTerminal();
        const auto& E=Unknown.Evidence;
        UE_LOG(LogTemp, Error, TEXT("[CanonicalFrameNumericUnknown] Frame=%llu GlobalPublished=%d Stage=%s Reason=%u MXCSR=%08X FCW=%04X MutationPossible=1 Terminal=1"),
            Context.NumFrame,bGlobalPublished ? 1 : 0,ExceptionStage,uint32(E.Failure),E.VectorControl,E.ScalarControl);
        if (!bGlobalPublished && !bOutcomeHandled)
            SpeedWorldSubsystem->AbortCanonicalFrame(Context.NumFrame, ECanonicalFrameAbortReason::PreparationFailed);
        return false; // Teardown notification is not a kinematic rollback.
    }
	catch (const std::exception& Exception)
	{
		if (!bGlobalPublished && !bOutcomeHandled)
			SpeedWorldSubsystem->AbortCanonicalFrame(Context.NumFrame, ECanonicalFrameAbortReason::PreparationFailed);
		MarkTerminal();
		UE_LOG(LogTemp, Error, TEXT("[CanonicalFrameExceptionTerminal] Frame=%llu GlobalPublished=%d Stage=%s Type=std Message=%s"),
			Context.NumFrame, bGlobalPublished ? 1 : 0, ExceptionStage, UTF8_TO_TCHAR(Exception.what()));
		return false; // No exception escapes into the UE worker/callback boundary.
	}
	catch (...)
	{
		if (!bGlobalPublished && !bOutcomeHandled)
			SpeedWorldSubsystem->AbortCanonicalFrame(Context.NumFrame, ECanonicalFrameAbortReason::PreparationFailed);
		MarkTerminal();
		UE_LOG(LogTemp, Error, TEXT("[CanonicalFrameExceptionTerminal] Frame=%llu GlobalPublished=%d Stage=%s Type=unknown"),
			Context.NumFrame, bGlobalPublished ? 1 : 0, ExceptionStage);
		return false; // No exception escapes into the UE worker/callback boundary.
	}
}

bool ASpeedPhysicsSimulation::RequestRollbackAndResimulation(
	const FSimulationSnapshot& Snapshot,
	const uint64 TargetFrameInclusive)
{
	if (!Speed::CanonicalFrameDriver::IsEnabled() || !InputJournal.IsSealed() ||
		Snapshot.InputJournalHash != InputJournal.StableHash() ||
		Snapshot.NumFrame > TargetFrameInclusive ||
		TargetFrameInclusive >= TNumericLimits<unsigned int>::Max())
	{
		return false;
	}
	// Registry replay is not a world/actor restore transaction yet. Reject
	// before queuing instead of rewinding physics underneath live owners.
	const auto InputView = ReadInputRegistryView();
	if (InputView && !InputView->Bindings.empty()) return false;
	FScopeLock Lock(&RollbackRequestMutex);
	if (PendingRollbackRequest.IsSet())
	{
		return false;
	}
	PendingRollbackRequest.Emplace(FPendingRollbackRequest{
		Snapshot, TargetFrameInclusive });
	return true;
}

bool ASpeedPhysicsSimulation::ProcessPendingRollbackRequest()
{
	TOptional<FPendingRollbackRequest> Request;
	{
		FScopeLock Lock(&RollbackRequestMutex);
		if (!PendingRollbackRequest.IsSet())
		{
			return true;
		}
#if !UE_BUILD_SHIPPING
        IndependentFastCoverage276.RollbackDequeued();
#endif
		Request = MoveTemp(PendingRollbackRequest);
		PendingRollbackRequest.Reset();
	}

	// A legacy restore has no completed input/world transaction. Retained reads
	// stay historical; future reads are unavailable before any physical mutation.
	PublishedCompletedWorldInput.store({}, std::memory_order_seq_cst);
	// Recheck on the worker: an input binding may have arrived since admission.
	const auto InputView = ReadInputRegistryView();
	if (InputView && !InputView->Bindings.empty()) return false;
	TArray<TSharedRef<ISimulationPresentationProducer, ESPMode::ThreadSafe>> RestoreProducers;
	{
		FScopeLock Lock(&PresentationProducerMutex);
		RestoreProducers = PresentationProducers;
	}
	TArray<FSimulationPresentationOutput> RestoreOutputs;
	if (Request->Snapshot.PresentationOutputs.Num() != RestoreProducers.Num()) return false;
	for (const auto& Producer : RestoreProducers)
	{
		const FSimulationPresentationOutput* Output = Request->Snapshot.PresentationOutputs.FindByPredicate(
			[&](const FSimulationPresentationOutput& Candidate)
			{
				return Candidate.OwnerStableId == Producer->OwnerStableId() && Candidate.Channel == Producer->Channel();
			});
		if (!Output || Output->NumFrame != Request->Snapshot.NumFrame ||
			!Producer->CanRestore(*Output, Request->TargetFrameInclusive)) return false;
		RestoreOutputs.Add(*Output);
	}
	if (!SpeedWorldSubsystem ||
		!SpeedWorldSubsystem->RestoreSimulationSnapshot(
			Request->Snapshot, InputJournal.StableHash()))
	{
		UE_LOG(LogTemp, Error,
			TEXT("[SimulationSnapshotRestoreRejected] Frame=%llu TargetFrame=%llu"),
			Request->Snapshot.NumFrame, Request->TargetFrameInclusive);
		return false;
	}

	Request->Snapshot.PresentationOutputs.Reset();
	for (int32 Index = 0; Index < RestoreProducers.Num(); ++Index)
	{
		FSimulationPresentationOutput Restored;
		RestoreProducers[Index]->RestoreValidated(RestoreOutputs[Index], Request->TargetFrameInclusive, Restored);
		Request->Snapshot.PresentationOutputs.Add(MoveTemp(Restored));
	}
	FrameHashes.RemoveFrom(Request->Snapshot.NumFrame);
	if (!FrameHashes.Append(
			Request->Snapshot.NumFrame, Request->Snapshot.StateHash) ||
		!SnapshotBuffer.Publish(Request->Snapshot))
	{
		UE_LOG(LogTemp, Error,
			TEXT("[SimulationSnapshotRepublishFailed] Frame=%llu"),
			Request->Snapshot.NumFrame);
		return false;
	}

	CanonicalNumFrame = Request->Snapshot.NumFrame + 1u;
	_NumFrame = static_cast<unsigned int>(CanonicalNumFrame);
	const uint64 ReplayFrameCount =
		Request->TargetFrameInclusive - Request->Snapshot.NumFrame;
	for (uint64 ReplayIndex = 0; ReplayIndex < ReplayFrameCount; ++ReplayIndex)
	{
		const double PhysicalFrameStartSeconds = FPlatformTime::Seconds();
        FCanonicalFrameContext ReplayContext(CanonicalNumFrame);
#if !UE_BUILD_SHIPPING
        ReplayContext.bResimulationForTesting=true;
#endif
#if WITH_DEV_AUTOMATION_TESTS
        const auto OwnedReplayStep = Speed::OwnedFrameAudit::BeginStep(ReplayContext.NumFrame, true);
#endif
        const bool ReplayStepSucceeded = StepCanonicalFrame(ReplayContext);
#if WITH_DEV_AUTOMATION_TESTS
        Speed::OwnedFrameAudit::EndStep(OwnedReplayStep, true, ReplayStepSucceeded);
#endif
        if (!ReplayStepSucceeded)
		{
			UE_LOG(LogTemp, Error,
				TEXT("[SimulationResimulationFailed] Frame=%llu TargetFrame=%llu"),
				CanonicalNumFrame, Request->TargetFrameInclusive);
			return false;
		}
		RecordStepPerformance(
			(FPlatformTime::Seconds() - PhysicalFrameStartSeconds) * 1000.0);
		++CanonicalNumFrame;
		_NumFrame = static_cast<unsigned int>(CanonicalNumFrame);
	}
	OnCanonicalTimelineRestored();
	return true;
}


void ASpeedPhysicsSimulation::SetReplaySink(TSharedPtr<ISimulationReplaySink, ESPMode::ThreadSafe> Sink)
{
    check(IsInGameThread());
    {
        FScopeLock Lock(&ReplaySinkMutex);
        Swap(ReplaySink,Sink);
        bReplayCaptureRequested.Store(ReplaySink.IsValid());
    }
    // Any old sink is released outside the publication lock, after in-flight refs.
}

bool ASpeedPhysicsSimulation::TryReplaceReplaySink(const ISimulationReplaySink* Expected,
    TSharedPtr<ISimulationReplaySink, ESPMode::ThreadSafe> Replacement)
{
    check(IsInGameThread());
    {
        FScopeLock Lock(&ReplaySinkMutex);
        if(ReplaySink.Get()!=Expected)return false;
        Swap(ReplaySink,Replacement);
        bReplayCaptureRequested.Store(ReplaySink.IsValid());
    }
    return true; // Replacement retains the old sink until after lock release.
}

void ASpeedPhysicsSimulation::ServiceReplayCapture(const FSimulationSnapshot& Snapshot) noexcept
{
    if (!bReplayCaptureRequested.Load()) return;
    TSharedPtr<ISimulationReplaySink, ESPMode::ThreadSafe> Sink;
    {
        FScopeLock Lock(&ReplaySinkMutex); Sink=ReplaySink;
    }
    if (!Sink.IsValid() || !Sink->IsCapturing()) return;
    uint64 UnsupportedStableId=0;
    try
    {
        FSimulationReplayFrame Frame;
        if (SpeedWorldSubsystem->CaptureReplayStateFields(Frame.Records,UnsupportedStableId))
        {
            Frame.Snapshot=Snapshot;
            Frame.Snapshot.PublicationSerial=SnapshotBuffer.PublishedSerial();
            Sink->Consume(MoveTemp(Frame));
            return;
        }
    }
    catch (...) { /* Allocation/schema failure affects recording only. */ }
    Sink->Failed(Snapshot.NumFrame,UnsupportedStableId);
    {
        FScopeLock Lock(&ReplaySinkMutex);
        // Do not disable a newer GT registration while retiring this generation.
        if (ReplaySink==Sink) {ReplaySink.Reset();bReplayCaptureRequested.Store(false);}
    }
}

namespace Speed::SimulationBoundary
{
void Detail::RequestLiveBoundaryCancellation(const std::shared_ptr<Detail::FLiveBoundaryControl>& Control) noexcept
{
    if (!Control) return;
    auto Phase = Control->Phase.load();
    if (Phase == Detail::ELiveBoundaryPhase::Disposed || Phase == Detail::ELiveBoundaryPhase::Retired) return;
    Control->CancelRequested.store(true);
    while (Phase == Detail::ELiveBoundaryPhase::Requested || Phase == Detail::ELiveBoundaryPhase::Acknowledged)
        if (Control->Phase.compare_exchange_weak(Phase, Detail::ELiveBoundaryPhase::ReleaseRequested)) break;
}
FLiveBoundaryRequest::~FLiveBoundaryRequest() { Detail::RequestLiveBoundaryCancellation(Control); }
ASpeedPhysicsSimulation* FLiveBoundaryLease::Driver() const { return Control->Driver.Get(); }
UWorld* FLiveBoundaryLease::World() const { return Control->World.Get(); }
uint64 FLiveBoundaryLease::WorkerGeneration() const { return Control->Generation; }
uint64 FLiveBoundaryLease::CompletedFrame() const { return Control->Snapshot.NumFrame; }
uint64 FLiveBoundaryLease::NextFrame() const { return Control->NextFrame; }
uint64 FLiveBoundaryLease::PublicationSerial() const { return Control->Snapshot.PublicationSerial; }
const FSimulationSnapshot& FLiveBoundaryLease::Snapshot() const { return Control->Snapshot; }
bool FLiveBoundaryLease::IsCurrent(const ASpeedPhysicsSimulation& Host) const { return Host.ValidateLiveBoundaryLease(*this); }
}

bool ASpeedPhysicsSimulation::HasLiveBoundaryScope() const
{
    using namespace Speed::SimulationBoundary::Detail;
    return IsInGameThread() && IsValid(this) && !IsActorBeingDestroyed() && LiveBoundaryControl && LiveBoundaryControl->ScopeActive
        && LiveBoundaryControl->Driver.Get() == this && LiveBoundaryControl->World.IsValid()
        && LiveBoundaryControl->World.Get() == GetWorld() && !LiveBoundaryControl->World->bIsTearingDown
        && LiveBoundaryControl->Generation == InputWorkerGeneration && IsOwnedWorkerExecutionMode()
        && LiveBoundaryControl->Phase.load() == ELiveBoundaryPhase::Executing
        && SimulationWorker && SimulationWorker->OwnsAcknowledgedLiveBoundary(LiveBoundaryControl);
}

bool ASpeedPhysicsSimulation::ValidateLiveBoundaryLease(const Speed::SimulationBoundary::FLiveBoundaryLease& Lease) const
{
    if (!HasLiveBoundaryScope() || Lease.Control != LiveBoundaryControl || LiveBoundaryControl->CancelRequested.load()) return false;
    const auto& Control = *LiveBoundaryControl;
    if (Control.Driver.Get() != this || !Control.World.IsValid() || Control.World.Get() != GetWorld()
        || Control.World->bIsTearingDown || Control.Generation != InputWorkerGeneration
        || !IsOwnedWorkerExecutionMode() || bInputOwnerRetired.Load() || bOwnedWorkerTerminal.Load()
        || bCanonicalPublicationTerminal.Load() || SimulationWorker->IsBoundaryServiceSuspendRequested()) return false;
    FSimulationSnapshot Current;
    return Control.Snapshot.NumFrame > 0 && Control.Snapshot.NumFrame != MAX_uint64
        && Control.Snapshot.PublicationSerial && ReadLatestSimulationSnapshot(Current)
        && Current.NumFrame == Control.Snapshot.NumFrame && Current.PublicationSerial == Control.Snapshot.PublicationSerial
        && Control.NextFrame == CanonicalNumFrame && Control.NextFrame == Current.NumFrame + 1;
}

TUniquePtr<Speed::SimulationBoundary::FLiveBoundaryRequest> ASpeedPhysicsSimulation::RequestLiveBoundary(FString& Failure)
{
    using namespace Speed::SimulationBoundary;
    Failure.Empty();
    if (!IsInGameThread() || !IsValid(this) || IsActorBeingDestroyed() || !HasActorBegunPlay() || !GetWorld() || GetWorld()->bIsTearingDown
        || !IsOwnedWorkerExecutionMode() || !SimulationWorker || !SimulationWorker->IsRunning()
        || bInputOwnerRetired.Load() || bOwnedWorkerTerminal.Load() || bCanonicalPublicationTerminal.Load()
        || OwnedBoundarySuspendDepth || SimulationWorker->IsBoundaryServiceSuspendRequested())
    { Failure = TEXT("Live boundary requires an available owned worker without a legacy suspension"); return {}; }
    if (LiveBoundaryControl)
    {
        const auto Phase = LiveBoundaryControl->Phase.load();
        if (Phase != Detail::ELiveBoundaryPhase::Disposed && Phase != Detail::ELiveBoundaryPhase::Retired)
        { Failure = TEXT("A live boundary request still owns its disposition"); return {}; }
        LiveBoundaryControl.reset();
    }
    FSimulationSnapshot Current;
    if (!ReadLatestSimulationSnapshot(Current) || !Current.PublicationSerial || !Current.NumFrame || Current.NumFrame == MAX_uint64)
    { Failure = TEXT("Live boundary requires a real completed frame after frame zero"); return {}; }
    auto Control = std::make_shared<Detail::FLiveBoundaryControl>();
    Control->Driver = this; Control->World = GetWorld(); Control->Generation = InputWorkerGeneration;
    auto Request = TUniquePtr<FLiveBoundaryRequest>(new FLiveBoundaryRequest(Control));
    if (!SimulationWorker->RequestLiveBoundaryControl(Control))
    { Failure = TEXT("Live boundary worker refused the structural request"); return {}; }
    LiveBoundaryControl = std::move(Control);
    return Request;
}

Speed::SimulationBoundary::ELiveBoundaryResult ASpeedPhysicsSimulation::PollLiveBoundary(
    Speed::SimulationBoundary::FLiveBoundaryRequest& Request,
    TFunctionRef<bool(const Speed::SimulationBoundary::FLiveBoundaryLease&, FString&)> AtBoundary, FString& Failure)
{
    using namespace Speed::SimulationBoundary;
    using namespace Speed::SimulationBoundary::Detail;
    Failure.Empty();
    const auto Control = Request.Control;
    if (!IsInGameThread() || !Control || Control->Driver.Get() != this)
    { Failure = TEXT("Live boundary request belongs to another owner or thread"); return ELiveBoundaryResult::Rejected; }
    const bool CurrentOwner = IsValid(this) && !IsActorBeingDestroyed() && HasActorBegunPlay() && Control->World.IsValid() && Control->World.Get() == GetWorld()
        && !Control->World->bIsTearingDown && Control->Generation == InputWorkerGeneration
        && IsOwnedWorkerExecutionMode() && SimulationWorker && SimulationWorker->IsRunning() && !SimulationWorker->bStopRequested.Load() && !bInputOwnerRetired.Load()
        && !bOwnedWorkerTerminal.Load() && !bCanonicalPublicationTerminal.Load();
    if (!CurrentOwner)
    {
        Control->RetirementRequested = true;
        Control->Failure = TEXT("Live boundary owner/world/generation was retired before disposition");
        RequestLiveBoundaryCancellation(Control);
        if (SimulationWorker) SimulationWorker->WakeLiveBoundaryControl();
    }
    auto Phase = Control->Phase.load();
    if (Phase == ELiveBoundaryPhase::Retired)
    { Failure = Control->Failure; return ELiveBoundaryResult::Retired; }
    if (Phase == ELiveBoundaryPhase::Disposed)
    {
        Failure = Control->Failure;
        if (LiveBoundaryControl == Control) LiveBoundaryControl.reset();
        if (Control->RetirementRequested) return ELiveBoundaryResult::Retired;
        if (Control->CancelRequested.load()) return ELiveBoundaryResult::Cancelled;
        return Control->CallbackRan && Control->CallbackAccepted ? ELiveBoundaryResult::Completed : ELiveBoundaryResult::Rejected;
    }
    if (!CurrentOwner || Control->CancelRequested.load() || Phase != ELiveBoundaryPhase::Acknowledged)
        return ELiveBoundaryResult::Waiting;
    if (LiveBoundaryControl != Control || OwnedBoundarySuspendDepth || SimulationWorker->IsBoundaryServiceSuspendRequested()
        || !SimulationWorker->OwnsAcknowledgedLiveBoundary(Control))
    {
        Control->Failure = TEXT("Live boundary ACK overlaps another structural owner");
        Control->CallbackRan = true;
        Control->Phase.store(ELiveBoundaryPhase::ReleaseRequested);
        SimulationWorker->WakeLiveBoundaryControl();
        return ELiveBoundaryResult::Waiting;
    }
    FSimulationSnapshot Current;
    if (!ReadLatestSimulationSnapshot(Current) || !Current.PublicationSerial || !Current.NumFrame || Current.NumFrame == MAX_uint64
        || CanonicalNumFrame != Current.NumFrame + 1)
    {
        Control->Failure = TEXT("Live boundary ACK has no exact completed publication/next frame");
        Control->CallbackRan = true;
        Control->Phase.store(ELiveBoundaryPhase::ReleaseRequested);
        SimulationWorker->WakeLiveBoundaryControl();
        return ELiveBoundaryResult::Waiting;
    }
    Control->Snapshot = std::move(Current); Control->NextFrame = CanonicalNumFrame;
    Control->Phase.store(ELiveBoundaryPhase::Executing); Control->ScopeActive = true; Control->CallbackRan = true;
    FLiveBoundaryLease Lease(Control);
    try { Control->CallbackAccepted = Lease.IsCurrent(*this) && AtBoundary(Lease, Control->Failure); }
    catch (...) { Control->CallbackAccepted = false; Control->Failure = TEXT("Live boundary callback failed"); }
    if (!Lease.IsCurrent(*this))
    { Control->CallbackAccepted = false; Control->RetirementRequested = !Control->CancelRequested.load(); Control->Failure = TEXT("Live boundary identity changed or cancellation requested during callback"); }
    Control->ScopeActive = false;
    // Admission required zero preexisting legacy depth, and every nested call
    // borrowed this request's ACK without sending a legacy worker request.
    // Those borrows expire with the callback. Reject a leaked borrow without
    // retaining a physical hold or releasing an independent legacy suspension.
    if (OwnedBoundarySuspendDepth)
    {
        OwnedBoundarySuspendDepth = 0;
        Control->CallbackAccepted = false;
        Control->Failure = TEXT("Live boundary callback retained an expired nested borrow");
    }
    if (Control->Phase.load() == ELiveBoundaryPhase::Retired) return ELiveBoundaryResult::Retired;
    Control->Phase.store(ELiveBoundaryPhase::ReleaseRequested);
    if (SimulationWorker) SimulationWorker->WakeLiveBoundaryControl();
    return ELiveBoundaryResult::Waiting;
}

Speed::SimulationBoundary::ELiveBoundaryResult ASpeedPhysicsSimulation::CancelLiveBoundary(
    Speed::SimulationBoundary::FLiveBoundaryRequest& Request, FString& Failure)
{
    using namespace Speed::SimulationBoundary;
    if (!IsInGameThread() || !Request.Control || Request.Control->Driver.Get() != this)
    { Failure = TEXT("Live boundary cancellation belongs to another owner or thread"); return ELiveBoundaryResult::Rejected; }
    Detail::RequestLiveBoundaryCancellation(Request.Control);
    if (SimulationWorker) SimulationWorker->WakeLiveBoundaryControl();
    return PollLiveBoundary(Request, [](const FLiveBoundaryLease&, FString&) { return false; }, Failure);
}

void ASpeedPhysicsSimulation::RetireLiveBoundaryRequest()
{
    if (!LiveBoundaryControl) return;
    // StopOwnedWorker has joined: no service callback or physical frame can
    // borrow this request after its actor/world/input generation is retired.
    LiveBoundaryControl->ScopeActive = false;
    LiveBoundaryControl->RetirementRequested = true;
    LiveBoundaryControl->Phase.store(Speed::SimulationBoundary::Detail::ELiveBoundaryPhase::Retired);
    LiveBoundaryControl.reset();
}
