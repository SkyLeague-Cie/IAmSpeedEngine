#include "IAmSpeed/World/Simulation/NativeEpochDiagnostics.h"
#include "SpeedPhysicsSimulation.h"
#include "IAmSpeed/Netcode/presentation-client/PresentationClientWorld.h"
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
#include "IAmSpeed/World/Simulation/FramePresentationTasks.h"
#include "CanonicalFrameContext.h"
#include "CanonicalEpisodeTestingScope.h"
#include "SimulationActorDiagnostics.h"
#include "OwnedFrameAudit.h"
#include "RealTimeSimulation.h"
#include "CanonicalFrameDriver.h"
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
    TWeakObjectPtr<ASpeedPhysicsSimulation> ReplayStartupKeyFor(const ASpeedPhysicsSimulation* Host)
    {return TWeakObjectPtr<ASpeedPhysicsSimulation>(const_cast<ASpeedPhysicsSimulation*>(Host));}
    bool HasReplayStartupHoldOnGameThread(const ASpeedPhysicsSimulation* Host)
    {check(IsInGameThread());return !ReplayStartupHolds.IsEmpty()&&ReplayStartupHolds.Contains(ReplayStartupKeyFor(Host));}
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
bool ASpeedPhysicsSimulation::BindOriginalDriverStorage(const std::shared_ptr<const Speed::NativeEpoch::IOriginalStorageRoute>& Route)
{
    if(!IsInGameThread()||!Route||OriginalDriverStorage||!OriginalDriverRoute.expired()||GetEngineFPS()!=300)return false;
    auto Native=std::make_unique<FOriginalDriverStorage>();
    Native->RealDeltaTime=1.0f/static_cast<float>(GetEngineFPS());
    Native->InputWorkerGeneration=NextInputWorkerGeneration.fetch_add(1);
    if(!Native->InputWorkerGeneration||Native->InputWorkerGeneration==MAX_uint64)return false;
    Native->InputSessionCommands=std::make_shared<Speed::Input::V2::FInputSessionCommands>();
    OriginalDriverStorage=std::move(Native);OriginalDriverRoute=Route;return true;
}

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
	RealDeltaTimeDriverStorage() = 1.0f / static_cast<float>(EngineFPS);
	InputWorkerGenerationDriverStorage() = NextInputWorkerGeneration.fetch_add(1);
	check(InputWorkerGenerationDriverStorage() && InputWorkerGenerationDriverStorage() != MAX_uint64);
	InputSessionCommandsDriverStorage() = std::make_shared<Speed::Input::V2::FInputSessionCommands>();
}

void ASpeedPhysicsSimulation::BeginPlay()
{
	if (Speed::Netcode::PresentationClient::SuppressesLocalPhysics(GetWorld())) { AActor::BeginPlay(); return; }
	Super::BeginPlay();
	// Fixed before the worker starts; diagnostics never change the canonical payload.
	bPublishPresentationDriverStorage() = FParse::Param(FCommandLine::Get(), TEXT("SpeedPublishPresentation"));
	bPublishCompletedWorldInputDriverStorage() = FParse::Param(FCommandLine::Get(), TEXT("SpeedPublishCompletedWorldInput"));
	RefreshExecutionMode();
}

bool ASpeedPhysicsSimulation::ReadPresentationPose(const uint64 StableId, FSimulationPoseConsumption& Out)
{
	check(IsInGameThread());
	if (!bPublishPresentationDriverStorage()) { Out = FSimulationPoseConsumption(); return false; }
	return PresentationLatchDriverStorage().ReadBody(GFrameCounter, SnapshotBufferDriverStorage(), StableId, Out);
}

bool ASpeedPhysicsSimulation::ReadCanonicalCameraSample(const uint64 NumFrame,
	FCameraCanonicalSample& Out) const
{
	check(IsInGameThread());
	return CameraSampleBufferDriverStorage().ReadFrame(NumFrame, Out);
}

bool ASpeedPhysicsSimulation::ReadPresentationOutput(const uint64 StableId, const uint32 Channel,
	FSimulationPresentationOutput& Out)
{
	check(IsInGameThread());
	if (!bPublishPresentationDriverStorage()) { Out = FSimulationPresentationOutput(); return false; }
	return PresentationLatchDriverStorage().ReadOutput(GFrameCounter, SnapshotBufferDriverStorage(), StableId, Channel, Out);
}

bool ASpeedPhysicsSimulation::RegisterPresentationProducer(
	TSharedRef<ISimulationPresentationProducer, ESPMode::ThreadSafe> Producer)
{
	check(IsInGameThread());
	FScopeLock Lock(&PresentationProducerMutexDriverStorage());
	if (bPresentationBindingClosedDriverStorage() || !Producer->OwnerStableId() || !Producer->Channel() || PresentationProducersDriverStorage().Num() >= 64 ||
		(Producer->IsCameraPresentation() && !AllowsCameraPresentation())) return false;
	for (const auto& Existing : PresentationProducersDriverStorage())
		if (Existing->OwnerStableId() == Producer->OwnerStableId() && Existing->Channel() == Producer->Channel()) return false;
	PresentationProducersDriverStorage().Add(Producer);
	return true;
}

void ASpeedPhysicsSimulation::UnregisterPresentationProducer(
	const TSharedRef<ISimulationPresentationProducer, ESPMode::ThreadSafe>& Producer)
{
	check(IsInGameThread());
	FScopeLock Lock(&PresentationProducerMutexDriverStorage());
	PresentationProducersDriverStorage().Remove(Producer);
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
		SPEED_NATIVE_LOG(LogTemp, Warning, TEXT("[PresentationUnbindingPauseUnacknowledged] Removal rejected; simulation remains pause-requested."));
		return false;
	}
	const bool bSuspensionLease = bOwnedLane && Boundary == ESimulationQuiescence::BoundaryAcknowledged;
	ON_SCOPE_EXIT { if (bSuspensionLease) ResumeOwnedBoundaryService(); if (!bWasPaused) ResumeOwnedSimulation(); };
	FScopeLock Lock(&PresentationProducerMutexDriverStorage());
	const int32 Index = PresentationProducersDriverStorage().IndexOfByKey(Producer);
	ReleaseAtBoundary();
	if (Index != INDEX_NONE) PresentationProducersDriverStorage().RemoveAt(Index);
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
            const FString Row=FString::Printf(TEXT("Refuse line=%d closed=%d mode=%d terminal=%d subsystem=%d collision=%d startup=%d worker=%d paused=%d frame=%llu boundary=%d owner=%llu target=%llu\n"),Line,bPresentationBindingClosedDriverStorage()?1:0,(int32)GetActiveExecutionMode(),bOwnedWorkerTerminalDriverStorage().Load()?1:0,SpeedWorldSubsystemDriverStorage()?1:0,(SpeedWorldSubsystemDriverStorage()&&SpeedWorldSubsystemDriverStorage()->GetStaticCollisionWorld())?1:0,HasReplayStartupHoldOnGameThread(this)?1:0,SimulationWorkerDriverStorage()?1:0,IsOwnedSimulationPaused()?1:0,(unsigned long long)CanonicalNumFrameDriverStorage(),ProbeBoundary,(unsigned long long)ProbeOwner,(unsigned long long)ProbeTarget);
            if(IFileManager::Get().FileSize(*File)<65536)FFileHelper::SaveStringToFile(Row,*File,FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM,&IFileManager::Get(),FILEWRITE_Append);
        }
        return nullptr;
    };
	if (bPresentationBindingClosedDriverStorage() || GetActiveExecutionMode() == ESimulationExecutionMode::UnrealAsyncCallback ||
		bOwnedWorkerTerminalDriverStorage().Load() || !EnsureSimulationWorldReady()) return Refuse(__LINE__);
	const bool bWasPaused = IsOwnedSimulationPaused();
	const bool bOwnedLane = GetActiveExecutionMode() == ESimulationExecutionMode::IAmSpeedThread;
	const ESimulationQuiescence Boundary = bOwnedLane ? TrySuspendOwnedBoundaryService() : TryPauseOwnedSimulation();
    ProbeBoundary=(int32)Boundary;
	if (Boundary != ESimulationQuiescence::BoundaryAcknowledged && Boundary != ESimulationQuiescence::AlreadyStopped)
	{
		SPEED_NATIVE_LOG(LogTemp, Warning, TEXT("[PresentationBindingPauseUnacknowledged] Registration rejected; simulation remains pause-requested."));
		return Refuse(__LINE__);
	}
	const bool bSuspensionLease = bOwnedLane && Boundary == ESimulationQuiescence::BoundaryAcknowledged;
	ON_SCOPE_EXIT { if (bSuspensionLease) ResumeOwnedBoundaryService(); if (!bWasPaused) ResumeOwnedSimulation(); };
	InitializeCanonicalFrame(0);
	FSimulationPresentationBinding Binding;
	Binding.OwnerStableId = SpeedWorldSubsystemDriverStorage()->GetSimulationStableId(OwnerComponent);
	Binding.TargetStableId = TargetComponent ? SpeedWorldSubsystemDriverStorage()->GetSimulationStableId(*TargetComponent) : 0;
	Binding.FirstFrame = CanonicalNumFrameDriverStorage();
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
		FScopeLock Lock(&PresentationProducerMutexDriverStorage());
		const int32 Index = PresentationProducersDriverStorage().IndexOfByKey(Previous.ToSharedRef());
		if (Index == INDEX_NONE || Previous->OwnerStableId() != Producer->OwnerStableId() ||
			Previous->Channel() != Producer->Channel()) return Refuse(__LINE__);
		// The acknowledged boundary makes replacement atomic with respect to Produce.
		PresentationProducersDriverStorage()[Index] = Producer.ToSharedRef();
	}
	else if (!RegisterPresentationProducer(Producer.ToSharedRef())) return Refuse(__LINE__);
	// Successful binding opts into its body/output sidecars. The owned worker
	// is pause-acknowledged above and resumes only after this boundary returns.
	// Failed factories/registration must leave publication policy unchanged.
	if (AdapterCommit) AdapterCommit->Finalize(Producer);
	bPublishPresentationDriverStorage() = true;
	return Producer;
}

bool ASpeedPhysicsSimulation::ReadCanonicalCameraSamples(const uint64 FirstFrame,
	const uint64 LastFrame, TArray<FCameraCanonicalSample>& Out) const
{
	check(IsInGameThread());
	return CameraSampleBufferDriverStorage().ReadRange(FirstFrame, LastFrame, Out);
}

uint64 ASpeedPhysicsSimulation::TryAcquireReplayStartupHoldBeforeBeginPlay()
{
    check(IsInGameThread());
    if(!::IsValid(this)||IsTemplate()||HasActorBegunPlay()||SimulationWorkerDriverStorage()||CanonicalNumFrameDriverStorage()!=0||
        bOwnedWorkerTerminalDriverStorage().Load()||bInputOwnerRetiredDriverStorage().Load()||!Speed::CanonicalFrameDriver::IsEnabled()||
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
	bPresentationBindingClosedDriverStorage() = true;
#if WITH_DEV_AUTOMATION_TESTS
	Speed::OwnedFrameAudit::TryExportForHostOnGameThread(this);
#endif
	StopOwnedWorker();
    Speed::ReplayLifecycle::RetireHost(*this);
	{
		FScopeLock Lock(&PresentationProducerMutexDriverStorage());
		PresentationProducersDriverStorage().Reset();
	}
	if (GetActiveExecutionMode() != ESimulationExecutionMode::UnrealAsyncCallback) PresentationRetiredAfterJoinDriverStorage()->Store(true);
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
    if(HasReplayStartupHoldOnGameThread(this))return;
	if (HostTransitionDelayTicksDriverStorage() > 0)
	{
		--HostTransitionDelayTicksDriverStorage();
		return;
	}
	const bool bExecutionPaused = bOwnedSimulationPausedDriverStorage().Load() ||
		Speed::CanonicalFrameDriver::IsOwnedThreadPaused();

	switch (GetActiveExecutionMode())
	{
	case ESimulationExecutionMode::UnrealAsyncCallback:
		return;
	case ESimulationExecutionMode::GameThread:
	{
		if (bExecutionPaused || bOwnedWorkerTerminalDriverStorage().Load())
		{
			return;
		}
		GameThreadAccumulatorSecondsDriverStorage() += FMath::Max(0.0f, DeltaSeconds);
		const double FrameIntervalSeconds =
			static_cast<double>(FCanonicalFrameContext::CanonicalPhysicalDeltaTime);
		while (GameThreadAccumulatorSecondsDriverStorage() + UE_DOUBLE_SMALL_NUMBER >= FrameIntervalSeconds)
		{
			const ESimulationWorkerResult Result = DriveSimulation(
				FCanonicalFrameContext::CanonicalPhysicalDeltaTime,
				GetCanonicalPulseSimTime());
			if (Result == ESimulationWorkerResult::Failed ||
				Result == ESimulationWorkerResult::Complete)
			{
				if (Result == ESimulationWorkerResult::Failed)
				{
					PublishedCompletedWorldInputDriverStorage().store({}, std::memory_order_seq_cst);
					bOwnedWorkerTerminalDriverStorage().Store(true);
					bOwnedSimulationPausedDriverStorage().Store(true);
				}
				GameThreadAccumulatorSecondsDriverStorage() = 0.0;
				return;
			}
			GameThreadAccumulatorSecondsDriverStorage() -= FrameIntervalSeconds;
			if (Result == ESimulationWorkerResult::Idle)
			{
				// Readiness delay is wall-clock time, not simulation time. Keep at
				// most one frame of debt and retry on the next game-thread tick.
				GameThreadAccumulatorSecondsDriverStorage() = FMath::Min(
					GameThreadAccumulatorSecondsDriverStorage(), FrameIntervalSeconds);
				return;
			}
		}
		return;
	}
	case ESimulationExecutionMode::IAmSpeedThread:
		StartOwnedWorkerIfReady();
		if (SimulationWorkerDriverStorage())
		{
			if (bExecutionPaused && !SimulationWorkerDriverStorage()->IsPaused())
			{
				SimulationWorkerDriverStorage()->Pause();
			}
			else if (!bExecutionPaused && SimulationWorkerDriverStorage()->IsPaused())
			{
				SimulationWorkerDriverStorage()->Resume();
			}
		}
		return;
	default:
		return;
	}
}

void ASpeedPhysicsSimulation::RefreshExecutionMode()
{
	if (Speed::Netcode::PresentationClient::SuppressesLocalPhysics(GetWorld())) return;
	const ESimulationExecutionMode DesiredMode =
		Speed::CanonicalFrameDriver::IsEnabled()
		? Speed::CanonicalFrameDriver::GetExecutionMode()
		: ESimulationExecutionMode::UnrealAsyncCallback;
	if (!bExecutionModeInitializedDriverStorage() || DesiredMode != GetActiveExecutionMode())
	{
		TransitionExecutionMode(DesiredMode);
	}
}

void ASpeedPhysicsSimulation::TransitionExecutionMode(
	const ESimulationExecutionMode NewMode)
{
	if (bInputOwnerRetiredDriverStorage().Load()) return;
    // A manual unsupported mode change invalidates startup admission; the
    // recording owner must refuse rather than override the requested mode.
    if(NewMode!=ESimulationExecutionMode::IAmSpeedThread&&!ReplayStartupHolds.IsEmpty())ReplayStartupHolds.Remove(ReplayStartupKeyFor(this));
	StopOwnedWorker();
    Speed::ReplayLifecycle::RetireHost(*this);
	bAsyncPhysicsTickEnabled = false;
	ResetCanonicalFrame();
	GameThreadAccumulatorSecondsDriverStorage() = 0.0;
	bOwnedWorkerTerminalDriverStorage().Store(false);
	ActiveExecutionModeValueDriverStorage().Store(static_cast<uint8>(NewMode));
	bExecutionModeInitializedDriverStorage() = true;
	HostTransitionDelayTicksDriverStorage() = NewMode == ESimulationExecutionMode::UnrealAsyncCallback ? 0 : 1;
	if (NewMode == ESimulationExecutionMode::UnrealAsyncCallback)
	{
		bAsyncPhysicsTickEnabled = true;
	}
}

void ASpeedPhysicsSimulation::StartOwnedWorkerIfReady()
{
	if (Speed::Netcode::PresentationClient::SuppressesLocalPhysics(GetWorld())) return;
    if(HasReplayStartupHoldOnGameThread(this))return;
	if (SimulationWorkerDriverStorage() || bOwnedWorkerTerminalDriverStorage().Load() ||
		!EnsureSimulationWorldReady())
	{
		return;
	}
	InitializeCanonicalFrame(0.0f);
	OnOwnedWorkerStarting();
#if WITH_DEV_AUTOMATION_TESTS
	// Admission is checked from the class that owns the private execution mode.
	const auto OwnedAudit = Speed::OwnedFrameAudit::CreateForHost(this,
		GetActiveExecutionMode() == ESimulationExecutionMode::IAmSpeedThread &&
		IsA<ARealTimeSimulation>() && CVarSimulationPerformanceAudit.GetValueOnGameThread() != 0,
		FCommandLine::Get());
#endif
	SimulationWorkerDriverStorage() = MakeUnique<FSimulationWorker>(
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
			Speed::OwnedFrameAudit::BeginWork(OwnedAudit, CanonicalNumFrameDriverStorage());
#endif
			const ESimulationWorkerResult Result = DriveOwnedWorkerPulse(
				FCanonicalFrameContext::CanonicalPhysicalDeltaTime,
				GetCanonicalPulseSimTime());
#if WITH_DEV_AUTOMATION_TESTS
			Speed::OwnedFrameAudit::EndWork(OwnedAudit, Result);
#endif
			if (Result == ESimulationWorkerResult::Complete ||
				Result == ESimulationWorkerResult::Failed)
			{
				PublishedCompletedWorldInputDriverStorage().store({}, std::memory_order_seq_cst);
				bOwnedWorkerTerminalDriverStorage().Store(true);
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
			Speed::OwnedFrameAudit::BeginService(OwnedAudit, bPauseRequested, CanonicalNumFrameDriverStorage());
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
		});
	if (!SimulationWorkerDriverStorage()->Start(bOwnedSimulationPausedDriverStorage().Load() || Speed::CanonicalFrameDriver::IsOwnedThreadPaused()))
	{
		SimulationWorkerDriverStorage().Reset();
		PublishedCompletedWorldInputDriverStorage().store({}, std::memory_order_seq_cst);
		bOwnedWorkerTerminalDriverStorage().Store(true);
		SPEED_NATIVE_LOG(LogTemp, Error, TEXT("[SimulationWorkerStartFailed]"));
	}
}

void ASpeedPhysicsSimulation::StopOwnedWorker()
{
	if (SimulationWorkerDriverStorage())
	{
		SimulationWorkerDriverStorage()->StopAndJoin();
		SimulationWorkerDriverStorage().Reset();
	}
	// After join, no in-flight frame can republish an endpoint.
	PublishedCompletedWorldInputDriverStorage().store({}, std::memory_order_seq_cst);
}

bool ASpeedPhysicsSimulation::PrepareControlledInputRun()
{
	check(IsInGameThread());
	if (GetActiveExecutionMode() != ESimulationExecutionMode::IAmSpeedThread
		|| bCanonicalPublicationTerminalDriverStorage().Load()) return false;
	const auto Boundary = TryPauseOwnedSimulation();
	if (Boundary != ESimulationQuiescence::BoundaryAcknowledged
		&& Boundary != ESimulationQuiescence::AlreadyStopped) return false;
	StopOwnedWorker(); // CloseInputSessionsOnWorker retires producers on their lane.
	FScopeLock Lock(&InputSessionAdmissionMutexDriverStorage());
	if (InputSessionRegistryDriverStorage()
		|| (bInputSessionAdmissionRequestedDriverStorage().Load() && !bInputOwnersClosedOnWorkerDriverStorage().Load())
		|| !PendingInputJournalsDriverStorage().empty() || !PendingInputObservationsDriverStorage().empty()
		|| !InputObservationsDriverStorage().empty() || bCanonicalPublicationTerminalDriverStorage().Load()) return false;
	try
	{
		auto NextCommands = std::make_shared<Speed::Input::V2::FInputSessionCommands>();
		const auto Generation = NextInputWorkerGeneration.fetch_add(1);
		if (!Generation || Generation == MAX_uint64) return false;
		InputSessionCommandsDriverStorage() = std::move(NextCommands);
		InputWorkerGenerationDriverStorage() = Generation; NextInputCommandIdDriverStorage() = 1;
	}
	catch (...) { return false; }
	PublishedCompletedWorldInputDriverStorage().store({}, std::memory_order_seq_cst);
	PublishedInputRegistryDriverStorage().store(std::shared_ptr<const Speed::Input::V2::FInputRegistryView>{}, std::memory_order_seq_cst);
	PublishedInputFrameDriverStorage().store(std::shared_ptr<const Speed::Input::V2::FRegistryFrame>{}, std::memory_order_seq_cst);
	bInputSessionAdmissionRequestedDriverStorage().Store(false); bInputOwnersClosedOnWorkerDriverStorage().Store(false);
	bInputNeutralizedDuringPauseDriverStorage() = false; NeutralizedInputRegistryVersionDriverStorage() = MAX_uint64;
	bInputOwnerRetiredDriverStorage().Store(false); bOwnedWorkerTerminalDriverStorage().Store(false);
	CanonicalReadyDelayPulsesObservedDriverStorage() = 0;
	return true; // Remains paused until the new scenario is sealed.
}

void ASpeedPhysicsSimulation::RestartControlledRun()
{
	check(IsInGameThread());
	if (bInputOwnerRetiredDriverStorage().Load() || bOwnedWorkerTerminalDriverStorage().Load()
		|| bCanonicalPublicationTerminalDriverStorage().Load() || !IsOwnedSimulationPaused())
	{
		SPEED_NATIVE_LOG(LogTemp, Error, TEXT("[ControlledRunNotPrepared] Prepare the new generation before admitting scenario inputs."));
		return;
	}
	// Never join here: replacement descriptors already belong to this generation.
	CanonicalReadyDelayPulsesObservedDriverStorage() = 0;
	ResumeOwnedSimulation();
}

ESimulationQuiescence ASpeedPhysicsSimulation::TryPauseOwnedSimulation(const uint32 TimeoutMilliseconds)
{
	check(IsInGameThread());
	PublishedCompletedWorldInputDriverStorage().store({}, std::memory_order_seq_cst);
	bOwnedSimulationPausedDriverStorage().Store(true);
	if (!SimulationWorkerDriverStorage() && HasActorBegunPlay()
		&& GetActiveExecutionMode() == ESimulationExecutionMode::UnrealAsyncCallback)
		return ESimulationQuiescence::Failed; // No owned boundary for legacy async callbacks.
	if (!SimulationWorkerDriverStorage() || !SimulationWorkerDriverStorage()->IsRunning())
	{
		OnOwnedSimulationPaused();
		return ESimulationQuiescence::AlreadyStopped;
	}
	if (!SimulationWorkerDriverStorage()->TryPause(TimeoutMilliseconds)) return ESimulationQuiescence::TimedOut;
	OnOwnedSimulationPaused();
	return ESimulationQuiescence::BoundaryAcknowledged;
}

ESimulationQuiescence ASpeedPhysicsSimulation::TrySuspendOwnedBoundaryService(const uint32 TimeoutMilliseconds)
{
	check(IsInGameThread());
	PublishedCompletedWorldInputDriverStorage().store({}, std::memory_order_seq_cst);
	if (GetActiveExecutionMode() != ESimulationExecutionMode::IAmSpeedThread)
		return ESimulationQuiescence::Failed;
	if (OwnedBoundarySuspendDepthDriverStorage())
	{
		// Joining while a recreation lease is held supersedes the parked
		// boundary. The GT may finish teardown after the worker is gone; the
		// outstanding lease still has to be released exactly once.
		if (!SimulationWorkerDriverStorage() || !SimulationWorkerDriverStorage()->IsRunning())
			return ESimulationQuiescence::AlreadyStopped;
		if (!SimulationWorkerDriverStorage()->IsBoundaryServiceSuspended())
			return ESimulationQuiescence::Failed;
		++OwnedBoundarySuspendDepthDriverStorage();
		return ESimulationQuiescence::BoundaryAcknowledged;
	}
	if (!SimulationWorkerDriverStorage() || !SimulationWorkerDriverStorage()->IsRunning())
		return ESimulationQuiescence::AlreadyStopped;
	bOwnedSimulationPausedDriverStorage().Store(true);
	if (!SimulationWorkerDriverStorage()->TrySuspendBoundaryService(TimeoutMilliseconds))
		return ESimulationQuiescence::TimedOut;
	OwnedBoundarySuspendDepthDriverStorage() = 1;
	return ESimulationQuiescence::BoundaryAcknowledged;
}

void ASpeedPhysicsSimulation::ResumeOwnedBoundaryService()
{
	check(IsInGameThread());
	if (!OwnedBoundarySuspendDepthDriverStorage()) return;
	if (--OwnedBoundarySuspendDepthDriverStorage() == 0 && SimulationWorkerDriverStorage())
		SimulationWorkerDriverStorage()->ResumeBoundaryService();
}

bool ASpeedPhysicsSimulation::IsOwnedBoundaryServiceSuspended() const
{
	return OwnedBoundarySuspendDepthDriverStorage() && SimulationWorkerDriverStorage() && SimulationWorkerDriverStorage()->IsBoundaryServiceSuspended();
}

void ASpeedPhysicsSimulation::PauseOwnedSimulation()
{
	PublishedCompletedWorldInputDriverStorage().store({}, std::memory_order_seq_cst);
	bOwnedSimulationPausedDriverStorage().Store(true);
	if (SimulationWorkerDriverStorage())
	{
		SimulationWorkerDriverStorage()->Pause();
	}
	OnOwnedSimulationPaused();
}

bool ASpeedPhysicsSimulation::ReadInputFirstFrameAtPausedBoundary(uint64& OutFrame)
{
	check(IsInGameThread());
	if (GetActiveExecutionMode() != ESimulationExecutionMode::IAmSpeedThread
		|| bInputOwnerRetiredDriverStorage().Load() || !bOwnedSimulationPausedDriverStorage().Load() || bCanonicalPublicationTerminalDriverStorage().Load()
		|| (SimulationWorkerDriverStorage() && SimulationWorkerDriverStorage()->IsRunning() && !SimulationWorkerDriverStorage()->IsPaused()))
	{
		// Current atomic observations after the existing rejection, not a new ACK/readiness predicate.
		RecordPrivateInputFailure(TEXT("SpeedPhysicsSimulation.ReadFirstFramePausedBoundaryRejected"), FString::Printf(
			TEXT("mode=%u retired=%d paused=%d publication_terminal=%d worker_present=%d worker_running=%d worker_paused=%d worker_terminal=%d driver_uid=%u generation=%llu"),
			uint32(GetActiveExecutionMode()), int32(bInputOwnerRetiredDriverStorage().Load()), int32(bOwnedSimulationPausedDriverStorage().Load()),
			int32(bCanonicalPublicationTerminalDriverStorage().Load()), int32(bool(SimulationWorkerDriverStorage())),
			SimulationWorkerDriverStorage() ? int32(SimulationWorkerDriverStorage()->IsRunning()) : -1,
			SimulationWorkerDriverStorage() ? int32(SimulationWorkerDriverStorage()->IsPaused()) : -1,
			int32(bOwnedWorkerTerminalDriverStorage().Load()), GetUniqueID(), GetInputWorkerGeneration()));
		return false;
	}
	InitializeCanonicalFrame(0.0f); OutFrame = CanonicalNumFrameDriverStorage(); return true;
}

bool ASpeedPhysicsSimulation::JoinOwnedSimulationForInputTeardown()
{
	check(IsInGameThread());
	// An async callback (or absent driver) cannot grant an owned-thread fence.
	if (GetActiveExecutionMode() != ESimulationExecutionMode::IAmSpeedThread) return false;
	bInputOwnerRetiredDriverStorage().Store(true);
	PublishedCompletedWorldInputDriverStorage().store({}, std::memory_order_seq_cst);
	bOwnedSimulationPausedDriverStorage().Store(true); bOwnedWorkerTerminalDriverStorage().Store(true);
	StopOwnedWorker();
	if (SimulationWorkerDriverStorage()) return false;
	if (!bInputOwnersClosedOnWorkerDriverStorage().Load())
	{
		FScopeLock Lock(&InputSessionAdmissionMutexDriverStorage());
		// No published registry plus no surviving worker-owned registry means
		// only inert, unserviced descriptors may remain. Do not manufacture a
		// worker-owner retirement receipt for a registry that actually ran.
		if (InputSessionRegistryDriverStorage() || ReadInputRegistryView() || !InputObservationsDriverStorage().empty()
			|| !InputSessionCommandsDriverStorage()->CancelUnprocessedAfterJoin()) return false;
		for (const auto& Observation : PendingInputObservationsDriverStorage()) Observation.second->Deactivate();
		PendingInputObservationsDriverStorage().clear(); PendingInputJournalsDriverStorage().clear();
		bInputOwnersClosedOnWorkerDriverStorage().Store(true); // Joined proof: zero owners were constructed.
	}
	return true;
}

void ASpeedPhysicsSimulation::ResumeOwnedSimulation()
{
    // Native input/gameplay callers are GT owners. Refuse a foreign-thread
    // resume before consulting GT lifecycle ownership or changing pause state.
    if(!IsInGameThread())return;
	if (bInputOwnerRetiredDriverStorage().Load()) return;
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
	if (SimulationWorkerDriverStorage() && SimulationWorkerDriverStorage()->IsBoundaryServiceSuspendRequested()) return;
    if (Speed::ReplayLifecycle::DeferResume(this)) return;
	bOwnedSimulationPausedDriverStorage().Store(false);
	OnOwnedSimulationResumed();
	if (SimulationWorkerDriverStorage() &&
		!Speed::CanonicalFrameDriver::IsOwnedThreadPaused())
	{
		SimulationWorkerDriverStorage()->Resume();
	}
}

ESimulationExecutionMode ASpeedPhysicsSimulation::GetActiveExecutionMode() const
{
	return static_cast<ESimulationExecutionMode>(ActiveExecutionModeValueDriverStorage().Load());
}

bool ASpeedPhysicsSimulation::IsOwnedWorkerExecutionMode() const
{
	return GetActiveExecutionMode() == ESimulationExecutionMode::IAmSpeedThread;
}

float ASpeedPhysicsSimulation::GetCanonicalPulseSimTime() const
{
	return static_cast<float>(CanonicalNumFrameDriverStorage()) *
		FCanonicalFrameContext::CanonicalPhysicalDeltaTime;
}

void ASpeedPhysicsSimulation::UpdateNumFrame(const float& SimTime)
{
	_NumFrameDriverStorage() = Speed::SimUtils::ComputeNumFrameFromSimTime(EngineFPS, SimTime);
}

unsigned int ASpeedPhysicsSimulation::NumFrame() const
{
    return _NumFrameDriverStorage();
}

USpeedWorldSubsystem* ASpeedPhysicsSimulation::GetSpeedWorldSubsystem(UWorld* World)
{
    if (!World) return nullptr;
    USpeedWorldSubsystem* SS = World->GetSubsystem<USpeedWorldSubsystem>();
	return SS;
}

void ASpeedPhysicsSimulation::AsyncPhysicsTickActor(float Dt, float SimTime)
{
	if (GetActiveExecutionMode() != ESimulationExecutionMode::UnrealAsyncCallback)
	{
		return;
	}
	if (bOwnedSimulationPausedDriverStorage().Load() ||
		Speed::CanonicalFrameDriver::IsOwnedThreadPaused())
	{
		return;
	}
	QUICK_SCOPE_CYCLE_COUNTER(STAT_IAmSpeed_AsyncPhysicsTickActor);
	TRACE_CPUPROFILER_EVENT_SCOPE(IAmSpeed_AsyncPhysicsTickActor);
	const double CallbackStartSeconds = FPlatformTime::Seconds();
	const uint64 StepSerialAtEntry = SpeedWorldSubsystemDriverStorage()
		? SpeedWorldSubsystemDriverStorage()->GetLastStepDiagnostics().Serial : 0;
	ON_SCOPE_EXIT
	{
		if (!SpeedWorldSubsystemDriverStorage() || !ShouldMeasureCallback())
		{
			return;
		}

		const FSpeedStepDiagnostics& Diagnostics =
			SpeedWorldSubsystemDriverStorage()->GetLastStepDiagnostics();
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
			(SlowFrameReportCountDriverStorage() == 0 ||
				Diagnostics.Frame >= LastSlowFrameReportDriverStorage() + 300u);
		if (!bDetailed && !bReportSlow)
		{
			return;
		}

		if (bReportSlow)
		{
			++SlowFrameReportCountDriverStorage();
			LastSlowFrameReportDriverStorage() = Diagnostics.Frame;
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
			SPEED_NATIVE_LOG(LogTemp, Warning, TEXT("%s"), *Message);
		}
		else
		{
			SPEED_NATIVE_LOG(LogTemp, Display, TEXT("%s"), *Message);
		}
	};

	DriveSimulation(Dt, SimTime);
}

bool ASpeedPhysicsSimulation::EnsureSimulationWorldReady()
{
	if (Speed::Netcode::PresentationClient::SuppressesLocalPhysics(GetWorld())) return false;
	if (!SpeedWorldSubsystemDriverStorage())
	{
		SpeedWorldSubsystemDriverStorage() = GetSpeedWorldSubsystem(GetWorld());
	}
	if (!SpeedWorldSubsystemDriverStorage()) return false;
	if (Speed::Analytic::FStaticWorldQueryAudit::IsSurfaceAnalyticBackend() &&
		!SpeedWorldSubsystemDriverStorage()->GetStaticCollisionWorld())
	{
		if (!bStaticCollisionReadinessErrorReportedDriverStorage())
		{
			bStaticCollisionReadinessErrorReportedDriverStorage() = true;
			SPEED_NATIVE_LOG(LogTemp, Error,
				TEXT("[StaticCollisionAuthorityNotReady] Canonical simulation is waiting for a valid analytical static collision world."));
		}
		return false;
	}
	bStaticCollisionReadinessErrorReportedDriverStorage() = false;
	return true;
}

ESimulationWorkerResult ASpeedPhysicsSimulation::CheckCanonicalRunReadiness()
{
	const ECanonicalRunControlState RunState =
		SpeedWorldSubsystemDriverStorage()->GetCanonicalRunControlState();
	if (RunState == ECanonicalRunControlState::WaitingForScenario ||
		(RunState == ECanonicalRunControlState::Uncontrolled &&
			RequiresCanonicalRunController()))
	{
		CanonicalReadyDelayPulsesObservedDriverStorage() = 0;
		return ESimulationWorkerResult::Idle;
	}
	if (RunState == ECanonicalRunControlState::Complete)
	{
		return ESimulationWorkerResult::Complete;
	}
	if (RunState == ECanonicalRunControlState::Ready &&
		CanonicalReadyDelayPulsesObservedDriverStorage() <
			Speed::CanonicalFrameDriver::GetFastReadyDelayCallbacks())
	{
		++CanonicalReadyDelayPulsesObservedDriverStorage();
		return ESimulationWorkerResult::Idle;
	}
	return ESimulationWorkerResult::Advanced;
}

void ASpeedPhysicsSimulation::InitializeCanonicalFrame(const float SimTime)
{
	if (bCanonicalFrameInitializedDriverStorage()) return;
	UpdateNumFrame(SimTime);
	CanonicalNumFrameDriverStorage() = NumFrame() > 0 ? uint64(NumFrame() - 1u) : 0u;
	bCanonicalFrameInitializedDriverStorage() = true;
}

void ASpeedPhysicsSimulation::ResetCanonicalFrame()
{
	bCanonicalFrameInitializedDriverStorage() = false;
	CanonicalReadyDelayPulsesObservedDriverStorage() = 0;
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
    if (!IsInGameThread() || SimulationWorkerDriverStorage() || !IsOwnedWorkerExecutionMode() ||
        !bInputOwnerRetiredDriverStorage().Load() || !bInputOwnersClosedOnWorkerDriverStorage().Load() || CaseSatDiagnosticEpochDriverStorage()==MAX_uint64)
    {
        // Never mutate identity potentially being read by an active owner, even
        // if the live cvar changed to0. This rejected diagnostic call is inert.
        try
        {
            if (IsEnabled()) SPEED_NATIVE_LOG(LogTemp, Warning, TEXT("[CaseSatUnavailable] Case=%d Reason=NoJoinedOwnedLane DiagnosticOnly=1"),NativeCaseId);
        }
        catch (...) { /* Diagnostics cannot interrupt ResetActors or mutate a live identity. */ }
        return false;
    }
    ParallelFrameTimingDriverStorage().ConfigureAfterJoin(uint64(reinterpret_cast<UPTRINT>(this)),
        NativeCaseId, BatchIndex, WorldGeneration);
    // Existing joined-owner fence above, outside the canonical frame body.
    if(!ParallelFrameTimingDriverStorage().IsActive() && ObserveJoinedAnalytic1075())
    {
        Speed::Analytic::ResetAnalyticCausalStats1075();
        Speed::Analytic::ResetPiecewiseProviderWaveStats1075();
        Speed::Analytic::ResetTensorCellWaveStats1213();
        Speed::Analytic::ResetAnalyticPrefetchStats1075();
    }
    // The already-joined owned lane is the authority. This case identity is
    // independent of CaseSat diagnostics, so real parallel kernels stay on.
    NativeWitnessCaseDriverStorage()=Speed::FNativeFrameWitnessCaseIdentity{};
    FString WitnessUUID;
    if(NativeWitnessEpochDriverStorage()!=MAX_uint64 && NativeCaseId>=0 &&
       FParse::Value(FCommandLine::Get(),TEXT("SLFrameWitnessRunUUID="),WitnessUUID) &&
       !WitnessUUID.IsEmpty() && WitnessUUID.Len()<=64)
    {
        NativeWitnessCaseDriverStorage().RunUUID=MoveTemp(WitnessUUID);
        NativeWitnessCaseDriverStorage().DriverKey=uint64(reinterpret_cast<UPTRINT>(this));
        NativeWitnessCaseDriverStorage().Epoch=++NativeWitnessEpochDriverStorage();NativeWitnessCaseDriverStorage().WorldGeneration=WorldGeneration;
        NativeWitnessCaseDriverStorage().NativeCaseId=NativeCaseId;NativeWitnessCaseDriverStorage().BatchIndex=BatchIndex;
        NativeWitnessCaseDriverStorage().bConfigured=true;
    }
    // Read the already-established joined owner fence, after mode initialization.
    // V2 is per-case and must match the real CaseBegin identity; it is not a
    // replacement for consumed input or callback/source closure.
    if(ParallelFrameTimingDriverStorage().IsActive() || NativeWitnessCaseDriverStorage().bConfigured)
        SPEED_NATIVE_LOG(LogTemp,Display,TEXT("[IAmSpeedOwnedCasePolicyV2] Driver=%llu Case=%d Batch=%d WorldGeneration=%llu ExecutionHost=%u OwnedInputFence=1 Scope=ExistingJoinedOwnerFence"),
            uint64(reinterpret_cast<UPTRINT>(this)),NativeCaseId,BatchIndex,WorldGeneration,uint32(GetActiveExecutionMode()));
    if (!IsEnabled()) { CaseSatDiagnosticIdentityDriverStorage()=FCaseIdentity{}; return true; }
    try
    {
        return BuildCaseIdentity(CaseSatDiagnosticIdentityDriverStorage(),
            uint64(reinterpret_cast<UPTRINT>(this)),++CaseSatDiagnosticEpochDriverStorage(),
            NativeCaseId,BatchIndex,WorldGeneration);
    }
    catch (...) { CaseSatDiagnosticIdentityDriverStorage()=FCaseIdentity{}; return false; }
}
#endif

#if !UE_BUILD_SHIPPING
void ASpeedPhysicsSimulation::BeginIndependentRealTimeCoverage696() noexcept
{
    ParallelFrameTimingDriverStorage().BeginDriver(false, CanonicalNumFrameDriverStorage(), IsOwnedWorkerExecutionMode());
    IndependentFastCoverage276DriverStorage().BeginRealTime(CaseSatDiagnosticIdentityDriverStorage(),CanonicalNumFrameDriverStorage(),
        PerformanceFrameCountDriverStorage(),IsOwnedWorkerExecutionMode());
}
void ASpeedPhysicsSimulation::EndIndependentRealTimeCoverage696(bool bCompleted, const TCHAR* Reason) noexcept
{
    ParallelFrameTimingDriverStorage().Close(bCompleted, Reason);
    if(ParallelFrameTimingDriverStorage().ClaimJoinedCaseLog())
        LogJoinedAnalytic1075(ParallelFrameTimingDriverStorage().JoinedCaseContextForLogging());
    IndependentFastCoverage276DriverStorage().EndRealTime(CanonicalNumFrameDriverStorage(),PerformanceFrameCountDriverStorage(),bCompleted,Reason);
}
void ASpeedPhysicsSimulation::BeginIndependentFastCoverage276() noexcept
{
    ParallelFrameTimingDriverStorage().BeginDriver(true, CanonicalNumFrameDriverStorage(), IsOwnedWorkerExecutionMode());
    IndependentFastCoverage276DriverStorage().BeginFast(CaseSatDiagnosticIdentityDriverStorage(),CanonicalNumFrameDriverStorage(),
        PerformanceFrameCountDriverStorage(),IsOwnedWorkerExecutionMode());
}
void ASpeedPhysicsSimulation::EndIndependentFastCoverage276(bool bCompleted, const TCHAR* Reason) noexcept
{
    ParallelFrameTimingDriverStorage().Close(bCompleted, Reason);
    if(ParallelFrameTimingDriverStorage().ClaimJoinedCaseLog())
        LogJoinedAnalytic1075(ParallelFrameTimingDriverStorage().JoinedCaseContextForLogging());
    IndependentFastCoverage276DriverStorage().EndFast(CanonicalNumFrameDriverStorage(),PerformanceFrameCountDriverStorage(),bCompleted,Reason);
}
#endif

FCanonicalFrameContext ASpeedPhysicsSimulation::MakeCanonicalFrameContext(const uint64 Frame) const
{
    FCanonicalFrameContext Context(Frame);
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
    if (CollisionMatrixTestHzDriverStorage() != 0)
    {
        check(Frame >= CollisionMatrixTestFirstFrameDriverStorage());
        Context.PhysicalDeltaTime = 1.0f / float(CollisionMatrixTestHzDriverStorage());
        Context.SimTime = float(Frame - CollisionMatrixTestFirstFrameDriverStorage()) * Context.PhysicalDeltaTime;
    }
#endif
    return Context;
}

bool ASpeedPhysicsSimulation::RunCanonicalFrames(const uint32 FrameCount)
{
#if !UE_BUILD_SHIPPING
    Speed::Diagnostics::CaseCoverage276::FScopedPreloop CallerPreloop276(IndependentFastCoverage276DriverStorage());
#endif
	if (!ProcessPendingRollbackRequest())
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
        Speed::Diagnostics::CaseCoverage276::FScopedForward CallerForward276(IndependentFastCoverage276DriverStorage(),CanonicalNumFrameDriverStorage());
        Speed::Analytic::CaseSatDiagnostics::FScopedFrame DiagnosticFrame(CaseSatDiagnosticIdentityDriverStorage(),CanonicalNumFrameDriverStorage());
#endif
		const double PhysicalFrameStartSeconds = FPlatformTime::Seconds();
#if WITH_DEV_AUTOMATION_TESTS
		const auto OwnedStep = Speed::OwnedFrameAudit::BeginStep(CanonicalNumFrameDriverStorage(), false);
#endif
		const bool StepSucceeded = StepCanonicalFrame(MakeCanonicalFrameContext(CanonicalNumFrameDriverStorage()));
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
		++CanonicalNumFrameDriverStorage();
	}
	return true;
}

void ASpeedPhysicsSimulation::Simulate(const float& DeltaTime, const float& SimTime)
{
    if (!SpeedWorldSubsystemDriverStorage())
    {
		SpeedWorldSubsystemDriverStorage() = GetSpeedWorldSubsystem(GetWorld());
		if (!SpeedWorldSubsystemDriverStorage())
		{
			return;
		}
	}

	const double PhysicalFrameStartSeconds = FPlatformTime::Seconds();
	SpeedWorldSubsystemDriverStorage()->Step(DeltaTime, SimTime, NumFrame());
	RecordStepPerformance(
		(FPlatformTime::Seconds() - PhysicalFrameStartSeconds) * 1000.0);
}

bool ASpeedPhysicsSimulation::QueueSimulationInput(const uint64 ActivationFrame,
	const uint64 TargetStableId, const TConstArrayView<uint8> Payload)
{
	return InputJournalDriverStorage().Append(ActivationFrame, TargetStableId, Payload);
}

void ASpeedPhysicsSimulation::SealSimulationInputs()
{
	InputJournalDriverStorage().Seal();
}

void ASpeedPhysicsSimulation::RecordStepPerformance(
	const double PhysicalFrameMilliseconds)
{
	if (!SpeedWorldSubsystemDriverStorage())
	{
		return;
	}
	const FSpeedStepDiagnostics& Diagnostics =
		SpeedWorldSubsystemDriverStorage()->GetLastStepDiagnostics();
	if (Diagnostics.Serial == 0 || Diagnostics.Serial == LastRecordedStepSerialDriverStorage())
	{
		return;
	}
	LastRecordedStepSerialDriverStorage() = Diagnostics.Serial;
	LastPhysicalFrameMillisecondsDriverStorage() = PhysicalFrameMilliseconds;
	++PerformanceFrameCountDriverStorage();
	PerformanceTotalStepMillisecondsDriverStorage() += PhysicalFrameMilliseconds;
	PerformanceMaximumStepMillisecondsDriverStorage() = FMath::Max(
		PerformanceMaximumStepMillisecondsDriverStorage(), PhysicalFrameMilliseconds);
	const double WarningThresholdMilliseconds =
		Diagnostics.PhysicalDeltaTimeMilliseconds * FMath::Max(
			0.0, static_cast<double>(
				CVarSimulationWarningFrameFraction.GetValueOnAnyThread()));
	PerformanceWarningFrameCountDriverStorage() +=
		PhysicalFrameMilliseconds > WarningThresholdMilliseconds ? 1u : 0u;
	PerformanceIterationLimitCountDriverStorage() +=
		Diagnostics.bIterationLimitReached ? 1u : 0u;
}

float ASpeedPhysicsSimulation::GetWarningFrameFraction() const
{
	return CVarSimulationWarningFrameFraction.GetValueOnAnyThread();
}

bool ASpeedPhysicsSimulation::StepCanonicalFrame(const FCanonicalFrameContext& Context)
{
    if(!AllowsNativeDriverExecution())return false;

    // Failure can occur before MarkTerminal exists (world lookup/admission).
    // Keep the previous completed pair during work, but retire current access
    // on every unsuccessful exit. Retained immutable historical reads survive.
    bool bCompletedPublicationFrameSucceeded = false;
    ON_SCOPE_EXIT
    {
        if (!bCompletedPublicationFrameSucceeded)
            PublishedCompletedWorldInputDriverStorage().store({}, std::memory_order_seq_cst);
    };
#if WITH_DEV_AUTOMATION_TESTS
    Speed::OwnedFrameAudit::FCanonicalPhaseScope OwnedPhases(Context.NumFrame);
#endif
#if !UE_BUILD_SHIPPING
    Speed::ParallelTiming::FFrameScope ParallelTimingScope(ParallelFrameTimingDriverStorage(), Context.NumFrame, Context.PhysicalDeltaTime);
#endif
	if (bCanonicalPublicationTerminalDriverStorage().Load()) return false;
	if (!SpeedWorldSubsystemDriverStorage())
	{
		SpeedWorldSubsystemDriverStorage() = GetSpeedWorldSubsystem(GetWorld());
		if (!SpeedWorldSubsystemDriverStorage())
		{
			return false;
		}
	}

	FString BindingFailure;
	bool bAdmitted = false;
	try { bAdmitted = SpeedWorldSubsystemDriverStorage()->BeginCanonicalFrame(BindingFailure); }
	catch (...)
	{
		bCanonicalPublicationTerminalDriverStorage().Store(true);
		PublishedCompletedWorldInputDriverStorage().store({}, std::memory_order_seq_cst);
		bOwnedWorkerTerminalDriverStorage().Store(true);
		SPEED_NATIVE_LOG(LogTemp, Error, TEXT("[CanonicalAdmissionExceptionTerminal] Frame=%llu"), Context.NumFrame);
		return false; // BeginCanonicalFrame sets its active flag only after validation returns.
	}
	if (!bAdmitted)
	{
		SPEED_NATIVE_LOG(LogTemp, Error, TEXT("[SimulationBindingRejected] frame=%llu %s"),
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
    const auto* ParallelTimingWorld = ParallelFrameTimingDriverStorage().IsActive()
        ? SpeedWorldSubsystemDriverStorage()->GetStaticCollisionWorld() : nullptr;
    if (ParallelFrameTimingDriverStorage().IsActive())
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
		SpeedWorldSubsystemDriverStorage()->EndCanonicalFrame();
	};
	const auto MarkTerminal = [this]()
	{
		PublishedCompletedWorldInputDriverStorage().store({}, std::memory_order_seq_cst);
		if (InputSessionRegistryDriverStorage()) InputSessionRegistryDriverStorage()->AbortFrame();
		PublishedInputFrameDriverStorage().store(std::shared_ptr<const Speed::Input::V2::FRegistryFrame>{}, std::memory_order_seq_cst);
		bCanonicalPublicationTerminalDriverStorage().Store(true);
		bOwnedWorkerTerminalDriverStorage().Store(true);
	};

#if !UE_BUILD_SHIPPING
        ParallelTimingScope.NextPhase(Speed::ParallelTiming::EFrameCostPhase::Input);
#endif
	const TCHAR* ExceptionStage = TEXT("InputRegistryRead");
	try
	{
		IAMSPEED_FRAME_SCOPE(Initialize);
		const auto InputView = InputSessionRegistryDriverStorage() ? InputSessionRegistryDriverStorage()->ReadRegistry() : nullptr;
#if WITH_DEV_AUTOMATION_TESTS
    OwnedPhases.Mark(Speed::OwnedFrameAudit::ECanonicalEdge::Registry);
#endif
		const bool bHasInputSessions = InputView && !InputView->Bindings.empty();
		if (bHasInputSessions)
		{
			ExceptionStage = TEXT("ScenarioInputs");
			if (!SpeedWorldSubsystemDriverStorage()->StageCanonicalScenarioInputs(Context, *InputSessionRegistryDriverStorage()))
				throw std::runtime_error("canonical scenario authoring rejected");
#if WITH_DEV_AUTOMATION_TESTS
    OwnedPhases.Mark(Speed::OwnedFrameAudit::ECanonicalEdge::ScenarioInputs);
#endif
			ExceptionStage = TEXT("InputPrepareFrame");
			if (!InputSessionRegistryDriverStorage()->PrepareFrame(Context.NumFrame))
			{
				const auto Failure = InputSessionRegistryDriverStorage()->ReadLastFrameFailure();
                SPEED_NATIVE_LOG(LogTemp, Error, TEXT("[CanonicalInputPollRejected] Frame=%llu Reason=%u Session=%llu Epoch=%llu OwnerPresent=%u OwnerStatusPresent=%u OwnerStatus=%u OwnerPhase=%u PollFailure=%u PollSerial=%llu RegistryVersion=%llu"),
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
			if (!InputSessionRegistryDriverStorage()->InstallAll())
			{
				const auto Failure = InputSessionRegistryDriverStorage()->ReadLastFrameFailure();
                SPEED_NATIVE_LOG(LogTemp, Error, TEXT("[CanonicalInputInstallRejected] Frame=%llu Reason=%u Session=%llu Epoch=%llu OwnerPresent=%u OwnerStatusPresent=%u OwnerStatus=%u OwnerPhase=%u PollFailure=%u PollSerial=%llu RegistryVersion=%llu"),
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
			const auto Installed = InputSessionRegistryDriverStorage()->ReadInstalled();
			if (!Installed || !SpeedWorldSubsystemDriverStorage()->InstallCanonicalInputs(*Installed) || !InputSessionRegistryDriverStorage()->BeginAll())
				throw std::runtime_error("canonical input installation rejected");
		}
#if WITH_DEV_AUTOMATION_TESTS
    OwnedPhases.Mark(Speed::OwnedFrameAudit::ECanonicalEdge::InputSnapshot);
#endif
#if !UE_BUILD_SHIPPING
        const auto WitnessInstalled=bHasInputSessions && InputSessionRegistryDriverStorage()?InputSessionRegistryDriverStorage()->ReadInstalled():nullptr;
        TUniquePtr<Speed::FFrameAppliedCommandWitnessScope> AppliedCommandWitness;
        TUniquePtr<Speed::FNativeFrameWitnessRecorder> FrameWitness;
        try { FrameWitness=Speed::FNativeFrameWitnessRecorder::Begin(
            Context.NumFrame,Context.PhysicalDeltaTime,WitnessInstalled.get(),*SpeedWorldSubsystemDriverStorage(),NativeWitnessCaseDriverStorage(),!bHasInputSessions); }
        catch(...) { SPEED_NATIVE_LOG(LogTemp,Error,TEXT("[FrameWitnessBeginUnknown] Frame=%llu"),Context.NumFrame); }
        if(FrameWitness)try { AppliedCommandWitness=MakeUnique<Speed::FFrameAppliedCommandWitnessScope>(
            Context.NumFrame,InputJournalDriverStorage().GetCommandsForFrame(Context.NumFrame).Num());
            SpeedWorldSubsystemDriverStorage()->DeclareExpectedFrameAnalogWitnessForTesting(); }
        catch(...) { SPEED_NATIVE_LOG(LogTemp,Error,TEXT("[FrameAppliedCommandBeginUnknown] Frame=%llu"),Context.NumFrame); }
#endif
		ExceptionStage = TEXT("StaticWorldAudit");
		Speed::Analytic::FStaticWorldQueryAudit::BeginFrame(
			Context.NumFrame, SpeedWorldSubsystemDriverStorage()->GetAnalyticWorldData(),
			SpeedWorldSubsystemDriverStorage());
		bAuditStarted = true;
#if WITH_DEV_AUTOMATION_TESTS
    OwnedPhases.Mark(Speed::OwnedFrameAudit::ECanonicalEdge::StaticAudit);
#endif
		ExceptionStage = TEXT("SimulationInputJournal");
		if (InputJournalDriverStorage().IsSealed() &&
			!SpeedWorldSubsystemDriverStorage()->ApplySimulationInputs(Context.NumFrame, InputJournalDriverStorage()))
		{
			if (!bInputConsumptionErrorReportedDriverStorage())
			{
				bInputConsumptionErrorReportedDriverStorage() = true;
				SPEED_NATIVE_LOG(LogTemp, Error,
					TEXT("[SimulationInputRejected] Frame=%llu JournalHash=%016llX"),
					Context.NumFrame, InputJournalDriverStorage().StableHash());
			}
			SpeedWorldSubsystemDriverStorage()->AbortCanonicalFrame(Context.NumFrame, ECanonicalFrameAbortReason::PreparationFailed);
			bOutcomeHandled = true;
			MarkTerminal();
			return false;
		}
#if WITH_DEV_AUTOMATION_TESTS
    OwnedPhases.Mark(Speed::OwnedFrameAudit::ECanonicalEdge::InputJournalDriverStorage());
#endif
		ExceptionStage = TEXT("CanonicalInputPrepare");
		if (!SpeedWorldSubsystemDriverStorage()->PrepareCanonicalInputs(Context))
		{
			SpeedWorldSubsystemDriverStorage()->AbortCanonicalFrame(Context.NumFrame, ECanonicalFrameAbortReason::PreparationFailed);
			bOutcomeHandled = true;
			MarkTerminal();
			SPEED_NATIVE_LOG(LogTemp, Error, TEXT("[CanonicalInputPreparationRejected] Frame=%llu"), Context.NumFrame);
			return false;
		}
#if WITH_DEV_AUTOMATION_TESTS
    OwnedPhases.Mark(Speed::OwnedFrameAudit::ECanonicalEdge::CanonicalInputs);
#endif
		bInputConsumptionErrorReportedDriverStorage() = false;

#if !UE_BUILD_SHIPPING
        ParallelTimingScope.NextPhase(Speed::ParallelTiming::EFrameCostPhase::Prepare);
#endif
		IAMSPEED_FRAME_PHASE(Prepare);
		ExceptionStage = TEXT("WorldPrepare");
#if !UE_BUILD_SHIPPING
        {
            Speed::FCanonicalEpisodeTestingScope PrepareOwnerScope(Context);
            SpeedWorldSubsystemDriverStorage()->PrepareCanonicalFrame(Context);
        }
#else
        SpeedWorldSubsystemDriverStorage()->PrepareCanonicalFrame(Context);
#endif
#if WITH_DEV_AUTOMATION_TESTS
    OwnedPhases.Mark(Speed::OwnedFrameAudit::ECanonicalEdge::WorldPrepare);
#endif

#if !UE_BUILD_SHIPPING
        ParallelTimingScope.NextPhase(Speed::ParallelTiming::EFrameCostPhase::WorldStep);
#endif
		IAMSPEED_FRAME_PHASE(Core);
		ExceptionStage = TEXT("WorldStep");
#if WITH_DEV_AUTOMATION_TESTS
    const uint64 OwnedDiagnosticSerialBefore = OwnedPhases.CapturesPhysicalDetail()
        ? SpeedWorldSubsystemDriverStorage()->GetLastStepDiagnostics().Serial : 0;
#endif
		SpeedWorldSubsystemDriverStorage()->Step(
			Context.PhysicalDeltaTime,
			Context.SimTime,
			static_cast<unsigned int>(Context.NumFrame));
#if WITH_DEV_AUTOMATION_TESTS
    OwnedPhases.Mark(Speed::OwnedFrameAudit::ECanonicalEdge::WorldStep);
    if (OwnedPhases.CapturesPhysicalDetail())
        OwnedPhases.CopyPhysicalDetail(SpeedWorldSubsystemDriverStorage()->GetLastStepDiagnostics(), OwnedDiagnosticSerialBefore);
#endif

#if !UE_BUILD_SHIPPING
        ParallelTimingScope.NextPhase(Speed::ParallelTiming::EFrameCostPhase::Snapshot);
#endif
		IAMSPEED_FRAME_PHASE(Snapshot);
		ExceptionStage = TEXT("SnapshotCapture");
		FSimulationSnapshot Snapshot = SpeedWorldSubsystemDriverStorage()->CaptureSimulationSnapshot(
			Context.NumFrame, InputJournalDriverStorage().StableHash(), bPublishPresentationDriverStorage());
#if WITH_DEV_AUTOMATION_TESTS
    OwnedPhases.Mark(Speed::OwnedFrameAudit::ECanonicalEdge::Snapshot);
#endif

#if !UE_BUILD_SHIPPING
        ParallelTimingScope.NextPhase(Speed::ParallelTiming::EFrameCostPhase::Presentation);
#endif
		ExceptionStage = TEXT("PresentationProducerList");
		TArray<TSharedRef<ISimulationPresentationProducer, ESPMode::ThreadSafe>> Producers;
		{
			FScopeLock Lock(&PresentationProducerMutexDriverStorage());
			Producers = PresentationProducersDriverStorage();
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
            bHasInputSessions, [&] { return !bHasInputSessions || InputSessionRegistryDriverStorage()->ValidateComplete(); },
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
            bHasInputSessions, [&] { return !bHasInputSessions || InputSessionRegistryDriverStorage()->ValidateComplete(); });
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
        // Reserve/copy all common-view storage before either irreversible commit.
        // The captured registry/Installed endpoint belongs to THIS canonical frame.
        std::shared_ptr<Speed::FCompletedSimulationPublication> CompletedDraft;
        if (bPublishCompletedWorldInputDriverStorage())
        {
            ExceptionStage = TEXT("CompletedWorldInputPrepare");
            const auto Installed = bHasInputSessions ? InputSessionRegistryDriverStorage()->ReadInstalled() : nullptr;
            // ReadRegistry() produces an observation copy. Bind the common view to
            // the exact registry retained by PrepareFrame, never a later sample.
            const auto CapturedRegistry = Installed ? Installed->Registry : InputView;
            if (!Speed::FCompletedSimulationPublication::CanPrepare(Snapshot, CapturedRegistry, Installed))
                throw std::runtime_error("completed world/input preparation rejected");
            CompletedDraft.reset(new Speed::FCompletedSimulationPublication(Snapshot, CapturedRegistry, Installed));
        }
        const auto PublishCompletedWorldInput = [&](const std::shared_ptr<const Speed::Input::V2::FRegistryFrame>& Completed)
        {
            if (!CompletedDraft) return true;
            if (!CompletedDraft->MatchesCompleted(Completed)
                || !CompletedDraft->BindPublicationSerial(SnapshotBufferDriverStorage().PublishedSerial())) return false;
            // No allocation, callbacks or separate endpoint loads in this store.
            // Moving consumes the only mutable alias; readers retain one object.
            std::shared_ptr<const Speed::FCompletedSimulationPublication> Sealed = std::move(CompletedDraft);
            PublishedCompletedWorldInputDriverStorage().store(std::move(Sealed), std::memory_order_seq_cst);
            return true;
        };
		IAMSPEED_FRAME_PHASE(Publish);
		ExceptionStage = TEXT("CanonicalPublish");
		const ECanonicalPublicationResult Publication = SpeedWorldSubsystemDriverStorage()->PublishCanonicalFrame(Context.NumFrame, [&]()
		{
			// One simulation owner. Avoid serial wrap before touching the inactive slot.
			if (SnapshotBufferDriverStorage().PublishedSerial() == MAX_uint64) return false;
#if !UE_BUILD_SHIPPING
            // Replay publications do not represent paced real-time starts.
            return Context.IsResimulation() ? SnapshotBufferDriverStorage().Publish(Snapshot) :
                SnapshotBufferDriverStorage().PublishWithCadenceTiming(Snapshot, GetCanonicalPublicationTiming(Context));
#else
            return SnapshotBufferDriverStorage().Publish(Snapshot);
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
			SPEED_NATIVE_LOG(LogTemp, Error, TEXT("[CanonicalPublicationTerminal] Frame=%llu Result=%u GlobalPublished=%d"),
				Context.NumFrame, static_cast<uint32>(Publication), bGlobalPublished ? 1 : 0);
			return false;
		}

#if !UE_BUILD_SHIPPING
        ParallelTimingScope.NextPhase(Speed::ParallelTiming::EFrameCostPhase::PostCommit);
#endif
		if (bHasInputSessions)
		{
			ExceptionStage = TEXT("InputComplete");
			if (!InputSessionRegistryDriverStorage()->CompleteAll())
			{
				MarkTerminal();
				return false;
			}
			const auto Completed = InputSessionRegistryDriverStorage()->ReadLatest();
            // A failed/mismatched completion never produces a common publication.
            if (!Completed || Completed->Frame != Context.NumFrame || !PublishCompletedWorldInput(Completed))
            {
                MarkTerminal();
                return false;
            }
			PublishedInputFrameDriverStorage().store(Completed, std::memory_order_seq_cst);
			for (const auto& Input : Completed->Inputs)
			{
				const auto Observation = InputObservationsDriverStorage().find(Input.Session);
				if (Observation != InputObservationsDriverStorage().end())
				{
					try { if (!Observation->second->Publish(Input.Snapshot)) Observation->second->Deactivate(); }
					catch (...) { SPEED_NATIVE_LOG(LogTemp, Error, TEXT("[InputObservationPublicationFailed] Frame=%llu"), Context.NumFrame); }
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
		SpeedWorldSubsystemDriverStorage()->NotifyCanonicalFramePublished(Context.NumFrame);
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
			CameraSample.PublicationSerial = SnapshotBufferDriverStorage().PublishedSerial();
			CameraSample.StateHash = Snapshot.StateHash;
			CameraSample.InputJournalHash = Snapshot.InputJournalHash;
			CameraSampleBufferDriverStorage().Publish(CameraSample);
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
                SPEED_NATIVE_LOG(LogTemp,Error,TEXT("[FrameAppliedCommandSaveUnknown] Frame=%llu"),Context.NumFrame);
            const auto WitnessCompleted=bHasInputSessions && InputSessionRegistryDriverStorage()?InputSessionRegistryDriverStorage()->ReadLatest():nullptr;
            FrameWitness->Finish(WitnessCompleted.get(),Snapshot.Payload,
                SpeedWorldSubsystemDriverStorage()->GetLastStepDiagnostics().ResolvedEventCount,WitnessCamera,true,!bHasInputSessions);
        }
        catch(...) { SPEED_NATIVE_LOG(LogTemp,Error,TEXT("[FrameWitnessFinishUnknown] Frame=%llu"),Context.NumFrame); }
#endif

#if !UE_BUILD_SHIPPING
        ParallelTimingScope.NextPhase(Speed::ParallelTiming::EFrameCostPhase::Finalize);
#endif
		IAMSPEED_FRAME_PHASE(Journal);
		ExceptionStage = TEXT("FrameHash");
		FrameHashesDriverStorage().Append(Context.NumFrame, Snapshot.StateHash);
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
		bCompletedPublicationFrameSucceeded = true;
		return true;
	}
	catch (const std::exception& Exception)
	{
		if (!bGlobalPublished && !bOutcomeHandled)
			SpeedWorldSubsystemDriverStorage()->AbortCanonicalFrame(Context.NumFrame, ECanonicalFrameAbortReason::PreparationFailed);
		MarkTerminal();
		SPEED_NATIVE_LOG(LogTemp, Error, TEXT("[CanonicalFrameExceptionTerminal] Frame=%llu GlobalPublished=%d Stage=%s Type=std Message=%s"),
			Context.NumFrame, bGlobalPublished ? 1 : 0, ExceptionStage, UTF8_TO_TCHAR(Exception.what()));
		return false; // No exception escapes into the UE worker/callback boundary.
	}
	catch (...)
	{
		if (!bGlobalPublished && !bOutcomeHandled)
			SpeedWorldSubsystemDriverStorage()->AbortCanonicalFrame(Context.NumFrame, ECanonicalFrameAbortReason::PreparationFailed);
		MarkTerminal();
		SPEED_NATIVE_LOG(LogTemp, Error, TEXT("[CanonicalFrameExceptionTerminal] Frame=%llu GlobalPublished=%d Stage=%s Type=unknown"),
			Context.NumFrame, bGlobalPublished ? 1 : 0, ExceptionStage);
		return false; // No exception escapes into the UE worker/callback boundary.
	}
}

bool ASpeedPhysicsSimulation::RequestRollbackAndResimulation(
	const FSimulationSnapshot& Snapshot,
	const uint64 TargetFrameInclusive)
{
    if (!AllowsLegacySnapshotRollback()) return false;
	if (!Speed::CanonicalFrameDriver::IsEnabled() || !InputJournalDriverStorage().IsSealed() ||
		Snapshot.InputJournalHash != InputJournalDriverStorage().StableHash() ||
		Snapshot.NumFrame > TargetFrameInclusive ||
		TargetFrameInclusive >= TNumericLimits<unsigned int>::Max())
	{
		return false;
	}
	// Registry replay is not a world/actor restore transaction yet. Reject
	// before queuing instead of rewinding physics underneath live owners.
	const auto InputView = ReadInputRegistryView();
	if (InputView && !InputView->Bindings.empty()) return false;
	FScopeLock Lock(&RollbackRequestMutexDriverStorage());
	if (PendingRollbackRequestDriverStorage().IsSet())
	{
		return false;
	}
	PendingRollbackRequestDriverStorage().Emplace(FPendingRollbackRequest{
		Snapshot, TargetFrameInclusive });
	return true;
}

bool ASpeedPhysicsSimulation::ProcessPendingRollbackRequest()
{
	TOptional<FPendingRollbackRequest> Request;
	{
		FScopeLock Lock(&RollbackRequestMutexDriverStorage());
		if (!PendingRollbackRequestDriverStorage().IsSet())
		{
			return true;
		}
#if !UE_BUILD_SHIPPING
        IndependentFastCoverage276DriverStorage().RollbackDequeued();
#endif
		Request = MoveTemp(PendingRollbackRequestDriverStorage());
		PendingRollbackRequestDriverStorage().Reset();
	}

	// Legacy restore is not a completed world/input transaction. Invalidate the
	// optional common view before any restore mutation; retained reads are historical.
	PublishedCompletedWorldInputDriverStorage().store({}, std::memory_order_seq_cst);
	// Recheck on the worker: an input binding may have arrived since admission.
	const auto InputView = ReadInputRegistryView();
	if (InputView && !InputView->Bindings.empty()) return false;
	TArray<TSharedRef<ISimulationPresentationProducer, ESPMode::ThreadSafe>> RestoreProducers;
	{
		FScopeLock Lock(&PresentationProducerMutexDriverStorage());
		RestoreProducers = PresentationProducersDriverStorage();
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
	if (!SpeedWorldSubsystemDriverStorage() ||
		!SpeedWorldSubsystemDriverStorage()->RestoreSimulationSnapshot(
			Request->Snapshot, InputJournalDriverStorage().StableHash()))
	{
		SPEED_NATIVE_LOG(LogTemp, Error,
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
	FrameHashesDriverStorage().RemoveFrom(Request->Snapshot.NumFrame);
	if (!FrameHashesDriverStorage().Append(
			Request->Snapshot.NumFrame, Request->Snapshot.StateHash) ||
		!SnapshotBufferDriverStorage().Publish(Request->Snapshot))
	{
		SPEED_NATIVE_LOG(LogTemp, Error,
			TEXT("[SimulationSnapshotRepublishFailed] Frame=%llu"),
			Request->Snapshot.NumFrame);
		return false;
	}

	CanonicalNumFrameDriverStorage() = Request->Snapshot.NumFrame + 1u;
	_NumFrameDriverStorage() = static_cast<unsigned int>(CanonicalNumFrameDriverStorage());
	const uint64 ReplayFrameCount =
		Request->TargetFrameInclusive - Request->Snapshot.NumFrame;
	for (uint64 ReplayIndex = 0; ReplayIndex < ReplayFrameCount; ++ReplayIndex)
	{
		const double PhysicalFrameStartSeconds = FPlatformTime::Seconds();
        FCanonicalFrameContext ReplayContext(CanonicalNumFrameDriverStorage());
        ReplayContext.Purpose=ECanonicalFramePurpose::Resimulation;
#if WITH_DEV_AUTOMATION_TESTS
        const auto OwnedReplayStep = Speed::OwnedFrameAudit::BeginStep(ReplayContext.NumFrame, true);
#endif
        const bool ReplayStepSucceeded = StepCanonicalFrame(ReplayContext);
#if WITH_DEV_AUTOMATION_TESTS
        Speed::OwnedFrameAudit::EndStep(OwnedReplayStep, true, ReplayStepSucceeded);
#endif
        if (!ReplayStepSucceeded)
		{
			SPEED_NATIVE_LOG(LogTemp, Error,
				TEXT("[SimulationResimulationFailed] Frame=%llu TargetFrame=%llu"),
				CanonicalNumFrameDriverStorage(), Request->TargetFrameInclusive);
			return false;
		}
		RecordStepPerformance(
			(FPlatformTime::Seconds() - PhysicalFrameStartSeconds) * 1000.0);
		++CanonicalNumFrameDriverStorage();
		_NumFrameDriverStorage() = static_cast<unsigned int>(CanonicalNumFrameDriverStorage());
	}
	OnCanonicalTimelineRestored();
	return true;
}


void ASpeedPhysicsSimulation::SetReplaySink(TSharedPtr<ISimulationReplaySink, ESPMode::ThreadSafe> Sink)
{
    check(IsInGameThread());
    {
        FScopeLock Lock(&ReplaySinkMutexDriverStorage());
        Swap(ReplaySinkDriverStorage(),Sink);
        bReplayCaptureRequestedDriverStorage().Store(ReplaySinkDriverStorage().IsValid());
    }
    // Any old sink is released outside the publication lock, after in-flight refs.
}

bool ASpeedPhysicsSimulation::TryReplaceReplaySink(const ISimulationReplaySink* Expected,
    TSharedPtr<ISimulationReplaySink, ESPMode::ThreadSafe> Replacement)
{
    check(IsInGameThread());
    {
        FScopeLock Lock(&ReplaySinkMutexDriverStorage());
        if(ReplaySinkDriverStorage().Get()!=Expected)return false;
        Swap(ReplaySinkDriverStorage(),Replacement);
        bReplayCaptureRequestedDriverStorage().Store(ReplaySinkDriverStorage().IsValid());
    }
    return true; // Replacement retains the old sink until after lock release.
}

void ASpeedPhysicsSimulation::ServiceReplayCapture(const FSimulationSnapshot& Snapshot) noexcept
{
    if (!bReplayCaptureRequestedDriverStorage().Load()) return;
    TSharedPtr<ISimulationReplaySink, ESPMode::ThreadSafe> Sink;
    {
        FScopeLock Lock(&ReplaySinkMutexDriverStorage()); Sink=ReplaySinkDriverStorage();
    }
    if (!Sink.IsValid() || !Sink->IsCapturing()) return;
    uint64 UnsupportedStableId=0;
    try
    {
        FSimulationReplayFrame Frame;
        if (SpeedWorldSubsystemDriverStorage()->CaptureReplayStateFields(Frame.Records,UnsupportedStableId))
        {
            Frame.Snapshot=Snapshot;
            Frame.Snapshot.PublicationSerial=SnapshotBufferDriverStorage().PublishedSerial();
            Sink->Consume(MoveTemp(Frame));
            return;
        }
    }
    catch (...) { /* Allocation/schema failure affects recording only. */ }
    Sink->Failed(Snapshot.NumFrame,UnsupportedStableId);
    {
        FScopeLock Lock(&ReplaySinkMutexDriverStorage());
        // Do not disable a newer GT registration while retiring this generation.
        if (ReplaySinkDriverStorage()==Sink) {ReplaySinkDriverStorage().Reset();bReplayCaptureRequestedDriverStorage().Store(false);}
    }
}
