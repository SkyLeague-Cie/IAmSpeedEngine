// Fill out your copyright notice in the Description page of Project Settings.

// USpeedWorldSubsystem.cpp

#include "SpeedWorldSubsystem.h"
#include "HAL/CriticalSection.h"
#include "Misc/ScopeLock.h"
#include <atomic>
#include <exception>
#include "Misc/CoreDelegates.h"
#include "HAL/PlatformTime.h"
#include "HAL/IConsoleManager.h"
#include "IAmSpeed/World/Collision/NativePreparationPrefix.h"
#include "IAmSpeed/World/Collision/PreparedPhysicalExecution.h"
#include "IAmSpeed/World/Collision/PreparedNativeFrameConsumer.h"
#include "IAmSpeed/World/Collision/PreparedNativeWheelProbe.h"
#include "IAmSpeed/World/Collision/PreparedSphereRsTrajectoryCapture.h"
#include "IAmSpeed/World/Simulation/OnePausedCanonicalWork.h"
#include "IAmSpeed/World/Collision/PreparedSweepEvidence.h"
#include "IAmSpeed/World/Collision/PreparedIntervalStaticWorld.h"
#include "IAmSpeed/World/Analytic/SpeedAnalyticContactComponent.h"
#include "IAmSpeed/World/Simulation/CcdPoseAudit.h"
#include "IAmSpeed/World/Simulation/SubBodyKinematicsTasks.h"
#include "IAmSpeed/World/Simulation/JoinedActorKinematics1213.h"
#include "IAmSpeed/World/Simulation/FrameParallelSettings.h"
#include "IAmSpeed/World/Analytic/AnalyticCaseSatDiagnostics.h"
#include "IAmSpeed/World/ReplayInertWorld.h"
#include "IAmSpeed/Input/InputSessionRegistry.h"
#include "IAmSpeed/Actors/SpeedStaticActor.h"
#include "IAmSpeed/Components/ISpeedComponent.h"
#include "IAmSpeed/World/Simulation/CanonicalFrameContext.h"
#include "IAmSpeed/World/Simulation/SpeedPhysicsSimulation.h"
#include "IAmSpeed/World/Simulation/SimulationActorDiagnostics.h"
#include "IAmSpeed/World/Collision/ResolvedPairSet.h"
#include "IAmSpeed/World/Collision/EarliestCollisionBatch.h"
#include "IAmSpeed/World/Simulation/FrameSolveWitness.h"
#include "IAmSpeed/World/Simulation/CollisionMatrixObservationForTesting.h"
#include "IAmSpeed/World/Simulation/FrameAppliedCommandWitness.h"
#include "IAmSpeed/World/Analytic/AnalyticLandscapeAdapter.h"
#include "IAmSpeed/World/Analytic/SpeedAnalyticCollisionAsset.h"
#include "IAmSpeed/World/Analytic/SpeedAnalyticSourceComponent.h"
#include "IAmSpeed/World/Analytic/StaticWorldQueryAudit.h"
#include "IAmSpeed/SubBodies/Solid/BoxSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SolidSubBody.h"
#include "IAmSpeed/SubBodies/Solid/RingSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SphereSubBody.h"
#include "GameFramework/Actor.h"
#include "Engine/World.h"
#include "PhysicsEngine/BodyInstance.h"
#include "EngineUtils.h"
#include "LandscapeProxy.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "TimerManager.h"
#include "Algo/Sort.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformTime.h"
#include "Misc/PackageName.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/FileManager.h"
#if !UE_BUILD_SHIPPING
#include "Misc/ScopeExit.h"
#endif



#if !UE_BUILD_SHIPPING
namespace RoofPostProfile
{
static TAutoConsoleVariable<int32> Enabled(TEXT("p.IAmSpeed.Simulation.RoofPostProfile"),0,
    TEXT("Observation only. Aggregate exact production roof fixture PostOwner stages."));
enum EStage { Begin, Census, Callback, End, Dispatch, StageCount };
struct FRow { std::atomic<uint64> Calls{0}, Ticks[StageCount]{}; };
static FRow Rows[2];
static std::atomic<bool> Valid{true};
thread_local uint64* Current=nullptr;

static TAutoConsoleVariable<int32> CallbackEnabled(TEXT("p.IAmSpeed.Simulation.RoofPostCallbackProfile"),0,
    TEXT("Observation only. Subdivide the exact roof Fox PostOwner callback."));
struct FCallbackRecord { uint64 Ticks[4]{}; uint32 Calls[4]{}; };
thread_local FCallbackRecord* CallbackCurrent=nullptr;
struct FSample;
thread_local const FSample* SupportCurrent=nullptr;
bool SupportReuseRequested();
static std::atomic<uint64> CallbackCalls[4]{},CallbackTicks[4]{};
static std::atomic<bool> CallbackValid{true};

void ObserveCallbackStage(uint32 stage,uint64 start)
{
    if(!CallbackCurrent)return;
    const uint64 end=FPlatformTime::Cycles64();
    auto& current=*CallbackCurrent;
    if(stage>=4||end<start||current.Calls[stage]==MAX_uint32||current.Ticks[stage]>MAX_uint64-(end-start))
    {CallbackValid.store(false);return;}
    ++current.Calls[stage];current.Ticks[stage]+=end-start;
}

static bool Fixture()
{
    FString cell;
    const TCHAR* command=FCommandLine::Get();
    return FParse::Param(command,TEXT("SLStadiumNativeFastBatch"))&&FParse::Param(command,TEXT("SLTestProductionFox"))&&
        FParse::Value(command,TEXT("SLStadiumMatrixCell="),cell)&&FPaths::GetCleanFilename(cell)==TEXT("c202-production-roof-flat.json");
}
static void Publish()
{
    for(int32 i=0;i<2;++i)
    {
        const auto& row=Rows[i];
        if(!row.Calls.load())continue;
        UE_LOG(LogTemp,Display,TEXT("[RoofPostProfile] Id=%d Calls=%llu Integrity=%d SecondsPerCycle=%.17g Begin=%llu Census=%llu Callback=%llu End=%llu Dispatch=%llu"),
            i?3:1,row.Calls.load(),int32(Valid.load()),FPlatformTime::GetSecondsPerCycle64(),
            row.Ticks[Begin].load(),row.Ticks[Census].load(),row.Ticks[Callback].load(),row.Ticks[End].load(),row.Ticks[Dispatch].load());
    }

    if(Rows[1].Calls.load()&&CallbackCalls[0].load())
    {
        UE_LOG(LogTemp,Display,TEXT("[RoofPostCallbackProfile] Id=3 Integrity=%d SecondsPerCycle=%.17g CorrectionCalls=%llu CorrectionTicks=%llu QuantizeCalls=%llu QuantizeTicks=%llu RecordCalls=%llu RecordTicks=%llu BoxSupportCalls=%llu BoxSupportTicks=%llu"),
            int32(CallbackValid.load()),FPlatformTime::GetSecondsPerCycle64(),
            CallbackCalls[0].load(),CallbackTicks[0].load(),CallbackCalls[1].load(),CallbackTicks[1].load(),
            CallbackCalls[2].load(),CallbackTicks[2].load(),CallbackCalls[3].load(),CallbackTicks[3].load());
    }
}
struct FRegister { FRegister(){FCoreDelegates::OnPreExit.AddStatic(&Publish);} };
static FRegister Register;
struct FSample
{
    uint64 Marks[7]{};
    FCallbackRecord Callback;
    bool ProfileCallback=false;
    bool ScopeSupport=false;
    int32 Row=INDEX_NONE;
    explicit FSample(const ISpeedComponent& owner)
    {
        const bool Profile = Enabled.GetValueOnAnyThread()==1;
        const bool Reuse = SupportReuseRequested();
        if (!Profile && !Reuse) return;
        static const bool IsRoofFixture = Fixture();
        if (!IsRoofFixture) return;
        const uint64 Id = owner.GetPublishedSimulationStableId();
        if (Id!=1 && Id!=3) return;
        if (Current || SupportCurrent)
        {
            if (Profile) Valid.store(false);
            return;
        }
        static const bool IsOffscreen = FParse::Param(FCommandLine::Get(),TEXT("NullRHI")) &&
            FParse::Param(FCommandLine::Get(),TEXT("unattended"));
        ScopeSupport = Reuse && IsOffscreen && Id==3;
        if (!Profile) return;
        ProfileCallback=Id==3&&CallbackEnabled.GetValueOnAnyThread()==1;
        Row=Id==1?0:1; Marks[0]=FPlatformTime::Cycles64();
    }
    ~FSample()
    {
        if (SupportCurrent==this) SupportCurrent=nullptr;
        if (Current==Marks) Current=nullptr;
        if (CallbackCurrent==&Callback) CallbackCurrent=nullptr;
    }
    void BeginReturned()
    {
        if (ScopeSupport) SupportCurrent=this;
        if(Row==INDEX_NONE)return;
        Marks[1]=FPlatformTime::Cycles64();Current=Marks;
        if(ProfileCallback)CallbackCurrent=&Callback;
    }
    void DispatchReturned()
    {
        if (SupportCurrent==this) SupportCurrent=nullptr;
        if(Row==INDEX_NONE)return;
        Current=nullptr;Marks[5]=FPlatformTime::Cycles64();
        if(ProfileCallback)CallbackCurrent=nullptr;
    }
    void EndReturned()
    {
        if(Row==INDEX_NONE)return;
        Marks[6]=FPlatformTime::Cycles64();
        for(int32 i=1;i<7;++i)
            if(!Marks[i]||Marks[i]<Marks[i-1]){Valid.store(false);return;}
        const uint64 ticks[]={Marks[1]-Marks[0],Marks[3]-Marks[2],Marks[4]-Marks[3],Marks[6]-Marks[5],Marks[5]-Marks[1]};
        auto& row=Rows[Row];
        for(int32 i=0;i<StageCount;++i)
            if(row.Ticks[i].fetch_add(ticks[i])>MAX_uint64-ticks[i])Valid.store(false);
        if(row.Calls.fetch_add(1)==MAX_uint64)Valid.store(false);
        if(ProfileCallback)
            for(uint32 i=0;i<4;++i)
            {
                if(CallbackCalls[i].fetch_add(Callback.Calls[i])>MAX_uint64-Callback.Calls[i])CallbackValid.store(false);
                if(CallbackTicks[i].fetch_add(Callback.Ticks[i])>MAX_uint64-Callback.Ticks[i])CallbackValid.store(false);
            }
    }
};
}
#endif

namespace StadiumClosedStepProfile70
{
struct FRecord {uint64 Bits[9]{};uint32 Queries=0,Attempts=0,Legacy=0;int32 Events=0;};
std::atomic<uint64> Calls{0},Finished{0},InFlight{0};std::atomic<bool> Integrity{true},ObservedEnabled{false};
FCriticalSection CaptureLock;TArray<FRecord> Records;
bool Enabled(){static const IConsoleVariable* cvar=IConsoleManager::Get().FindConsoleVariable(TEXT("p.IAmSpeed.AnalyticWorld.PhaseTiming"));return cvar&&cvar->GetInt()!=0;}
void Add(std::atomic<uint64>& field,uint64 n){if(field.fetch_add(n)>MAX_uint64-n)Integrity.store(false);}
void Observe(const FSpeedStepDiagnostics& d)
{
 if(!Enabled())return;ObservedEnabled.store(true);Add(Calls,1);Add(InFlight,1);
 // Worker capture is integer/object-byte only. No new FP sum, conversion,
 // timestamp, rounding-control operation or sticky-flag repair occurs here.
 const double* values[]={&d.TotalMilliseconds,&d.ResetMilliseconds,&d.IntegrateMilliseconds,&d.SweepMilliseconds,&d.ResolveMilliseconds,&d.ProjectionMilliseconds,&d.PostMilliseconds,&d.StaticAuthorityMilliseconds,&d.PhysicalDeltaTimeMilliseconds};
 FRecord record;for(uint32 i=0;i<9;++i){FMemory::Memcpy(&record.Bits[i],values[i],sizeof(uint64));const uint64 magnitude=record.Bits[i]&0x7fffffffffffffffULL;if(magnitude>=0x7ff0000000000000ULL||((record.Bits[i]>>63)&&magnitude))Integrity.store(false);}
 record.Queries=d.StaticQueryCount;record.Attempts=d.AuthorityAttemptCount;record.Legacy=d.LegacySweepCount;record.Events=d.ResolvedEventCount;if(record.Events<0)Integrity.store(false);
 {FScopeLock lock(&CaptureLock);if(Records.Num()>=65536)Integrity.store(false);else Records.Add(record);}
 Add(Finished,1);if(InFlight.fetch_sub(1)==0)Integrity.store(false);
}
void Publish()
{
 const uint64 calls=Calls.load(),flight=InFlight.load(),finished=Finished.load();TArray<FRecord> records;{FScopeLock lock(&CaptureLock);records=Records;}
 bool coherent=Integrity.load()&&flight==0&&InFlight.load()==0&&calls==Calls.load()&&calls==finished&&calls==uint64(records.Num());
 // FP aggregation is on the PreExit publication thread, outside Observe.
 // This program prefix does not prove terminal closure of physical queries.
 double sums[9]{};uint64 queries=0,attempts=0,legacy=0,events=0;
 const auto sumCount=[&](uint64& total,uint64 value){if(total>MAX_uint64-value){coherent=false;return;}total+=value;};
 for(const auto& record:records)
 {
  for(uint32 i=0;i<9;++i){double value=0;FMemory::Memcpy(&value,&record.Bits[i],sizeof(value));if(!FMath::IsFinite(value)||value<0){coherent=false;continue;}const double after=sums[i]+value;if(!FMath::IsFinite(after)){coherent=false;continue;}sums[i]=after;}
  sumCount(queries,record.Queries);sumCount(attempts,record.Attempts);sumCount(legacy,record.Legacy);if(record.Events<0)coherent=false;else sumCount(events,uint64(record.Events));
 }
 UE_LOG(LogTemp,Display,TEXT("[ClosedStepPhasesProfile70] Scope=ProgramPrefixAtPreExit PerCase=0 TerminalQueryJoin=0 NoPASS=1 IncludesWarmup=1 LaterFrameFailurePossible=1 InclusivePhasesNotAdditive=1 AlreadyAcquiredDiagnostics=1 IntegerWorkerCapture=1 AggregatedAtPreExit=1 RawRecordLimit=65536 ResolveScope=WorldCCDBatchNotAllSupportCorrections Enabled=%d SnapshotCoherent=%d ClosedStepSites=%llu Finished=%llu InFlight=%llu CapturedRecords=%d TotalMs=%.9f ResetMs=%.9f IntegrateMs=%.9f SweepMs=%.9f ResolveMs=%.9f ProjectionMs=%.9f PostMs=%.9f StaticAuthorityMs=%.9f PhysicalDeltaSumMs=%.9f StaticQueries=%llu AuthorityAttempts=%llu LegacySweeps=%llu ResolvedCCDEvents=%llu"),int32(ObservedEnabled.load()),int32(coherent),calls,finished,flight,records.Num(),sums[0],sums[1],sums[2],sums[3],sums[4],sums[5],sums[6],sums[7],sums[8],queries,attempts,legacy,events);
}
struct FRegister {FRegister(){FCoreDelegates::OnPreExit.AddStatic(&Publish);}};FRegister Register;
}

static TAutoConsoleVariable<int32> CVarIAmSpeedPersistentDynamicPairs(
	TEXT("p.IAmSpeed.Collision.PersistentDynamicPairs"),
	1,
	TEXT("Enables finite-mass persistent constraints for opted-in dynamic body pairs."));

static TAutoConsoleVariable<int32> CVarIAmSpeedExactBoxRestingSupport(
	TEXT("p.IAmSpeed.Collision.ExactBoxRestingSupport"),
	1,
	TEXT("Certify exact planar box support before CCD/integration. Requires strict SurfaceAnalytic; set to 0 for the legacy support comparison path."));

static TAutoConsoleVariable<int32> CVarIAmSpeedUnilateralRollingPairs(
	TEXT("p.IAmSpeed.Collision.UnilateralRollingPairs"),
	0,
	TEXT("Experimental: maintains opted-in sphere/box contacts from current geometry as unilateral rolling manifolds."));

static TAutoConsoleVariable<float> CVarIAmSpeedRollingSeparationTolerance(
	TEXT("p.IAmSpeed.Collision.RollingSeparationToleranceCm"),
	0.25f,
	TEXT("Maximum positive sphere/box gap retained only to preserve experimental rolling-manifold identity."));

static TAutoConsoleVariable<float> CVarIAmSpeedRollingGentleAcquisitionSpeed(
	TEXT("p.IAmSpeed.Collision.RollingGentleAcquisitionSpeedCmS"),
	5.0f,
	TEXT("Maximum pre-impact normal closing speed that acquires the ordinary gentle rolling manifold without changing impact response."));

static TAutoConsoleVariable<float> CVarIAmSpeedRollingPenetrationSlop(
	TEXT("p.IAmSpeed.Collision.RollingPenetrationSlopCm"),
	0.02f,
	TEXT("Penetration ignored by rolling-manifold velocity stabilization."));

static TAutoConsoleVariable<float> CVarIAmSpeedAnalyticLandscapeFlatnessTolerance(
	TEXT("p.IAmSpeed.AnalyticWorld.Landscape.FlatnessToleranceCm"),
	0.001f,
	TEXT("Maximum full-source height residual accepted by the shadow-only flat Landscape adapter."));

#if !UE_BUILD_SHIPPING
static TAutoConsoleVariable<int32> CVarIAmSpeedSolverEventTraceFrame(
	TEXT("p.IAmSpeed.Collision.SolverEventTraceFrame"),
	-1,
	TEXT("Physical frame for observation-only deterministic solver candidate and resolution tracing."));
#endif

namespace
{
	// The ordinary pause ACK only excludes physical frames. Adapter access from
	// the game thread needs the stronger ACK that also parks ServiceBoundary.
	class FScopedGameThreadAdapterAccess
	{
	public:
		explicit FScopedGameThreadAdapterAccess(UWorld* World)
		{
			if (!IsInGameThread() || !World) return;
			for (TActorIterator<ASpeedPhysicsSimulation> It(World); It; ++It)
			{
				Driver = *It;
				break;
			}
			if (!Driver || !Driver->IsOwnedWorkerExecutionMode()) return;
			bWasPaused = Driver->IsOwnedSimulationPaused();
			const auto Result = Driver->TrySuspendOwnedBoundaryService(1000);
			if (Result == ESimulationQuiescence::BoundaryAcknowledged)
				bOwnsSuspension = true;
			else if (Result != ESimulationQuiescence::AlreadyStopped)
				bValid = false;
		}
		~FScopedGameThreadAdapterAccess()
		{
			if (bOwnsSuspension && Driver)
			{
				Driver->ResumeOwnedBoundaryService();
				if (!bWasPaused) Driver->ResumeOwnedSimulation();
			}
		}
		bool IsValid() const { return bValid; }
	private:
		ASpeedPhysicsSimulation* Driver = nullptr;
		bool bWasPaused = true;
		bool bOwnsSuspension = false;
		bool bValid = true;
	};

	FString AuthoredObjectPath(const UObject& Object)
	{
		return UWorld::RemovePIEPrefix(Object.GetPathName());
	}
}

void USpeedWorldSubsystem::OnWorldBeginPlay(UWorld& InWorld)
{
	if (Speed::ReplayIsolation::IsInertReplayWorld(&InWorld)) return;

	Super::OnWorldBeginPlay(InWorld);
	SimulationWorld.ResetStaticCollisionWorld();
	AnalyticWorldData.Reset();
	AnalyticSourceComponents.Reset();
	AnalyticWorldBuildAttempt = 0;
    PreparedIntervalScene.Reset();PreparedIntervalAsset.Reset();StaticWorldPublicationDepth=0;bPreparedWorldResetComplete=true;
    bPreparedIntervalRouteRequested=FParse::Param(FCommandLine::Get(),TEXT("SpeedPreparedIntervalWorld"));
    if(bPreparedIntervalRouteRequested)return; // Explicit preparation lane; old builder stays untouched otherwise.
	if (!Speed::Analytic::FStaticWorldQueryAudit::ShouldBuildAnalyticWorld())
	{
		return;
	}
	BuildAnalyticWorldFromLoadedSources();
}

TWeakObjectPtr<UPrimitiveComponent> USpeedWorldSubsystem::FindAnalyticSourceComponent(
	const uint64 SourceId) const
{
	if (const TWeakObjectPtr<UPrimitiveComponent>* Component =
		AnalyticSourceComponents.Find(SourceId))
	{
		return *Component;
	}
	return nullptr;
}


#define REPLAY_ANALYTIC_LOG(Category,Verbosity,...) do { UE_LOG(Category,Verbosity,__VA_ARGS__); if(FParse::Param(FCommandLine::Get(),TEXT("ReplayDeliveryProbe"))) { const FString File=FPaths::ProjectSavedDir()/TEXT("ReplayAnalyticDiagnostics.txt"); if(IFileManager::Get().FileSize(*File)<65536) FFileHelper::SaveStringToFile(FString::Printf(__VA_ARGS__)+TEXT("\n"),*File,FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM,&IFileManager::Get(),FILEWRITE_Append); } } while(false)
void USpeedWorldSubsystem::BuildAnalyticWorldFromLoadedSources()
{
    if(bPreparedIntervalRouteRequested||PreparedIntervalScene)return; // A queued legacy build must never replace the dedicated snapshot.
	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}
	++AnalyticWorldBuildAttempt;
	// No provisional owner mappings escape a failed build or a deferred retry.
	TMap<uint64, TWeakObjectPtr<UPrimitiveComponent>> PendingSourceComponents;
	auto TryBindSource = [&PendingSourceComponents](uint64 Id, UPrimitiveComponent* Component)
	{
		if (Id == 0 || !Component) return false;
		if (const auto* Existing = PendingSourceComponents.Find(Id))
		{
			if (Existing->Get() != Component)
			{
				REPLAY_ANALYTIC_LOG(LogTemp, Error,
					TEXT("[AnalyticSourceBinding] Conflicting SourceId=%016llX"), Id);
				return false;
			}
			return true;
		}
		PendingSourceComponents.Add(Id, Component);
		return true;
	};

	TArray<ALandscapeProxy*> Landscapes;
	for (TActorIterator<ALandscapeProxy> It(World); It; ++It)
	{
		Landscapes.Add(*It);
	}
	Landscapes.Sort([](const ALandscapeProxy& A, const ALandscapeProxy& B)
	{
		return A.GetPathName() < B.GetPathName();
	});

	TSharedPtr<Speed::Analytic::FAnalyticWorldData> Imported =
		MakeShared<Speed::Analytic::FAnalyticWorldData>();
	TSharedPtr<const Speed::Analytic::FAnalyticWorldData> BakedRuntimeWorld;
	bool bRuntimeWorldMutated = false;
	bool bRuntimeWorldIncrementallyFinalized = false;
	uint64 CombinedSourceHash = 0;
	bool bSourceReadinessPending = false;
	TArray<Speed::Analytic::FBoundedPlane> AdditionalLandscapePlanes;

	TArray<AActor*> AnalyticMeshSourceActors;
	for (TActorIterator<AActor> It(World); It; ++It)
	{
		if (It->FindComponentByClass<USpeedAnalyticSourceComponent>())
		{
			AnalyticMeshSourceActors.Add(*It);
		}
	}
	AnalyticMeshSourceActors.Sort([](const AActor& A, const AActor& B)
	{
		return A.GetPathName() < B.GetPathName();
	});
	uint64 MeshSourceInventoryHash = 0;
	int32 MeshSourceComponentCount = 0;
	TArray<UStaticMeshComponent*> AnalyticMeshComponents;
	const USpeedAnalyticCollisionAsset* ExplicitBakedAsset = nullptr;
	FString ExplicitBakedAssetPath;
	TMap<uint64, const USpeedAnalyticSourceComponent*> AuthoredDeclarations;
	for (const AActor* SourceActor : AnalyticMeshSourceActors)
	{
		TArray<UStaticMeshComponent*> MeshComponents;
		TArray<USpeedAnalyticSourceComponent*> SourceComponents;
		SourceActor->GetComponents<USpeedAnalyticSourceComponent>(SourceComponents);
		if (SourceComponents.Num() != 1)
		{
			REPLAY_ANALYTIC_LOG(LogTemp, Error,
				TEXT("[AnalyticSourceBinding] Ambiguous declarations Owner=%s Count=%d"),
				*SourceActor->GetPathName(), SourceComponents.Num());
			return;
		}
		const USpeedAnalyticSourceComponent* SourceComponent = SourceComponents[0];
		if (SourceComponent->AuthoredSourceId != 0)
		{
			if (SourceComponent->bIncludeOwnedStaticMeshes ||
				SourceComponent->AnalyticCollisionAsset.IsNull() ||
				SourceComponent->AuthoredContactComponentPath.IsEmpty() ||
				AuthoredDeclarations.Contains(SourceComponent->AuthoredSourceId))
			{
				REPLAY_ANALYTIC_LOG(LogTemp, Error,
					TEXT("[AnalyticSourceBinding] Incomplete or duplicate declaration Owner=%s"),
					*SourceActor->GetPathName());
				return;
			}
			AuthoredDeclarations.Add(SourceComponent->AuthoredSourceId, SourceComponent);
		}
		else
		{
			if (!SourceComponent->AuthoredContactComponentPath.IsEmpty())
			{
				REPLAY_ANALYTIC_LOG(LogTemp, Error,
					TEXT("[AnalyticSourceBinding] Contact path without authored SourceId Owner=%s"),
					*SourceActor->GetPathName());
				return;
			}
			if (const ASpeedStaticActor* SpeedStaticActor = Cast<ASpeedStaticActor>(SourceActor))
				SpeedStaticActor->GatherAnalyticStaticMeshes(MeshComponents);
			else
				SourceComponent->GatherOwnedStaticMeshes(MeshComponents);
		}
		if (SourceComponent && !SourceComponent->AnalyticCollisionAsset.IsNull())
		{
			const FString AssetPath =
				SourceComponent->AnalyticCollisionAsset.ToSoftObjectPath().ToString();
			const USpeedAnalyticCollisionAsset* CandidateAsset =
				SourceComponent->AnalyticCollisionAsset.LoadSynchronous();
			if (!CandidateAsset)
			{
				REPLAY_ANALYTIC_LOG(LogTemp, Error,
					TEXT("[AnalyticExplicitBake] Asset=%s Result=Missing Owner=%s"),
					*AssetPath, *SourceActor->GetPathName());
				return;
			}
			if (ExplicitBakedAsset && ExplicitBakedAsset != CandidateAsset)
			{
				REPLAY_ANALYTIC_LOG(LogTemp, Error,
					TEXT("[AnalyticExplicitBake] Result=ConflictingAssets First=%s Second=%s"),
					*ExplicitBakedAssetPath, *AssetPath);
				return;
			}
			ExplicitBakedAsset = CandidateAsset;
			ExplicitBakedAssetPath = AssetPath;
		}
		for (const UStaticMeshComponent* MeshComponent : MeshComponents)
		{
			const UStaticMesh* Mesh = MeshComponent
				? MeshComponent->GetStaticMesh() : nullptr;
			if (!MeshComponent || !Mesh)
			{
				continue;
			}
			++MeshSourceComponentCount;
			AnalyticMeshComponents.Add(
				const_cast<UStaticMeshComponent*>(MeshComponent));
			const FString ActorPath = AuthoredObjectPath(*SourceActor);
			const FString ComponentPath = AuthoredObjectPath(*MeshComponent);
			if (!TryBindSource(Speed::Analytic::StableStringId(ComponentPath),
				const_cast<UStaticMeshComponent*>(MeshComponent))) return;
			const FString StableIdentity = FString::Printf(
				TEXT("%s|%s|%s|%s"), *ActorPath,
				*ComponentPath, *Mesh->GetPathName(),
				*MeshComponent->GetComponentTransform().ToString());
			MeshSourceInventoryHash = Speed::Analytic::CombineStableIds(
				MeshSourceInventoryHash,
				Speed::Analytic::StableStringId(StableIdentity));
			REPLAY_ANALYTIC_LOG(LogTemp, Display,
				TEXT("[AnalyticMeshSource] Owner=%s Component=%s Mesh=%s"),
				*SourceActor->GetPathName(), *MeshComponent->GetName(),
				*Mesh->GetPathName());
		}
	}

	bool bMeshBakeLoaded = false;
	// PIE duplicates a world under UEDPIE_<instance>_<map>. Generated collision
	// certificates belong to the authored map and must keep the same stable ids
	// in editor play, standalone and packaged execution.
	const FString WorldName = UWorld::RemovePIEPrefix(
		FPackageName::GetShortName(World->GetOutermost()));
	const FString BakedPackagePath = FString::Printf(
		TEXT("/Game/Generated/Analytic/%s_AnalyticWorld"), *WorldName);
	const FString BakedObjectPath = FString::Printf(
		TEXT("%s.%s_AnalyticWorld"), *BakedPackagePath, *WorldName);
	const bool bUsesExplicitBake = ExplicitBakedAsset != nullptr;
	const USpeedAnalyticCollisionAsset* BakedAsset = ExplicitBakedAsset;
	if (!BakedAsset)
	{
		// The convention-based bake is optional: worlds without a generated
		// package still build from their loaded analytic sources below. Preserve
		// in-memory assets, and only load from disk when the package exists.
		BakedAsset = FindObject<USpeedAnalyticCollisionAsset>(
			nullptr, *BakedObjectPath);
		if (!BakedAsset && FPackageName::DoesPackageExist(BakedPackagePath))
		{
			BakedAsset = LoadObject<USpeedAnalyticCollisionAsset>(
				nullptr, *BakedObjectPath);
		}
	}
	if (BakedAsset)
	{
		if (BakedAsset->AuthoredSources.Num() != AuthoredDeclarations.Num() ||
			(!BakedAsset->AuthoredSources.IsEmpty() && !bUsesExplicitBake))
		{
			REPLAY_ANALYTIC_LOG(LogTemp, Error,
				TEXT("[AnalyticSourceBinding] Authored source inventory mismatch or implicit asset."));
			return;
		}
		for (const FSpeedAnalyticAuthoredSourceRecord& Record : BakedAsset->AuthoredSources)
		{
			const auto* Declaration = AuthoredDeclarations.Find(Record.SourceId);
			FString BindingReason;
			UPrimitiveComponent* Contact = Declaration
				? (*Declaration)->ResolveAuthoredContact(Record, *BakedAsset, &BindingReason) : nullptr;
			if (!Contact)
			{
				REPLAY_ANALYTIC_LOG(LogTemp, Error,
					TEXT("[AnalyticSourceBinding] Rejected SourceId=%016llX Detail=%s"),
					Record.SourceId, Declaration ? *BindingReason : TEXT("Declaration missing."));
				return;
			}
            // Source/component registration order must not leave a stale empty
            // bounds cache. Refresh once after exact binding, never per query.
            if (auto* AnalyticContact = Cast<USpeedAnalyticContactComponent>(Contact))
            {
                bool BoundsReady = false;
                AnalyticContact->RefreshAuthoredBounds(BoundsReady, BindingReason);
                if (!BoundsReady)
                {
                    REPLAY_ANALYTIC_LOG(LogTemp, Error, TEXT("[AnalyticSourceBinding] Invalid analytic contact bounds: %s"), *BindingReason);
                    return;
                }
            }
			if (!TryBindSource(Record.SourceId, Contact)) return;
		}

		int32 SerializedPlaneAuthorityCount = 0;
		for (const FSpeedAnalyticBoundedPlaneRecord& Plane :
			BakedAsset->BoundedPlanes)
		{
			SerializedPlaneAuthorityCount += Plane.bAuthorityEligible ? 1 : 0;
		}
		int32 SerializedQuinticAuthorityCount = 0;
		for (const FSpeedAnalyticExtrudedQuinticPatchRecord& Patch :
			BakedAsset->ExtrudedQuinticPatches)
		{
			SerializedQuinticAuthorityCount += Patch.bAuthorityEligible ? 1 : 0;
		}
		int32 SerializedTensorAuthorityCount = 0;
		for (const FSpeedAnalyticTensorBezierPatchRecord& Patch :
			BakedAsset->TensorBezierPatches)
		{
			SerializedTensorAuthorityCount += Patch.bAuthorityEligible ? 1 : 0;
		}
		int32 SerializedPiecewiseAuthorityCount = 0;
		for (const FSpeedAnalyticPiecewiseTensorBezierPatchRecord& Patch :
			BakedAsset->PiecewiseTensorBezierPatches)
		{
			SerializedPiecewiseAuthorityCount += Patch.bAuthorityEligible ? 1 : 0;
		}
		uint64 BakedInventoryHash = 0;
		// Bake records are serialized in SourceId order. Live discovery above is
		// ordered by authored actor/component paths. Compare the same inventory
		// order without changing baked geometry, primitive ids or contact order.
		TArray<const FSpeedAnalyticMeshSourceRecord*> InventorySources;
		for (const FSpeedAnalyticMeshSourceRecord& Source : BakedAsset->MeshSources)
			InventorySources.Add(&Source);
		InventorySources.Sort([](const FSpeedAnalyticMeshSourceRecord& A,
			const FSpeedAnalyticMeshSourceRecord& B)
		{
			return A.ActorPath == B.ActorPath
				? A.ComponentPath < B.ComponentPath : A.ActorPath < B.ActorPath;
		});
		for (const FSpeedAnalyticMeshSourceRecord* SourceRecord : InventorySources)
		{
			const FSpeedAnalyticMeshSourceRecord& Source = *SourceRecord;
			const FString StableIdentity = FString::Printf(
				TEXT("%s|%s|%s|%s"), *Source.ActorPath, *Source.ComponentPath,
				*Source.MeshPath, *Source.WorldTransform.ToString());
			BakedInventoryHash = Speed::Analytic::CombineStableIds(
				BakedInventoryHash,
				Speed::Analytic::StableStringId(StableIdentity));
		}
		FString BakedReason;
		const bool bAuthorityProvidersOnly =
			Speed::Analytic::FStaticWorldQueryAudit::IsSurfaceAnalyticBackend();
		const double RuntimeBuildStartSeconds = FPlatformTime::Seconds();
		const TSharedPtr<const Speed::Analytic::FAnalyticWorldData> BakedRuntime =
			BakedAsset->BuildRuntimeData(&BakedReason, false,
				bAuthorityProvidersOnly);
		REPLAY_ANALYTIC_LOG(LogTemp, Display,
			TEXT("[AnalyticRuntimeLoadTiming] AuthorityOnly=%d Seconds=%.6f Success=%d"),
			bAuthorityProvidersOnly ? 1 : 0,
			FPlatformTime::Seconds() - RuntimeBuildStartSeconds,
			BakedRuntime ? 1 : 0);
		int32 RuntimePlaneAuthorityCount = 0;
		int32 RuntimeQuinticAuthorityCount = 0;
		int32 RuntimeTensorAuthorityCount = 0;
		int32 RuntimePiecewiseAuthorityCount = 0;
		int64 RuntimeTensorApproximationCellCount = 0;
		int64 RuntimePiecewiseCellCount = 0;
		int64 RuntimePiecewiseApproximationCellCount = 0;
		int64 RuntimePiecewiseApproximationBvhNodeCount = 0;
		if (BakedRuntime)
		{
			for (const Speed::Analytic::FBoundedPlane& Plane : BakedRuntime->Planes)
			{
				RuntimePlaneAuthorityCount += Plane.bAuthorityEligible ? 1 : 0;
			}
			for (const Speed::Analytic::FExtrudedQuinticPatch& Patch :
				BakedRuntime->ExtrudedQuinticPatches)
			{
				RuntimeQuinticAuthorityCount += Patch.bAuthorityEligible ? 1 : 0;
			}
			for (const Speed::Analytic::FTensorBezierPatch& Patch :
				BakedRuntime->TensorBezierPatches)
			{
				RuntimeTensorAuthorityCount += Patch.bAuthorityEligible ? 1 : 0;
				RuntimeTensorApproximationCellCount += Patch.ApproximationCells.Num();
			}
			for (const Speed::Analytic::FPiecewiseTensorBezierPatch& Patch :
				BakedRuntime->PiecewiseTensorBezierPatches)
			{
				RuntimePiecewiseAuthorityCount += Patch.bAuthorityEligible ? 1 : 0;
				RuntimePiecewiseCellCount += Patch.Cells.Num();
				for (const Speed::Analytic::FPiecewiseTensorBezierCell& Cell :
					Patch.Cells)
				{
					RuntimePiecewiseApproximationCellCount +=
						Cell.ApproximationCells.Num();
					RuntimePiecewiseApproximationBvhNodeCount +=
						Cell.ApproximationCellBvhNodes.Num();
				}
			}
		}
		const int32 RuntimePlaneCount = BakedRuntime ? BakedRuntime->Planes.Num() : 0;
		const int32 RuntimeQuinticCount = BakedRuntime ?
			BakedRuntime->ExtrudedQuinticPatches.Num() : 0;
		const int32 RuntimeTensorCount = BakedRuntime ?
			BakedRuntime->TensorBezierPatches.Num() : 0;
		const int32 RuntimePiecewiseCount = BakedRuntime ?
			BakedRuntime->PiecewiseTensorBezierPatches.Num() : 0;
		REPLAY_ANALYTIC_LOG(LogTemp, Display,
			TEXT("[AnalyticMeshBakeAuthority] BakeSchema=%u SerializedPlanes=%d/%d SerializedQuintics=%d/%d SerializedTensors=%d/%d SerializedPiecewise=%d/%d RuntimePlanes=%d/%d RuntimeQuintics=%d/%d RuntimeTensors=%d/%d RuntimePiecewise=%d/%d"),
			BakedAsset->BakeSchemaVersion,
			SerializedPlaneAuthorityCount, BakedAsset->BoundedPlanes.Num(),
			SerializedQuinticAuthorityCount,
			BakedAsset->ExtrudedQuinticPatches.Num(),
			SerializedTensorAuthorityCount, BakedAsset->TensorBezierPatches.Num(),
			SerializedPiecewiseAuthorityCount,
			BakedAsset->PiecewiseTensorBezierPatches.Num(),
			RuntimePlaneAuthorityCount, RuntimePlaneCount,
			RuntimeQuinticAuthorityCount, RuntimeQuinticCount,
			RuntimeTensorAuthorityCount, RuntimeTensorCount,
			RuntimePiecewiseAuthorityCount, RuntimePiecewiseCount);
		REPLAY_ANALYTIC_LOG(LogTemp, Display,
			TEXT("[AnalyticMeshBakeAcceleration] TensorApproximationCells=%lld PiecewiseCells=%lld PiecewiseApproximationCells=%lld PiecewiseApproximationBvhNodes=%lld"),
			RuntimeTensorApproximationCellCount, RuntimePiecewiseCellCount,
			RuntimePiecewiseApproximationCellCount,
			RuntimePiecewiseApproximationBvhNodeCount);
		bool bInventoryMatches =
			BakedInventoryHash == MeshSourceInventoryHash &&
			BakedAsset->MeshSources.Num() == MeshSourceComponentCount;
		if (bUsesExplicitBake)
		{
			bInventoryMatches =
				BakedAsset->MeshSources.Num() == MeshSourceComponentCount;
			TSet<const UStaticMeshComponent*> ClaimedComponents;
			for (const FSpeedAnalyticMeshSourceRecord& Source :
				BakedAsset->MeshSources)
			{
				const UStaticMeshComponent* MatchedComponent = nullptr;
				for (const UStaticMeshComponent* Candidate : AnalyticMeshComponents)
				{
					if (!Candidate || ClaimedComponents.Contains(Candidate) ||
						!Candidate->GetStaticMesh() ||
						Candidate->GetStaticMesh()->GetPathName() != Source.MeshPath ||
						!Candidate->GetComponentTransform().Equals(
							Source.WorldTransform, 0.01f))
					{
						continue;
					}
					MatchedComponent = Candidate;
					break;
				}
				if (!MatchedComponent)
				{
					bInventoryMatches = false;
					break;
				}
				ClaimedComponents.Add(MatchedComponent);
				if (!TryBindSource(Source.SourceId,
					const_cast<UStaticMeshComponent*>(MatchedComponent))) return;
			}
		}
		if (BakedRuntime && bInventoryMatches)
		{
			BakedRuntimeWorld = BakedRuntime;
			CombinedSourceHash = Speed::Analytic::CombineStableIds(
				CombinedSourceHash, BakedRuntime->SourceHash);
			bMeshBakeLoaded = true;
			REPLAY_ANALYTIC_LOG(LogTemp, Display,
				TEXT("[AnalyticExplicitBake] Asset=%s Result=%s"),
				bUsesExplicitBake ? *ExplicitBakedAssetPath : *BakedObjectPath,
				bUsesExplicitBake ? TEXT("Accepted") : TEXT("MapFallbackAccepted"));
		}
		else
		{
			REPLAY_ANALYTIC_LOG(LogTemp, Warning,
				TEXT("[AnalyticMeshBake] Asset=%s Result=Rejected LiveHash=%016llX BakedHash=%016llX Detail=%s"),
				bUsesExplicitBake ? *ExplicitBakedAssetPath : *BakedObjectPath,
				MeshSourceInventoryHash, BakedInventoryHash,
				BakedRuntime ? TEXT("Source inventory mismatch.") : *BakedReason);
		}
	}
	REPLAY_ANALYTIC_LOG(LogTemp, Display,
		TEXT("[AnalyticMeshSourceInventory] Actors=%d Components=%d Hash=%016llX Baked=%d Triangles=%d"),
		AnalyticMeshSourceActors.Num(), MeshSourceComponentCount,
		MeshSourceInventoryHash, bMeshBakeLoaded ? 1 : 0,
		BakedRuntimeWorld ? BakedRuntimeWorld->Triangles.Num() : Imported->Triangles.Num());
	if ((MeshSourceComponentCount > 0 || !AuthoredDeclarations.IsEmpty() ||
		ExplicitBakedAsset != nullptr) && !bMeshBakeLoaded)
	{
		REPLAY_ANALYTIC_LOG(LogTemp, Error,
			TEXT("[AnalyticAuthorityCoverageMissing] World=%s Components=%d Hash=%016llX Detail=The strict analytical world will remain unavailable until the authored source inventory has a matching immutable bake."),
			*WorldName, MeshSourceComponentCount, MeshSourceInventoryHash);
		return;
	}

	for (const ALandscapeProxy* Landscape : Landscapes)
	{
		if (!Landscape)
		{
			continue;
		}
		if (!Landscape->CollisionComponents.IsEmpty() &&
			Landscape->CollisionComponents[0])
		{
			const FString LandscapePath = AuthoredObjectPath(*Landscape);
			if (!TryBindSource(Speed::Analytic::StableStringId(LandscapePath),
				Landscape->CollisionComponents[0])) return;
		}
		const FString LandscapePath = AuthoredObjectPath(*Landscape);
		const uint64 LandscapeSourceId =
			Speed::Analytic::StableStringId(LandscapePath);
		const Speed::Analytic::FAnalyticWorldData& CandidateWorld =
			BakedRuntimeWorld && !bRuntimeWorldMutated
				? *BakedRuntimeWorld : *Imported;
		if (CandidateWorld.Planes.ContainsByPredicate(
			[LandscapeSourceId](const Speed::Analytic::FBoundedPlane& Plane)
			{
				return Plane.SourceId == LandscapeSourceId &&
					!Plane.bRequiresCompactOptIn;
			}))
		{
			REPLAY_ANALYTIC_LOG(LogTemp, Display,
				TEXT("[AnalyticLandscapeAdapter] Source=%s Result=BakedAuthority Detail=Immutable baked Landscape certificate loaded."),
				*Landscape->GetPathName());
			continue;
		}
		const Speed::Analytic::FFlatLandscapeAdapterOutput Output =
			Speed::Analytic::BuildFlatLandscapePlane(
				*Landscape,
				CVarIAmSpeedAnalyticLandscapeFlatnessTolerance.GetValueOnGameThread());
		REPLAY_ANALYTIC_LOG(LogTemp, Display,
			TEXT("[AnalyticLandscapeAdapter] Source=%s Result=%u ResidualCm=%.9g Detail=%s"),
			*Landscape->GetPathName(), static_cast<uint8>(Output.Result),
			Output.MaximumHeightResidual, *Output.Diagnostic);
		if (Output.Result ==
			Speed::Analytic::EFlatLandscapeAdapterResult::SuccessShadowOnly ||
			Output.Result == Speed::Analytic::EFlatLandscapeAdapterResult::
				SuccessAuthorityEligible)
		{
			AdditionalLandscapePlanes.Add(Output.Plane);
			CombinedSourceHash = Speed::Analytic::CombineStableIds(
				CombinedSourceHash, Output.Plane.SurfaceId);
		}
		else if (Output.Result ==
			Speed::Analytic::EFlatLandscapeAdapterResult::NoSourceData)
		{
			bSourceReadinessPending = true;
		}
	}
	constexpr uint8 MaximumBuildAttempts = 8;
	if (bSourceReadinessPending && AnalyticWorldBuildAttempt < MaximumBuildAttempts)
	{
		World->GetTimerManager().SetTimerForNextTick(
			FTimerDelegate::CreateUObject(
				this, &USpeedWorldSubsystem::BuildAnalyticWorldFromLoadedSources));
		return;
	}
	if (!AdditionalLandscapePlanes.IsEmpty())
	{
		if (BakedRuntimeWorld)
		{
			Imported = MakeShared<Speed::Analytic::FAnalyticWorldData>(
				*BakedRuntimeWorld);
			Imported->SourceHash = CombinedSourceHash;
			FString ExtensionReason;
			if (!Imported->AppendFinalizedNonCompactPlanes(
				MoveTemp(AdditionalLandscapePlanes), &ExtensionReason))
			{
				REPLAY_ANALYTIC_LOG(LogTemp, Warning,
					TEXT("[AnalyticWorldBuild] IncrementalPlaneExtensionRejected Detail=%s"),
					*ExtensionReason);
				return;
			}
			bRuntimeWorldMutated = true;
			bRuntimeWorldIncrementallyFinalized = true;
		}
		else
		{
			Imported->Planes.Append(MoveTemp(AdditionalLandscapePlanes));
		}
	}
    TArray<Speed::Analytic::FTorusSurface> AuthoredTori;
    for (TActorIterator<AActor> It(World);It;++It)
    {
        TArray<URingSubBody*> Rings;It->GetComponents<URingSubBody>(Rings);
        for (URingSubBody* Ring:Rings)
        {
            const FString Identity=Ring->AnalyticIdentity.IsNone() ? AuthoredObjectPath(*Ring) : Ring->AnalyticIdentity.ToString();
            const uint64 Id=Speed::Analytic::StableStringId(TEXT("AuthoredTorus|")+Identity);
            Speed::Analytic::FTorusSurface Torus;FString Error;
            if (!Ring->BuildStaticTorus(Id,Torus,Error))
            { REPLAY_ANALYTIC_LOG(LogTemp,Error,TEXT("[AnalyticTorusRejected] %s: %s"),*Ring->GetPathName(),*Error); return; }
            AuthoredTori.Add(Torus);
            if (!TryBindSource(Id, Ring)) return;
        }
    }
    if (!AuthoredTori.IsEmpty())
    {
        if (BakedRuntimeWorld && !bRuntimeWorldMutated)
            Imported=MakeShared<Speed::Analytic::FAnalyticWorldData>(*BakedRuntimeWorld);
        AuthoredTori.Sort([](const auto& A,const auto& B){return A.PrimitiveId<B.PrimitiveId;});
        for (const auto& Torus:AuthoredTori)
            CombinedSourceHash=Speed::Analytic::CombineStableIds(CombinedSourceHash,Torus.SourceId);
        Imported->Tori.Append(MoveTemp(AuthoredTori));
        bRuntimeWorldMutated=true;bRuntimeWorldIncrementallyFinalized=false;
        REPLAY_ANALYTIC_LOG(LogTemp,Display,TEXT("[AnalyticTorusAdmission] Count=%d"),Imported->Tori.Num());
    }
	TSharedPtr<const Speed::Analytic::FAnalyticWorldData> FinalizedWorld;
	if (BakedRuntimeWorld && !bRuntimeWorldMutated)
	{
		FinalizedWorld = BakedRuntimeWorld;
	}
	else if (bRuntimeWorldIncrementallyFinalized)
	{
		FinalizedWorld = Imported;
	}
	else
	{
		Imported->SourceHash = CombinedSourceHash;
		FString ValidationReason;
		if (!Imported->FinalizeAndValidate(&ValidationReason))
		{
			REPLAY_ANALYTIC_LOG(LogTemp, Warning,
				TEXT("[AnalyticWorldBuild] Rejected Detail=%s"), *ValidationReason);
			return;
		}
		FinalizedWorld = Imported;
	}
	const Speed::Analytic::FAnalyticWorldData& Finalized = *FinalizedWorld;
	int32 BoundaryEdgeCount = 0;
	int32 NonManifoldEdgeCount = 0;
	int32 SmoothEdgeCount = 0;
	int32 CreaseEdgeCount = 0;
	int32 PlanarCandidateCount = 0;
	int32 ValidShapeSampleCount = 0;
	for (const Speed::Analytic::FTriangleMeshEdge& Edge : Finalized.MeshEdges)
	{
		BoundaryEdgeCount += Edge.IsBoundary() ? 1 : 0;
		NonManifoldEdgeCount += Edge.IsNonManifold() ? 1 : 0;
		SmoothEdgeCount += Edge.Continuity ==
			Speed::Analytic::EEdgeContinuity::Smooth ? 1 : 0;
		CreaseEdgeCount += Edge.Continuity ==
			Speed::Analytic::EEdgeContinuity::Crease ? 1 : 0;
	}
	for (const Speed::Analytic::FSurfacePatch& Patch : Finalized.SurfacePatches)
	{
		PlanarCandidateCount += Patch.Kind ==
			Speed::Analytic::ESurfacePatchKind::PlanarCandidate ? 1 : 0;
	}
	for (const Speed::Analytic::FVertexShapeSample& Sample : Finalized.VertexShapeSamples)
	{
		ValidShapeSampleCount += Sample.bValid ? 1 : 0;
	}
	REPLAY_ANALYTIC_LOG(LogTemp, Display,
		TEXT("[AnalyticWorldBuild] Attempt=%u Planes=%d ExtrudedQuintics=%d Triangles=%d Vertices=%d Edges=%d BoundaryEdges=%d NonManifoldEdges=%d SmoothEdges=%d CreaseEdges=%d SmoothRegions=%d Patches=%d PlanarCandidates=%d ShapeValid=%d BvhNodes=%d Hash=%016llX AuthorityEligible=%d PendingSource=%d"),
		AnalyticWorldBuildAttempt, Finalized.Planes.Num(),
		Finalized.ExtrudedQuinticPatches.Num(), Finalized.Triangles.Num(),
		Finalized.MeshVertices.Num(), Finalized.MeshEdges.Num(), BoundaryEdgeCount,
		NonManifoldEdgeCount, SmoothEdgeCount, CreaseEdgeCount,
		Finalized.SmoothSurfaceRegions.Num(), Finalized.SurfacePatches.Num(), PlanarCandidateCount,
		ValidShapeSampleCount, Finalized.TriangleBvh.Num(), Finalized.StableHash(),
		Finalized.IsAuthorityEligible() ? 1 : 0,
		bSourceReadinessPending ? 1 : 0);
#if !UE_BUILD_SHIPPING
	uint64 ExtrudedFacetCount = 0;
	uint64 ExtrudedFacetBoundsAllocatedBytes = 0;
	uint64 ExtrudedFacetGeometryAllocatedBytes = 0;
	for (const Speed::Analytic::FExtrudedQuinticPatch& Patch : Finalized.ExtrudedQuinticPatches)
	{
		ExtrudedFacetCount += Patch.SectionSegmentBounds.Num();
		ExtrudedFacetBoundsAllocatedBytes += Patch.SectionSegmentBounds.GetAllocatedSize();
		ExtrudedFacetGeometryAllocatedBytes += Patch.SectionFacetGeometry.GetAllocatedSize();
	}
	REPLAY_ANALYTIC_LOG(LogTemp, Display,
		TEXT("[AnalyticQueryStorage] ExtrudedFacets=%llu FacetBoundsAllocatedBytes=%llu FacetGeometryAllocatedBytes=%llu"),
		ExtrudedFacetCount, ExtrudedFacetBoundsAllocatedBytes, ExtrudedFacetGeometryAllocatedBytes);
#endif
    AnalyticWorldData = MoveTemp(FinalizedWorld);
    AnalyticSourceComponents = MoveTemp(PendingSourceComponents);
    const Speed::FFrameParallelSettings ParallelSettings = Speed::CaptureFrameParallelSettings();
    SimulationWorld.SetStaticCollisionWorld(MakeUnique<Speed::FAnalyticStaticCollisionWorld>(
        *AnalyticWorldData, ParallelSettings.LaneLimit, ParallelSettings.MinQueryItems,
        ParallelSettings.MinFacetItems));
}

static TAutoConsoleVariable<float> CVarIAmSpeedRollingBaumgarte(
	TEXT("p.IAmSpeed.Collision.RollingBaumgarte"),
	0.20f,
	TEXT("Fraction of rolling-manifold penetration corrected per simulation step."));

static TAutoConsoleVariable<float> CVarIAmSpeedRollingMaxCorrectionSpeed(
	TEXT("p.IAmSpeed.Collision.RollingMaxCorrectionSpeed"),
	50.0f,
	TEXT("Maximum separating speed requested by rolling-manifold penetration correction, in cm/s."));

static TAutoConsoleVariable<float> CVarIAmSpeedRollingReleaseSpeed(
	TEXT("p.IAmSpeed.Collision.RollingReleaseSpeedCmS"),
	3.0f,
	TEXT("Separating normal speed above which a zero-gap rolling pair releases."));

static TAutoConsoleVariable<float> CVarIAmSpeedRollingTangentialFakePhysicsScale(
	TEXT("p.IAmSpeed.Collision.RollingTangentialFakePhysicsScale"),
	0.0f,
	TEXT("Amplitude of the fake-physics share projected into an active manifold's tangent plane. It never changes application frequency."));

static TAutoConsoleVariable<int32> CVarIAmSpeedDebugPersistentDynamicPairs(
	TEXT("p.IAmSpeed.Collision.DebugPersistentDynamicPairs"),
	0,
	TEXT("Logs registration, rejection and impulses for persistent dynamic pairs."));

#undef REPLAY_ANALYTIC_LOG

bool USpeedWorldSubsystem::AreUnilateralRollingPairsEnabled()
{
	return CVarIAmSpeedUnilateralRollingPairs.GetValueOnAnyThread() != 0;
}

bool USpeedWorldSubsystem::ShouldAcquireUnilateralRollingPair(
	const float RelativeNormalSpeed)
{
	return AreUnilateralRollingPairsEnabled() &&
		RelativeNormalSpeed <= FMath::Max(
			0.0f, CVarIAmSpeedRollingGentleAcquisitionSpeed.GetValueOnAnyThread());
}

void USpeedWorldSubsystem::RegisterSpeedComponent(ISpeedComponent* Comp)
{
    if (!Comp) return;
    FScopeLock Lock(&PendingCS);
    PendingOps.Add({ true, Comp });
}

void USpeedWorldSubsystem::UnregisterSpeedComponent(ISpeedComponent* Comp)
{
    if (!Comp) return;
    FScopeLock Lock(&PendingCS);
    PendingOps.Add({ false, Comp });
}

void USpeedWorldSubsystem::ApplyPendingOps()
{
	FScopedGameThreadAdapterAccess Access(GetWorld());
	if (!Access.IsValid())
	{
		UE_LOG(LogTemp, Error, TEXT("Adapter admission deferred: exclusive worker boundary not acknowledged"));
		return;
	}
	// Registrations queued during a frame are admitted at the next boundary.
	// Prepare/Step must never pick up an adapter that missed validation.
	if (bCanonicalFrameActive) return;
    TArray<FPendingOp> Local;
    {
        FScopeLock Lock(&PendingCS);
        if (Speed::PreparedExecution::RejectPreparedPendingMutation(
            bPreparedIntervalRouteRequested && Speed::OnePausedWork::IsActive(), !PendingOps.IsEmpty()))
            throw std::runtime_error("Prepared native pending registry mutation refused before MoveTemp/reset");
        Local = MoveTemp(PendingOps);
        PendingOps.Reset();
    }

    if (Local.Num() == 0) return;

    for (const FPendingOp& Op : Local)
    {
        if (!Op.Comp) continue;

        if (Op.bAdd)
        {
			if (!SimulationWorld.ContainsAdapter(*Op.Comp)) AddComponent(*Op.Comp);
        }
        else
        {
			RemoveComponent(*Op.Comp);
        }
    }
}

void USpeedWorldSubsystem::RebuildSortedIfNeeded()
{
	SimulationWorld.RebuildOrderedAdapters();
}

void USpeedWorldSubsystem::AddComponent(ISpeedComponent& Comp)
{
    // Add New component to all existing ones
    for (ISpeedComponent* Component : SimulationWorld.GetAdapters())
    {
		Component->AddExternalSubBodies(Comp.GetSubBodies());
    }

	// Add existing ones to the new component
    for (ISpeedComponent* Component : SimulationWorld.GetAdapters())
	{
        Comp.AddExternalSubBodies(Component->GetSubBodies());
	}

	// Finally add to list
    SimulationWorld.AddAdapter(Comp);
}

void USpeedWorldSubsystem::RemoveComponent(ISpeedComponent& Comp)
{
	if (!Comp.RetireInputProcessingOnWorker())
	{ UE_LOG(LogTemp, Fatal, TEXT("Input processors must retire on the owning lane before adapter removal")); return; }
	// Remove component from list
	SimulationWorld.RemoveAdapter(Comp);

	// Remove sub-bodies of the removed component from all existing ones
	for (ISpeedComponent* Component : SimulationWorld.GetAdapters())
    {
        Component->RemoveExternalSubBodies(Comp.GetSubBodies());
	}
}

void USpeedWorldSubsystem::RegisterDynamicContactPair(
	USolidSubBody& BodyA,
	USolidSubBody& BodyB,
	const FVector& ContactPoint,
	const FVector& NormalBToA,
	const float ImpactRelativeNormalSpeed)
{
	unsigned int& CurrentStepFrame = SimulationWorld.GetCurrentStepFrame();
	if (CVarIAmSpeedPersistentDynamicPairs.GetValueOnAnyThread() == 0)
	{
		return;
	}

	ISpeedComponent* CompA = BodyA.GetParentComponent();
	ISpeedComponent* CompB = BodyB.GetParentComponent();
	if (!CompA || !CompB || CompA == CompB ||
		!BodyA.UsesPersistentBilateralContact() ||
		!BodyB.UsesPersistentBilateralContact())
	{
		return;
	}

	const uint32 IdA = BodyA.GetUniqueID();
	const uint32 IdB = BodyB.GetUniqueID();
	const uint32 Lo = FMath::Min(IdA, IdB);
	const uint32 Hi = FMath::Max(IdA, IdB);
	const uint64 PairKey = (static_cast<uint64>(Lo) << 32) | Hi;
	FDynamicContactPair* Pair = SimulationWorld.FindDynamicContactPair(PairKey);
	bool bPendingPair = false;
	if (!Pair && AreUnilateralRollingPairsEnabled())
	{
		// Collision resolution runs inside the TOI iteration. Queue structural
		// changes so the active pair array remains immutable until the iteration
		// has completed.
		bPendingPair = true;
		Pair = SimulationWorld.FindPendingRollingContactPair(PairKey);
	}
	if (!Pair)
	{
		Pair = bPendingPair
			? &SimulationWorld.AddPendingRollingContactPair(PairKey, BodyA, BodyB)
			: &SimulationWorld.AddDynamicContactPair(PairKey, BodyA, BodyB);
		Pair->FirstSeenFrame = CurrentStepFrame;
		Pair->AcquisitionNormalSpeed = ImpactRelativeNormalSpeed;
	}

	Pair->LastSeenFrame = CurrentStepFrame;
	Pair->LastCOMA = CompA->GetPhysCOM();
	Pair->LastCOMB = CompB->GetPhysCOM();
	Pair->LocalAnchorA = CompA->GetPhysRotation().UnrotateVector(ContactPoint - CompA->GetPhysCOM());
	Pair->LocalAnchorB = CompB->GetPhysRotation().UnrotateVector(ContactPoint - CompB->GetPhysCOM());
	Pair->LocalNormalB = CompB->GetPhysRotation().UnrotateVector(NormalBToA.GetSafeNormal());
	if (CVarIAmSpeedDebugPersistentDynamicPairs.GetValueOnAnyThread() != 0)
	{
		UE_LOG(LogTemp, Warning,
			TEXT("[PersistentPair][Register] Frame=%u A=%s B=%s Normal=%s Point=%s AcquisitionNormalSpeed=%.6f"),
			CurrentStepFrame, *BodyA.GetName(), *BodyB.GetName(),
			*NormalBToA.ToString(), *ContactPoint.ToString(),
			ImpactRelativeNormalSpeed);
	}
}

void USpeedWorldSubsystem::ActivatePendingRollingContactPairAtTOI(
	const USolidSubBody& BodyA,
	const USolidSubBody& BodyB,
	const float RemainingDt)
{
    Speed::PreparedExecution::FStepScope::RequireNativeContactPairTargets(SimulationWorld,&BodyA,&BodyB);
	if (!AreUnilateralRollingPairsEnabled())
	{
		return;
	}

	const uint32 IdA = BodyA.GetUniqueID();
	const uint32 IdB = BodyB.GetUniqueID();
	const uint32 Lo = FMath::Min(IdA, IdB);
	const uint32 Hi = FMath::Max(IdA, IdB);
	const uint64 PairKey = (static_cast<uint64>(Lo) << 32) | Hi;
	if (!SimulationWorld.ActivatePendingRollingContactPair(PairKey))
	{
		return;
	}

	if (RemainingDt > KINDA_SMALL_NUMBER)
	{
		SolveDynamicContactPairs(RemainingDt, PairKey, true, true);
	}
}

bool USpeedWorldSubsystem::IsDynamicContactPairOwnedByRollingManifold(
	const USolidSubBody& BodyA,
	const USolidSubBody& BodyB) const
{
	const auto& DynamicContactPairs = SimulationWorld.GetDynamicContactPairs();
	const unsigned int CurrentStepFrame = SimulationWorld.GetCurrentStepFrame();
	if (!bLoggedRollingOwnershipState && AreUnilateralRollingPairsEnabled())
	{
		bLoggedRollingOwnershipState = true;
		UE_LOG(LogTemp, Warning,
			TEXT("[PersistentPair][OwnershipQueryState] Frame=%u PairCount=%d A=%s B=%s"),
			CurrentStepFrame, DynamicContactPairs.Num(),
			*BodyA.GetName(), *BodyB.GetName());
	}
	if (DynamicContactPairs.IsEmpty() ||
		!AreUnilateralRollingPairsEnabled())
	{
		return false;
	}

	const uint32 IdA = BodyA.GetUniqueID();
	const uint32 IdB = BodyB.GetUniqueID();
	const uint32 Lo = FMath::Min(IdA, IdB);
	const uint32 Hi = FMath::Max(IdA, IdB);
	const uint64 PairKey = (static_cast<uint64>(Lo) << 32) | Hi;
	const FDynamicContactPair* Pair = SimulationWorld.FindDynamicContactPair(PairKey);
	// Once an acquired pair has been solved at its TOI, it owns the remaining
	// substep and later sweeps must yield to the manifold.
	return Pair && Pair->bRollingManifoldReady &&
		Pair->BodyA.IsValid() && Pair->BodyB.IsValid();
}

ERollingManifoldContactState USpeedWorldSubsystem::GetRollingManifoldContactState(
	const USolidSubBody& Body) const
{
	if (!AreUnilateralRollingPairsEnabled())
	{
		return ERollingManifoldContactState::Absent;
	}

	const TSet<uint64>* PairKeys = SimulationWorld.FindDynamicContactPairKeys(Body.GetUniqueID());
	if (!PairKeys)
	{
		return ERollingManifoldContactState::Absent;
	}

	ERollingManifoldContactState Result = ERollingManifoldContactState::Absent;
	for (const uint64 PairKey : *PairKeys)
	{
		const FDynamicContactPair* Pair = SimulationWorld.FindDynamicContactPair(PairKey);
		if (!Pair || !Pair->bRollingManifoldReady ||
			!Pair->BodyA.IsValid() || !Pair->BodyB.IsValid() ||
			(Pair->BodyA.Get() != &Body && Pair->BodyB.Get() != &Body))
		{
			continue;
		}

		if (Pair->bActiveContactLastSolve)
		{
			return ERollingManifoldContactState::ActiveContact;
		}
		Result = ERollingManifoldContactState::IdentityOnly;
	}
	return Result;
}

void USpeedWorldSubsystem::SolveDynamicContactPairs(
	const float Dt,
	const uint64 PairKeyFilter,
	const bool bFilterByPairKey,
	const bool bSolveFirstSeenFrame)
{
    Speed::PreparedExecution::FStepScope::RequireNativeContactPairTargets(SimulationWorld);
	auto& DynamicContactPairs = SimulationWorld.GetDynamicContactPairs();
	unsigned int& CurrentStepFrame = SimulationWorld.GetCurrentStepFrame();
	constexpr unsigned int MaxUnseenFrames = 2;
#if !UE_BUILD_SHIPPING // IAmSpeed contact witness1075 BEGIN

 using CW=Speed::FFrameSolveWitnessScope;
 using CE=Speed::EFrameDynamicContactEvent;
 using CO=Speed::EFrameDynamicContactOutcome;
 const uint64 ContactWitnessPass=CW::IsActive()?CW::BeginDynamicPass(CurrentStepFrame,
  Speed::EFrameDynamicContactPass::Persistent,Dt,PairKeyFilter,bFilterByPairKey,
  bSolveFirstSeenFrame,DynamicContactPairs.Num(),
  bFilterByPairKey?SimulationWorld.FindDynamicContactPair(PairKeyFilter):nullptr):0;
 ON_SCOPE_EXIT {if(ContactWitnessPass)CW::EndDynamicPass(ContactWitnessPass,DynamicContactPairs.Num());};
#endif // IAmSpeed contact witness1075 END

	const float SeparationToleranceCm = FMath::Max(
		0.0f, CVarIAmSpeedRollingSeparationTolerance.GetValueOnAnyThread());
	const auto LogRollingRelease = [&CurrentStepFrame](
		const FDynamicContactPair& Pair,
		const TCHAR* Reason,
		const float Separation = 0.0f,
		const float RelativeNormalVelocity = 0.0f)
	{
		if (!Pair.bRollingManifoldReady ||
			CVarIAmSpeedDebugPersistentDynamicPairs.GetValueOnAnyThread() == 0)
		{
			return;
		}

		UE_LOG(LogTemp, Warning,
			TEXT("[PersistentPair][RollingSummary] Frame=%u PairKey=%llu Reason=%s AcquisitionNormalSpeed=%.6f LifetimeFrames=%u IdentityFrames=%u ActiveContactFrames=%u ReactionFrames=%u FeatureTransitions=%u NormalImpulse=%.6f NormalActivity=%.6f NormalSupportActivity=%.6f PeakNormalActivity=%.6f PeakNormalSupportActivity=%.6f MinSeparation=%.6f MaxSeparation=%.6f Separation=%.6f RelN=%.6f"),
			CurrentStepFrame,
			Pair.PairKey,
			Reason,
			Pair.AcquisitionNormalSpeed,
			CurrentStepFrame >= Pair.FirstSeenFrame
				? CurrentStepFrame - Pair.FirstSeenFrame + 1u
				: 0u,
			Pair.SupportedFrameCount,
			Pair.ActiveContactFrameCount,
			Pair.ReactionFrameCount,
			Pair.FeatureTransitionCount,
			Pair.AccumulatedNormalImpulse,
			Pair.AccumulatedNormalActivity,
			Pair.AccumulatedNormalSupportActivity,
			Pair.PeakNormalActivity,
			Pair.PeakNormalSupportActivity,
			Pair.MinimumSeparation == TNumericLimits<float>::Max()
				? 0.0f : Pair.MinimumSeparation,
			Pair.MaximumSeparation == TNumericLimits<float>::Lowest()
				? 0.0f : Pair.MaximumSeparation,
			Separation,
			RelativeNormalVelocity);
	};

	int32 FirstPairIndex = DynamicContactPairs.Num() - 1;
	int32 LastPairIndex = 0;
	if (bFilterByPairKey)
	{
		FirstPairIndex = SimulationWorld.FindDynamicContactPairIndex(PairKeyFilter);
		if (FirstPairIndex == INDEX_NONE)
		{
			return;
		}
		LastPairIndex = FirstPairIndex;
	}
	for (int32 Index = FirstPairIndex; Index >= LastPairIndex; --Index)
	{
		FDynamicContactPair& Pair = DynamicContactPairs[Index];
#if !UE_BUILD_SHIPPING // IAmSpeed contact witness1075 BEGIN

 const uint64 WitnessPairKey=Pair.PairKey;
 const TWeakObjectPtr<USolidSubBody> WitnessBodyA=ContactWitnessPass?Pair.BodyA:TWeakObjectPtr<USolidSubBody>();
 const TWeakObjectPtr<USolidSubBody> WitnessBodyB=ContactWitnessPass?Pair.BodyB:TWeakObjectPtr<USolidSubBody>();
 CO WitnessOutcome=CO::Completed;bool bWitnessRolling=false;
 if(ContactWitnessPass)CW::RecordDynamicEvent(ContactWitnessPass,CE::PairBegin,Index,
  WitnessPairKey,&Pair,WitnessBodyA.Get(),WitnessBodyB.Get());
 ON_SCOPE_EXIT
 {
  if(ContactWitnessPass)CW::RecordDynamicEvent(ContactWitnessPass,CE::PairEnd,Index,
   WitnessPairKey,SimulationWorld.FindDynamicContactPair(WitnessPairKey),
   WitnessBodyA.Get(),WitnessBodyB.Get(),WitnessOutcome,bWitnessRolling);
 };
#endif // IAmSpeed contact witness1075 END

		if ((bFilterByPairKey && Pair.PairKey != PairKeyFilter) ||
			Pair.LastSolvedFrame == CurrentStepFrame)
		{
#if !UE_BUILD_SHIPPING // IAmSpeed contact witness1075 BEGIN
 WitnessOutcome=(bFilterByPairKey && Pair.PairKey!=PairKeyFilter)?CO::Filtered:CO::AlreadySolved;
#endif // IAmSpeed contact witness1075 END

			continue;
		}
		Pair.bActiveContactLastSolve = false;
		USolidSubBody* BodyA = Pair.BodyA.Get();
		USolidSubBody* BodyB = Pair.BodyB.Get();
		ISpeedComponent* CompA = BodyA ? BodyA->GetParentComponent() : nullptr;
		ISpeedComponent* CompB = BodyB ? BodyB->GetParentComponent() : nullptr;
		if (!BodyA || !BodyB || !CompA || !CompB)
		{
			LogRollingRelease(Pair, TEXT("InvalidBody"));
#if !UE_BUILD_SHIPPING // IAmSpeed contact witness1075 BEGIN
 WitnessOutcome=CO::InvalidBody;
#endif // IAmSpeed contact witness1075 END

			SimulationWorld.RemoveDynamicContactPairAtSwap(Index);
			continue;
		}

		USphereSubBody* Sphere = Cast<USphereSubBody>(BodyA);
		UBoxSubBody* Box = Cast<UBoxSubBody>(BodyB);
		if (!Sphere || !Box)
		{
			Sphere = Cast<USphereSubBody>(BodyB);
			Box = Cast<UBoxSubBody>(BodyA);
		}
		const bool bUseRollingManifold =
			AreUnilateralRollingPairsEnabled() &&
			Sphere != nullptr && Box != nullptr;
#if !UE_BUILD_SHIPPING // IAmSpeed contact witness1075 BEGIN
 bWitnessRolling=bUseRollingManifold;
#endif // IAmSpeed contact witness1075 END

		if (bUseRollingManifold && !bLoggedRollingSolveState)
		{
			bLoggedRollingSolveState = true;
			UE_LOG(LogTemp, Warning,
				TEXT("[PersistentPair][RollingSolveState] Frame=%u FirstSeen=%u LastSeen=%u PairCount=%d A=%s B=%s"),
				CurrentStepFrame, Pair.FirstSeenFrame, Pair.LastSeenFrame,
				DynamicContactPairs.Num(), *BodyA->GetName(), *BodyB->GetName());
		}
		if (!bUseRollingManifold &&
			CurrentStepFrame - Pair.LastSeenFrame > MaxUnseenFrames)
		{
#if !UE_BUILD_SHIPPING // IAmSpeed contact witness1075 BEGIN
 WitnessOutcome=CO::Unseen;
#endif // IAmSpeed contact witness1075 END

			SimulationWorld.RemoveDynamicContactPairAtSwap(Index);
			continue;
		}

		// Scenario resets can rewind the component frame counter. A pair from the
		// previous timeline must never retain ownership of CCD in the new one.
		if (CurrentStepFrame < Pair.FirstSeenFrame)
		{
			if (CVarIAmSpeedDebugPersistentDynamicPairs.GetValueOnAnyThread() != 0)
			{
				UE_LOG(LogTemp, Warning,
					TEXT("[PersistentPair][ReleaseFrameRewind] Frame=%u FirstSeenFrame=%u"),
					CurrentStepFrame, Pair.FirstSeenFrame);
			}
			LogRollingRelease(Pair, TEXT("FrameRewind"));
#if !UE_BUILD_SHIPPING // IAmSpeed contact witness1075 BEGIN
 WitnessOutcome=CO::FrameRewind;
#endif // IAmSpeed contact witness1075 END

			SimulationWorld.RemoveDynamicContactPairAtSwap(Index);
			continue;
		}

		// The ordinary end-of-frame pass must not resolve a newly registered pair
		// twice. The targeted TOI continuation is the only caller allowed to solve
		// it on its acquisition frame.
		if (CurrentStepFrame == Pair.FirstSeenFrame && !bSolveFirstSeenFrame)
		{
#if !UE_BUILD_SHIPPING // IAmSpeed contact witness1075 BEGIN
 WitnessOutcome=CO::FirstSeenDeferred;
#endif // IAmSpeed contact witness1075 END

			Pair.bRollingManifoldReady = bUseRollingManifold;
			continue;
		}
		Pair.LastSolvedFrame = CurrentStepFrame;

		const FVector CurrentCOMA = CompA->GetPhysCOM();
		const FVector CurrentCOMB = CompB->GetPhysCOM();
		const float MaxExpectedMoveA = CompA->GetPhysCOMVelocity().Size() * Dt * 2.0f + 10.0f;
		const float MaxExpectedMoveB = CompB->GetPhysCOMVelocity().Size() * Dt * 2.0f + 10.0f;
		if (FVector::DistSquared(CurrentCOMA, Pair.LastCOMA) > FMath::Square(MaxExpectedMoveA) ||
			FVector::DistSquared(CurrentCOMB, Pair.LastCOMB) > FMath::Square(MaxExpectedMoveB))
		{
			LogRollingRelease(Pair, TEXT("DiscontinuousMotion"));
#if !UE_BUILD_SHIPPING // IAmSpeed contact witness1075 BEGIN
 WitnessOutcome=CO::DiscontinuousMotion;
#endif // IAmSpeed contact witness1075 END

			SimulationWorld.RemoveDynamicContactPairAtSwap(Index);
			continue;
		}
		Pair.LastCOMA = CurrentCOMA;
		Pair.LastCOMB = CurrentCOMB;

		const FQuat RotA = CompA->GetPhysRotation();
		const FQuat RotB = CompB->GetPhysRotation();
		FVector N = RotB.RotateVector(Pair.LocalNormalB).GetSafeNormal();
		FVector AnchorA = CompA->GetPhysCOM() + RotA.RotateVector(Pair.LocalAnchorA);
		FVector AnchorB = CompB->GetPhysCOM() + RotB.RotateVector(Pair.LocalAnchorB);
		float ShapeSeparation = FVector::DotProduct(AnchorA - AnchorB, N);
		if (Sphere && Box)
		{
			const SSphere SphereShape = Sphere->MakeSphere();
			const SSBox BoxShape = Box->MakeBox();
			FVector ClosestPoint = FVector::ZeroVector;
			ShapeSeparation = BoxShape.SphereOBBSeparation(
				BoxShape.Rot,
				BoxShape.AbsoluteCenter(),
				SphereShape.Center,
				SphereShape.Radius,
				&ClosestPoint,
				SphereShape.bProductionNonpenetratingBoxContacts);
			const FVector CurrentBoxToSphereNormal =
				(SphereShape.Center - ClosestPoint).GetSafeNormal();
			const FVector StoredBoxToSphereNormal = Sphere == BodyA ? N : -N;
			if (ShapeSeparation > SeparationToleranceCm ||
				CurrentBoxToSphereNormal.IsNearlyZero() ||
				(!bUseRollingManifold &&
					FVector::DotProduct(CurrentBoxToSphereNormal, StoredBoxToSphereNormal) < 0.5f))
			{
				if (CVarIAmSpeedDebugPersistentDynamicPairs.GetValueOnAnyThread() != 0)
				{
					UE_LOG(LogTemp, Warning,
						TEXT("[PersistentPair][RejectGeometry] Frame=%u Separation=%.3f CurrentNormal=%s StoredNormal=%s"),
						CurrentStepFrame, ShapeSeparation,
						*CurrentBoxToSphereNormal.ToString(), *StoredBoxToSphereNormal.ToString());
				}
				LogRollingRelease(Pair, TEXT("Geometry"), ShapeSeparation);
#if !UE_BUILD_SHIPPING // IAmSpeed contact witness1075 BEGIN
 WitnessOutcome=CO::RejectedGeometry;
#endif // IAmSpeed contact witness1075 END

				SimulationWorld.RemoveDynamicContactPairAtSwap(Index);
				continue;
			}

			if (bUseRollingManifold)
			{
				const FVector PreviousNormal = N;
				N = Sphere == BodyA
					? CurrentBoxToSphereNormal
					: -CurrentBoxToSphereNormal;
				const FVector SphereContactPoint =
					SphereShape.Center - CurrentBoxToSphereNormal * SphereShape.Radius;
				if (Sphere == BodyA)
				{
					AnchorA = SphereContactPoint;
					AnchorB = ClosestPoint;
				}
				else
				{
					AnchorA = ClosestPoint;
					AnchorB = SphereContactPoint;
				}

				Pair.LocalAnchorA = RotA.UnrotateVector(AnchorA - CompA->GetPhysCOM());
				Pair.LocalAnchorB = RotB.UnrotateVector(AnchorB - CompB->GetPhysCOM());
				Pair.LocalNormalB = RotB.UnrotateVector(N);
				Pair.LastSeenFrame = CurrentStepFrame;
				if (FVector::DotProduct(PreviousNormal, N) < 0.995f)
				{
					++Pair.FeatureTransitionCount;
					if (CVarIAmSpeedDebugPersistentDynamicPairs.GetValueOnAnyThread() != 0)
					{
						UE_LOG(LogTemp, Warning,
							TEXT("[PersistentPair][FeatureTransition] Frame=%u Separation=%.3f PreviousNormal=%s CurrentNormal=%s"),
							CurrentStepFrame, ShapeSeparation,
							*PreviousNormal.ToString(), *N.ToString());
					}
				}
			}
		}

		if (N.IsNearlyZero())
		{
#if !UE_BUILD_SHIPPING // IAmSpeed contact witness1075 BEGIN
 WitnessOutcome=CO::ZeroNormal;
#endif // IAmSpeed contact witness1075 END

			continue;
		}
		Pair.bRollingManifoldReady = bUseRollingManifold;

		const FVector OffsetA = AnchorA - CompA->GetPhysCOM();
		const FVector OffsetB = AnchorB - CompB->GetPhysCOM();
		const FVector VelocityA = CompA->GetPhysCOMVelocity() +
			FVector::CrossProduct(CompA->GetPhysAngularVelocity(), OffsetA);
		const FVector VelocityB = CompB->GetPhysCOMVelocity() +
			FVector::CrossProduct(CompB->GetPhysAngularVelocity(), OffsetB);
		const float Separation = bUseRollingManifold
			? ShapeSeparation
			: FVector::DotProduct(AnchorA - AnchorB, N);
		const float RelativeNormalVelocity = FVector::DotProduct(VelocityA - VelocityB, N);
		if (bUseRollingManifold)
		{
			++Pair.SupportedFrameCount;
			Pair.MinimumSeparation = FMath::Min(Pair.MinimumSeparation, Separation);
			Pair.MaximumSeparation = FMath::Max(Pair.MaximumSeparation, Separation);
		}
		const float PredictedRelativeNormalVelocity = RelativeNormalVelocity +
			Dt * FVector::DotProduct(
				CompA->GetPhysAccelerationAtPoint(AnchorA) -
				CompB->GetPhysAccelerationAtPoint(AnchorB), N);
		const float PenetrationSlop = FMath::Max(
			0.0f, CVarIAmSpeedRollingPenetrationSlop.GetValueOnAnyThread());
		if (bUseRollingManifold && Separation >= -PenetrationSlop &&
			PredictedRelativeNormalVelocity > FMath::Max(
				0.0f, CVarIAmSpeedRollingReleaseSpeed.GetValueOnAnyThread()))
		{
			LogRollingRelease(
				Pair, TEXT("SeparatingVelocity"), Separation,
				PredictedRelativeNormalVelocity);
			SimulationWorld.RemoveDynamicContactPairAtSwap(Index);
#if !UE_BUILD_SHIPPING // IAmSpeed contact witness1075 BEGIN
 WitnessOutcome=CO::SeparatingVelocity;
#endif // IAmSpeed contact witness1075 END

			continue;
		}

		// Retaining pair identity across a small positive gap must not create
		// speculative support. Activate the constraint only at contact or when
		// unconstrained motion would cross the contact plane during this tick.
		const bool bContactActive = !bUseRollingManifold ||
			Separation <= PenetrationSlop ||
			Separation + Dt * PredictedRelativeNormalVelocity <= PenetrationSlop;
		if (!bContactActive)
		{
#if !UE_BUILD_SHIPPING // IAmSpeed contact witness1075 BEGIN
 WitnessOutcome=CO::IdentityOnly;
#endif // IAmSpeed contact witness1075 END

			if (CVarIAmSpeedDebugPersistentDynamicPairs.GetValueOnAnyThread() != 0)
			{
				UE_LOG(LogTemp, Warning,
					TEXT("[PersistentPair][IdentityOnly] Frame=%u Separation=%.6f RelN=%.6f PredictedRelN=%.6f"),
					CurrentStepFrame, Separation, RelativeNormalVelocity,
					PredictedRelativeNormalVelocity);
			}
			continue;
		}
		if (bUseRollingManifold)
		{
			++Pair.ActiveContactFrameCount;
			Pair.bActiveContactLastSolve = true;
		}

		const float PenetrationDepth = FMath::Max(0.0f, -Separation);
		const float TargetRelativeNormalVelocity = bUseRollingManifold &&
			PenetrationDepth > PenetrationSlop
			? FMath::Min(
				FMath::Max(0.0f, CVarIAmSpeedRollingMaxCorrectionSpeed.GetValueOnAnyThread()),
				FMath::Max(0.0f, CVarIAmSpeedRollingBaumgarte.GetValueOnAnyThread()) *
					(PenetrationDepth - PenetrationSlop) / Dt)
			: 0.0f;
		const float InvMassA = 1.0f / FMath::Max(CompA->GetPhysMass(), 1.0f);
		const float InvMassB = 1.0f / FMath::Max(CompB->GetPhysMass(), 1.0f);
		const FVector AngularA = FVector::CrossProduct(
			BodyA->ComputeWorldInvInertiaTensor().TransformVector(
				FVector::CrossProduct(OffsetA, N)), OffsetA);
		const FVector AngularB = FVector::CrossProduct(
			BodyB->ComputeWorldInvInertiaTensor().TransformVector(
				FVector::CrossProduct(OffsetB, N)), OffsetB);
		const float Denominator = InvMassA + InvMassB +
			FVector::DotProduct(N, AngularA + AngularB);
		if (Denominator <= KINDA_SMALL_NUMBER)
		{
#if !UE_BUILD_SHIPPING // IAmSpeed contact witness1075 BEGIN
 WitnessOutcome=CO::ZeroDenominator;
#endif // IAmSpeed contact witness1075 END

			continue;
		}

		const float NormalImpulseMagnitude = FMath::Max(
			0.0f,
			(TargetRelativeNormalVelocity - PredictedRelativeNormalVelocity) / Denominator);
		const float NormalActivity = NormalImpulseMagnitude * Denominator;
		// This is the velocity change required to prevent further normal closure.
		// Keep Baumgarte separation out of it: penetration correction is numerical
		// stabilization, not evidence of additional gameplay-shot activity.
		const float NormalSupportActivity = FMath::Max(
			0.0f, -PredictedRelativeNormalVelocity);
		if (bUseRollingManifold && NormalImpulseMagnitude > KINDA_SMALL_NUMBER)
		{
			++Pair.ReactionFrameCount;
			Pair.AccumulatedNormalImpulse += NormalImpulseMagnitude;
			Pair.AccumulatedNormalActivity += NormalActivity;
			Pair.AccumulatedNormalSupportActivity += NormalSupportActivity;
			Pair.PeakNormalActivity = FMath::Max(Pair.PeakNormalActivity, NormalActivity);
			Pair.PeakNormalSupportActivity = FMath::Max(
				Pair.PeakNormalSupportActivity, NormalSupportActivity);
			if (CVarIAmSpeedDebugPersistentDynamicPairs.GetValueOnAnyThread() != 0 &&
				(CurrentStepFrame - Pair.FirstSeenFrame) % 30u == 0u)
			{
				UE_LOG(LogTemp, Warning,
					TEXT("[PersistentPair][RollingTelemetry] Frame=%u PairKey=%llu LifetimeFrames=%u SupportedFrames=%u ReactionFrames=%u FeatureTransitions=%u NormalImpulse=%.6f NormalActivity=%.6f NormalSupportActivity=%.6f PeakNormalActivity=%.6f PeakNormalSupportActivity=%.6f Separation=%.6f RelN=%.6f"),
					CurrentStepFrame,
					Pair.PairKey,
					CurrentStepFrame - Pair.FirstSeenFrame + 1u,
					Pair.SupportedFrameCount,
					Pair.ReactionFrameCount,
					Pair.FeatureTransitionCount,
					Pair.AccumulatedNormalImpulse,
					Pair.AccumulatedNormalActivity,
					Pair.AccumulatedNormalSupportActivity,
					Pair.PeakNormalActivity,
					Pair.PeakNormalSupportActivity,
					Separation,
					RelativeNormalVelocity);
			}
		}
		const FVector Impulse = NormalImpulseMagnitude * N;
		const FVector ContactPoint = 0.5f * (AnchorA + AnchorB);
#if !UE_BUILD_SHIPPING // IAmSpeed contact witness1075 BEGIN
 if(ContactWitnessPass)CW::RecordDynamicEvent(ContactWitnessPass,CE::NormalCandidate,Index,
  Pair.PairKey,&Pair,BodyA,BodyB,CO::Completed,bUseRollingManifold,Impulse,ContactPoint,NormalImpulseMagnitude,Separation);
#endif // IAmSpeed contact witness1075 END

		FFakePhysicsImpactContext RollingFakePhysicsContext;
		if (bUseRollingManifold)
		{
			RollingFakePhysicsContext.SelfParentKinematics = Sphere == BodyA
				? CompA->GetKinematicState()
				: CompB->GetKinematicState();
			RollingFakePhysicsContext.OtherParentKinematics = Sphere == BodyA
				? CompB->GetKinematicState()
				: CompA->GetKinematicState();
			RollingFakePhysicsContext.FakeImpulseScale = FMath::Max(
				0.0f,
				CVarIAmSpeedRollingTangentialFakePhysicsScale.GetValueOnAnyThread());
			RollingFakePhysicsContext.bProjectFakeImpulseOntoContactPlane = true;
			RollingFakePhysicsContext.FakeTangentialVelocityScale = 1.0f;
			RollingFakePhysicsContext.bUseContactPointRelativeVelocity = true;
		}
		if (CVarIAmSpeedDebugPersistentDynamicPairs.GetValueOnAnyThread() != 0)
		{
			UE_LOG(LogTemp, Warning,
				TEXT("[PersistentPair][Impulse] Frame=%u A=%s B=%s J=%s Separation=%.3f RelN=%.3f NormalActivity=%.6f NormalSupportActivity=%.6f AccumulatedNormalActivity=%.6f AccumulatedNormalSupportActivity=%.6f"),
				CurrentStepFrame, *BodyA->GetName(), *BodyB->GetName(),
				*Impulse.ToString(), Separation, RelativeNormalVelocity,
				NormalActivity, NormalSupportActivity,
				Pair.AccumulatedNormalActivity,
				Pair.AccumulatedNormalSupportActivity);
		}
		if (NormalImpulseMagnitude > KINDA_SMALL_NUMBER)
		{

#if !UE_BUILD_SHIPPING // IAmSpeed contact witness1075 BEGIN
 if(ContactWitnessPass)CW::RecordDynamicEvent(ContactWitnessPass,CE::NormalBeforeA,Index,
  Pair.PairKey,&Pair,BodyA,BodyB,CO::Completed,bUseRollingManifold,Impulse,ContactPoint,NormalImpulseMagnitude,Separation);
#endif // IAmSpeed contact witness1075 END
			BodyA->ApplyImpulse(Impulse, ContactPoint);
#if !UE_BUILD_SHIPPING // IAmSpeed contact witness1075 BEGIN
 if(ContactWitnessPass)CW::RecordDynamicEvent(ContactWitnessPass,CE::NormalAfterA,Index,
  Pair.PairKey,&Pair,BodyA,BodyB,CO::Completed,bUseRollingManifold,Impulse,ContactPoint,NormalImpulseMagnitude,Separation);
#endif // IAmSpeed contact witness1075 END


#if !UE_BUILD_SHIPPING // IAmSpeed contact witness1075 BEGIN
 if(ContactWitnessPass)CW::RecordDynamicEvent(ContactWitnessPass,CE::NormalBeforeB,Index,
  Pair.PairKey,&Pair,BodyA,BodyB,CO::Completed,bUseRollingManifold,Impulse,ContactPoint,NormalImpulseMagnitude,Separation);
#endif // IAmSpeed contact witness1075 END
			BodyB->ApplyImpulse(-Impulse, ContactPoint);
#if !UE_BUILD_SHIPPING // IAmSpeed contact witness1075 BEGIN
 if(ContactWitnessPass)CW::RecordDynamicEvent(ContactWitnessPass,CE::NormalAfterB,Index,
  Pair.PairKey,&Pair,BodyA,BodyB,CO::Completed,bUseRollingManifold,Impulse,ContactPoint,NormalImpulseMagnitude,Separation);
#endif // IAmSpeed contact witness1075 END

		}

		if (bUseRollingManifold && NormalImpulseMagnitude > KINDA_SMALL_NUMBER)
		{
			const FVector PostVelocityA = CompA->GetPhysVelocityAtPoint(AnchorA);
			const FVector PostVelocityB = CompB->GetPhysVelocityAtPoint(AnchorB);
			const FVector RelativeVelocity = PostVelocityA - PostVelocityB;
			const FVector TangentialVelocity = RelativeVelocity -
				FVector::DotProduct(RelativeVelocity, N) * N;
			const float TangentialSpeed = TangentialVelocity.Size();
			if (TangentialSpeed > KINDA_SMALL_NUMBER)
			{
				const FVector Tangent = TangentialVelocity / TangentialSpeed;
				const FVector TangentialAngularA = FVector::CrossProduct(
					BodyA->ComputeWorldInvInertiaTensor().TransformVector(
						FVector::CrossProduct(OffsetA, Tangent)), OffsetA);
				const FVector TangentialAngularB = FVector::CrossProduct(
					BodyB->ComputeWorldInvInertiaTensor().TransformVector(
						FVector::CrossProduct(OffsetB, Tangent)), OffsetB);
				const float TangentialDenominator = InvMassA + InvMassB +
					FVector::DotProduct(Tangent, TangentialAngularA + TangentialAngularB);
				if (TangentialDenominator > KINDA_SMALL_NUMBER)
				{
					const float ManifoldFriction =
						USolidSubBody::ResolveSphereBoxManifoldFriction(
						*Sphere,
						*Box,
						USolidSubBody::MixFriction(
							Sphere->GetStaticFriction(), Box->GetStaticFriction(), EMixMode::E_Max));
					const float MaxFrictionImpulse =
						NormalImpulseMagnitude * FMath::Max(0.0f, ManifoldFriction);
					const float TangentialImpulseMagnitude = FMath::Min(
						TangentialSpeed / TangentialDenominator, MaxFrictionImpulse);
					const FVector TangentialImpulse = -TangentialImpulseMagnitude * Tangent;
#if !UE_BUILD_SHIPPING // IAmSpeed contact witness1075 BEGIN
 if(ContactWitnessPass)CW::RecordDynamicEvent(ContactWitnessPass,CE::TangentialCandidate,Index,
  Pair.PairKey,&Pair,BodyA,BodyB,CO::Completed,bUseRollingManifold,TangentialImpulse,ContactPoint,TangentialImpulseMagnitude,TangentialDenominator);
#endif // IAmSpeed contact witness1075 END


#if !UE_BUILD_SHIPPING // IAmSpeed contact witness1075 BEGIN
 if(ContactWitnessPass)CW::RecordDynamicEvent(ContactWitnessPass,CE::TangentialBeforeA,Index,
  Pair.PairKey,&Pair,BodyA,BodyB,CO::Completed,bUseRollingManifold,TangentialImpulse,ContactPoint,TangentialImpulseMagnitude,TangentialDenominator);
#endif // IAmSpeed contact witness1075 END
					BodyA->ApplyImpulse(TangentialImpulse, ContactPoint);
#if !UE_BUILD_SHIPPING // IAmSpeed contact witness1075 BEGIN
 if(ContactWitnessPass)CW::RecordDynamicEvent(ContactWitnessPass,CE::TangentialAfterA,Index,
  Pair.PairKey,&Pair,BodyA,BodyB,CO::Completed,bUseRollingManifold,TangentialImpulse,ContactPoint,TangentialImpulseMagnitude,TangentialDenominator);
#endif // IAmSpeed contact witness1075 END


#if !UE_BUILD_SHIPPING // IAmSpeed contact witness1075 BEGIN
 if(ContactWitnessPass)CW::RecordDynamicEvent(ContactWitnessPass,CE::TangentialBeforeB,Index,
  Pair.PairKey,&Pair,BodyA,BodyB,CO::Completed,bUseRollingManifold,TangentialImpulse,ContactPoint,TangentialImpulseMagnitude,TangentialDenominator);
#endif // IAmSpeed contact witness1075 END
					BodyB->ApplyImpulse(-TangentialImpulse, ContactPoint);
#if !UE_BUILD_SHIPPING // IAmSpeed contact witness1075 BEGIN
 if(ContactWitnessPass)CW::RecordDynamicEvent(ContactWitnessPass,CE::TangentialAfterB,Index,
  Pair.PairKey,&Pair,BodyA,BodyB,CO::Completed,bUseRollingManifold,TangentialImpulse,ContactPoint,TangentialImpulseMagnitude,TangentialDenominator);
#endif // IAmSpeed contact witness1075 END

				}
			}

		}

		// A valid active manifold represents one contact during this simulation
		// frame. Evaluate fake physics exactly once, with the manifold owning the
		// normal response and the fake share projected into its tangent plane.
		if (bUseRollingManifold)
		{
			const FVector BoxToSphereNormal = Sphere == BodyA ? N : -N;
			SHitResult RollingHit(
				true,
				ContactPoint,
				BoxToSphereNormal,
				0.0f);
			RollingHit.Location = Sphere->GetKinematicState().Location;
			RollingHit.PenetrationDepth = PenetrationDepth;
			RollingHit.ContactPointThis = Sphere == BodyA ? AnchorA : AnchorB;
			RollingHit.ContactPointOther = Sphere == BodyA ? AnchorB : AnchorA;
			RollingHit.SubBody = Box;

#if !UE_BUILD_SHIPPING // IAmSpeed contact witness1075 BEGIN
 if(ContactWitnessPass)CW::RecordDynamicEvent(ContactWitnessPass,CE::FakePhysicsBefore,Index,
  Pair.PairKey,&Pair,BodyA,BodyB,CO::Completed,bUseRollingManifold,Impulse,ContactPoint,NormalImpulseMagnitude,Separation,&RollingHit,&RollingFakePhysicsContext);
#endif // IAmSpeed contact witness1075 END
			Sphere->ApplyFakePhysicsOn(
				*Box, RollingHit, Dt, &RollingFakePhysicsContext);
#if !UE_BUILD_SHIPPING // IAmSpeed contact witness1075 BEGIN
 if(ContactWitnessPass)CW::RecordDynamicEvent(ContactWitnessPass,CE::FakePhysicsAfter,Index,
  Pair.PairKey,&Pair,BodyA,BodyB,CO::Completed,bUseRollingManifold,Impulse,ContactPoint,NormalImpulseMagnitude,Separation,&RollingHit,&RollingFakePhysicsContext);
#endif // IAmSpeed contact witness1075 END

		}
	}
}

void USpeedWorldSubsystem::ProjectDynamicContactPairs()
{
    Speed::PreparedExecution::FStepScope::RequireNativeContactPairTargets(SimulationWorld);
	auto& DynamicContactPairs = SimulationWorld.GetDynamicContactPairs();
	auto& PendingRollingContactPairs = SimulationWorld.GetPendingRollingContactPairs();
#if !UE_BUILD_SHIPPING // IAmSpeed contact witness1075 BEGIN

 using CW=Speed::FFrameSolveWitnessScope;
 using CE=Speed::EFrameDynamicContactEvent;
 using CO=Speed::EFrameDynamicContactOutcome;
 const uint64 ContactWitnessPass=CW::IsActive()?CW::BeginDynamicPass(SimulationWorld.GetCurrentStepFrame(),
  Speed::EFrameDynamicContactPass::Projection,0.0f,0,false,false,DynamicContactPairs.Num()):0;
 ON_SCOPE_EXIT {if(ContactWitnessPass)CW::EndDynamicPass(ContactWitnessPass,DynamicContactPairs.Num());};
#endif // IAmSpeed contact witness1075 END

	if (!USphereSubBody::IsSphereBoxProjectionEnabled())
	{
#if !UE_BUILD_SHIPPING // IAmSpeed contact witness1075 BEGIN

 if(ContactWitnessPass)CW::RecordDynamicEvent(ContactWitnessPass,CE::PassDisabled,
  0,0,nullptr,nullptr,nullptr,CO::ProjectionDisabled);
#endif // IAmSpeed contact witness1075 END

		return;
	}

	const auto ProjectPairs = [](TArray<FDynamicContactPair>& Pairs)
	{
#if !UE_BUILD_SHIPPING // IAmSpeed contact witness1075 BEGIN

 const uint64 WitnessProjectionPass=CW::CurrentDynamicPass();int32 WitnessProjectionIndex=0;
#endif // IAmSpeed contact witness1075 END

		for (FDynamicContactPair& Pair : Pairs)
		{
#if !UE_BUILD_SHIPPING // IAmSpeed contact witness1075 BEGIN

 const int32 WitnessIndex=WitnessProjectionIndex++;
 const USSubBody* WitnessBodyA=WitnessProjectionPass?Pair.BodyA.Get():nullptr;
 const USSubBody* WitnessBodyB=WitnessProjectionPass?Pair.BodyB.Get():nullptr;
 if(WitnessProjectionPass)CW::RecordDynamicEvent(WitnessProjectionPass,CE::PairBegin,
  WitnessIndex,Pair.PairKey,&Pair,WitnessBodyA,WitnessBodyB);
 ON_SCOPE_EXIT {if(WitnessProjectionPass)CW::RecordDynamicEvent(WitnessProjectionPass,
  CE::PairEnd,WitnessIndex,Pair.PairKey,&Pair,WitnessBodyA,WitnessBodyB);};
#endif // IAmSpeed contact witness1075 END

			USolidSubBody* BodyA = Pair.BodyA.Get();
			USolidSubBody* BodyB = Pair.BodyB.Get();
			if (!BodyA || !BodyB)
			{
				continue;
			}

			USphereSubBody* Sphere = Cast<USphereSubBody>(BodyA);
			UBoxSubBody* Box = Cast<UBoxSubBody>(BodyB);
			if (!Sphere || !Box)
			{
				Sphere = Cast<USphereSubBody>(BodyB);
				Box = Cast<UBoxSubBody>(BodyA);
			}
			if (Sphere && Box)
			{
#if !UE_BUILD_SHIPPING // IAmSpeed contact witness1075 BEGIN
 if(WitnessProjectionPass)CW::RecordDynamicEvent(WitnessProjectionPass,CE::ProjectBefore,
  WitnessIndex,Pair.PairKey,&Pair,Sphere,Box,CO::Completed);
#endif // IAmSpeed contact witness1075 END

				Sphere->ProjectOutOfBox(*Box);
#if !UE_BUILD_SHIPPING // IAmSpeed contact witness1075 BEGIN
 if(WitnessProjectionPass)CW::RecordDynamicEvent(WitnessProjectionPass,CE::ProjectAfter,
  WitnessIndex,Pair.PairKey,&Pair,Sphere,Box,CO::Completed);
#endif // IAmSpeed contact witness1075 END

			}
		}
	};


#if !UE_BUILD_SHIPPING // IAmSpeed contact witness1075 BEGIN
 if(ContactWitnessPass)CW::RecordDynamicEvent(ContactWitnessPass,CE::ProjectActiveBegin,
  DynamicContactPairs.Num(),0,nullptr,nullptr,nullptr);
#endif // IAmSpeed contact witness1075 END
	ProjectPairs(DynamicContactPairs);
#if !UE_BUILD_SHIPPING // IAmSpeed contact witness1075 BEGIN
 if(ContactWitnessPass)CW::RecordDynamicEvent(ContactWitnessPass,CE::ProjectActiveEnd,
  DynamicContactPairs.Num(),0,nullptr,nullptr,nullptr);
#endif // IAmSpeed contact witness1075 END


#if !UE_BUILD_SHIPPING // IAmSpeed contact witness1075 BEGIN
 if(ContactWitnessPass)CW::RecordDynamicEvent(ContactWitnessPass,CE::ProjectPendingBegin,
  PendingRollingContactPairs.Num(),0,nullptr,nullptr,nullptr);
#endif // IAmSpeed contact witness1075 END
	ProjectPairs(PendingRollingContactPairs);
#if !UE_BUILD_SHIPPING // IAmSpeed contact witness1075 BEGIN
 if(ContactWitnessPass)CW::RecordDynamicEvent(ContactWitnessPass,CE::ProjectPendingEnd,
  PendingRollingContactPairs.Num(),0,nullptr,nullptr,nullptr);
#endif // IAmSpeed contact witness1075 END


	// New encounters and teleports do not necessarily own a persistent
	// manifold yet. Measure every independent sphere/box pair from refreshed
	// parent states so the next frame cannot inherit an overlap.

#if !UE_BUILD_SHIPPING // IAmSpeed contact witness1075 BEGIN

 int32 WitnessFreshProjectionIndex=0;
 if(ContactWitnessPass)CW::RecordDynamicEvent(ContactWitnessPass,CE::ProjectFreshBegin,
  SimulationWorld.GetProjectionSpheres().Num(),0,nullptr,nullptr,nullptr);
#endif // IAmSpeed contact witness1075 END
	for (USphereSubBody* Sphere : SimulationWorld.GetProjectionSpheres())
	{
		for (UBoxSubBody* Box : SimulationWorld.GetProjectionBoxes())
		{
			if (Sphere && Box &&
				Sphere->GetParentComponent() != Box->GetParentComponent())
			{
#if !UE_BUILD_SHIPPING // IAmSpeed contact witness1075 BEGIN

 if(ContactWitnessPass)CW::RecordDynamicEvent(ContactWitnessPass,CE::ProjectBefore,
  WitnessFreshProjectionIndex++,0,nullptr,Sphere,Box,CO::Completed);
#endif // IAmSpeed contact witness1075 END

				Sphere->ProjectOutOfBox(*Box);
#if !UE_BUILD_SHIPPING // IAmSpeed contact witness1075 BEGIN

 if(ContactWitnessPass)CW::RecordDynamicEvent(ContactWitnessPass,CE::ProjectAfter,
  WitnessFreshProjectionIndex-1,0,nullptr,Sphere,Box,CO::Completed);
#endif // IAmSpeed contact witness1075 END

			}
		}
	}
#if !UE_BUILD_SHIPPING // IAmSpeed contact witness1075 BEGIN
 if(ContactWitnessPass)CW::RecordDynamicEvent(ContactWitnessPass,CE::ProjectFreshEnd,
  WitnessFreshProjectionIndex,0,nullptr,nullptr,nullptr);
#endif // IAmSpeed contact witness1075 END

}

bool USpeedWorldSubsystem::BeginCanonicalFrame(FString& OutReason)
{
    if(StaticWorldPublicationDepth){OutReason=TEXT("HostedGT static World publication active");return false;}
	if (bCanonicalFrameActive)
	{
		OutReason = TEXT("canonical frame already active");
		return false;
	}
	if (!ValidateSimulationBindings(OutReason)) return false;
    Speed::PreparationPrefix::FEffectScope effect(Speed::PreparationPrefix::EEffect::AdmitCanonicalFrame);
	bCanonicalFrameActive = true;
    try { effect.Returned(); }
    catch (...)
    {
        // This is canonical ownership cleanup, never a restoration of physical
        // or input effects. Host's later lexical cleanup does not exist yet.
        bCanonicalFrameActive=false;throw;
    }
	return true;
}

bool USpeedWorldSubsystem::ValidateSimulationBindings(FString& OutReason)
{
    Speed::PreparationPrefix::FEffectScope::VerifyStructuralBoundary();
	ApplyPendingOps();
	RebuildSortedIfNeeded();
	for (const ISpeedComponent* Component : SimulationWorld.GetOrderedAdapters())
	{
        if (Component)
        {
            Speed::PreparationPrefix::FEffectScope effect(Speed::PreparationPrefix::EEffect::ValidateOwnerBinding,Component);
            const bool valid=Component->ValidateSimulationBindings(OutReason);
            effect.Returned(valid,valid); if(!valid)return false;
        }
	}
	return true;
}

bool USpeedWorldSubsystem::ServiceInputRetirementsAtBoundary()
{
	for (ISpeedComponent* Component : SimulationWorld.GetAdapters())
		if (Component && !Component->ServiceInputRetirementAtBoundary()) return false;
	return true;
}

bool USpeedWorldSubsystem::RetireInputProcessingOnWorker()
{
	bool Retired = true;
	for (ISpeedComponent* Component : SimulationWorld.GetAdapters())
		if (Component && !Component->RetireInputProcessingOnWorker()) Retired = false;
	return Retired;
}

bool USpeedWorldSubsystem::NeutralizeCanonicalInputs(const Speed::Input::V2::FInputRegistryView& View)
{
	if (bCanonicalFrameActive) return false;
	for (const auto& Session : View.Bindings)
		for (const auto& Actor : Session.Actors)
		{
			ISpeedComponent* Match = nullptr;
			for (ISpeedComponent* Component : SimulationWorld.GetOrderedAdapters())
				if (Component && Component->GetPublishedSimulationStableId() == Actor.Id) { Match = Component; break; }
			if (!Match || Actor.Generation != Actor.Id || !Match->NeutralizeCanonicalInputAtBoundary()) return false;
		}
	return true;
}

bool USpeedWorldSubsystem::InstallCanonicalInputs(const Speed::Input::V2::FRegistryFrame& Frame)
{
	if (!bCanonicalFrameActive || !Frame.Registry || Frame.Registry->Terminal) return false;
	// Resolve every identity before the first install. Stable ids are monotonic
	// for this world lifetime; generation equals that non-reused registration id.
	TArray<TPair<ISpeedComponent*, std::shared_ptr<const Speed::Input::V2::FOwnerInputSnapshot>>, TInlineAllocator<16>> Installs;
	for (const auto& Session : Frame.Inputs)
	{
		if (!Session.Snapshot || Session.Snapshot->Input.GetData().ConsumptionFrame != Frame.Frame) return false;
		for (const auto& Actor : Session.Actors)
		{
			ISpeedComponent* Match = nullptr;
			for (ISpeedComponent* Component : SimulationWorld.GetOrderedAdapters())
				if (Component && Component->GetPublishedSimulationStableId() == Actor.Id) { Match = Component; break; }
			if (!Match || Actor.Generation != Actor.Id) return false;
			Installs.Emplace(Match, Session.Snapshot);
		}
	}
	for (const auto& Install : Installs)
    {
        Speed::PreparationPrefix::FEffectScope effect(Speed::PreparationPrefix::EEffect::InstallOwnerInput,Install.Key);
        const bool installed=Install.Key->InstallCanonicalInput(Frame.Frame,Install.Value);
        effect.Returned(installed,installed);if(!installed)return false;
    }
	return true;
}

bool USpeedWorldSubsystem::StageCanonicalScenarioInputs(const FCanonicalFrameContext& Context,
	Speed::Input::V2::FInputSessionRegistry& Registry)
{
	if (!bCanonicalFrameActive) return false;
	for (ISpeedComponent* Component : SimulationWorld.GetOrderedAdapters())
        if (Component)
        {
            Speed::PreparationPrefix::FEffectScope effect(Speed::PreparationPrefix::EEffect::StageScenarioInput,Component);
            const bool staged=Component->StageCanonicalScenarioInput(Context,Registry);
            effect.Returned(staged,staged);if(!staged)return false;
        }
	return true;
}

bool USpeedWorldSubsystem::PrepareCanonicalInputs(const FCanonicalFrameContext& Context)
{
	// BeginCanonicalFrame has already frozen/validated the active adapter view.
	for (ISpeedComponent* Component : SimulationWorld.GetOrderedAdapters())
        if (Component)
        {
            Speed::PreparationPrefix::FEffectScope effect(Speed::PreparationPrefix::EEffect::PrepareOwnerInput,Component);
            const bool prepared=Component->PrepareCanonicalInputs(Context);
            effect.Returned(prepared,prepared);if(!prepared)return false;
        }
	return true;
}

#if !UE_BUILD_SHIPPING
bool USpeedWorldSubsystem::InspectBodyContactPairsForTesting(
 const USolidSubBody& Body,uint64 ExpectedPreparingFrame,Speed::FBodyContactPairInspection& Out) const
{
 Out=Speed::FBodyContactPairInspection();Out.PreparingFrame=ExpectedPreparingFrame;
 // Owned simulation lane only; no game-thread or completed-frame query contract.
 if(!bCanonicalFrameActive || TestingPreparingFrame!=ExpectedPreparingFrame || StepSerial==0)
 {Out.Failure=6;return false;}
 const bool Complete=SimulationWorld.InspectBodyContactPairsForTesting(Body,ExpectedPreparingFrame,Out);
 Out.WorldStepSerial=StepSerial;return Complete;
}
#endif

void USpeedWorldSubsystem::PrepareCanonicalFrame(
	const FCanonicalFrameContext& Context)
{
    Speed::PreparationPrefix::FEffectScope::VerifyStructuralBoundary();
#if !UE_BUILD_SHIPPING
    const uint64 PreviousTestingFrame=TestingPreparingFrame;
    TestingPreparingFrame=Context.NumFrame;
    ON_SCOPE_EXIT{TestingPreparingFrame=PreviousTestingFrame;};
#endif
	ApplyPendingOps();
	RebuildSortedIfNeeded();

	for (bool bPrescribed : {true, false})
	for (ISpeedComponent* Component : SimulationWorld.GetOrderedAdapters())
	{
        if(!Component)continue;
        bool prescribed=false;
        {
            Speed::PreparationPrefix::FEffectScope effect(Speed::PreparationPrefix::EEffect::ReadPrescribedMotion,Component,int32(bPrescribed));
            prescribed=Component->IsPrescribedMotion();effect.Returned(true,prescribed);
        }
		if (prescribed==bPrescribed)
		{
			IAMSPEED_ACTOR_SCOPE(SimulationWorld, *Component, Prepare);
            {
                Speed::PreparationPrefix::FEffectScope effect(Speed::PreparationPrefix::EEffect::BindStaticWorld,Component,int32(bPrescribed));
			Component->SetStaticCollisionWorldForFrame(
				CVarIAmSpeedExactBoxRestingSupport.GetValueOnAnyThread() != 0 &&
				Speed::Analytic::FStaticWorldQueryAudit::IsSurfaceAnalyticBackend()
					? SimulationWorld.GetStaticCollisionWorld() : nullptr, Context.PhysicalDeltaTime);
                effect.Returned();
            }
            {
                Speed::PreparationPrefix::FEffectScope effect(Speed::PreparationPrefix::EEffect::PrepareOwner,Component,int32(bPrescribed));
                Speed::PreparedExecution::FOrdinaryOwnerProbePhaseScope probes(
                    Speed::PreparedExecution::ENativeOwnerProbePhase::Prepare,*Component,Context.PhysicalDeltaTime);
                if(!Speed::PreparedExecution::FCanonicalFramePreflight::DispatchNativePrepare(*Component,Context))Component->PrepareCanonicalFrame(Context);
                probes.Returned();effect.Returned();
            }
		}
	}
}

ECanonicalRunControlState USpeedWorldSubsystem::GetCanonicalRunControlState()
{
	ApplyPendingOps();
	RebuildSortedIfNeeded();

	bool bFoundController = false;
	bool bAllReady = true;
	bool bAllComplete = true;
	for (const ISpeedComponent* Component : SimulationWorld.GetOrderedAdapters())
	{
		if (!Component || !Component->IsCanonicalRunController())
		{
			continue;
		}

		bFoundController = true;
		bAllReady &= Component->IsCanonicalRunReady();
		bAllComplete &= Component->IsCanonicalRunComplete();
	}

	if (!bFoundController)
	{
		return ECanonicalRunControlState::Uncontrolled;
	}
	if (!bAllReady)
	{
		return ECanonicalRunControlState::WaitingForScenario;
	}
	return bAllComplete
		? ECanonicalRunControlState::Complete
		: ECanonicalRunControlState::Ready;
}

#if !UE_BUILD_SHIPPING
ECanonicalRunControlState USpeedWorldSubsystem::InspectAdmittedCanonicalRunControlForTesting() const
{
	bool bFoundController = false;
	bool bAllReady = true;
	bool bAllComplete = true;
	for (const ISpeedComponent* Component : SimulationWorld.GetOrderedAdapters())
	{
		if (!Component || !Component->IsCanonicalRunController())
		{
			continue;
		}

		bFoundController = true;
		bAllReady &= Component->IsCanonicalRunReady();
		bAllComplete &= Component->IsCanonicalRunComplete();
	}

	if (!bFoundController)
	{
		return ECanonicalRunControlState::Uncontrolled;
	}
	if (!bAllReady)
	{
		return ECanonicalRunControlState::WaitingForScenario;
	}
	return bAllComplete
		? ECanonicalRunControlState::Complete
		: ECanonicalRunControlState::Ready;
}
#endif

void USpeedWorldSubsystem::NotifyCanonicalFramePublished(uint64 NumFrame)
{
	SimulationWorld.NotifyCanonicalFramePublished(NumFrame);
}

ECanonicalPublicationResult USpeedWorldSubsystem::PublishCanonicalFrame(uint64 Frame, TFunctionRef<bool()> Publish)
{
	return SimulationWorld.PublishCanonicalFrame(Frame, Publish);
}

bool USpeedWorldSubsystem::DispatchPostPrepareInitialPosition(const ASpeedPhysicsSimulation& host)
{
    using namespace Speed::PreparationPrefix;
    const auto* c=FCursor::Current;
    if(!c||c->Observing||c->PositionDispatch||c->Context.Bridge!=this||c->Context.Host!=&host||
        !c->CurrentIdentity()||!Speed::OnePausedWork::HasInitialPosition(c->Contract->GetPurpose()))return false;
    // PendingCS is recursive on this native Windows target. CurrentIdentity
    // rechecks it inside every Source observation while this outer lock holds.
    FScopeLock lock(&PendingCS);
    if(!PendingOps.IsEmpty()||!SimulationWorld.GetDynamicContactPairs().IsEmpty()||
        !SimulationWorld.GetPendingRollingContactPairs().IsEmpty()||!c->CurrentIdentity())return false;
    return FCursor::ApplyPostPrepareInitialPosition(host);
}
bool USpeedWorldSubsystem::RetirePreparationInputTransactions(const FCanonicalFrameContext& context)
{
    using namespace Speed::PreparationPrefix;
    auto* c=FCursor::Current;
    if(!c||c->Observing||c->Context.Bridge!=this||c->Context.Frame!=context.NumFrame||
        !c->CurrentIdentity()||!c->Stack.IsEmpty()||c->Contract->Reached||c->Contract->AllPrepareReached)
        return false;
    const auto& ordered=SimulationWorld.GetOrderedAdapters();
    const auto purpose=c->Contract->GetPurpose();
    const bool preparation=purpose==Speed::OnePausedWork::EPurpose::PreparationPrefixStop&&c->NextCall==30&&c->Contract->Calls.Num()==33;
    const bool position=purpose==Speed::OnePausedWork::EPurpose::PostPrepareInitialPositionStop&&c->NextCall==31&&c->Contract->Calls.Num()==34&&
        c->Contract->HasAcceptedPosition()&&c->Contract->Records[30].Attempted&&c->Contract->Records[30].Returned&&c->Contract->Records[30].Accepted;
    const bool sync=purpose==Speed::OnePausedWork::EPurpose::TerminalStepSyncStop&&c->NextCall==35&&c->Contract->Calls.Num()==38&&
        c->Contract->HasCompletedStepSync()&&c->CurrentStep==c->Window->GetExpectedStep();
    const bool capture=purpose==Speed::OnePausedWork::EPurpose::TerminalSphereRsCaptureStop&&c->NextCall==38&&c->Contract->Calls.Num()==41&&
        c->Contract->HasClosedNativeRsCapture()&&c->CurrentStep==c->Window->GetExpectedStep();
    const bool consumedAdvance=purpose==Speed::OnePausedWork::EPurpose::TerminalConsumedPrefixAdvanceStop&&c->NextCall==42&&c->Contract->Calls.Num()==45&&
        c->Contract->HasCompletedNativeRsAdvance()&&c->CurrentStep==c->Window->GetExpectedStep();
    const bool contactAdvance=purpose==Speed::OnePausedWork::EPurpose::TerminalRsContactAdvanceStop&&c->NextCall==42&&c->Contract->Calls.Num()==45&&
        c->Contract->HasCompletedNativeRsContactAdvance()&&c->CurrentStep==c->Window->GetExpectedStep();
    const bool rsResponse=purpose==Speed::OnePausedWork::EPurpose::TerminalRsResponseStop&&c->NextCall==46&&c->Contract->Calls.Num()==49&&
        c->Contract->HasCompletedNativeRsResponse()&&c->CurrentStep==c->Window->GetExpectedStep();
    const bool rsResponseSync=purpose==Speed::OnePausedWork::EPurpose::TerminalRsResponseSyncStop&&c->NextCall==47&&c->Contract->Calls.Num()==50&&
        c->Contract->HasCompletedNativeRsResponseSync()&&c->CurrentStep==c->Window->GetExpectedStep();
    const bool postSyncRs=purpose==Speed::OnePausedWork::EPurpose::TerminalPostSyncRsCaptureStop&&c->NextCall==50&&c->Contract->Calls.Num()==53&&
        c->Contract->HasCompletedNativePostSyncRsCapture()&&c->CurrentStep==c->Window->GetExpectedStep();
    if(ordered.Num()!=3||(!preparation&&!position&&!sync&&!capture&&!consumedAdvance&&!contactAdvance&&!rsResponse&&!rsResponseSync&&!postSyncRs))return false;
    // Authenticate the COMPLETE suffix before dispatching the first callback.
    // The closed producer also proves exact classes, no reservation/commit lock
    // and every input/endpoint transition. This check alone grants no contract.
    for(int32 i=0;i<ordered.Num();++i)
    {
        const auto* owner=ordered[i];const auto& call=c->Contract->Calls[c->NextCall+i];
        if(!owner||!c->Window->GetSnapshot()->GetNativeOwners()->Find(owner)||
            call.Effect!=EEffect::RetireOwnerInput||call.Owner!=owner||call.Pass!=INDEX_NONE)return false;
    }
    for(int32 i=0;i<c->NextCall;++i)
        if(!c->IsClosedPlannedRow(i))return false;
    c->Contract->AllPrepareReached=true;
    for(auto* owner:ordered)
    {
        FEffectScope effect(EEffect::RetireOwnerInput,owner);
        owner->AbortCanonicalFrame(context.NumFrame,ECanonicalFrameAbortReason::TerminalPreparationRetirement);
        effect.Returned();
    }
    return true;
}
void USpeedWorldSubsystem::AbortCanonicalFrame(uint64 Frame, ECanonicalFrameAbortReason Reason) noexcept
{
	SimulationWorld.AbortCanonicalFrame(Frame, Reason);
}

FSimulationSnapshot USpeedWorldSubsystem::CaptureSimulationSnapshot(
	const uint64 NumFrame, const uint64 InputJournalHash, const bool bIncludePresentation)
{
	RebuildSortedIfNeeded();
	return SimulationWorld.CaptureSnapshot(NumFrame, InputJournalHash, bIncludePresentation);
}

bool USpeedWorldSubsystem::RestoreSimulationSnapshot(
	const FSimulationSnapshot& Snapshot,
	const uint64 ExpectedInputJournalHash)
{
	ApplyPendingOps();
	RebuildSortedIfNeeded();
	return SimulationWorld.RestoreSnapshot(Snapshot, ExpectedInputJournalHash);
}

uint64 USpeedWorldSubsystem::GetSimulationStableId(const ISpeedComponent& Component)
{
	FScopedGameThreadAdapterAccess Access(GetWorld());
	if (!Access.IsValid()) return 0;
	ApplyPendingOps();
	RebuildSortedIfNeeded();
	return SimulationWorld.FindStableId(Component);
}

bool USpeedWorldSubsystem::ApplySimulationInputs(
	const uint64 NumFrame, const Speed::SimulationBoundary::FInputJournal& Journal)
{
	RebuildSortedIfNeeded();
	return SimulationWorld.ApplyInputsForFrame(NumFrame, Journal);
}

#if WITH_DEV_AUTOMATION_TESTS
namespace Speed::Testing
{
namespace
{
thread_local FCcdPoseAuditObserver CcdObserver=nullptr;
thread_local void* CcdContext=nullptr;
thread_local const ISpeedComponent* CcdPrimitiveOwner=nullptr;
thread_local uint64 CcdRunGeneration=0,CcdPrimitiveSerial=0;
thread_local bool CcdPrimitiveDelivering=false,CcdPrimitiveViolation=false;
}
bool SetCcdPoseAuditObserverForCurrentThread(FCcdPoseAuditObserver Observer, void* Context) noexcept
{
    if(!Observer || !Context || CcdObserver || CcdPrimitiveDelivering || CcdRunGeneration==MAX_uint64) return false;
    ++CcdRunGeneration; CcdPrimitiveSerial=0; CcdPrimitiveOwner=nullptr; CcdPrimitiveViolation=false;
    CcdObserver=Observer; CcdContext=Context; return true;
}
bool ClearCcdPoseAuditObserverForCurrentThread(FCcdPoseAuditObserver Observer, void* Context) noexcept
{
    if(CcdPrimitiveDelivering || CcdObserver!=Observer || CcdContext!=Context) return false;
    CcdObserver=nullptr; CcdContext=nullptr; CcdPrimitiveOwner=nullptr; return true;
}
bool EnableCcdPrimitiveCaptureForCurrentThread(FCcdPoseAuditObserver Observer,void* Context,const ISpeedComponent* OwnerIdentity) noexcept
{
    if(!Observer || !Context || !CcdObserver || !OwnerIdentity || CcdPrimitiveDelivering || CcdPrimitiveViolation || CcdObserver!=Observer || CcdContext!=Context || CcdPrimitiveOwner) return false;
    CcdPrimitiveOwner=OwnerIdentity; return true;
}
bool IsCcdPrimitiveCaptureEnabledForCurrentThread(const ISpeedComponent* OwnerIdentity) noexcept
{
    return CcdObserver && !CcdPrimitiveViolation && OwnerIdentity && CcdPrimitiveOwner==OwnerIdentity;
}
FCcdPrimitiveStamp NextCcdPrimitiveCaptureStampForCurrentThread() noexcept
{
    if(!CcdObserver || !CcdPrimitiveOwner || CcdPrimitiveViolation || CcdPrimitiveSerial==MAX_uint64) return {};
    return {CcdRunGeneration,++CcdPrimitiveSerial};
}
bool CcdPrimitiveCaptureHadViolationForCurrentThread() noexcept { return CcdPrimitiveViolation; }
void MarkCcdPrimitiveCaptureViolationForCurrentThread() noexcept
{
    CcdPrimitiveViolation=true;CcdPrimitiveOwner=nullptr;
}
void ObserveCcdPrimitiveForCurrentThread(ECcdPoseAuditStage Stage,FCcdPrimitiveCapture Sample) noexcept
{
    if(CcdPrimitiveDelivering || !IsCcdPrimitiveCaptureEnabledForCurrentThread(static_cast<const ISpeedComponent*>(Sample.OwnerIdentity)) || !Sample.Serial || Sample.RunGeneration!=CcdRunGeneration || Sample.Serial>CcdPrimitiveSerial) return;
    if(Stage!=ECcdPoseAuditStage::SphereSweepCapture && Stage!=ECcdPoseAuditStage::BeforePrimitiveIntegrate && Stage!=ECcdPoseAuditStage::AfterPrimitiveIntegrate) return;
    CcdPrimitiveDelivering=true;
    CcdObserver(CcdContext,{nullptr,Sample.OwnerFrame,Stage,0.f,Sample.Delta,&Sample});
    CcdPrimitiveDelivering=false;
}
static void ObserveCcdPose(const USpeedWorldSubsystem* World, uint32 Frame, ECcdPoseAuditStage Stage,
    float TimePassed, float SubDelta) noexcept
{
    if(CcdObserver) CcdObserver(CcdContext,{World,Frame,Stage,TimePassed,SubDelta});
}
}
#endif

void USpeedWorldSubsystem::Step(const float& Dt, const float& SimTime, const unsigned int& Frame)
{
    if(StaticWorldPublicationDepth)throw std::runtime_error("HostedGT static World publication active");
    Speed::OnePausedWork::FCanonicalFrameWindowPtr window;
    const auto* currentProvider=SimulationWorld.GetStaticCollisionWorld();
    if(bPreparedIntervalRouteRequested&&currentProvider&&currentProvider->GetPreparedSnapshot()&&currentProvider->GetPreparedSnapshot()->GetReference()&&
        !Speed::OnePausedWork::ReadExecutionWindow(window))throw std::runtime_error("Retained reference Step has no live GT execution window");
    // This effect is inert without a closed native cursor. In the distinct
    // terminal purpose it authenticates the census/pending queue BEFORE ++StepSerial.
    Speed::PreparationPrefix::FEffectScope TerminalStepEntry(Speed::PreparationPrefix::EEffect::EnterTerminalStep);
    if(!Speed::OnePausedWork::BeforeWorldEffects(this,currentProvider,StepSerial,Frame,Dt,SimTime))
        throw std::runtime_error("Native execution window refused before Step effects");
    if(const auto* provider=SimulationWorld.GetStaticCollisionWorld())
        if(!provider->IsReadyForPhysicsStep()&&!Speed::PreparationPrefix::FCursor::IsTerminalStepSyncEntry(*this))
            throw std::runtime_error("Prepared static World has no qualified physical consumer");
#if !UE_BUILD_SHIPPING
    Speed::Analytic::CaseSatDiagnostics::FScopedCpuPhase400 Phase400(
        Speed::Analytic::CaseSatDiagnostics::ECpuPhase400::SubsystemStep);
#endif
	unsigned int& CurrentStepFrame = SimulationWorld.GetCurrentStepFrame();
	const double StepStartSeconds = FPlatformTime::Seconds();
	LastStepDiagnostics = FSpeedStepDiagnostics();
	LastStepDiagnostics.Serial = ++StepSerial;
    if(!Speed::OnePausedWork::ConfirmWorldSerial(StepSerial)||!Speed::PreparationPrefix::FCursor::ConfirmTerminalStepSerial(*this))
        throw std::runtime_error("Native assigned Step serial differs from its pre-effect window");
	LastStepDiagnostics.Frame = Frame;
	LastStepDiagnostics.PhysicalDeltaTimeMilliseconds =
		static_cast<double>(Dt) * 1000.0;
	constexpr int32 MaxIter = 24;
	LastStepDiagnostics.MaximumIterationCount = MaxIter;
#if WITH_DEV_AUTOMATION_TESTS
    Speed::Testing::ObserveCcdPose(this,Frame,Speed::Testing::ECcdPoseAuditStage::FrameBegin,0.f,0.f);
#endif

#if !UE_BUILD_SHIPPING
	const bool bCaptureSolverEventTrace =
		CVarIAmSpeedSolverEventTraceFrame.GetValueOnAnyThread() == static_cast<int32>(Frame);
	const auto TraceEvent = [&](const TCHAR* Stage, const TCHAR* Disposition,
		const int32 Iteration, const int32 Order, const uint64 PairKey,
		const SHitResult& Hit, USSubBody* Resolver, const int32 GenerationBefore,
		const int32 GenerationAfter)
	{
		if (!bCaptureSolverEventTrace) return;
		ISpeedComponent* Parent = Resolver ? Resolver->GetParentComponent() : nullptr;
		const uint64 ComponentStableId = Parent ? SimulationWorld.FindStableId(*Parent) : 0;
		const int32 SubBodyIndex = Parent ? Parent->GetSubBodies().IndexOfByKey(Resolver) : INDEX_NONE;
		UE_LOG(LogTemp, Display,
			TEXT("[SpeedSolverEventV2] Frame=%u Iter=%d Stage=%s Disposition=%s Order=%d ComponentStableId=%llu ResolverClass=%s SubBodyIndex=%d PairKey=%llu SourceId=%llu SurfaceId=%llu FeatureId=%llu PrimitiveId=%llu ThisFeature=%d ThisIndex=%d OtherFeature=%d OtherIndex=%d HitTOI=%.9g GenerationBefore=%d GenerationAfter=%d"),
			Frame, Iteration, Stage, Disposition, Order,
			static_cast<unsigned long long>(ComponentStableId),
			Resolver ? *Resolver->GetClass()->GetName() : TEXT("None"), SubBodyIndex,
			static_cast<unsigned long long>(PairKey),
			static_cast<unsigned long long>(Hit.SourceId),
			static_cast<unsigned long long>(Hit.SurfaceId),
			static_cast<unsigned long long>(Hit.FeatureId),
			static_cast<unsigned long long>(Hit.PrimitiveId),
			static_cast<int32>(Hit.ContactFeatureThis),
			static_cast<int32>(Hit.ContactFeatureIndexThis),
			static_cast<int32>(Hit.ContactFeatureOther),
			static_cast<int32>(Hit.ContactFeatureIndexOther), Hit.TOI,
			GenerationBefore, GenerationAfter);
	};
#endif
	CurrentStepFrame = Frame;
    ApplyPendingOps();
    RebuildSortedIfNeeded();
    Speed::FPreparedPhysicalQueryDomainPtr PhysicalFilters;
    if(bPreparedIntervalRouteRequested)
    {
        const auto* base=SimulationWorld.GetStaticCollisionWorld();
        if(!base||!PreparedIntervalScene)throw std::runtime_error("Prepared native Step input absent");
        const auto snapshot=base->GetPreparedSnapshot();
        if(!snapshot||snapshot->GetNominalScene()!=PreparedIntervalScene)
            throw std::runtime_error("Prepared native Step provider snapshot type/identity differs");
        PhysicalFilters=snapshot->GetFilters();
    }
    Speed::PreparedExecution::FStepScope PhysicalStep(bPreparedIntervalRouteRequested,*this,SimulationWorld,
        SimulationWorld.GetStaticCollisionWorld(),StepSerial,Frame,Dt,SimTime,PreparedIntervalScene,MoveTemp(PhysicalFilters));

    if (Dt <= 0.f || SimulationWorld.GetOrderedAdapters().Num() == 0)
	{
		LastStepDiagnostics.TotalMilliseconds =
			(FPlatformTime::Seconds() - StepStartSeconds) * 1000.0;
        return;
	}

    // ------------------------------------------------------------
    // 1) Reset frame
    // ------------------------------------------------------------
	const double ResetStartSeconds = FPlatformTime::Seconds();
	// A shadow-built analytical payload is not the selected collision authority.
	// Keep Legacy/Hybrid unchanged, even if analytical diagnostic data is loaded.
	const Speed::IStaticCollisionWorld* RestingWorld =
		CVarIAmSpeedExactBoxRestingSupport.GetValueOnAnyThread() != 0 &&
		Speed::Analytic::FStaticWorldQueryAudit::IsSurfaceAnalyticBackend()
			? SimulationWorld.GetStaticCollisionWorld() : nullptr;
	const Speed::FFrameParallelSettings ParallelPreparationSettings = Speed::CaptureFrameParallelSettings();
    Speed::FrameParallel::FJoinedActorKinematics1213 JoinedActors1213;
    if(Speed::FrameParallel::IsJoinedActorKinematicWaveEnabled1213())
        JoinedActors1213.CaptureAndCompute(SimulationWorld.GetOrderedAdapters(),
            ParallelPreparationSettings.LaneLimit,ParallelPreparationSettings.MinPrepareItems);
    TerminalStepEntry.Returned();
	for (ISpeedComponent* Comp : SimulationWorld.GetOrderedAdapters())
    {
        if (!Comp) continue;

        Speed::PreparationPrefix::FEffectScope TerminalOwnerSync(Speed::PreparationPrefix::EEffect::SyncStepOwner,Comp);
        {
		IAMSPEED_ACTOR_SCOPE(SimulationWorld, *Comp, Reset);
        // Preserve the exact pre-existing native child-batch admission and
        // fallback choice. Shadow only observes around this original block.
        const auto OriginalOwnerOperation1213=[&]()
        {
            if(!Speed::FrameParallel::TryUpdateSubBodyKinematicsInParallel(*Comp,
                ParallelPreparationSettings.LaneLimit,ParallelPreparationSettings.MinPrepareItems))
                Comp->UpdateSubBodiesKinematics();
        };
        if(!JoinedActors1213.TryInstall(*Comp,OriginalOwnerOperation1213))
            OriginalOwnerOperation1213();
        Comp->ResetForFrame(Dt);
		Comp->SetStaticCollisionWorldForFrame(RestingWorld, Dt);
        }
        TerminalOwnerSync.Returned();
    }
    if(Speed::PreparationPrefix::FCursor::FinishTerminalStepSync(*this))return;
    const bool TerminalRsCapture=Speed::PreparationPrefix::FCursor::IsCurrentRsNativeWindow(*this);

	LastStepDiagnostics.ResetMilliseconds =
		(FPlatformTime::Seconds() - ResetStartSeconds) * 1000.0;

    // Anti double-resolve (pair) on the frame
    Speed::Collision::FResolvedPairSet ResolvedPairs;

    float TimePassed = 0.f;
    float LastSubDelta = Dt;
	const float CompletionTolerance = FMath::Max(1.0e-8f, Dt * 1.0e-6f);
	TArray<Speed::PreparedExecution::FComponentSweepEvidencePtr> PreparedSweepBundles;
	const auto IntegrateAll = [&](const float SubDelta,const bool HasActualSelection=false,
        const TConstArrayView<SComponentTOI> SelectedBatch={},const float SweepHorizon=-1.f)
	{
        PhysicalStep.RequireAdvanceEvidence(MakeArrayView(PreparedSweepBundles),SubDelta,HasActualSelection,SelectedBatch,SweepHorizon);
        PhysicalStep.InvalidateQueries(); // includes completion/max-iteration integration paths
        PhysicalStep.BeginOrdinaryIntegration(SubDelta,HasActualSelection,SelectedBatch,SweepHorizon,TimePassed);
		LastSubDelta = SubDelta;
		const double IntegrateStartSeconds = FPlatformTime::Seconds();
        int32 IntegrationIndex=0;
        for (ISpeedComponent* Comp : SimulationWorld.GetOrderedAdapters())
        {
            PhysicalStep.BeginOrdinaryIntegrationOwner(Comp,IntegrationIndex);
            if (Comp)
            {
                IAMSPEED_ACTOR_SCOPE(SimulationWorld, *Comp, Integrate);
                if(!PhysicalStep.DispatchNativeIntegration(*Comp,SubDelta))Comp->IntegrateKinematics(SubDelta);
            }
            PhysicalStep.EndOrdinaryIntegrationOwner(Comp,IntegrationIndex,Comp!=nullptr);
            ++IntegrationIndex;
        }
		LastStepDiagnostics.IntegrateMilliseconds +=
			(FPlatformTime::Seconds() - IntegrateStartSeconds) * 1000.0;
		TimePassed += SubDelta;
        PhysicalStep.EndOrdinaryIntegration(TimePassed);
#if WITH_DEV_AUTOMATION_TESTS
    Speed::Testing::ObserveCcdPose(this,Frame,Speed::Testing::ECcdPoseAuditStage::AfterIntegrate,TimePassed,SubDelta);
#endif

	};

    int32 Iter = 0;
	while (TimePassed < Dt)
    {
        float Remaining = Dt - TimePassed;
        TUniquePtr<Speed::PreparationPrefix::FEffectScope> RsPrelude;
        if(TerminalRsCapture)RsPrelude.Reset(new Speed::PreparationPrefix::FEffectScope(Speed::PreparationPrefix::EEffect::BeginRsCaptureSweep));
		if (Remaining <= CompletionTolerance&&!PhysicalStep.NativeConsumer)
		{
			IntegrateAll(Remaining);
			break;
		}
		if (Iter >= MaxIter)
		{
			LastStepDiagnostics.bIterationLimitReached = true;
			IntegrateAll(Remaining);
			break;
		}
		++Iter;
		LastStepDiagnostics.IterationCount = Iter;
		// A material stop is a real temporal boundary just like a TOI. Never
		// sweep a trajectory that decelerates past zero and reverses its slip.
        for(const ISpeedComponent* Comp:SimulationWorld.GetOrderedAdapters())
            if(Comp)
            {
                const float before=Remaining;const float actual=Comp->GetMaximumCanonicalSupportInterval(Remaining);
                Remaining=FMath::Min(Remaining,actual);
                PhysicalStep.ObserveTerminalRsSupportInterval(Comp,before,actual);
            }

        // ------------------------------------------------------------
		// 2) Find every independent hit at the global earliest TOI. The
		// iteration budget bounds temporal substeps, not the number of objects
		// that happen to collide at the same instant.
        // ------------------------------------------------------------
        PhysicalStep.BeginSweepIteration(Iter,Remaining,TimePassed);
        if(RsPrelude)RsPrelude->Returned();
        PreparedSweepBundles.Reset();
		constexpr float SimultaneousTOITolerance = 1.0e-9f;
		TArray<SComponentTOI, TInlineAllocator<16>> EarliestHits;
		float EarliestTOI = Remaining;
        PhysicalStep.BeginOrdinaryWorldSelection(ResolvedPairs,Remaining,TimePassed,Dt,SimultaneousTOITolerance);

        const double SweepStartSeconds = FPlatformTime::Seconds();
        for (ISpeedComponent* Comp : SimulationWorld.GetOrderedAdapters())
        {
            if(!Comp)continue;
            TUniquePtr<Speed::PreparationPrefix::FEffectScope> RsOwnerEffect;
            if(TerminalRsCapture)RsOwnerEffect.Reset(new Speed::PreparationPrefix::FEffectScope(
                Speed::PreparationPrefix::EEffect::SweepOwnerForRsCapture,Comp));
            ++LastStepDiagnostics.ComponentSweepCount;
            PhysicalStep.BeginTerminalRsSweepOwner(Comp,LastSubDelta);
            SComponentTOI Ctoi;
            PhysicalStep.BeginOrdinaryOwnerSweep(Comp,LastSubDelta);
            try
            {
                IAMSPEED_ACTOR_SCOPE(SimulationWorld,*Comp,Sweep);
                Ctoi=Comp->SweepTOISubBodies(Remaining,LastSubDelta
#if !UE_BUILD_SHIPPING
                    ,bCaptureSolverEventTrace
#endif
                );
            }
            catch(const Speed::PreparedExecution::FTerminalRsCaptureStop& stop)
            {
                if(!TerminalRsCapture||!RsOwnerEffect)throw;
                const auto captured=PhysicalStep.CloseTerminalRsCapture(stop);
                RsOwnerEffect->InterruptedAtRsCapture(captured);
                if(Speed::OnePausedWork::HasConsumedSphereAdvancePrefix(PhysicalStep.Window->GetPurpose()))
                {
                    Speed::PreparationPrefix::FEffectScope advance(Speed::PreparationPrefix::EEffect::AdvanceRsPrefix);
                    const float consumed=PhysicalStep.BeginTerminalConsumedPrefixAdvance();
                    const double start=FPlatformTime::Seconds();int32 index=0;
                    for(ISpeedComponent* owner:SimulationWorld.GetOrderedAdapters())
                    {
                        Speed::PreparationPrefix::FEffectScope effect(Speed::PreparationPrefix::EEffect::AdvanceRsPrefixOwner,owner);
                        PhysicalStep.BeginTerminalConsumedPrefixAdvanceOwner(owner,index);
                        try {IAMSPEED_ACTOR_SCOPE(SimulationWorld,*owner,Integrate);owner->IntegrateKinematics(consumed);}
                        catch(...){PhysicalStep.RecordTerminalConsumedPrefixAdvanceStates(owner,index,false);throw;}
                        PhysicalStep.EndTerminalConsumedPrefixAdvanceOwner(owner,index);effect.Returned();++index;
                    }
                    LastSubDelta=consumed;TimePassed+=consumed;
                    LastStepDiagnostics.IntegrateMilliseconds+=(FPlatformTime::Seconds()-start)*1000.;
                    PhysicalStep.EndTerminalConsumedPrefixAdvance();advance.Returned();
                    if(Speed::OnePausedWork::HasFirstRsResponsePrefix(PhysicalStep.Window->GetPurpose()))
                    {
                        using Speed::PreparationPrefix::EEffect;
                        Speed::PreparationPrefix::FEffectScope response(EEffect::ResponseRsContact);
                        const auto* raw=PhysicalStep.BeginTerminalFirstRsResponse(LastSubDelta,TimePassed,SimTime);
                        auto* resolver=const_cast<USphereSubBody*>(raw);
                        const EEffect effects[3]={EEffect::SetRsFutureHit,EEffect::AcceptRsHit,EEffect::ResolveRsHit};
                        for(int32 call=0;call<3;++call)
                        {
                            Speed::PreparationPrefix::FEffectScope effect(effects[call],resolver->GetParentComponent());
                            const SHitResult hit=PhysicalStep.BeginTerminalFirstRsResponseCall(resolver,call);
                            try
                            {
                                // Authenticate BEFORE virtual dispatch; each
                                // base entry checks the same active ticket.
                                if(call==0)resolver->SetFutureHit(hit);
                                else if(call==1)resolver->AcceptHit();
                                else resolver->ResolveCurrentHit(LastSubDelta,SimTime);
                            }
                            catch(...){PhysicalStep.RecordTerminalFirstRsResponseStates(resolver,call,false);throw;}
                            PhysicalStep.EndTerminalFirstRsResponseCall(resolver,call);effect.Returned();
                        }
                        PhysicalStep.EndTerminalFirstRsResponse();response.Returned();
                        if(Speed::OnePausedWork::HasFirstRsResponseSyncPrefix(PhysicalStep.Window->GetPurpose()))
                        {
                            Speed::PreparationPrefix::FEffectScope synchronize(EEffect::SyncRsResponseOwner,resolver->GetParentComponent());
                            auto* owner=PhysicalStep.BeginTerminalFirstRsResponseSynchronization();
                            try {owner->UpdateSubBodiesKinematics();}
                            catch(...){PhysicalStep.RecordTerminalFirstRsResponseSynchronizationStates(owner,false);throw;}
                            PhysicalStep.EndTerminalFirstRsResponseSynchronization(owner);synchronize.Returned();
                        }
                    }
                }
                if(PhysicalStep.Window->GetPurpose()==Speed::OnePausedWork::EPurpose::TerminalPostSyncRsCaptureStop)
                {
                    // A distinct native continuation. Every value below comes
                    // from this very Step and its actual first response/sync.
                    float PostRemaining=Dt-TimePassed;
                    {
                        Speed::PreparationPrefix::FEffectScope prelude(Speed::PreparationPrefix::EEffect::BeginPostSyncRsCaptureSweep);
                        PhysicalStep.BeginPostSyncRsPrelude(PostRemaining,TimePassed,LastSubDelta);
                        // Keep the ordinary completion/iteration boundaries;
                        // this capture-only purpose cannot integrate a fallback.
                        if(PostRemaining<=CompletionTolerance||Iter>=MaxIter)
                            throw std::runtime_error("Post-sync Rs second sweep is outside actual completion/iteration limits");
                        ++Iter;LastStepDiagnostics.IterationCount=Iter;
                        for(const ISpeedComponent* owner:SimulationWorld.GetOrderedAdapters())
                        {
                            if(!owner)throw std::runtime_error("Post-sync Rs support owner missing");
                            const float before=PostRemaining;const float actual=owner->GetMaximumCanonicalSupportInterval(PostRemaining);
                            PostRemaining=FMath::Min(PostRemaining,actual);
                            PhysicalStep.ObservePostSyncRsSupport(owner,before,actual);
                        }
                        PhysicalStep.BeginSweepIteration(Iter,PostRemaining,TimePassed);prelude.Returned();
                    }
                    const auto& postOwners=SimulationWorld.GetOrderedAdapters();
                    if(postOwners.Num()!=3)throw std::runtime_error("Post-sync Rs actual owner census differs");
                    for(int32 postIndex=0;postIndex<2;++postIndex)
                    {
                        auto* owner=postOwners[postIndex];
                        Speed::PreparationPrefix::FEffectScope effect(Speed::PreparationPrefix::EEffect::SweepOwnerForPostSyncRsCapture,owner);
                        ++LastStepDiagnostics.ComponentSweepCount;PhysicalStep.BeginPostSyncRsOwner(owner,LastSubDelta);
                        SComponentTOI actual;
                        try
                        {
                            IAMSPEED_ACTOR_SCOPE(SimulationWorld,*owner,Sweep);
                            actual=owner->SweepTOISubBodies(PostRemaining,LastSubDelta
#if !UE_BUILD_SHIPPING
                                ,bCaptureSolverEventTrace
#endif
                            );
                        }
                        catch(const Speed::PreparedExecution::FTerminalRsCaptureStop& postStop)
                        {
                            const auto captured2=PhysicalStep.ClosePostSyncRsCapture(postStop);
                            effect.InterruptedAtPostSyncRsCapture(captured2);
                            return; // No second advance/response, selection or publication.
                        }
                        PhysicalStep.CompletePostSyncRsRunner(owner,actual);effect.Returned();
                    }
                    throw std::runtime_error("Post-sync Rs ball sweep failed to produce its actual private capture stop");
                }
                return; // Terminal only: optional p7 first response, no next sweep/publication/frame increment.
            }
            if(TerminalRsCapture)
            {
                try {PhysicalStep.CompleteTerminalRsRunner(Comp,Ctoi);}
                catch(const std::exception& error)
                {
                    UE_LOG(LogTemp,Error,TEXT("[TerminalRsRunnerValidationRefused] %hs"),error.what());
                    RsOwnerEffect->Returned(false);throw;
                }
                catch(...)
                {
                    UE_LOG(LogTemp,Error,TEXT("[TerminalRsRunnerValidationRefused] Unclassified native validator failure"));
                    RsOwnerEffect->Returned(false);throw;
                }
                RsOwnerEffect->Returned();
            }
            PhysicalStep.EndOrdinaryOwnerSweep(Comp,Ctoi);
            if(Speed::PreparedExecution::Requested())PreparedSweepBundles.Add(Ctoi.PreparedEvidence);
            // TOI sanity
            if (!Ctoi.bHit)
            {
                PhysicalStep.ObserveOrdinaryWorldCandidate(Comp,Ctoi,ResolvedPairs,{},false,0.f,MakeArrayView(EarliestHits),EarliestTOI);
                continue;
            }
#if !UE_BUILD_SHIPPING
			if (bCaptureSolverEventTrace)
			{
				for (int32 CandidateIndex = 0;
					CandidateIndex < Ctoi.DiagnosticCandidates.Num(); ++CandidateIndex)
				{
					const SSubBodyTOIDiagnostic& Candidate = Ctoi.DiagnosticCandidates[CandidateIndex];
					USSubBody* CandidateResolver = Candidate.Resolver.Get();
					uint64 CandidatePairKey = USSubBody::MakePairKey(
						CandidateResolver ? CandidateResolver : Candidate.Sweeper.Get(),
						Candidate.Hit.Component.Get());
					const uint64 CandidateProviderId = Candidate.Hit.CanonicalGroupId != 0
						? Candidate.Hit.CanonicalGroupId : Candidate.Hit.SurfaceId;
					if (CandidateProviderId != 0)
					{
						CandidatePairKey = Speed::Analytic::CombineStableIds(
							CandidatePairKey, CandidateProviderId);
					}
					if (RestingWorld && Candidate.Hit.SurfaceId != 0 &&
						!Candidate.Hit.bSurfaceNormalMayVary &&
						Candidate.Hit.ContactFeatureThis == Speed::EContactFeatureKind::Vertex &&
						Candidate.Hit.ContactFeatureIndexThis >= 0)
					{
						CandidatePairKey = Speed::Analytic::CombineStableIds(CandidatePairKey,
							0x504c414e00000000ull |
							static_cast<uint64>(Candidate.Hit.ContactFeatureIndexThis + 1));
					}
					const int32 CandidateGeneration = ResolvedPairs.GetGeneration(CandidatePairKey);
					TraceEvent(TEXT("SubBodyCandidate"),
						CandidateIndex == Ctoi.DiagnosticBestCandidate ? TEXT("Selected") : TEXT("NotSelected"),
						Iter, Candidate.SweepOrder, CandidatePairKey, Candidate.Hit,
						CandidateResolver, CandidateGeneration, CandidateGeneration);
				}
			}
#endif
			// Once an analytical provider has produced its event impulse during
			// this canonical frame, it is an established unilateral constraint.
			// Rediscovering the same time-zero event would only consume the bounded
			// CCD iteration budget; IntegrateKinematics now owns its continuation.
            Speed::PreparedExecution::FOrdinaryWorldFilterValues WorldFilter;
            WorldFilter.ResolverValid=Ctoi.Resolver.IsValid();
            const bool bEligibleExactReacquisition = WorldFilter.ResolverValid &&
                (Ctoi.Resolver->CanResolveRepeatedContact(Ctoi.Hit)||
                    PhysicalStep.IsPreparedCurvedBoxExactProgress(Ctoi,Ctoi.Resolver.Get(),false));
            WorldFilter.EligibleRepeated=bEligibleExactReacquisition;
            WorldFilter.Applied=Ctoi.Hit.SurfaceId!=0;WorldFilter.Allowed=true;
            if(WorldFilter.Applied)
            {
                WorldFilter.PairWasMember=ResolvedPairs.Contains(Ctoi.PairKey);
                if(WorldFilter.PairWasMember)
                {
                    WorldFilter.ProgressRead=bEligibleExactReacquisition;
                    WorldFilter.Allowed=bEligibleExactReacquisition&&ResolvedPairs.CanSelectExactProgress(
                        Ctoi.PairKey,Ctoi.Hit.TOI,TimePassed,Dt);
                }
                else
                {
                    WorldFilter.OrdinaryRead=true;
                    WorldFilter.Allowed=ResolvedPairs.CanResolve(Ctoi.PairKey,bEligibleExactReacquisition);
                }
            }
            if (!WorldFilter.Allowed)
			{
#if !UE_BUILD_SHIPPING
				const int32 Generation = ResolvedPairs.GetGeneration(Ctoi.PairKey);
				TraceEvent(TEXT("ComponentFilter"), TEXT("Filtered"), Iter, 0,
					Ctoi.PairKey, Ctoi.Hit, Ctoi.Resolver.Get(), Generation, Generation);
#endif
                PhysicalStep.ObserveOrdinaryWorldCandidate(Comp,Ctoi,ResolvedPairs,WorldFilter,false,0.f,MakeArrayView(EarliestHits),EarliestTOI);
				continue;
			}
#if !UE_BUILD_SHIPPING
			{
				const int32 Generation = ResolvedPairs.GetGeneration(Ctoi.PairKey);
				TraceEvent(TEXT("ComponentFilter"), TEXT("Eligible"), Iter, 0,
					Ctoi.PairKey, Ctoi.Hit, Ctoi.Resolver.Get(), Generation, Generation);
			}
#endif

            const float T = FMath::Clamp(Ctoi.TOI, 0.f, Remaining);
            Speed::Collision::AppendEarliestCollision(Ctoi,T,SimultaneousTOITolerance,EarliestHits,EarliestTOI);
            PhysicalStep.ObserveOrdinaryWorldCandidate(Comp,Ctoi,ResolvedPairs,WorldFilter,true,T,MakeArrayView(EarliestHits),EarliestTOI);
        }
		LastStepDiagnostics.SweepMilliseconds +=
			(FPlatformTime::Seconds() - SweepStartSeconds) * 1000.0;
#if !UE_BUILD_SHIPPING
		if (bCaptureSolverEventTrace)
		{
			for (int32 HitIndex = 0; HitIndex < EarliestHits.Num(); ++HitIndex)
			{
				const SComponentTOI& Hit = EarliestHits[HitIndex];
				const int32 Generation = ResolvedPairs.GetGeneration(Hit.PairKey);
				TraceEvent(TEXT("BeforeBatchSort"), TEXT("Queued"), Iter, HitIndex,
					Hit.PairKey, Hit.Hit, Hit.Resolver.Get(), Generation, Generation);
			}
		}
#endif
		EarliestHits.Sort([&PhysicalStep](const SComponentTOI& A, const SComponentTOI& B)
		{
			if (A.PairKey != B.PairKey)
			{
				return A.PairKey < B.PairKey;
			}
			const USSubBody* ResolverA = A.Resolver.Get();
			const USSubBody* ResolverB = B.Resolver.Get();
			const uint32 ResolverIdA = ResolverA ? ResolverA->GetUniqueID() : 0u;
			const uint32 ResolverIdB = ResolverB ? ResolverB->GetUniqueID() : 0u;
			PhysicalStep.ObserveOrdinaryWorldSortKey(A,ResolverA,ResolverIdA);
            PhysicalStep.ObserveOrdinaryWorldSortKey(B,ResolverB,ResolverIdB);
            return ResolverIdA < ResolverIdB;
		});
#if !UE_BUILD_SHIPPING
		if (bCaptureSolverEventTrace)
		{
			for (int32 HitIndex = 0; HitIndex < EarliestHits.Num(); ++HitIndex)
			{
				const SComponentTOI& Hit = EarliestHits[HitIndex];
				const int32 Generation = ResolvedPairs.GetGeneration(Hit.PairKey);
				TraceEvent(TEXT("AfterBatchSort"), TEXT("Queued"), Iter, HitIndex,
					Hit.PairKey, Hit.Hit, Hit.Resolver.Get(), Generation, Generation);
			}
		}
#endif

        // ------------------------------------------------------------
		// 3) Integrate everyone once to the shared TOI, or through the full
		// remaining interval when there is no hit.
        // ------------------------------------------------------------
		const bool bWillResolve = !EarliestHits.IsEmpty();
		const float SubDelta = FMath::Clamp(
			bWillResolve ? EarliestTOI : Remaining, 0.0f, Remaining);
        PhysicalStep.FinishOrdinaryWorldSelection(MakeArrayView(EarliestHits),EarliestTOI,SubDelta,Remaining,TimePassed,Dt);
        const float TimeBeforeIntegration = TimePassed;
		IntegrateAll(SubDelta,true,MakeArrayView(EarliestHits),Remaining);
#if !UE_BUILD_SHIPPING
		if (bCaptureSolverEventTrace)
		{
			UE_LOG(LogTemp, Display,
				TEXT("[SpeedSolverIterationV2] Frame=%u Iter=%d SweepHorizon=%.9g EarliestTOI=%.9g SubDelta=%.9g TimePassed=%.9g Batch=%d"),
				Frame, Iter, Remaining, EarliestTOI, SubDelta, TimePassed, EarliestHits.Num());
		}
#endif

        // No hit => End
        if (!bWillResolve)
        {
            if (TimePassed >= Dt) break;
            continue;
        }

        // ------------------------------------------------------------
		// 4) Resolve the simultaneous batch once per pair. All candidates were
		// measured from the same pre-resolution state, so independent objects do
		// not consume additional temporal iterations.
        // ------------------------------------------------------------
        PhysicalStep.BeginOrdinaryResponseBatch(MakeArrayView(EarliestHits),ResolvedPairs,SubDelta,TimeBeforeIntegration,TimePassed,SimTime);
        for (int32 HitIndex = 0; HitIndex < EarliestHits.Num(); ++HitIndex)
        {
            const SComponentTOI& Hit = EarliestHits[HitIndex];
            USSubBody* Resolver = Hit.Resolver.Get();
			const bool bEligibleExactReacquisition = Resolver &&
				(Resolver->CanResolveRepeatedContact(Hit.Hit)||
					PhysicalStep.IsPreparedCurvedBoxExactProgress(Hit,Resolver,true));
            const bool bResponseAllowed = Resolver && (ResolvedPairs.Contains(Hit.PairKey)
                ? bEligibleExactReacquisition && ResolvedPairs.CanResolveExactProgress(
                    Hit.PairKey, Hit.Hit.TOI, TimeBeforeIntegration, TimePassed, Dt)
                : ResolvedPairs.CanResolve(Hit.PairKey, bEligibleExactReacquisition));
            PhysicalStep.ObserveOrdinaryResponseDecision(HitIndex,Hit,Resolver,bEligibleExactReacquisition,bResponseAllowed,ResolvedPairs);
            if (!bResponseAllowed)
			{
#if !UE_BUILD_SHIPPING
				const int32 Generation = ResolvedPairs.GetGeneration(Hit.PairKey);
				TraceEvent(TEXT("ResolveDecision"), Resolver ? TEXT("Filtered") : TEXT("Ineligible"),
					Iter, HitIndex, Hit.PairKey, Hit.Hit, Resolver, Generation, Generation);
#endif
				continue;
			}

#if !UE_BUILD_SHIPPING
            const int32 GenerationBefore = ResolvedPairs.GetGeneration(Hit.PairKey);
#endif
            PhysicalStep.BeginOrdinaryResponseCall(0);
            if(!PhysicalStep.DispatchNativeResponse(0,*Resolver,Hit.Hit,SubDelta,SimTime))Resolver->SetFutureHit(Hit.Hit);
            PhysicalStep.EndOrdinaryResponseCall(0);
            PhysicalStep.BeginOrdinaryResponseCall(1);
            if(!PhysicalStep.DispatchNativeResponse(1,*Resolver,Hit.Hit,SubDelta,SimTime))Resolver->AcceptHit();
            PhysicalStep.EndOrdinaryResponseCall(1);
			bool bIgnored = false;
			if (UPrimitiveComponent* HitComponent =
				Resolver->GetHit().Component.Get())
			{
				if (Resolver->ComponentHasBeenIgnored(*HitComponent))
				{
					bIgnored = true;
					ResolvedPairs.Add(Hit.PairKey);
				}
			}
			USolidSubBody* RollingBodyA = Cast<USolidSubBody>(Resolver);
			USolidSubBody* RollingBodyB =
				Cast<USolidSubBody>(Resolver->GetHit().SubBody.Get());

			const double ResolveStartSeconds = FPlatformTime::Seconds();
#if WITH_DEV_AUTOMATION_TESTS
    Speed::Testing::ObserveCcdPose(this,Frame,Speed::Testing::ECcdPoseAuditStage::BeforeResolve,TimePassed,SubDelta);
#endif

#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
            auto MatrixResolution042 = Speed::CollisionMatrix042::FObserver::Before(
                Frame, TimePassed, *Resolver, Hit.Hit);
#endif
            PhysicalStep.BeginOrdinaryResponseCall(2);
            if(!PhysicalStep.DispatchNativeResponse(2,*Resolver,Hit.Hit,SubDelta,SimTime))Resolver->ResolveCurrentHit(SubDelta, SimTime);
            PhysicalStep.EndOrdinaryResponseCall(2);
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
            Speed::CollisionMatrix042::FObserver::After(MatrixResolution042);
#endif
#if WITH_DEV_AUTOMATION_TESTS
    Speed::Testing::ObserveCcdPose(this,Frame,Speed::Testing::ECcdPoseAuditStage::AfterResolve,TimePassed,SubDelta);
#endif

			LastStepDiagnostics.ResolveMilliseconds +=
				(FPlatformTime::Seconds() - ResolveStartSeconds) * 1000.0;
			++LastStepDiagnostics.ResolvedEventCount;
            // A resolved exact time-zero vertex is an initial event, not a
            // repeat. Seed its identity only now, so a later positive return
            // can use an actual-time progress certificate; the legacy generation
            // remains saturated at two for diagnostics and legacy callers.
            const bool bRecordExactGeneration = !bIgnored &&
                (bEligibleExactReacquisition || Resolver->CanInitializeRepeatedContact(Hit.Hit));
			if (Hit.Hit.SurfaceId != 0)
			{
				if (bRecordExactGeneration)
				{
					ResolvedPairs.RecordExactProgressResolution(Hit.PairKey, TimePassed, Dt);
				}
				else
				{
					ResolvedPairs.Add(Hit.PairKey);
				}
			}
#if !UE_BUILD_SHIPPING
			const int32 GenerationAfter = ResolvedPairs.GetGeneration(Hit.PairKey);
			TraceEvent(TEXT("ResolveResult"), bIgnored ? TEXT("Ignored") : TEXT("Resolved"),
				Iter, HitIndex, Hit.PairKey, Hit.Hit, Resolver,
				GenerationBefore, GenerationAfter);
            if (Speed::FFrameSolveWitnessScope::IsActive())
            {
                ISpeedComponent* WitnessParent=Resolver?Resolver->GetParentComponent():nullptr;
                Speed::FFrameSolveWitnessScope::Record(Frame,Iter,HitIndex,
                    WitnessParent?SimulationWorld.FindStableId(*WitnessParent):0,
                    WitnessParent?WitnessParent->GetSubBodies().IndexOfByKey(Resolver):INDEX_NONE,
                    Hit.PairKey,Hit.Hit,bIgnored,GenerationBefore,GenerationAfter,Resolver,RestingWorld!=nullptr);
            }
#endif
            if (RollingBodyA && RollingBodyB)
            {
                ActivatePendingRollingContactPairAtTOI(
                    *RollingBodyA, *RollingBodyB, Dt - TimePassed);
            }
            PhysicalStep.EndOrdinaryResponseEntry(HitIndex,bIgnored,bRecordExactGeneration,ResolvedPairs);
            // Resolve may project a COM or change velocity/acceleration.
            // Refresh the actual child caches before another response or
            // sweep reads them. This advances no time and changes no hit/TOI.
            TSet<ISpeedComponent*> ResponseOwners;
            if(ISpeedComponent* Owner=Resolver->GetParentComponent())ResponseOwners.Add(Owner);
            if(USSubBody* Other=Resolver->GetHit().SubBody.Get())
                if(ISpeedComponent* Owner=Other->GetParentComponent())ResponseOwners.Add(Owner);
            for(ISpeedComponent* Owner:SimulationWorld.GetOrderedAdapters())
            {
                if(!Owner||!ResponseOwners.Contains(Owner))continue;
                if(PhysicalStep.NativeConsumer)
                {
                    PhysicalStep.RequireNativeFrameContext();
                    if(!PhysicalStep.NativeConsumer->ContainsOwner(Owner))
                        throw std::runtime_error("Post-response sync owner is outside the retained native census");
                }
                // Same non-virtual operation as the captured native Sync
                // callback; no private dispatch access or new capability.
                Owner->UpdateSubBodiesKinematics();
                if(PhysicalStep.NativeConsumer)PhysicalStep.RequireNativeFrameContext();
            }

        }
        PhysicalStep.EndOrdinaryResponseBatch();

        // Optional: post update at each substep (useful if certain gameplay sensors need to react "immediately")
        /*
        for (ISpeedComponent* Comp : SimulationWorld.GetOrderedAdapters())
        {
            if (!Comp) continue;
            Comp->PostPhysicsUpdate();
        }
        */
    }

	// A contact acquired at the last TOI has no remaining integration substep.
	// Project once at the frame boundary so PostPhysics observers never receive
	// an infeasible analytical pose. Moving the body here also avoids changing
	// the within-frame CCD trajectory from inside an impact callback.
	const double ProjectionStartSeconds = FPlatformTime::Seconds();
    int32 FinalProjectionIndex=0;
	for (ISpeedComponent* Comp : SimulationWorld.GetOrderedAdapters())
	{
		if (Comp)
		{
			IAMSPEED_ACTOR_SCOPE(SimulationWorld, *Comp, Projection);
            PhysicalStep.BeginNativeFinalOperation(Speed::PreparedExecution::ENativeFrameFinalOperation::StaticProjectionOwner,Comp,FinalProjectionIndex);
            {
                Speed::PreparedExecution::FOrdinaryOwnerProbePhaseScope probes(
                    Speed::PreparedExecution::ENativeOwnerProbePhase::FinalProjection,*Comp,Dt);
                if(!PhysicalStep.DispatchNativeFinalOwner(*Comp,Dt,false))Comp->ProjectEstablishedStaticContacts(Dt);
                probes.Returned();
            }
            PhysicalStep.EndNativeFinalOperation();
        }
        ++FinalProjectionIndex;
    }

    // Make persistent finite-mass contacts geometrically feasible before any
    // PostPhysics observer samples them.
    PhysicalStep.BeginNativeFinalOperation(Speed::PreparedExecution::ENativeFrameFinalOperation::DynamicProjectionBeforePost);
	ProjectDynamicContactPairs();
    PhysicalStep.EndNativeFinalOperation();
#if WITH_DEV_AUTOMATION_TESTS
    Speed::Testing::ObserveCcdPose(this,Frame,Speed::Testing::ECcdPoseAuditStage::AfterProjection,TimePassed,0.f);
#endif
	LastStepDiagnostics.ProjectionMilliseconds =
		(FPlatformTime::Seconds() - ProjectionStartSeconds) * 1000.0;

    // ------------------------------------------------------------
    // 5) Final post update
    // ------------------------------------------------------------
	const double PostStartSeconds = FPlatformTime::Seconds();
    int32 FinalPostIndex=0;
    for (ISpeedComponent* Comp : SimulationWorld.GetOrderedAdapters())
    {
        if (!Comp) continue;
		IAMSPEED_ACTOR_SCOPE(SimulationWorld, *Comp, Post);
#if !UE_BUILD_SHIPPING
        RoofPostProfile::FSample roofPost(*Comp);
#endif
        PhysicalStep.BeginNativeFinalOperation(Speed::PreparedExecution::ENativeFrameFinalOperation::PostOwner,Comp,FinalPostIndex);
#if !UE_BUILD_SHIPPING
        roofPost.BeginReturned();
#endif
        {
            Speed::PreparedExecution::FOrdinaryOwnerProbePhaseScope probes(
                Speed::PreparedExecution::ENativeOwnerProbePhase::FinalPost,*Comp,Dt);
            if(!PhysicalStep.DispatchNativeFinalOwner(*Comp,Dt,true))Comp->PostPhysicsUpdate(Dt);
            probes.Returned();
        }
#if !UE_BUILD_SHIPPING
        roofPost.DispatchReturned();
#endif
        PhysicalStep.EndNativeFinalOperation();++FinalPostIndex;
#if !UE_BUILD_SHIPPING
        roofPost.EndReturned();
#endif
	}

    const bool MergeFinalPairs=!SimulationWorld.GetPendingRollingContactPairs().IsEmpty();
    PhysicalStep.BeginNativeFinalOperation(Speed::PreparedExecution::ENativeFrameFinalOperation::MergeRollingPairs,nullptr,INDEX_NONE,MergeFinalPairs);
    if(MergeFinalPairs)SimulationWorld.MergePendingRollingContactPairs();
    PhysicalStep.EndNativeFinalOperation();
    PhysicalStep.BeginNativeFinalOperation(Speed::PreparedExecution::ENativeFrameFinalOperation::SolvePairs);
    SolveDynamicContactPairs(Dt);PhysicalStep.EndNativeFinalOperation();
    PhysicalStep.BeginNativeFinalOperation(Speed::PreparedExecution::ENativeFrameFinalOperation::DynamicProjectionAfterPost);
    ProjectDynamicContactPairs();PhysicalStep.EndNativeFinalOperation();
    PhysicalStep.CloseNativeFrameStep(TimePassed);
#if WITH_DEV_AUTOMATION_TESTS
    Speed::Testing::ObserveCcdPose(this,Frame,Speed::Testing::ECcdPoseAuditStage::AfterPost,TimePassed,0.f);
#endif
	LastStepDiagnostics.PostMilliseconds =
		(FPlatformTime::Seconds() - PostStartSeconds) * 1000.0;
	LastStepDiagnostics.TotalMilliseconds =
		(FPlatformTime::Seconds() - StepStartSeconds) * 1000.0;
	const Speed::Analytic::FStaticWorldQueryCounters QueryCounters =
		Speed::Analytic::FStaticWorldQueryAudit::GetCurrentFrameCounters();
	LastStepDiagnostics.StaticQueryCount = QueryCounters.QueryCount;
	LastStepDiagnostics.LegacySweepCount = QueryCounters.LegacySweepCount;
	LastStepDiagnostics.AuthorityAttemptCount = QueryCounters.AuthorityAttemptCount;
	LastStepDiagnostics.StaticAuthorityMilliseconds =
		QueryCounters.AuthorityMilliseconds;
	LastStepDiagnostics.MaximumStaticAuthorityMilliseconds =
		QueryCounters.MaximumAuthorityMilliseconds;
    ::StadiumClosedStepProfile70::Observe(LastStepDiagnostics);

	if (LastStepDiagnostics.bIterationLimitReached)
	{
		UE_LOG(LogTemp, Error,
			TEXT("[SpeedSolverIterationLimit] Frame=%u Iterations=%d MaxIterations=%d ComponentSweeps=%d StaticQueries=%u AuthorityAttempts=%u ResolvedEvents=%d TimePassed=%.9g Dt=%.9g StepMs=%.6f"),
			Frame, LastStepDiagnostics.IterationCount,
			LastStepDiagnostics.MaximumIterationCount,
			LastStepDiagnostics.ComponentSweepCount,
			LastStepDiagnostics.StaticQueryCount,
			LastStepDiagnostics.AuthorityAttemptCount,
			LastStepDiagnostics.ResolvedEventCount,
			TimePassed, Dt, LastStepDiagnostics.TotalMilliseconds);
	}
}

#if !UE_BUILD_SHIPPING
void USpeedWorldSubsystem::DeclareExpectedFrameAnalogWitnessForTesting() const
{
 // The canonical immutable adapter list declares coverage independently of
 // UpdateInputs. Missing callbacks cannot erase their own expectation.
 for(const ISpeedComponent* Adapter:SimulationWorld.GetOrderedAdapters())
 {
  if(!Adapter){Speed::FFrameAppliedCommandWitnessScope::MarkCoverageUnknown();continue;}
  switch(Adapter->DeclareFrameAnalogWitnessForTesting())
  {
  case EFrameAnalogWitnessDeclaration::RequiredAnalogConsumer:
   Speed::FFrameAppliedCommandWitnessScope::ExpectAnalogActor(SimulationWorld.FindStableId(*Adapter));break;
  case EFrameAnalogWitnessDeclaration::NoAnalogConsumer:break;
  default:Speed::FFrameAppliedCommandWitnessScope::MarkCoverageUnknown();break;
  }
 }
 if(!Speed::FFrameAppliedCommandWitnessScope::SealExpectedAnalogActors())
  Speed::FFrameAppliedCommandWitnessScope::MarkCoverageUnknown();
}
void USpeedWorldSubsystem::FrameAdapterCensusForTesting(TArray<FString>& OutRows) const
{
 OutRows.Reset();
 static constexpr TCHAR Hex[]=TEXT("0123456789abcdef");
 for(const ISpeedComponent* Adapter:SimulationWorld.GetOrderedAdapters())
 {
  if(!Adapter)continue;
  const uint64 Id=SimulationWorld.FindStableId(*Adapter);
  const FString ClassPath=Adapter->FrameWitnessAdapterClassForTesting();
  const FTCHARToUTF8 Utf8(*ClassPath);FString Encoded;
  for(int32 Index=0;Index<Utf8.Length();++Index)
  {
   const uint8 Byte=uint8(Utf8.Get()[Index]);
   Encoded.AppendChar(Hex[Byte>>4]);Encoded.AppendChar(Hex[Byte&15]);
  }
  OutRows.Add(FString::Printf(TEXT("{\"actor\":%llu,\"class_path_utf8_hex\":\"%s\",\"sub_bodies\":%d,\"analog_declaration\":%u}"),
   Id,*Encoded,Adapter->GetSubBodies().Num(),uint32(Adapter->DeclareFrameAnalogWitnessForTesting())));
 }
}
void USpeedWorldSubsystem::RegisterFramePointerWitnessForTesting(Speed::FFramePointerRegistry& Registry,int32& ExpectedActors) const
{
 ExpectedActors=SimulationWorld.GetOrderedAdapters().Num();
 TMap<UPTRINT,TArray<uint64>> ActorOwners;
 // Identities are run-stable world ids and canonical per-owner sub-body order.
 // Shared/unknown static aliases are deliberately not guessed from addresses.
 for(const ISpeedComponent* Adapter:SimulationWorld.GetOrderedAdapters())
 {
  if(!Adapter)continue;const uint64 Id=SimulationWorld.FindStableId(*Adapter);
  Registry.Register(Adapter,Speed::FFramePointerKey{1,Id,0,1,0});
  const auto& Bodies=Adapter->GetSubBodies();
  for(int32 Index=0;Index<Bodies.Num();++Index)if(Bodies[Index])
  {
   Registry.Register(Bodies[Index],Speed::FFramePointerKey{2,Id,0,1,uint64(Index+1)});
   // The typed owning actor relationship supplies semantic adapter membership.
   if(const auto* Owner=Bodies[Index]->GetOwner())
    ActorOwners.FindOrAdd(reinterpret_cast<UPTRINT>(Owner)).AddUnique(Id);
  }
 }
 for(auto& Entry:ActorOwners)
 {
  Entry.Value.Sort();Speed::FFramePointerKey Key{5,Entry.Value[0],0,1,0};Key.Membership=Entry.Value;
  Registry.Register(Entry.Key,Key);
 }
 if(const auto* Authority=SimulationWorld.GetStaticCollisionWorld())
  Registry.Register(Authority,Speed::FFramePointerKey{3,0,0,1,0});
 TMap<UPTRINT,TArray<uint64>> Members;
 for(const auto& Entry:AnalyticSourceComponents)if(const auto* Source=Entry.Value.Get())
  Members.FindOrAdd(reinterpret_cast<UPTRINT>(Source)).Add(Entry.Key);
 for(auto& Entry:Members)
 {
  Entry.Value.Sort();Speed::FFramePointerKey Key{4,Entry.Value[0],0,1,0};Key.Membership=Entry.Value;
  Registry.Register(Entry.Key,Key);
 }
}
#endif
bool USpeedWorldSubsystem::CaptureReplayStateFields(
    TArray<FSimulationReplayStateRecord>& Out, uint64& UnsupportedStableId) const
{
    return SimulationWorld.CaptureReplayStateFields(Out, UnsupportedStableId);
}

#include "PreparedIntervalWorldPublication.inl"
