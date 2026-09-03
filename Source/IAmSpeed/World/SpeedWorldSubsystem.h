// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "IAmSpeed/SubBodies/SSubBody.h"
#include "IAmSpeed/World/Analytic/AnalyticWorldData.h"
#include "IAmSpeed/World/StaticCollisionWorld.h"
#include "SpeedWorldSubsystem.generated.h"

class ISpeedComponent;
class USolidSubBody;
struct FCanonicalFrameContext;

enum class ERollingManifoldContactState : uint8
{
	Absent,
	IdentityOnly,
	ActiveContact,
};

enum class ECanonicalRunControlState : uint8
{
	Uncontrolled,
	WaitingForScenario,
	Ready,
	Complete,
};

struct FSpeedStepDiagnostics
{
	uint64 Serial = 0;
	unsigned int Frame = 0;
	int32 IterationCount = 0;
	int32 ComponentSweepCount = 0;
	int32 ResolvedEventCount = 0;
	uint32 StaticQueryCount = 0;
	uint32 LegacySweepCount = 0;
	uint32 AuthorityAttemptCount = 0;
	double StaticAuthorityMilliseconds = 0.0;
	double MaximumStaticAuthorityMilliseconds = 0.0;
	int32 MaximumIterationCount = 0;
	double PhysicalDeltaTimeMilliseconds = 0.0;
	double TotalMilliseconds = 0.0;
	double ResetMilliseconds = 0.0;
	double SweepMilliseconds = 0.0;
	double IntegrateMilliseconds = 0.0;
	double ResolveMilliseconds = 0.0;
	double ProjectionMilliseconds = 0.0;
	double PostMilliseconds = 0.0;
	bool bIterationLimitReached = false;
};

struct FPendingOp
{
    bool bAdd = false;
    ISpeedComponent* Comp = nullptr;
};

struct FDynamicContactPair
{
	TWeakObjectPtr<USolidSubBody> BodyA;
	TWeakObjectPtr<USolidSubBody> BodyB;
	FVector LocalAnchorA = FVector::ZeroVector;
	FVector LocalAnchorB = FVector::ZeroVector;
	FVector LocalNormalB = FVector::UpVector;
	FVector LastCOMA = FVector::ZeroVector;
	FVector LastCOMB = FVector::ZeroVector;
	uint64 PairKey = 0;
	unsigned int FirstSeenFrame = 0;
	unsigned int LastSeenFrame = 0;
	unsigned int LastSolvedFrame = TNumericLimits<unsigned int>::Max();
	unsigned int SupportedFrameCount = 0;
	unsigned int ActiveContactFrameCount = 0;
	unsigned int ReactionFrameCount = 0;
	unsigned int FeatureTransitionCount = 0;
	float AcquisitionNormalSpeed = 0.0f;
	float AccumulatedNormalImpulse = 0.0f;
	float AccumulatedNormalActivity = 0.0f;
	float AccumulatedNormalSupportActivity = 0.0f;
	float PeakNormalActivity = 0.0f;
	float PeakNormalSupportActivity = 0.0f;
	float MinimumSeparation = TNumericLimits<float>::Max();
	float MaximumSeparation = TNumericLimits<float>::Lowest();
	bool bRollingManifoldReady = false;
	bool bActiveContactLastSolve = false;
};

/**
 * USpeedWorldSubsystem : Subsystem responsible for managing and updating all speed components in the world.
 */
UCLASS()
class IAMSPEED_API USpeedWorldSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()
public:
	virtual void OnWorldBeginPlay(UWorld& InWorld) override;
	static bool AreUnilateralRollingPairsEnabled();
	static bool ShouldAcquireUnilateralRollingPair(float RelativeNormalSpeed);
    void RegisterSpeedComponent(ISpeedComponent* Comp);
    void UnregisterSpeedComponent(ISpeedComponent* Comp);
    void ApplyPendingOps();
	void RegisterDynamicContactPair(
		USolidSubBody& BodyA,
		USolidSubBody& BodyB,
		const FVector& ContactPoint,
		const FVector& NormalBToA,
		float ImpactRelativeNormalSpeed);
	void ActivatePendingRollingContactPairAtTOI(
		const USolidSubBody& BodyA,
		const USolidSubBody& BodyB,
		float RemainingDt);
	bool IsDynamicContactPairOwnedByRollingManifold(
		const USolidSubBody& BodyA,
		const USolidSubBody& BodyB) const;
	ERollingManifoldContactState GetRollingManifoldContactState(
		const USolidSubBody& Body) const;

	void PrepareCanonicalFrame(const FCanonicalFrameContext& Context);
	ECanonicalRunControlState GetCanonicalRunControlState();
	const Speed::Analytic::FAnalyticWorldData* GetAnalyticWorldData() const
	{
		return AnalyticWorldData.Get();
	}
	const Speed::IStaticCollisionWorld* GetStaticCollisionWorld() const
	{
		return StaticCollisionWorld.Get();
	}
	const FSpeedStepDiagnostics& GetLastStepDiagnostics() const
	{
		return LastStepDiagnostics;
	}
	TWeakObjectPtr<UPrimitiveComponent> FindAnalyticSourceComponent(
		uint64 SourceId) const;
    void Step(const float& Dt, const float& SimTime, const unsigned int& Frame);
private:
    TArray<ISpeedComponent*> Components;
	TArray<FDynamicContactPair> DynamicContactPairs;
	TArray<FDynamicContactPair> PendingRollingContactPairs;
	unsigned int CurrentStepFrame = 0;
	uint64 StepSerial = 0;
	FSpeedStepDiagnostics LastStepDiagnostics;
	mutable bool bLoggedRollingOwnershipState = false;
	bool bLoggedRollingSolveState = false;
	// Runtime data is immutable after validation. Baked assets already return a
	// finalized shared payload, so retaining it avoids a deep copy and a second
	// rebuild of every compact-provider acceleration structure at world start.
	TSharedPtr<const Speed::Analytic::FAnalyticWorldData> AnalyticWorldData;
	// Authoritative immutable collision service. Its lifetime is owned by the
	// physical world, independently from per-frame audit/shadow instrumentation.
	TUniquePtr<Speed::IStaticCollisionWorld> StaticCollisionWorld;
	// Runtime-only Unreal bridge. The analytical payload retains stable ids;
	// legacy consumers receive their source component without coupling the
	// generic query backend to UObjects.
	TMap<uint64, TWeakObjectPtr<UPrimitiveComponent>> AnalyticSourceComponents;
	uint8 AnalyticWorldBuildAttempt = 0;
	void BuildAnalyticWorldFromLoadedSources();

	// Sorted array of components based on their UObject ID
    TArray<ISpeedComponent*> ComponentsSorted;
    bool bDirtyOrder = true;

    void RebuildSortedIfNeeded();
	void AddComponent(ISpeedComponent& Comp);
	void RemoveComponent(ISpeedComponent& Comp);
	void SolveDynamicContactPairs(
		float Dt,
		uint64 PairKeyFilter = 0,
		bool bFilterByPairKey = false,
		bool bSolveFirstSeenFrame = false);
	void ProjectDynamicContactPairs();

    FCriticalSection PendingCS;
    TArray<FPendingOp> PendingOps;
};
