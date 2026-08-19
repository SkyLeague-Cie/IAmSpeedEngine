// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "IAmSpeed/SubBodies/SSubBody.h"
#include "IAmSpeed/World/Analytic/AnalyticWorldData.h"
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
    void Step(const float& Dt, const float& SimTime, const unsigned int& Frame);
private:
    TArray<ISpeedComponent*> Components;
	TArray<FDynamicContactPair> DynamicContactPairs;
	TArray<FDynamicContactPair> PendingRollingContactPairs;
	unsigned int CurrentStepFrame = 0;
	mutable bool bLoggedRollingOwnershipState = false;
	bool bLoggedRollingSolveState = false;
	TUniquePtr<Speed::Analytic::FAnalyticWorldData> AnalyticWorldData;
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
