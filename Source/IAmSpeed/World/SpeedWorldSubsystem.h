// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "IAmSpeed/SubBodies/SSubBody.h"
#include "SpeedWorldSubsystem.generated.h"

class ISpeedComponent;
class USolidSubBody;

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
};

/**
 * USpeedWorldSubsystem : Subsystem responsible for managing and updating all speed components in the world.
 */
UCLASS()
class IAMSPEED_API USpeedWorldSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()
public:
    void RegisterSpeedComponent(ISpeedComponent* Comp);
    void UnregisterSpeedComponent(ISpeedComponent* Comp);
    void ApplyPendingOps();
	void RegisterDynamicContactPair(
		USolidSubBody& BodyA,
		USolidSubBody& BodyB,
		const FVector& ContactPoint,
		const FVector& NormalBToA);

    void Step(const float& Dt, const float& SimTime, const unsigned int& Frame);
private:
    TArray<ISpeedComponent*> Components;
	TArray<FDynamicContactPair> DynamicContactPairs;
	unsigned int CurrentStepFrame = 0;

	// Sorted array of components based on their UObject ID
    TArray<ISpeedComponent*> ComponentsSorted;
    bool bDirtyOrder = true;

    void RebuildSortedIfNeeded();
	void AddComponent(ISpeedComponent& Comp);
	void RemoveComponent(ISpeedComponent& Comp);
	void SolveDynamicContactPairs(float Dt);

    FCriticalSection PendingCS;
    TArray<FPendingOp> PendingOps;
};
