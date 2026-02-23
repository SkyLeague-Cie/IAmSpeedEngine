// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "IAmSpeed/SubBodies/SSubBody.h"
#include "SpeedWorldSubsystem.generated.h"

class ISpeedComponent;

struct FPendingOp
{
    bool bAdd = false;
    ISpeedComponent* Comp = nullptr;
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

    void Step(const float& Dt, const float& SimTime, const unsigned int& Frame);
private:
    TArray<ISpeedComponent*> Components;

	// Sorted array of components based on their UObject ID
    TArray<ISpeedComponent*> ComponentsSorted;
    bool bDirtyOrder = true;

    void RebuildSortedIfNeeded();
	void AddComponent(ISpeedComponent& Comp);
	void RemoveComponent(ISpeedComponent& Comp);

    FCriticalSection PendingCS;
    TArray<FPendingOp> PendingOps;
};
