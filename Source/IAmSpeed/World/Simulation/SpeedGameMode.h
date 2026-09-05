// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameMode.h"
#include "SpeedGameMode.generated.h"


class ASpeedSimulation;

/**
 *
 */
UCLASS()
class IAMSPEED_API ASpeedGameMode : public AGameMode
{
	GENERATED_BODY()

public:
	ASpeedGameMode();
	virtual void BeginPlay() override;
	/** Pauses or resumes the separately-owned simulation lane when it exists. */
	void SetSimulationPaused(bool bPaused);
	/** Returns the single authoritative simulation actor owned by this game mode. */
	ASpeedSimulation* GetSpeedSimulation() const { return SpeedSimActor; }

protected:
	UPROPERTY(EditDefaultsOnly)
	TSubclassOf<class ASpeedSimulation> SpeedSimActorClass;

private:
	UPROPERTY()
	TObjectPtr<ASpeedSimulation> SpeedSimActor;
};
