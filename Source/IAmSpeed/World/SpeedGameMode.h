// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameMode.h"
#include "SpeedGameMode.generated.h"


class ASpeedSimActor;

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

protected:
	UPROPERTY(EditDefaultsOnly)
	TSubclassOf<class ASpeedSimActor> SpeedSimActorClass;

private:
	UPROPERTY()
	TObjectPtr<ASpeedSimActor> SpeedSimActor;
};
