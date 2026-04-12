// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "SpeedSimActor.generated.h"

class USpeedWorldSubsystem;

/*
* ASpeedSimActor : Actor responsible for ticking the IAmSpeed Engine simulation.
* It is the one that calls Step() on the SpeedWorldSubsystem every physics frame.
*/
UCLASS()
class IAMSPEED_API ASpeedSimActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ASpeedSimActor();

	void AsyncPhysicsTickActor(float DeltaTime, float SimTime) override;
	void Simulate(const float& DeltaTime, const float& SimTime);
	bool ShouldTickIfViewportsOnly() const override { return true; }
	static unsigned int GetEngineFPS() { return EngineFPS; }
protected:
	void UpdateNumFrame(const float& SimTime);
	unsigned int NumFrame() const;
	static USpeedWorldSubsystem* GetSpeedWorldSubsystem(UWorld* World);
private:
	USpeedWorldSubsystem* SpeedWorldSubsystem = nullptr;

	unsigned int _NumFrame = 0;
	static unsigned int EngineFPS; // The FPS at which the IAmSpeed Engine is running
};
