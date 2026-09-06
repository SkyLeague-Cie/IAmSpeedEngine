// Fill out your copyright notice in the Description page of Project Settings.


#include "SpeedGameMode.h"
#include "CanonicalFrameDriver.h"
#include "FastSimulation.h"
#include "RealTimeSimulation.h"

ASpeedGameMode::ASpeedGameMode()
{
    SpeedSimActorClass = ARealTimeSimulation::StaticClass();
}

void ASpeedGameMode::BeginPlay()
{
    Super::BeginPlay();

    if (!HasAuthority()) return;
    if (SpeedSimActor) return;

    UWorld* World = GetWorld();
    if (!World || !SpeedSimActorClass) return;

	TSubclassOf<ASpeedSimulation> SimulationClass = SpeedSimActorClass;
	if (Speed::CanonicalFrameDriver::IsFastSimulationEnabled())
	{
		SimulationClass = AFastSimulation::StaticClass();
	}

    FActorSpawnParameters Params;
    Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    Params.bNoFail = true;

    SpeedSimActor = World->SpawnActor<ASpeedSimulation>(SimulationClass, FTransform::Identity, Params);
}

void ASpeedGameMode::SetSimulationPaused(const bool bPaused)
{
	if (!SpeedSimActor)
	{
		return;
	}

	if (bPaused)
	{
		SpeedSimActor->PauseOwnedSimulation();
	}
	else
	{
		SpeedSimActor->ResumeOwnedSimulation();
	}
}
