// Fill out your copyright notice in the Description page of Project Settings.


#include "SpeedGameMode.h"
#include "SpeedSimActor.h"

ASpeedGameMode::ASpeedGameMode()
{
    SpeedSimActorClass = ASpeedSimActor::StaticClass();
}

void ASpeedGameMode::BeginPlay()
{
    Super::BeginPlay();

    if (!HasAuthority()) return;
    if (SpeedSimActor) return;

    UWorld* World = GetWorld();
    if (!World || !SpeedSimActorClass) return;

    FActorSpawnParameters Params;
    Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    Params.bNoFail = true;

    SpeedSimActor = World->SpawnActor<ASpeedSimActor>(SpeedSimActorClass, FTransform::Identity, Params);
}
