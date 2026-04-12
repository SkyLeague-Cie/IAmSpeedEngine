// Fill out your copyright notice in the Description page of Project Settings.


#include "SpeedSimActor.h"
#include "IAmSpeed/Base/SUtils.h"
#include "Net/UnrealNetwork.h"
#include "SpeedWorldSubsystem.h"
#include "PhysicsEngine/PhysicsSettings.h"

unsigned int ASpeedSimActor::EngineFPS = 120; // Default value, can be overridden by config or in-game

// Sets default values
ASpeedSimActor::ASpeedSimActor()
{
    bReplicates = true;
    bAlwaysRelevant = true;
    SetNetUpdateFrequency(100.f);
    SetReplicateMovement(false);
	bAsyncPhysicsTickEnabled = true;
	EngineFPS = Speed::SimUtils::ComputePhysicsFPS(UPhysicsSettings::Get()->AsyncFixedTimeStepSize);
}

void ASpeedSimActor::UpdateNumFrame(const float& SimTime)
{
	_NumFrame = Speed::SimUtils::ComputeNumFrameFromSimTime(EngineFPS, SimTime);
}

unsigned int ASpeedSimActor::NumFrame() const
{
    return _NumFrame;
}

USpeedWorldSubsystem* ASpeedSimActor::GetSpeedWorldSubsystem(UWorld* World)
{
    if (!World) return nullptr;
    USpeedWorldSubsystem* SS = World->GetSubsystem<USpeedWorldSubsystem>();
	return SS;
}

void ASpeedSimActor::AsyncPhysicsTickActor(float Dt, float SimTime)
{
	UpdateNumFrame(SimTime);

	Simulate(Dt, SimTime);
}

void ASpeedSimActor::Simulate(const float& DeltaTime, const float& SimTime)
{
    if (!SpeedWorldSubsystem)
    {
		SpeedWorldSubsystem = GetSpeedWorldSubsystem(GetWorld());
		if (!SpeedWorldSubsystem)
		{
			return;
		}
	}

	SpeedWorldSubsystem->Step(DeltaTime, SimTime, NumFrame());
}
