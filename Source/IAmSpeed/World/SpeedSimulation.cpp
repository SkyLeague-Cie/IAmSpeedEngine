// Fill out your copyright notice in the Description page of Project Settings.


#include "SpeedSimulation.h"
#include "CanonicalFrameContext.h"
#include "CanonicalFrameDriver.h"
#include "IAmSpeed/World/Analytic/StaticWorldQueryAudit.h"
#include "IAmSpeed/Base/SUtils.h"
#include "Net/UnrealNetwork.h"
#include "SpeedWorldSubsystem.h"
#include "PhysicsEngine/PhysicsSettings.h"
#include "HAL/PlatformTime.h"

unsigned int ASpeedSimulation::EngineFPS = 120; // Default value, can be overridden by config or in-game

// Sets default values
ASpeedSimulation::ASpeedSimulation()
{
    bReplicates = true;
    bAlwaysRelevant = true;
    SetNetUpdateFrequency(100.f);
    SetReplicateMovement(false);
	bAsyncPhysicsTickEnabled = true;
	EngineFPS = Speed::SimUtils::ComputePhysicsFPS(UPhysicsSettings::Get()->AsyncFixedTimeStepSize);
	RealDeltaTime = 1.0f / static_cast<float>(EngineFPS);
}

void ASpeedSimulation::UpdateNumFrame(const float& SimTime)
{
	_NumFrame = Speed::SimUtils::ComputeNumFrameFromSimTime(EngineFPS, SimTime);
}

unsigned int ASpeedSimulation::NumFrame() const
{
    return _NumFrame;
}

USpeedWorldSubsystem* ASpeedSimulation::GetSpeedWorldSubsystem(UWorld* World)
{
    if (!World) return nullptr;
    USpeedWorldSubsystem* SS = World->GetSubsystem<USpeedWorldSubsystem>();
	return SS;
}

void ASpeedSimulation::AsyncPhysicsTickActor(float Dt, float SimTime)
{
	if (Speed::CanonicalFrameDriver::IsEnabled())
	{
		if (!bCanonicalFrameInitialized)
		{
			// Preserve startup/toggle alignment once, then let the integer counter
			// become the sole time authority for all following canonical frames.
			UpdateNumFrame(SimTime);
			CanonicalNumFrame = NumFrame() > 0 ? uint64(NumFrame() - 1u) : 0u;
			bCanonicalFrameInitialized = true;
		}

		if (Speed::CanonicalFrameDriver::IsFastSimulationEnabled())
		{
			RealDeltaTime = 0.0f;
			if (!SpeedWorldSubsystem)
			{
				SpeedWorldSubsystem = GetSpeedWorldSubsystem(GetWorld());
			}
			if (!SpeedWorldSubsystem || bFastSimulationBudgetExceeded)
			{
				return;
			}

			const ECanonicalRunControlState RunState =
				SpeedWorldSubsystem->GetCanonicalRunControlState();
			if (RunState == ECanonicalRunControlState::WaitingForScenario ||
				RunState == ECanonicalRunControlState::Uncontrolled)
			{
				FastReadyDelayCallbacksObserved = 0;
				return;
			}
			if (RunState == ECanonicalRunControlState::Complete)
			{
				return;
			}
			if (FastReadyDelayCallbacksObserved <
				Speed::CanonicalFrameDriver::GetFastReadyDelayCallbacks())
			{
				++FastReadyDelayCallbacksObserved;
				return;
			}

			if (!FastSimulation(
				Speed::CanonicalFrameDriver::GetFastMaxFramesPerRun()))
			{
				bFastSimulationBudgetExceeded = true;
				UE_LOG(LogTemp, Error,
					TEXT("FastSimulation exhausted its frame budget at canonical frame %llu without a terminal result."),
					CanonicalNumFrame);
			}
		}
		else
		{
			RealTimeSimulation();
		}
		return;
	}

	bCanonicalFrameInitialized = false;
	bFastSimulationBudgetExceeded = false;
	FastReadyDelayCallbacksObserved = 0;
	RealDeltaTime = 1.0f / static_cast<float>(EngineFPS);
	UpdateNumFrame(SimTime);
	Simulate(Dt, SimTime);
}

void ASpeedSimulation::RealTimeSimulation()
{
	RealDeltaTime = 1.0f / static_cast<float>(EngineFPS);
	RunCanonicalFrames(1u);
}

bool ASpeedSimulation::FastSimulation(const uint32 MaxFrameCount)
{
	RealDeltaTime = 0.0f;
	const uint64 StartFrame = CanonicalNumFrame;
	const double StartSeconds = FPlatformTime::Seconds();
	for (uint32 FrameIndex = 0; FrameIndex < MaxFrameCount; ++FrameIndex)
	{
		RunCanonicalFrames(1u);
		if (SpeedWorldSubsystem->GetCanonicalRunControlState() ==
			ECanonicalRunControlState::Complete)
		{
			const double WallSeconds = FPlatformTime::Seconds() - StartSeconds;
			const uint64 SimulatedFrames = CanonicalNumFrame - StartFrame;
			const double SimulatedSeconds = static_cast<double>(SimulatedFrames) /
				static_cast<double>(FCanonicalFrameContext::PhysicsFramesPerSecond);
			UE_LOG(LogTemp, Display,
				TEXT("[FastSimulationRun] StartFrame=%llu EndFrame=%llu Frames=%llu WallSeconds=%.9f SimulatedSeconds=%.9f RealtimeMultiplier=%.3f"),
				StartFrame,
				CanonicalNumFrame > 0 ? CanonicalNumFrame - 1u : 0u,
				SimulatedFrames,
				WallSeconds,
				SimulatedSeconds,
				WallSeconds > 0.0 ? SimulatedSeconds / WallSeconds : 0.0);
			return true;
		}
	}
	return false;
}

void ASpeedSimulation::RunCanonicalFrames(const uint32 FrameCount)
{
	for (uint32 FrameIndex = 0; FrameIndex < FrameCount; ++FrameIndex)
	{
		StepCanonicalFrame(FCanonicalFrameContext(CanonicalNumFrame));
		++CanonicalNumFrame;
	}
}

void ASpeedSimulation::Simulate(const float& DeltaTime, const float& SimTime)
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

void ASpeedSimulation::StepCanonicalFrame(const FCanonicalFrameContext& Context)
{
	if (!SpeedWorldSubsystem)
	{
		SpeedWorldSubsystem = GetSpeedWorldSubsystem(GetWorld());
		if (!SpeedWorldSubsystem)
		{
			return;
		}
	}

	Speed::Analytic::FStaticWorldQueryAudit::BeginFrame(
		Context.NumFrame, SpeedWorldSubsystem->GetAnalyticWorldData());
	SpeedWorldSubsystem->PrepareCanonicalFrame(Context);
	SpeedWorldSubsystem->Step(
		Context.PhysicalDeltaTime,
		Context.SimTime,
		static_cast<unsigned int>(Context.NumFrame));
	Speed::Analytic::FStaticWorldQueryAudit::EndFrame();
}
