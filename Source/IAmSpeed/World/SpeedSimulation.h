// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "SpeedSimulation.generated.h"

class USpeedWorldSubsystem;
struct FCanonicalFrameContext;

/*
* ASpeedSimulation : Actor responsible for ticking the IAmSpeed Engine simulation.
* It is the one that calls Step() on the SpeedWorldSubsystem every physics frame.
*/
UCLASS()
class IAMSPEED_API ASpeedSimulation : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ASpeedSimulation();

	void AsyncPhysicsTickActor(float DeltaTime, float SimTime) override;
	void Simulate(const float& DeltaTime, const float& SimTime);
	void StepCanonicalFrame(const FCanonicalFrameContext& Context);
	bool ShouldTickIfViewportsOnly() const override { return true; }
	static unsigned int GetEngineFPS() { return EngineFPS; }
protected:
	void UpdateNumFrame(const float& SimTime);
	unsigned int NumFrame() const;
	static USpeedWorldSubsystem* GetSpeedWorldSubsystem(UWorld* World);
	void RealTimeSimulation();
	bool FastSimulation(uint32 MaxFrameCount);
	float RealDeltaTime = 0.0f;
private:
	void RunCanonicalFrames(uint32 FrameCount);
	void RecordStepPerformance(double PhysicalFrameMilliseconds);
	USpeedWorldSubsystem* SpeedWorldSubsystem = nullptr;

	unsigned int _NumFrame = 0;
	uint64 CanonicalNumFrame = 0;
	bool bCanonicalFrameInitialized = false;
	bool bFastSimulationBudgetExceeded = false;
	bool bStaticCollisionReadinessErrorReported = false;
	uint32 FastReadyDelayCallbacksObserved = 0;
	uint64 LastSlowFrameReport = 0;
	uint32 SlowFrameReportCount = 0;
	uint64 LastRecordedStepSerial = 0;
	uint64 PerformanceFrameCount = 0;
	uint64 PerformanceWarningFrameCount = 0;
	uint64 PerformanceIterationLimitCount = 0;
	double PerformanceTotalStepMilliseconds = 0.0;
	double PerformanceMaximumStepMilliseconds = 0.0;
	double LastPhysicalFrameMilliseconds = 0.0;
	static unsigned int EngineFPS; // The FPS at which the IAmSpeed Engine is running
};
