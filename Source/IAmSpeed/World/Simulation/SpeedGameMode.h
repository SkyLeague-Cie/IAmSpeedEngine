// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameMode.h"
#include "SpeedPhysicsSimulation.h"
#include "GameModeReplayRecorder.h"
#include <memory>
#include "SpeedGameMode.generated.h"


class ASpeedPhysicsSimulation;

/** Common gameplay simulation and replay entry point. */
UCLASS()
class IAMSPEED_API ASpeedGameMode : public AGameMode
{
	GENERATED_BODY()
#if WITH_DEV_AUTOMATION_TESTS
	friend struct FLocalNormalCameraFixtureAccess;
	friend class FSkyCameraAbsoluteParentTest; // Test fixture declaration, no runtime fields or bypass.
#endif
#if WITH_DEV_AUTOMATION_TESTS
	friend class FSkyAIInputPauseRegistryTest;
#endif
#if WITH_DEV_AUTOMATION_TESTS
	friend class FIAmSpeedControllerInputLifecycleTest;
	friend class FSLControlBatchTest;
#endif

public:
	ASpeedGameMode();
	void InitGame(const FString& MapName, const FString& Options, FString& ErrorMessage) override;
	void EndPlay(const EEndPlayReason::Type Reason) override;
	// Common recording entry for every gameplay mode, independent of map/menu.
	bool SaveReplay(APlayerController& Player);
	bool AttachReplayRecorder(std::shared_ptr<Speed::Replay::IGameModeReplayRecorder> Recorder);
	bool DetachReplayRecorder(const Speed::Replay::IGameModeReplayRecorder* Expected);
	/** Automated scenarios reserve input authority and lifecycle for their
	 * exact producer. Merely opening a test map may still allow devices. */
	virtual bool AllowsDeviceInputAuthority() const { return true; }
	virtual void BeginPlay() override;
	/** Pauses or resumes the separately-owned simulation lane when it exists. */
	void SetSimulationPaused(bool bPaused);
	/** Returns the single authoritative simulation actor owned by this game mode. */
	ASpeedPhysicsSimulation* GetSpeedSimulation() const { return SpeedSimActor; }
	/** Common immutable presentation interface; no physical capability is implied. */
	ASpeedSimulation* GetSimulationPresentation() const { return SpeedSimActor; }

protected:
	UPROPERTY(EditDefaultsOnly)
	TSubclassOf<class ASpeedPhysicsSimulation> SpeedSimActorClass;

private:
	std::shared_ptr<Speed::Replay::IGameModeReplayRecorder> ReplayRecorder;
	UPROPERTY()
	TObjectPtr<ASpeedPhysicsSimulation> SpeedSimActor;
};
