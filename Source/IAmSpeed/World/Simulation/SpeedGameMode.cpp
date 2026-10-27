// Fill out your copyright notice in the Description page of Project Settings.


#include "SpeedGameMode.h"
#include "Engine/World.h"
#include "CanonicalFrameDriver.h"
#include "FastSimulation.h"
#include "RealTimeSimulation.h"
#include "GameFramework/PlayerController.h"

Speed::Replay::FGameModeRecorderRequested& Speed::Replay::OnGameModeRecorderRequested()
{
    static FGameModeRecorderRequested Requested;
    return Requested;
}

Speed::Replay::FGameModeReplayUnavailable& Speed::Replay::OnGameModeReplayUnavailable()
{
    static FGameModeReplayUnavailable Unavailable;
    return Unavailable;
}

ASpeedGameMode::ASpeedGameMode()
{
    SpeedSimActorClass = ARealTimeSimulation::StaticClass();
}

void ASpeedGameMode::InitGame(const FString& MapName, const FString& Options, FString& ErrorMessage)
{
    Super::InitGame(MapName, Options, ErrorMessage);
    // InitGame precedes player/simulation BeginPlay. The recorder can observe
    // the actual world's first physical frame without guessing a map URL.
    if (HasAuthority() && GetWorld() && GetWorld()->IsGameWorld())
        Speed::Replay::OnGameModeRecorderRequested().Broadcast(*this);
}

bool ASpeedGameMode::AttachReplayRecorder(std::shared_ptr<Speed::Replay::IGameModeReplayRecorder> Recorder)
{
    check(IsInGameThread());
    if (!HasAuthority() || !Recorder || ReplayRecorder || !GetWorld() || GetWorld()->bIsTearingDown ||
        GetWorld()->GetAuthGameMode() != this)
        return false;
    ReplayRecorder = std::move(Recorder);
    return true;
}

bool ASpeedGameMode::DetachReplayRecorder(const Speed::Replay::IGameModeReplayRecorder* Expected)
{
    check(IsInGameThread());
    if (!Expected || ReplayRecorder.get() != Expected) return false;
    ReplayRecorder.reset();
    return true;
}

bool ASpeedGameMode::SaveReplay(APlayerController& Player)
{
    check(IsInGameThread());
    if (!HasAuthority() || !Player.IsLocalController() || Player.GetWorld() != GetWorld() ||
        !GetWorld() || GetWorld()->bIsTearingDown || GetWorld()->GetAuthGameMode() != this) return false;
    if (!ReplayRecorder)
    {
        Speed::Replay::OnGameModeReplayUnavailable().Broadcast(Player);
        return false;
    }
    // Retain across callbacks: retirement can synchronously detach the owner.
    const auto Recorder = ReplayRecorder;
    return Recorder->SaveReplay(Player);
}

void ASpeedGameMode::EndPlay(const EEndPlayReason::Type Reason)
{
    auto Recorder = std::move(ReplayRecorder);
    if (Recorder) Recorder->StopRecording();
    // The backend retains pending write receipts until its normal IO closure.
    // Stopping the GameMode's entry point is not a claim that IO has joined.
    Super::EndPlay(Reason);
}

void ASpeedGameMode::BeginPlay()
{
    Super::BeginPlay();

    if (!HasAuthority()) return;
    if (SpeedSimActor) return;

    UWorld* World = GetWorld();
    if (!World || !SpeedSimActorClass) return;

	TSubclassOf<ASpeedPhysicsSimulation> SimulationClass = SpeedSimActorClass;
	if (Speed::CanonicalFrameDriver::IsFastSimulationEnabled())
	{
		SimulationClass = AFastSimulation::StaticClass();
	}

    FActorSpawnParameters Params;
    Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    Params.bNoFail = true;

    SpeedSimActor = World->SpawnActor<ASpeedPhysicsSimulation>(SimulationClass, FTransform::Identity, Params);
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
