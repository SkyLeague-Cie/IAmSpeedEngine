// Fill out your copyright notice in the Description page of Project Settings.


#include "SpeedCar.h"
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "IAmSpeed/Controllers/SpeedController.h"
#if !UE_BUILD_SHIPPING
#include "IAmSpeed/World/Simulation/SpeedGameMode.h"
#include "IAmSpeed/World/Simulation/SpeedSimulation.h"
#endif
#include "EnhancedInputComponent.h"

ASpeedCar::ASpeedCar(const FObjectInitializer& ObjectInitializer) :
	Super(ObjectInitializer.SetDefaultSubobjectClass<USpeedWheeledComponent>(ASpeedCar::VehicleMovementComponentName))
{
	SetReplicateMovement(false); // we do not want to replicate movement, we use our own replication system
	// Configure the car mesh
	GetMesh()->SetSimulatePhysics(false);
	GetMesh()->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Ignore);
	SpeedWheeledComponent = Cast<USpeedWheeledComponent>(GetVehicleMovement());
}

void ASpeedCar::BeginPlay()
{
	Super::BeginPlay();
	SpeedWheeledComponent->SetOwner(this);
}

void ASpeedCar::Tick(float Delta)
{
	Super::Tick(Delta);
#if !UE_BUILD_SHIPPING
	AuditPresentationFrameCadence(Delta);
#endif
	HandleKinematics();
	HandleSparkle();
}

#if !UE_BUILD_SHIPPING
void ASpeedCar::AuditPresentationFrameCadence(const float GameDeltaSeconds)
{
	if (!PresentationSimulation.IsValid())
	{
		const UWorld* World = GetWorld();
		const ASpeedGameMode* SpeedGameMode = World
			? Cast<ASpeedGameMode>(World->GetAuthGameMode())
			: nullptr;
		PresentationSimulation = SpeedGameMode
			? SpeedGameMode->GetSpeedSimulation()
			: nullptr;
	}

	if (ASpeedSimulation* Simulation = PresentationSimulation.Get())
	{
		// The virtual policy is deliberately invoked immediately before the
		// actor consumes the newest physical state in HandleKinematics().
		Simulation->AuditPresentationFrameCadence(
			*this, GameDeltaSeconds, LastObservedSimulationFrame);
	}
}
#endif

void ASpeedCar::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Super::EndPlay(EndPlayReason);

	// Alternatively you can clear ALL timers that belong to this (Actor) instance.
	GetWorld()->GetTimerManager().ClearAllTimersForObject(this);
}

void ASpeedCar::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

	if (UEnhancedInputComponent* EnhancedInputComponent =
		Cast<UEnhancedInputComponent>(PlayerInputComponent))
	{
		if (ASpeedController* SpeedController =
			Cast<ASpeedController>(GetController()))
		{
			SpeedController->SetupEnhancedInputComponent(EnhancedInputComponent);
		}
	}
}

void ASpeedCar::FreezeMovement()
{
	SpeedWheeledComponent->FreezeMovement();
}

void ASpeedCar::HandleKinematics()
{
	SetActorLocation(SpeedWheeledComponent->GetPhysLocation());
	SetActorRotation(SpeedWheeledComponent->GetPhysRotation());
	// GEngine->AddOnScreenDebugMessage(-1, 0.0f, FColor::Purple, FString::Printf(TEXT("[%s] Rotation = %s"), *GetRole(),
	//	*GetActorRotation().ToString()));
	GetMesh()->SetPhysicsLinearVelocity(SpeedWheeledComponent->GetPhysVelocity());
	GetMesh()->SetPhysicsAngularVelocityInDegrees(SpeedWheeledComponent->GetPhysAngularVelocity());
	SpeedWheeledComponent->UpdateWheelVisuals();
}

void ASpeedCar::SetThrottleInput(const float& Throttle)
{
	SpeedWheeledComponent->SetPhysThrottleInput(Throttle);
}

void ASpeedCar::SetBrakeInput(const float& Brake)
{
	SpeedWheeledComponent->SetPhysBrakeInput(Brake);
}

void ASpeedCar::SetSteeringInput(const float& Steering)
{
	SpeedWheeledComponent->SetPhysSteeringInput(Steering);
}

void ASpeedCar::SetPhysSparkleLocation(const FVector& HitLocation)
{
	hasSparkleLocation = true;
	SparkleLocation = HitLocation;
}

void ASpeedCar::HandleSparkle()
{
	if (hasSparkleLocation)
	{
		SetSparkleLoction(SparkleLocation);
		StartSparkleTimer();
		hasSparkleLocation = false;
	}
}

void ASpeedCar::DemoedByPrv(ASpeedCar* car)
{

}

void ASpeedCar::StartSparkleTimer()
{
	OnStartSparkle();
	GetWorld()->GetTimerManager().SetTimer(SparkleTimerHandler, this, &ASpeedCar::StopSparkle, 0.2, false);
}

void ASpeedCar::DemoedBy(ASpeedCar* otherCar)
{
	if (otherCar != nullptr && otherCar->IsValidLowLevelFast())
	{
		DemoedByPrv(otherCar);
	}
}

void ASpeedCar::StartConfrontationInSec(const float& TimeSec)
{
	if (!HasAuthority())
	{
		return;
	}
	SpeedWheeledComponent->StartConfrontationInSec(TimeSec);
}

bool ASpeedCar::OnTheSameTeamAs(const ASpeedCar& OtherCar) const
{
	return false;
}

bool ASpeedCar::HasAuthority() const
{
	return SpeedWheeledComponent->HasAuthority();
}

bool ASpeedCar::IsOwningClient() const
{
	return SpeedWheeledComponent->IsOwningClient();
}

bool ASpeedCar::IsRemoteClient() const
{
	return SpeedWheeledComponent->IsRemoteClient();
}
