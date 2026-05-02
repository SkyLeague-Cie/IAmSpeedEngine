// Fill out your copyright notice in the Description page of Project Settings.

#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "ChaosVehicleManager.h"
#include "IAmSpeed/Base/SpeedConstant.h"
#include "IAmSpeed/SubBodies/Configs/WheelSubBodyConfig.h"
#include "IAmSpeed/SubBodies/Solid/BoxSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SWheelSubBody.h"
#include "IAmSpeed/World/SpeedWorldSubsystem.h"
#include "IAmSpeed/Actors/SpeedCar.h"
#include "ChaosVehicleWheel.h"

DEFINE_LOG_CATEGORY(WheelNetcodeLog);
DEFINE_LOG_CATEGORY(SpeedInputLog);
DEFINE_LOG_CATEGORY(SpeedPhysicsLog);

const FName USpeedWheeledComponent::HitboxName = TEXT("Hitbox");
const TArray<FName> USpeedWheeledComponent::WheelNames = { TEXT("Wheel_0"), TEXT("Wheel_1"), TEXT("Wheel_2"), TEXT("Wheel_3")};

USpeedWheeledComponent::USpeedWheeledComponent(const FObjectInitializer& ObjectInitializer) :
	Super(ObjectInitializer)
{
	SetIsReplicatedByDefault(true);

	InitChaosVehicle();
	InitNetwork();

	SetEngineFPS(Speed::SimUtils::ComputePhysicsFPS(UPhysicsSettings::Get()->AsyncFixedTimeStepSize));
	GravityZ = GetDefault<UPhysicsSettings>()->DefaultGravityZ;
	MinNbFramesBeforeCanMove = TimeBeforeCanMove * EngineFPS;
	WheeledPhysicsState.nbFramesbeforeCanMove = MinNbFramesBeforeCanMove;

	SetSubBodies(CreateSubBodies());
	if (SubBodies.Num() > 0)
	{
		HitboxSubBody = CastChecked<UBoxSubBody>(SubBodies[0]);
	}

	for (USSubBody* SubBody : SubBodies)
	{
		if (USWheelSubBody* WheelSubBody = Cast<USWheelSubBody>(SubBody))
		{
			WheelSubBodies.Add(WheelSubBody);
		}
	}
}

void USpeedWheeledComponent::SetOwner(AActor* NewOwner)
{
	if (NewOwner == nullptr)
	{
		return;
	}
	SpeedCarOwner = Cast<ASpeedCar>(NewOwner);
	SetPhysLocation(NewOwner->GetActorLocation());
	SetPhysRotation(NewOwner->GetActorRotation().Quaternion());

	// init sub-bodies
	if (SubBodies.Num() > 0)
	{
		HitboxSubBody = Cast<UBoxSubBody>(SubBodies[0]);
	}

	WheelSubBodies.Empty();
	for (USSubBody* SubBody : SubBodies)
	{
		if (USWheelSubBody* WheelSubBody = Cast<USWheelSubBody>(SubBody))
		{
			WheelSubBodies.Add(WheelSubBody);
		}
	}

	for (int WheelIdx = 0; WheelIdx < Wheels.Num(); WheelIdx++)
	{
		auto Wheel = Wheels[WheelIdx].Get();
		auto& Suspension = Wheel->GetPhysicsSuspensionConfig();
		auto& WheelSubBody = WheelSubBodies[WheelIdx];
		WheelSubBody->SetIdx(WheelIdx);
		WheelSubBody->SetChaosWheel(Wheel);
		WheelSubBody->SetWheelSim(&SkySimulation->PVehicle->Wheels[WheelIdx]);
		WheelSubBody->SetSuspensionSim(&SkySimulation->PVehicle->Suspension[WheelIdx]);
	}

	// Adjust wheel local offsets to be symmetric on both sides of the vehicle
	for (auto& Axle : SkySimulation->PVehicle->GetAxles())
	{
		uint16 WheelIdxA = Axle.Setup.WheelIndex[0];
		uint16 WheelIdxB = Axle.Setup.WheelIndex[1];

		auto& WheelA = *WheelSubBodies[WheelIdxA].Get();
		auto& WheelB = *WheelSubBodies[WheelIdxB].Get();

		FVector WheelALocalPos = WheelA.GetLocalOffset();
		FVector WheelBLocalPos = WheelB.GetLocalOffset();
		auto xLocalOffset = 0.5f * (WheelALocalPos.X + WheelBLocalPos.X);
		auto yMeanLocalOffset = 0.5f * (FMath::Abs(WheelALocalPos.Y) + FMath::Abs(WheelBLocalPos.Y));
		auto yALocalOffset = -yMeanLocalOffset;
		auto yBLocalOffset = yMeanLocalOffset;
		auto zLocalOffset = 0.5f * (WheelALocalPos.Z + WheelBLocalPos.Z);
		FVector ALocalOffset = FVector(xLocalOffset, yALocalOffset, zLocalOffset);
		WheelA.SetLocalOffset(ALocalOffset);
		FVector BLocalOffset = FVector(xLocalOffset, yBLocalOffset, zLocalOffset);
		WheelB.SetLocalOffset(BLocalOffset);
#if !(UE_BUILD_SHIPPING)
		/*UE_LOG(LogTemp, Log,
			TEXT("Wheel[%d] Local Offset: %s"),
			WheelIdxA,
			*WheelALocalPos.ToString()
		);
		UE_LOG(LogTemp, Log,
			TEXT("Wheel[%d] Local Offset: %s"),
			WheelIdxB,
			*WheelBLocalPos.ToString()
		);*/
#endif
	}

	// init all sub-bodies with the owner
	HitboxSubBody->Initialize(this);
	for (auto& WheelSubBody : WheelSubBodies)
	{
		WheelSubBody->Initialize(this);
	}
	CarLocalInvI = Speed::SBox::ComputeLocalInverseInertiaTensor(HitboxSubBody->GetBoxExtent(), GetPhysMass());

	UpdateSubBodiesKinematics();

	// start physics thread
	SetAsyncPhysicsTickEnabled(true);
	// Register in SpeedWorldSubsystem
	if (UWorld* World = GetWorld())
	{
		if (USpeedWorldSubsystem* SpeedWorldSubsystem = World->GetSubsystem<USpeedWorldSubsystem>())
		{
			SpeedWorldSubsystem->RegisterSpeedComponent(this);
		}
	}
}

ASpeedCar* USpeedWheeledComponent::GetSpeedCarOwner() const
{
	return SpeedCarOwner;
}

void USpeedWheeledComponent::OnCreatePhysicsState()
{
	// =========> START OVERLOAD Chaos prediction here
	UPawnMovementComponent::OnCreatePhysicsState();

	VehicleSetupTag = FChaosVehicleManager::VehicleSetupTag;

	// only create Physics vehicle in game
	UWorld* World = GetWorld();
	if (World->IsGameWorld())
	{
		if (FPhysScene* PhysScene = World->GetPhysicsScene())
		{
			if (FChaosVehicleManager::GetVehicleManagerFromScene(PhysScene))
			{
				CreateVehicle();
				FixupSkeletalMesh();
				VehicleSimulationPT->PVehicle->bSuspensionEnabled = bSuspensionEnabled;
				VehicleSimulationPT->PVehicle->bWheelFrictionEnabled = bWheelFrictionEnabled;
				VehicleSimulationPT->PVehicle->bMechanicalSimEnabled = bMechanicalSimEnabled;

				if (PVehicleOutput)
				{
					FChaosVehicleManager* VehicleManager = FChaosVehicleManager::GetVehicleManagerFromScene(PhysScene);
					VehicleManager->AddVehicle(this);
				}
			}
		}
	}

	FBodyInstance* BodyInstance = nullptr;
	if (USkeletalMeshComponent* SkeletalMesh = GetSkeletalMesh())
	{
		// this line was causing the server wheel positions to not be updated - this is already a user property so don't override it here and leave it to user to select the right option for their scenario
		//SkeletalMesh->VisibilityBasedAnimTickOption = EVisibilityBasedAnimTickOption::OnlyTickPoseWhenRendered;
		BodyInstance = &SkeletalMesh->BodyInstance;
	}
	// =========> END OVERLOAD Chaos pediction here

	// only create data history in game
	if (World->IsGameWorld())
	{
		if (SNetworkPhysicsComponent)
		{
			SNetworkPhysicsComponent->CreateDataHistory<FPhysicsSpeedTraits>(this);
		}
		if (WheeledNetworkPhysicsComponent)
		{
			WheeledNetworkPhysicsComponent->CreateDataHistory<FPhysicsWheeledTraits>(this);
		}
	}
}

void USpeedWheeledComponent::OnDestroyPhysicsState()
{
	if (PVehicleOutput.IsValid())
	{
		FChaosVehicleManager* VehicleManager = FChaosVehicleManager::GetVehicleManagerFromScene(GetWorld()->GetPhysicsScene());
		VehicleManager->RemoveVehicle(this);
		PVehicleOutput.Reset(nullptr);

		if (UpdatedComponent)
		{
			UpdatedComponent->RecreatePhysicsState();
		}
	}

	if (SNetworkPhysicsComponent)
	{
		SNetworkPhysicsComponent->RemoveDataHistory();
	}
	if (WheeledNetworkPhysicsComponent)
	{
		WheeledNetworkPhysicsComponent->RemoveDataHistory();
	}
	UPawnMovementComponent::OnDestroyPhysicsState();
	bUsingNetworkPhysicsPrediction = false; // stop prediction to prevent game crash
}

TArray<USSubBody*> USpeedWheeledComponent::CreateSubBodies()
{
	TArray<USSubBody*> ret;
	ret.Add(CreateHitboxSubBody());
	ret.Append(CreateWheelSubBodies());
	return ret;
}

TArray<USWheelSubBody*> USpeedWheeledComponent::CreateWheelSubBodies()
{
	TArray<USWheelSubBody*> ret;
	for (int WheelIdx = 0; WheelIdx < WheelSetups.Num(); WheelIdx++)
	{
		ret.Add(CreateWheelSubBody(WheelIdx));
	}
	return ret;
}

UBoxSubBody* USpeedWheeledComponent::CreateHitboxSubBody()
{
	return CreateDefaultSubobject<UBoxSubBody, UBoxSubBody>(HitboxName);
}

USWheelSubBody* USpeedWheeledComponent::CreateWheelSubBody(const int& WheelIndex)
{
	return CreateDefaultSubobject<USWheelSubBody, USWheelSubBody>(WheelNames[WheelIndex]);
}

void USpeedWheeledComponent::SetSubBodies(const TArray<USSubBody*>& NewSubBodies)
{
	SubBodies = NewSubBodies;
}

unsigned int USpeedWheeledComponent::NumFrame() const
{
	return BaseGameState.NumFrame;
}

uint16 USpeedWheeledComponent::GetMinNbFramesBeforeCanMove() const
{
	return MinNbFramesBeforeCanMove;
}

float USpeedWheeledComponent::GetPhysMass() const
{
	return PhysMass;
}

FVector USpeedWheeledComponent::GetPhysCOM() const
{
	return GetPhysLocation() + CenterOfMass;
}

const TArray<USSubBody*>& USpeedWheeledComponent::GetSubBodies() const
{
	return SubBodies;
}

SubBodyConfig USpeedWheeledComponent::GetSubBodyConfig(const USSubBody& SubBody) const
{
	if (HitboxSubBody && &SubBody == HitboxSubBody)
	{
		SubBodyConfig Config;
		return Config;
	}
	return SubBodyConfig();
}

WheelSubBodyConfig USpeedWheeledComponent::GetWheelSubBodyConfig(const USWheelSubBody& SubBody) const
{
	// we will not use this to configure wheel sub-bodies since the wheel sub-body configuration is determined by the corresponding wheel setup
	return WheelSubBodyConfig();
}

const SKinematic& USpeedWheeledComponent::GetKinematicsOfSubBody(const USSubBody& SubBody, const unsigned int& LocalFrame) const
{
	const int32 Idx = LocalFrame % SpeedConstants::PredictedHistorySize;
	if (PredictedBaseFrames[Idx] == LocalFrame)
	{
		return PredictedBaseStates[Idx].Kinematic;
	}
	return BasePhysicsState.Kinematic;
}

FMatrix USpeedWheeledComponent::ComputeWorldInvInertiaTensor() const
{
	return SSBox::ComputeWorldInverseInertiaTensor(GetPhysRotation(), CarLocalInvI);
}

FMatrix USpeedWheeledComponent::ComputeWorldInvInertiaTensorOfSubBody(const USSubBody& SubBody) const
{
	// Done in sub-bodies, maybe remove this function from the interface since it's not really necessary to have it here if the sub-bodies can compute their own world inverse inertia tensor
	return FMatrix();
}

const SKinematic& USpeedWheeledComponent::GetKinematicState() const
{
	return BasePhysicsState.Kinematic;
}

const SKinematic& USpeedWheeledComponent::GetKinematicStateForFrame(const unsigned int& LocalFrame) const
{
	const int32 Idx = LocalFrame % SpeedConstants::PredictedHistorySize;
	if (PredictedBaseFrames[Idx] == LocalFrame)
	{
		return PredictedBaseStates[Idx].Kinematic;
	}
	return BasePhysicsState.Kinematic;
}

float USpeedWheeledComponent::GetPhysMaxSpeed() const
{
	return PhysMaxSpeed;
}

float USpeedWheeledComponent::GetPhysMaxAngularSpeed() const
{
	return PhysMaxAngularSpeed;
}

void USpeedWheeledComponent::PostPhysicsUpdatePrv(const float& delta)
{
	ISpeedWheeledComponent::PostPhysicsUpdatePrv(delta);
	ApplyNetworkCorrection(delta);
	QuantizePhysicalState();
	RecordPredictedState();
	RegisterWheelState();
}

void USpeedWheeledComponent::SetIsUpsideDown(bool bUpsideDown)
{
	WheeledGameState.bIsUpsideDown = bUpsideDown;
}

bool USpeedWheeledComponent::IsUpsideDown() const
{
	return WheeledGameState.bIsUpsideDown;
}

bool USpeedWheeledComponent::IsInAutoRecover() const
{
	return false;
}

bool USpeedWheeledComponent::IsSubBodyInAutoRecoverMode() const
{
	return IsInWheelAutoRecover();
}

bool USpeedWheeledComponent::IsInWheelAutoRecover() const
{
	return OneWheelOnGround() && !IsOnTheGround();
}

void USpeedWheeledComponent::RcvImpactOnSubBody(const USSubBody& SubBody, const FVector& Location)
{
	if (HitboxSubBody && &SubBody == HitboxSubBody)
	{
		SpeedCarOwner->SetPhysSparkleLocation(Location);
	}
	WheeledPhysicsState.FramesSinceLastImpact = 0;
}

void USpeedWheeledComponent::SetKinematicState(const SKinematic& NewKinematicState)
{
	BasePhysicsState.Kinematic = NewKinematicState;
}

void USpeedWheeledComponent::RegisterTestVelocity(const FVector& InitialVelocity)
{
	BaseGameState.TestVelocity = InitialVelocity;
}

void USpeedWheeledComponent::ApplyTestVelocity()
{
	if (BaseGameState.TestVelocity.IsZero())
	{
		return;
	}
	SetPhysVelocity(BaseGameState.TestVelocity);
	BaseGameState.TestVelocity = FVector::ZeroVector;
}

void USpeedWheeledComponent::AsyncPhysicsTickComponent(float DeltaTime, float SimTime)
{
	PhysicsTick(DeltaTime, SimTime);
}

void USpeedWheeledComponent::DemoedBy(ASpeedCar* otherCar)
{
	if (HasAuthority())
	{
		SpeedCarOwner->DemoedBy(otherCar);
	}
}

void USpeedWheeledComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Super::EndPlay(EndPlayReason);
	// Unregister from SpeedWorldSubsystem
	if (UWorld* World = GetWorld())
	{
		if (USpeedWorldSubsystem* SpeedWorldSubsystem = World->GetSubsystem<USpeedWorldSubsystem>())
		{
			SpeedWorldSubsystem->UnregisterSpeedComponent(this);
		}
	}

	// Make car sleeps else suspension will crash the game
	VehicleSimulationPT->VehicleState.bSleeping = true;
}

void USpeedWheeledComponent::SetEngineFPS(const unsigned int& FPS)
{
	EngineFPS = FPS;
}

float USpeedWheeledComponent::GetEngineFPS() const
{
	return EngineFPS;
}

void USpeedWheeledComponent::PhysicsTick(const float& DeltaTime, const float& SimTime)
{
	// Update NumFrame at the beginning of the tick so that it can be used in the rest of the tick functions
	UpdateFrameState(SimTime);

	// Handle forces that should be applied before the gameplay tick (e.g. gravity, damping, rest force)
	HandleGravity();
	PreGameplayTick(DeltaTime, SimTime);

	// Handle gameplay forces (e.g. from player input or AI)
	GameplayTick(DeltaTime, SimTime);

	// Handle rest force after gameplay tick so that the component can be at rest at the end of the tick if it should be
	UpdateAutoRecoverState();
	HandleRestForce();
	HandlePhysicsAutoRecover();

	// Apply velocity that was set for a test
	ApplyTestVelocity();

	// Handle any necessary updates after the gameplay tick
	ApplyAccelKinematicsConstraint(DeltaTime);
	HandleTimers();
	PostGameplayTick(DeltaTime, SimTime);
}

void USpeedWheeledComponent::UpdateFrameState(const float& SimTime)
{
	UpdateNumFrame(SimTime);
	UpdateInputs();
	TagStateHistoryProxyRole();
	RecoverWheelState();
}

void USpeedWheeledComponent::UpdateNumFrame(const float& SimTime)
{
	BaseGameState.NumFrame = Speed::SimUtils::ComputeNumFrameFromSimTime(EngineFPS, SimTime);
}

void USpeedWheeledComponent::UpdateInputs()
{
	// Update inputs
	WheeledPhysicalInput = WheeledUserInput; // copy all inputs for physics simulation
}

void USpeedWheeledComponent::UpdateState(float DeltaTime)
{
	// DO NOTHING HERE
}

void USpeedWheeledComponent::RecordPredictedState()
{
	const int32 LF = NumFrame();
	const int32 Idx = LF % SpeedConstants::PredictedHistorySize;

	PredictedBaseFrames[Idx] = LF;
	PredictedBaseStates[Idx] = BasePhysicsState;
	PredictedWheeledStates[Idx] = WheeledPhysicsState;
}

bool USpeedWheeledComponent::GetPredictedState(const int32& LocalFrame, FBasePhysicsState& OutState) const
{
	const int32 Idx = LocalFrame % SpeedConstants::PredictedHistorySize;
	if (PredictedBaseFrames[Idx] == LocalFrame)
	{
		OutState = PredictedBaseStates[Idx];
		return true;
	}
	return false;
}

bool USpeedWheeledComponent::GetPredictedState(const int32& LocalFrame, FWheeledPhysicsState& OutState) const
{
	const int32 Idx = LocalFrame % SpeedConstants::PredictedHistorySize;
	if (PredictedBaseFrames[Idx] == LocalFrame)
	{
		OutState = PredictedWheeledStates[Idx];
		return true;
	}
	return false;
}

void USpeedWheeledComponent::HandleGravity()
{
	if (bEnableGravity)
	{
		AddPhysAcceleration(FVector(0.0f, 0.0f, GravityZ));
	}
}

void USpeedWheeledComponent::PreGameplayTick(const float& DeltaTime, const float& SimTime)
{
	HandleGroundAirState();
	HandleGroundFriction(DeltaTime);
	HandleDamping(DeltaTime);
	HandleAngularDamping(DeltaTime);
	HandleSuspension(DeltaTime);
}

void USpeedWheeledComponent::HandleDamping(const float& delta)
{
	if (PhysDamping <= 0.0)
	{
		return;
	}

	FVector newVelocity = (1 - PhysDamping) * GetPhysVelocity();
	auto speed = GetPhysVelocity().Size();
	if (speed <= 1.0)
	{
		newVelocity = FVector::ZeroVector;
	}
	auto ForceToApply = (newVelocity - GetPhysVelocity()) / delta;
	AddPhysAcceleration(ForceToApply);
}

void USpeedWheeledComponent::HandleRestForce()
{
	if (!HitboxSubBody)
		return;

	if (!HitboxSubBody->HasPersistentGroundContact())
		return;

	const TArray<FVector>& Contacts = HitboxSubBody->GetGroundContacts();
	if (Contacts.Num() == 0)
		return;

	const bool bEdgeSupport = (Contacts.Num() == 2);

	// IMPORTANT: use persistent ground plane normal
	const FVector N = HitboxSubBody->GetGroundPlaneNormal().GetSafeNormal();

	// Total acceleration (world-space)
	const FVector Accel = GetPhysAcceleration();

	// --- HARD STOP: never apply rest force if we are separating from the plane ---
	const FVector Vcm = GetPhysVelocity();
	const float vN = FVector::DotProduct(Vcm, N);

	// If separating or stationary relative to plane, DO NOT apply rest force
	constexpr float VN_SEP_EPS = 3.0f; // cm/s
	if (vN >= VN_SEP_EPS)
		return;

	// Now check acceleration INTO the plane
	const float aN = FVector::DotProduct(Accel, N);
	if (aN >= 0.f)
		return;

	// Reaction force cancels normal acceleration
	const FVector RestForce = -Mass * aN * N;

	// ---------------------------------------------------------------------
	// Force application policy:
	//  - Face contact (>=3): distribute on manifold (stability)
	//  - Edge contact (==2): apply at COM to avoid creating stabilizing torque
	// ---------------------------------------------------------------------
	if (!bEdgeSupport)
	{
		const FVector ForcePerContact = RestForce / float(Contacts.Num());
		for (const FVector& P : Contacts)
		{
			AddPhysForceAtPoint(ForcePerContact, P, HitboxSubBody);
		}
	}
	else
	{
		// Apply at COM: same net force, near-zero artificial torque
		const FVector COM = HitboxSubBody->GetKinematicsFromOwner(NumFrame() + 1).Location;
		// AddForceOnHitboxAtLocation(RestForce, COM, Delta);
		AddPhysAcceleration(-aN * N);
	}

	// ---------------------------------------------------------------------
	// Angular edge constraint: DISABLE on edge support and during edge recover
	// because it prevents the car from falling off the edge.
	// ---------------------------------------------------------------------
	if (!bEdgeSupport)
	{
		FVector LockAxisWorld = FVector::ZeroVector;
		if (HitboxSubBody->ComputeEdgeLockAxis(
			HitboxSubBody->GetGroundPlaneNormal(),
			Contacts,
			LockAxisWorld))
		{
			// remove forbidden component in WORLD space directly
			const float a = FVector::DotProduct(GetPhysAngularAcceleration(), LockAxisWorld);
			constexpr float A_EPS = 0.02f;
			if (FMath::Abs(a) > A_EPS)
			{
				const FVector CorrectionWorld = -a * LockAxisWorld;
				AddPhysAngularAcceleration(CorrectionWorld);
			}
		}
	}
}

void USpeedWheeledComponent::UpdateAutoRecoverState()
{
}

void USpeedWheeledComponent::HandleGroundFriction(const float& delta)
{
	unsigned char NbWheelsOnGround = NumWheelsOnGround();
	FVector UpVector = GetNormalFromWheels(); // surface normal
	FVector GroundVelocity = GetPhysVelocity() - (GetPhysVelocity().Dot(UpVector)) * UpVector;
	float GroundSpeed = GroundVelocity.Size();

	// compute right vector
	FVector PhysRightVector = GetPhysRightVector();
	FVector RightVector = PhysRightVector - (PhysRightVector.Dot(UpVector)) * UpVector;
	RightVector.Normalize();

	// compute forward vector
	FVector PhysForwardVector = GetPhysForwardVector();
	FVector ForwardVector = PhysForwardVector - (PhysForwardVector.Dot(UpVector)) * UpVector;
	ForwardVector.Normalize();

	float ForwardSpeed = GetPhysVelocity().Dot(ForwardVector);
	float SideSpeed = GetPhysVelocity().Dot(RightVector);
	auto LocalForwardFriction = GetLocalForwardFriction();

	//linearDamping = 600;
	if (GroundSpeed != 0.0)
	{
		// Dampen forward velocity
		if (ForwardSpeed != 0.0f)
		{
			if (abs(ForwardSpeed) < LocalForwardFriction * delta)
			{
				// AddPhysAcceleration(-(ForwardSpeed / delta) * ForwardVector); // may not work now with quantization of velocity, so we directly set velocity to 0
				SetPhysVelocity(GetPhysVelocity() - ForwardSpeed * ForwardVector); // remove forward velocity
			}
			else
			{
				FVector ForwardVelocityVector = FMath::Sign(ForwardSpeed) * ForwardVector;
				AddPhysAcceleration(-LocalForwardFriction * ForwardVelocityVector);
				// GEngine->AddOnScreenDebugMessage(-1, 0.0f, FColor::Red, FString::Printf(TEXT("[%s] LocalForwardFriction: %f"), *SkycarOwner->GetRole(),
				//	LocalForwardFriction));
			}
		}

		// Dampen side velocity
		if (SideSpeed != 0.0f)
		{
			float allowedSideSpeed = WheeledPhysicsState.AllowedSideVelocity.Dot(RightVector);
			float overSpeed = FMath::Sign(SideSpeed) * (SideSpeed - allowedSideSpeed);
			if (overSpeed <= 0.0f)
			{
				// no need to dampen side speed
				return;
			}

			float LocalSideDamping = (GetLocalSideFriction()) * (NbWheelsOnGround / 4);
			if (overSpeed < LocalSideDamping * delta)
			{
				// addForce(-FMath::Sign(SideSpeed) * overSpeed * RightVector / delta, delta); // may not work now with quantization of velocity, so we directly set velocity to allowed side speed
				SetPhysVelocity(GetPhysVelocity() - FMath::Sign(SideSpeed) * overSpeed * RightVector); // remove side velocity
			}
			else
			{
				FVector SideVelocityVector = FMath::Sign(SideSpeed) * RightVector;
				AddPhysAcceleration(-LocalSideDamping * SideVelocityVector);
				// GEngine->AddOnScreenDebugMessage(-1, 0.0f, FColor::Red, FString::Printf(TEXT("[%s] LocalSideDamping: %f"), *SkycarOwner->GetRole(),
				//	LocalSideDamping));
			}
		}
	}
}

void USpeedWheeledComponent::HandleSuspension(const float& delta)
{
	if (delta <= KINDA_SMALL_NUMBER)
		return;

	// 1) Update suspension for each wheel
	for (auto& Wheel : WheelSubBodies)
	{
		Wheel->UpdatePhysicsState(delta);
		Wheel->UpdateSuspension(delta);
	}

	if (DisableSuspensionThisFrame())
		return;

	// 2) Apply suspension force + wall force
	for (auto& Axle : SkySimulation->PVehicle->GetAxles())
	{
		if (Axle.Setup.WheelIndex.Num() != 2)
			continue;

		uint16 WheelIdxA = Axle.Setup.WheelIndex[0];
		uint16 WheelIdxB = Axle.Setup.WheelIndex[1];

		auto& WheelA = WheelSubBodies[WheelIdxA];
		auto& WheelB = WheelSubBodies[WheelIdxB];

		// --- Base spring and damping forces ---
		auto ApplyWheelSuspension = [this, delta](USWheelSubBody& W)
			{
				if (!W.IsOnGround())
					return;

				const FVector SuspensionDir = GetPhysUpVector();
				const float Load = W.GetSuspensionForce();

				// optionnal projection on wall
				float Dot = FVector::DotProduct(SuspensionDir, W.GetHitContactNormal());
				float Scale = FMath::Clamp(Dot, 0.f, 1.f);

				FVector Force = SuspensionDir * Load * Scale;
				// const FVector ApplicationPoint = W.WorldPos();
				const FVector ApplicationPoint = W.GetHit().Location - W.GetHitContactNormal() * W.Radius();
				/*UE_LOG(
					SpeedPhysicsLog,
					Warning,
					TEXT("[ApplyWheelSuspension] For Frame=%d on wheel=%d. Force=%s, ApplicationPoint=%s, Kinematic=%s"), NumFrame(), W.Idx(),
					*Force.ToString(), *ApplicationPoint.ToString(), *BasePhysicsState.Kinematic.ToString());
					if (W.Idx() == 0)
					{
						UE_LOG(
							SpeedPhysicsLog,
							Warning,
							TEXT("[ApplyWheelSuspension] For Frame=%d on wheel=%d. ChassisUp=%s, HitNormal=%s, ApplicationPoint=%s, HitDistance=%fcm Force=%f Vel=%fcm/s SpringDisplacement=%fcm"), NumFrame(), W.Idx(), *GetPhysUpVector().ToString(),
							*W.GetHitContactNormal().ToString(), *ApplicationPoint.ToString(), W.GetHit().Distance, Load, GetPhysVelocity().Size(), W.SpringDisplacement());
					}*/

				AddPhysForceAtPoint(Force, ApplicationPoint);
			};

		ApplyWheelSuspension(*WheelA);
		ApplyWheelSuspension(*WheelB);
	}
}

void USpeedWheeledComponent::HandleAngularDamping(const float& delta)
{
	// generic case
	// aerial damping
	auto angularVelocityToDampen = GetPhysAngularVelocity() - WheeledPhysicsState.AllowedAngularVelocity;
	DampenAirAngularVelocity(angularVelocityToDampen, delta);

	// ground damping
	if (OneWheelOnGround())
	{
		auto GroundNormal = GetNormalFromWheels();
		auto GroundAngularVelocity = FVector::DotProduct(angularVelocityToDampen, GroundNormal) * GroundNormal;
		DampenAngularVelocity(GroundAngularDamping, GroundAngularVelocity, delta);
	}
}

void USpeedWheeledComponent::DampenAngularVelocity(const float& DampingFactor, const FVector& VelocityToDampen, const float& delta)
{
	if (DampingFactor <= 0)
	{
		return;
	}

	auto velocityLength = VelocityToDampen.Length();
	if (velocityLength == 0.0)
	{
		return;
	}
	if (velocityLength <= DampingFactor * delta)
	{
		SetPhysAngularVelocity(GetPhysAngularVelocity() - VelocityToDampen);
	}
	else
	{
		auto velocityNormalized = VelocityToDampen / velocityLength;
		auto newVelocity = (1 - DampingFactor / 100.0) * VelocityToDampen;
		auto AngAccelToApply = (newVelocity - VelocityToDampen) / delta;
		AddPhysAngularAcceleration(AngAccelToApply);
	}
}

void USpeedWheeledComponent::DampenAirAngularVelocity(const FVector& VelocityToDampen, const float& delta)
{
	// get axes
	const FVector RollAxis = GetPhysForwardVector();
	const FVector PitchAxis = GetPhysRightVector();
	const FVector YawAxis = GetPhysUpVector();
	const float RollVelocity = FVector::DotProduct(VelocityToDampen, RollAxis);
	const float PitchVelocity = FVector::DotProduct(VelocityToDampen, PitchAxis);
	const float YawVelocity = FVector::DotProduct(VelocityToDampen, YawAxis);

	// get drag coefficients
	const float roll_drag = RollDragCoeff;
	const float pitch_drag = PitchDragCoeff;
	const float yaw_drag = YawDragCoeff;

	// compute new angular velocities along each axis
	const float NewRollVelocity = FMath::Abs(RollVelocity) <= roll_drag * delta ? 0.0 :
		RollVelocity * (1.0 - roll_drag * delta);
	const float NewPitchVelocity = FMath::Abs(PitchVelocity) <= pitch_drag * delta ? 0.0 :
		PitchVelocity * (1.0 - pitch_drag * delta);
	const float NewYawVelocity = FMath::Abs(YawVelocity) <= yaw_drag * delta ? 0.0 :
		YawVelocity * (1.0 - yaw_drag * delta);

	const FVector TargetAngularVelocity = NewRollVelocity * RollAxis + NewPitchVelocity * PitchAxis + NewYawVelocity * YawAxis;
	FVector VelocityToSet = FVector::ZeroVector;
	if (NewRollVelocity != 0)
	{
		VelocityToSet += RollVelocity * RollAxis;
	}
	if (NewPitchVelocity != 0)
	{
		VelocityToSet += PitchVelocity * PitchAxis;
	}
	if (NewYawVelocity != 0)
	{
		VelocityToSet += YawVelocity * YawAxis;
	}
	SetDampenedAirAngularVelocity(TargetAngularVelocity, VelocityToSet, delta);
}

void USpeedWheeledComponent::SetDampenedAirAngularVelocity(const FVector& TargetAngularVelocity, const FVector& AngVelocityToSetNow, const float& delta)
{
	const FVector VelocityToSet = AngVelocityToSetNow + WheeledPhysicsState.AllowedAngularVelocity;
	SetPhysAngularVelocity(VelocityToSet);
	const FVector TorqueToApply = (TargetAngularVelocity - AngVelocityToSetNow) / delta;
	AddPhysAngularAcceleration(TorqueToApply);
}

void USpeedWheeledComponent::GameplayTick(const float& DeltaTime, const float& SimTime)
{
	HandleInputs(DeltaTime);
}

void USpeedWheeledComponent::HandleInputs(const float& DeltaTime)
{
	HandleSteering(DeltaTime);
	HandleAcceleration();
}

void USpeedWheeledComponent::ApplyAccelKinematicsConstraint(const float& delta)
{
	applyAccelerationConstraint(delta);
	applyAngularAccelerationConstraint(delta);
}

void USpeedWheeledComponent::applyAccelerationConstraint(const float& delta)
{
	auto accel = GetPhysAcceleration();
	auto vel = GetPhysVelocity();
	auto newVel = SSBox::AdvanceVelocity(vel, accel, delta);
	newVel = newVel.GetClampedToMaxSize(GetPhysMaxSpeed());
	auto ActualAccel = (newVel - vel) / delta;
	SetPhysAcceleration(ActualAccel);
}

void USpeedWheeledComponent::applyAngularAccelerationConstraint(const float& delta)
{
	auto angularAccel = GetPhysAngularAcceleration();
	auto angularVel = GetPhysAngularVelocity();
	auto newAngularVel = SSBox::AdvanceAngularVelocity(angularVel, angularAccel, delta);
	newAngularVel = newAngularVel.GetClampedToMaxSize(GetPhysMaxAngularSpeed());
	auto ActualAngularAccel = (newAngularVel - angularVel) / delta;
	SetPhysAngularAcceleration(ActualAngularAccel);
}

void USpeedWheeledComponent::PostGameplayTick(const float& DeltaTime, const float& SimTime)
{
	WheeledUserInput.bCanMove = CanMove();
}


float USpeedWheeledComponent::GetSteeringSpeedCap() const
{
	const float steeringAbs = FMath::Abs(GetPhysSteeringInput());
	constexpr float MaxSteeringLoss = 0.11857f;

	return MaxThrottleSpeed * (1.f - MaxSteeringLoss * steeringAbs);
}

float USpeedWheeledComponent::GetMaxThrottleSpeed() const
{
	return MaxThrottleSpeed;
}

bool USpeedWheeledComponent::IsAffectedByGravity() const
{
	return bEnableGravity;
}

void USpeedWheeledComponent::SetIsAffectedByGravity(bool value)
{
	bEnableGravity = value;
}

void USpeedWheeledComponent::EnableGravity()
{
	bEnableGravity = true;
}


bool USpeedWheeledComponent::HasAuthority() const
{
	return GetWorld()->GetAuthGameMode() != nullptr;
}

bool USpeedWheeledComponent::IsOwningClient() const
{
	return GetOwnerRole() == ROLE_AutonomousProxy;
}

bool USpeedWheeledComponent::IsRemoteClient() const
{
	return !HasAuthority() && !IsOwningClient();
}

FString USpeedWheeledComponent::GetRole() const
{
	if (HasAuthority())
	{
		return TEXT("Server");
	}
	else if (IsOwningClient())
	{
		return TEXT("Client");
	}
	else if (IsRemoteClient())
	{
		return TEXT("Remote");
	}
	return TEXT("Unknown");
}

unsigned char USpeedWheeledComponent::NumWheelsOnGround() const
{
	return static_cast<unsigned char>(WheeledGameState.NbWheelsOnGround);
}

const TArray<TObjectPtr<USWheelSubBody>>& USpeedWheeledComponent::GetWheelSubBodies() const
{
	return WheelSubBodies;
}

void USpeedWheeledComponent::UpdateWheelVisuals()
{
	for (auto& W : WheelSubBodies)
	{
		const SWheelRenderData& R = W->GetRenderData();

		FRotator steerRot(R.SteerRotation);
		FRotator rollRot(R.RollRotation);
		PVehicleOutput->Wheels[W->Idx()].SteeringAngle = steerRot.Yaw;
		PVehicleOutput->Wheels[W->Idx()].AngularPosition = W->GetRollAngle();
	}
}

void USpeedWheeledComponent::RecoverWheelState()
{
	for (auto& W : WheelSubBodies)
	{
		// float rollAngle = WheeledPhysicsState.WheelsAngularPosition[W->Idx()];
		// W->SetRollAngle(rollAngle);
		// float omega = WheeledPhysicsState.WheelsOmega[W->Idx()];
		// W->SetAngularVelocity(omega);
		float lastDisplacement = WheeledPhysicsState.SuspensionLastDisplacement[W->Idx()];
		W->SetLastDisplacement(lastDisplacement);
	}
}

void USpeedWheeledComponent::RegisterWheelState()
{
	for (auto& W : WheelSubBodies)
	{
		WheeledPhysicsState.SuspensionLastDisplacement[W->Idx()] = W->GetLastDisplacement();
	}
}

float USpeedWheeledComponent::GetSuspensionOffset(int WheelIndex)
{
	if (WheelIndex < 0 || WheelIndex >= WheelSubBodies.Num())
	{
		return 0.0f;
	}
	return WheelSubBodies[WheelIndex]->GetSuspensionOffset();
}

bool USpeedWheeledComponent::CanMove() const
{
	return WheeledPhysicsState.nbFramesbeforeCanMove == 0;
}

bool USpeedWheeledComponent::CountdownHasStarted() const
{
	return WheeledPhysicsState.bStartCountdown;
}

void USpeedWheeledComponent::StartConfrontationInSec(const float& Time)
{
	StartConfrontationMulti(Time);
}

void USpeedWheeledComponent::StartConfrontationLocal(const float& TimeSec)
{
	/*
	GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Blue, FString::Printf(TEXT("[%s] Car will be able to move in %ds"),
		*GetRole(), TimeSec));
	}*/
	WheeledPhysicsState.bStartCountdown = true;
	const uint32 Now = NumFrame();
	uint16 BaseNbFrame = FMath::RoundToInt(TimeSec * EngineFPS);
	constexpr uint16 MinNbFrames = 45; // 150ms @300Hz
	WheeledPhysicsState.nbFramesbeforeCanMove = FMath::Max(BaseNbFrame, MinNbFramesBeforeCanMove);
}

void USpeedWheeledComponent::StartConfrontationMulti_Implementation(const float& TimeSec)
{
	StartConfrontationLocal(TimeSec);
}

void USpeedWheeledComponent::StartTestWithVelocity(const FVector& InitialVelocity)
{
	StartTestWithVelocityMulti(InitialVelocity);
}

void USpeedWheeledComponent::StartTestWithVelocityLocal(const FVector& InitialVelocity)
{
	RegisterTestVelocity(InitialVelocity);
}

void USpeedWheeledComponent::StartTestWithVelocityMulti_Implementation(const FVector& InitialVelocity)
{
	StartTestWithVelocityLocal(InitialVelocity);
}

void USpeedWheeledComponent::SetCannotMove()
{
	// GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Blue, FString::Printf(TEXT("[Server] Car cannot Move")));
	SetCannotMoveMulti();
}

void USpeedWheeledComponent::SetCannotMoveLocal()
{
	WheeledPhysicsState.bStartCountdown = false;
	WheeledPhysicsState.nbFramesbeforeCanMove = 1;
}

void USpeedWheeledComponent::SetCannotMoveMulti_Implementation()
{
	// GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Blue, FString::Printf(TEXT("[Client] Car cannot Move")));
	SetCannotMoveLocal();
}

void USpeedWheeledComponent::UpdateWheelOnGroundStates()
{
	WheeledGameState.NbWheelsOnGround = 0;
	for (const auto& W : WheelSubBodies)
	{
		if (W->IsOnGround())
		{
			WheeledGameState.NbWheelsOnGround++;
		}
	}
}

float USpeedWheeledComponent::GetPhysThrottleInput() const
{
	return static_cast<float>(WheeledPhysicalInput.Throttle) / 255.0f;
}

float USpeedWheeledComponent::GetPhysBrakeInput() const
{
	return static_cast<float>(WheeledPhysicalInput.Brake) / 255.0f;
}

float USpeedWheeledComponent::GetPhysSteeringInput() const
{
	return static_cast<float>(WheeledPhysicalInput.Steer) / 127.0f;
}

void USpeedWheeledComponent::SetPhysThrottleInput(const float& Throttle)
{
	auto ClampedThrottle = FMath::Clamp(Throttle, 0.0f, 1.0f);
	WheeledUserInput.Throttle = static_cast<uint8>(FMath::RoundToInt(ClampedThrottle * 255));
}

void USpeedWheeledComponent::SetPhysBrakeInput(const float& Brake)
{
	auto ClampedBrake = FMath::Clamp(Brake, 0.0f, 1.0f);
	WheeledUserInput.Brake = static_cast<uint8>(FMath::RoundToInt(ClampedBrake * 255));
}

void USpeedWheeledComponent::SetPhysSteeringInput(const float& Steering)
{
	auto ClampedSteering = FMath::Clamp(Steering, -1.0f, 1.0f);
	WheeledUserInput.Steer = static_cast<int8>(FMath::RoundToInt(ClampedSteering * 127));
}

TArray<SWheelGroundContact>& USpeedWheeledComponent::GetPendingWheelContacts()
{
	return PendingWheelContacts;
}

void USpeedWheeledComponent::RegisterWheelGroundContact(const SWheelGroundContact& Contact)
{
	PendingWheelContacts.Add(Contact);
}

float USpeedWheeledComponent::GetForwardFriction() const
{
	return GroundFriction.X;
}

float USpeedWheeledComponent::GetSideFriction() const
{
	return GroundFriction.Y;
}

float USpeedWheeledComponent::GetLocalForwardFriction() const
{
	return GetForwardFriction();
}

float USpeedWheeledComponent::GetLocalSideFriction() const
{
	return GetSideFriction();
}

bool USpeedWheeledComponent::LeftFrontWheelIsOnGround() const
{
	return WheelIdxIsOnGround(0);
}

bool USpeedWheeledComponent::RightFrontWheelIsOnGround() const
{
	return WheelIdxIsOnGround(1);
}

bool USpeedWheeledComponent::LeftRearWheelIsOnGround() const
{
	return WheelIdxIsOnGround(2);
}

bool USpeedWheeledComponent::RightRearWheelIsOnGround() const
{
	return WheelIdxIsOnGround(3);
}

bool USpeedWheeledComponent::FrontWheelsAreOnGround() const
{
	return LeftFrontWheelIsOnGround() && RightFrontWheelIsOnGround();
}

bool USpeedWheeledComponent::RearWheelsAreOnGround() const
{
	return LeftRearWheelIsOnGround() && RightRearWheelIsOnGround();
}

bool USpeedWheeledComponent::LeftWheelsAreOnGround() const
{
	return LeftFrontWheelIsOnGround() && LeftRearWheelIsOnGround();
}

bool USpeedWheeledComponent::RightWheelsAreOnGround() const
{
	return RightFrontWheelIsOnGround() && RightRearWheelIsOnGround();
}

void USpeedWheeledComponent::HandlePhysicsAutoRecover()
{

}

void USpeedWheeledComponent::HandleGroundAirState()
{
	if (IsOnTheGround())
		SetGroundState();
	else
		SetAirState();
}

void USpeedWheeledComponent::SetGroundState()
{
	bool WasInAir = !WheeledGameState.bGroundState;
	if (WasInAir)
	{
		GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Blue, FString::Printf(TEXT("[%s] Time in air: %fs"), *GetRole(),
			WheeledGameState.NbFrameSinceInAir * (1.0 / EngineFPS)));
#if !(UE_BUILD_SHIPPING)
		UE_LOG(SpeedPhysicsLog, Log, TEXT("[%s(%s)][GROUND LANDING] At Frame = %d, Time in air: %fs"),
			*GetOwner()->GetName(), *GetRole(), NumFrame(), WheeledGameState.NbFrameSinceInAir * (1.0 / EngineFPS));
#endif
		// PrintKinematics();
	}

	// update ground state
	WheeledGameState.bGroundState = true;
	WheeledPhysicsState.NbFramesSinceGroundContact = 0;
	WheeledGameState.NbFrameSinceInAir = 0;

	SetGroundStatePrv();
}

void USpeedWheeledComponent::SetAirState()
{
	// if car was on ground, store current date
	bool WasOnGround = WheeledGameState.bGroundState;
	if (WasOnGround)
	{
#if !(UE_BUILD_SHIPPING)
		UE_LOG(SpeedPhysicsLog, Log, TEXT("[%s(%s)][AIR TAKEOFF] At Frame = %d, Kinematics: %s"),
			*GetOwner()->GetName(), *GetRole(), NumFrame(), *BasePhysicsState.Kinematic.ToString());
#endif
	}
	// update ground state
	WheeledGameState.bGroundState = false;
	WheeledPhysicsState.NbFramesSinceGroundContact++;
	WheeledGameState.NbFrameSinceInAir++;

	SetAirStatePrv();
}

void USpeedWheeledComponent::SetGroundStatePrv()
{
}

void USpeedWheeledComponent::SetAirStatePrv()
{
}

void USpeedWheeledComponent::HandleAcceleration()
{
	if (!CanMove())
	{
		// GEngine->AddOnScreenDebugMessage(-1, 0.0f, FColor::Red, FString::Printf(TEXT("[%s] Cannot move for the moment!!!!"), *GetRole()));
		return;
	}
	float throttleAccel = 0.0f;
	if (GetPhysThrottleInput() > 0)
	{
		throttleAccel = ComputeThrottleAccel();
	}

	float brakeAccel = 0.0f;
	if (GetPhysBrakeInput() > 0)
	{
		brakeAccel = ComputeBrakeAccel();
	}

	const float TotalAccel = throttleAccel + brakeAccel;
	if (TotalAccel != 0)
	{
		// UE_LOG(SpeedInputLog, Log, TEXT("[%s(%s)] At NumFrame=%d throttle=%f brake=%f TotalAccel = %fcm/s^2"), *GetName(), *GetRole(), NumFrame(),
		//	GetPhysThrottleInput(), GetPhysBrakeInput(), TotalAccel);
		auto GroundNormal = GetNormalFromWheels();
		auto AccelVector = GetPhysForwardVector() - GetPhysForwardVector().Dot(GroundNormal) * GroundNormal;
		AccelVector.Normalize();
		AddPhysAcceleration(AccelVector * TotalAccel);
	}
}

float USpeedWheeledComponent::ComputeThrottleAccel() const
{
	return ComputeAccel(GetPhysThrottleInput(), true);
}

float USpeedWheeledComponent::ComputeBrakeAccel() const
{
	return ComputeAccel(GetPhysBrakeInput(), false);
}

float USpeedWheeledComponent::ComputeAccel(const float& InputValue, bool wantToMoveForward) const
{
	float accel = 0.0f;
	unsigned char nbWheelsOnGround = NumWheelsOnGround();
	float forwardSpeed = GetPhysForwardSpeed();
	float absForwardSpeed = abs(forwardSpeed);
	const float steeringSpeedCap = GetSteeringSpeedCap();
	const float steeringAbs = FMath::Abs(GetPhysSteeringInput());
	constexpr float SteeringFadeExponent = 1.3f;
	float steeringAuthority = FMath::Pow(1.f - steeringAbs, SteeringFadeExponent);

	if (wantToMoveForward == IsPhysGoingForward())
	{
		// Throttle case
		auto forward_damping = GetForwardFriction();
		const float absBaseAccel = InputValue > 0 ? (forward_damping * nbWheelsOnGround / 4) : 0;
		float baseAccel = wantToMoveForward ? absBaseAccel : -absBaseAccel;
		float inputAccel = 0.0;
		const float subSteeringSpeedCap = FMath::Max(0.0, steeringSpeedCap - 10.0);
		const float airAccel = wantToMoveForward ? InputValue * AirForwardThrottle * (4 - nbWheelsOnGround) / 4 :
			-InputValue * AirBackwardThrottle * (4 - nbWheelsOnGround) / 4;
		if (absForwardSpeed <= subSteeringSpeedCap)
		{
			const float absGroundAccel = InputValue * FMath::Lerp(ThrottleAccel, 160, absForwardSpeed / subSteeringSpeedCap) * (nbWheelsOnGround / 4);
			const float groundAccel = wantToMoveForward ? absGroundAccel : -absGroundAccel;
			inputAccel = groundAccel + airAccel;
		}
		else if (absForwardSpeed <= steeringSpeedCap)
		{
			float ratio = (absForwardSpeed - subSteeringSpeedCap) / (steeringSpeedCap - subSteeringSpeedCap);
			const float absGroundAccel = InputValue * FMath::Lerp(160, 0.0, ratio) * (nbWheelsOnGround / 4);
			const float groundAccel = wantToMoveForward ? absGroundAccel : -absGroundAccel;
			inputAccel = groundAccel + airAccel;
		}
		else
		{
			inputAccel = airAccel;
			baseAccel = steeringAuthority * baseAccel;
		}
		accel = baseAccel + inputAccel;
	}
	else
	{
		// Brake case
		auto GroundAccel = (BrakeAccel * nbWheelsOnGround) / 4;
		accel = wantToMoveForward ?
			InputValue * (GroundAccel + AirForwardThrottle * (4 - nbWheelsOnGround) / 4) :
			-InputValue * (GroundAccel + AirBackwardThrottle * (4 - nbWheelsOnGround) / 4);

	}

	return accel;
}

void USpeedWheeledComponent::HandleTimers()
{
	HandleCountdownTimer();
	HandleWheelTimers();
}

void USpeedWheeledComponent::HandleCountdownTimer()
{
	if (!CanMove() && CountdownHasStarted())
	{
		WheeledPhysicsState.nbFramesbeforeCanMove--;
		if (CanMove())
		{
			UE_LOG(SpeedPhysicsLog, Log, TEXT("[%s(%s)][CAN MOVE] At Frame = %d, Kinematics: %s"),
				*GetOwner()->GetName(), *GetRole(), NumFrame(), *BasePhysicsState.Kinematic.ToString());
		}
	}
}

void USpeedWheeledComponent::HandleWheelTimers()
{
	for (auto& wheel : WheelSubBodies)
	{
		wheel->HandleTimers();
	}
}

void USpeedWheeledComponent::HandleSteering(const float& delta)
{
	const float steeringInput = GetPhysSteeringInput(); // [-1,1]
	const float forwardVelocity = GetPhysForwardSpeed();

	if (NoWheelOnGround())
	{
		WheeledPhysicsState.AllowedSideVelocity = FVector::ZeroVector;
		WheeledPhysicsState.AllowedAngularVelocity = FVector::ZeroVector;
		return;
	}

	// Reset accumulators when going straight or very slow or powersliding
	if (NoSteeringAllowed())
	{
		WheeledPhysicsState.AllowedSideVelocity = FMath::VInterpTo(
			WheeledPhysicsState.AllowedSideVelocity, FVector::ZeroVector, delta, 8.f
		);
		WheeledPhysicsState.AllowedAngularVelocity = FMath::VInterpTo(
			WheeledPhysicsState.AllowedAngularVelocity, FVector::ZeroVector, delta, 8.f
		);
		return;
	}
	/*UE_LOG(SpeedPhysicsLog, Log,
		TEXT("[%s(%s)][Steering] NumFrame=%d SteeringInput=%.2f"), *GetOwner()->GetName(), *GetRole(),
		NumFrame(), steeringInput);*/

	// Ground normal coming from wheels
	const FVector UpVector = GetNormalFromWheels();
	const float upDot = UpVector.Z;               // =1 ground, =0 wall
	const float incline = 1.f - upDot;            // =0 ground, =1 wall (good smooth parameter)

	// -------------------------------------
	// Build ground tangent basis (fallback)
	// -------------------------------------
	FVector FwdGround = GetPhysForwardVector()
		- FVector::DotProduct(GetPhysForwardVector(), UpVector) * UpVector;
	FwdGround.Normalize();

	FVector RightGround = FVector::CrossProduct(UpVector, FwdGround);
	RightGround.Normalize();

	// -------------------------------------
	// Desired turning radius
	// -------------------------------------
	const float speed = GetPhysVelocity().Size();
	const float desiredRadius = ComputeSteeringRadius(forwardVelocity, FMath::Abs(steeringInput));

	// Wall radius boost (+20% on full wall)
	const float wallRadiusBoost = 0.20f;
	const float radiusBoost = 1.f; // +wallRadiusBoost * incline;
	const float desiredRadiusWall = desiredRadius * radiusBoost;

	// Target yaw-rate (physics)
	const float absTargetYawRate = speed / desiredRadiusWall; // rad/s
	const float targetYaw = FMath::Sign(steeringInput) * FMath::Sign(forwardVelocity) * absTargetYawRate;
	FVector TargetAngularVelocity = targetYaw * UpVector;

	// -------------------------------------
	// Side slip target
	// -------------------------------------
	const float steeringAbs = FMath::Abs(steeringInput);
	float speedNorm = FMath::Clamp(FMath::Abs(forwardVelocity) / 2300.f, 0.f, 1.f);

	float minSlipRatio = 0.05f;        // low-speed drift
	float maxSlipHighSpeed = 0.10f;    // high-speed drift

	float slipRatio = FMath::Lerp(minSlipRatio, maxSlipHighSpeed, speedNorm * steeringAbs);
	const float sideSpeedTarget = slipRatio * FMath::Abs(forwardVelocity);

	const float steerGain = 1.0f;

	// Wheel force distribution
	const float frontRatio = 0.40f;
	const float rearRatio = 0.10f;

	const float vehicleMass = Mass;
	const float gravity = GravityZ;

	// -------------------------------------
	// Save current accelerations
	// -------------------------------------
	FVector oldAccel = GetPhysAcceleration();
	FVector oldAngAccel = GetPhysAngularAcceleration();

	// -------------------------------------
	// Wheel-by-wheel steering force
	// -------------------------------------
	for (int i = 0; i < 4; i++)
	{
		auto& W = WheelSubBodies[i];
		if (!W->IsOnGround())
			continue;

		const FVector SurfaceNormal = W->GetHitContactNormal();

		// Compute forward projected onto the surface plane (required for walls)
		FVector ForwardProjected =
			GetPhysForwardVector() -
			FVector::DotProduct(GetPhysForwardVector(), SurfaceNormal) * SurfaceNormal;
		ForwardProjected.Normalize();

		// True lateral axis tangent to the surface (ground or wall)
		FVector Right = FVector::CrossProduct(SurfaceNormal, ForwardProjected).GetSafeNormal();

		// Grip scaling: ground -> wall
		float inclineSmooth = FMath::Clamp(incline * 1.5f, 0.f, 1.f);
		float gripSpeedBoost = 1.0f + 1.0f * speedNorm;

		float gripGround = 1.0f * gripSpeedBoost;
		float gripWall = 0.8 * gripGround;       // slightly reduced grip on wall

		// float gripScale = FMath::Lerp(gripGround, gripWall, inclineSmooth);
		float gripScale = gripGround;

		// Normal force proportional to UpVector alignment
		float NormalForce = vehicleMass * FMath::Abs(gravity); // *upDot;
		if (NormalForce < 1.f)
			NormalForce = 1.f;

		const float ratio = (i < 2) ? frontRatio : rearRatio;

		// Ideal turning force
		FVector idealForce = ComputeTurningForce(steeringInput, upDot, forwardVelocity, desiredRadiusWall, Right);
		idealForce *= ratio;

		// Clamp against surface grip
		float maxLat = gripScale * NormalForce;
		float latMag = idealForce.Size();
		if (latMag > maxLat)
		{
			idealForce *= (maxLat / latMag);
			latMag = maxLat;
		}

		// Apply force
		AddPhysForceAtPoint(idealForce, W->SteeringPos());

#if !(UE_BUILD_SHIPPING)
		/*UE_LOG(SpeedPhysicsLog, Log,
			TEXT("Wheel[%d] idealForce=%s"), // Right.idealForce=%.2f"),
			W->Idx(),
			*idealForce.ToString());*/
			// FVector::DotProduct(Right, idealForce));
#endif
	}

	// -------------------------------------
	// Update AllowedSideVelocity / Angular
	// -------------------------------------
	FVector newAccel = GetPhysAcceleration();
	FVector deltaAccel = newAccel - oldAccel;
	WheeledPhysicsState.AllowedSideVelocity += deltaAccel * delta * steerGain;
	const float SteeringTimeConstant = 0.05f; // seconds
	const float alpha = 1.f - FMath::Exp(-delta / SteeringTimeConstant);
	WheeledPhysicsState.AllowedAngularVelocity = FMath::Lerp(WheeledPhysicsState.AllowedAngularVelocity, TargetAngularVelocity, alpha);

	// -------------------------------------
	// Clamp AllowedSideVelocity (surface tangent, not ground frame)
	// -------------------------------------
	FVector RightSurface = RightGround; // fallback for ground

	// On walls, rebuild a correct tangent axis
	if (incline > 0.05f)
	{
		FVector FwdProj = GetPhysForwardVector()
			- FVector::DotProduct(GetPhysForwardVector(), UpVector) * UpVector;
		FwdProj.Normalize();
		RightSurface = FVector::CrossProduct(UpVector, FwdProj).GetSafeNormal();
	}

	float allowedSideScalar = FVector::DotProduct(WheeledPhysicsState.AllowedSideVelocity, RightSurface);
	float sideSign = FMath::Sign(allowedSideScalar);
	float allowedSideAbs = FMath::Min(FMath::Abs(allowedSideScalar), sideSpeedTarget);
	float clampedSide = sideSign * allowedSideAbs;

	WheeledPhysicsState.AllowedSideVelocity += (clampedSide - allowedSideScalar) * RightSurface;

	// -------------------------------------
	// Clamp AllowedAngularVelocity (yaw)
	// -------------------------------------
	float allowedYaw = FVector::DotProduct(WheeledPhysicsState.AllowedAngularVelocity, UpVector);
	float yawSign = FMath::Sign(allowedYaw);
	float allowedYawAbs = FMath::Min(FMath::Abs(allowedYaw), absTargetYawRate);
	float clampedYaw = yawSign * allowedYawAbs;

	WheeledPhysicsState.AllowedAngularVelocity += (clampedYaw - allowedYaw) * UpVector;

	// Override yaw angular velocity along Up
	{
		FVector currentAngVel = GetPhysAngularVelocity();
		FVector correctedAngVel = WheeledPhysicsState.AllowedAngularVelocity - FVector::DotProduct(currentAngVel, UpVector) * UpVector;
		FVector torqueToAdd = (correctedAngVel) / delta;
		AddPhysAngularAcceleration(torqueToAdd);
	}


#if !(UE_BUILD_SHIPPING)
	/*FVector up = GetPhysUpVector();
	float yawRate = FVector::DotProduct(GetPhysAngularVelocity(), up); // rad/s
	if (FMath::Abs(yawRate) > KINDA_SMALL_NUMBER)
	{
		float radius = speed / FMath::Abs(yawRate);
		UE_LOG(SpeedPhysicsLog, Log, TEXT("AllowedYaw=%.3f | RealYaw=%.3f | TargetYaw=%.3f"), allowedYaw, yawRate, targetYaw);
		UE_LOG(SpeedPhysicsLog, Log,
			TEXT("[Steering] NumFrame=%d, ForwardSpeed=%.1f cm/s, upDot=%.2f, SteeringInput=%.2f, YawRate=%.3f rad/s, AllowedSideSpeed=%f, Actual Radius=%.1f cm, Desired Radius=%.1f cm"),
			NumFrame(), forwardVelocity, upDot, steeringInput, yawRate,
			FVector::DotProduct(WheeledPhysicsState.AllowedSideVelocity, RightSurface),
			radius, desiredRadiusWall);
	}*/
#endif
}

bool USpeedWheeledComponent::DisableSuspensionThisFrame() const
{
	return false;
}

bool USpeedWheeledComponent::NoSteeringAllowed() const
{
	return FMath::Abs(GetPhysForwardSpeed()) < 50.f || FMath::IsNearlyZero(GetPhysSteeringInput());
}

float USpeedWheeledComponent::ComputeSteeringRadius(const float& ForwardVelocity, const float& AbsSteeringInput) const
{
	const float MinRadius = 2100.f; // cm, at low speed
	const float MaxRadius = 6000.f; // cm, at high speed
	const float SpeedForMinRadius = 500.f; // cm/s
	const float SpeedForMaxRadius = 2000.f; // cm/s
	float speedFactor = FMath::Clamp((FMath::Abs(ForwardVelocity) - SpeedForMinRadius) / (SpeedForMaxRadius - SpeedForMinRadius), 0.f, 1.f);
	float radius = FMath::Lerp(MinRadius, MaxRadius, speedFactor);
	// Steering input effect (more input -> smaller radius)
	radius /= FMath::Lerp(1.f, 3.f, AbsSteeringInput); // up to 3x smaller radius at full lock
	return radius;
}

FVector USpeedWheeledComponent::ComputeTurningForce(const float& SteeringInput_, const float& UpDot, const float& ForwardVelocity, const float& DesiredRadius, const FVector& WheelRight) const
{
	// 1) If no speed -> no steering force
	if (FMath::Abs(ForwardVelocity) < 50.f)
		return FVector::ZeroVector;

	// 2) Determine turn direction
	float turnSign = FMath::Sign(SteeringInput_) * FMath::Sign(ForwardVelocity);
	// 3) Required centripetal force magnitude
	float v = ForwardVelocity;
	float F = (Mass * v * v) / FMath::Max(DesiredRadius, 10.f);

	// 4) Ensure purely lateral direction
	FVector force = WheelRight * (turnSign * F);

	return force;
}


// ================== NETWORK CORRECTION ==================

void USpeedWheeledComponent::TagStateHistoryProxyRole()
{
	if (HasAuthority() || !SNetworkPhysicsComponent || !WheeledNetworkPhysicsComponent)
		return;

	const TSharedPtr<Chaos::FBaseRewindHistory>& History = SNetworkPhysicsComponent->GetStateHistory_Internal();
	const TSharedPtr<Chaos::FBaseRewindHistory>& WheeledHistory = WheeledNetworkPhysicsComponent->GetStateHistory_Internal();
	if (!History.IsValid() || !WheeledHistory.IsValid())
		return;

	auto* SpeedHistory = static_cast<Chaos::TDataRewindHistory<FNetworkBaseSpeedState>*>(History.Get());
	auto* WheeledSpeedHistory = static_cast<Chaos::TDataRewindHistory<FNetworkWheeledSpeedState>*>(WheeledHistory.Get());
	if (!SpeedHistory || !WheeledSpeedHistory)
		return;

	const bool bAuto = IsOwningClient();
	const int32 Latest = FMath::Min(SpeedHistory->GetLatestFrame(), WheeledSpeedHistory->GetLatestFrame());
	const int32 Start = FMath::Max(0, Latest - 500);

	for (int32 f = Start; f <= Latest; ++f)
	{
		if (!SpeedHistory->EvalData(f))
			continue;
		SpeedHistory->GetCurrentData().bIsAutonomousProxy = bAuto;
		if (!WheeledSpeedHistory->EvalData(f))
			continue;
		WheeledSpeedHistory->GetCurrentData().bIsAutonomousProxy = bAuto;
	}
}


void USpeedWheeledComponent::ApplyNetworkCorrection(const float& DeltaSeconds)
{
	if (HasAuthority() || !SNetworkPhysicsComponent || !WheeledNetworkPhysicsComponent)
	{
		return;
	}

	const TSharedPtr<Chaos::FBaseRewindHistory>& History = SNetworkPhysicsComponent->GetStateHistory_Internal();
	const TSharedPtr<Chaos::FBaseRewindHistory>& WheeledHistory = WheeledNetworkPhysicsComponent->GetStateHistory_Internal();
	if (!History.IsValid() || !WheeledHistory.IsValid())
		return;

	auto* SpeedHistory = static_cast<Chaos::TDataRewindHistory<FNetworkBaseSpeedState>*>(History.Get());
	auto* WheeledSpeedHistory = static_cast<Chaos::TDataRewindHistory<FNetworkWheeledSpeedState>*>(WheeledHistory.Get());
	if (!SpeedHistory || !WheeledSpeedHistory)
		return;

	const int32 CurrentFrame = NumFrame();
	// ----------------------------
	// Find last server state
	// ----------------------------
	const FNetworkBaseSpeedState* LastServerState = nullptr;
	for (int32 f = CurrentFrame; f >= 0; --f)
	{
		if (!SpeedHistory->EvalData(f))
			continue;

		const FNetworkBaseSpeedState& S = SpeedHistory->GetCurrentData();
		if (S.bReceivedData)
		{
			LastServerState = &S;
			break;
		}
	}

	if (!LastServerState)
		return;

	const FNetworkBaseSpeedState& Target = *LastServerState;

	const bool bNewTarget = (Target.ServerFrame != NetCorr_LastServerFrame)
		|| (Target.LocalFrame != NetCorr_LastLocalFrame);

	if (bNewTarget)
	{
		NetCorr_LastServerFrame = Target.ServerFrame;
		NetCorr_LastLocalFrame = Target.LocalFrame;
		NetCorrTickCount = 0;
		NetCorr_BaseNumPredictedFrames = FMath::Max(1, CurrentFrame - Target.LocalFrame);
	}
	else
	{
		NetCorrTickCount++;
	}
	const int32 FramesToCorrect = FMath::Clamp(NetCorr_BaseNumPredictedFrames, 1, 64);

	const int32 CurrentWheeledFrame = WheeledSpeedHistory->GetLatestFrame();
	// ----------------------------
	// Find last server state
	// ----------------------------
	const FNetworkWheeledSpeedState* LastWheeledServerState = nullptr;
	for (int32 f = CurrentFrame; f >= 0; --f)
	{
		if (!WheeledSpeedHistory->EvalData(f))
			continue;

		const FNetworkWheeledSpeedState& S = WheeledSpeedHistory->GetCurrentData();
		if (S.bReceivedData)
		{
			LastWheeledServerState = &S;
			break;
		}
	}

	if (!LastWheeledServerState)
		return;

	const FNetworkWheeledSpeedState& WheeledTarget = *LastWheeledServerState;

	// ----------------------------
	// Find corresponding predicted state
	// ----------------------------

	// Last Client Frame received from Server
	const int32 LocalFrame = Target.LocalFrame;
	FBasePhysicsState PastPredictedState;
	if (!GetPredictedState(LocalFrame, PastPredictedState))
		return;

	FWheeledPhysicsState PastPredictedWheeledState;
	if (!GetPredictedState(LocalFrame, PastPredictedWheeledState))
		return;

	const int32 NumPredictedFrames = CurrentFrame - LocalFrame - NetCorrTickCount;
	if (NetCorrTickCount >= FramesToCorrect || NumPredictedFrames <= 0)
		return;

	// check coutdown mismatch
	if (bNewTarget && (WheeledTarget.WheeledState.bStartCountdown != PastPredictedWheeledState.bStartCountdown))
	{
		// UE_LOG(WheelNetcodeLog, Log, TEXT("[CAN MOVE MISMATCH] Countdown mismatch: Target=%d, PastPred=%d"), WheeledTarget.WheeledState.bStartCountdown, PastPredictedWheeledState.bStartCountdown);
		WheeledPhysicsState.bStartCountdown = WheeledTarget.WheeledState.bStartCountdown;
	}
	/*else
	{
		UE_LOG(WheelNetcodeLog, Log, TEXT("[CAN MOVE] No Countdown mismatch?: Target=%d, PastPred=%d"), WheeledTarget.WheeledState.bStartCountdown, PastPredictedWheeledState.bStartCountdown);
	}*/

	// check nbFramesbeforeCanMove alignment
	if (bNewTarget && (WheeledPhysicsState.nbFramesbeforeCanMove > NumPredictedFrames - 1) && (WheeledTarget.WheeledState.nbFramesbeforeCanMove != PastPredictedWheeledState.nbFramesbeforeCanMove)
		// && (WheeledTarget.WheeledState.nbFramesbeforeCanMove != WheeledPhysicsState.nbFramesbeforeCanMove - NumPredictedFrames - 1)
		&& (WheeledPhysicsState.nbFramesbeforeCanMove != WheeledTarget.WheeledState.nbFramesbeforeCanMove - NumPredictedFrames))
	{
		if (WheeledTarget.WheeledState.nbFramesbeforeCanMove == 0 && WheeledPhysicsState.nbFramesbeforeCanMove != 0)
		{
			WheeledPhysicsState.nbFramesbeforeCanMove = 0;
#if !(UE_BUILD_SHIPPING)
			UE_LOG(WheelNetcodeLog, Log, TEXT("[LATE CAN MOVE MISMATCH] nbFramesbeforeCanMove mismatch: Target=%d, PastPred=%d"), WheeledTarget.WheeledState.nbFramesbeforeCanMove, PastPredictedWheeledState.nbFramesbeforeCanMove);
			UE_LOG(SpeedPhysicsLog, Log, TEXT("[%s(%s)][LATE CAN MOVE] At Frame = %d, Kinematics: %s"),
				*GetOwner()->GetName(), *GetRole(), NumFrame(), *BasePhysicsState.Kinematic.ToString());
#endif
		}
		else
		{
#if !(UE_BUILD_SHIPPING)
			UE_LOG(WheelNetcodeLog, Log, TEXT("[CAN MOVE MISMATCH] nbFramesbeforeCanMove mismatch: Target=%d, PastPred=%d"), WheeledTarget.WheeledState.nbFramesbeforeCanMove, PastPredictedWheeledState.nbFramesbeforeCanMove);
#endif
			WheeledPhysicsState.nbFramesbeforeCanMove = WheeledTarget.WheeledState.nbFramesbeforeCanMove - NumPredictedFrames;
		}
	}
	/*else
	{
		UE_LOG(WheelNetcodeLog, Log, TEXT("[CAN MOVE] No nbFramesbeforeCanMove mismatch?: Target=%d, PastPred=%d"), WheeledTarget.WheeledState.nbFramesbeforeCanMove, PastPredictedWheeledState.nbFramesbeforeCanMove);
	}*/


	// ------------------------------------------------------------
	// Build / refresh correction budget ONCE per received server target
	// ------------------------------------------------------------
	const FVector GroundN = GetNormalFromWheels();
	FVector ErrPos = Target.BaseState.Kinematic.Location - PastPredictedState.Kinematic.Location;
	FVector ErrVel = Target.BaseState.Kinematic.Velocity - PastPredictedState.Kinematic.Velocity;
	FVector ErrAngVel = Target.BaseState.Kinematic.AngularVelocity - PastPredictedState.Kinematic.AngularVelocity;
	FQuat ErrRot = PastPredictedState.Kinematic.Rotation.Inverse() * Target.BaseState.Kinematic.Rotation;
	if (bNewTarget)
	{
		NetCorrAccum.SourceServerFrame = Target.ServerFrame;
		NetCorrAccum.SourceLocalFrame = Target.LocalFrame;

		NetCorrAccum.RemPos = ErrPos;

		// delta rot : Past^-1 * Target
		NetCorrAccum.RemRot = ErrRot;
		NetCorrAccum.RemRot.Normalize();

		// vel budgets : Target - PastPred - "impulse mismatch"
		NetCorrAccum.RemVel = ErrVel;
		NetCorrAccum.RemAngVel = ErrAngVel;

		NetCorrAccum.bActive = true;
	}

	if (!NetCorrAccum.bActive)
		return;

	// ------------------------------------------------------------
	// Contact mode (persistent ground) -> projection on contact plane
	// ------------------------------------------------------------
	const bool bAllWheelsContact = IsOnTheGround();
	FVector WheelN = bAllWheelsContact ? GroundN : FVector::ZeroVector;
	if (bAllWheelsContact && !WheelN.IsNearlyZero())
	{
		const float d = FVector::DotProduct(WheelN, NetCorr_LastContactN);
		if (d > 0.995f) // ~5.7° max
		{
			if (NetCorr_StableNFrames < 255) NetCorr_StableNFrames++;
		}
		else
		{
			NetCorr_StableNFrames = 0;
		}
		NetCorr_LastContactN = WheelN;
	}
	else
	{
		NetCorr_StableNFrames = 0;
	}

	const bool bStableContact = (NetCorr_StableNFrames > 6); // ~6 frames = 20ms @300Hz
	const bool bPersistentContact = bStableContact || (IsOnTheGround() && (WheeledPhysicsState.FramesSinceLastImpact > NumPredictedFrames));
	const FVector ContactN = bPersistentContact ? NetCorr_LastContactN : FVector::ZeroVector;
	const bool bHasN = bPersistentContact && !ContactN.IsNearlyZero(1e-3f);

	// ------------------------------------------------------------
	// Exponential drain at 300 Hz
	// ------------------------------------------------------------
	// tune this
	const float TauPos = IsOwningClient() ? 0.18f : 0.10f;  // s
	const float TauVel = IsOwningClient() ? 0.12f : 0.09f;  // s
	const float TauW = IsOwningClient() ? 0.14f : 0.10f;  // s
	const float TauRot = IsOwningClient() ? 0.18f : 0.12f;  // s

	const float aX = FNetCorrAccum::AlphaFromTau(DeltaSeconds, TauPos);
	const float aV = FNetCorrAccum::AlphaFromTau(DeltaSeconds, TauVel);
	const float aW = FNetCorrAccum::AlphaFromTau(DeltaSeconds, TauW);
	const float aR = FNetCorrAccum::AlphaFromTau(DeltaSeconds, TauRot);

	// ----------------------------
	// Position drain (+ clamp)
	// ----------------------------
	FVector dX = NetCorrAccum.RemPos * aX;
	constexpr float MaxPosCorrPerSec = 600.f; // cm/s
	dX = dX.GetClampedToMaxSize(MaxPosCorrPerSec * DeltaSeconds);
	dX = FNetCorrAccum::RejectNormal(dX, GroundN);

	// ----------------------------
	// Velocity drain (+ clamp)
	// ----------------------------
	FVector dV = NetCorrAccum.RemVel * aV;
	constexpr float MaxVelCorrPerSec = 3600.f; // cm/s²
	dV = dV.GetClampedToMaxSize(MaxVelCorrPerSec * DeltaSeconds);
	// all wheel on ground -> reject correction along the ground normal
	if (bStableContact)
	{
		dV = FNetCorrAccum::RejectNormal(dV, GroundN);
	}
	else if (OneWheelOnGround())
	{
		const FVector vN = FVector::DotProduct(dV, GroundN) * GroundN;
		const FVector vT = dV - vN;

		// keep a fraction of the normal correction
		const float KeepNormal = 0.10f; // 10% of normal allowed
		dV = vT + KeepNormal * vN;

		// Optional : clamp the normal to avoid "pops"
		const float MaxDVNormalPerTick = 15.f * DeltaSeconds; // cm/s per tick
		const float s = FVector::DotProduct(dV, GroundN);
		dV -= (s - FMath::Clamp(s, -MaxDVNormalPerTick, +MaxDVNormalPerTick)) * GroundN;
	}

	// ----------------------------
	// Angular velocity drain (+ clamp)
	// ----------------------------
	FVector dW = NetCorrAccum.RemAngVel * aW;
	constexpr float MaxAngVelCorrPerSec = 9.f; // rad/s per sec -> rad/s per tick
	dW = dW.GetClampedToMaxSize(MaxAngVelCorrPerSec * DeltaSeconds);
	if (bStableContact)
	{
		dW = FNetCorrAccum::RejectNormal(dW, ContactN);
	}

	// ----------------------------
	// Rotation drain (delta quat restant)
	// ----------------------------
	FQuat dR = FQuat::Identity;
	{
		FVector Axis; float Angle;
		NetCorrAccum.RemRot.ToAxisAndAngle(Axis, Angle);
		Axis.Normalize();

		// if persistent contact, yaw-only around the contact normal
		if (bHasN)
		{
			const float s = FVector::DotProduct(Axis, ContactN);
			Axis = (s >= 0.f) ? ContactN : -ContactN;
		}

		const float StepAngle = Angle * aR;
		// clamp en degrés/sec
		constexpr float MaxRotCorrDegPerSec = 180.f;
		const float MaxStepAngle = FMath::DegreesToRadians(MaxRotCorrDegPerSec) * DeltaSeconds;
		const float ClampedStepAngle = FMath::Clamp(StepAngle, -MaxStepAngle, +MaxStepAngle);

		dR = FQuat(Axis, ClampedStepAngle);
		dR.Normalize();
	}

	// ------------------------------------------------------------
	// Apply to current state
	// ------------------------------------------------------------
	// Deadzones (on the quantity applied) -> avoids micro jitter at convergence
	constexpr float PosApplyDeadzone = 0.001f;  // cm
	constexpr float VelApplyDeadzone = 0.002f;   // cm/s
	constexpr float AngVelApplyDeadzone = 0.00005f; // rad/s
	constexpr float RotApplyDeadzoneDeg = 0.001f;
	const int32 kickoff = 0;
	const bool canLog = bNewTarget && Target.ServerFrame > kickoff && CanMove();

	if (dV.Size() > VelApplyDeadzone)
	{
		SetPhysVelocity(GetPhysVelocity() + dV);
		if (canLog)
		{
#if !(UE_BUILD_SHIPPING)
			UE_LOG(WheelNetcodeLog, Log, TEXT("[%s(%s)][CORR] ServerFrame=%d NumPredictedFrames=%d ErrVel=%.2f AppliedVelCorr=%.4fcm/s TargetVel=%fcm/s GroundN.Z=%.2f StableNFrames=%d "
				"NbWheelOnGround=%d"), *GetOwner()->GetName(), *GetRole(), Target.ServerFrame, NumPredictedFrames, NetCorrAccum.RemVel.Size(), dV.Size(),
				Target.BaseState.Kinematic.Velocity.Size(), GroundN.Z, NetCorr_StableNFrames, WheeledGameState.NbWheelsOnGround);
#endif
		}
	}
	if (dW.Size() > AngVelApplyDeadzone)
	{
		SetPhysAngularVelocity(GetPhysAngularVelocity() + dW);
		if (canLog)
		{
#if !(UE_BUILD_SHIPPING)
			UE_LOG(WheelNetcodeLog, Log, TEXT("[%s(%s)][CORR] ServerFrame=%d NumPredictedFrames=%d ErrAngVel=%.4frad/s AppliedAngVelCorr=%.4frad/s TargetAngVel=%.4frad/s"),
				*GetOwner()->GetName(), *GetRole(), Target.ServerFrame, NumPredictedFrames, NetCorrAccum.RemAngVel.Size(), dW.Size(),
				Target.BaseState.Kinematic.AngularVelocity.Size());
#endif
		}
	}
	if (dX.Size() > PosApplyDeadzone)
	{
		SetPhysLocation(GetPhysLocation() + dX);
		if (canLog)
		{
#if !(UE_BUILD_SHIPPING)
			UE_LOG(WheelNetcodeLog, Log, TEXT("[%s(%s)][CORR] ServerFrame=%d NumPredictedFrames=%d ErrPos=%.2f AppliedPosCorr=%.2fcm"),
				*GetOwner()->GetName(), *GetRole(), Target.ServerFrame, NumPredictedFrames, NetCorrAccum.RemPos.Size(), dX.Size());
#endif
		}
	}

	// rotation deadzone
	{
		FVector AxisTmp; float AngleTmp;
		dR.ToAxisAndAngle(AxisTmp, AngleTmp);
		const float AngleDeg = FMath::RadiansToDegrees(FMath::Abs(AngleTmp));
		if (AngleDeg > RotApplyDeadzoneDeg)
		{
			SetPhysRotation((GetPhysRotation() * dR).GetNormalized());
			if (canLog)
			{
#if !(UE_BUILD_SHIPPING)
				UE_LOG(WheelNetcodeLog, Log, TEXT("[%s(%s)][CORR] ServerFrame=%d NumPredictedFrames=%d ErrRot=%.2fdeg AppliedRotCorr=%.2fdeg"),
					*GetOwner()->GetName(), *GetRole(), Target.ServerFrame, NumPredictedFrames,
					FMath::RadiansToDegrees(NetCorrAccum.RemRot.GetAngle()), AngleDeg);
#endif
			}
		}
	}

	// ------------------------------------------------------------
	// Consume budgets
	// ------------------------------------------------------------
	NetCorrAccum.RemPos -= dX;
	NetCorrAccum.RemVel -= dV;
	NetCorrAccum.RemAngVel -= dW;

	// Remove rotation applied from the remaining delta:
	// RemRot := dR^-1 * RemRot
	NetCorrAccum.RemRot = dR.Inverse() * NetCorrAccum.RemRot;
	NetCorrAccum.RemRot.Normalize();

	// ...but when bHasN you already removed normal part via Forgive
	// => ensure RemPos doesn't reintroduce normal via float drift
	if (bHasN)
	{
		NetCorrAccum.RemPos = FNetCorrAccum::RejectNormal(NetCorrAccum.RemPos, ContactN);
	}

	// Stop condition
	const bool bDone =
		NetCorrAccum.RemPos.Size() < 0.005f &&
		NetCorrAccum.RemVel.Size() < 0.005f &&
		NetCorrAccum.RemAngVel.Size() < 0.0001f &&
		NetCorrAccum.RemRot.AngularDistance(FQuat::Identity) < FMath::DegreesToRadians(0.05f);

	if (bDone)
	{
		NetCorrAccum.Reset();
	}
}

void USpeedWheeledComponent::QuantizePhysicalState()
{
	// do not quantize at start
	if (!CanMove())
		return;

	// Quantize kinematic state to reduce precision errors on client and server
	BasePhysicsState.Kinematic.Quantize(GetKinematicStateForFrame(NumFrame() - 1).Rotation);

	// round trip on last suspension length to avoid precision drift on client and server
	for (int WheelIdx = 0; WheelIdx < WheelSubBodies.Num(); WheelIdx++)
	{
		WheeledPhysicsState.SuspensionLastDisplacement[WheelIdx] = WheeledPhysicsState.DequantizeLastSuspensionDisplacement(WheeledPhysicsState.QuantizeLastSuspensionDisplacement(WheelIdx));
	}
}

// ================== Initialization Methods ==================

void USpeedWheeledComponent::InitChaosVehicle()
{
	bThrottleAsBrake = true;
	bSuspensionEnabled = false;
	bWheelFrictionEnabled = false;

	// Note: for faster iteration times, the vehicle setup can be tweaked in the Blueprint instead
	// Set up the chassis
	ChassisHeight = 122.0f;
	DragCoefficient = 0.31f;

	// Setup the wheels
	bLegacyWheelFrictionPosition = true;
	WheelSetups.SetNum(4);

	WheelSetups[0].WheelClass = UChaosVehicleWheel::StaticClass();
	WheelSetups[0].BoneName = FName("Phys_Wheel_FL");
	WheelSetups[0].AdditionalOffset = FVector(0.0f, 0.0f, 0.0f);

	WheelSetups[1].WheelClass = UChaosVehicleWheel::StaticClass();
	WheelSetups[1].BoneName = FName("Phys_Wheel_FR");
	WheelSetups[1].AdditionalOffset = FVector(0.0f, 0.0f, 0.0f);

	WheelSetups[2].WheelClass = UChaosVehicleWheel::StaticClass();
	WheelSetups[2].BoneName = FName("Phys_Wheel_BL");
	WheelSetups[2].AdditionalOffset = FVector(0.0f, 0.0f, 0.0f);

	WheelSetups[3].WheelClass = UChaosVehicleWheel::StaticClass();
	WheelSetups[3].BoneName = FName("Phys_Wheel_BR");
	WheelSetups[3].AdditionalOffset = FVector(0.0f, 0.0f, 0.0f);

	// Setup the engine
	EngineSetup.MaxTorque = 400.0f;
	EngineSetup.MaxRPM = 3875.0f;
	EngineSetup.EngineIdleRPM = 900.0f;
	EngineSetup.EngineBrakeEffect = 0.337f;
	EngineSetup.EngineRevUpMOI = 5.0f;
	EngineSetup.EngineRevDownRate = 600.0f;

	ThrottleInputRate.RiseRate = 6.0f;
	ThrottleInputRate.FallRate = 5.0f;
	ThrottleInputRate.InputCurveFunction = EInputFunctionType::LinearFunction;

	TransmissionSetup.ForwardGearRatios.SetNum(1);
	TransmissionSetup.ForwardGearRatios[0] = 1.0f;

	TransmissionSetup.ReverseGearRatios.SetNum(1);
	TransmissionSetup.ReverseGearRatios[0] = 1.0f;
	DifferentialSetup.DifferentialType = EVehicleDifferential::FrontWheelDrive;
	bMechanicalSimEnabled = false;

	// Steering parameters
	SteeringInputRate.RiseRate = 60.0f;
	SteeringInputRate.FallRate = 40.0f;
	SteeringInputRate.InputCurveFunction = EInputFunctionType::LinearFunction;
	bEnableCenterOfMassOverride = true;
	SteeringSetup.SteeringType = ESteeringType::Ackermann;
}

void USpeedWheeledComponent::InitNetwork()
{
	static const FName SpeedNetSettingsName(TEXT("PC_SpeedNetSettingsName"));
	SNetworkSettings = CreateDefaultSubobject<UNetworkPhysicsSettingsComponent, UNetworkPhysicsSettingsComponent>(SpeedNetSettingsName);
	NetDataAsset = CreateDefaultSubobject<UNetworkPhysicsSettingsDataAsset, UNetworkPhysicsSettingsDataAsset>(TEXT("SpeedSettingsDataAsset"));
	SNetworkSettings->SettingsDataAsset = NetDataAsset;

	static const FName SpeedNetPCName(TEXT("PC_SpeedNetPCName"));
	SNetworkPhysicsComponent = CreateDefaultSubobject<UNetworkPhysicsComponent, UNetworkPhysicsComponent>(SpeedNetPCName);
	SNetworkPhysicsComponent->SetNetAddressable(); // Make DSO components net addressable
	SNetworkPhysicsComponent->SetIsReplicated(true);

	static const FName WheeledNetPCName(TEXT("PC_WheeledNetPCName"));
	WheeledNetworkPhysicsComponent = CreateDefaultSubobject<UNetworkPhysicsComponent, UNetworkPhysicsComponent>(WheeledNetPCName);
	WheeledNetworkPhysicsComponent->SetNetAddressable(); // Make DSO components net addressable
	WheeledNetworkPhysicsComponent->SetIsReplicated(true);

	PredictedBaseFrames.Init(INDEX_NONE, SpeedConstants::PredictedHistorySize);
	PredictedBaseFrames.SetNum(SpeedConstants::PredictedHistorySize);
	PredictedBaseStates.SetNum(SpeedConstants::PredictedHistorySize);
	PredictedWheeledStates.SetNum(SpeedConstants::PredictedHistorySize);
}
