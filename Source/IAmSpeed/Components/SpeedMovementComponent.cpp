// Fill out your copyright notice in the Description page of Project Settings.


#include "SpeedMovementComponent.h"
#include "IAmSpeed/SubBodies/Configs/SubBodyConfig.h"
#include "PhysicsEngine/PhysicsSettings.h"
#include "IAmSpeed/Base/SpeedConstant.h"
#include "IAmSpeed/SubBodies/SolidSubBody.h"
#include "IAmSpeed/World/SpeedWorldSubsystem.h"

USpeedMovementComponent::USpeedMovementComponent(const FObjectInitializer& ObjectInitializer):
	Super(ObjectInitializer)
{
	SetIsReplicatedByDefault(true);

	InitNetwork();
	PredictedBaseFrames.Init(INDEX_NONE, SpeedConstants::PredictedHistorySize);
	PredictedBaseStates.SetNum(SpeedConstants::PredictedHistorySize);


	SetEngineFPS(Speed::SimUtils::ComputePhysicsFPS(UPhysicsSettings::Get()->AsyncFixedTimeStepSize));
	GravityZ = GetDefault<UPhysicsSettings>()->DefaultGravityZ;
	SubBodies = CreateSubBodies();
	for (USSubBody* SubBody : SubBodies)
	{
		if (USolidSubBody* SolidSubBody = Cast<USolidSubBody>(SubBody))
		{
			SolidSubBodies.AddUnique(SolidSubBody);
		}
	}
}

void USpeedMovementComponent::SetOwner(AActor* NewOwner)
{
	if (NewOwner == nullptr)
	{
		return;
	}
	SetPhysLocation(NewOwner->GetActorLocation());
	SetPhysRotation(NewOwner->GetActorRotation().Quaternion());
	SetPhysVelocity(FVector::ZeroVector);
	SetPhysAngularVelocity(FVector::ZeroVector);
	SetPhysAcceleration(FVector::ZeroVector);
	SetPhysAngularAcceleration(FVector::ZeroVector);
}

void USpeedMovementComponent::OnCreatePhysicsState()
{
	Super::OnCreatePhysicsState();

	UWorld* World = GetWorld();
	if (World->IsGameWorld())
	{
		if (SNetworkPhysicsComponent)
		{
			SNetworkPhysicsComponent->CreateDataHistory<FPhysicsSpeedTraits>(this);
		}
	}
}

void USpeedMovementComponent::OnDestroyPhysicsState()
{
	Super::OnDestroyPhysicsState();
	if (SNetworkPhysicsComponent)
	{
		SNetworkPhysicsComponent->RemoveDataHistory();
	}
}

TArray<USSubBody*> USpeedMovementComponent::CreateSubBodies()
{
	return TArray<USSubBody*>();
}

unsigned int USpeedMovementComponent::NumFrame() const
{
	return BaseGameState.NumFrame;
}

float USpeedMovementComponent::GetPhysMass() const
{
	return PhysMass;
}

FVector USpeedMovementComponent::GetPhysCOM() const
{
	return GetPhysLocation() + CenterOfMass;
}

const TArray<USSubBody*>& USpeedMovementComponent::GetSubBodies() const
{
	return SubBodies;
}

SubBodyConfig USpeedMovementComponent::GetSubBodyConfig(const USSubBody& SubBody) const
{
	return SubBodyConfig();
}

const SKinematic& USpeedMovementComponent::GetKinematicsOfSubBody(const USSubBody& SubBody, const unsigned int& LocalFrame) const
{
	const int32 Idx = LocalFrame % SpeedConstants::PredictedHistorySize;
	if (PredictedBaseFrames[Idx] == LocalFrame)
	{
		return PredictedBaseStates[Idx].Kinematic;
	}
	return BasePhysicsState.Kinematic;
}

FMatrix USpeedMovementComponent::ComputeWorldInvInertiaTensor() const
{
	return FMatrix();
}

FMatrix USpeedMovementComponent::ComputeWorldInvInertiaTensorOfSubBody(const USSubBody& SubBody) const
{
	return FMatrix();
}

const SKinematic& USpeedMovementComponent::GetKinematicState() const
{
	return BasePhysicsState.Kinematic;
}

const SKinematic& USpeedMovementComponent::GetKinematicStateForFrame(const unsigned int& LocalFrame) const
{
	const int32 Idx = LocalFrame % SpeedConstants::PredictedHistorySize;
	if (PredictedBaseFrames[Idx] == LocalFrame)
	{
		return PredictedBaseStates[Idx].Kinematic;
	}
	return BasePhysicsState.Kinematic;
}

float USpeedMovementComponent::GetPhysMaxSpeed() const
{
	return PhysMaxSpeed;
}

float USpeedMovementComponent::GetPhysMaxAngularSpeed() const
{
	return PhysMaxAngularSpeed;
}

void USpeedMovementComponent::PostPhysicsUpdatePrv(const float& delta)
{
	ApplyNetworkCorrection(delta);
	QuantizePhysicalState();
	RecordPredictedState();
}

void USpeedMovementComponent::SetIsUpsideDown(bool bUpsideDown)
{
}

bool USpeedMovementComponent::IsUpsideDown() const
{
	return false;
}

bool USpeedMovementComponent::IsInAutoRecover() const
{
	return false;
}

bool USpeedMovementComponent::IsSubBodyInAutoRecoverMode() const
{
	return false;
}

void USpeedMovementComponent::RcvImpactOnSubBody(const USSubBody& SubBody, const FVector& Location)
{
}

void USpeedMovementComponent::SetKinematicState(const SKinematic& NewKinematicState)
{
	BasePhysicsState.Kinematic = NewKinematicState;
}

void USpeedMovementComponent::RegisterTestVelocity(const FVector& InitialVelocity)
{
	BaseGameState.TestVelocity = InitialVelocity;
}

void USpeedMovementComponent::ApplyTestVelocity()
{
	if (BaseGameState.TestVelocity.IsZero())
	{
		return;
	}
	SetPhysVelocity(BaseGameState.TestVelocity);
	BaseGameState.TestVelocity = FVector::ZeroVector;
}

void USpeedMovementComponent::AsyncPhysicsTickComponent(float DeltaTime, float SimTime)
{
	PhysicsTick(DeltaTime, SimTime);
}

void USpeedMovementComponent::SetEngineFPS(const unsigned int& FPS)
{
	EngineFPS = FPS;
}

void USpeedMovementComponent::PhysicsTick(const float& DeltaTime, const float& SimTime)
{
	// Update NumFrame at the beginning of the tick so that it can be used in the rest of the tick functions
	UpdateNumFrame(SimTime);
	TagStateHistoryProxyRole();

	// Handle forces that should be applied before the gameplay tick (e.g. gravity, damping, rest force)
	HandleGravity();
	PreGameplayTick(DeltaTime, SimTime);
	HandleRestForce(); // Handle rest force before gameplay tick so that the component can be at rest at the beginning of the tick if it should be

	// Handle gameplay forces (e.g. from player input or AI)
	GameplayTick(DeltaTime, SimTime);

	// Apply velocity that was set for a test
	ApplyTestVelocity();

	// Handle any necessary updates after the gameplay tick (e.g. record predicted state for the current frame, apply network corrections etc.)
	ApplyAccelKinematicsConstraint(DeltaTime);
	PostGameplayTick(DeltaTime, SimTime);
}

void USpeedMovementComponent::UpdateNumFrame(const float& SimTime)
{
	BaseGameState.NumFrame = Speed::SimUtils::ComputeNumFrameFromSimTime(EngineFPS, SimTime);
}

void USpeedMovementComponent::RecordPredictedState()
{
	const int32 LF = NumFrame();
	const int32 Idx = LF % SpeedConstants::PredictedHistorySize;

	PredictedBaseFrames[Idx] = LF;
	PredictedBaseStates[Idx] = BasePhysicsState;
}

bool USpeedMovementComponent::GetPredictedState(const int32& LocalFrame, FBasePhysicsState& OutState) const
{
	const int32 Idx = LocalFrame % SpeedConstants::PredictedHistorySize;
	if (PredictedBaseFrames[Idx] == LocalFrame)
	{
		OutState = PredictedBaseStates[Idx];
		return true;
	}
	return false;
}

void USpeedMovementComponent::SetSubBodies(const TArray<USSubBody*>& NewSubBodies)
{
	SubBodies = NewSubBodies;
}

void USpeedMovementComponent::HandleGravity()
{
	if (bEnableGravity)
	{
		AddPhysAcceleration(FVector(0.0f, 0.0f, GravityZ));
	}
}

void USpeedMovementComponent::HandleDamping(const float& delta)
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

void USpeedMovementComponent::HandleRestForce()
{
	for (USolidSubBody* SubBody : SolidSubBodies)
	{
		if (!SubBody)
			continue;
		if (!SubBody->HasToApplyRestForce() || !SubBody->HasHit())
			continue;
		FVector N = SubBody->GetHit().ImpactNormal; // surface normal
		FVector accel = GetPhysAcceleration();
		float aN = FVector::DotProduct(accel, N); // normal component of acceleration
		if (aN < 0.f) // only if moving into surface
		{
			// Apply an acceleration opposite to the normal component of acceleration to quickly reduce it to zero
			FVector aN_in = -aN * N;
			AddPhysAcceleration(aN_in);
		}
	}
}

void USpeedMovementComponent::PreGameplayTick(const float& DeltaTime, const float& SimTime)
{
	HandleDamping(DeltaTime);
}

void USpeedMovementComponent::GameplayTick(const float& DeltaTime, const float& SimTime)
{

}

void USpeedMovementComponent::ApplyAccelKinematicsConstraint(const float& delta)
{
	applyAccelerationConstraint(delta);
	applyAngularAccelerationConstraint(delta);
}

void USpeedMovementComponent::applyAccelerationConstraint(const float& delta)
{
	auto accel = GetPhysAcceleration();
	auto vel = GetPhysVelocity();
	auto newVel = SSBox::AdvanceVelocity(vel, accel, delta);
	newVel = newVel.GetClampedToMaxSize(GetPhysMaxSpeed());
	auto ActualAccel = (newVel - vel) / delta;
	SetPhysAcceleration(ActualAccel);
}

void USpeedMovementComponent::applyAngularAccelerationConstraint(const float& delta)
{
	auto angularAccel = GetPhysAngularAcceleration();
	auto angularVel = GetPhysAngularVelocity();
	auto newAngularVel = SSBox::AdvanceAngularVelocity(angularVel, angularAccel, delta);
	newAngularVel = newAngularVel.GetClampedToMaxSize(GetPhysMaxAngularSpeed());
	auto ActualAngularAccel = (newAngularVel - angularVel) / delta;
	SetPhysAngularAcceleration(ActualAngularAccel);
}

void USpeedMovementComponent::PostGameplayTick(const float& DeltaTime, const float& SimTime)
{
}

bool USpeedMovementComponent::IsAffectedByGravity() const
{
	return bEnableGravity;
}

void USpeedMovementComponent::SetIsAffectedByGravity(bool value)
{
	bEnableGravity = value;
}

void USpeedMovementComponent::EnableGravity()
{
	bEnableGravity = true;
}

bool USpeedMovementComponent::HasAuthority() const
{
	return GetWorld()->GetAuthGameMode() != nullptr;
}

bool USpeedMovementComponent::IsOwningClient() const
{
	return GetOwnerRole() == ROLE_AutonomousProxy;
}

bool USpeedMovementComponent::IsRemoteClient() const
{
	return !HasAuthority() && !IsOwningClient();
}

FString USpeedMovementComponent::GetRole() const
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

void USpeedMovementComponent::BeginPlay()
{
	Super::BeginPlay();

	SetAsyncPhysicsTickEnabled(true);

	// Register in SpeedWorldSubsystem
	if (UWorld* World = GetWorld())
	{
		if (USpeedWorldSubsystem* SpeedWorldSubsystem = World->GetSubsystem<USpeedWorldSubsystem>())
		{
			// UE_LOG(LogTemp, Warning, TEXT("[%s(%s)][BeginPlay] RESPAWN"), *GetName(), *GetOwner()->GetName());
			SpeedWorldSubsystem->RegisterSpeedComponent(this);
		}
	}
}

void USpeedMovementComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
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
}

void USpeedMovementComponent::StartTestWithVelocity(const FVector& InitialVelocity)
{
	StartTestWithVelocityMulti(InitialVelocity);
}

void USpeedMovementComponent::StartTestWithVelocityLocal(const FVector& InitialVelocity)
{
	RegisterTestVelocity(InitialVelocity);
}

void USpeedMovementComponent::StartTestWithVelocityMulti_Implementation(const FVector& InitialVelocity)
{
	StartTestWithVelocityLocal(InitialVelocity);
}


void USpeedMovementComponent::ApplyNetworkCorrection(const float& DeltaSeconds)
{
	if (HasAuthority() || !SNetworkPhysicsComponent)
	{
		return;
	}

	const TSharedPtr<Chaos::FBaseRewindHistory>& History = SNetworkPhysicsComponent->GetStateHistory_Internal();
	if (!History.IsValid())
		return;

	auto* SpeedHistory = static_cast<Chaos::TDataRewindHistory<FNetworkBaseSpeedState>*>(History.Get());
	if (!SpeedHistory)
		return;

	const int32 CurrentFrame = SpeedHistory->GetLatestFrame();
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

	// ----------------------------
	// Find corresponding predicted state (at Target.LocalFrame)
	// ----------------------------
	const int32 LocalFrame = Target.LocalFrame;

	FBasePhysicsState PastPredictedState;
	if (!GetPredictedState(LocalFrame, PastPredictedState))
	{
		// Too old / overwritten in our local ring, nothing to do
		return;
	}

	const int32 NumPredictedFrames = CurrentFrame - LocalFrame - NetCorrTickCount;
	if (NetCorrTickCount >= FramesToCorrect || NumPredictedFrames <= 0)
	{
		return;
	}

	const SKinematic& TargetKS = Target.BaseState.Kinematic;

	// ------------------------------------------------------------
	// (A) Historical error at the SAME frame:
	//   Err0 = Server(LocalFrame) - Predicted(LocalFrame)
	// ------------------------------------------------------------
	const FVector ErrX0 = TargetKS.Location - PastPredictedState.Kinematic.Location;
	const FVector ErrV0 = TargetKS.Velocity - PastPredictedState.Kinematic.Velocity;
	const FVector ErrW0 = TargetKS.AngularVelocity - PastPredictedState.Kinematic.AngularVelocity;

	const FQuat   ErrQ0 = PastPredictedState.Kinematic.Rotation.Inverse() * TargetKS.Rotation;
	const FVector ErrEuler0 = ErrQ0.Euler(); // deg

	// ------------------------------------------------------------
	// (1) HARD MISMATCH RELAX (NOT the old "impulse relax"):
	// If the historical mismatch is huge (typically real hit resolved
	// slightly differently client/server), temporarily weaken velocity
	// and angvel correction to avoid visible jerk.
	// ------------------------------------------------------------
	const float ErrV = ErrV0.Size();
	const float ErrW = ErrW0.Size();

	// ------------------------------------------------------------
	// Correction multipliers (base)
	// ------------------------------------------------------------
	float VelMult = VelStabilityMultiplier;
	float AngMult = AngVelStabilityMultiplier;

	const float alphaX = PosStabilityMultiplier / FramesToCorrect;
	const float alphaV = VelMult / FramesToCorrect;
	const float alphaR = RotStabilityMultiplier / FramesToCorrect;
	const float alphaW = AngMult / FramesToCorrect;

	// ------------------------------------------------------------
	// 2) General correction (not carried)
	// ------------------------------------------------------------
	// Current state (present)
	const FVector CurX = GetPhysLocation();
	const FQuat   CurR = GetPhysRotation();
	const FVector CurV = GetPhysVelocity();
	const FVector CurW = GetPhysAngularVelocity();
	{
		const FVector CorrX = (ErrX0 * alphaX).GetClampedToMaxSize(MaxPosCorrectionPerSecond * DeltaSeconds);
		const FVector CorrV = (ErrV0 * alphaV).GetClampedToMaxSize(MaxVelCorrectionPerSecond * DeltaSeconds);
		const FVector CorrW = (ErrW0 * alphaW).GetClampedToMaxSize(MaxAngVelCorrectionPerSecond * DeltaSeconds);

		const FVector CorrEuler = (ErrEuler0 * alphaR).GetClampedToMaxSize(MaxRotCorrectionPerSecond * DeltaSeconds);
		const FQuat   CorrQuat = FQuat::MakeFromEuler(CorrEuler);

		if (ErrV0.Size() > VelCorrDeadzone)
		{
			SetPhysVelocity(GetPhysVelocity() + CorrV);
			if (bNewTarget)
			{
#if !(UE_BUILD_SHIPPING)
				UE_LOG(SpeedNetcodeLog, Log,
					TEXT("[%s(%s)][CORR] ServerFrame=%d NumPredictedFrames=%d LinVelDiff0=%.2f Corr=%.2f TargetVelNow=%fcm/s"),
					*GetOwner()->GetName(), *GetRole(), Target.ServerFrame, NumPredictedFrames, ErrV0.Size(), CorrV.Size(), GetPhysVelocity().Size());
#endif
			}
		}

		if (ErrX0.Size() > PosCorrDeadzone)
		{
			SetPhysLocation(GetPhysLocation() + CorrX);
			if (bNewTarget)
			{
#if !(UE_BUILD_SHIPPING)
				UE_LOG(SpeedNetcodeLog, Log,
					TEXT("[%s(%s)][CORR] ServerFrame=%d NumPredictedFrames=%d PosDiff0=%.2fcm Corr=%.2fcm TargetPosNow=%s"),
					*GetOwner()->GetName(), *GetRole(), Target.ServerFrame, NumPredictedFrames, ErrX0.Size(), CorrX.Size(), *GetPhysLocation().ToString());
#endif
			}
		}

		if (ErrW0.Size() > AngVelCorrDeadzone)
		{
			SetPhysAngularVelocity(GetPhysAngularVelocity() + CorrW);
			if (bNewTarget)
			{
#if !(UE_BUILD_SHIPPING)
				UE_LOG(SpeedNetcodeLog, Log,
					TEXT("[%s(%s)][CORR] ServerFrame=%d NumPredictedFrames=%d AngVelDiff0=%.4f Corr=%.4f TargetAngVelNow=%.4frad/s"),
					*GetOwner()->GetName(), *GetRole(), Target.ServerFrame, NumPredictedFrames, ErrW0.Size(), CorrW.Size(), GetPhysAngularVelocity().Size());
#endif
			}
		}

		if (ErrEuler0.Size() > RotCorrDeadzone)
		{
			SetPhysRotation((GetPhysRotation() * CorrQuat).GetNormalized());
			if (bNewTarget)
			{
#if !(UE_BUILD_SHIPPING)
				UE_LOG(SpeedNetcodeLog, Log,
					TEXT("[%s(%s)][CORR] ServerFrame=%d NumPredictedFrames=%d RotDiff0=%.2fdeg Corr=%.2fdeg"),
					*GetOwner()->GetName(), *GetRole(), Target.ServerFrame, NumPredictedFrames, ErrEuler0.Size(), CorrEuler.Size());
#endif
			}
		}
	}
}

void USpeedMovementComponent::QuantizePhysicalState()
{
	// Quantize kinematic state to reduce precision errors on client and server
	BasePhysicsState.Kinematic.Quantize(GetKinematicStateForFrame(NumFrame() - 1).Rotation);
}

void USpeedMovementComponent::TagStateHistoryProxyRole()
{
	if (HasAuthority())
		return;

	const TSharedPtr<Chaos::FBaseRewindHistory>& History = SNetworkPhysicsComponent->GetStateHistory_Internal();
	if (!History.IsValid())
		return;

	auto* SpeedHistory = static_cast<Chaos::TDataRewindHistory<FNetworkBaseSpeedState>*>(History.Get());
	if (!SpeedHistory)
		return;

	const bool bAuto = IsOwningClient();
	const int32 Latest = SpeedHistory->GetLatestFrame();
	const int32 Start = FMath::Max(0, Latest - 500);

	for (int32 f = Start; f <= Latest; ++f)
	{
		if (!SpeedHistory->EvalData(f))
			continue;
		SpeedHistory->GetCurrentData().bIsAutonomousProxy = bAuto;
	}
}

void USpeedMovementComponent::InitNetwork()
{
	static const FName SpeedNetPCName(TEXT("PC_SpeedNetPCName"));
	SNetworkPhysicsComponent = CreateDefaultSubobject<UNetworkPhysicsComponent, UNetworkPhysicsComponent>(SpeedNetPCName);
	SNetworkPhysicsComponent->SetNetAddressable(); // Make DSO components net addressable
	SNetworkPhysicsComponent->SetIsReplicated(true);

	static const FName SpeedNetSettingsName(TEXT("PC_SpeedNetSettingsName"));
	SNetworkSettings = CreateDefaultSubobject<UNetworkPhysicsSettingsComponent, UNetworkPhysicsSettingsComponent>(SpeedNetSettingsName);
	NetDataAsset = CreateDefaultSubobject<UNetworkPhysicsSettingsDataAsset, UNetworkPhysicsSettingsDataAsset>(TEXT("NetPhysicsSettingsDataAsset"));
	SNetworkSettings->SettingsDataAsset = NetDataAsset;
}