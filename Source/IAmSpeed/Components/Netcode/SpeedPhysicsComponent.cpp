// Fill out your copyright notice in the Description page of Project Settings.


#include "SpeedPhysicsComponent.h"
#include "SpeedMovementComponent.h"
#include "SpeedWheeledComponent.h"

DEFINE_LOG_CATEGORY(SpeedNetcodeLog);

void FNetworkBaseSpeedState::ApplyData(UActorComponent* NetworkComponent) const
{
	if (USpeedMovementComponent* Mover = Cast<USpeedMovementComponent>(NetworkComponent))
	{
	#if !(UE_BUILD_SHIPPING)
		UE_LOG(SpeedNetcodeLog, Warning, TEXT("[BaseSpeed] ApplyData (RESIMULATION?) Triggered for frame = %d"), Mover->NumFrame());
	#endif
		Mover->BasePhysicsState = BaseState;
	}
	else if (USpeedWheeledComponent* WheeledMover = Cast<USpeedWheeledComponent>(NetworkComponent))
	{
#if !(UE_BUILD_SHIPPING)
		UE_LOG(WheelNetcodeLog, Warning, TEXT("[BaseSpeed] ApplyData (RESIMULATION?) Triggered for frame = %d"), WheeledMover->NumFrame());
#endif
		WheeledMover->BasePhysicsState = BaseState;
	}
}


void FNetworkBaseSpeedState::BuildData(const UActorComponent* NetworkComponent)
{
	if (NetworkComponent)
	{
		if (const USpeedMovementComponent* Mover = Cast<const USpeedMovementComponent>(NetworkComponent))
		{
			const int32 CurrentRecordedFrame = FMath::Max(int32(LocalFrame), 0);
			const int32 PreviousRecordedFrame = Speed::SimUtils::GetRecordedFrameFromNetworkLocalFrame(LocalFrame);
			const int32 ComponentFrame = FMath::Max(int32(Mover->NumFrame()), 0);
			const int32 PreviousComponentFrame = FMath::Max(ComponentFrame - 1, 0);
			const auto TryGetBaseState = [this, Mover](const int32 Frame)
			{
				if (Mover->GetBaseState(Frame, BaseState))
				{
					SourceLocalFrame = Frame;
					SourceFramesSinceCanMove = INDEX_NONE;
					return true;
				}
				return false;
			};
			const bool bGotState = TryGetBaseState(CurrentRecordedFrame)
				|| (PreviousRecordedFrame != CurrentRecordedFrame && TryGetBaseState(PreviousRecordedFrame))
				|| (ComponentFrame != CurrentRecordedFrame && ComponentFrame != PreviousRecordedFrame && TryGetBaseState(ComponentFrame))
				|| (PreviousComponentFrame != CurrentRecordedFrame && PreviousComponentFrame != PreviousRecordedFrame && PreviousComponentFrame != ComponentFrame && TryGetBaseState(PreviousComponentFrame));
			if (!bGotState)
			{
				SourceLocalFrame = ComponentFrame;
				SourceFramesSinceCanMove = INDEX_NONE;
				BaseState = Mover->BasePhysicsState;
			}
		}
		else if (const USpeedWheeledComponent* WheeledMover = Cast<const USpeedWheeledComponent>(NetworkComponent))
		{
			const int32 CurrentRecordedFrame = FMath::Max(int32(LocalFrame), 0);
			const int32 PreviousRecordedFrame = Speed::SimUtils::GetRecordedFrameFromNetworkLocalFrame(LocalFrame);
			const int32 ComponentFrame = FMath::Max(int32(WheeledMover->NumFrame()), 0);
			const int32 PreviousComponentFrame = FMath::Max(ComponentFrame - 1, 0);
			const auto TryGetBaseState = [this, WheeledMover](const int32 Frame)
			{
				if (WheeledMover->GetBaseState(Frame, BaseState))
				{
					SourceLocalFrame = Frame;
					const int32 SinceCanMoveFrame = WheeledMover->GetSinceCanMoveFrame();
					SourceFramesSinceCanMove = (SinceCanMoveFrame != INDEX_NONE && Frame >= SinceCanMoveFrame)
						? Frame - SinceCanMoveFrame
						: INDEX_NONE;
					return true;
				}
				return false;
			};
			const bool bGotState = TryGetBaseState(CurrentRecordedFrame)
				|| (PreviousRecordedFrame != CurrentRecordedFrame && TryGetBaseState(PreviousRecordedFrame))
				|| (ComponentFrame != CurrentRecordedFrame && ComponentFrame != PreviousRecordedFrame && TryGetBaseState(ComponentFrame))
				|| (PreviousComponentFrame != CurrentRecordedFrame && PreviousComponentFrame != PreviousRecordedFrame && PreviousComponentFrame != ComponentFrame && TryGetBaseState(PreviousComponentFrame));
			if (!bGotState)
			{
				SourceLocalFrame = ComponentFrame;
				const int32 SinceCanMoveFrame = WheeledMover->GetSinceCanMoveFrame();
				SourceFramesSinceCanMove = (SinceCanMoveFrame != INDEX_NONE && ComponentFrame >= SinceCanMoveFrame)
					? ComponentFrame - SinceCanMoveFrame
					: INDEX_NONE;
				BaseState = WheeledMover->BasePhysicsState;
			}
		}
	}
}


bool FNetworkBaseSpeedState::NetSerialize(FArchive& Ar, UPackageMap* Map, bool& bOutSuccess)
{
	FNetworkPhysicsData::SerializeFrames(Ar);
	Ar << SourceLocalFrame;
	Ar << SourceFramesSinceCanMove;
	Ar << BaseState.Kinematic;
	Ar << BaseState.bIsFrozen;
	bOutSuccess = true;
	return true;
}

int32 FNetworkBaseSpeedState::GetSourceLocalFrame() const
{
	if (SourceLocalFrame == INDEX_NONE)
	{
		return LocalFrame;
	}

	return bReceivedData ? SourceLocalFrame - (ServerFrame - LocalFrame) : SourceLocalFrame;
}

void FNetworkBaseSpeedState::InterpolateData(const FNetworkPhysicsData& MinData, const FNetworkPhysicsData& MaxData)
{
	const FNetworkBaseSpeedState& MinState = static_cast<const FNetworkBaseSpeedState&>(MinData);
	const FNetworkBaseSpeedState& MaxState = static_cast<const FNetworkBaseSpeedState&>(MaxData);

	const int32 MinFrame = static_cast<int32>(MinState.LocalFrame);
	const int32 MaxFrame = static_cast<int32>(MaxState.LocalFrame);
	const int32 ThisFrame = static_cast<int32>(LocalFrame);
	const int32 FrameDelta = MaxFrame - MinFrame;
	const float LerpFactor = FrameDelta != 0 ? float(ThisFrame - MinFrame) / float(FrameDelta) : 1.f;
	const bool bUseMaxState = (MaxFrame == ThisFrame) || LerpFactor >= 0.5f;
	const FNetworkBaseSpeedState& SourceState = bUseMaxState ? MaxState : MinState;

	BaseState = SourceState.BaseState;
	SourceLocalFrame = SourceState.SourceLocalFrame;
	SourceFramesSinceCanMove = SourceState.SourceFramesSinceCanMove;
	bIsAutonomousProxy = SourceState.bIsAutonomousProxy;
	ClientNetSettings = SourceState.ClientNetSettings;
	SimProxyNetSettings = SourceState.SimProxyNetSettings;
}

bool FNetworkBaseSpeedState::CompareData(const FNetworkPhysicsData& PredictedData)
{
	// Startup / kickoff
	constexpr uint32 KICKOFF_GRACE_FRAMES = 30;

	// ==================================================
	// State aliases
	// ==================================================

	const FNetworkBaseSpeedState& C =
		static_cast<const FNetworkBaseSpeedState&>(PredictedData);
	const FNetworkBaseSpeedState& S = *this;

	const SKinematic& KS_C = C.BaseState.Kinematic;
	const SKinematic& KS_S = S.BaseState.Kinematic;

	const FVector DeltaPos = KS_C.Location - KS_S.Location;
	const FQuat DeltaQuat = KS_C.Rotation.Inverse() * KS_S.Rotation;
	const float DeltaAngleDeg = FMath::RadiansToDegrees(DeltaQuat.GetAngle());
	const FVector DeltaVel = KS_C.Velocity - KS_S.Velocity;
	const FVector DeltaAngVel = KS_C.AngularVelocity - KS_S.AngularVelocity;

	// ==================================================
	// 0) Startup / Countdown (NEVER resim)
	// ==================================================

	// Grace window after GO
	if (uint32(C.LocalFrame) <  KICKOFF_GRACE_FRAMES)
	{
		return true;
	}

	// ==================================================
	// 1) Sim Proxy Special case
	// ==================================================
	if (!C.bIsAutonomousProxy)
	{
		return CompareDataSimProxy(C);
	}

	const float MaxDeltaPosAir = ClientNetSettings.PositionResimulationThreshold;
	if (ClientNetSettings.bEnablePositionResimulation && DeltaPos.Size() > MaxDeltaPosAir)
	{
		UE_LOG(SpeedNetcodeLog, Warning, TEXT("[CLIENT][MAX DELTA POS AIR] Frame=%d DeltaP=%fcm"), S.LocalFrame, DeltaPos.Size());
		return false;
	}

	const float MaxDeltaRotAir = ClientNetSettings.RotationResimulationThreshold;
	if (ClientNetSettings.bEnableRotationResimulation && DeltaAngleDeg > MaxDeltaRotAir)
	{
		UE_LOG(SpeedNetcodeLog, Warning, TEXT("[CLIENT][MAX DELTA ROT AIR] Frame=%d DeltaRot=%fdeg"), S.LocalFrame, DeltaAngleDeg);
		return false;
	}

	const float MaxDeltaVelAir = ClientNetSettings.VelocityResimulationThreshold;
	if (ClientNetSettings.bEnableVelocityResimulation && DeltaVel.Size() > MaxDeltaVelAir)
	{
		UE_LOG(SpeedNetcodeLog, Warning, TEXT("[CLIENT][MAX DELTA VEL AIR] Frame=%d DeltaV=%fcm/s"), S.LocalFrame, DeltaVel.Size());
		return false;
	}

	const float MaxDeltaAngVelAir = ClientNetSettings.AngularVelocityResimulationThreshold;
	if (ClientNetSettings.bEnableAngularVelocityResimulation && DeltaAngVel.Size() > MaxDeltaAngVelAir)
	{
		UE_LOG(SpeedNetcodeLog, Warning, TEXT("[CLIENT][MAX DELTA ANG VEL AIR] Frame=%d DeltaAng=%frad/s"), S.LocalFrame, DeltaAngVel.Size());
		return false;
	}

	/*UE_LOG(SpeedNetcodeLog, Log,
		TEXT("[NET][ADMITTED] Frame=%d DeltaP=%.3f DeltaV=%.3f DeltaAng=%.3f"),
		S.LocalFrame,
		DeltaPos.Size(),
		DeltaVel.Size(),
		DeltaAngVel.Size()
	);*/
	return true;
}

bool FNetworkBaseSpeedState::CompareDataSimProxy(const FNetworkBaseSpeedState& C)
{
	const FNetworkBaseSpeedState& S = *this;

	const SKinematic& KS_C = C.BaseState.Kinematic;
	const SKinematic& KS_S = S.BaseState.Kinematic;

	const FVector DeltaPos = KS_C.Location - KS_S.Location;
	const FQuat DeltaQuat = KS_C.Rotation.Inverse() * KS_S.Rotation;
	const FVector DeltaRot = DeltaQuat.Euler();
	const FVector DeltaVel = KS_C.Velocity - KS_S.Velocity;
	const FVector DeltaAngVel = KS_C.AngularVelocity - KS_S.AngularVelocity;

	const float MaxDeltaPosAir = SimProxyNetSettings.PositionResimulationThreshold;
	if (SimProxyNetSettings.bEnablePositionResimulation && DeltaPos.Size() > MaxDeltaPosAir)
	{
		UE_LOG(SpeedNetcodeLog, Warning, TEXT("[SIM PROXY][MAX DELTA POS] Frame=%d DeltaP=%fcm"), S.LocalFrame, DeltaPos.Size());
		return false;
	}

	const float MaxDeltaRotAir = SimProxyNetSettings.RotationResimulationThreshold;
	if (SimProxyNetSettings.bEnableRotationResimulation && DeltaRot.Size() > MaxDeltaRotAir)
	{
		UE_LOG(SpeedNetcodeLog, Warning, TEXT("[SIM PROXY][MAX DELTA ROT] Frame=%d DeltaRot=%fdeg"), S.LocalFrame, DeltaRot.Size());
		return false;
	}

	const float MaxDeltaVelAir = SimProxyNetSettings.VelocityResimulationThreshold;
	if (SimProxyNetSettings.bEnableVelocityResimulation && DeltaVel.Size() > MaxDeltaVelAir)
	{
		UE_LOG(SpeedNetcodeLog, Warning, TEXT("[SIM PROXY][MAX DELTA VEL] Frame=%d DeltaV=%fcm/s"), S.LocalFrame, DeltaVel.Size());
		return false;
	}

	const float MaxDeltaAngVelAir = SimProxyNetSettings.AngularVelocityResimulationThreshold;
	if (SimProxyNetSettings.bEnableAngularVelocityResimulation && DeltaAngVel.Size() > MaxDeltaAngVelAir)
	{
		UE_LOG(SpeedNetcodeLog, Warning, TEXT("[SIM PROXY][MAX DELTA ANG VEL] Frame=%d DeltaAng=%frad/s"), S.LocalFrame, DeltaAngVel.Size());
		return false;
	}
	return true;
}

void FNetworkBaseSpeedState::MergeData(const FNetworkPhysicsData& FromData)
{
	// Perform merge through InterpolateData
	InterpolateData(FromData, *this);
}

bool FNetworkBaseSpeedInputState::NetSerialize(FArchive& Ar, UPackageMap* Map, bool& bOutSuccess)
{
	FNetworkPhysicsData::SerializeFrames(Ar);
	return true;
}
