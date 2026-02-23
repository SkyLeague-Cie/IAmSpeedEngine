// Fill out your copyright notice in the Description page of Project Settings.


#include "IAmSpeed/Components/Netcode/SpeedWheeledPhysicsComponent.h"
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "IAmSpeed/Base/SpeedConstant.h"


void FNetworkWheeledSpeedState::ApplyData(UActorComponent* NetworkComponent) const
{
	if (USpeedWheeledComponent* Mover = Cast<USpeedWheeledComponent>(NetworkComponent))
	{
#if !(UE_BUILD_SHIPPING)
		UE_LOG(WheelNetcodeLog, Warning, TEXT("[WheeledSpeed] ApplyData (RESIMULATION?) Triggered for frame = %d"), Mover->NumFrame());
#endif
		Mover->WheeledPhysicsState = WheeledState;
	}
}

void FNetworkWheeledSpeedState::BuildData(const UActorComponent* NetworkComponent)
{
	if (NetworkComponent)
	{
		if (const USpeedWheeledComponent* Mover = Cast<const USpeedWheeledComponent>(NetworkComponent))
		{
			WheeledState = Mover->WheeledPhysicsState;
		}
	}
}

bool FNetworkWheeledSpeedState::NetSerialize(FArchive& Ar, UPackageMap* Map, bool& bOutSuccess)
{
	FNetworkPhysicsData::SerializeFrames(Ar);
	Ar << WheeledState.nbFramesbeforeCanMove;
	Ar << WheeledState.AllowedSideVelocity;
	Ar << WheeledState.AllowedAngularVelocity;
	Ar << WheeledState.NbFramesSinceGroundContact;
	Ar << WheeledState.FramesSinceLastImpact;

	uint8 Flags = 0;
	if (Ar.IsSaving())
	{
		Flags |= (WheeledState.bStartCountdown ? 1 << 0 : 0);
	}
	Ar << Flags;
	if (Ar.IsLoading())
	{
		WheeledState.bStartCountdown = (Flags & (1 << 0)) != 0;
	}

	for (int32 i = 0; i < 4; ++i)
	{
		// Suspension displacement (gameplay)
		if (Ar.IsSaving())
		{
			int16 qSusp = WheeledState.QuantizeLastSuspensionDisplacement(i);
			Ar << qSusp;
		}
		else
		{
			int16 qSusp; Ar << qSusp;
			WheeledState.SuspensionLastDisplacement[i] = WheeledState.DequantizeLastSuspensionDisplacement(qSusp);
		}
	}

	bOutSuccess = true;
	return true;
}

void FNetworkWheeledSpeedState::InterpolateData(const FNetworkPhysicsData& MinData, const FNetworkPhysicsData& MaxData)
{
	const FNetworkWheeledSpeedState& MinState = static_cast<const FNetworkWheeledSpeedState&>(MinData);
	const FNetworkWheeledSpeedState& MaxState = static_cast<const FNetworkWheeledSpeedState&>(MaxData);

	const float LerpFactor = MaxState.LocalFrame == LocalFrame
		? 1.0f / (MaxState.LocalFrame - MinState.LocalFrame + 1) // Merge from min into max
		: (LocalFrame - MinState.LocalFrame) / (MaxState.LocalFrame - MinState.LocalFrame); // Interpolate from min to max

	WheeledState.bStartCountdown = LerpFactor < 0.5 ? MinState.WheeledState.bStartCountdown : MaxState.WheeledState.bStartCountdown;
	WheeledState.nbFramesbeforeCanMove = Speed::Interpolate(MinState.WheeledState.nbFramesbeforeCanMove, MaxState.WheeledState.nbFramesbeforeCanMove, LerpFactor);
	WheeledState.AllowedSideVelocity = FMath::Lerp(MinState.WheeledState.AllowedSideVelocity, MaxState.WheeledState.AllowedSideVelocity, LerpFactor);
	WheeledState.AllowedAngularVelocity = FMath::Lerp(MinState.WheeledState.AllowedAngularVelocity, MaxState.WheeledState.AllowedAngularVelocity, LerpFactor);
	WheeledState.NbFramesSinceGroundContact = Speed::Interpolate(MinState.WheeledState.NbFramesSinceGroundContact, MaxState.WheeledState.NbFramesSinceGroundContact, LerpFactor);

	int32 NumWheels = 4;
	for (int32 WheelIdx = 0; WheelIdx < NumWheels; ++WheelIdx)
	{
		WheeledState.SuspensionLastDisplacement[WheelIdx] = FMath::Lerp(MinState.WheeledState.SuspensionLastDisplacement[WheelIdx], MaxState.WheeledState.SuspensionLastDisplacement[WheelIdx], LerpFactor);
	}
}

bool FNetworkWheeledSpeedState::CompareData(const FNetworkPhysicsData& PredictedData)
{
	const FNetworkWheeledSpeedState& PredictedState = static_cast<const FNetworkWheeledSpeedState&>(PredictedData);

	return true;
}

void FNetworkWheeledSpeedState::MergeData(const FNetworkPhysicsData& FromData)
{
	// Perform merge through InterpolateData
	InterpolateData(FromData, *this);
}


int16 FWheeledPhysicsState::QuantizeLastSuspensionDisplacement(int wheelIndex) const
{
	return FWheeledPhysicsState::QuantizeSigned(SuspensionLastDisplacement[wheelIndex], SpeedConstants::SuspScale);
}

float FWheeledPhysicsState::DequantizeLastSuspensionDisplacement(int16 q) const
{
	return FWheeledPhysicsState::DequantizeSigned(q, SpeedConstants::SuspScale);
}

void FNetworkWheeledSpeedInputState::ApplyData(UActorComponent* NetworkComponent) const
{
	if (USpeedWheeledComponent* Mover = Cast<USpeedWheeledComponent>(NetworkComponent))
	{
		Mover->WheeledUserInput = WheeledInput;
	}
}

void FNetworkWheeledSpeedInputState::BuildData(const UActorComponent* NetworkComponent)
{
	if (NetworkComponent)
	{
		if (const USpeedWheeledComponent* Mover = Cast<const USpeedWheeledComponent>(NetworkComponent))
		{
			WheeledInput = Mover->WheeledUserInput;
			ClientFrame = Mover->NumFrame();
			bIsAutonomousProxy = Mover->GetOwnerRole() == ROLE_AutonomousProxy;
		}
	}
}

bool FNetworkWheeledSpeedInputState::NetSerialize(FArchive& Ar, UPackageMap* Map, bool& bOutSuccess)
{
	FNetworkPhysicsData::SerializeFrames(Ar);
	Ar << WheeledInput.Throttle;
	Ar << WheeledInput.Brake;
	Ar << WheeledInput.Steer;
	Ar << ClientFrame;
	Ar << bIsAutonomousProxy;
	bOutSuccess = true;
	return true;
}

void FNetworkWheeledSpeedInputState::InterpolateData(const FNetworkPhysicsData& MinData, const FNetworkPhysicsData& MaxData)
{
	const FNetworkWheeledSpeedInputState& MinState = static_cast<const FNetworkWheeledSpeedInputState&>(MinData);
	const FNetworkWheeledSpeedInputState& MaxState = static_cast<const FNetworkWheeledSpeedInputState&>(MaxData);
	const float LerpFactor = MaxState.LocalFrame == LocalFrame
		? 1.0f / (MaxState.LocalFrame - MinState.LocalFrame + 1) // Merge from min into max
		: (LocalFrame - MinState.LocalFrame) / (MaxState.LocalFrame - MinState.LocalFrame); // Interpolate from min to max
	/*UE_LOG(WheelNetcodeLog, Log, TEXT("[WheeledSpeed] InterpolateData Triggered for frame = %d -> ")
		TEXT("Min(Throttle=%u Brake=%u Steer=%d) ")
		TEXT("Max(Throttle=%u Brake=%u Steer=%d)"), LocalFrame,
		(uint8)MinState.WheeledInput.Throttle, (uint8)MinState.WheeledInput.Brake, (int8)MinState.WheeledInput.Steer,
		(uint8)MaxState.WheeledInput.Throttle, (uint8)MaxState.WheeledInput.Brake, (int8)MaxState.WheeledInput.Steer);*/
	WheeledInput.Throttle = Speed::Interpolate(MinState.WheeledInput.Throttle, MaxState.WheeledInput.Throttle, LerpFactor);
	WheeledInput.Brake = Speed::Interpolate(MinState.WheeledInput.Brake, MaxState.WheeledInput.Brake, LerpFactor);
	WheeledInput.Steer = Speed::Interpolate(MinState.WheeledInput.Steer, MaxState.WheeledInput.Steer, LerpFactor);
}

bool FNetworkWheeledSpeedInputState::CompareData(const FNetworkPhysicsData& PredictedData)
{
	const FNetworkWheeledSpeedInputState& C = static_cast<const FNetworkWheeledSpeedInputState&>(PredictedData); // Client predicted
	const FNetworkWheeledSpeedInputState& S = *this;                                                  // Server (authoritative for that frame)

	if (!C.WheeledInput.bCanMove)
	{
		return true;
	}

	const FWheeledInputState& IC = C.WheeledInput;
	const FWheeledInputState& IS = S.WheeledInput;
	const bool bAnalogOk = IS.Throttle == IC.Throttle
		&& IS.Brake == IC.Brake
		&& IS.Steer == IC.Steer;
	if (!bAnalogOk)
	{
		UE_LOG(WheelNetcodeLog, Warning,
			TEXT("[NET][INPUT MISMATCH] Frame=%u ClientFrame=%u Autonomous=%d ")
			TEXT("C(Throttle=%u Brake=%u Steer=%d) ")
			TEXT("S(Throttle=%u Brake=%u Steer=%d)"),
			S.LocalFrame, S.ClientFrame, C.bIsAutonomousProxy,
			(uint8)IC.Throttle, (uint8)IC.Brake, (int8)IC.Steer,
			(uint8)IS.Throttle, (uint8)IS.Brake, (int8)IS.Steer
		);
	}
	return true;
}

void FNetworkWheeledSpeedInputState::DecayData(float DecayAmount)
{
}

void FNetworkWheeledSpeedInputState::MergeData(const FNetworkPhysicsData& FromData)
{
	const FNetworkWheeledSpeedInputState& FromState = static_cast<const FNetworkWheeledSpeedInputState&>(FromData);
	// Perform merge through InterpolateData
	InterpolateData(FromState, *this);
}
