// Fill out your copyright notice in the Description page of Project Settings.


#include "IAmSpeed/Components/Netcode/SpeedWheeledPhysicsComponent.h"
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "IAmSpeed/Base/SpeedConstant.h"

bool FSpeedCarCameraPhysicalInput::IsValid() const
{
	return Version == 1 && Flags <= 3 && Yaw != MIN_int8 && Pitch != MIN_int8 &&
		(IsPresent() || (Flags == 0 && Yaw == 0 && Pitch == 0));
}

bool FSpeedCarCameraPhysicalInput::Equals(const FSpeedCarCameraPhysicalInput& Other) const
{
	return Version == Other.Version && Flags == Other.Flags && Yaw == Other.Yaw && Pitch == Other.Pitch;
}

bool FSpeedCarCameraPhysicalInput::Quantize(bool bBack, float InYaw, float InPitch, FSpeedCarCameraPhysicalInput& Out)
{
	if (!FMath::IsFinite(InYaw) || !FMath::IsFinite(InPitch)) return false;
	FSpeedCarCameraPhysicalInput Value;
	Value.Flags = 1 | (bBack ? 2 : 0);
	Value.Yaw = FMath::RoundToInt(FMath::Clamp(InYaw, -1.0f, 1.0f) * 127.0f);
	Value.Pitch = FMath::RoundToInt(FMath::Clamp(InPitch, -1.0f, 1.0f) * 127.0f);
	Out = Value;
	return true;
}

bool FSpeedCarCameraPhysicalInput::Serialize(FArchive& Ar)
{
	auto Value = *this;
	Ar << Value.Version << Value.Flags << Value.Yaw << Value.Pitch;
	if (Ar.IsError() || !Value.IsValid()) { Ar.SetError(); return false; }
	if (Ar.IsLoading()) *this = Value;
	return true;
}


void FNetworkWheeledSpeedState::ApplyData(UActorComponent* NetworkComponent) const
{
	if (USpeedWheeledComponent* Mover = Cast<USpeedWheeledComponent>(NetworkComponent))
	{
		if (!Mover->AllowsExternalNetworkStateRestore()) return;
#if !(UE_BUILD_SHIPPING)
		UE_LOG(WheelNetcodeLog, Warning, TEXT("[WheeledSpeed] ApplyData (RESIMULATION?) Triggered for frame = %d"), Mover->NumFrame());
#endif
		Mover->WheeledPhysicsState = WheeledState;
		if (SourceFramesSinceCanMove != INDEX_NONE)
		{
			Mover->SinceCanMoveFrame = FMath::Max(0, GetSourceLocalFrame() - SourceFramesSinceCanMove);
		}
		Mover->RestoreWheeledPhysicalInputFromState();
	}
}

void FNetworkWheeledSpeedState::BuildData(const UActorComponent* NetworkComponent)
{
	if (NetworkComponent)
	{
		if (const USpeedWheeledComponent* Mover = Cast<const USpeedWheeledComponent>(NetworkComponent))
		{
			const int32 CurrentRecordedFrame = FMath::Max(int32(LocalFrame), 0);
			const int32 PreviousRecordedFrame = Speed::SimUtils::GetRecordedFrameFromNetworkLocalFrame(LocalFrame);
			const int32 ComponentFrame = FMath::Max(int32(Mover->NumFrame()), 0);
			const int32 PreviousComponentFrame = FMath::Max(ComponentFrame - 1, 0);
			const auto TryGetWheeledState = [this, Mover](const int32 Frame)
			{
				if (Mover->GetWheeledState(Frame, WheeledState))
				{
					SourceLocalFrame = Frame;
					const int32 SinceCanMoveFrame = Mover->GetSinceCanMoveFrame();
					SourceFramesSinceCanMove = (SinceCanMoveFrame != INDEX_NONE && Frame >= SinceCanMoveFrame)
						? Frame - SinceCanMoveFrame
						: INDEX_NONE;
					return true;
				}
				return false;
			};
			const bool bGotState = TryGetWheeledState(CurrentRecordedFrame)
				|| (PreviousRecordedFrame != CurrentRecordedFrame && TryGetWheeledState(PreviousRecordedFrame))
				|| (ComponentFrame != CurrentRecordedFrame && ComponentFrame != PreviousRecordedFrame && TryGetWheeledState(ComponentFrame))
				|| (PreviousComponentFrame != CurrentRecordedFrame && PreviousComponentFrame != PreviousRecordedFrame && PreviousComponentFrame != ComponentFrame && TryGetWheeledState(PreviousComponentFrame));
			if (!bGotState)
			{
				SourceLocalFrame = ComponentFrame;
				const int32 SinceCanMoveFrame = Mover->GetSinceCanMoveFrame();
				SourceFramesSinceCanMove = (SinceCanMoveFrame != INDEX_NONE && ComponentFrame >= SinceCanMoveFrame)
					? ComponentFrame - SinceCanMoveFrame
					: INDEX_NONE;
				WheeledState = Mover->WheeledPhysicsState;
			}
		}
	}
}

bool FNetworkWheeledSpeedState::NetSerialize(FArchive& Ar, UPackageMap* Map, bool& bOutSuccess)
{
	FNetworkPhysicsData::SerializeFrames(Ar);
	Ar << SourceLocalFrame;
	Ar << SourceFramesSinceCanMove;
	Ar << WheeledState.nbFramesbeforeCanMove;
	Ar << WheeledState.AllowedSideVelocity;
	Ar << WheeledState.AllowedAngularVelocity;
	Ar << WheeledState.NbFramesSinceGroundContact;
	Ar << WheeledState.FramesSinceLastImpact;
	Ar << WheeledState.PhysicalThrottleInput;
	Ar << WheeledState.PhysicalBrakeInput;
	Ar << WheeledState.PhysicalSteerInput;

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

int32 FNetworkWheeledSpeedState::GetSourceLocalFrame() const
{
	if (SourceLocalFrame == INDEX_NONE)
	{
		return LocalFrame;
	}

	return bReceivedData ? SourceLocalFrame - (ServerFrame - LocalFrame) : SourceLocalFrame;
}

void FNetworkWheeledSpeedState::InterpolateData(const FNetworkPhysicsData& MinData, const FNetworkPhysicsData& MaxData)
{
	const FNetworkWheeledSpeedState& MinState = static_cast<const FNetworkWheeledSpeedState&>(MinData);
	const FNetworkWheeledSpeedState& MaxState = static_cast<const FNetworkWheeledSpeedState&>(MaxData);

	const int32 MinFrame = static_cast<int32>(MinState.LocalFrame);
	const int32 MaxFrame = static_cast<int32>(MaxState.LocalFrame);
	const int32 ThisFrame = static_cast<int32>(LocalFrame);
	const int32 FrameDelta = MaxFrame - MinFrame;
	const float LerpFactor = FrameDelta != 0 ? float(ThisFrame - MinFrame) / float(FrameDelta) : 1.f;
	const bool bUseMaxState = (MaxFrame == ThisFrame) || LerpFactor >= 0.5f;
	const FNetworkWheeledSpeedState& SourceState = bUseMaxState ? MaxState : MinState;

	WheeledState = SourceState.WheeledState;
	SourceLocalFrame = SourceState.SourceLocalFrame;
	SourceFramesSinceCanMove = SourceState.SourceFramesSinceCanMove;
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

int32 FNetworkWheeledSpeedInputState::ResolveActivationFrame(int32 SinceCanMoveFrame, bool bLocalCapture) const
{
	const int64 Frame = (ClientFramesSinceCanMove != INDEX_NONE && SinceCanMoveFrame != INDEX_NONE)
		? int64(SinceCanMoveFrame) + ClientFramesSinceCanMove + 1
		: (bLocalCapture ? int64(ClientFrame) + 1 : int64(LocalFrame));
	return Frame >= 0 && Frame <= MAX_int32 ? int32(Frame) : INDEX_NONE;
}

void FNetworkWheeledSpeedInputState::ApplyData(UActorComponent* NetworkComponent) const
{
	if (!WheeledInput.Camera.IsValid()) return;
	if (USpeedWheeledComponent* Mover = Cast<USpeedWheeledComponent>(NetworkComponent))
	{
		if (Mover->IsTestInputOverrideEnabled() || Mover->HasProducedInputAuthority()) return;
		const int32 Activation = ResolveActivationFrame(Mover->GetPublishedInputTimelineOrigin(), false);
		const auto Result = Mover->SubmitLegacyWheeledInput(ClientFrame, Activation, WheeledInput);
		if (Result != Speed::Input::ELegacyRemoteAdmission::Accepted)
			UE_LOG(SpeedInputLog, Verbose, TEXT("[LegacyRemotePreSlew] wheel packet rejected status=%u source=%u activation=%d"), uint32(Result), ClientFrame, Activation);
	}
}

void FNetworkWheeledSpeedInputState::BuildData(const UActorComponent* NetworkComponent)
{
	bInputProjectionValid = false;
	const auto* Mover = Cast<const USpeedWheeledComponent>(NetworkComponent);
	if (!Mover) return;
	const auto Input = std::atomic_load(&Mover->PublishedWheeledNetworkInput);
	if (!Input) return;
	WheeledInput = Input->Requested; ClientFrame = Input->LocalFrame;
	ClientFramesSinceCanMove = Input->SinceCanMove != INDEX_NONE && int64(ClientFrame) >= Input->SinceCanMove
		? int32(int64(ClientFrame) - Input->SinceCanMove) : INDEX_NONE;
	bIsAutonomousProxy = Mover->GetOwnerRole() == ROLE_AutonomousProxy;
	bInputProjectionValid = true;
}

bool FNetworkWheeledSpeedInputState::NetSerialize(FArchive& Ar, UPackageMap* Map, bool& bOutSuccess)
{
	if (Ar.IsSaving() && !bInputProjectionValid) { bOutSuccess = false; return false; }
	// Decode the complete INPUT transactionally, including the new versioned
	// camera tail. Old mechanical-state serializers remain byte-identical.
	auto Value = *this;
	Value.FNetworkPhysicsData::SerializeFrames(Ar);
	Ar << Value.WheeledInput.Throttle;
	Ar << Value.WheeledInput.Brake;
	Ar << Value.WheeledInput.Steer;
	Ar << Value.ClientFrame;
	Ar << Value.ClientFramesSinceCanMove;
	Ar << Value.bIsAutonomousProxy;
	Ar << Value.WheeledInput.bCanMove;
	bOutSuccess = Value.WheeledInput.Camera.Serialize(Ar) && !Ar.IsError();
	if (bOutSuccess && Ar.IsLoading()) { Value.bInputProjectionValid=true; *this = Value; }
	return bOutSuccess;
}

void FNetworkWheeledSpeedInputState::InterpolateData(const FNetworkPhysicsData& MinData, const FNetworkPhysicsData& MaxData)
{
	const FNetworkWheeledSpeedInputState& MinState =
		static_cast<const FNetworkWheeledSpeedInputState&>(MinData);

	const FNetworkWheeledSpeedInputState& MaxState =
		static_cast<const FNetworkWheeledSpeedInputState&>(MaxData);

	const int32 MinFrame = int32(MinState.LocalFrame);
	const int32 MaxFrame = int32(MaxState.LocalFrame);
	const int32 ThisFrame = int32(LocalFrame);

	const int32 FrameDelta = MaxFrame - MinFrame;
	const float LerpFactor =
		FrameDelta != 0
		? float(ThisFrame - MinFrame) / float(FrameDelta)
		: 1.f;

	const bool bUseMax =
		(ThisFrame >= MaxFrame) || (LerpFactor >= 0.5f);

	const FNetworkWheeledSpeedInputState& Source =
		bUseMax ? MaxState : MinState;

	if (!Source.WheeledInput.Camera.IsValid()) return;
	WheeledInput = Source.WheeledInput;
	ClientFrame = Source.ClientFrame;
	ClientFramesSinceCanMove = Source.ClientFramesSinceCanMove;
	bIsAutonomousProxy = Source.bIsAutonomousProxy;
}

bool FNetworkWheeledSpeedInputState::CompareData(const FNetworkPhysicsData& PredictedData)
{
	const FNetworkWheeledSpeedInputState& C = static_cast<const FNetworkWheeledSpeedInputState&>(PredictedData); // Client predicted
	const FNetworkWheeledSpeedInputState& S = *this;                                                  // Server (authoritative for that frame)

	if (!S.WheeledInput.Camera.IsValid() || !C.WheeledInput.Camera.IsValid() ||
		!S.WheeledInput.Camera.Equals(C.WheeledInput.Camera)) return false;
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
