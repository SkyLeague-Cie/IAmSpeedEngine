#include "SpeedWheeledComponent.h"
#include "Misc/ScopeLock.h"

bool USpeedWheeledComponent::ClaimLegacyRemoteInputAuthority()
{
	FScopeLock Lock(&FrameInputProducerMutex);
	if (HasProducedInputAuthority() || IsTestInputOverrideEnabled()) return false;
	bLegacyRemoteInputAuthority.store(true, std::memory_order_release);
	return true;
}

Speed::Input::ELegacyRemoteAdmission USpeedWheeledComponent::SubmitLegacyWheeledInput(
	uint64 SourceFrame, int32 ActivationFrame, const FWheeledInputState& Wire)
{
	using namespace Speed::Input;
	if (ActivationFrame < 1) return ELegacyRemoteAdmission::Stale;
	const auto N = FromLegacyLocalFrame(uint32(ActivationFrame));
	if (!N || !Wire.Camera.IsValid()) return ELegacyRemoteAdmission::Stale;
	if (!ClaimLegacyRemoteInputAuthority()) return ELegacyRemoteAdmission::WrongSource;
	FScopeLock Lock(&LegacyWheeledIngressMutex);
	if (bLegacyWheeledRetired) return ELegacyRemoteAdmission::Closed;
	// Legacy transport has no wire epoch. This capability is bound once to this
	// component lifetime and is never reopened after retirement/reset.
	const uint64 Producer = uint64(GetUniqueID()) + 1;
	if (!LegacyWheeledIngress) LegacyWheeledIngress = std::make_shared<FLegacyWheeledIngress>(Producer, 1, 0);
	FActionValues Requested{};
	Requested[Throttle] = Wire.Throttle; Requested[Brake] = Wire.Brake; Requested[Steering] = Wire.Steer;
	return LegacyWheeledIngress->Submit({Producer, 1, SourceFrame, *N}, Requested, Wire);
}

bool USpeedWheeledComponent::PrepareLegacyWheeledInput(uint64 Frame)
{
	using namespace Speed::Input;
	std::shared_ptr<FLegacyWheeledIngress> Ingress;
	{ FScopeLock Lock(&LegacyWheeledIngressMutex); Ingress = LegacyWheeledIngress; }
	if (!Ingress) return true; // The independent Sky wire lane may arrive first.
	if (!LegacyWheeledOwner)
	{
		std::array<uint16, ActionCount> Steps{};
		Steps[Throttle] = Steps[Brake] = Steps[Steering] = 16;
		FActionValues Initial{};
		Initial[Throttle] = WheeledPhysicalInput.Throttle; Initial[Brake] = WheeledPhysicalInput.Brake; Initial[Steering] = WheeledPhysicalInput.Steer;
		LegacyWheeledOwner = std::make_unique<FLegacyWheeledOwner>(Ingress, Frame, Steps, Initial);
	}
	const auto Input = LegacyWheeledOwner->Poll(Frame);
	if (!Input) return false;
	WheeledPhysicalInput.Throttle = uint8(Input->Applied[Throttle]);
	WheeledPhysicalInput.Brake = uint8(Input->Applied[Brake]);
	WheeledPhysicalInput.Steer = int8(Input->Applied[Steering]);
	WheeledPhysicalInput.bCanMove = CanMove();
	if (Input->Packet) WheeledPhysicalInput.Camera = Input->Packet->Wire.Camera;
	SyncWheeledPhysicalInputToState();
	return true;
}

bool USpeedWheeledComponent::RetireInputProcessingOnWorker()
{
	FScopeLock Lock(&LegacyWheeledIngressMutex);
	if (LegacyWheeledOwner && !LegacyWheeledOwner->IsOwnerThread()) return false;
	bLegacyWheeledRetired = true;
	if (LegacyWheeledOwner) { LegacyWheeledOwner->Abort(); LegacyWheeledOwner.reset(); }
	if (LegacyWheeledIngress) LegacyWheeledIngress->Close();
	bInputRetirementCompleted.store(true, std::memory_order_release);
	return true;
}

bool USpeedWheeledComponent::ServiceInputRetirementAtBoundary()
{
	return !bInputRetirementRequested.load(std::memory_order_acquire)
		|| bInputRetirementCompleted.load(std::memory_order_acquire) || RetireInputProcessingOnWorker();
}
