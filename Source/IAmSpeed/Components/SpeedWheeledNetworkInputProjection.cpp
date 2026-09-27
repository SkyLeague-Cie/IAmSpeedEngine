#include "SpeedWheeledComponent.h"

int32 USpeedWheeledComponent::GetPublishedInputTimelineOrigin() const
{
	const auto Input = std::atomic_load(&PublishedWheeledNetworkInput);
	return Input ? Input->SinceCanMove : INDEX_NONE;
}

bool USpeedWheeledComponent::PrepareWheeledNetworkInput(uint64 Frame) const
{
	using namespace Speed::Input;
	if (Frame >= MAX_uint32) return false;
	if (PreparedWheeledNetworkInput && PreparedWheeledNetworkInput->LocalFrame == Frame + 1) return true;
	auto Result = std::make_shared<FCompletedWheeledInput>();
	Result->Requested = WheeledPhysicalInput;
	Result->LocalFrame = uint32(Frame + 1); Result->SinceCanMove = SinceCanMoveFrame;
	const FActionValues* Values = nullptr;
	std::shared_ptr<const FLegacyWheeledOwner::FSnapshot> Remote;
	if (CanonicalInputSnapshot) Values = &CanonicalInputSnapshot->Requested;
	else if (LegacyWheeledOwner)
	{
		Remote = LegacyWheeledOwner->ReadPending();
		if (!Remote || Remote->Frame != Frame) return false;
		Values = &Remote->Requested;
	}
	if (Values)
	{
		Result->Requested.Throttle = uint8((*Values)[Throttle]);
		Result->Requested.Brake = uint8((*Values)[Brake]); Result->Requested.Steer = int8((*Values)[Steering]);
	}
	PreparedWheeledNetworkInput = std::move(Result);
	return true;
}
