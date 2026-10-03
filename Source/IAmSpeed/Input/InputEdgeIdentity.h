#pragma once

#include "InputFrameV2.h"

namespace Speed::Input::V2
{
// Values-only identity authored from the immutable physical frame, before any
// application. Independent of the presentation cursor and render cadence.
struct FInputEdgeIdentity
{
	std::uint64_t Epoch = 0;
	FFrameNumber Frame = 0;
	std::uint64_t SourceSequence = 0;
	std::uint32_t Ordinal = 0;
	FActionId Action = 0;
	EProducerKind ProducerKind = EProducerKind::Device;
	std::uint64_t ProducerId = 0;
	bool IsValid() const { return Epoch && SourceSequence && ProducerId && ProducerKind <= EProducerKind::Network && Ordinal < MaxEdges && Action < ActionCount; }
};
inline bool operator==(const FInputEdgeIdentity& A, const FInputEdgeIdentity& B)
{
	return A.Epoch == B.Epoch && A.Frame == B.Frame && A.SourceSequence == B.SourceSequence
		&& A.Ordinal == B.Ordinal && A.Action == B.Action && A.ProducerKind == B.ProducerKind && A.ProducerId == B.ProducerId;
}
inline std::optional<FInputEdgeIdentity> IdentifyEdge(const FInputFrame& Frame, std::size_t Ordinal)
{
	const auto& D = Frame.GetData();
	if (!Frame.GetContract() || !Frame.IsValidFor(*Frame.GetContract()) || Ordinal >= D.Transitions.size()) return {};
	return FInputEdgeIdentity{D.StreamEpoch.Value, D.ConsumptionFrame, D.SourceSequence,
		static_cast<std::uint32_t>(Ordinal), D.Transitions[Ordinal].Action, D.Producer.Kind, D.Producer.Id};
}
}
