#pragma once

#include "InputStreamV2.h"
#include "InputEdgeIdentity.h"
#include <functional>

namespace Speed::Input::V2
{
struct FActionEvent
{
	const FInputFrame& Frame;
	const FActionId Action;
	const EStateAction State;
	const std::int16_t Value;
	// Only actual Started/Completed have identities. Held Triggered is not an edge.
	const std::optional<FInputEdgeIdentity> Identity;
};
enum class EDispatchStatus : std::uint8_t { Dispatched, NoChange, ResyncRequired, Resynchronized, Detached, NotSealed, Reentrant };

// Presentation-lane facade only. A worker never owns or calls these bindings.
// Register/seal before activating the stream. Receivers use weak lifetime;
// opaque snapshot callbacks exist solely for the legacy portable observation API.
class FInputPresentationBindings final
{
public:
	using FSnapshotCallback = std::function<void(const FInputFrame&, FActionId)>;
	explicit FInputPresentationBindings(const std::shared_ptr<IInputPublicationSource>& InStream)
		: Stream(InStream), Cursor{InStream ? InStream->GetEpoch() : FStreamEpoch{}, 0} {}
	template<class T>
	bool BindAction(std::string Name, FActionId Action, EStateAction State,
		std::weak_ptr<T> Receiver, void (T::*Method)(const FActionEvent&))
	{
		if (!Method || Receiver.expired() || State > EStateAction::Completed) return false;
		return Add({std::move(Name), EBindingKind::Stateful, Action, State,
			[Receiver, Method](const FInputFrame& Frame, FActionId Id, EStateAction Event, std::int16_t Value, const std::optional<FInputEdgeIdentity>& Identity)
			{ if (auto Target = Receiver.lock()) ((*Target).*Method)(FActionEvent{Frame, Id, Event, Value, Identity}); }});
	}
	template<class T>
	bool BindReset(std::string Name, std::weak_ptr<T> Receiver, void (T::*Method)(const FInputFrame&))
	{
		if (!Method || Receiver.expired()) return false;
		return Add({std::move(Name), EBindingKind::Reset, 0, EStateAction::Started,
			[Receiver, Method](const FInputFrame& Frame, FActionId, EStateAction, std::int16_t, const std::optional<FInputEdgeIdentity>&)
			{ if (auto Target = Receiver.lock()) ((*Target).*Method)(Frame); }});
	}
	template<class T>
	bool BindSnapshotAction(std::string Name, FActionId Action,
		std::weak_ptr<T> Receiver, void (T::*Method)(const FInputFrame&, FActionId))
	{
		if (!Method || Receiver.expired()) return false;
		return BindSnapshotAction(std::move(Name), Action,
			[Receiver, Method](const FInputFrame& Frame, FActionId Id)
			{ if (auto Target = Receiver.lock()) ((*Target).*Method)(Frame, Id); });
	}
	bool BindSnapshotAction(std::string Name, FActionId Action, FSnapshotCallback Callback)
	{
		if (!Callback) return false;
		return Add({std::move(Name), EBindingKind::Snapshot, Action, EStateAction::Triggered,
			[Callback = std::move(Callback)](const FInputFrame& Frame, FActionId Id, EStateAction, std::int16_t, const std::optional<FInputEdgeIdentity>&) { Callback(Frame, Id); }});
	}
	// Deprecated semantic alias: this observes each NEW LATEST even if neutral.
	// It is NOT a stateful Triggered binding. Kept warning-free for /WX clients.
	bool BindAction(std::string Name, FActionId Action, FSnapshotCallback Callback)
	{ return BindSnapshotAction(std::move(Name), Action, std::move(Callback)); }
	bool Unbind(const std::string& Name)
	{
		if (!CanEdit()) return false;
		for (auto I = Bindings.begin(); I != Bindings.end(); ++I)
			if (I->Name == Name) { Bindings.erase(I); return true; }
		return false;
	}
	bool Seal()
	{
		if (!CanEdit()) return false;
		Sealed = true; return true;
	}
	EDispatchStatus Resynchronize()
	{
		if (Dispatching || FPresentationInputScope::IsActive()) return EDispatchStatus::Reentrant;
		if (!Sealed) return EDispatchStatus::NotSealed;
		const auto Source = Stream.lock();
		if (!Source || !Source->IsActive()) return Detach();
		const auto Latest = Source->ReadLatest();
		if (!Latest) return EDispatchStatus::NoChange;
		std::optional<FInputFrame> NextPresented{Latest->Frame};
		static_assert(std::is_nothrow_swappable_v<std::optional<FInputFrame>>);
		LastPresented.swap(NextPresented);
		Cursor = {Source->GetEpoch(), Latest->Serial};
		NeedsResync = false;
		return EDispatchStatus::Resynchronized; // Explicit baseline; no invented edges/callbacks.
	}
	EDispatchStatus HandleInputs()
	{
		if (Dispatching || FPresentationInputScope::IsActive()) return EDispatchStatus::Reentrant;
		if (!Sealed) return EDispatchStatus::NotSealed;
		const auto Source = Stream.lock();
		if (!Source || !Source->IsActive()) return Detach();
		if (NeedsResync) return EDispatchStatus::ResyncRequired;
		const auto Batch = Source->ReadPublishedSince(Cursor); // Coherent copy; lock released before callbacks.
		if (Batch.Status == EReadStatus::Detached) return Detach();
		if (Batch.Status == EReadStatus::NoChange) return EDispatchStatus::NoChange;
		if (Batch.Status != EReadStatus::Batch || !ValidBatch(Batch, *Source))
		{ NeedsResync = true; return EDispatchStatus::ResyncRequired; }
		const auto Snapshot = Bindings;
		// Prepare continuity before the first callback. A post-callback allocation
		// failure must never leave a cursor advanced past its continuity frame.
		std::optional<FInputFrame> NextPresented{Batch.Frames.back().Frame};
		Dispatching = true;
		struct FDispatchGuard { bool& Flag; ~FDispatchGuard() { Flag = false; } } Guard{Dispatching};
		FPresentationInputScope ReadOnly;
		for (const auto& Published : Batch.Frames)
		{
			const auto& Frame = Published.Frame;
			if (Frame.GetData().Reset)
				for (const auto& B : Snapshot) if (B.Kind == EBindingKind::Reset)
				{
					if (!Source->IsActive()) return Detach();
					if (!Invoke(B, *Source, Frame, B.Action, B.State, 0)) return Detach();
				}
			for (std::size_t Ordinal = 0; Ordinal < Frame.GetData().Transitions.size(); ++Ordinal)
			{
				const auto& E = Frame.GetData().Transitions[Ordinal];
				const auto State = E.State == ETransition::Started ? EStateAction::Started : EStateAction::Completed;
				for (const auto& B : Snapshot) if (B.Kind == EBindingKind::Stateful && B.Action == E.Action && B.State == State)
				{
					if (!Source->IsActive()) return Detach();
						if (!Invoke(B, *Source, Frame, E.Action, State, E.ValueAtTransition, IdentifyEdge(Frame, Ordinal))) return Detach();
				}
			}
		}
		const auto& Latest = Batch.Frames.back().Frame;
		for (const auto& B : Snapshot)
			if (B.Kind == EBindingKind::Stateful && B.State == EStateAction::Triggered
				&& (Latest.GetData().ActiveMask & (std::uint32_t{1} << B.Action)))
			{
				if (!Source->IsActive()) return Detach();
				if (!Invoke(B, *Source, Latest, B.Action, B.State, Latest.GetData().Values[B.Action])) return Detach();
			}
		for (const auto& B : Snapshot) if (B.Kind == EBindingKind::Snapshot)
		{
			if (!Source->IsActive()) return Detach();
			if (!Invoke(B, *Source, Latest, B.Action, B.State, Latest.GetData().Values[B.Action])) return Detach();
		}
		if (!Source->IsActive()) return Detach();
		LastPresented.swap(NextPresented);
		Cursor = Batch.Next;
		return EDispatchStatus::Dispatched;
	}
private:
	enum class EBindingKind : std::uint8_t { Stateful, Snapshot, Reset };
	struct FBinding
	{
		std::string Name; EBindingKind Kind; FActionId Action; EStateAction State;
		std::function<void(const FInputFrame&, FActionId, EStateAction, std::int16_t, const std::optional<FInputEdgeIdentity>&)> Callback;
	};
	static bool Invoke(const FBinding& B, IInputPublicationSource& Source, const FInputFrame& Frame,
		FActionId Action, EStateAction State, std::int16_t Value, const std::optional<FInputEdgeIdentity>& Identity = {})
	{
		try { B.Callback(Frame, Action, State, Value, Identity); return true; }
		catch (...) { Source.Deactivate(); return false; } // Never retry a delivered prefix after a throwing callback.
	}
	bool CanEdit() const
	{
		const auto Source = Stream.lock();
		return !Dispatching && !Sealed && !FPresentationInputScope::IsActive() && Source && Source->CanConfigure();
	}
	bool Add(FBinding Binding)
	{
		if (!CanEdit() || Binding.Name.empty() || Bindings.size() == 128) return false;
		const auto Source = Stream.lock();
		if (!Source || !Source->GetContract()) return false;
		const auto* Action = Source->GetContract()->Find(Binding.Action);
		if (Binding.Kind != EBindingKind::Reset && (!Action || Action->Wiring != EActionWiring::Wired)) return false;
		for (const auto& B : Bindings) if (B.Name == Binding.Name) return false;
		Bindings.push_back(std::move(Binding)); return true;
	}
	EDispatchStatus Detach() { Cursor = {}; LastPresented.reset(); NeedsResync = true; return EDispatchStatus::Detached; }
	bool ValidBatch(const FPublishedBatch& Batch, const IInputPublicationSource& Source) const
	{
		if (Batch.Frames.empty() || !Source.GetContract() || Batch.Next.Epoch.Value != Cursor.Epoch.Value) return false;
		auto Previous = LastPresented;
		std::uint64_t Serial = Cursor.Serial;
		for (const auto& P : Batch.Frames)
		{
			if (Serial == std::numeric_limits<std::uint64_t>::max() || P.Serial != ++Serial || !P.Frame.IsValidFor(*Source.GetContract())) return false;
			const auto& D = P.Frame.GetData();
			if (D.StreamEpoch.Value != Cursor.Epoch.Value || (!Previous && !D.Reset)) return false;
			if (Previous)
			{
				const auto N = Previous->GetData().ConsumptionFrame;
				if (N == std::numeric_limits<FFrameNumber>::max() || D.ConsumptionFrame != N + 1) return false;
			}
			Previous = P.Frame;
		}
		return Serial == Batch.Next.Serial;
	}
	const std::weak_ptr<IInputPublicationSource> Stream;
	FPublicationCursor Cursor;
	std::optional<FInputFrame> LastPresented;
	std::vector<FBinding> Bindings;
	bool Sealed = false, Dispatching = false, NeedsResync = false;
};
}
