#include "IAmSpeed/Input/ActionDispatch.h"
#include "IAmSpeed/Input/Testing/TestInputProducerV2.h"
#include <cstdlib>
#include <iostream>

using namespace Speed::Input::V2;
using Speed::Input::EProducerKind;
static unsigned Checks = 0;
static void Check(bool Value, const char* Message)
{ ++Checks; if (!Value) { std::cerr << "FAIL " << Message << '\n'; std::exit(1); } }

static std::shared_ptr<const FInputActionContract> Contract()
{
	FInputActionContractDescription D; D.Revision = {1}; D.Actions = FInputActionContract::BaseActions();
	FActionDefinition Button; Button.Id = 3; Button.Owner = "Fixture"; Button.Name = "Button"; Button.Wiring = EActionWiring::Wired;
	D.Actions.push_back(Button);
	D.Mapping = {{{ERawControlKind::KeyboardUsage, 4}, 3, 1}};
	D.Physical = {{0, EPhysicalDestination::Throttle}, {1, EPhysicalDestination::Brake}, {2, EPhysicalDestination::Steering}};
	return FInputActionContract::Create(D);
}
static FRawInputSample Raw(std::size_t Changes, bool Mapped)
{
	FRawInputSample R; R.DeviceId = 1; R.Generation = {1}; R.Sequence = Changes ? 2 : 1;
	R.Status = Changes ? ERawSampleStatus::Valid : ERawSampleStatus::Resync;
	R.Kind = ERawDeviceKind::Keyboard;
	R.FinalState = {{{ERawControlKind::KeyboardUsage, 4}, 0}, {{ERawControlKind::KeyboardUsage, 5}, 0}};
	for (std::size_t I = 0; I < Changes; ++I)
		R.Changes.push_back({{{ERawControlKind::KeyboardUsage, std::uint16_t(Mapped ? 4 : 5)}, I % 2 ? 0.0f : 1.0f}, {2, std::uint32_t(I)}});
	R.FinalState[Mapped ? 0 : 1].Value = Changes % 2 ? 1.0f : 0.0f;
	return R;
}
struct FReceiver
{
	std::vector<FInputEdgeIdentity> Edges;
	void Event(const FActionEvent& E)
	{
		Check(E.Identity.has_value(), "real edge has stable identity");
		Edges.push_back(*E.Identity);
	}
};
int main()
{
	const auto C = Contract(); Check(bool(C), "contract");
	for (bool Mapped : {false, true})
		for (std::size_t N : Mapped ? std::vector<std::size_t>{63, 64, 65} : std::vector<std::size_t>{255, 256, 257})
		{
			auto Mapper = FActionMapper::Create(C, {1}, {EProducerKind::Device, 1});
			Check(Mapper->Map(Raw(0, Mapped), 0).Frame.has_value(), "capacity baseline");
			const auto Result = Mapper->Map(Raw(N, Mapped), 1);
			const bool Fits = N <= (Mapped ? 64u : 256u);
			Check(Result.Frame.has_value() == Fits, "N-1/N/N+1 admission exact");
			if (Fits)
				Check(Result.Frame->GetData().Transitions.size() == (Mapped ? N : 0), "every transition retained, no truncation");
			else
			{
				Check(Result.Status == EMappingStatus::Overflow && !Result.Frame, "overflow rejects whole frame");
				Check(Mapper->Map(Raw(2, Mapped), 1).Status == EMappingStatus::ResyncRequired, "overflow requires explicit resync");
				auto Reset = Raw(0, Mapped); Reset.Sequence = 3;
				Reset.FinalState[0].Value = 1;
				const auto Recovery = Mapper->Map(Reset, 1);
				Check(Recovery.Frame && Recovery.Frame->GetData().Reset && Recovery.Frame->GetData().Transitions.empty()
					&& Recovery.Frame->GetData().Values[3] == 1, "fresh held recovery creates no synthetic edge");
			}
		}
	for (std::size_t Count : {255u, 256u, 257u})
	{
		std::vector<FInputFrame> Frames;
		for (std::size_t N = 0; N <= Count; ++N)
		{
			FInputFrameData D; D.ConsumptionFrame = N; D.SourceSequence = N + 1;
			D.StreamEpoch = {1}; D.Producer = {EProducerKind::Device, 1}; D.DeviceGeneration = {1}; D.Reset = N == 0;
			if (N == 1) D.Transitions = {{3, ETransition::Started, 1, {2, 0}}, {3, ETransition::Completed, 0, {2, 1}},
				{3, ETransition::Started, 1, {2, 2}}, {3, ETransition::Completed, 0, {2, 3}}};
			Frames.emplace_back(C, D);
		}
		std::shared_ptr<IInputProducer> P = FTestInputProducer::Create(C, {1}, {EProducerKind::Device, 1}, 0, Frames);
		Check(bool(P), "sealed capacity timeline");
		auto S = std::make_shared<FInputStream>(P, C, FStreamEpoch{1}, Speed::Input::FProducerIdentity{EProducerKind::Device, 1});
		auto Receiver = std::make_shared<FReceiver>(); FInputPresentationBindings Bindings(S);
		Check(Bindings.BindAction("start", 3, EStateAction::Started, std::weak_ptr<FReceiver>(Receiver), &FReceiver::Event)
			&& Bindings.BindAction("complete", 3, EStateAction::Completed, std::weak_ptr<FReceiver>(Receiver), &FReceiver::Event)
			&& Bindings.Seal() && S->Activate(), "stateful capacity bindings");
		for (std::size_t N = 0; N < Count; ++N)
		{
			const auto R = S->Consume(N);
			Check(R.Reservation && S->ConfirmPhysicalCommit(*R.Reservation, true)
				&& S->PreparePublication(*R.Reservation) && S->FinalizePublication(*R.Reservation), "completed physical transaction");
		}
		const auto Batch = S->ReadPublishedSince({{1}, 0});
		Check(Batch.Next.Serial == Count, "gap reports exact high watermark");
		if (Count <= 256)
		{
			Check(Batch.Status == EReadStatus::Batch && Batch.Frames.size() == Count, "capacity includes every frame");
			Check(Bindings.HandleInputs() == EDispatchStatus::Dispatched && Receiver->Edges.size() == 4, "two brief presses delivered completely");
			for (std::size_t I = 0; I < Receiver->Edges.size(); ++I)
				Check(Receiver->Edges[I].Frame == 1 && Receiver->Edges[I].Ordinal == I && Receiver->Edges[I].Action == 3, "edge identity keeps intra-frame order");
			Check(Bindings.HandleInputs() == EDispatchStatus::NoChange && Receiver->Edges.size() == 4, "repeat observer never duplicates");
		}
		else
		{
			Check(Batch.Status == EReadStatus::Overflow && Batch.Frames.empty(), "no partial retained suffix on gap");
			Check(Bindings.HandleInputs() == EDispatchStatus::ResyncRequired && Receiver->Edges.empty(), "gap gives no successful edge dispatch");
			Check(Bindings.Resynchronize() == EDispatchStatus::Resynchronized && Receiver->Edges.empty(), "resync never replays applied inputs");
			Check(S->ReadOutcome()->Frame == Count - 1 && S->ReadOutcome()->Outcome == ETransactionOutcome::Completed,
				"presentation gap does not revoke physical completion");
		}
	}
	std::cout << "PASS BooleanCapacityProbe checks=" << Checks << '\n';
}
