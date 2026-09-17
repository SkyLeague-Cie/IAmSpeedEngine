#include "IAmSpeed/Input/InputStream.h"
#include <atomic>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <type_traits>

using namespace Speed::Input;
static unsigned Checks = 0;
static void Check(bool Condition, const char* Label)
{
	++Checks;
	if (!Condition) { std::cerr << "FAIL: " << Label << '\n'; std::exit(1); }
}

// Fieldwise fingerprint: never hash padding, mutexes, or platform object layout.
static std::uint64_t Hash(const FInputFrame& Frame)
{
	std::uint64_t Result = 14695981039346656037ull;
	auto Add = [&Result](std::uint64_t Value)
	{
		for (unsigned I = 0; I < 8; ++I) { Result ^= (Value >> (I * 8)) & 255; Result *= 1099511628211ull; }
	};
	Add(Frame.GetSourceFrame()); Add(Frame.GetConsumptionFrame());
	Add(Frame.GetProducer().Id); Add(static_cast<unsigned>(Frame.GetProducer().Kind));
	for (auto Value : Frame.GetActions()) Add(static_cast<std::uint16_t>(Value));
	Add(Frame.GetEdgeCount());
	for (std::size_t I = 0; I < Frame.GetEdgeCount(); ++I)
	{
		const auto& E = Frame.GetEdges()[I]; Add(E.Action); Add(static_cast<unsigned>(E.Kind)); Add(E.SourceFrame);
	}
	return Result;
}

int main()
{
	static_assert(std::is_same_v<decltype(std::declval<FInputFrame&>().GetActions()), const FActionValues&>);
	static_assert(std::is_same_v<decltype(std::declval<FInputFrame&>().GetEdges()), const std::array<FActionEdge, MaxEdges>&>);
	Check(!FromLegacyLocalFrame(0), "legacy zero rejected");
	Check(*FromLegacyLocalFrame(1) == 0, "legacy first frame is canonical zero");
	Check(*FromLegacyLocalFrame(60) == 59, "CanMove boundary conversion");
	Check(*QuantizeAxis(0.5f, false) == 128 && *QuantizeAxis(-0.5f, true) == -63, "quantization ties");
	Check(*QuantizeAxis(2, false) == 255 && *QuantizeAxis(-2, false) == 0, "unsigned clamps");
	Check(*QuantizeAxis(2, true) == 127 && *QuantizeAxis(-2, true) == -127, "signed clamps");
	Check(!QuantizeAxis(std::numeric_limits<float>::quiet_NaN(), true), "NaN rejected");
	Check(!QuantizeAxis(std::numeric_limits<float>::infinity(), false), "infinity rejected");
	Check(*QuantizeAxis(-0.0f, true) == 0, "negative zero canonicalized");

	constexpr FActionId Jump = 3, Powerslide = 4; // Game-defined test schema only.
	FDeviceInputProducer Device(17);
	Check(Device.SetAction(40, Throttle, 10, false), "first throttle");
	Check(Device.SetAction(40, Throttle, 203, false), "latest different value wins");
	Check(Device.SetAction(40, Jump, 1, true), "JumpStart");
	Check(Device.SetAction(40, Powerslide, 1, true), "PowerslideStart");
	Check(Device.SetAction(40, Jump, 0, true), "JumpStop");
	Check(Device.SetAction(40, Jump, 1, true), "JumpStart again same source frame");
	Check(Device.SetAction(41, Powerslide, 0, true), "PowerslideStop next source frame");
	const auto First = Device.Produce(0);
	Check(First && First->IsValid() && First->GetActions()[Throttle] == 203, "complete snapshot at canonical zero");
	Check(First->GetSourceFrame() == 41 && First->GetEdgeCount() == 5, "all ordered edges retained");
	Check(First->GetEdges()[0].Action == Jump && First->GetEdges()[1].Action == Powerslide
		&& First->GetEdges()[2].Kind == EEdgeKind::Stop && First->GetEdges()[3].Kind == EEdgeKind::Start
		&& First->GetEdges()[4].Action == Powerslide && First->GetEdges()[4].SourceFrame == 41, "total callback order");
	const auto FirstHash = Hash(*First);
	Check(Device.SetAction(42, Throttle, 80, false), "new live value");
	Check(Hash(*Device.Produce(0)) == FirstHash && Hash(*First) == FirstHash, "idempotent replay and immutable copy");
	Check(!Device.Produce(2), "gap rejected");
	const auto Held = Device.Produce(1);
	Check(Held && Held->GetActions()[Jump] == 1 && Held->GetEdgeCount() == 0
		&& Held->GetActions()[Throttle] == 80, "held action does not repeat edges");
	Check(!Device.SetAction(41, Jump, 0, true), "stale source rejected");
	Check(!Device.SetAction(42, Throttle, 256, false) && !Device.SetAction(42, 32, 1, true), "invalid action/value rejected");
	for (FFrameNumber Frame = 2; Frame <= HistoryCapacity; ++Frame) Check(bool(Device.Produce(Frame)), "contiguous history");
	Check(!Device.Produce(0) && Hash(*First) == FirstHash, "evicted replay fails without changing saved copy");

	FDeviceInputProducer Overflow(19);
	for (std::size_t I = 0; I < MaxEdges; ++I)
		Check(Overflow.SetAction(1, Jump, (I % 2) ? 0 : 1, true), "bounded edge insertion");
	Check(!Overflow.SetAction(2, Jump, 1, true), "overflow rejected atomically");
	const auto Full = Overflow.Produce(0);
	Check(Full && Full->GetEdgeCount() == MaxEdges && Full->GetActions()[Jump] == 0
		&& Full->GetSourceFrame() == 1, "overflow changes neither values nor source");
	Check(Overflow.SetAction(2, Jump, 1, true), "capacity reusable after physical consumption");

	FActionValues Bad{}; Bad[Steering] = -128;
	Check(!FInputFrame(0, 0, {EProducerKind::AI, 1}, Bad).IsValid(), "noncanonical steering rejected");
	Check(!FInputFrame(0, 0, {EProducerKind::AI, 0}, {}).IsValid(), "zero identity rejected");
	std::array<FActionEdge, MaxEdges> BadEdges{};
	BadEdges[0] = {Jump, EEdgeKind::Start, 2};
	Check(!FInputFrame(1, 0, {EProducerKind::AI, 1}, {}, BadEdges, 1).IsValid(), "future edge rejected");
	Check(!FInputFrame(2, 0, {EProducerKind::AI, 1}, {}, BadEdges, 0).IsValid(), "unused edge data rejected");
	BadEdges[1] = {Jump, EEdgeKind::Stop, 1};
	Check(!FInputFrame(2, 0, {EProducerKind::AI, 1}, {}, BadEdges, 2).IsValid(), "source edge ordering validated");
	Check(!FInputFrame(2, 0, {EProducerKind::AI, 1}, {}, {}, MaxEdges + 1).IsValid(), "invalid edge count rejected");

	FAIInputProducer AI(17, 0);
	FNetworkInputProducer Network(17, 0);
	for (auto* Producer : {static_cast<FQueuedInputProducer*>(&AI), static_cast<FQueuedInputProducer*>(&Network)})
	{
		const auto Kind = Producer == &AI ? EProducerKind::AI : EProducerKind::Network;
		const FInputFrame Matching(First->GetSourceFrame(), 0, {Kind, 17}, First->GetActions(), First->GetEdges(), First->GetEdgeCount());
		Check(!Producer->Produce(0), "missing frame never invents hold-last");
		Check(!Producer->Submit(*First), "foreign producer kind rejected");
		Check(Producer->Submit(Matching) && !Producer->Submit(Matching), "exact submission and duplicate rejection");
		Check(!Producer->Submit(FInputFrame(0, HistoryCapacity, {Kind, 17}, {})), "future window bound");
		Check(!Producer->Produce(1), "future request cannot skip frame");
		const auto Actual = Producer->Produce(0);
		Check(Actual && Actual->GetActions() == First->GetActions() && Actual->GetEdgeCount() == 5, "canonical representation parity");
		const FInputFrame DeviceEquivalent(Actual->GetSourceFrame(), Actual->GetConsumptionFrame(), First->GetProducer(), Actual->GetActions(), Actual->GetEdges(), Actual->GetEdgeCount());
		Check(Hash(DeviceEquivalent) == FirstHash, "identical values edges and frame across three producers");
		Check(!Producer->Submit(Matching) && Hash(*Producer->Produce(0)) == Hash(*Actual), "late submission rejected replay stable");
		Check(!Producer->Produce(1), "next missing exact frame");
		FActionValues Next{}; Next[Throttle] = 7;
		Check(Producer->Submit(FInputFrame(43, 2, {Kind, 17}, Next)), "out of order future arrival");
		Next[Throttle] = 6;
		Check(Producer->Submit(FInputFrame(42, 1, {Kind, 17}, Next)), "late expected arrival");
		Check(Producer->Produce(1)->GetActions()[Throttle] == 6 && Producer->Produce(2)->GetActions()[Throttle] == 7, "distinct due values consumed in frame order");
	}
	Check(*FNetworkInputProducer::FromCanMove(59, -1) == 58 && *FNetworkInputProducer::FromCanMove(59, 0) == 59
		&& *FNetworkInputProducer::FromCanMove(59, 1) == 60, "CanMove offsets");
	Check(!FNetworkInputProducer::FromCanMove(0, -1)
		&& !FNetworkInputProducer::FromCanMove(1, std::numeric_limits<std::int64_t>::min())
		&& !FNetworkInputProducer::FromCanMove(std::numeric_limits<FFrameNumber>::max(), 1), "frame arithmetic overflow");
	FAIInputProducer Last(1, std::numeric_limits<FFrameNumber>::max());
	const FInputFrame Final(0, std::numeric_limits<FFrameNumber>::max(), {EProducerKind::AI, 1}, {});
	Check(Last.Submit(Final) && bool(Last.Produce(Final.GetConsumptionFrame())) && !Last.Submit(Final), "terminal frame cannot wrap");

	auto Source = std::make_shared<FDeviceInputProducer>(23);
	auto Stream = std::make_shared<FInputStream>(Source);
	Check(!Stream->ReadLatest() && !Stream->PublishCompleted(0), "nothing published before consume");
	Check(Source->SetAction(1, Throttle, 21, false), "presentation input seed");
	Check(bool(Stream->Consume(0)) && !Stream->ReadLatest(), "latch is not physical completion");
	Check(Stream->PublishCompleted(0), "completed frame published");
	const auto Copy = *Stream->ReadLatest();
	const auto Before = Hash(*Stream->ReadRecorded(0));
	FInputPresentationBindings Bindings;
	std::vector<unsigned> Order;
	unsigned RejectedWrites = 0;
	Check(Bindings.BindAction("Throttle", Throttle, [&](const FInputFrame& F, FActionId A)
	{
		Order.push_back(1);
		Check(A == Throttle && F.GetActions()[A] == 21, "callback exact latest value");
		RejectedWrites += !Source->SetAction(99, Throttle, 255, false);
		RejectedWrites += !Stream->Consume(1);
		RejectedWrites += !Stream->PublishCompleted(0);
		RejectedWrites += !Network.Submit(FInputFrame(99, 3, {EProducerKind::Network, 17}, {}));
		Check(!Bindings.BindAction("Injected", Jump, [](const auto&, auto) {}), "no reentrant binding mutation");
	}), "named binding");
	Check(Bindings.BindAction("ThrottleSound", Throttle, [&](const auto&, auto) { Order.push_back(2); }), "registration order");
	Check(!Bindings.BindAction("Throttle", Brake, [](const auto&, auto) {}), "duplicate name rejected");
	Check(!Bindings.BindAction("", Brake, [](const auto&, auto) {}) && !Bindings.BindAction("Unknown", 32, [](const auto&, auto) {}), "invalid name and slot rejected");
	for (int I = 0; I < 20; ++I) Bindings.HandleInputs(*Stream->ReadLatest());
	Check(Order == std::vector<unsigned>({1, 2}) && RejectedWrites == 4, "one dispatch per serial and blocked reinjection");
	Check(Hash(*Stream->ReadRecorded(0)) == Before && !FPresentationInputScope::IsActive(), "history unchanged scope released");
	Check(Source->SetAction(2, Jump, 1, true), "legitimate input outside callback remains valid");
	Check(bool(Stream->Consume(1)) && Stream->PublishCompleted(1), "intermediate press completed");
	Check(Source->SetAction(3, Jump, 0, true), "release before next GT poll");
	Check(bool(Stream->Consume(2)) && Stream->PublishCompleted(2), "latest release completed");
	Bindings.HandleInputs(*Stream->ReadLatest());
	Check(Order.size() == 4 && Stream->ReadRecorded(1)->GetEdges()[0].Kind == EEdgeKind::Start
		&& Stream->ReadLatest()->Frame.GetEdges()[0].Kind == EEdgeKind::Stop, "GT coalesces while physical edge history remains");
	Check(Hash(Copy.Frame) == Before && Copy.Serial == 1, "published copy survives later publications");
	Check(Stream->Consume(3)->GetActions()[Throttle] == 21, "rejected callback did not affect next physical frame");
	Stream->Skip(4);
	Check(!Stream->ReadRecorded(4) && !Stream->PublishCompleted(4), "suppressed script frame never masquerades as consumed live input");
	Check(bool(Stream->Consume(5)), "live source resumes after scripted suppression");

	// Synthetic game adapter: exact Jump edges are applied by the worker once;
	// presentation observes them and cannot become a second physical dispatcher.
	auto JumpDevice = std::make_shared<FDeviceInputProducer>(29);
	FInputStream JumpStream(JumpDevice);
	FInputPresentationBindings JumpBindings;
	unsigned PhysicalStarts = 0, PhysicalStops = 0, Notifications = 0;
	JumpBindings.BindAction("Jump", Jump, [&](const FInputFrame& F, FActionId Slot)
	{
		for (std::size_t I = 0; I < F.GetEdgeCount(); ++I)
			if (F.GetEdges()[I].Action == Slot) ++Notifications;
		Check(!JumpDevice->SetAction(50, Jump, 1, true), "Jump presentation cannot reinject");
	});
	for (FFrameNumber I = 0; I < 2; ++I)
	{
		JumpDevice->SetAction(I, Jump, I == 0 ? 1 : 0, true);
		const auto Physical = JumpStream.Consume(I);
		for (std::size_t E = 0; E < Physical->GetEdgeCount(); ++E)
			if (Physical->GetEdges()[E].Kind == EEdgeKind::Start) ++PhysicalStarts; else ++PhysicalStops;
		JumpStream.PublishCompleted(I);
		for (unsigned Tick = 0; Tick < 3; ++Tick) JumpBindings.HandleInputs(*JumpStream.ReadLatest());
	}
	Check(PhysicalStarts == 1 && PhysicalStops == 1 && Notifications == 2, "exactly one physical Jump transition and at most one GT notification per edge");

	// A slow GT reader races with publication, never participates in input.
	auto ConcurrentSource = std::make_shared<FDeviceInputProducer>(31);
	FInputStream Concurrent(ConcurrentSource);
	std::atomic<bool> Complete{false}, Coherent{true};
	std::thread Reader([&]
	{
		while (!Complete.load())
		{
			if (auto P = Concurrent.ReadLatest())
				if (P->Frame.GetSourceFrame() != P->Frame.GetConsumptionFrame()
					|| P->Frame.GetActions()[Throttle] != static_cast<std::int16_t>(P->Frame.GetSourceFrame())) Coherent.store(false);
		}
	});
	for (FFrameNumber I = 0; I < 100; ++I)
	{
		Check(ConcurrentSource->SetAction(I, Throttle, static_cast<std::int16_t>(I), false), "concurrent source update");
		Check(bool(Concurrent.Consume(I)) && Concurrent.PublishCompleted(I), "concurrent complete publication");
	}
	Complete.store(true); Reader.join();
	Check(Coherent.load(), "coherent publication across threads");
	FDeviceInputProducer Reference(31);
	for (FFrameNumber I = 0; I < 100; ++I)
	{
		Reference.SetAction(I, Throttle, static_cast<std::int16_t>(I), false);
		Check(Hash(*Reference.Produce(I)) == Hash(*Concurrent.ReadRecorded(I)), "polling independent physical history hashes");
	}
	std::weak_ptr<IInputProducer> Lifetime = Source;
	Source.reset();
	Check(!Lifetime.expired(), "shared stream retains values source after controller release");
	Stream.reset();
	Check(Lifetime.expired(), "source freed after final worker handle");
	std::cout << "PASS InputProducerProbe checks=" << Checks << " first_hash=" << FirstHash << '\n';
	return 0;
}
