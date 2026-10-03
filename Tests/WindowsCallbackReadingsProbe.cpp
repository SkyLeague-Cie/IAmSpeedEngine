#include "WindowsDiscoveryFakes.h"
#include "IAmSpeed/Input/Windows/GameInputCallbackReadings.h"
#include "IAmSpeed/Input/Windows/GameInputSelectedSource.h"
#include "IAmSpeed/Input/Windows/GameInputRawAcquisition.h"
#include "IAmSpeed/Input/ActionMapping.h"

using Speed::Input::Windows::FGameInputCallbackReadCursor;

class CallbackApi final : public DiscoveryApi
{
public:
	ComPtr<FakeDevice> Pad;
	std::deque<ComPtr<FakeReading>> History;
	std::size_t HistoryLimit = 256;
	GameInputReadingCallback ReadingCallback = nullptr;
	void* ReadingContext = nullptr;
	std::function<void()> BeforeRegistration;
	std::function<void()> AfterRegistration;
	std::function<void()> DuringCurrent;
	bool ForceOldHistory = false;
	bool FailReadingUnregister = false;
	std::mutex ReadingGate;
	std::condition_variable ReadingFinished;
	unsigned ReadingRunning = 0;
	CallbackApi() { Pad = MakeDevice(1); Initial = {{Pad, 1, true}}; }
	void Push(bool Held, std::uint64_t Stamp)
	{
		ComPtr<FakeReading> Reading; Reading.Attach(new FakeReading);
		Reading->Pad.buttons = Held ? GameInputGamepadA : GameInputGamepadNone;
		Reading->Stamp = Stamp; Reading->ObservedDevice = Pad;
		GameInputReadingCallback Function = nullptr; void* ReadingData = nullptr;
		{
			std::lock_guard<std::mutex> Lock(ReadingGate);
			History.push_back(Reading);
			while (History.size() > HistoryLimit) History.pop_front();
			Function = ReadingCallback; ReadingData = ReadingContext;
			if (Function) ++ReadingRunning;
		}
		if (Function)
		{
			Function(2, ReadingData, Reading.Get());
			std::lock_guard<std::mutex> Lock(ReadingGate);
			--ReadingRunning; ReadingFinished.notify_all();
		}
	}
	HRESULT STDMETHODCALLTYPE GetCurrentReading(GameInputKind, IGameInputDevice*, IGameInputReading** Out) override
	{
		++CurrentCalls;
		if (DuringCurrent) { auto Hook = std::move(DuringCurrent); DuringCurrent = {}; Hook(); }
		std::lock_guard<std::mutex> Lock(ReadingGate);
		*Out = History.empty() ? nullptr : History.back().Get();
		if (*Out) { (*Out)->AddRef(); return S_OK; }
		return GAMEINPUT_E_READING_NOT_FOUND;
	}
	HRESULT STDMETHODCALLTYPE GetNextReading(IGameInputReading* Previous, GameInputKind, IGameInputDevice*, IGameInputReading** Out) override
	{
		++NextCalls; std::lock_guard<std::mutex> Lock(ReadingGate); *Out = nullptr;
		if (ForceOldHistory) return GAMEINPUT_E_REFERENCE_READING_TOO_OLD;
		for (std::size_t I = 0; I < History.size(); ++I)
			if (History[I].Get() == Previous)
			{
				if (I + 1 == History.size()) return GAMEINPUT_E_READING_NOT_FOUND;
				*Out = History[I + 1].Get(); (*Out)->AddRef(); return S_OK;
			}
		return GAMEINPUT_E_REFERENCE_READING_TOO_OLD;
	}
	HRESULT STDMETHODCALLTYPE RegisterReadingCallback(IGameInputDevice* Device, GameInputKind Kind,
		void* ReadingData, GameInputReadingCallback Function, GameInputCallbackToken* Token) override
	{
		if (Device != Pad.Get() || Kind != GameInputKindGamepad) return E_INVALIDARG;
		if (BeforeRegistration) { auto Hook = std::move(BeforeRegistration); BeforeRegistration = {}; Hook(); }
		{
			std::lock_guard<std::mutex> Lock(ReadingGate);
			ReadingContext = ReadingData; ReadingCallback = Function; *Token = 2;
		}
		if (AfterRegistration) { auto Hook = std::move(AfterRegistration); AfterRegistration = {}; Hook(); }
		return S_OK;
	}
	bool STDMETHODCALLTYPE UnregisterCallback(GameInputCallbackToken Token) override
	{
		if (Token != 2) return FakeApi::UnregisterCallback(Token);
		if (FailReadingUnregister) return false;
		std::unique_lock<std::mutex> Lock(ReadingGate);
		ReadingCallback = nullptr; ReadingContext = nullptr;
		ReadingFinished.wait(Lock, [&] { return ReadingRunning == 0; });
		return true;
	}
};

int main()
{
	ComPtr<CallbackApi> Api; Api.Attach(new CallbackApi);
	Api->Push(false, 100);
	FGameInputCallbackReadCursor Cursor;
	Api->BeforeRegistration = [&] { Api->Push(true, 101); Api->Push(false, 101); };
	auto First = Cursor.PollRaw(*Api.Get(), Api->Pad.Get(), GameInputKindGamepad);
	Check(First.Result.Status == Speed::Input::Windows::EReadBatchStatus::Updated
		&& First.FreshBaseline && First.Count == 1
		&& !First.States[0].GamepadButtons && Api->NextCalls == 0,
		"initial snapshot admits state, not pre-subscription history");
	Api->HistoryLimit = 2;
	Api->Push(true, 102); Api->Push(false, 102); Api->Push(true, 103);
	auto Second = Cursor.PollRaw(*Api.Get(), Api->Pad.Get(), GameInputKindGamepad);
	Check(Second.Count == 3 && !Second.FreshBaseline && Second.States[0].GamepadButtons
		&& !Second.States[1].GamepadButtons && Second.States[2].GamepadButtons,
		"callback preserves three states even after GameInput history eviction");
	Check(Cursor.Stop(), "unregister callback fences queue");
	Api->Push(false, 104);
	auto Resume = Cursor.PollRaw(*Api.Get(), Api->Pad.Get(), GameInputKindGamepad);
	Check(Resume.Count == 1 && Resume.FreshBaseline && !Resume.States[0].GamepadButtons,
		"fresh baseline after pause discards old edges");
	for (unsigned I = 0; I < 257; ++I) Api->Push((I & 1) != 0, 105 + I);
	auto Overflow = Cursor.PollRaw(*Api.Get(), Api->Pad.Get(), GameInputKindGamepad);
	Check(Overflow.Result.Status == Speed::Input::Windows::EReadBatchStatus::Resynchronize
		&& Overflow.Count == 0 && Overflow.Reject == Speed::Input::Windows::ERawReadReject::QueueOverflow, "bounded queue overflow is an explicit gap with exact reason");
	Check(Cursor.Stop(), "overflow teardown is idempotent");
    ComPtr<CallbackApi> Capacity; Capacity.Attach(new CallbackApi);
    Capacity->Push(false, 1);
    Capacity->AfterRegistration = [&] { for(unsigned I=0; I<65; ++I) Capacity->Push((I&1)!=0, 2+I); };
    FGameInputCallbackReadCursor CapacityCursor;
    const auto Full = CapacityCursor.PollRaw(*Capacity.Get(), Capacity->Pad.Get(), GameInputKindGamepad);
    const auto Remainder = CapacityCursor.PollRaw(*Capacity.Get(), Capacity->Pad.Get(), GameInputKindGamepad);
    Check(Full.Result.Status == Speed::Input::Windows::EReadBatchStatus::Updated
        && Full.Count == 64 && Full.FreshBaseline && Remainder.Count == 1
        && !Remainder.FreshBaseline && Remainder.States[0].TimestampMicroseconds == 66,
        "initial callback queue is bounded per batch without dropping its remainder");
    Check(CapacityCursor.Stop(), "capacity diagnostic teardown");
    ComPtr<CallbackApi> InvalidPad; InvalidPad.Attach(new CallbackApi);
    InvalidPad->Push(false, 1000); InvalidPad->History.back()->ValidPad = false;
    FGameInputCallbackReadCursor DecodeCursor;
    const auto Decode = DecodeCursor.PollRaw(*InvalidPad.Get(), InvalidPad->Pad.Get(), GameInputKindGamepad);
    Check(Decode.Result.Status == Speed::Input::Windows::EReadBatchStatus::Resynchronize
        && Decode.Count == 0 && Decode.Reject == Speed::Input::Windows::ERawReadReject::GamepadDecode,
        "failed native gamepad conversion has distinct diagnostic and no partial state");
    Check(DecodeCursor.Stop(), "decode diagnostic teardown");


	ComPtr<CallbackApi> Duplicated; Duplicated.Attach(new CallbackApi);
	Duplicated->Push(false, 400);
	Duplicated->AfterRegistration = [&] { Duplicated->Push(true, 401); Duplicated->Push(false, 401); };
	FGameInputCallbackReadCursor DuplicateCursor;
	const auto Bridged = DuplicateCursor.PollRaw(*Duplicated.Get(), Duplicated->Pad.Get(), GameInputKindGamepad);
	const auto Duplicates = DuplicateCursor.PollRaw(*Duplicated.Get(), Duplicated->Pad.Get(), GameInputKindGamepad);
	Check(Bridged.Count == 2 && Duplicates.Count == 0
		&& Duplicates.Result.Status == Speed::Input::Windows::EReadBatchStatus::NoChange,
		"earliest callback baseline retains equal timestamp order without double consumption");
	std::thread Racing([&] { for (unsigned I = 0; I < 32; ++I) Duplicated->Push((I & 1) != 0, 402 + I); });
	Check(DuplicateCursor.Stop(), "unregister waits for concurrent callback delivery");
	Racing.join();
	ComPtr<CallbackApi> Integrated; Integrated.Attach(new CallbackApi);
	Integrated->Push(false, 500);
	auto Source = Speed::Input::Windows::FGameInputSelectedSource::CreateRaw(Integrated.Get(), 501, {}, true);
	Check(bool(Source) && Source->RequestSelection(std::make_pair(Key(1), EDeviceKind::Gamepad)),
		"callback path selected by raw acquisition source");
	auto Journal = std::make_shared<Speed::Input::V2::FRawAcquisitionJournal>(7);
	Speed::Input::Windows::FGameInputRawAcquisition Owner(std::move(Source), Journal);
	Check(Owner.Pump() == Speed::Input::V2::EAcquisitionPumpResult::Installed,
		"selected callback baseline published through journal");
	const auto Baseline = Journal->Poll(0);
	Check(Baseline && Baseline->FinalState[0].Value == 0, "initial physical snapshot neutral");
	Integrated->HistoryLimit = 2;
	Integrated->Push(true, 501); Integrated->Push(false, 501);
	Check(Owner.Pump() == Speed::Input::V2::EAcquisitionPumpResult::Installed,
		"two presses delivered after GameInput history eviction");
	const auto Physical = Journal->Poll(1);
	Check(Physical && Physical->Changes.size() == 2 && Physical->Changes[0].State.Value == 1
		&& Physical->Changes[1].State.Value == 0 && Physical->FinalState[0].Value == 0,
		"brief press and release retain two exact physical transitions");
	{
		using namespace Speed::Input::V2;
		FInputActionContractDescription Definition;
		Definition.Revision = {1}; Definition.Actions = FInputActionContract::BaseActions();
		FActionDefinition Jump; Jump.Id = 3; Jump.Owner = "Fixture"; Jump.Name = "Jump";
		Jump.Wiring = EActionWiring::Wired; Definition.Actions.push_back(Jump);
		Definition.Mapping = {{{ERawControlKind::PadButton, static_cast<std::uint16_t>(EPadButton::South)}, 3, 1}};
		Definition.Physical = {{0, EPhysicalDestination::Throttle}, {1, EPhysicalDestination::Brake},
			{2, EPhysicalDestination::Steering}};
		auto Contract = FInputActionContract::Create(Definition);
		Check(bool(Contract), "callback action contract");
		auto Mapper = FActionMapper::Create(Contract, {1}, {Speed::Input::EProducerKind::Device, 7});
		Check(bool(Mapper) && bool(Mapper->Map(*Baseline, 0).Frame), "callback baseline maps on physical frame");
		const auto Mapped = Mapper->Map(*Physical, 1);
		Check(Mapped.Frame && Mapped.Frame->GetData().Values[3] == 0
			&& Mapped.Frame->GetData().Transitions.size() == 2
			&& Mapped.Frame->GetData().Transitions[0].Action == 3
			&& Mapped.Frame->GetData().Transitions[0].State == ETransition::Started
			&& Mapped.Frame->GetData().Transitions[1].Action == 3
			&& Mapped.Frame->GetData().Transitions[1].State == ETransition::Completed,
			"callback short tap maps to Started and Completed in one physical frame");
	}
	Check(Owner.Close(), "callback registration closed with acquisition owner");
	ComPtr<CallbackApi> RetryApi; RetryApi.Attach(new CallbackApi);
	RetryApi->Push(false, 600);
	auto RetrySource = Speed::Input::Windows::FGameInputSelectedSource::CreateRaw(RetryApi.Get(), 777, {}, true);
	Check(bool(RetrySource) && RetrySource->RequestSelection(std::make_pair(Key(1), EDeviceKind::Gamepad)),
		"retry fixture selected");
	auto RetryJournal = std::make_shared<Speed::Input::V2::FRawAcquisitionJournal>(8);
	Speed::Input::Windows::FGameInputRawAcquisition RetryOwner(std::move(RetrySource), RetryJournal);
	Check(RetryOwner.Pump() == Speed::Input::V2::EAcquisitionPumpResult::Installed,
		"retry fixture has live callback");
	RetryApi->FailReadingUnregister = true;
	Check(!RetryOwner.Close(), "failed unregister retains callback context for retry");
	RetryApi->Push(true, 601);
	Check(RetryOwner.Pump() == Speed::Input::V2::EAcquisitionPumpResult::Closed,
		"failed close never admits another input reading");
	RetryApi->FailReadingUnregister = false;
	Check(RetryOwner.Close(), "second close fences callback and releases owner");
	ComPtr<CallbackApi> SinkApi; SinkApi.Attach(new CallbackApi);
	SinkApi->Push(false, 650);
	auto SinkSource = Speed::Input::Windows::FGameInputSelectedSource::CreateRaw(SinkApi.Get(), 779, {}, true);
	Check(bool(SinkSource) && SinkSource->RequestSelection(std::make_pair(Key(1), EDeviceKind::Gamepad)),
		"successful-reset fixture selected");
	Check(SinkSource->PollRaw(1, [](const auto&) noexcept { return true; }),
		"successful-reset fixture establishes callback baseline");
	SinkApi->Push(true, 651);
	Check(!SinkSource->PollRaw(2, [](const auto&) noexcept { return false; })
		&& SinkSource->GetLastPollStatus() == Speed::Input::Windows::EPollStatus::Resynchronized,
		"valid lease sink rejection resets exactly once and remains recoverable");
	SinkApi->Push(true, 652);
	Speed::Input::Windows::FGameInputSelectedSource::FSelectedRawBatch Fresh;
	Check(SinkSource->PollRaw(3, [&](const auto& Batch) noexcept { Fresh = Batch; return true; })
		&& Fresh.Readings.FreshBaseline && Fresh.Readings.Count == 1
		&& Fresh.Readings.States[0].GamepadButtons == GameInputGamepadA,
		"new callback generation accepts first fresh held reading after sink rejection");
	Check(SinkSource->Shutdown(), "successful-reset fixture closes");
	ComPtr<CallbackApi> FailureApi; FailureApi.Attach(new CallbackApi);
	FailureApi->Push(false, 700);
	auto FailureSource = Speed::Input::Windows::FGameInputSelectedSource::CreateRaw(FailureApi.Get(), 778, {}, true);
	Check(bool(FailureSource) && FailureSource->RequestSelection(std::make_pair(Key(1), EDeviceKind::Gamepad)),
		"failed-reset fixture selected");
	Check(FailureSource->PollRaw(1, [](const auto&) noexcept { return true; }),
		"failed-reset fixture establishes callback baseline");
	FailureApi->FailReadingUnregister = true;
	FailureApi->Push(true, 701);
	Check(!FailureSource->PollRaw(2, [](const auto&) noexcept { return false; })
		&& FailureSource->GetLastPollStatus() == Speed::Input::Windows::EPollStatus::Failed
		&& FAILED(FailureSource->GetLastError()),
		"failed callback reset stays fatal instead of reporting resynchronized");
	Check(!FailureSource->Shutdown(), "failed unregister retains selected-source context");
	FailureApi->FailReadingUnregister = false;
	Check(FailureSource->Shutdown(), "selected-source close retry fences callback");

    ComPtr<CallbackApi> Idle; Idle.Attach(new CallbackApi);
    Idle->Push(false, 10000); Idle->ForceOldHistory = true;
    FGameInputCallbackReadCursor IdleCursor;
    const auto IdleFirst = IdleCursor.PollRaw(*Idle.Get(), Idle->Pad.Get(), GameInputKindGamepad);
    Check(IdleFirst.FreshBaseline && IdleFirst.Count == 1 && Idle->NextCalls == 0,
        "valid idle current outside history initializes without treating history error as success");
    Idle->DuringCurrent = {}; // Initialization has completed; subsequent reads use callbacks.
    Idle->Push(true, 10001); Idle->Push(false, 10001);
    const auto Tap = IdleCursor.PollRaw(*Idle.Get(), Idle->Pad.Get(), GameInputKindGamepad);
    Check(Tap.Count == 2 && Tap.States[0].GamepadButtons && !Tap.States[1].GamepadButtons
        && Idle->NextCalls == 0, "post-admission short tap preserved despite unavailable history");
    auto RepeatIdentity = Idle->History.back();
    Idle->ReadingCallback(2, Idle->ReadingContext, RepeatIdentity.Get());
    Check(IdleCursor.PollRaw(*Idle.Get(), Idle->Pad.Get(), GameInputKindGamepad).Count == 0,
        "same callback COM identity replay produces no duplicate state");
    Idle->Push(true, 9999);
    const auto Older = IdleCursor.PollRaw(*Idle.Get(), Idle->Pad.Get(), GameInputKindGamepad);
    Check(Older.Result.Status == Speed::Input::Windows::EReadBatchStatus::Resynchronize
        && Older.Count == 0 && Older.Reject == Speed::Input::Windows::ERawReadReject::TimestampRegression,
        "distinct delayed older callback remains fail closed and never becomes an idle waiver");
    Check(IdleCursor.Stop(), "idle recovery teardown fenced");
    ComPtr<CallbackApi> During; During.Attach(new CallbackApi);
    During->Push(false, 11000);
    During->DuringCurrent = [&] { During->Push(false, 11001); During->Push(true, 11002); During->Push(false, 11002); };
    FGameInputCallbackReadCursor DuringCursor;
    const auto DuringBatch = DuringCursor.PollRaw(*During.Get(), During->Pad.Get(), GameInputKindGamepad);
    Check(DuringBatch.FreshBaseline && DuringBatch.Count == 3 && !DuringBatch.States[0].GamepadButtons
        && DuringBatch.States[1].GamepadButtons && !DuringBatch.States[2].GamepadButtons,
        "earliest queued baseline preserves short tap during current snapshot acquisition");
    Check(DuringCursor.Stop(), "snapshot race teardown fenced");
    ComPtr<CallbackApi> InitialOverflow; InitialOverflow.Attach(new CallbackApi);
    InitialOverflow->Push(false, 12000);
    InitialOverflow->AfterRegistration = [&] { for(unsigned I=0; I<257; ++I) InitialOverflow->Push((I&1)!=0, 12001+I); };
    FGameInputCallbackReadCursor InitialOverflowCursor;
    const auto LostInitial = InitialOverflowCursor.PollRaw(*InitialOverflow.Get(), InitialOverflow->Pad.Get(), GameInputKindGamepad);
    Check(LostInitial.Count == 0 && LostInitial.Reject == Speed::Input::Windows::ERawReadReject::QueueOverflow
        && LostInitial.Result.Status == Speed::Input::Windows::EReadBatchStatus::Resynchronize,
        "initial queue overflow never falls back to snapshot and hides loss");
    Check(InitialOverflowCursor.Stop(), "initial overflow teardown fenced");
    ComPtr<CallbackApi> Empty; Empty.Attach(new CallbackApi);
    FGameInputCallbackReadCursor EmptyCursor;
    const auto EmptyFirst = EmptyCursor.PollRaw(*Empty.Get(), Empty->Pad.Get(), GameInputKindGamepad);
    Check(EmptyFirst.Count == 0 && EmptyFirst.Result.Status == Speed::Input::Windows::EReadBatchStatus::NoChange
        && Empty->ReadingCallback == nullptr, "missing initial snapshot unregisters before retry");
    Empty->Push(false, 13000);
    const auto EmptyRetry = EmptyCursor.PollRaw(*Empty.Get(), Empty->Pad.Get(), GameInputKindGamepad);
    Check(EmptyRetry.FreshBaseline && EmptyRetry.Count == 1, "snapshot absence retries with fresh subscription");
    Check(EmptyCursor.Stop(), "snapshot retry teardown fenced");
	std::cout << "PASS WindowsCallbackReadingsProbe checks=" << Checks << " hardware=none sdk=v3\n";
}
