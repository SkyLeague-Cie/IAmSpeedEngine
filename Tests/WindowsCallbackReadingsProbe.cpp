#include "WindowsDiscoveryFakes.h"
#include "IAmSpeed/Input/Windows/GameInputCallbackReadings.h"
#include "IAmSpeed/Input/Windows/GameInputSelectedSource.h"
#include "IAmSpeed/Input/Windows/GameInputRawAcquisition.h"

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
		++CurrentCalls; std::lock_guard<std::mutex> Lock(ReadingGate);
		*Out = History.empty() ? nullptr : History.back().Get();
		if (*Out) { (*Out)->AddRef(); return S_OK; }
		return GAMEINPUT_E_READING_NOT_FOUND;
	}
	HRESULT STDMETHODCALLTYPE GetNextReading(IGameInputReading* Previous, GameInputKind, IGameInputDevice*, IGameInputReading** Out) override
	{
		++NextCalls; std::lock_guard<std::mutex> Lock(ReadingGate); *Out = nullptr;
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
		&& First.FreshBaseline && First.Count == 3
		&& !First.States[0].GamepadButtons && First.States[1].GamepadButtons
		&& !First.States[2].GamepadButtons, "registration gap bridged; equal timestamps remain ordered");
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
		&& Overflow.Count == 0, "bounded queue overflow is an explicit gap");
	Check(Cursor.Stop(), "overflow teardown is idempotent");
	ComPtr<CallbackApi> Duplicated; Duplicated.Attach(new CallbackApi);
	Duplicated->Push(false, 400);
	Duplicated->AfterRegistration = [&] { Duplicated->Push(true, 401); Duplicated->Push(false, 401); };
	FGameInputCallbackReadCursor DuplicateCursor;
	const auto Bridged = DuplicateCursor.PollRaw(*Duplicated.Get(), Duplicated->Pad.Get(), GameInputKindGamepad);
	const auto Duplicates = DuplicateCursor.PollRaw(*Duplicated.Get(), Duplicated->Pad.Get(), GameInputKindGamepad);
	Check(Bridged.Count == 3 && Duplicates.Count == 0
		&& Duplicates.Result.Status == Speed::Input::Windows::EReadBatchStatus::NoChange,
		"bridge and callback expose each COM reading only once");
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
	Check(Owner.Close(), "callback registration closed with acquisition owner");
	std::cout << "PASS WindowsCallbackReadingsProbe checks=" << Checks << " hardware=none sdk=v3\n";
}
