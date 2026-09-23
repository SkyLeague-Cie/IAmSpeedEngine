// Explicitly launched diagnostic only; never linked into a game/Unreal module.
#define NOMINMAX
#include <Windows.h>
#include <GameInput.h>
#include <wrl/client.h>
#include "SelectedKeyboardDiagnostic.h"
#include "IAmSpeed/Input/Windows/GameInputSelectedSource.h"
#include "IAmSpeed/Input/InputStream.h"
#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>
#if GAMEINPUT_API_VERSION != 3
#error This diagnostic is reviewed against GameInput v3.
#endif
using namespace GameInput::v3;
using namespace KeyboardDiagnostic;
using namespace Speed::Input;
using namespace Speed::Input::Windows;
using SelectedKeyboardDiagnostic::FormatId;
using SelectedKeyboardDiagnostic::F9Slot;
template<class T> using ComPtr = Microsoft::WRL::ComPtr<T>;
using Clock = std::chrono::steady_clock;
static constexpr UINT CueMessage = WM_APP + 1, DoneMessage = WM_APP + 2, InventoryMessage = WM_APP + 3;
static HWND Window = nullptr, Label = nullptr, StartButton = nullptr, DeviceList = nullptr;
static std::vector<FDiscoveredDevice> Inventory;
static std::mutex InventoryMutex;
static std::atomic<int> Choice{-1};
static std::thread Worker;
static std::mutex OutputMutex;
static std::atomic<bool> Stop{false}, Started{false};
static std::atomic<int> AckIndex{-1}, Result{9};
static std::atomic<std::uint64_t> Epoch{0}, AckTime{0};
static bool Background = false;
static std::uint64_t MonotonicUs()
{ return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(Clock::now().time_since_epoch()).count()); }
static std::uint64_t Elapsed() { return MonotonicUs() - Epoch.load(); }
static void Log(const std::string& S)
{ std::lock_guard<std::mutex> Lock(OutputMutex); std::cout << S << '\n' << std::flush; }

static void Acquire()
{
	FReadings Model; FCues Cues;
	ComPtr<IGameInput> Api;
	std::shared_ptr<FGameInputSelectedSource> Source;
	std::unique_ptr<FInputStream> Stream;
	std::optional<FDiscoveredDevice> Chosen;
	std::optional<std::uint64_t> Generation;
	FFrameNumber FrameNumber = 0;
	std::uint64_t CurrentSuccess = 0, NextSuccess = 0, Ordinal = 0;
	std::uint64_t LastOsTimestamp = 0;
	bool LastHeld = false, HaveHeld = false;
	bool Failed = false, Protocol = false;
	try
	{
		const auto Hr = GameInputCreate(Api.GetAddressOf());
		Log("{\"type\":\"init\",\"hr\":" + std::to_string(Hr) + ",\"sdk\":3}");
		if (FAILED(Hr) || !Api) Failed = true;
		if (!Failed)
		{
			Api->SetFocusPolicy(Background ? GameInputEnableBackgroundInput : GameInputDefaultFocusPolicy);
			std::array<bool, ActionCount> Digital{}; Digital[F9Slot] = true;
            Source = FGameInputSelectedSource::Create(Api.Get(), 901, Digital,
                [](const FDeviceState& R, FActionValues& V) { V[F9Slot] = R.VirtualKeys[VK_F9] ? 1 : 0; return true; },
                [](const FDeviceState&, FActionValues&) { return false; });
            if (!Source || !Source->EnableObservations()) throw 1;
            {
            std::lock_guard<std::mutex> Lock(InventoryMutex);
            for (const auto& D : Source->Snapshot()) if (D.Connected && (D.SupportedKinds & 1)) Inventory.push_back(D);
            Log("{\"type\":\"inventory\",\"count\":" + std::to_string(Inventory.size()) + "}");
            for (const auto& D : Inventory) Log("{\"type\":\"device\",\"id\":\"" + FormatId(D.Id) + "\",\"revision\":" + std::to_string(D.Revision) + "}");
            if (Inventory.empty()) throw 1;
            }
            if (!PostMessageW(Window, InventoryMessage, 0, 0)) throw 1;
            while (Choice.load(std::memory_order_acquire) < 0 && !Stop.load()) std::this_thread::sleep_for(std::chrono::milliseconds(5));
            if (Stop.load()) throw 1;
            {
            std::lock_guard<std::mutex> Lock(InventoryMutex);
            const auto Index = static_cast<std::size_t>(Choice.load());
            if (Index >= Inventory.size()) throw 1;
            Chosen = Inventory[Index];
            }
            if (!SelectedKeyboardDiagnostic::Eligible(Source->Snapshot(), Chosen->Id)
                || !Source->RequestSelection(std::make_pair(Chosen->Id, EDeviceKind::Keyboard))) throw 1;
            Log("{\"type\":\"selection\",\"id\":\"" + FormatId(Chosen->Id) + "\",\"revision\":" + std::to_string(Chosen->Revision) + "}");
            Stream = std::make_unique<FInputStream>(Source);
			Epoch.store(MonotonicUs());
			int LastAck = -1; bool LastFocus = false; std::uint64_t NextHeartbeat = 0;
			while (!Stop.load() && !Failed)
			{
				const auto Now = Elapsed();
				const int Ack = AckIndex.load(std::memory_order_acquire);
				if (Ack != LastAck)
				{
					if (Ack < 0 || !Cues.Acknowledge(static_cast<unsigned>(Ack), AckTime.load())) Failed = true;
					LastAck = Ack;
				}
				const auto Cue = Cues.Advance(Now);
				if (Cue)
				{
					Log("{\"type\":\"cue_post\",\"index\":" + std::to_string(*Cue) + ",\"due_us\":" + std::to_string(KeyboardCues[*Cue].AtUs) + ",\"us\":" + std::to_string(Now) + "}");
					if (!PostMessageW(Window, CueMessage, *Cue, 0)) Failed = true;
				}
				if (Cues.IsFailed()) Failed = true;
				const bool Focused = GetForegroundWindow() == Window;
				Model.Focus(Focused);
				Cues.ObserveFocus(Focused); // Also applies to READING_NOT_FOUND below.
				if (Now == 0 || Focused != LastFocus)
					Log("{\"type\":\"focus\",\"us\":" + std::to_string(Now) + ",\"foreground\":" + (Focused ? "true" : "false") + "}");
				LastFocus = Focused;
                const auto Frame = Stream->Consume(FrameNumber);
                const auto O = Source->ReadObservation();
                const auto Status = Source->GetLastPollStatus();
                if (!Frame || !O || O->Frame != FrameNumber || !O->AcquisitionTicket
                    || O->AcquisitionTicket->Device.Id != Chosen->Id
                    || O->AcquisitionTicket->Device.Revision != Chosen->Revision
                    || (Status != EPollStatus::Updated && Status != EPollStatus::NoChange)
                    || O->SubmitRejected) throw 1;
                const auto G = O->AcquisitionTicket->Generation;
                if (!G || (Generation && *Generation != G)) throw 1;
                Generation = G;
                Log("{\"type\":\"poll\",\"frame\":" + std::to_string(FrameNumber)
                    + ",\"ticket_id\":\"" + FormatId(O->AcquisitionTicket->Device.Id)
                    + "\",\"revision\":" + std::to_string(O->AcquisitionTicket->Device.Revision)
                    + ",\"attempted_generation\":" + std::to_string(G)
                    + ",\"committed_generation\":" + (O->SubmitAccepted ? std::to_string(G) : "null")
                    + ",\"submit_accepted\":" + std::to_string(O->SubmitAccepted)
                    + ",\"submit_rejected\":" + std::to_string(O->SubmitRejected)
                    + ",\"status\":" + std::to_string(static_cast<int>(Status))
                    + ",\"hr\":" + std::to_string(Source->GetLastError()) + "}");
                for (std::size_t I=0; I<O->Readings.Count; ++I) {
                    const auto& R=O->Readings.Calls[I];
                    if (R.HasReading && (!R.HasDeviceId || R.DeviceId != Chosen->Id || !R.IdentityValid || R.SamePrevious)) throw 1;
                    if (R.HasReading) { ++Ordinal; LastOsTimestamp=R.Timestamp; if(R.Current) ++CurrentSuccess; else ++NextSuccess; }
                    Log("{\"type\":\"reading\",\"frame\":" + std::to_string(FrameNumber)
                        + ",\"current\":" + (R.Current ? "true":"false") + ",\"hr\":" + std::to_string(R.Result)
                        + ",\"returned\":" + (R.HasReading ? "true":"false")
                        + ",\"ordinal\":" + std::to_string(Ordinal) + ",\"os_us\":" + std::to_string(R.Timestamp)
                        + ",\"observed_id\":\"" + (R.HasDeviceId ? FormatId(R.DeviceId) : "")
                        + "\",\"identity_valid\":" + (R.IdentityValid ? "true":"false")
                        + ",\"same_previous\":" + (R.SamePrevious ? "true":"false") + "}");
                }
                // Diagnostic simulated completion; never a gameplay physics claim.
                if (!Stream->PublishCompleted(FrameNumber)) throw 1;
                Log(SelectedKeyboardDiagnostic::FrameJson(*Frame, Chosen->Id, 0));
                const bool Held = Frame->GetActions()[F9Slot] != 0;
                if (Ordinal) {
                    if (!HaveHeld || Held != LastHeld) {
                        if (HaveHeld) ++Model.Transitions;
                        ++Model.Samples; HaveHeld=true; LastHeld=Held;
                        Log("{\"type\":\"f9\",\"us\":" + std::to_string(Now) + ",\"os_us\":" + std::to_string(LastOsTimestamp) + ",\"f9\":" + (Held ? "true":"false") + "}");
                    }
                    Cues.Observe(Held, Focused);
                }
                ++FrameNumber;
				if (Now >= NextHeartbeat)
				{
					Log("{\"type\":\"heartbeat\",\"us\":" + std::to_string(Now) + ",\"api_us\":" + std::to_string(Api->GetCurrentTimestamp()) + ",\"samples\":" + std::to_string(Model.Samples) + ",\"duplicates\":" + std::to_string(Model.Duplicates) + "}");
					NextHeartbeat = Now + 1000000;
				}
				if (Cues.IsFailed()) Failed = true;
				if (LastAck == static_cast<int>(KeyboardCues.size()) - 1)
				{ Protocol = Cues.Complete() && Model.Transitions >= 4 && CurrentSuccess >= 1 && NextSuccess >= 1; break; }
				if (Now > 31000000) Failed = true;
				std::this_thread::sleep_for(std::chrono::milliseconds(5));
			}
		}
	}
	catch (...) { Failed = true; Log("{\"type\":\"exception\"}"); }
	Model.Stop();
    Stream.reset();
    if (Source && !Source->Shutdown()) { Failed=true; Log("{\"type\":\"shutdown_failure\"}"); std::terminate(); }
    Source.reset(); Api.Reset();
	Result.store(!Failed && Protocol && !Stop.load() ? 0 : 6);
	Log("{\"type\":\"worker_end\",\"protocol_pass\":" + std::string(Result.load() == 0 ? "true" : "false")
		+ ",\"reading_released\":true,\"api_released\":true,\"transitions\":" + std::to_string(Model.Transitions) + "}");
	if (!PostMessageW(Window, DoneMessage, 0, 0)) { Result.store(6); Stop.store(true); }
}

static LRESULT CALLBACK Procedure(HWND H, UINT Message, WPARAM W, LPARAM L)
{
	switch (Message)
	{
    case InventoryMessage:
    {
        std::lock_guard<std::mutex> Lock(InventoryMutex);
        for (const auto& D : Inventory) {
            const std::string Text=FormatId(D.Id)+" revision="+std::to_string(D.Revision);
            const std::wstring Wide(Text.begin(),Text.end());
            SendMessageW(DeviceList,LB_ADDSTRING,0,reinterpret_cast<LPARAM>(Wide.c_str()));
        }
        SetWindowTextW(Label,L"Choisissez explicitement une ligne clavier, puis cliquez Demarrer.");
        return 0;
    }
    case WM_COMMAND:
        if (LOWORD(W)==2 && HIWORD(W)==LBN_SELCHANGE) EnableWindow(StartButton,TRUE);
        if (LOWORD(W)==1 && !Started.load()) {
            std::lock_guard<std::mutex> Lock(InventoryMutex);
            const auto Index=SendMessageW(DeviceList,LB_GETCURSEL,0,0);
            if(Index==LB_ERR || Index<0 || static_cast<std::size_t>(Index)>=Inventory.size()) return 0;
            Started.store(true); EnableWindow(StartButton,FALSE); EnableWindow(DeviceList,FALSE);
            Choice.store(static_cast<int>(Index),std::memory_order_release);
        }
        return 0;
	case CueMessage:
	{
		if (W >= KeyboardCues.size()) { Stop.store(true); return 0; }
		SetWindowTextW(Label, KeyboardCues[W].Text);
		if (!RedrawWindow(Label, nullptr, nullptr, RDW_INVALIDATE | RDW_UPDATENOW)) { Stop.store(true); return 0; }
		const auto Displayed = Elapsed();
		const bool SoundRequested = MessageBeep(MB_OK) != FALSE; // Advisory, visual cue is authoritative.
		Log("{\"type\":\"cue_display\",\"index\":" + std::to_string(W) + ",\"us\":" + std::to_string(Displayed) + ",\"sound_requested\":" + (SoundRequested ? "true" : "false") + "}");
		AckTime.store(Displayed); AckIndex.store(static_cast<int>(W), std::memory_order_release);
		return 0;
	}
	case WM_CLOSE:
		Stop.store(true); SetWindowTextW(Label, L"Arret en cours...");
		return 0;
	case DoneMessage: DestroyWindow(H); return 0;
	case WM_DESTROY: PostQuitMessage(0); return 0;
	default: return DefWindowProcW(H, Message, W, L);
	}
}

int main(int Argc, char** Argv)
{
	// A build/test invocation cannot accidentally initialize devices.
	if (Argc != 3 || std::strcmp(Argv[1], "--hardware") != 0
		|| std::strcmp(Argv[2], "--foreground") != 0)
	{ std::cout << "Explicit reviewed hardware launch required: --hardware --foreground\n"; return 2; }
	Background = std::strcmp(Argv[2], "--background") == 0;
	Log("{\"type\":\"config\",\"background_policy\":" + std::string(Background ? "true" : "false")
		+ ",\"filter\":\"keyboard_selected_device\",\"com_initialization\":\"none_nano_com\",\"ui_pump\":true,\"poll_thread\":\"worker\"}");
	const auto Instance = GetModuleHandleW(nullptr);
	WNDCLASSW Class{}; Class.lpfnWndProc = Procedure; Class.hInstance = Instance;
	Class.lpszClassName = L"SelectedKeyboardDiagnosticWindow"; Class.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
	Class.hCursor = LoadCursorW(nullptr, MAKEINTRESOURCEW(32512));
	if (!RegisterClassW(&Class)) return 3;
	Window = CreateWindowExW(0, Class.lpszClassName, L"Diagnostic clavier F9 - 30 secondes", WS_OVERLAPPEDWINDOW,
		CW_USEDEFAULT, CW_USEDEFAULT, 1000, 360, nullptr, nullptr, Instance, nullptr);
	if (!Window) { UnregisterClassW(Class.lpszClassName, Instance); return 3; }
	Label = CreateWindowExW(0, L"STATIC", L"Enumeration des claviers. Attendez la liste puis choisissez une ligne.", WS_CHILD | WS_VISIBLE,
		20, 25, 940, 70, Window, nullptr, Instance, nullptr);
	StartButton = CreateWindowExW(0, L"BUTTON", L"Demarrer", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
		20, 240, 160, 40, Window, reinterpret_cast<HMENU>(1), Instance, nullptr);
	if (!Label || !StartButton) { DestroyWindow(Window); UnregisterClassW(Class.lpszClassName, Instance); return 3; }
    DeviceList=CreateWindowExW(0,L"LISTBOX",nullptr,WS_CHILD|WS_VISIBLE|WS_BORDER|LBS_NOTIFY|WS_VSCROLL,
        20,100,940,120,Window,reinterpret_cast<HMENU>(2),Instance,nullptr);
    if (!DeviceList) { DestroyWindow(Window); return 3; }
    EnableWindow(StartButton,FALSE);
	ShowWindow(Window, SW_SHOWNORMAL); UpdateWindow(Window);
	try { Worker=std::thread(Acquire); } catch (...) { DestroyWindow(Window); return 6; }
	MSG Message{}; BOOL Got = 0;
	while ((Got = GetMessageW(&Message, nullptr, 0, 0)) > 0) { TranslateMessage(&Message); DispatchMessageW(&Message); }
	Stop.store(true);
	if (Worker.joinable()) Worker.join();
	const bool WindowGone = !IsWindow(Window);
	if (!WindowGone) DestroyWindow(Window);
	UnregisterClassW(Class.lpszClassName, Instance);
	Log("{\"type\":\"end\",\"worker_joined\":true,\"window_destroyed\":" + std::string(!IsWindow(Window) ? "true" : "false") + ",\"exit_code\":" + std::to_string(Result.load()) + "}");
	return Got < 0 || !WindowGone ? 6 : Result.load();
}
