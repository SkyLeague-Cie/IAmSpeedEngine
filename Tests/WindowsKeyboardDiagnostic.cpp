// Explicitly launched diagnostic only; never linked into a game/Unreal module.
#define NOMINMAX
#include <Windows.h>
#include <GameInput.h>
#include <wrl/client.h>
#include "KeyboardDiagnosticModel.h"
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
template<class T> using ComPtr = Microsoft::WRL::ComPtr<T>;
using Clock = std::chrono::steady_clock;
static constexpr UINT CueMessage = WM_APP + 1, DoneMessage = WM_APP + 2;
static HWND Window = nullptr, Label = nullptr, StartButton = nullptr;
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
	ComPtr<IGameInputReading> Previous;
	bool Failed = false, Protocol = false;
	try
	{
		const auto Hr = GameInputCreate(Api.GetAddressOf());
		Log("{\"type\":\"init\",\"hr\":" + std::to_string(Hr) + ",\"sdk\":3}");
		if (FAILED(Hr) || !Api) Failed = true;
		if (!Failed)
		{
			Api->SetFocusPolicy(Background ? GameInputEnableBackgroundInput : GameInputDefaultFocusPolicy);
			// Factory cost is outside the capture clock. Start is local, after
			// initialization, with a displayed rest phase before any requested press.
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
				if (Now == 0 || Focused != LastFocus)
					Log("{\"type\":\"focus\",\"us\":" + std::to_string(Now) + ",\"foreground\":" + (Focused ? "true" : "false") + "}");
				LastFocus = Focused;
				ComPtr<IGameInputReading> Reading;
				const auto ReadHr = Api->GetCurrentReading(GameInputKindKeyboard, nullptr, Reading.GetAddressOf());
				if (ReadHr != GAMEINPUT_E_READING_NOT_FOUND)
				{
					if (FAILED(ReadHr) || !Reading)
					{
						Log("{\"type\":\"read_error\",\"hr\":" + std::to_string(ReadHr) + "}"); Failed = true;
					}
					else
					{
						std::array<GameInputKeyState, 256> Keys{};
						const auto Count = Reading->GetKeyCount();
						const auto Read = Reading->GetKeyState(static_cast<std::uint32_t>(Keys.size()), Keys.data());
						if (Count > Keys.size() || Read != Count) Failed = true;
						else
						{
							bool F9 = false;
							for (std::uint32_t K = 0; K < Count; ++K) F9 = F9 || Keys[K].virtualKey == VK_F9;
							const auto Observation = Model.Observe(Reading->GetTimestamp(), F9, Reading.Get() == Previous.Get());
							if (Observation == EObservation::Invalid) Failed = true;
							else if (Observation != EObservation::Unchanged)
								Log("{\"type\":\"f9\",\"us\":" + std::to_string(Now) + ",\"os_us\":" + std::to_string(Reading->GetTimestamp()) + ",\"f9\":" + (F9 ? "true" : "false") + "}");
							Cues.Observe(F9, Focused);
						}
						Previous = std::move(Reading); // Retain identity, not just its timestamp.
					}
				}
				if (Now >= NextHeartbeat)
				{
					Log("{\"type\":\"heartbeat\",\"us\":" + std::to_string(Now) + ",\"api_us\":" + std::to_string(Api->GetCurrentTimestamp()) + ",\"samples\":" + std::to_string(Model.Samples) + ",\"duplicates\":" + std::to_string(Model.Duplicates) + "}");
					NextHeartbeat = Now + 1000000;
				}
				if (Cues.IsFailed()) Failed = true;
				if (LastAck == static_cast<int>(KeyboardCues.size()) - 1)
				{ Protocol = Cues.Complete() && Model.Transitions >= 4; break; }
				if (Now > 31000000) Failed = true;
				std::this_thread::sleep_for(std::chrono::milliseconds(5));
			}
		}
	}
	catch (...) { Failed = true; Log("{\"type\":\"exception\"}"); }
	Model.Stop(); Previous.Reset(); Api.Reset(); // No callbacks exist in this probe.
	Result.store(!Failed && Protocol && !Stop.load() ? 0 : 6);
	Log("{\"type\":\"worker_end\",\"protocol_pass\":" + std::string(Result.load() == 0 ? "true" : "false")
		+ ",\"reading_released\":true,\"api_released\":true,\"transitions\":" + std::to_string(Model.Transitions) + "}");
	if (!PostMessageW(Window, DoneMessage, 0, 0)) { Result.store(6); Stop.store(true); }
}

static LRESULT CALLBACK Procedure(HWND H, UINT Message, WPARAM W, LPARAM L)
{
	switch (Message)
	{
	case WM_COMMAND:
		if (LOWORD(W) == 1 && !Started.exchange(true))
		{
			EnableWindow(StartButton, FALSE);
			SetWindowTextW(Label, L"Initialisation. Relachez F9 et attendez les consignes ici.");
			try { Worker = std::thread(Acquire); }
			catch (...) { Result.store(6); DestroyWindow(H); }
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
		if (!Started.load()) DestroyWindow(H);
		else { Stop.store(true); SetWindowTextW(Label, L"Arret en cours..."); }
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
		|| (std::strcmp(Argv[2], "--foreground") != 0 && std::strcmp(Argv[2], "--background") != 0))
	{ std::cout << "Explicit reviewed hardware launch required: --hardware --foreground|--background\n"; return 2; }
	Background = std::strcmp(Argv[2], "--background") == 0;
	Log("{\"type\":\"config\",\"background_policy\":" + std::string(Background ? "true" : "false")
		+ ",\"filter\":\"keyboard_any_device\",\"com_initialization\":\"none_nano_com\",\"ui_pump\":true,\"poll_thread\":\"worker\"}");
	const auto Instance = GetModuleHandleW(nullptr);
	WNDCLASSW Class{}; Class.lpfnWndProc = Procedure; Class.hInstance = Instance;
	Class.lpszClassName = L"KeyboardDiagnosticWindow"; Class.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
	Class.hCursor = LoadCursorW(nullptr, IDC_ARROW);
	if (!RegisterClassW(&Class)) return 3;
	Window = CreateWindowExW(0, Class.lpszClassName, L"Diagnostic clavier F9 - 30 secondes", WS_OVERLAPPEDWINDOW,
		CW_USEDEFAULT, CW_USEDEFAULT, 740, 230, nullptr, nullptr, Instance, nullptr);
	if (!Window) { UnregisterClassW(Class.lpszClassName, Instance); return 3; }
	Label = CreateWindowExW(0, L"STATIC", L"Relachez F9. Cliquez Demarrer, puis suivez les consignes locales.", WS_CHILD | WS_VISIBLE,
		20, 25, 680, 70, Window, nullptr, Instance, nullptr);
	StartButton = CreateWindowExW(0, L"BUTTON", L"Demarrer", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
		20, 110, 160, 40, Window, reinterpret_cast<HMENU>(1), Instance, nullptr);
	if (!Label || !StartButton) { DestroyWindow(Window); UnregisterClassW(Class.lpszClassName, Instance); return 3; }
	ShowWindow(Window, SW_SHOWNORMAL); UpdateWindow(Window);
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
