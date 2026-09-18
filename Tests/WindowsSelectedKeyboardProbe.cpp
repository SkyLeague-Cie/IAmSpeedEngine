#include "WindowsDiscoveryFakes.h"
#include "SelectedKeyboardDiagnostic.h"
#include "IAmSpeed/Input/Windows/GameInputShadowHost.h"
using namespace SelectedKeyboardDiagnostic;
using Speed::Input::Windows::FGameInputShadowHost;
using Speed::Input::Windows::FGameInputShadowConfig;

// History is addressed by retained reading identity, never timestamp equality.
class KeyboardHistoryApi final : public DiscoveryApi
{
public:
	struct FCall { FakeDevice* Device; bool Current; std::uint64_t Token; std::uint64_t Stamp; };
	std::map<IGameInputDevice*, std::vector<ComPtr<FakeReading>>> History;
	std::map<IGameInputReading*, std::uint64_t> Tokens;
	std::vector<FCall> Calls;
	std::uint64_t NextToken = 0;
	void Sample(FakeDevice* D, std::uint64_t Stamp, bool Held)
	{
		ComPtr<FakeReading> R; R.Attach(new FakeReading); R->Stamp = Stamp;
		if (Held) { GameInputKeyState K{}; K.virtualKey = VK_F9; R->Keys.push_back(K); }
		Tokens[R.Get()] = ++NextToken; History[D].push_back(R);
	}
	HRESULT Read(IGameInputDevice* D, IGameInputReading* Previous, bool Current, IGameInputReading** Out)
	{
		++ReadCalls; *Out = nullptr;
		Check(D != nullptr, "never query keyboard_any_device in selected path");
		auto& H = History[D]; FakeReading* Found = nullptr;
		HRESULT Status = GAMEINPUT_E_READING_NOT_FOUND;
		if (Current) { if (!H.empty()) Found = H.back().Get(); }
		else {
			Status = GAMEINPUT_E_REFERENCE_READING_TOO_OLD;
			for (std::size_t I = 0; I < H.size(); ++I) if (H[I].Get() == Previous) {
				Status = GAMEINPUT_E_READING_NOT_FOUND;
				if (I + 1 < H.size()) Found = H[I + 1].Get();
				break;
			}
		}
		Calls.push_back({static_cast<FakeDevice*>(D), Current, Found ? Tokens.at(Found) : 0, Found ? Found->Stamp : 0});
		if (!Found) return Status;
		*Out = Found; Found->AddRef(); return S_OK;
	}
	HRESULT STDMETHODCALLTYPE GetCurrentReading(GameInputKind Kind, IGameInputDevice* D, IGameInputReading** Out) override
	{ ++CurrentCalls; Check(Kind == GameInputKindKeyboard, "selected keyboard kind"); return Read(D, nullptr, true, Out); }
	HRESULT STDMETHODCALLTYPE GetNextReading(IGameInputReading* Previous, GameInputKind Kind, IGameInputDevice* D, IGameInputReading** Out) override
	{ ++NextCalls; Check(Kind == GameInputKindKeyboard, "next keyboard kind"); return Read(D, Previous, false, Out); }
};
struct Fixture
{
	ComPtr<KeyboardHistoryApi> Api;
	ComPtr<FakeDevice> A, B;
	std::unique_ptr<FGameInputShadowHost> Host;
	std::uint64_t Epoch = 0;
	FDeviceId Requested = Key(1);
	Fixture()
	{
		Api.Attach(new KeyboardHistoryApi); A = MakeDevice(1, GameInputKindKeyboard); B = MakeDevice(2, GameInputKindKeyboard);
		Api->Initial = {{A, 1, true}, {B, 1, true}};
		FGameInputShadowConfig C; C.Enabled = true; C.ProducerId = 801; C.Digital[F9Slot] = true;
		C.Keyboard = [](const FDeviceState& S, FActionValues& V) { V[F9Slot] = S.VirtualKeys[VK_F9] ? 1 : 0; return true; };
		C.Gamepad = [](const FDeviceState&, FActionValues&) { return false; }; // No accidental gamepad mapping.
		Host = FGameInputShadowHost::Create(C, [&]() -> ComPtr<IGameInput> { return Api; });
		Check(Host && Host->BindOwnerThread() && Host->RequestSelection(std::make_pair(Requested, EDeviceKind::Keyboard)), "selected keyboard diagnostic setup");
	}
	FInputFrame Frame(FFrameNumber N)
	{
		auto R = Host->BeginFrame(N); Check(R.has_value() && !Host->ReadRecorded(N), "pending remains private");
		Check(Host->CompleteFrame(N), "diagnostic completion after simulated frame boundary");
		const auto Copy = Host->ReadRecorded(N); Check(Copy.has_value(), "completed copy available");
		std::cout << FrameJson(*Copy, Requested, Epoch) << '\n'; return *Copy;
	}
};
int main()
{
	Check(ParseId(FormatId(Key(1))) == Key(1), "exact ID roundtrip");
	for (const auto& S : std::array<std::string, 7>{"", "any", "first", "0x01", std::string(63,'0'), std::string(64,'z'), std::string(65,'0')})
		Check(!ParseId(S), "no implicit/partial ID selection");
	Check(Eligible({{Key(1),1,1,true}},Key(1)), "explicit connected keyboard ID");
	Check(!Eligible({{Key(1),1,2,true}},Key(1)) && !Eligible({{Key(1),1,1,false}},Key(1))
		&& !Eligible({{Key(1),1,1,true},{Key(1),1,1,true}},Key(1)), "wrong kind/disconnected/ambiguous IDs rejected");
	{
		Fixture F; F.Api->Sample(F.A.Get(),100,false); F.Api->Sample(F.B.Get(),100,true);
		Check(F.Frame(0).GetActions()[F9Slot] == 0, "other keyboard held cannot leak");
		F.Api->Sample(F.A.Get(),100,true); F.Api->Sample(F.A.Get(),100,false);
		const auto R = F.Frame(1);
		Check(R.GetEdgeCount() == 2 && R.GetEdges()[0].Kind == EEdgeKind::Start
			&& R.GetEdges()[1].Kind == EEdgeKind::Stop && R.GetActions()[F9Slot] == 0, "equal timestamps preserve press-release between polls");
		Check(R.GetEdges()[0].SourceFrame < R.GetEdges()[1].SourceFrame, "commit sequence not OS timestamp");
		std::vector<std::uint64_t> Seen;
		for (const auto& C : F.Api->Calls) { Check(C.Device == F.A.Get(), "exact selected-ID isolation"); if(C.Token) Seen.push_back(C.Token); }
		Check(Seen.size() == 3 && Seen[0] != Seen[1] && Seen[1] != Seen[2], "distinct reading objects despite equal timestamps");
		const auto Calls = F.Api->ReadCalls.load();
		Check(F.Frame(2).GetEdgeCount() == 0 && F.Api->CurrentCalls == 1 && F.Api->ReadCalls > Calls, "next end-of-history no repeated edges");
		const auto BeforeReplay = F.Api->ReadCalls.load();
		Check(F.Host->ReadRecorded(1)->GetEdgeCount() == 2 && F.Api->ReadCalls == BeforeReplay, "replay copy without OS read");
		F.Api->History[F.A.Get()].clear(); F.Api->Sample(F.A.Get(),200,true);
		const auto Lost = F.Frame(3); Check(Lost.RequiresReset() && Lost.GetActions()[F9Slot] == 0, "history loss neutral reset");
		const auto Fresh = F.Frame(4); Check(Fresh.GetActions()[F9Slot] == 1 && Fresh.GetEdgeCount() == 0 && F.Api->CurrentCalls == 2, "history recovery fresh current held without synthetic edge");
		Check(F.Host->Shutdown(), "history fixture shutdown");
	}
	{
		Fixture F; F.Api->Sample(F.A.Get(),1,false); F.Frame(0);
		F.Api->Sample(F.A.Get(),2,true); Check(F.Host->BeginFrame(1).has_value(), "pre-pause pending");
		const auto Pause = F.Host->SetPaused(true); Check(Pause.has_value(), "pause acknowledged"); F.Epoch=*Pause;
		Check(!F.Host->ReadRecorded(1) && !F.Host->CompleteFrame(1), "pause invalidates pending publication");
		F.Requested=Key(2); Check(F.Host->RequestSelection(std::make_pair(F.Requested,EDeviceKind::Keyboard)), "explicit reselection while paused");
		const auto Before=F.Api->ReadCalls.load(); Check(F.Frame(2).GetActions()[F9Slot]==0 && F.Api->ReadCalls==Before, "paused frame never reads devices");
		const auto Resume=F.Host->SetPaused(false); Check(Resume && *Resume>*Pause,"control epoch increases"); F.Epoch=*Resume;
		F.Api->Sample(F.B.Get(),3,true); const auto Held=F.Frame(3);
		Check(Held.RequiresReset() && Held.GetActions()[F9Slot]==1 && Held.GetEdgeCount()==0,"reselection resume fresh held no inherited pending edge");
		Check(F.Api->Calls.back().Device==F.B.Get(),"new selected object only"); Check(F.Host->Shutdown(),"pause fixture shutdown");
	}
	{
		// Exact production generation tickets are observable on this lower-level
		// catalogue fixture. They are not fabricated for the opaque host above.
		std::array<bool,ActionCount> Digital{}; Digital[F9Slot]=true; FDeviceDiscovery D(802,Digital);
		D.Update({Key(1),1,1,true}); D.Select(std::make_pair(Key(1),EDeviceKind::Keyboard));
		const auto Old=D.Selected(); Check(Old.has_value(),"initial generation ticket");
		D.SetPaused(true); D.SetPaused(false); const auto New=D.Selected();
		Check(New && New->Generation>Old->Generation,"actual session generation invalidated across pause");
		FActionValues V{}; V[F9Slot]=1;
		Check(!D.Submit(*Old,1,V) && D.Submit(*New,1,V),"old ticket cannot commit into new generation");
		const auto R=D.Produce(0); Check(R && R->RequiresReset() && R->GetEdgeCount()==0,"new ticket baseline no synthetic edges");
		std::cout<<FrameJson(*R,Key(1),2,New->Generation)<<'\n';
	}
	{
		FCues C;
		for(unsigned I=0;I<Cues.size();++I) { Check(C.Advance(Cues[I].AtUs)==I && C.Acknowledge(I,Cues[I].AtUs+1),"reuse accepted local cue clock/ack"); C.Observe(Cues[I].ExpectedF9,true); }
		Check(C.Complete(),"same seven-phase F9 protocol");
	}
	std::cout<<"PASS WindowsSelectedKeyboardProbe checks="<<Checks<<" hardware=none sdk=v3\n";
}
