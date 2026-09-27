#include "WindowsDiscoveryFakes.h"
#include "IAmSpeed/Input/Windows/GameInputShadowHost.h"
using Speed::Input::Windows::FGameInputShadowHost;
using Speed::Input::Windows::FGameInputShadowConfig;

static FGameInputShadowConfig Config()
{
	FGameInputShadowConfig C;
	C.Enabled = true; C.ProducerId = 701; C.Digital[3] = true;
	C.Keyboard = [](const FDeviceState& S, FActionValues& V) { V[Brake] = S.VirtualKeys['W'] ? 255 : 0; return true; };
	C.Gamepad = [](const FDeviceState& S, FActionValues& V) {
		const auto Value = QuantizeAxis(S.Axes[1], false); if (!Value) return false;
		V[Throttle] = *Value; V[3] = (S.GamepadButtons & GameInputGamepadA) ? 1 : 0; return true;
	};
	return C;
}
struct Fixture
{
	ComPtr<DiscoveryApi> Api;
	ComPtr<FakeDevice> Pad;
	unsigned FactoryCalls = 0;
	Fixture() { Api.Attach(new DiscoveryApi); Pad = MakeDevice(1); Api->Initial = {{Pad, 1, true}}; }
	FGameInputShadowHost::FBackendFactory Backend()
	{
		return [this]() -> ComPtr<IGameInput> { ++FactoryCalls; return Api; };
	}
};
int main()
{
	{
		Fixture F;
		Check(!FGameInputShadowHost::Create({}, F.Backend()) && F.FactoryCalls == 0, "default disabled never creates backend");
		auto C = Config(); C.ProducerId = 0;
		Check(!FGameInputShadowHost::Create(C, F.Backend()) && F.FactoryCalls == 0, "invalid identity before backend");
		C = Config(); C.Keyboard = {};
		Check(!FGameInputShadowHost::Create(C, F.Backend()) && F.FactoryCalls == 0, "missing mapper before backend");
		C = Config(); C.Activity = FActivityConfig{};
		Check(!FGameInputShadowHost::Create(C, F.Backend()) && F.FactoryCalls == 0, "invalid activity before backend");
		Check(!FGameInputShadowHost::Create(Config(), {}) && F.FactoryCalls == 0, "missing backend rejected");
		Check(!FGameInputShadowHost::Create(Config(), [] { return ComPtr<IGameInput>{}; }), "null backend rejected");
	}
	{
		Fixture F; auto Host = FGameInputShadowHost::Create(Config(), F.Backend());
		Check(bool(Host) && F.FactoryCalls == 1 && F.Api->ReadCalls == 0, "factory enumerates but does not poll");
		Check(!Host->BeginFrame(0) && !Host->ReadLatest(), "unbound owner cannot poll or publish");
		Check(Host->BindOwnerThread(), "explicit acquisition owner");
		Check(Host->RequestSelection(std::make_pair(Key(1), EDeviceKind::Gamepad)), "queue explicit selected device");
		F.Api->Reading(1, true, 100);
		const auto First = Host->BeginFrame(0);
		Check(First && First->GetActions()[Throttle] == 255 && First->GetEdgeCount() == 0, "fresh held diagnostic baseline");
		Check(!Host->ReadRecorded(0), "observer cannot read pending frame history");
		const auto Calls = F.Api->ReadCalls.load();
		Check(Host->BeginFrame(0).has_value() && F.Api->ReadCalls == Calls, "same pending frame no second read");
		Check(!Host->BeginFrame(1) && !Host->ReadLatest(), "one pending frame and no early publication");
		bool OtherRejected = false;
		std::thread Other([&] { OtherRejected = !Host->BindOwnerThread() && !Host->BeginFrame(0) && !Host->CompleteFrame(0); });
		Other.join(); Check(OtherRejected && F.Api->ReadCalls == Calls, "foreign owner cannot consume or publish");
		Check(!Host->CompleteFrame(1) && Host->CompleteFrame(0), "completion matches frame exactly");
		const auto Published = Host->ReadLatest();
		Check(Published && Published->Serial == 1 && Published->Frame.GetConsumptionFrame() == 0, "completed copy published");
		Check(Host->ReadRecorded(0).has_value(), "observer history exposed only after successful completion");
		Check(!Host->BeginFrame(0) && Host->ReadRecorded(0)->GetActions() == First->GetActions() && F.Api->ReadCalls == Calls, "replay observation does not repoll");
		F.Api->Reading(0, false, 101); const auto Next = Host->BeginFrame(1);
		Check(Next && Next->GetEdgeCount() == 1 && !Host->ReadLatest()->Frame.GetEdgeCount(), "later edges do not mutate completed copy");
		Check(Host->AbortFrame(1) && !Host->CompleteFrame(1) && !Host->BeginFrame(1), "aborted frame cannot be resurrected");
		Check(!Host->ReadRecorded(1), "observer cannot read aborted frame history");
		Check(Host->ReadLatest()->Serial == 1 && Published->Frame.GetActions()[Throttle] == 255, "abort and later reads preserve old immutable observation");
		F.Api->Reading(1, true, 102); Check(Host->BeginFrame(2).has_value(), "next contiguous frame after abort");
		std::optional<std::uint64_t> Paused;
		std::thread Control([&] { Paused = Host->SetPaused(true); });
		Control.join();
		Check(Paused && *Paused == 1 && !Host->CompleteFrame(2) && !Host->BeginFrame(2), "pause ack invalidates pre-control pending frame/cache");
		Check(!Host->ReadRecorded(2) && Host->ReadRecorded(0).has_value(), "pause hides invalidated frame while retaining completed history");
		const auto Before = F.Api->ReadCalls.load();
		const auto Neutral = Host->BeginFrame(3);
		Check(Neutral && Neutral->RequiresReset() && Neutral->GetActions() == FActionValues{} && F.Api->ReadCalls == Before, "paused neutral frame no API reads");
		Check(Host->CompleteFrame(3), "paused diagnostic completion");
		Check(Host->ReadRecorded(3).has_value() && !Host->ReadRecorded(1) && !Host->ReadRecorded(2), "later completion does not expose historical aborted or invalidated frames");
		const auto Resumed = Host->SetPaused(false); Check(Resumed && *Resumed == 2, "resume synchronous monotonic ack");
		F.Api->Reading(1, true, 103); const auto Held = Host->BeginFrame(4);
		Check(Held && Held->RequiresReset() && Held->GetActions()[Throttle] == 255 && Held->GetEdgeCount() == 0, "resume first fresh held no synthetic edges");
		Check(Host->CompleteFrame(4), "resume completion");
		F.Api->FailUnregister = true;
		Check(!Host->Shutdown() && !Host->BeginFrame(5) && !Host->SetPaused(false) && !Host->RequestSelection(std::nullopt), "failed shutdown closes host while retaining callback ownership");
		F.Api->FailUnregister = false;
		Check(Host->Shutdown() && Host->Shutdown() && !Host->BindOwnerThread(), "shutdown retry and idempotence");
		Check(Host->ReadLatest()->Frame.GetConsumptionFrame() == 4, "stopped observer returns retained completed copy only");
	}
	{
		Fixture F; auto Host = FGameInputShadowHost::Create(Config(), F.Backend());
		Check(Host->BindOwnerThread(), "bounded completed history owner");
		for (FFrameNumber Frame = 0; Frame <= HistoryCapacity; ++Frame)
		{
			Check(Host->BeginFrame(Frame).has_value() && !Host->ReadRecorded(Frame), "wrapped pending slot is not completed");
			Check(Host->CompleteFrame(Frame) && Host->ReadRecorded(Frame).has_value(), "completed marker matches exact frame through wrap");
		}
		Check(!Host->ReadRecorded(0) && Host->ReadRecorded(HistoryCapacity).has_value(), "evicted completed frame cannot alias new slot");
		Check(Host->Shutdown(), "bounded completed history cleanup");
	}
	{
		Fixture F; auto C = Config(); C.Activity = FActivityConfig{{.2f,.15f,.1f},{.08f,.05f,.04f},0,8,EDeviceKind::Gamepad};
		auto Host = FGameInputShadowHost::Create(C, F.Backend());
		Check(bool(Host) && Host->RequestLock(Key(1)) && !Host->RequestSelection(std::nullopt), "automatic factory preserves explicit lock policy");
		Check(Host->Shutdown(), "automatic factory cleanup");
	}
	{
		Fixture F;
		{
			FPresentationInputScope Scope;
			Check(!FGameInputShadowHost::Create(Config(), F.Backend()) && F.FactoryCalls == 0, "presentation cannot instantiate acquisition");
		}
		auto Host = FGameInputShadowHost::Create(Config(), F.Backend());
		Check(Host->BindOwnerThread(), "presentation fixture owner");
		{
			FPresentationInputScope Scope;
			Check(!Host->BeginFrame(0) && !Host->SetPaused(true) && !Host->RequestSelection(std::nullopt)
				&& !Host->RequestLock(std::nullopt) && !Host->BindOwnerThread(), "presentation rejects host mutations even on owner");
			Check(F.Api->ReadCalls == 0 && !Host->ReadLatest(), "presentation observation never starts acquisition");
		}
		Check(Host->Shutdown(), "presentation fixture cleanup");
	}
	std::cout << "PASS WindowsShadowHostProbe checks=" << Checks << " hardware=none sdk=v3\n";
}
