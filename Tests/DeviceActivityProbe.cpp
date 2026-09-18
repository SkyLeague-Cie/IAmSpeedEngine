#include "IAmSpeed/Input/DeviceActivityPolicy.h"
#include <cstdlib>
#include <iostream>
using namespace Speed::Input;
static unsigned Checks = 0;
static void Check(bool Value, const char* What) { ++Checks; if (!Value) { std::cerr << "FAIL " << What << '\n'; std::exit(1); } }
static FDeviceId Id(std::uint8_t N) { FDeviceId R{}; R[0] = N; return R; }
static FActivityConfig Config(FFrameNumber Residence = 0) { return {{.20f,.15f,.10f},{.08f,.05f,.04f},Residence,8,EDeviceKind::Gamepad}; }
static std::vector<FDiscoveredDevice> Devices() { return {{Id(1),1,2,true},{Id(2),1,1,true},{Id(3),1,2,true}}; }
static FActivityState Button(bool Held) { FActivityState S; S.Buttons[0] = Held; return S; }
int main()
{
	Check(FDeviceActivityPolicy::ValidConfig(Config()), "explicit candidate config valid");
	for (float Invalid : {0.f, -1.f, std::numeric_limits<float>::quiet_NaN(), 2.f})
	{ auto C = Config(); C.Stick.Delta = Invalid; Check(!FDeviceActivityPolicy::ValidConfig(C), "invalid delta rejected"); }
	for (bool Reverse : {false,true})
	{
		FDeviceActivityPolicy P(Config()); P.Sync(Devices());
		for (auto N : {1,2,3}) P.Observe(Id(static_cast<std::uint8_t>(N)),1,1,{});
		Check(!P.Decide(0), "startup no activity has no implicit default");
		P.Observe(Id(Reverse ? 2 : 1),1,2,Button(true)); P.Observe(Id(Reverse ? 1 : 2),1,2,Button(true));
		Check(P.Decide(1)->Id == Id(1), "simultaneous tie stable ID independent of observe order");
		P.Observe(Id(1),1,3,Button(false)); P.Observe(Id(2),1,3,Button(false));
		Check(P.Decide(2)->Id == Id(1), "releases do not reclaim");
		P.Observe(Id(2),1,4,Button(true)); Check(P.Decide(3)->Id == Id(2), "new press claims");
		P.Observe(Id(2),1,5,Button(false)); P.Observe(Id(1),1,6,Button(true)); P.Observe(Id(2),1,6,Button(true));
		Check(P.Decide(4)->Id == Id(2), "tie retains current even if ID larger");
		P.Observe(Id(3),1,5,Button(true)); Check(P.Decide(5)->Id == Id(2), "delayed older activity cannot reclaim");
		P.SetLock(Id(3)); Check(P.Decide(6)->Id == Id(3), "lock exact same-kind device");
		P.Observe(Id(1),1,7,Button(false)); P.Observe(Id(1),1,8,Button(true));
		Check(P.Decide(7)->Id == Id(3), "lock suppresses other device");
		P.SetLock(std::nullopt); Check(P.Decide(8)->Id == Id(3), "unlock does not replay suppressed activity");
		auto D = Devices(); D[2].Connected = false; P.SetLock(Id(3)); P.Sync(D);
		Check(!P.Decide(9), "disconnected locked device neutral without fallback");
		D[2].Connected = true; D[2].Revision = 2; P.Sync(D); P.Observe(Id(3),2,10,Button(true));
		Check(P.Decide(10)->Id == Id(3), "locked reconnect held baseline");
		P.SetPaused(true); Check(!P.Decide(11), "pause neutral");
		P.SetPaused(false); P.Observe(Id(3),2,11,Button(true)); Check(P.Decide(12)->Id == Id(3), "resume remembers lock without fake activity");
		P.SetLock(Id(8)); Check(!P.Decide(13), "absent lock neutral");
	}
	{
		FDeviceActivityPolicy P(Config()); P.Sync(Devices()); P.Observe(Id(1),1,1,{}); P.Observe(Id(2),1,1,{});
		P.Observe(Id(2),1,2,Button(true)); P.Decide(0);
		std::uint64_t Stamp=3; FFrameNumber Frame=1; FActivityState S;
		for (float Drift : {.01f,.14f,.16f,.19f,.16f,.14f,0.f})
		{ S.Axes[2]=Drift; P.Observe(Id(1),1,Stamp++,S); Check(P.Decide(Frame++)->Id == Id(2), "drift and hysteresis band never claim"); }
		S.Axes[2]=.3f; P.Observe(Id(1),1,Stamp++,S); Check(P.Decide(Frame++)->Id == Id(1), "intentional stick enter claims");
		P.Observe(Id(2),1,Stamp++,{}); P.Observe(Id(2),1,Stamp++,Button(true)); P.Decide(Frame++);
		for (float Value : {.3f,.31f,.32f,.29f})
		{ S.Axes[2]=Value; P.Observe(Id(1),1,Stamp++,S); Check(P.Decide(Frame++)->Id == Id(2), "held active stick minor drift cannot refresh activity"); }
		S.Axes[2]=-.4f; P.Observe(Id(1),1,Stamp++,S); Check(P.Decide(Frame++)->Id == Id(1), "deliberate direction change claims");
		Check(!P.Observe(Id(1),0,Stamp++,S), "stale lifecycle revision rejected");
		S.Axes[0]=std::numeric_limits<float>::quiet_NaN(); Check(!P.Observe(Id(1),1,Stamp++,S), "nonfinite raw reading rejected");
	}
	{
		FDeviceActivityPolicy P(Config(3)); P.Sync(Devices()); P.Observe(Id(1),1,1,{}); P.Observe(Id(2),1,1,{});
		P.Observe(Id(1),1,2,Button(true)); P.Decide(0); P.Observe(Id(2),1,3,Button(true));
		Check(P.Decide(1)->Id == Id(1), "configured residence suppresses switch"); P.Decide(2);
		Check(P.Decide(3)->Id == Id(1), "suppressed activity not delayed until residence expires");
		P.Observe(Id(2),1,4,{}); P.Observe(Id(2),1,5,Button(true)); Check(P.Decide(4)->Id == Id(2), "fresh eligible activity after residence");
		P.SetLock(Id(1)); Check(P.Decide(4)->Id == Id(2), "same-frame decision replay immutable");
		Check(!P.Decide(6) && P.IsFailed(), "gapped policy decision fails explicitly");
	}
	{
		FDeviceActivityPolicy P(Config()); P.Sync(Devices()); FActivityState Held; Held.Axes[0]=1; Held.Buttons[0]=true;
		P.Observe(Id(1),1,1,Held); Check(!P.Decide(0), "held initial state is not an activity edge");
		P.Observe(Id(2),1,1,{}); P.Observe(Id(2),1,2,Button(true)); P.Decide(1);
		FActivityState S; S.Axes[0]=.05f; P.Observe(Id(1),1,3,S); P.Decide(2);
		S.Axes[0]=.079f; P.Observe(Id(1),1,4,S); Check(P.Decide(3)->Id==Id(2),"trigger below enter does not claim");
		S.Axes[0]=.08f; P.Observe(Id(1),1,5,S); Check(P.Decide(4)->Id==Id(1),"trigger exact enter claims after rearm");
		P.Observe(Id(2),1,6,{}); P.Observe(Id(2),1,7,Button(true)); P.Decide(5);
		S.Axes[0]=.10f; P.Observe(Id(1),1,8,S); Check(P.Decide(6)->Id==Id(2),"trigger subdelta does not reclaim");
		S.Axes[0]=.13f; P.Observe(Id(1),1,9,S); Check(P.Decide(7)->Id==Id(1),"trigger delta from last activity anchor claims");
		P.Observe(Id(2),1,10,{}); P.Observe(Id(2),1,11,Button(true)); P.Decide(8);
		S.Axes[2]=.15f; S.Axes[3]=.15f; P.Observe(Id(1),1,12,S); Check(P.Decide(9)->Id==Id(1),"radial diagonal stick enters despite each component below threshold");
		auto C=Config(); C.MaximumDevices=1; FDeviceActivityPolicy Limited(C);
		Check(!Limited.Sync(Devices()) && Limited.IsFailed(),"candidate budget fails closed");
	}
	std::cout << "PASS DeviceActivityProbe checks=" << Checks << '\n';
}
