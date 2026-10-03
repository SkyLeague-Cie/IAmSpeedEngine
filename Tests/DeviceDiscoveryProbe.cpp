#include "IAmSpeed/Input/DeviceDiscovery.h"
#include <algorithm>
#include <cstdlib>
#include <iostream>
using namespace Speed::Input;
static unsigned Checks = 0;
static void Check(bool Value, const char* What)
{
	++Checks; if (!Value) { std::cerr << "FAIL " << What << '\n'; std::exit(1); }
}
static FDeviceId Id(std::uint16_t N) { FDeviceId R{}; R[0] = static_cast<std::uint8_t>(N >> 8); R[1] = static_cast<std::uint8_t>(N); return R; }
int main()
{
	std::array<bool, ActionCount> Digital{}; Digital[3] = true;
	std::array<int, 3> Order{1, 2, 3};
	do
	{
		FDeviceDiscovery D(1, Digital);
		Check(D.Select(std::make_pair(Id(2), EDeviceKind::Gamepad)), "explicit absent request");
		for (int N : Order) Check(D.Update({Id(static_cast<std::uint16_t>(N)), 1, 3, true}) == EDiscoveryUpdate::Applied, "enumeration event");
		const auto List = D.Snapshot();
		Check(List.size() == 3 && List[0].Id == Id(1) && List[1].Id == Id(2) && List[2].Id == Id(3), "sorted independent of callback order");
		Check(D.Selected() && D.Selected()->Device.Id == Id(2), "stable explicit selection for every permutation");
	} while (std::next_permutation(Order.begin(), Order.end()));
	FDeviceDiscovery D(2, Digital);
	D.Update({Id(1), 10, 3, true}); D.Update({Id(2), 10, 2, true}); D.Update({Id(3), 10, 0, true});
	Check(!D.Selected(), "enumeration never picks a default");
	D.Select(std::make_pair(Id(3), EDeviceKind::Keyboard)); Check(!D.Selected(), "unsupported device visible but cannot supply keyboard");
	D.Select(std::make_pair(Id(1), EDeviceKind::Keyboard)); const auto A = *D.Selected();
	FActionValues Held{}; Held[Throttle] = 255; Held[3] = 1;
	Check(D.Submit(A, 1, Held), "initial baseline"); const auto Old = *D.Produce(0);
	Check(Old.RequiresReset() && Old.GetEdgeCount() == 0 && Old.GetActions() == Held, "fresh held baseline no synthetic start");
	D.Submit(A, 2, {}); D.Submit(A, 3, Held); // Pending Stop/Start must not survive switch.
	D.Select(std::make_pair(Id(2), EDeviceKind::Gamepad)); const auto B = *D.Selected();
	Check(B.Generation != A.Generation && !D.Submit(A, 4, Held), "switch rejects old ticket");
	Check(D.Submit(B, 1, Held) && D.Skip(1) && D.Skip(2), "new baseline and overrides");
	const auto Switched = *D.Produce(3);
	Check(Switched.RequiresReset() && Switched.GetEdgeCount() == 0 && Switched.GetActions() == Held, "switch purges old edges and reset survives skip");
	D.Select(std::make_pair(Id(2), EDeviceKind::Gamepad)); Check(D.Selected()->Generation == B.Generation, "same request is idempotent");
	Check(D.Update({Id(2), 9, 2, false}) == EDiscoveryUpdate::Stale && D.Selected()->Generation == B.Generation, "stale removal ignored");
	Check(D.Update({Id(2), 10, 2, true}) == EDiscoveryUpdate::Duplicate, "duplicate event ignored");
	D.Update({Id(2), 11, 2, false});
	Check(!D.Selected() && !D.Submit(B, 2, Held), "disconnect invalidates ticket immediately");
	Check(D.Produce(4)->GetActions() == FActionValues{}, "disconnect neutral");
	D.Update({Id(2), 12, 2, true}); const auto Reconnected = *D.Selected();
	Check(Reconnected.Generation != B.Generation && D.Submit(Reconnected, 1, Held), "reconnection fresh generation");
	Check(D.Produce(5)->GetEdgeCount() == 0, "reconnection baseline not a trigger");
	D.Update({Id(2), 13, 2, true}); const auto Replacement = *D.Selected();
	Check(Replacement.Generation != Reconnected.Generation && !D.Submit(Reconnected, 2, Held), "same ID replacement invalidates old generation");
	Check(D.SetPaused(true) && !D.Submit(Replacement, 1, Held), "pause invalidates ticket");
	Check(D.SetPaused(false), "resume"); const auto Resumed = *D.Selected();
	Check(D.Submit(Resumed, 1, Held) && D.Produce(6)->GetEdgeCount() == 0, "fresh held resume");
	Check(D.Produce(0)->GetActions() == Old.GetActions() && D.Produce(0)->RequiresReset() == Old.RequiresReset(), "history unaffected by lifecycle");
	{ FPresentationInputScope Scope; Check(!D.Select(std::nullopt) && !D.Submit(Resumed, 2, {}), "presentation cannot mutate selection or sample"); }
	Check(D.Update({Id(2), 13, 2, false}) == EDiscoveryUpdate::Failed && D.IsFailed(), "conflicting same revision fails closed");
	Check(D.Produce(7)->RequiresReset() && D.Produce(8)->GetActions() == FActionValues{}, "failure delivers reset and neutral cadence");
	FDeviceDiscovery Full(3, Digital);
	for (std::uint16_t N = 0; N < FDeviceDiscovery::Capacity; ++N) Check(Full.Update({Id(N), 0, 0, false}) == EDiscoveryUpdate::Applied, "bounded tombstones");
	Check(Full.Update({Id(256), 0, 1, true}) == EDiscoveryUpdate::Failed, "capacity exhaustion explicit");
	std::cout << "PASS DeviceDiscoveryProbe checks=" << Checks << '\n';
}
