#include "KeyboardDiagnosticModel.h"
#include <cstdlib>
#include <iostream>
using namespace KeyboardDiagnostic;
static unsigned Checks = 0;
static void Check(bool Value, const char* Name)
{ ++Checks; if (!Value) { std::cerr << "FAIL " << Name << '\n'; std::exit(1); } }
int main()
{
	FReadings R;
	Check(R.Observe(100, false, false) == EObservation::Initial, "initial neutral");
	for (int I = 0; I < 100; ++I) Check(R.Observe(100, false, true) == EObservation::Unchanged, "no update");
	Check(R.Samples == 1 && R.Transitions == 0, "duplicates are not activity");
	Check(R.Observe(101, true, false) == EObservation::Updated && R.Transitions == 1, "press");
	Check(R.Observe(101, false, false) == EObservation::Updated && R.Transitions == 2, "different readings may share timestamp");
	R.Focus(true); R.Focus(false);
	Check(R.FocusChanges == 2 && !R.IsHeld() && R.Transitions == 2, "focus metadata does not fabricate input");
	Check(R.Observe(99, true, false) == EObservation::Invalid && !R.IsHeld(), "late timestamp rejected without overwriting baseline");
	Check(R.Observe(102, true, false) == EObservation::Invalid, "failure sticky");
	R.Stop(); Check(R.Observe(103, true, false) == EObservation::Stopped, "no acquisition after stop");
	FReadings Held; Check(Held.Observe(0, true, false) == EObservation::Initial && Held.Transitions == 0, "initial held is not a press");
	Check(Held.Observe(0, false, true) == EObservation::Invalid, "same reading identity cannot change state");
	FCues C;
	for (unsigned I = 0; I < KeyboardCues.size(); ++I)
	{
		const auto At = KeyboardCues[I].AtUs;
		Check(C.Advance(At) == I, "scheduled cue");
		Check(C.Acknowledge(I, At + 1000), "actual display acknowledged");
		C.Observe(KeyboardCues[I].ExpectedF9, true);
	}
	Check(C.Complete(), "all required phases covered");
	FCues Silent;
	for (unsigned I = 0; I < KeyboardCues.size(); ++I)
	{ Silent.Advance(KeyboardCues[I].AtUs); Silent.Acknowledge(I, KeyboardCues[I].AtUs); Silent.Observe(false, true); }
	Check(!Silent.Complete(), "no-update capture never qualifies keyboard");
	FCues Late; Check(!Late.Advance(FCues::MaxLagUs + 1) && Late.IsFailed(), "late cue fails instead of skipping");
	FCues Unseen; Unseen.Advance(0); Unseen.Advance(FCues::MaxLagUs + 1);
	Check(Unseen.IsFailed(), "unacknowledged cue timeout");
	FCues Lost; Lost.Advance(0); Lost.Acknowledge(0, 0); Lost.Observe(false, false);
	Check(Lost.IsFailed(), "focus loss invalidates foreground protocol");
	FCues MissingOnBlur;
	MissingOnBlur.Advance(0); MissingOnBlur.Acknowledge(0, 0); MissingOnBlur.Observe(false, true);
	MissingOnBlur.ObserveFocus(false); // No reading: do not call Observe(F9,...).
	MissingOnBlur.ObserveFocus(true);
	Check(MissingOnBlur.IsFailed(), "brief focus loss without a reading remains invalid after regain");
	FCues CoveredBeforeBlur;
	for (unsigned I = 0; I < KeyboardCues.size(); ++I)
	{ CoveredBeforeBlur.Advance(KeyboardCues[I].AtUs); CoveredBeforeBlur.Acknowledge(I, KeyboardCues[I].AtUs); CoveredBeforeBlur.Observe(KeyboardCues[I].ExpectedF9, true); }
	Check(CoveredBeforeBlur.Complete(), "fixture establishes completed phases before loss");
	CoveredBeforeBlur.ObserveFocus(false);
	Check(!CoveredBeforeBlur.Complete(), "covered phases cannot hide focus loss without a reading");
	FCues Backwards; Backwards.Advance(1000); Backwards.Advance(999);
	Check(Backwards.IsFailed(), "monotonic local clock required");
	FCues Wrong; Wrong.Advance(0); Check(!Wrong.Acknowledge(1, 1), "wrong cue acknowledgement rejected");
	FCleanup Cleanup;
	Check(!Cleanup.Complete(), "cleanup starts unproven");
	Cleanup.WorkerStopped = Cleanup.ReadingReleased = Cleanup.ApiReleased = true;
	Check(!Cleanup.Complete(), "window still alive is incomplete cleanup");
	Cleanup.WindowDestroyed = true; Check(Cleanup.Complete(), "all owned resources released");
	std::cout << "PASS KeyboardDiagnosticModelProbe " << Checks << " checks\n";
}
