#if WITH_DEV_AUTOMATION_TESTS
#include "ReplayPlaybackClock.h"
#include "Misc/AutomationTest.h"
#include <limits>

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FReplayPlaybackClockTest,
    "IAmSpeed.Replay.PlaybackClock", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FReplayPlaybackClockTest::RunTest(const FString&)
{
    using Speed::Replay::FPlaybackClock;
    FPlaybackClock Clock;
    TestFalse(TEXT("No empty recording"), Clock.Open(0, 300));
    TestTrue(TEXT("Open physical-frame addressed recording"), Clock.Open(3000, 300));
    const uint64 FirstGeneration = Clock.Epoch();
    Clock.Advance(1.);
    TestEqual(TEXT("Initial paused frame does not advance"), Clock.Index(), uint64(0));
    TestTrue(TEXT("Play"), Clock.Play());
    Clock.Advance(1.);
    TestEqual(TEXT("One second selects physical frame300"), Clock.Index(), uint64(300));
    TestTrue(TEXT("Fast playback accepted"), Clock.SetRate(4.));
    Clock.Advance(.5);
    TestEqual(TEXT("Four-times speed selects900, without physical substeps"), Clock.Index(), uint64(900));
    TestTrue(TEXT("Reverse rate accepted"), Clock.SetRate(-2.));
    Clock.Advance(.5);
    TestEqual(TEXT("Reverse selects an earlier recorded frame"), Clock.Index(), uint64(600));
    TestEqual(TEXT("Continuous rate changes preserve publication generation"), Clock.Epoch(), FirstGeneration);
    TestTrue(TEXT("Backward seek"), Clock.Seek(10));
    TestTrue(TEXT("Seek changes generation, invalidating old async reply"), Clock.Epoch() > FirstGeneration);
    const uint64 SeekGeneration = Clock.Epoch();
    TestTrue(TEXT("Same-frame seek is a new discontinuity"), Clock.Seek(10));
    TestTrue(TEXT("Same-frame seek invalidates old effects/interpolation"), Clock.Epoch() > SeekGeneration);
    Clock.Advance(1.);
    TestEqual(TEXT("Reverse clamps at first frame"), Clock.Index(), uint64(0));
    TestTrue(TEXT("Reverse boundary pauses"), Clock.IsPaused());
    TestTrue(TEXT("Seek above recording clamps"), Clock.Seek(INT64_MAX));
    TestEqual(TEXT("Last actual frame, no synthetic endpoint"), Clock.Index(), uint64(2999));
    TestFalse(TEXT("Reject negative dt"), Clock.Advance(-.1));
    TestFalse(TEXT("Reject NaN dt"), Clock.Advance(std::numeric_limits<double>::quiet_NaN()));
    TestFalse(TEXT("Reject zero rate, pause is explicit"), Clock.SetRate(0));
    TestFalse(TEXT("Reject infinite rate"), Clock.SetRate(std::numeric_limits<double>::infinity()));
    TestFalse(TEXT("Reject unbounded speed"), Clock.SetRate(9));
    Clock.Pause(); Clock.Advance(1.);
    TestEqual(TEXT("Paused endpoint remains exact"), Clock.Index(), uint64(2999));
    const auto BeforeClose = Clock.Epoch(); Clock.Close();
    TestTrue(TEXT("Close invalidates previous replies"), Clock.Epoch() > BeforeClose);
    TestFalse(TEXT("Closed driver cannot play"), Clock.Play());
    // Different render cadences must select the same physical frame after the
    // same playback time, while no per-physics-frame Tick is needed on the GT.
    FPlaybackClock At60, At144;
    At60.Open(3000, 300); At144.Open(3000, 300); At60.Play(); At144.Play();
    for (int I = 0; I < 60; ++I) At60.Advance(1. / 60.);
    for (int I = 0; I < 144; ++I) At144.Advance(1. / 144.);
    TestEqual(TEXT("60Hz vs144Hz physical-frame selection"), At60.Index(), At144.Index());
    TestEqual(TEXT("One second independent of display cadence"), At60.Index(), uint64(300));
    // A 3000-frame clip has 3000 display intervals, not 2999. Check both
    // directions and real durations at several rates without synthesizing
    // endpoint frames or taking any physical simulation substeps.
    for(const double Rate:{1.,-1.,.25,-.25,8.,-8.})
    {
        FPlaybackClock Exposure;Exposure.Open(3000,300);Exposure.SetRate(Rate);
        Exposure.Seek(Rate<0?2999:0);Exposure.Play();
        Exposure.Advance(2999.5/(300.*std::abs(Rate)));
        TestFalse(TEXT("Endpoint snapshot retains its final half interval"),Exposure.IsPaused());
        TestEqual(TEXT("Endpoint is the actual recorded snapshot"),Exposure.Index(),Rate<0?uint64(0):uint64(2999));
        Exposure.Advance(.5/(300.*std::abs(Rate)));
        TestTrue(TEXT("Exactly N/F/rate elapsed seconds complete the clip"),Exposure.IsPaused());
        TestEqual(TEXT("Completion never publishes a synthetic endpoint frame"),Exposure.Index(),Rate<0?uint64(0):uint64(2999));
    }
    for(const double Rate:{1.,-1.})
    {
        FPlaybackClock Single;Single.Open(1,300);Single.SetRate(Rate);Single.Play();
        Single.Advance(.5/300.);
        TestFalse(TEXT("A one-frame recording still occupies a full frame interval"),Single.IsPaused());
        Single.Advance(.5/300.);
        TestTrue(TEXT("One-frame exposure completes at one source interval"),Single.IsPaused());
        TestEqual(TEXT("The sole snapshot remains index zero"),Single.Index(),uint64(0));
    }
    FPlaybackClock Direction;Direction.Open(3000,300);Direction.Seek(100);Direction.Play();
    Direction.Advance(.25/300.);
    TestEqual(TEXT("Partial forward interval retains the selected snapshot"),Direction.Index(),uint64(100));
    Direction.SetRate(-1.);
    TestEqual(TEXT("Direction change alone cannot select a future snapshot"),Direction.Index(),uint64(100));
    Direction.Advance(.5/300.);
    TestEqual(TEXT("Partial reverse interval retains the selected snapshot"),Direction.Index(),uint64(100));
    Direction.Advance(.5/300.);
    TestEqual(TEXT("One full reverse interval selects the preceding snapshot"),Direction.Index(),uint64(99));
    Direction.Pause();Direction.SetRate(1.);Direction.Advance(1.);
    TestEqual(TEXT("Direction change while paused does not move the snapshot"),Direction.Index(),uint64(99));
    return true;
}
#endif
