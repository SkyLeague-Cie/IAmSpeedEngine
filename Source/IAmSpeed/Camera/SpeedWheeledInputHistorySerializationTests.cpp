#if WITH_DEV_AUTOMATION_TESTS
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "Misc/AutomationTest.h"
#include "Serialization/BitWriter.h"
#include "Serialization/BitReader.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedWheeledInputHistorySerializationTest,
    "IAmSpeed.Network.WheeledInputHistorySerialization",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)
bool FIAmSpeedWheeledInputHistorySerializationTest::RunTest(const FString&)
{
    // Exercise the genuine UE container: it ignores a false per-entry return.
    // A missing committed projection must fail the outer archive, never become
    // a truncated successful RPC. This is not a live transport qualification.
    for (bool bMissingProjection : {false, true})
    {
        TNetRewindHistory<FNetworkWheeledSpeedInputState, true> Saved(2, true);
        for (int32 Frame = 100; Frame <= 101; ++Frame)
        {
            FNetworkWheeledSpeedInputState Input;
            Input.LocalFrame = Input.ServerFrame = Frame;
            Input.ClientFrame = Frame;
            Input.WheeledInput.Throttle = 201;
            Input.bInputProjectionValid = !bMissingProjection || Frame != 101;
            if (!TestTrue(TEXT("Record actual history entry"), Saved.RecordData(Frame, &Input))) return false;
        }
        FBitWriter Writer(32768, true);
        Saved.NetSerialize(Writer, nullptr, nullptr);
        TestEqual(TEXT("Missing projection fails the containing history archive"), Writer.IsError(), bMissingProjection);
        if (bMissingProjection) continue; // Never transmit/decode a rejected archive.
        TNetRewindHistory<FNetworkWheeledSpeedInputState, true> Loaded(2, false);
        FBitReader Reader(Writer.GetData(), Writer.GetNumBits());
        Loaded.NetSerialize(Reader, nullptr, nullptr);
        if (!TestFalse(TEXT("Complete history remains readable"), Reader.IsError())) return false;
        TestEqual(TEXT("Complete history consumes exactly all bits"), Reader.GetBitsLeft(), int64(0));
        for (int32 Frame = 100; Frame <= 101; ++Frame)
        {
            FNetworkWheeledSpeedInputState Input;
            if (!TestTrue(TEXT("Extract received history entry"), Loaded.ExtractData(Frame, true, &Input, true))) return false;
            TestEqual(TEXT("Requested throttle survives history serialization"), Input.WheeledInput.Throttle, uint8(201));
            TestEqual(TEXT("Source frame survives history serialization"), Input.ClientFrame, uint32(Frame));
        }
    }
    return true;
}
#endif
