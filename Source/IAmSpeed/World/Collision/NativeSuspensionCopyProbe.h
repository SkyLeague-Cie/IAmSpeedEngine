#pragma once
#include "NativePrepareGTInspection.h"
#if WITH_DEV_AUTOMATION_TESTS
namespace Speed::PreparationValues
{
// Raw observations from the real native value fixture in IAmSpeed, which owns
// the ChaosVehiclesCore dependency. No Chaos type/API crosses this interface.
// No aggregate PASS, World/frame callback, or physical capability is returned.
struct FSuspensionCopyProbe
{
    FNativePrepareGTFields Before,ChangedCopy,LiveAfter,LiveAfterBudget;
    float CopyGetterResult=0;
    bool BeforeCaptured=false,ChangedCopyCaptured=false,LiveAfterCaptured=false;
    bool ExhaustedBudgetCaptureReturned=false,LiveAfterBudgetCaptured=false;
};
IAMSPEED_API void CaptureSuspensionCopyProbeForTesting(FSuspensionCopyProbe& Out);
}
#endif
