#if WITH_DEV_AUTOMATION_TESTS
#include "IAmSpeed/SubBodies/SSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SWheelSubBody.h"
#include "Misc/AutomationTest.h"
#include "UObject/StrongObjectPtr.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedSubBodyIgnoredScratchTest,
    "IAmSpeed.ContactDetection.IgnoredScratch",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedSubBodyIgnoredScratchTest::RunTest(const FString& Parameters)
{
    TStrongObjectPtr<USSubBody> Ordinary(NewObject<USSubBody>());
    TStrongObjectPtr<USWheelSubBody> Wheel(NewObject<USWheelSubBody>());
    TStrongObjectPtr<USSubBody> PermanentA(NewObject<USSubBody>());
    TStrongObjectPtr<USSubBody> PermanentB(NewObject<USSubBody>());
    TStrongObjectPtr<USSubBody> Temporary(NewObject<USSubBody>());
    for (USSubBody* Body : { Ordinary.Get(), static_cast<USSubBody*>(Wheel.Get()) })
    {
        Body->AlwaysIgnoredComponents = { PermanentA.Get(), PermanentB.Get() };
        Body->IgnoredComponents.Reserve(8);
        UPrimitiveComponent** WarmBuffer = Body->IgnoredComponents.GetData();
        for (int32 Frame = 0; Frame < 32; ++Frame)
        {
            Body->ResetForFrame(1.0f / 300.0f);
            TestTrue(TEXT("ordinary resets reuse the existing scratch buffer"),
                WarmBuffer == Body->IgnoredComponents.GetData());
            TestEqual(TEXT("only permanent exclusions survive reset"), Body->IgnoredComponents.Num(), 2);
            TestTrue(TEXT("permanent exclusion order is unchanged"),
                Body->IgnoredComponents[0] == PermanentA.Get() && Body->IgnoredComponents[1] == PermanentB.Get());
            TestFalse(TEXT("previous frame's temporary contact is eligible again"),
                Body->ComponentHasBeenIgnored(*Temporary.Get()));
            SHitResult Hit;
            Hit.bHit = true;
            Hit.bBlockingHit = true;
            Hit.Component = Temporary.Get();
            Body->SetHit(Hit);
            Body->SetFutureHit(Hit);
            Body->AcceptHit();
            TestTrue(TEXT("accepted contact remains ignored within its frame"),
                Body->ComponentHasBeenIgnored(*Temporary.Get()));
        }
        Body->IgnoredComponents.SetNumZeroed(1024);
        Body->ResetForFrame(1.0f / 300.0f);
        TestTrue(TEXT("exceptional contact capacity is released on the next reset"),
            Body->IgnoredComponents.GetAllocatedSize() <= FMath::Max<SIZE_T>(1024, Body->AlwaysIgnoredComponents.GetAllocatedSize()));
        TestEqual(TEXT("trimming still restores permanent exclusions only"), Body->IgnoredComponents.Num(), 2);
    }
    TestFalse(TEXT("ordinary reset clears the current hit"), Ordinary->GetHit().bHit);
    TestTrue(TEXT("wheel reset retains current hit for its existing suspension/netcode contract"), Wheel->GetHit().bHit);
    return true;
}
#endif
