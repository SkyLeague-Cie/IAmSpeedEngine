#if WITH_DEV_AUTOMATION_TESTS

#include "IAmSpeed/World/Simulation/SubBodyKinematicsTasks.h"
#include "IAmSpeed/SubBodies/SSubBody.h"
#include "IAmSpeed/SubBodies/Solid/BoxSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SphereSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SWheelSubBody.h"
#include "Misc/AutomationTest.h"
#include "UObject/StrongObjectPtr.h"

namespace
{
template<class T> bool ExactBits(const T& A, const T& B) { return FMemory::Memcmp(&A, &B, sizeof(T)) == 0; }
bool ExactKinematic(const SKinematic& A, const SKinematic& B)
{
    return ExactBits(A.Location, B.Location) && ExactBits(A.Rotation, B.Rotation)
        && ExactBits(A.Velocity, B.Velocity) && ExactBits(A.Acceleration, B.Acceleration)
        && ExactBits(A.AngularVelocity, B.AngularVelocity) && ExactBits(A.AngularAcceleration, B.AngularAcceleration);
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedSubBodyTaskNativeEquivalenceTest,
    "IAmSpeed.Simulation.FrameParallel.SubBodyNativeExact",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)
bool FIAmSpeedSubBodyTaskNativeEquivalenceTest::RunTest(const FString&)
{
    // The independent oracle is the unchanged existing virtual native method.
    TArray<TStrongObjectPtr<USSubBody>> Bodies;
    Bodies.Emplace(NewObject<UBoxSubBody>());
    Bodies.Emplace(NewObject<USphereSubBody>());
    Bodies.Emplace(NewObject<USWheelSubBody>());
    for (int32 Index = 0; Index < Bodies.Num(); ++Index)
    {
        Bodies[Index]->SetRelativeLocation(FVector(20 + Index, -30 - Index * 2, 7 + Index));
        Bodies[Index]->SetRelativeRotation(FRotator(17 + Index, -49 + Index * 7, 11 - Index));
    }
    for (int32 Case = 0; Case < 32; ++Case)
    {
        SKinematic Parent;
        Parent.Location = FVector(37.125 * Case, -19.75 * Case, 150 + Case);
        Parent.Rotation = FRotator(Case * 2.25, Case * -13.125, 27.625).Quaternion();
        Parent.Velocity = FVector(129.125, -733.875, 5.25 * Case);
        Parent.Acceleration = FVector(51.375, -7.125 * Case, -980);
        Parent.AngularVelocity = FVector(.125 * Case, -.375, 2.25);
        Parent.AngularAcceleration = FVector(3.125, 1.75 * Case, -.625);
        for (const auto& Body : Bodies)
        {
            Speed::FrameParallel::FSubBodyKinematicInput Input;
            if (!TestTrue(TEXT("native input captured as values"),
                Speed::FrameParallel::CaptureNativeSubBodyKinematicInput(*Body.Get(), Input))) return false;
            Body->UpdateKinematicsFromOwner(Parent);
            const SKinematic Legacy = Body->GetKinematicState();
            const SKinematic Prepared = Speed::FrameParallel::CalculateSubBodyKinematics(Parent, Input);
            if (!TestTrue(TEXT("extracted arithmetic matches legacy native bits"), ExactKinematic(Legacy, Prepared))) return false;
        }
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedSubBodyTaskBatchTest,
    "IAmSpeed.Simulation.FrameParallel.SubBodyBatchExact",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)
bool FIAmSpeedSubBodyTaskBatchTest::RunTest(const FString&)
{
    SKinematic Parent;
    Parent.Location = FVector(123.125, -35.75, 210);
    Parent.Rotation = FRotator(51.125, -29.875, 17.25).Quaternion();
    Parent.Velocity = FVector(-174.5, 71.25, 31.75);
    Parent.Acceleration = FVector(21.25, -5.75, -980);
    Parent.AngularVelocity = FVector(.125, -2.375, 4.625);
    Parent.AngularAcceleration = FVector(3.125, -1.75, .625);
    TArray<Speed::FrameParallel::FSubBodyKinematicInput> Inputs;
    Inputs.SetNum(31);
    TArray<SKinematic> Serial, Parallel;
    Serial.SetNum(Inputs.Num()); Parallel.SetNum(Inputs.Num());
    for (int32 Index = 0; Index < Inputs.Num(); ++Index)
    {
        auto& Input = Inputs[Index];
        Input.RelativeLocation = FVector(Index * 3.125, -Index * 1.75, Index * .625);
        Input.RelativeRotation = FRotator(Index * 7.125, Index * -13.75, Index * .5).Quaternion();
        Input.bWheel = Index % 3 == 0;
        Input.WheelSpringDisplacement = float(Index - 17) * .125f;
    }
    TestTrue(TEXT("serial batch completes"), Speed::FrameParallel::CalculateSubBodyKinematicsBatch(
        Parent, Inputs, Serial, 1));
    for (const int32 Lanes : {2, 4, 8, 12})
    {
        TestTrue(TEXT("bounded worker batch completes"), Speed::FrameParallel::CalculateSubBodyKinematicsBatch(
            Parent, Inputs, Parallel, Lanes));
        for (int32 Index = 0; Index < Inputs.Num(); ++Index)
            if (!TestTrue(TEXT("all indexed results are bit exact for every width"), ExactKinematic(Serial[Index], Parallel[Index]))) return false;
    }
    const auto Before = Parallel;
    TestFalse(TEXT("out-of-budget lanes rejected before writes"), Speed::FrameParallel::CalculateSubBodyKinematicsBatch(
        Parent, Inputs, Parallel, 13));
    TestFalse(TEXT("bad output span rejected before writes"), Speed::FrameParallel::CalculateSubBodyKinematicsBatch(
        Parent, Inputs, MakeArrayView(Parallel.GetData(), Parallel.Num() - 1), 4));
    TestFalse(TEXT("zero workers rejected before writes"), Speed::FrameParallel::CalculateSubBodyKinematicsBatch(
        Parent, Inputs, Parallel, 0));
    for (int32 Index = 0; Index < Inputs.Num(); ++Index)
        TestTrue(TEXT("rejection preserves outputs"), ExactKinematic(Before[Index], Parallel[Index]));
    return true;
}

#endif
