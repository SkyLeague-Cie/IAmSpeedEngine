#include "SpeedCarCameraArm.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS
namespace
{
class FIndependentNominalPolicy final : public ISpeedCarCameraFramePolicy
{
public:
    explicit FIndependentNominalPolicy(bool Target) : bTarget(Target) {}
    bool bTarget;
    uint32 Begins = 0, Ends = 0, Queries = 0, Commits = 0;
    void BeginFrame() override { ++Begins; }
    void EndFrame() override { ++Ends; }
    bool IsTargetMode() const override { return bTarget; }
    float PreviousTargetDistanceOffset() const override { return 0; }
    void AdvanceTarget(const FVector&, const FVector&, const FVector&, float) override {}
    FRotator TargetRotation() const override { return FRotator(20, 90, 0); }
    ESpeedCarCameraReason ResolveConstraint(uint64, const FVector&, const FVector& Eye,
        const FQuat&, FVector& Out) override
    { ++Queries; Out = Eye; return ESpeedCarCameraReason::None; }
    void CommitConstraint(uint64) override { ++Commits; }
};
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSpeedCarCameraIndependentNominalTest,
    "IAmSpeed.Camera.IndependentNominalTrajectoriesNoPhysicalCertificate",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FSpeedCarCameraIndependentNominalTest::RunTest(const FString&)
{
    // Synthetic nominal inputs only. This test never claims real contacts,
    // a complete camera, physical admission or a full-frame timing budget.
    FSimulationPresentationBody Car;
    Car.StableId = 1;
    Car.COMState.Location = FVector(100, 200, 30);
    Car.COMState.Rotation = FQuat::Identity;
    Car.COMState.Velocity = FVector(150, 0, 0);
    FSpeedCarCameraArmParameters Parameters;
    FSpeedCarCameraFrameInput Input;
    Input.bOnGround = true;
    auto CarState = FSpeedCarCameraArm::DefaultState();
    auto BallState = FSpeedCarCameraArm::DefaultState();
    FIndependentNominalPolicy CarPolicy(false), BallPolicy(true);
    FSpeedCarCameraNominalFrame CarGoal, BallGoal;
    for (uint64 Frame = 0; Frame < 600; ++Frame)
    {
        CarGoal = FSpeedCarCameraArm::AdvanceNominal(CarState, Parameters, Frame, Car, Input, CarPolicy);
        BallGoal = FSpeedCarCameraArm::AdvanceNominal(BallState, Parameters, Frame, Car, Input, BallPolicy);
        if (!TestTrue(TEXT("both independent trajectories advance each physical input frame"),
            CarGoal.bInputsAndNominalReady && BallGoal.bInputsAndNominalReady)) return false;
        if (!TestTrue(TEXT("nominal outputs never acquire published physical Valid status"),
            CarGoal.Result.Status != ESpeedCarCameraStatus::Valid &&
            BallGoal.Result.Status != ESpeedCarCameraStatus::Valid)) return false;
        if (!TestEqual(TEXT("CarCam history remains contiguous"), CarState.LastFrame, Frame) ||
            !TestEqual(TEXT("BallCam history remains contiguous while unselected"), BallState.LastFrame, Frame)) return false;
    }
    TestEqual(TEXT("nominal CarCam performs no collision query"), CarPolicy.Queries, 0u);
    TestEqual(TEXT("nominal BallCam performs no collision query"), BallPolicy.Queries, 0u);
    TestEqual(TEXT("nominal trajectories commit no certificates"), CarPolicy.Commits + BallPolicy.Commits, 0u);
    TestEqual(TEXT("every frame enters both trajectories"), CarPolicy.Begins + BallPolicy.Begins, 1200u);
    TestEqual(TEXT("all independent frame scopes close"), CarPolicy.Ends + BallPolicy.Ends, 1200u);
    TestTrue(TEXT("two mode endpoints differ without sharing a lag state"),
        CarGoal.ViewRotation != BallGoal.ViewRotation && CarGoal.DesiredEye != BallGoal.DesiredEye);
    auto ColdState = FSpeedCarCameraArm::DefaultState();
    FIndependentNominalPolicy ColdPolicy(true);
    const auto ColdGoal = FSpeedCarCameraArm::AdvanceNominal(ColdState, Parameters, 599, Car, Input, ColdPolicy);
    TestTrue(TEXT("warm inactive endpoint has already advanced beyond cold acquisition"),
        BallGoal.ViewRotation.Rotator().Yaw > ColdGoal.ViewRotation.Rotator().Yaw);
    const auto CarBefore = CarState;
    const auto Duplicate = FSpeedCarCameraArm::AdvanceNominal(BallState, Parameters, 599, Car, Input, BallPolicy);
    TestFalse(TEXT("duplicate frame cannot replay nominal advancement"), Duplicate.bInputsAndNominalReady);
    TestTrue(TEXT("duplicate frame poisons that trajectory"), BallState.bTimelineInvalid);
    TestTrue(TEXT("independent CarCam timeline is not poisoned by BallCam rejection"),
        !CarState.bTimelineInvalid && CarState.LastFrame == CarBefore.LastFrame &&
        CarState.PreviousDesiredRot == CarBefore.PreviousDesiredRot);
    return true;
}
#endif
