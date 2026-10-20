#include "PrescribedMotion.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
using namespace Speed::Motion;
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPrescribedMotionClockTest,"IAmSpeed.Motion.StageClock",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FPrescribedMotionClockTest::RunTest(const FString&)
{
    FStageMotionClock C;
    for(uint64 I=100;I<400;++I)TestTrue(TEXT("Rest frames admitted"),C.Advance(I,false,false));
    TestEqual(TEXT("Rest does not animate"),C.ActiveTicks,uint64(0));
    for(uint64 I=400;I<700;++I)TestTrue(TEXT("Inside frames admitted"),C.Advance(I,true,false));
    TestEqual(TEXT("One canonical second"),C.Seconds(),1.);
    for(uint64 I=700;I<1000;++I)C.Advance(I,true,true);
    TestEqual(TEXT("Pause holds phase"),C.ActiveTicks,uint64(300));
    for(uint64 I=1000;I<1300;++I)C.Advance(I,false,false);
    TestEqual(TEXT("Exit holds phase"),C.ActiveTicks,uint64(300));
    C.Advance(1300,true,false);TestEqual(TEXT("Reentry resumes"),C.ActiveTicks,uint64(301));
    const FStageMotionClock Before=C;
    TestFalse(TEXT("Duplicate rejected"),C.Advance(1300,true,false));
    TestFalse(TEXT("Gap rejected"),C.Advance(1302,true,false));
    TestEqual(TEXT("Rejected frame cannot mutate phase"),C.ActiveTicks,Before.ActiveTicks);
    TestEqual(TEXT("Rejected frame cannot mutate address"),C.LastFrame,Before.LastFrame);
    const FPrescribedMotionKey OffGridKeys[]={{0.,{0.,0.,0.},0.},{.001,{1.,0.,0.},0.}};
    FPrescribedMotionTrack OffGrid;TestTrue(TEXT("Generic off-grid numeric track"),OffGrid.Configure(MakeArrayView(OffGridKeys),{0.,0.,1.},false));
    FPrescribedMotionSample Refused;TestFalse(TEXT("Canonical evaluator refuses rounded off-grid trajectory"),OffGrid.EvaluateCanonicalTicks(.5,Refused));
    // Later cycles retain exact key addresses and outgoing derivatives.
    const FPrescribedMotionKey Grid[]={{0.,{0.,0.,0.},0.},{7./300.,{7.,0.,0.},0.},{19./300.,{0.,0.,0.},0.}};
    FPrescribedMotionTrack Cyclic;TestTrue(TEXT("Integer-addressed loop configured"),Cyclic.Configure(MakeArrayView(Grid),{0.,0.,1.},true));
    for(uint32 Cycle:{0u,1u,1000u,1000000u})
    {
        FPrescribedMotionSample Addressed;TestTrue(TEXT("Later-cycle tick sample"),Cyclic.EvaluateCanonicalTicks(double(Cycle)*19.+7.,Addressed));
        TestTrue(TEXT("Exact key pose"),Addressed.Translation==FVector3d(7.,0.,0.));TestTrue(TEXT("Outgoing derivative at exact key"),Addressed.Velocity.Equals(FVector3d(-175.,0.,0.),1.e-12));
    }
    return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPrescribedMotionTrackTest,"IAmSpeed.Motion.LinearTrack",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FPrescribedMotionTrackTest::RunTest(const FString&)
{
    FPrescribedMotionTrack Track;FPrescribedMotionSample S;
    const FPrescribedMotionKey Keys[]={{0.,{0.,0.,0.},0.},{2.,{100.,0.,0.},UE_DOUBLE_PI},{4.,{0.,0.,0.},2.*UE_DOUBLE_PI}};
    TestTrue(TEXT("Closed linear cycle admitted"),Track.Configure(MakeArrayView(Keys),{0.,0.,1.},true));
    Track.Evaluate(1.,S);
    TestTrue(TEXT("True interpolated translation"),S.Translation.Equals({50.,0.,0.},1.e-12));
    TestTrue(TEXT("Physical velocity"),S.Velocity.Equals({50.,0.,0.},1.e-12));
    TestTrue(TEXT("Unwrapped rotation preserves revolution"),S.Rotation.RotateVector({1.,0.,0.}).Equals({0.,1.,0.},1.e-12));
    TestTrue(TEXT("Angular velocity"),S.AngularVelocity.Equals({0.,0.,UE_DOUBLE_PI/2.},1.e-12));
    TestEqual(TEXT("CCD horizon ends at key"),S.SecondsToBoundary,1.);
    Track.Evaluate(2.,S);TestTrue(TEXT("Boundary uses next physical velocity"),S.Velocity.Equals({-50.,0.,0.},1.e-12));
    Track.Evaluate(4.,S);TestTrue(TEXT("Loop closed without jump"),S.Translation.IsNearlyZero(1.e-12));
    Track.Evaluate(5.,S);TestTrue(TEXT("Next cycle same pose"),S.Translation.Equals({50.,0.,0.},1.e-12));
    const FPrescribedMotionKey Open[]={{0.,{0.,0.,0.},0.},{1.,{10.,0.,0.},0.}};
    TestFalse(TEXT("A teleporting loop is not silently admitted"),Track.Configure(MakeArrayView(Open),{0.,0.,1.},true));
    Track.Evaluate(1.,S);TestTrue(TEXT("Bad replacement preserves admitted track"),S.Translation.Equals({50.,0.,0.},1.e-12));
    TestTrue(TEXT("Nonlooping track admitted"),Track.Configure(MakeArrayView(Open),{0.,0.,1.},false));
    Track.Evaluate(2.,S);TestTrue(TEXT("Terminal key holds without residual speed"),S.Translation.Equals({10.,0.,0.},1.e-12)&&S.Velocity.IsZero()&&S.AngularVelocity.IsZero());
    TestFalse(TEXT("Negative time rejected"),Track.Evaluate(-1.,S));
    return !HasAnyErrors();
}
#endif
