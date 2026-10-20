#include "FrameSolveWitness.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFrameSolveWitnessTests,
    "IAmSpeed.Simulation.FrameWitness.OrderedSolveBytes",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FFrameSolveWitnessTests::RunTest(const FString&)
{
    using namespace Speed;
    FFramePointerRegistry Registry;TestTrue(TEXT("explicit null-only identities"),Registry.Seal());
    SHitResult Hit;Hit.bHit=true;Hit.TOI=0.125f;Hit.SourceId=17;
    TArray<uint8> A,B,CA,CB;
    {
        FFrameSolveWitnessScope Scope(9,A,Registry);
        FFrameSolveWitnessScope::Record(9,0,0,1,0,0,Hit,false,0,1);
        FFrameSolveWitnessScope::Record(9,0,1,2,0,0,Hit,false,0,1);
        TestEqual(TEXT("two actual invocation witnesses"),Scope.NumRows(),2);
        TestTrue(TEXT("complete selected-order witness"),Scope.MakeCanonical(CA));
    }
    {
        FFrameSolveWitnessScope Scope(9,B,Registry);
        FFrameSolveWitnessScope::Record(9,0,1,2,0,0,Hit,false,0,1);
        FFrameSolveWitnessScope::Record(9,0,0,1,0,0,Hit,false,0,1);
        TestTrue(TEXT("reverse invocation witness"),Scope.MakeCanonical(CB));
    }
    TestTrue(TEXT("same unordered contact set cannot hide changed solve order"),CA!=CB);
    {
        TArray<uint8> Raw,Out;FFrameSolveWitnessScope Scope(9,Raw,Registry,1);
        FFrameSolveWitnessScope::Record(9,0,0,1,0,0,Hit,false,0,1);
        FFrameSolveWitnessScope::Record(9,0,1,2,0,0,Hit,false,0,1);
        TestFalse(TEXT("dropped actual contact is UNKNOWN"),Scope.MakeCanonical(Out));
        TestTrue(TEXT("contact overflow explicit"),Scope.HasOverflow());
    }
    {
        TArray<uint8> Raw,Out;FFrameSolveWitnessScope Scope(9,Raw,Registry);
        FFrameSolveWitnessScope::Record(10,0,0,1,0,0,Hit,false,0,1);
        TestFalse(TEXT("wrong frame cannot be silently attributed"),Scope.MakeCanonical(Out));
    }
    return true;
}
#endif
