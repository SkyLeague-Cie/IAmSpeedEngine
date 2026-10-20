#include "FramePointerWitness.h"
#include "IAmSpeed/Base/EpisodeStateBytesForTesting.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFramePointerWitnessTests,
    "IAmSpeed.Simulation.FrameWitness.PointerIdentity",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FFramePointerWitnessTests::RunTest(const FString&)
{
    using namespace Speed;
    int A=1,B=2,C=3;
    const FFramePointerKey Key{1,41,2,7,0},Other{1,42,2,7,0};
    FFramePointerRegistry RA,RB;
    TestTrue(TEXT("explicit left identity"),RA.Register(&A,Key));
    TestTrue(TEXT("expected alias repeats the same identity"),RA.Register(&A,Key));
    TestTrue(TEXT("explicit right identity"),RB.Register(&B,Key));
    TestTrue(TEXT("seal left"),RA.Seal());TestTrue(TEXT("seal right"),RB.Seal());
    TArray<uint8> RawA,RawB,CanA,CanB;
    {
        FFramePointerCaptureScope Scope(RawA,RA);
        FEpisodeStateBytesForTesting::Add(RawA,uint32(11));
        FEpisodeStateBytesForTesting::Add(RawA,&A);
        FEpisodeStateBytesForTesting::Add(RawA,&A);
        FEpisodeStateBytesForTesting::Add(RawA,static_cast<int*>(nullptr));
        TestTrue(TEXT("known left pointers canonicalize"),Scope.MakeCanonical(CanA));
        TestEqual(TEXT("all typed pointer positions observed"),Scope.Observations().Num(),3);
    }
    {
        FFramePointerCaptureScope Scope(RawB,RB);
        FEpisodeStateBytesForTesting::Add(RawB,uint32(11));
        FEpisodeStateBytesForTesting::Add(RawB,&B);
        FEpisodeStateBytesForTesting::Add(RawB,&B);
        FEpisodeStateBytesForTesting::Add(RawB,static_cast<int*>(nullptr));
        TestTrue(TEXT("known right pointers canonicalize"),Scope.MakeCanonical(CanB));
    }
    TestTrue(TEXT("canonical semantic bytes match across different addresses"),CanA==CanB);
    TestTrue(TEXT("raw bytes are retained and differ"),RawA!=RawB);
    {
        TArray<uint8> Outer,Inner,Out;FFramePointerCaptureScope OuterScope(Outer,RA);
        {
            FFramePointerCaptureScope InnerScope(Inner,RB);
            FEpisodeStateBytesForTesting::Add(Outer,&A);
            FEpisodeStateBytesForTesting::Add(Inner,&B);
            TestTrue(TEXT("nested target independently captured"),InnerScope.MakeCanonical(Out));
        }
        TestEqual(TEXT("nested scope cannot hide outer typed pointers"),OuterScope.Observations().Num(),1);
        TestTrue(TEXT("outer target remains complete"),OuterScope.MakeCanonical(Out));
    }
    {
        TArray<uint8> Raw,Out;FFramePointerCaptureScope Scope(Raw,RA);
        FEpisodeStateBytesForTesting::Add(Raw,uint32(12));
        FEpisodeStateBytesForTesting::Add(Raw,&A);FEpisodeStateBytesForTesting::Add(Raw,&A);
        FEpisodeStateBytesForTesting::Add(Raw,static_cast<int*>(nullptr));
        TestTrue(TEXT("changed scalar witness canonicalizes"),Scope.MakeCanonical(Out));
        TestTrue(TEXT("pointer normalization cannot hide an ordinary state mutation"),Out!=CanA);
    }
    {
        TArray<uint8> Raw,Unchanged{99};FFramePointerCaptureScope Scope(Raw,RA);
        FEpisodeStateBytesForTesting::Add(Raw,&C);
        TestFalse(TEXT("unregistered pointer is UNKNOWN, not erased"),Scope.MakeCanonical(Unchanged));
        TestTrue(TEXT("unknown identity visible"),Scope.HasUnknown());
        TestEqual(TEXT("failed canonicalization leaves output untouched"),Unchanged[0],uint8(99));
    }
    FFramePointerRegistry AliasConflict;
    TestTrue(TEXT("first alias declaration"),AliasConflict.Register(&A,Key));
    TestFalse(TEXT("distinct pointers cannot silently share one semantic key"),AliasConflict.Register(&B,Key));
    TestFalse(TEXT("conflicting registry cannot seal as valid"),AliasConflict.Seal());
    FFramePointerRegistry IdentityConflict;
    TestTrue(TEXT("first object declaration"),IdentityConflict.Register(&A,Key));
    TestFalse(TEXT("one address cannot silently change semantic identity"),IdentityConflict.Register(&A,Other));
    {
        TArray<uint8> Raw,Out;FFramePointerCaptureScope Scope(Raw,RA,1);
        FEpisodeStateBytesForTesting::Add(Raw,&A);FEpisodeStateBytesForTesting::Add(Raw,&A);
        TestFalse(TEXT("bounded pointer overflow is UNKNOWN"),Scope.MakeCanonical(Out));
        TestTrue(TEXT("overflow visible"),Scope.HasOverflow());
    }
    {
        TArray<uint8> Raw,Out;FFramePointerCaptureScope Scope(Raw,RA);
        FEpisodeStateBytesForTesting::Add(Raw,&A);Raw[0]^=1;
        TestFalse(TEXT("raw bytes cannot drift after typed observation"),Scope.MakeCanonical(Out));
    }
    return true;
}
#endif
