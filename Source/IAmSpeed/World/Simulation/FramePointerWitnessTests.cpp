#include "FramePointerWitness.h"
#include "FrameSolveWitness.h"
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
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFramePointerPreTouchTests,
    "IAmSpeed.Simulation.FrameWitness.PointerSnapshotPreTouch",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FFramePointerPreTouchTests::RunTest(const FString&)
{
    using namespace Speed;
    int Object=1,Unknown=2;const FFramePointerKey Key{1,41,2,7,0};
    FFramePointerRegistry Registry;
    if(!TestTrue(TEXT("explicit pre-touch test identity"),Registry.Register(&Object,Key))
        ||!TestTrue(TEXT("pre-touch registry seals"),Registry.Seal()))return false;
    const uint32 Prefix=0x9273a51bu;const uint16 Suffix=0xd34fu;
    TArray<uint8> Raw,ExpectedRaw,ExpectedCanonical;
    // Independent byte oracle: explicit scalar fields, no production byte Add
    // and no raw struct/ABI copy. Ordinary pointer keys have no extra marker.
    const auto Scalar=[](TArray<uint8>& Out,const auto& Value)
    {
        const int32 Offset=Out.AddUninitialized(sizeof(Value));
        FMemory::Memcpy(Out.GetData()+Offset,&Value,sizeof(Value));
    };
    Scalar(ExpectedRaw,Prefix);Scalar(ExpectedRaw,reinterpret_cast<UPTRINT>(&Object));
    Scalar(ExpectedRaw,UPTRINT(0));Scalar(ExpectedRaw,Suffix);
    Scalar(ExpectedCanonical,Prefix);
    for(const FFramePointerKey& Identity:{Key,FFramePointerKey{}})
    {
        Scalar(ExpectedCanonical,Identity.Domain);Scalar(ExpectedCanonical,Identity.Owner);
        Scalar(ExpectedCanonical,Identity.Generation);Scalar(ExpectedCanonical,Identity.Role);
        Scalar(ExpectedCanonical,Identity.Element);Scalar(ExpectedCanonical,uint32(0));
    }
    Scalar(ExpectedCanonical,Suffix);
    FFramePointerCanonicalSnapshot1456 Prepared,Control,Tiny;
    Control.Reserve1456(32,2);
    TestTrue(TEXT("explicit two-MiB pre-touch succeeds"),Prepared.ReserveAndPreTouch1456(2*1024*1024,2048));
    TestTrue(TEXT("pre-touch keeps raw byte count zero"),Prepared.RawForEvidence1456().IsEmpty());
    TestEqual(TEXT("pre-touch creates no pointer observations"),Prepared.NumObservations1456(),0);
    TestTrue(TEXT("tiny reservation is only a growable hint"),Tiny.ReserveAndPreTouch1456(1,1));
    TArray<uint8> Uncaptured{99};
    TestFalse(TEXT("pre-touch alone never creates a certificate"),Prepared.MakeCanonical(Uncaptured));
    TestTrue(TEXT("uncaptured result stays untouched"),Uncaptured==TArray<uint8>{99});
    {
        FFramePointerCaptureScope Scope(Raw,Registry);
        FEpisodeStateBytesForTesting::Add(Raw,Prefix);
        FEpisodeStateBytesForTesting::Add(Raw,&Object);
        FEpisodeStateBytesForTesting::Add(Raw,static_cast<int*>(nullptr));
        FEpisodeStateBytesForTesting::Add(Raw,Suffix);
        TestTrue(TEXT("prepared snapshot captures complete owner bytes"),Scope.CaptureCanonicalSnapshot1456(Prepared));
        TestTrue(TEXT("unchanged reserve-only control captures"),Scope.CaptureCanonicalSnapshot1456(Control));
        TestTrue(TEXT("raw/events grow past tiny reservation"),Scope.CaptureCanonicalSnapshot1456(Tiny));
    }
    TestTrue(TEXT("owner raw stream matches independent oracle"),Raw==ExpectedRaw);
    for(FFramePointerCanonicalSnapshot1456* Snapshot:{&Prepared,&Control,&Tiny})
    {
        TArray<uint8> Canonical;
        TestTrue(TEXT("snapshot retains all raw bytes"),Snapshot->RawForEvidence1456()==ExpectedRaw);
        TestEqual(TEXT("snapshot retains both pointer observations"),Snapshot->NumObservations1456(),2);
        TestTrue(TEXT("snapshot canonicalizes normally"),Snapshot->MakeCanonical(Canonical));
        TestTrue(TEXT("canonical bytes match independent explicit-key oracle"),Canonical==ExpectedCanonical);
        TestFalse(TEXT("pre-touch refuses an existing certificate"),Snapshot->ReserveAndPreTouch1456(64,4));
        TestTrue(TEXT("refusal leaves raw certificate untouched"),Snapshot->RawForEvidence1456()==ExpectedRaw);
        TestEqual(TEXT("refusal leaves observation count untouched"),Snapshot->NumObservations1456(),2);
        TArray<uint8> After;
        TestTrue(TEXT("refused certificate still canonicalizes"),Snapshot->MakeCanonical(After));
        TestTrue(TEXT("refusal leaves canonical certificate untouched"),After==ExpectedCanonical);
    }
    {
        TArray<uint8> Empty;FFramePointerCanonicalSnapshot1456 EmptyCertificate;
        FFramePointerCaptureScope Scope(Empty,Registry);
        TestTrue(TEXT("empty stream may still be a real certificate"),Scope.CaptureCanonicalSnapshot1456(EmptyCertificate));
        TestFalse(TEXT("pre-touch refuses even an empty captured certificate"),EmptyCertificate.ReserveAndPreTouch1456(64,2));
        TArray<uint8> Out{99};
        TestTrue(TEXT("empty certificate validity survives refusal"),EmptyCertificate.MakeCanonical(Out));
        TestTrue(TEXT("empty certificate bytes survive refusal"),Out.IsEmpty());
    }
    for(int32 Failure=0;Failure<3;++Failure)
    {
        TArray<uint8> BadRaw;FFramePointerCanonicalSnapshot1456 Destination;
        TestTrue(TEXT("prepare destination for rejected capture"),Destination.ReserveAndPreTouch1456(64,2));
        FFramePointerCaptureScope Scope(BadRaw,Registry,Failure==1?1:16384);
        FEpisodeStateBytesForTesting::Add(BadRaw,Failure==0?&Unknown:&Object);
        if(Failure==1)FEpisodeStateBytesForTesting::Add(BadRaw,&Object);
        if(Failure==2)BadRaw[0]^=1;
        if(Failure<2)
        {
            TestFalse(TEXT("unknown/overflow still rejects snapshot capture"),Scope.CaptureCanonicalSnapshot1456(Destination));
            TestTrue(TEXT("rejected capture leaves empty destination"),Destination.RawForEvidence1456().IsEmpty());
        }
        else
        {
            // Deferred capture keeps the raw drift; projection must still
            // reject it rather than erase the retained evidence.
            TestTrue(TEXT("capture retains raw drift for validation"),Scope.CaptureCanonicalSnapshot1456(Destination));
            TestTrue(TEXT("corrupted raw evidence remains intact"),Destination.RawForEvidence1456()==BadRaw);
        }
        TArray<uint8> Out{99};
        TestFalse(TEXT("unknown/overflow/raw drift never canonicalizes"),Destination.MakeCanonical(Out));
        TestTrue(TEXT("failed projection leaves destination bytes untouched"),Out==TArray<uint8>{99});
    }
    // Final transfer is checked against the independently assembled byte/key
    // oracle above, as well as the unchanged copy API. Destinations precede
    // scopes in lifetime and never move while the writer scope is open.
    {
        TArray<uint8> Source;FFramePointerCanonicalSnapshot1456 Copy,Consumed,Second;
        TestTrue(TEXT("consume destination prepares empty storage"),Consumed.ReserveAndPreTouch1456(128,4));
        {
            FFramePointerCaptureScope Scope(Source,Registry);
            FEpisodeStateBytesForTesting::Add(Source,Prefix);FEpisodeStateBytesForTesting::Add(Source,&Object);
            FEpisodeStateBytesForTesting::Add(Source,static_cast<int*>(nullptr));FEpisodeStateBytesForTesting::Add(Source,Suffix);
            const uint8* OriginalStorage=Source.GetData();
            TestTrue(TEXT("copy remains supported before final transfer"),Scope.CaptureCanonicalSnapshot1456(Copy));
            TestTrue(TEXT("final transfer accepts complete stream"),Scope.ConsumeCanonicalSnapshot1456(Consumed));
            TestTrue(TEXT("transfer leaves valid empty source"),Source.IsEmpty());
            TestTrue(TEXT("transfer preserves original raw storage ownership"),Consumed.RawForEvidence1456().GetData()==OriginalStorage);
            TestTrue(TEXT("transferred raw bytes match independent oracle"),Consumed.RawForEvidence1456()==ExpectedRaw);
            TestEqual(TEXT("all transferred pointer events retained"),Consumed.NumObservations1456(),2);
            TArray<uint8> Premature{99};
            TestFalse(TEXT("deferred projection waits for scope closure"),Consumed.MakeCanonical(Premature));
            TestTrue(TEXT("premature destination untouched"),Premature==TArray<uint8>{99});
            TestFalse(TEXT("double consume refuses without replacing first snapshot"),Scope.ConsumeCanonicalSnapshot1456(Second));
            TestFalse(TEXT("copy from consumed source cannot produce empty certificate"),Scope.CaptureCanonicalSnapshot1456(Second));
            {
                TArray<uint8> Inner;FFramePointerCaptureScope Nested(Inner,Registry);
                FEpisodeStateBytesForTesting::Add(Inner,Prefix);FEpisodeStateBytesForTesting::Add(Inner,&Object);
                FEpisodeStateBytesForTesting::Add(Inner,static_cast<int*>(nullptr));FEpisodeStateBytesForTesting::Add(Inner,Suffix);
                TArray<uint8> Canonical;
                TestTrue(TEXT("independent nested scope stays usable"),Nested.MakeCanonical(Canonical));
                TestTrue(TEXT("nested raw/canonical remains complete"),Inner==ExpectedRaw&&Canonical==ExpectedCanonical);
                TestFalse(TEXT("nested copy cannot overwrite an active consumed destination"),Nested.CaptureCanonicalSnapshot1456(Consumed));
            }
        }
        TArray<uint8> A,B;
        TestTrue(TEXT("closed transfer canonicalizes"),Consumed.MakeCanonical(A));
        TestTrue(TEXT("unchanged copy canonicalizes"),Copy.MakeCanonical(B));
        TestTrue(TEXT("copy and transfer retain exact independent raw bytes"),Copy.RawForEvidence1456()==ExpectedRaw&&Consumed.RawForEvidence1456()==ExpectedRaw);
        TestTrue(TEXT("copy and transfer match independent canonical oracle"),A==ExpectedCanonical&&B==ExpectedCanonical);
    }
    // Nonempty membership is an owned semantic value, not a registry reference.
    {
        FFramePointerKey MemberKey=Key;MemberKey.Membership={11,17,29};FFramePointerRegistry MemberRegistry;
        if(!TestTrue(TEXT("membership identity registers"),MemberRegistry.Register(&Object,MemberKey))||!TestTrue(TEXT("membership registry seals"),MemberRegistry.Seal()))return false;
        FFramePointerCanonicalSnapshot1456 Consumed;TArray<uint8> Source,Expected;
        Scalar(Expected,MemberKey.Domain);Scalar(Expected,MemberKey.Owner);Scalar(Expected,MemberKey.Generation);
        Scalar(Expected,MemberKey.Role);Scalar(Expected,MemberKey.Element);Scalar(Expected,uint32(3));
        for(uint64 Member:MemberKey.Membership)Scalar(Expected,Member);
        {
            FFramePointerCaptureScope Scope(Source,MemberRegistry);FEpisodeStateBytesForTesting::Add(Source,&Object);
            TestTrue(TEXT("membership stream transfers"),Scope.ConsumeCanonicalSnapshot1456(Consumed));
        }
        TArray<uint8> Actual;TestTrue(TEXT("transferred membership projects"),Consumed.MakeCanonical(Actual));
        TestTrue(TEXT("all ordered membership scalars preserved"),Actual==Expected);
    }
    for(int32 Failure=0;Failure<3;++Failure)
    {
        FFramePointerCanonicalSnapshot1456 Consumed;TArray<uint8> Source,Before;
        {
            FFramePointerCaptureScope Scope(Source,Registry,Failure==1?1:16384);
            FEpisodeStateBytesForTesting::Add(Source,Failure==0?&Unknown:&Object);
            if(Failure==1)FEpisodeStateBytesForTesting::Add(Source,&Object);
            if(Failure==2)Source[0]^=1;
            Before=Source;
            if(Failure<2)
            {
                TestFalse(TEXT("unknown/overflow transfer refuses transactionally"),Scope.ConsumeCanonicalSnapshot1456(Consumed));
                TestTrue(TEXT("failed transfer retains source and empty destination"),Source==Before&&Consumed.RawForEvidence1456().IsEmpty());
            }
            else
            {
                TestTrue(TEXT("transfer retains raw drift for original validator"),Scope.ConsumeCanonicalSnapshot1456(Consumed));
                TestTrue(TEXT("transferred drift bytes are not normalized"),Consumed.RawForEvidence1456()==Before);
            }
        }
        TArray<uint8> Out{99};TestFalse(TEXT("closed invalid transfer never canonicalizes"),Consumed.MakeCanonical(Out));
        TestTrue(TEXT("invalid transfer leaves output untouched"),Out==TArray<uint8>{99});
    }
    for(int32 LateKind=0;LateKind<2;++LateKind)
    {
        FFramePointerCanonicalSnapshot1456 Consumed;TArray<uint8> Source;
        {
            FFramePointerCaptureScope Scope(Source,Registry);FEpisodeStateBytesForTesting::Add(Source,&Object);
            TestTrue(TEXT("misuse fixture initially transfers"),Scope.ConsumeCanonicalSnapshot1456(Consumed));
            if(LateKind==0)FEpisodeStateBytesForTesting::Add(Source,Prefix); // unobserved scalar
            else FEpisodeStateBytesForTesting::Add(Source,&Object); // observed pointer
        }
        TArray<uint8> Out{99};TestFalse(TEXT("late source write invalidates closed certificate"),Consumed.MakeCanonical(Out));
        TestTrue(TEXT("late write never produces partial canonical bytes"),Out==TArray<uint8>{99});
    }
    {
        FFramePointerCanonicalSnapshot1456 Consumed;TArray<uint8> Source;
        {
            FFrameSolveWitnessScope Solve(9,Source,Registry);
            TestTrue(TEXT("closed empty solve stream can be transferred"),Solve.SealAndConsumeCanonicalSnapshot1456(Consumed));
            const uint64 Pass=FFrameSolveWitnessScope::BeginDynamicPass(9,EFrameDynamicContactPass::Persistent,1.f/300,0,false,false,0);
            TestEqual(TEXT("late physical record admission is rejected"),Pass,uint64(0));
            TestTrue(TEXT("late physical record is visibly UNKNOWN"),Solve.HasOverflow());
            TestTrue(TEXT("rejected late physical record appends no raw bytes"),Source.IsEmpty());
        }
        TArray<uint8> Out{99};TestFalse(TEXT("late physical record invalidates deferred snapshot"),Consumed.MakeCanonical(Out));
    }
    return true;
}
#endif
