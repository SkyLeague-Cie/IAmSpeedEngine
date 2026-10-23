#include "FramePointerWitness.h"
#include "FrameSolveWitness.h"
#include "IAmSpeed/Base/EpisodeStateBytesForTesting.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "IAmSpeed/SubBodies/Solid/BoxSubBody.h"
#include "IAmSpeed/World/Analytic/AnalyticWorldData.h"
#include "UObject/StrongObjectPtr.h"
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
    {
        // Nonempty membership owns heap storage. Exercise every observation
        // entrypoint, event-array growth, semantic pair swaps and final array
        // transfer against an independent explicit scalar-byte oracle.
        TStrongObjectPtr<UBoxSubBody> Left(NewObject<UBoxSubBody>()),Right(NewObject<UBoxSubBody>());
        if(!TestTrue(TEXT("membership bodies exist"),Left.IsValid()&&Right.IsValid()))return false;
        FFramePointerKey LeftKey{3,77,5,9,11},RightKey{3,77,5,9,11};
        LeftKey.Membership={13,21,34};RightKey.Membership={8,55};
        FFramePointerRegistry Members;
        if(!TestTrue(TEXT("left membership declared"),Members.Register(Left.Get(),LeftKey))
            ||!TestTrue(TEXT("right membership declared"),Members.Register(Right.Get(),RightKey))
            ||!TestTrue(TEXT("membership registry seals"),Members.Seal()))return false;
        TArray<uint8> Raw,ExpectedRaw,ExpectedCanonical,Actual;
        const auto Scalar=[](TArray<uint8>& Out,const auto& Value)
        {const int32 At=Out.AddUninitialized(sizeof(Value));FMemory::Memcpy(Out.GetData()+At,&Value,sizeof(Value));};
        const auto KeyBytes=[&](const FFramePointerKey& K)
        {
            Scalar(ExpectedCanonical,K.Domain);Scalar(ExpectedCanonical,K.Owner);Scalar(ExpectedCanonical,K.Generation);
            Scalar(ExpectedCanonical,K.Role);Scalar(ExpectedCanonical,K.Element);Scalar(ExpectedCanonical,uint32(K.Membership.Num()));
            for(uint64 Id:K.Membership)Scalar(ExpectedCanonical,Id);
        };
        const uint32 Prefix=0x91a7c3e5u;const uint16 Suffix=0x6bd2u;
        const uint32 LeftId=Left->GetUniqueID(),RightId=Right->GetUniqueID();
        const uint32 Lo=LeftId<RightId?LeftId:RightId,Hi=LeftId<RightId?RightId:LeftId;
        const uint64 Pair=(uint64(Hi)<<32)|uint64(Lo),Dynamic=(uint64(Lo)<<32)|uint64(Hi);
        const uint64 Provider=0x73519ac2468def01ull;
        const uint64 Solver=Speed::Analytic::CombineStableIds(Pair,Provider);
        FFramePointerCanonicalSnapshot1456 Snapshot;
        {
            FFramePointerCaptureScope Scope(Raw,Members);
            FEpisodeStateBytesForTesting::Add(Raw,Prefix);Scalar(ExpectedRaw,Prefix);Scalar(ExpectedCanonical,Prefix);
            for(int32 I=0;I<32;++I)
            {
                UBoxSubBody* Body=(I&1)?Right.Get():Left.Get();
                FEpisodeStateBytesForTesting::Add(Raw,Body);Scalar(ExpectedRaw,reinterpret_cast<UPTRINT>(Body));
                KeyBytes((I&1)?RightKey:LeftKey);
            }
            FFramePointerCaptureScope::ObserveObjectId(Raw,LeftId,Left.Get());
            FEpisodeStateBytesForTesting::Add(Raw,LeftId);Scalar(ExpectedRaw,LeftId);
            Scalar(ExpectedCanonical,uint32(0xfffffffdu));KeyBytes(LeftKey);
            for(int32 Reverse=0;Reverse<2;++Reverse)
            {
                UBoxSubBody* Source=Reverse?Right.Get():Left.Get();
                UBoxSubBody* OtherBody=Reverse?Left.Get():Right.Get();
                FFramePointerCaptureScope::ObservePairToken(Raw,Pair,Source,OtherBody);
                FEpisodeStateBytesForTesting::Add(Raw,Pair);Scalar(ExpectedRaw,Pair);
                Scalar(ExpectedCanonical,uint32(0xfffffffeu));KeyBytes(RightKey);KeyBytes(LeftKey);
                SHitResult Hit;Hit.Component=OtherBody;Hit.CanonicalGroupId=Provider;
                FFramePointerCaptureScope::ObserveSolverPairToken(Raw,Solver,Source,Hit,false);
                FEpisodeStateBytesForTesting::Add(Raw,Solver);Scalar(ExpectedRaw,Solver);
                Scalar(ExpectedCanonical,uint32(0xfffffffcu));KeyBytes(Reverse?RightKey:LeftKey);KeyBytes(Reverse?LeftKey:RightKey);
                Scalar(ExpectedCanonical,Provider);Scalar(ExpectedCanonical,uint64(0));
                FFramePointerCaptureScope::ObserveDynamicPairToken(Raw,Dynamic,Source,OtherBody);
                FEpisodeStateBytesForTesting::Add(Raw,Dynamic);Scalar(ExpectedRaw,Dynamic);
                Scalar(ExpectedCanonical,uint32(0xfffffffeu));KeyBytes(RightKey);KeyBytes(LeftKey);
            }
            FEpisodeStateBytesForTesting::Add(Raw,Suffix);Scalar(ExpectedRaw,Suffix);Scalar(ExpectedCanonical,Suffix);
            TestEqual(TEXT("all membership observations retained through growth"),Scope.Observations().Num(),39);
            TestTrue(TEXT("membership raw oracle is exact"),Raw==ExpectedRaw);
            TestTrue(TEXT("membership live projection succeeds"),Scope.MakeCanonical(Actual));
            TestTrue(TEXT("membership canonical scalar oracle is exact"),Actual==ExpectedCanonical);
            TestTrue(TEXT("left declaration not moved from"),Members.Find(reinterpret_cast<UPTRINT>(Left.Get()))->Membership==LeftKey.Membership);
            TestTrue(TEXT("right declaration not moved from"),Members.Find(reinterpret_cast<UPTRINT>(Right.Get()))->Membership==RightKey.Membership);
            TestTrue(TEXT("membership final array transfer succeeds"),Scope.ConsumeCanonicalSnapshot1456(Snapshot));
            TArray<uint8> Open{0x42};TestFalse(TEXT("membership transferred snapshot awaits scope closure"),Snapshot.MakeCanonical(Open));
            TestTrue(TEXT("open membership output unchanged"),Open==TArray<uint8>{0x42});
        }
        TArray<uint8> Deferred;
        TestTrue(TEXT("membership transferred raw oracle is exact"),Snapshot.RawForEvidence1456()==ExpectedRaw);
        TestTrue(TEXT("membership deferred projection succeeds"),Snapshot.MakeCanonical(Deferred));
        TestTrue(TEXT("membership deferred oracle survives event swaps"),Deferred==ExpectedCanonical);
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
            TArray<uint8> ScratchOut,ScratchBuild{77};
            TestFalse(TEXT("consumed source cannot use explicit scratch projector"),Scope.MakeCanonicalWithScratch1368(ScratchOut,ScratchBuild));
            TestTrue(TEXT("consumed refusal preserves scratch destination and build"),ScratchOut.IsEmpty()&&ScratchBuild==TArray<uint8>{77});
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
    // Raw recovery is explicitly separate from canonical validity. This is
    // required when an incomplete multi-frame run still exports earlier frames.
    {
        FFramePointerCanonicalSnapshot1456 Consumed,UncapturedSnapshot;TArray<uint8> Source;
        TArray<uint8> Sentinel{99};
        TestTrue(TEXT("uncaptured raw recovery is explicit and non-mutating"),
            UncapturedSnapshot.CopyRawForEvidenceAfterClosure1456(Sentinel)==EFramePointerRawEvidenceCopy1456::NotCaptured&&Sentinel==TArray<uint8>{99});
        {
            FFramePointerCaptureScope Scope(Source,Registry);
            FEpisodeStateBytesForTesting::Add(Source,Prefix);FEpisodeStateBytesForTesting::Add(Source,&Object);
            FEpisodeStateBytesForTesting::Add(Source,static_cast<int*>(nullptr));FEpisodeStateBytesForTesting::Add(Source,Suffix);
            TestTrue(TEXT("boundary oracle stream consumes"),Scope.ConsumeCanonicalSnapshot1456(Consumed));
            TestTrue(TEXT("raw recovery refuses an open writer without touching destination"),
                Consumed.CopyRawForEvidenceAfterClosure1456(Sentinel)==EFramePointerRawEvidenceCopy1456::WriterScopeOpen&&Sentinel==TArray<uint8>{99});
        }
        TestTrue(TEXT("raw recovery refuses to overwrite unrelated bytes"),
            Consumed.CopyRawForEvidenceAfterClosure1456(Sentinel)==EFramePointerRawEvidenceCopy1456::DestinationNotEmpty&&Sentinel==TArray<uint8>{99});
        TArray<uint8> Recovered;
        TestTrue(TEXT("closed captured raw is independently exact"),
            Consumed.CopyRawForEvidenceAfterClosure1456(Recovered)==EFramePointerRawEvidenceCopy1456::Copied&&Recovered==ExpectedRaw);
        const int32 PrefixEnd=sizeof(Prefix),FirstEnd=PrefixEnd+sizeof(UPTRINT),SecondEnd=FirstEnd+sizeof(UPTRINT);
        const int32 CanonicalKeyBytes=(ExpectedCanonical.Num()-sizeof(Prefix)-sizeof(Suffix))/2;
        const TArray<int32> RawBounds{0,PrefixEnd,PrefixEnd,FirstEnd,SecondEnd,ExpectedRaw.Num()};
        const TArray<int32> ExpectedBounds{0,PrefixEnd,PrefixEnd,PrefixEnd+CanonicalKeyBytes,PrefixEnd+2*CanonicalKeyBytes,ExpectedCanonical.Num()};
        TArray<uint8> Canonical;TArray<int32> Projected;
        TestTrue(TEXT("duplicate boundaries and every complete token map exactly"),Consumed.MakeCanonicalWithBoundaries1534(Canonical,RawBounds,Projected));
        TestTrue(TEXT("canonical bytes and field boundaries match independently assembled oracle"),Canonical==ExpectedCanonical&&Projected==ExpectedBounds);
        for(const TArray<int32>& Bad:TArray<TArray<int32>>{{-1},{ExpectedRaw.Num()+1},{1,0},{PrefixEnd+1}})
        {
            TArray<uint8> Untouched{99};TArray<int32> UntouchedBounds{77};
            TestFalse(TEXT("invalid or within-token boundary refuses"),Consumed.MakeCanonicalWithBoundaries1534(Untouched,Bad,UntouchedBounds));
            TestTrue(TEXT("boundary failure cannot publish partial bytes or spans"),Untouched==TArray<uint8>{99}&&UntouchedBounds==TArray<int32>{77});
        }
    }
    {
        FFramePointerCanonicalSnapshot1456 Copy,Consumed;TArray<uint8> Source;TArray<int32> CopyBounds,ConsumeBounds{77};
        {
            FFrameSolveWitnessScope Solve(7,Source,Registry);
            TestTrue(TEXT("row-boundary capture explicitly enabled"),Solve.EnableRowBoundaryCapture1534());
            const uint64 Pass=FFrameSolveWitnessScope::BeginDynamicPass(7,EFrameDynamicContactPass::Persistent,1.f/300,0,false,false,0);
            TestFalse(TEXT("open dynamic pass cannot consume row-framed stream"),Solve.SealAndConsumeCanonicalSnapshotWithRowBoundaries1534(Consumed,ConsumeBounds));
            TestTrue(TEXT("failed consume leaves destination boundaries untouched"),ConsumeBounds==TArray<int32>{77});
            FFrameSolveWitnessScope::EndDynamicPass(Pass,0);
            FFrameSolveWitnessScope::Record(7,2,3,41,0,0,SHitResult{},false,0,1);
            TestFalse(TEXT("1217 unframed API still refuses row-framed capture"),Solve.SealAndConsumeCanonicalSnapshot1456(Consumed));
            TestTrue(TEXT("copy control retains schema1 and schema2 rows"),Solve.CaptureCanonicalSnapshotWithRowBoundaries1534(Copy,CopyBounds));
            TestEqual(TEXT("copy boundaries include every row and terminator"),CopyBounds.Num(),Solve.NumRows()+1);
            TestTrue(TEXT("dedicated transfer retains row-framed stream"),Solve.SealAndConsumeCanonicalSnapshotWithRowBoundaries1534(Consumed,ConsumeBounds));
            TestTrue(TEXT("transferred exact raw and boundaries match unchanged capture"),Consumed.RawForEvidence1456()==Copy.RawForEvidence1456()&&ConsumeBounds==CopyBounds&&Source.IsEmpty());
            FFramePointerCanonicalSnapshot1456 Second;TArray<int32> SecondBounds{77};
            TestFalse(TEXT("row-framed transfer is one shot"),Solve.SealAndConsumeCanonicalSnapshotWithRowBoundaries1534(Second,SecondBounds));
            TestTrue(TEXT("repeated transfer cannot replace either certificate or spans"),SecondBounds==TArray<int32>{77}&&Second.RawForEvidence1456().IsEmpty()&&Consumed.RawForEvidence1456()==Copy.RawForEvidence1456());
            TArray<uint8> Premature{99};TArray<int32> Spans{77};
            TestFalse(TEXT("row-framed projection also waits for writer closure"),Consumed.MakeCanonicalWithBoundaries1534(Premature,ConsumeBounds,Spans));
            TestTrue(TEXT("open scope leaves row projection destinations untouched"),Premature==TArray<uint8>{99}&&Spans==TArray<int32>{77});
        }
        TArray<uint8> A,B,Recovered;TArray<int32> ABounds,BBounds;
        TestTrue(TEXT("copy row projector succeeds after closure"),Copy.MakeCanonicalWithBoundaries1534(A,CopyBounds,ABounds));
        TestTrue(TEXT("consumed row projector succeeds after closure"),Consumed.MakeCanonicalWithBoundaries1534(B,ConsumeBounds,BBounds));
        TestTrue(TEXT("main/dynamic canonical bytes and all row spans retained"),A==B&&ABounds==BBounds);
        TestTrue(TEXT("complete raw row pack recovers after closure"),Consumed.CopyRawForEvidenceAfterClosure1456(Recovered)==EFramePointerRawEvidenceCopy1456::Copied&&Recovered==Copy.RawForEvidence1456());
    }
    for(int32 Failure=0;Failure<2;++Failure)
    {
        FFramePointerCanonicalSnapshot1456 Destination;TArray<uint8> Source;TArray<int32> Bounds{77};
        {
            FFrameSolveWitnessScope Solve(8,Source,Registry,1);
            TestTrue(TEXT("negative row transfer explicitly enables boundaries"),Solve.EnableRowBoundaryCapture1534());
            if(Failure==0)FEpisodeStateBytesForTesting::Add(Source,&Unknown);
            else
            {
                FFrameSolveWitnessScope::Record(8,0,0,41,0,0,SHitResult{},false,0,1);
                FFrameSolveWitnessScope::Record(8,0,1,41,0,0,SHitResult{},false,0,1);
            }
            const TArray<uint8> Before=Source;
            TestFalse(TEXT("unknown pointer or solve overflow refuses row transfer"),Solve.SealAndConsumeCanonicalSnapshotWithRowBoundaries1534(Destination,Bounds));
            TestTrue(TEXT("rejected row transfer preserves raw and both output destinations"),Source==Before&&Destination.RawForEvidence1456().IsEmpty()&&Bounds==TArray<int32>{77});
        }
    }
    {
        // Simulate a failed second frame: recovery must not be hidden behind
        // Finished && !Failed, and the unknown partial frame stays original.
        FFramePointerCanonicalSnapshot1456 Captured[3];TArray<uint8> Sources[3];
        {
            FFramePointerCaptureScope First(Sources[0],Registry);FEpisodeStateBytesForTesting::Add(Sources[0],&Object);
            TestTrue(TEXT("earlier successful frame consumes"),First.ConsumeCanonicalSnapshot1456(Captured[0]));
        }
        {
            FFramePointerCaptureScope Partial(Sources[1],Registry);FEpisodeStateBytesForTesting::Add(Sources[1],&Unknown);
            TestFalse(TEXT("unknown partial frame refuses consume without losing raw"),Partial.ConsumeCanonicalSnapshot1456(Captured[1]));
        }
        const TArray<uint8> PartialRaw=Sources[1];
        TestTrue(TEXT("prior frame recovers although later frame failed"),Captured[0].CopyRawForEvidenceAfterClosure1456(Sources[0])==EFramePointerRawEvidenceCopy1456::Copied);
        TestTrue(TEXT("non-consumed partial raw stays untouched"),Captured[1].CopyRawForEvidenceAfterClosure1456(Sources[1])==EFramePointerRawEvidenceCopy1456::NotCaptured&&Sources[1]==PartialRaw);
        TArray<uint8> ActualPack,ExpectedPack,FirstRaw,SecondRaw;
        Scalar(FirstRaw,reinterpret_cast<UPTRINT>(&Object));Scalar(SecondRaw,reinterpret_cast<UPTRINT>(&Unknown));
        FEpisodeStateBytesForTesting::Add(ActualPack,uint32(0x31434149));FEpisodeStateBytesForTesting::Add(ActualPack,uint32(3));
        Scalar(ExpectedPack,uint32(0x31434149));Scalar(ExpectedPack,uint32(3));
        for(uint32 N=0;N<3;++N)
        {
            FEpisodeStateBytesForTesting::Add(ActualPack,uint64(N));FEpisodeStateBytesForTesting::Add(ActualPack,uint32(Sources[N].Num()));ActualPack.Append(Sources[N]);
            const TArray<uint8>& Expected=N==0?FirstRaw:N==1?SecondRaw:Sources[2];
            Scalar(ExpectedPack,uint64(N));Scalar(ExpectedPack,uint32(Expected.Num()));ExpectedPack.Append(Expected);
        }
        TestTrue(TEXT("failed-run pack preserves complete earlier raw and partial unknown bytes with exact framing"),ActualPack==ExpectedPack);
    }
    for(int32 Kind=0;Kind<2;++Kind)
    {
        FFramePointerCanonicalSnapshot1456 Consumed;TArray<uint8> Source,Original,Debris;
        {
            FFramePointerCaptureScope Scope(Source,Registry);FEpisodeStateBytesForTesting::Add(Source,&Object);Original=Source;
            if(Kind==0)Source[0]^=1; // Original projector must still reject raw drift.
            Original=Source;
            TestTrue(TEXT("forensic stream consumes without normalizing bytes"),Scope.ConsumeCanonicalSnapshot1456(Consumed));
            if(Kind==1)FEpisodeStateBytesForTesting::Add(Source,Prefix); // Illegal unobserved append.
        }
        if(Kind==1)
        {
            const TArray<uint8> Before=Source;
            TestTrue(TEXT("late source debris is never overwritten by recovery"),Consumed.CopyRawForEvidenceAfterClosure1456(Source)==EFramePointerRawEvidenceCopy1456::DestinationNotEmpty&&Source==Before);
            Swap(Source,Debris);
            TestTrue(TEXT("late source scalar bytes survive separately"),Debris==Before&&!Debris.IsEmpty());
        }
        TestTrue(TEXT("UNKNOWN raw remains exportable after writer closure"),Consumed.CopyRawForEvidenceAfterClosure1456(Source)==EFramePointerRawEvidenceCopy1456::Copied&&Source==Original);
        TArray<uint8> Canonical{99};TestFalse(TEXT("forensic recovery never makes an invalid certificate pass"),Consumed.MakeCanonical(Canonical));
        TestTrue(TEXT("invalid projection destination remains untouched"),Canonical==TArray<uint8>{99});
    }

    // Independent schema3 bytes: no production wheel encoder/projector is used
    // to assemble this oracle. A fake typed owner is only an identity token;
    // RecordWheelManifold never dereferences it.
    const auto* WheelOwner1368=reinterpret_cast<const ISpeedComponent*>(&Object);
    const auto EmitWheel1368=[&](uint8 Event,TConstArrayView<float> Values)
    {FFrameSolveWitnessScope::RecordWheelManifold1368(Event,WheelOwner1368,3,4,{}, {},Values,{}, {});};
    const auto IndependentWheel1368=[&](TArray<uint8>& Pack,uint64 FrameNumber,int32 Sequence,uint8 Event,TConstArrayView<float> Values,bool CanonicalOwner)
    {
        TArray<uint8> Row;
        Scalar(Row,uint32(3));Scalar(Row,FrameNumber);Scalar(Row,Sequence);Scalar(Row,int32(0));Scalar(Row,Event);
        if(CanonicalOwner)
        {
            Scalar(Row,Key.Domain);Scalar(Row,Key.Owner);Scalar(Row,Key.Generation);Scalar(Row,Key.Role);Scalar(Row,Key.Element);Scalar(Row,uint32(0));
        }
        else Scalar(Row,reinterpret_cast<UPTRINT>(&Object));
        Scalar(Row,int32(3));Scalar(Row,int32(4));Scalar(Row,int32(0));Scalar(Row,int32(0));
        Scalar(Row,int32(Values.Num()));for(float Value:Values)Scalar(Row,Value);
        Scalar(Row,int32(0));Scalar(Row,int32(0));
        Scalar(Pack,uint32(0x334d5749));Scalar(Pack,uint32(Row.Num()));Pack.Append(Row);
    };
    TArray<float> LargeWheelValues1368;for(int32 I=0;I<32;++I)LargeWheelValues1368.Add(float(I)-17.5f);
    const float SmallWheelValues1368[]={-0.0f};
    // Opt-in two-buffer projection uses the same independent scalar/key oracle.
    // Warm both arrays at the largest row before checking storage reuse; this
    // is a buffer invariant, not an allocation-free or cadence assertion.
    {
        TArray<uint8> Canonical,Build;
        const uint8* WarmA=nullptr;const uint8* WarmB=nullptr;int32 WarmCapacity=0;
        const int32 Sizes[]={32,32,32,1,-1,32,1,-1};
        for(int32 Pass=0;Pass<int32(UE_ARRAY_COUNT(Sizes));++Pass)
        {
            TArray<uint8> Source,Expected,Legacy;
            FFramePointerCaptureScope Scope(Source,Registry);
            if(Sizes[Pass]>=0)
            {
                FEpisodeStateBytesForTesting::Add(Source,&Object);
                FEpisodeStateBytesForTesting::Add(Source,Sizes[Pass]);
                Scalar(Expected,Key.Domain);Scalar(Expected,Key.Owner);Scalar(Expected,Key.Generation);
                Scalar(Expected,Key.Role);Scalar(Expected,Key.Element);Scalar(Expected,uint32(0));Scalar(Expected,Sizes[Pass]);
                for(int32 N=0;N<Sizes[Pass];++N)
                {
                    const float Value=Sizes[Pass]==1?-0.0f:float(N)-17.5f;
                    FEpisodeStateBytesForTesting::Add(Source,Value);Scalar(Expected,Value);
                }
            }
            const TArray<uint8> Original=Source;
            TestTrue(TEXT("scratch control uses unchanged legacy API"),Scope.MakeCanonical(Legacy));
            Canonical.Reset();
            TestTrue(TEXT("explicit scratch projection accepts large small and empty rows"),Scope.MakeCanonicalWithScratch1368(Canonical,Build));
            TestTrue(TEXT("scratch and legacy match independent bytes including signed zero"),Canonical==Expected&&Legacy==Expected&&Source==Original);
            if(Pass==1){WarmA=Canonical.GetData();WarmB=Build.GetData();WarmCapacity=Canonical.Max();}
            if(Pass>=2)
            {
                TestTrue(TEXT("both warmed canonical allocations remain owned and alternate"),
                    (Canonical.GetData()==WarmA&&Build.GetData()==WarmB)||(Canonical.GetData()==WarmB&&Build.GetData()==WarmA));
                TestTrue(TEXT("short and empty canonical rows retain warmed capacities"),Canonical.Max()==WarmCapacity&&Build.Max()==WarmCapacity);
            }
        }
    }
    {
        TArray<uint8> Source,Sentinel{99},Build{88},Empty;
        FFramePointerCaptureScope Scope(Source,Registry);FEpisodeStateBytesForTesting::Add(Source,&Object);
        const TArray<uint8> Original=Source;
        TestFalse(TEXT("scratch nonempty output refuses transactionally"),Scope.MakeCanonicalWithScratch1368(Sentinel,Build));
        TestTrue(TEXT("refusal preserves output build and raw"),Sentinel==TArray<uint8>{99}&&Build==TArray<uint8>{88}&&Source==Original);
        TestFalse(TEXT("scratch output and build must not alias"),Scope.MakeCanonicalWithScratch1368(Empty,Empty));
        TestFalse(TEXT("scratch build cannot alias retained raw"),Scope.MakeCanonicalWithScratch1368(Empty,Source));
        TestFalse(TEXT("scratch output cannot alias retained raw"),Scope.MakeCanonicalWithScratch1368(Source,Build));
        TestTrue(TEXT("alias refusals leave every byte intact"),Source==Original&&Empty.IsEmpty()&&Build==TArray<uint8>{88});
    }
    for(int32 Failure=0;Failure<4;++Failure)
    {
        TArray<uint8> Source,Sentinel{99},Build{88};
        FFramePointerCaptureScope Scope(Source,Registry,Failure==1?1:16);
        FEpisodeStateBytesForTesting::Add(Source,Failure==0?&Unknown:&Object);
        if(Failure==1)FEpisodeStateBytesForTesting::Add(Source,&Object);
        if(Failure==2)Source[0]^=1;
        if(Failure==3){FEpisodeStateBytesForTesting::Add(Source,&Object);Source[int32(sizeof(UPTRINT))]^=1;}
        const TArray<uint8> Original=Source;
        TestFalse(TEXT("scratch UNKNOWN overflow and raw drift preserve sentinel output"),Scope.MakeCanonicalWithScratch1368(Sentinel,Build));
        TestTrue(TEXT("failed sentinel transaction preserves exact evidence"),Sentinel==TArray<uint8>{99}&&Build==TArray<uint8>{88}&&Source==Original);
        Sentinel.Reset();
        TestFalse(TEXT("scratch validation rejects invalid source even with empty destination"),Scope.MakeCanonicalWithScratch1368(Sentinel,Build));
        TestTrue(TEXT("partial scratch is not promoted to canonical evidence"),Sentinel.IsEmpty()&&Source==Original);
        if(Failure==3)TestFalse(TEXT("failure after a valid first token retains uncertified partial scratch only"),Build.IsEmpty());
    }
    // Run the existing independent full-wheel framing/lifecycle/failure-pack
    // oracles under BOTH legacy allocation and the opt-in scratch lease.
    for(int32 ReuseMode1368=0;ReuseMode1368<2;++ReuseMode1368)
    {
    for(uint64 FrameNumber1368=0;FrameNumber1368<2;++FrameNumber1368)
    {
        FWheelWitnessSnapshot1368 Snapshot1368;TArray<uint8> Main,ControlRaw,ControlCanonical,ExpectedWheelRaw,ExpectedWheelCanonical;
        TArray<uint8> RawOut,CanonicalOut,RawDebris,CanonicalDebris;RawOut.Reserve(512*1024);
        {
            FFrameSolveWitnessScope Solve(FrameNumber1368,Main,Registry);
            TestTrue(TEXT("wheel transfer test enables real observer"),Solve.EnableWheelManifoldCapture1368(ReuseMode1368!=0));
            const TConstArrayView<float> Large=FrameNumber1368==0?TConstArrayView<float>(LargeWheelValues1368):TConstArrayView<float>{};
            EmitWheel1368(0,Large);EmitWheel1368(1,MakeArrayView(SmallWheelValues1368));EmitWheel1368(255,{});
            IndependentWheel1368(ExpectedWheelRaw,FrameNumber1368,0,0,Large,false);
            IndependentWheel1368(ExpectedWheelRaw,FrameNumber1368,1,1,MakeArrayView(SmallWheelValues1368),false);
            IndependentWheel1368(ExpectedWheelRaw,FrameNumber1368,2,255,{},false);
            IndependentWheel1368(ExpectedWheelCanonical,FrameNumber1368,0,0,Large,true);
            IndependentWheel1368(ExpectedWheelCanonical,FrameNumber1368,1,1,MakeArrayView(SmallWheelValues1368),true);
            IndependentWheel1368(ExpectedWheelCanonical,FrameNumber1368,2,255,{},true);
            ControlRaw=Solve.RawWheelManifold1368();TestTrue(TEXT("legacy canonical wheel copy remains available"),Solve.MakeWheelManifoldCanonical1368(ControlCanonical));
            TestTrue(TEXT("legacy wheel raw and canonical match independent byte oracle"),ControlRaw==ExpectedWheelRaw&&ControlCanonical==ExpectedWheelCanonical);
            TestTrue(TEXT("final wheel capture transfers complete streams"),Solve.ConsumeWheelWitness1368(Snapshot1368));
            TestFalse(TEXT("legacy canonical API cannot bless emptied transferred source"),Solve.MakeWheelManifoldCanonical1368(CanonicalOut));
            TestTrue(TEXT("release cannot precede scope closure"),Snapshot1368.ReleaseAfterClosure(RawOut,CanonicalOut,RawDebris,CanonicalDebris)==EWheelWitnessRelease1368::WriterOpen&&RawOut.IsEmpty());
            FWheelWitnessSnapshot1368 Duplicate;
            TestFalse(TEXT("wheel capture is one shot"),Solve.ConsumeWheelWitness1368(Duplicate));
            // A nested unrelated solve does not change the outer certificate.
            TArray<uint8> NestedRaw;{FFrameSolveWitnessScope Nested(99,NestedRaw,Registry);TestTrue(TEXT("nested solve remains independent"),Nested.EnableWheelManifoldCapture1368(ReuseMode1368!=0));EmitWheel1368(0,{});EmitWheel1368(255,{});}
        }
        TArray<uint8> Sentinel{99};
        TestTrue(TEXT("nonempty release destination is transactional"),Snapshot1368.ReleaseAfterClosure(Sentinel,CanonicalOut,RawDebris,CanonicalDebris)==EWheelWitnessRelease1368::DestinationNotEmpty&&Sentinel==TArray<uint8>{99}&&CanonicalOut.IsEmpty());
        TestTrue(TEXT("alias release destinations refuse without transfer"),Snapshot1368.ReleaseAfterClosure(RawOut,RawOut,RawDebris,CanonicalDebris)==EWheelWitnessRelease1368::AliasedDestinations);
        TestTrue(TEXT("closed wheel release restores exact complete raw and canonical evidence"),Snapshot1368.ReleaseAfterClosure(RawOut,CanonicalOut,RawDebris,CanonicalDebris)==EWheelWitnessRelease1368::TransferredValid&&RawOut==ControlRaw&&CanonicalOut==ControlCanonical&&RawDebris.IsEmpty()&&CanonicalDebris.IsEmpty());
        TArray<uint8> AgainRaw,AgainCanonical;
        TestTrue(TEXT("closed evidence cannot be released twice"),Snapshot1368.ReleaseAfterClosure(AgainRaw,AgainCanonical,RawDebris,CanonicalDebris)==EWheelWitnessRelease1368::AlreadyReleased&&AgainRaw.IsEmpty());
    }
    for(int32 Failure1368=0;Failure1368<4;++Failure1368)
    {
        TArray<uint8> Main,OriginalRaw,OriginalCanonical,OutRaw,OutCanonical,DebrisRaw,DebrisCanonical;
        FWheelWitnessSnapshot1368 Captured;FFramePointerCanonicalSnapshot1456 MainCaptured1368;
        {
            FFrameSolveWitnessScope Solve(17,Main,Registry);
            Solve.EnableWheelManifoldCapture1368(ReuseMode1368!=0);EmitWheel1368(0,{});EmitWheel1368(255,{});
            OriginalRaw=Solve.RawWheelManifold1368();Solve.MakeWheelManifoldCanonical1368(OriginalCanonical);
            TestTrue(TEXT("main contact certificate seals before final wheel transfer"),Solve.SealAndConsumeCanonicalSnapshot1456(MainCaptured1368));
            TestTrue(TEXT("late-write test first captures a complete wheel stream"),Solve.ConsumeWheelWitness1368(Captured));
            if(Failure1368==0)EmitWheel1368(0,{}); // Late observed callback is rejected and invalidates.
            if(Failure1368==1)FFrameSolveWitnessScope::Record(17,0,0,41,0,0,SHitResult{},false,0,1);
            if(Failure1368==2)const_cast<TArray<uint8>&>(Solve.RawWheelManifold1368()).Add(0x5a); // Unobserved source debris.
            if(Failure1368==3)FFrameSolveWitnessScope::BeginDynamicPass(17,EFrameDynamicContactPass::Persistent,1.f/300,0,false,false,0);
        }
        TestTrue(TEXT("late callback or source debris cannot become valid after closure"),Captured.ReleaseAfterClosure(OutRaw,OutCanonical,DebrisRaw,DebrisCanonical)==EWheelWitnessRelease1368::TransferredInvalid);
        TestTrue(TEXT("invalid late stream still preserves exact original captured pack bytes"),OutRaw==OriginalRaw&&OutCanonical==OriginalCanonical);
        if(Failure1368!=2){TArray<uint8> MainCanonical1368;TestFalse(TEXT("late observed callback also invalidates the existing main certificate"),MainCaptured1368.MakeCanonical(MainCanonical1368));}
        TestTrue(TEXT("unobserved raw source debris is retained separately"),Failure1368==2?DebrisRaw==TArray<uint8>{0x5a}:DebrisRaw.IsEmpty());
    }
    for(int32 Failure1368=0;Failure1368<3;++Failure1368)
    {
        TArray<uint8> Main,RawBefore,CanonicalOut,OutRaw,DebrisRaw,DebrisCanonical;FWheelWitnessSnapshot1368 Captured;
        {
            FFrameSolveWitnessScope Solve(18,Main,Registry,Failure1368==2?1:65536);Solve.EnableWheelManifoldCapture1368(ReuseMode1368!=0);
            if(Failure1368==0)EmitWheel1368(0,{}); // Owner remains open.
            if(Failure1368==1)FFrameSolveWitnessScope::RecordWheelManifold1368(0,reinterpret_cast<const ISpeedComponent*>(&Unknown),3,4,{},{});
            if(Failure1368==2){EmitWheel1368(0,{});EmitWheel1368(255,{});} // Overflow.
            RawBefore=Solve.RawWheelManifold1368();
            TestFalse(TEXT("open owner unknown or overflow cannot transfer wheel capture"),Solve.ConsumeWheelWitness1368(Captured));
            TestTrue(TEXT("failed transfer preserves partial raw evidence"),Solve.RawWheelManifold1368()==RawBefore);
        }
        TestTrue(TEXT("noncaptured failed frame is distinct and leaves preexisting evidence untouched"),Captured.ReleaseAfterClosure(OutRaw,CanonicalOut,DebrisRaw,DebrisCanonical)==EWheelWitnessRelease1368::NotCaptured&&OutRaw.IsEmpty());
    }

    {
        FWheelWitnessSnapshot1368 Empty;TArray<uint8> Main,RawOut,CanonicalOut,DebrisRaw,DebrisCanonical;
        {FFrameSolveWitnessScope Solve(19,Main,Registry);Solve.EnableWheelManifoldCapture1368(ReuseMode1368!=0);TestTrue(TEXT("empty complete wheel evidence is transferable"),Solve.ConsumeWheelWitness1368(Empty));}
        TestTrue(TEXT("empty wheel stream is not an unobserved frame"),Empty.ReleaseAfterClosure(RawOut,CanonicalOut,DebrisRaw,DebrisCanonical)==EWheelWitnessRelease1368::TransferredValid&&RawOut.IsEmpty()&&CanonicalOut.IsEmpty());
    }
    {
        FWheelWitnessSnapshot1368 Frames1368[3];TArray<uint8> Main[3],RawFrames[3],CanonicalFrames[3],DebrisRaw[3],DebrisCanonical[3];
        TArray<uint8> ExpectedFirstRaw,ExpectedPartialRaw;
        {
            FFrameSolveWitnessScope First(0,Main[0],Registry);First.EnableWheelManifoldCapture1368(ReuseMode1368!=0);EmitWheel1368(0,{});EmitWheel1368(255,{});
            ExpectedFirstRaw=First.RawWheelManifold1368();TestTrue(TEXT("failed-run first frame consumes"),First.ConsumeWheelWitness1368(Frames1368[0]));
        }
        {
            FFrameSolveWitnessScope Partial(1,Main[1],Registry);Partial.EnableWheelManifoldCapture1368(ReuseMode1368!=0);EmitWheel1368(0,{});
            TestFalse(TEXT("failed-run open second frame refuses consume"),Partial.ConsumeWheelWitness1368(Frames1368[1]));
            RawFrames[1]=Partial.RawWheelManifold1368();ExpectedPartialRaw=RawFrames[1];
        }
        for(int32 N=0;N<3;++N)
        {
            const auto Status=Frames1368[N].ReleaseAfterClosure(RawFrames[N],CanonicalFrames[N],DebrisRaw[N],DebrisCanonical[N]);
            TestTrue(TEXT("failure-independent release retains captured versus partial evidence distinction"),N==0?Status==EWheelWitnessRelease1368::TransferredValid:Status==EWheelWitnessRelease1368::NotCaptured);
        }
        TArray<uint8> ActualPack,ExpectedPack;
        Scalar(ActualPack,uint32(0x31434149));Scalar(ActualPack,uint32(3));Scalar(ExpectedPack,uint32(0x31434149));Scalar(ExpectedPack,uint32(3));
        for(uint64 N=0;N<3;++N)
        {
            const TArray<uint8>& Expected=N==0?ExpectedFirstRaw:N==1?ExpectedPartialRaw:RawFrames[2];
            Scalar(ActualPack,N);Scalar(ActualPack,uint32(RawFrames[N].Num()));ActualPack.Append(RawFrames[N]);
            Scalar(ExpectedPack,N);Scalar(ExpectedPack,uint32(Expected.Num()));ExpectedPack.Append(Expected);
        }
        TestTrue(TEXT("failed-run wheel pack framing preserves prior transferred and partial failed raw streams"),ActualPack==ExpectedPack);
    }

    }
    // Row-storage reuse is checked against both the unchanged owned path and
    // a scalar/key byte oracle, including semantic Membership and a failed tail.
    {
        FFramePointerKey MemberKey=Key;MemberKey.Membership={11,17,29};
        FFramePointerRegistry MemberRegistry;
        if(!TestTrue(TEXT("scratch member registry registers"),MemberRegistry.Register(&Object,MemberKey))
            ||!TestTrue(TEXT("scratch member registry seals"),MemberRegistry.Seal()))return false;
        TArray<FFramePointerObservation> Scratch;int32 RetainedCapacity=0;
        TArray<uint8> OwnedRawPack,BorrowedRawPack,ExpectedRawPack,OwnedCanonicalPack,BorrowedCanonicalPack,ExpectedCanonicalPack;
        const int32 Counts[]={65,1,0,3};
        for(int32 RowIndex=0;RowIndex<UE_ARRAY_COUNT(Counts);++RowIndex)
        {
            TArray<uint8> ExpectedRow,ExpectedCan;
            Scalar(ExpectedRow,uint32(RowIndex));Scalar(ExpectedCan,uint32(RowIndex));
            for(int32 J=0;J<Counts[RowIndex];++J)
            {
                const bool IsUnknown=RowIndex==3&&J==2;
                int* Address=IsUnknown?&Unknown:(J%2?nullptr:&Object);
                Scalar(ExpectedRow,reinterpret_cast<UPTRINT>(Address));
                const FFramePointerKey Identity=Address?MemberKey:FFramePointerKey{};
                Scalar(ExpectedCan,Identity.Domain);Scalar(ExpectedCan,Identity.Owner);Scalar(ExpectedCan,Identity.Generation);
                Scalar(ExpectedCan,Identity.Role);Scalar(ExpectedCan,Identity.Element);Scalar(ExpectedCan,uint32(Identity.Membership.Num()));
                for(uint64 Member:Identity.Membership)Scalar(ExpectedCan,Member);
            }
            Scalar(ExpectedRow,Suffix);Scalar(ExpectedCan,Suffix);
            for(int32 Mode=0;Mode<2;++Mode)
            {
                TArray<uint8> Source,Can;
                {
                    FFramePointerCaptureScope Scope(Source,MemberRegistry,16384,Mode?&Scratch:nullptr);
                    FEpisodeStateBytesForTesting::Add(Source,uint32(RowIndex));
                    for(int32 J=0;J<Counts[RowIndex];++J)
                        FEpisodeStateBytesForTesting::Add(Source,RowIndex==3&&J==2?&Unknown:(J%2?static_cast<int*>(nullptr):&Object));
                    FEpisodeStateBytesForTesting::Add(Source,Suffix);
                    const bool Made=Scope.MakeCanonical(Can);
                    TestEqual(TEXT("borrowed/default unknown row eligibility agrees"),Made,RowIndex!=3);
                    TestTrue(TEXT("borrowed/default raw row matches independent scalar bytes"),Source==ExpectedRow);
                    if(Made)TestTrue(TEXT("borrowed/default canonical row matches independent semantic keys"),Can==ExpectedCan);
                    else TestTrue(TEXT("unknown failed row preserves empty canonical destination"),Can.IsEmpty());
                    auto& RawPack=Mode?BorrowedRawPack:OwnedRawPack;
                    Scalar(RawPack,uint32(Source.Num()));RawPack.Append(Source);
                    if(Made){auto& CanonicalPack=Mode?BorrowedCanonicalPack:OwnedCanonicalPack;Scalar(CanonicalPack,uint32(Can.Num()));CanonicalPack.Append(Can);}
                }
                if(Mode)
                {
                    TestTrue(TEXT("row destructor clears observations but retains outer allocation"),Scratch.IsEmpty()&&Scratch.Max()>=RetainedCapacity);
                    RetainedCapacity=Scratch.Max();
                }
            }
            Scalar(ExpectedRawPack,uint32(ExpectedRow.Num()));ExpectedRawPack.Append(ExpectedRow);
            if(RowIndex!=3){Scalar(ExpectedCanonicalPack,uint32(ExpectedCan.Num()));ExpectedCanonicalPack.Append(ExpectedCan);}
        }
        TestTrue(TEXT("all raw rows and canonical prefix survive the unknown tail identically"),OwnedRawPack==ExpectedRawPack&&BorrowedRawPack==ExpectedRawPack&&OwnedCanonicalPack==ExpectedCanonicalPack&&BorrowedCanonicalPack==ExpectedCanonicalPack);
        // Same scratch cannot be reentered; an unrelated nested scratch remains
        // legal and pointer routing resumes on the outer scope afterward.
        TArray<uint8> OuterRaw,InnerRaw,OtherRaw;TArray<FFramePointerObservation> OtherScratch;
        {
            FFramePointerCaptureScope Outer(OuterRaw,Registry,16384,&Scratch);
            FEpisodeStateBytesForTesting::Add(OuterRaw,&Object);
            {
                FFramePointerCaptureScope Reentered(InnerRaw,Registry,16384,&Scratch);
                TestTrue(TEXT("same scratch nested lease fails closed"),Reentered.HasOverflow());
                TestEqual(TEXT("failed nested lease leaves outer observation intact"),Outer.Observations().Num(),1);
            }
            {
                FFramePointerCaptureScope Nested(OtherRaw,Registry,16384,&OtherScratch);
                FEpisodeStateBytesForTesting::Add(OtherRaw,&Object);TArray<uint8> Out;
                TestTrue(TEXT("distinct nested scratch canonicalizes"),Nested.MakeCanonical(Out));
            }
            FEpisodeStateBytesForTesting::Add(OuterRaw,&Object);
            TestEqual(TEXT("outer typed routing survives nested scopes"),Outer.Observations().Num(),2);
            FFramePointerCanonicalSnapshot1456 Refused,Copied;const TArray<uint8> Before=OuterRaw;
            TestFalse(TEXT("borrowed events cannot be consumed out of their lease"),Outer.ConsumeCanonicalSnapshot1456(Refused));
            TestTrue(TEXT("refused consume keeps raw and destination exact"),OuterRaw==Before&&Refused.RawForEvidence1456().IsEmpty());
            TestTrue(TEXT("ordinary copy snapshot remains available on borrowed scope"),Outer.CaptureCanonicalSnapshot1456(Copied));
            FEpisodeStateBytesForTesting::Add(OuterRaw,&Object);
            TestEqual(TEXT("refused consume does not seal or suppress later valid observations"),Outer.Observations().Num(),3);
        }
        TestTrue(TEXT("nested leases return both independent scratch arrays empty"),Scratch.IsEmpty()&&OtherScratch.IsEmpty());
        // Nonempty storage is refused transactionally; it is never Reset by a
        // constructor which did not acquire that storage.
        Scratch.AddDefaulted();Scratch[0].RawAddress=123;
        {TArray<uint8> Source;FFramePointerCaptureScope Refused(Source,Registry,16384,&Scratch);TestTrue(TEXT("nonempty scratch fails closed without mutation"),Refused.HasOverflow()&&Scratch.Num()==1&&Scratch[0].RawAddress==123);}
        Scratch.Reset();
        for(int32 Mode=0;Mode<2;++Mode)
        {
            TArray<uint8> Source,Can{99};
            {FFramePointerCaptureScope Scope(Source,Registry,1,Mode?&Scratch:nullptr);FEpisodeStateBytesForTesting::Add(Source,&Object);FEpisodeStateBytesForTesting::Add(Source,&Object);TestTrue(TEXT("borrowed/default event limit is unchanged"),Scope.HasOverflow());TestFalse(TEXT("overflow keeps canonical unavailable"),Scope.MakeCanonical(Can));}
            TestTrue(TEXT("overflow preserves failed canonical destination and all raw pointer bytes"),Can==TArray<uint8>{99}&&Source.Num()==2*sizeof(UPTRINT));
        }
        // Unwinding an inner row lease restores TLS and leaves storage reusable.
        TArray<uint8> AfterUnwind,Aborted;
        {FFramePointerCaptureScope Outer(AfterUnwind,Registry);
         try{FFramePointerCaptureScope Inner(Aborted,Registry,16384,&Scratch);FEpisodeStateBytesForTesting::Add(Aborted,&Object);throw int32(17);}
         catch(int32 Marker){TestEqual(TEXT("synthetic row unwind is observed"),Marker,17);}
         FEpisodeStateBytesForTesting::Add(AfterUnwind,&Object);TestEqual(TEXT("unwind restores outer TLS route"),Outer.Observations().Num(),1);}
        TestTrue(TEXT("unwind returns scratch empty without erasing aborted raw evidence"),Scratch.IsEmpty()&&Aborted.Num()==sizeof(UPTRINT));
    }
    return true;
}
#endif
