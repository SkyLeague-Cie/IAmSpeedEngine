#if WITH_DEV_AUTOMATION_TESTS
#include "IAmSpeed/Netcode/transport/AuthorityInputJournal.h"
#include "Misc/AutomationTest.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIASAuthorityInputJournal,"IAmSpeed.Netcode.Transport.AllActorAuthorityInputJournal",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FIASAuthorityInputJournal::RunTest(const FString&)
{
    using namespace Speed::Netcode;
    FBinding A;A.Session=11;A.Epoch=3;A.Actor=101;A.Incarnation=7;A.OwnerPeer=4;A.ServerPeer=5;
    A.SourceProducer=42;A.SourceKind=0;A.Contract[0]=7;
    auto B=A;B.Actor=102;B.OwnerPeer=8;B.SourceProducer=43;
    const auto Packet=[](const FBinding& R,U64 Frame)
    {
        FPacket P;P.Kind=EKind::AuthorityInput;P.Session=R.Session;P.Epoch=R.Epoch;
        P.Actor=R.Actor;P.Incarnation=R.Incarnation;P.Contract=R.Contract;P.Frame=Frame;P.Sequence=Frame+1;
        P.Input.SourceSequence=Frame+1;P.Input.DeviceGeneration=1;P.Input.ProducerId=R.SourceProducer;
        P.Input.ProducerKind=R.SourceKind;P.Input.Reset=Frame==0;P.Input.Values[0]=128;P.Input.ActiveMask=1;
        return P;
    };
    TestFalse(TEXT("duplicate semantic actor cannot mint closed roster"),bool(FAuthorityInputJournal::Create({A,A},3)));
    TestFalse(TEXT("unbounded journal refused"),bool(FAuthorityInputJournal::Create({A,B},4097)));
    auto OtherEpoch=B;++OtherEpoch.Epoch;
    TestFalse(TEXT("one journal cannot combine epochs"),bool(FAuthorityInputJournal::Create({A,OtherEpoch},3)));
    auto J=FAuthorityInputJournal::Create({B,A},3);if(!TestTrue(TEXT("finite exact two-actor roster"),bool(J)))return false;
    auto A0=Packet(A,0),A1=Packet(A,1),A2=Packet(A,2),B0=Packet(B,0),B1=Packet(B,1),B2=Packet(B,2);
    TestTrue(TEXT("owner peer is not server authority"),J->Retain(A.OwnerPeer,A0)==EResult::WrongPeer);
    TestTrue(TEXT("owner retention ACK cannot prove server consumption"),J->Retain(5,AckFor(A0))==EResult::Invalid);
    TestTrue(TEXT("out-of-order original authority retained"),J->Retain(5,A1)==EResult::Accepted);
    TestTrue(TEXT("holes never count as a contiguous cut"),J->CommonContiguousCount()==0&&J->ContiguousCount(A.Actor)==0);
    TestTrue(TEXT("remote prefix alone cannot qualify local owner"),J->Retain(5,B0)==EResult::Accepted&&J->CommonContiguousCount()==0);
    TestTrue(TEXT("actual missing owner frame fills prefix"),J->Retain(5,A0)==EResult::Accepted&&J->CommonContiguousCount()==1&&J->ContiguousCount(A.Actor)==2);
    const auto Original=J->Read(A.Actor,0);
    TestTrue(TEXT("duplicate immutable record remains duplicate"),J->Retain(5,A0)==EResult::Duplicate&&J->Read(A.Actor,0)==Original);
    auto Changed=A0;Changed.Input.Values[0]=129;
    TestTrue(TEXT("conflict cannot overwrite retained original"),J->Retain(5,Changed)==EResult::Conflict&&J->Read(A.Actor,0)==Original);
    Changed=A0;++Changed.Incarnation;
    TestTrue(TEXT("new incarnation cannot fill old journal"),J->Retain(5,Changed)==EResult::WrongStream);
    Changed=A0;++Changed.Input.ProducerId;
    TestTrue(TEXT("numeric actor equality cannot substitute producer"),J->Retain(5,Changed)==EResult::WrongStream);
    TestTrue(TEXT("remote hole remains visible despite later frames"),J->Retain(5,B2)==EResult::Accepted&&J->CommonContiguousCount()==1);
    TestTrue(TEXT("own late suffix does not advance remote cut"),J->Retain(5,A2)==EResult::Accepted&&J->CommonContiguousCount()==1);
    TestTrue(TEXT("both genuinely contiguous journals reach finite cut"),J->Retain(5,B1)==EResult::Accepted&&J->CommonContiguousCount()==3);
    TestTrue(TEXT("finite history refuses overflow without eviction"),J->Retain(5,Packet(A,3))==EResult::TooFar&&J->Read(A.Actor,0)==Original);
    AddInfo(TEXT("Values journal and negative source/roster/hole/conflict checks only. Actual server-consumption provenance requires the separate native process probe; no checkpoint, hidden-state, lifetime, localized rollback or game admission."));
    return !HasAnyErrors();
}
#endif
