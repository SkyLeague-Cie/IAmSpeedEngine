#include "FramePointerWitness.h"
#include "FrameInputWitness.h"
#include "IAmSpeed/Base/EpisodeStateBytesForTesting.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "UObject/Package.h"
#include "IAmSpeed/SubBodies/Solid/BoxSubBody.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFrameWitnessMembershipTest,
 "IAmSpeed.Simulation.FrameWitness.StaticAliasMembership",
 EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FFrameWitnessMembershipTest::RunTest(const FString&)
{
 using namespace Speed;int A=1,B=1;
 FFramePointerKey KA{4,7,0,1,0},KB=KA;KA.Membership={7,9};KB.Membership={7,11};
 TestFalse(TEXT("same minimum/count cannot alias different source sets"),KA==KB);
 FFramePointerRegistry RA,RB;TestTrue(TEXT("left membership"),RA.Register(&A,KA));TestTrue(TEXT("right membership"),RB.Register(&B,KB));
 TestTrue(TEXT("left seal"),RA.Seal());TestTrue(TEXT("right seal"),RB.Seal());
 TArray<uint8> RawA,RawB,CanA,CanB;
 {FFramePointerCaptureScope Scope(RawA,RA);FEpisodeStateBytesForTesting::Add(RawA,&A);TestTrue(TEXT("left capture"),Scope.MakeCanonical(CanA));}
 {FFramePointerCaptureScope Scope(RawB,RB);FEpisodeStateBytesForTesting::Add(RawB,&B);TestTrue(TEXT("right capture"),Scope.MakeCanonical(CanB));}
 TestTrue(TEXT("complete membership participates in canonical bytes"),CanA!=CanB);
 FFramePointerRegistry Conflict;TestTrue(TEXT("first membership"),Conflict.Register(&A,KA));
 TestFalse(TEXT("one address cannot change its semantic source set"),Conflict.Register(&A,KB));
 TestFalse(TEXT("conflict remains UNKNOWN"),Conflict.Seal());return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFrameWitnessCrossActorAliasTest,
 "IAmSpeed.Simulation.FrameWitness.CrossActorAliasGraph",
 EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FFrameWitnessCrossActorAliasTest::RunTest(const FString&)
{
 using namespace Speed;int A=1,B=1,C=1;const FFramePointerKey Root{100,1,0,10,0},Own{100,2,0,10,0};
 FFramePointerRegistry Left,Right;TestTrue(TEXT("left actor-one private node"),Left.Register(&A,Root));
 TestTrue(TEXT("independently allocated right actor-one private node"),Right.Register(&B,Root));
 Left.Seal();Right.Seal();auto ExtendedLeft=Left.CloneForAdditionalDeclarations();auto ExtendedRight=Right.CloneForAdditionalDeclarations();
 TestTrue(TEXT("actor-two can reference actor-one shared node"),ExtendedLeft.Find(reinterpret_cast<UPTRINT>(&A))!=nullptr);
 ExtendedLeft.Seal();ExtendedRight.Seal();TArray<uint8> L,R,CL,CR;
 {FFramePointerCaptureScope Scope(L,ExtendedLeft);FEpisodeStateBytesForTesting::Add(L,&A);TestTrue(TEXT("shared left capture"),Scope.MakeCanonical(CL));}
 {FFramePointerCaptureScope Scope(R,ExtendedRight);FEpisodeStateBytesForTesting::Add(R,&B);TestTrue(TEXT("shared right capture"),Scope.MakeCanonical(CR));}
 TestTrue(TEXT("shared relationship survives address relocation"),CL==CR);
 auto Changed=Right.CloneForAdditionalDeclarations();TestTrue(TEXT("actor-two now has a distinct equal-valued node"),Changed.Register(&C,Own));Changed.Seal();
 TArray<uint8> Raw,Canonical;{FFramePointerCaptureScope Scope(Raw,Changed);FEpisodeStateBytesForTesting::Add(Raw,&C);TestTrue(TEXT("distinct node capture"),Scope.MakeCanonical(Canonical));}
 TestTrue(TEXT("equal contents cannot hide changed cross-actor aliasing"),Canonical!=CL);return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFrameWitnessAbsentInputTest,
 "IAmSpeed.Simulation.FrameWitness.AbsentRegistryEndpoint",
 EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FFrameWitnessAbsentInputTest::RunTest(const FString&)
{
 using namespace Speed;TArray<uint8> Installed,Completed,OtherFrame,Present;
 AppendAbsentFrameInputWitness(19,EFrameInputWitnessStage::Installed,Installed);
 AppendAbsentFrameInputWitness(19,EFrameInputWitnessStage::RegistryCompleted,Completed);
 AppendAbsentFrameInputWitness(20,EFrameInputWitnessStage::Installed,OtherFrame);
 TestTrue(TEXT("absence still proves endpoint stage"),Installed!=Completed);
 TestTrue(TEXT("absence still proves exact frame"),Installed!=OtherFrame);
 Input::V2::FRegistryFrame Empty;Empty.Frame=19;Empty.Registry=std::make_shared<Input::V2::FInputRegistryView>();
 TestTrue(TEXT("present empty real registry captured"),AppendFrameInputWitness(Empty,EFrameInputWitnessStage::Installed,Present));
 TestTrue(TEXT("absent and present-empty cannot be conflated"),Installed!=Present);
 TArray<uint8> Endpoint;
 TestFalse(TEXT("null alone cannot certify absence"),AppendFrameInputEndpointWitness(19,EFrameInputWitnessStage::Installed,nullptr,false,Endpoint));
 TestTrue(TEXT("driver-certified no-session endpoint"),AppendFrameInputEndpointWitness(19,EFrameInputWitnessStage::Installed,nullptr,true,Endpoint));
 TestFalse(TEXT("stale present endpoint cannot certify another frame"),AppendFrameInputEndpointWitness(20,EFrameInputWitnessStage::Installed,&Empty,false,Endpoint));
 TestFalse(TEXT("present endpoint contradicts absent certificate"),AppendFrameInputEndpointWitness(19,EFrameInputWitnessStage::Installed,&Empty,true,Endpoint));
 TestTrue(TEXT("exact present frame accepted"),AppendFrameInputEndpointWitness(19,EFrameInputWitnessStage::Installed,&Empty,false,Endpoint));
 Empty.Registry.reset();TestFalse(TEXT("malformed present endpoint is still UNKNOWN"),AppendFrameInputWitness(Empty,EFrameInputWitnessStage::Installed,Present));return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFrameWitnessObjectIdTest,
 "IAmSpeed.Simulation.FrameWitness.ProcessLocalObjectId",
 EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FFrameWitnessObjectIdTest::RunTest(const FString&)
{
 using namespace Speed;UObject* A=NewObject<UBoxSubBody>(GetTransientPackage());UObject* B=NewObject<UBoxSubBody>(GetTransientPackage());
 const FFramePointerKey Key{2,17,0,1,1};FFramePointerRegistry RA,RB;RA.Register(A,Key);RB.Register(B,Key);RA.Seal();RB.Seal();
 TArray<uint8> RawA,RawB,CA,CB;
 {FFramePointerCaptureScope Scope(RawA,RA);FFramePointerCaptureScope::ObserveObjectId(RawA,A->GetUniqueID(),A);FEpisodeStateBytesForTesting::Add(RawA,A->GetUniqueID());TestTrue(TEXT("typed left id"),Scope.MakeCanonical(CA));}
 {FFramePointerCaptureScope Scope(RawB,RB);FFramePointerCaptureScope::ObserveObjectId(RawB,B->GetUniqueID(),B);FEpisodeStateBytesForTesting::Add(RawB,B->GetUniqueID());TestTrue(TEXT("typed right id"),Scope.MakeCanonical(CB));}
 TestTrue(TEXT("raw process IDs retained"),RawA!=RawB && RawA.Num()==sizeof(uint32));TestTrue(TEXT("same semantic object survives native ID relocation"),CA==CB);
 TArray<uint8> Wrong,Ignored;
 {FFramePointerCaptureScope Scope(Wrong,RA);FFramePointerCaptureScope::ObserveObjectId(Wrong,B->GetUniqueID(),A);FEpisodeStateBytesForTesting::Add(Wrong,B->GetUniqueID());TestFalse(TEXT("ID/object mismatch UNKNOWN"),Scope.MakeCanonical(Ignored));}
 return true;
}
#endif
