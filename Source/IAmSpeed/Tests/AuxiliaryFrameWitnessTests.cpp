#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "IAmSpeed/SubBodies/Sensor/SphereSensor.h"
#include "IAmSpeed/World/Simulation/FramePointerWitness.h"
#include "IAmSpeed/World/Simulation/AuxiliaryFrameWitnessForTesting.h"
#include "IAmSpeed/Base/EpisodeStateBytesForTesting.h"
#include "Misc/AutomationTest.h"
#include "UObject/UObjectGlobals.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAuxiliarySensorStateMutationTest,
 "IAmSpeed.Physics.FrameWitness.AuxiliarySensorMutation",
 EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FAuxiliarySensorStateMutationTest::RunTest(const FString&)
{
 auto* Sensor=NewObject<USphereSensor>();
 Speed::FFramePointerRegistry Registry;
 if(!TestTrue(TEXT("empty known semantic registry seals"),Registry.Seal()))return false;
 const float Original=Sensor->GetRadius();TArray<uint8> A,B,C;
 TestTrue(TEXT("known empty sensor overlap state is observable"),Sensor->AppendOverlapStateForFrameWitnessForTesting(A,Registry));
 TestEqual(TEXT("observer leaves sensor shape untouched"),Sensor->GetRadius(),Original);
 Sensor->SetRadius(Original+0.25f);
 TestTrue(TEXT("mutated real shape remains observable"),Sensor->AppendOverlapStateForFrameWitnessForTesting(B,Registry));
 TestTrue(TEXT("a hidden sensor shape change cannot pass byte equality"),A!=B);
 Sensor->SetRadius(Original);
 TestTrue(TEXT("restored shape remains observable"),Sensor->AppendOverlapStateForFrameWitnessForTesting(C,Registry));
 TestTrue(TEXT("observer is deterministic and does not retain prior capture state"),A==C);
 return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAuxiliarySharedAliasesTest,
 "IAmSpeed.Physics.FrameWitness.AuxiliarySharedAliases",
 EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FAuxiliarySharedAliasesTest::RunTest(const FString&)
{
 using namespace Speed;int Shared=1,Split=2;
 FFramePointerRegistry Global;TestTrue(TEXT("initial registry seals"),Global.Seal());
 auto Author=Global.CloneForAdditionalDeclarations();
 TestTrue(TEXT("author declares shared contract"),DeclareAuxiliaryFramePointer(Author,&Shared,45,51));
 TestTrue(TEXT("author extension seals"),Author.Seal());Global=MoveTemp(Author);
 auto Car=Global.CloneForAdditionalDeclarations();
 TestTrue(TEXT("later actor retains existing semantic alias"),DeclareAuxiliaryFramePointer(Car,&Shared,47,11));
 TestTrue(TEXT("shared extension seals"),Car.Seal());
 const auto* Key=Car.Find(reinterpret_cast<UPTRINT>(&Shared));
 TestTrue(TEXT("one shared contract retains author ownership"),Key && Key->Owner==45 && Key->Role==51);
 TArray<uint8> RawA,RawB,CanA,CanB;
 {FFramePointerCaptureScope Capture(RawA,Global);FEpisodeStateBytesForTesting::Add(RawA,&Shared);
  TestTrue(TEXT("author alias canonicalizes"),Capture.MakeCanonical(CanA));}
 {FFramePointerCaptureScope Capture(RawB,Car);FEpisodeStateBytesForTesting::Add(RawB,&Shared);
  TestTrue(TEXT("car shared alias canonicalizes"),Capture.MakeCanonical(CanB));}
 TestTrue(TEXT("cross-actor shared contract uses identical semantic bytes"),CanA==CanB);
 auto Distinct=Global.CloneForAdditionalDeclarations();
 TestTrue(TEXT("distinct car contract is separately declared"),DeclareAuxiliaryFramePointer(Distinct,&Split,47,11));
 TestTrue(TEXT("distinct contract registry seals"),Distinct.Seal());
 {TArray<uint8> Raw,Canonical;FFramePointerCaptureScope Capture(Raw,Distinct);
  FEpisodeStateBytesForTesting::Add(Raw,&Split);TestTrue(TEXT("split contract canonicalizes"),Capture.MakeCanonical(Canonical));
  TestTrue(TEXT("changing shared ownership into separate objects changes witness"),Canonical!=CanA);}
 return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAuxiliarySensorDelegateShapeTest,
 "IAmSpeed.Physics.FrameWitness.AuxiliarySensorDelegateShape",
 EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FAuxiliarySensorDelegateShapeTest::RunTest(const FString&)
{
 auto* Sensor=NewObject<USphereSensor>();Speed::FFramePointerRegistry Registry;Registry.Seal();
 TArray<uint8> Before,During,After;
 TestTrue(TEXT("initial sensor fields captured"),Sensor->AppendOverlapStateForFrameWitnessForTesting(Before,Registry));
 const auto Handle=Sensor->OnEndOverlap.AddLambda([](USensorSubBody*,UPrimitiveComponent*){});
 TestTrue(TEXT("delegate-presence fields remain observable"),Sensor->AppendOverlapStateForFrameWitnessForTesting(During,Registry));
 TestTrue(TEXT("unexpected delegate presence changes the witness"),Before!=During);
 Sensor->OnEndOverlap.Remove(Handle);
 TestTrue(TEXT("restored sensor fields captured"),Sensor->AppendOverlapStateForFrameWitnessForTesting(After,Registry));
 TestTrue(TEXT("observer does not alter delegates"),Before==After);
 // Presence booleans do not certify invocation multiplicity or target function.
 return !HasAnyErrors();
}
#endif
