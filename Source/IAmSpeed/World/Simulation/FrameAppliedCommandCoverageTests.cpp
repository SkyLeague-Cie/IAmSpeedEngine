#include "FrameAppliedCommandWitness.h"
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "Misc/AutomationTest.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFrameDeclaredAnalogCoverageTest,
 "IAmSpeed.Simulation.FrameWitness.DeclaredAnalogCoverage",
 EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FFrameDeclaredAnalogCoverageTest::RunTest(const FString&)
{
 using namespace Speed;TArray<uint8> Payload;Payload.Add(1);Payload.AddZeroed(3);
 {
  FFrameAppliedCommandWitnessScope Scope(19,0);
  TestFalse(TEXT("forgotten declaration cannot certify empty consumption"),Scope.Complete());
  FFrameAppliedCommandWitnessScope::ExpectAnalogActor(1);
  TestTrue(TEXT("independent expected actor set sealed before consumption"),FFrameAppliedCommandWitnessScope::SealExpectedAnalogActors());
  TestFalse(TEXT("omitted actual callback remains incomplete"),Scope.Complete());
  FFrameAppliedCommandWitnessScope::RecordAnalog(1,0,Payload);
  TestTrue(TEXT("exact declared consumer observed"),Scope.Complete());
 }
 {
  FFrameAppliedCommandWitnessScope Scope(20,0);
  TestTrue(TEXT("explicit no-analog adapter set may be empty"),FFrameAppliedCommandWitnessScope::SealExpectedAnalogActors());
  TestTrue(TEXT("certified empty set completes"),Scope.Complete());
  FFrameAppliedCommandWitnessScope::RecordAnalog(1,0,Payload);
  TestFalse(TEXT("unexpected consumer remains UNKNOWN"),Scope.Complete());
 }
 {
  FFrameAppliedCommandWitnessScope Scope(21,0);
  FFrameAppliedCommandWitnessScope::ExpectAnalogActor(1);
  FFrameAppliedCommandWitnessScope::RecordAnalog(1,0,Payload);
  FFrameAppliedCommandWitnessScope::SealExpectedAnalogActors();
  TestFalse(TEXT("consumer cannot retroactively declare its own coverage"),Scope.Complete());
 }
 {
  FFrameAppliedCommandWitnessScope Scope(22,0);
  FFrameAppliedCommandWitnessScope::MarkCoverageUnknown();
  FFrameAppliedCommandWitnessScope::SealExpectedAnalogActors();
  TestFalse(TEXT("unsupported adapter cannot be certified as no consumer"),Scope.Complete());
 }
 return !HasAnyErrors();
}
#endif
