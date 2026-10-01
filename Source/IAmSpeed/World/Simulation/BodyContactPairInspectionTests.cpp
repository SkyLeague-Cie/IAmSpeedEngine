#include "SimulationWorld.h"

#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "IAmSpeed/Components/SpeedMovementComponent.h"
#include "IAmSpeed/SubBodies/Solid/SphereSubBody.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "Engine/World.h"
#include "Misc/AutomationTest.h"
#include "UObject/StrongObjectPtr.h"

namespace Speed
{
// Registry-only owner fixture: no collision step or foreign actor model.
struct FBodyContactPairReadOnlyTestFixture
{
 TStrongObjectPtr<USpeedMovementComponent> Owners[3];
 TStrongObjectPtr<USphereSubBody> Bodies[3];
 FSimulationWorld World;
 FBodyContactPairReadOnlyTestFixture()
 {
  for(int32 I=0;I<3;++I)
  {
   Owners[I]=TStrongObjectPtr<USpeedMovementComponent>(NewObject<USpeedMovementComponent>());
   Bodies[I]=TStrongObjectPtr<USphereSubBody>(NewObject<USphereSubBody>());
   Bodies[I]->Initialize(Owners[I].Get());
  }
  Bind(World);
 }
 void Bind(FSimulationWorld& W)
 {
  for(int32 I=0;I<3;++I)W.AddAdapter(*Owners[I]);
  W.RebuildOrderedAdapters();
  for(int32 I=0;I<3;++I)
  {
   const uint64 Id=W.FindStableId(*Owners[I])<<16;
   W.StableSubBodyIds.Add(Bodies[I].Get(),Id);
   W.SolidSubBodiesByStableId.Add(Id,Bodies[I].Get());
  }
  W.CurrentStepFrame=7;
 }
 static uint64 Key(const USolidSubBody& A,const USolidSubBody& B)
 {return (uint64(FMath::Min(A.GetUniqueID(),B.GetUniqueID()))<<32)|FMath::Max(A.GetUniqueID(),B.GetUniqueID());}
 void AliasQueryIdentity()
 {World.StableSubBodyIds.Add(Bodies[0].Get(),World.FindStableSubBodyId(Bodies[1].Get()));}
 void BindPrepareScope(USpeedWorldSubsystem& S)
 {Bind(S.SimulationWorld);S.bCanonicalFrameActive=true;S.StepSerial=1;S.TestingPreparingFrame=8;}
 static void EndPrepareScope(USpeedWorldSubsystem& S){S.TestingPreparingFrame=MAX_uint64;}
};
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedBodyContactPairReadOnlyInspectionTest,
 "IAmSpeed.Simulation.BodyContactPairReadOnlyInspection",
 EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedBodyContactPairReadOnlyInspectionTest::RunTest(const FString& Parameters)
{
 using Speed::FBodyContactPairReadOnlyTestFixture;
 using Speed::FBodyContactPairInspection;
 {
  FBodyContactPairReadOnlyTestFixture F;FBodyContactPairInspection O;
  const auto Before=F.World.CaptureSnapshot(7,123);
  TestTrue(TEXT("no pair gives a complete inventory"),F.World.InspectBodyContactPairsForTesting(*F.Bodies[0],8,O));
  TestTrue(TEXT("complete zero incidence permits only this precondition"),O.HasNoIncidentPairs());
  TestEqual(TEXT("query keeps the full physical snapshot hash"),F.World.CaptureSnapshot(7,123).StateHash,Before.StateHash);
  TestEqual(TEXT("query returns stable body identity"),O.QueryStableBodyId,uint64(1)<<16);
  TestEqual(TEXT("preparing and completed epochs are distinct"),O.CompletedStepFrame,uint32(7));
 }
 {
  FBodyContactPairReadOnlyTestFixture F;FBodyContactPairInspection O;
  F.World.AddDynamicContactPair(F.Key(*F.Bodies[0],*F.Bodies[1]),*F.Bodies[0],*F.Bodies[1]);
  const auto Before=F.World.CaptureSnapshot(7,123);
  TestTrue(TEXT("dynamic-only inventory is complete"),F.World.InspectBodyContactPairsForTesting(*F.Bodies[0],8,O));
  TestFalse(TEXT("dynamic incident blocks rebase"),O.HasNoIncidentPairs());
  TestEqual(TEXT("dynamic incidence"),O.DynamicIncident,1);TestEqual(TEXT("pending incidence remains zero"),O.PendingIncident,0);
  TestEqual(TEXT("dynamic query changes no physical state"),F.World.CaptureSnapshot(7,123).StateHash,Before.StateHash);
 }
 {
  FBodyContactPairReadOnlyTestFixture F;FBodyContactPairInspection O;
  F.World.AddPendingRollingContactPair(F.Key(*F.Bodies[0],*F.Bodies[1]),*F.Bodies[0],*F.Bodies[1]);
  const auto Before=F.World.CaptureSnapshot(7,123);
  TestTrue(TEXT("pending-only inventory is complete"),F.World.InspectBodyContactPairsForTesting(*F.Bodies[0],8,O));
  TestFalse(TEXT("pending incident blocks rebase even with empty dynamic adjacency"),O.HasNoIncidentPairs());
  TestEqual(TEXT("pending incidence"),O.PendingIncident,1);TestEqual(TEXT("dynamic incidence remains zero"),O.DynamicIncident,0);
  TestEqual(TEXT("pending query changes no physical state"),F.World.CaptureSnapshot(7,123).StateHash,Before.StateHash);
 }
 {
  FBodyContactPairReadOnlyTestFixture F;FBodyContactPairInspection O;
  F.World.AddDynamicContactPair(F.Key(*F.Bodies[0],*F.Bodies[1]),*F.Bodies[0],*F.Bodies[1]);
  F.World.AddPendingRollingContactPair(F.Key(*F.Bodies[0],*F.Bodies[2]),*F.Bodies[0],*F.Bodies[2]);
  TestTrue(TEXT("both collections inspected"),F.World.InspectBodyContactPairsForTesting(*F.Bodies[0],8,O));
  TestEqual(TEXT("all dynamic rows scanned"),O.DynamicScanned,O.DynamicTotal);
  TestEqual(TEXT("all pending rows scanned"),O.PendingScanned,O.PendingTotal);
  TestEqual(TEXT("separate values-only rows retained"),O.Rows.Num(),2);
  TestFalse(TEXT("two incident collections block"),O.HasNoIncidentPairs());
 }
 {
  FBodyContactPairReadOnlyTestFixture F;FBodyContactPairInspection O;
  F.World.AddPendingRollingContactPair(F.Key(*F.Bodies[1],*F.Bodies[2]),*F.Bodies[1],*F.Bodies[2]);
  TestTrue(TEXT("valid unrelated pending pair is still inspected"),F.World.InspectBodyContactPairsForTesting(*F.Bodies[0],8,O));
  TestTrue(TEXT("valid unrelated pair does not block this body's precondition"),O.HasNoIncidentPairs());
  TestEqual(TEXT("unrelated collection row is not omitted"),O.PendingScanned,1);
 }
 {
  FBodyContactPairReadOnlyTestFixture F;FBodyContactPairInspection O;F.AliasQueryIdentity();
  TestFalse(TEXT("stable identity alias rejects inventory"),F.World.InspectBodyContactPairsForTesting(*F.Bodies[0],8,O));
  TestFalse(TEXT("alias never means absence"),O.HasNoIncidentPairs());
 }
 {
  FBodyContactPairReadOnlyTestFixture F;FBodyContactPairInspection O;
  F.World.AddDynamicContactPair(F.Key(*F.Bodies[0],*F.Bodies[0]),*F.Bodies[0],*F.Bodies[0]);
  TestFalse(TEXT("self-pair rejects"),F.World.InspectBodyContactPairsForTesting(*F.Bodies[0],8,O));
  TestFalse(TEXT("self-pair not clear"),O.HasNoIncidentPairs());
 }
 {
  FBodyContactPairReadOnlyTestFixture F;FBodyContactPairInspection O;
  auto& P=F.World.AddPendingRollingContactPair(F.Key(*F.Bodies[1],*F.Bodies[2]),*F.Bodies[1],*F.Bodies[2]);P.BodyB.Reset();
  TestFalse(TEXT("unrelated expired endpoint cannot silently be skipped"),F.World.InspectBodyContactPairsForTesting(*F.Bodies[0],8,O));
  TestFalse(TEXT("incomplete inventory is not absence"),O.HasNoIncidentPairs());
 }
 {
  FBodyContactPairReadOnlyTestFixture F;FBodyContactPairInspection O;
  TestFalse(TEXT("wrong preparing epoch rejects before scan"),F.World.InspectBodyContactPairsForTesting(*F.Bodies[0],9,O));
  TestFalse(TEXT("wrong epoch cannot arm"),O.HasNoIncidentPairs());
  TStrongObjectPtr<UWorld> Host(NewObject<UWorld>());
  TStrongObjectPtr<USpeedWorldSubsystem> S(NewObject<USpeedWorldSubsystem>(Host.Get()));
  TestFalse(TEXT("outside prepare scope rejects"),S->InspectBodyContactPairsForTesting(*F.Bodies[0],8,O));
  F.BindPrepareScope(*S);
  TestTrue(TEXT("owned synthetic prepare scope reaches complete inventory"),S->InspectBodyContactPairsForTesting(*F.Bodies[0],8,O));
  TestTrue(TEXT("zero incidence is clear in bound scope"),O.HasNoIncidentPairs());
  F.EndPrepareScope(*S);
  TestFalse(TEXT("after prepare scope no stale certificate"),S->InspectBodyContactPairsForTesting(*F.Bodies[0],8,O));
 }
 return true;
}
#endif
