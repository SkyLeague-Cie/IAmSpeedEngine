#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "IAmSpeed/Tests/PrivateRoadBirthFixtureObjects.h"
#include "Engine/World.h"
#include "Components/SceneComponent.h"
#include "GameFramework/Actor.h"
#include "Misc/AutomationTest.h"
#include "Misc/ScopeExit.h"
#include "UObject/StrongObjectPtr.h"
#include "ChaosVehicleWheel.h"
#include "WheelSystem.h"
#include "SuspensionSystem.h"
#include "Math/QuatRotationTranslationMatrix.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIASLegacyFirstOwnerBirthTraversal,
    "IAmSpeed.Netcode.Prediction.LegacyFirstOwnerBirthTraversal",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FIASLegacyFirstOwnerBirthTraversal::RunTest(const FString&)
{
    const auto Options=UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(false)
        .CreateNavigation(false).CreateAISystem(false).ShouldSimulatePhysics(false);
    UWorld* World=UWorld::CreateWorld(EWorldType::Game,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Options);
    if(!TestNotNull(TEXT("actual legacy fixture world"),World))return false;
    ON_SCOPE_EXIT{World->DestroyWorld(false);};
    auto* Actor=World->SpawnActor<AActor>();if(!Actor)return false;
    TStrongObjectPtr<USceneComponent> ActualRoot(NewObject<USceneComponent>(Actor));
    if(!ActualRoot.Get())return false;Actor->SetRootComponent(ActualRoot.Get());
    TStrongObjectPtr<UIASBirthFixtureComponent> Parent(NewObject<UIASBirthFixtureComponent>(Actor));
    TStrongObjectPtr<UIASBirthFixtureBox> Box(NewObject<UIASBirthFixtureBox>(Actor));
    TStrongObjectPtr<UIASBirthFixtureWheel> A(NewObject<UIASBirthFixtureWheel>(Actor)),C(NewObject<UIASBirthFixtureWheel>(Actor));
    TStrongObjectPtr<UChaosVehicleWheel> CA(NewObject<UChaosVehicleWheel>(Actor)),CC(NewObject<UChaosVehicleWheel>(Actor));
    if(!Parent.Get()||!Box.Get()||!A.Get()||!C.Get()||!CA.Get()||!CC.Get())return false;
    Chaos::FSimpleWheelConfig WheelSetup;Chaos::FSimpleSuspensionConfig SuspensionSetup;
    Chaos::FSimpleWheelSim SA(&WheelSetup),SC(&WheelSetup);
    Chaos::FSimpleSuspensionSim UA(&SuspensionSetup),UC(&SuspensionSetup);
    const FRotator RelativeRotation(17,31,-23);const FQuat ActorRotation=FRotator(-29,43,11).Quaternion();
    const FVector ActorLocation(123,-47,89);
    Parent->BodyConfig.bValid=true;Parent->BodyConfig.Mass=111;
    Parent->BodyConfig.BoxExtent=FVector(13,27,39);Parent->BodyConfig.Radius=14;
    Parent->BodyConfig.LocalOffset=FVector(5,3,2);Parent->BodyConfig.LocalRotation=RelativeRotation.Quaternion();
    Parent->WheelConfig.bValid=false;
    Parent->SetSubBodies({Box.Get(),A.Get(),C.Get()});
    ON_SCOPE_EXIT{Parent->Trace.reset();Parent->SetSubBodies({});Box->Trace.reset();A->Trace.reset();C->Trace.reset();A->SetWheelSim(nullptr);C->SetWheelSim(nullptr);A->SetSuspensionSim(nullptr);C->SetSuspensionSim(nullptr);};
    for(int32 Override=0;Override<2;++Override)
    {
        auto Trace=std::make_shared<FIASBirthFixtureTrace>();Parent->Trace=Trace;Box->Trace=Trace;A->Trace=Trace;C->Trace=Trace;
        Parent->CenterOfMassWheeledDomainStorage()=FVector(13,17,-2);
        Parent->CenterOfMassOverrideWheeledDomainStorage()=Override?FVector(7,-11,5):FVector::ZeroVector;
        Parent->bEnableCenterOfMassOverrideWheeledDomainStorage()=false;
        A->SetChaosWheel(CA.Get());C->SetChaosWheel(CC.Get());A->SetWheelSim(&SA);C->SetWheelSim(&SC);A->SetSuspensionSim(&UA);C->SetSuspensionSim(&UC);
        TestTrue(TEXT("actual legacy actor location API returned"),Actor->SetActorLocation(ActorLocation));
        TestTrue(TEXT("actual legacy actor rotation API returned"),Actor->SetActorRotation(ActorRotation));
        uint32 RealReallocations=0;bool DifferentAllocationSameRoster=false;
        Trace->AfterChildUpdate=[&]() {
            if(RealReallocations==0)
            {
                DifferentAllocationSameRoster=Parent->ReallocateSameBodyRosterForFixture();
                ++RealReallocations;
            }
        };
        Parent->SetOwner(Actor); // REAL ordinary SetOwner/shared program, no native operation
        Trace->AfterChildUpdate=nullptr; // callback never escapes these actual owner calls
        TestEqual(TEXT("one real custom child callback reallocated the parent list"),RealReallocations,uint32(1));
        TestTrue(TEXT("actual old/new allocations differ with identical live roster"),DifferentAllocationSameRoster);
        TestEqual(TEXT("both real custom child updates returned after the reallocation"),Trace->ChildUpdateReturns,uint32(2));
        TestTrue(TEXT("custom child update order unchanged after parent reallocation"),Trace->ChildUpdateOrder.Num()==2
            &&Trace->ChildUpdateOrder[0]==A.Get()&&Trace->ChildUpdateOrder[1]==C.Get());
        AddInfo(FString::Printf(TEXT("[LegacyFirstOwnerChildArrayReallocation] case=%d reallocationCalls=%u differentAllocationSameRoster=%d childReturns=%u. Real custom virtual callback tests shared traversal; no private exact-class callback or lifetime lease is fabricated."),
            Override,RealReallocations,int(DifferentAllocationSameRoster),Trace->ChildUpdateReturns));
        Parent->SetAsyncPhysicsTickEnabled(false);
        const FVector ExpectedCOM=Override?FVector(7,-11,5):FVector(13,17,-2);
        TestTrue(TEXT("override precedence / zero override fallback unchanged"),Parent->CenterOfMassWheeledDomainStorage()==ExpectedCOM
            &&Parent->CenterOfMassOverrideWheeledDomainStorage()==ExpectedCOM&&Parent->bEnableCenterOfMassOverrideWheeledDomainStorage());
        TestTrue(TEXT("actual COM pose remains origin plus rotated local COM"),Parent->GetPhysCOM()==ActorLocation+ActorRotation.RotateVector(ExpectedCOM));
        TestTrue(TEXT("actual ordered partial roster retained"),Parent->ReadHitboxForPhysics().Get()==Box.Get()
            &&Parent->GetWheelSubBodies().Num()==2&&Parent->GetWheelSubBodies()[0].Get()==A.Get()&&Parent->GetWheelSubBodies()[1].Get()==C.Get());
        TestEqual(TEXT("three real subbody queries per custom body"),Trace->SubBodyQueries,uint32(9));
        TestEqual(TEXT("custom Wheel queries retained"),Trace->WheelQueries,uint32(2));
        TestEqual(TEXT("actual custom inverse dispatch only during initializers"),Trace->Inverse.Num(),6);
        if(Trace->Inverse.Num()!=6)return false;
        const FMatrix Stored=Box->ReadInverseForFixture();const FMatrix R=FQuatRotationMatrix(Box->GetLocalRotation());
        TestTrue(TEXT("chassis tensor rotates the final STORED custom tensor without recomputing"),
            Stored==Trace->Inverse[1].Returned&&Parent->CarLocalInvIWheeledDomainStorage()==R.GetTransposed()*Stored*R
            &&Parent->CarLocalInvIWheeledDomainStorage()!=Stored);
        TestTrue(TEXT("initial null-vehicle clear keeps Chaos config, clears live wheel/suspension"),A->GetChaosWheelConfiguration()==CA.Get()
            &&C->GetChaosWheelConfiguration()==CC.Get()&&!A->GetWheelSim()&&!C->GetWheelSim()&&!A->GetSuspensionSim()&&!C->GetSuspensionSim());
        TestTrue(TEXT("all actual bodies initialized against this parent"),Box->GetParentComponent()==Parent.Get()&&A->GetParentComponent()==Parent.Get()&&C->GetParentComponent()==Parent.Get());
    }
    // Independent real legacy virtual calls, with no live suspension. Check
    // the original second velocity write at nontrivial pose/spin exactly.
    SKinematic ParentState;ParentState.Location=ActorLocation;ParentState.Rotation=ActorRotation;
    ParentState.Velocity=FVector(3,5,7);ParentState.AngularVelocity=FVector(0.1,0.2,0.3);
    ParentState.Acceleration=FVector(11,13,17);ParentState.AngularAcceleration=FVector(0.4,0.5,0.6);
    uint32 ActualLegacyUpdates=0;bool LegacyOrderDistinguished=false;
    for(int32 Index=0;Index<4;++Index)
    {
        const FVector AuthoredOffset(21+Index*7,-31+Index*11,-9+Index);
        A->SetLocalOffset(AuthoredOffset);A->UpdateKinematicsFromOwner(ParentState);++ActualLegacyUpdates;
        const auto Actual=A->CopyKinematicsForFixture();
        const FVector Offset=ParentState.Rotation.RotateVector(AuthoredOffset);
        const FVector ExpectedLocation=ParentState.Location+Offset;
        const FVector ExpectedWheelVelocity=ParentState.Velocity+FVector::CrossProduct(
            ParentState.AngularVelocity,ExpectedLocation-ParentState.Location);
        const FVector BaseOnlyVelocity=ParentState.Velocity+FVector::CrossProduct(ParentState.AngularVelocity,Offset);
        TestTrue(TEXT("real legacy wheel exact authored world location"),Actual.Location==ExpectedLocation);
        TestTrue(TEXT("real legacy wheel exact original second-write velocity"),Actual.Velocity==ExpectedWheelVelocity);
        LegacyOrderDistinguished=LegacyOrderDistinguished||(ExpectedWheelVelocity!=BaseOnlyVelocity&&Actual.Velocity==ExpectedWheelVelocity);
    }
    TestEqual(TEXT("four real legacy wheel update calls returned"),ActualLegacyUpdates,uint32(4));
    TestTrue(TEXT("real legacy wheel distinguishes base-only velocity shortcut"),LegacyOrderDistinguished);
    AddInfo(FString::Printf(TEXT("[LegacyFirstOwnerChildArithmetic] actualUpdates=%u orderDistinguished=%d"),ActualLegacyUpdates,int(LegacyOrderDistinguished)));
    TestFalse(TEXT("authored fixture remains unregistered"),Parent->IsRegistered());
    AddInfo(TEXT("[LegacyFirstOwnerBirthTraversal] cases=2 customBodiesEach=3 subbodyQueriesEachCase=9 wheelQueriesEachCase=2 inverseCallsEachCase=6 nonzeroCOMPose=1 rotatedStoredTensor=1 clearKeepsConfig=1 orderedPartialRoster=1. Real ordinary SetOwner/default/custom dispatch fixtures with actor pose APIs and shared tick/register order; no native lease or network claim."));
    return !HasAnyErrors();
}
#endif
