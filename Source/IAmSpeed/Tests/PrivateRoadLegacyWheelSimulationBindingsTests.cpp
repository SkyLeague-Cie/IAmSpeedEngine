#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "IAmSpeed/Tests/PrivateRoadBirthFixtureObjects.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Misc/AutomationTest.h"
#include "Misc/ScopeExit.h"
#include "UObject/StrongObjectPtr.h"
#include "ChaosVehicleWheel.h"
#include "SimpleVehicle.h"
#include "WheelSystem.h"
#include "SuspensionSystem.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIASLegacyWheelSimulationBindings,
    "IAmSpeed.Netcode.Prediction.LegacyWheelSimulationBindings",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FIASLegacyWheelSimulationBindings::RunTest(const FString&)
{
    const auto Options=UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(false)
        .CreateNavigation(false).CreateAISystem(false).ShouldSimulatePhysics(false);
    UWorld* World=UWorld::CreateWorld(EWorldType::Game,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Options);
    if(!TestNotNull(TEXT("actual legacy binding fixture world"),World))return false;
    ON_SCOPE_EXIT{World->DestroyWorld(false);};
    auto* Actor=World->SpawnActor<AActor>();if(!Actor)return false;
    TStrongObjectPtr<UIASBirthFixtureComponent> Parent(NewObject<UIASBirthFixtureComponent>(Actor));
    TStrongObjectPtr<UIASBirthFixtureWheel> A(NewObject<UIASBirthFixtureWheel>(Actor));
    TStrongObjectPtr<UIASBirthFixtureWheel> B(NewObject<UIASBirthFixtureWheel>(Actor));
    TStrongObjectPtr<UIASBirthFixtureWheel> C(NewObject<UIASBirthFixtureWheel>(Actor));
    TStrongObjectPtr<UChaosVehicleWheel> CA(NewObject<UChaosVehicleWheel>(Actor));
    TStrongObjectPtr<UChaosVehicleWheel> CB(NewObject<UChaosVehicleWheel>(Actor));
    TStrongObjectPtr<UChaosVehicleWheel> CC(NewObject<UChaosVehicleWheel>(Actor));
    if(!Parent.Get()||!A.Get()||!B.Get()||!C.Get()||!CA.Get()||!CB.Get()||!CC.Get())return false;
    Chaos::FSimpleWheelConfig WheelConfig;Chaos::FSimpleSuspensionConfig SuspensionConfig;
    Chaos::FSimpleWheeledVehicle Full,Partial;
    for(int32 Index=0;Index<3;++Index){Full.Wheels.Emplace(&WheelConfig);Full.Suspension.Emplace(&SuspensionConfig);}
    for(int32 Index=0;Index<2;++Index)Partial.Wheels.Emplace(&WheelConfig);
    Partial.Suspension.Emplace(&SuspensionConfig);
    const auto SetBodies=[&](bool NullMiddle)
    {
        Parent->WheelSubBodiesWheeledDomainStorage()={A.Get(),NullMiddle?nullptr:B.Get(),C.Get()};
    };
    const auto SetConfigurations=[&](bool All)
    {
        Parent->WheelsWheeledDomainStorage().Reset();Parent->WheelsWheeledDomainStorage().Add(CA.Get());
        if(All){Parent->WheelsWheeledDomainStorage().Add(CB.Get());Parent->WheelsWheeledDomainStorage().Add(CC.Get());}
    };
    ON_SCOPE_EXIT{
        Parent->ClearWheelSimulationPointers();
        for(auto* Body:{A.Get(),B.Get(),C.Get()}){Body->SetWheelSim(nullptr);Body->SetSuspensionSim(nullptr);Body->SetChaosWheel(nullptr);}
        Parent->WheelSubBodiesWheeledDomainStorage().Reset();Parent->WheelsWheeledDomainStorage().Reset();
    };
    A->AuthorWheelBindingFixture(27.f,7.f,11.f);B->AuthorWheelBindingFixture(31.f,-1.f,19.f);C->AuthorWheelBindingFixture(39.f,13.f,-1.f);
    CA->SuspensionMaxRaise=101.f;CA->SuspensionMaxDrop=201.f;
    CB->SuspensionMaxRaise=102.f;CB->SuspensionMaxDrop=202.f;
    CC->SuspensionMaxRaise=103.f;CC->SuspensionMaxDrop=203.f;
    SetBodies(false);SetConfigurations(true);Parent->BindWheelSimulationPointers(&Full);
    TestTrue(TEXT("complete actual legacy pointer binding"),A->GetWheelSim()==&Full.Wheels[0]&&B->GetWheelSim()==&Full.Wheels[1]&&C->GetWheelSim()==&Full.Wheels[2]
        &&A->GetSuspensionSim()==&Full.Suspension[0]&&B->GetSuspensionSim()==&Full.Suspension[1]&&C->GetSuspensionSim()==&Full.Suspension[2]);
    TestTrue(TEXT("actual legacy mass and index assignments"),A->Idx()==0&&B->Idx()==1&&C->Idx()==2
        &&Full.Wheels[0].MassPerWheel==27.f&&Full.Wheels[1].MassPerWheel==31.f&&Full.Wheels[2].MassPerWheel==39.f
        &&A->ReadMassForFixture()==27.f&&B->ReadMassForFixture()==31.f&&C->ReadMassForFixture()==39.f);
    TestTrue(TEXT("both override predicates matter"),CA->SuspensionMaxRaise==7.f&&CA->SuspensionMaxDrop==11.f
        &&CB->SuspensionMaxRaise==102.f&&CB->SuspensionMaxDrop==202.f&&CC->SuspensionMaxRaise==103.f&&CC->SuspensionMaxDrop==203.f);
    Parent->ClearWheelSimulationPointers();
    TestTrue(TEXT("ordinary Clear retains Chaos config while clearing both sims"),A->GetChaosWheelConfiguration()==CA.Get()
        &&B->GetChaosWheelConfiguration()==CB.Get()&&C->GetChaosWheelConfiguration()==CC.Get()
        &&!A->GetWheelSim()&&!B->GetWheelSim()&&!C->GetWheelSim()&&!A->GetSuspensionSim()&&!B->GetSuspensionSim()&&!C->GetSuspensionSim());
    Parent->BindWheelSimulationPointers(&Full);Parent->BindWheelSimulationPointers(nullptr);
    TestTrue(TEXT("null vehicle uses Clear and retains configuration"),A->GetChaosWheelConfiguration()==CA.Get()&&B->GetChaosWheelConfiguration()==CB.Get()
        &&C->GetChaosWheelConfiguration()==CC.Get()&&!A->GetWheelSim()&&!B->GetWheelSim()&&!C->GetWheelSim());
    SetBodies(true);SetConfigurations(true);B->SetIdx(71);
    // Deliberate legacy count mismatch: retain/recognize this exact expected
    // warning. Other warnings/errors remain failures of the campaign.
    AddExpectedMessage(TEXT("[WheelSuspensionBindingIncomplete]"),ELogVerbosity::Warning,EAutomationExpectedMessageFlags::Contains,1,false);
    Parent->BindWheelSimulationPointers(&Partial);
    TestTrue(TEXT("Min3 binds first, skips null middle, clears unmatched tail"),A->GetWheelSim()==&Partial.Wheels[0]
        &&A->GetSuspensionSim()==&Partial.Suspension[0]&&A->Idx()==0&&B->Idx()==71&&C->Idx()==2
        &&!C->GetWheelSim()&&!C->GetSuspensionSim()&&!C->GetChaosWheelConfiguration());
    SetBodies(false);SetConfigurations(false);B->SetChaosWheel(CB.Get());
    Parent->BindWheelSimulationPointers(&Full);
    TestTrue(TEXT("missing config index leaves prior configuration while binding actual sims"),B->GetChaosWheelConfiguration()==CB.Get()
        &&B->GetWheelSim()==&Full.Wheels[1]&&B->GetSuspensionSim()==&Full.Suspension[1]);
    TestFalse(TEXT("legacy fixture remains unregistered"),Parent->IsRegistered());
    AddInfo(TEXT("[LegacyWheelSimulationBindings] scenarios=5 complete=1 clearKeepsConfig=1 nullVehicleClear=1 partialMin3NullBodyTail=1 missingConfigurationIndex=1 meaningfulMassesAndBothOverridePredicates=1. Real authored legacy fixtures only; not native lifetime/state or network qualification."));
    return !HasAnyErrors();
}
#endif
