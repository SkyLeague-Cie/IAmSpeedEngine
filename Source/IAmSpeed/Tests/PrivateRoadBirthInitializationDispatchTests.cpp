#if WITH_DEV_AUTOMATION_TESTS
#include "IAmSpeed/Tests/PrivateRoadBirthFixtureObjects.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Misc/AutomationTest.h"
#include "Misc/ScopeExit.h"
#include "UObject/StrongObjectPtr.h"
#include "ChaosVehicleWheel.h"
#include "WheelSystem.h"
#include "SuspensionSystem.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIASLegacyBirthInitializationDispatch,
    "IAmSpeed.Netcode.Prediction.LegacyBirthInitializationDispatch",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FIASLegacyBirthInitializationDispatch::RunTest(const FString&)
{
    const auto Options=UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(false)
        .CreateNavigation(false).CreateAISystem(false).ShouldSimulatePhysics(false);
    UWorld* World=UWorld::CreateWorld(EWorldType::Game,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Options);
    if(!TestNotNull(TEXT("real legacy fixture world"),World))return false;
    ON_SCOPE_EXIT{World->DestroyWorld(false);};
    auto* Actor=World->SpawnActor<AActor>();if(!TestNotNull(TEXT("real owner"),Actor))return false;
    TStrongObjectPtr<UIASBirthFixtureComponent> Parent(NewObject<UIASBirthFixtureComponent>(Actor));
    TStrongObjectPtr<UChaosVehicleWheel> ChaosWheel(NewObject<UChaosVehicleWheel>(Actor));
    Chaos::FSimpleWheelConfig WheelSetup;WheelSetup.WheelRadius=37.f;
    Chaos::FSimpleWheelSim WheelSim(&WheelSetup);
    Chaos::FSimpleSuspensionConfig SuspensionSetup;SuspensionSetup.SetSuspensionMaxRaise(10.f);SuspensionSetup.SetSuspensionMaxDrop(10.f);
    Chaos::FSimpleSuspensionSim SuspensionSim(&SuspensionSetup);
    if(!TestNotNull(TEXT("actual derived fixture component"),Parent.Get())||!TestNotNull(TEXT("actual Chaos wheel"),ChaosWheel.Get()))return false;
    Parent->BodyConfig.bValid=true;Parent->BodyConfig.Mass=111.f;
    Parent->BodyConfig.BoxExtent=FVector(13,27,39);Parent->BodyConfig.Radius=14.f;
    Parent->BodyConfig.LocalOffset=FVector(2,3,4);Parent->BodyConfig.LocalRotation=FQuat::Identity;
    Parent->BodyConfig.Restitution=0.3f;Parent->BodyConfig.StaticFriction=0.7f;Parent->BodyConfig.DynamicFriction=0.4f;
    Parent->WheelConfig.bValid=true;Parent->WheelConfig.WheelIndex=2;
    Parent->WheelConfig.ChaosWheel=ChaosWheel.Get();Parent->WheelConfig.PWheel=&WheelSim;Parent->WheelConfig.PSuspension=&SuspensionSim;
    ON_SCOPE_EXIT{Parent->WheelConfig={};Parent->Trace.reset();Parent->SetBodiesForFixture(nullptr);};
    const TArray<FString> BaseEvents={TEXT("subbody"),TEXT("subbody"),TEXT("inverse"),TEXT("subbody"),TEXT("inverse")};
    const auto Check=[&](const FString& Name,const std::shared_ptr<FIASBirthFixtureTrace>& Trace,uint32 InverseCount,uint32 WheelQueries)
    {
        TestEqual(*(Name+TEXT(" three actual subbody configuration calls")),Trace->SubBodyQueries,uint32(3));
        TestEqual(*(Name+TEXT(" actual virtual inverse call count")),Trace->Inverse.Num(),int32(InverseCount));
        TestEqual(*(Name+TEXT(" wheel configuration count")),Trace->WheelQueries,WheelQueries);
        TArray<FString> Expected=BaseEvents;if(WheelQueries)Expected.Add(TEXT("wheel"));if(InverseCount==3)Expected.Add(TEXT("inverse"));
        TestTrue(*(Name+TEXT(" actual call order")),Trace->Events==Expected);
    };
    for(int32 Valid=0;Valid<2;++Valid)
    {
        TStrongObjectPtr<UIASBirthFixtureBox> Box(NewObject<UIASBirthFixtureBox>(Actor));if(!Box.Get())return false;
        auto Trace=std::make_shared<FIASBirthFixtureTrace>();Box->Trace=Trace;Parent->Trace=Trace;Parent->SetBodiesForFixture(Box.Get());
        Parent->BodyConfig.bValid=Valid!=0;const auto InitialExtent=Box->GetBoxExtent();const float InitialMass=Box->ReadMassForFixture();
        Box->Initialize(Parent.Get());Check(Valid?TEXT("valid Box"):TEXT("invalid Box"),Trace,2,0);
        if(Trace->Inverse.Num()!=2)return false;
        TestTrue(TEXT("first Box inverse precedes Box extent assignment"),Trace->Inverse[0].Shape==InitialExtent);
        TestTrue(TEXT("second Box inverse observes valid extent or unchanged invalid extent"),Trace->Inverse[1].Shape==(Valid?Parent->BodyConfig.BoxExtent:InitialExtent));
        TestTrue(TEXT("Box mass precedes both inverse calls"),Trace->Inverse[0].Mass==(Valid?111.f:InitialMass)&&Trace->Inverse[1].Mass==(Valid?111.f:InitialMass));
        TestTrue(TEXT("second virtual Box result is stored"),Box->ReadInverseForFixture()==Trace->Inverse[1].Returned);
        TestEqual(TEXT("Base actual body list is consumed"),Box->ReadIgnoredCountForFixture(),1);
    }
    Parent->BodyConfig.bValid=true;
    {
        TStrongObjectPtr<UIASBirthFixtureSphere> Sphere(NewObject<UIASBirthFixtureSphere>(Actor));if(!Sphere.Get())return false;
        auto Trace=std::make_shared<FIASBirthFixtureTrace>();Sphere->Trace=Trace;Parent->Trace=Trace;Parent->SetBodiesForFixture(Sphere.Get());
        const float InitialRadius=Sphere->GetRadius();Sphere->Initialize(Parent.Get());Check(TEXT("valid Sphere"),Trace,2,0);
        if(Trace->Inverse.Num()!=2)return false;
        TestTrue(TEXT("Sphere radius changes between two actual inverse calls"),Trace->Inverse[0].Shape.X==InitialRadius&&Trace->Inverse[1].Shape.X==14.f);
        TestTrue(TEXT("Sphere valid mass retained"),Trace->Inverse[0].Mass==111.f&&Trace->Inverse[1].Mass==111.f);
        TestTrue(TEXT("second virtual Sphere result stored"),Sphere->ReadInverseForFixture()==Trace->Inverse[1].Returned);
    }
    for(int32 WheelValid=0;WheelValid<2;++WheelValid)
    {
        TStrongObjectPtr<UIASBirthFixtureWheel> Wheel(NewObject<UIASBirthFixtureWheel>(Actor));if(!Wheel.Get())return false;
        auto Trace=std::make_shared<FIASBirthFixtureTrace>();Wheel->Trace=Trace;Parent->Trace=Trace;Parent->SetBodiesForFixture(Wheel.Get());
        Parent->WheelConfig.bValid=WheelValid!=0;
        ON_SCOPE_EXIT{Wheel->SetWheelSim(nullptr);Wheel->SetSuspensionSim(nullptr);Wheel->SetChaosWheel(nullptr);};
        const float InitialRadius=Wheel->GetRadius();Wheel->Initialize(Parent.Get());
        Check(WheelValid?TEXT("valid Wheel"):TEXT("invalid wheel config"),Trace,WheelValid?3u:2u,1);
        if(Trace->Inverse.Num()!=(WheelValid?3:2))return false;
        TestTrue(TEXT("Wheel inherited Solid and Sphere order retained"),Trace->Inverse[0].Mass==111.f&&Trace->Inverse[0].Shape.X==InitialRadius
            &&Trace->Inverse[1].Mass==111.f&&Trace->Inverse[1].Shape.X==14.f);
        if(WheelValid)TestTrue(TEXT("last Wheel inverse follows actual simulator radius and default wheel mass"),
            Trace->Inverse[2].Shape.X==WheelSim.GetEffectiveRadius()&&Trace->Inverse[2].Mass==USWheelSubBody::DefaultWheelMass);
        TestTrue(TEXT("last virtual Wheel inverse stored"),Wheel->ReadInverseForFixture()==Trace->Inverse.Last().Returned);
    }
    Parent->WheelConfig={};Parent->Trace.reset();Parent->SetBodiesForFixture(nullptr);
    TestFalse(TEXT("legacy fixtures remain unregistered"),Parent->IsRegistered());
    AddInfo(TEXT("[LegacyBirthInitializationDispatch] realUClassCases=5 boxInverse=2 sphereInverse=2 wheelInverseValid=3 wheelInverseInvalid=2 subbodyQueriesEach=3 validNativeWheelAndSuspensionObjects=1. Legacy/custom callback and assignment-order controls only; no native operation, source/lifetime coverage, state restoration or network qualification."));
    return !HasAnyErrors();
}
#endif
