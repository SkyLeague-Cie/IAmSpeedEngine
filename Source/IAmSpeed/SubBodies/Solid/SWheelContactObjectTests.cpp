#include "SWheelSubBody.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "Misc/ScopeExit.h"
#include "Components/BoxComponent.h"
#include "WheelSystem.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSpeedWheelContactObjectTest, "IAmSpeed.Wheel.ContactObjectClassification",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FSpeedWheelContactObjectTest::RunTest(const FString& Parameters)
{
    // Injected stored hits, no queries or real collision/restore provenance proof.
    auto* Wheel=NewObject<USWheelSubBody>();
    auto* Support=NewObject<UBoxComponent>();
    Chaos::FSimpleWheelConfig Setup;
    Chaos::FSimpleWheelSim Sim(&Setup);
    Wheel->SetWheelSim(&Sim);
    ON_SCOPE_EXIT { Wheel->SetWheelSim(nullptr); };
    Sim.SetOnGround(true);
    SHitResult Hit;
    Hit.bHit=true;
    Wheel->SetHit(Hit);
    TestFalse(TEXT("Missing component is not static"),Wheel->IsOnStaticObject());
    TestFalse(TEXT("Missing component is not dynamic"),Wheel->IsOnDynamicObject());
    Hit.Component=Support;
    Support->SetMobility(EComponentMobility::Static);
    Wheel->SetHit(Hit);
    TestTrue(TEXT("Static component classified"),Wheel->IsOnStaticObject());
    TestFalse(TEXT("Static component never dynamic"),Wheel->IsOnDynamicObject());
    Support->SetMobility(EComponentMobility::Movable);
    TestTrue(TEXT("Movable component classified"),Wheel->IsOnDynamicObject());
    TestFalse(TEXT("Movable component never static"),Wheel->IsOnStaticObject());
    Support->SetMobility(EComponentMobility::Stationary);
    TestFalse(TEXT("Stationary remains unknown, not static"),Wheel->IsOnStaticObject());
    TestFalse(TEXT("Stationary remains unknown, not dynamic"),Wheel->IsOnDynamicObject());
    Support->SetMobility(EComponentMobility::Movable);
    Sim.SetOnGround(false);
    TestFalse(TEXT("Retained hit while airborne is not dynamic"),Wheel->IsOnDynamicObject());
    Sim.SetOnGround(true);
    Hit.bHit=false; Wheel->SetHit(Hit);
    TestFalse(TEXT("No hit is not dynamic"),Wheel->IsOnDynamicObject());
    Hit.bHit=true; Wheel->SetHit(Hit);
    Support->MarkAsGarbage();
    TestFalse(TEXT("Invalid component is not dynamic"),Wheel->IsOnDynamicObject());
    return true;
}
#endif
