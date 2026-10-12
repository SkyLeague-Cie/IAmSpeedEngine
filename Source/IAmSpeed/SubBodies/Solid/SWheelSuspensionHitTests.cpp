#include "SWheelSubBody.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "BoxSubBody.h"
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "IAmSpeed/Base/SUtils.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Misc/AutomationTest.h"
#include "Misc/ScopeExit.h"
#include "WheelSystem.h"
#include "SuspensionSystem.h"

// Expose inherited member pointers only. No derived UObject is instantiated,
// no production friendship/access/API change and no sweep override is used.
struct FWheelSuspensionTestAccess : USWheelSubBody
{
    static auto SphereSweep() { return &FWheelSuspensionTestAccess::SweepSuspensionOnSpheres; }
    static auto BoxSweep() { return &FWheelSuspensionTestAccess::SweepSuspensionOnBoxes; }
    static auto Spheres() { return &FWheelSuspensionTestAccess::ExternalSphereSubBodies; }
    static auto Boxes() { return &FWheelSuspensionTestAccess::ExternalBoxSubBodies; }
    static auto Margin() { return &FWheelSuspensionTestAccess::CollisionMargin; }
};

struct FWheelMovementTestAccess : USpeedWheeledComponent
{
    static auto PendingContacts()
    {
        using FGetter = TArray<SWheelGroundContact>& (USpeedWheeledComponent::*)();
        return static_cast<FGetter>(&FWheelMovementTestAccess::GetPendingWheelContacts);
    }
};

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FWheelSuspensionHitProducerTest,
    "IAmSpeed.Wheel.SuspensionHitProducers",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FWheelSuspensionHitProducerTest::RunTest(const FString& Parameters)
{
    // Real primitive producers, but no world queries, BeginPlay, network,
    // physics scene, UpdateSuspension force integration or cue rendering.
    const auto Options = UWorld::InitializationValues().AllowAudioPlayback(false)
        .CreatePhysicsScene(false).CreateNavigation(false).CreateAISystem(false)
        .ShouldSimulatePhysics(false);
    UWorld* World = UWorld::CreateWorld(EWorldType::Game, false, NAME_None,
        nullptr, true, ERHIFeatureLevel::Num, &Options);
    if (!TestNotNull(TEXT("Fixture world"), World)) return false;
    ON_SCOPE_EXIT { World->DestroyWorld(false); };
    AActor* Owner = World->SpawnActor<AActor>();
    AActor* Other = World->SpawnActor<AActor>();
    if (!TestNotNull(TEXT("Owner"), Owner) || !TestNotNull(TEXT("Other owner"), Other)) return false;
    auto* Movement = NewObject<USpeedWheeledComponent>(Owner);
    auto* Wheel = NewObject<USWheelSubBody>(Owner);
    // Initialize performs an ISpeedWheeledComponent downcast: the fixture must
    // supply that real interface. Its default wheel config is invalid, so the
    // explicit stack-owned simulators below remain the only simulator owners.
    Wheel->Initialize(Movement);
    TestFalse(TEXT("Movement remains unregistered"), Movement->IsRegistered());
    TestTrue(TEXT("Default chassis mass is finite and positive"),
        FMath::IsFinite(Movement->GetPhysMass()) && Movement->GetPhysMass() > 0.0f);
    Chaos::FSimpleWheelConfig WheelSetup;
    Chaos::FSimpleWheelSim WheelSim(&WheelSetup);
    Chaos::FSimpleSuspensionConfig SuspensionSetup;
    SuspensionSetup.SetSuspensionMaxRaise(10.0f);
    SuspensionSetup.SetSuspensionMaxDrop(10.0f);
    Chaos::FSimpleSuspensionSim SuspensionSim(&SuspensionSetup);
    Wheel->SetWheelSim(&WheelSim);
    Wheel->SetSuspensionSim(&SuspensionSim);
    auto& Wheels = const_cast<TArray<TObjectPtr<USWheelSubBody>>&>(Movement->GetWheelSubBodies());
    Wheels.Add(Wheel);
    auto& PendingContacts = (Movement->*FWheelMovementTestAccess::PendingContacts())();
    ON_SCOPE_EXIT
    {
        PendingContacts.Reset();
        Wheels.Reset();
        Wheel->SetWheelSim(nullptr);
        Wheel->SetSuspensionSim(nullptr);
    };
    Wheel->SetRadiusForConfiguration(2.0f);
    Wheel->ConfigureSuspensionTravel(10.0f, 10.0f);
    Wheel->SetLocalOffset(FVector::ZeroVector);
    Movement->SetPhysLocation(FVector::ZeroVector);
    Movement->SetPhysRotation(FQuat::Identity);
    SKinematic WheelPose;
    WheelPose.Location = FVector::ZeroVector;
    Wheel->SetKinematicState(WheelPose);
    WheelSim.SetOnGround(true);
    constexpr float Delta = 1.0f / 120.0f;

    auto& Spheres = Wheel->*FWheelSuspensionTestAccess::Spheres();
    auto& Boxes = Wheel->*FWheelSuspensionTestAccess::Boxes();
    auto SphereSweep = FWheelSuspensionTestAccess::SphereSweep();
    auto BoxSweep = FWheelSuspensionTestAccess::BoxSweep();
    auto PoseAt = [](USSubBody* Body, float Z)
    {
        SKinematic Pose;
        Pose.Location = FVector(0.0f, 0.0f, Z);
        Pose.Rotation = FQuat::Identity;
        Body->SetKinematicState(Pose);
        Body->SetMobility(EComponentMobility::Movable);
    };
    auto* NearSphere = NewObject<USphereSubBody>(Other);
    auto* FarSphere = NewObject<USphereSubBody>(Other);
    auto* TieSphere = NewObject<USphereSubBody>(Other);
    auto* SelfSphere = NewObject<USphereSubBody>(Owner);
    for (auto* Sphere : {NearSphere, FarSphere, TieSphere, SelfSphere})
        Sphere->SetRadiusForConfiguration(2.0f);
    PoseAt(NearSphere, -4.0f); PoseAt(FarSphere, -8.0f);
    PoseAt(TieSphere, -4.0f); PoseAt(SelfSphere, 3.0f);
    auto* NearBox = NewObject<UBoxSubBody>(Other);
    auto* FarBox = NewObject<UBoxSubBody>(Other);
    auto* TieBox = NewObject<UBoxSubBody>(Other);
    auto* SelfBox = NewObject<UBoxSubBody>(Owner);
    for (auto* Box : {NearBox, FarBox, TieBox, SelfBox}) Box->SetBoxExtent(FVector(2.0f));
    PoseAt(NearBox, -4.0f); PoseAt(FarBox, -8.0f);
    PoseAt(TieBox, -4.0f); PoseAt(SelfBox, 3.0f);

    auto CheckHit = [&](const SHitResult& Hit, USSubBody* Support, const SHitResult& Geometry)
    {
        TestTrue(TEXT("Selected component identity"), Hit.Component.Get() == Support);
        TestTrue(TEXT("Selected subbody identity"), Hit.SubBody.Get() == Support);
        TestEqual(TEXT("Producer frame stamp"), Hit.FrameTag, static_cast<uint32>(Movement->NumFrame()));
        TestEqual(TEXT("TOI retained"), Hit.TOI, Geometry.TOI);
        TestEqual(TEXT("Impact point retained"), Hit.ImpactPoint, Geometry.ImpactPoint);
        TestEqual(TEXT("Blocking flag retained"), Hit.bBlockingHit, Geometry.bBlockingHit);
        TestEqual(TEXT("Penetration flag retained"), Hit.bStartPenetrating, Geometry.bStartPenetrating);
        TestEqual(TEXT("Penetration depth retained"), Hit.PenetrationDepth, Geometry.PenetrationDepth);
        const FVector Normal = Speed::QuantizeUnitNormal(Geometry.ImpactNormal);
        TestEqual(TEXT("Existing normal quantization"), Hit.ImpactNormal, Normal);
        TestEqual(TEXT("Existing wheel location"), Hit.Location, Geometry.ImpactPoint + Wheel->Radius() * Normal);
        TestEqual(TEXT("No fabricated static face"), Hit.FaceIndex, Geometry.FaceIndex);
        TestEqual(TEXT("No fabricated analytic identity"), Hit.SourceId, Geometry.SourceId);
    };
    // Derive the fixture's sweep segment from the configured travel. With zero
    // spring displacement and safety margin, PredictNextDisplacement = -drop.
    const FVector Start(0.0f, 0.0f, 10.0f);
    const FVector End(0.0f, 0.0f, -10.0f - (Wheel->*FWheelSuspensionTestAccess::Margin())());
    const SSphere Shape(Start, Wheel->Radius(), FVector::ZeroVector, FVector::ZeroVector);

    // Run each actual producer with its native candidate array and comparator.
    auto Exercise = [&](auto& Candidates, auto Sweep, auto* Near, auto* Far, auto* Tie,
                        auto* Self, const SHitResult& Geometry)
    {
        // Production SweepSuspension passes CurrentHit itself to these branches.
        // Keep that alias so real SetOnGround/register-contact sees the winner.
        SHitResult& Hit = const_cast<SHitResult&>(Wheel->GetHit());
        Hit.FrameTag = 123456u;
        Candidates = {nullptr, Self, Far, Near};
        if (!TestTrue(TEXT("Real producer hit"), (Wheel->*Sweep)(Hit, Delta))) return;
        TestEqual(TEXT("Real notification recounts the one fixture wheel"),
            Movement->NumWheelsOnGround(), static_cast<unsigned char>(1));
        TestEqual(TEXT("Real contact registration remains bounded"),
            PendingContacts.Num(), 1);
        CheckHit(Hit, Near, Geometry);
        Wheel->SetHit(Hit);
        TestTrue(TEXT("Produced movable contact is classified"), Wheel->IsOnDynamicObject());
        const SHitResult Produced = Hit;

        Candidates = {Near, Tie};
        TestTrue(TEXT("Equal TOI hit"), (Wheel->*Sweep)(Hit, Delta));
        TestTrue(TEXT("Equal TOI keeps first candidate"), Hit.SubBody.Get() == Near);
        Candidates = {Tie, Near};
        TestTrue(TEXT("Reversed equal TOI hit"), (Wheel->*Sweep)(Hit, Delta));
        TestTrue(TEXT("Equal TOI follows existing order"), Hit.SubBody.Get() == Tie);

        Candidates = {Near}; Wheel->SetHit(Produced); Wheel->AcceptHit();
        TestFalse(TEXT("AcceptHit excludes selected component this frame"), (Wheel->*Sweep)(Hit, Delta));
        Wheel->ResetForFrame(Delta);
        TestTrue(TEXT("ResetForFrame readmits selected support"), (Wheel->*Sweep)(Hit, Delta));
        Near->SetMobility(EComponentMobility::Static);
        TestTrue(TEXT("Static support still produced"), (Wheel->*Sweep)(Hit, Delta));
        Wheel->SetHit(Hit);
        TestTrue(TEXT("Dynamic to static classified static"), Wheel->IsOnStaticObject());
        TestFalse(TEXT("Static is not dynamic"), Wheel->IsOnDynamicObject());
        Near->SetMobility(EComponentMobility::Stationary);
        TestTrue(TEXT("Stationary support still produced"), (Wheel->*Sweep)(Hit, Delta));
        Wheel->SetHit(Hit);
        TestFalse(TEXT("Stationary classification unknown static"), Wheel->IsOnStaticObject());
        TestFalse(TEXT("Stationary classification unknown dynamic"), Wheel->IsOnDynamicObject());
        Near->SetMobility(EComponentMobility::Movable);

        Candidates = {nullptr, Self};
        TestFalse(TEXT("Absent and self candidates miss"), (Wheel->*Sweep)(Hit, Delta));
        // A branch miss does not itself clear retained CurrentHit. Do not claim
        // that IsOnDynamicObject is a freshness or restore certificate.
        Wheel->SetHit(Produced); WheelSim.SetOnGround(false);
        TestFalse(TEXT("Airborne retained hit is rejected"), Wheel->IsOnDynamicObject());
        WheelSim.SetOnGround(true); Wheel->SetHit(Produced);
        TestTrue(TEXT("Restored same-frame stored hit only classifies mobility"), Wheel->IsOnDynamicObject());
        TestEqual(TEXT("Restore may preserve same frame tag"), Wheel->GetHit().FrameTag, Produced.FrameTag);

        Near->MarkAsGarbage(); Candidates = {Near};
        TestFalse(TEXT("Destroyed candidate rejected by actual producer"), (Wheel->*Sweep)(Hit, Delta));
        TestFalse(TEXT("Destroyed retained support is not dynamic"), Wheel->IsOnDynamicObject());
        TestFalse(TEXT("Destroyed retained support is not static"), Wheel->IsOnStaticObject());
        Candidates.Reset(); Wheel->ResetForFrame(Delta);
        // No contact solver is run. Clear real registered records before the
        // next producer fixture and before detaching stack-owned simulators.
        PendingContacts.Reset();
    };
    Exercise(Spheres, SphereSweep, NearSphere, FarSphere, TieSphere, SelfSphere,
        Shape.IntersectDuringMovement(NearSphere->MakeSphere(), Start, End, Delta));
    Exercise(Boxes, BoxSweep, NearBox, FarBox, TieBox, SelfBox,
        Shape.IntersectDuringMovement(NearBox->MakeBox(), Start, End, Delta));
    return true;
}
#endif
