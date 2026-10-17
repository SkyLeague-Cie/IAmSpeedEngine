#include "SpeedPrescribedComponent.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "UObject/StrongObjectPtr.h"
#include "IAmSpeed/SubBodies/Solid/BoxSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SphereSubBody.h"
#include "IAmSpeed/World/Simulation/CanonicalFrameContext.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPrescribedPhysicalComponentTest,"IAmSpeed.Motion.PhysicalComponent",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FPrescribedPhysicalComponentTest::RunTest(const FString&)
{
    using namespace Speed::Motion;
    TStrongObjectPtr<USpeedPrescribedComponent> M(NewObject<USpeedPrescribedComponent>());
    TStrongObjectPtr<UBoxSubBody> Box(NewObject<UBoxSubBody>());
    const FPrescribedMotionKey Keys[]={{0.,{0.,0.,0.},0.},{2.,{100.,0.,0.},0.},{4.,{0.,0.,0.},0.}};
    FPrescribedMotionTrack Track;Track.Configure(MakeArrayView(Keys),{0.,0.,1.},true);
    auto Gate=std::make_shared<FActivation>();Gate->Stage=0;
    SubBodyConfig Config;Config.bValid=true;Config.Index=0;Config.BoxExtent={2.,3.,2.};Config.Mass=0.;Config.Restitution=1.;
    TestTrue(TEXT("Mechanical body configured"),M->Configure(Box.Get(),Config,Track,FTransform::Identity,0,Gate));
    // Isolated component unit test, not a UWorld/runtime admission claim.
    Box->Initialize(M.Get());M->bAdmitted=true;
    auto Step=[&](uint64 Frame){M->PrepareCanonicalFrame(FCanonicalFrameContext(Frame));M->IntegrateKinematics(1.f/300.f);};
    Step(0);TestTrue(TEXT("Rest stage has zero physical speed"),M->GetPhysCOMVelocity().IsZero()&&M->GetPhysCOM().IsZero());
    Gate->bInside=true;Step(1);
    TestTrue(TEXT("Entering stage advances exact physical pose"),FVector(M->GetPhysCOM()).Equals({1./6.,0.,0.},1.e-8));
    TestTrue(TEXT("Physical speed is observable at contacts"),(FVector(M->GetPhysCOMVelocity())*double(FCanonicalFrameContext::CanonicalPhysicalDeltaTime)).Equals({1./6.,0.,0.},1.e-9));
    const SKinematic Before=M->GetKinematicState();
    M->AddPhysImpulseAtPoint({1.e8,1.e8,1.e8},{100.,100.,100.});
    TestTrue(TEXT("Impulse cannot overwrite prescribed pose or velocity"),M->GetPhysCOM()==Before.Location&&M->GetPhysCOMVelocity()==Before.Velocity&&M->GetPhysAngularVelocity()==Before.AngularVelocity);
    TestTrue(TEXT("Box zero-mass inertia remains finite zero"),Box->ComputeWorldInvInertiaTensor().Equals(FMatrix(ForceInitToZero),0.));
    Gate->bInside=false;Step(2);TestTrue(TEXT("Exiting stage holds current phase"),FVector(M->GetPhysCOM()).Equals({1./6.,0.,0.},1.e-8)&&M->GetPhysCOMVelocity().IsZero());
    TArray<uint8> Saved;M->AppendSimulationSnapshot(Saved);
    TestEqual(TEXT("Explicit numeric payload width"),Saved.Num(),58);
    TestTrue(TEXT("Own snapshot admitted"),M->CanRestoreSimulationSnapshot(Saved));
    auto Bad=Saved;Bad[0]=255;TestFalse(TEXT("Unknown snapshot version rejected"),M->CanRestoreSimulationSnapshot(Bad));
    Gate->bInside=true;Step(3);TestTrue(TEXT("Reentry resumes phase"),FVector(M->GetPhysCOM()).Equals({1./3.,0.,0.},1.e-8));
    M->RestoreSimulationSnapshot(Before,false,Saved);
    TestTrue(TEXT("Snapshot restores the saved phase and stopped speed"),FVector(M->GetPhysCOM()).Equals({1./6.,0.,0.},1.e-8)&&M->GetPhysCOMVelocity().IsZero());
    M->SetIsFrozen(true);Step(3);TestTrue(TEXT("Pause holds moving-stage phase"),FVector(M->GetPhysCOM()).Equals({1./6.,0.,0.},1.e-8));
    M->SetIsFrozen(false);Step(4);
    // Real impulse routine, not a copied formula: mass100 sphere hit by an
    // infinite-inertia wall at50cm/s with restitution1 must leave at100cm/s.
    TStrongObjectPtr<USpeedMovementComponent> Peer(NewObject<USpeedMovementComponent>());Peer->PhysMass=100.f;
    TStrongObjectPtr<USphereSubBody> Sphere(NewObject<USphereSubBody>());Sphere->SetRadiusForConfiguration(1.f);Sphere->Initialize(Peer.Get());
    SKinematic B;B.Location=FVector(M->GetPhysCOM())+FVector(3.,0.,0.);Peer->SetKinematicState(B);Sphere->UpdateKinematicsFromOwner(B);
    const FVector P=FVector(M->GetPhysCOM())+FVector(2.,0.,0.);
    SHitResult Hit(true,P,-FVector::ForwardVector,0.f,Sphere.Get());Hit.SubBody=Sphere.Get();
    const FVector WallVelocityBeforeContact=M->GetPhysCOMVelocity();
    TestTrue(TEXT("Prescribed contact response admitted"),USolidSubBody::ResolvePrescribedContact(*Box,*Sphere,Hit,0.));
    TestTrue(TEXT("Physical sphere receives finite wall-relative rebound"),FVector(Peer->GetPhysCOMVelocity()).Equals({100.,0.,0.},1.e-4));
    TestTrue(TEXT("Wall velocity retained after real response"),M->GetPhysCOMVelocity()==WallVelocityBeforeContact);
    Gate->bInside=false;++Gate->Episode;Step(5);
    TestTrue(TEXT("Explicit stopped episode reset restores initial obstacle phase"),M->GetPhysCOM().IsZero()&&M->GetPhysCOMVelocity().IsZero());
    return !HasAnyErrors();
}
#endif
