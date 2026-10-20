#include "SpeedPrescribedComponent.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "IAmSpeed/World/Collision/StaticCollisionWorld.h"
#include "UObject/StrongObjectPtr.h"
#include "IAmSpeed/SubBodies/Solid/BoxSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SphereSubBody.h"
#include "IAmSpeed/World/Simulation/CanonicalFrameContext.h"
namespace
{
struct FPrescribedCountingStaticWorld final:Speed::IStaticCollisionWorld
{
    mutable int32 Calls=0;
    Speed::Analytic::FWorldHit SweepSingle(const Speed::Analytic::FWorldQuery&)const override{++Calls;return {};}
    bool TryFindDeepestOverlap(const Speed::Analytic::FWorldQuery&,Speed::Analytic::FWorldHit& Hit)const override{++Calls;Hit={};return true;}
    bool HasAuthorityCoverage(const Speed::Analytic::FWorldQuery&)const override{++Calls;return true;}
    bool TrySweepAuthority(const Speed::Analytic::FWorldQuery&,Speed::Analytic::FWorldHit& Hit)const override{++Calls;Hit={};return false;}
    Speed::EStaticCollisionBackend GetBackend()const override{return Speed::EStaticCollisionBackend::SurfaceAnalytic;}
    void VisitPlanarCandidates(const Speed::Analytic::FWorldQuery&,TFunctionRef<void(const Speed::Analytic::FWorldHit&)>)const override{++Calls;}
};
}
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
    FPrescribedCountingStaticWorld StaticWorld;
    M->SetStaticCollisionWorldForFrame(&StaticWorld,1.f/300.f);
    Gate->bInside=true;Step(5);
    const SKinematic ExactTrack=M->GetKinematicState();
    TestFalse(TEXT("Mechanical track never projects onto static road geometry"),M->ProjectEstablishedStaticContacts(1.f/300.f));
    TestEqual(TEXT("Static material stop events cannot split the prescribed clock"),M->GetMaximumCanonicalSupportInterval(1.f/300.f),1.f/300.f);
    M->QuantizePhysicalState();
    TestTrue(TEXT("Track sample survives ordinary quantization entry exactly"),M->GetPhysCOM()==ExactTrack.Location&&M->GetPhysCOMVelocity()==ExactTrack.Velocity&&M->GetPhysRotation()==ExactTrack.Rotation);
    TestEqual(TEXT("Prescribed integration/projection/quantization makes zero static-support queries"),StaticWorld.Calls,0);
    // Independent negative witness: an ordinary physical box still invokes
    // the same static feasibility path. Only the prescribed subtype bypasses it.
    Box->SetMass(100.f);Box->Initialize(Peer.Get());Peer->SetStaticCollisionWorldForFrame(&StaticWorld,1.f/300.f);
    Box->CanApplyQuantizedPose(StaticWorld,Peer->GetKinematicState());
    TestTrue(TEXT("Ordinary finite-mass body keeps static pose feasibility queries"),StaticWorld.Calls>0);
    Box->Initialize(M.Get());M->SetStaticCollisionWorldForFrame(nullptr);
    Gate->bInside=false;++Gate->Episode;Step(6);
    TestTrue(TEXT("Explicit stopped episode reset restores initial obstacle phase"),M->GetPhysCOM().IsZero()&&M->GetPhysCOMVelocity().IsZero());
    return !HasAnyErrors();
}
#endif
