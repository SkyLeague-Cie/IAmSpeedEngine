#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "UObject/StrongObjectPtr.h"
#include "IAmSpeed/Components/SpeedMovementComponent.h"
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "IAmSpeed/Motion/SpeedPrescribedComponent.h"
#include "IAmSpeed/SubBodies/Solid/BoxSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SphereSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SWheelSubBody.h"
#include "IAmSpeed/SubBodies/Solid/CylinderSubBody.h"
#include "IAmSpeed/SubBodies/Sensor/BoxSensor.h"
#include "IAmSpeed/SubBodies/Sensor/SphereSensor.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSubBodySensorPairMatrix,"IAmSpeed.Collision.SubBodyMatrix.SensorPeers",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FSubBodySensorPairMatrix::RunTest(const FString&)
{
    constexpr float Dt=1.f/300.f;
    for(int32 SensorKind=0;SensorKind<2;++SensorKind)for(int32 PeerKind=0;PeerKind<3;++PeerKind)
    {
        TStrongObjectPtr<USpeedMovementComponent> Owner(NewObject<USpeedMovementComponent>());
        TStrongObjectPtr<USpeedWheeledComponent> PeerOwner(NewObject<USpeedWheeledComponent>());
        TStrongObjectPtr<USensorSubBody> Sensor(SensorKind==0 ? static_cast<USensorSubBody*>(NewObject<UBoxSensor>()) : static_cast<USensorSubBody*>(NewObject<USphereSensor>()));
        TStrongObjectPtr<USolidSubBody> Peer(PeerKind==0 ? static_cast<USolidSubBody*>(NewObject<UBoxSubBody>()) : PeerKind==1 ? static_cast<USolidSubBody*>(NewObject<USphereSubBody>()) : static_cast<USolidSubBody*>(NewObject<USWheelSubBody>()));
        Sensor->Initialize(Owner.Get());Peer->Initialize(PeerOwner.Get());
        if(auto* Box=Cast<UBoxSensor>(Sensor.Get()))Box->SetBoxExtent(FVector(10.));
        if(auto* Sphere=Cast<USphereSensor>(Sensor.Get()))Sphere->SetRadius(10.f);
        if(auto* Box=Cast<UBoxSubBody>(Peer.Get()))Box->SetBoxExtent(FVector(10.));
        if(auto* Sphere=Cast<USphereSubBody>(Peer.Get()))Sphere->SetRadiusForConfiguration(10.f);
        Sensor->AddExternalSubBodies({Peer.Get()});
        SKinematic Moving;Moving.Location={-40.,0.,0.};Moving.Velocity={12000.,0.,0.};
        SKinematic Still;Owner->SetKinematicState(Moving);PeerOwner->SetKinematicState(Still);
        Sensor->SetKinematicState(Moving);Peer->SetKinematicState(Still);
        int32 Begins=0,Ends=0;
        Sensor->OnBeginOverlap.AddLambda([&](auto*,auto*){++Begins;});
        Sensor->OnEndOverlap.AddLambda([&](auto*,auto*){++Ends;});
        Sensor->ResetForFrame(Dt);float TOI=-1.;
        TestTrue(TEXT("Sensor detects fast crossing through actual dynamic SubBody sweeper"),Sensor->SweepTOI(Dt,TOI));
        TestTrue(TEXT("Contact lies within finite canonical step"),TOI>=0.f && TOI<Dt);
        Sensor->AcceptHit();TestTrue(TEXT("Exact peer identity survives dispatch"),Sensor->GetHit().SubBody.Get()==Peer.Get());
        Sensor->ResolveCurrentHit(Dt,0.f);Sensor->ResolveCurrentHit(Dt,0.f);
        TestEqual(TEXT("Repeated hit does not duplicate enter"),Begins,1);
        Sensor->PostPhysicsUpdate();TestTrue(TEXT("Overlap stays active while touched"),Sensor->HasOverlaps());
        TestTrue(TEXT("Sensor never changes peer velocity"),PeerOwner->GetPhysCOMVelocity().IsZero());
        TestTrue(TEXT("Sensor never changes its owner velocity"),FVector(Owner->GetPhysCOMVelocity()).Equals(Moving.Velocity,0.));
        Sensor->ResetForFrame(Dt);Moving.Location.Y=500.;Sensor->SetKinematicState(Moving);
        TestFalse(TEXT("Separated near-miss does not hit"),Sensor->SweepTOI(Dt,TOI));
        Sensor->PostPhysicsUpdate();TestEqual(TEXT("Exit emitted once"),Ends,1);TestFalse(TEXT("Exit removes overlap"),Sensor->HasOverlaps());
        Sensor->PostPhysicsUpdate();TestEqual(TEXT("Repeated empty update does not duplicate exit"),Ends,1);
    }
    return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSubBodyCylinderPeerMatrix,"IAmSpeed.Collision.SubBodyMatrix.CylinderPeers",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FSubBodyCylinderPeerMatrix::RunTest(const FString&)
{
    using namespace Speed::Motion;
    constexpr float Dt=1.f/300.f;
    for(int32 PeerKind=0;PeerKind<3;++PeerKind)
    {
        TStrongObjectPtr<USpeedPrescribedComponent> Owner(NewObject<USpeedPrescribedComponent>());
        TStrongObjectPtr<UCylinderSubBody> Cylinder(NewObject<UCylinderSubBody>());
        TestTrue(TEXT("Finite cylinder dimensions admitted"),Cylinder->Configure(10.,20.));
        const FPrescribedMotionKey Keys[]={{0.,{0.,0.,0.},0.},{1.,{0.,0.,0.},0.}};
        FPrescribedMotionTrack Track;Track.Configure(MakeArrayView(Keys),FVector::UpVector,true);
        auto Gate=std::make_shared<FActivation>();Gate->Stage=0;
        SubBodyConfig Config;Config.bValid=true;Config.Index=0;Config.Mass=0.;
        TestTrue(TEXT("Prescribed cylinder owner configured"),Owner->Configure(Cylinder.Get(),Config,Track,FTransform::Identity,0,Gate));
        Cylinder->Initialize(Owner.Get()); // Pair CCD fixture; canonical admission lifecycle is tested separately.
        TStrongObjectPtr<USpeedWheeledComponent> PeerOwner(NewObject<USpeedWheeledComponent>());
        TStrongObjectPtr<USolidSubBody> Peer(PeerKind==0 ? static_cast<USolidSubBody*>(NewObject<UBoxSubBody>()) : PeerKind==1 ? static_cast<USolidSubBody*>(NewObject<USphereSubBody>()) : static_cast<USolidSubBody*>(NewObject<USWheelSubBody>()));
        Peer->Initialize(PeerOwner.Get());
        if(auto* Box=Cast<UBoxSubBody>(Peer.Get()))Box->SetBoxExtent(FVector(2.));
        if(auto* Sphere=Cast<USphereSubBody>(Peer.Get()))Sphere->SetRadiusForConfiguration(2.f);
        Peer->SetCollisionResponseToAllChannels(ECR_Block);Cylinder->SetCollisionResponseToAllChannels(ECR_Block);
        Cylinder->AddExternalSubBodies({Peer.Get()});
        SKinematic Moving;Moving.Location={30.,0.,0.};Moving.Velocity={-12000.,0.,0.};
        PeerOwner->SetKinematicState(Moving);Peer->SetKinematicState(Moving);Cylinder->SetKinematicState(SKinematic());
        float TOI=-1.;TestTrue(TEXT("Exact cylinder dispatch catches crossing peer"),Cylinder->SweepTOI(Dt,TOI));
        TestTrue(TEXT("Earliest finite cylinder contact agrees with independent side plane"),FMath::Abs(TOI-18.f/12000.f)<2.e-5f);
        Cylinder->AcceptHit();TestTrue(TEXT("Cylinder hit names the correct actual SubBody"),Cylinder->GetHit().SubBody.Get()==Peer.Get());
        Cylinder->ResetForFrame(Dt);Moving.Location.Y=100.;PeerOwner->SetKinematicState(Moving);Peer->SetKinematicState(Moving);
        TestFalse(TEXT("Empty space outside finite cylinder stays empty"),Cylinder->SweepTOI(Dt,TOI));
        TestTrue(TEXT("Detection does not retime prescribed owner"),Owner->GetPhysCOMVelocity().IsZero());
    }
    return !HasAnyErrors();
}
#endif
