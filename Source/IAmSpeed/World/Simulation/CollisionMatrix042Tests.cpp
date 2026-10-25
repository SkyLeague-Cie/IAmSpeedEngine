#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "FastSimulation.h"
#include "CanonicalFrameContext.h"
#include "CollisionMatrixObservationForTesting.h"
#include "IAmSpeed/Components/ISpeedComponent.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "IAmSpeed/World/Collision/StaticCollisionWorld.h"
#include "IAmSpeed/SubBodies/Solid/SphereSubBody.h"
#include "IAmSpeed/SubBodies/Solid/BoxSubBody.h"
#include "IAmSpeed/SubBodies/Solid/CylinderSubBody.h"
#include "IAmSpeed/SubBodies/Solid/RingSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SWheelSubBody.h"
#include "IAmSpeed/Components/ISpeedWheeledComponent.h"
#include "IAmSpeed/SubBodies/Configs/WheelSubBodyConfig.h"
#include "IAmSpeed/SubBodies/Configs/SubBodyConfig.h"
#include "ChaosVehicleWheel.h"
#include "SimpleVehicle.h"
#include "SuspensionSystem.h"
#include "WheelSystem.h"
#include "Components/BoxComponent.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformProcess.h"
#include "HAL/FileManager.h"
#include "Math/QuatRotationTranslationMatrix.h"
#include "Misc/AutomationTest.h"
#include "Misc/FileHelper.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"
#include "Misc/ScopeExit.h"
#include "UObject/GarbageCollection.h"

namespace
{
using namespace Speed;
enum class EMatrixShape : uint8 { Sphere, Box, Wheel, Cylinder, Plane, Ring };
const TCHAR* Name(EMatrixShape S)
{
    switch (S) {
    case EMatrixShape::Sphere: return TEXT("Sphere");
    case EMatrixShape::Box: return TEXT("Box");
    case EMatrixShape::Wheel: return TEXT("Wheel");
    case EMatrixShape::Cylinder: return TEXT("CylinderPrescribed");
    case EMatrixShape::Plane: return TEXT("AnalyticPlane");
    default: return TEXT("RingStatic");
    }
}
FString KinematicCsv(const SKinematic& K)
{
    return FString::Printf(TEXT("%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g"),
        K.Location.X,K.Location.Y,K.Location.Z,K.Rotation.X,K.Rotation.Y,K.Rotation.Z,K.Rotation.W,
        K.Velocity.X,K.Velocity.Y,K.Velocity.Z,K.AngularVelocity.X,K.AngularVelocity.Y,K.AngularVelocity.Z);
}
class FMatrixCVar
{
    IConsoleVariable* V; int32 Old;
public:
    FMatrixCVar(const TCHAR* N, int32 X) : V(IConsoleManager::Get().FindConsoleVariable(N)), Old(V ? V->GetInt() : 0)
    { if (V) V->Set(X, ECVF_SetByCode); }
    ~FMatrixCVar() { if (V) V->Set(Old, ECVF_SetByCode); }
};

/** Real native sub-bodies; this owner supplies no replacement collision solver. */
class FMatrixBody final : public ISpeedComponent
{
public:
    SKinematic State; SubBodyConfig Config; TArray<USSubBody*> Bodies;
    USolidSubBody* Body = nullptr; FVector Local = FVector::ZeroVector;
    FQuat Previous = FQuat::Identity;
    bool Prescribed = false, Controller = false, Frozen = false, ClockValid = true;
    uint32 Frame = 0, Hz = 300, Published = 0, Limit = 0;
    uint64 FirstFrame = 0;
    TFunction<void(uint64)> PublicationObserver;
    void Initialize(AActor& Actor, EMatrixShape Shape)
    {
        Prescribed = Shape == EMatrixShape::Cylinder;
        Config.bValid = Config.bIsMainSubBody = true;
        Config.Mass = Prescribed ? 0.f : 10.f;
        Config.Radius = 12.5f; Config.BoxExtent = FVector(20, 12, 8);
        Config.Restitution = Config.StaticFriction = Config.DynamicFriction = 0;
        Config.ImpactThreshold = 0; Config.HitDamping = 1;
        if (Shape == EMatrixShape::Sphere) Body = NewObject<USphereSubBody>(&Actor);
        else if (Shape == EMatrixShape::Box) Body = NewObject<UBoxSubBody>(&Actor);
        else {
            auto* Cylinder = NewObject<UCylinderSubBody>(&Actor);
            Cylinder->Configure(25., 30.); Body = Cylinder;
        }
        Bodies.Add(Body); Actor.AddInstanceComponent(Body); Body->Initialize(this);
        // This is a blocking-pair matrix. Cylinder's exact dispatch correctly
        // checks BOTH native response containers, whose base default is Ignore.
        Body->SetCollisionResponseToAllChannels(ECR_Block);
    }
    bool IsCanonicalRunController() const override { return Controller; }
    bool IsCanonicalRunReady() const override { return true; }
    bool IsCanonicalRunComplete() const override { return Published >= Limit; }
    bool IsPrescribedMotion() const override { return Prescribed; }
    void PrepareCanonicalFrame(const FCanonicalFrameContext& C) override
    {
        Frame = uint32(C.NumFrame) + 1;
        ClockValid &= C.NumFrame == FirstFrame + Published
            && C.PhysicalDeltaTime == 1.f / float(Hz)
            && C.SimTime == float(Published) * C.PhysicalDeltaTime;
        State.Acceleration = FVector::ZeroVector;
        State.AngularAcceleration = FVector::ZeroVector;
    }
    void OnCanonicalFramePublished(uint64 N) override
    { ClockValid &= N == FirstFrame + Published; ++Published; if (PublicationObserver) PublicationObserver(N); }
    unsigned int NumFrame() const override { return Frame; }
    float GetPhysMass() const override { return Config.Mass; }
    const FVector& GetPhysCOM() const override { return State.Location; }
    const FVector& GetPhysCenterOfMassLocal() const override { return Local; }
    const TArray<USSubBody*>& GetSubBodies() const override { return Bodies; }
    TArray<USSubBody*> CreateSubBodies() override { return Bodies; }
    SubBodyConfig GetSubBodyConfig(const USSubBody&) const override { return Config; }
    bool IsFrozen() const override { return Frozen; }
    void SetIsFrozen(bool X) override { Frozen = X; }
    SKinematic GetKinematicsOfSubBody(const USSubBody& B, const unsigned int&) const override
    {
        if (const auto* Box=Cast<UBoxSubBody>(&B)) return Box->GetKinematicsFromOwnerKS(State);
        return State; // this fixture's sphere/cylinder local pose is identity
    }
    FMatrix ComputeWorldInvInertiaTensor() const override
    {
        if (Prescribed) return FMatrix(ForceInitToZero);
        FMatrix I = FMatrix::Identity;
        if (Cast<USphereSubBody>(Body)) {
            I.M[0][0] = I.M[1][1] = I.M[2][2] = 2.5 / (Config.Mass * Config.Radius * Config.Radius);
        } else {
            const FVector E = Config.BoxExtent;
            I.M[0][0] = 3. / (Config.Mass * (E.Y*E.Y + E.Z*E.Z));
            I.M[1][1] = 3. / (Config.Mass * (E.X*E.X + E.Z*E.Z));
            I.M[2][2] = 3. / (Config.Mass * (E.X*E.X + E.Y*E.Y));
        }
        const FMatrix R = FQuatRotationMatrix(State.Rotation);
        return R.GetTransposed() * I * R;
    }
    FMatrix ComputeWorldInvInertiaTensorOfSubBody(const USSubBody&) const override { return ComputeWorldInvInertiaTensor(); }
    const SKinematic& GetKinematicState() const override { return State; }
    const SKinematic& GetKinematicStateForFrame(const unsigned int&) const override { return State; }
    void SetKinematicState(const SKinematic& S) override
    {
        // Match the production prescribed owner: only the authored track may
        // write it. Generic impulses/projections cannot deflect this fixture.
        if (!Prescribed) State = S;
    }
    float GetPhysMaxSpeed() const override { return 20000; }
    float GetPhysMaxAngularSpeed() const override { return 100; }
    FVector GetNominalGravityAcceleration() const override { return FVector::ZeroVector; }
    void SetIsUpsideDown(bool) override {}
    bool IsUpsideDown() const override { return false; }
    bool IsInAutoRecover() const override { return false; }
    bool IsSubBodyInAutoRecoverMode() const override { return false; }
    void RcvImpactOnSubBody(const USSubBody&, const FVector&) override {}
    void PostPhysicsUpdatePrv(const float&) override
    {
        // A stationary prescribed pose is authored exactly, including a common
        // fractional translation; the generic free-body quantizer is no writer.
        if (Prescribed) return;
        QuantizeKinematicState(State, Previous); Previous = State.Rotation;
    }
};

/** One native wheel, its real Chaos wheel/suspension storage and the production
 * ISpeedWheeledComponent support solver. It is a unit-body fixture, not a car
 * gameplay certificate. Core holds authored rigid-body values, not a solver. */
class FMatrixWheelOwner final : public ISpeedWheeledComponent
{
public:
    FMatrixBody Core;
    Chaos::FSimpleWheelConfig WheelConfig;
    Chaos::FSimpleSuspensionConfig SuspensionConfig;
    TUniquePtr<Chaos::FSimpleWheeledVehicle> Vehicle;
    TArray<TObjectPtr<USWheelSubBody>> Wheels;
    TArray<SWheelGroundContact> Contacts;
    UChaosVehicleWheel* ChaosWheel = nullptr;
    TFunction<void(const SHitResult&,const SKinematic&,const SKinematic&)> SupportObserver;
    void Initialize(AActor& Actor)
    {
        Core.Config.bValid = Core.Config.bIsMainSubBody = true;
        Core.Config.Mass = 18; Core.Config.Radius = 12.5;
        Core.Config.ImpactThreshold = 0; Core.Config.HitDamping = 1;
        WheelConfig.WheelRadius = 12.5; WheelConfig.WheelMass = 18;
        SuspensionConfig.SetSuspensionMaxRaise(2); SuspensionConfig.SetSuspensionMaxDrop(2);
        Vehicle = MakeUnique<Chaos::FSimpleWheeledVehicle>();
        Vehicle->Wheels.Emplace(&WheelConfig); Vehicle->Suspension.Emplace(&SuspensionConfig);
        ChaosWheel = NewObject<UChaosVehicleWheel>(&Actor);
        ChaosWheel->WheelRadius=12.5f; ChaosWheel->WheelMass=18.f;
        ChaosWheel->SuspensionMaxRaise=2.f; ChaosWheel->SuspensionMaxDrop=2.f;
        auto* Wheel = NewObject<USWheelSubBody>(&Actor);
        Wheels.Add(Wheel); Core.Bodies.Add(Wheel); Core.Body = Wheel;
        Actor.AddInstanceComponent(Wheel); Wheel->Initialize(this);
        Wheel->SetCollisionResponseToAllChannels(ECR_Block);
    }
    ~FMatrixWheelOwner()
    {
        for (auto& Wheel : Wheels) if (Wheel) {
            Wheel->SetWheelSim(nullptr); Wheel->SetSuspensionSim(nullptr); Wheel->SetChaosWheel(nullptr);
        }
    }
    bool ValidateSimulationBindings(FString& Reason) const override
    {
        const bool Valid = Vehicle && Vehicle->Wheels.Num()==1 && Vehicle->Suspension.Num()==1
            && Wheels.Num()==1 && Wheels[0] && Wheels[0]->GetWheelSim()==&Vehicle->Wheels[0]
            && Wheels[0]->GetSuspensionSim()==&Vehicle->Suspension[0]
            && Wheels[0]->GetChaosWheelConfiguration()==ChaosWheel;
        if (!Valid) Reason=TEXT("CollisionMatrix042 real wheel storage binding invalid");
        return Valid;
    }
    WheelSubBodyConfig GetWheelSubBodyConfig(const USWheelSubBody&) const override
    {
        WheelSubBodyConfig C; static_cast<SubBodyConfig&>(C)=Core.Config;
        C.bValid=true; C.WheelIndex=0; C.ChaosWheel=ChaosWheel;
        C.PWheel=&Vehicle->Wheels[0]; C.PSuspension=&Vehicle->Suspension[0]; return C;
    }
    TArray<USWheelSubBody*> CreateWheelSubBodies() override { return {Wheels[0].Get()}; }
    const TArray<TObjectPtr<USWheelSubBody>>& GetWheelSubBodies() const override { return Wheels; }
    void UpdateWheelOnGroundStates() override {} // no separate gameplay ground-state cache
    float GetPhysThrottleInput() const override { return 0; }
    float GetPhysBrakeInput() const override { return 0; }
    float GetPhysSteeringInput() const override { return 0; }
    void RegisterWheelGroundContact(const SWheelGroundContact& C) override { Contacts.Add(C); }
    bool IsCanonicalRunController() const override { return Core.Controller; }
    bool IsCanonicalRunReady() const override { return true; }
    bool IsCanonicalRunComplete() const override { return Core.Published>=Core.Limit; }
    void PrepareCanonicalFrame(const FCanonicalFrameContext& C) override { Core.PrepareCanonicalFrame(C); }
    void OnCanonicalFramePublished(uint64 N) override { Core.OnCanonicalFramePublished(N); }
    unsigned int NumFrame() const override { return Core.Frame; }
    float GetPhysMass() const override { return Core.Config.Mass; }
    const FVector& GetPhysCOM() const override { return Core.State.Location; }
    const FVector& GetPhysCenterOfMassLocal() const override { return Core.Local; }
    const TArray<USSubBody*>& GetSubBodies() const override { return Core.Bodies; }
    TArray<USSubBody*> CreateSubBodies() override { return Core.Bodies; }
    SubBodyConfig GetSubBodyConfig(const USSubBody&) const override { return Core.Config; }
    bool IsFrozen() const override { return Core.Frozen; }
    void SetIsFrozen(bool X) override { Core.Frozen=X; }
    SKinematic GetKinematicsOfSubBody(const USSubBody&,const unsigned int&) const override { return Core.State; }
    FMatrix ComputeWorldInvInertiaTensor() const override { return Core.ComputeWorldInvInertiaTensor(); }
    FMatrix ComputeWorldInvInertiaTensorOfSubBody(const USSubBody&) const override { return ComputeWorldInvInertiaTensor(); }
    const SKinematic& GetKinematicState() const override { return Core.State; }
    const SKinematic& GetKinematicStateForFrame(const unsigned int&) const override { return Core.State; }
    void SetKinematicState(const SKinematic& S) override { Core.State=S; }
    float GetPhysMaxSpeed() const override { return 20000; }
    float GetPhysMaxAngularSpeed() const override { return 100; }
    FVector GetNominalGravityAcceleration() const override { return FVector::ZeroVector; }
    void SetIsUpsideDown(bool) override {}
    bool IsUpsideDown() const override { return false; }
    bool IsInAutoRecover() const override { return false; }
    bool IsSubBodyInAutoRecoverMode() const override { return false; }
    void RcvImpactOnSubBody(const USSubBody&,const FVector&) override {}
protected:
    TArray<SWheelGroundContact>& GetPendingWheelContacts() override { return Contacts; }
    const TArray<SWheelGroundContact>& GetPendingWheelContacts() const override { return Contacts; }
    void PostPhysicsUpdatePrv(const float& Dt) override
    {
        const SKinematic Before=Core.State;
        ISpeedWheeledComponent::PostPhysicsUpdatePrv(Dt);
        QuantizeKinematicState(Core.State,Core.Previous); Core.Previous=Core.State.Rotation;
        // Suspension contacts are real but are NOT CCD-selected impacts.
        // Observe the actual frame-end support response without inventing a TOI.
        if (SupportObserver && Wheels[0]->IsOnGround() && Wheels[0]->GetHit().bBlockingHit)
            SupportObserver(Wheels[0]->GetHit(),Before,Core.State);
    }
};

struct FMatrixCase
{
    EMatrixShape A = EMatrixShape::Sphere, B = EMatrixShape::Plane;
    int32 Variant = 0; uint32 Hz = 300; FVector Offset = FVector::ZeroVector;
};
struct FMatrixResult
{
    bool Complete = false, ClockValid = false, HasHit = false, Overflow = false;
    uint32 Events = 0, Frames = 0;
    uint32 DriverGeneration = 0;
    bool TerminalFailure = false;
    uint32 SupportSamples=0;
    TArray<FString> RawSupportSamples;
    double Time = 0; FVector Point = FVector::ZeroVector, Normal = FVector::ZeroVector;
    FVector VelocityA=FVector::ZeroVector, AngularA=FVector::ZeroVector, VelocityB=FVector::ZeroVector, AngularB=FVector::ZeroVector;
    SKinematic FinalA, FinalB;
    SKinematic InitialA,InitialB;
    FVector EnvironmentOrigin=FVector::ZeroVector;
    double MassA=0,MassB=0;
    EContactFeatureKind FeatureA = EContactFeatureKind::Unknown, FeatureB = EContactFeatureKind::Unknown;
    TArray<FString> RawEvents;
    uint64 FirstGlobalFrame=0; uint32 WorldId=0;
    bool IterationExhausted=false, ActualDeltaValid=true, RealWheelHit=false;
    double MaximumDeltaErrorMs=0;
};
class FMatrixObserver final : public CollisionMatrix042::FObserver
{
public:
    const ISpeedComponent* A; uint64 FirstFrame; float Delta; FVector Offset;
    FMatrixResult& Result;
    FMatrixObserver(const ISpeedComponent* InA, uint64 First, float Dt, const FVector& O, FMatrixResult& R)
        : A(InA), FirstFrame(First), Delta(Dt), Offset(O), Result(R) {}
    void Resolved(const CollisionMatrix042::FResolution& R) override
    {
        if (R.Owner != A && R.Other != A) return;
        if (!R.Hit.bHit || !R.Hit.bBlockingHit) return;
        Result.RealWheelHit |= A->GetSubBodies().Num()==1 && Cast<USWheelSubBody>(A->GetSubBodies()[0])!=nullptr;
        if (++Result.Events > 4096) { Result.Overflow = true; return; }
        const bool Direct = R.Owner == A;
        const auto& SA = Direct ? R.After : R.OtherAfter;
        const auto& SB = Direct ? R.OtherAfter : R.After;
        const FVector Point = R.Hit.ImpactPoint - Offset;
        const FVector Normal = Direct ? R.Hit.ImpactNormal : -R.Hit.ImpactNormal;
        Result.RawEvents.Add(FString::Printf(TEXT("%llu,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%u,%u,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g\n"),
            R.Frame - FirstFrame, double(R.TimeWithinFrame), double(R.Hit.TOI), Point.X, Point.Y, Point.Z,
            Normal.X, Normal.Y, Normal.Z,
            unsigned(Direct ? R.Hit.ContactFeatureThis : R.Hit.ContactFeatureOther),
            unsigned(Direct ? R.Hit.ContactFeatureOther : R.Hit.ContactFeatureThis),
            SA.Velocity.X, SA.Velocity.Y, SA.Velocity.Z, SA.AngularVelocity.X, SA.AngularVelocity.Y, SA.AngularVelocity.Z,
            SB.Velocity.X, SB.Velocity.Y, SB.Velocity.Z, SB.AngularVelocity.X, SB.AngularVelocity.Y, SB.AngularVelocity.Z));
        if (Result.HasHit) return;
        Result.HasHit = true;
        Result.Time = double(R.Frame - FirstFrame) * double(Delta) + R.TimeWithinFrame;
        Result.Point = R.Hit.ImpactPoint - Offset;
        Result.Normal = Direct ? R.Hit.ImpactNormal : -R.Hit.ImpactNormal;
        Result.FeatureA = Direct ? R.Hit.ContactFeatureThis : R.Hit.ContactFeatureOther;
        Result.FeatureB = Direct ? R.Hit.ContactFeatureOther : R.Hit.ContactFeatureThis;
        Result.VelocityA = SA.Velocity; Result.AngularA = SA.AngularVelocity;
        Result.VelocityB = SB.Velocity; Result.AngularB = SB.AngularVelocity;
    }
};
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedCollisionMatrix042Test,
    "IAmSpeed.PhysicalLaws.CollisionMatrix042.FastBatch", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedCollisionMatrix042Test::RunTest(const FString&)
{
    FMatrixCVar Canonical(TEXT("p.IAmSpeed.CanonicalFrame.Enabled"), 1);
    FMatrixCVar Fast(TEXT("p.IAmSpeed.FastSimulation.Enabled"), 1);
    FMatrixCVar Host(TEXT("p.IAmSpeed.Simulation.ExecutionMode"), 2);
    FMatrixCVar Delay(TEXT("p.IAmSpeed.FastSimulation.ReadyDelayCallbacks"), 0);
    FMatrixCVar Budget(TEXT("p.IAmSpeed.FastSimulation.MaxFramesPerRun"), 128);
    FMatrixCVar Backend(TEXT("p.IAmSpeed.StaticCollision.Backend"), 2);
    const auto Options = UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(false)
        .RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false).ShouldSimulatePhysics(false).SetTransactional(false);
    UWorld* World = UWorld::CreateWorld(EWorldType::Game, false, NAME_None, nullptr, true, ERHIFeatureLevel::Num, &Options);
    if (!TestNotNull(TEXT("one world for the entire matrix"), World)) return false;
    GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
    ON_SCOPE_EXIT { World->DestroyWorld(false); GEngine->DestroyWorldContext(World); };
    auto* Driver = World->SpawnActor<AFastSimulation>();
    auto* Bridge = World->GetSubsystem<USpeedWorldSubsystem>();
    if (!Driver || !Bridge) return false;
    Driver->SpeedWorldSubsystem = Bridge;
    Driver->ActiveExecutionModeValue.Store(uint8(ESimulationExecutionMode::IAmSpeedThread));
    Driver->InitializeCanonicalFrame(0);
    Driver->SealSimulationInputs();
    uint32 DriversCreated=1;
    bool ReplaceDriverAtNextCase=false;

    auto Run = [&](const FMatrixCase& C, FMatrixResult& Out) -> bool
    {
        // The previous owner has joined before this GT-only replacement boundary.
        if (!Bridge->SimulationWorld.ResetEmptyCollisionMatrixEpisodeForTesting()) return false;
        if (ReplaceDriverAtNextCase)
        {
            // A production terminal latch is never cleared. The failed case and
            // its uncommitted frame stay failed; a fresh test driver starts the
            // independent next episode in this SAME world, with no fake receipt.
            const uint64 NextUnpublishedFrame=Driver->CanonicalNumFrame;
            Driver->Destroy();
            Driver=World->SpawnActor<AFastSimulation>();
            if (!Driver) return false;
            Driver->SpeedWorldSubsystem=Bridge;
            Driver->ActiveExecutionModeValue.Store(uint8(ESimulationExecutionMode::IAmSpeedThread));
            Driver->InitializeCanonicalFrame(0);
            Driver->CanonicalNumFrame=NextUnpublishedFrame;
            Driver->SealSimulationInputs();
            ++DriversCreated; ReplaceDriverAtNextCase=false;
        }
        Out.DriverGeneration=DriversCreated;
        Bridge->AnalyticSourceComponents.Reset(); Bridge->AnalyticWorldData.Reset();
        Driver->CollisionMatrixTestFirstFrame = Driver->CanonicalNumFrame;
        Driver->CollisionMatrixTestHz = C.Hz;
        Driver->bRunMetricsActive = false;
        Out.FirstGlobalFrame=Driver->CanonicalNumFrame; Out.WorldId=World->GetUniqueID();
        FMatrixBody PlainA, B;
        TUniquePtr<FMatrixWheelOwner> Wheel;
        if (C.A==EMatrixShape::Wheel) Wheel=MakeUnique<FMatrixWheelOwner>();
        FMatrixBody& A = Wheel ? Wheel->Core : PlainA;
        ISpeedComponent* AComponent = Wheel ? static_cast<ISpeedComponent*>(Wheel.Get()) : &PlainA;
        A.Controller = true;
        A.PublicationObserver = [&](uint64) {
            const auto& D=Bridge->GetLastStepDiagnostics();
            Out.IterationExhausted |= D.bIterationLimitReached;
            Out.MaximumDeltaErrorMs=FMath::Max(Out.MaximumDeltaErrorMs,
                FMath::Abs(D.PhysicalDeltaTimeMilliseconds - double(1.f/float(C.Hz))*1000.));
            Out.ActualDeltaValid &= Out.MaximumDeltaErrorMs<1.e-5;
        };
        A.Hz = B.Hz = C.Hz; A.FirstFrame = B.FirstFrame = Driver->CanonicalNumFrame;
        A.Limit = B.Limit = C.Hz / 20;
        auto* ActorA = World->SpawnActor<AActor>(); auto* ActorB = World->SpawnActor<AActor>();
        if (!ActorA || !ActorB) return false;
        if (Wheel) Wheel->Initialize(*ActorA); else A.Initialize(*ActorA, C.A);
        if (Wheel) Wheel->SupportObserver=[&](const SHitResult& Hit,const SKinematic& Before,const SKinematic& After)
        {
            const FVector Point=Hit.ImpactPoint-C.Offset;
            ++Out.SupportSamples;
            Out.RawSupportSamples.Add(FString::Printf(TEXT("%u,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%s,%s\n"),
                A.Published,double(A.Published+1)*double(1.f/float(C.Hz)),double(Hit.TOI),
                Point.X,Point.Y,Point.Z,Hit.ImpactNormal.X,Hit.ImpactNormal.Y,Hit.ImpactNormal.Z,
                *KinematicCsv(Before),*KinematicCsv(After)));
        };
        FString BindingError;
        if (!AComponent->ValidateSimulationBindings(BindingError)) { AddError(BindingError); return false; }
        const bool DynamicPair = C.B == EMatrixShape::Sphere || C.B == EMatrixShape::Box || C.B == EMatrixShape::Cylinder;
        if (DynamicPair) B.Initialize(*ActorB, C.B);
        ON_SCOPE_EXIT {
            Bridge->UnregisterSpeedComponent(AComponent);
            if (DynamicPair) Bridge->UnregisterSpeedComponent(&B);
            Bridge->ApplyPendingOps();
            ActorA->Destroy(); ActorB->Destroy();
        };
        const double Speed = C.Variant == 1 ? 300. : C.Variant == 2 ? 4600. : 1200.;
        const double Spin = C.Variant == 3 ? 1. : C.Variant == 4 ? -3. : 0.;
        const double Tangent = C.Variant == 5 ? 80. : 0.;
        const double OffsetY = C.Variant == 6 ? 3. : 0.;
        A.State.Rotation = C.Variant == 3 || C.Variant == 4 ? FQuat(FVector::YAxisVector, .17)
            : C.Variant == 7 ? FQuat(FVector::ZAxisVector, .37) : FQuat::Identity;
        A.State.AngularVelocity = FVector(0, Spin, 0);
        const double SupportA = C.A == EMatrixShape::Box ? 20. : 12.5;
        auto Data = MakeShared<Analytic::FAnalyticWorldData>();
        if (C.B == EMatrixShape::Plane) {
            const double Height = C.A == EMatrixShape::Box ? 8. : 12.5;
            A.State.Location = C.Offset + FVector(OffsetY, 0, Height + Speed * .021);
            A.State.Velocity = FVector(Tangent, 0, -Speed);
            Analytic::FBoundedPlane Plane;
            Plane.SourceId = Plane.SurfaceId = Plane.FeatureId = Plane.PrimitiveId = 1;
            Plane.ObjectType = uint32(ECC_WorldStatic); Plane.BlockingChannels = MAX_uint64;
            // Authored finite fixture geometry is the selected analytic authority.
            // FBoundedPlane defaults to diagnostic-only, which cannot block CCD.
            Plane.bAuthorityEligible = true;
            Plane.Origin = FVector3d(C.Offset); Plane.HalfExtents = FVector2d(1000, 1000);
            Data->Planes.Add(Plane);
        } else if (C.B == EMatrixShape::Ring) {
            auto* Ring = NewObject<URingSubBody>(ActorB); ActorB->AddInstanceComponent(Ring);
            Ring->MajorRadius = 60.; Ring->TubeRadius = 10.;
            Ring->SetWorldLocation(C.Offset); Ring->AnalyticIdentity = TEXT("CollisionMatrix042Ring");
            Analytic::FTorusSurface Torus; FString Error;
            if (!Ring->BuildStaticTorus(1, Torus, Error)) return false;
            Data->Tori.Add(Torus);
            A.State.Location = C.Offset + FVector(10. + SupportA + Speed * .021, 60. + OffsetY, 0);
            A.State.Velocity = FVector(-Speed, Tangent, 0);
        } else {
            const double SupportB = C.B == EMatrixShape::Sphere ? 12.5 : C.B == EMatrixShape::Box ? 20. : 25.;
            A.State.Location = C.Offset + FVector(-SupportA - SupportB - Speed * .021, OffsetY, 0);
            A.State.Velocity = FVector(Speed, Tangent, 0); B.State.Location = C.Offset;
            // A valid static authority is still supplied for body/body cases.
            // This remote floor is outside the finite episode's swept bounds.
            Analytic::FBoundedPlane Plane;
            Plane.SourceId = Plane.SurfaceId = Plane.FeatureId = Plane.PrimitiveId = 1;
            Plane.ObjectType = uint32(ECC_WorldStatic); Plane.BlockingChannels = MAX_uint64;
            Plane.bAuthorityEligible = true;
            Plane.Origin = FVector3d(C.Offset + FVector(0,0,-10000));
            Plane.HalfExtents = FVector2d(1000,1000); Data->Planes.Add(Plane);
        }
        {
            FString Error;
            if (!Data->FinalizeAndValidate(&Error)) { AddError(Error); return false; }
            auto* Source = NewObject<UBoxComponent>(ActorB); ActorB->AddInstanceComponent(Source);
            Source->SetMobility(EComponentMobility::Static);
            Source->SetCollisionObjectType(ECC_WorldStatic); Source->SetCollisionResponseToAllChannels(ECR_Block);
            Source->SetWorldLocation(C.Offset + (DynamicPair ? FVector(0,0,-10000) : FVector::ZeroVector));
            Bridge->AnalyticSourceComponents.Add(1, Source);
            Bridge->AnalyticWorldData = Data;
            Bridge->SimulationWorld.SetStaticCollisionWorld(MakeUnique<FAnalyticStaticCollisionWorld>(*Data, 1, 1, 1));
        }
        Bridge->RegisterSpeedComponent(AComponent);
        if (DynamicPair) Bridge->RegisterSpeedComponent(&B);
        Bridge->ApplyPendingOps();
        if (!Driver->EnsureSimulationWorldReady()) return false;
        Out.InitialA=A.State; Out.InitialB=B.State;
        if (!DynamicPair) Out.InitialB.Location=C.Offset;
        Out.MassA=A.Config.Mass; Out.MassB=DynamicPair ? B.Config.Mass : 0.;
        Out.EnvironmentOrigin=Data->Planes.Num() ? FVector(Data->Planes[0].Origin) : Data->Tori[0].Pose.GetLocation();
        ESimulationWorkerResult Final = ESimulationWorkerResult::Idle;
        FMatrixObserver Observer(AComponent, A.FirstFrame, 1.f / float(C.Hz), C.Offset, Out);
        FSimulationWorker Owner([&] {
            auto* Previous = CollisionMatrix042::FObserver::Exchange(&Observer);
            ON_SCOPE_EXIT { CollisionMatrix042::FObserver::Exchange(Previous); };
            Final = Driver->DriveSimulation(1.f / float(C.Hz), 0.f);
            return Final;
        }, [](FSimulationWorkerWaitContext&) {});
        if (!Owner.Start()) return false;
        const double End = FPlatformTime::Seconds() + 10.;
        while (Owner.IsRunning() && FPlatformTime::Seconds() < End) FPlatformProcess::SleepNoStats(.0001f);
        const bool Finished = !Owner.IsRunning(); Owner.StopAndJoin();
        Out.Complete = Finished && Final == ESimulationWorkerResult::Complete && A.Published == A.Limit;
        Out.TerminalFailure=Driver->bCanonicalPublicationTerminal.Load() || Driver->bOwnedWorkerTerminal.Load();
        Out.ClockValid = A.ClockValid && (!DynamicPair || B.ClockValid);
        Out.Frames = A.Published; Out.FinalA = A.State; Out.FinalB = B.State;
        if (!DynamicPair) Out.FinalB.Location=C.Offset;
        Out.FinalA.Location -= C.Offset; Out.FinalB.Location -= C.Offset;
        const bool Valid=Out.Complete && Out.ClockValid && Out.ActualDeltaValid && !Out.Overflow && !Out.IterationExhausted;
        ReplaceDriverAtNextCase=!Valid;
        return Valid;
    };

    if (FParse::Param(FCommandLine::Get(), TEXT("IAmSpeedCollisionMatrix042Smoke"))
        || FParse::Param(FCommandLine::Get(), TEXT("IAmSpeedCollisionMatrix042WheelSmoke")))
    {
        FMatrixResult R; FMatrixCase C;
        if (FParse::Param(FCommandLine::Get(), TEXT("IAmSpeedCollisionMatrix042WheelSmoke"))) C.A=EMatrixShape::Wheel;
        TestTrue(TEXT("real native FastBatch smoke completed"), Run(C,R));
        TestTrue(TEXT("real native analytic collision selected"), R.HasHit);
        // Exercise driver replacement after join without clearing any latch.
        ReplaceDriverAtNextCase=true;
        FMatrixResult Shifted; C.Offset=FVector(1000,-2000,3000);
        TestTrue(TEXT("joined second case in the same world completed"), Run(C,Shifted));
        TestTrue(TEXT("translated native analytic collision selected"),Shifted.HasHit);
        TestTrue(TEXT("on-grid smoke impact point is translation invariant"),
            R.HasHit && Shifted.HasHit && FVector::Distance(R.Point,Shifted.Point)<=.0001);
        TestTrue(TEXT("on-grid smoke TOI is translation invariant"),
            R.HasHit && Shifted.HasHit && FMath::Abs(R.Time-Shifted.Time)<=1.e-6);
        AddInfo(FString::Printf(TEXT("[CollisionMatrix042Smoke] Worlds=1 Cases=2 Frames=%u Hits=%u Time=%.17g ShiftHits=%u ShiftTime=%.17g InitialA=%s FinalA=%s ShiftFinalA=%s Drivers=%u FullMatrix=0"),
            R.Frames+Shifted.Frames,R.Events,R.Time,Shifted.Events,Shifted.Time,
            *KinematicCsv(R.InitialA),*KinematicCsv(R.FinalA),*KinematicCsv(Shifted.FinalA),DriversCreated));
        return !HasAnyErrors();
    }
    // This sparse design varies each declared axis; it is not the Cartesian
    // product of all velocities, orientations, offsets and angular velocities.
    FString Output;
    if (!FParse::Value(FCommandLine::Get(), TEXT("IAmSpeedCollisionMatrix042Output="), Output))
        Output = FPaths::ProjectSavedDir() / TEXT("CollisionMatrix042") / FGuid::NewGuid().ToString(EGuidFormats::Digits);
    if (IFileManager::Get().DirectoryExists(*Output)) {
        AddError(TEXT("Matrix output already exists; no previous evidence is overwritten")); return false;
    }
    IFileManager::Get().MakeDirectory(*Output, true);
    const FString CasesFile = Output / TEXT("cases.csv"), EventsFile = Output / TEXT("events.csv"), EndFile=Output/TEXT("end-states.csv"), InitialFile=Output/TEXT("initial-states.csv");
    const FString SupportFile=Output/TEXT("support-samples.csv");
    FFileHelper::SaveStringToFile(TEXT("case,local_frame,sample_seconds,query_parameter,point_x,point_y,point_z,normal_x,normal_y,normal_z,before_x,before_y,before_z,before_qx,before_qy,before_qz,before_qw,before_vx,before_vy,before_vz,before_wx,before_wy,before_wz,after_x,after_y,after_z,after_qx,after_qy,after_qz,after_qw,after_vx,after_vy,after_vz,after_wx,after_wy,after_wz\n"),*SupportFile);
    FFileHelper::SaveStringToFile(TEXT("case,a_x,a_y,a_z,a_qx,a_qy,a_qz,a_qw,a_vx,a_vy,a_vz,a_wx,a_wy,a_wz,b_x,b_y,b_z,b_qx,b_qy,b_qz,b_qw,b_vx,b_vy,b_vz,b_wx,b_wy,b_wz,environment_x,environment_y,environment_z,mass_a,mass_b\n"),*InitialFile);
    FFileHelper::SaveStringToFile(TEXT("case,shape_a,shape_b,variant,hz,translation,complete,clock_valid,hits,impact_seconds,point_x,point_y,point_z,normal_x,normal_y,normal_z,frequency_pass,translation_pass,oracle_pass,point_bound_cm,time_bound_s,frequency_point_error_cm,frequency_time_error_s,translation_point_error_cm,first_global_frame,frames,world_id,actual_delta_valid,iteration_exhausted,observer_overflow,real_wheel_hit,max_delta_error_ms,driver_generation,terminal_failure\n"), *CasesFile);
    FFileHelper::SaveStringToFile(TEXT("case,sample_seconds,position_a_x,position_a_y,position_a_z,velocity_a_x,velocity_a_y,velocity_a_z,angular_a_x,angular_a_y,angular_a_z,position_b_x,position_b_y,position_b_z,velocity_b_x,velocity_b_y,velocity_b_z,angular_b_x,angular_b_y,angular_b_z\n"), *EndFile);
    FFileHelper::SaveStringToFile(TEXT("case,local_frame,elapsed_in_frame,query_toi,point_x,point_y,point_z,normal_x,normal_y,normal_z,feature_a,feature_b,velocity_a_x,velocity_a_y,velocity_a_z,angular_a_x,angular_a_y,angular_a_z,velocity_b_x,velocity_b_y,velocity_b_z,angular_b_x,angular_b_y,angular_b_z\n"), *EventsFile);
    const TArray<uint32> Rates = {1440, 720, 360, 300, 240, 120, 100, 60};
    const TArray<FVector> Translations = { FVector::ZeroVector, FVector(1000,-2000,3000),
        FVector(-1048576,2097152,-524288), FVector(100000000,-100000000,100000000), FVector(.003,-.004,.001) };
    uint32 CaseCount = 0, Failed = 0, FrequencyFailures = 0, TranslationFailures = 0, OracleFailures = 0;
    uint64 FrameCount = 0;
    uint32 WheelCases=0,WheelHitCases=0,IterationCases=0,OverflowCases=0;
    uint32 CompletedCases=0,TerminalFailureCases=0;
    const double Start = FPlatformTime::Seconds();
    bool Incomplete = false, ExportValid = true;
    // Frozen pre-run numerical gates. 0.005 cm is half the canonical position
    // grid. Per-frame accumulation is charged separately from CCD geometry.
    const double GeometricCm = .02, NormalRadians = .002, TranslationCm = .0001;
    FString RateJson,TranslationJson;
    for (uint32 H : Rates) { if (!RateJson.IsEmpty()) RateJson+=TEXT(","); RateJson+=FString::FromInt(int32(H)); }
    for (const FVector& T : Translations) {
        if (!TranslationJson.IsEmpty()) TranslationJson+=TEXT(",");
        TranslationJson+=FString::Printf(TEXT("[%.17g,%.17g,%.17g]"),T.X,T.Y,T.Z);
    }
    const FString Spec=FString::Printf(TEXT("{\"schema\":1,\"rates_hz\":[%s],\"translations_cm\":[%s],\"shape_a\":[\"Sphere\",\"Box\",\"Wheel\"],\"shape_b\":[\"AnalyticPlane\",\"RingStatic\",\"Sphere\",\"Box\",\"CylinderPrescribed\"],\"variants\":8,\"duration_seconds\":0.05,\"reference_hz\":1440,\"position_grid_cm\":0.01,\"geometric_cm\":%.17g,\"normal_radians\":%.17g,\"translation_cm\":%.17g,\"translation_time_s\":0.000001,\"translation_normal_radians\":0.000002,\"translation_velocity_cm_s\":0.01,\"translation_angular_rad_s\":0.001,\"gravity_cm_s2\":[0,0,0],\"native_wheel_model\":\"USWheelSubBody plus real wheel and suspension storage with native ISpeedWheeledComponent solve; unit fixture, not whole car\",\"scope\":\"first impact and common50ms post state; sparse scenario design, not full Cartesian velocity product; no crossrate rollback or production multirate qualification\"}\n"),
        *RateJson,*TranslationJson,GeometricCm,NormalRadians,TranslationCm);
    ExportValid &= FFileHelper::SaveStringToFile(Spec,*(Output/TEXT("spec.json")));
    auto NormalError = [](const FVector& A, const FVector& B) {
        return FMath::Acos(FMath::Clamp(FVector::DotProduct(A.GetSafeNormal(), B.GetSafeNormal()), -1., 1.));
    };
    for (EMatrixShape ShapeA : {EMatrixShape::Sphere, EMatrixShape::Box, EMatrixShape::Wheel})
    for (EMatrixShape ShapeB : {EMatrixShape::Plane, EMatrixShape::Ring, EMatrixShape::Sphere, EMatrixShape::Box, EMatrixShape::Cylinder})
    for (int32 Variant = 0; Variant < 8; ++Variant)
    {
        TArray<FMatrixResult> Fine; Fine.SetNum(Translations.Num());
        for (uint32 Hz : Rates)
        {
            FMatrixResult Origin;
            for (int32 T = 0; T < Translations.Num(); ++T)
            {
                if (FPlatformTime::Seconds() - Start > 60.) { Incomplete = true; break; }
                FMatrixCase C; C.A = ShapeA; C.B = ShapeB; C.Variant = Variant; C.Hz = Hz; C.Offset = Translations[T];
                FMatrixResult R;
                const bool Valid = Run(C, R);
                ++CaseCount; FrameCount += R.Frames;
                CompletedCases+=R.Complete; TerminalFailureCases+=R.TerminalFailure;
                if (T == 0) Origin = R;
                if (Hz == Rates[0]) Fine[T] = R;
                const auto& Ref = Fine[T];
                const double Speed = Variant == 1 ? 300. : Variant == 2 ? 4600. : 1200.;
                const double PointBound = GeometricCm + FMath::Sqrt(3.) * .005 * double(R.Frames + Ref.Frames);
                const double TimeBound = 1.e-6 + PointBound / (Speed - 75.);
                const double VelocityBound = .02 + NormalRadians * Speed;
                const double AngularBound = .01 + NormalRadians * Speed / 12.5;
                const double PointError = FVector::Distance(R.Point, Ref.Point);
                const double TimeError = FMath::Abs(R.Time - Ref.Time);
                const bool Presence = Valid && R.HasHit && Ref.Complete && Ref.ClockValid && Ref.HasHit;
                const bool FrequencyPass = Presence && PointError <= PointBound && TimeError <= TimeBound
                    && R.FeatureA == Ref.FeatureA && R.FeatureB == Ref.FeatureB
                    && NormalError(R.Normal, Ref.Normal) <= NormalRadians
                    && FVector::Distance(R.VelocityA, Ref.VelocityA) <= VelocityBound
                    && FVector::Distance(R.VelocityB, Ref.VelocityB) <= VelocityBound
                    && FVector::Distance(R.AngularA, Ref.AngularA) <= AngularBound
                    && FVector::Distance(R.AngularB, Ref.AngularB) <= AngularBound
                    && FVector::Distance(R.FinalA.Velocity, Ref.FinalA.Velocity) <= VelocityBound
                    && FVector::Distance(R.FinalB.Velocity, Ref.FinalB.Velocity) <= VelocityBound
                    && FVector::Distance(R.FinalA.AngularVelocity, Ref.FinalA.AngularVelocity) <= AngularBound
                    && FVector::Distance(R.FinalB.AngularVelocity, Ref.FinalB.AngularVelocity) <= AngularBound;
                // Fractional-grid translations are subjected to the same strict
                // invariance gate; a quantization phase failure stays a failure.
                const double TranslationPointError = FVector::Distance(R.Point, Origin.Point);
                const bool TranslationPass = Valid && R.HasHit && Origin.HasHit
                    && R.FeatureA == Origin.FeatureA && R.FeatureB == Origin.FeatureB
                    && TranslationPointError <= TranslationCm && FMath::Abs(R.Time - Origin.Time) <= 1.e-6
                    && NormalError(R.Normal, Origin.Normal) <= 2.e-6
                    && FVector::Distance(R.VelocityA, Origin.VelocityA) <= .01
                    && FVector::Distance(R.VelocityB, Origin.VelocityB) <= .01
                    && FVector::Distance(R.AngularA, Origin.AngularA) <= .001
                    && FVector::Distance(R.AngularB, Origin.AngularB) <= .001
                    && FVector::Distance(R.FinalA.Velocity, Origin.FinalA.Velocity) <= .01
                    && FVector::Distance(R.FinalB.Velocity, Origin.FinalB.Velocity) <= .01
                    && FVector::Distance(R.FinalA.AngularVelocity, Origin.FinalA.AngularVelocity) <= .001
                    && FVector::Distance(R.FinalB.AngularVelocity, Origin.FinalB.AngularVelocity) <= .001;
                const bool HasOracle = ShapeA == EMatrixShape::Sphere && ShapeB == EMatrixShape::Plane
                    && Variant != 3 && Variant != 4;
                const double OraclePointBound = GeometricCm + .005 * R.Frames;
                const bool OraclePass = !HasOracle || (R.HasHit && FMath::Abs(R.Time - .021) <= 1.e-6 + OraclePointBound / Speed
                    && FMath::Abs(R.Point.Z) <= GeometricCm
                    && NormalError(R.Normal, FVector::UpVector) <= 1.e-6
                    && FMath::Abs(R.VelocityA.Z) <= .02);
                FrequencyFailures += !FrequencyPass; TranslationFailures += !TranslationPass; OracleFailures += !OraclePass;
                const bool Pass = FrequencyPass && TranslationPass && OraclePass;
                Failed += !Pass;
                WheelCases += ShapeA==EMatrixShape::Wheel;
                WheelHitCases += ShapeA==EMatrixShape::Wheel && R.RealWheelHit;
                IterationCases += R.IterationExhausted; OverflowCases += R.Overflow;
                if (!Pass && Failed <= 20) AddError(FString::Printf(TEXT("CollisionMatrix042 case=%u %s/%s variant=%d Hz=%u offset=%d native=%d frequency=%d translation=%d oracle=%d"),
                    CaseCount, Name(ShapeA), Name(ShapeB), Variant, Hz, T, Valid, FrequencyPass, TranslationPass, OraclePass));
                const FString Row = FString::Printf(TEXT("%u,%s,%s,%d,%u,%d,%d,%d,%u,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%d,%d,%d,%.17g,%.17g,%.17g,%.17g,%.17g,%llu,%u,%u,%d,%d,%d,%d,%.17g,%u,%d\n"),
                    CaseCount, Name(ShapeA), Name(ShapeB), Variant, Hz, T, R.Complete, R.ClockValid, R.Events,
                    R.Time, R.Point.X, R.Point.Y, R.Point.Z, R.Normal.X, R.Normal.Y, R.Normal.Z,
                    FrequencyPass, TranslationPass, OraclePass, PointBound, TimeBound, PointError, TimeError, TranslationPointError,
                    R.FirstGlobalFrame,R.Frames,R.WorldId,R.ActualDeltaValid,R.IterationExhausted,R.Overflow,R.RealWheelHit,R.MaximumDeltaErrorMs,R.DriverGeneration,R.TerminalFailure);
                ExportValid &= FFileHelper::SaveStringToFile(Row, *CasesFile, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM, &IFileManager::Get(), FILEWRITE_Append);
                const FString InitialRow=FString::Printf(TEXT("%u,%s,%s,%.17g,%.17g,%.17g,%.17g,%.17g\n"),CaseCount,
                    *KinematicCsv(R.InitialA),*KinematicCsv(R.InitialB),R.EnvironmentOrigin.X,R.EnvironmentOrigin.Y,R.EnvironmentOrigin.Z,R.MassA,R.MassB);
                ExportValid &= FFileHelper::SaveStringToFile(InitialRow,*InitialFile,FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM,&IFileManager::Get(),FILEWRITE_Append);
                const FString EndRow=FString::Printf(TEXT("%u,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g\n"),
                    CaseCount,double(R.Frames)/double(Hz),R.FinalA.Location.X,R.FinalA.Location.Y,R.FinalA.Location.Z,
                    R.FinalA.Velocity.X,R.FinalA.Velocity.Y,R.FinalA.Velocity.Z,R.FinalA.AngularVelocity.X,R.FinalA.AngularVelocity.Y,R.FinalA.AngularVelocity.Z,
                    R.FinalB.Location.X,R.FinalB.Location.Y,R.FinalB.Location.Z,R.FinalB.Velocity.X,R.FinalB.Velocity.Y,R.FinalB.Velocity.Z,
                    R.FinalB.AngularVelocity.X,R.FinalB.AngularVelocity.Y,R.FinalB.AngularVelocity.Z);
                ExportValid &= FFileHelper::SaveStringToFile(EndRow,*EndFile,FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM,&IFileManager::Get(),FILEWRITE_Append);
                FString Events;
                for (const auto& Event : R.RawEvents) Events += FString::Printf(TEXT("%u,"), CaseCount) + Event;
                ExportValid &= FFileHelper::SaveStringToFile(Events, *EventsFile, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM, &IFileManager::Get(), FILEWRITE_Append);
                FString Supports;
                for (const auto& Sample:R.RawSupportSamples) Supports+=FString::Printf(TEXT("%u,"),CaseCount)+Sample;
                ExportValid &= FFileHelper::SaveStringToFile(Supports,*SupportFile,FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM,&IFileManager::Get(),FILEWRITE_Append);
                if (CaseCount % 128 == 0) CollectGarbage(RF_NoFlags);
            }
            if (Incomplete) break;
        }
        if (Incomplete) break;
    }
    const uint32 Expected = 3 * 5 * 8 * Rates.Num() * Translations.Num();
    uint64 ExpectedFrames=0;
    for(uint32 H:Rates)ExpectedFrames+=uint64(3*5*8*Translations.Num())*(H/20);
    Incomplete|=CompletedCases!=Expected || FrameCount!=ExpectedFrames;
    const FString Summary = FString::Printf(TEXT("{\"schema\":1,\"worlds_created_by_harness\":1,\"driver\":\"AFastSimulation\",\"fast_batch\":true,\"highest_hz\":1440,\"expected_cases\":%u,\"actual_cases\":%u,\"frames\":%llu,\"failed\":%u,\"frequency_failures\":%u,\"translation_failures\":%u,\"oracle_failures\":%u,\"incomplete\":%s,\"export_valid\":%s,\"wheel_coverage\":%s,\"wheel_cases\":%u,\"wheel_hit_cases\":%u,\"iteration_limit_cases\":%u,\"observer_overflow_cases\":%u}\n"),
        Expected, CaseCount, FrameCount, Failed, FrequencyFailures, TranslationFailures, OracleFailures,
        Incomplete ? TEXT("true") : TEXT("false"), ExportValid ? TEXT("true") : TEXT("false"),
        WheelCases>0 && WheelHitCases==WheelCases ? TEXT("true") : TEXT("false"),WheelCases,WheelHitCases,IterationCases,OverflowCases);
    ExportValid &= FFileHelper::SaveStringToFile(Summary, *(Output / TEXT("summary.json")));
    const FString Execution=FString::Printf(TEXT("{\"completed_cases\":%u,\"attempted_cases\":%u,\"expected_frames\":%llu,\"actual_frames\":%llu,\"drivers_created\":%u,\"terminal_failure_cases\":%u,\"worlds_created\":1,\"production_terminal_latches_cleared\":false}\n"),
        CompletedCases,CaseCount,ExpectedFrames,FrameCount,DriversCreated,TerminalFailureCases);
    ExportValid &= FFileHelper::SaveStringToFile(Execution,*(Output/TEXT("execution.json")));
    TestTrue(TEXT("matrix completed every declared case"), !Incomplete && CaseCount == Expected);
    TestTrue(TEXT("all matrix evidence saved"), ExportValid);
    TestEqual(TEXT("all collision and invariance gates"), Failed, uint32(0));
    TestTrue(TEXT("actual native Wheel contacts covered every Wheel case"),WheelCases>0 && WheelHitCases==WheelCases);
    AddInfo(FString::Printf(TEXT("[CollisionMatrix042] Worlds=1 Cases=%u/%u Frames=%llu Failures=%u Evidence=%s WheelHits=%u/%u"),
        CaseCount, Expected, FrameCount, Failed, *Output,WheelHitCases,WheelCases));
    return !HasAnyErrors();
}
#endif
