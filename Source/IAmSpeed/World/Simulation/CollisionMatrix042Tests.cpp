#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "FastSimulation.h"
#include "CanonicalFrameContext.h"
#include "CollisionMatrixObservationForTesting.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "IAmSpeed/World/Collision/StaticCollisionWorld.h"
#include "IAmSpeed/SubBodies/Solid/SphereSubBody.h"
#include "IAmSpeed/SubBodies/Solid/BoxSubBody.h"
#include "IAmSpeed/SubBodies/Solid/CylinderSubBody.h"
#include "IAmSpeed/SubBodies/Solid/RingSubBody.h"
#include "IAmSpeed/SubBodies/Configs/SubBodyConfig.h"
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
enum class EMatrixShape : uint8 { Sphere, Box, Cylinder, Plane, Ring };
const TCHAR* Name(EMatrixShape S)
{
    switch (S) {
    case EMatrixShape::Sphere: return TEXT("Sphere");
    case EMatrixShape::Box: return TEXT("Box");
    case EMatrixShape::Cylinder: return TEXT("CylinderPrescribed");
    case EMatrixShape::Plane: return TEXT("AnalyticPlane");
    default: return TEXT("RingStatic");
    }
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
    { ClockValid &= N == FirstFrame + Published; ++Published; }
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
    { return B.GetKinematicsFromOwnerKS(State); }
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
    void SetKinematicState(const SKinematic& S) override { State = S; }
    float GetPhysMaxSpeed() const override { return 20000; }
    float GetPhysMaxAngularSpeed() const override { return 100; }
    FVector GetNominalGravityAcceleration() const override { return FVector::ZeroVector; }
    void SetIsUpsideDown(bool) override {}
    bool IsUpsideDown() const override { return false; }
    bool IsInAutoRecover() const override { return false; }
    bool IsSubBodyInAutoRecoverMode() const override { return false; }
    void RcvImpactOnSubBody(const USSubBody&, const FVector&) override {}
    void PostPhysicsUpdatePrv(const float&) override
    { QuantizeKinematicState(State, Previous); Previous = State.Rotation; }
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
    double Time = 0; FVector Point = FVector::ZeroVector, Normal = FVector::ZeroVector;
    FVector VelocityA, AngularA, VelocityB, AngularB;
    SKinematic FinalA, FinalB;
    EContactFeatureKind FeatureA = EContactFeatureKind::Unknown, FeatureB = EContactFeatureKind::Unknown;
    TArray<FString> RawEvents;
};
class FMatrixObserver final : public CollisionMatrix042::FObserver
{
public:
    const FMatrixBody* A; uint64 FirstFrame; float Delta; FVector Offset;
    FMatrixResult& Result;
    FMatrixObserver(const FMatrixBody* InA, uint64 First, float Dt, const FVector& O, FMatrixResult& R)
        : A(InA), FirstFrame(First), Delta(Dt), Offset(O), Result(R) {}
    void Resolved(const CollisionMatrix042::FResolution& R) override
    {
        if (R.Owner != A && R.Other != A) return;
        if (!R.Hit.bHit || !R.Hit.bBlockingHit) return;
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

    auto Run = [&](const FMatrixCase& C, FMatrixResult& Out) -> bool
    {
        // The previous owner has joined before this GT-only replacement boundary.
        if (!Bridge->SimulationWorld.ResetEmptyCollisionMatrixEpisodeForTesting()) return false;
        Bridge->AnalyticSourceComponents.Reset(); Bridge->AnalyticWorldData.Reset();
        Driver->CollisionMatrixTestFirstFrame = Driver->CanonicalNumFrame;
        Driver->CollisionMatrixTestHz = C.Hz;
        Driver->bRunMetricsActive = false;
        FMatrixBody A, B; A.Controller = true;
        A.Hz = B.Hz = C.Hz; A.FirstFrame = B.FirstFrame = Driver->CanonicalNumFrame;
        A.Limit = B.Limit = C.Hz / 20;
        auto* ActorA = World->SpawnActor<AActor>(); auto* ActorB = World->SpawnActor<AActor>();
        if (!ActorA || !ActorB) return false;
        A.Initialize(*ActorA, C.A);
        const bool DynamicPair = C.B == EMatrixShape::Sphere || C.B == EMatrixShape::Box || C.B == EMatrixShape::Cylinder;
        if (DynamicPair) B.Initialize(*ActorB, C.B);
        ON_SCOPE_EXIT {
            Bridge->UnregisterSpeedComponent(&A);
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
        const double SupportA = C.A == EMatrixShape::Sphere ? 12.5 : 20.;
        auto Data = MakeShared<Analytic::FAnalyticWorldData>();
        if (C.B == EMatrixShape::Plane) {
            const double Height = C.A == EMatrixShape::Sphere ? 12.5 : 8.;
            A.State.Location = C.Offset + FVector(OffsetY, 0, Height + Speed * .021);
            A.State.Velocity = FVector(Tangent, 0, -Speed);
            Analytic::FBoundedPlane Plane;
            Plane.SourceId = Plane.SurfaceId = Plane.FeatureId = Plane.PrimitiveId = 1;
            Plane.ObjectType = uint32(ECC_WorldStatic); Plane.BlockingChannels = MAX_uint64;
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
            Plane.Origin = FVector3d(C.Offset + FVector(0,0,-10000));
            Plane.HalfExtents = FVector2d(1000,1000); Data->Planes.Add(Plane);
        }
        {
            FString Error;
            if (!Data->FinalizeAndValidate(&Error)) { AddError(Error); return false; }
            auto* Source = NewObject<UBoxComponent>(ActorB); ActorB->AddInstanceComponent(Source);
            Source->SetMobility(EComponentMobility::Static);
            Source->SetCollisionObjectType(ECC_WorldStatic); Source->SetCollisionResponseToAllChannels(ECR_Block);
            Bridge->AnalyticSourceComponents.Add(1, Source);
            Bridge->AnalyticWorldData = Data;
            Bridge->SimulationWorld.SetStaticCollisionWorld(MakeUnique<FAnalyticStaticCollisionWorld>(*Data, 1, 1, 1));
        }
        Bridge->RegisterSpeedComponent(&A);
        if (DynamicPair) Bridge->RegisterSpeedComponent(&B);
        Bridge->ApplyPendingOps();
        if (!Driver->EnsureSimulationWorldReady()) return false;
        ESimulationWorkerResult Final = ESimulationWorkerResult::Idle;
        FMatrixObserver Observer(&A, A.FirstFrame, 1.f / float(C.Hz), C.Offset, Out);
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
        Out.ClockValid = A.ClockValid && (!DynamicPair || B.ClockValid);
        Out.Frames = A.Published; Out.FinalA = A.State; Out.FinalB = B.State;
        Out.FinalA.Location -= C.Offset; Out.FinalB.Location -= C.Offset;
        return Out.Complete && Out.ClockValid && !Out.Overflow;
    };

    if (FParse::Param(FCommandLine::Get(), TEXT("IAmSpeedCollisionMatrix042Smoke")))
    {
        FMatrixResult R; FMatrixCase C;
        TestTrue(TEXT("real native FastBatch smoke completed"), Run(C,R));
        TestTrue(TEXT("real native analytic collision selected"), R.HasHit);
        AddInfo(FString::Printf(TEXT("[CollisionMatrix042Smoke] Worlds=1 Cases=1 Frames=%u Hits=%u Time=%.17g FullMatrix=0"), R.Frames,R.Events,R.Time));
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
    const FString CasesFile = Output / TEXT("cases.csv"), EventsFile = Output / TEXT("events.csv");
    FFileHelper::SaveStringToFile(TEXT("case,shape_a,shape_b,variant,hz,translation,complete,clock_valid,hits,impact_seconds,point_x,point_y,point_z,normal_x,normal_y,normal_z,frequency_pass,translation_pass,oracle_pass,point_bound_cm,time_bound_s,frequency_point_error_cm,frequency_time_error_s,translation_point_error_cm\n"), *CasesFile);
    FFileHelper::SaveStringToFile(TEXT("case,local_frame,elapsed_in_frame,query_toi,point_x,point_y,point_z,normal_x,normal_y,normal_z,feature_a,feature_b,velocity_a_x,velocity_a_y,velocity_a_z,angular_a_x,angular_a_y,angular_a_z,velocity_b_x,velocity_b_y,velocity_b_z,angular_b_x,angular_b_y,angular_b_z\n"), *EventsFile);
    const TArray<uint32> Rates = {1440, 720, 360, 300, 240, 120, 100, 60};
    const TArray<FVector> Translations = { FVector::ZeroVector, FVector(1000,-2000,3000),
        FVector(-1048576,2097152,-524288), FVector(100000000,-100000000,100000000), FVector(.003,-.004,.001) };
    uint32 CaseCount = 0, Failed = 0, FrequencyFailures = 0, TranslationFailures = 0, OracleFailures = 0;
    uint64 FrameCount = 0;
    const double Start = FPlatformTime::Seconds();
    bool Incomplete = false, ExportValid = true;
    // Frozen pre-run numerical gates. 0.005 cm is half the canonical position
    // grid. Per-frame accumulation is charged separately from CCD geometry.
    const double GeometricCm = .02, NormalRadians = .002, TranslationCm = .0001;
    auto NormalError = [](const FVector& A, const FVector& B) {
        return FMath::Acos(FMath::Clamp(FVector::DotProduct(A.GetSafeNormal(), B.GetSafeNormal()), -1., 1.));
    };
    for (EMatrixShape ShapeA : {EMatrixShape::Sphere, EMatrixShape::Box})
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
                if (T == 0) Origin = R;
                if (Hz == Rates[0]) Fine[T] = R;
                const auto& Ref = Fine[T];
                const double Speed = Variant == 1 ? 300. : Variant == 2 ? 4600. : 1200.;
                const double PointBound = GeometricCm + .005 * double(R.Frames + Ref.Frames);
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
                if (!Pass && Failed <= 20) AddError(FString::Printf(TEXT("CollisionMatrix042 case=%u %s/%s variant=%d Hz=%u offset=%d native=%d frequency=%d translation=%d oracle=%d"),
                    CaseCount, Name(ShapeA), Name(ShapeB), Variant, Hz, T, Valid, FrequencyPass, TranslationPass, OraclePass));
                const FString Row = FString::Printf(TEXT("%u,%s,%s,%d,%u,%d,%d,%d,%u,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%d,%d,%d,%.17g,%.17g,%.17g,%.17g,%.17g\n"),
                    CaseCount, Name(ShapeA), Name(ShapeB), Variant, Hz, T, R.Complete, R.ClockValid, R.Events,
                    R.Time, R.Point.X, R.Point.Y, R.Point.Z, R.Normal.X, R.Normal.Y, R.Normal.Z,
                    FrequencyPass, TranslationPass, OraclePass, PointBound, TimeBound, PointError, TimeError, TranslationPointError);
                ExportValid &= FFileHelper::SaveStringToFile(Row, *CasesFile, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM, &IFileManager::Get(), FILEWRITE_Append);
                FString Events;
                for (const auto& Event : R.RawEvents) Events += FString::Printf(TEXT("%u,"), CaseCount) + Event;
                ExportValid &= FFileHelper::SaveStringToFile(Events, *EventsFile, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM, &IFileManager::Get(), FILEWRITE_Append);
                if (CaseCount % 128 == 0) CollectGarbage(RF_NoFlags);
            }
            if (Incomplete) break;
        }
        if (Incomplete) break;
    }
    const uint32 Expected = 2 * 5 * 8 * Rates.Num() * Translations.Num();
    const FString Summary = FString::Printf(TEXT("{\"schema\":1,\"worlds_created_by_harness\":1,\"driver\":\"AFastSimulation\",\"fast_batch\":true,\"highest_hz\":1440,\"expected_cases\":%u,\"actual_cases\":%u,\"frames\":%llu,\"failed\":%u,\"frequency_failures\":%u,\"translation_failures\":%u,\"oracle_failures\":%u,\"incomplete\":%s,\"export_valid\":%s,\"wheel_coverage\":false}\n"),
        Expected, CaseCount, FrameCount, Failed, FrequencyFailures, TranslationFailures, OracleFailures,
        Incomplete ? TEXT("true") : TEXT("false"), ExportValid ? TEXT("true") : TEXT("false"));
    ExportValid &= FFileHelper::SaveStringToFile(Summary, *(Output / TEXT("summary.json")));
    TestTrue(TEXT("matrix completed every declared case"), !Incomplete && CaseCount == Expected);
    TestTrue(TEXT("all matrix evidence saved"), ExportValid);
    TestEqual(TEXT("all collision and invariance gates"), Failed, uint32(0));
    AddInfo(FString::Printf(TEXT("[CollisionMatrix042] Worlds=1 Cases=%u/%u Frames=%llu Failures=%u Evidence=%s WheelCoverage=0"),
        CaseCount, Expected, FrameCount, Failed, *Output));
    return !HasAnyErrors();
}
#endif
