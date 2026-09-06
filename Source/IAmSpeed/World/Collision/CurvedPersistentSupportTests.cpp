#if WITH_DEV_AUTOMATION_TESTS
#include "StaticCollisionWorld.h"
#include "IAmSpeed/Components/ISpeedComponent.h"
#include "IAmSpeed/SubBodies/Configs/SubBodyConfig.h"
#include "IAmSpeed/SubBodies/Solid/BoxSubBody.h"
#include "Components/BoxComponent.h"
#include "HAL/IConsoleManager.h"
#include "Misc/AutomationTest.h"
#include "Misc/ScopeExit.h"
#include "UObject/StrongObjectPtr.h"

namespace
{
// State-only adapter: persistent-contact behavior is the production UBoxSubBody.
class FCurvedPersistenceBody final : public ISpeedComponent
{
public:
    SKinematic State;
    SubBodyConfig Config;
    FVector LocalCOM = FVector::ZeroVector;
    TStrongObjectPtr<UBoxSubBody> Box{NewObject<UBoxSubBody>()};
    TArray<USSubBody*> Bodies;

    FCurvedPersistenceBody()
    {
        Config.bValid = true;
        Config.bIsMainSubBody = true;
        Config.Mass = 10;
        Config.BoxExtent = FVector(4, 4, 2);
        Bodies.Add(Box.Get());
        Box->Initialize(this);
    }
    void PrepareCanonicalFrame(const FCanonicalFrameContext&) override {}
    unsigned int NumFrame() const override { return 60; }
    float GetPhysMass() const override { return Config.Mass; }
    const FVector& GetPhysCOM() const override { return State.Location; }
    const FVector& GetPhysCenterOfMassLocal() const override { return LocalCOM; }
    const TArray<USSubBody*>& GetSubBodies() const override { return Bodies; }
    TArray<USSubBody*> CreateSubBodies() override { return Bodies; }
    SubBodyConfig GetSubBodyConfig(const USSubBody&) const override { return Config; }
    bool IsFrozen() const override { return false; }
    void SetIsFrozen(bool) override {}
    SKinematic GetKinematicsOfSubBody(const USSubBody&, const unsigned int&) const override
    { return Box->GetKinematicsFromOwnerKS(State); }
    const SKinematic& GetKinematicState() const override { return State; }
    const SKinematic& GetKinematicStateForFrame(const unsigned int&) const override { return State; }
    void SetKinematicState(const SKinematic& Value) override { State = Value; }
    float GetPhysMaxSpeed() const override { return 10000; }
    float GetPhysMaxAngularSpeed() const override { return 100; }
    void SetIsUpsideDown(bool) override {}
    bool IsUpsideDown() const override { return true; }
    bool IsInAutoRecover() const override { return false; }
    bool IsSubBodyInAutoRecoverMode() const override { return false; }
    void RcvImpactOnSubBody(const USSubBody&, const FVector&) override {}
    FMatrix ComputeWorldInvInertiaTensor() const override { return FMatrix::Identity; }
    FMatrix ComputeWorldInvInertiaTensorOfSubBody(const USSubBody&) const override { return FMatrix::Identity; }
    void PostPhysicsUpdatePrv(const float&) override {}
};
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedCurvedPersistentSupportTest,
    "IAmSpeed.PhysicalLaws.CurvedPersistentSupport.NoCachedPlaneAttraction",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedCurvedPersistentSupportTest::RunTest(const FString& Parameters)
{
    using namespace Speed::Analytic;
    IConsoleVariable* Backend = IConsoleManager::Get().FindConsoleVariable(TEXT("p.IAmSpeed.StaticCollision.Backend"));
    IConsoleVariable* Contact = IConsoleManager::Get().FindConsoleVariable(TEXT("p.IAmSpeed.FaceSupportContactEpsilonCm"));
    IConsoleVariable* Penetration = IConsoleManager::Get().FindConsoleVariable(TEXT("p.IAmSpeed.FaceSupportPenetrationTolCm"));
    if (!TestNotNull(TEXT("backend CVar exists"), Backend) ||
        !TestNotNull(TEXT("contact CVar exists"), Contact) ||
        !TestNotNull(TEXT("penetration CVar exists"), Penetration)) return false;
    const int32 OldBackend = Backend->GetInt();
    const float OldContact = Contact->GetFloat(), OldPenetration = Penetration->GetFloat();
    const EConsoleVariableFlags BackendFlags = Backend->GetFlags(), ContactFlags = Contact->GetFlags(), PenetrationFlags = Penetration->GetFlags();
    ON_SCOPE_EXIT
    {
        Backend->Set(OldBackend, ECVF_SetByCode); Backend->SetFlags(BackendFlags);
        Contact->Set(OldContact, ECVF_SetByCode); Contact->SetFlags(ContactFlags);
        Penetration->Set(OldPenetration, ECVF_SetByCode); Penetration->SetFlags(PenetrationFlags);
    };
    Contact->Set(3.f, ECVF_SetByCode);
    Penetration->Set(3.f, ECVF_SetByCode);

    FAnalyticWorldData Data;
    FExtrudedQuinticPatch Patch;
    Patch.SourceId = 1; Patch.SurfaceId = 2; Patch.FeatureId = 3; Patch.PrimitiveId = 4;
    Patch.CanonicalGroupId = 5; Patch.bCanonicalC2ByConstruction = true;
    Patch.bQueryCollisionEnabled = true; Patch.bAuthorityEligible = true;
    Patch.BlockingChannels = MAX_uint64;
    // x=40t, z=4t^2: a genuinely curved regular section with the z=0 tangent
    // at its first endpoint. Its extrusion ends at y=+/-10, not infinity.
    for (int32 I = 0; I < 6; ++I)
        Patch.SectionControlPoints[I] = FVector3d(8. * I, 0, .2 * I * (I - 1));
    Patch.ExtrusionAxis = FVector3d::RightVector;
    Patch.MinimumExtrusionCoordinate = -10; Patch.MaximumExtrusionCoordinate = 10;
    Patch.Bounds = FBox3d(FVector3d(0, -10, 0), FVector3d(40, 10, 4));
    Data.ExtrudedQuinticPatches.Add(Patch);
    FString Reason;
    if (!TestTrue(TEXT("finite curved fixture validates"), Data.FinalizeAndValidate(&Reason)))
    { AddError(Reason); return false; }
    const Speed::FAnalyticStaticCollisionWorld FiniteWorld(Data);
    FWorldQuery Q;
    Q.Shape = EQueryShape::Box;
    Q.bIncludeCompactPatches = true; Q.bAuthorityOnly = true;
    Q.InitialOverlapTolerance = 0;
    Q.HalfExtent = FVector3d(4, 4, 2);
    Q.Rotation = FQuat4d(FVector3d::ForwardVector, UE_DOUBLE_PI);
    Q.Start = Q.End = FVector3d(10, 0, .5);
    TestTrue(TEXT("finite provider really collides within its authored domain"), FiniteWorld.SweepSingle(Q).bHit);
    TStrongObjectPtr<UBoxComponent> Ground(NewObject<UBoxComponent>());
    Ground->SetMobility(EComponentMobility::Static);

    // Explicitly seed the stale manifold at the update phase, not by running a
    // long trajectory. In182 this update attracted the owner before CCD while
    // the cached box remained at its old pose. Both signed corrections must be
    // absent for native curved support; non-native control lanes still move.
    for (int32 Mode = 0; Mode < 4; ++Mode)
    for (const double Side : {-1., 1.})
    for (const double Z : {20., -10.})
    {
        const bool bNativeCurved = Mode == 0;
        Backend->Set(Mode == 1 ? 0 : 2, ECVF_SetByCode);
        FCurvedPersistenceBody Body;
        UBoxSubBody& Box = *Body.Box;
        Body.State.Location = FVector(0, Side * 50., Z);
        Body.State.Rotation = FQuat(FVector::ForwardVector, UE_DOUBLE_PI);
        Body.State.Velocity = FVector(30, 0, 0);
        Body.UpdateSubBodiesKinematics();
        Q.Start = Q.End = FVector3d(Body.State.Location);
        TestFalse(TEXT("box outside the finite extrusion has no contact"), FiniteWorld.SweepSingle(Q).bHit);
        // Independent separating axis: every box corner is at least36cm from
        // the y-domain. Neither signed tangent-plane gap is penetration.
        TestTrue(TEXT("independent finite-domain separation"), FMath::Abs(Body.State.Location.Y) - 4. > 10.);
        Box.GroundHit = SHitResult();
        Box.GroundHit.bHit = Box.GroundHit.bBlockingHit = true;
        Box.GroundHit.SourceId = 1; Box.GroundHit.SurfaceId = 2;
        Box.GroundHit.CanonicalGroupId = Mode == 3 ? 0 : 5;
        Box.GroundHit.bSurfaceNormalMayVary = Mode != 2;
        Box.GroundHit.ImpactNormal = FVector::UpVector;
        Box.GroundHit.ImpactPoint = FVector(0, Side * 10., 0);
        Box.GroundHit.Component = Ground.Get();
        Box.GroundComp = Ground.Get();
        Box.RoofSurfaceComp = Ground.Get();
        Box.bGroundPlaneValid = Box.bHasGroundContact = true;
        Box.GroundPlaneN = FVector::UpVector;
        Box.GroundPlanePointWS = Box.GroundHit.ImpactPoint;
        Box.GroundPlaneD = 0;
        Box.bRoofSurfaceTraversalLatched = true;
        Box.LastRoofSurfaceContactFrame = Body.NumFrame();
        TestTrue(TEXT("seed reaches the real roof-persistence branch"), Box.HasRoofSurfaceTraversalSupport());
        const SKinematic Before = Body.State;
        Box.UpdatePersistentGroundContact(1.f / 300.f);
        const FString Label = FString::Printf(TEXT("mode%d side%.0f z%.0f"), Mode, Side, Z);
        if (bNativeCurved)
        {
            TestTrue(Label + TEXT(" native curved pose cannot be attracted to a tangent extension"), Body.State.Location == Before.Location);
            TestTrue(Label + TEXT(" cached and canonical box remain consistent"), Box.GetKinematicState().Location == Body.State.Location);
            TestFalse(Label + TEXT(" out-of-band tangent data does not retain physical support"), Box.HasPersistentGroundContact());
        }
        else
        {
            const double ExpectedZ = Z > 0 ? 2.5 : 1.5;
            TestTrue(Label + TEXT(" non-native control executes the signed plane correction"),
                Body.State.Location.Equals(FVector(0, Side * 50., ExpectedZ), 1.e-8));
        }
        TestTrue(Label + TEXT(" update never changes velocity or rotation"),
            Body.State.Velocity == Before.Velocity && Body.State.Rotation == Before.Rotation &&
            Body.State.AngularVelocity == Before.AngularVelocity);
    }
    return !HasAnyErrors();
}
#endif
