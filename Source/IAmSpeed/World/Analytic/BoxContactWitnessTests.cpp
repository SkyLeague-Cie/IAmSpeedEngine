#if WITH_DEV_AUTOMATION_TESTS
#include "AnalyticWorldQuery.h"
#include "IAmSpeed/World/Collision/StaticCollisionWorld.h"
#include "Misc/AutomationTest.h"

namespace BoxContactWitnessTestPrivate
{
using namespace Speed::Analytic;

/** Builds an exact finite rectangle through the public extruded provider. */
void AddRectangle(FAnalyticWorldData& World, uint64 Id, double Height, double HalfX, double HalfY)
{
    FExtrudedQuinticPatch Patch;
    Patch.SourceId=100+Id; Patch.SurfaceId=200+Id;
    Patch.FeatureId=300+Id; Patch.PrimitiveId=400+Id;
    for (int32 I=0; I<6; ++I)
        Patch.SectionControlPoints[I]=FVector3d(-HalfX+2.*HalfX*I/5.,0,Height);
    Patch.ExtrusionAxis=FVector3d::RightVector;
    Patch.MinimumExtrusionCoordinate=-HalfY;
    Patch.MaximumExtrusionCoordinate=HalfY;
    Patch.Bounds=FBox3d(FVector3d(-HalfX,-HalfY,Height),FVector3d(HalfX,HalfY,Height));
    World.ExtrudedQuinticPatches.Add(Patch);
}

void MarkAuthority(FAnalyticWorldData& World)
{
    for (auto& Patch : World.ExtrudedQuinticPatches) Patch.bAuthorityEligible=true;
}

/** Compare public results fieldwise; struct padding is not deterministic data. */
bool SameHit(const FWorldHit& A, const FWorldHit& B)
{
    return A.bHit==B.bHit && A.bStartPenetrating==B.bStartPenetrating &&
        A.Time==B.Time && A.PenetrationDepth==B.PenetrationDepth &&
        A.Location==B.Location && A.Point==B.Point && A.QueryPoint==B.QueryPoint && A.Normal==B.Normal &&
        A.QueryFeatureKind==B.QueryFeatureKind && A.SurfaceFeatureKind==B.SurfaceFeatureKind &&
        A.QueryFeatureIndex==B.QueryFeatureIndex && A.SurfaceFeatureIndex==B.SurfaceFeatureIndex &&
        A.SourceId==B.SourceId && A.SurfaceId==B.SurfaceId && A.FeatureId==B.FeatureId &&
        A.PrimitiveId==B.PrimitiveId && A.CanonicalGroupId==B.CanonicalGroupId &&
        A.MaterialId==B.MaterialId && A.GeometricErrorBoundCm==B.GeometricErrorBoundCm &&
        A.AdditionalResidualAgreementAllowanceCm==B.AdditionalResidualAgreementAllowanceCm &&
        A.bSurfaceNormalMayVary==B.bSurfaceNormalMayVary;
}

/** Independent boundary oracle: volume membership alone is insufficient. */
bool IsBoxBoundaryPoint(const FWorldQuery& Q, const FVector3d& Center,
    const FVector3d& Point, double Tolerance)
{
    const FVector3d Axes[3]={Q.Rotation.GetAxisX(),Q.Rotation.GetAxisY(),Q.Rotation.GetAxisZ()};
    bool bOnBoundary=false;
    for (int32 I=0; I<3; ++I)
    {
        const double Coordinate=FMath::Abs(FVector3d::DotProduct(Point-Center,Axes[I]));
        if (Coordinate>Q.HalfExtent[I]+Tolerance) return false;
        bOnBoundary |= FMath::Abs(Coordinate-Q.HalfExtent[I])<=Tolerance;
    }
    return bOnBoundary;
}

FWorldQuery MakeBoxQuery()
{
    FWorldQuery Q;
    Q.Shape=EQueryShape::Box; Q.bIncludeCompactPatches=true;
    Q.InitialOverlapTolerance=0; Q.DomainTolerance=1.e-8;
    Q.HalfExtent=FVector3d(5,4,3);
    return Q;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedBoxWitnessWorldArbitrationTest,
    "IAmSpeed.AnalyticWorld.BoxWitnessAfterWorldArbitration",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedBoxWitnessWorldArbitrationTest::RunTest(const FString& Parameters)
{
    using namespace Speed::Analytic;
    using namespace BoxContactWitnessTestPrivate;
    FWorldHit FirstOrderWinner;
    for (int32 Reverse=0; Reverse<2; ++Reverse)
    {
        FAnalyticWorldData RawWorld;
        for (int32 I=0; I<2; ++I)
        {
            const int32 Id=Reverse ? 2-I : I+1;
            AddRectangle(RawWorld,Id,Id==1 ? 0. : 1.,Id==1 ? 50. : 100.,Id==1 ? 25. : 100.);
        }
        FString Reason;
        if (!TestTrue(TEXT("two-provider witness fixture validates"),RawWorld.FinalizeAndValidate(&Reason)))
        { AddError(Reason); return false; }
        // Draft/default queries preserve the raw finite SAT witnesses. This
        // gives an independent observable reference for PRE-repair arbitration.
        const FWorldQueryService Raw(RawWorld);
        FWorldQuery Q=MakeBoxQuery();
        Q.Start=Q.End=FVector3d(49,0,2);
        FWorldQuery OnlyA=Q; OnlyA.RequiredSurfaceId=201;
        FWorldQuery OnlyB=Q; OnlyB.RequiredSurfaceId=202;
        const FWorldHit A=Raw.Sweep(OnlyA), B=Raw.Sweep(OnlyB);
        if (!TestTrue(TEXT("both same-time candidates are penetrating"),
            A.bHit && B.bHit && A.bStartPenetrating && B.bStartPenetrating)) return false;
        const double KeyA=(A.Point-A.QueryPoint).SquaredLength();
        const double KeyB=(B.Point-B.QueryPoint).SquaredLength();
        TestTrue(TEXT("fixture distinguishes arbitration before and after repair"),
            A.Time==B.Time && A.PenetrationDepth<B.PenetrationDepth && KeyA>KeyB &&
            FMath::Square(A.PenetrationDepth)<KeyB);
        TestTrue(TEXT("raw world selects B by the existing witness-distance key"),SameHit(Raw.Sweep(Q),B));

        FAnalyticWorldData StrictWorld=RawWorld;
        MarkAuthority(StrictWorld);
        const Speed::FAnalyticStaticCollisionWorld Strict(StrictWorld);
        Q.bAuthorityOnly=true;
        const FWorldHit Winner=Strict.SweepSingle(Q);
        // This does not declare B the optimal physical constraint. It protects
        // the narrower contract: witness cleanup must not retune arbitration.
        TestTrue(TEXT("repair after world arbitration preserves the selected provider and result"),SameHit(Winner,B));
        TestTrue(TEXT("cached global winner is unchanged"),SameHit(Strict.SweepSingle(Q),Winner));
        if (Reverse==0) FirstOrderWinner=Winner;
        else TestTrue(TEXT("provider insertion order cannot change the global winner"),SameHit(Winner,FirstOrderWinner));

        OnlyA.bUseFiniteContactDomain=true;
        const FWorldHit RepairedA=Raw.Sweep(OnlyA);
        TestTrue(TEXT("A alone repairs its witness without changing its local geometric result"),
            RepairedA.bHit && RepairedA.Time==A.Time && RepairedA.Normal==A.Normal &&
            RepairedA.Point==A.Point && RepairedA.PenetrationDepth==A.PenetrationDepth &&
            RepairedA.PrimitiveId==A.PrimitiveId &&
            (RepairedA.Point-RepairedA.QueryPoint).Equals(RepairedA.PenetrationDepth*RepairedA.Normal,1.e-7));
        OnlyA.bUseFiniteContactDomain=false;
        TestTrue(TEXT("strict witness repair cannot leak into a cached default query"),SameHit(Raw.Sweep(OnlyA),A));

        const FWorldQueryService Coverage(StrictWorld);
        FWorldHit ConservativeHit, StrictInputHit;
        const bool bConservative=Coverage.TrySweepAuthority(Q,ConservativeHit);
        Q.bUseFiniteContactDomain=true;
        const bool bStrictInput=Coverage.TrySweepAuthority(Q,StrictInputHit);
        TestEqual(TEXT("incoming strict bit preserves Hybrid coverage"),bStrictInput,bConservative);
        TestTrue(TEXT("incoming strict bit preserves the complete Hybrid witness and winner"),SameHit(StrictInputHit,ConservativeHit));
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedBoxWitnessBoundaryTest,
    "IAmSpeed.AnalyticWorld.BoxWitnessBoundaryMembership",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedBoxWitnessBoundaryTest::RunTest(const FString& Parameters)
{
    using namespace Speed::Analytic;
    using namespace BoxContactWitnessTestPrivate;
    FAnalyticWorldData World;
    AddRectangle(World,1,0,50,25);
    FString Reason;
    if (!TestTrue(TEXT("boundary-witness fixture validates"),World.FinalizeAndValidate(&Reason)))
    { AddError(Reason); return false; }
    const FWorldQueryService Raw(World);
    FAnalyticWorldData StrictWorld=World;
    MarkAuthority(StrictWorld);
    const Speed::FAnalyticStaticCollisionWorld Strict(StrictWorld);
    for (const double SX : {-1.,1.})
    for (const double SY : {-1.,1.})
    for (const double SZ : {-1.,1.})
    for (const double Yaw : {0.,UE_DOUBLE_PI/4.})
    {
        FWorldQuery Q=MakeBoxQuery();
        Q.Rotation=FQuat4d(FVector3d::UpVector,Yaw);
        Q.Start=Q.End=FVector3d(49*SX,24*SY,2*SZ);
        const FWorldHit Before=Raw.Sweep(Q);
        Q.bAuthorityOnly=true;
        const FWorldHit Hit=Strict.SweepSingle(Q);
        if (!TestTrue(TEXT("mirrored/rotated partial overhang has a finite overlap"),Hit.bHit && Hit.bStartPenetrating)) continue;
        const FVector3d Translation=Hit.PenetrationDepth*Hit.Normal;
        TestTrue(TEXT("witness cleanup preserves local MTD, point and TOI"),
            Before.bHit && Hit.Point==Before.Point && Hit.Normal==Before.Normal &&
            Hit.PenetrationDepth==Before.PenetrationDepth && Hit.Time==Before.Time);
        TestTrue(TEXT("the returned witness is on the original box boundary"),
            IsBoxBoundaryPoint(Q,Hit.Location,Hit.QueryPoint,1.e-7));
        TestTrue(TEXT("the surface witness touches the translated box boundary"),
            IsBoxBoundaryPoint(Q,Hit.Location+Translation,Hit.Point,1.e-7));
        TestTrue(TEXT("the witness vector represents the selected MTD"),
            (Hit.Point-Hit.QueryPoint).Equals(Translation,1.e-7));
        TestTrue(TEXT("a repaired witness cannot have an interior/unknown feature"),
            Hit.QueryFeatureKind!=EContactFeatureKind::Unknown);
        TestTrue(TEXT("surface witness remains in the finite rectangle"),
            FMath::Abs(Hit.Point.X)<=50.+1.e-7 && FMath::Abs(Hit.Point.Y)<=25.+1.e-7 && FMath::Abs(Hit.Point.Z)<=1.e-7);
        // This validates the independent oracle, not the private clamp helper.
        // A public valid SAT query cannot inject an arbitrary invalid FWorldHit.
        TestFalse(TEXT("volume-center membership is not a boundary witness"),
            IsBoxBoundaryPoint(Q,Hit.Location+Translation,Hit.Location+Translation,1.e-7));
        TestTrue(TEXT("boundary witness cache is deterministic"),SameHit(Hit,Strict.SweepSingle(Q)));
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedBoxFiniteCornerWitnessTest,
    "IAmSpeed.AnalyticWorld.BoxFiniteCornerWitnessControls",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedBoxFiniteCornerWitnessTest::RunTest(const FString& Parameters)
{
    using namespace Speed::Analytic;
    using namespace BoxContactWitnessTestPrivate;
    FAnalyticWorldData World;
    AddRectangle(World,1,0,50,25);
    FString Reason;
    if (!TestTrue(TEXT("finite corner fixture validates"),World.FinalizeAndValidate(&Reason)))
    { AddError(Reason); return false; }
    const FWorldQueryService Raw(World);
    FAnalyticWorldData StrictWorld=World;
    MarkAuthority(StrictWorld);
    const Speed::FAnalyticStaticCollisionWorld Strict(StrictWorld);
    const FWorldQueryService Coverage(StrictWorld);
    for (const double SX : {-1.,1.})
    for (const double SY : {-1.,1.})
    {
        FWorldQuery Q=MakeBoxQuery(); Q.HalfExtent=FVector3d(5,1,3);
        Q.Rotation=FQuat4d(FVector3d::UpVector,FMath::Atan2(SY,SX));
        Q.Start=Q.End=FVector3d(54*SX,29*SY,2);
        const double AabbHalfX=5*FMath::Abs(Q.Rotation.GetAxisX().X)+FMath::Abs(Q.Rotation.GetAxisY().X);
        const double AabbHalfY=5*FMath::Abs(Q.Rotation.GetAxisX().Y)+FMath::Abs(Q.Rotation.GetAxisY().Y);
        TestTrue(TEXT("negative corner fixture overlaps the provider AABB"),54-AabbHalfX<50 && 29-AabbHalfY<25);
        TestFalse(TEXT("finite SAT rejects the separated rotated corner despite AABB overlap"),Strict.SweepSingle(Q).bHit);
        Q.End=FVector3d(46*SX,21*SY,2);
        const FWorldHit Original=Raw.Sweep(Q);
        Q.bAuthorityOnly=true;
        const FWorldHit Hit=Strict.SweepSingle(Q);
        const double ExpectedTime=(4.-5./FMath::Sqrt(2.))/8.;
        TestTrue(TEXT("the true finite corner is reached at its geometric time"),
            Hit.bHit && !Hit.bStartPenetrating && FMath::IsNearlyEqual(Hit.Time,ExpectedTime,1.e-8));
        const FVector3d ExpectedNormal=FVector3d(SX,SY,0).GetSafeNormal();
        TestTrue(TEXT("true terminal contact retains the outward SAT normal"),
            Hit.bHit && FVector3d::DotProduct(Hit.Normal,ExpectedNormal)>1.-1.e-8);
        TestTrue(TEXT("penetrating-witness cleanup leaves nonpenetrating corner results unchanged"),SameHit(Hit,Original));
        TestTrue(TEXT("finite corner result survives exact cache replay"),SameHit(Hit,Strict.SweepSingle(Q)));
        FWorldHit DefaultCoverage, StrictInputCoverage;
        const bool bDefault=Coverage.TrySweepAuthority(Q,DefaultCoverage);
        Q.bUseFiniteContactDomain=true;
        const bool bStrictInput=Coverage.TrySweepAuthority(Q,StrictInputCoverage);
        // Replacement can certify a MISS after conservative provider coverage
        // excludes this terminal contact. Covered=true is not a collision hit.
        TestTrue(TEXT("the existing Hybrid path retains its covered miss at the terminal corner"),
            bDefault && !DefaultCoverage.bHit);
        TestEqual(TEXT("strict corner input cannot weaken Hybrid coverage"),bStrictInput,bDefault);
        TestTrue(TEXT("Hybrid corner witnesses remain policy-isolated"),SameHit(DefaultCoverage,StrictInputCoverage));
    }
    return true;
}
#endif
