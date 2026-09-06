#if WITH_DEV_AUTOMATION_TESTS
#include "AnalyticWorldQuery.h"
#include "IAmSpeed/World/Collision/StaticCollisionWorld.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedSphereFiniteDomainTest,
    "IAmSpeed.AnalyticWorld.SphereFiniteFaceDomain",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedSphereFiniteDomainTest::RunTest(const FString& Parameters)
{
    using namespace Speed::Analytic;
    // Finite face contact and conservative Hybrid replacement coverage are
    // different contracts. A larger sphere cannot lose an interior face hit.
    for (const bool bExtruded : {false, true})
    {
        FAnalyticWorldData World;
        if (bExtruded)
        {
            FExtrudedQuinticPatch Patch;
            Patch.SourceId=1; Patch.SurfaceId=2; Patch.FeatureId=3; Patch.PrimitiveId=4;
            for (int32 I=0; I<6; ++I)
                Patch.SectionControlPoints[I]=FVector3d(-50+20*I, 0, 0);
            Patch.ExtrusionAxis=FVector3d::RightVector;
            Patch.MinimumExtrusionCoordinate=-25; Patch.MaximumExtrusionCoordinate=25;
            Patch.Bounds=FBox3d(FVector3d(-50,-25,0),FVector3d(50,25,0));
            World.ExtrudedQuinticPatches.Add(Patch);
        }
        else
        {
            FBoundedPlane Plane;
            Plane.SourceId=1; Plane.SurfaceId=2; Plane.FeatureId=3; Plane.PrimitiveId=4;
            Plane.HalfExtents=FVector2d(50,25);
            World.Planes.Add(Plane);
        }
        FString Reason;
        if (!TestTrue(TEXT("finite fixture validates"),World.FinalizeAndValidate(&Reason)))
        { AddError(Reason); return false; }
        if (bExtruded) World.ExtrudedQuinticPatches[0].bAuthorityEligible=true;
        else World.Planes[0].bAuthorityEligible=true;
        const Speed::FAnalyticStaticCollisionWorld Strict(World);
        const FWorldQueryService Conservative(World);
        for (const FVector3d Position : {FVector3d(49,0,4),FVector3d(-49,0,4),
                                        FVector3d(0,24,4),FVector3d(0,-24,4)})
        {
            FWorldQuery Q;
            Q.Shape=EQueryShape::Sphere; Q.bIncludeCompactPatches=true;
            Q.bAuthorityOnly=true; Q.InitialOverlapTolerance=0;
            Q.Start=Q.End=Position;
            double PreviousDepth=0;
            for (const double Radius : {5.,5.015,6.})
            {
                Q.Radius=Radius;
                const FWorldHit Hit=Strict.SweepSingle(Q);
                TestTrue(TEXT("strict interior face contact survives footprint crossing a boundary"),Hit.bHit);
                if (Hit.bHit)
                {
                    TestTrue(TEXT("overlap has its exact face depth"),FMath::IsNearlyEqual(Hit.PenetrationDepth,Radius-4.,1.e-9));
                    TestTrue(TEXT("radius inflation cannot reduce penetration"),Hit.PenetrationDepth>=PreviousDepth);
                    TestTrue(TEXT("finite face witnesses remain coherent"),
                        (Hit.Point-Hit.QueryPoint).Equals(Hit.PenetrationDepth*Hit.Normal,1.e-9));
                    PreviousDepth=Hit.PenetrationDepth;
                }
                TestFalse(TEXT("conservative replacement coverage still rejects the incomplete footprint"),Conservative.Sweep(Q).bHit);
                Q.bUseFiniteContactDomain=true;
                TestTrue(TEXT("same service/cache distinguishes finite domain from conservative coverage"),Conservative.Sweep(Q).bHit);
                FWorldHit Replacement;
                Conservative.TrySweepAuthority(Q,Replacement);
                TestFalse(TEXT("Hybrid replacement resets an incoming strict domain policy"),Replacement.bHit);
                Q.bUseFiniteContactDomain=false;
                TestFalse(TEXT("finite-domain cache hit does not leak into conservative coverage"),Conservative.Sweep(Q).bHit);
            }
            Q.Radius=5; Q.Start.Z=10; Q.End.Z=0;
            TestTrue(TEXT("fresh CCD reaches the finite face, not only a later projection"),Strict.SweepSingle(Q).bHit);
            Q.Start=Q.End=Position;
            // Keep the swept AABB intersecting the finite face, so this guard
            // reaches the narrow-phase domain test rather than a broad miss.
            if (Position.X!=0) Q.Start.X=Q.End.X=FMath::Sign(Position.X)*50.5;
            else Q.Start.Y=Q.End.Y=FMath::Sign(Position.Y)*25.5;
            TestFalse(TEXT("no infinite-plane extension beyond the real domain"),Strict.SweepSingle(Q).bHit);
        }
    }

    // Curved finite extrusion: frozen numeric witness independently reproduces
    // the larger-radius miss, without loading any gameplay asset or actor.
    FAnalyticWorldData World;
    FExtrudedQuinticPatch Patch;
    Patch.SourceId=1; Patch.SurfaceId=2; Patch.FeatureId=3; Patch.PrimitiveId=4;
    const FVector3d Controls[6]={
        {-.18542512947599127,4045.1588916359865,2279.0708007812505},
        {-.19293596593910706,4209.012232271407,2279.0708007812505},
        {-.20044680240222285,4372.865572906828,2279.0708007812505},
        {-.2106658793467098,4595.8007817597145,2079.073406996422},
        {-.2106658793467098,4595.8007817597145,1932.0789449792228},
        {-.2106658793467098,4595.8007817597145,1785.0844829620237}};
    for (int32 I=0; I<6; ++I) Patch.SectionControlPoints[I]=Controls[I];
    Patch.ExtrusionAxis=FVector3d(.9999999985490485,1.5766505725425256e-5,5.1510388583022374e-5);
    Patch.MinimumExtrusionCoordinate=-4017.6929963619355;
    Patch.MaximumExtrusionCoordinate=4016.064607506742;
    Patch.Bounds=FBox3d(FVector3d(-4017.90366,4045.09554,1784.87753),FVector3d(4015.87918,4595.86411,2279.27767));
    World.ExtrudedQuinticPatches.Add(Patch);
    FString Reason;
    if (!TestTrue(TEXT("curved finite fixture validates"),World.FinalizeAndValidate(&Reason)))
    { AddError(Reason); return false; }
    World.ExtrudedQuinticPatches[0].bAuthorityEligible=true;
    const Speed::FAnalyticStaticCollisionWorld Strict(World);
    FWorldQuery Q;
    Q.Shape=EQueryShape::Sphere; Q.bIncludeCompactPatches=true;
    Q.bAuthorityOnly=true; Q.InitialOverlapTolerance=0;
    Q.Start=Q.End=FVector3d(3906.384,4470.559,1925.578);
    Q.Radius=109.5;
    const FWorldHit Physical=Strict.SweepSingle(Q);
    Q.Radius=109.515;
    const FWorldHit Inflated=Strict.SweepSingle(Q);
    TestTrue(TEXT("physical sphere overlaps the curved face"),Physical.bHit&&Physical.bStartPenetrating);
    TestTrue(TEXT("inflated certificate cannot miss the same curved face"),Inflated.bHit&&Inflated.bStartPenetrating);
    if (Physical.bHit&&Inflated.bHit)
        TestTrue(TEXT("curved face depth increases by the radius delta"),
            FMath::IsNearlyEqual(Inflated.PenetrationDepth-Physical.PenetrationDepth,.015,1.e-8));
    Q.Start=FVector3d(3914.770,4481.900,1930.420);
    Q.End=FVector3d(3910.574,4482.107,1929.961);
    Q.Radius=109.50009918212891;
    TestTrue(TEXT("fresh CCD sees the wall before the large delayed projection"),Strict.SweepSingle(Q).bHit);
    return true;
}
#endif
