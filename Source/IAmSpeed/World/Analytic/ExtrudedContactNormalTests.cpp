#if WITH_DEV_AUTOMATION_TESTS
#include "AnalyticWorldQuery.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedExtrudedContactNormalTest,
    "IAmSpeed.AnalyticWorld.ExtrudedInternalContactNormal",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedExtrudedContactNormalTest::RunTest(const FString& Parameters)
{
    using namespace Speed::Analytic;
    // A smooth ceiling-to-wall profile. Its adaptive chord edges and triangle
    // diagonals are query implementation details, not physical surface creases.
    FAnalyticWorldData World;
    FExtrudedQuinticPatch Patch;
    Patch.SourceId = 1; Patch.SurfaceId = 2; Patch.FeatureId = 3; Patch.PrimitiveId = 4;
    const FVector3d Controls[6] = {
        {-0.18542512947599127, -4045.1588916359865, 2279.0708007812505},
        {-0.19293596593910706, -4209.0122322714069, 2279.0708007812505},
        {-0.20044680240222285, -4372.8655729068278, 2279.0708007812505},
        {-0.2106658793467098, -4595.8007817597145, 2079.0734069964219},
        {-0.2106658793467098, -4595.8007817597145, 1932.0789449792228},
        {-0.2106658793467098, -4595.8007817597145, 1785.0844829620237}};
    for (int32 I = 0; I < 6; ++I) Patch.SectionControlPoints[I] = Controls[I];
    Patch.ExtrusionAxis = FVector3d(0.99999999128865724, -0.0001175831520572529, -5.9974058857485914e-05);
    Patch.MinimumExtrusionCoordinate = -4014.7121855191917;
    Patch.MaximumExtrusionCoordinate = 4020.5515474105314;
    Patch.Bounds = FBox3d(FVector3d(-4014.92282,-4596.27354,1784.84335),
        FVector3d(4020.36609,-4044.68682,2279.31159));
    World.ExtrudedQuinticPatches.Add(Patch);
    // A second nearly parallel extrusion overlaps this finite domain. Query
    // arbitration must not turn either approximation's internal edge into a wall.
    Patch.PrimitiveId = 5;
    Patch.ExtrusionAxis = FVector3d(0.99999999854904875,-1.576650572542526e-05,5.1510388583022388e-05);
    Patch.MinimumExtrusionCoordinate = -4017.6929963619355;
    Patch.MaximumExtrusionCoordinate = 4016.064607506742;
    Patch.Bounds = FBox3d(FVector3d(-4017.90366,-4595.86411,1784.87753),
        FVector3d(4015.87918,-4045.09554,2279.27767));
    World.ExtrudedQuinticPatches.Add(Patch);
    FString Reason;
    if (!TestTrue(TEXT("regular extruded world validates"), World.FinalizeAndValidate(&Reason)))
    { AddError(Reason); return false; }
    for (auto& P : World.ExtrudedQuinticPatches) P.bAuthorityEligible = true;
    FWorldQueryService Service(World);
    FWorldQuery Q;
    Q.bIncludeCompactPatches = true;
    Q.Shape = EQueryShape::Box;
    Q.Start = FVector3d(-2604.0547065080846,-4336.4990365196909,2146.6306957497372);
    Q.End = FVector3d(-2604.0349651425604,-4332.7711603961989,2148.4815991007572);
    Q.Rotation = FQuat4d(0.42194105855942582,-0.21041720780992407,0.69064299851775302,0.54836355673734694);
    Q.HalfExtent = FVector3d(78.790000915527344,52.299999237060547,19.139999389648438);
    const FWorldHit Hit = Service.Sweep(Q);
    if (!TestTrue(TEXT("shallow curved contact is not discarded"), Hit.bHit)) return false;
    AddInfo(FString::Printf(TEXT("normal=%s point=%s query=%s depth=%.12g"),
        *Hit.Normal.ToString(), *Hit.Point.ToString(), *Hit.QueryPoint.ToString(), Hit.PenetrationDepth));
    TestTrue(TEXT("internal contact points toward the concave interior, not against tangent travel"),
        Hit.Normal.Y > 0 && Hit.Normal.Z < 0);
    TestTrue(TEXT("normal and depth describe the same finite separation witness"),
        (Hit.Point - Hit.QueryPoint).Equals(Hit.PenetrationDepth * Hit.Normal, 1.e-9));
    for (int32 Mirror = 0; Mirror < 4; ++Mirror)
    {
        const double SX = (Mirror & 1) ? -1 : 1;
        const double SY = (Mirror & 2) ? -1 : 1;
        const auto Reflect = [SX, SY](const FVector3d& P) { return FVector3d(SX*P.X,SY*P.Y,P.Z); };
        FAnalyticWorldData MirroredWorld;
        for (const auto& P : World.ExtrudedQuinticPatches)
        {
            auto Copy = P;
            // Runtime acceleration data is rebuilt from the reflected geometry.
            Copy.bAuthorityEligible = false;
            Copy.Bounds = FBox3d(EForceInit::ForceInit);
            for (FVector3d& Point : Copy.SectionControlPoints) Point = Reflect(Point);
            Copy.ExtrusionAxis = Reflect(Copy.ExtrusionAxis);
            for (const auto& Point : Copy.SectionControlPoints)
            {
                Copy.Bounds += Point + Copy.MinimumExtrusionCoordinate * Copy.ExtrusionAxis;
                Copy.Bounds += Point + Copy.MaximumExtrusionCoordinate * Copy.ExtrusionAxis;
            }
            MirroredWorld.ExtrudedQuinticPatches.Add(Copy);
        }
        if (!TestTrue(TEXT("reflected finite world validates"), MirroredWorld.FinalizeAndValidate())) return false;
        for (auto& P : MirroredWorld.ExtrudedQuinticPatches) P.bAuthorityEligible = true;
        FWorldQueryService MirroredService(MirroredWorld);
        for (double Offset : {-0.01, 0.0, 0.01})
        {
            auto Probe = Q;
            Probe.Start = Reflect(Q.Start + FVector3d(0,Offset,0));
            Probe.End = Reflect(Q.End + FVector3d(0,Offset,0));
            // A quaternion's vector part transforms axially under a reflection.
            Probe.Rotation = FQuat4d(SY*Q.Rotation.X,SX*Q.Rotation.Y,SX*SY*Q.Rotation.Z,Q.Rotation.W);
            const auto Result = MirroredService.Sweep(Probe);
            TestTrue(TEXT("mirrored/perturbed overlap keeps a contact"), Result.bHit);
            TestTrue(TEXT("submillimetre perturbations cannot switch to an internal wall normal"),
                FVector3d::DotProduct(Result.Normal, Reflect(Hit.Normal)) > 0.99999);
        }
    }
    return true;
}
#endif
