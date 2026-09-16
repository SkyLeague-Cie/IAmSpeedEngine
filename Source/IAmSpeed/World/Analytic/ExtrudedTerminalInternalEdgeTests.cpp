#if WITH_DEV_AUTOMATION_TESTS
#include "AnalyticWorldQuery.h"
#include "IAmSpeed/World/Collision/StaticCollisionWorld.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedExtrudedTerminalInternalEdgeTest,
    "IAmSpeed.AnalyticWorld.ExtrudedTerminalInternalEdge",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedExtrudedTerminalInternalEdgeTest::RunTest(const FString& Parameters)
{
    using namespace Speed::Analytic;
    // Exact vehicle input, ContactPrecision01 roof-2, frame16/attempt102.
    // Asset SHA ABA47D94D1E7E71753BD1A005503C04C85417FBFEC46DAAC78ADF18F612A0228.
    // Baseline provider D5C7B9B6573745B2, facet 56B91C519CB8C4A4:
    // TOI .61955971192293469, normal(-.18454462430440521,.18467611763490227,-.96531757117315853).
    FExtrudedQuinticPatch Base;
    Base.SourceId=0x6B096F692F3206D0ull; Base.SurfaceId=0xD1F2DF807735DBD9ull;
    Base.FeatureId=0x09C354C9ADDBA88Cull; Base.PrimitiveId=0xD5C7B9B6573745B2ull;
    Base.ObjectType=0; Base.BlockingChannels=0xFFFFFEEFull;
    Base.bQueryCollisionEnabled=true;
    Base.SectionControlPoints[0]=FVector3d(3998.050891337107,-3998.051677522742,-0.0035156249068677425);
    Base.SectionControlPoints[1]=FVector3d(4108.262069761709,-4108.262877619516,-0.0035156249068677425);
    Base.SectionControlPoints[2]=FVector3d(4218.473248186312,-4218.4740777162915,-0.0035156249068677425);
    Base.SectionControlPoints[3]=FVector3d(4367.95850864448,-4367.959367569575,230.24849607066363);
    Base.SectionControlPoints[4]=FVector3d(4367.958472241777,-4367.959331166863,370.515780060177);
    Base.SectionControlPoints[5]=FVector3d(4367.958435839072,-4367.959294764152,510.7830640496904);
    Base.ExtrusionAxis=FVector3d(-0.7069229312894242,-0.707290563667015,-0.0001666276214726403);
    Base.MinimumExtrusionCoordinate=-1791.512980255973;
    Base.MaximumExtrusionCoordinate=381.3725302467989;
    FWorldQuery Q;
    Q.Shape=EQueryShape::Box;
    Q.Start=FVector3d(4838.788842338774,-3841.6828395910561,433.50933012635699);
    Q.End=FVector3d(4838.7853800906105,-3841.6852299041957,434.86490604799235);
    Q.Rotation=FQuat4d(.73365345930700532,-.3037957028906868,.56160831535029665,.2325013390959712);
    Q.HalfExtent=FVector3d(78.790000915527344,52.299999237060547,19.139999389648438);
    Q.DomainTolerance=1.e-6; Q.InitialOverlapTolerance=.05;
    Q.TraceChannel=1; Q.BlockingObjectTypes=1; Q.bApplyCollisionFilter=true;
    Q.bIncludeCompactPatches=true;
    // Isolated geometry is not an authored authority certificate. All geometric
    // query fields are captured, and the strict wrapper still enables finite domains.
    Q.bAuthorityOnly=false;
    for (const FVector3d Mirror : {FVector3d(1,1,1),FVector3d(-1,1,1),FVector3d(1,-1,1),FVector3d(-1,-1,1)})
    {
        FAnalyticWorldData World;
        FExtrudedQuinticPatch P=Base;
        for (auto& Point:P.SectionControlPoints) Point*=Mirror;
        P.ExtrusionAxis*=Mirror;
        P.Bounds=FBox3d(EForceInit::ForceInit);
        for (const auto& Point:P.SectionControlPoints)
        {
            P.Bounds+=Point+P.MinimumExtrusionCoordinate*P.ExtrusionAxis;
            P.Bounds+=Point+P.MaximumExtrusionCoordinate*P.ExtrusionAxis;
        }
        World.ExtrudedQuinticPatches.Add(P);
        FString Reason;
        if (!TestTrue(TEXT("frozen reflected provider validates"),World.FinalizeAndValidate(&Reason)))
        { AddError(Reason);return false; }
        FWorldQuery Reflected=Q;
        Reflected.Start*=Mirror; Reflected.End*=Mirror;
        Reflected.Rotation=FQuat4d(Mirror.Y*Q.Rotation.X,Mirror.X*Q.Rotation.Y,
            Mirror.X*Mirror.Y*Q.Rotation.Z,Q.Rotation.W);
        // Non-vacuous control: the same exact query must first hit the raw
        // terminal facet. Encode its genuine skew quad in an orthonormal
        // plane chart; no fabricated authority or enlarged finite domain.
        const auto& Built=World.ExtrudedQuinticPatches[0];
        const FVector3d A=Built.SectionPolyline[Built.SectionPolyline.Num()-2];
        const FVector3d B=Built.SectionPolyline.Last();
        FBoundedPlane Face;
        Face.SourceId=P.SourceId;Face.SurfaceId=P.SurfaceId;Face.FeatureId=P.FeatureId;
        Face.PrimitiveId=5;Face.ObjectType=P.ObjectType;Face.BlockingChannels=P.BlockingChannels;
        Face.bQueryCollisionEnabled=true;
        Face.AxisU=(B-A).GetSafeNormal();
        Face.Normal=FVector3d::CrossProduct(Face.AxisU,P.ExtrusionAxis).GetSafeNormal();
        Face.AxisV=FVector3d::CrossProduct(Face.Normal,Face.AxisU).GetSafeNormal();
        Face.Origin=.5*(A+B)+.5*(P.MinimumExtrusionCoordinate+P.MaximumExtrusionCoordinate)*P.ExtrusionAxis;
        Face.HalfExtents=FVector2d(1.e-12,1.e-12);
        Face.Bounds=FBox3d(EForceInit::ForceInit);
        const FVector3d Vertices[]={A+P.MinimumExtrusionCoordinate*P.ExtrusionAxis,
            B+P.MinimumExtrusionCoordinate*P.ExtrusionAxis,B+P.MaximumExtrusionCoordinate*P.ExtrusionAxis,
            A+P.MaximumExtrusionCoordinate*P.ExtrusionAxis};
        for (const auto& Vertex:Vertices)
        {
            const FVector3d D=Vertex-Face.Origin;
            const FVector2d Local(FVector3d::DotProduct(D,Face.AxisU),FVector3d::DotProduct(D,Face.AxisV));
            Face.DomainVertices.Add(Local);
            Face.HalfExtents.X=FMath::Max(Face.HalfExtents.X,FMath::Abs(Local.X));
            Face.HalfExtents.Y=FMath::Max(Face.HalfExtents.Y,FMath::Abs(Local.Y));
            Face.Bounds+=Vertex;
        }
        FAnalyticWorldData RawWorld;
        RawWorld.Planes.Add(Face);
        if (!TestTrue(TEXT("raw finite facet validates"),RawWorld.FinalizeAndValidate(&Reason)))
        { AddError(Reason);return false; }
        const FWorldHit Raw=Speed::FAnalyticStaticCollisionWorld(RawWorld).SweepSingle(Reflected);
        TestTrue(TEXT("exact query reaches the raw internal edge before joined-surface policy"),
            Raw.bHit && Raw.Normal.Z<-.9 && Raw.SurfaceFeatureKind==EContactFeatureKind::Edge);
        if (Raw.bHit) TestTrue(TEXT("raw facet reproduces captured time"),
            FMath::IsNearlyEqual(Raw.Time,.61955971192293469,1.e-9));
        FWorldQuery Probe=Reflected;
        Probe.Shape=EQueryShape::Sphere;Probe.Radius=1;
        const FVector3d Middle=Built.EvaluateSection(.5)+.5*(P.MinimumExtrusionCoordinate+P.MaximumExtrusionCoordinate)*P.ExtrusionAxis;
        // Reflect the inward side as a vector, not a cross product (axial
        // vectors change sign under an odd reflection).
        const FVector3d MiddleNormal=-FVector3d::CrossProduct(Base.EvaluateSectionDerivative(.5),Base.ExtrusionAxis).GetSafeNormal()*Mirror;
        Probe.Start=Middle+10*MiddleNormal;Probe.End=Middle-10*MiddleNormal;
        TestTrue(TEXT("filtered provider is active and a genuine interior face still collides"),
            Speed::FAnalyticStaticCollisionWorld(World).SweepSingle(Probe).bHit);
        const FWorldHit Hit=Speed::FAnalyticStaticCollisionWorld(World).SweepSingle(Reflected);
        AddInfo(FString::Printf(TEXT("ExactTerminalEdge Mirror=(%g,%g) Hit=%d Normal=(%.17g,%.17g,%.17g) TOI=%.17g"),
            Mirror.X,Mirror.Y,Hit.bHit,Hit.Normal.X,Hit.Normal.Y,Hit.Normal.Z,Hit.Time));
        TestFalse(TEXT("concave internal section edge must not become a downward wall"),
            Hit.bHit && Hit.Normal.Z<-.9);
        if (Hit.bHit) TestFalse(TEXT("repair does not create an initial overlap"),Hit.bStartPenetrating);
    }
    // Genuine boundaries must remain finite and blocking. A flat one-facet
    // patch also exercises the triangulation diagonal and its shared vertex.
    FAnalyticWorldData FlatWorld;
    FExtrudedQuinticPatch Flat;
    Flat.SourceId=1; Flat.SurfaceId=2; Flat.FeatureId=3; Flat.PrimitiveId=4;
    for (int32 I=0;I<6;++I) Flat.SectionControlPoints[I]=FVector3d(20*I,0,0);
    Flat.ExtrusionAxis=FVector3d::RightVector;
    Flat.MinimumExtrusionCoordinate=-25; Flat.MaximumExtrusionCoordinate=25;
    Flat.Bounds=FBox3d(FVector3d(0,-25,0),FVector3d(100,25,0));
    FlatWorld.ExtrudedQuinticPatches.Add(Flat);
    FString Reason;
    if (!TestTrue(TEXT("finite boundary control validates"),FlatWorld.FinalizeAndValidate(&Reason)))
    { AddError(Reason);return false; }
    const Speed::FAnalyticStaticCollisionWorld FlatCollision(FlatWorld);
    FWorldQuery Control;
    Control.Shape=EQueryShape::Box; Control.HalfExtent=FVector3d(2,2,2);
    Control.bIncludeCompactPatches=true;
    const FVector3d Starts[]={FVector3d(-3,0,1),FVector3d(103,0,1),
        FVector3d(50,-28,1),FVector3d(50,28,1),FVector3d(103,28,1),FVector3d(50,0,5)};
    const FVector3d Ends[]={FVector3d(3,0,1),FVector3d(97,0,1),
        FVector3d(50,-22,1),FVector3d(50,22,1),FVector3d(97,22,1),FVector3d(50,0,-5)};
    for (int32 I=0;I<UE_ARRAY_COUNT(Starts);++I)
    {
        Control.Start=Starts[I];Control.End=Ends[I];
        const FWorldHit Hit=FlatCollision.SweepSingle(Control);
        TestTrue(TEXT("terminal/extrusion/vertex/diagonal control retains collision"),Hit.bHit);
        if (Hit.bHit) TestTrue(TEXT("real boundary time remains geometric"),
            FMath::IsNearlyEqual(Hit.Time,I==5 ? .3 : 1./6.,1.e-8));
        Control.End=Control.Start;
        TestFalse(TEXT("control outside finite geometry remains clear"),FlatCollision.SweepSingle(Control).bHit);
    }
    // A convex arc remains a real obstacle, in both traversal directions.
    FAnalyticWorldData ConvexWorld;
    FExtrudedQuinticPatch Convex=Flat;
    for (int32 I=0;I<6;++I)
        Convex.SectionControlPoints[I]=FVector3d(-50+20*I,0,-25+20*I-5*I*(I-1));
    Convex.Bounds=FBox3d(FVector3d(-50,-25,-25),FVector3d(50,25,15));
    ConvexWorld.ExtrudedQuinticPatches.Add(Convex);
    if (!TestTrue(TEXT("convex arc validates"),ConvexWorld.FinalizeAndValidate(&Reason)))
    { AddError(Reason);return false; }
    for (const double Sign:{-1.,1.})
    {
        Control.Start=FVector3d(Sign*10,0,5);Control.End=FVector3d(Sign*8,0,-5);
        const FWorldHit Hit=Speed::FAnalyticStaticCollisionWorld(ConvexWorld).SweepSingle(Control);
        TestTrue(TEXT("convex section contact is not suppressed"),Hit.bHit);
        if (Hit.bHit) TestTrue(TEXT("convex contact remains outward and nonpenetrating"),
            Hit.Normal.Z>.9 && !Hit.bStartPenetrating);
    }
    return true;
}
#endif
