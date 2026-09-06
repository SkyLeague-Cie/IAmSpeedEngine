#if WITH_DEV_AUTOMATION_TESTS
#include "AnalyticWorldQuery.h"
#include "IAmSpeed/World/Collision/StaticCollisionWorld.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedBoxFiniteDomainTest,
    "IAmSpeed.AnalyticWorld.BoxFiniteContactDomain",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedBoxFiniteDomainTest::RunTest(const FString& Parameters)
{
    using namespace Speed::Analytic;
    FAnalyticWorldData World;
    FExtrudedQuinticPatch Patch;
    Patch.SourceId=1; Patch.SurfaceId=2; Patch.FeatureId=3; Patch.PrimitiveId=4;
    for (int32 I=0; I<6; ++I)
        Patch.SectionControlPoints[I]=FVector3d(-50+20*I,0,0);
    Patch.ExtrusionAxis=FVector3d::RightVector;
    Patch.MinimumExtrusionCoordinate=-25; Patch.MaximumExtrusionCoordinate=25;
    Patch.Bounds=FBox3d(FVector3d(-50,-25,0),FVector3d(50,25,0));
    World.ExtrudedQuinticPatches.Add(Patch);
    FString Reason;
    if (!TestTrue(TEXT("finite box fixture validates"),World.FinalizeAndValidate(&Reason)))
    { AddError(Reason); return false; }
    World.ExtrudedQuinticPatches[0].bAuthorityEligible=true;
    const Speed::FAnalyticStaticCollisionWorld Strict(World);
    const FWorldQueryService Conservative(World);
    FWorldQuery Q;
    Q.Shape=EQueryShape::Box; Q.bIncludeCompactPatches=true;
    Q.bAuthorityOnly=true; Q.InitialOverlapTolerance=0;
    Q.HalfExtent=FVector3d(5,4,3);
    // A finite SAT proof of intersection is not a certificate that the whole
    // query footprint can replace Legacy. Eroding the face loses real contacts
    // near an incident provider, then suddenly reports a deep overlap later.
    for (const FVector3d Position : {FVector3d(49,0,2),FVector3d(-49,0,2),
                                    FVector3d(0,24,2),FVector3d(0,-24,2)})
    {
        Q.Start=Q.End=Position;
        const FWorldHit Hit=Strict.SweepSingle(Q);
        TestTrue(TEXT("strict finite face retains a partially overhanging box"),Hit.bHit);
        if (Hit.bHit)
        {
            TestTrue(TEXT("face overlap has exact depth"),FMath::IsNearlyEqual(Hit.PenetrationDepth,1.,1.e-8));
            TestTrue(TEXT("finite box witnesses agree with the MTD"),
                (Hit.Point-Hit.QueryPoint).Equals(Hit.PenetrationDepth*Hit.Normal,1.e-8));
        }
        FWorldHit OriginalReplacement, StrictInputReplacement;
        const bool bOriginalCoverage=Conservative.TrySweepAuthority(Q,OriginalReplacement);
        Q.bUseFiniteContactDomain=true;
        const bool bStrictInputCoverage=Conservative.TrySweepAuthority(Q,StrictInputReplacement);
        TestEqual(TEXT("incoming strict policy cannot change Hybrid coverage"),bStrictInputCoverage,bOriginalCoverage);
        TestEqual(TEXT("incoming strict policy cannot change the Hybrid winner"),StrictInputReplacement.bHit,OriginalReplacement.bHit);
        if (OriginalReplacement.bHit&&StrictInputReplacement.bHit)
        {
            TestEqual(TEXT("Hybrid time remains exact"),StrictInputReplacement.Time,OriginalReplacement.Time);
            TestEqual(TEXT("Hybrid depth remains exact"),StrictInputReplacement.PenetrationDepth,OriginalReplacement.PenetrationDepth);
        }
        Q.bUseFiniteContactDomain=false;
        Q.Start.Z=6; Q.End.Z=0;
        const FWorldHit CCD=Strict.SweepSingle(Q);
        TestTrue(TEXT("CCD reaches the finite face before any overlap projection"),CCD.bHit);
        if (CCD.bHit) TestTrue(TEXT("finite face CCD has the exact time"),FMath::IsNearlyEqual(CCD.Time,.5,1.e-8));
    }
    for (const double Sign : {-1.,1.})
    {
        Q.Start=Q.End=FVector3d(Sign*56,0,2);
        TestFalse(TEXT("a box beyond the real section domain does not hit an infinite face"),Strict.SweepSingle(Q).bHit);
        Q.End.X=Sign*44;
        const FWorldHit Edge=Strict.SweepSingle(Q);
        TestTrue(TEXT("finite SAT retains first contact with the real terminal edge"),Edge.bHit);
        if (Edge.bHit)
            TestTrue(TEXT("terminal edge entry is not delayed until a full footprint fits"),FMath::IsNearlyEqual(Edge.Time,1./12.,1.e-8));
        Q.Start=Q.End=FVector3d(0,Sign*30,2);
        TestFalse(TEXT("box outside the true extrusion domain misses"),Strict.SweepSingle(Q).bHit);
        Q.End.Y=Sign*18;
        const FWorldHit ExtrusionEdge=Strict.SweepSingle(Q);
        TestTrue(TEXT("finite SAT retains first extrusion-edge contact"),ExtrusionEdge.bHit);
        if (ExtrusionEdge.bHit)
            TestTrue(TEXT("extrusion-edge time is not coverage-eroded"),FMath::IsNearlyEqual(ExtrusionEdge.Time,1./12.,1.e-8));
    }
    Q.Start=Q.End=FVector3d(54,29,2);
    Q.HalfExtent=FVector3d(5,1,3);
    Q.Rotation=FQuat4d(FVector3d::UpVector,UE_DOUBLE_PI/4);
    TestFalse(TEXT("overlapping AABBs do not replace the finite OBB SAT proof"),Strict.SweepSingle(Q).bHit);
    FAnalyticWorldData JoinedWorld;
    for (int32 Side=0; Side<2; ++Side)
    {
        FExtrudedQuinticPatch Joined=Patch;
        Joined.PrimitiveId=10+Side;
        for (int32 I=0; I<6; ++I)
            Joined.SectionControlPoints[I]=FVector3d(-50+50*Side+10*I,0,0);
        Joined.Bounds=FBox3d(FVector3d(-50+50*Side,-25,0),FVector3d(50*Side,25,0));
        JoinedWorld.ExtrudedQuinticPatches.Add(Joined);
    }
    if (!TestTrue(TEXT("coplanar joined fixture validates"),JoinedWorld.FinalizeAndValidate(&Reason)))
    { AddError(Reason); return false; }
    for (auto& Joined : JoinedWorld.ExtrudedQuinticPatches) Joined.bAuthorityEligible=true;
    const Speed::FAnalyticStaticCollisionWorld JoinedStrict(JoinedWorld);
    Q.HalfExtent=FVector3d(5,4,3); Q.Rotation=FQuat4d::Identity;
    for (const double Sign : {-1.,1.})
    {
        Q.Start=FVector3d(-10*Sign,0,3); Q.End=FVector3d(10*Sign,0,3);
        const FWorldHit SeamHit=JoinedStrict.SweepSingle(Q);
        TestTrue(TEXT("a coplanar provider junction is not a tangent wall"),
            !SeamHit.bHit || (FMath::Abs(SeamHit.Normal.X)<=1.e-8 && SeamHit.PenetrationDepth<=1.e-8));
    }
    // Frozen curved extrusion: a legitimate overlap was hidden until the
    // selected witness happened to enter the eroded coverage domain. This is
    // a finite SAT intersection test, not an expectation about the MTD normal.
    FAnalyticWorldData CurvedWorld;
    const FVector3d Controls[6]={
        {5074.417201043741,0,-.0035156249068677425},
        {5222.085249425973,0,-.0035156249068677425},
        {5369.753297808205,0,-.0035156249068677425},
        {5569.922208854623,0,179.87388305553776},
        {5569.922208854623,0,312.5725337718541},
        {5569.922208854623,0,445.2711844881705}};
    for (int32 I=0; I<6; ++I) Patch.SectionControlPoints[I]=Controls[I];
    Patch.ExtrusionAxis=FVector3d(.00019345588954042844,.9999999329565996,.0003109045111583612);
    Patch.MinimumExtrusionCoordinate=949.9999363087696;
    Patch.MaximumExtrusionCoordinate=3039.7103290137034;
    Patch.Bounds=FBox3d(FVector3d(5074.600984,949.999872,.291843),FVector3d(5570.510259,3039.710330,446.216245));
    CurvedWorld.ExtrudedQuinticPatches.Add(Patch);
    if (!TestTrue(TEXT("curved box fixture validates"),CurvedWorld.FinalizeAndValidate(&Reason)))
    { AddError(Reason); return false; }
    CurvedWorld.ExtrudedQuinticPatches[0].bAuthorityEligible=true;
    const Speed::FAnalyticStaticCollisionWorld CurvedStrict(CurvedWorld);
    Q.HalfExtent=FVector3d(78.790,52.300,19.140);
    Q.Start=Q.End=FVector3d(5556.413,959.098,315.143);
    Q.Rotation=FQuat4d(.281755538,-.384291846,.610677199,.632461029).GetNormalized();
    const FWorldHit CurvedHit=CurvedStrict.SweepSingle(Q);
    TestTrue(TEXT("finite curved SAT intersection cannot be hidden by replacement coverage"),
        CurvedHit.bHit&&CurvedHit.bStartPenetrating);
    return true;
}
#endif
