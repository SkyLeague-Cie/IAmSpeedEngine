#if WITH_DEV_AUTOMATION_TESTS
#include "AnalyticWorldQuery.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedBoxPoseQueryToleranceTest,
    "IAmSpeed.AnalyticWorld.BoxPoseQueryTolerance",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedBoxPoseQueryToleranceTest::RunTest(const FString& Parameters)
{
    using namespace Speed::Analytic;
    FAnalyticWorldData Data;
    FPiecewiseTensorBezierPatch Patch;
    Patch.SourceId=1; Patch.SurfaceId=2; Patch.PrimitiveId=3; Patch.CanonicalGroupId=4;
    Patch.BlockingChannels=MAX_uint64;
    auto& Cell=Patch.Cells.AddDefaulted_GetRef();
    Cell.PrimitiveId=4; Cell.FeatureId=5;
    Cell.MinimumU=.39583333333333331; Cell.MaximumU=.40625;
    Cell.MinimumV=.73333333333333328; Cell.MaximumV=.75;
    Cell.Surface.DegreeU=Cell.Surface.DegreeV=3;
    // Frozen actual finite cell of the residual-overlap witness. Query pose is
    // rounded only to the published diagnostic precision; no stadium UObject.
    Cell.Surface.ControlPoints={
        {-6579.3923616616166,-131.16228027343755,551.03462526041653},
        {-6577.6828016129612,-131.16228027343755,555.02782912258544},
        {-6575.8270952804305,-131.16228027343755,559.02103298475447},
        {-6573.936957255637,-131.16228027343755,563.0142368469235},
        {-6579.3923555011015,-126.79020426432298,551.03462526041653},
        {-6577.6828034954351,-126.79020426432298,555.02782912258544},
        {-6575.8271462245893,-126.79020426432298,559.02103298475436},
        {-6573.9370016888179,-126.79020426432298,563.0142368469235},
        {-6579.3923493405855,-122.4181282552084,551.03462526041676},
        {-6577.6828053680829,-122.4181282552084,555.02782912258567},
        {-6575.8271971320746,-122.4181282552084,559.02103298475436},
        {-6573.9370461219978,-122.4181282552084,563.0142368469235},
        {-6579.3923431800704,-118.04605224609384,551.03462526041676},
        {-6577.6828072433809,-118.04605224609384,555.02782912258567},
        {-6575.8272480494497,-118.04605224609384,559.02103298475447},
        {-6573.9370905551787,-118.04605224609384,563.01423684692361}};
    Data.PiecewiseTensorBezierPatches.Add(MoveTemp(Patch));
    FString Reason;
    if (!TestTrue(TEXT("finite overlap fixture validates"),Data.FinalizeAndValidate(&Reason)))
    { AddError(Reason); return false; }
    Data.PiecewiseTensorBezierPatches[0].bAuthorityEligible=true;
    FWorldQueryService Service(Data);
    FWorldQuery Q;
    Q.Shape=EQueryShape::Box;
    Q.Start=Q.End=FVector3d(-6503.736,-181.861,553.182);
    Q.Rotation=FQuat4d(-.301802895,-.093260674,-.532991639,.784944184).GetNormalized();
    Q.HalfExtent=FVector3d(78.790000915527344,52.299999237060547,19.139999389648438);
    Q.bAuthorityOnly=Q.bIncludeCompactPatches=true;
    for (const double Tolerance : {0.,1.e-6})
    {
        Q.DomainTolerance=Tolerance;
        FWorldHit Hit;
        TestTrue(TEXT("stationary operation is available"),Service.TryFindDeepestOverlap(Q,Hit));
        AddInfo(FString::Printf(TEXT("domainTolerance=%.17g hit=%d depth=%.17g normal=%s"),
            Tolerance,Hit.bHit?1:0,Hit.PenetrationDepth,*Hit.Normal.ToString()));
        TestTrue(TEXT("finite overlap is not lost at zero domain tolerance"),
            Hit.bStartPenetrating && Hit.PenetrationDepth>.3);
    }
    return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedBoxPoseJoinedQueryToleranceTest,
    "IAmSpeed.AnalyticWorld.BoxPoseJoinedQueryTolerance",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedBoxPoseJoinedQueryToleranceTest::RunTest(const FString& Parameters)
{
    using namespace Speed::Analytic;
    FAnalyticWorldData Data;
    FPiecewiseTensorBezierPatch Patch;
    Patch.SourceId=1; Patch.SurfaceId=2; Patch.PrimitiveId=3; Patch.CanonicalGroupId=4;
    Patch.BlockingChannels=MAX_uint64;
#include "BoxPoseQueryToleranceFixture.inl"
    Data.PiecewiseTensorBezierPatches.Add(MoveTemp(Patch));
    FString Reason;
    if (!TestTrue(TEXT("joined finite fixture validates"),Data.FinalizeAndValidate(&Reason)))
    { AddError(Reason); return false; }
    Data.PiecewiseTensorBezierPatches[0].bAuthorityEligible=true;
    FWorldQueryService Service(Data);
    FWorldQuery Q;
    Q.Shape=EQueryShape::Box;
    Q.Start=Q.End=FVector3d(-6503.7357183949534,-181.8611131071973,553.18182629799776);
    Q.Rotation=FQuat4d(-.30180289472588029,-.093260673889108303,-.53299163898678303,.78494418413732558);
    Q.HalfExtent=FVector3d(78.790000915527344,52.299999237060547,19.139999389648438);
    Q.bAuthorityOnly=Q.bIncludeCompactPatches=true;
    for (const double Tolerance : {0.,1.e-6})
    {
        Q.DomainTolerance=Tolerance;
        FWorldHit Hit;
        TestTrue(TEXT("stationary joined operation is available"),Service.TryFindDeepestOverlap(Q,Hit));
        AddInfo(FString::Printf(TEXT("joined domainTolerance=%.17g hit=%d depth=%.17g normal=%s"),
            Tolerance,Hit.bHit?1:0,Hit.PenetrationDepth,*Hit.Normal.ToString()));
        TestTrue(TEXT("internal face overlap survives zero domain allowance"),
            Hit.bStartPenetrating && Hit.PenetrationDepth>.3);
    }
    // Test the numerical law, not just one world-coordinate bit pattern.
    // Reflection conjugates the quaternion; translation changes cancellation
    // in the projection/reconstruction without changing finite geometry.
    for (const double MX : {-1.,1.})
    for (const double MY : {-1.,1.})
    for (const FVector3d Translation : {FVector3d::ZeroVector,-Q.Start,FVector3d(1.e6,-1.e6,1.e6)})
    {
        const FVector3d Mirror(MX,MY,1.);
        FAnalyticWorldData Transformed=Data;
        auto& P=Transformed.PiecewiseTensorBezierPatches[0];
        P.bAuthorityEligible=false;
        for (auto& C : P.Cells)
        for (auto& Point : C.Surface.ControlPoints) Point=Point*Mirror+Translation;
        if (!TestTrue(TEXT("transformed neighborhood validates"),Transformed.FinalizeAndValidate(&Reason)))
        { AddError(Reason); return false; }
        P.bAuthorityEligible=true;
        FWorldQueryService TransformedService(Transformed);
        for (const double Offset : {-.001,0.,.001})
        {
            FWorldQuery Probe=Q;
            Probe.Start=Probe.End=(Q.Start+FVector3d(Offset,Offset,0))*Mirror+Translation;
            Probe.Rotation=FQuat4d(MY*Q.Rotation.X,MX*Q.Rotation.Y,MX*MY*Q.Rotation.Z,Q.Rotation.W);
            FWorldHit Tight,Ordinary;
            Probe.DomainTolerance=0;
            TestTrue(TEXT("tight transformed query available"),TransformedService.TryFindDeepestOverlap(Probe,Tight));
            Probe.DomainTolerance=1.e-6;
            TestTrue(TEXT("ordinary transformed query available"),TransformedService.TryFindDeepestOverlap(Probe,Ordinary));
            TestTrue(TEXT("finite penetration survives translation/reflection/perturbation"),
                Tight.bStartPenetrating && Tight.PenetrationDepth>.3 && Tight.PenetrationDepth<.5);
            TestTrue(TEXT("domain allowance cannot hide the interior overlap"),
                Tight.PenetrationDepth==Ordinary.PenetrationDepth && Tight.Normal==Ordinary.Normal);
            // Far beyond the finite patch: the arithmetic bound must not turn
            // its smooth supporting plane into infinite physical collision.
            Probe.Start=Probe.End=Probe.Start+FVector3d(0,1000,0);
            Probe.DomainTolerance=0;
            FWorldHit Outside;
            TestTrue(TEXT("finite exterior query available"),TransformedService.TryFindDeepestOverlap(Probe,Outside));
            TestFalse(TEXT("finite patch exterior remains clear"),Outside.bHit);
        }
    }
    return true;
}
#endif
