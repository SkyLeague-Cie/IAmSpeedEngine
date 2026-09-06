#if WITH_DEV_AUTOMATION_TESTS
#include "AnalyticWorldQuery.h"
#include "TensorContactTopology.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedTensorContactNormalTest,
    "IAmSpeed.AnalyticWorld.TensorInternalContactNormal",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedTensorContactNormalTest::RunTest(const FString& Parameters)
{
    using namespace Speed::Analytic;
    FAnalyticWorldData World;
    FPiecewiseTensorBezierPatch Patch;
    Patch.SourceId = 1; Patch.SurfaceId = 2; Patch.PrimitiveId = 3; Patch.CanonicalGroupId = 4;
    Patch.bQueryCollisionEnabled = true;
    Patch.BlockingChannels = MAX_uint64;
    // Frozen finite neighborhood covers the complete swept OBB bounds plus20cm.
    // It retains the exact tensor nets/domains and internal adjacency. An authored
    // cell boundary certified smooth is not a physical ridge of the whole patch.
#include "TensorContactNormalFixture.inl"
    World.PiecewiseTensorBezierPatches.Add(MoveTemp(Patch));
    FString Reason;
    if (!TestTrue(TEXT("finite smooth tensor neighborhood validates"), World.FinalizeAndValidate(&Reason)))
    { AddError(Reason); return false; }
    for (auto& P : World.PiecewiseTensorBezierPatches) P.bAuthorityEligible = true;
    FWorldQueryService Service(World);
    FWorldQuery Q;
    Q.bIncludeCompactPatches = true;
    Q.Shape = EQueryShape::Box;
    Q.Start = FVector3d(6440.740,-804.011,498.124);
    Q.End = FVector3d(6442.819,-801.798,501.156);
    Q.Rotation = FQuat4d(-.271854552,.254244180,-.512943330,.773527077).GetNormalized();
    Q.HalfExtent = FVector3d(78.790000915527344,52.299999237060547,19.139999389648438);
    const auto Hit = Service.Sweep(Q);
    if (!TestTrue(TEXT("nearby smooth wall is hit"), Hit.bHit)) return false;
    AddInfo(FString::Printf(TEXT("normal=%s point=%s depth=%.12g primitive=%016llX"),
        *Hit.Normal.ToString(), *Hit.Point.ToString(), Hit.PenetrationDepth, Hit.PrimitiveId));
    // The actual analytic tangent normal here is approximately(-.50,+.82,-.28).
    // A nearly downward facet-edge axis must not replace this smooth wall contact.
    TestTrue(TEXT("internal tensor edge does not masquerade as a downward ceiling"),
        Hit.Normal.Y > .7 && Hit.Normal.Z > -.5);
    for (const double MX : {-1.0,1.0})
    for (const double MY : {-1.0,1.0})
    {
        FAnalyticWorldData Mirrored = World;
        // Like the base fixture, validate geometric construction before the
        // test-only authority override; no external source certificate is faked.
        for (auto& P : Mirrored.PiecewiseTensorBezierPatches) P.bAuthorityEligible=false;
        for (auto& C : Mirrored.PiecewiseTensorBezierPatches[0].Cells)
        for (auto& Point : C.Surface.ControlPoints) Point *= FVector3d(MX,MY,1);
        if (!TestTrue(TEXT("reflected finite neighborhood rebuilds"),Mirrored.FinalizeAndValidate(&Reason)))
        { AddError(Reason); return false; }
        for (auto& P : Mirrored.PiecewiseTensorBezierPatches) P.bAuthorityEligible=true;
        FWorldQueryService MirroredService(Mirrored);
        for (const double Offset : {-.01,-.001,0.,.001,.01})
        {
            FWorldQuery Probe=Q;
            Probe.Start=(Q.Start+FVector3d(Offset,Offset,0))*FVector3d(MX,MY,1);
            Probe.End=(Q.End+FVector3d(Offset,Offset,0))*FVector3d(MX,MY,1);
            // Reflection conjugation R'=M R M; quaternion's vector part is axial.
            Probe.Rotation=FQuat4d(MY*Q.Rotation.X,MX*Q.Rotation.Y,MX*MY*Q.Rotation.Z,Q.Rotation.W);
            const FWorldHit H=MirroredService.Sweep(Probe);
            TestTrue(TEXT("mirror/perturbation keeps the finite wall contact"),H.bHit);
            if (!H.bHit) continue;
            TestTrue(TEXT("mirror/perturbation cannot create a downward ceiling"),MY*H.Normal.Y>.7 && H.Normal.Z>-.5);
            if (H.bStartPenetrating)
            {
                AddInfo(FString::Printf(TEXT("mirror=(%.0f,%.0f) offset=%.3f kind=%d normal=%s depth=%.12g witnessDepth=%.12g error=%.12g"),
                    MX,MY,Offset,int32(H.SurfaceFeatureKind),*H.Normal.ToString(),H.PenetrationDepth,
                    FVector3d::DotProduct(H.Point-H.QueryPoint,H.Normal),H.GeometricErrorBoundCm));
                TestTrue(TEXT("depth and both witnesses agree along the returned normal"),
                    FMath::Abs(FVector3d::DotProduct(H.Point-H.QueryPoint,H.Normal)-H.PenetrationDepth)<1e-6);
            }
        }
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedTensorConcavityTopologyTest,
    "IAmSpeed.AnalyticWorld.TensorConcavityTopology",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedTensorConcavityTopologyTest::RunTest(const FString& Parameters)
{
    using namespace Speed::Analytic;
    for (const double Curvature : {-1.0,1.0})
    {
        FPiecewiseTensorBezierPatch Patch;
        for (int32 Side=0; Side<2; ++Side)
        {
            auto& C=Patch.Cells.AddDefaulted_GetRef();
            C.PrimitiveId=Side+1; C.FeatureId=Side+11;
            C.MinimumU=.5*Side; C.MaximumU=.5*(Side+1);
            C.Surface.DegreeU=C.Surface.DegreeV=2;
            const double X0=10*(Side-1), X1=10*Side;
            const double Z[3]={X0*X0,X0*X1,X1*X1};
            for (int32 U=0; U<3; ++U)
            for (int32 V=0; V<3; ++V)
                C.Surface.ControlPoints.Add(FVector3d(X0+5*U,-10+10*V,Curvature*.02*Z[U]));
        }
        if (!TestTrue(TEXT("parabolic fixture builds"),Patch.BuildQueryApproximation())) return false;
        TestEqual(TEXT("join is certified in both directions"),Patch.Adjacencies.Num(),2);
        const auto CheckMasks=[this,Curvature](const FPiecewiseTensorBezierPatch& P, const bool bJoined)
        {
            int32 SeamEdges=0;
            for (int32 Side=0; Side<2; ++Side)
            for (const auto& C : P.Cells[Side].ApproximationCells)
            {
                const int32 T=Side==0 ? 0 : 1, E=Side==0 ? 1 : 2;
                if ((Side==0 && C.MaximumU==1) || (Side==1 && C.MinimumU==0))
                {
                    ++SeamEdges;
                    const bool Positive=(C.PositiveConcaveEdges[T] & (1u<<E))!=0;
                    const bool Negative=(C.NegativeConcaveEdges[T] & (1u<<E))!=0;
                    TestEqual(TEXT("only concave side removes the internal edge"),Positive,bJoined && Curvature>0);
                    TestEqual(TEXT("convex side retains SAT"),Negative,bJoined && Curvature<0);
                }
                if (Side==0 && C.MinimumU==0)
                    TestEqual(TEXT("free U-min border is not internal"),int32((C.PositiveConcaveEdges[1]|C.NegativeConcaveEdges[1])&4),0);
                if (Side==1 && C.MaximumU==1)
                    TestEqual(TEXT("free U-max border is not internal"),int32((C.PositiveConcaveEdges[0]|C.NegativeConcaveEdges[0])&2),0);
            }
            TestTrue(TEXT("nonempty seam actually tested"),SeamEdges>0);
        };
        CheckMasks(Patch,true);
        Patch.Adjacencies.Reset();
        BuildTensorContactTopology(Patch);
        CheckMasks(Patch,false);
    }
    return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedTensorVertexNormalTest,
    "IAmSpeed.AnalyticWorld.TensorInteriorVertexNormal",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedTensorVertexNormalTest::RunTest(const FString& Parameters)
{
    using namespace Speed::Analytic;
    FPiecewiseTensorBezierPatch Patch;
    Patch.SourceId=1; Patch.SurfaceId=2; Patch.PrimitiveId=3; Patch.CanonicalGroupId=4;
    Patch.bQueryCollisionEnabled=true; Patch.BlockingChannels=MAX_uint64;
#include "TensorVertexNormalFixture.inl"
    FAnalyticWorldData World;
    World.PiecewiseTensorBezierPatches.Add(MoveTemp(Patch));
    FString Reason;
    if (!TestTrue(TEXT("second finite neighborhood validates"),World.FinalizeAndValidate(&Reason)))
    { AddError(Reason); return false; }
    for (auto& P : World.PiecewiseTensorBezierPatches) P.bAuthorityEligible=true;
    FWorldQuery Q;
    Q.bIncludeCompactPatches=true; Q.Shape=EQueryShape::Box;
    Q.Start=FVector3d(6444.159,797.676,296.772);
    Q.End=FVector3d(6445.976,795.224,293.158);
    Q.Rotation=FQuat4d(.784063107,.498359486,.257178230,.265974107).GetNormalized();
    Q.HalfExtent=FVector3d(78.790000915527344,52.299999237060547,19.139999389648438);
    const FWorldHit H=FWorldQueryService(World).Sweep(Q);
    if (!TestTrue(TEXT("second smooth wall is hit"),H.bHit)) return false;
    AddInfo(FString::Printf(TEXT("kind=%d normal=%s point=%s depth=%.12g primitive=%016llX"),
        int32(H.SurfaceFeatureKind),*H.Normal.ToString(),*H.Point.ToString(),H.PenetrationDepth,H.PrimitiveId));
    for (const auto& C : World.PiecewiseTensorBezierPatches[0].Cells)
    for (int32 I=0; I<C.ApproximationCells.Num(); ++I)
    for (int32 T=0; T<2; ++T)
    {
        if (CombineStableIds(C.PrimitiveId,uint64(2*I+T+1))!=H.PrimitiveId) continue;
        const auto& A=C.ApproximationCells[I];
        AddInfo(FString::Printf(TEXT("cell=%016llX approximate=%d tri=%d UV=(%.9g,%.9g)-(%.9g,%.9g) mask=(%d,%d)"),
            C.PrimitiveId,I,T,A.MinimumU,A.MinimumV,A.MaximumU,A.MaximumV,
            int32(A.PositiveConcaveEdges[T]),int32(A.NegativeConcaveEdges[T])));
    }
    TestTrue(TEXT("internal smooth-wall witness cannot oppose the surface normal"),H.Normal.Y<-.6 && H.Normal.Z<.6);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedTensorCornerConeTopologyTest,
    "IAmSpeed.AnalyticWorld.TensorCornerConeTopology",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedTensorCornerConeTopologyTest::RunTest(const FString& Parameters)
{
    using namespace Speed::Analytic;
    // Plane, elliptic bowl and saddle: the normal envelope is geometric, not
    // a hard-coded slope/angle cutoff, and does not assume positive curvature.
    for (const double Curvature : {0.,.02,-.02})
    {
        FPiecewiseTensorBezierPatch P;
        P.SourceId=1; P.SurfaceId=2; P.PrimitiveId=3; P.CanonicalGroupId=4;
        for (int32 U=0; U<2; ++U)
        for (int32 V=0; V<2; ++V)
        {
            auto& C=P.Cells.AddDefaulted_GetRef();
            C.PrimitiveId=1+2*U+V; C.FeatureId=11+2*U+V;
            C.MinimumU=.5*U; C.MaximumU=.5*(U+1);
            C.MinimumV=.5*V; C.MaximumV=.5*(V+1);
            C.Surface.DegreeU=C.Surface.DegreeV=2;
            const double X0=10*(U-1),X1=10*U,Y0=10*(V-1),Y1=10*V;
            const double XX[3]={X0*X0,X0*X1,X1*X1},YY[3]={Y0*Y0,Y0*Y1,Y1*Y1};
            for (int32 I=0; I<3; ++I)
            for (int32 J=0; J<3; ++J)
                C.Surface.ControlPoints.Add(FVector3d(X0+5*I,Y0+5*J,FMath::Abs(Curvature)*XX[I]+Curvature*YY[J]));
        }
        if (!TestTrue(TEXT("complete four-cell star builds"),P.BuildQueryApproximation())) return false;
        TestEqual(TEXT("one shared cone for one internal vertex"),P.InternalCornerNormalCones.Num(),1);
        for (int32 C=0; C<4; ++C)
        for (int32 V=0; V<4; ++V)
            TestEqual(TEXT("only center corner is certified, exterior corners stay open"),
                P.Cells[C].InternalCornerNormalCone[V],V==3-C ? 0 : int32(INDEX_NONE));
        if (!P.InternalCornerNormalCones.IsEmpty())
        {
            const auto Cone=P.InternalCornerNormalCones[0];
            TestTrue(TEXT("symmetric geometric envelope contains the true normal"),Cone.Z>=Cone.W-1e-12);
            TestTrue(TEXT("envelope is a finite convex cone"),Cone.W>0 && Cone.W<=1);
        }
        FAnalyticWorldData World;
        World.PiecewiseTensorBezierPatches.Add(P);
        const uint64 Hash=World.StableHash();
        World.PiecewiseTensorBezierPatches[0].InternalCornerNormalCones.Reset();
        for (auto& C : World.PiecewiseTensorBezierPatches[0].Cells)
            for (auto& Index : C.InternalCornerNormalCone) Index=INDEX_NONE;
        TestEqual(TEXT("derived topology is excluded from the canonical geometry hash"),World.StableHash(),Hash);
        // Both directed records describe the same geometric certificate.
        // Remove one complete interface to test an actual open fan.
        const auto Pair=P.Adjacencies[0];
        P.Adjacencies.RemoveAll([&Pair](const auto& A)
        {
            return (A.CellPrimitiveId==Pair.CellPrimitiveId && A.AdjacentCellPrimitiveId==Pair.AdjacentCellPrimitiveId) ||
                (A.CellPrimitiveId==Pair.AdjacentCellPrimitiveId && A.AdjacentCellPrimitiveId==Pair.CellPrimitiveId);
        });
        BuildTensorContactTopology(P);
        TestEqual(TEXT("an incomplete C2 ring gets no normal cone"),P.InternalCornerNormalCones.Num(),0);
    }
    return true;
}
#endif
