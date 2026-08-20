#include "AnalyticLandscapeAdapter.h"
#include "AnalyticWorldQuery.h"
#include "IAmSpeed/World/StaticCollisionWorld.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FIAmSpeedAnalyticBoundedPlaneTest,
	"IAmSpeed.AnalyticWorld.BoundedPlane",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedAnalyticBoundedPlaneTest::RunTest(const FString& Parameters)
{
	using namespace Speed::Analytic;

	FAnalyticWorldData World;
	FBoundedPlane Plane;
	Plane.SurfaceId = 10;
	Plane.FeatureId = 11;
	Plane.Origin = FVector3d::ZeroVector;
	Plane.HalfExtents = FVector2d(100.0, 50.0);
	World.Planes.Add(Plane);
	FBoundedPlane AdjacentPlane = Plane;
	AdjacentPlane.PrimitiveId = 1;
	AdjacentPlane.Origin.X = 400.0;
	World.Planes.Add(AdjacentPlane);
	FString ValidationReason;
	TestTrue(TEXT("Planes may share a semantic feature when primitives differ"),
		World.FinalizeAndValidate(&ValidationReason));
	TestEqual(TEXT("Bounded plane count"), World.Planes.Num(), 2);

	FWorldQuery SphereQuery;
	SphereQuery.Shape = EQueryShape::Sphere;
	SphereQuery.Start = FVector3d(0.0, 0.0, 20.0);
	SphereQuery.End = FVector3d(0.0, 0.0, -20.0);
	SphereQuery.Radius = 5.0;
	TestFalse(TEXT("Draft plane is not authority coverage"),
		FWorldQueryService(World).HasAuthorityCoverage(SphereQuery));
	FWorldQuery AuthorityOnlyQuery = SphereQuery;
	AuthorityOnlyQuery.bAuthorityOnly = true;
	TestFalse(TEXT("Authority query excludes draft plane"),
		FWorldQueryService(World).Sweep(AuthorityOnlyQuery).bHit);
	const FWorldHit SphereHit = FWorldQueryService(World).Sweep(SphereQuery);
	TestTrue(TEXT("Sphere sweep hits plane"), SphereHit.bHit);
	TestTrue(TEXT("Sphere sweep normalized time"),
		FMath::IsNearlyEqual(SphereHit.Time, 0.375, 1.0e-12));
	TestTrue(TEXT("Sphere sweep center"),
		SphereHit.Location.Equals(FVector3d(0.0, 0.0, 5.0), 1.0e-12));
	TestTrue(TEXT("Sphere sweep contact point"),
		SphereHit.Point.Equals(FVector3d::ZeroVector, 1.0e-12));
	FWorldQuery BackFaceQuery = SphereQuery;
	BackFaceQuery.Start.Z = -20.0;
	BackFaceQuery.End.Z = 20.0;
	const FWorldHit BackFaceHit = FWorldQueryService(World).Sweep(BackFaceQuery);
	TestTrue(TEXT("Bounded planes collide from the back face"), BackFaceHit.bHit);
	TestTrue(TEXT("Back-face normal points toward the query"),
		BackFaceHit.Normal.Equals(-FVector3d::UpVector, 1.0e-12));
	TestTrue(TEXT("Back-face sweep does not treat the entire back half-space as penetration"),
		!BackFaceHit.bStartPenetrating);

	FWorldQuery BoxQuery;
	BoxQuery.Shape = EQueryShape::Box;
	BoxQuery.Start = FVector3d(0.0, 0.0, 20.0);
	BoxQuery.End = FVector3d(0.0, 0.0, -20.0);
	BoxQuery.HalfExtent = FVector3d(2.0, 3.0, 4.0);
	const FWorldHit BoxHit = FWorldQueryService(World).Sweep(BoxQuery);
	TestTrue(TEXT("Box sweep hits plane"), BoxHit.bHit);
	TestTrue(TEXT("Box support radius controls TOI"),
		FMath::IsNearlyEqual(BoxHit.Time, 0.4, 1.0e-12));

	FWorldQuery OutsideQuery = SphereQuery;
	OutsideQuery.Start.X = 200.0;
	OutsideQuery.End.X = 200.0;
	TestFalse(TEXT("Bounded domain rejects outside contact"),
		FWorldQueryService(World).Sweep(OutsideQuery).bHit);
	OutsideQuery.Start.X = 1000.0;
	OutsideQuery.End.X = 1000.0;
	TestFalse(TEXT("Far queries are outside compact coverage"),
		FWorldQueryService(World).HasAuthorityCoverage(OutsideQuery));

	World.Planes[0].bAuthorityEligible = true;
	const Speed::FAnalyticStaticCollisionWorld StaticWorld(World);
	TestEqual(TEXT("Native static world reports the analytical backend"),
		StaticWorld.GetBackend(), Speed::EStaticCollisionBackend::SurfaceAnalytic);
	TestTrue(TEXT("Native static-world sweep returns an IAmSpeed hit"),
		StaticWorld.SweepSingle(AuthorityOnlyQuery).bHit);
	TestTrue(TEXT("Certified plane hit is hybrid authority coverage"),
		StaticWorld.HasHybridWinningHit(SphereQuery));
	FWorldQuery DomainNearMiss = SphereQuery;
	DomainNearMiss.Start.X = 104.0;
	DomainNearMiss.End.X = 104.0;
	TestFalse(TEXT("Intersecting provider bounds without a hit are not hybrid coverage"),
		StaticWorld.HasHybridWinningHit(DomainNearMiss));

	FAnalyticWorldData ProviderWorld;
	FBoundedPlane CertifiedPlane = Plane;
	CertifiedPlane.SourceId = 100;
	CertifiedPlane.SurfaceId = 100;
	CertifiedPlane.PrimitiveId = 100;
	CertifiedPlane.bAuthorityEligible = true;
	ProviderWorld.Planes.Add(CertifiedPlane);
	FBoundedPlane NearerDraftPlane = CertifiedPlane;
	NearerDraftPlane.SourceId = 200;
	NearerDraftPlane.SurfaceId = 200;
	NearerDraftPlane.PrimitiveId = 200;
	NearerDraftPlane.Origin.Z = 10.0;
	NearerDraftPlane.bAuthorityEligible = false;
	ProviderWorld.Planes.Add(NearerDraftPlane);
	TestTrue(TEXT("Provider-selection world validates"),
		ProviderWorld.FinalizeAndValidate(&ValidationReason));
	TestFalse(TEXT("Nearer draft provider blocks hybrid authority selection"),
		Speed::FAnalyticStaticCollisionWorld(ProviderWorld).
			HasHybridWinningHit(SphereQuery));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FIAmSpeedAnalyticExtrudedQuinticTest,
	"IAmSpeed.AnalyticWorld.ExtrudedQuintic",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedAnalyticExtrudedQuinticTest::RunTest(const FString& Parameters)
{
	using namespace Speed::Analytic;

	FAnalyticWorldData World;
	FExtrudedQuinticPatch Patch;
	Patch.SourceId = 30;
	Patch.SurfaceId = 31;
	Patch.FeatureId = 32;
	Patch.PrimitiveId = 33;
	Patch.ObjectType = 2;
	Patch.BlockingChannels = 1ull << 3;
	for (int32 Index = 0; Index < 6; ++Index)
	{
		Patch.SectionControlPoints[Index] =
			FVector3d(-50.0 + 20.0 * Index, 0.0, 0.0);
	}
	Patch.ExtrusionAxis = FVector3d::RightVector;
	Patch.MinimumExtrusionCoordinate = -25.0;
	Patch.MaximumExtrusionCoordinate = 25.0;
	Patch.Bounds = FBox3d(
		FVector3d(-50.0, -25.0, 0.0),
		FVector3d(50.0, 25.0, 0.0));
	Patch.bQueryCollisionEnabled = true;
	World.ExtrudedQuinticPatches.Add(Patch);
	FString ValidationReason;
	TestTrue(TEXT("Extruded quintic world validates"),
		World.FinalizeAndValidate(&ValidationReason));
	TestTrue(TEXT("Quintic midpoint is exact"),
		Patch.EvaluateSection(0.5).Equals(FVector3d::ZeroVector, 1.0e-12));
	TestTrue(TEXT("Quintic derivative is regular"),
		Patch.EvaluateSectionDerivative(0.5).Equals(
			FVector3d(100.0, 0.0, 0.0), 1.0e-12));
	FExtrudedQuinticPatch CorrectedPatch = Patch;
	CorrectedPatch.InteriorCorrectionControlPoints[0] =
		FVector3d(0.0, 0.0, 10.0);
	CorrectedPatch.InteriorCorrectionControlPoints[1] =
		FVector3d(0.0, 0.0, -5.0);
	TestTrue(TEXT("Interior correction preserves start position"),
		CorrectedPatch.EvaluateSection(0.0).Equals(
			Patch.EvaluateSection(0.0), 1.0e-12));
	TestTrue(TEXT("Interior correction preserves end position"),
		CorrectedPatch.EvaluateSection(1.0).Equals(
			Patch.EvaluateSection(1.0), 1.0e-12));
	TestTrue(TEXT("Interior correction preserves start derivative"),
		CorrectedPatch.EvaluateSectionDerivative(0.0).Equals(
			Patch.EvaluateSectionDerivative(0.0), 1.0e-12));
	TestTrue(TEXT("Interior correction preserves end derivative"),
		CorrectedPatch.EvaluateSectionDerivative(1.0).Equals(
			Patch.EvaluateSectionDerivative(1.0), 1.0e-12));
	TestTrue(TEXT("Interior correction changes only the interior"),
		!CorrectedPatch.EvaluateSection(0.5).Equals(
			Patch.EvaluateSection(0.5), 1.0e-12));

	FWorldQuery Query;
	Query.Shape = EQueryShape::Sphere;
	Query.Start = FVector3d(0.0, 0.0, 20.0);
	Query.End = FVector3d(0.0, 0.0, -20.0);
	Query.Radius = 5.0;
	Query.bApplyCollisionFilter = true;
	Query.TraceChannel = 3;
	TestFalse(TEXT("Compact patches remain opt-in"),
		FWorldQueryService(World).Sweep(Query).bHit);
	Query.bIncludeCompactPatches = true;
	const FWorldHit Hit = FWorldQueryService(World).Sweep(Query);
	TestTrue(TEXT("Sphere hits extruded quintic face"), Hit.bHit);
	TestTrue(TEXT("Extruded quintic face TOI"),
		FMath::IsNearlyEqual(Hit.Time, 0.375, 1.0e-12));
	TestEqual(TEXT("Extruded patch surface id"), Hit.SurfaceId, uint64(31));
	TestEqual(TEXT("Extruded patch feature id"), Hit.FeatureId, uint64(32));
	TestTrue(TEXT("Extruded patch segment id is stable"), Hit.PrimitiveId != 0);
	FWorldQuery RayQuery = Query;
	RayQuery.Shape = EQueryShape::Ray;
	RayQuery.Radius = 0.0;
	const FWorldHit RayHit = FWorldQueryService(World).Sweep(RayQuery);
	TestTrue(TEXT("Ray hits extruded quintic face"), RayHit.bHit);
	TestTrue(TEXT("Extruded quintic ray TOI"),
		FMath::IsNearlyEqual(RayHit.Time, 0.5, 1.0e-12));
	FWorldQuery BoxQuery = Query;
	BoxQuery.Shape = EQueryShape::Box;
	BoxQuery.Radius = 0.0;
	BoxQuery.HalfExtent = FVector3d(2.0, 3.0, 4.0);
	const FWorldHit BoxHit = FWorldQueryService(World).Sweep(BoxQuery);
	TestTrue(TEXT("Box hits extruded quintic face"), BoxHit.bHit);
	TestTrue(TEXT("Extruded quintic box TOI"),
		FMath::IsNearlyEqual(BoxHit.Time, 0.4, 1.0e-12));

	Query.TraceChannel = 4;
	TestFalse(TEXT("Collision channel rejects compact patch"),
		FWorldQueryService(World).Sweep(Query).bHit);
	Query.bObjectQuery = true;
	Query.ObjectTypes = 1ull << 2;
	TestTrue(TEXT("Object query accepts compact patch"),
		FWorldQueryService(World).Sweep(Query).bHit);
	Query.bObjectQuery = false;
	Query.TraceChannel = 3;
	Query.BlockingObjectTypes = 0;
	TestFalse(TEXT("Query-side response rejects compact patch object type"),
		FWorldQueryService(World).Sweep(Query).bHit);
	Query.BlockingObjectTypes = 1ull << 2;
	TestTrue(TEXT("Mutual response accepts compact patch"),
		FWorldQueryService(World).Sweep(Query).bHit);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FIAmSpeedAnalyticTriangleFaceTest,
	"IAmSpeed.AnalyticWorld.TriangleFaceBvh",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedAnalyticTriangleFaceTest::RunTest(const FString& Parameters)
{
	using namespace Speed::Analytic;

	FAnalyticWorldData World;
	FTriangleSurface Triangle;
	Triangle.SourceId = 19;
	Triangle.SurfaceId = 20;
	Triangle.FeatureId = 21;
	Triangle.PrimitiveId = 22;
	Triangle.Vertices[0] = FVector3d(-100.0, -100.0, 0.0);
	Triangle.Vertices[1] = FVector3d(100.0, -100.0, 0.0);
	Triangle.Vertices[2] = FVector3d(0.0, 100.0, 0.0);
	Triangle.FaceNormal = FVector3d::UpVector;
	Triangle.Bounds = FBox3d(EForceInit::ForceInit);
	for (int32 Corner = 0; Corner < 3; ++Corner)
	{
		Triangle.Bounds += Triangle.Vertices[Corner];
	}
	Triangle.ObjectType = 0;
	Triangle.BlockingChannels = 1ull << 3;
	Triangle.bQueryCollisionEnabled = true;
	World.Triangles.Add(Triangle);
	FTriangleSurface AdjacentTriangle = Triangle;
	AdjacentTriangle.PrimitiveId = 23;
	AdjacentTriangle.Vertices[0] = FVector3d(-100.0, -100.0, 0.0);
	AdjacentTriangle.Vertices[1] = FVector3d(0.0, 100.0, 0.0);
	AdjacentTriangle.Vertices[2] = FVector3d(-200.0, 100.0, 0.0);
	AdjacentTriangle.Bounds = FBox3d(EForceInit::ForceInit);
	for (int32 Corner = 0; Corner < 3; ++Corner)
	{
		AdjacentTriangle.Bounds += AdjacentTriangle.Vertices[Corner];
	}
	World.Triangles.Add(AdjacentTriangle);
	FString ValidationReason;
	TestTrue(TEXT("Triangle world validates"),
		World.FinalizeAndValidate(&ValidationReason));
	World.BuildRecognitionDiagnostics();
	TestEqual(TEXT("Triangle BVH has one leaf"), World.TriangleBvh.Num(), 1);
	TestEqual(TEXT("Indexed topology deduplicates shared vertices"),
		World.MeshVertices.Num(), 4);
	TestEqual(TEXT("Indexed topology deduplicates shared edge"),
		World.MeshEdges.Num(), 5);
	TestEqual(TEXT("First triangle sees manifold neighbor"),
		World.TriangleNeighborIndices[0].Z, 1);
	TestEqual(TEXT("Second triangle sees manifold neighbor"),
		World.TriangleNeighborIndices[1].X, 0);
	const int32 SharedEdgeIndex = World.TriangleEdgeIndices[0].Z;
	TestTrue(TEXT("Shared edge is manifold"),
		World.MeshEdges[SharedEdgeIndex].IsManifold());
	TestEqual(TEXT("Matching imported normals classify a smooth edge"),
		World.MeshEdges[SharedEdgeIndex].Continuity, EEdgeContinuity::Smooth);
	TestEqual(TEXT("Smooth neighbors form one G1 region"),
		World.SmoothSurfaceRegions.Num(), 1);
	TestEqual(TEXT("G1 region retains both triangles"),
		World.SmoothSurfaceRegions[0].TriangleCount, 2);
	int32 ValidFlatShapeSamples = 0;
	for (const FVertexShapeSample& Sample : World.VertexShapeSamples)
	{
		if (!Sample.bValid) continue;
		++ValidFlatShapeSamples;
		TestTrue(TEXT("Flat sample has zero minimum curvature"),
			FMath::IsNearlyZero(Sample.MinimumPrincipalCurvature, 1.0e-12));
		TestTrue(TEXT("Flat sample has zero maximum curvature"),
			FMath::IsNearlyZero(Sample.MaximumPrincipalCurvature, 1.0e-12));
	}
	TestEqual(TEXT("Bounded two-ring fixture solves shared-edge vertices"),
		ValidFlatShapeSamples, 2);
	TestEqual(TEXT("Coplanar neighbors form one patch"),
		World.SurfacePatches.Num(), 1);
	TestEqual(TEXT("Coplanar patch retains both triangles"),
		World.SurfacePatches[0].TriangleCount, 2);
	TestEqual(TEXT("Coplanar patch is a planar candidate"),
		World.SurfacePatches[0].Kind, ESurfacePatchKind::PlanarCandidate);
	TestEqual(TEXT("Coplanar patch forms one architectural plane group"),
		World.PlanarSurfaceGroups.Num(), 1);
	TestEqual(TEXT("Architectural plane group retains the patch"),
		World.PlanarSurfaceGroups[0].PatchCount, 1);
	TestEqual(TEXT("Architectural plane group retains both triangles"),
		World.PlanarSurfaceGroups[0].TriangleCount, 2);
	TestTrue(TEXT("Largest flat fixture group is a hard plane constraint"),
		World.PlanarSurfaceGroups[0].bArchitecturalConstraint);
	TestTrue(TEXT("Architectural plane group is positionally exact"),
		FMath::IsNearlyZero(
			World.PlanarSurfaceGroups[0].MaximumPlaneResidual, 1.0e-12));
	TestEqual(TEXT("Architectural plane group has X and Y mirror diagnostics"),
		World.PlanarGroupMirrorMatches.Num(), 2);
	TestEqual(TEXT("Single plane group is its own X mirror candidate"),
		World.PlanarGroupMirrorMatches[0].TargetGroupIndex, 0);
	TestTrue(TEXT("Single plane X mirror candidate is reciprocal"),
		World.PlanarGroupMirrorMatches[0].bReciprocal);
	TestEqual(TEXT("Flat fixture emits one evidence record per triangle"),
		World.TriangleCurvatureEvidence.Num(), 2);
	TestEqual(TEXT("First flat triangle is an exact planar constraint"),
		World.TriangleCurvatureEvidence[0].Kind,
		ECurvatureEvidenceKind::PlanarConstraint);
	TestEqual(TEXT("Second flat triangle is an exact planar constraint"),
		World.TriangleCurvatureEvidence[1].Kind,
		ECurvatureEvidenceKind::PlanarConstraint);
	TestEqual(TEXT("Exact planar constraints do not create residual regions"),
		World.CurvatureSurfaceRegions.Num(), 0);
	TestEqual(TEXT("Non-elongated flat fixture emits no extrusion region"),
		World.ExtrusionSurfaceRegions.Num(), 0);
	TestEqual(TEXT("Flat fixture emits no extrusion mirror diagnostic"),
		World.ExtrusionRegionMirrorMatches.Num(), 0);
	TestEqual(TEXT("Flat fixture emits no ellipse boundary diagnostic"),
		World.QuarterEllipseBoundaryMatches.Num(), 0);
	TestEqual(TEXT("Flat fixture emits no C2 transition diagnostic"),
		World.C2TransitionSectionFits.Num(), 0);
	TestEqual(TEXT("Flat fixture emits no shared C2 pair diagnostic"),
		World.SharedC2TransitionPairFits.Num(), 0);
	TestEqual(TEXT("Flat fixture emits no shared C2 frame ledger"),
		World.SharedC2FrameLedger.Num(), 0);
	TestEqual(TEXT("Flat fixture emits no global C2 plane constraint"),
		World.GlobalC2PlaneConstraints.Num(), 0);
	TestEqual(TEXT("Flat fixture emits no global C2 join certificate"),
		World.GlobalC2JoinCertificates.Num(), 0);
	TestEqual(TEXT("Flat fixture emits no symmetrized C2 plane constraint"),
		World.SymmetrizedC2PlaneConstraints.Num(), 0);
	TestFalse(TEXT("Flat fixture emits no fitted C2 symmetry placement frame"),
		World.C2SymmetryPlacementFrame.bFittedFromMirrorPlanes);
	TestEqual(TEXT("Flat fixture emits no coupled C2 transition solution"),
		World.CoupledC2TransitionSolutions.Num(), 0);
	TestEqual(TEXT("Flat fixture emits no coupled C2 transition family"),
		World.CoupledC2TransitionFamilies.Num(), 0);
	TestEqual(TEXT("Flat fixture emits no coupled C2 family member"),
		World.CoupledC2FamilyRegionIndices.Num(), 0);
	TestEqual(TEXT("Flat fixture emits no C2 transition coverage entry"),
		World.C2TransitionCoverage.Num(), 0);
	TestEqual(TEXT("Flat fixture emits no playable C2 orbit candidate"),
		World.PlayableC2OrbitCandidates.Num(), 0);
	TestEqual(TEXT("Flat fixture emits no playable C2 orbit member"),
		World.PlayableC2OrbitMembers.Num(), 0);
	TestEqual(TEXT("Flat fixture emits no playable C2 triangle support candidate"),
		World.PlayableC2TriangleSupportCandidates.Num(), 0);
	TestEqual(TEXT("Flat fixture emits no playable C2 triangle support member"),
		World.PlayableC2TriangleSupportMembers.Num(), 0);
	TestEqual(TEXT("Flat fixture emits no playable C2 canonical support point"),
		World.PlayableC2CanonicalSupportPoints.Num(), 0);
	TestEqual(TEXT("Flat fixture emits no playable C2 plane binding"),
		World.PlayableC2PlaneBindings.Num(), 0);
	TestEqual(TEXT("Flat fixture emits no playable C2 network plane"),
		World.PlayableC2NetworkPlaneConstraints.Num(), 0);
	TestEqual(TEXT("Flat fixture emits no end-wall boundary component"),
		World.EndWallBoundaryComponents.Num(), 0);
	TestEqual(TEXT("Flat fixture emits no end-wall boundary edge"),
		World.EndWallBoundaryEdgeIndices.Num(), 0);
	TestEqual(TEXT("Flat fixture emits no opening-mouth rim candidate"),
		World.OpenRimCandidates.Num(), 0);
	TestEqual(TEXT("Flat fixture emits no opening-mouth rim edge"),
		World.OpenRimEdgeIndices.Num(), 0);
	TestEqual(TEXT("Flat fixture emits no canonical opening arch"),
		World.CanonicalOpenArchSolutions.Num(), 0);
	TestEqual(TEXT("Flat fixture emits no canonical opening arch member"),
		World.CanonicalOpenArchMemberFits.Num(), 0);
	TestEqual(TEXT("Flat fixture emits no opening-mouth surface-band observation"),
		World.OpenRimSurfaceBandObservations.Num(), 0);
	TestEqual(TEXT("Flat fixture emits no opening-mouth transverse section"),
		World.OpenRimTransverseSections.Num(), 0);
	TestEqual(TEXT("Flat fixture emits no opening-mouth transverse section point"),
		World.OpenRimTransverseSectionPoints.Num(), 0);
	TestEqual(TEXT("Flat fixture emits no opening-mouth transition section point"),
		World.OpenRimTransitionSectionPoints.Num(), 0);
	TestEqual(TEXT("Flat fixture emits no opening-mouth transition family fit"),
		World.OpenRimTransitionFamilyFits.Num(), 0);
	TestEqual(TEXT("Flat fixture emits no opening-mouth transition member fit"),
		World.OpenRimTransitionMemberFits.Num(), 0);
	TestEqual(TEXT("Flat fixture emits no opening-mouth C2 loft station"),
		World.OpenRimC2LoftStations.Num(), 0);
	TestEqual(TEXT("Flat fixture emits no opening-mouth C2 loft segment"),
		World.OpenRimC2LoftSegments.Num(), 0);
	TestEqual(TEXT("Flat fixture emits no opening-mouth C2 loft fit"),
		World.OpenRimC2LoftFits.Num(), 0);
	TestEqual(TEXT("Flat fixture emits no canonical opening surface sample"),
		World.OpenRimCanonicalSurfaceSamples.Num(), 0);
	TestEqual(TEXT("Flat fixture emits no canonical opening section correspondence"),
		World.OpenRimCanonicalSectionCorrespondences.Num(), 0);
	TestEqual(TEXT("Flat fixture emits no canonical opening surface fit"),
		World.OpenRimCanonicalSurfaceFits.Num(), 0);
	TestEqual(TEXT("Flat fixture emits no opening-mouth support transition intent"),
		World.OpenRimSupportTransitionIntents.Num(), 0);
	FC2TransitionSectionFit TransitionFixture;
	TransitionFixture.RadiusU = 10.0;
	TransitionFixture.RadiusV = 20.0;
	TransitionFixture.SignU = 1.0;
	TransitionFixture.SignV = 1.0;
	TransitionFixture.FlatteningFraction = 0.25;
	const FVector2d TransitionStart = TransitionFixture.EvaluatePosition(0.0);
	const FVector2d TransitionEnd = TransitionFixture.EvaluatePosition(1.0);
	TestTrue(TEXT("Quintic transition starts on its V plane"),
		TransitionStart.Equals(FVector2d(0.0, 20.0), 1.0e-12));
	TestTrue(TEXT("Quintic transition ends on its U plane"),
		TransitionEnd.Equals(FVector2d(10.0, 0.0), 1.0e-12));
	TestTrue(TEXT("Quintic transition starts tangent to U"),
		FMath::Abs(TransitionFixture.EvaluateFirstDerivative(0.0).Y) <= 1.0e-12);
	TestTrue(TEXT("Quintic transition ends tangent to V"),
		FMath::Abs(TransitionFixture.EvaluateFirstDerivative(1.0).X) <= 1.0e-12);
	TestTrue(TEXT("Quintic transition has zero start second derivative"),
		TransitionFixture.EvaluateSecondDerivative(0.0).IsNearlyZero(1.0e-12));
	TestTrue(TEXT("Quintic transition has zero end second derivative"),
		TransitionFixture.EvaluateSecondDerivative(1.0).IsNearlyZero(1.0e-12));
	FAnalyticWorldData SymmetricWorld;
	SymmetricWorld.MeshVertices = {
		FVector3d(-2.0, -3.0, 1.0), FVector3d(2.0, -3.0, 1.0),
		FVector3d(-2.0, 3.0, 1.0), FVector3d(2.0, 3.0, 1.0) };
	const FPlanarSymmetryMetrics SymmetryX = SymmetricWorld.MeasurePlanarSymmetry(
		EPlanarSymmetryAxis::X, 1.0e-6);
	const FPlanarSymmetryMetrics SymmetryY = SymmetricWorld.MeasurePlanarSymmetry(
		EPlanarSymmetryAxis::Y, 1.0e-6);
	TestEqual(TEXT("Fixture has complete X mirror coverage"),
		SymmetryX.MatchedVertexCount, SymmetricWorld.MeshVertices.Num());
	TestEqual(TEXT("Fixture has complete Y mirror coverage"),
		SymmetryY.MatchedVertexCount, SymmetricWorld.MeshVertices.Num());
	TestTrue(TEXT("Fixture X mirror is exact"),
		FMath::IsNearlyZero(SymmetryX.MaximumResidual, 1.0e-12));
	TestTrue(TEXT("Fixture Y mirror is exact"),
		FMath::IsNearlyZero(SymmetryY.MaximumResidual, 1.0e-12));
	FAnalyticWorldData SurfaceSymmetricWorld;
	FTriangleSurface SurfaceSymmetricTriangle = Triangle;
	SurfaceSymmetricTriangle.Vertices[0] = FVector3d(-2.0, 0.0, 0.0);
	SurfaceSymmetricTriangle.Vertices[1] = FVector3d(2.0, 0.0, 0.0);
	SurfaceSymmetricTriangle.Vertices[2] = FVector3d(0.0, 3.0, 0.0);
	SurfaceSymmetricTriangle.Bounds = FBox3d(EForceInit::ForceInit);
	for (int32 Corner = 0; Corner < 3; ++Corner)
	{
		SurfaceSymmetricTriangle.Bounds += SurfaceSymmetricTriangle.Vertices[Corner];
	}
	SurfaceSymmetricWorld.Triangles.Add(SurfaceSymmetricTriangle);
	TestTrue(TEXT("Surface-symmetry fixture validates"),
		SurfaceSymmetricWorld.FinalizeAndValidate(&ValidationReason));
	const FSurfaceSymmetryMetrics SurfaceSymmetryX =
		SurfaceSymmetricWorld.MeasurePlanarSurfaceSymmetry(
			EPlanarSymmetryAxis::X, 1.0e-6, 0.0);
	TestEqual(TEXT("Fixture has complete X mirrored-surface coverage"),
		SurfaceSymmetryX.WithinToleranceVertexCount,
		SurfaceSymmetricWorld.MeshVertices.Num());
	TestTrue(TEXT("Fixture X mirrored-surface residual is exact"),
		FMath::IsNearlyZero(SurfaceSymmetryX.MaximumNearestResidual, 1.0e-12));

	FWorldQuery Query;
	Query.Shape = EQueryShape::Sphere;
	Query.Start = FVector3d(0.0, 0.0, 20.0);
	Query.End = FVector3d(0.0, 0.0, -20.0);
	Query.Radius = 5.0;
	Query.bIncludeTriangles = true;
	Query.bApplyCollisionFilter = true;
	Query.TraceChannel = 3;
	const FWorldHit Hit = FWorldQueryService(World).Sweep(Query);
	TestTrue(TEXT("Sphere hits triangle face"), Hit.bHit);
	TestTrue(TEXT("Triangle face TOI"), FMath::IsNearlyEqual(Hit.Time, 0.375, 1.0e-12));
	TestEqual(TEXT("Triangle primitive tie-break id"), Hit.PrimitiveId, uint64(22));

	Query.Start = FVector3d(0.0, 0.0, -20.0);
	Query.End = FVector3d(0.0, 0.0, 20.0);
	const FWorldHit BackFaceHit = FWorldQueryService(World).Sweep(Query);
	TestTrue(TEXT("Sphere hits triangle back face"), BackFaceHit.bHit);
	TestTrue(TEXT("Triangle back-face TOI"),
		FMath::IsNearlyEqual(BackFaceHit.Time, 0.375, 1.0e-12));
	TestTrue(TEXT("Triangle back-face normal faces query"),
		BackFaceHit.Normal.Equals(-FVector3d::UpVector, 1.0e-12));
	Query.Start = FVector3d(0.0, 0.0, 20.0);
	Query.End = FVector3d(0.0, 0.0, -20.0);

	Query.TraceChannel = 4;
	TestFalse(TEXT("Collision channel filter rejects triangle"),
		FWorldQueryService(World).Sweep(Query).bHit);
	Query.bObjectQuery = true;
	Query.ObjectTypes = 1ull << 0;
	TestTrue(TEXT("Object query filter accepts triangle"),
		FWorldQueryService(World).Sweep(Query).bHit);
	Query.ObjectTypes = 1ull << 1;
	TestFalse(TEXT("Object query filter rejects triangle"),
		FWorldQueryService(World).Sweep(Query).bHit);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FIAmSpeedFlatLandscapeAdapterTest,
	"IAmSpeed.AnalyticWorld.FlatLandscapeAdapter",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedFlatLandscapeAdapterTest::RunTest(const FString& Parameters)
{
	using namespace Speed::Analytic;

	FFlatLandscapeSource Source;
	Source.SourceId = 42;
	Source.WorldBounds = FBox3d(
		FVector3d(-100.0, -200.0, 50.0),
		FVector3d(100.0, 200.0, 50.0));
	Source.WorldHeights = { 50.0, 50.0, 50.0, 50.0 };
	Source.bHoleCoverageValidated = true;
	Source.bMaterialCoverageValidated = true;
	Source.bCollisionPolicyValidated = true;
	Source.MaterialId = 7;
	Source.ObjectType = 2;
	Source.BlockingChannels = 1ull << 3;
	Source.bQueryCollisionEnabled = true;
	const FFlatLandscapeAdapterOutput Flat = BuildFlatLandscapePlane(Source, 1.0e-6);
	TestEqual(TEXT("Fully classified flat source is authority eligible"), Flat.Result,
		EFlatLandscapeAdapterResult::SuccessAuthorityEligible);
	TestTrue(TEXT("Flat source keeps exact height"),
		FMath::IsNearlyEqual(Flat.Plane.Origin.Z, 50.0, 1.0e-12));
	TestTrue(TEXT("Validated flat Landscape permits authority"),
		Flat.Plane.bAuthorityEligible);
	TestEqual(TEXT("Flat Landscape retains material"), Flat.Plane.MaterialId,
		uint32(7));
	TestEqual(TEXT("Flat Landscape retains object type"), Flat.Plane.ObjectType,
		uint32(2));

	Source.WorldHeights[3] = 50.01;
	const FFlatLandscapeAdapterOutput Curved = BuildFlatLandscapePlane(Source, 0.001);
	TestEqual(TEXT("Non-flat source is rejected"), Curved.Result,
		EFlatLandscapeAdapterResult::NonFlat);
	return true;
}

#endif
