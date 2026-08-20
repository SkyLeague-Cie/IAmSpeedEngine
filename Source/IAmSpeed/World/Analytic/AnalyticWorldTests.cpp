#include "AnalyticLandscapeAdapter.h"
#include "AnalyticWorldQuery.h"
#include "IAmSpeed/Base/SHitResult.h"
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
	TestTrue(TEXT("Box face contact reports coincident witnesses"),
		BoxHit.QueryPoint.Equals(BoxHit.Point, 2.0e-6));
	TestEqual(TEXT("Box face contact classifies the surface witness"),
		BoxHit.SurfaceFeatureKind, EContactFeatureKind::Face);

	FWorldQuery BoxEdgeQuery = BoxQuery;
	BoxEdgeQuery.Start = FVector3d(110.0, 0.0, 0.0);
	BoxEdgeQuery.End = FVector3d(90.0, 0.0, 0.0);
	BoxEdgeQuery.HalfExtent = FVector3d(5.0);
	const FWorldHit BoxEdgeHit = FWorldQueryService(World).Sweep(BoxEdgeQuery);
	TestTrue(TEXT("Box sweep detects a finite plane edge"), BoxEdgeHit.bHit);
	TestTrue(TEXT("Box edge sweep has exact TOI"),
		FMath::IsNearlyEqual(BoxEdgeHit.Time, 0.25, 1.0e-6));
	TestTrue(TEXT("Box edge sweep normal points out of the domain"),
		BoxEdgeHit.Normal.Equals(FVector3d::ForwardVector, 1.0e-9));
	TestTrue(TEXT("Box edge sweep reports the boundary witness"),
		FMath::IsNearlyEqual(BoxEdgeHit.Point.X, 100.0, 1.0e-6));

	FWorldQuery BoxVertexQuery = BoxQuery;
	BoxVertexQuery.Start = FVector3d(110.0, 60.0, 0.0);
	BoxVertexQuery.End = FVector3d(90.0, 40.0, 0.0);
	BoxVertexQuery.HalfExtent = FVector3d(5.0);
	const FWorldHit BoxVertexHit = FWorldQueryService(World).Sweep(BoxVertexQuery);
	TestTrue(TEXT("Box sweep detects a finite plane vertex"), BoxVertexHit.bHit);
	TestTrue(TEXT("Box vertex sweep has exact TOI"),
		FMath::IsNearlyEqual(BoxVertexHit.Time, 0.25, 1.0e-6));
	TestTrue(TEXT("Box vertex sweep reports the corner witness"),
		BoxVertexHit.Point.Equals(FVector3d(100.0, 50.0, 0.0), 2.0e-6));

	FWorldQuery BoxPenetrationQuery = BoxQuery;
	BoxPenetrationQuery.Start = FVector3d(0.0, 0.0, 3.0);
	BoxPenetrationQuery.End = BoxPenetrationQuery.Start;
	const FWorldHit BoxPenetrationHit =
		FWorldQueryService(World).Sweep(BoxPenetrationQuery);
	TestTrue(TEXT("Box initial overlap is reported"),
		BoxPenetrationHit.bHit && BoxPenetrationHit.bStartPenetrating);
	TestTrue(TEXT("Box initial overlap reports the MTD depth"),
		FMath::IsNearlyEqual(BoxPenetrationHit.PenetrationDepth, 1.0, 1.0e-9));
	TestTrue(TEXT("Box penetration witnesses encode the MTD"),
		(BoxPenetrationHit.Point - BoxPenetrationHit.QueryPoint).
			Equals(BoxPenetrationHit.PenetrationDepth *
				BoxPenetrationHit.Normal, 2.0e-6));
	const Speed::SHitResult NativePenetrationHit =
		Speed::SHitResult::FromAnalyticHit(BoxPenetrationHit, 1.0f / 300.0f);
	TestTrue(TEXT("Native hit preserves the moving-shape witness"),
		FVector3d(NativePenetrationHit.ContactPointThis).
			Equals(BoxPenetrationHit.QueryPoint, 1.0e-6));
	TestTrue(TEXT("Native hit preserves the static-surface witness"),
		FVector3d(NativePenetrationHit.ContactPointOther).
			Equals(BoxPenetrationHit.Point, 1.0e-6));
	TestEqual(TEXT("Native hit preserves the authored feature id"),
		NativePenetrationHit.FeatureId, BoxPenetrationHit.FeatureId);
	TestEqual(TEXT("Native hit preserves the local feature classification"),
		NativePenetrationHit.ContactFeatureOther,
		BoxPenetrationHit.SurfaceFeatureKind);
	FWorldQuery BoxReleasedQuery = BoxQuery;
	BoxReleasedQuery.Start = FVector3d(0.0, 0.0, 4.001);
	BoxReleasedQuery.End = FVector3d(0.0, 0.0, 10.0);
	TestFalse(TEXT("Box contact releases after separation"),
		FWorldQueryService(World).Sweep(BoxReleasedQuery).bHit);
	FWorldQuery BoxShellReleaseQuery = BoxQuery;
	BoxShellReleaseQuery.Start = FVector3d(0.0, 0.0, 3.975);
	BoxShellReleaseQuery.End = BoxShellReleaseQuery.Start;
	TestFalse(TEXT("Stationary shallow OBB overlap releases inside contact shell"),
		FWorldQueryService(World).Sweep(BoxShellReleaseQuery).bHit);
	BoxShellReleaseQuery.End.Z = 3.9;
	const FWorldHit ClosingShellHit =
		FWorldQueryService(World).Sweep(BoxShellReleaseQuery);
	TestTrue(TEXT("Closing OBB cannot cross the contact shell"),
		ClosingShellHit.bHit && !ClosingShellHit.bStartPenetrating &&
		FMath::IsNearlyZero(ClosingShellHit.Time));

	FWorldQuery OutsideQuery = SphereQuery;
	OutsideQuery.Start.X = 200.0;
	OutsideQuery.End.X = 200.0;
	TestFalse(TEXT("Bounded domain rejects outside contact"),
		FWorldQueryService(World).Sweep(OutsideQuery).bHit);
	FBoundedPlane PolygonPlane = Plane;
	PolygonPlane.SurfaceId = 20;
	PolygonPlane.FeatureId = 21;
	PolygonPlane.PrimitiveId = 20;
	PolygonPlane.DomainVertices = {
		FVector2d(-100.0, -50.0), FVector2d(0.0, -50.0),
		FVector2d(0.0, 0.0), FVector2d(100.0, 0.0),
		FVector2d(100.0, 50.0), FVector2d(-100.0, 50.0) };
	FAnalyticWorldData PolygonWorld;
	PolygonWorld.Planes.Add(PolygonPlane);
	TestTrue(TEXT("Concave polygon domain validates"),
		PolygonWorld.FinalizeAndValidate(&ValidationReason));
	FWorldQuery PolygonInside = SphereQuery;
	PolygonInside.Start.X = -50.0;
	PolygonInside.End.X = -50.0;
	PolygonInside.Start.Y = -25.0;
	PolygonInside.End.Y = -25.0;
	TestTrue(TEXT("Polygon domain accepts an interior contact"),
		FWorldQueryService(PolygonWorld).Sweep(PolygonInside).bHit);
	FWorldQuery PolygonCutout = PolygonInside;
	PolygonCutout.Start.X = 50.0;
	PolygonCutout.End.X = 50.0;
	TestFalse(TEXT("Polygon domain rejects a concave cutout"),
		FWorldQueryService(PolygonWorld).Sweep(PolygonCutout).bHit);
	TestTrue(TEXT("Polygon domain includes its boundary within tolerance"),
		PolygonPlane.ContainsProjectedPoint(FVector3d(0.0, -25.0, 0.0), 1.0e-9));
	FBoundedPlane SelfIntersectingPlane = PolygonPlane;
	SelfIntersectingPlane.DomainVertices = {
		FVector2d(-50.0, -50.0), FVector2d(50.0, 50.0),
		FVector2d(-50.0, 50.0), FVector2d(50.0, -50.0) };
	TestFalse(TEXT("Self-intersecting polygon domain is rejected"),
		SelfIntersectingPlane.IsValid(&ValidationReason));
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
		StaticWorld.HasAuthorityCoverage(SphereQuery));
	FWorldQuery EdgeOverlap = SphereQuery;
	EdgeOverlap.Start.X = 98.0;
	EdgeOverlap.End.X = 98.0;
	TestTrue(TEXT("Draft face query still reports an interior contact near an edge"),
		FWorldQueryService(World).Sweep(EdgeOverlap).bHit);
	EdgeOverlap.bAuthorityOnly = true;
	TestFalse(TEXT("Authority declines a sphere whose footprint crosses a plane edge"),
		FWorldQueryService(World).Sweep(EdgeOverlap).bHit);
	FAnalyticWorldData CompactBoxWorld;
	FBoundedPlane CompactBoxPlane = Plane;
	CompactBoxPlane.SourceId = 30;
	CompactBoxPlane.SurfaceId = 30;
	CompactBoxPlane.FeatureId = 31;
	CompactBoxPlane.PrimitiveId = 30;
	CompactBoxPlane.bRequiresCompactOptIn = true;
	CompactBoxPlane.bAuthorityEligible = true;
	CompactBoxWorld.Planes.Add(CompactBoxPlane);
	TestTrue(TEXT("Compact box world validates"),
		CompactBoxWorld.FinalizeAndValidate(&ValidationReason));
	FWorldQuery CompactBoxQuery;
	CompactBoxQuery.Shape = EQueryShape::Box;
	CompactBoxQuery.Start = FVector3d(0.0, 0.0, 20.0);
	CompactBoxQuery.End = FVector3d(0.0, 0.0, -20.0);
	CompactBoxQuery.HalfExtent = FVector3d(5.0);
	CompactBoxQuery.bIncludeCompactPatches = true;
	TestTrue(TEXT("Draft compact plane still supports box shadow queries"),
		FWorldQueryService(CompactBoxWorld).Sweep(CompactBoxQuery).bHit);
	CompactBoxQuery.bAuthorityOnly = true;
	const FWorldHit CompactAuthorityBoxHit =
		FWorldQueryService(CompactBoxWorld).Sweep(CompactBoxQuery);
	TestTrue(TEXT("Certified compact plane accepts native OBB authority"),
		CompactAuthorityBoxHit.bHit);
	TestTrue(TEXT("Certified compact OBB hit carries both contact witnesses"),
		CompactAuthorityBoxHit.QueryPoint.Equals(
			CompactAuthorityBoxHit.Point, 2.0e-6));
	FWorldQuery DomainNearMiss = SphereQuery;
	DomainNearMiss.Start.X = 104.0;
	DomainNearMiss.End.X = 104.0;
	TestFalse(TEXT("Intersecting provider bounds without a hit are not hybrid coverage"),
		StaticWorld.HasAuthorityCoverage(DomainNearMiss));

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
			HasAuthorityCoverage(SphereQuery));
	FAnalyticWorldData SameSourceProviderWorld;
	SameSourceProviderWorld.Planes.Add(CertifiedPlane);
	NearerDraftPlane.SourceId = CertifiedPlane.SourceId;
	NearerDraftPlane.SurfaceId = CertifiedPlane.SurfaceId;
	NearerDraftPlane.FeatureId = CertifiedPlane.FeatureId + 1;
	NearerDraftPlane.PrimitiveId = CertifiedPlane.PrimitiveId + 1;
	SameSourceProviderWorld.Planes.Add(NearerDraftPlane);
	TestTrue(TEXT("Same-source provider-selection world validates"),
		SameSourceProviderWorld.FinalizeAndValidate(&ValidationReason));
	TestFalse(TEXT("Nearer draft primitive on the same source and surface blocks hybrid authority"),
		Speed::FAnalyticStaticCollisionWorld(SameSourceProviderWorld).
			HasAuthorityCoverage(SphereQuery));

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
	TestTrue(TEXT("Corrected quintic builds a certified adaptive polyline"),
		CorrectedPatch.BuildQueryApproximation());
	TestTrue(TEXT("Curved quintic subdivides beyond a single chord"),
		CorrectedPatch.SectionPolyline.Num() > 2);
	TestTrue(TEXT("Adaptive quintic respects its public error contract"),
		CorrectedPatch.MaximumChordErrorCm <=
			ExtrudedQuinticChordToleranceCm);
	double MeasuredMaximumDeviationCm = 0.0;
	int32 SegmentIndex = 0;
	for (int32 Sample = 0; Sample <= 256; ++Sample)
	{
		const double T = static_cast<double>(Sample) / 256.0;
		while (SegmentIndex + 2 < CorrectedPatch.SectionParameters.Num() &&
			T > CorrectedPatch.SectionParameters[SegmentIndex + 1])
		{
			++SegmentIndex;
		}
		const double T0 = CorrectedPatch.SectionParameters[SegmentIndex];
		const double T1 = CorrectedPatch.SectionParameters[SegmentIndex + 1];
		const double Alpha = (T - T0) / (T1 - T0);
		const FVector3d ChordPoint = FMath::Lerp(
			CorrectedPatch.SectionPolyline[SegmentIndex],
			CorrectedPatch.SectionPolyline[SegmentIndex + 1], Alpha);
		MeasuredMaximumDeviationCm = FMath::Max(MeasuredMaximumDeviationCm,
			FVector3d::Distance(CorrectedPatch.EvaluateSection(T), ChordPoint));
	}
	TestTrue(TEXT("Sampled deviation stays below the certified bound"),
		MeasuredMaximumDeviationCm <=
			CorrectedPatch.MaximumChordErrorCm + 1.0e-9);

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
	TestTrue(TEXT("Quintic hit exposes its certified geometric error"),
		Hit.GeometricErrorBoundCm <= ExtrudedQuinticChordToleranceCm);
	const Speed::SHitResult NativeQuinticHit =
		Speed::SHitResult::FromAnalyticHit(Hit, 1.0f / 300.0f);
	TestTrue(TEXT("Native hit preserves the quintic geometric error bound"),
		FMath::IsNearlyEqual(
			NativeQuinticHit.GeometricErrorBoundCm,
			static_cast<float>(Hit.GeometricErrorBoundCm), 1.0e-9f));
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

	// A curved patch is represented by certified finite chords. Exercise an
	// internal chord join explicitly: it is an approximation detail, not an
	// authored boundary, and must neither create an authority hole nor retain a
	// shallow separating overlap.
	FExtrudedQuinticPatch CurvedPatch = CorrectedPatch;
	CurvedPatch.SourceId = 40;
	CurvedPatch.SurfaceId = 41;
	CurvedPatch.FeatureId = 42;
	CurvedPatch.PrimitiveId = 43;
	CurvedPatch.CanonicalGroupId = 44;
	CurvedPatch.CanonicalSymmetryAxisMask = 1;
	CurvedPatch.bCanonicalC2ByConstruction = true;
	CurvedPatch.bCanonicalSymmetryByConstruction = true;
	CurvedPatch.bAuthorityEligible = false;
	FAnalyticWorldData CurvedWorld;
	CurvedWorld.ExtrudedQuinticPatches.Add(CurvedPatch);
	TestTrue(TEXT("Curved extruded-quintic world validates"),
		CurvedWorld.FinalizeAndValidate(&ValidationReason));
	const FExtrudedQuinticPatch& RuntimeCurvedPatch =
		CurvedWorld.ExtrudedQuinticPatches[0];
	TestTrue(TEXT("Curved fixture exposes an internal adaptive join"),
		RuntimeCurvedPatch.SectionPolyline.Num() >= 4);
	const int32 JoinIndex = RuntimeCurvedPatch.SectionPolyline.Num() / 2;
	const FVector3d JoinPoint = RuntimeCurvedPatch.SectionPolyline[JoinIndex];
	const FVector3d PreviousChord =
		(RuntimeCurvedPatch.SectionPolyline[JoinIndex] -
			RuntimeCurvedPatch.SectionPolyline[JoinIndex - 1]).GetSafeNormal();
	const FVector3d NextChord =
		(RuntimeCurvedPatch.SectionPolyline[JoinIndex + 1] -
			RuntimeCurvedPatch.SectionPolyline[JoinIndex]).GetSafeNormal();
	const FVector3d PreviousNormal = FVector3d::CrossProduct(
		PreviousChord, RuntimeCurvedPatch.ExtrusionAxis).GetSafeNormal();
	FVector3d NextNormal = FVector3d::CrossProduct(
		NextChord, RuntimeCurvedPatch.ExtrusionAxis).GetSafeNormal();
	if (FVector3d::DotProduct(PreviousNormal, NextNormal) < 0.0)
	{
		NextNormal = -NextNormal;
	}
	const FVector3d JoinNormal = (PreviousNormal + NextNormal).GetSafeNormal();

	FWorldQuery JoinBoxQuery;
	JoinBoxQuery.Shape = EQueryShape::Box;
	JoinBoxQuery.Rotation = FQuat4d(
		RuntimeCurvedPatch.ExtrusionAxis, FMath::DegreesToRadians(17.0));
	JoinBoxQuery.HalfExtent = FVector3d(2.0, 3.0, 4.0);
	JoinBoxQuery.bIncludeCompactPatches = true;
	JoinBoxQuery.bAuthorityOnly = true;
	const FVector3d JoinBoxAxisX = JoinBoxQuery.Rotation.RotateVector(
		FVector3d::ForwardVector);
	const FVector3d JoinBoxAxisY = JoinBoxQuery.Rotation.RotateVector(
		FVector3d::RightVector);
	const FVector3d JoinBoxAxisZ = JoinBoxQuery.Rotation.RotateVector(
		FVector3d::UpVector);
	const double JoinSupport =
		FMath::Abs(FVector3d::DotProduct(JoinBoxAxisX, JoinNormal)) *
			JoinBoxQuery.HalfExtent.X +
		FMath::Abs(FVector3d::DotProduct(JoinBoxAxisY, JoinNormal)) *
			JoinBoxQuery.HalfExtent.Y +
		FMath::Abs(FVector3d::DotProduct(JoinBoxAxisZ, JoinNormal)) *
			JoinBoxQuery.HalfExtent.Z;
	JoinBoxQuery.Start = JoinPoint + (JoinSupport + 5.0) * JoinNormal;
	JoinBoxQuery.End = JoinPoint - (JoinSupport + 5.0) * JoinNormal;
	TestFalse(TEXT("Draft curved patch is excluded from authority queries"),
		FWorldQueryService(CurvedWorld).Sweep(JoinBoxQuery).bHit);
	CurvedWorld.ExtrudedQuinticPatches[0].bAuthorityEligible = true;
	const FWorldHit JoinHit = FWorldQueryService(CurvedWorld).Sweep(JoinBoxQuery);
	TestTrue(TEXT("Certified curved patch covers an internal adaptive join"),
		JoinHit.bHit);
	TestTrue(TEXT("Internal join reports a stable outward normal"),
		FVector3d::DotProduct(JoinHit.Normal, JoinNormal) > 0.99);
	FWorldQuery JoinSphereQuery = JoinBoxQuery;
	JoinSphereQuery.Shape = EQueryShape::Sphere;
	JoinSphereQuery.Rotation = FQuat4d::Identity;
	JoinSphereQuery.HalfExtent = FVector3d::ZeroVector;
	const FVector3d InternalChordMidpoint = 0.5 *
		(RuntimeCurvedPatch.SectionPolyline[JoinIndex - 1] + JoinPoint);
	const double InternalChordLength = FVector3d::Distance(
		RuntimeCurvedPatch.SectionPolyline[JoinIndex - 1], JoinPoint);
	JoinSphereQuery.Radius = 0.75 * InternalChordLength;
	JoinSphereQuery.Start = InternalChordMidpoint +
		(JoinSphereQuery.Radius + 5.0) * PreviousNormal;
	JoinSphereQuery.End = InternalChordMidpoint -
		(JoinSphereQuery.Radius + 5.0) * PreviousNormal;
	FWorldQuery JoinSphereShadowQuery = JoinSphereQuery;
	JoinSphereShadowQuery.bAuthorityOnly = false;
	const FWorldHit JoinSphereShadowHit =
		FWorldQueryService(CurvedWorld).Sweep(JoinSphereShadowQuery);
	TestTrue(TEXT("Draft query sees the sphere contact at an internal adaptive join"),
		JoinSphereShadowHit.bHit);
	const FWorldHit JoinSphereHit =
		FWorldQueryService(CurvedWorld).Sweep(JoinSphereQuery);
	AddInfo(FString::Printf(
		TEXT("InternalJoinSphere ShadowHit=%d ShadowTime=%.17g AuthorityHit=%d AuthorityTime=%.17g Join=%s Radius=%.17g"),
		JoinSphereShadowHit.bHit ? 1 : 0, JoinSphereShadowHit.Time,
		JoinSphereHit.bHit ? 1 : 0, JoinSphereHit.Time,
		*FVector(InternalChordMidpoint).ToString(), JoinSphereQuery.Radius));
	TestTrue(TEXT("Certified quintic accepts a sphere across an internal adaptive join"),
		JoinSphereHit.bHit);
	TestTrue(TEXT("Internal adaptive join does not delay sphere TOI"),
		JoinSphereHit.Time < 0.5);
	FVector3d ExactJoinNormal = FVector3d::CrossProduct(
		RuntimeCurvedPatch.EvaluateSectionDerivative(
			RuntimeCurvedPatch.SectionParameters[JoinIndex]).GetSafeNormal(),
		RuntimeCurvedPatch.ExtrusionAxis).GetSafeNormal();
	if (FVector3d::DotProduct(ExactJoinNormal, JoinHit.Normal) < 0.0)
	{
		ExactJoinNormal = -ExactJoinNormal;
	}
	FWorldQuery SmoothNormalQuery = JoinBoxQuery;
	SmoothNormalQuery.Shape = EQueryShape::Ray;
	SmoothNormalQuery.Rotation = FQuat4d::Identity;
	SmoothNormalQuery.HalfExtent = FVector3d::ZeroVector;
	SmoothNormalQuery.Start = JoinPoint + 5.0 * ExactJoinNormal;
	SmoothNormalQuery.End = JoinPoint - 5.0 * ExactJoinNormal;
	const FWorldHit SmoothNormalHit =
		FWorldQueryService(CurvedWorld).Sweep(SmoothNormalQuery);
	TestTrue(TEXT("Ray resolves the authored internal join"), SmoothNormalHit.bHit);
	TestTrue(TEXT("Internal adaptive join exposes the smooth authored normal"),
		FVector3d::DotProduct(SmoothNormalHit.Normal, ExactJoinNormal) >
			1.0 - 1.0e-10);
	TestEqual(TEXT("Internal join preserves the authored feature id"),
		JoinHit.FeatureId, CurvedPatch.FeatureId);
	TestTrue(TEXT("Internal join receives a derived primitive id"),
		JoinHit.PrimitiveId != CurvedPatch.PrimitiveId);

	FWorldQuery OverlapQuery = JoinBoxQuery;
	OverlapQuery.InitialOverlapTolerance = 0.0;
	OverlapQuery.Start = JoinPoint + (JoinSupport - 0.02) * JoinNormal;
	OverlapQuery.End = OverlapQuery.Start;
	const FWorldHit OverlapHit =
		FWorldQueryService(CurvedWorld).Sweep(OverlapQuery);
	TestTrue(TEXT("Internal join initial overlap is detected"),
		OverlapHit.bHit && OverlapHit.bStartPenetrating);
	TestTrue(TEXT("Internal join MTD remains local"),
		OverlapHit.PenetrationDepth > 0.0 &&
		OverlapHit.PenetrationDepth <= 0.02 +
			RuntimeCurvedPatch.MaximumChordErrorCm + 1.0e-6);
	TestTrue(TEXT("Internal join MTD witnesses match depth"),
		FMath::IsNearlyEqual(
			FVector3d::DotProduct(
				OverlapHit.Point - OverlapHit.QueryPoint, OverlapHit.Normal),
			OverlapHit.PenetrationDepth, 1.0e-6));

	FWorldQuery ReleaseQuery = JoinBoxQuery;
	ReleaseQuery.Start = JoinPoint + (JoinSupport - 0.02) * JoinNormal;
	ReleaseQuery.End = ReleaseQuery.Start + JoinNormal;
	TestFalse(TEXT("Internal join releases a shallow separating overlap"),
		FWorldQueryService(CurvedWorld).Sweep(ReleaseQuery).bHit);
	TestTrue(TEXT("Authority coverage accepts the certified internal join"),
		FWorldQueryService(CurvedWorld).HasAuthorityCoverage(JoinBoxQuery));

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
