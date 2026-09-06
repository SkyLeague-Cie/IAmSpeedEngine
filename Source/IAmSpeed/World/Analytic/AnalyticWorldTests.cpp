#include "AnalyticLandscapeAdapter.h"
#include "AnalyticWorldQuery.h"
#include "IAmSpeed/Base/SHitResult.h"
#include "IAmSpeed/World/Collision/StaticCollisionWorld.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FIAmSpeedEmptyCollisionFilterTest,
	"IAmSpeed.AnalyticWorld.EmptyCollisionFilter",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedEmptyCollisionFilterTest::RunTest(const FString& Parameters)
{
	using namespace Speed::Analytic;
	FAnalyticWorldData World;
	FBoundedPlane Plane;
	Plane.SurfaceId = 1;
	Plane.FeatureId = 2;
	Plane.HalfExtents = FVector2d(100, 100);
	Plane.BlockingChannels = MAX_uint64;
	World.Planes.Add(Plane);
	TestTrue(TEXT("filtered test plane is valid"), World.FinalizeAndValidate());
	FWorldQueryService Service(World);
	for (EQueryShape Shape : { EQueryShape::Ray, EQueryShape::Sphere, EQueryShape::Box })
	{
		FWorldQuery Q;
		Q.Shape = Shape;
		Q.Start = FVector3d(0, 0, 20);
		Q.End = FVector3d(0, 0, -20);
		Q.Radius = 2;
		Q.HalfExtent = FVector3d(2);
		Q.bApplyCollisionFilter = true;
		TestTrue(TEXT("nonempty mutual trace filter hits"), Service.Sweep(Q).bHit);
		Q.BlockingObjectTypes = 0;
		TestTrue(TEXT("empty trace response rejects all geometry"), Q.RejectsAllCollision());
		TestFalse(TEXT("empty trace cannot reuse earlier cached hit"), Service.Sweep(Q).bHit);
		Q.bObjectQuery = true;
		TestFalse(TEXT("empty object mask misses"), Service.Sweep(Q).bHit);
		Q.ObjectTypes = 1;
		TestTrue(TEXT("object mask ignores the trace response mask"), Service.Sweep(Q).bHit);
		Q.bObjectQuery = false;
		Q.BlockingObjectTypes = 1;
		Q.TraceChannel = 63;
		TestTrue(TEXT("highest supported channel is valid"), Service.Sweep(Q).bHit);
		Q.TraceChannel = 64;
		TestFalse(TEXT("out-of-range channel misses"), Service.Sweep(Q).bHit);
		Q.bApplyCollisionFilter = false;
		TestTrue(TEXT("disabled filtering preserves geometry-only queries"), Service.Sweep(Q).bHit);
	}
	return true;
}

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
	TestFalse(TEXT("Plane contact has a constant surface normal"),
		BoxHit.bSurfaceNormalMayVary);
	// A polygon's triangulation diagonal is not an edge of the physical plane.
	// Every point of this box remains strictly inside the rectangular domain.
	for (double Yaw : { 0.0, 30.0, 90.0 })
	for (double Tilt : { 0.0, 1.e-8, 1.e-5 })
	{
		FWorldQuery Interior = BoxQuery;
		Interior.HalfExtent = FVector3d(4, 2, 2);
		Interior.Rotation = FRotator(Tilt, Yaw, 0).Quaternion();
		const double Radius = FMath::Abs(Interior.Rotation.GetAxisX().Z) * 4 +
			FMath::Abs(Interior.Rotation.GetAxisY().Z) * 2 + FMath::Abs(Interior.Rotation.GetAxisZ().Z) * 2;
		Interior.Start = Interior.End = FVector3d(-8, 0, Radius - 1.e-6);
		Interior.DomainTolerance = Interior.InitialOverlapTolerance = 0;
		const FWorldHit InteriorHit = FWorldQueryService(World).Sweep(Interior);
		TestTrue(TEXT("interior plane overlap preserves its complete normal depth"),
			InteriorHit.bHit && InteriorHit.bStartPenetrating &&
			FMath::IsNearlyEqual(InteriorHit.PenetrationDepth, 1.e-6, 1.e-12));
		TestEqual(TEXT("internal triangulation cannot turn a plane face into an edge"),
			InteriorHit.SurfaceFeatureKind, EContactFeatureKind::Face);
	}

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
	for (double TinyYaw : { 1.e-10, 1.e-8, 1.e-6 })
	{
		FWorldQuery NearlyAligned = BoxPenetrationQuery;
		NearlyAligned.Rotation = FRotator(0, TinyYaw, 0).Quaternion();
		NearlyAligned.DomainTolerance = 0;
		NearlyAligned.InitialOverlapTolerance = 0;
		const FWorldHit Hit = FWorldQueryService(World).Sweep(NearlyAligned);
		TestTrue(TEXT("near-parallel SAT axes must not hide finite box/plane penetration"),
			Hit.bHit && Hit.bStartPenetrating && FMath::IsNearlyEqual(Hit.PenetrationDepth, 1.0, 1.e-10));
	}
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
	TestTrue(TEXT("A broad-phase miss cannot reach the only analytical source"),
		FWorldQueryService(World).HasAuthorityCoverage(OutsideQuery));

	World.Planes[0].bAuthorityEligible = true;
	const Speed::FAnalyticStaticCollisionWorld StaticWorld(World);
	TestEqual(TEXT("Native static world reports the analytical backend"),
		StaticWorld.GetBackend(), Speed::EStaticCollisionBackend::SurfaceAnalytic);
	TestTrue(TEXT("Native static-world sweep returns an IAmSpeed hit"),
		StaticWorld.SweepSingle(AuthorityOnlyQuery).bHit);
	TestTrue(TEXT("Certified plane hit is hybrid authority coverage"),
		StaticWorld.HasAuthorityCoverage(SphereQuery));
	FWorldHit ReusedAuthorityHit;
	TestTrue(TEXT("Hybrid coverage returns its already-evaluated authority hit"),
		StaticWorld.TrySweepAuthority(SphereQuery, ReusedAuthorityHit));
	const FWorldHit DirectAuthorityHit = StaticWorld.SweepSingle(AuthorityOnlyQuery);
	TestEqual(TEXT("Reused authority primitive matches direct authority"),
		ReusedAuthorityHit.PrimitiveId, DirectAuthorityHit.PrimitiveId);
	TestEqual(TEXT("Reused authority TOI matches direct authority"),
		ReusedAuthorityHit.Time, DirectAuthorityHit.Time);
	FWorldQuery EdgeOverlap = SphereQuery;
	EdgeOverlap.Start.X = 98.0;
	EdgeOverlap.End.X = 98.0;
	TestTrue(TEXT("Draft face query still reports an interior contact near an edge"),
		FWorldQueryService(World).Sweep(EdgeOverlap).bHit);
	EdgeOverlap.bAuthorityOnly = true;
	TestFalse(TEXT("Authority excludes a sphere whose footprint crosses a plane edge"),
		FWorldQueryService(World).Sweep(EdgeOverlap).bHit);
	FAnalyticWorldData CoplanarUnionWorld;
	FBoundedPlane UnionLeft;
	UnionLeft.SourceId = 40;
	UnionLeft.SurfaceId = 41;
	UnionLeft.FeatureId = 42;
	UnionLeft.PrimitiveId = 43;
	UnionLeft.HalfExtents = FVector2d(10.0);
	UnionLeft.DomainVertices = {
		FVector2d(-10.0, -10.0), FVector2d(0.0, -10.0),
		FVector2d(0.0, 10.0), FVector2d(-10.0, 10.0) };
	UnionLeft.bAuthorityEligible = true;
	CoplanarUnionWorld.Planes.Add(UnionLeft);
	FBoundedPlane UnionRight = UnionLeft;
	UnionRight.PrimitiveId = 44;
	UnionRight.DomainVertices = {
		FVector2d(0.0, -10.0), FVector2d(10.0, -10.0),
		FVector2d(10.0, 10.0), FVector2d(0.0, 10.0) };
	CoplanarUnionWorld.Planes.Add(UnionRight);
	TestTrue(TEXT("Coplanar semantic union validates"),
		CoplanarUnionWorld.FinalizeAndValidate(&ValidationReason));
	FWorldQuery UnionSeamQuery = SphereQuery;
	UnionSeamQuery.bAuthorityOnly = true;
	TestTrue(TEXT("Authority footprint crosses an internal coplanar semantic seam"),
		FWorldQueryService(CoplanarUnionWorld).Sweep(UnionSeamQuery).bHit);
	FWorldQuery UnionOuterEdgeQuery = UnionSeamQuery;
	UnionOuterEdgeQuery.Start.X = -8.0;
	UnionOuterEdgeQuery.End.X = -8.0;
	TestFalse(TEXT("Coplanar union preserves its external footprint boundary"),
		FWorldQueryService(CoplanarUnionWorld).Sweep(UnionOuterEdgeQuery).bHit);
	FAnalyticWorldData QuantizedSeamWorld;
	QuantizedSeamWorld.Planes.Add(UnionLeft);
	FBoundedPlane QuantizedRight = UnionRight;
	QuantizedRight.HalfExtents.X = 10.5;
	QuantizedRight.DomainVertices = {
		FVector2d(0.5, -10.0), FVector2d(10.5, -10.0),
		FVector2d(10.5, 10.0), FVector2d(0.5, 10.0) };
	QuantizedSeamWorld.Planes.Add(QuantizedRight);
	TestTrue(TEXT("Sub-centimeter coplanar seam world validates"),
		QuantizedSeamWorld.FinalizeAndValidate(&ValidationReason));
	TestTrue(TEXT("Authority closes a sub-centimeter semantic seam"),
		FWorldQueryService(QuantizedSeamWorld).Sweep(UnionSeamQuery).bHit);
	FAnalyticWorldData WideGapWorld;
	WideGapWorld.Planes.Add(UnionLeft);
	FBoundedPlane WideGapRight = UnionRight;
	WideGapRight.HalfExtents.X = 12.0;
	WideGapRight.DomainVertices = {
		FVector2d(2.0, -10.0), FVector2d(12.0, -10.0),
		FVector2d(12.0, 10.0), FVector2d(2.0, 10.0) };
	WideGapWorld.Planes.Add(WideGapRight);
	TestTrue(TEXT("Wide coplanar gap world validates"),
		WideGapWorld.FinalizeAndValidate(&ValidationReason));
	TestFalse(TEXT("Authority preserves a gap wider than seam tolerance"),
		FWorldQueryService(WideGapWorld).Sweep(UnionSeamQuery).bHit);
	FAnalyticWorldData DistinctFeatureWorld = CoplanarUnionWorld;
	DistinctFeatureWorld.Planes[1].FeatureId = 45;
	TestTrue(TEXT("Distinct-feature plane pair validates"),
		DistinctFeatureWorld.FinalizeAndValidate(&ValidationReason));
	TestFalse(TEXT("Authority does not merge a seam between distinct features"),
		FWorldQueryService(DistinctFeatureWorld).Sweep(UnionSeamQuery).bHit);
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
	const FWorldHit UnrestrictedProviderHit =
		FWorldQueryService(ProviderWorld).Sweep(SphereQuery);
	TestEqual(TEXT("Unrestricted query selects the nearer provider"),
		UnrestrictedProviderHit.SurfaceId, NearerDraftPlane.SurfaceId);
	FWorldQuery RequiredProviderQuery = SphereQuery;
	RequiredProviderQuery.RequiredSourceId = CertifiedPlane.SourceId;
	RequiredProviderQuery.RequiredSurfaceId = CertifiedPlane.SurfaceId;
	const FWorldHit RequiredProviderHit =
		FWorldQueryService(ProviderWorld).Sweep(RequiredProviderQuery);
	TestEqual(TEXT("Stable provider restriction reacquires the requested surface"),
		RequiredProviderHit.SurfaceId, CertifiedPlane.SurfaceId);
	TestTrue(TEXT("Stable provider restriction cannot be replaced by a nearer surface"),
		RequiredProviderHit.Time > UnrestrictedProviderHit.Time);
	RequiredProviderQuery.RequiredSurfaceId = 999;
	TestFalse(TEXT("Unknown stable provider restriction reports no contact"),
		FWorldQueryService(ProviderWorld).Sweep(RequiredProviderQuery).bHit);
	TestFalse(TEXT("Nearer draft provider blocks hybrid authority selection"),
		Speed::FAnalyticStaticCollisionWorld(ProviderWorld).
			HasAuthorityCoverage(SphereQuery));
	FAnalyticWorldData FartherDraftWorld;
	FartherDraftWorld.Planes.Add(CertifiedPlane);
	FBoundedPlane FartherDraftPlane = NearerDraftPlane;
	FartherDraftPlane.Origin.Z = -10.0;
	FartherDraftWorld.Planes.Add(FartherDraftPlane);
	TestTrue(TEXT("Farther draft provider does not block an earlier authority hit"),
		FartherDraftWorld.FinalizeAndValidate(&ValidationReason));
	TestTrue(TEXT("Hybrid compares only the nearer draft competition"),
		Speed::FAnalyticStaticCollisionWorld(FartherDraftWorld).
			HasAuthorityCoverage(SphereQuery));
	FAnalyticWorldData LaterTieDraftWorld;
	LaterTieDraftWorld.Planes.Add(CertifiedPlane);
	FBoundedPlane LaterTieDraftPlane = NearerDraftPlane;
	LaterTieDraftPlane.Origin.Z = 0.0;
	LaterTieDraftWorld.Planes.Add(LaterTieDraftPlane);
	TestTrue(TEXT("Provider tie world validates"),
		LaterTieDraftWorld.FinalizeAndValidate(&ValidationReason));
	TestTrue(TEXT("Stable ordering retains authority ahead of a later draft tie"),
		Speed::FAnalyticStaticCollisionWorld(LaterTieDraftWorld).
			HasAuthorityCoverage(SphereQuery));
	FAnalyticWorldData EarlierTieDraftWorld;
	EarlierTieDraftWorld.Planes.Add(CertifiedPlane);
	FBoundedPlane EarlierTieDraftPlane = LaterTieDraftPlane;
	EarlierTieDraftPlane.SourceId = 50;
	EarlierTieDraftPlane.SurfaceId = 50;
	EarlierTieDraftPlane.FeatureId = 50;
	EarlierTieDraftPlane.PrimitiveId = 50;
	EarlierTieDraftWorld.Planes.Add(EarlierTieDraftPlane);
	TestTrue(TEXT("Earlier provider tie world validates"),
		EarlierTieDraftWorld.FinalizeAndValidate(&ValidationReason));
	TestFalse(TEXT("Stable ordering lets an earlier draft tie block authority"),
		Speed::FAnalyticStaticCollisionWorld(EarlierTieDraftWorld).
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

	FBoundedPlane IncrementalPlane = Plane;
	IncrementalPlane.SourceId = 300;
	IncrementalPlane.SurfaceId = 300;
	IncrementalPlane.FeatureId = 300;
	IncrementalPlane.PrimitiveId = 300;
	IncrementalPlane.Origin.Z = 10.0;
	IncrementalPlane.bAuthorityEligible = true;
	TArray<FBoundedPlane> IncrementalPlanes;
	IncrementalPlanes.Add(IncrementalPlane);
	TestTrue(TEXT("A finalized world accepts a non-compact plane incrementally"),
		World.AppendFinalizedNonCompactPlanes(
			MoveTemp(IncrementalPlanes), &ValidationReason));
	TestEqual(TEXT("Incremental extension preserves and adds plane providers"),
		World.Planes.Num(), 3);
	const FWorldHit IncrementalHit = FWorldQueryService(World).Sweep(SphereQuery);
	TestEqual(TEXT("Incremental plane participates in queries without full rebuild"),
		IncrementalHit.SurfaceId, IncrementalPlane.SurfaceId);
	FBoundedPlane InvalidIncrementalPlane = IncrementalPlane;
	InvalidIncrementalPlane.PrimitiveId = 301;
	InvalidIncrementalPlane.bRequiresCompactOptIn = true;
	TArray<FBoundedPlane> InvalidIncrementalPlanes;
	InvalidIncrementalPlanes.Add(InvalidIncrementalPlane);
	TestFalse(TEXT("Incremental extension rejects compact providers"),
		World.AppendFinalizedNonCompactPlanes(
			MoveTemp(InvalidIncrementalPlanes), &ValidationReason));
	TestEqual(TEXT("Rejected incremental extension is transactional"),
		World.Planes.Num(), 3);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FIAmSpeedRectangleTriangulationTest,
	"IAmSpeed.AnalyticWorld.RectangleTriangulation",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedRectangleTriangulationTest::RunTest(const FString& Parameters)
{
	using namespace Speed::Analytic;
	FRandomStream Random(31041);
	int32 Hits = 0;
	for (int32 Fixture = 0; Fixture < 3; ++Fixture)
	{
		FBoundedPlane Plane;
		Plane.SurfaceId = 1;
		Plane.PrimitiveId = 2;
		Plane.FeatureId = 3;
		Plane.Origin = FVector3d(1000.0 * Fixture, -400.0 * Fixture, 20.0 * Fixture);
		const FQuat4d Frame = FRotator3d(13.0 * Fixture, 37.0 * Fixture, 19.0 * Fixture).Quaternion();
		Plane.AxisU = Frame.RotateVector(FVector3d::ForwardVector);
		Plane.AxisV = Frame.RotateVector(FVector3d::RightVector);
		Plane.Normal = Frame.RotateVector(FVector3d::UpVector);
		Plane.HalfExtents = FVector2d(100.0, Fixture == 2 ? 0.01 : 50.0);
		FAnalyticWorldData RectangleWorld;
		RectangleWorld.Planes.Add(Plane);
		FString ValidationReason;
		if (!RectangleWorld.FinalizeAndValidate(&ValidationReason))
		{
			AddError(ValidationReason);
			return false;
		}
		// Explicit vertices force the original polygon ear-clipping path. This
		// independently checks diagonal/feature choices as well as contact pose.
		Plane.DomainVertices = {
			FVector2d(-Plane.HalfExtents.X, -Plane.HalfExtents.Y),
			FVector2d( Plane.HalfExtents.X, -Plane.HalfExtents.Y),
			FVector2d( Plane.HalfExtents.X,  Plane.HalfExtents.Y),
			FVector2d(-Plane.HalfExtents.X,  Plane.HalfExtents.Y) };
		FAnalyticWorldData PolygonWorld;
		PolygonWorld.Planes.Add(Plane);
		if (!PolygonWorld.FinalizeAndValidate(&ValidationReason))
		{
			AddError(ValidationReason);
			return false;
		}
		FWorldQueryService RectangleService(RectangleWorld);
		FWorldQueryService PolygonService(PolygonWorld);
		for (int32 Sample = 0; Sample < 400; ++Sample)
		{
			FWorldQuery Probe;
			Probe.Shape = EQueryShape::Box;
			Probe.DomainTolerance = Sample % 2 ? 0.0 : 1.0e-6;
			Probe.InitialOverlapTolerance = Sample % 3 ? 0.0 : 0.05;
			Probe.HalfExtent = FVector3d(6.0, 2.0, 0.5);
			Probe.Rotation = FRotator3d(Random.FRandRange(-90.0f, 90.0f),
				Random.FRandRange(-180.0f, 180.0f), Random.FRandRange(-90.0f, 90.0f)).Quaternion();
			const FVector3d Target = Plane.Origin +
				Random.FRandRange(-1.2f, 1.2f) * Plane.HalfExtents.X * Plane.AxisU +
				Random.FRandRange(-1.2f, 1.2f) * Plane.HalfExtents.Y * Plane.AxisV;
			const FVector3d Travel = Frame.RotateVector(FVector3d(
				Random.FRandRange(-15.0f, 15.0f), Random.FRandRange(-15.0f, 15.0f), 20.0));
			Probe.Start = Target + Travel;
			Probe.End = Target - Travel;
			if (Sample % 4 == 0) Probe.Start = Probe.End = Target + 0.1 * Plane.Normal;
			const FWorldHit Actual = RectangleService.Sweep(Probe);
			const FWorldHit Expected = PolygonService.Sweep(Probe);
			Hits += Expected.bHit ? 1 : 0;
			TestTrue(TEXT("Rectangle fast path is exactly the polygon oracle"),
				Actual.bHit == Expected.bHit && Actual.bStartPenetrating == Expected.bStartPenetrating &&
				Actual.Time == Expected.Time && Actual.PenetrationDepth == Expected.PenetrationDepth &&
				Actual.Location == Expected.Location && Actual.Point == Expected.Point &&
				Actual.QueryPoint == Expected.QueryPoint && Actual.Normal == Expected.Normal &&
				Actual.PrimitiveId == Expected.PrimitiveId &&
				Actual.QueryFeatureKind == Expected.QueryFeatureKind &&
				Actual.SurfaceFeatureKind == Expected.SurfaceFeatureKind &&
				Actual.QueryFeatureIndex == Expected.QueryFeatureIndex &&
				Actual.SurfaceFeatureIndex == Expected.SurfaceFeatureIndex);
		}
	}
	TestTrue(TEXT("Rectangle oracle includes contacts, not only misses"), Hits > 400);
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
	TestTrue(TEXT("Quintic contact identifies its varying surface normal"),
		Hit.bSurfaceNormalMayVary);
	TestTrue(TEXT("Native hit preserves varying-normal classification"),
		NativeQuinticHit.bSurfaceNormalMayVary);
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
	World.ExtrudedQuinticPatches[0].bAuthorityEligible = true;
	FWorldQuery FiniteDomainBoxQuery = BoxQuery;
	FiniteDomainBoxQuery.Start.Y = FiniteDomainBoxQuery.End.Y = 24.0;
	TestFalse(TEXT("Certified provider cannot expose an extruded endpoint as a wall"),
		FWorldQueryService(World).Sweep(FiniteDomainBoxQuery).bHit);
	FiniteDomainBoxQuery.bAuthorityOnly = true;
	TestFalse(TEXT("Authority-only OBB also rejects an extruded endpoint"),
		FWorldQueryService(World).Sweep(FiniteDomainBoxQuery).bHit);
	FiniteDomainBoxQuery.Start.Y = FiniteDomainBoxQuery.End.Y = 0.0;
	TestTrue(TEXT("Certified OBB retains contact inside the extruded domain"),
		FWorldQueryService(World).Sweep(FiniteDomainBoxQuery).bHit);

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
	TestEqual(TEXT("One derived bound per extruded facet"),
		RuntimeCurvedPatch.SectionSegmentBounds.Num(), RuntimeCurvedPatch.SectionPolyline.Num() - 1);
	for (int32 Segment = 0; Segment < RuntimeCurvedPatch.SectionSegmentBounds.Num(); ++Segment)
	{
		FBox3d ExpectedBounds(EForceInit::ForceInit);
		for (int32 Endpoint = Segment; Endpoint <= Segment + 1; ++Endpoint)
		{
			ExpectedBounds += RuntimeCurvedPatch.SectionPolyline[Endpoint] +
				RuntimeCurvedPatch.MinimumExtrusionCoordinate * RuntimeCurvedPatch.ExtrusionAxis;
			ExpectedBounds += RuntimeCurvedPatch.SectionPolyline[Endpoint] +
				RuntimeCurvedPatch.MaximumExtrusionCoordinate * RuntimeCurvedPatch.ExtrusionAxis;
		}
		TestTrue(TEXT("Facet bounds enclose the four original corners exactly"),
			ExpectedBounds == RuntimeCurvedPatch.SectionSegmentBounds[Segment]);
	}
	// An intentionally unpruned copy preserves facet order and narrow phase.
	// It is an oracle for node AND facet rejection, including long oblique paths,
	// stationary overlap, zero domain tolerance and contacts near finite ends.
	FAnalyticWorldData UnprunedWorld = CurvedWorld;
	// Include the outer compact hierarchy/provider rejection in the oracle,
	// while retaining the finite facet domains and original candidate order.
	for (FTriangleBvhNode& Node : UnprunedWorld.CompactBvh)
	{
		Node.Bounds = FBox3d(FVector3d(-10000.0), FVector3d(10000.0));
	}
	UnprunedWorld.ExtrudedQuinticPatches[0].Bounds =
		FBox3d(FVector3d(-10000.0), FVector3d(10000.0));
	for (FTriangleBvhNode& Node : UnprunedWorld.ExtrudedQuinticPatches[0].SectionSegmentBvhNodes)
	{
		Node.Bounds = FBox3d(FVector3d(-10000.0), FVector3d(10000.0));
	}
	for (FBox3d& Bounds : UnprunedWorld.ExtrudedQuinticPatches[0].SectionSegmentBounds)
	{
		Bounds = FBox3d(FVector3d(-10000.0), FVector3d(10000.0));
	}
	FWorldQueryService PrunedService(CurvedWorld);
	FWorldQueryService UnprunedService(UnprunedWorld);
	FRandomStream Random(60427);
	int32 OracleHits = 0;
	for (int32 Sample = 0; Sample < 600; ++Sample)
	{
		FWorldQuery Probe;
		Probe.bIncludeCompactPatches = true;
		Probe.Shape = static_cast<EQueryShape>(Sample % 3);
		Probe.DomainTolerance = Sample % 2 ? 0.0 : 1.0e-4;
		Probe.InitialOverlapTolerance = 0.0;
		Probe.Radius = Random.FRandRange(0.1f, 6.0f);
		Probe.HalfExtent = FVector3d(6.0, 0.3, 2.0);
		Probe.Rotation = FRotator3d(Random.FRandRange(-90.0f, 90.0f),
			Random.FRandRange(-180.0f, 180.0f), Random.FRandRange(-90.0f, 90.0f)).Quaternion();
		const double T = static_cast<double>(Sample % 101) / 100.0;
		const FVector3d Target = RuntimeCurvedPatch.EvaluateSection(T) +
			Random.FRandRange(-28.0f, 28.0f) * RuntimeCurvedPatch.ExtrusionAxis;
		Probe.Start = Target + FVector3d(Random.FRandRange(-50.0f, 50.0f),
			Random.FRandRange(-30.0f, 30.0f), Random.FRandRange(-20.0f, 20.0f));
		Probe.End = 2.0 * Target - Probe.Start;
		if (Sample % 4 == 0) Probe.Start = Probe.End = Target;
		const FWorldHit Actual = PrunedService.Sweep(Probe);
		const FWorldHit Expected = UnprunedService.Sweep(Probe);
		OracleHits += Expected.bHit ? 1 : 0;
		TestEqual(TEXT("Extruded BVH retains every contact/miss"), Actual.bHit, Expected.bHit);
		TestEqual(TEXT("Extruded BVH retains exact TOI"), Actual.Time, Expected.Time);
		TestEqual(TEXT("Extruded BVH retains exact overlap depth"), Actual.PenetrationDepth, Expected.PenetrationDepth);
		TestEqual(TEXT("Extruded BVH retains overlap classification"), Actual.bStartPenetrating, Expected.bStartPenetrating);
		TestTrue(TEXT("Extruded BVH retains exact witnesses and normal"),
			Actual.Point == Expected.Point && Actual.QueryPoint == Expected.QueryPoint &&
			Actual.Location == Expected.Location && Actual.Normal == Expected.Normal);
		TestTrue(TEXT("Extruded BVH retains feature arbitration"),
			Actual.PrimitiveId == Expected.PrimitiveId &&
			Actual.QueryFeatureKind == Expected.QueryFeatureKind &&
			Actual.SurfaceFeatureKind == Expected.SurfaceFeatureKind &&
			Actual.QueryFeatureIndex == Expected.QueryFeatureIndex &&
			Actual.SurfaceFeatureIndex == Expected.SurfaceFeatureIndex);
	}
	TestTrue(TEXT("Extruded BVH oracle actually exercises collisions"), OracleHits > 100);
	// Isolate facet rejection from all outer BVHs. Short sphere/ray probes near
	// every finite corner exercise the two-axis tolerance allowance explicitly.
	FAnalyticWorldData FacetOnlyWorld = UnprunedWorld;
	FacetOnlyWorld.ExtrudedQuinticPatches[0].SectionSegmentBounds = RuntimeCurvedPatch.SectionSegmentBounds;
	FWorldQueryService FacetOnlyService(FacetOnlyWorld);
	const auto SameBits = [](const auto& A, const auto& B)
	{
		return FMemory::Memcmp(&A, &B, sizeof(A)) == 0;
	};
	for (int32 Segment = 0; Segment < RuntimeCurvedPatch.SectionSegmentBounds.Num(); ++Segment)
	{
		const FVector3d A = RuntimeCurvedPatch.SectionPolyline[Segment];
		const FVector3d B = RuntimeCurvedPatch.SectionPolyline[Segment + 1];
		const FVector3d U = (B - A).GetSafeNormal();
		const FVector3d V = RuntimeCurvedPatch.ExtrusionAxis;
		const FVector3d Normal = FVector3d::CrossProduct(U, V).GetSafeNormal();
		for (const double Tolerance : { 0.0, 1.e-6, 0.01 })
		for (const double MarginScale : { -1.0, 0.0, 1.0, 1.01 })
		for (const EQueryShape Shape : { EQueryShape::Ray, EQueryShape::Sphere })
		for (int32 Corner = 0; Corner < 4; ++Corner)
		{
			const bool EndU = (Corner & 1) != 0, EndV = (Corner & 2) != 0;
			const FVector3d Target = (EndU ? B : A) +
				(EndV ? RuntimeCurvedPatch.MaximumExtrusionCoordinate : RuntimeCurvedPatch.MinimumExtrusionCoordinate) * V +
				MarginScale * Tolerance * ((EndU ? U : -U) + (EndV ? V : -V));
			FWorldQuery Probe;
			Probe.bIncludeCompactPatches = true;
			Probe.Shape = Shape;
			Probe.DomainTolerance = Tolerance;
			Probe.InitialOverlapTolerance = 0.0;
			Probe.Radius = Shape == EQueryShape::Sphere ? 0.01 : 0.0;
			Probe.Start = Target + (Probe.Radius + 1.e-4) * Normal;
			Probe.End = Target - 1.e-4 * Normal;
			const FWorldHit Actual = FacetOnlyService.Sweep(Probe);
			const FWorldHit Expected = UnprunedService.Sweep(Probe);
			if (!TestTrue(FString::Printf(TEXT("Facet corner oracle segment=%d corner=%d shape=%d tolerance=%.9g margin=%.9g"),
				Segment, Corner, int32(Shape), Tolerance, MarginScale),
				Actual.bHit == Expected.bHit && Actual.bStartPenetrating == Expected.bStartPenetrating &&
				SameBits(Actual.Time, Expected.Time) && SameBits(Actual.PenetrationDepth, Expected.PenetrationDepth) &&
				SameBits(Actual.Location, Expected.Location) && SameBits(Actual.Point, Expected.Point) &&
				SameBits(Actual.QueryPoint, Expected.QueryPoint) && SameBits(Actual.Normal, Expected.Normal) &&
				Actual.PrimitiveId == Expected.PrimitiveId && Actual.CanonicalGroupId == Expected.CanonicalGroupId &&
				Actual.QueryFeatureKind == Expected.QueryFeatureKind && Actual.SurfaceFeatureKind == Expected.SurfaceFeatureKind &&
				Actual.QueryFeatureIndex == Expected.QueryFeatureIndex && Actual.SurfaceFeatureIndex == Expected.SurfaceFeatureIndex)) return false;
		}
	}
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
	FWorldQuery RequiredCurvedPatchQuery = JoinBoxQuery;
	RequiredCurvedPatchQuery.RequiredSourceId = CurvedPatch.SourceId;
	RequiredCurvedPatchQuery.RequiredSurfaceId = CurvedPatch.SurfaceId;
	RequiredCurvedPatchQuery.RequiredCanonicalGroupId =
		CurvedPatch.CanonicalGroupId;
	const FWorldHit RequiredCurvedPatchHit =
		FWorldQueryService(CurvedWorld).Sweep(RequiredCurvedPatchQuery);
	TestTrue(TEXT("Stable provider restriction reacquires a curved patch"),
		RequiredCurvedPatchHit.bHit);
	TestEqual(TEXT("Curved support probe preserves the requested source"),
		RequiredCurvedPatchHit.SourceId, CurvedPatch.SourceId);
	TestEqual(TEXT("Curved support probe preserves the requested surface"),
		RequiredCurvedPatchHit.SurfaceId, CurvedPatch.SurfaceId);
	RequiredCurvedPatchQuery.ReferenceNormal = JoinNormal;
	RequiredCurvedPatchQuery.MinimumReferenceNormalDot = 0.9;
	TestTrue(TEXT("Established contact accepts a continuous provider normal"),
		FWorldQueryService(CurvedWorld).Sweep(RequiredCurvedPatchQuery).bHit);
	RequiredCurvedPatchQuery.ReferenceNormal = -JoinNormal;
	TestFalse(TEXT("Established contact rejects a discontinuous provider normal"),
		FWorldQueryService(CurvedWorld).Sweep(RequiredCurvedPatchQuery).bHit);
	RequiredCurvedPatchQuery.ReferenceNormal = JoinNormal;
	RequiredCurvedPatchQuery.RequiredCanonicalGroupId = 999;
	TestFalse(TEXT("Unknown canonical patch group reports no contact"),
		FWorldQueryService(CurvedWorld).Sweep(RequiredCurvedPatchQuery).bHit);
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

	OverlapQuery.RequiredSourceId = CurvedPatch.SourceId;
	OverlapQuery.RequiredSurfaceId = CurvedPatch.SurfaceId;
	OverlapQuery.RequiredCanonicalGroupId = CurvedPatch.CanonicalGroupId;
	const Speed::FAnalyticStaticCollisionWorld CurvedStaticWorld(CurvedWorld);
	const Speed::FEstablishedStaticContactProjection Projection =
		Speed::ProjectEstablishedStaticContact(
			CurvedStaticWorld, OverlapQuery, 4, 1.0e-6);
	TestTrue(TEXT("Established curved OBB contact is retained"),
		Projection.bContact);
	TestTrue(TEXT("Established curved OBB pose projection converges"),
		Projection.bConverged);
	TestTrue(TEXT("Established curved OBB projection has bounded cost"),
		Projection.CorrectionIterations <= 4);
	TestTrue(TEXT("Established curved OBB projection removes penetration"),
		Projection.ResidualPenetrationDepth <= 1.0e-6);
	TestTrue(TEXT("Established curved OBB projection moves outward"),
		FVector3d::DotProduct(
			Projection.Location - OverlapQuery.Start, JoinNormal) > 0.0);
	FWorldQuery ReprojectedQuery = OverlapQuery;
	ReprojectedQuery.Start = Projection.Location;
	ReprojectedQuery.End = Projection.Location;
	const Speed::FEstablishedStaticContactProjection Reprojection =
		Speed::ProjectEstablishedStaticContact(
			CurvedStaticWorld, ReprojectedQuery, 4, 1.0e-6);
	TestTrue(TEXT("Feasible curved OBB pose remains feasible"),
		Reprojection.ResidualPenetrationDepth <= 1.0e-6);
	TestEqual(TEXT("Feasible curved OBB pose needs no further correction"),
		Reprojection.CorrectionIterations, uint32(0));
	FWorldQuery UnknownProjectionQuery = OverlapQuery;
	UnknownProjectionQuery.RequiredCanonicalGroupId = 999;
	const Speed::FEstablishedStaticContactProjection UnknownProjection =
		Speed::ProjectEstablishedStaticContact(
			CurvedStaticWorld, UnknownProjectionQuery, 4, 1.0e-6);
	TestFalse(TEXT("Projection cannot switch to an unknown provider"),
		UnknownProjection.bContact);

	const FVector TransportFrom = FVector::UpVector;
	const FVector TransportTo = FVector(0.0f, 1.0f, 1.0f).GetSafeNormal();
	const FVector TangentialVelocity(0.0f, 100.0f, 0.0f);
	const FVector TransportedVelocity =
		Speed::TransportEstablishedContactVelocity(
			TangentialVelocity, FVector::ZeroVector,
			TransportFrom, TransportTo);
	TestTrue(TEXT("Curved contact transport preserves tangential speed"),
		FMath::IsNearlyEqual(
			TransportedVelocity.Size(), TangentialVelocity.Size(), 1.0e-4f));
	TestTrue(TEXT("Curved contact transport remains tangent"),
		FMath::Abs(FVector::DotProduct(
			TransportedVelocity, TransportTo)) <= 1.0e-4f);
	const FVector ReverseTransport =
		Speed::TransportEstablishedContactVelocity(
			TransportedVelocity, FVector::ZeroVector,
			TransportTo, TransportFrom);
	TestTrue(TEXT("Curved contact transport is symmetric"),
		ReverseTransport.Equals(TangentialVelocity, 1.0e-3f));
	const FVector RotationalClosingVelocity = -10.0f * TransportTo;
	const FVector CorrectedTransport =
		Speed::TransportEstablishedContactVelocity(
			TangentialVelocity, RotationalClosingVelocity,
			TransportFrom, TransportTo);
	TestTrue(TEXT("Curved contact transport removes residual closing speed"),
		FVector::DotProduct(
			CorrectedTransport + RotationalClosingVelocity,
			TransportTo) >= -1.0e-4f);
	TestTrue(TEXT("Curved contact transport is deterministic"),
		CorrectedTransport == Speed::TransportEstablishedContactVelocity(
			TangentialVelocity, RotationalClosingVelocity,
			TransportFrom, TransportTo));

	FWorldQuery ReleaseQuery = JoinBoxQuery;
	ReleaseQuery.Start = JoinPoint + (JoinSupport - 0.02) * JoinNormal;
	ReleaseQuery.End = ReleaseQuery.Start + JoinNormal;
	TestFalse(TEXT("Internal join releases a shallow separating overlap"),
		FWorldQueryService(CurvedWorld).Sweep(ReleaseQuery).bHit);
	TestTrue(TEXT("Authority coverage accepts the certified internal join"),
		FWorldQueryService(CurvedWorld).HasAuthorityCoverage(JoinBoxQuery));

	// A deliberately displaced physical provider may carry a local certificate
	// without weakening the public ten-centimetre agreement rule for every
	// other provider. Keep the residual source face earlier along the ray so the
	// selected primitive proves that arbitration consumed the local allowance.
	auto BuildAgreementWorld = [&](const double AdditionalAllowanceCm)
	{
		FAnalyticWorldData AgreementWorld;
		FExtrudedQuinticPatch AgreementPatch = Patch;
		AgreementPatch.SourceId = 90;
		AgreementPatch.SurfaceId = 91;
		AgreementPatch.FeatureId = 92;
		AgreementPatch.PrimitiveId = 93;
		AgreementPatch.CanonicalGroupId = 97;
		AgreementPatch.CanonicalSymmetryAxisMask = 1;
		AgreementPatch.bCanonicalC2ByConstruction = true;
		AgreementPatch.bCanonicalSymmetryByConstruction = true;
		AgreementPatch.bAuthorityEligible = true;
		AgreementPatch.AdditionalResidualAgreementAllowanceCm =
			AdditionalAllowanceCm;
		AgreementWorld.ExtrudedQuinticPatches.Add(AgreementPatch);
		FTriangleSurface ResidualFace;
		ResidualFace.SourceId = 90;
		ResidualFace.SurfaceId = 94;
		ResidualFace.FeatureId = 95;
		ResidualFace.PrimitiveId = 96;
		ResidualFace.Vertices[0] = FVector3d(-100.0, -100.0, 15.0);
		ResidualFace.Vertices[1] = FVector3d(100.0, -100.0, 15.0);
		ResidualFace.Vertices[2] = FVector3d(0.0, 100.0, 15.0);
		ResidualFace.FaceNormal = FVector3d::UpVector;
		for (FVector3d& Normal : ResidualFace.VertexNormals)
			Normal = FVector3d::UpVector;
		ResidualFace.Bounds = FBox3d(EForceInit::ForceInit);
		for (const FVector3d& Vertex : ResidualFace.Vertices)
			ResidualFace.Bounds += Vertex;
		ResidualFace.bQueryCollisionEnabled = true;
		ResidualFace.bAuthorityEligible = true;
		AgreementWorld.Triangles.Add(ResidualFace);
		return AgreementWorld;
	};
	FAnalyticWorldData DefaultAgreementWorld = BuildAgreementWorld(0.0);
	const bool bDefaultAgreementValid =
		DefaultAgreementWorld.FinalizeAndValidate(&ValidationReason);
	TestTrue(TEXT("Default residual-agreement fixture validates"),
		bDefaultAgreementValid);
	if (!bDefaultAgreementValid) AddInfo(ValidationReason);
	FWorldQuery AgreementQuery;
	AgreementQuery.Start = FVector3d(0.0, 0.0, 30.0);
	AgreementQuery.End = FVector3d(0.0, 0.0, -30.0);
	AgreementQuery.bAuthorityOnly = true;
	AgreementQuery.bIncludeCompactPatches = true;
	AgreementQuery.bIncludeTriangles = true;
	const FWorldHit DefaultAgreementHit =
		FWorldQueryService(DefaultAgreementWorld).Sweep(AgreementQuery);
	TestEqual(TEXT("Default agreement keeps a residual face fifteen centimetres away"),
		DefaultAgreementHit.PrimitiveId, uint64(96));
	FAnalyticWorldData CertifiedAgreementWorld = BuildAgreementWorld(6.0);
	const bool bCertifiedAgreementValid =
		CertifiedAgreementWorld.FinalizeAndValidate(&ValidationReason);
	TestTrue(TEXT("Provider-local agreement fixture validates"),
		bCertifiedAgreementValid);
	if (!bCertifiedAgreementValid) AddInfo(ValidationReason);
	const FWorldHit CertifiedAgreementHit =
		FWorldQueryService(CertifiedAgreementWorld).Sweep(AgreementQuery);
	TestEqual(TEXT("Provider-local allowance selects only the certified physical surface"),
		CertifiedAgreementHit.SurfaceId, uint64(91));
	TestTrue(TEXT("Selected hit carries the provider-local agreement allowance"),
		FMath::IsNearlyEqual(
			CertifiedAgreementHit.AdditionalResidualAgreementAllowanceCm, 6.0));

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
	FIAmSpeedTensorBezierSurfaceTest,
	"IAmSpeed.AnalyticWorld.TensorBezierSurface",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedTensorBezierSurfaceTest::RunTest(const FString& Parameters)
{
	using namespace Speed::Analytic;
	AddInfo(TEXT(
		"Contract=IAMSPEED.PHYS.CONTACT_DETECTION.V1.TENSOR_APPROXIMATION"));

	FTensorBezierSurface Plane;
	Plane.DegreeU = 1;
	Plane.DegreeV = 1;
	Plane.ControlPoints = {
		FVector3d(0.0, 0.0, 0.0), FVector3d(0.0, 20.0, 0.0),
		FVector3d(10.0, 0.0, 0.0), FVector3d(10.0, 20.0, 0.0) };
	FString Reason;
	TestTrue(TEXT("Regular bilinear tensor surface validates"),
		Plane.IsValid(&Reason));
	TestTrue(TEXT("Tensor surface midpoint is exact"),
		Plane.Evaluate(0.5, 0.5).Equals(FVector3d(5.0, 10.0, 0.0), 1.0e-12));
	TestTrue(TEXT("Tensor U derivative is exact"),
		Plane.EvaluateDerivativeU(0.37, 0.61).Equals(
			FVector3d(10.0, 0.0, 0.0), 1.0e-12));
	TestTrue(TEXT("Tensor V derivative is exact"),
		Plane.EvaluateDerivativeV(0.37, 0.61).Equals(
			FVector3d(0.0, 20.0, 0.0), 1.0e-12));
	TestTrue(TEXT("Tensor normal follows control-net orientation"),
		Plane.EvaluateNormal(0.5, 0.5).Equals(FVector3d::UpVector, 1.0e-12));

	FTensorBezierSurface Curved;
	Curved.DegreeU = 2;
	Curved.DegreeV = 2;
	for (int32 UIndex = 0; UIndex <= 2; ++UIndex)
	{
		for (int32 VIndex = 0; VIndex <= 2; ++VIndex)
		{
			Curved.ControlPoints.Add(FVector3d(
				10.0 * UIndex, 10.0 * VIndex,
				UIndex == 1 && VIndex == 1 ? 8.0 : 0.0));
		}
	}
	TestTrue(TEXT("Regular curved tensor surface validates"),
		Curved.IsValid(&Reason));
	TestTrue(TEXT("Interior tensor control changes the surface interior"),
		Curved.Evaluate(0.5, 0.5).Z > 0.0);
	TestTrue(TEXT("Curved tensor normal remains normalized"),
		FMath::IsNearlyEqual(
			Curved.EvaluateNormal(0.31, 0.67).SquaredLength(), 1.0, 1.0e-12));
	TestTrue(TEXT("Tensor evaluation is exactly deterministic"),
		Curved.Evaluate(0.3125, 0.6875) ==
			Curved.Evaluate(0.3125, 0.6875));
	TArray<FTensorBezierApproximationCell> Cells;
	double CertifiedMaximumErrorCm = 0.0;
	TestTrue(TEXT("Curved tensor builds a certified bilinear approximation"),
		Curved.BuildBilinearApproximation(
			0.05, Cells, &CertifiedMaximumErrorCm));
	TestTrue(TEXT("Curved tensor subdivides beyond one bilinear cell"),
		Cells.Num() > 1);
	TestTrue(TEXT("Every accepted tensor cell respects the requested bound"),
		CertifiedMaximumErrorCm <= 0.05);
	double MeasuredMaximumErrorCm = 0.0;
	for (const FTensorBezierApproximationCell& Cell : Cells)
	{
		for (int32 UIndex = 0; UIndex <= 4; ++UIndex)
		{
			const double LocalU = static_cast<double>(UIndex) / 4.0;
			const double U = FMath::Lerp(Cell.MinimumU, Cell.MaximumU, LocalU);
			for (int32 VIndex = 0; VIndex <= 4; ++VIndex)
			{
				const double LocalV = static_cast<double>(VIndex) / 4.0;
				const double V = FMath::Lerp(
					Cell.MinimumV, Cell.MaximumV, LocalV);
				const FVector3d Bilinear = FMath::Lerp(
					FMath::Lerp(Cell.Corners[0], Cell.Corners[1], LocalV),
					FMath::Lerp(Cell.Corners[2], Cell.Corners[3], LocalV),
					LocalU);
				MeasuredMaximumErrorCm = FMath::Max(MeasuredMaximumErrorCm,
					FVector3d::Distance(Curved.Evaluate(U, V), Bilinear));
			}
		}
	}
	TestTrue(TEXT("Measured tensor deviation stays below the certified bound"),
		MeasuredMaximumErrorCm <= CertifiedMaximumErrorCm + 1.0e-9);

	FTensorBezierPatch Patch;
	Patch.SourceId = 70;
	Patch.SurfaceId = 71;
	Patch.FeatureId = 72;
	Patch.PrimitiveId = 73;
	Patch.CanonicalGroupId = 74;
	Patch.MaterialId = 75;
	Patch.ObjectType = 2;
	Patch.BlockingChannels = 1ull << 3;
	Patch.Surface = Curved;
	Patch.bQueryCollisionEnabled = true;
	FAnalyticWorldData World;
	World.TensorBezierPatches.Add(Patch);
	TestTrue(TEXT("Draft tensor-patch world validates"),
		World.FinalizeAndValidate(&Reason));
	TestTrue(TEXT("Tensor provider builds certified finite cells"),
		World.TensorBezierPatches[0].bApproximationCertified &&
			World.TensorBezierPatches[0].MaximumApproximationErrorCm <= 0.01);
	TestTrue(TEXT("Tensor provider builds a deterministic cell BVH"),
		!World.TensorBezierPatches[0].ApproximationCellBvhNodes.IsEmpty() &&
		World.TensorBezierPatches[0].ApproximationCellBvhIndices.Num() ==
			World.TensorBezierPatches[0].ApproximationCells.Num());
	FWorldQuery RayQuery;
	RayQuery.Shape = EQueryShape::Ray;
	RayQuery.Start = FVector3d(10.0, 10.0, 20.0);
	RayQuery.End = FVector3d(10.0, 10.0, -20.0);
	RayQuery.bIncludeCompactPatches = true;
	RayQuery.bApplyCollisionFilter = true;
	RayQuery.TraceChannel = 3;
	const FWorldHit DraftRayHit = FWorldQueryService(World).Sweep(RayQuery);
	TestTrue(TEXT("Ray hits a draft tensor provider"), DraftRayHit.bHit);
	TestEqual(TEXT("Tensor hit preserves source identity"),
		DraftRayHit.SourceId, Patch.SourceId);
	TestEqual(TEXT("Tensor hit preserves surface identity"),
		DraftRayHit.SurfaceId, Patch.SurfaceId);
	TestEqual(TEXT("Tensor hit preserves canonical group"),
		DraftRayHit.CanonicalGroupId, Patch.CanonicalGroupId);
	TestTrue(TEXT("Tensor hit exposes its certified geometric error"),
		DraftRayHit.GeometricErrorBoundCm <= 0.01);
	TestTrue(TEXT("Tensor hit reports its varying smooth normal"),
		DraftRayHit.bSurfaceNormalMayVary && DraftRayHit.Normal.Z > 0.999);
	FWorldQuery AuthorityRayQuery = RayQuery;
	AuthorityRayQuery.bAuthorityOnly = true;
	TestFalse(TEXT("Draft tensor provider is excluded from authority"),
		FWorldQueryService(World).Sweep(AuthorityRayQuery).bHit);
	World.TensorBezierPatches[0].bAuthorityEligible = true;
	AuthorityRayQuery.RequiredCanonicalGroupId = Patch.CanonicalGroupId;
	const FWorldHit AuthorityRayHit =
		FWorldQueryService(World).Sweep(AuthorityRayQuery);
	TestTrue(TEXT("Certified tensor provider may answer an authority ray"),
		AuthorityRayHit.bHit);
	FWorldQuery WrongCanonicalGroupQuery = AuthorityRayQuery;
	WrongCanonicalGroupQuery.RequiredCanonicalGroupId = Patch.CanonicalGroupId + 1;
	TestFalse(TEXT("Tensor provider rejects a different canonical family"),
		FWorldQueryService(World).Sweep(WrongCanonicalGroupQuery).bHit);
	FWorldQuery SphereQuery = AuthorityRayQuery;
	SphereQuery.Shape = EQueryShape::Sphere;
	SphereQuery.Radius = 1.0;
	const FWorldHit SphereHit = FWorldQueryService(World).Sweep(SphereQuery);
	TestTrue(TEXT("Sphere sweep hits a certified tensor provider"),
		SphereHit.bHit);
	TestEqual(TEXT("Sphere selects the convex tensor tessellation vertex"),
		SphereHit.SurfaceFeatureKind, EContactFeatureKind::Vertex);
	const FWorldHit RepeatedSphereHit =
		FWorldQueryService(World).Sweep(SphereQuery);
	TestTrue(TEXT("Tensor sphere sweep is exactly deterministic"),
		RepeatedSphereHit.bHit == SphereHit.bHit &&
		RepeatedSphereHit.Time == SphereHit.Time &&
		RepeatedSphereHit.Point == SphereHit.Point &&
		RepeatedSphereHit.Normal == SphereHit.Normal &&
		RepeatedSphereHit.PrimitiveId == SphereHit.PrimitiveId);
	FWorldQuery SphereOverlapQuery = SphereQuery;
	SphereOverlapQuery.Start = FVector3d(10.0, 10.0, 2.5);
	SphereOverlapQuery.End = SphereOverlapQuery.Start;
	const FWorldHit SphereOverlapHit =
		FWorldQueryService(World).Sweep(SphereOverlapQuery);
	AddInfo(FString::Printf(TEXT(
		"TensorOverlap Hit=%d StartPenetrating=%d Depth=%.17g Bound=%.17g Feature=%u"),
		SphereOverlapHit.bHit ? 1 : 0,
		SphereOverlapHit.bStartPenetrating ? 1 : 0,
		SphereOverlapHit.PenetrationDepth,
		SphereOverlapHit.GeometricErrorBoundCm,
		static_cast<uint8>(SphereOverlapHit.SurfaceFeatureKind)));
	TestTrue(TEXT("Tensor sphere reports an initial overlap"),
		SphereOverlapHit.bHit && SphereOverlapHit.bStartPenetrating);
	TestTrue(TEXT("Tensor sphere penetration respects its geometric certificate"),
		FMath::Abs(SphereOverlapHit.PenetrationDepth - 0.5) <=
			SphereOverlapHit.GeometricErrorBoundCm + 1.0e-8);
	FWorldQuery SphereReleaseQuery = SphereOverlapQuery;
	SphereReleaseQuery.Start.Z = 2.975;
	SphereReleaseQuery.End = SphereReleaseQuery.Start + FVector3d(0.0, 0.0, 1.0);
	TestFalse(TEXT("Tensor sphere releases a shallow separating overlap"),
		FWorldQueryService(World).Sweep(SphereReleaseQuery).bHit);
	SphereReleaseQuery.End = SphereReleaseQuery.Start - FVector3d(0.0, 0.0, 1.0);
	const FWorldHit SphereClosingShellHit =
		FWorldQueryService(World).Sweep(SphereReleaseQuery);
	TestTrue(TEXT("Tensor sphere preserves a shallow closing contact"),
		SphereClosingShellHit.bHit && !SphereClosingShellHit.bStartPenetrating &&
		SphereClosingShellHit.Time == 0.0);
	FWorldQuery BoxQuery = AuthorityRayQuery;
	BoxQuery.Shape = EQueryShape::Box;
	BoxQuery.HalfExtent = FVector3d(1.0, 1.5, 2.0);
	BoxQuery.Rotation = FQuat4d(FVector3d::UpVector,
		FMath::DegreesToRadians(17.0));
	TestTrue(TEXT("Oriented box sweep hits a certified tensor provider"),
		FWorldQueryService(World).Sweep(BoxQuery).bHit);

	FTensorBezierSurface Invalid = Plane;
	// A point away from the diagonal belongs strictly to local triangle 0.
	// All three kernels must decorate its stable identity after a real hit.
	FAnalyticWorldData FlatWorld;
	FTensorBezierPatch FlatPatch = Patch;
	FlatPatch.Surface = Plane;
	FlatWorld.TensorBezierPatches.Add(FlatPatch);
	if (!TestTrue(TEXT("Flat tensor witness fixture validates"),
		FlatWorld.FinalizeAndValidate(&Reason))) return false;
	TestEqual(TEXT("Flat tensor fixture has exactly one cell"),
		FlatWorld.TensorBezierPatches[0].ApproximationCells.Num(), 1);
	FWorldQuery FlatQuery;
	FlatQuery.Start = FVector3d(7.0, 3.0, 20.0);
	FlatQuery.End = FVector3d(7.0, 3.0, -20.0);
	FlatQuery.Radius = 0.1;
	FlatQuery.HalfExtent = FVector3d(0.1);
	FlatQuery.bIncludeCompactPatches = true;
	for (EQueryShape Shape : { EQueryShape::Ray, EQueryShape::Sphere, EQueryShape::Box })
	{
		FlatQuery.Shape = Shape;
		const FWorldHit FlatHit = FWorldQueryService(FlatWorld).Sweep(FlatQuery);
		TestTrue(TEXT("Each query shape hits the expected tensor triangle"), FlatHit.bHit);
		TestEqual(TEXT("Tensor hit gets the original stable triangle id"),
			FlatHit.PrimitiveId, CombineStableIds(FlatPatch.PrimitiveId, uint64(1)));
		TestEqual(TEXT("Tensor hit retains authored feature id"), FlatHit.FeatureId, FlatPatch.FeatureId);
		TestEqual(TEXT("Tensor hit retains material id"), FlatHit.MaterialId, FlatPatch.MaterialId);
	}
	Invalid.ControlPoints.Pop();
	TestFalse(TEXT("Incomplete tensor control net is rejected"),
		Invalid.IsValid(&Reason));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FIAmSpeedPiecewiseTensorBezierProviderTest,
	"IAmSpeed.AnalyticWorld.PiecewiseTensorBezierProvider",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedPiecewiseTensorBezierProviderTest::RunTest(
	const FString& Parameters)
{
	using namespace Speed::Analytic;
	AddInfo(TEXT(
		"Contract=IAMSPEED.PHYS.CONTACT_DETECTION.V1.PIECEWISE_TENSOR_C2"));

	FPiecewiseTensorBezierPatch Patch;
	Patch.SourceId = 810;
	Patch.SurfaceId = 811;
	Patch.PrimitiveId = 812;
	Patch.CanonicalGroupId = 813;
	Patch.MaterialId = 814;
	Patch.ObjectType = 2;
	Patch.BlockingChannels = 1ull << 3;
	Patch.bQueryCollisionEnabled = true;
	Patch.bSourceResidualCertified = true;
	Patch.bAuthorityEligible = true;
	for (int32 CellIndex = 0; CellIndex < 2; ++CellIndex)
	{
		FPiecewiseTensorBezierCell& Cell = Patch.Cells.AddDefaulted_GetRef();
		Cell.FeatureId = 820 + CellIndex;
		Cell.PrimitiveId = 830 + CellIndex;
		Cell.MinimumU = 0.5 * CellIndex;
		Cell.MaximumU = 0.5 * (CellIndex + 1);
		Cell.Surface.DegreeU = 3;
		Cell.Surface.DegreeV = 3;
		for (int32 U = 0; U <= 3; ++U)
		{
			for (int32 V = 0; V <= 3; ++V)
			{
				Cell.Surface.ControlPoints.Add(FVector3d(
					10.0 * CellIndex + 10.0 * U / 3.0,
					20.0 * V / 3.0, 0.0));
			}
		}
	}
	FAnalyticWorldData World;
	World.PiecewiseTensorBezierPatches.Add(Patch);
	FTensorBezierPatch OutOfDomainPatch;
	OutOfDomainPatch.SourceId = 810;
	OutOfDomainPatch.SurfaceId = 840;
	OutOfDomainPatch.FeatureId = 841;
	OutOfDomainPatch.PrimitiveId = 842;
	OutOfDomainPatch.CanonicalGroupId = 843;
	OutOfDomainPatch.MaterialId = 814;
	OutOfDomainPatch.ObjectType = 2;
	OutOfDomainPatch.BlockingChannels = 1ull << 3;
	OutOfDomainPatch.Surface.DegreeU = 1;
	OutOfDomainPatch.Surface.DegreeV = 1;
	OutOfDomainPatch.Surface.ControlPoints = {
		FVector3d(-100.0, -100.0, 15.0), FVector3d(-100.0, 100.0, 15.0),
		FVector3d(100.0, -100.0, 15.0), FVector3d(100.0, 100.0, 15.0) };
	OutOfDomainPatch.bQueryCollisionEnabled = true;
	OutOfDomainPatch.bAuthorityEligible = true;
	World.TensorBezierPatches.Add(OutOfDomainPatch);
	FTriangleSurface ResidualTriangle;
	ResidualTriangle.SourceId = 810;
	ResidualTriangle.SurfaceId = 850;
	ResidualTriangle.FeatureId = 851;
	ResidualTriangle.PrimitiveId = 852;
	ResidualTriangle.MaterialId = 814;
	ResidualTriangle.ObjectType = 2;
	ResidualTriangle.BlockingChannels = 1ull << 3;
	ResidualTriangle.Vertices[0] = FVector3d(-100.0, -100.0, 0.0);
	ResidualTriangle.Vertices[1] = FVector3d(100.0, -100.0, 0.0);
	ResidualTriangle.Vertices[2] = FVector3d(0.0, 100.0, 0.0);
	ResidualTriangle.VertexNormals[0] = FVector3d::UpVector;
	ResidualTriangle.VertexNormals[1] = FVector3d::UpVector;
	ResidualTriangle.VertexNormals[2] = FVector3d::UpVector;
	ResidualTriangle.FaceNormal = FVector3d::UpVector;
	ResidualTriangle.Bounds = FBox3d(EForceInit::ForceInit);
	for (const FVector3d& Vertex : ResidualTriangle.Vertices)
	{
		ResidualTriangle.Bounds += Vertex;
	}
	ResidualTriangle.bQueryCollisionEnabled = true;
	ResidualTriangle.bAuthorityEligible = true;
	World.Triangles.Add(ResidualTriangle);
	FString Reason;
	if (!TestTrue(TEXT("Certified piecewise tensor world validates"),
		World.FinalizeAndValidate(&Reason)))
	{
		AddError(Reason);
		return false;
	}
	const FPiecewiseTensorBezierPatch& Certified =
		World.PiecewiseTensorBezierPatches[0];
	TestEqual(TEXT("Both directed seam adjacencies are recorded"),
		Certified.Adjacencies.Num(), 2);
	bool bAllAdjacenciesCertifiedC2 = true;
	for (const FPiecewiseTensorBezierAdjacency& Link : Certified.Adjacencies)
	{
		bAllAdjacenciesCertifiedC2 &= Link.bC2ByConstruction;
	}
	TestTrue(TEXT("Every recorded seam has a numerical C2 certificate"),
		bAllAdjacenciesCertifiedC2);

	FWorldQuery Query;
	Query.Start = FVector3d(10.0, 10.0, 20.0);
	Query.End = FVector3d(10.0, 10.0, -20.0);
	Query.bIncludeCompactPatches = true;
	Query.bAuthorityOnly = true;
	Query.RequiredSurfaceId = Patch.SurfaceId;
	const FWorldHit RayHit = FWorldQueryService(World).Sweep(Query);
	TestTrue(TEXT("Authority ray hits the piecewise provider at its C2 seam"),
		RayHit.bHit && RayHit.SurfaceId == Patch.SurfaceId);
	const FWorldHit RepeatedRayHit = FWorldQueryService(World).Sweep(Query);
	TestTrue(TEXT("Piecewise seam selection is exactly deterministic"),
		RepeatedRayHit.bHit == RayHit.bHit &&
		RepeatedRayHit.Time == RayHit.Time &&
		RepeatedRayHit.Point == RayHit.Point &&
		RepeatedRayHit.Normal == RayHit.Normal &&
		RepeatedRayHit.PrimitiveId == RayHit.PrimitiveId);

	FWorldQuery SphereQuery = Query;
	SphereQuery.Shape = EQueryShape::Sphere;
	SphereQuery.Radius = 1.0;
	TestTrue(TEXT("Sphere sweep hits the piecewise provider"),
		FWorldQueryService(World).Sweep(SphereQuery).bHit);
	FWorldQuery SphereOverlap = SphereQuery;
	SphereOverlap.Start = FVector3d(10.0, 10.0, 0.5);
	SphereOverlap.End = SphereOverlap.Start;
	const FWorldHit OverlapHit = FWorldQueryService(World).Sweep(SphereOverlap);
	TestTrue(TEXT("Piecewise sphere reports initial penetration"),
		OverlapHit.bHit && OverlapHit.bStartPenetrating &&
		FMath::Abs(OverlapHit.PenetrationDepth - 0.5) <=
			OverlapHit.GeometricErrorBoundCm + 1.0e-8);
	FWorldQuery SphereRelease = SphereOverlap;
	SphereRelease.Start.Z = 0.975;
	SphereRelease.End = SphereRelease.Start + FVector3d(0.0, 0.0, 1.0);
	TestFalse(TEXT("Piecewise sphere releases a shallow separating overlap"),
		FWorldQueryService(World).Sweep(SphereRelease).bHit);
	SphereRelease.End = SphereRelease.Start - FVector3d(0.0, 0.0, 1.0);
	const FWorldHit ClosingHit = FWorldQueryService(World).Sweep(SphereRelease);
	TestTrue(TEXT("Piecewise sphere preserves a shallow closing contact"),
		ClosingHit.bHit && !ClosingHit.bStartPenetrating &&
		ClosingHit.Time == 0.0);

	FWorldQuery BoxQuery = Query;
	BoxQuery.Shape = EQueryShape::Box;
	BoxQuery.HalfExtent = FVector3d(1.0, 1.5, 2.0);
	BoxQuery.Rotation = FQuat4d(FVector3d::UpVector,
		FMath::DegreesToRadians(17.0));
	TestTrue(TEXT("Oriented box sweep hits the piecewise provider"),
		FWorldQueryService(World).Sweep(BoxQuery).bHit);
	FWorldQuery ArbitrationQuery = Query;
	ArbitrationQuery.RequiredSurfaceId = 0;
	ArbitrationQuery.bIncludeTriangles = true;
	const FWorldHit ArbitrationHit = FWorldQueryService(World).Sweep(ArbitrationQuery);
	AddInfo(FString::Printf(TEXT(
		"AgreementArbitration Surface=%llu Primitive=%llu Time=%.17g Point=%s"),
		ArbitrationHit.SurfaceId, ArbitrationHit.PrimitiveId, ArbitrationHit.Time,
		*ArbitrationHit.Point.ToString()));
	TestTrue(TEXT("Authority selects an agreeing polished provider behind an earlier out-of-domain provider"),
		ArbitrationHit.bHit && ArbitrationHit.SurfaceId == Patch.SurfaceId);

	// Exercise many providers with deliberately non-spatial
	// insertion order. Each provider has a unique identity and translated cells.
	FAnalyticWorldData IndexedWorld;
	for (int32 Index = 40; Index >= 0; --Index)
	{
		FPiecewiseTensorBezierPatch Copy = Patch;
		Copy.SourceId += 1000 * Index;
		Copy.SurfaceId += 1000 * Index;
		// Two spatially distinct primitives deliberately share the same surface:
		// an indexed lookup must retain both, not just the first or last match.
		if (Index == 1) Copy.SurfaceId = Patch.SurfaceId;
		Copy.PrimitiveId += 1000 * Index;
		Copy.CanonicalGroupId += 1000 * Index;
		for (FPiecewiseTensorBezierCell& Cell : Copy.Cells)
		{
			Cell.FeatureId += 1000 * Index;
			Cell.PrimitiveId += 1000 * Index;
			for (FVector3d& Point : Cell.Surface.ControlPoints)
			{
				Point.X += 100.0 * Index;
			}
		}
		IndexedWorld.PiecewiseTensorBezierPatches.Add(MoveTemp(Copy));
	}
	if (!TestTrue(TEXT("Indexed provider world validates"),
		IndexedWorld.FinalizeAndValidate(&Reason)))
	{
		AddError(Reason);
		return false;
	}
	FWorldQueryService IndexedService(IndexedWorld);
	for (int32 Index = 0; Index <= 40; ++Index)
	{
		FWorldQuery IndexedQuery = Query;
		IndexedQuery.RequiredSurfaceId = 0;
		IndexedQuery.Start.X += 100.0 * Index;
		IndexedQuery.End.X += 100.0 * Index;
		const FWorldHit Hit = IndexedService.Sweep(IndexedQuery);
		const uint64 ExpectedSurfaceId = Patch.SurfaceId + (Index == 1 ? 0 : 1000 * Index);
		TestTrue(TEXT("Broad phase reaches every provider"),
			Hit.bHit && Hit.SurfaceId == ExpectedSurfaceId &&
			FMath::IsNearlyEqual(Hit.Time, RayHit.Time, 1.0e-12));
		IndexedQuery.RequiredSurfaceId = ExpectedSurfaceId;
		const FWorldHit RestrictedHit = IndexedService.Sweep(IndexedQuery);
		TestTrue(TEXT("Restricted lookup preserves the exact unrestricted winner"),
			RestrictedHit.bHit && RestrictedHit.SurfaceId == Hit.SurfaceId &&
			RestrictedHit.PrimitiveId == Hit.PrimitiveId &&
			RestrictedHit.Time == Hit.Time && RestrictedHit.Point == Hit.Point &&
			RestrictedHit.Normal == Hit.Normal);
		IndexedQuery.RequiredSourceId = MAX_uint64;
		TestFalse(TEXT("Indexed surface lookup still applies source filtering"),
			IndexedService.Sweep(IndexedQuery).bHit);
		IndexedQuery.RequiredSourceId = 0;
		IndexedQuery.RequiredSurfaceId = MAX_uint64;
		TestFalse(TEXT("An absent surface does not fall back to unrestricted providers"),
			IndexedService.Sweep(IndexedQuery).bHit);
	}
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
	FWorldQuery BoxQuery = Query;
	BoxQuery.Shape = EQueryShape::Box;
	BoxQuery.Radius = 0.0;
	BoxQuery.HalfExtent = FVector3d(5.0, 5.0, 5.0);
	const FWorldHit BoxHit = FWorldQueryService(World).Sweep(BoxQuery);
	TestTrue(TEXT("Box hits residual triangle face"), BoxHit.bHit);
	TestTrue(TEXT("Residual triangle box TOI is exact"),
		FMath::IsNearlyEqual(BoxHit.Time, 0.375, 1.0e-12));
	TestTrue(TEXT("Residual triangle box exposes native witnesses"),
		BoxHit.QueryFeatureKind != EContactFeatureKind::Unknown &&
		BoxHit.SurfaceFeatureKind != EContactFeatureKind::Unknown);

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
	FIAmSpeedAnalyticProviderSolidSubBodyMatrixTest,
	"IAmSpeed.AnalyticWorld.ProviderSolidSubBodyMatrix",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedAnalyticProviderSolidSubBodyMatrixTest::RunTest(
	const FString& Parameters)
{
	using namespace Speed::Analytic;
	struct FProviderFixture
	{
		const TCHAR* Name;
		uint64 SurfaceId;
		FAnalyticWorldData World;
	};
	TArray<FProviderFixture> Fixtures;
	{
		FProviderFixture& Fixture = Fixtures.AddDefaulted_GetRef();
		Fixture.Name = TEXT("BoundedPlane");
		Fixture.SurfaceId = 1001;
		FBoundedPlane& Plane = Fixture.World.Planes.AddDefaulted_GetRef();
		Plane.SourceId = 1000;
		Plane.SurfaceId = Fixture.SurfaceId;
		Plane.FeatureId = 1002;
		Plane.PrimitiveId = 1003;
		Plane.Origin = FVector3d::ZeroVector;
		Plane.HalfExtents = FVector2d(400.0);
		Plane.bQueryCollisionEnabled = true;
		Plane.bAuthorityEligible = true;
	}
	{
		FProviderFixture& Fixture = Fixtures.AddDefaulted_GetRef();
		Fixture.Name = TEXT("ExtrudedQuintic");
		Fixture.SurfaceId = 1101;
		FExtrudedQuinticPatch& Patch =
			Fixture.World.ExtrudedQuinticPatches.AddDefaulted_GetRef();
		Patch.SourceId = 1100;
		Patch.SurfaceId = Fixture.SurfaceId;
		Patch.FeatureId = 1102;
		Patch.PrimitiveId = 1103;
		Patch.CanonicalGroupId = 1104;
		for (int32 Index = 0; Index < 6; ++Index)
		{
			Patch.SectionControlPoints[Index] = FVector3d(
				FMath::Lerp(-200.0, 200.0, static_cast<double>(Index) / 5.0),
				0.0, 0.0);
		}
		Patch.ExtrusionAxis = FVector3d::RightVector;
		Patch.MinimumExtrusionCoordinate = -200.0;
		Patch.MaximumExtrusionCoordinate = 200.0;
		Patch.bQueryCollisionEnabled = true;
		Patch.bCanonicalC2ByConstruction = true;
		Patch.bCanonicalSymmetryByConstruction = true;
		Patch.BuildQueryApproximation();
		Patch.bAuthorityEligible = true;
	}
	{
		FProviderFixture& Fixture = Fixtures.AddDefaulted_GetRef();
		Fixture.Name = TEXT("TensorBezier");
		Fixture.SurfaceId = 1201;
		FTensorBezierPatch& Patch =
			Fixture.World.TensorBezierPatches.AddDefaulted_GetRef();
		Patch.SourceId = 1200;
		Patch.SurfaceId = Fixture.SurfaceId;
		Patch.FeatureId = 1202;
		Patch.PrimitiveId = 1203;
		Patch.CanonicalGroupId = 1204;
		Patch.Surface.DegreeU = Patch.Surface.DegreeV = 1;
		Patch.Surface.ControlPoints = {
			FVector3d(-200.0, -200.0, 0.0),
			FVector3d(-200.0, 200.0, 0.0),
			FVector3d(200.0, -200.0, 0.0),
			FVector3d(200.0, 200.0, 0.0) };
		Patch.bQueryCollisionEnabled = true;
		Patch.bAuthorityEligible = true;
	}
	{
		FProviderFixture& Fixture = Fixtures.AddDefaulted_GetRef();
		Fixture.Name = TEXT("PiecewiseTensorBezier");
		Fixture.SurfaceId = 1301;
		FPiecewiseTensorBezierPatch& Patch =
			Fixture.World.PiecewiseTensorBezierPatches.AddDefaulted_GetRef();
		Patch.SourceId = 1300;
		Patch.SurfaceId = Fixture.SurfaceId;
		Patch.PrimitiveId = 1302;
		Patch.CanonicalGroupId = 1303;
		Patch.bQueryCollisionEnabled = true;
		Patch.bSourceResidualCertified = true;
		Patch.bAuthorityEligible = true;
		FPiecewiseTensorBezierCell& Cell = Patch.Cells.AddDefaulted_GetRef();
		Cell.FeatureId = 1304;
		Cell.PrimitiveId = 1305;
		Cell.MinimumU = Cell.MinimumV = 0.0;
		Cell.MaximumU = Cell.MaximumV = 1.0;
		Cell.Surface.DegreeU = Cell.Surface.DegreeV = 1;
		Cell.Surface.ControlPoints = {
			FVector3d(-200.0, -200.0, 0.0),
			FVector3d(-200.0, 200.0, 0.0),
			FVector3d(200.0, -200.0, 0.0),
			FVector3d(200.0, 200.0, 0.0) };
	}

	struct FProxyShape
	{
		const TCHAR* Name;
		EQueryShape Shape;
		double Radius;
		FVector3d HalfExtent;
		double VerticalSupport;
	};
	const FProxyShape Shapes[] = {
		{ TEXT("SmallSphere"), EQueryShape::Sphere, 10.0,
			FVector3d::ZeroVector, 10.0 },
		{ TEXT("LargeSphere"), EQueryShape::Sphere, 109.5,
			FVector3d::ZeroVector, 109.5 },
		{ TEXT("OrientedBox"), EQueryShape::Box, 0.0,
			FVector3d(20.0, 15.0, 10.0), 10.0 }
	};
	for (FProviderFixture& Fixture : Fixtures)
	{
		FString Reason;
		if (!TestTrue(*FString::Printf(TEXT("%s fixture validates"), Fixture.Name),
			Fixture.World.FinalizeAndValidate(&Reason)))
		{
			AddError(Reason);
			continue;
		}
		for (const FProxyShape& Shape : Shapes)
		{
			auto MakeQuery = [&]()
			{
				FWorldQuery Query;
				Query.Shape = Shape.Shape;
				Query.Radius = Shape.Radius;
				Query.HalfExtent = Shape.HalfExtent;
				Query.Rotation = FQuat4d(FVector3d::UpVector,
					FMath::DegreesToRadians(17.0));
				Query.RequiredSurfaceId = Fixture.SurfaceId;
				Query.bIncludeCompactPatches = true;
				Query.bAuthorityOnly = true;
				return Query;
			};
			const FString Context = FString::Printf(TEXT("%s x %s"),
				Fixture.Name, Shape.Name);
			FWorldQuery Sweep = MakeQuery();
			Sweep.Start = FVector3d(0.0, 0.0, 200.0);
			Sweep.End = FVector3d(0.0, 0.0, -200.0);
			const FWorldHit SweepHit =
				FWorldQueryService(Fixture.World).Sweep(Sweep);
			TestTrue(*(Context + TEXT(" sweep")), SweepHit.bHit &&
				!SweepHit.bStartPenetrating && SweepHit.Normal.Z > 0.99);

			FWorldQuery Overlap = MakeQuery();
			Overlap.Start = FVector3d(0.0, 0.0,
				Shape.VerticalSupport - 1.0);
			Overlap.End = Overlap.Start;
			const FWorldHit OverlapHit =
				FWorldQueryService(Fixture.World).Sweep(Overlap);
			TestTrue(*(Context + TEXT(" overlap")), OverlapHit.bHit &&
				OverlapHit.bStartPenetrating &&
				OverlapHit.PenetrationDepth > 0.0);

			FWorldQuery Release = MakeQuery();
			Release.Start = FVector3d(0.0, 0.0,
				Shape.VerticalSupport + 0.1);
			Release.End = Release.Start + FVector3d(0.0, 0.0, 5.0);
			TestFalse(*(Context + TEXT(" release")),
				FWorldQueryService(Fixture.World).Sweep(Release).bHit);
		}
	}
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
