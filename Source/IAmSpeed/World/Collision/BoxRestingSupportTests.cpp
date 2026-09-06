#if WITH_DEV_AUTOMATION_TESTS
#include "BoxRestingSupport.h"
#include "IAmSpeed/Components/SpeedMovementComponent.h"
#include "IAmSpeed/SubBodies/Solid/BoxSubBody.h"
#include "Misc/AutomationTest.h"
#include "UObject/StrongObjectPtr.h"
#include <cmath>

namespace
{
	Speed::Analytic::FAnalyticWorldData RestingPlane(const FVector& Offset = FVector::ZeroVector)
	{
		Speed::Analytic::FAnalyticWorldData Data;
		Speed::Analytic::FBoundedPlane Plane;
		Plane.SourceId = 1; Plane.SurfaceId = 2; Plane.FeatureId = 3; Plane.PrimitiveId = 4;
		Plane.Origin = Offset;
		Plane.HalfExtents = FVector2d(1000, 1000);
		Plane.BlockingChannels = MAX_uint64;
		Plane.bAuthorityEligible = true;
		Data.Planes.Add(Plane);
		check(Data.FinalizeAndValidate());
		return Data;
	}

	Speed::Analytic::FWorldQuery RestingBox()
	{
		Speed::Analytic::FWorldQuery Query;
		Query.Shape = Speed::Analytic::EQueryShape::Box;
		Query.Start = Query.End = FVector(0, 0, 20);
		Query.HalfExtent = FVector(60, 40, 20);
		Query.bAuthorityOnly = true;
		return Query;
	}

	// Independent signed vertex clearance, without provider contact/MTD telemetry.
	double MinimumFloorClearance(const FVector& Center, const FQuat& Rotation,
		const FVector& Extent, const double FloorZ)
	{
		double Minimum = DBL_MAX;
		for (int32 Corner = 0; Corner < 8; ++Corner)
		{
			const FVector Local((Corner & 1) ? Extent.X : -Extent.X,
				(Corner & 2) ? Extent.Y : -Extent.Y, (Corner & 4) ? Extent.Z : -Extent.Z);
			Minimum = FMath::Min(Minimum, (Center + Rotation.RotateVector(Local)).Z - FloorZ);
		}
		return Minimum;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedBoxPlanarCornerRoundingTest,
	"IAmSpeed.PhysicalLaws.BoxEquilibrium.PlanarCornerRounding",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedBoxPlanarCornerRoundingTest::RunTest(const FString& Parameters)
{
	// Reproduce a composed local/owner rotation, not an idealized box matrix.
	// User-approved release boundary: sub-roundoff disagreements are measured
	// advisory debt, not a release blocker. Finite overlap/separation must still
	// be detected; native trajectory tests separately reject amplified motion.
	const FVector Extent(78.79f, 52.30f, 19.14f);
	const FQuat Local = FRotator(-0.55, 0, 0).Quaternion();
	const FVector Faces[] = { FVector::UpVector, FVector::DownVector,
		FVector::ForwardVector, -FVector::ForwardVector, FVector::RightVector, -FVector::RightVector };
	int32 Cells = 0, ArithmeticDisagreements = 0;
	double MaximumArithmeticDisagreement = 0;
	for (double Floor : { 0.0, -0.0035156249068749703, 2000.123456789 })
	for (int32 Face = 0; Face < UE_ARRAY_COUNT(Faces); ++Face)
	for (double Yaw : { 0.0, 40.0, 80.0 })
	{
		const auto Data = RestingPlane(FVector(0, 0, Floor));
		Speed::FAnalyticStaticCollisionWorld World(Data);
		FQuat Owner = FQuat::FindBetweenNormals(Faces[Face], FVector::DownVector) * Local.Inverse();
		Owner.Normalize();
		Owner = FRotator(0, Yaw, 0).Quaternion() * Owner;
		Owner.Normalize();
		auto Query = RestingBox();
		Query.Rotation = Owner * Local;
		Query.HalfExtent = Extent;
		Query.DomainTolerance = Query.InitialOverlapTolerance = 0;
		const double Height = -MinimumFloorClearance(FVector::ZeroVector, Query.Rotation, Extent, 0);
		for (int32 Step : { -3, -2, -1, 0, 1, 2, 3 })
		{
			double Z = Floor + Height;
			if (FMath::Abs(Step) == 3) Z += Step < 0 ? -0.0001 : 0.0001;
			else for (int32 I = 0; I < FMath::Abs(Step); ++I)
				Z = std::nextafter(Z, Step < 0 ? -DBL_MAX : DBL_MAX);
			Query.Start = Query.End = FVector(0, 0, Z);
			const double Gap = MinimumFloorClearance(Query.Start, Query.Rotation, Extent, Floor);
			const auto Hit = World.SweepSingle(Query);
			const double Roundoff = 64 * DBL_EPSILON * FMath::Max(1.0, FMath::Abs(Z) + FMath::Abs(Floor) + Extent.GetMax());
			const double DepthError = FMath::Abs(Hit.PenetrationDepth - FMath::Max(0.0, -Gap));
			const bool bSignDisagrees = Hit.bStartPenetrating != (Gap < 0);
			if (!FMath::IsFinite(DepthError) || DepthError > Roundoff || (bSignDisagrees && FMath::Abs(Gap) > Roundoff))
				AddError(FString::Printf(TEXT("corner/provider disagreement floor=%.17g face=%d yaw=%.0f step=%d gap=%.17g depth=%.17g"),
					Floor, Face, Yaw, Step, Gap, Hit.PenetrationDepth));
			else if (bSignDisagrees || DepthError > 0)
			{
				++ArithmeticDisagreements;
				MaximumArithmeticDisagreement = FMath::Max(MaximumArithmeticDisagreement,
					FMath::Max(DepthError, bSignDisagrees ? FMath::Abs(Gap) : 0.0));
			}
			++Cells;
		}
	}
	AddInfo(FString::Printf(TEXT("%d corner/plane comparisons; arithmetic-only debt: count=%d max_cm=%.17g. No runtime tolerance changed."),
		Cells, ArithmeticDisagreements, MaximumArithmeticDisagreement));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedBoxRestingSupportTest,
	"IAmSpeed.PhysicalLaws.RestingEquilibrium.BoxSupportCertificate",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedBoxMicroRockingTest,
	"IAmSpeed.PhysicalLaws.BoxEquilibrium.MicroRockingGuards",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedBoxMicroRockingTest::RunTest(const FString& Parameters)
{
	const auto Data = RestingPlane();
	Speed::FAnalyticStaticCollisionWorld World(Data);
	const auto Query = RestingBox();
	SKinematic Incoming;
	Incoming.Location = Query.Start;
	Incoming.Acceleration = FVector(0, 0, -650);
	Incoming.Velocity = FVector(0, 0, -.1);
	Incoming.AngularVelocity = FVector(.001, 0, 0);
	// Uniform box, mass 10: independent principal inertias m*(b^2+c^2)/3.
	FMatrix InvI = FMatrix::Identity;
	InvI.M[0][0] = 3.0 / (10 * (40 * 40 + 20 * 20));
	InvI.M[1][1] = 3.0 / (10 * (60 * 60 + 20 * 20));
	InvI.M[2][2] = 3.0 / (10 * (60 * 60 + 40 * 40));
	auto Allowed = [&](const SKinematic& State, double E = 0, double Mu = .5)
	{
		return Speed::CanStabilizeBoxMicroRocking(World, Query, State, InvI, .1, E, Mu);
	};
	TestTrue(TEXT("inelastic, incoming sub-resolution rocking on a complete stable face"), Allowed(Incoming));
	TestFalse(TEXT("elastic impact cannot be captured"), Allowed(Incoming, 1));
	TestFalse(TEXT("frictionless impact cannot be captured"), Allowed(Incoming, 0, 0));
	SKinematic SlowGlide = Incoming; SlowGlide.Velocity.X = 1;
	TestFalse(TEXT("tiny rocking cannot stop a resolvable nearly-frictionless glide"), Allowed(SlowGlide, 0, .000001));
	SKinematic Changed = Incoming;
	Changed.Velocity.X = 1400;
	TestFalse(TEXT("resolvable glide remains unchanged"), Allowed(Changed));
	Changed = Incoming; Changed.AngularVelocity = FVector(.2, 0, 0);
	TestFalse(TEXT("resolvable rocking remains unchanged"), Allowed(Changed));
	Changed = Incoming; Changed.AngularVelocity = FVector::ZeroVector;
	TestFalse(TEXT("pure fall is not micro-rocking"), Allowed(Changed));
	Changed.Velocity = FVector(.01, 0, 0);
	TestFalse(TEXT("even a very slow pure slide keeps its Coulomb law"), Allowed(Changed));
	Changed = Incoming; Changed.AngularVelocity = FVector(0, 0, .001);
	TestFalse(TEXT("pure normal-axis spin keeps its friction law"), Allowed(Changed));
	Changed = Incoming; Changed.Velocity.Z = 1;
	TestFalse(TEXT("all separating corners preserve a waking impulse"), Allowed(Changed));
	Changed = Incoming; Changed.AngularAcceleration.X = .001;
	TestFalse(TEXT("ongoing torque prevents capture"), Allowed(Changed));
	Changed = Incoming; Changed.Acceleration.X = 1;
	TestFalse(TEXT("a tangential external load prevents capture"), Allowed(Changed));
	Changed = Incoming; Changed.Acceleration = FVector::ZeroVector;
	TestFalse(TEXT("no supporting load prevents capture"), Allowed(Changed));
	Changed = Incoming; Changed.Location.X = 61;
	TestFalse(TEXT("COM beyond the support face prevents capture"), Allowed(Changed));
	Changed = Incoming; Changed.Location.X = 60 - 1.e-9;
	TestFalse(TEXT("an almost-tipping COM cannot be captured above its escape energy"), Allowed(Changed));
	// Values bracket the actual energy bound, including rotational energy.
	const double AngularEnergy = .5 * .1 * FMath::Square(.001) / InvI.M[0][0];
	const double HalfStep = .5 / SKinematic::PositionQuantizationScale;
	const double Angle = HalfStep / (2 * FVector(60, 40, 20).Size());
	const double Limit = 650 * (40 * FMath::Sin(Angle) - 40 * FMath::Square(FMath::Sin(.5 * Angle)));
	Changed = Incoming; Changed.Velocity.Z = -FMath::Sqrt(2 * (.999 * Limit - AngularEnergy));
	TestTrue(TEXT("just below the positional energy bound"), Allowed(Changed));
	Changed.Velocity.Z = -FMath::Sqrt(2 * (1.001 * Limit - AngularEnergy));
	TestFalse(TEXT("just above the positional energy bound"), Allowed(Changed));
	for (double Gap : { -.001, .001, 100.0 })
	{
		auto Shifted = Query; Shifted.Start.Z += Gap; Shifted.End = Shifted.Start;
		Changed = Incoming; Changed.Location = Shifted.Start;
		TestFalse(TEXT("penetration, hovering and free flight are not captured"),
			Speed::CanStabilizeBoxMicroRocking(World, Shifted, Changed, InvI, .1, 0, .5));
	}
	auto FiniteData = RestingPlane();
	FiniteData.Planes[0].HalfExtents.X = 30;
	check(FiniteData.FinalizeAndValidate());
	Speed::FAnalyticStaticCollisionWorld PartialWorld(FiniteData);
	TestFalse(TEXT("missing corners at a support boundary prevent capture"),
		Speed::CanStabilizeBoxMicroRocking(PartialWorld, Query, Incoming, InvI, .1, 0, .5));
	return true;
}

bool FIAmSpeedBoxRestingSupportTest::RunTest(const FString& Parameters)
{
	const auto Data = RestingPlane();
	Speed::FAnalyticStaticCollisionWorld World(Data);
	const FVector Gravity(0, 0, -650);
	auto Query = RestingBox();
	Speed::FBoxRestingSupport Support;
	const FVector COM(13, -9, 25);
	if (!TestTrue(TEXT("finite face supports an off-center COM"),
		Speed::TryBuildBoxRestingSupport(World, Query, COM, Gravity, Support))) return false;
	double TotalWeight = 0.0;
	FVector TorquePerMass = FVector::ZeroVector;
	for (int32 Index = 0; Index < 4; ++Index)
	{
		TestTrue(TEXT("unilateral positive contact load"), Support.LoadFractions[Index] > 0.0);
		TestEqual(TEXT("actual contact lies on plane"), Support.Points[Index].Z, 0.0);
		TotalWeight += Support.LoadFractions[Index];
		TorquePerMass += FVector::CrossProduct(Support.Points[Index] - COM,
			Support.ResultantAcceleration * Support.LoadFractions[Index]);
	}
	TestTrue(TEXT("load weights sum to one within arithmetic roundoff"), FMath::Abs(TotalWeight - 1.0) < 1.e-14);
	TestTrue(TEXT("distributed normal forces have zero torque within arithmetic roundoff"), TorquePerMass.Size() < 1.e-10);
	TestTrue(TEXT("resultant exactly balances the external load"), (Support.ResultantAcceleration + Gravity).IsZero());
	Speed::FBoxRestingSupport Sliding;
	TestTrue(TEXT("Coulomb face slide has a feasible pressure center"),
		Speed::TryBuildBoxSlidingSupport(World, Query, COM, Gravity, FVector(1400, 0, 0), 0.5, Sliding));
	FVector FrictionTorque = FVector::ZeroVector;
	for (int32 I = 0; I < 4; ++I)
		FrictionTorque += FVector::CrossProduct(Sliding.Points[I] - COM,
			Sliding.ResultantAcceleration * Sliding.LoadFractions[I]);
	TestTrue(TEXT("shifted pressure center balances normal and friction torque"), FrictionTorque.Size() < 1.e-9);
	TestTrue(TEXT("sliding friction opposes motion with the configured magnitude"),
		Sliding.ResultantAcceleration == FVector(-325, 0, 650));
	TestEqual(TEXT("Coulomb stop time is a material event"), Sliding.StopAfterSeconds, 1400.0 / 325.0);
	TestTrue(TEXT("a finite interior segment retains the material reaction"),
		Speed::TryBuildBoxSlidingSupport(World, Query, COM, Gravity, FVector(1400, 0, 0), .5, Sliding, 1.0 / 300));
	TestFalse(TEXT("a long segment cannot carry support beyond the finite plane"),
		Speed::TryBuildBoxSlidingSupport(World, Query, COM, Gravity, FVector(1400, 0, 0), 0, Sliding, 1));
	TestTrue(TEXT("rejected temporal support leaves no reaction"), Sliding.ResultantAcceleration.IsZero());
	TestTrue(TEXT("a physical stop before the boundary remains supported for a longer horizon"),
		Speed::TryBuildBoxSlidingSupport(World, Query, COM, Gravity, FVector(1, 0, 0), .5, Sliding, 30));
	TestFalse(TEXT("friction whose pressure center lies outside the face must tip/release"),
		Speed::TryBuildBoxSlidingSupport(World, Query, COM, Gravity, FVector(1400, 0, 0), 3.0, Sliding));
	TestTrue(TEXT("rejected sliding support cannot reuse an earlier reaction"), Sliding.ResultantAcceleration.IsZero());

	const auto Reject = [&](const TCHAR* Name, const Speed::Analytic::FWorldQuery& Q,
		const FVector& Center, const FVector& Load)
	{
		const bool bAccepted = Speed::TryBuildBoxRestingSupport(World, Q, Center, Load, Support);
		TestFalse(Name, bAccepted);
		TestTrue(TEXT("failure clears all prior reaction state"), Support.ResultantAcceleration.IsZero());
	};
	Reject(TEXT("tipping COM outside face"), Query, FVector(61, 0, 20), Gravity);
	Reject(TEXT("marginal COM on face edge"), Query, FVector(60, 0, 20), Gravity);
	Reject(TEXT("separating load"), Query, COM, -Gravity);
	Reject(TEXT("unbalanced tangential force"), Query, COM, FVector(1, 0, -650));
	auto Changed = Query;
	Changed.Start.Z += 0.000001; Changed.End = Changed.Start;
	Reject(TEXT("a tiny real gap must not become support"), Changed, Changed.Start, Gravity);
	Changed = Query; Changed.Start.Z -= 0.000001; Changed.End = Changed.Start;
	Reject(TEXT("a tiny penetration must not become equilibrium"), Changed, Changed.Start, Gravity);
	Changed = Query; Changed.Start.X = 980; Changed.End = Changed.Start;
	Reject(TEXT("partial finite footprint is not certified by this full-face contract"), Changed, Changed.Start, Gravity);
	Changed = Query; Changed.Rotation = FRotator(0.001, 0, 0).Quaternion();
	Reject(TEXT("tilted box has no full supporting face"), Changed, Changed.Start, Gravity);
	Changed = Query; Changed.bApplyCollisionFilter = true; Changed.BlockingObjectTypes = 0;
	Reject(TEXT("collision filtering still owns support"), Changed, Changed.Start, Gravity);
	Speed::Analytic::FAnalyticWorldData Empty;
	TestTrue(TEXT("empty world validates"), Empty.FinalizeAndValidate());
	Speed::FAnalyticStaticCollisionWorld NoSupport(Empty);
	TestFalse(TEXT("world without support rejects zero-speed body"),
		Speed::TryBuildBoxRestingSupport(NoSupport, Query, COM, Gravity, Support));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedFinitePlanarSupportSegmentTest,
	"IAmSpeed.PhysicalLaws.BoxEquilibrium.FiniteSupportSegment",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedFinitePlanarSupportSegmentTest::RunTest(const FString& Parameters)
{
	auto Data = RestingPlane();
	auto& Plane = Data.Planes[0];
	// Concave U: both arms support the endpoints, but the notch does not.
	Plane.HalfExtents = FVector2d(100, 100);
	Plane.DomainVertices = { FVector2d(-100,-100), FVector2d(100,-100), FVector2d(100,100),
		FVector2d(20,100), FVector2d(20,-20), FVector2d(-20,-20), FVector2d(-20,100), FVector2d(-100,100) };
	if (!TestTrue(TEXT("concave finite plane validates"), Data.FinalizeAndValidate())) return false;
	Speed::FAnalyticStaticCollisionWorld World(Data);
	const FVector Left(-50, 50, 0), Right(50, 50, 0);
	TestTrue(TEXT("both notch endpoints individually lie on the provider"),
		Plane.ContainsProjectedPoint(Left) && Plane.ContainsProjectedPoint(Right));
	TestFalse(TEXT("support cannot bridge a notch between valid endpoints"), Plane.ContainsProjectedSegment(Left, Right));
	TestFalse(TEXT("notch rejection is symmetric"), Plane.ContainsProjectedSegment(Right, Left));
	TestTrue(TEXT("the interior connector is continuously supported"),
		Plane.ContainsProjectedSegment(FVector(-50, -50, 0), FVector(50, -50, 0)));
	auto Query = RestingBox();
	const FVector Points[] = { Left };
	TestFalse(TEXT("world-level support certificate checks the entire trajectory"),
		World.IsPlanarSupportTranslationCertified(Query, MakeArrayView(Points), FVector::UpVector, Right - Left));
	TestTrue(TEXT("an arm-parallel path remains supported"),
		World.IsPlanarSupportTranslationCertified(Query, MakeArrayView(Points), FVector::UpVector, FVector(0,-100,0)));
	Query.bApplyCollisionFilter = true; Query.BlockingObjectTypes = 0;
	TestFalse(TEXT("temporal support obeys collision filtering"),
		World.IsPlanarSupportTranslationCertified(Query, MakeArrayView(Points), FVector::UpVector, FVector(0,-100,0)));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedBoxRestingEquilibriumTest,
	"IAmSpeed.PhysicalLaws.RestingEquilibrium.BoxUnquantizedIntegration",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedBoxRestingEquilibriumTest::RunTest(const FString& Parameters)
{
	constexpr float Delta = 1.0f / 300.0f;
	constexpr int32 Frames = 9000;
	const FVector Extent(60, 40, 20), Gravity(0, 0, -650);
	const FRotator Faces[] = { FRotator::ZeroRotator, FRotator(0, 0, 180),
		FRotator(90, 0, 0), FRotator(-90, 0, 0), FRotator(0, 0, 90), FRotator(0, 0, -90) };
	int32 Cells = 0;
	for (const FVector Offset : { FVector::ZeroVector, FVector(3500, -4500, 2000) })
	for (int32 Face = 0; Face < UE_ARRAY_COUNT(Faces); ++Face)
	for (const double Yaw : { 0.0, 40.0, 80.0 })
	{
		const auto Data = RestingPlane(Offset);
		Speed::FAnalyticStaticCollisionWorld World(Data);
		TStrongObjectPtr<USpeedMovementComponent> Owner(NewObject<USpeedMovementComponent>());
		TStrongObjectPtr<UBoxSubBody> Box(NewObject<UBoxSubBody>());
		Owner->SubBodies = { Box.Get() };
		Owner->SolidSubBodies = { Box.Get() };
		Owner->CenterOfMass = FVector(7, -3, 2);
		Box->Initialize(Owner.Get());
		Box->SetBoxExtent(Extent);
		Box->SetMass(Owner->GetPhysMass());
		Owner->SetIsFrozen(false);
		SKinematic Initial;
		Initial.Rotation = FRotator(0, Yaw, 0).Quaternion() * Faces[Face].Quaternion();
		Initial.Rotation.Normalize();
		FVector BoxCenter = Offset;
		BoxCenter.Z -= MinimumFloorClearance(BoxCenter, Initial.Rotation, Extent, Offset.Z);
		Initial.Location = BoxCenter + Initial.Rotation.RotateVector(Owner->CenterOfMass);
		Initial.Acceleration = Gravity;
		Owner->SetKinematicState(Initial);
		Owner->UpdateSubBodiesKinematics();
		// Recompute the initial origin through the same COM boundary used in play.
		// If subtraction rounded downward, move only this fixture's initial COM
		// upward by its measured negative clearance, before taking the baseline.
		const double InitialGap = MinimumFloorClearance(Owner->GetPhysLocation(), Initial.Rotation, Extent, Offset.Z);
		if (InitialGap < 0.0)
		{
			Initial.Location.Z -= InitialGap;
			Owner->SetKinematicState(Initial);
			Owner->UpdateSubBodiesKinematics();
		}
		// The independent vertex transform and the provider's projected-radius
		// formula can differ by a few ULPs. Establish a legal INITIAL pose for
		// both, never depenetrate or change the pose during the measured interval.
		auto InitialQuery = RestingBox();
		InitialQuery.Rotation = Initial.Rotation;
		InitialQuery.InitialOverlapTolerance = 0.0;
		InitialQuery.DomainTolerance = 0.0;
		for (int32 Ulp = 0; Ulp < 8; ++Ulp)
		{
			InitialQuery.Start = InitialQuery.End = Owner->GetPhysLocation();
			const auto Hit = World.SweepSingle(InitialQuery);
			if (!Hit.bStartPenetrating && Hit.PenetrationDepth == 0.0 &&
				MinimumFloorClearance(Owner->GetPhysLocation(), Initial.Rotation, Extent, Offset.Z) >= 0.0) break;
			Initial.Location.Z = std::nextafter(double(Initial.Location.Z), DBL_MAX);
			Owner->SetKinematicState(Initial);
			Owner->UpdateSubBodiesKinematics();
		}
		Owner->SetStaticCollisionWorldForFrame(&World);
		if (Owner->GetStaticRestingReaction() != -Gravity)
		{
			Speed::FBoxRestingSupport Diagnosis;
			InitialQuery.Start = InitialQuery.End = Owner->GetPhysLocation();
			Speed::TryBuildBoxRestingSupport(World, InitialQuery, Initial.Location, Gravity, Diagnosis);
			AddError(FString::Printf(TEXT("initial face=%d yaw=%.0f offset=%s support_result=%d witness=%d gap=%.17g clearance=%.17g"),
				Face, Yaw, *Offset.ToString(), int32(Diagnosis.Result), Diagnosis.WitnessIndex,
				Diagnosis.WitnessGapCm, MinimumFloorClearance(Owner->GetPhysLocation(), Initial.Rotation, Extent, Offset.Z)));
			return false;
		}
		for (int32 Frame = 0; Frame < Frames; ++Frame)
		{
			Owner->ResetForFrame(Delta);
			// No quantization/PostPhysicsUpdate, no sleep support, no pose/velocity
			// reimposition. This is the production ISpeedComponent integrator.
			Owner->IntegrateKinematics(Delta);
			const SKinematic& State = Owner->GetKinematicState();
			const double Clearance = MinimumFloorClearance(Owner->GetPhysLocation(), State.Rotation, Extent, Offset.Z);
			const bool bExact = State.Location == Initial.Location && State.Rotation == Initial.Rotation &&
				State.Velocity.IsZero() && State.AngularVelocity.IsZero() && Clearance >= 0.0 &&
				State.Acceleration == Gravity && !Owner->IsPhysicsSleeping();
			if (!bExact)
			{
				AddError(FString::Printf(TEXT("face=%d yaw=%.0f frame=%d clearance=%.17g drift=%.17g speed=%.17g spin=%.17g"),
					Face, Yaw, Frame, Clearance, FVector(State.Location - Initial.Location).Size(),
					State.Velocity.Size(), State.AngularVelocity.Size()));
				return false;
			}
		}
		++Cells;

		// A collision can occur between two substeps of the very same frame.
		Owner->AddPhysVelocity(FVector(0, 0, 100));
		TestTrue(TEXT("new impulse immediately releases reaction"), Owner->GetStaticRestingReaction().IsZero());
		Owner->IntegrateKinematics(Delta);
		TestTrue(TEXT("gravity resumes during that same frame, without a cached cancellation"),
			Owner->GetPhysCOMVelocity().Z < 100.0 && Owner->GetPhysCOMVelocity().Z > 0.0);
		Owner->SetKinematicState(Initial);
		Owner->SetPhysAngularVelocity(FVector(0, 0, 2));
		TestTrue(TEXT("even normal-axis spin is not certified rest"), Owner->GetStaticRestingReaction().IsZero());
		Owner->SetKinematicState(Initial);
		Owner->SetPhysAngularAcceleration(FVector(1, 0, 0));
		TestTrue(TEXT("torque releases before angular motion begins"), Owner->GetStaticRestingReaction().IsZero());
		Owner->SetKinematicState(Initial);
		Owner->SetPhysAcceleration(Gravity * 2);
		TestTrue(TEXT("changed normal load is recalculated, not cached"), Owner->GetStaticRestingReaction() == -Gravity * 2);
		Owner->SetStaticCollisionWorldForFrame(nullptr);
		TestTrue(TEXT("support unbinding releases reaction"), Owner->GetStaticRestingReaction().IsZero());
	}
	AddInfo(FString::Printf(TEXT("Box equilibrium: %d cells x %d unquantized frames at 300 Hz; six faces, three yaws, two origins."), Cells, Frames));
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedBoxCanonicalRestTest,
	"IAmSpeed.PhysicalLaws.BoxStandingStill.CanonicalQuantization",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedBoxCanonicalRestTest::RunTest(const FString& Parameters)
{
	constexpr float Delta = 1.0f / 300.0f;
	const FVector Extent(60, 40, 20), Gravity(0, 0, -650);
	const FVector Faces[] = { FVector::UpVector, FVector::DownVector,
		FVector::ForwardVector, -FVector::ForwardVector, FVector::RightVector, -FVector::RightVector };
	int32 Cells = 0;
	for (const FVector Offset : { FVector(0, 0, -0.0035156249068749703), FVector(3500, -4500, 2000.123456789) })
	for (const FVector& Face : Faces)
	for (const double Yaw : { 0.0, 40.0, 80.0 })
	{
		const auto Data = RestingPlane(Offset);
		Speed::FAnalyticStaticCollisionWorld World(Data);
		TStrongObjectPtr<USpeedMovementComponent> Owner(NewObject<USpeedMovementComponent>());
		TStrongObjectPtr<UBoxSubBody> Box(NewObject<UBoxSubBody>());
		Owner->SubBodies = { Box.Get() };
		Owner->SolidSubBodies = { Box.Get() };
		Owner->CenterOfMass = FVector(7, -3, 2);
		Owner->GravityZ = -650;
		Box->Initialize(Owner.Get());
		Box->SetBoxExtent(Extent);
		Box->SetRelativeRotation(FRotator(-0.55, 0, 0));
		Box->SetMass(Owner->GetPhysMass());
		Owner->SetIsFrozen(false);
		SKinematic Initial;
		Initial.Rotation = FRotator(0, Yaw, 0).Quaternion() *
			FQuat::FindBetweenNormals(Face, FVector::DownVector) * Box->GetRelativeRotation().Quaternion().Inverse();
		Initial.Rotation.Normalize();
		const FQuat BoxRotation = Initial.Rotation * Box->GetRelativeRotation().Quaternion();
		FVector BoxCenter = Offset;
		BoxCenter.Z -= MinimumFloorClearance(BoxCenter, BoxRotation, Extent, Offset.Z);
		Initial.Location = BoxCenter + Initial.Rotation.RotateVector(Owner->CenterOfMass);
		Initial.Acceleration = Gravity;
		Owner->SetKinematicState(Initial);
		Owner->UpdateSubBodiesKinematics();
		Owner->SetStaticCollisionWorldForFrame(&World);
		Speed::FBoxRestingSupport Support;
		for (int32 Ulp = 0; Ulp < 8; ++Ulp)
		{
			if (Box->EvaluateStaticRestingSupport(World, Gravity, Support) &&
				MinimumFloorClearance(Owner->GetPhysLocation(), BoxRotation, Extent, Offset.Z) >= 0) break;
			Initial.Location.Z = std::nextafter(double(Initial.Location.Z), DBL_MAX);
			Owner->SetKinematicState(Initial);
		}
		if (!TestTrue(TEXT("initial exact face support on non-grid plane"), Owner->HasExactStaticRestingSupport())) return false;
		// Exercise the actual canonical quantizer BEFORE integration too: a host
		// reset can occur in another component's post-update registration slot.
		for (int32 Frame = 0; Frame < 9000; ++Frame)
		{
			Owner->QuantizePhysicalState();
			Owner->ResetForFrame(Delta);
			Owner->IntegrateKinematics(Delta);
			const auto& State = Owner->GetKinematicState();
			if (State.Location != Initial.Location || State.Rotation != Initial.Rotation ||
				!State.Velocity.IsZero() || !State.AngularVelocity.IsZero() ||
				MinimumFloorClearance(Owner->GetPhysLocation(), BoxRotation, Extent, Offset.Z) < 0)
			{
				AddError(FString::Printf(TEXT("canonical rest cell=%d frame=%d drift=%.17g"),
					Cells, Frame, FVector(State.Location - Initial.Location).Size()));
				return false;
			}
		}
		// Unsupported/moving poses retain the existing quantization contract.
		Owner->SetStaticCollisionWorldForFrame(nullptr);
		SKinematic Expected = Initial;
		const FQuat PreviousRotation = Owner->GetKinematicStateForFrame(Owner->NumFrame() - 1).Rotation;
		Expected.Quantize(PreviousRotation);
		Owner->QuantizePhysicalState();
		TestTrue(TEXT("unbound world retains ordinary pose quantization"),
			Owner->GetPhysCOMLocation() == Expected.Location && Owner->GetPhysRotation() == Expected.Rotation);
		Owner->SetKinematicState(Initial);
		Owner->SetStaticCollisionWorldForFrame(&World);
		Owner->SetPhysCOMVelocity(FVector(0, 0, 1));
		TestFalse(TEXT("separating motion cannot receive pose preservation"), Owner->HasExactStaticRestingSupport());
		++Cells;
	}
	AddInfo(FString::Printf(TEXT("Canonical quantized rest: %d cells x 9000 frames, local pitch and non-grid support."), Cells));
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedBoxApproachQuantizationTest,
	"IAmSpeed.PhysicalLaws.BoxEquilibrium.NonGridApproachQuantization",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedBoxApproachQuantizationTest::RunTest(const FString& Parameters)
{
	const double FloorZ = .0035156249068749703;
	const auto Data = RestingPlane(FVector(0, 0, FloorZ));
	Speed::FAnalyticStaticCollisionWorld World(Data);
	TStrongObjectPtr<USpeedMovementComponent> Owner(NewObject<USpeedMovementComponent>());
	TStrongObjectPtr<UBoxSubBody> Box(NewObject<UBoxSubBody>());
	Owner->SubBodies = { Box.Get() }; Owner->SolidSubBodies = { Box.Get() };
	Owner->CenterOfMass = FVector::ZeroVector;
	Box->Initialize(Owner.Get()); Box->SetBoxExtent(FVector(60, 40, 20));
	Owner->SetIsFrozen(false);
	SKinematic Initial;
	Initial.Location = FVector(0, 0, FloorZ + 20 + .001);
	Initial.Velocity = FVector(0, 0, -100);
	Owner->SetKinematicState(Initial); Owner->UpdateSubBodiesKinematics();
	Owner->SetStaticCollisionWorldForFrame(&World, 1.0f / 300);
	SKinematic Rounded = Initial;
	Rounded.Quantize(FQuat::Identity);
	TestTrue(TEXT("fixture starts exterior but independent canonical rounding would penetrate"),
		Initial.Location.Z - 20 - FloorZ > 0 && Rounded.Location.Z - 20 - FloorZ < 0);
	TestFalse(TEXT("approaching airborne body has no established face support"), Owner->HasExactStaticFaceSupport());
	Owner->QuantizePhysicalState();
	TestTrue(TEXT("quantization cannot introduce a contact that CCD has not reached"),
		Owner->GetPhysCOMLocation().Z - 20 - FloorZ >= 0);
	TestTrue(TEXT("a rejected quantized pose retains the actual integrated exterior pose"),
		Owner->GetPhysCOMLocation() == Initial.Location && Owner->GetPhysRotation() == Initial.Rotation);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedBoxObjectOnlyQuantizationTest,
	"IAmSpeed.PhysicalLaws.BoxEquilibrium.ObjectOnlyApproachQuantization",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedBoxObjectOnlyQuantizationTest::RunTest(const FString& Parameters)
{
	// Static pose feasibility includes object-query surfaces even when their
	// trace-channel mask is empty. Quantization must use the same geometry as
	// post-integration overlap projection, without enabling new CCD responses.
	constexpr double FloorZ = .0035156249068749703;
	auto Data = RestingPlane(FVector(0, 0, FloorZ));
	Data.Planes[0].BlockingChannels = 0;
	if (!TestTrue(TEXT("object-only finite floor validates"), Data.FinalizeAndValidate())) return false;
	Speed::FAnalyticStaticCollisionWorld World(Data);
	TStrongObjectPtr<USpeedMovementComponent> Owner(NewObject<USpeedMovementComponent>());
	TStrongObjectPtr<UBoxSubBody> Box(NewObject<UBoxSubBody>());
	Owner->SubBodies = { Box.Get() }; Owner->SolidSubBodies = { Box.Get() };
	Owner->CenterOfMass = FVector::ZeroVector;
	Box->Initialize(Owner.Get()); Box->SetBoxExtent(FVector(60, 40, 20));
	Owner->SetIsFrozen(false);
	SKinematic Initial;
	Initial.Location = FVector(0, 0, FloorZ + 20 + .001);
	Initial.Velocity = FVector(0, 0, -100);
	Owner->SetKinematicState(Initial); Owner->UpdateSubBodiesKinematics();
	Owner->SetStaticCollisionWorldForFrame(&World, 1.0f / 300);
	SKinematic Rounded = Initial;
	Rounded.Quantize(FQuat::Identity);
	auto Query = RestingBox();
	Query.Start = Query.End = Rounded.Location;
	Query.bApplyCollisionFilter = true;
	Query.InitialOverlapTolerance = 0;
	TestFalse(TEXT("trace response remains disabled"), World.SweepSingle(Query).bHit);
	Query.bObjectQuery = true;
	Query.ObjectTypes = uint64(1) << ECC_WorldStatic;
	Speed::Analytic::FWorldHit Hit;
	TestTrue(TEXT("stationary object query remains supported"), World.TryFindDeepestOverlap(Query, Hit));
	TestTrue(TEXT("rounded pose overlaps the object-only floor"), Hit.bStartPenetrating && Hit.PenetrationDepth > 0);
	TestTrue(TEXT("integrated pose is independently outside"), Initial.Location.Z - 20 - FloorZ > 0);
	Owner->QuantizePhysicalState();
	TestTrue(TEXT("object-only floor cannot be crossed by rounding"),
		Owner->GetPhysCOMLocation().Z - 20 - FloorZ >= 0);
	TestTrue(TEXT("rejecting rounding preserves the integrated pose"),
		Owner->GetPhysCOMLocation() == Initial.Location && Owner->GetPhysRotation() == Initial.Rotation);
	// Rejecting an invalid pose must not disable ordinary quantization in clear
	// space, outside this finite patch, or against query-disabled/dynamic data.
	SKinematic Clear = Rounded;
	Clear.Location.Z += 1;
	TestTrue(TEXT("exterior pose remains admissible"), Box->CanApplyQuantizedPose(World, Clear));
	Clear = Rounded; Clear.Location.X = 2000;
	TestTrue(TEXT("finite floor cannot become an infinite plane"), Box->CanApplyQuantizedPose(World, Clear));
	auto DisabledData = Data;
	DisabledData.Planes[0].bQueryCollisionEnabled = false;
	DisabledData.Planes[0].bAuthorityEligible = false;
	TestTrue(TEXT("disabled fixture validates"), DisabledData.FinalizeAndValidate());
	Speed::FAnalyticStaticCollisionWorld DisabledWorld(DisabledData);
	TestTrue(TEXT("disabled collision remains excluded"), Box->CanApplyQuantizedPose(DisabledWorld, Rounded));
	auto DynamicData = Data;
	DynamicData.Planes[0].ObjectType = ECC_WorldDynamic;
	TestTrue(TEXT("dynamic fixture validates"), DynamicData.FinalizeAndValidate());
	Speed::FAnalyticStaticCollisionWorld DynamicWorld(DynamicData);
	TestTrue(TEXT("static feasibility does not include dynamic object types"), Box->CanApplyQuantizedPose(DynamicWorld, Rounded));
	return true;
}
#endif
