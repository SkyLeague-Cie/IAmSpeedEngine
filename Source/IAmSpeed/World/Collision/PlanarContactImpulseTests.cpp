#if WITH_DEV_AUTOMATION_TESTS
#include "PlanarContactImpulse.h"
#include "Misc/AutomationTest.h"
#include "Math/QuatRotationTranslationMatrix.h"
#include <algorithm>

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedPlanarNormalImpulseTest,
	"IAmSpeed.PhysicalLaws.NormalResponse.SimultaneousPlanarContacts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedPlanarNormalImpulseTest::RunTest(const FString& Parameters)
{
	const FVector COM(0, 0, 20), N(0, 0, 1);
	const FVector Points[] = { FVector(-60, -40, 0), FVector(60, -40, 0),
		FVector(-60, 40, 0), FVector(60, 40, 0) };
	FMatrix InvI = FMatrix::Identity;
	InvI.M[0][0] = 3.0 / (10 * (40.0 * 40 + 20.0 * 20));
	InvI.M[1][1] = 3.0 / (10 * (60.0 * 60 + 20.0 * 20));
	InvI.M[2][2] = 3.0 / (10 * (60.0 * 60 + 40.0 * 40));
	for (int32 Count : { 1, 2, 3, 4 })
	for (FVector W : { FVector::ZeroVector, FVector(2, 4, 0), FVector(-2, -4, 3) })
	for (double VN : { -323.0, -2.0, 0.0, 25.0 })
	{
		const FVector V(10, 20, VN);
		const auto Response = Speed::SolvePlanarNormalImpulse(MakeArrayView(Points, Count), N, COM, V, W, 0.1, InvI, 0);
		TestTrue(TEXT("bounded active-set solve finds a unilateral response"), Response.bSolved);
		if (!Response.bSolved) continue;
		const FVector AfterV = V + Response.DeltaVelocity, AfterW = W + Response.DeltaAngularVelocity;
		for (int32 I = 0; I < Count; ++I)
		{
			const double Closing = FVector::DotProduct(AfterV + FVector::CrossProduct(AfterW, Points[I] - COM), N);
			TestTrue(TEXT("no closing point speed beyond solve roundoff"), Closing >= -1.e-10);
			TestTrue(TEXT("normal impulse never attracts the body"), Response.Impulses[I] >= 0);
		}
		const auto Energy = [&](const FVector& Linear, const FVector& Angular)
		{
			return 5 * Linear.SizeSquared() + 0.5 *
				(Angular.X * Angular.X / InvI.M[0][0] + Angular.Y * Angular.Y / InvI.M[1][1] + Angular.Z * Angular.Z / InvI.M[2][2]);
		};
		TestTrue(TEXT("plastic response cannot add kinetic energy"), Energy(AfterV, AfterW) <= Energy(V, W) + 1.e-8);
		TestTrue(TEXT("normal-only response leaves tangential COM velocity and normal spin unchanged"),
			AfterV.X == V.X && AfterV.Y == V.Y && AfterW.Z == W.Z);
		FVector Reverse[4];
		for (int32 I = 0; I < Count; ++I) Reverse[I] = Points[Count - I - 1];
		const auto Reversed = Speed::SolvePlanarNormalImpulse(MakeArrayView(Reverse, Count), N, COM, V, W, 0.1, InvI, 0);
		TestTrue(TEXT("contact permutation preserves the physical resultant within arithmetic roundoff"),
			Reversed.bSolved && Reversed.DeltaVelocity.Equals(Response.DeltaVelocity, 1.e-10) &&
			Reversed.DeltaAngularVelocity.Equals(Response.DeltaAngularVelocity, 1.e-10));
	}
	const auto Flat = Speed::SolvePlanarNormalImpulse(MakeArrayView(Points), N, COM, FVector(0, 0, -323), FVector::ZeroVector, 0.1, InvI, 0);
	TestTrue(TEXT("simultaneous flat impact removes the full incoming normal speed without spin"),
		Flat.bSolved && Flat.DeltaVelocity.Equals(FVector(0, 0, 323), 1.e-10) && Flat.DeltaAngularVelocity.IsNearlyZero(1.e-10));
	const auto Elastic = Speed::SolvePlanarNormalImpulse(MakeArrayView(Points), N, COM, FVector(0, 0, -323), FVector::ZeroVector, 0.1, InvI, 1);
	TestTrue(TEXT("configured elastic flat impact rebounds instead of being frozen"), Elastic.bSolved && Elastic.DeltaVelocity.Equals(FVector(0, 0, 646), 1.e-10));
	TestFalse(TEXT("invalid normal is rejected"), Speed::SolvePlanarNormalImpulse(MakeArrayView(Points), FVector::ZeroVector, COM, FVector::ZeroVector, FVector::ZeroVector, 0.1, InvI, 0).bSolved);
	for (int32 Count : { 2, 4 })
	for (double Speed : { 0.0, 20.0, 2300.0 })
	{
		const FVector InitialV(0, Speed, -100);
		const auto Coupled = Speed::SolvePlanarCoulombImpulse(MakeArrayView(Points, Count), N, COM,
			InitialV, FVector::ZeroVector, .1, InvI, 0, .5);
		TestTrue(TEXT("coupled Coulomb solver converges for symmetric edge/face impacts"), Coupled.bSolved);
		if (!Coupled.bSolved) continue;
		TestTrue(TEXT("symmetric impact cannot manufacture lateral speed or yaw spin"),
			FMath::Abs(Coupled.DeltaVelocity.X) <= 1.e-9 && FMath::Abs(Coupled.DeltaAngularVelocity.Z) <= 1.e-9);
		for (int32 I = 0; I < Count; ++I)
		{
			const FVector Vp = InitialV + Coupled.DeltaVelocity + FVector::CrossProduct(Coupled.DeltaAngularVelocity, Points[I] - COM);
			TestTrue(TEXT("friction leaves every contact nonclosing"), FVector::DotProduct(Vp, N) >= -1.e-9);
		}
	}
	const double Bias[] = { -80, 80 };
	const auto EdgeArrival = Speed::SolvePlanarCoulombImpulse(MakeArrayView(Points), N, COM,
		FVector(0, 76.104, -175.436), FVector(-4.386, 0, 0), .1, InvI, 0, .5);
	TestTrue(TEXT("tilting edge-to-face Coulomb impact converges without a legacy fallback"), EdgeArrival.bSolved);
	const auto Force = Speed::SolvePlanarNormalImpulse(MakeArrayView(Points, 2), N, COM,
		FVector(0, 0, -650), FVector::ZeroVector, .1, InvI, 0, MakeArrayView(Bias));
	TestTrue(TEXT("normal acceleration response accepts rotational point bias"), Force.bSolved);
	for (int32 I = 0; I < 2; ++I)
		TestTrue(TEXT("continuous reaction balances the complete point-normal acceleration"),
			(-650 + Force.DeltaVelocity.Z + FVector::CrossProduct(Force.DeltaAngularVelocity, Points[I] - COM).Z + Bias[I]) >= -1.e-9);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedPlanarStickingCertificateTest,
	"IAmSpeed.PhysicalLaws.NormalResponse.PlanarStickingCertificate",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedPlanarStickingCertificateTest::RunTest(const FString& Parameters)
{
	const FVector N(0, 0, 1), COM(7, -3, 20);
	const FVector Corners[] = { FVector(-60, -40, 0), FVector(60, -40, 0),
		FVector(-60, 40, 0), FVector(60, 40, 0) };
	FMatrix InvI = FMatrix::Identity;
	InvI.M[0][0] = 3.0 / (10 * (40.0 * 40 + 20.0 * 20));
	InvI.M[1][1] = 3.0 / (10 * (60.0 * 60 + 20.0 * 20));
	InvI.M[2][2] = 3.0 / (10 * (60.0 * 60 + 40.0 * 40));
	// Construct incoming momenta from an independently feasible face impulse,
	// then recover it for all 24 point permutations. No velocity threshold.
	for (const FVector COP : { FVector(0, 0, 0), FVector(40, 25, 0), FVector(-40, -25, 0) })
	for (const FVector J : { FVector(0, 0, 1000), FVector(300, 0, 1000), FVector(-200, 300, 1000) })
	{
		const FVector IncomingV = -0.1 * J;
		const FVector IncomingW = -FVector(InvI.TransformVector(FVector::CrossProduct(COP - COM, J)));
		int32 Order[] = { 0, 1, 2, 3 };
		do
		{
			FVector Points[4];
			for (int32 I = 0; I < 4; ++I) Points[I] = Corners[Order[I]];
			const auto R = Speed::SolvePlanarCoulombImpulse(MakeArrayView(Points), N, COM,
				IncomingV, IncomingW, .1, InvI, 0, .5);
			if (!TestTrue(TEXT("feasible full-face impulse is certified independently of contact ordering"),
				R.bSolved && R.IterationCount == 0)) continue;
			TestTrue(TEXT("feasible plastic sticking removes linear and angular momentum exactly"),
				(R.DeltaVelocity + IncomingV).IsZero() && (R.DeltaAngularVelocity + IncomingW).IsZero());
			FVector TotalJ = FVector::ZeroVector, TotalTorque = FVector::ZeroVector;
			for (int32 I = 0; I < 4; ++I)
			{
				const FVector PointJ = J * (R.Impulses[I] / J.Z);
				TestTrue(TEXT("each witness carries nonnegative normal impulse within its Coulomb cone"),
					R.Impulses[I] >= 0 && (PointJ - PointJ.Z * N).Size() <= .5 * PointJ.Z);
				TotalJ += PointJ;
				TotalTorque += FVector::CrossProduct(Points[I] - COM, PointJ);
			}
			TestTrue(TEXT("witness impulses independently reconstruct the linear response"),
				(TotalJ * .1).Equals(R.DeltaVelocity, 1.e-11));
			TestTrue(TEXT("witness moments independently reconstruct the angular response"),
				FVector(InvI.TransformVector(TotalTorque)).Equals(R.DeltaAngularVelocity, 1.e-11));
		} while (std::next_permutation(Order, Order + 4));
	}
	const auto RequireNotFrozen = [&](const FVector V, const FVector W, double Mu, double E)
	{
		const auto R = Speed::SolvePlanarCoulombImpulse(MakeArrayView(Corners), N, COM, V, W, .1, InvI, E, Mu);
		TestFalse(TEXT("an infeasible or elastic impulse is not certified as complete sticking"),
			R.bSolved && (V + R.DeltaVelocity).IsZero() && (W + R.DeltaAngularVelocity).IsZero());
	};
	RequireNotFrozen(FVector(1000, 0, -100), FVector::ZeroVector, .01, 0);
	RequireNotFrozen(FVector(0, 0, -100), FVector(1000, 0, 0), .5, 0);
	RequireNotFrozen(FVector(0, 0, 100), FVector::ZeroVector, .5, 0);
	RequireNotFrozen(FVector(0, 0, -100), FVector::ZeroVector, .5, 1);
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedPlanarEdgeStickingTest,
	"IAmSpeed.PhysicalLaws.NormalResponse.EdgeStickingCertificate",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedPlanarEdgeStickingTest::RunTest(const FString& Parameters)
{
	// Exact immutable input from a rejected two-point impact. This reproduces
	// the numerical kernel alone, without vehicle helpers, a world or a timer.
	const FVector COM(-2988.7483695817564, 1500.0000000006592, 33.755973446014771);
	const FVector Points[] = {
		FVector(-2920.6317547566791, 1447.7000007636157, -0.0035156249068749703),
		FVector(-2920.6317547567055, 1552.2999992377368, -0.0035156249068749703) };
	const FVector V(-0.29895736109774762, 9.9148138543778625e-09, -5.1767644973500397);
	const FVector W(-8.3484147248856257e-15, 0.033333513885736465, 6.0696394717633371e-21);
	FMatrix InvI = FMatrix::Identity;
	const double Entries[3][3] = {
		{ 8.0672797789060802e-07, 8.9807285961064994e-15, 4.0689477100039102e-08 },
		{ 8.9807285961064963e-15, 3.9701486134748614e-07, 2.7385460655438141e-15 },
		{ 4.0689477100039102e-08, 2.7385460655438157e-15, 3.2516702901535868e-07 } };
	for (int32 I = 0; I < 3; ++I)
	for (int32 J = 0; J < 3; ++J) InvI.M[I][J] = Entries[I][J];
	const auto R = Speed::SolvePlanarCoulombImpulse(MakeArrayView(Points), FVector::UpVector,
		COM, V, W, .001, InvI, 0, 0.30000001192092896);
	if (!TestTrue(TEXT("feasible off-center two-point impact resolves without legacy fallback"), R.bSolved)) return false;
	const FVector AfterV = V + R.DeltaVelocity, AfterW = W + R.DeltaAngularVelocity;
	for (const FVector& P : Points)
		TestTrue(TEXT("both edge points stick without freezing the remaining edge rotation"),
			(AfterV + FVector::CrossProduct(AfterW, P - COM)).Size() < 1.e-10);
	TestTrue(TEXT("edge-axis angular momentum remains free, not locked to zero"), AfterW.Size() > .01);
	const FMatrix Inertia = InvI.Inverse();
	const auto Energy = [&](FVector Linear, FVector Angular)
	{
		return 500 * Linear.SizeSquared() + .5 * FVector::DotProduct(Angular, Inertia.TransformVector(Angular));
	};
	TestTrue(TEXT("plastic edge impact dissipates kinetic energy"), Energy(AfterV, AfterW) <= Energy(V, W));
	FVector TotalJ = FVector::ZeroVector, TotalTorque = FVector::ZeroVector;
	for (int32 I = 0; I < 2; ++I)
	{
		const FVector J = R.Impulses[I] * FVector::UpVector + R.TangentImpulses[I];
		TestTrue(TEXT("edge witnesses satisfy each unilateral Coulomb cone"), R.Impulses[I] >= 0 &&
			R.TangentImpulses[I].Size() <= 0.30000001192092896 * R.Impulses[I] + 1.e-10);
		TotalJ += J; TotalTorque += FVector::CrossProduct(Points[I] - COM, J);
	}
	TestTrue(TEXT("independent point impulses reconstruct both momentum changes"),
		(TotalJ * .001).Equals(R.DeltaVelocity, 1.e-10) &&
		FVector(InvI.TransformVector(TotalTorque)).Equals(R.DeltaAngularVelocity, 1.e-10));
	const FVector Reversed[] = { Points[1], Points[0] };
	const auto Reverse = Speed::SolvePlanarCoulombImpulse(MakeArrayView(Reversed), FVector::UpVector,
		COM, V, W, .001, InvI, 0, 0.30000001192092896);
	TestTrue(TEXT("reversing the edge preserves the response"), Reverse.bSolved &&
		Reverse.DeltaVelocity.Equals(R.DeltaVelocity, 1.e-10) &&
		Reverse.DeltaAngularVelocity.Equals(R.DeltaAngularVelocity, 1.e-10));
	// Construct feasible edge impulses independently across spin, heading and
	// translation. The known final state still rotates about the support edge.
	for (double Angle : { 0.0, 40.0, 80.0 })
	for (double Spin : { 0.0, .6, -4.0 })
	for (double Shift : { 0.0, 3000.0 })
	for (bool bReverse : { false, true })
	{
		const FQuat Q = FRotator(Angle, Angle * .5, Angle * .25).Quaternion();
		const FVector Translation(Shift, -Shift, Shift * .1);
		FVector EdgePoints[] = { Q.RotateVector(FVector(60, -40, 0)) + Translation,
			Q.RotateVector(FVector(60, 40, 0)) + Translation };
		const FVector Center = Q.RotateVector(FVector(7, -3, 20)) + Translation;
		const FVector Normal = Q.RotateVector(FVector::UpVector);
		const FVector Axis = Q.RotateVector(FVector::RightVector);
		FMatrix LocalInverse = FMatrix::Identity;
		LocalInverse.M[0][0] = .00015; LocalInverse.M[1][1] = .000075; LocalInverse.M[2][2] = .00006;
		const FMatrix Rotation = FQuatRotationMatrix(Q);
		const FMatrix WorldInverse = Rotation * LocalInverse * Rotation.GetTransposed();
		const FVector J0 = Q.RotateVector(FVector(0, 0, 500)), J1 = Q.RotateVector(FVector(100, 50, 800));
		const FVector TargetW = Spin * Axis;
		const FVector TargetV = -FVector::CrossProduct(TargetW, EdgePoints[0] - Center);
		const FVector IncomingV = TargetV - .1 * (J0 + J1);
		const FVector IncomingW = TargetW - FVector(WorldInverse.TransformVector(
			FVector::CrossProduct(EdgePoints[0] - Center, J0) + FVector::CrossProduct(EdgePoints[1] - Center, J1)));
		if (bReverse) Swap(EdgePoints[0], EdgePoints[1]);
		const auto Response = Speed::SolvePlanarCoulombImpulse(MakeArrayView(EdgePoints), Normal,
			Center, IncomingV, IncomingW, .1, WorldInverse, 0, .5);
		if (!TestTrue(TEXT("feasible rotated off-center edge has a direct certificate"), Response.bSolved && Response.IterationCount == 0)) continue;
		TestTrue(TEXT("edge free-axis motion is recovered independently of translation and order"),
			(IncomingV + Response.DeltaVelocity).Equals(TargetV, 1.e-8) &&
			(IncomingW + Response.DeltaAngularVelocity).Equals(TargetW, 1.e-10));
		FVector SumJ = FVector::ZeroVector, SumTorque = FVector::ZeroVector;
		for (int32 I = 0; I < 2; ++I)
		{
			const FVector PointJ = Response.Impulses[I] * Normal + Response.TangentImpulses[I];
			TestTrue(TEXT("rotated edge witnesses stay unilateral and inside each friction cone"),
				Response.Impulses[I] >= 0 && Response.TangentImpulses[I].Size() <= .5 * Response.Impulses[I] + 1.e-9);
			SumJ += PointJ; SumTorque += FVector::CrossProduct(EdgePoints[I] - Center, PointJ);
		}
		TestTrue(TEXT("rotated witnesses reconstruct the rigid response"),
			(SumJ * .1).Equals(Response.DeltaVelocity, 1.e-8) &&
			FVector(WorldInverse.TransformVector(SumTorque)).Equals(Response.DeltaAngularVelocity, 1.e-10));
	}
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedPlanarSlidingReactionTest,
	"IAmSpeed.PhysicalLaws.Friction.SupportedFaceSlideAndSpin",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedPlanarSlidingReactionTest::RunTest(const FString& Parameters)
{
	const FVector N(0, 0, 1), COM(0, 0, 20), A(0, 0, -650);
	const FVector Points[] = { FVector(-60, -40, 0), FVector(60, -40, 0),
		FVector(-60, 40, 0), FVector(60, 40, 0) };
	FMatrix InvI = FMatrix::Identity;
	InvI.M[0][0] = 3.0 / (10 * (40.0 * 40 + 20.0 * 20));
	InvI.M[1][1] = 3.0 / (10 * (60.0 * 60 + 20.0 * 20));
	InvI.M[2][2] = 3.0 / (10 * (60.0 * 60 + 40.0 * 40));
	for (double Speed : { -1400.0, 0.0, 450.0, 2300.0 })
	for (double Spin : { -.15, .15 })
	for (double Mu : { 0.0, .3 })
	{
		const FVector V(Speed, 0, 0), W(0, 0, Spin), Alpha(6, -4, 0);
		const auto R = Speed::SolvePlanarSlidingReaction(MakeArrayView(Points), N, COM,
			V, W, A, Alpha, .1, InvI, Mu, 1.0 / 300);
		TestTrue(TEXT("supported sliding/spinning face has a bounded continuous reaction"), R.bSolved);
		if (!R.bSolved) continue;
		FVector Force = FVector::ZeroVector, Torque = FVector::ZeroVector;
		double FrictionPower = 0;
		for (int32 I = 0; I < 4; ++I)
		{
			const FVector Lever = Points[I] - COM;
			const FVector VP = V + FVector::CrossProduct(W, Lever);
			const FVector AP = A + R.DeltaVelocity + FVector::CrossProduct(Alpha + R.DeltaAngularVelocity, Lever) +
				FVector::CrossProduct(W, FVector::CrossProduct(W, Lever));
			TestTrue(TEXT("all support rows balance normal acceleration"), FMath::Abs(AP.Z) < 1.e-7);
			TestTrue(TEXT("pressure is non-attractive and traction obeys Coulomb"), R.Impulses[I] >= 0 &&
				FMath::Abs(R.TangentImpulses[I].Size() - Mu * R.Impulses[I]) < 1.e-8);
			FrictionPower += FVector::DotProduct(R.TangentImpulses[I], VP);
			const FVector F = R.Impulses[I] * N + R.TangentImpulses[I];
			Force += F; Torque += FVector::CrossProduct(Lever, F);
		}
		TestTrue(TEXT("friction never supplies mechanical power"), FrictionPower <= 0);
		TestTrue(TEXT("force witnesses reconstruct the reaction"), (Force * .1).Equals(R.DeltaVelocity, 1.e-8) &&
			FVector(InvI.TransformVector(Torque)).Equals(R.DeltaAngularVelocity, 1.e-8));
		if (Mu == 0) TestTrue(TEXT("frictionless support preserves tangent motion and normal spin"),
			R.DeltaVelocity.X == 0 && R.DeltaVelocity.Y == 0 && R.DeltaAngularVelocity.Z == 0);
		else if (Speed == 0) TestTrue(TEXT("pure normal-axis spin dissipates instead of being ignored"), R.DeltaAngularVelocity.Z * Spin < 0);
		FVector Reversed[] = { Points[3], Points[2], Points[1], Points[0] };
		const auto Reverse = Speed::SolvePlanarSlidingReaction(MakeArrayView(Reversed), N, COM,
			V, W, A, Alpha, .1, InvI, Mu, 1.0 / 300);
		TestTrue(TEXT("point permutation preserves reaction"), Reverse.bSolved &&
			Reverse.DeltaVelocity.Equals(R.DeltaVelocity, 1.e-7) && Reverse.DeltaAngularVelocity.Equals(R.DeltaAngularVelocity, 1.e-7));
	}
	TestFalse(TEXT("sticking is not classified as sliding"), Speed::SolvePlanarSlidingReaction(
		MakeArrayView(Points), N, COM, FVector::ZeroVector, FVector::ZeroVector, A,
		FVector::ZeroVector, .1, InvI, .3, 1.0 / 300).bSolved);
	TestFalse(TEXT("possible slip reversal falls back to event resolution"), Speed::SolvePlanarSlidingReaction(
		MakeArrayView(Points), N, COM, FVector(.01, 0, 0), FVector::ZeroVector, A,
		FVector::ZeroVector, .1, InvI, .3, 1.0 / 300).bSolved);
	// Edge/vertex support must use the same sliding traction law as a face,
	// while leaving the unsupplied vertices free to tip toward the plane.
	for (int32 Count : { 1, 2 })
	for (double Sign : { -1.0, 1.0 })
	{
		const FVector V(450 * Sign, 0, 0);
		const auto R = Speed::SolvePlanarSlidingReaction(MakeArrayView(Points, Count), N,
			COM, V, FVector::ZeroVector, A, FVector::ZeroVector, .1, InvI, .3, 1.0 / 300);
		if (!TestTrue(TEXT("sliding vertex/edge has a coupled reaction"), R.bSolved)) continue;
		FVector Force = FVector::ZeroVector, Torque = FVector::ZeroVector;
		for (int32 I = 0; I < Count; ++I)
		{
			const FVector Lever = Points[I] - COM;
			const FVector F = R.Impulses[I] * N + R.TangentImpulses[I];
			Force += F; Torque += FVector::CrossProduct(Lever, F);
			TestTrue(TEXT("sliding partial contact balances normal load and dissipates"),
				FMath::Abs((A + R.DeltaVelocity + FVector::CrossProduct(R.DeltaAngularVelocity, Lever)).Z) < 1.e-7 &&
				R.Impulses[I] >= 0 && FVector::DotProduct(R.TangentImpulses[I], V) < 0 &&
				FMath::Abs(R.TangentImpulses[I].Size() - .3 * R.Impulses[I]) < 1.e-8);
		}
		TestTrue(TEXT("partial reaction reconstructs force and torque without locking orientation"),
			(Force * .1).Equals(R.DeltaVelocity, 1.e-8) &&
			FVector(InvI.TransformVector(Torque)).Equals(R.DeltaAngularVelocity, 1.e-8) &&
			R.DeltaAngularVelocity.Size() > 1);
	}
	return true;
}
#endif
