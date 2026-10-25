#if WITH_DEV_AUTOMATION_TESTS
#include "KinematicQuantizationCache.h"
#include "SUtils.h"
#include "Misc/AutomationTest.h"
#include "Math/RandomStream.h"
#include "IAmSpeed/Components/ISpeedComponent.h"
#include "IAmSpeed/SubBodies/Configs/SubBodyConfig.h"

namespace
{
class FStationaryQuantizationBody final : public ISpeedComponent
{
public:
	using ISpeedComponent::QuantizeKinematicState;
	void PrepareCanonicalFrame(const FCanonicalFrameContext&) override {}
	unsigned int NumFrame() const override { return 1; }
	float GetPhysMass() const override { return 1; }
	const FVector& GetPhysCOM() const override { return Zero; }
	const FVector& GetPhysCenterOfMassLocal() const override { return Zero; }
	const TArray<USSubBody*>& GetSubBodies() const override { return SubBodies; }
	TArray<USSubBody*> CreateSubBodies() override { return {}; }
	bool IsFrozen() const override { return false; }
	SubBodyConfig GetSubBodyConfig(const USSubBody&) const override { return {}; }
	SKinematic GetKinematicsOfSubBody(const USSubBody&, const unsigned int&) const override { return State; }
	FMatrix ComputeWorldInvInertiaTensor() const override { return FMatrix::Identity; }
	FMatrix ComputeWorldInvInertiaTensorOfSubBody(const USSubBody&) const override { return FMatrix::Identity; }
	const SKinematic& GetKinematicState() const override { return State; }
	const SKinematic& GetKinematicStateForFrame(const unsigned int&) const override { return State; }
	float GetPhysMaxSpeed() const override { return 0; }
	float GetPhysMaxAngularSpeed() const override { return 0; }
	void SetIsUpsideDown(bool) override {}
	bool IsUpsideDown() const override { return false; }
	bool IsInAutoRecover() const override { return false; }
	bool IsSubBodyInAutoRecoverMode() const override { return false; }
	void RcvImpactOnSubBody(const USSubBody&, const FVector&) override {}
protected:
	void PostPhysicsUpdatePrv(const float&) override {}
	void SetKinematicState(const SKinematic& S) override { State=S; }
	void SetIsFrozen(bool) override {}
private:
	FVector Zero = FVector::ZeroVector;
	SKinematic State;
	TArray<USSubBody*> SubBodies;
};
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedKinematicQuantizationTest,
	"IAmSpeed.Simulation.IdentityKinematicQuantizationCache",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedKinematicQuantizationTest::RunTest(const FString& Parameters)
{
	Speed::FIdentityKinematicQuantizationCache Cache;
	const auto Check = [this, &Cache](Speed::FKinematicState State, const FQuat& PreviousRotation)
	{
		Speed::FKinematicState Expected = State;
		Expected.Quantize(PreviousRotation); // Uncached outer operation; independent of this no-op cache.
		const bool bReused = Cache.Quantize(State, PreviousRotation);
		const auto EqualVector = [](const FVector& A, const FVector& B)
		{
			return FMemory::Memcmp(&A, &B, sizeof(FVector)) == 0;
		};
		TestTrue(TEXT("all fields match the canonical quantizer bit for bit"),
			EqualVector(Expected.Location, State.Location) && EqualVector(Expected.Velocity, State.Velocity) &&
			EqualVector(Expected.AngularVelocity, State.AngularVelocity) &&
			EqualVector(Expected.Acceleration, State.Acceleration) &&
			EqualVector(Expected.AngularAcceleration, State.AngularAcceleration) &&
			FMemory::Memcmp(&Expected.Rotation, &State.Rotation, sizeof(FQuat)) == 0);
		return bReused;
	};
	Speed::FKinematicState State;
	// Build an exact point-fix rather than assuming the identity quaternion has the quantizer's signed zeros.
	State.Quantize(FQuat::Identity);
	const FQuat Previous = State.Rotation;
	State.Quantize(Previous);
	TestFalse(TEXT("first encounter is evaluated"), Check(State, Previous));
	TestTrue(TEXT("proven no-op is reused"), Check(State, Previous));
	State.Acceleration = FVector(1.25, -4.75, 650.0);
	State.AngularAcceleration = FVector(-1.0, 2.0, .125);
	TestTrue(TEXT("unquantized accelerations remain live on reuse"), Check(State, Previous));
	Cache.Reset();
	TestFalse(TEXT("restore invalidation forces reevaluation"), Check(State, Previous));
	Check(State, FQuat(-Previous.X, -Previous.Y, -Previous.Z, -Previous.W));
	State.Location.X = -0.0;
	Check(State, Previous);
	for (double Boundary : { -.005000001, -.005, -.004999999, .004999999, .005, .005000001 })
	{
		State.Location.X = Boundary;
		State.Velocity.Z = Boundary;
		State.AngularVelocity.Y = Boundary * .01;
		Check(State, Previous);
	}
	FRandomStream Random(0x1de17);
	// Missing history may return the live state itself, so the reference can alias Rotation.
	Speed::FKinematicState Aliased;
	Aliased.Rotation = FRotator(13.7, -162.3, 8.1).Quaternion();
	for (int32 Repeat = 0; Repeat < 10; ++Repeat)
	{
		Speed::FKinematicState Expected = Aliased;
		Expected.Quantize(Expected.Rotation);
		Cache.Quantize(Aliased, Aliased.Rotation);
		TestTrue(TEXT("live rotation reference alias is preserved"),
			FMemory::Memcmp(&Expected.Rotation, &Aliased.Rotation, sizeof(FQuat)) == 0);
	}
	for (int32 Index = 0; Index < 1000; ++Index)
	{
		State.Location = Random.VRand() * 7000.0;
		State.Velocity = Random.VRand() * 2000.0;
		State.AngularVelocity = Random.VRand() * 5.5;
		State.Rotation = FRotator(Random.FRandRange(-180.f, 180.f),
			Random.FRandRange(-180.f, 180.f), Random.FRandRange(-180.f, 180.f)).Quaternion();
		const FQuat Reference = State.Rotation;
		for (int32 Repeat = 0; Repeat < 4; ++Repeat)
		{
			Check(State, Reference);
			State.Quantize(Reference);
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedQuantizationLargeCoordinates042Test,
	"IAmSpeed.PhysicalLaws.QuantizationLargeCoordinates042",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedQuantizationLargeCoordinates042Test::RunTest(const FString&)
{
	for (const uint32 Scale : {100u, 10000u})
	{
		for (const double Offset : {0.0, 1000.0, -1048576.0, 2097152.0, 100000000.0, -100000000.0})
		{
			for (const double Local : {-25.673, -12.341, -0.013, 0.017, 12.342, 25.677})
			{
				const FVector Input(Offset + Local, Offset - Local, Offset + .01);
				const FVector Rounded = Speed::RoundVectorToNetQuantize(Input, Scale);
				const double HalfStep = .5 / double(Scale);
				TestTrue(TEXT("rounding error stays within half the declared grid at large coordinates"),
					(Rounded - Input).GetAbsMax() <= HalfStep + 2.e-8);
				const FVector LocalRounded = Speed::RoundVectorToNetQuantize(FVector(Local, -Local, .01), Scale);
				TestTrue(TEXT("whole-grid translations preserve the rounded local coordinates"),
					(Rounded - FVector(Offset) - LocalRounded).GetAbsMax() < 2.e-8);
			}
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedStationaryAxes042Test,
	"IAmSpeed.PhysicalLaws.StationaryAxesTranslation042",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedStationaryAxes042Test::RunTest(const FString&)
{
	FStationaryQuantizationBody Component;
	for (const FVector Offset : {FVector::ZeroVector, FVector(.003,-.004,.001),
		FVector(100000000,-100000000,100000000)})
	{
		SKinematic State;
		State.Location = Offset + FVector(-25.673,3.017,12.341);
		State.Velocity = FVector(1200,0,0);
		State.Acceleration = FVector::ZeroVector;
		const FVector Authored = State.Location;
		Component.QuantizeKinematicState(State,FQuat::Identity);
		TestEqual(TEXT("unmoving Y preserves its exact authored coordinate"),State.Location.Y,Authored.Y);
		TestEqual(TEXT("unmoving Z preserves its exact authored coordinate"),State.Location.Z,Authored.Z);
		TestEqual(TEXT("unforced moving X preserves the integrated coordinate"),State.Location.X,Authored.X);
		State.Acceleration.Y = 1.;
		Component.QuantizeKinematicState(State,FQuat::Identity);
		TestEqual(TEXT("new acceleration revokes stationary-axis preservation"),State.Location.Y,
			Speed::RoundVectorToNetQuantize(Authored,100).Y);
		State.Location.Z = Authored.Z; State.Velocity.Z = .00001;
		Component.QuantizeKinematicState(State,FQuat::Identity);
		TestEqual(TEXT("unforced sub-grid velocity does not inject a position snap"),State.Location.Z,Authored.Z);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedUnforcedTrajectory042Test,
	"IAmSpeed.PhysicalLaws.UnforcedTrajectory042",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedUnforcedTrajectory042Test::RunTest(const FString&)
{
	FStationaryQuantizationBody Component;
	for (const int32 Hz : {60,100,120,240,300,360,720,1440})
	for (const FVector Offset : {FVector::ZeroVector,FVector(.003,-.004,.001),
		FVector(1000,-2000,3000),FVector(-1048576,2097152,-524288),FVector(1e8,-1e8,1e8)})
	{
		SKinematic State;
		const FVector Start=Offset+FVector(-121.6,3.017,12.341);
		const FVector Velocity(4600,80,-300);
		State.Location=Start;State.Velocity=Velocity;State.Acceleration=FVector::ZeroVector;
		const float Delta=1.f/float(Hz);
		for (int32 Frame=0;Frame<Hz/20;++Frame)
		{
			State=State.Integrate(Delta);
			Component.QuantizeKinematicState(State,FQuat::Identity);
			const FVector Expected=Start+Velocity*(double(Frame+1)*double(Delta));
			TestTrue(TEXT("unforced trajectory follows distance=velocity*time at every rate and translation"),
				(State.Location-Expected).GetAbsMax()<1e-5);
			TestTrue(TEXT("declared constant grid velocities remain exact"),State.Velocity==Velocity);
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedSphereSphereContinuous042Test,
	"IAmSpeed.PhysicalLaws.SphereSphereContinuous042",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedSphereSphereContinuous042Test::RunTest(const FString&)
{
	using Speed::SSphere;
	for (const int32 Hz : {60, 100, 120, 240, 300, 360, 720, 1440})
	{
		for (const FVector Offset : {FVector::ZeroVector, FVector(1000,-2000,3000),
			FVector(-1048576,2097152,-524288), FVector(1.e8,-1.e8,1.e8), FVector(.003,-.004,.001)})
		{
			const float Delta = 1.f / float(Hz);
			const FVector CommonVelocity(50, 10, -20);
			const FVector Velocity = FVector(4600,0,0) + CommonVelocity;
			const FVector Initial = Offset + FVector(-25.0 - 4600.0 * .021, 0, 0);
			bool Found = false;
			for (int32 Frame=0; Frame<Hz/20 && !Found; ++Frame)
			{
				const double StartSeconds = double(Frame) * double(Delta);
				const SSphere A(Initial + Velocity * StartSeconds, 12.5f, Velocity, FVector::ZeroVector);
				const SSphere B(Offset + CommonVelocity * StartSeconds, 12.5f, CommonVelocity, FVector::ZeroVector);
				const SHitResult Hit = A.IntersectNextFrame(B, Delta);
				if (!Hit.bHit) continue;
				Found = true;
				const double Seconds = StartSeconds + double(Hit.TOI);
				TestTrue(TEXT("first impact matches the independent distance/speed oracle at every rate"),
					FMath::Abs(Seconds - .021) < 5.e-8);
				TestTrue(TEXT("relative contact normal is translation invariant"),
					Hit.ImpactNormal.Equals(FVector(-1,0,0), 1.e-7));
				const FVector ExpectedPoint = Offset + CommonVelocity * Seconds + FVector(-12.5,0,0);
				TestTrue(TEXT("impact witness preserves common translation and common velocity"),
					(Hit.ImpactPoint - ExpectedPoint).Size() < .0001);
			}
			TestTrue(TEXT("coarse through-crossing is detected even with separated frame endpoints"), Found);
		}
	}
	const SSphere Origin(FVector::ZeroVector,12.5f,FVector::ZeroVector,FVector::ZeroVector);
	const SSphere Tangent(FVector(-30,25,0),12.5f,FVector(3000,0,0),FVector::ZeroVector);
	const SHitResult Grazing = Tangent.IntersectNextFrame(Origin,.02f);
	TestTrue(TEXT("affine grazing contact is retained"), Grazing.bHit && FMath::Abs(Grazing.TOI-.01f)<1.e-7f);
	const SSphere Clear(FVector(-30,25.01,0),12.5f,FVector(3000,0,0),FVector::ZeroVector);
	TestFalse(TEXT("a true near miss is not turned into a contact"),Clear.IntersectNextFrame(Origin,.02f).bHit);
	const SSphere Accelerating(FVector(-30,0,0),12.5f,FVector::ZeroVector,FVector(2000,0,0));
	const SHitResult Accelerated = Accelerating.IntersectNextFrame(Origin,.1f);
	TestTrue(TEXT("accelerated refinement returns its accepted root rather than averaging a stale endpoint"),
		Accelerated.bHit && FMath::Abs(double(Accelerated.TOI)-FMath::Sqrt(.005))<1.e-6);
	return true;
}
#endif
