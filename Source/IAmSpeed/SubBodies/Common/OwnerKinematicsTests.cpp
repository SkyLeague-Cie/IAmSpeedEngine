#if WITH_DEV_AUTOMATION_TESTS
#include "IAmSpeed/SubBodies/SSubBody.h"
#include "IAmSpeed/SubBodies/Sensor/SphereSensor.h"
#include "IAmSpeed/SubBodies/Solid/SWheelSubBody.h"
#include "Misc/AutomationTest.h"
#include "Math/RandomStream.h"
#include "UObject/StrongObjectPtr.h"

namespace
{
	SKinematic OriginalOwnerTransform(const SKinematic& Parent, const FVector& Local, const FRotator& Rotation)
	{
		SKinematic Result;
		const FQuat LocalRotation = Rotation.Quaternion();
		const FVector Offset = Parent.Rotation.RotateVector(Local);
		Result.Location = Parent.Location + Offset;
		Result.Velocity = Parent.Velocity + FVector::CrossProduct(Parent.AngularVelocity, Offset);
		Result.Acceleration = Parent.Acceleration + FVector::CrossProduct(Parent.AngularAcceleration, Offset) +
			FVector::CrossProduct(Parent.AngularVelocity, FVector::CrossProduct(Parent.AngularVelocity, Offset));
		Result.Rotation = Parent.Rotation * LocalRotation;
		Result.AngularVelocity = Parent.AngularVelocity;
		Result.AngularAcceleration = Parent.AngularAcceleration;
		return Result;
	}

	bool EqualStateBits(const SKinematic& A, const SKinematic& B)
	{
		const auto EqualVector = [](const FVector& X, const FVector& Y)
		{
			return FMemory::Memcmp(&X, &Y, sizeof(FVector)) == 0;
		};
		return EqualVector(A.Location, B.Location) && EqualVector(A.Velocity, B.Velocity) &&
			EqualVector(A.Acceleration, B.Acceleration) && EqualVector(A.AngularVelocity, B.AngularVelocity) &&
			EqualVector(A.AngularAcceleration, B.AngularAcceleration) &&
			FMemory::Memcmp(&A.Rotation, &B.Rotation, sizeof(FQuat)) == 0;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedOwnerKinematicsTest,
	"IAmSpeed.Simulation.OwnerKinematics",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedOwnerKinematicsTest::RunTest(const FString& Parameters)
{
	TStrongObjectPtr<USSubBody> Direct(NewObject<USphereSensor>());
	SKinematic Parent;
	FVector Local(3, -7, 9);
	FRotator Rotation(12, 34, -56);
	const auto Check = [&]()
	{
		Direct->SetRelativeLocation(Local);
		Direct->SetRelativeRotation(Rotation);
		const SKinematic Expected = OriginalOwnerTransform(
			Parent, Direct->GetRelativeLocation(), Direct->GetRelativeRotation());
		for (int32 Repeat = 0; Repeat < 3; ++Repeat)
		{
			Direct->SetKinematicState(SKinematic());
			Direct->UpdateKinematicsFromOwner(Parent);
			if (!TestTrue(TEXT("repeated owner transforms retain every original output bit"),
				EqualStateBits(Expected, Direct->GetKinematicState()))) return false;
		}
		return true;
	};
	if (!Check()) return false;
	// Change every input independently, including accelerations and signed zeros.
	double* Fields[] = {
		&Parent.Location.X, &Parent.Location.Y, &Parent.Location.Z,
		&Parent.Velocity.X, &Parent.Velocity.Y, &Parent.Velocity.Z,
		&Parent.Acceleration.X, &Parent.Acceleration.Y, &Parent.Acceleration.Z,
		&Parent.Rotation.X, &Parent.Rotation.Y, &Parent.Rotation.Z, &Parent.Rotation.W,
		&Parent.AngularVelocity.X, &Parent.AngularVelocity.Y, &Parent.AngularVelocity.Z,
		&Parent.AngularAcceleration.X, &Parent.AngularAcceleration.Y, &Parent.AngularAcceleration.Z,
		&Local.X, &Local.Y, &Local.Z, &Rotation.Pitch, &Rotation.Yaw, &Rotation.Roll };
	for (double* Field : Fields)
	{
		const double Saved = *Field;
		for (const double Value : { Saved + .25, -0.0, 0.0, Saved })
		{
			*Field = Value;
			if (!Check()) return false;
		}
	}
	FRandomStream Random(81503);
	const auto Vector = [&]()
	{
		return FVector(Random.FRandRange(-1000, 1000), Random.FRandRange(-1000, 1000), Random.FRandRange(-1000, 1000));
	};
	for (int32 Sample = 0; Sample < 1000; ++Sample)
	{
		Parent.Location = Vector(); Parent.Velocity = Vector(); Parent.Acceleration = Vector();
		Parent.AngularVelocity = Vector() * .01; Parent.AngularAcceleration = Vector() * .01;
		Parent.Rotation = FRotator(Random.FRandRange(-180, 180), Random.FRandRange(-180, 180), Random.FRandRange(-180, 180)).Quaternion();
		Local = Vector();
		Rotation = FRotator(Random.FRandRange(-360, 360), Random.FRandRange(-360, 360), Random.FRandRange(-360, 360));
		if (!Check()) return false;
	}
	// The real base/virtual wheel paths must replace intervening integration or
	// suspension state even when the parent/local inputs themselves are identical.
	// USSubBody deliberately has no concrete Unreal CalcBounds implementation.
	// A sphere sensor supplies bounds while inheriting the ordinary base transform.
	TStrongObjectPtr<USSubBody> Ordinary(NewObject<USphereSensor>());
	TStrongObjectPtr<USWheelSubBody> Wheel(NewObject<USWheelSubBody>());
	for (USSubBody* Body : { Ordinary.Get(), static_cast<USSubBody*>(Wheel.Get()) })
	{
		Body->SetRelativeLocation(FVector(4, -2, 17));
		Body->SetRelativeRotation(FRotator(16, 37, -11));
		for (int32 Repeat = 0; Repeat < 8; ++Repeat)
		{
			SKinematic Poison; Poison.Location = FVector(1.0e9);
			Body->SetKinematicState(Poison);
			Body->UpdateKinematicsFromOwner(Parent);
			SKinematic Expected = OriginalOwnerTransform(Parent, Body->GetRelativeLocation(), Body->GetRelativeRotation());
			if (Body == Wheel.Get())
			{
				Expected.Location += Parent.Rotation.RotateVector(Wheel->SpringDisplacement() * FVector::UpVector);
				Expected.Velocity = Parent.Velocity + FVector::CrossProduct(Parent.AngularVelocity, Expected.Location - Parent.Location);
			}
			if (!TestTrue(TEXT("ordinary and wheel virtual updates restore their exact live result"),
				EqualStateBits(Expected, Body->GetKinematicState()))) return false;
		}
	}
	return true;
}
#endif
