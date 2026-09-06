#if WITH_DEV_AUTOMATION_TESTS
#include "SpeedWheeledComponent.h"
#include "Misc/AutomationTest.h"
#include "Math/RandomStream.h"
#include "UObject/StrongObjectPtr.h"
#include "HAL/IConsoleManager.h"
#include "Misc/ScopeExit.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedWheeledInertiaCovarianceTest,
	"IAmSpeed.PhysicalLaws.VehicleInertia.Covariance",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedWheeledInertiaCovarianceTest::RunTest(const FString& Parameters)
{
	IConsoleVariable* Candidate = IConsoleManager::Get().FindConsoleVariable(
		TEXT("p.IAmSpeed.VehicleInertia.CovariantFrame"));
	if (!TestNotNull(TEXT("development-only candidate is explicitly selectable"), Candidate)) return false;
	const int32 Previous = Candidate->GetInt();
	Candidate->Set(1, ECVF_SetByCode);
	ON_SCOPE_EXIT { Candidate->Set(Previous, ECVF_SetByCode); };
	TStrongObjectPtr<USpeedWheeledComponent> Body(NewObject<USpeedWheeledComponent>());
	FRandomStream Random(150315);
	// A chassis tensor need not be diagonal: a rotated/offset box creates
	// products of inertia. The oracle only rotates vectors, not tensor matrices.
	for (const bool bProducts : { false, true })
	{
		FMatrix Local = FMatrix::Identity;
		Local.M[0][0] = 1.0 / 8000;
		Local.M[1][1] = 1.0 / 17000;
		Local.M[2][2] = 1.0 / 23000;
		if (bProducts)
		{
			Local.M[0][2] = Local.M[2][0] = 1.0 / 100000;
			Local.M[0][1] = Local.M[1][0] = -1.0 / 200000;
		}
		Body->CarLocalInvI = Local;
		for (int32 Sample = 0; Sample < 100; ++Sample)
		{
			SKinematic State;
			State.Rotation = FRotator(Random.FRandRange(-180, 180),
				Random.FRandRange(-180, 180), Random.FRandRange(-180, 180)).Quaternion();
			if (Sample < 2) State.Rotation = FRotator(
				Sample == 0 ? 89.9999 : -89.9999, 30, 40).Quaternion();
			Body->SetKinematicState(State);
			for (const FVector& Torque : { FVector(10000, 0, 0),
				FVector(0, 10000, 0), FVector(0, 0, 10000) })
			{
				const FVector Expected = State.Rotation.RotateVector(
					Local.TransformVector(State.Rotation.UnrotateVector(Torque)));
				const FVector Actual = Body->ComputeWorldInvInertiaTensor().TransformVector(Torque);
				if (!TestTrue(TEXT("chassis response equals unrotate/local response/rotate, including pitch poles"),
					Actual.Equals(Expected, 256 * DBL_EPSILON * FMath::Max(1.0, Expected.Size()))))
				{
					AddInfo(FString::Printf(TEXT("Products=%d Sample=%d Error=%.12g Expected=%s Actual=%s"),
						bProducts, Sample, (Actual - Expected).Size(), *Expected.ToString(), *Actual.ToString()));
					return false;
				}
			}
		}
	}
	return true;
}
#endif
