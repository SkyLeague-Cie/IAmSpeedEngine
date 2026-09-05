#if WITH_DEV_AUTOMATION_TESTS
#include "AnalyticWorldData.h"
#include "Misc/AutomationTest.h"
#include "Math/RandomStream.h"

namespace
{
	FVector3d OriginalDerivative(const Speed::Analytic::FExtrudedQuinticPatch& Patch, double T)
	{
		const double ClampedT = FMath::Clamp(T, 0.0, 1.0);
		const double OneMinusT = 1.0 - ClampedT;
		const double Basis[5] = {
			FMath::Pow(OneMinusT, 4.0),
			4.0 * ClampedT * FMath::Pow(OneMinusT, 3.0),
			6.0 * FMath::Square(ClampedT) * FMath::Square(OneMinusT),
			4.0 * FMath::Pow(ClampedT, 3.0) * OneMinusT,
			FMath::Pow(ClampedT, 4.0) };
		FVector3d Result = FVector3d::ZeroVector;
		for (int32 Index = 0; Index < 5; ++Index)
			Result += 5.0 * Basis[Index] *
				(Patch.SectionControlPoints[Index + 1] - Patch.SectionControlPoints[Index]);
		const double CorrectionDerivativeA =
			105.0 * FMath::Square(ClampedT) * FMath::Pow(OneMinusT, 4.0) -
			140.0 * FMath::Pow(ClampedT, 3.0) * FMath::Pow(OneMinusT, 3.0);
		const double CorrectionDerivativeB =
			140.0 * FMath::Pow(ClampedT, 3.0) * FMath::Pow(OneMinusT, 3.0) -
			105.0 * FMath::Pow(ClampedT, 4.0) * FMath::Square(OneMinusT);
		Result += CorrectionDerivativeA * Patch.InteriorCorrectionControlPoints[0] +
			CorrectionDerivativeB * Patch.InteriorCorrectionControlPoints[1];
		return Result;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedExtrudedDerivativeTest,
	"IAmSpeed.AnalyticWorld.ExtrudedDerivativeBits",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedExtrudedDerivativeTest::RunTest(const FString& Parameters)
{
	Speed::Analytic::FExtrudedQuinticPatch Patch;
	FRandomStream Random(150935);
	const auto Check = [this, &Patch](double T)
	{
		const FVector3d Expected = OriginalDerivative(Patch, T);
		const FVector3d Actual = Patch.EvaluateSectionDerivative(T);
		return TestTrue(TEXT("quintic derivative retains every original output bit"),
			FMemory::Memcmp(&Expected, &Actual, sizeof(FVector3d)) == 0);
	};
	for (int32 Sample = 0; Sample < 1024; ++Sample)
	{
		const auto Vector = [&]()
		{
			return Sample % 11 == 0 ? FVector3d(-0.0, 0.0, -0.0) :
				FVector3d(Random.FRandRange(-10000, 10000), Random.FRandRange(-10000, 10000),
					Random.FRandRange(-10000, 10000));
		};
		for (FVector3d& Point : Patch.SectionControlPoints) Point = Vector();
		for (FVector3d& Point : Patch.InteriorCorrectionControlPoints) Point = Vector();
		for (double T : { -1.0, -0.0, 0.0, 0.5, 1.0, 2.0, static_cast<double>(Random.FRand()) })
			if (!Check(T)) return false;
	}
	return true;
}
#endif
