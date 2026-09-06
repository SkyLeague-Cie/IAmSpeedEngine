#if WITH_DEV_AUTOMATION_TESTS
#include "AnalyticWorldData.h"
#include "Misc/AutomationTest.h"
#include "Math/RandomStream.h"
#include "HAL/PlatformTime.h"

namespace
{
	// Independent original copy-then-reduce implementation. Do not share the
	// production in-place helper: this oracle protects operation order and bits.
	FVector3d OriginalCurve(TArrayView<const FVector3d> ControlPoints, double Parameter)
	{
		if (ControlPoints.IsEmpty()) return FVector3d::ZeroVector;
		TArray<FVector3d, TInlineAllocator<16>> Work;
		Work.Append(ControlPoints.GetData(), ControlPoints.Num());
		const double T = FMath::Clamp(Parameter, 0.0, 1.0);
		for (int32 Remaining = Work.Num() - 1; Remaining > 0; --Remaining)
		{
			for (int32 Index = 0; Index < Remaining; ++Index)
				Work[Index] = FMath::Lerp(Work[Index], Work[Index + 1], T);
		}
		return Work[0];
	}

	FVector3d OriginalTensor(const Speed::Analytic::FTensorBezierSurface& Surface,
		double U, double V, int32 Derivative)
	{
		const int32 DegreeU = Surface.DegreeU;
		const int32 DegreeV = Surface.DegreeV;
		if (DegreeU < 0 || DegreeV < 0 ||
			(Derivative == 1 && DegreeU == 0) || (Derivative == 2 && DegreeV == 0) ||
			Surface.ControlPoints.Num() != (DegreeU + 1) * (DegreeV + 1))
			return FVector3d::ZeroVector;
		TArray<FVector3d, TInlineAllocator<16>> AlongU;
		const int32 VCount = DegreeV + 1;
		for (int32 UIndex = 0; UIndex <= DegreeU - (Derivative == 1 ? 1 : 0); ++UIndex)
		{
			TArray<FVector3d, TInlineAllocator<16>> AlongV;
			for (int32 VIndex = 0; VIndex <= DegreeV - (Derivative == 2 ? 1 : 0); ++VIndex)
			{
				const FVector3d& Point = Surface.ControlPoints[UIndex * VCount + VIndex];
				if (Derivative == 1)
					AlongV.Add(static_cast<double>(DegreeU) *
						(Surface.ControlPoints[(UIndex + 1) * VCount + VIndex] - Point));
				else if (Derivative == 2)
					AlongV.Add(static_cast<double>(DegreeV) *
						(Surface.ControlPoints[UIndex * VCount + VIndex + 1] - Point));
				else
					AlongV.Add(Point);
			}
			AlongU.Add(OriginalCurve(AlongV, V));
		}
		return OriginalCurve(AlongU, U);
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedTensorEvaluationTest,
	"IAmSpeed.AnalyticWorld.TensorEvaluationBits",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedTensorEvaluationTest::RunTest(const FString& Parameters)
{
	using namespace Speed::Analytic;
	FRandomStream Random(916037);
	const int32 Degrees[] = { 0, 1, 2, 3, 5, 7, 15, 16 };
	const double Values[] = { -0.5, -0.0, 0.0, 0.371923, 1.0, 1.5 };
	const auto EqualBits = [this](const FVector3d& A, const FVector3d& B)
	{
		return TestTrue(TEXT("tensor evaluation retains original output bits"),
			FMemory::Memcmp(&A, &B, sizeof(FVector3d)) == 0);
	};
	// Include asymmetric degrees, degree zero, the supported maximum, and the
	// public evaluator's above-inline path (not an authoring-valid degree).
	for (int32 DegreeU : Degrees)
	{
		for (int32 DegreeV : Degrees)
		{
			FTensorBezierSurface Surface;
			Surface.DegreeU = DegreeU;
			Surface.DegreeV = DegreeV;
			for (int32 Index = 0; Index < (DegreeU + 1) * (DegreeV + 1); ++Index)
			{
				Surface.ControlPoints.Add(Index % 7 == 0 ? FVector3d(-0.0, 0.0, -0.0) :
					FVector3d(Random.FRandRange(-10000, 10000), Random.FRandRange(-10000, 10000),
						Random.FRandRange(-10000, 10000)));
			}
			const auto OriginalPoints = Surface.ControlPoints;
			for (int32 Sample = 0; Sample < UE_ARRAY_COUNT(Values); ++Sample)
			{
				const double U = Values[Sample];
				const double V = Values[(Sample + 3) % UE_ARRAY_COUNT(Values)];
				const FVector3d ExpectedU = OriginalTensor(Surface, U, V, 1);
				const FVector3d ExpectedV = OriginalTensor(Surface, U, V, 2);
				if (!EqualBits(Surface.Evaluate(U, V), OriginalTensor(Surface, U, V, 0)) ||
					!EqualBits(Surface.EvaluateDerivativeU(U, V), ExpectedU) ||
					!EqualBits(Surface.EvaluateDerivativeV(U, V), ExpectedV) ||
					!EqualBits(Surface.EvaluateNormal(U, V),
						FVector3d::CrossProduct(ExpectedU, ExpectedV).GetSafeNormal())) return false;
			}
			if (!TestTrue(TEXT("evaluation never modifies authored controls"),
				FMemory::Memcmp(Surface.ControlPoints.GetData(), OriginalPoints.GetData(),
					OriginalPoints.Num() * sizeof(FVector3d)) == 0)) return false;
			Surface.ControlPoints.Pop();
			if (!EqualBits(Surface.Evaluate(.3, .7), FVector3d::ZeroVector) ||
				!EqualBits(Surface.EvaluateDerivativeU(.3, .7), FVector3d::ZeroVector) ||
				!EqualBits(Surface.EvaluateDerivativeV(.3, .7), FVector3d::ZeroVector)) return false;
		}
	}
	return true;
}

namespace
{
	template<bool NormalApi>
	FORCENOINLINE double MeasureTensorNormal(const TArray<Speed::Analytic::FTensorBezierSurface>& Surfaces, uint64& Checksum)
	{
		uint64 Sum = 0;
		const double Start = FPlatformTime::Seconds();
		for (int32 Repeat = 0; Repeat < 64; ++Repeat)
		{
			for (int32 Index = 0; Index < Surfaces.Num(); ++Index)
			{
				const auto& Surface = Surfaces[Index];
				const double U = (Index % 23) / 22.0, V = (Index % 37) / 36.0;
				FVector3d Normal;
				if constexpr (NormalApi) Normal = Surface.EvaluateNormal(U, V);
				else Normal = FVector3d::CrossProduct(Surface.EvaluateDerivativeU(U, V), Surface.EvaluateDerivativeV(U, V)).GetSafeNormal();
				uint64 Bits;
				FMemory::Memcpy(&Bits, &Normal.X, sizeof(Bits));
				Sum = Sum * 33 + Bits;
			}
		}
		const double Elapsed = FPlatformTime::Seconds() - Start;
		Checksum = Sum;
		return Elapsed * 1.e9 / (64 * Surfaces.Num());
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedBicubicNormalTest,
	"IAmSpeed.AnalyticWorld.BicubicNormal", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedBicubicNormalTest::RunTest(const FString& Parameters)
{
	using namespace Speed::Analytic;
	FRandomStream Random(150951);
	TArray<FTensorBezierSurface> Surfaces;
	for (int32 Index = 0; Index < 1024; ++Index)
	{
		auto& Surface = Surfaces.AddDefaulted_GetRef();
		Surface.DegreeU = Surface.DegreeV = 3;
		for (int32 I = 0; I < 16; ++I)
			Surface.ControlPoints.Add(I % 7 == 0 ? FVector3d(-0.0, 0.0, -0.0) :
				FVector3d(Random.FRandRange(-10000.f, 10000.f), Random.FRandRange(-10000.f, 10000.f), Random.FRandRange(-10000.f, 10000.f)));
		for (const double U : { -0.5, -0.0, 0.0, 0.371923, 1.0, 1.5 })
		{
			const double V = Random.FRandRange(-0.5f, 1.5f);
			const FVector3d Expected = FVector3d::CrossProduct(OriginalTensor(Surface, U, V, 1), OriginalTensor(Surface, U, V, 2)).GetSafeNormal();
			const FVector3d Actual = Surface.EvaluateNormal(U, V);
			if (!TestTrue(TEXT("bicubic normal retains independent oracle bits"), FMemory::Memcmp(&Expected, &Actual, sizeof(Actual)) == 0)) return false;
		}
	}
	uint64 OriginalSum = 0, SpecializedSum = 0;
	MeasureTensorNormal<false>(Surfaces, OriginalSum);
	MeasureTensorNormal<true>(Surfaces, SpecializedSum);
	for (int32 Pair = 0; Pair < 5; ++Pair)
	{
		double Original, Specialized;
		if (Pair % 2 == 0)
		{
			Original = MeasureTensorNormal<false>(Surfaces, OriginalSum);
			Specialized = MeasureTensorNormal<true>(Surfaces, SpecializedSum);
		}
		else
		{
			Specialized = MeasureTensorNormal<true>(Surfaces, SpecializedSum);
			Original = MeasureTensorNormal<false>(Surfaces, OriginalSum);
		}
		TestEqual(TEXT("normal microbenchmark checksums match"), OriginalSum, SpecializedSum);
		AddInfo(FString::Printf(TEXT("[BicubicNormalMicro] Pair=%d OriginalNs=%.6f SpecializedNs=%.6f"), Pair, Original, Specialized));
	}
	return true;
}
#endif
