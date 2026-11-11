#if WITH_DEV_AUTOMATION_TESTS
#include "AnalyticWorldData.h"
#include "Misc/AutomationTest.h"
#include "Math/RandomStream.h"
#include "HAL/PlatformTime.h"
#include "IAmSpeed/Base/SHitResult.h"
#include "AnalyticWorldQuery.h"
#include "HAL/IConsoleManager.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedTensorTriangulationBoundTest,
	"IAmSpeed.AnalyticWorld.TensorTriangulationBound",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedTensorTriangulationBoundTest::RunTest(const FString& Parameters)
{
	using namespace Speed::Analytic;
	FTensorBezierSurface Surface;
	Surface.DegreeU = Surface.DegreeV = 1;
	// Independent bilinear saddle z=4*u*v. Its corner-triangle diagonal has
	// z=2 at the parameter center, while the analytic surface has z=1.
	Surface.ControlPoints = { FVector3d(0, 0, 0), FVector3d(0, 1, 0),
		FVector3d(1, 0, 0), FVector3d(1, 1, 4) };
	TArray<FTensorBezierApproximationCell> Cells;
	double MaximumError = 0.0;
	if (!TestTrue(TEXT("saddle builds within the existing query tolerance"),
		Surface.BuildBilinearApproximation(TensorBezierQueryApproximationToleranceCm,
			Cells, &MaximumError))) return false;
	TestTrue(TEXT("nonplanar bilinear surface is not falsely a zero-error cell"),
		Cells.Num() > 1);
	for (const FTensorBezierApproximationCell& Cell : Cells)
	{
		const double U = 0.5 * (Cell.MinimumU + Cell.MaximumU);
		const double V = 0.5 * (Cell.MinimumV + Cell.MaximumV);
		const FVector3d TriangleDiagonalMidpoint = 0.5 * (Cell.Corners[0] + Cell.Corners[3]);
		const double VerticalGap = FMath::Abs(Surface.Evaluate(U, V).Z - TriangleDiagonalMidpoint.Z);
		// These saddle coordinates and half subdivisions are exact dyadics.
		TestTrue(TEXT("reported cell bound covers the actual triangle deviation"),
			VerticalGap <= Cell.MaximumErrorCm);
	}
	TestTrue(TEXT("total bound respects the unchanged tolerance"),
		MaximumError <= TensorBezierQueryApproximationToleranceCm);
	Surface.ControlPoints.Last().Z = 0.0;
	TestTrue(TEXT("planar control still builds"),
		Surface.BuildBilinearApproximation(TensorBezierQueryApproximationToleranceCm,
			Cells, &MaximumError));
	TestEqual(TEXT("planar control needs one cell"), Cells.Num(), 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticHitErrorBoundRoundingTest,
	"IAmSpeed.AnalyticWorld.HitErrorBoundRounding",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FAnalyticHitErrorBoundRoundingTest::RunTest(const FString&)
{
	Speed::Analytic::FWorldHit Hit;
	Hit.bHit = true;
	const auto Convert = [&Hit](double Value)
	{
		Hit.GeometricErrorBoundCm = Value;
		return Speed::SHitResult::FromAnalyticHit(Hit, 0.0f, nullptr, 0).GeometricErrorBoundCm;
	};
	TestEqual(TEXT("exact planes retain zero error"), Convert(0.0), 0.0f);
	const uint64 AboveHalfBits = 0x3fe0000000000001ull;
	double AboveHalf = 0.0;
	FMemory::Memcpy(&AboveHalf, &AboveHalfBits, sizeof(AboveHalf));
	TestTrue(TEXT("a down-rounded cast remains a majorant"),
		static_cast<double>(Convert(AboveHalf)) >= AboveHalf);
	TestEqual(TEXT("exact representable normal stays unchanged"), Convert(0.5), 0.5f);
	const uint64 TinyBits = 1;
	double Tiny = 0.0;
	FMemory::Memcpy(&Tiny, &TinyBits, sizeof(Tiny));
	const float TinyBound = Convert(Tiny);
	uint32 TinyBoundBits = 0;
	FMemory::Memcpy(&TinyBoundBits, &TinyBound, sizeof(TinyBoundBits));
	TestTrue(TEXT("positive tiny errors retain a normal bound even with FTZ/DAZ"),
		TinyBoundBits >= 0x00800000u && TinyBoundBits < 0x7f800000u);
	return true;
}

namespace
{
	int32 GStadiumMathRuns = 0, GStadiumMathPassed = 0;
	int32 GStadiumMathErrors = -1, GStadiumMathWarnings = -1;
	int32 GStadiumTriangulationPassed = -1, GStadiumRoundingPassed = -1;
	FAutoConsoleVariableRef StadiumMathRuns(TEXT("Speed.Analytic.MathUnit.Runs"), GStadiumMathRuns, TEXT("Actual isolated math test runs."), ECVF_ReadOnly);
	FAutoConsoleVariableRef StadiumMathPassed(TEXT("Speed.Analytic.MathUnit.Passed"), GStadiumMathPassed, TEXT("Isolated math test result."), ECVF_ReadOnly);
	FAutoConsoleVariableRef StadiumMathErrors(TEXT("Speed.Analytic.MathUnit.Errors"), GStadiumMathErrors, TEXT("Actual math test errors."), ECVF_ReadOnly);
	FAutoConsoleVariableRef StadiumMathWarnings(TEXT("Speed.Analytic.MathUnit.Warnings"), GStadiumMathWarnings, TEXT("Actual math test warnings."), ECVF_ReadOnly);
	FAutoConsoleVariableRef StadiumTriangulationPassed(TEXT("Speed.Analytic.MathUnit.TriangulationPassed"), GStadiumTriangulationPassed, TEXT("Existing triangulation test result."), ECVF_ReadOnly);
	FAutoConsoleVariableRef StadiumRoundingPassed(TEXT("Speed.Analytic.MathUnit.RoundingPassed"), GStadiumRoundingPassed, TEXT("Hit-bound conversion test result."), ECVF_ReadOnly);
	FAutoConsoleCommand StadiumMathCommand(TEXT("Speed.Analytic.RunMathUnits"),
		TEXT("Run two existing world-free numerical tests once in a guarded commandlet."),
		FConsoleCommandDelegate::CreateLambda([]()
		{
			FAutomationTestFramework& Framework = FAutomationTestFramework::Get();
			if (!IsRunningCommandlet() || !FParse::Param(FCommandLine::Get(), TEXT("StadiumTensorMathUnit")) ||
				GIsAutomationTesting || GIsSlowTask || GIsPlayInEditorWorld || GStadiumMathRuns != 0 ||
				!Framework.IsLatentCommandQueueEmpty())
			{
				UE_LOG(LogTemp, Error, TEXT("Stadium math unit entry refused."));
				return;
			}
			const TCHAR* const Names[] = {TEXT("IAmSpeed.AnalyticWorld.TensorTriangulationBound"),
				TEXT("IAmSpeed.AnalyticWorld.HitErrorBoundRounding")};
			// Automation registers the C++ class name; Names are the public report paths.
			const TCHAR* const Commands[] = {TEXT("FIAmSpeedTensorTriangulationBoundTest"),
				TEXT("FAnalyticHitErrorBoundRoundingTest")};
			constexpr int32 NumTests = static_cast<int32>(UE_ARRAY_COUNT(Names));
			static_assert(UE_ARRAY_COUNT(Commands) == UE_ARRAY_COUNT(Names));
			GStadiumMathErrors = 0; GStadiumMathWarnings = 0;
			bool bAllPassed = true;
			for (int32 Index = 0; Index < NumTests; ++Index)
			{
				if (GIsAutomationTesting || GIsSlowTask || GIsPlayInEditorWorld || Framework.GetCurrentTest() ||
					!Framework.ContainsTest(Commands[Index]) || !Framework.IsLatentCommandQueueEmpty())
				{
					UE_LOG(LogTemp, Error, TEXT("Stadium math test registration or empty queue required."));
					return;
				}
				Framework.SetRequestedTestFilter(EAutomationTestFlags::EngineFilter);
				Framework.StartTestByName(Commands[Index], 0, Names[Index]);
				if (!GIsAutomationTesting || !Framework.GetCurrentTest() || Framework.GetCurrentTestFullPath() != Names[Index])
				{
					UE_LOG(LogTemp, Error, TEXT("Stadium math test did not start as requested."));
					return;
				}
				const bool bSynchronous = Framework.IsLatentCommandQueueEmpty();
				if (!bSynchronous)
				{
					UE_LOG(LogTemp, Error, TEXT("Unexpected latent command; isolated math invocation refused without draining any queue."));
					return;
				}
				FAutomationTestExecutionInfo Info;
				const bool bStoppedSuccessfully = Framework.StopTest(Info);
				const bool bPassed = bSynchronous && bStoppedSuccessfully && Info.bSuccessful && Info.GetErrorTotal() == 0 && Info.GetWarningTotal() == 0;
				++GStadiumMathRuns; GStadiumMathErrors += Info.GetErrorTotal(); GStadiumMathWarnings += Info.GetWarningTotal();
				(Index == 0 ? GStadiumTriangulationPassed : GStadiumRoundingPassed) = bPassed;
				bAllPassed = bAllPassed && bPassed;
				UE_LOG(LogTemp, Display, TEXT("STADIUM_MATH_UNIT name=%s passed=%d errors=%d warnings=%d"), Names[Index], bPassed, Info.GetErrorTotal(), Info.GetWarningTotal());
			}
			GStadiumMathPassed = bAllPassed && GStadiumMathRuns == NumTests;
		}));
}

#include "TensorDistanceBounds.h"
#include "IAmSpeed/World/Simulation/FrameFloatingPointContext.h"
namespace
{
struct FTensorBoundsTestContext
{
    Speed::FFrameFloatingPointContext Original=Speed::CaptureFrameFloatingPointContext();
    Speed::FScopedFrameFloatingPointContext Restore{Original};
    std::fenv_t Held;
    bool Ready=Original.bValid && Restore.IsApplied() && std::feholdexcept(&Held)==0 && std::fesetround(FE_TONEAREST)==0;
};
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticIntervalEnclosureTest,
    "IAmSpeed.AnalyticWorld.TensorDistance.IntervalEnclosure",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FAnalyticIntervalEnclosureTest::RunTest(const FString&)
{
    FTensorBoundsTestContext Context;if (!TestTrue(TEXT("RNE fixture established"),Context.Ready))return false;
    using namespace AnalyticCertified;
    const uint32 OriginalVector=VectorGetControlRegister();
    for (uint32 Flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((OriginalVector&~uint32(0x8040))|Flush);
        const auto sum=Add(Interval::Exact(1.25),Interval::Exact(0.5));
        const auto product=Multiply(Interval::Exact(1.25),Interval::Exact(0.5));
        TestTrue(TEXT("exact dyadic sum enclosed in every flush mode"),Valid(sum)&&sum.Lower<=1.75&&sum.Upper>=1.75);
        TestTrue(TEXT("exact dyadic product enclosed in every flush mode"),Valid(product)&&product.Lower<=0.625&&product.Upper>=0.625);
        const auto tiny=FromBits(1),large=FromBits(0x7fe0000000000000ULL);
        const auto enclosed=Multiply(Interval::Exact(tiny),Interval::Exact(large));
        const double exactProduct=FromBits(0x3cc0000000000000ULL); // 2^-51 exactly
        TestTrue(TEXT("DAZ input enclosure retains exact tiny times huge product"),Valid(enclosed)&&enclosed.Lower<=exactProduct&&enclosed.Upper>=exactProduct);
        const auto forged=Multiply(Interval{tiny,tiny,true},Interval::Exact(large));
        TestFalse(TEXT("forged subnormal interval never bypasses invariant under DAZ"),forged.Known);
        TestFalse(TEXT("inverted interval is unavailable"),Add(Interval{2.,1.,true},Interval::Exact(0.)).Known);
        TestFalse(TEXT("overflow stays unknown"),Add(Interval::Exact(FromBits(0x7fefffffffffffffULL)),Interval::Exact(FromBits(0x7fefffffffffffffULL))).Known);
        const auto square=Square(Interval{-2.,3.,true});
        TestTrue(TEXT("square crossing zero encloses exact range zero to nine"),Valid(square)&&square.Lower==0.&&square.Upper>=9.);
        const auto half=HalfSum(Interval::Exact(-1.),Interval::Exact(3.));
        TestTrue(TEXT("half-sum keeps exact dyadic midpoint"),Valid(half)&&half.Lower<=1.&&half.Upper>=1.);
    }
    TestTrue(TEXT("original floating context restored"),Context.Restore.Restore());
    return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticTensorDistanceBoundsTest,
    "IAmSpeed.AnalyticWorld.TensorDistance.PolynomialBounds",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FAnalyticTensorDistanceBoundsTest::RunTest(const FString&)
{
    FTensorBoundsTestContext Context;if (!TestTrue(TEXT("RNE fixture established"),Context.Ready))return false;
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    const auto encloses=[](const FTensorDistanceBounds& r,double truth){return r.Status!=EDistanceBoundStatus::Unknown&&r.LowerSquared<=truth&&truth<=r.UpperSquared;};
    FTensorBezierSurface constant;constant.ControlPoints.Add(FVector3d(3,4,0));
    FDistanceBudget constantBudget{1};auto c=BoundTensorDistanceSquared(constant,FVector3d::ZeroVector,constantBudget,1.e-8);
    TestTrue(TEXT("constant polynomial accepted without regular-surface validation, exact distance squared25"),encloses(c,25.)&&c.Status==EDistanceBoundStatus::Converged);
    TestEqual(TEXT("one root consumes one shared node"),constantBudget.Used,uint64(1));
    FTensorBezierSurface line;line.DegreeU=1;line.ControlPoints={FVector3d(-1,0,0),FVector3d(3,0,0)};
    FDistanceBudget splitBudget{4};auto r=BoundTensorDistanceSquared(line,FVector3d::ZeroVector,splitBudget,1.e-8);
    TestTrue(TEXT("left child contains zero; global minimum must not use right-child lower one"),encloses(r,0.)&&r.Status==EDistanceBoundStatus::Converged);
    TestEqual(TEXT("actual witness has dyadic u=one quarter"),r.SampleUV.X,0.25);
    TestEqual(TEXT("root plus paid KKT predicate and two children consume exactly4 shared nodes"),splitBudget.Used,uint64(4));
    FTensorBezierSurface saddle;saddle.DegreeU=saddle.DegreeV=1;
    saddle.ControlPoints={FVector3d(0,0,0),FVector3d(0,1,0),FVector3d(1,0,0),FVector3d(1,1,1)};
    FDistanceBudget saddleBudget{1};auto s=BoundTensorDistanceSquared(saddle,FVector3d(0.5,0.5,0.25),saddleBudget,1.e-8);
    TestTrue(TEXT("real bilinear center atz=.25 is retained despite coarse diagonal atz=.5"),encloses(s,0.)&&s.Status==EDistanceBoundStatus::Converged);
    FTensorBezierSurface plane;plane.DegreeU=plane.DegreeV=1;
    plane.ControlPoints={FVector3d(-1,-1,0),FVector3d(-1,1,0),FVector3d(1,-1,0),FVector3d(1,1,0)};
    FDistanceBudget planeBudget{9};auto p=BoundTensorDistanceSquared(plane,FVector3d(4,0,2),planeBudget,1.e-8);
    TestTrue(TEXT("finite plane nearest boundary atx=1 has exact distance squared13"),encloses(p,13.));
    TestTrue(TEXT("shared node use never exceeds caller budget"),planeBudget.Used<=9);
    TestTrue(TEXT("original floating context restored"),Context.Restore.Restore());
    return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticTensorDistanceRefusalTest,
    "IAmSpeed.AnalyticWorld.TensorDistance.RefusalAndBudget",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FAnalyticTensorDistanceRefusalTest::RunTest(const FString&)
{
    FTensorBoundsTestContext Context;if (!TestTrue(TEXT("RNE fixture established"),Context.Ready))return false;
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    FTensorBezierSurface surface;surface.ControlPoints.Add(FVector3d(1,0,0));
    FDistanceBudget empty{0};TestTrue(TEXT("no remaining budget is Unknown, never miss"),BoundTensorDistanceSquared(surface,FVector3d::ZeroVector,empty,1.e-8).Status==EDistanceBoundStatus::Unknown);
    FDistanceBudget shared{1};auto first=BoundTensorDistanceSquared(surface,FVector3d::ZeroVector,shared,1.e-8);auto second=BoundTensorDistanceSquared(surface,FVector3d::ZeroVector,shared,1.e-8);
    TestTrue(TEXT("one caller budget shared across separate providers, no reset"),first.Status!=EDistanceBoundStatus::Unknown&&second.Status==EDistanceBoundStatus::Unknown&&shared.Used==1);
    VectorSetControlRegister(VectorGetControlRegister()|0x40u);
    FDistanceBudget invalid{3};TestTrue(TEXT("negative subnormal tolerance refused by bits under DAZ"),BoundTensorDistanceSquared(surface,FVector3d::ZeroVector,invalid,AnalyticCertified::FromBits(0x8000000000000001ULL)).Status==EDistanceBoundStatus::Unknown&&invalid.Used==0);
    FDistanceBudget overflow{3,MAX_uint64};TestTrue(TEXT("forged counter cannot wrap"),BoundTensorDistanceSquared(surface,FVector3d::ZeroVector,overflow,1.e-8).Status==EDistanceBoundStatus::Unknown);
    surface.ControlPoints[0].X=AnalyticCertified::FromBits(0x7ff8000000000001ULL);
    FDistanceBudget badNet{3};TestTrue(TEXT("nonfinite controls do not produce a witness"),BoundTensorDistanceSquared(surface,FVector3d::ZeroVector,badNet,1.e-8).Status==EDistanceBoundStatus::Unknown);
    surface.ControlPoints[0].X=1.;FDistanceBudget wrongMode{3};bool refused=false;
    if (!TestEqual(TEXT("controlled non-nearest mode established"),std::fesetround(FE_UPWARD),0))return false;
    try{(void)BoundTensorDistanceSquared(surface,FVector3d::ZeroVector,wrongMode,1.e-8);}catch(const FAnalyticNumericUnknown&){refused=true;}
    std::fesetround(FE_TONEAREST);
    TestTrue(TEXT("mandatory guard refuses nonnearest even with no world contract or actor"),refused&&wrongMode.Used==0);
    TestTrue(TEXT("original floating context restored"),Context.Restore.Restore());
    return !HasAnyErrors();
}
namespace
{
FAutoConsoleCommand TensorDistanceUnits(TEXT("Speed.Analytic.RunTensorDistanceUnits"),
    TEXT("Run three existing-framework tensor distance bound tests in an isolated commandlet."),
    FConsoleCommandDelegate::CreateLambda([]()
    {
        static bool Used=false;
        auto& Framework=FAutomationTestFramework::Get();
        if (!IsRunningCommandlet() || !FParse::Param(FCommandLine::Get(),TEXT("AnalyticTensorDistanceUnits")) ||
            Used || GIsAutomationTesting || GIsSlowTask || GIsPlayInEditorWorld ||
            Framework.GetCurrentTest() || !Framework.IsLatentCommandQueueEmpty())
        { UE_LOG(LogTemp,Error,TEXT("Analytic tensor bound unit commandlet refused")); return; }
        Used=true;
        const TCHAR* Classes[]={TEXT("FAnalyticIntervalEnclosureTest"),TEXT("FAnalyticTensorDistanceBoundsTest"),TEXT("FAnalyticTensorDistanceRefusalTest")};
        const TCHAR* Names[]={TEXT("IAmSpeed.AnalyticWorld.TensorDistance.IntervalEnclosure"),TEXT("IAmSpeed.AnalyticWorld.TensorDistance.PolynomialBounds"),TEXT("IAmSpeed.AnalyticWorld.TensorDistance.RefusalAndBudget")};
        for (int32 I=0; I<3; ++I)
        {
            if (GIsAutomationTesting || GIsSlowTask || GIsPlayInEditorWorld || Framework.GetCurrentTest() ||
                !Framework.IsLatentCommandQueueEmpty() || !Framework.ContainsTest(Classes[I]))
            { UE_LOG(LogTemp,Error,TEXT("Analytic tensor bound unit ownership refused")); return; }
            Framework.SetRequestedTestFilter(EAutomationTestFlags::EngineFilter);
            Framework.StartTestByName(Classes[I],0,Names[I]);
            if (!GIsAutomationTesting || !Framework.GetCurrentTest() || Framework.GetCurrentTestFullPath()!=Names[I] || !Framework.IsLatentCommandQueueEmpty())
            { UE_LOG(LogTemp,Error,TEXT("Analytic tensor bound unit synchronous ownership refused")); return; }
            FAutomationTestExecutionInfo Info;
            const bool Stopped=Framework.StopTest(Info);
            const bool Passed=Stopped && Info.bSuccessful && Info.GetErrorTotal()==0 && Info.GetWarningTotal()==0;
            UE_LOG(LogTemp,Display,TEXT("ANALYTIC_TENSOR_BOUND_UNIT name=%s passed=%d errors=%d warnings=%d"),Names[I],Passed,Info.GetErrorTotal(),Info.GetWarningTotal());
        }
    }));
}

#include "TensorSphereTimeTests.inl"
#include "TensorSphereContactTests.inl"
#include "TensorIntervalInputTests.inl"
#include "TensorProjectedDistanceTests.inl"
#include "KinematicCenterPathTests.inl"
#include "FloorNativeSolidSweepTests.inl"
#include "BoxPoseCorridorTests.inl"
#include "GroupedBoxVolumeTests.inl"
#include "InitialFloorSphereGeometryTests.inl"
#include "NativeAnalyticContactValuesTests.inl"
#include "IndexedIntervalUnionTests.inl"
#include "TensorContactLeavesTests.inl"
#include "ScalarSphereContactTests.inl"
#include "ConsumedSecondsTests.inl"
#include "RuntimeSphereCaptureTests.inl"
#include "StrictAuthorityRefusalTests.inl"
#include "PreparedPhysicalFilterDomainTests.inl"
#include "PreparedPhysicalExecutionTests.inl"
#include "ReferenceSphereConsumerTests.inl"
#include "SharedNativeSolidUnits.inl"

namespace
{
Speed::Analytic::Certified::FIntervalTensorSurface PlaneUV95(bool curved=false)
{
    namespace C=Speed::Analytic::Certified;C::FIntervalTensorSurface s;
    s.DegreeU=curved?2:1;s.DegreeV=1;
    for(int32 i=0;i<=s.DegreeU;++i)for(int32 j=0;j<=1;++j)
    {
        const double u=double(i)/s.DegreeU,v=double(j);
        s.ControlPoints.Add(C::Detail::Point(curved?FVector3d(4*u,4*v,i==1?8.:0.):FVector3d(8*u,8*v,8*u+4*v)));
    }
    return s;
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDirectPlaneUVProposal4495Test,
    "IAmSpeed.AnalyticWorld.TensorDistance.DirectPlaneUVProposal4495",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FDirectPlaneUVProposal4495Test::RunTest(const FString&)
{
    namespace C=Speed::Analytic::Certified;namespace Q=C::PlaneProposal95;namespace A=AnalyticCertified;
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("numeric fixture"),context.Ready))return false;
    const uint32 originalVector=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((originalVector&~uint32(0x8040))|flush);
        auto s=PlaneUV95();const auto center=C::Detail::Point(FVector3d(1.75,5.875,5.25));
        C::Detail::FNode off;off.DegreeU=1;off.DegreeV=1;for(const auto& p:s.ControlPoints)off.Net.Add(p);
        auto on=off;C::FDistanceBudget a{8192},b{8192};a.Consume();b.Consume();
        Q::FWork wa{&s,&a,0,8192,0},wb{&s,&b,0,8192,0};
        {Q::FScopedTestEnable disabled(false);if(!TestTrue(TEXT("original Bounds"),C::Detail::Bounds(off,center,nullptr,nullptr,&wa)))return false;}
        {Q::FScopedTestEnable enabled(true);if(!TestTrue(TEXT("certified proposed Bounds"),C::Detail::Bounds(on,center,nullptr,nullptr,&wb)))return false;}
        TestTrue(TEXT("interior projection improves far midpoint/corners"),on.UpperSquared<off.UpperSquared*.1&&on.WitnessUV.X>0&&on.WitnessUV.X<1&&on.WitnessUV.Y>0&&on.WitnessUV.Y<1);
        // Signed distance to z=x+y/2: (9/16)^2/(9/4)=9/64.
        TestTrue(TEXT("analytic exact distance enclosed"),on.LowerSquared<=.140625&&on.UpperSquared>=.140625&&on.LowerSquared>=off.LowerSquared);
        C::FIntervalPoint same;
        TestTrue(TEXT("sample is the original full tensor at stored UV"),C::ProjectedDistanceDetail::EvaluateFixedUV(s,on.WitnessUV,same));
        for(int32 d=0;d<3;++d)TestTrue(TEXT("same fixed-UV point bits"),A::Bits(same.Axis[d].Lower)==A::Bits(on.WitnessPoint.Axis[d].Lower)&&A::Bits(same.Axis[d].Upper)==A::Bits(on.WitnessPoint.Axis[d].Upper));
        Q::FScopedTestEnable enabled(true);const auto threshold=A::Interval::Exact(.25);
        C::FDistanceBudget one{8192},retained{8192};C::Detail::FMinimumMidpointMemo memo;
        const auto direct=C::BoundTensorDistanceSquaredIntervalCenter(s,center,one,0,8192,&threshold,&memo);
        C::FRetainedTensorMinimum minimum(s,center,retained,0,8192,&threshold,&memo);const auto held=minimum.Advance();
        TestTrue(TEXT("both real time evaluator routes receive the proposal"),direct.Status!=C::EDistanceBoundStatus::Unknown&&held.Status!=C::EDistanceBoundStatus::Unknown&&direct.UpperSquared<.25&&held.UpperSquared<.25&&minimum.GetCountersForInspection().UVSplits==0);
        // Singleton equality is not needed: every coefficient can vary in z.
        for(auto& p:s.ControlPoints){p.Axis[2].Lower-=.125;p.Axis[2].Upper+=.125;}
        C::FDistanceBudget boxes{8192};const auto wide=C::BoundTensorDistanceSquaredIntervalCenter(s,center,boxes,1.,8192);
        TestTrue(TEXT("non-singleton coefficients retain universal fixed-net witness"),wide.Status!=C::EDistanceBoundStatus::Unknown&&wide.UpperSquared>=.140625&&wide.SampleUV.X>=0&&wide.SampleUV.X<=1);
    }
    TestTrue(TEXT("numeric context restored"),context.Restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDirectPlaneUVMixedAndContinuous4497Test,
    "IAmSpeed.AnalyticWorld.TensorDistance.DirectPlaneUVMixedAndContinuous4497",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FDirectPlaneUVMixedAndContinuous4497Test::RunTest(const FString&)
{
    namespace C=Speed::Analytic::Certified;namespace Q=C::PlaneProposal95;namespace A=AnalyticCertified;
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("numeric fixture"),context.Ready))return false;
    Q::FScopedTestEnable enabled(true);
    auto curve=PlaneUV95(true);
    // Force an improving INTERIOR proposal. An original corner witness is
    // already exact and is not required to have the same rounded enclosure as
    // a later full de Casteljau evaluation at that corner.
    const FVector3d centerValue(1,2,3.125);
    C::FDistanceBudget work{8192};const auto center=C::Detail::Point(centerValue);
    const auto witness=C::BoundTensorDistanceSquaredIntervalCenter(curve,center,work,16.,8192);
    C::FIntervalPoint realPoint;
    if(!TestTrue(TEXT("curved provider remains a bounded original-net problem"),witness.Status!=C::EDistanceBoundStatus::Unknown&&C::ProjectedDistanceDetail::EvaluateFixedUV(curve,witness.SampleUV,realPoint)))return false;
    TestTrue(TEXT("actual interior proposal was selected, not a retained original corner"),witness.SampleUV.X==.25&&witness.SampleUV.Y==.5);
    const double exactPoint[3]={1,2,3};
    for(int32 d=0;d<3;++d)
    {
        TestTrue(TEXT("selected original-net evaluation retains its exact interval bits"),A::Bits(realPoint.Axis[d].Lower)==A::Bits(witness.SamplePoint.Axis[d].Lower)&&A::Bits(realPoint.Axis[d].Upper)==A::Bits(witness.SamplePoint.Axis[d].Upper));
        TestTrue(TEXT("independent true curved point, including z3 instead of fake planar z0"),witness.SamplePoint.Axis[d].Lower<=exactPoint[d]&&witness.SamplePoint.Axis[d].Upper>=exactPoint[d]);
    }
    TestTrue(TEXT("independent squared distance one sixty-fourth enclosed"),witness.LowerSquared<=.015625&&witness.UpperSquared>=.015625&&witness.UpperSquared<.02);
    for(int32 i=0;i<=32;++i)for(int32 j=0;j<=8;++j)
    {
        const double u=double(i)/32,v=double(j)/8;
        const FVector3d p(4*u,4*v,16*u*(1-u));
        TestTrue(TEXT("independent curved samples obey certified lower"),witness.LowerSquared<=(p-centerValue).SizeSquared());
    }
    C::FIntervalTensorSurface plane;plane.DegreeU=plane.DegreeV=1;
    for(double x:{-8.,8.})for(double y:{-8.,8.})plane.ControlPoints.Add(C::Detail::Point(FVector3d(x,y,0)));
    TArray<C::FIntervalTensorSurface> mixed{curve,plane};
    const auto observe=[](const TCHAR* label,bool on,const C::FTensorSphereContactEnclosure& r,const C::FDistanceBudget& b)
    {
        UE_LOG(LogTemp,Display,TEXT("[ProposalMixedContract97] Case=%s On=%d Status=%u Failure=%u TimeFailure=%u WorkSite=%s Used=%llu Remaining=%d Lower=%.17g Upper=%.17g Spatial=%llu"),
            label,on?1:0,uint32(r.Status),uint32(r.Failure),uint32(r.TimeFailure),r.WorkSite,
            static_cast<unsigned long long>(b.Used),b.Remaining,r.LowerTime,r.UpperTime,static_cast<unsigned long long>(r.SpatialVisitedCells));
    };
    // Keep the failed coarse fixture as a NEW off/on discriminant. Unknown is
    // not a miss. These observations do not qualify the fine contact below.
    for(bool on:{false,true})
    {
        Q::FScopedTestEnable selected(on);C::FDistanceBudget coarse{1048576};
        const auto r=C::BoundTensorSphereFirstContact(MakeArrayView(mixed),FVector3d(2,2,6),FVector3d(2,2,-2),.25,coarse,1./128.,0,8192,.02,.02);
        observe(TEXT("mixed-original-coarse"),on,r,coarse);
        TestTrue(TEXT("coarse uncertainty never becomes a false geometric miss"),r.Status!=C::ETensorSphereContactStatus::NoContactWithProvidedTensors);
    }
    for(bool on:{false,true})
    {
        Q::FScopedTestEnable selected(on);C::FDistanceBudget contactWork{1048576};
        // TIGHTER time bracket; all original spatial/point/normal qualities
        // and every per-query work allowance remain unchanged.
        const auto r=C::BoundTensorSphereFirstContact(MakeArrayView(mixed),FVector3d(2,2,6),FVector3d(2,2,-2),.25,contactWork,1./65536.,0,8192,.02,.02);
        observe(TEXT("mixed-fine"),on,r,contactWork);
        if(TestTrue(TEXT("off and on both enclose the earlier real curved contact"),r.Status==C::ETensorSphereContactStatus::FirstContactEnclosed&&!r.Leaves.IsEmpty()))
        {
            TestTrue(TEXT("exact first contact seven thirty-seconds enclosed before the plane"),r.LowerTime<=.21875&&r.UpperTime>=.21875&&r.UpperTime<.5);
            for(const auto& leaf:r.Leaves)TestTrue(TEXT("only the earlier curved provider owns possible first contact"),leaf.SurfaceIndex==0);
        }
    }
    C::FDistanceBudget insideWork{65536};const auto inside=C::BoundTensorSphereFirstContact(MakeArrayView(&plane,1),FVector3d(0,0,.125),FVector3d(0,0,-2),.25,insideWork,1./64.,0,8192,.02,.02);
    TestTrue(TEXT("initial overlap is not fabricated FirstContact"),inside.Status==C::ETensorSphereContactStatus::InitialInsideTube);
    C::FDistanceBudget tangentWork{65536};const auto tangent=C::BoundTensorSphereFirstContact(MakeArrayView(&plane,1),FVector3d(0,0,.25),FVector3d(2,0,.25),.25,tangentWork,1./64.,0,8192,.02,.02);
    TestTrue(TEXT("threshold tangency does not become a strict hit or miss"),tangent.Status==C::ETensorSphereContactStatus::Unknown);
    C::FIntervalBezierCenterPath pva;pva.Degree=2;pva.ControlPoints={C::Detail::Point(FVector3d(0,0,2)),C::Detail::Point(FVector3d(0,0,-3)),C::Detail::Point(FVector3d(0,0,2))};
    for(bool on:{false,true})
    {
        Q::FScopedTestEnable selected(on);C::FDistanceBudget coarse{65536};
        const auto r=C::BoundTensorSphereFirstContact(MakeArrayView(&plane,1),pva,.25,coarse,1./64.,0,8192,.02,.02);
        observe(TEXT("quadratic-original-coarse"),on,r,coarse);
        TestTrue(TEXT("returning quadratic cannot acquire a false chord miss"),r.Status!=C::ETensorSphereContactStatus::NoContactWithProvidedTensors);
    }
    for(bool on:{false,true})
    {
        Q::FScopedTestEnable selected(on);C::FDistanceBudget pvaWork{65536};
        const auto r=C::BoundTensorSphereFirstContact(MakeArrayView(&plane,1),pva,.25,pvaWork,1./65536.,0,8192,.02,.02);
        observe(TEXT("quadratic-fine"),on,r,pvaWork);
        if(TestTrue(TEXT("off and on both retain continuous quadratic contact"),r.Status==C::ETensorSphereContactStatus::FirstContactEnclosed&&r.UpperTime<.5))
        {
            const auto polynomial=[](double t)
            {
                const auto x=A::Interval::Exact(t);
                return A::Add(A::Subtract(A::Multiply(A::Interval::Exact(80),A::Square(x)),A::Multiply(A::Interval::Exact(80),x)),A::Interval::Exact(14));
            };
            const auto lo=polynomial(r.LowerTime),hi=polynomial(r.UpperTime);
            TestTrue(TEXT("exact polynomial 80t squared minus80t plus14 changes sign across first root"),A::Valid(lo)&&A::Valid(hi)&&lo.Lower>=0&&hi.Upper<=0);
        }
    }
    TestTrue(TEXT("numeric context restored"),context.Restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDirectPlaneUVRefusalAndBudget4495Test,
    "IAmSpeed.AnalyticWorld.TensorDistance.DirectPlaneUVRefusalAndBudget4495",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FDirectPlaneUVRefusalAndBudget4495Test::RunTest(const FString&)
{
    namespace C=Speed::Analytic::Certified;namespace Q=C::PlaneProposal95;
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("numeric fixture"),context.Ready))return false;
    Q::FScopedTestEnable enabled(true);auto s=PlaneUV95();const auto center=C::Detail::Point(FVector3d(1.75,5.875,5.25));
    C::FDistanceBudget shortWork{4};C::FRetainedTensorMinimum shortMinimum(s,center,shortWork,0,8192);
    TestTrue(TEXT("committed global debit failure is terminal"),shortMinimum.Advance().Status==C::EDistanceBoundStatus::Unknown&&shortMinimum.IsTerminalForInspection()&&shortMinimum.GetFailureForInspection()==C::ERetainedTensorMinimumFailure::Work&&shortWork.Used==4);
    const uint64 used=shortWork.Used;TestTrue(TEXT("no retry/refund after work failure"),shortMinimum.Advance().Status==C::EDistanceBoundStatus::Unknown&&shortWork.Used==used);
    C::FDistanceBudget floorWork{64};floorWork.ConsumptionFloor=63;
    C::FRetainedTensorMinimum floorMinimum(s,center,floorWork,0,8192);
    TestTrue(TEXT("literal floor preserved"),floorMinimum.Advance().Status==C::EDistanceBoundStatus::Unknown&&floorWork.Remaining==63&&floorWork.ConsumptionFloor==63);
    C::FDistanceBudget coarseWork{8192};const auto coarse=C::BoundTensorDistanceSquaredIntervalCenter(s,center,coarseWork,0,1);
    TestTrue(TEXT("preclassified one-node local scope stays original"),coarse.Status!=C::EDistanceBoundStatus::Unknown&&coarseWork.Used==1);
    auto line=s;for(auto& p:line.ControlPoints)p=C::Detail::Point(FVector3d(1,1,1));
    C::FDistanceBudget degenerate{8192};const auto deg=C::BoundTensorDistanceSquaredIntervalCenter(line,C::Detail::Point(FVector3d(3,3,3)),degenerate,100.,8192);
    TestTrue(TEXT("degenerate inverse skips with admission retained"),deg.Status!=C::EDistanceBoundStatus::Unknown&&degenerate.Used==2);
    // Boundary UV is permitted as a distance witness, not native face authority.
    C::FDistanceBudget edgeWork{8192};const auto edge=C::BoundTensorDistanceSquaredIntervalCenter(s,C::Detail::Point(FVector3d(-1,-1,-1)),edgeWork,100.,8192);
    TestTrue(TEXT("boundary witness remains in closed parameter square"),edge.Status!=C::EDistanceBoundStatus::Unknown&&edge.SampleUV.X>=0&&edge.SampleUV.X<=1&&edge.SampleUV.Y>=0&&edge.SampleUV.Y<=1);
    C::FDistanceBudget forged{8192,MAX_uint64};TestTrue(TEXT("counter overflow never wraps"),C::BoundTensorDistanceSquaredIntervalCenter(s,center,forged,0,8192).Status==C::EDistanceBoundStatus::Unknown);
    TestTrue(TEXT("numeric context restored"),context.Restore.Restore());return !HasAnyErrors();
}

// Append in the existing TensorSphereContactTests.inl translation unit.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticCurveStationaryDomain98Test,
    "IAmSpeed.AnalyticWorld.TensorContact.CurveStationaryDomain98",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticCurveStationaryDomain98Test::RunTest(const FString&)
{
    using namespace Speed::Analytic::Certified;
    namespace A=::AnalyticCertified;
    using E=ContactDetail::ECurveStationaryDomain98;
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture"),context.Ready))return false;
    // These are mathematical STRUCTURE fixtures, not a stadium admission.
    // Position and original derivative are SAME-cell restrictions with
    // original V=[1/4,1/2], so local w has dv/dw=1/4.
    const auto line=[]()
    {
        Detail::FNode n;n.DegreeU=0;n.DegreeV=1;n.DepthV=2;n.MinV=.25;n.MaxV=.5;
        n.Net.Add(Detail::Point(FVector3d(-1,0,0)));n.Net.Add(Detail::Point(FVector3d(1,0,0)));return n;
    };
    const auto lineDv=[]()
    {
        Detail::FNode d;d.DegreeU=d.DegreeV=0;d.DepthV=2;d.MinV=.25;d.MaxV=.5;
        d.Net.Add(Detail::Point(FVector3d(8,0,0)));return d;
    };
    auto n=line(),d=lineDv();FDistanceBudget budget{100000};Interval domain;
    auto c=Detail::Point(FVector3d(.5,1,0));
    TestTrue(TEXT("off-center interior root has necessary domain"),ContactDetail::CurveStationaryDomain98(n,d,c,budget,domain)==E::Proved);
    TestTrue(TEXT("true local root .75 retained; contracts to right child"),A::Valid(domain)&&domain.Lower<=.75&&domain.Upper>=.75&&domain.Lower>.5&&domain.Upper<1);
    c.Axis[0]={.4,.6,true};
    TestTrue(TEXT("center box retained"),ContactDetail::CurveStationaryDomain98(n,d,c,budget,domain)==E::Proved);
    TestTrue(TEXT("all center realizations .7 to .8 retained"),domain.Lower<=.7&&domain.Upper>=.8&&domain.Lower>.5);
    c=Detail::Point(FVector3d(3,1,0));
    TestTrue(TEXT("all stationary roots outside interior cell excludes it"),ContactDetail::CurveStationaryDomain98(n,d,c,budget,domain)==E::Excluded);
    auto endpoint=n;endpoint.MinV=0;
    TestTrue(TEXT("original endpoint is not a stationary-root theorem"),ContactDetail::CurveStationaryDomain98(endpoint,d,c,budget,domain)==E::NotApplicable);
    // Nonzero curvature: S=(2w-1,(2w-1)^2/4,0), represented by degree2.
    // D_originalV=(8,8w-4,0), h=.25. At C=(0,-1,0), the unique root is .5;
    // derivative g' is strictly positive over the whole cell.
    Detail::FNode bowl;bowl.DegreeU=0;bowl.DegreeV=2;bowl.DepthV=2;bowl.MinV=.25;bowl.MaxV=.5;
    bowl.Net.Add(Detail::Point(FVector3d(-1,.25,0)));bowl.Net.Add(Detail::Point(FVector3d(0,-.25,0)));bowl.Net.Add(Detail::Point(FVector3d(1,.25,0)));
    Detail::FNode bd;bd.DegreeU=0;bd.DegreeV=1;bd.DepthV=2;bd.MinV=.25;bd.MaxV=.5;
    bd.Net.Add(Detail::Point(FVector3d(8,-4,0)));bd.Net.Add(Detail::Point(FVector3d(8,4,0)));
    c=Detail::Point(FVector3d(0,-1,0));
    TestTrue(TEXT("nonzero curvature admits positive derivative"),ContactDetail::CurveStationaryDomain98(bowl,bd,c,budget,domain)==E::Proved);
    TestTrue(TEXT("curved true root .5 retained"),A::Valid(domain)&&domain.Lower<=.5&&domain.Upper>=.5);
    // At w=.75, S=(.5,.0625,0), D=(8,2,0), and S-C=(.25,-1,0).
    // Their dot product is exactly0; the whole-cell derivative stays positive.
    c=Detail::Point(FVector3d(.25,1.0625,0));
    TestTrue(TEXT("off-center curved root contracts to right half"),
        ContactDetail::CurveStationaryDomain98(bowl,bd,c,budget,domain)==E::Proved&&
        domain.Lower>.5&&domain.Lower<=.75&&domain.Upper>=.75);
    c.Axis[0]={.24,.26,true};
    TestTrue(TEXT("curved center-box uncertainty retains nominal root"),
        ContactDetail::CurveStationaryDomain98(bowl,bd,c,budget,domain)==E::Proved&&
        domain.Lower>.5&&domain.Lower<=.75&&domain.Upper>=.75);
    // Center high above the same bowl makes the derivative interval contain0.
    c=Detail::Point(FVector3d(0,2,0));
    TestTrue(TEXT("zero-containing derivative does not pick a root"),ContactDetail::CurveStationaryDomain98(bowl,bd,c,budget,domain)==E::NotApplicable);
    auto uncertain=d;uncertain.Net[0].Axis[0]={-8,8,true};
    TestTrue(TEXT("independent derivative uncertainty cannot forge contraction"),ContactDetail::CurveStationaryDomain98(n,uncertain,Detail::Point(FVector3d(.5,1,0)),budget,domain)==E::NotApplicable);
    FDistanceBudget empty{0};
    TestTrue(TEXT("work refusal is sticky and distinct"),ContactDetail::CurveStationaryDomain98(n,d,c,empty,domain)==E::Work&&empty.Used==0&&empty.Remaining==0);
    auto invalid=n;invalid.Net[0].Axis[0]={};
    TestTrue(TEXT("invalid interval refuses without NaN output"),ContactDetail::CurveStationaryDomain98(invalid,d,c,budget,domain)==E::Arithmetic&&!A::Valid(domain));
    TestTrue(TEXT("original floating context restored"),context.Restore.Restore());
    return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticCurveStationaryContinuous98Test,
    "IAmSpeed.AnalyticWorld.TensorContact.CurveStationaryContinuous98",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticCurveStationaryContinuous98Test::RunTest(const FString&)
{
    using namespace Speed::Analytic::Certified;
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture"),context.Ready))return false;
    FIntervalTensorSurface curve;curve.DegreeU=0;curve.DegreeV=2;
    for(const auto& v:{FVector3d(-2,0,1),FVector3d(0,0,-1),FVector3d(2,0,1)})curve.ControlPoints.Add(Detail::Point(v));
    // True parabola z=x*x/4; SAME continuous quadratic path on/off.
    // At x=.375, surfaceZ=.03515625. The center first reaches y=.25
    // at t=.25; both endpoints are separated, so its chord would miss.
    FIntervalBezierCenterPath path;path.Degree=2;
    path.ControlPoints={Detail::Point(FVector3d(.375,1,.03515625)),Detail::Point(FVector3d(.375,-1,.03515625)),Detail::Point(FVector3d(.375,1,.03515625))};
    FTensorSphereContactEnclosure oldResult,newResult;
    for(bool on:{false,true})
    {
        CurveStationaryContact98::FEnabledScope enabled(on);FDistanceBudget work{4000000};
        int32 calls=0;
        const auto accept=[&](const FTensorSphereContactEnclosure& candidate)
        {++calls;return calls==1?EContactEnclosureDecision::Refine:EContactEnclosureDecision::Accept;};
        const FContactEnclosureFilter filter(accept);
        auto result=BoundTensorSphereFirstContact(MakeArrayView(&curve,1),path,.25,work,1./65536.,0,8192,.02,.02,24,&filter);
        TestTrue(TEXT("both versions certify the actual first contact"),result.Status==ETensorSphereContactStatus::FirstContactEnclosed&&!result.Leaves.IsEmpty());
        if(result.Status==ETensorSphereContactStatus::FirstContactEnclosed)
        {
            TestTrue(TEXT("true continuous time retained"),result.LowerTime<=.25&&result.UpperTime>=.25);
            TestTrue(TEXT("true finite-curve closest point retained"),result.SurfacePoint.Axis[0].Lower<=.375&&result.SurfacePoint.Axis[0].Upper>=.375&&result.SurfacePoint.Axis[2].Lower<=.03515625&&result.SurfacePoint.Axis[2].Upper>=.03515625);
            TestTrue(TEXT("consumer continuation retained"),calls>=2&&result.ConsumerWidthRefinements>=1);
            for(const auto& leaf:result.Leaves)TestTrue(TEXT("all leaf domains remain original closed square"),leaf.MinU>=0&&leaf.MaxU<=1&&leaf.MinV>=0&&leaf.MaxV<=1);
        }
        if(on)newResult=MoveTemp(result);else oldResult=MoveTemp(result);
    }
    TestTrue(TEXT("both time enclosures overlap"),oldResult.LowerTime<=newResult.UpperTime&&newResult.LowerTime<=oldResult.UpperTime);
    TestTrue(TEXT("context restored"),context.Restore.Restore());return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticCurveStationaryEndpointsAndRefusal98Test,
    "IAmSpeed.AnalyticWorld.TensorContact.CurveStationaryEndpointsAndRefusal98",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticCurveStationaryEndpointsAndRefusal98Test::RunTest(const FString&)
{
    using namespace Speed::Analytic::Certified;
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture"),context.Ready))return false;
    FIntervalTensorSurface line;line.DegreeU=0;line.DegreeV=1;
    line.ControlPoints={Detail::Point(FVector3d(0,0,0)),Detail::Point(FVector3d(1,0,0))};
    for(bool on:{false,true})
    {
        CurveStationaryContact98::FEnabledScope enabled(on);FDistanceBudget work{1000000};
        const auto result=BoundTensorSphereFirstContact(MakeArrayView(&line,1),FVector3d(-1,2,0),FVector3d(-1,-2,0),1.25,work,1./65536.,0,8192,.02,.02);
        TestTrue(TEXT("original endpoint closest contact remains admitted"),result.Status==ETensorSphereContactStatus::FirstContactEnclosed);
        if(result.Status==ETensorSphereContactStatus::FirstContactEnclosed)
            TestTrue(TEXT("finite endpoint retained, never infinite line"),result.SurfacePoint.Axis[0].Lower<=0&&result.SurfacePoint.Axis[0].Upper>=0&&result.SurfacePoint.Axis[0].Upper<.02);
        FDistanceBudget refusedWork{1000000};int32 calls=0;
        const auto refuse=[&](const FTensorSphereContactEnclosure&){++calls;return EContactEnclosureDecision::Refuse;};
        const FContactEnclosureFilter filter(refuse);FIntervalBezierCenterPath path;
        TestTrue(TEXT("same linear continuous input"),CenterDetail::Linear(FVector3d(-1,2,0),FVector3d(-1,-2,0),path));
        const auto rejected=BoundTensorSphereFirstContact(MakeArrayView(&line,1),path,1.25,refusedWork,1./65536.,0,8192,.02,.02,24,&filter);
        TestTrue(TEXT("refusal remains terminal with no partial contact"),calls==1&&rejected.Status==ETensorSphereContactStatus::Unknown&&rejected.Leaves.IsEmpty()&&rejected.Failure==ETensorSphereContactFailure::ConsumerRefusal);
    }
    TestTrue(TEXT("context restored"),context.Restore.Restore());return !HasAnyErrors();
}

#endif

