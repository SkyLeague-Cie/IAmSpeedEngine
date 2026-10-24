#if WITH_DEV_AUTOMATION_TESTS
#include "ResolvedPairSet.h"
#include "Misc/AutomationTest.h"
#include "Math/RandomStream.h"
#include "HAL/PlatformTime.h"
#include <limits>

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedResolvedPairSetTest,
	"IAmSpeed.Simulation.ResolvedPairSet",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedResolvedPairSetTest::RunTest(const FString& Parameters)
{
	{
		Speed::Collision::FResolvedPairSet Generations;
		TestTrue(TEXT("new exact identity may resolve initially"), Generations.CanResolve(91, true));
		Generations.RecordEligibleResolution(91);
		TestEqual(TEXT("initial resolution records generation one"), Generations.GetGeneration(91), uint8(1));
		TestTrue(TEXT("one exact reacquisition remains eligible"), Generations.CanResolve(91, true));
		Generations.RecordEligibleResolution(91);
		TestEqual(TEXT("reacquisition saturates at generation two"), Generations.GetGeneration(91), uint8(2));
		TestFalse(TEXT("second repeat is rejected"), Generations.CanResolve(91, true));
		Generations.RecordEligibleResolution(91);
		TestEqual(TEXT("generation stays saturated"), Generations.GetGeneration(91), uint8(2));
		TestTrue(TEXT("another vertex runtime key is independent"), Generations.CanResolve(92, true));
		TestTrue(TEXT("another provider runtime key is independent"), Generations.CanResolve(93, true));
		TestTrue(TEXT("another pair runtime key is independent"), Generations.CanResolve(94, true));

		Speed::Collision::FResolvedPairSet Excluded;
		Excluded.Add(101);
		TestEqual(TEXT("ordinary or ignored add consumes no generation"), Excluded.GetGeneration(101), uint8(0));
		TestFalse(TEXT("ordinary resolved pair cannot reacquire"), Excluded.CanResolve(101, false));
		TestFalse(TEXT("unrecorded eligible generation cannot bypass ordinary membership"), Excluded.CanResolve(101, true));

		SHitResult Eligible;
		Eligible.TOI = .001f;
		Eligible.SourceId = 11; Eligible.SurfaceId = 12;
		Eligible.ContactFeatureThis = Speed::EContactFeatureKind::Vertex;
		Eligible.ContactFeatureIndexThis = 7;
		Eligible.ContactFeatureOther = Speed::EContactFeatureKind::Face;
		TestTrue(TEXT("exact positive vertex-face event is structurally eligible"),
			Speed::Collision::IsEligibleExactStaticRepeatHit(Eligible));
		for (const float InvalidTOI : { 0.f, -.001f })
		{
			SHitResult Invalid = Eligible;
			Invalid.TOI = InvalidTOI;
			TestFalse(TEXT("nonpositive TOI is excluded"),
				Speed::Collision::IsEligibleExactStaticRepeatHit(Invalid));
		}
        SHitResult InitialExact = Eligible;InitialExact.TOI = 0;
        TestTrue(TEXT("exact time-zero event qualifies only for initial bookkeeping"), Speed::Collision::IsEligibleExactStaticInitialHit(InitialExact));
        TestFalse(TEXT("initial zero event never becomes repeat-query eligible"), Speed::Collision::IsEligibleExactStaticRepeatHit(InitialExact));
        TestFalse(TEXT("positive repeat is not an initial event"), Speed::Collision::IsEligibleExactStaticInitialHit(Eligible));
        Speed::Collision::FResolvedPairSet InitiallyResolved;
        InitiallyResolved.RecordEligibleResolution(103); // Actual qualified initial resolution, unlike Add.
        TestEqual(TEXT("qualified initial contact seeds generation one"),InitiallyResolved.GetGeneration(103),uint8(1));
        TestTrue(TEXT("later certified positive return can reacquire initial identity"),InitiallyResolved.CanResolve(103,Speed::Collision::IsEligibleExactStaticRepeatHit(Eligible)));
        TestFalse(TEXT("zero-time rediscovery remains rejected even after initialization"),InitiallyResolved.CanResolve(103,Speed::Collision::IsEligibleExactStaticRepeatHit(InitialExact)));
        InitiallyResolved.RecordEligibleResolution(103);
        TestFalse(TEXT("initial plus one positive return saturates existing generation budget"),InitiallyResolved.CanResolve(103,true));
        for(int32 InvalidCase=0;InvalidCase<8;++InvalidCase)
        {
            SHitResult InvalidInitial=InitialExact;
            if(InvalidCase==0)InvalidInitial.TOI=-.001f;
            if(InvalidCase==1)InvalidInitial.bSurfaceNormalMayVary=true;
            if(InvalidCase==2)InvalidInitial.GeometricErrorBoundCm=.001f;
            if(InvalidCase==3)InvalidInitial.bStartPenetrating=true;
            if(InvalidCase==4)InvalidInitial.PenetrationDepth=.001f;
            if(InvalidCase==5)InvalidInitial.SourceId=0;
            if(InvalidCase==6)InvalidInitial.SurfaceId=0;
            if(InvalidCase==7)InvalidInitial.ContactFeatureOther=Speed::EContactFeatureKind::Edge;
            TestFalse(TEXT("ineligible initial geometry/time cannot seed exact reacquisition"),Speed::Collision::IsEligibleExactStaticInitialHit(InvalidInitial));
        }
		SHitResult Penetrating = Eligible;
		Penetrating.bStartPenetrating = true;
		Penetrating.PenetrationDepth = .001f;
		TestFalse(TEXT("penetrating hit is excluded"),
			Speed::Collision::IsEligibleExactStaticRepeatHit(Penetrating));
		SHitResult Curved = Eligible;
		Curved.bSurfaceNormalMayVary = true;
		TestFalse(TEXT("varying normal is excluded"),
			Speed::Collision::IsEligibleExactStaticRepeatHit(Curved));
		SHitResult Uncertain = Eligible;
		Uncertain.GeometricErrorBoundCm = static_cast<float>(.001);
		TestFalse(TEXT("uncertain geometry is excluded"),
			Speed::Collision::IsEligibleExactStaticRepeatHit(Uncertain));
		SHitResult DifferentFeature = Eligible;
		DifferentFeature.ContactFeatureThis = Speed::EContactFeatureKind::Face;
		TestFalse(TEXT("non vertex-face contact is excluded"),
			Speed::Collision::IsEligibleExactStaticRepeatHit(DifferentFeature));
	}
    {
        Speed::Collision::FResolvedPairSet Timed;
        TestFalse(TEXT("no unrecorded identity has temporal eligibility"),Timed.CanResolveExactProgress(201,.1f,0,.1f,1));
        TestTrue(TEXT("actual qualified initial resolution seeds time zero"),Timed.RecordExactProgressResolution(201,0,1));
        TestTrue(TEXT("positive original repeat and actual float advance admitted"),Timed.CanResolveExactProgress(201,.1f,0,.1f,1));
        TestTrue(TEXT("actual first return records its real time"),Timed.RecordExactProgressResolution(201,.1f,1));
        TestEqual(TEXT("temporal path preserves legacy saturation"),Timed.GetGeneration(201),uint8(2));
        TestFalse(TEXT("legacy API still rejects third resolution"),Timed.CanResolve(201,true));
        TestTrue(TEXT("separate API admits third resolution only at later time"),Timed.CanResolveExactProgress(201,.1f,.1f,.2f,1));
        TestTrue(TEXT("actual third resolution records progress"),Timed.RecordExactProgressResolution(201,.2f,1));
        TestEqual(TEXT("third resolution does not forge a legacy generation"),Timed.GetGeneration(201),uint8(2));
        const float NaN=std::numeric_limits<float>::quiet_NaN(),Inf=std::numeric_limits<float>::infinity();
        const float Invalid[][4]={
            {0,.2f,.3f,1},{-.1f,.2f,.3f,1},{NaN,.2f,.3f,1},{Inf,.2f,.3f,1},
            {.1f,.2f,.2f,1},{.1f,.3f,.2f,1},{.1f,-.1f,.3f,1},{.1f,.1f,.3f,1},
            {.1f,.2f,1.1f,1},{.1f,NaN,.3f,1},{.1f,.2f,NaN,1},{.1f,.2f,Inf,1},
            {.1f,.2f,.3f,NaN},{.1f,.2f,.3f,Inf},{.1f,.2f,.3f,-1},
            {.000000001f,1,1+.000000001f,2}};
        for(const auto& T:Invalid)TestFalse(TEXT("invalid TOI/range/backward/nonadvancing actual float time rejected"),Timed.CanResolveExactProgress(201,T[0],T[1],T[2],T[3]));
        TestTrue(TEXT("exact frame endpoint is admissible"),Timed.CanResolveExactProgress(201,.8f,.2f,1,1));
        TestTrue(TEXT("preselection accepts real bounded float advance"),Timed.CanSelectExactProgress(201,.1f,.2f,1));
        TestFalse(TEXT("positive TOI whose float sum does not advance is rejected"),Timed.CanSelectExactProgress(201,1.e-10f,.2f,1));
        TestFalse(TEXT("oversized finite TOI rejected before any overflow addition"),Timed.CanSelectExactProgress(201,std::numeric_limits<float>::max(),.2f,1));
        TestFalse(TEXT("invalid frame rejected before arrival arithmetic"),Timed.CanSelectExactProgress(201,.1f,.2f,Inf));
        TestFalse(TEXT("preselected event cannot use predicted progress if actual integration did not advance"),Timed.CanResolveExactProgress(201,.1f,.2f,.2f,1));
        TestFalse(TEXT("earlier simultaneous actual batch time cannot reuse last recorded time"),Timed.CanResolveExactProgress(201,.1f,.2f,.2f,1));
        TestFalse(TEXT("another identity has no borrowed certificate"),Timed.CanResolveExactProgress(202,.1f,.2f,.3f,1));
        Timed.Add(201);
        TestFalse(TEXT("ordinary or ignored Add invalidates existing time certificate"),Timed.CanResolveExactProgress(201,.1f,.2f,.3f,1));
        TestFalse(TEXT("later exact record cannot resurrect invalidated member"),Timed.RecordExactProgressResolution(201,.3f,1));
        TestEqual(TEXT("invalidating Add preserves saturated legacy generation"),Timed.GetGeneration(201),uint8(2));
        Speed::Collision::FResolvedPairSet Ordinary;Ordinary.Add(203);
        TestFalse(TEXT("ordinary membership cannot become timed by a new exact record"),Ordinary.RecordExactProgressResolution(203,.1f,1));
        TestEqual(TEXT("rejected ordinary record preserves generation zero"),Ordinary.GetGeneration(203),uint8(0));
        TestFalse(TEXT("rejected ordinary record cannot arm legacy reacquisition"),Ordinary.CanResolve(203,true));
        Speed::Collision::FResolvedPairSet Legacy;Legacy.RecordEligibleResolution(204);
        TestFalse(TEXT("legacy generation alone is not a temporal certificate"),Legacy.CanResolveExactProgress(204,.1f,0,.1f,1));
        TestFalse(TEXT("legacy record cannot be relabeled as timed"),Legacy.RecordExactProgressResolution(204,.1f,1));
        Speed::Collision::FResolvedPairSet Downgraded;Downgraded.RecordExactProgressResolution(205,0,1);Downgraded.RecordEligibleResolution(205);
        TestFalse(TEXT("legacy record invalidates a prior temporal certificate"),Downgraded.CanResolveExactProgress(205,.1f,0,.1f,1));
        for(float InvalidTime:{-.1f,NaN,Inf,1.1f})
        {
            Speed::Collision::FResolvedPairSet BadSeed;
            TestFalse(TEXT("invalid actual initial time never seeds"),BadSeed.RecordExactProgressResolution(206,InvalidTime,1));
            TestEqual(TEXT("invalid initial record leaves ordinary generation zero"),BadSeed.GetGeneration(206),uint8(0));
            TestFalse(TEXT("invalid initial record cannot arm legacy reacquisition"),BadSeed.CanResolve(206,true));
            TestFalse(TEXT("failed seed never gains time eligibility later"),BadSeed.RecordExactProgressResolution(206,.2f,1));
            TestEqual(TEXT("rejected seed retry still preserves generation zero"),BadSeed.GetGeneration(206),uint8(0));
            TestFalse(TEXT("rejected seed retry cannot arm legacy reacquisition"),BadSeed.CanResolve(206,true));
        }
        for(float InvalidTime:{0.f,-.1f,NaN,Inf,1.1f})
        {
            Speed::Collision::FResolvedPairSet BadReturn;BadReturn.RecordExactProgressResolution(207,0,1);
            TestFalse(TEXT("invalid actual return invalidates without inventing progress"),BadReturn.RecordExactProgressResolution(207,InvalidTime,1));
            TestEqual(TEXT("invalid actual return preserves prior legacy generation"),BadReturn.GetGeneration(207),uint8(1));
            TestFalse(TEXT("invalid actual return cannot leave reusable timed proof"),BadReturn.CanResolveExactProgress(207,.1f,0,.1f,1));
        }
    }
	FRandomStream Random(150941);
	for (int32 Frame = 0; Frame < 16; ++Frame)
	{
		Speed::Collision::FResolvedPairSet Actual;
		const TSet<uint64> Empty;
		TSet<uint64> Expected;
		Expected.Reserve(128);
		// TSet's accounting includes its small inline hash even before allocation.
		TestEqual(TEXT("empty frame has only default set storage"), Actual.Pairs.GetAllocatedSize(), Empty.GetAllocatedSize());
		TestTrue(TEXT("empty frame avoids the 128-entry reserve"), Actual.Pairs.GetAllocatedSize() < Expected.GetAllocatedSize());
		TestFalse(TEXT("new frame remembers no old pair"), Actual.Contains(0));
		for (int32 Index = 0; Index < 4096; ++Index)
		{
			const uint64 Key = Index % 7 == 0 ? 0 :
				(uint64(Random.GetUnsignedInt() % 300) << 32) | (Random.GetUnsignedInt() % 3);
			TestEqual(TEXT("membership before insertion"), Actual.Contains(Key), Expected.Contains(Key));
			Actual.Add(Key); Expected.Add(Key);
			TestTrue(TEXT("inserted pair can be found"), Actual.Contains(Key));
			TestEqual(TEXT("duplicates and growth preserve cardinality"), Actual.Pairs.Num(), Expected.Num());
			TestEqual(TEXT("nonempty frame has original capacity"), Actual.Pairs.GetAllocatedSize(), Expected.GetAllocatedSize());
		}
		for (const uint64 Key : Expected)
		{
			if (!TestTrue(TEXT("all old pairs survive growth"), Actual.Contains(Key))) return false;
		}
	}
	return true;
}

namespace
{
	struct FOriginalResolvedPairSet : TSet<uint64>
	{
		FOriginalResolvedPairSet() { Reserve(128); }
	};

	template<typename SetType>
	FORCENOINLINE double MeasureResolvedPairs(const int32 PairCount, uint64& Checksum)
	{
		constexpr int32 Frames = 32768;
		uint64 Sum = 0;
		const double Start = FPlatformTime::Seconds();
		for (int32 Frame = 0; Frame < Frames; ++Frame)
		{
			SetType Pairs;
			for (int32 Index = 0; Index < PairCount; ++Index)
			{
				const uint64 Key = (uint64(Frame) << 32) | uint64(Index);
				Sum += Pairs.Contains(Key);
				Pairs.Add(Key);
				Sum += Pairs.Contains(Key);
			}
			Sum += Pairs.Contains(uint64(Frame));
		}
		const double Elapsed = FPlatformTime::Seconds() - Start;
		Checksum = Sum;
		return 1.e9 * Elapsed / Frames;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedResolvedPairSetCostTest,
	"IAmSpeed.Simulation.ResolvedPairSetCost",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedResolvedPairSetCostTest::RunTest(const FString& Parameters)
{
	for (const int32 Count : { 0, 1, 8, 128 })
	{
		uint64 OriginalSum = 0, LazySum = 0;
		MeasureResolvedPairs<FOriginalResolvedPairSet>(Count, OriginalSum);
		MeasureResolvedPairs<Speed::Collision::FResolvedPairSet>(Count, LazySum);
		for (int32 Pair = 0; Pair < 5; ++Pair)
		{
			double Original, Lazy;
			if (Pair % 2 == 0)
			{
				Original = MeasureResolvedPairs<FOriginalResolvedPairSet>(Count, OriginalSum);
				Lazy = MeasureResolvedPairs<Speed::Collision::FResolvedPairSet>(Count, LazySum);
			}
			else
			{
				Lazy = MeasureResolvedPairs<Speed::Collision::FResolvedPairSet>(Count, LazySum);
				Original = MeasureResolvedPairs<FOriginalResolvedPairSet>(Count, OriginalSum);
			}
			TestEqual(TEXT("paired set checksums match"), OriginalSum, LazySum);
			AddInfo(FString::Printf(TEXT("[ResolvedPairSetMicro] Count=%d Pair=%d OriginalNs=%.6f LazyNs=%.6f"),
				Count, Pair, Original, Lazy));
		}
	}
	return true;
}
#endif
