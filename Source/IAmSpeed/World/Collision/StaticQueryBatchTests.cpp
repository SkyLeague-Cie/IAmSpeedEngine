#if WITH_DEV_AUTOMATION_TESTS
#include "StaticCollisionWorld.h"
#include "BoxRestingSupport.h"
#include "Misc/AutomationTest.h"
#include "Math/RandomStream.h"
#include "Algo/Reverse.h"

namespace Speed::StaticQueryBatchTests
{
using namespace Analytic;
bool SameHit(const FWorldHit& A, const FWorldHit& B)
{
    const auto Bits = [](const auto& X, const auto& Y)
    { return FMemory::Memcmp(&X, &Y, sizeof(X)) == 0; };
    return A.bHit == B.bHit && A.bStartPenetrating == B.bStartPenetrating &&
        Bits(A.Time, B.Time) && Bits(A.PenetrationDepth, B.PenetrationDepth) &&
        Bits(A.GeometricErrorBoundCm, B.GeometricErrorBoundCm) &&
        Bits(A.AdditionalResidualAgreementAllowanceCm, B.AdditionalResidualAgreementAllowanceCm) &&
        A.bSurfaceNormalMayVary == B.bSurfaceNormalMayVary &&
        Bits(A.Location, B.Location) && Bits(A.Point, B.Point) &&
        Bits(A.QueryPoint, B.QueryPoint) && Bits(A.Normal, B.Normal) &&
        A.QueryFeatureKind == B.QueryFeatureKind && A.SurfaceFeatureKind == B.SurfaceFeatureKind &&
        A.QueryFeatureIndex == B.QueryFeatureIndex && A.SurfaceFeatureIndex == B.SurfaceFeatureIndex &&
        A.SourceId == B.SourceId && A.SurfaceId == B.SurfaceId && A.FeatureId == B.FeatureId &&
        A.PrimitiveId == B.PrimitiveId && A.CanonicalGroupId == B.CanonicalGroupId &&
        A.MaterialId == B.MaterialId;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedStaticQueryBatchBitsTest,
    "IAmSpeed.Parallel.StaticQueryBatch.AllHitFields",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedStaticQueryBatchBitsTest::RunTest(const FString& Parameters)
{
    using namespace Speed;
    using namespace StaticQueryBatchTests;
    FAnalyticWorldData Data;
    for (int32 Index = 0; Index < 3; ++Index)
    {
        FBoundedPlane Plane;
        Plane.SourceId = Index + 1; Plane.SurfaceId = Index + 11;
        Plane.FeatureId = Index + 21; Plane.PrimitiveId = Index + 31;
        Plane.Origin = FVector3d(0, 0, Index * 25.0);
        Plane.HalfExtents = FVector2d(200, 175);
        Plane.BlockingChannels = MAX_uint64;
        Data.Planes.Add(Plane);
    }
    FString Reason;
    if (!TestTrue(TEXT("fixture validates"), Data.FinalizeAndValidate(&Reason)))
    { AddError(Reason); return false; }
    for (auto& Plane : Data.Planes) Plane.bAuthorityEligible = true;
    const FAnalyticStaticCollisionWorld Serial(Data);
    FRandomStream Random(40104);
    TArray<FWorldQuery> Queries;
    TArray<FWorldHit> Expected;
    for (int32 Index = 0; Index < 255; ++Index)
    {
        FWorldQuery Q;
        Q.Shape = EQueryShape(Index % 3);
        Q.Start = FVector3d(Random.FRandRange(-300.f, 300.f), Random.FRandRange(-250.f, 250.f), Random.FRandRange(-20.f, 150.f));
        Q.End = Q.Start + FVector3d(Random.FRandRange(-50.f, 50.f), 0, Random.FRandRange(-120.f, 20.f));
        Q.HalfExtent = FVector3d(12, 8, 5); Q.Radius = 7;
        Q.bAuthorityOnly = true; Q.bIncludeCompactPatches = true;
        if (Index % 7 == 0) Q.End = Q.Start;
        if (Index % 11 == 0) Q.RequiredSourceId = 1;
        if (Index % 13 == 0) { Q.bApplyCollisionFilter = true; Q.BlockingObjectTypes = 0; }
        if (Index % 17 == 0) Q.Rotation = FRotator3d(15, 35, 20).Quaternion();
        Queries.Add(Q); Expected.Add(Serial.SweepSingle(Q));
    }
    uint64 ParallelDispatches = 0;
    for (const int32 Lanes : {1, 2, 4, 8, 12})
    {
        FAnalyticStaticCollisionWorld World(Data, Lanes);
        TArray<FWorldHit> Actual;
        Actual.SetNum(Queries.Num());
        for (int32 Repeat = 0; Repeat < 3; ++Repeat)
        {
            FIndependentQueryBatchStats Stats;
            if (!TestTrue(TEXT("batch joins"), World.SweepIndependentBatch(Queries, Actual, &Stats))) return false;
            TestEqual(TEXT("requested lanes recorded"), Stats.RequestedLanes, Lanes);
            if (Lanes > 1)
            {
                if (!TestTrue(TEXT("unprofiled native fixture must exercise actual parallel dispatch"),
                    Stats.Fallback == EIndependentQueryFallback::None)) return false;
                ++ParallelDispatches;
                TestEqual(TEXT("all logical lanes joined"), Stats.DispatchedLogicalLanes, Lanes);
                TestTrue(TEXT("actual native participation is recorded separately"),
                    Stats.DistinctNativeThreads >= 1 && Stats.DistinctNativeThreads <= Lanes);
            }
            else TestEqual(TEXT("explicit serial policy or diagnostic fallback"), Stats.DispatchedLogicalLanes, 1);
            for (int32 Index = 0; Index < Queries.Num(); ++Index)
                if (!SameHit(Expected[Index], Actual[Index]))
                { AddError(FString::Printf(TEXT("first divergence lanes=%d repeat=%d query=%d"), Lanes, Repeat, Index)); return false; }
            // Change assignment to persistent lane caches without changing the oracle.
            Algo::Reverse(Queries); Algo::Reverse(Expected);
        }
        const FWorldHit Sentinel = Actual[0];
        TestFalse(TEXT("bad output extent rejected before writes"), World.SweepIndependentBatch(Queries, MakeArrayView(Actual.GetData(), 1)));
        TestTrue(TEXT("rejected output preserved"), SameHit(Sentinel, Actual[0]));
        TestTrue(TEXT("empty batch joins"), World.SweepIndependentBatch({}, {}));
    }
    TestTrue(TEXT("positive dispatch coverage is mandatory"), ParallelDispatches > 0);
    // A separate over-capacity call proves the bounded fallback without letting
    // the equivalence campaign silently pass by running entirely in serial.
    const FWorldQuery ExtraQuery = Queries.Last();
    const FWorldHit ExtraExpected = Expected.Last();
    Queries.Add(ExtraQuery); Queries.Add(ExtraQuery);
    Expected.Add(ExtraExpected); Expected.Add(ExtraExpected);
    FAnalyticStaticCollisionWorld CapacityWorld(Data, 4);
    TArray<FWorldHit> CapacityActual; CapacityActual.SetNum(Queries.Num());
    FIndependentQueryBatchStats CapacityStats;
    TestTrue(TEXT("over-capacity route completes"), CapacityWorld.SweepIndependentBatch(Queries, CapacityActual, &CapacityStats));
    TestTrue(TEXT("over-capacity fallback remains explicit"), CapacityStats.Fallback == EIndependentQueryFallback::BatchCapacityExceeded);
    TestEqual(TEXT("over-capacity route owns one logical lane"), CapacityStats.DispatchedLogicalLanes, 1);
    for (int32 Index = 0; Index < Queries.Num(); ++Index)
        if (!SameHit(Expected[Index], CapacityActual[Index]))
        { AddError(FString::Printf(TEXT("capacity fallback divergence query=%d"), Index)); return false; }
    return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedStaticQueryBoxSupportOracleTest,
    "IAmSpeed.Parallel.StaticQueryBatch.BoxSupportOracle",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FIAmSpeedStaticQueryBoxSupportOracleTest::RunTest(const FString& Parameters)
{
    using namespace Speed;
    using namespace StaticQueryBatchTests;
    FAnalyticWorldData Data;
    FBoundedPlane Plane;
    Plane.SourceId = 1; Plane.SurfaceId = 2; Plane.FeatureId = 3; Plane.PrimitiveId = 4;
    Plane.HalfExtents = FVector2d(100, 90); Plane.BlockingChannels = MAX_uint64;
    Data.Planes.Add(Plane);
    if (!TestTrue(TEXT("support fixture validates"), Data.FinalizeAndValidate())) return false;
    Data.Planes[0].bAuthorityEligible = true;
    FAnalyticStaticCollisionWorld Serial(Data);
    const auto Bits = [](const auto& A, const auto& B) { return FMemory::Memcmp(&A, &B, sizeof(A)) == 0; };
    for (const int32 Lanes : {1, 2, 4, 8, 12})
    for (const double Height : {19.999, 20.0, 20.000000000001, 20.001})
    for (const double X : {0.0, 39.0, 40.0, 80.0, 110.0})
    for (const double COMOffset : {0.0, 59.0, 60.0, 61.0})
    {
        FWorldQuery Query; Query.Shape = EQueryShape::Box;
        Query.Start = Query.End = FVector3d(X, 0, Height);
        Query.HalfExtent = FVector3d(60, 40, 20); Query.bAuthorityOnly = true;
        const FVector COM = Query.Start + FVector(COMOffset, 0, 0);
        FBoxRestingSupport Expected, Actual;
        const bool ExpectedOK = TryBuildBoxRestingSupport(Serial, Query, COM, FVector(0, 0, -980), Expected);
        FAnalyticStaticCollisionWorld Parallel(Data, Lanes);
        const bool ActualOK = TryBuildBoxRestingSupport(Parallel, Query, COM, FVector(0, 0, -980), Actual);
        bool Equal = ExpectedOK == ActualOK && Expected.Result == Actual.Result &&
            Expected.WitnessIndex == Actual.WitnessIndex && SameHit(Expected.ProviderWitness, Actual.ProviderWitness) &&
            Bits(Expected.ResultantAcceleration, Actual.ResultantAcceleration) && Bits(Expected.Normal, Actual.Normal) &&
            Bits(Expected.WitnessGapCm, Actual.WitnessGapCm) && Bits(Expected.StopAfterSeconds, Actual.StopAfterSeconds);
        for (int32 Point = 0; Point < 4; ++Point)
            Equal &= Bits(Expected.Points[Point], Actual.Points[Point]) && Bits(Expected.LoadFractions[Point], Actual.LoadFractions[Point]);
        if (!Equal)
        { AddError(FString::Printf(TEXT("support divergence lanes=%d x=%.17g z=%.17g com=%.17g"), Lanes, X, Height, COMOffset)); return false; }
    }
    return true;
}
#endif
