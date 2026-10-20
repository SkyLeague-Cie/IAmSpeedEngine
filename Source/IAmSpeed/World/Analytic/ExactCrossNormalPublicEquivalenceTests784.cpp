// SOURCE-ONLY candidate784. Not installed or runtime-qualified.
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "AnalyticWorldQuery.h"
#include "HAL/IConsoleManager.h"
#include "HAL/CriticalSection.h"
#include "Misc/AutomationTest.h"
#include "Misc/ScopeLock.h"
#include <limits>

namespace ExactCrossNormal784
{
using namespace Speed::Analytic;
static FCriticalSection TestMutex;

static uint64 Bits(double Value)
{
    static_assert(sizeof(double) == sizeof(uint64), "IEEE double required");
    static_assert(std::numeric_limits<double>::is_iec559, "IEEE double required");
    uint64 Result; FMemory::Memcpy(&Result, &Value, sizeof(Result)); return Result;
}

// Never compare structure padding, and never use UE's tolerant TestEqual(double).
static const TCHAR* FirstDifference(const FWorldHit& A, const FWorldHit& B)
{
#define SCALAR(Name) if (A.Name != B.Name) return TEXT(#Name)
#define REAL(Name) if (Bits(A.Name) != Bits(B.Name)) return TEXT(#Name)
#define VECTOR(Name) REAL(Name.X); REAL(Name.Y); REAL(Name.Z)
    SCALAR(bHit); SCALAR(bStartPenetrating); REAL(Time); REAL(PenetrationDepth);
    REAL(GeometricErrorBoundCm); REAL(AdditionalResidualAgreementAllowanceCm);
    SCALAR(bSurfaceNormalMayVary); VECTOR(Location); VECTOR(Point);
    VECTOR(QueryPoint); VECTOR(Normal); SCALAR(QueryFeatureKind); SCALAR(SurfaceFeatureKind);
    SCALAR(QueryFeatureIndex); SCALAR(SurfaceFeatureIndex); SCALAR(SourceId);
    SCALAR(SurfaceId); SCALAR(FeatureId); SCALAR(PrimitiveId); SCALAR(CanonicalGroupId);
    SCALAR(MaterialId);
#undef VECTOR
#undef REAL
#undef SCALAR
    return nullptr;
}

struct FConsoleLease
{
    IConsoleVariable* Variable;
    int32 Value;
    uint32 Flags;
    bool bRestored = false;
    explicit FConsoleLease(IConsoleVariable* In) : Variable(In), Value(In->GetInt()), Flags(In->GetFlags()) {}
    FConsoleLease(const FConsoleLease&) = delete;
    bool Set(int32 NewValue)
    {
        Variable->ClearFlags(ECVF_SetByMask);
        Variable->Set(NewValue, ECVF_SetByCode);
        return Variable->GetInt() == NewValue;
    }
    bool Restore()
    {
        if (!bRestored)
        {
            Variable->ClearFlags(ECVF_SetByMask);
            Variable->Set(Value, ECVF_SetByCode);
            Variable->ClearFlags(static_cast<EConsoleVariableFlags>(MAX_uint32));
            Variable->SetFlags(static_cast<EConsoleVariableFlags>(Flags));
            bRestored = true;
        }
        return Variable->GetInt() == Value && Variable->GetFlags() == Flags;
    }
    ~FConsoleLease() { Restore(); }
};

enum class EOracle : uint8 { Miss, FlatQuarterSweep, FlatUnitOverlap, AdmissionRejected, Unknown };
struct FProbe
{
    FString Name;
    FWorldQuery Query;
    EOracle SweepOracle = EOracle::Unknown;
    EOracle PoseOracle = EOracle::Unknown;
    bool bSweep = true;
    int32 World = 0;
};

static FWorldQuery Query()
{
    FWorldQuery Q;
    Q.Shape = EQueryShape::Box; Q.HalfExtent = FVector3d(2, 2, 2);
    Q.Start = FVector3d(0, 0, 4); Q.End = FVector3d(0, 0, -4);
    Q.bIncludeCompactPatches = true; Q.bUseFiniteContactDomain = true;
    Q.DomainTolerance = 0; Q.InitialOverlapTolerance = 0;
    return Q;
}

static bool BuildWorld(FAnalyticWorldData& Data, int32 Kind, FString& Reason)
{
    // Bounds include the full control hull and both extrusion endpoints before finalization.
    FExtrudedQuinticPatch P;
    P.SourceId = 30; P.SurfaceId = 31; P.FeatureId = 32; P.PrimitiveId = 33;
    const double Bend = Kind == 1 ? 3.0 : Kind == 2 ? -3.0 : 0.0;
    for (int32 I = 0; I < 6; ++I)
        P.SectionControlPoints[I] = FVector3d(-50 + 20 * I, 0, Bend * I * (5 - I));
    P.ExtrusionAxis = Kind == 3 ? FVector3d(.6, .8, 0) : FVector3d::RightVector;
    P.MinimumExtrusionCoordinate = -25; P.MaximumExtrusionCoordinate = 25;
    P.bQueryCollisionEnabled = true; P.ObjectType = 1; P.BlockingChannels = MAX_uint64;
    P.Bounds = FBox3d(EForceInit::ForceInit);
    for (const FVector3d& C : P.SectionControlPoints)
    {
        P.Bounds += C + P.MinimumExtrusionCoordinate * P.ExtrusionAxis;
        P.Bounds += C + P.MaximumExtrusionCoordinate * P.ExtrusionAxis;
    }
    Data.ExtrudedQuinticPatches.Add(P);
    if (Kind == 4)
    {
        // Equal surfaces with independent provider IDs exercise public tie selection.
        P.SourceId = 40; P.SurfaceId = 41; P.FeatureId = 42; P.PrimitiveId = 43;
        Data.ExtrudedQuinticPatches.Add(P);
    }
    if (!Data.FinalizeAndValidate(&Reason)) return false;
    if ((Kind == 1 || Kind == 2) && Data.ExtrudedQuinticPatches[0].SectionPolyline.Num() < 3)
    { Reason = TEXT("curved fixture did not produce multiple facets"); return false; }
    return true;
}

static bool ExpectedWitness(EOracle Oracle, bool bOperationAvailable, const FWorldHit& H)
{
    switch (Oracle)
    {
    case EOracle::Miss: return bOperationAvailable && !H.bHit;
    case EOracle::FlatQuarterSweep:
        // Independent slab equation: (4 - halfZ=2) / travel=8 = 1/4, all dyadic.
        return bOperationAvailable && H.bHit && !H.bStartPenetrating && Bits(H.Time) == Bits(.25);
    case EOracle::FlatUnitOverlap:
        // Pose center Z=1, halfZ=2 above flat Z=0: positive penetration exactly 1.
        return bOperationAvailable && H.bHit && H.bStartPenetrating && Bits(H.PenetrationDepth) == Bits(1.0);
    case EOracle::AdmissionRejected: return !bOperationAvailable && !H.bHit;
    case EOracle::Unknown: return true; // Logged as UNKNOWN; never counted as an independent oracle.
    }
    return false;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedExtrudedExactCrossNormal784Test,
    "IAmSpeed.AnalyticWorld.ExactCrossNormalPublicEquivalence784",
    (EAutomationTestFlags::EditorContext | EAutomationTestFlags::ClientContext |
     EAutomationTestFlags::ServerContext | EAutomationTestFlags::CommandletContext) | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedExtrudedExactCrossNormal784Test::RunTest(const FString& Parameters)
{
    using namespace ExactCrossNormal784;
    FScopeLock Lock(&TestMutex);
    if (!IsInGameThread()) { AddError(TEXT("784 requires the game thread and an external exclusive job")); return false; }
    auto Find = [](const TCHAR* Name) { return IConsoleManager::Get().FindConsoleVariable(Name); };
    IConsoleVariable* Scope = Find(TEXT("p.IAmSpeed.AnalyticWorld.ExactCrossNormalCache784"));
    IConsoleVariable* Lazy = Find(TEXT("p.IAmSpeed.AnalyticWorld.LazyExtrudedBoxSatAxes"));
    if (!Scope || !Lazy) { AddError(TEXT("784 and lazy flags must both exist")); return false; }
    for (const TCHAR* Name : { TEXT("p.IAmSpeed.AnalyticWorld.PhaseTiming"),
            TEXT("p.IAmSpeed.AnalyticWorld.CacheProfile"), TEXT("p.IAmSpeed.AnalyticWorld.ExtrudedFacetGeometryCache"),
            TEXT("p.IAmSpeed.AnalyticWorld.ExtrudedBoxPrimarySatScopeCache"), TEXT("p.IAmSpeed.AnalyticWorld.ExtrudedBoxBoundsQueryReuse435"), TEXT("p.IAmSpeed.AnalyticWorld.ExtrudedTriangleContextConstruction541") })
    {
        IConsoleVariable* V = Find(Name);
        if (!V || V->GetInt() != 0) { AddError(FString::Printf(TEXT("784 requires %s=0"), Name)); return false; }
    }
    IConsoleVariable* CaseDiagnostic = Find(TEXT("p.IAmSpeed.AnalyticWorld.CaseSatDiagnostics"));
    if (CaseDiagnostic && CaseDiagnostic->GetInt() != 0)
    { AddError(TEXT("784 requires CaseSatDiagnostics absent or zero")); return false; }
    FConsoleLease ScopeLease(Scope), LazyLease(Lazy);
    TArray<FProbe> Probes;
    auto Add = [&](const TCHAR* Name, int32 World, EOracle Sweep, EOracle Pose) -> FProbe&
    {
        FProbe& P = Probes.AddDefaulted_GetRef(); P.Name = Name; P.World = World;
        P.Query = Query(); P.SweepOracle = Sweep; P.PoseOracle = Pose; return P;
    };
    Add(TEXT("flat_dyadic_quarter_and_unit_depth"), 0, EOracle::FlatQuarterSweep, EOracle::FlatUnitOverlap);
    Add(TEXT("finite_edge_x"), 0, EOracle::Unknown, EOracle::Unknown).Query.Start.X = 49;
    Probes.Last().Query.End.X = 49;
    Add(TEXT("finite_edge_y"), 0, EOracle::Unknown, EOracle::Unknown).Query.Start.Y = 24;
    Probes.Last().Query.End.Y = 24;
    Add(TEXT("equal_provider_TOI_and_depth"), 4, EOracle::FlatQuarterSweep, EOracle::FlatUnitOverlap);
    for (int32 World : {1, 2, 3})
    {
        FProbe& P = Add(*FString::Printf(TEXT("curved_or_oblique_%d"), World), World, EOracle::Unknown, EOracle::Unknown);
        P.Query.Start = FVector3d(-45, -20, 45); P.Query.End = FVector3d(45, 20, -45);
        P.Query.HalfExtent = FVector3d(6, .3, 2);
        P.Query.Rotation = FRotator3d(13, 27, 11).Quaternion();
        FProbe& Far = Add(*FString::Printf(TEXT("control_hull_far_miss_%d"), World), World, EOracle::Miss, EOracle::Miss);
        Far.Query.Start = FVector3d(0, 10000, 4); Far.Query.End = FVector3d(0, 10000, -4);
    }
    for (double Angle : {2.e-11, 1.e-10})
    {
        FProbe& P = Add(*FString::Printf(TEXT("cross_threshold_requested_%.17g"), Angle), 1, EOracle::Unknown, EOracle::Unknown);
        P.Query.Rotation = FQuat4d(FVector3d::UpVector, Angle);
    }
    for (double Domain : {0., 1.e-4, .01})
    {
        FProbe& P = Add(*FString::Printf(TEXT("finite_domain_%.17g"), Domain), 1, EOracle::Unknown, EOracle::Unknown);
        P.Query.Start = FVector3d(49, 24, 8); P.Query.End = FVector3d(49, 24, -8);
        P.Query.DomainTolerance = Domain;
    }
    FProbe& Zero = Add(TEXT("raw_signed_zero"), 0, EOracle::FlatQuarterSweep, EOracle::FlatUnitOverlap);
    Zero.Query.Start.X = -0.0; Zero.Query.End.X = -0.0;
    FProbe& NegativeZero = Add(TEXT("raw_negative_zero_extent"), 0, EOracle::Unknown, EOracle::AdmissionRejected);
    NegativeZero.Query.HalfExtent.X = -0.0;
    FProbe& PositiveZero = Add(TEXT("raw_positive_zero_extent"), 0, EOracle::Unknown, EOracle::AdmissionRejected);
    PositiveZero.Query.HalfExtent.X = +0.0;
    FProbe& NonUnit = Add(TEXT("deepest_nonunit_admission"), 0, EOracle::Unknown, EOracle::AdmissionRejected);
    NonUnit.bSweep = false; NonUnit.Query.Rotation = FQuat4d(0, 0, 0, 2);
    FProbe& Negative = Add(TEXT("deepest_negative_extent_admission"), 0, EOracle::Unknown, EOracle::AdmissionRejected);
    Negative.Query.HalfExtent.X = -2;
    FProbe& NegativeDomain = Add(TEXT("deepest_negative_domain_admission"), 0, EOracle::Unknown, EOracle::AdmissionRejected);
    NegativeDomain.bSweep = false; NegativeDomain.Query.DomainTolerance = -1;
    FProbe& NonFinite = Add(TEXT("deepest_nonfinite_admission"), 0, EOracle::Unknown, EOracle::AdmissionRejected);
    NonFinite.bSweep = false; NonFinite.Query.Start.X = std::numeric_limits<double>::quiet_NaN();
    FProbe& Overflow = Add(TEXT("finite_overflow_requested"), 0, EOracle::Unknown, EOracle::Unknown);
    Overflow.Query.HalfExtent = FVector3d(1.e200, 1.e200, 1.e200);
    Overflow.Query.Start = FVector3d(0, 0, 2.e200); Overflow.Query.End = FVector3d(0, 0, -2.e200);

    int32 ExpectedPairs = 0;
    for (const FProbe& P : Probes) ExpectedPairs += P.bSweep ? 4 : 2;
    AddInfo(FString::Printf(TEXT("[ExactCrossNormal784Begin] Probes=%d ExpectedPairs=%d ExpectedPublicCalls=%d ExactFields=21"), Probes.Num(), 2 * ExpectedPairs, 3 * ExpectedPairs));
    int32 PairedCalls = 0, OracleChecks = 0, UnknownOracles = 0;
    for (const FProbe& Probe : Probes)
    {
        FAnalyticWorldData Data; FString Reason;
        if (!BuildWorld(Data, Probe.World, Reason)) { AddError(Probe.Name + TEXT(": fixture: ") + Reason); return false; }
        for (int32 LazyMode : {0, 1})
        for (int32 Operation : {0, 1})
        {
            if (Operation == 0 && !Probe.bSweep) continue;
            FWorldHit Results[3]; bool Available[3] = {true, true, true};
            for (int32 Mode : {0, 1, 2})
            {
                if (!LazyLease.Set(LazyMode) || !ScopeLease.Set(Mode))
                { AddError(TEXT("effective flag differs from requested mode")); return false; }
                FWorldQuery Q = Probe.Query; // Fresh query and service for EVERY public call.
                if (Operation == 0) Results[Mode] = FWorldQueryService(Data).Sweep(Q);
                else
                {
                    Q.End = Q.Start;
                    if (Probe.SweepOracle == EOracle::FlatQuarterSweep) Q.Start.Z = Q.End.Z = 1;
                    Available[Mode] = FWorldQueryService(Data).TryFindDeepestOverlap(Q, Results[Mode]);
                }
                const EOracle Oracle = Operation == 0 ? Probe.SweepOracle : Probe.PoseOracle;
                if (Oracle == EOracle::Unknown) ++UnknownOracles;
                else
                {
                    ++OracleChecks;
                    if (!ExpectedWitness(Oracle, Available[Mode], Results[Mode]))
                    { AddError(FString::Printf(TEXT("FIRST baselineExpectedWitness divergence case=%s Lazy=%d Scope=%d API=%d"), *Probe.Name, LazyMode, Mode, Operation)); return false; }
                }
            }
            for (int32 Mode : {1, 2})
            {
                ++PairedCalls;
                const TCHAR* Field = FirstDifference(Results[0], Results[Mode]);
                if (Available[0] != Available[Mode] || Field)
                { AddError(FString::Printf(TEXT("FIRST exact divergence case=%s Lazy=%d API=%d Mode=%d field=%s"), *Probe.Name, LazyMode, Operation, Mode, Field ? Field : TEXT("operationAvailable"))); return false; }
            }
        }
    }
    const bool ScopeRestored = ScopeLease.Restore();
    const bool LazyRestored = LazyLease.Restore();
    const bool Restored = ScopeRestored && LazyRestored;
    if (!Restored) { AddError(TEXT("CVar value/priority/flags restoration failed")); return false; }
    if (PairedCalls != 2 * ExpectedPairs || Probes.Num() != 23 || ExpectedPairs != 86 || OracleChecks != 108 || UnknownOracles != 150)
    { AddError(TEXT("784 incomplete public call/oracle coverage")); return false; }
    AddInfo(FString::Printf(TEXT("[ExactCrossNormal784End] Status=PUBLIC_EQUIVALENCE_PASS Probes=%d Pairs=%d PublicCalls=%d ExactFields=21 IndependentOracleChecks=%d UNKNOWN_oracles=%d Restored=1 PrivatePrimarySlots=UNKNOWN PrivateReuse=UNKNOWN TwoTriangleVisit=UNKNOWN ThresholdBranch=UNKNOWN OverflowFallback=UNKNOWN Timing=UNKNOWN"), Probes.Num(), PairedCalls, 3 * ExpectedPairs, OracleChecks, UnknownOracles));
    return true;
}
#endif
