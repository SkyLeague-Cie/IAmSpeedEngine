#if WITH_DEV_AUTOMATION_TESTS
#include "AnalyticNumericContext.h"
#include "AnalyticWorldQuery.h"
#include "IAmSpeed/World/Collision/StaticCollisionWorld.h"
#include "IAmSpeed/World/Simulation/FrameFloatingPointContext.h"
#include "Misc/AutomationTest.h"
#include "HAL/IConsoleManager.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

namespace
{
using namespace Speed::Analytic;
constexpr auto RequiredNumeric=EAnalyticNumericContract::Binary64NearestAllowFlush;
FAnalyticWorldData GuardTestWorld()
{
    // Independent finite-plane unit fixture. No stadium asset or authority is changed.
    FAnalyticWorldData World;
    World.NumericContract=RequiredNumeric;
    FBoundedPlane Plane;
    Plane.SourceId=1; Plane.SurfaceId=2; Plane.FeatureId=3; Plane.PrimitiveId=4;
    Plane.HalfExtents=FVector2d(100,100); Plane.BlockingChannels=MAX_uint64;
    Plane.bAuthorityEligible=true;
    World.Planes.Add(Plane);
    return World;
}
FWorldQuery GuardTestQuery()
{
    FWorldQuery Q;
    Q.Shape=EQueryShape::Sphere; Q.Radius=2;
    Q.Start=FVector3d(0,0,20); Q.End=FVector3d(0,0,-20);
    return Q;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticNumericModesTest,
    "IAmSpeed.AnalyticWorld.NumericContext.Modes",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FAnalyticNumericModesTest::RunTest(const FString&)
{
    using namespace Speed;
    using namespace Speed::Analytic;
    FAnalyticNumericEvidence Good;
    Good.bHardwareAvailable=true; Good.VectorControl=Good.SnapshotVectorControl=0x1f80u;
    Good.ScalarControl=0x037fu;
    for (uint32 Flush : {0u,0x40u,0x8000u,0x8040u})
    {
        auto E=Good; E.VectorControl|=Flush; E.SnapshotVectorControl=E.VectorControl;
        TestTrue(TEXT("certificate explicitly permits every FTZ/DAZ combination"),
            ClassifyAnalyticNumericContext(RequiredNumeric,E).Status==EAnalyticNumericStatus::Complete);
        if (Flush) TestTrue(TEXT("flush controls are part of cache identity"),E.ControlFingerprint()!=Good.ControlFingerprint());
    }
    for (uint32 Rounding=1; Rounding<4; ++Rounding)
    {
        auto E=Good; E.VectorControl|=Rounding<<13; E.SnapshotVectorControl=E.VectorControl;
        TestTrue(TEXT("each non-nearest SSE mode is unavailable"),
            ClassifyAnalyticNumericContext(RequiredNumeric,E).Failure==EAnalyticNumericFailure::VectorRounding);
        E=Good; E.ScalarControl=(E.ScalarControl & ~0x0c00u)|(Rounding<<10);
        TestTrue(TEXT("each independent non-nearest x87 mode is unavailable"),
            ClassifyAnalyticNumericContext(RequiredNumeric,E).Failure==EAnalyticNumericFailure::ScalarRounding);
    }
    for (uint32 Bit=0; Bit<6; ++Bit)
    {
        auto E=Good; E.VectorControl&=~(1u<<(Bit+7)); E.SnapshotVectorControl=E.VectorControl;
        TestTrue(TEXT("every SSE exception mask is required"),
            ClassifyAnalyticNumericContext(RequiredNumeric,E).Failure==EAnalyticNumericFailure::UnmaskedExceptions);
        E=Good; E.ScalarControl&=~(1u<<Bit);
        TestTrue(TEXT("every independent x87 exception mask is required"),
            ClassifyAnalyticNumericContext(RequiredNumeric,E).Failure==EAnalyticNumericFailure::UnmaskedExceptions);
    }
    auto Sticky=Good; Sticky.VectorControl|=0x3fu; Sticky.SnapshotVectorControl=Sticky.VectorControl;
    TestEqual(TEXT("sticky flags are not an arithmetic-mode cache key"),Sticky.ControlFingerprint(),Good.ControlFingerprint());
    auto Bad=Good; Bad.bHardwareAvailable=false;
    TestTrue(TEXT("unknown hardware cannot be admitted"),ClassifyAnalyticNumericContext(RequiredNumeric,Bad).Status==EAnalyticNumericStatus::Unknown);
    Bad=Good; Bad.SnapshotVectorControl^=0x40u;
    TestTrue(TEXT("inconsistent reads cannot be admitted"),ClassifyAnalyticNumericContext(RequiredNumeric,Bad).Failure==EAnalyticNumericFailure::InconsistentVectorRead);
    TestTrue(TEXT("unknown contract never means legacy"),ClassifyAnalyticNumericContext(static_cast<EAnalyticNumericContract>(255),Good).Status==EAnalyticNumericStatus::Unknown);
    TestTrue(TEXT("unrelated legacy policy remains unspecified"),ClassifyAnalyticNumericContext(EAnalyticNumericContract::Unspecified,Bad).Status==EAnalyticNumericStatus::NotApplicable);
    const auto Original=CaptureFrameFloatingPointContext();
    if (!TestTrue(TEXT("hardware fixture can preserve original context"),Original.bValid)) return false;
    FScopedFrameFloatingPointContext Restore(Original);
    std::fenv_t Held; if (!TestEqual(TEXT("mask traps for intentional mode changes"),std::feholdexcept(&Held),0)) return false;
    TestEqual(TEXT("nearest fixture"),std::fesetround(FE_TONEAREST),0);
    const uint32 Before=VectorGetControlRegister();
    const auto Observed=InspectCurrentAnalyticNumericContext(RequiredNumeric);
    const uint32 After=VectorGetControlRegister();
    TestTrue(TEXT("actual nearest hardware accepted"),Observed.Status==EAnalyticNumericStatus::Complete);
    TestEqual(TEXT("predicate does not change control or sticky flags"),After,Before);
    for (int Mode : {FE_UPWARD,FE_DOWNWARD,FE_TOWARDZERO})
    {
        TestEqual(TEXT("alternate hardware mode applies"),std::fesetround(Mode),0);
        TestTrue(TEXT("actual non-nearest hardware refused"),InspectCurrentAnalyticNumericContext(RequiredNumeric).Status==EAnalyticNumericStatus::Unknown);
    }
    std::fesetround(FE_TONEAREST);
    const uint32 BeforeTransferRefusal=VectorGetControlRegister();
    bool TypedRestoreFailure=false;
    try { RequireAnalyticNumericTransfer(RequiredNumeric,false,EAnalyticNumericFailure::ContextRestoration); }
    catch (const FAnalyticNumericUnknown& E) { TypedRestoreFailure=E.Evidence.Failure==EAnalyticNumericFailure::ContextRestoration; }
    TestTrue(TEXT("unavailable transfer preserves its explicit unknown reason"),TypedRestoreFailure);
    TestEqual(TEXT("refusal does not normalize the hardware context"),VectorGetControlRegister(),BeforeTransferRefusal);
    return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticNumericCacheRefusalTest,
    "IAmSpeed.AnalyticWorld.NumericContext.CachedHitRefusal",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FAnalyticNumericCacheRefusalTest::RunTest(const FString&)
{
    using namespace Speed;
    using namespace Speed::Analytic;
    const auto Original=CaptureFrameFloatingPointContext();
    if (!TestTrue(TEXT("original hardware captured"),Original.bValid)) return false;
    FScopedFrameFloatingPointContext Restore(Original);
    std::fenv_t Held;
    if (!TestEqual(TEXT("mask traps for controlled fixture"),std::feholdexcept(&Held),0) ||
        !TestEqual(TEXT("nearest fixture mode"),std::fesetround(FE_TONEAREST),0)) return false;
    auto World=GuardTestWorld(); FString Reason;
    if (!TestTrue(TEXT("independent plane finalizes"),World.FinalizeAndValidate(&Reason))) return false;
    FWorldQueryService Service(World); auto Query=GuardTestQuery();
    for (bool AuthorityOnly : {true,false})
    {
        Query.bAuthorityOnly=AuthorityOnly;
        TestTrue(TEXT("nearest query populates an actual cached contact"),Service.Sweep(Query).bHit);
        std::fesetround(FE_UPWARD);
        bool Refused=false;
        try { (void)Service.Sweep(Query); }
        catch (const FAnalyticNumericUnknown& Unknown) { Refused=Unknown.Evidence.Status==EAnalyticNumericStatus::Unknown; }
        std::fesetround(FE_TONEAREST);
        TestTrue(TEXT("incompatible caller never receives cached hit, including strict backend queries with authorityOnly false"),Refused);
    }
    int32 Visits=0; bool CallbackRefused=false;
    try { Service.VisitPlanarCandidates(Query,[&](const FWorldHit&) {++Visits; std::fesetround(FE_DOWNWARD);}); }
    catch (const FAnalyticNumericUnknown&) { CallbackRefused=true; }
    std::fesetround(FE_TONEAREST);
    TestEqual(TEXT("callback executed exactly once"),Visits,1);
    TestTrue(TEXT("external callback cannot leave geometry running under an incompatible mode"),CallbackRefused);
    return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticNumericBatchRefusalTest,
    "IAmSpeed.AnalyticWorld.NumericContext.BatchOutputRefusal",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FAnalyticNumericBatchRefusalTest::RunTest(const FString&)
{
    using namespace Speed;
    using namespace Speed::Analytic;
    const auto Original=CaptureFrameFloatingPointContext();
    if (!TestTrue(TEXT("original hardware captured"),Original.bValid)) return false;
    FScopedFrameFloatingPointContext Restore(Original);
    std::fenv_t Held;
    if (!TestEqual(TEXT("mask traps for controlled fixture"),std::feholdexcept(&Held),0) ||
        !TestEqual(TEXT("nearest fixture mode"),std::fesetround(FE_TONEAREST),0)) return false;
    auto World=GuardTestWorld(); FString Reason;
    if (!TestTrue(TEXT("independent batch world finalizes"),World.FinalizeAndValidate(&Reason))) return false;
    const FAnalyticStaticCollisionWorld StaticWorld(World,1);
    TArray<FWorldQuery> Queries={GuardTestQuery(),GuardTestQuery()};
    TArray<FWorldHit> Outputs; Outputs.SetNum(2);
    Outputs[0].PrimitiveId=701; Outputs[1].PrimitiveId=702;
    std::fesetround(FE_TOWARDZERO); bool Refused=false;
    try { (void)StaticWorld.SweepIndependentBatch(Queries,Outputs); }
    catch (const FAnalyticNumericUnknown&) { Refused=true; }
    std::fesetround(FE_TONEAREST);
    TestTrue(TEXT("batch exposes typed unknown rather than miss or false fallback"),Refused);
    TestEqual(TEXT("first caller output remains untouched"),Outputs[0].PrimitiveId,uint64(701));
    TestEqual(TEXT("second caller output remains untouched"),Outputs[1].PrimitiveId,uint64(702));
    return !HasAnyErrors();
}

namespace
{
FAutoConsoleCommand NumericGuardUnits(TEXT("Speed.Analytic.RunNumericGuardUnits"),
    TEXT("Run three existing-framework numeric guard tests in an isolated commandlet."),
    FConsoleCommandDelegate::CreateLambda([]()
    {
        static bool Used=false;
        auto& Framework=FAutomationTestFramework::Get();
        if (!IsRunningCommandlet() || !FParse::Param(FCommandLine::Get(),TEXT("AnalyticNumericGuardUnits")) ||
            Used || GIsAutomationTesting || GIsSlowTask || GIsPlayInEditorWorld ||
            Framework.GetCurrentTest() || !Framework.IsLatentCommandQueueEmpty())
        { UE_LOG(LogTemp,Error,TEXT("Analytic numeric unit commandlet refused")); return; }
        Used=true;
        const TCHAR* Classes[]={TEXT("FAnalyticNumericModesTest"),TEXT("FAnalyticNumericCacheRefusalTest"),TEXT("FAnalyticNumericBatchRefusalTest")};
        const TCHAR* Names[]={TEXT("IAmSpeed.AnalyticWorld.NumericContext.Modes"),TEXT("IAmSpeed.AnalyticWorld.NumericContext.CachedHitRefusal"),TEXT("IAmSpeed.AnalyticWorld.NumericContext.BatchOutputRefusal")};
        for (int32 I=0; I<3; ++I)
        {
            if (GIsAutomationTesting || GIsSlowTask || GIsPlayInEditorWorld || Framework.GetCurrentTest() ||
                !Framework.IsLatentCommandQueueEmpty() || !Framework.ContainsTest(Classes[I]))
            { UE_LOG(LogTemp,Error,TEXT("Analytic numeric unit ownership refused")); return; }
            Framework.SetRequestedTestFilter(EAutomationTestFlags::EngineFilter);
            Framework.StartTestByName(Classes[I],0,Names[I]);
            if (!GIsAutomationTesting || !Framework.GetCurrentTest() || Framework.GetCurrentTestFullPath()!=Names[I] || !Framework.IsLatentCommandQueueEmpty())
            { UE_LOG(LogTemp,Error,TEXT("Analytic numeric unit synchronous ownership refused")); return; }
            FAutomationTestExecutionInfo Info;
            const bool Stopped=Framework.StopTest(Info);
            const bool Passed=Stopped && Info.bSuccessful && Info.GetErrorTotal()==0 && Info.GetWarningTotal()==0;
            UE_LOG(LogTemp,Display,TEXT("ANALYTIC_NUMERIC_UNIT name=%s passed=%d errors=%d warnings=%d"),Names[I],Passed,Info.GetErrorTotal(),Info.GetWarningTotal());
        }
    }));
}
#endif
