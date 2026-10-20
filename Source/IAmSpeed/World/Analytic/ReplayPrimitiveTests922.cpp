#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "ExactCrossReplayKernels913.h"
#include "ReplayTypedClock918.h"
#include "ReplayWorkBudget921.h"
#include "Misc/AutomationTest.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FReplayPrimitives922Test,
    "IAmSpeed.AnalyticWorld.ReplayPrimitives922",
    (EAutomationTestFlags::EditorContext | EAutomationTestFlags::ClientContext |
     EAutomationTestFlags::ServerContext | EAutomationTestFlags::CommandletContext) |
    EAutomationTestFlags::EngineFilter)
bool FReplayPrimitives922Test::RunTest(const FString& Parameters)
{
    using namespace Speed::Analytic;
    using namespace Speed::Analytic::Private;
    FWorldQuery Q;Q.Shape=EQueryShape::Box;
    FBoxSweepContext X(Q);X.bLazyExtrudedSatAxes=true;
    FExactCrossReplayKernels913::FLookupShadow Shadow;
    FExactCrossNormalScope Actual(Q,X,false);
    for(int32 I=0;I<512;++I)
    {
        const FVector3d Cross(I%7==0?-0.0:0.0,double(I%5+1),4.0);
        const double Squared=Cross.SquaredLength();
        const FVector3d Original=Cross/FMath::Sqrt(Squared);
        FExactCrossReplayKernels913::FPreparedRow R{Cross,Squared,I%3,Original};
        if(!FExactCrossNormalScope::SameBits(Original,Shadow.Lookup(R)) ||
           !FExactCrossNormalScope::SameBits(Original,Actual.Normalize(Cross,Squared,R.Axis)))
        { AddError(TEXT("922 shadow and real helper must return original bits across signedzero, axis partitions and eviction"));return false; }
    }
    FReplayClockRead918 A,B;
    A.Kind=B.Kind=EReplayClock918::QpcElapsed;A.Thread=B.Thread=7;
    A.bValid=B.bValid=true;A.Count=100;B.Count=150;
    A.QpcFrequency=B.QpcFrequency=10000000;
    auto D=DifferenceReplayClock918(A,B);
    if(!D.bValid || D.Raw!=50 || D.QpcFrequency!=10000000) return AddError(TEXT("922 QPC typed delta")),false;
    B.Thread=8;if(DifferenceReplayClock918(A,B).bValid) return AddError(TEXT("922 reject foreign thread")),false;
    B.Thread=7;B.Count=99;if(DifferenceReplayClock918(A,B).bValid) return AddError(TEXT("922 reject underflow")),false;
    B.Count=150;B.QpcFrequency=1;if(DifferenceReplayClock918(A,B).bValid) return AddError(TEXT("922 reject changing frequency")),false;
    A.Kind=B.Kind=EReplayClock918::ThreadCycleAccounting;A.QpcFrequency=B.QpcFrequency=0;
    D=DifferenceReplayClock918(A,B);
    if(!D.bValid || D.Raw!=50 || D.QpcFrequency!=0) return AddError(TEXT("922 CPU-cycle counts stay unconverted")),false;
    A.QpcFrequency=1;if(DifferenceReplayClock918(A,B).bValid) return AddError(TEXT("922 reject invented cycle frequency")),false;
    A.QpcFrequency=0;B.bValid=false;if(DifferenceReplayClock918(A,B).bValid) return AddError(TEXT("922 API failure remains invalid")),false;
    FReplayWorkBudget921 Budget;
    if(Budget.TryReserve(0,1) || Budget.TryReserve(513,1) || Budget.TryReserve(1,65537) || Budget.ReadRemaining()!=FReplayWorkBudget921::Limit)
        return AddError(TEXT("922 invalid requests must not consume budget")),false;
    if(!Budget.TryReserve(512,32768) || Budget.ReadRemaining()!=0 || Budget.TryReserve(1,1))
        return AddError(TEXT("922 exact budget exhaustion has no reset or increase")),false;
    // No ReadReplayClock call or timed workload. Unit evidence is not runtime cost.
    UE_LOG(LogTemp,Display,TEXT("[ReplayPrimitives922] Rows=512 Bits=PASS TypedDeltas=PASS WorkBudget=PASS ClockAPIsCalled=0 TimedWork=0 SyntheticOnly=1 RuntimeCost=UNKNOWN"));
    return true;
}
#endif
