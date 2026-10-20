#pragma once
#include "ExactCrossReplayKernels913.h"
#include "ReplayTypedClock918.h"
#include "ReplayWorkBudget921.h"
#include "AnalyticCaseSatDiagnostics.h"
#include "HAL/PlatformProcess.h"
#include "Containers/StringConv.h"
#if !UE_BUILD_SHIPPING
namespace Speed::Analytic::Private
{
#define SL_REPLAY_LOG925(...) FReplayProbe925::Emit(FString::Printf(__VA_ARGS__))
struct FReplayProbe925 final
{
    using K=FExactCrossReplayKernels913;
    using Identity=CaseSatDiagnostics::FOwnedForwardIdentity917;
    struct FState
    {
        std::atomic<uint64> Accepted{0},Denied{0},Unavailable{0},DroppedLogRows{0},Footers{0};
        std::atomic<bool> Enabled{false},Overflow{false};
    };
    static FState& State(){static FState S;return S;}
    static void Increment(std::atomic<uint64>& Counter)
    {
        uint64 C=Counter.load(std::memory_order_relaxed);
        while(C!=MAX_uint64)
            if(Counter.compare_exchange_weak(C,C+1,std::memory_order_relaxed)) return;
        State().Overflow.store(true,std::memory_order_relaxed);
    }
    static void Emit(const FString& Line)
    {
        // Bound every formatted diagnostic payload before passing it to the logger.
        // At most32*(512input+128guard+100block+2boundary)+64footer rows/process.
        if(FTCHARToUTF8(*Line).Length()>2048) { Increment(State().DroppedLogRows);return; }
        UE_LOG(LogTemp,Display,TEXT("%s"),*Line);
    }
    static bool Claim(uint64& OutReport)
    {
        State().Enabled.store(true,std::memory_order_relaxed);
        uint64 C=State().Accepted.load(std::memory_order_relaxed);
        while(C<32)
            if(State().Accepted.compare_exchange_weak(C,C+1,std::memory_order_relaxed)) { OutReport=C+1;return true; }
        Increment(State().Denied);return false;
    }
    static void Unavailable(){State().Enabled.store(true,std::memory_order_relaxed);Increment(State().Unavailable);}
    static const TCHAR* Name(K::EKernel Kernel)
    {
        switch(Kernel){case K::EKernel::Payload:return TEXT("Payload");case K::EKernel::Normalization:return TEXT("Normalization");case K::EKernel::LookupShadow:return TEXT("LookupShadow");default:return TEXT("ProductionReuse");}
    }
    static uint64 Work(K::EKernel Kernel,const TArray<K::FPreparedRow>& Rows,
        const FWorldQuery& Q,const FBoxSweepContext& X,uint32 Repeats)
    {
        switch(Kernel)
        {
        case K::EKernel::Payload:return K::Batch<K::EKernel::Payload>(Rows,Q,X,Repeats);
        case K::EKernel::Normalization:return K::Batch<K::EKernel::Normalization>(Rows,Q,X,Repeats);
        case K::EKernel::LookupShadow:return K::Batch<K::EKernel::LookupShadow>(Rows,Q,X,Repeats);
        default:return K::Batch<K::EKernel::ProductionReuse>(Rows,Q,X,Repeats);
        }
    }
    struct FMeasured
    {
        FReplayClockRead918 A,B;
        FReplayClockDelta918 Delta;
    };
    static FMeasured Measure(K::EKernel Kernel,const TArray<K::FPreparedRow>& Rows,
        const FWorldQuery& Q,const FBoxSweepContext& X,uint32 Repeats,EReplayClock918 Clock)
    {
        FMeasured M;M.A=ReadReplayClock918(Clock);
        volatile uint64 Sink=Work(Kernel,Rows,Q,X,Repeats);(void)Sink;
        M.B=ReadReplayClock918(Clock);M.Delta=DifferenceReplayClock918(M.A,M.B);return M;
    }
    static void Block(const Identity& I,uint64 Provider,uint32 Family,uint32 BlockIndex,
        bool Calibration,K::EKernel Kernel,uint32 Rows,uint32 Repeats,
        uint64 EmptyMax,const FMeasured& M)
    {
        SL_REPLAY_LOG925(TEXT("[ReplayBlock925] Run=%s Driver=%llu Epoch=%llu Case=%d Batch=%d WorldGeneration=%llu Frame=%llu Thread=%u PID=%u Provider=%llu Family=%u Block=%u Calibration=%u Kernel=%s Clock=%s Rows=%u Repeats=%u Begin=%llu End=%llu Frequency=%llu Valid=%u SameThread=%u EmptyMax=%llu Scope=RecordedInputs FrameStatus=InProgress LiveShare=UNKNOWN"),
            I.RunUUID,I.DriverKey,I.Epoch,I.NativeCaseId,I.BatchIndex,I.WorldGeneration,I.CanonicalFrame,I.ThreadId,FPlatformProcess::GetCurrentProcessId(),Provider,Family,BlockIndex,uint32(Calibration),Name(Kernel),M.A.Kind==EReplayClock918::QpcElapsed?TEXT("QPC"):TEXT("QueryThreadCycleTime"),Rows,Repeats,M.A.Count,M.B.Count,M.A.QpcFrequency,uint32(M.Delta.bValid),uint32(M.A.Thread==I.ThreadId && M.B.Thread==I.ThreadId),EmptyMax);
    }
    static bool Above(const FMeasured& M,uint64 EmptyMax)
    {
        // No multiplication overflow and no integer truncation at the 2ms threshold.
        return M.Delta.bValid && M.Delta.QpcFrequency>0 && EmptyMax<=MAX_uint64/1000 &&
            M.Delta.Raw>EmptyMax*1000 &&
            M.Delta.Raw>=M.Delta.QpcFrequency/500+(M.Delta.QpcFrequency%500!=0?1:0);
    }
    static uint64 Empty(EReplayClock918 Clock,bool& Valid,const Identity& I,uint64 Provider)
    {
        uint64 Maximum=0;Valid=true;
        for(int32 N=0;N<64;++N)
        {
            const auto A=ReadReplayClock918(Clock);const auto B=ReadReplayClock918(Clock);
            const auto D=DifferenceReplayClock918(A,B);Valid=Valid && D.bValid;
            SL_REPLAY_LOG925(TEXT("[ReplayGuard925] Thread=%u Provider=%llu Clock=%s Index=%d Begin=%llu End=%llu Frequency=%llu Valid=%u"),I.ThreadId,Provider,Clock==EReplayClock918::QpcElapsed?TEXT("QPC"):TEXT("QueryThreadCycleTime"),N,A.Count,B.Count,A.QpcFrequency,uint32(D.bValid));
            if(D.bValid) Maximum=FMath::Max(Maximum,D.Raw);
        }
        return Maximum;
    }
    static bool Family(const Identity& I,uint64 Provider,uint32 FamilyId,
        K::EKernel First,K::EKernel Second,const TArray<K::FPreparedRow>& Rows,
        const FWorldQuery& Q,const FBoxSweepContext& X,uint64 QpcEmpty,uint64 CycleEmpty)
    {
        auto& Budget=ProcessReplayWorkBudget921();uint32 Repeats=1,Round=0;bool Resolved=false;
        for(;;)
        {
            // Reserve BOTH calibration blocks before either runs.
            if(!Budget.TryReserveWork(uint64(Rows.Num())*Repeats*2)) return false;
            const auto A=Measure(First,Rows,Q,X,Repeats,EReplayClock918::QpcElapsed);
            const auto B=Measure(Second,Rows,Q,X,Repeats,EReplayClock918::QpcElapsed);
            Block(I,Provider,FamilyId,Round*2,true,First,Rows.Num(),Repeats,QpcEmpty,A);
            Block(I,Provider,FamilyId,Round*2+1,true,Second,Rows.Num(),Repeats,QpcEmpty,B);
            ++Round;Resolved=Above(A,QpcEmpty) && Above(B,QpcEmpty);
            if(Resolved || Repeats==65536) break;
            Repeats*=2;
        }
        if(!Resolved) return false;
        // ABBA then BAAB, once per clock. Same work/repeats within this family.
        // Reserve all16 assessment blocks atomically. Never promote a partial subset.
        if(!Budget.TryReserveWork(uint64(Rows.Num())*Repeats*16)) return false;
        const bool Order[8]={false,true,true,false,true,false,false,true};
        for(uint32 Pass=0;Pass<2;++Pass)
            for(uint32 B=0;B<8;++B)
            {
                const auto Clock=Pass==0?EReplayClock918::QpcElapsed:EReplayClock918::ThreadCycleAccounting;
                const auto Kernel=Order[B]?Second:First;
                const auto M=Measure(Kernel,Rows,Q,X,Repeats,Clock);
                Block(I,Provider,FamilyId,Pass*8+B,false,Kernel,Rows.Num(),Repeats,Pass==0?QpcEmpty:CycleEmpty,M);
            }
        return true;
    }
    static void Report(const FExactCrossNormalCapture844& Capture,const FWorldQuery& Q,
        const FBoxSweepContext& X,const Identity& I,uint64 Provider)
    {
        // Claim is made before the bounded512-row capture. At most32 reports/process.
        TArray<K::FPreparedRow> Rows;Rows.Reserve(Capture.Rows.Num());
        const bool Verified=Capture.Rows.Num()>0 && ProcessReplayWorkBudget921().TryReserveWork(uint64(Capture.Rows.Num())*3);
        uint64 Mismatches=0;FExactCrossNormalScope Actual(Q,X,false);K::FLookupShadow Shadow;
        for(const auto& R:Capture.Rows)
        {
            const FVector3d Original=Verified?R.Cross/FMath::Sqrt(R.Squared):FVector3d::ZeroVector;
            K::FPreparedRow Prepared{R.Cross,R.Squared,R.Axis,Original};
            if(Verified && (!FExactCrossNormalScope::SameBits(Original,Actual.Normalize(R.Cross,R.Squared,R.Axis)) ||
               !FExactCrossNormalScope::SameBits(Original,Shadow.Lookup(Prepared)))) ++Mismatches;
            Rows.Add(Prepared);
        }
        SL_REPLAY_LOG925(TEXT("[ReplayBegin925] Run=%s Driver=%llu Epoch=%llu Case=%d Batch=%d WorldGeneration=%llu Frame=%llu Thread=%u PID=%u Provider=%llu Rows=%d Dropped=%llu Mismatches=%llu Complete=%u Verified=%u Scope=RecordedInputs FrameStatus=InProgress FullCoverage=UNKNOWN ProviderMeaning=AcceptedProcessReport"),I.RunUUID,I.DriverKey,I.Epoch,I.NativeCaseId,I.BatchIndex,I.WorldGeneration,I.CanonicalFrame,I.ThreadId,FPlatformProcess::GetCurrentProcessId(),Provider,Rows.Num(),Capture.Dropped,Mismatches,uint32(Capture.IsComplete()),uint32(Verified));
        for(int32 N=0;N<Rows.Num();++N)
        {
            const auto& R=Rows[N];
            SL_REPLAY_LOG925(TEXT("[ReplayInput925] Thread=%u Provider=%llu Index=%d Axis=%d X=%llu Y=%llu Z=%llu Squared=%llu UnitX=%llu UnitY=%llu UnitZ=%llu"),I.ThreadId,Provider,N,R.Axis,FExactCrossNormalScope::Bits(R.Cross.X),FExactCrossNormalScope::Bits(R.Cross.Y),FExactCrossNormalScope::Bits(R.Cross.Z),FExactCrossNormalScope::Bits(R.Squared),FExactCrossNormalScope::Bits(R.OriginalUnit.X),FExactCrossNormalScope::Bits(R.OriginalUnit.Y),FExactCrossNormalScope::Bits(R.OriginalUnit.Z));
        }
        bool Primary=false,Secondary=false,QpcValid=false,CycleValid=false;
        uint64 QpcEmpty=0,CycleEmpty=0;
        if(Capture.IsComplete() && Rows.Num()>0 && Mismatches==0 && Verified)
        {
            QpcEmpty=Empty(EReplayClock918::QpcElapsed,QpcValid,I,Provider);
            CycleEmpty=Empty(EReplayClock918::ThreadCycleAccounting,CycleValid,I,Provider);
            if(QpcValid && CycleValid)
            {
                Primary=Family(I,Provider,0,K::EKernel::Normalization,K::EKernel::ProductionReuse,Rows,Q,X,QpcEmpty,CycleEmpty);
                if(Primary) Secondary=Family(I,Provider,1,K::EKernel::Payload,K::EKernel::LookupShadow,Rows,Q,X,QpcEmpty,CycleEmpty);
            }
        }
        SL_REPLAY_LOG925(TEXT("[ReplayEnd925] Thread=%u Provider=%llu Rows=%d PrimaryComplete=%u SecondaryComplete=%u QpcGuardsValid=%u CycleGuardsValid=%u QpcEmpty=%llu CycleEmpty=%llu BudgetRemaining=%llu LiveShare=UNKNOWN Gain=UNKNOWN"),I.ThreadId,Provider,Rows.Num(),uint32(Primary),uint32(Secondary),uint32(QpcValid),uint32(CycleValid),QpcEmpty,CycleEmpty,ProcessReplayWorkBudget921().ReadRemaining());
    }
    static void Flush(const CaseSatDiagnostics::FCaseIdentity& C)
    {
        if(!State().Enabled.load(std::memory_order_relaxed)) return;
        uint64 Footer=State().Footers.load(std::memory_order_relaxed);
        for(;;) { if(Footer>=64) return; if(State().Footers.compare_exchange_weak(Footer,Footer+1,std::memory_order_relaxed)) break; } // Missing case footer remains UNKNOWN, never a success.
        SL_REPLAY_LOG925(TEXT("[ReplayTotals925] Run=%s Driver=%llu Epoch=%llu Case=%d Batch=%d WorldGeneration=%llu Accepted=%llu Denied=%llu Unavailable=%llu Overflow=%u DroppedLogRows=%llu BudgetRemaining=%llu CaptureLimit=32 WorkLimit=16777216 CounterScope=ProcessSnapshot UnselectedCoverage=UNKNOWN"),*C.RunUUID,C.DriverKey,C.Epoch,C.NativeCaseId,C.BatchIndex,C.WorldGeneration,State().Accepted.load(std::memory_order_relaxed),State().Denied.load(std::memory_order_relaxed),State().Unavailable.load(std::memory_order_relaxed),uint32(State().Overflow.load(std::memory_order_relaxed)),State().DroppedLogRows.load(std::memory_order_relaxed),ProcessReplayWorkBudget921().ReadRemaining());
    }
};
#undef SL_REPLAY_LOG925
}
#endif
