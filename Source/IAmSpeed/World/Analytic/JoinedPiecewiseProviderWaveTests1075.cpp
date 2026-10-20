#if WITH_DEV_AUTOMATION_TESTS
#include "JoinedPiecewiseProviderWave1075.h"
#include "IAmSpeed/World/Collision/StaticCollisionWorld.h"
#include "IAmSpeed/World/Simulation/FrameParallelExecutor.h"
#include "IAmSpeed/World/Simulation/FrameParallelSettings.h"
#include "HAL/IConsoleManager.h"
#include "Misc/AutomationTest.h"
#include <atomic>
#include <limits>
#include <stdexcept>

namespace Speed::Analytic::ParallelTests1075
{bool SameHit(const FWorldHit&,const FWorldHit&);bool PiecewiseWorld(FAnalyticWorldData&,FString&);}
namespace Speed::Analytic::ProviderTests1075
{
struct FScopedInt
{
    IConsoleVariable* Variable;int32 Previous;
    FScopedInt(const TCHAR* Name,int32 Value):Variable(IConsoleManager::Get().FindConsoleVariable(Name)),Previous(Variable?Variable->GetInt():0)
    {if(Variable)Variable->Set(Value,ECVF_SetByCode);}
    ~FScopedInt(){if(Variable)Variable->Set(Previous,ECVF_SetByCode);}
    bool Is(int32 Value)const{return Variable&&Variable->GetInt()==Value;}
};
bool SameNumericProviderTests1075(const FFrameFloatingPointContext& A,const FFrameFloatingPointContext& B)
{return A.bValid&&B.bValid&&A.VectorControl==B.VectorControl&&FMemory::Memcmp(&A.Environment,&B.Environment,sizeof(std::fenv_t))==0;}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedProviderWaveWholeProviderOracle1075,
    "IAmSpeed.Parallel.Analytic1075.ProviderWaveWholeProviderOracle",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FIAmSpeedProviderWaveWholeProviderOracle1075::RunTest(const FString& Parameters)
{
    using namespace Speed;using namespace Analytic;using namespace ParallelTests1075;
    ProviderTests1075::FScopedInt Enabled(TEXT("p.IAmSpeed.Parallel.PiecewiseProviderWave1075"),1);
    ProviderTests1075::FScopedInt Observe(TEXT("p.IAmSpeed.Parallel.PiecewiseProviderObserve1075"),1);
    ProviderTests1075::FScopedInt CausalOff(TEXT("p.IAmSpeed.Parallel.AnalyticCausal1075"),0);
    if(!Enabled.Is(1)||!Observe.Is(1)||!CausalOff.Is(0))return false;
    FAnalyticWorldData Data;FString Reason;
    if(!PiecewiseWorld(Data,Reason)){AddError(Reason);return false;}
    TArray<FWorldQuery> Queries;
    for(int32 I=0;I<24;++I)
    {
        FWorldQuery Q;Q.Shape=EQueryShape(I%3);Q.Radius=3;Q.HalfExtent=FVector3d(5,7,4);
        Q.Start=FVector3d((I%3-1)*20,0,30);Q.End=FVector3d(Q.Start.X,0,-30);
        Q.bAuthorityOnly=true;Q.bIncludeCompactPatches=true;Q.DomainTolerance=0;Q.InitialOverlapTolerance=0;
        if(I%4==0){Q.Start=FVector3d(0,0,33);Q.End=FVector3d(0,0,3);} // end-time equality
        if(I%5==0){Q.Start=Q.End=FVector3d(0,0,1);} // initial penetration / time zero
        if(I%7==0)Q.Start.X=Q.End.X=-0.0;
        Queries.Add(Q);
    }
    FAnalyticStaticCollisionWorld Serial(Data,1,2,2);TArray<FWorldHit> Expected;
    for(const auto& Q:Queries)Expected.Add(Serial.SweepSingle(Q));
    uint64 TotalWaves=0,TotalReused=0,TotalBoundRecomputed=0;
    for(int32 Lanes:{1,2,4,8,12})
    {
        FAnalyticStaticCollisionWorld World(Data,Lanes,2,2);ResetPiecewiseProviderWaveStats1075();
        for(int32 Repeat=0;Repeat<2;++Repeat)
        for(int32 I=0;I<Queries.Num();++I)
        {
            const FWorldHit Actual=World.SweepSingle(Queries[I]);
            if(!SameHit(Expected[I],Actual)){AddError(FString::Printf(TEXT("provider wave divergence lanes=%d repeat=%d query=%d"),Lanes,Repeat,I));return false;}
        }
        const auto S=SnapshotPiecewiseProviderWaveStats1075();
        if(Lanes>1)
        {
            TestTrue(TEXT("real whole-provider native wave required"),S.Waves>0&&S.Computed>0&&S.Reused>0&&S.UsefulLaneMax>1&&S.NativeMax>1&&S.UsefulNativeMax>1);
            TestEqual(TEXT("computed results split exactly into reused and wasted"),S.Computed,S.Reused+S.Wasted);
        }
        TotalWaves+=S.Waves;TotalReused+=S.Reused;TotalBoundRecomputed+=S.RejectBound;
    }
    TestTrue(TEXT("native oracle covered positive reuse"),TotalWaves>0&&TotalReused>0);
    TestTrue(TEXT("tightening forces exact-bound recomputation"),TotalBoundRecomputed>0);
    ResetPiecewiseProviderWaveStats1075();return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedProviderWaveKeysBoundsAndRefusals1075,
    "IAmSpeed.Parallel.Analytic1075.ProviderWaveKeysBoundsAndRefusals",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FIAmSpeedProviderWaveKeysBoundsAndRefusals1075::RunTest(const FString& Parameters)
{
    using namespace Speed;using namespace Analytic;using namespace ProviderTests1075;
    FScopedInt Enabled(TEXT("p.IAmSpeed.Parallel.PiecewiseProviderWave1075"),1);
    FScopedInt Observe(TEXT("p.IAmSpeed.Parallel.PiecewiseProviderObserve1075"),1);
    if(!Enabled.Is(1)||!Observe.Is(1))return false;
    FWorldQuery Query;Query.Start=FVector3d(0,0,30);Query.End=FVector3d(0,0,-30);Query.Shape=EQueryShape::Sphere;Query.Radius=3;
    Query.DomainTolerance=0;Query.InitialOverlapTolerance=0;Query.bAuthorityOnly=true;Query.bIncludeCompactPatches=true;
    const FBox3d Bounds(FVector3d(-5,-5,-35),FVector3d(5,5,35));
    int32 WorldToken=1,IndexToken=2,InvocationToken=3,OtherToken=4;
    const FPiecewiseProviderWaveIdentity1075 Identity{&WorldToken,&IndexToken,&InvocationToken,0};
    const TArray<int32> Indices{0,1};std::atomic<int32> Calls{0};int32 BoundCalls=0;
    const auto Bound=[&](){++BoundCalls;return 1.0;};
    const auto Evaluate=[&](int32 Index,double Maximum){Calls.fetch_add(1);FWorldHit H;H.bHit=true;H.Time=Maximum;H.SourceId=Index+1;return H;};
    {
        FJoinedPiecewiseProviderWave1075 Wave(Query,Bounds,Identity,Indices,2,4,Bound,Evaluate);
        TestTrue(TEXT("native wave ready"),Wave.IsReady());FWorldHit Hit;
        TestTrue(TEXT("exact equal bound consumes"),Wave.TryConsume(0,Query,Bounds,Identity,1.0,Hit));
        TestTrue(TEXT("equal-time provider tie preserves identity"),Hit.Time==1.0&&Hit.SourceId==1);
        TestFalse(TEXT("joined provider consumed once"),Wave.TryConsume(0,Query,Bounds,Identity,1.0,Hit));
        TestTrue(TEXT("second equal-bound provider remains independently consumable"),Wave.TryConsume(1,Query,Bounds,Identity,1.0,Hit));
    }
    const auto Reject=[&](auto Mutate)
    {
        FJoinedPiecewiseProviderWave1075 Wave(Query,Bounds,Identity,Indices,2,4,Bound,Evaluate);
        auto Q=Query;auto B=Bounds;auto C=Identity;double Maximum=1.0;Mutate(Q,B,C,Maximum);
        FWorldHit Hit;Hit.PrimitiveId=0x12345678;const auto Sentinel=Hit;
        return TestFalse(TEXT("changed key never consumes speculative hit"),Wave.TryConsume(0,Q,B,C,Maximum,Hit))&&
            TestTrue(TEXT("rejected output remains untouched"),ParallelTests1075::SameHit(Sentinel,Hit));
    };
    if(!Reject([](auto& Q,auto&,auto&,double&){Q.Shape=EQueryShape::Box;}))return false; // shape mutation
    if(!Reject([](auto& Q,auto&,auto&,double&){Q.Start.X=1;}))return false; // start mutation
    if(!Reject([](auto& Q,auto&,auto&,double&){Q.End.X=1;}))return false; // end mutation
    if(!Reject([](auto& Q,auto&,auto&,double&){Q.Rotation=FRotator3d(0,13,0).Quaternion();}))return false; // rotation mutation
    if(!Reject([](auto& Q,auto&,auto&,double&){Q.Radius=4;}))return false; // radius mutation
    if(!Reject([](auto& Q,auto&,auto&,double&){Q.HalfExtent.X=1;}))return false; // extent mutation
    if(!Reject([](auto& Q,auto&,auto&,double&){Q.DomainTolerance=1.e-7;}))return false; // domain tolerance mutation
    if(!Reject([](auto& Q,auto&,auto&,double&){Q.InitialOverlapTolerance=0.05;}))return false; // initial-overlap shell mutation
    if(!Reject([](auto& Q,auto&,auto&,double&){Q.TraceChannel=1;}))return false; // trace mask mutation
    if(!Reject([](auto& Q,auto&,auto&,double&){Q.ObjectTypes=1;}))return false; // object mask mutation
    if(!Reject([](auto& Q,auto&,auto&,double&){Q.BlockingObjectTypes=0;}))return false; // response mask mutation
    if(!Reject([](auto& Q,auto&,auto&,double&){Q.RequiredSourceId=2;}))return false; // required source mutation
    if(!Reject([](auto& Q,auto&,auto&,double&){Q.RequiredSurfaceId=12;}))return false; // required surface mutation
    if(!Reject([](auto& Q,auto&,auto&,double&){Q.RequiredCanonicalGroupId=32;}))return false; // required canonical group mutation
    if(!Reject([](auto& Q,auto&,auto&,double&){Q.ReferenceNormal=FVector3d::UpVector;}))return false; // from-normal reprobe mutation
    if(!Reject([](auto& Q,auto&,auto&,double&){Q.MinimumReferenceNormalDot=0.9;}))return false; // normal cone mutation
    if(!Reject([](auto& Q,auto&,auto&,double&){Q.bAllowEstablishedFaceContactAtBoundary=!Q.bAllowEstablishedFaceContactAtBoundary;}))return false; // boundary reacquisition mutation
    if(!Reject([](auto& Q,auto&,auto&,double&){Q.bUseFiniteContactDomain=!Q.bUseFiniteContactDomain;}))return false; // finite-domain mutation
    if(!Reject([](auto& Q,auto&,auto&,double&){Q.bObjectQuery=!Q.bObjectQuery;}))return false; // object policy mutation
    if(!Reject([](auto& Q,auto&,auto&,double&){Q.bApplyCollisionFilter=!Q.bApplyCollisionFilter;}))return false; // filter policy mutation
    if(!Reject([](auto& Q,auto&,auto&,double&){Q.bAuthorityOnly=!Q.bAuthorityOnly;}))return false; // authority policy mutation
    if(!Reject([](auto& Q,auto&,auto&,double&){Q.bExcludeAuthorityEligible=!Q.bExcludeAuthorityEligible;}))return false; // exclude-authority policy mutation
    if(!Reject([](auto& Q,auto&,auto&,double&){Q.bIncludeCompactPatches=!Q.bIncludeCompactPatches;}))return false; // compact provider policy mutation
    if(!Reject([](auto& Q,auto&,auto&,double&){Q.bIncludeTriangles=!Q.bIncludeTriangles;}))return false; // triangle policy mutation
    if(!Reject([](auto& Q,auto&,auto&,double&){Q.Start.X=-0.0;}))return false; // signed-zero mutation
    if(!Reject([](auto&,auto&,auto&,double& B){B=0.5;}))return false;
    if(!Reject([](auto&,auto&,auto&,double& B){B=-0.0;}))return false;
    if(!Reject([](auto& Q,auto&,auto&,double&){Q.Start.X=-0.0;}))return false;
    if(!Reject([](auto& Q,auto&,auto&,double&){Q.InitialOverlapTolerance=0.1;}))return false;
    if(!Reject([](auto& Q,auto&,auto&,double&){Q.ReferenceNormal=FVector3d::UpVector;}))return false;
    if(!Reject([](auto& Q,auto&,auto&,double&){Q.BlockingObjectTypes=0;}))return false;
    if(!Reject([](auto& Q,auto&,auto&,double&){Q.bUseFiniteContactDomain=!Q.bUseFiniteContactDomain;}))return false;
    if(!Reject([](auto&,auto& B,auto&,double&){B.Min.X-=1;}))return false;
    if(!Reject([&](auto&,auto&,auto& C,double&){C.World=&OtherToken;}))return false;
    if(!Reject([&](auto&,auto&,auto& C,double&){C.Index=&OtherToken;}))return false;
    if(!Reject([&](auto&,auto&,auto& C,double&){C.Invocation=&OtherToken;}))return false;
    if(!Reject([](auto&,auto&,auto& C,double&){++C.Selection;}))return false;
    for(double Invalid:{std::numeric_limits<double>::infinity(),-std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN(),-0.1,1.1})
    {
        Calls=0;BoundCalls=0;const auto Before=CaptureFrameFloatingPointContext();
        {FJoinedPiecewiseProviderWave1075 Wave(Query,Bounds,Identity,Indices,2,4,[&](){++BoundCalls;return Invalid;},Evaluate);TestFalse(TEXT("invalid bound refuses whole wave"),Wave.IsReady());}
        TestEqual(TEXT("invalid bound executes no provider"),Calls.load(),0);
        TestTrue(TEXT("invalid-bound refusal restores owner FP"),SameNumericProviderTests1075(Before,CaptureFrameFloatingPointContext()));
    }
    const auto Refused=[&](TConstArrayView<int32> Items,int32 CandidateCount,int32 Lanes)
    {
        Calls=0;BoundCalls=0;
        {FJoinedPiecewiseProviderWave1075 Wave(Query,Bounds,Identity,Items,CandidateCount,Lanes,Bound,Evaluate);if(Wave.IsReady())return false;}
        return Calls.load()==0&&BoundCalls==0;
    };
    TestTrue(TEXT("serial refusal has no hidden work"),Refused(Indices,2,1));
    TestTrue(TEXT("below-grain refusal has no hidden work"),Refused(MakeArrayView(Indices.GetData(),1),1,4));
    TestTrue(TEXT("capacity refusal has no hidden work"),Refused(Indices,257,4));
    {FScopedFrameParallelTask Nested;TestTrue(TEXT("nested refusal has no hidden bound/kernel work"),Refused(Indices,2,4));}
    {FScopedInt Audit(TEXT("p.IAmSpeed.StaticWorldQuery.Audit"),1);if(!Audit.Is(1))return false;TestTrue(TEXT("diagnostic refusal has no hidden work"),Refused(Indices,2,4));}
    {FScopedInt Off(TEXT("p.IAmSpeed.Parallel.PiecewiseProviderWave1075"),0);TestTrue(TEXT("OFF executes no capture/kernel"),Refused(Indices,2,4));}
    {
        FScopedInt NoTimers(TEXT("p.IAmSpeed.Parallel.PiecewiseProviderMaxSamples1075"),0);
        ResetPiecewiseProviderWaveStats1075();
        {FJoinedPiecewiseProviderWave1075 Wave(Query,Bounds,Identity,Indices,2,4,Bound,Evaluate);if(!Wave.IsReady())return false;}
        const auto S=SnapshotPiecewiseProviderWaveStats1075();
        TestEqual(TEXT("zero timing cap records no timed attempt"),S.TimedAttempts,uint64(0));
        TestEqual(TEXT("timing cap skip explicit"),S.TimingSkippedAttempts,uint64(1));
        TestEqual(TEXT("zero cap starts no kernel timers"),S.KernelCycles,uint64(0));
        TestTrue(TEXT("timing cap retains exact participation counts"),S.Computed==2&&S.UsefulLaneMax==2&&S.UsefulNativeMax>=1&&S.UsefulNativeMax<=S.NativeMax&&S.UsefulNativeMax<=S.UsefulLaneMax);
    }
    {
        FScopedInt NoObserver(TEXT("p.IAmSpeed.Parallel.PiecewiseProviderObserve1075"),0);
        ResetPiecewiseProviderWaveStats1075();
        {FJoinedPiecewiseProviderWave1075 Wave(Query,Bounds,Identity,Indices,2,4,Bound,Evaluate);if(!Wave.IsReady())return false;}
        TestEqual(TEXT("observer OFF leaves global counters untouched"),SnapshotPiecewiseProviderWaveStats1075().Attempts,uint64(0));
    }
    ResetPiecewiseProviderWaveStats1075();return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedProviderWaveNativeAdmissionAndFP1075,
    "IAmSpeed.Parallel.Analytic1075.ProviderWaveNativeAdmissionAndFP",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FIAmSpeedProviderWaveNativeAdmissionAndFP1075::RunTest(const FString& Parameters)
{
    using namespace Speed;using namespace Analytic;using namespace ProviderTests1075;
    std::atomic<int32> HiddenCalls{0},Accepted{0};
    DispatchPrivateFrameParallelLanes(2,[&](int32)
    {if(TryDispatchPrivateFrameParallelLanes1075(2,[&](int32){++HiddenCalls;}))++Accepted;});
    TestEqual(TEXT("nested native-only executor refuses every group"),Accepted.load(),0);
    TestEqual(TEXT("nested native-only executor never invokes hidden callbacks"),HiddenCalls.load(),0);
    FScopedInt Enabled(TEXT("p.IAmSpeed.Parallel.PiecewiseProviderWave1075"),1);
    FScopedInt Observe(TEXT("p.IAmSpeed.Parallel.PiecewiseProviderObserve1075"),1);
    if(!Enabled.Is(1)||!Observe.Is(1))return false;
    int32 Token=1;const FPiecewiseProviderWaveIdentity1075 Identity{&Token,&Token,&Token,0};
    FWorldQuery Q;const FBox3d Bounds(FVector3d(-1),FVector3d(1));const TArray<int32> Indices{0,1};
    const auto Before=CaptureFrameFloatingPointContext();
    {
        FJoinedPiecewiseProviderWave1075 Wave(Q,Bounds,Identity,Indices,2,2,[](){return 1.0;},[](int32,double){return FWorldHit{};});
        TestTrue(TEXT("prepared native wave exists before FP mutation"),Wave.IsReady());
        auto Changed=CaptureFrameFloatingPointContext();Changed.VectorControl^=0x2000;
        FScopedFrameFloatingPointContext Numeric(Changed);if(!Numeric.IsApplied())return false;
        FWorldHit Hit;TestFalse(TEXT("numeric environment change refuses reuse"),Wave.TryConsume(0,Q,Bounds,Identity,1.0,Hit));
        if(!Numeric.Restore())return false;
    }
    TestTrue(TEXT("owner FP survives changed-context rejection"),SameNumericProviderTests1075(Before,CaptureFrameFloatingPointContext()));
    {
        const TArray<int32> FourIndices{0,1,2,3};bool SameEntry[4]={};
        const auto ExpectedNumeric=CaptureFrameFloatingPointContext();
        FJoinedPiecewiseProviderWave1075 Wave(Q,Bounds,Identity,FourIndices,4,2,[](){return 1.0;},
            [&](int32 Index,double)
            {
                const auto Entry=CaptureFrameFloatingPointContext();SameEntry[Index]=SameNumericProviderTests1075(ExpectedNumeric,Entry);
                auto Changed=Entry;Changed.VectorControl^=0x2000;
                if(!ApplyFrameFloatingPointContext(Changed))throw std::runtime_error("test numeric mutation failed");
                return FWorldHit{};
            });
        TestTrue(TEXT("every provider independently receives the captured full FP environment"),
            SameEntry[0]&&SameEntry[1]&&SameEntry[2]&&SameEntry[3]);
    }
    TestTrue(TEXT("per-provider numeric mutations cannot escape the joined wave"),SameNumericProviderTests1075(Before,CaptureFrameFloatingPointContext()));
    {
        const auto Incoming=CaptureFrameFloatingPointContext();
        FScopedFrameFloatingPointContext RestoreIncoming(Incoming);
        FFrameFloatingPointContext ExpectedOutput=Incoming;ExpectedOutput.VectorControl^=0x2000;
        FFrameFloatingPointContext ActualOutputs[2];
        FJoinedPiecewiseProviderWave1075 Wave(Q,Bounds,Identity,Indices,2,2,[](){return 1.0;},
            [&](int32 Index,double)
            {
                if(!ApplyFrameFloatingPointContext(ExpectedOutput))throw std::runtime_error("test output FP setup failed");
                ActualOutputs[Index]=CaptureFrameFloatingPointContext();
                return FWorldHit{};
            });
        const bool SpeculationRestored=SameNumericProviderTests1075(Incoming,CaptureFrameFloatingPointContext());
        FWorldHit Hit;const bool Used=Wave.TryConsume(0,Q,Bounds,Identity,1.0,Hit);
        const bool OutputReplayed=SameNumericProviderTests1075(ActualOutputs[0],CaptureFrameFloatingPointContext());
        if(!RestoreIncoming.Restore())return false;
        TestTrue(TEXT("speculative FP effects stay private until actual consumption"),SpeculationRestored);
        TestTrue(TEXT("validated consumption replays the exact serial kernel output FP state"),Used&&OutputReplayed);
    }
    {
        // Provider1 throws only at the speculative initial bound. Provider0
        // tightens the actual bound, so provider1 must be recomputed without
        // publishing either its speculative exception or numeric side effects.
        const auto Incoming=CaptureFrameFloatingPointContext();
        FScopedFrameFloatingPointContext RestoreIncoming(Incoming);
        std::atomic<int32> Calls{0};
        const auto Evaluate=[&](int32 Index,double Maximum)->FWorldHit
        {
            Calls.fetch_add(1);
            if(Index==1&&Maximum==1.0)
            {std::feraiseexcept(FE_INEXACT);throw std::runtime_error("speculative-bound-only");}
            FWorldHit H;H.bHit=true;H.Time=Index==0?0.2:Maximum;H.SourceId=Index+1;return H;
        };
        bool Constructed=false,Rejected=false,NoSerialThrow=false,PrivateNumeric=false;FWorldHit First,Second;
        try
        {
            FJoinedPiecewiseProviderWave1075 Wave(Q,Bounds,Identity,Indices,2,2,[](){return 1.0;},Evaluate);
            Constructed=Wave.IsReady()&&Calls.load()==2;
            PrivateNumeric=SameNumericProviderTests1075(Incoming,CaptureFrameFloatingPointContext());
            if(!Wave.TryConsume(0,Q,Bounds,Identity,1.0,First))return false;
            Second.PrimitiveId=0x12345678;const auto Sentinel=Second;
            Rejected=!Wave.TryConsume(1,Q,Bounds,Identity,0.2,Second)&&ParallelTests1075::SameHit(Sentinel,Second);
            Second=Evaluate(1,0.2);NoSerialThrow=Second.bHit&&Second.Time==0.2&&Second.SourceId==2;
        }
        catch(...){NoSerialThrow=false;}
        if(!RestoreIncoming.Restore())return false;
        TestTrue(TEXT("speculative exception does not escape construction and other jobs complete"),Constructed);
        TestTrue(TEXT("discarded speculative exception keeps numeric effects private"),PrivateNumeric);
        TestTrue(TEXT("tightened bound rejects speculative provider exception"),Rejected);
        TestTrue(TEXT("original provider at bound0.2 succeeds without speculative exception"),NoSerialThrow);
    }
    {
        // Clear incoming status under a saved owner guard, then make the second
        // provider throw with an inexact sticky flag at an unchanged bound.
        const auto Incoming=CaptureFrameFloatingPointContext();
        FScopedFrameFloatingPointContext RestoreIncoming(Incoming);
        std::feclearexcept(FE_ALL_EXCEPT);
        auto Clean=CaptureFrameFloatingPointContext();Clean.VectorControl&=~uint32(0x3f);
        if(!ApplyFrameFloatingPointContext(Clean))return false;
        Clean=CaptureFrameFloatingPointContext();
        std::atomic<int32> Calls{0};
        const auto Evaluate=[&](int32 Index,double)->FWorldHit
        {
            Calls.fetch_add(1);
            if(Index==1)
            {std::feraiseexcept(FE_INEXACT);throw std::runtime_error("canonical-provider1");}
            FWorldHit H;H.bHit=true;H.Time=1.0;H.SourceId=1;return H;
        };
        FFrameFloatingPointContext SerialExceptionOutput;bool SerialReferenceThrew=false;
        try{Evaluate(1,1.0);}
        catch(const std::runtime_error& E)
        {
            SerialReferenceThrew=FCStringAnsi::Strcmp(E.what(),"canonical-provider1")==0;
            SerialExceptionOutput=CaptureFrameFloatingPointContext();
        }
        if(!ApplyFrameFloatingPointContext(Clean))return false;
        Calls=0;
        bool Constructed=false,FirstUsed=false,CanonicalThrow=false,CorrectFP=false,Sticky=false,PrivateNumeric=false;
        try
        {
            FJoinedPiecewiseProviderWave1075 Wave(Q,Bounds,Identity,Indices,2,2,[](){return 1.0;},Evaluate);
            Constructed=Wave.IsReady()&&Calls.load()==2;
            PrivateNumeric=SameNumericProviderTests1075(Clean,CaptureFrameFloatingPointContext());
            FWorldHit Hit;FirstUsed=Wave.TryConsume(0,Q,Bounds,Identity,1.0,Hit);
            try{Wave.TryConsume(1,Q,Bounds,Identity,1.0,Hit);}
            catch(const std::runtime_error& E)
            {
                CanonicalThrow=FCStringAnsi::Strcmp(E.what(),"canonical-provider1")==0;
                CorrectFP=SameNumericProviderTests1075(SerialExceptionOutput,CaptureFrameFloatingPointContext());
                Sticky=(std::fetestexcept(FE_INEXACT)&FE_INEXACT)!=0;
            }
        }
        catch(...){CanonicalThrow=false;}
        if(!RestoreIncoming.Restore())return false;
        TestTrue(TEXT("unchanged-bound exception stays private through construction"),Constructed&&PrivateNumeric&&FirstUsed);
        TestTrue(TEXT("same exception rethrows only at canonical consumption"),CanonicalThrow);
        TestTrue(TEXT("canonical exception publishes exact output FEnv and sticky flag"),SerialReferenceThrew&&CorrectFP&&Sticky);
    }
    FFrameFloatingPointContext SerialCanonicalOutput;
    try{throw std::runtime_error("provider0");}
    catch(const std::runtime_error&){SerialCanonicalOutput=CaptureFrameFloatingPointContext();}
    if(!ApplyFrameFloatingPointContext(Before))return false;
    bool Caught=false;
    // A runtime predicate gives UE's generic Invoke adapter a reachable return.
    // Both actual provider callbacks still throw; assert their execution below.
    std::atomic<bool> ThrowFromProvider{true};
    std::atomic<int32> ThrowingCallbacks{0};
    try
    {
        FJoinedPiecewiseProviderWave1075 Wave(Q,Bounds,Identity,Indices,2,2,[](){return 1.0;},
            [&](int32 Index,double)->FWorldHit
            {
                if(ThrowFromProvider.load(std::memory_order_relaxed))
                {
                    ThrowingCallbacks.fetch_add(1,std::memory_order_relaxed);
                    throw std::runtime_error(Index==0?"provider0":"provider1");
                }
                return FWorldHit{};
            });
        FWorldHit Hit;
        Wave.TryConsume(0,Q,Bounds,Identity,1.0,Hit); // The canonical boundary, never construction.
    }
    catch(const std::runtime_error& E){Caught=FCStringAnsi::Strcmp(E.what(),"provider0")==0;}
    TestEqual(TEXT("both original callbacks actually throw before canonical consumption"),ThrowingCallbacks.load(),int32(2));
    TestTrue(TEXT("canonical provider exception is rethrown after join"),Caught);
    const bool CanonicalNumericMatches=SameNumericProviderTests1075(SerialCanonicalOutput,CaptureFrameFloatingPointContext());
    if(!ApplyFrameFloatingPointContext(Before))return false;
    TestTrue(TEXT("canonical exception output FP matches the original serial exception"),CanonicalNumericMatches);
    ResetPiecewiseProviderWaveStats1075();return true;
}
#endif
