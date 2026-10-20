#if WITH_DEV_AUTOMATION_TESTS
#include "JoinedAnalyticQueryPrefetch1075.h"
#include "AnalyticCausalStats1075.h"
#include "IAmSpeed/World/Collision/StaticCollisionWorld.h"
#include "IAmSpeed/World/Simulation/FrameParallelSettings.h"
#include "HAL/IConsoleManager.h"
#include "Misc/AutomationTest.h"

namespace Speed::Analytic::ParallelTests1075
{
bool SameHit(const FWorldHit& A, const FWorldHit& B);
bool PiecewiseWorld(FAnalyticWorldData& Data, FString& Reason);
}
namespace Speed::Analytic::PrefetchTests1075
{
struct FScopedInt
{
    IConsoleVariable* Variable; int32 Previous;
    FScopedInt(const TCHAR* Name,int32 Value)
        : Variable(IConsoleManager::Get().FindConsoleVariable(Name)),Previous(Variable?Variable->GetInt():0)
    { if(Variable)Variable->Set(Value,ECVF_SetByCode); }
    ~FScopedInt(){if(Variable)Variable->Set(Previous,ECVF_SetByCode);}
    bool Is(int32 Value)const{return Variable&&Variable->GetInt()==Value;}
};
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedJoinedQueryMutation1075,
    "IAmSpeed.Parallel.Analytic1075.JoinedQueryMutation",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FIAmSpeedJoinedQueryMutation1075::RunTest(const FString& Parameters)
{
    using namespace Speed;
    using namespace Analytic;
    using namespace ParallelTests1075;
    FAnalyticWorldData Data; FString Reason;
    if (!PiecewiseWorld(Data,Reason)){AddError(Reason);return false;}
    FAnalyticStaticCollisionWorld Serial(Data), Parallel(Data,4,2,2), NewGeneration(Data,4,2,2);
    TestTrue(TEXT("each immutable adapter/index build has a distinct generation"),
        Parallel.GetImmutableQueryIdentity1075().Generation!=NewGeneration.GetImmutableQueryIdentity1075().Generation);
    int32 OwnerToken=1, TypeToken=2, OtherToken=3, WorldToken=4, BridgeToken=5;
    FAnalyticPrefetchQuery1075 Entry;
    Entry.Consumer.Owner=&OwnerToken; Entry.Consumer.ExactType=&TypeToken;
    const float Delta=1.0f/300.0f;
    FMemory::Memcpy(&Entry.Consumer.DeltaBits,&Delta,sizeof(Delta));
    Entry.Query.Shape=EQueryShape::Sphere; Entry.Query.Radius=3;
    Entry.Query.Start=FVector3d(0,0,30); Entry.Query.End=FVector3d(0,0,-30);
    Entry.Query.bAuthorityOnly=true; Entry.Query.bIncludeCompactPatches=true;
    Entry.Query.DomainTolerance=0; Entry.Query.InitialOverlapTolerance=0;
    const FWorldHit Expected=Serial.SweepSingle(Entry.Query);
    if (!TestTrue(TEXT("positive piecewise contact"),Expected.bHit)) return false;
    TArray<FWorldQuery> Queries{Entry.Query,Entry.Query}; TArray<FWorldHit> Hits; Hits.SetNum(2);
    FIndependentQueryBatchStats Dispatch;
    if (!TestTrue(TEXT("native joined prefetch branch executes"),Parallel.SweepPrefetchBatch1075(Queries,Hits,&Dispatch))) return false;
    TestTrue(TEXT("actual logical dispatch is required"),Dispatch.DispatchedLogicalLanes>1);
    for (const auto& Hit : Hits) if (!SameHit(Expected,Hit)) return false;
    const FAnalyticPrefetchContext1075 Context{&WorldToken,&BridgeToken,Parallel.GetImmutableQueryIdentity1075(),17,
        CaptureFrameFloatingPointContext()};
    const auto NewCache=[&]()
    { return FJoinedAnalyticQueryCache1075(MakeArrayView(&Entry,1),MakeArrayView(Hits.GetData(),1),Context); };
    {
        auto Cache=NewCache(); FWorldHit Actual;
        TestTrue(TEXT("identical current query consumes the joined result"),
            Cache.TryConsume(Context,Entry.Consumer,Entry.Query,1.0,Actual)==EAnalyticPrefetchReuse1075::Reused);
        TestTrue(TEXT("every hit field survives consumption"),SameHit(Expected,Actual));
        TestTrue(TEXT("a result is consumed once"),
            Cache.TryConsume(Context,Entry.Consumer,Entry.Query,1.0,Actual)==EAnalyticPrefetchReuse1075::NoEntry);
    }
    const auto RejectQuery=[&](const TCHAR* Label,auto Mutate)
    {
        auto Cache=NewCache(); FWorldQuery Changed=Entry.Query; Mutate(Changed);
        FWorldHit Untouched; Untouched.PrimitiveId=0x12345678;
        const FWorldHit Sentinel=Untouched;
        const auto Result=Cache.TryConsume(Context,Entry.Consumer,Changed,1.0,Untouched);
        if (!TestTrue(Label,Result==EAnalyticPrefetchReuse1075::QueryChanged))return false;
        if (!TestTrue(TEXT("rejected speculative result leaves output untouched"),SameHit(Sentinel,Untouched)))return false;
        // The caller follows the original serial path with the CURRENT query.
        FAnalyticStaticCollisionWorld Fresh(Data);
        if (!SameHit(Fresh.SweepSingle(Changed),Serial.SweepSingle(Changed)))return false;
        return TestEqual(TEXT("one rejected result is separately accountable"),Cache.Rejected(),1);
    };
    if(!RejectQuery(TEXT("shape mutation"),[](auto& Q){Q.Shape=EQueryShape::Box;}))return false;
    if(!RejectQuery(TEXT("start mutation"),[](auto& Q){Q.Start.X=1;}))return false;
    if(!RejectQuery(TEXT("end mutation"),[](auto& Q){Q.End.X=1;}))return false;
    if(!RejectQuery(TEXT("rotation mutation"),[](auto& Q){Q.Rotation=FRotator3d(0,13,0).Quaternion();}))return false;
    if(!RejectQuery(TEXT("radius mutation"),[](auto& Q){Q.Radius=4;}))return false;
    if(!RejectQuery(TEXT("extent mutation"),[](auto& Q){Q.HalfExtent.X=1;}))return false;
    if(!RejectQuery(TEXT("domain tolerance mutation"),[](auto& Q){Q.DomainTolerance=1.e-7;}))return false;
    if(!RejectQuery(TEXT("initial-overlap shell mutation"),[](auto& Q){Q.InitialOverlapTolerance=0.05;}))return false;
    if(!RejectQuery(TEXT("trace mask mutation"),[](auto& Q){Q.TraceChannel=1;}))return false;
    if(!RejectQuery(TEXT("object mask mutation"),[](auto& Q){Q.ObjectTypes=1;}))return false;
    if(!RejectQuery(TEXT("response mask mutation"),[](auto& Q){Q.BlockingObjectTypes=0;}))return false;
    if(!RejectQuery(TEXT("required source mutation"),[](auto& Q){Q.RequiredSourceId=2;}))return false;
    if(!RejectQuery(TEXT("required surface mutation"),[](auto& Q){Q.RequiredSurfaceId=12;}))return false;
    if(!RejectQuery(TEXT("required canonical group mutation"),[](auto& Q){Q.RequiredCanonicalGroupId=32;}))return false;
    if(!RejectQuery(TEXT("from-normal reprobe mutation"),[](auto& Q){Q.ReferenceNormal=FVector3d::UpVector;}))return false;
    if(!RejectQuery(TEXT("normal cone mutation"),[](auto& Q){Q.MinimumReferenceNormalDot=0.9;}))return false;
    if(!RejectQuery(TEXT("boundary reacquisition mutation"),[](auto& Q){Q.bAllowEstablishedFaceContactAtBoundary=!Q.bAllowEstablishedFaceContactAtBoundary;}))return false;
    if(!RejectQuery(TEXT("finite-domain mutation"),[](auto& Q){Q.bUseFiniteContactDomain=!Q.bUseFiniteContactDomain;}))return false;
    if(!RejectQuery(TEXT("object policy mutation"),[](auto& Q){Q.bObjectQuery=!Q.bObjectQuery;}))return false;
    if(!RejectQuery(TEXT("filter policy mutation"),[](auto& Q){Q.bApplyCollisionFilter=!Q.bApplyCollisionFilter;}))return false;
    if(!RejectQuery(TEXT("authority policy mutation"),[](auto& Q){Q.bAuthorityOnly=!Q.bAuthorityOnly;}))return false;
    if(!RejectQuery(TEXT("exclude-authority policy mutation"),[](auto& Q){Q.bExcludeAuthorityEligible=!Q.bExcludeAuthorityEligible;}))return false;
    if(!RejectQuery(TEXT("compact provider policy mutation"),[](auto& Q){Q.bIncludeCompactPatches=!Q.bIncludeCompactPatches;}))return false;
    if(!RejectQuery(TEXT("triangle policy mutation"),[](auto& Q){Q.bIncludeTriangles=!Q.bIncludeTriangles;}))return false;
    if(!RejectQuery(TEXT("signed-zero mutation"),[](auto& Q){Q.Start.X=-0.0;}))return false;
    const auto RejectContext=[&](auto Mutate)
    {
        auto Cache=NewCache(); auto Current=Context; Mutate(Current); FWorldHit Hit;
        return Cache.TryConsume(Current,Entry.Consumer,Entry.Query,1.0,Hit)==EAnalyticPrefetchReuse1075::ContextChanged;
    };
    TestTrue(TEXT("world replacement invalidates reuse"),RejectContext([&](auto& C){C.World=&OtherToken;}));
    TestTrue(TEXT("bridge replacement invalidates reuse"),RejectContext([&](auto& C){C.RuntimeBridge=&OtherToken;}));
    TestTrue(TEXT("payload replacement invalidates reuse"),RejectContext([&](auto& C){C.ImmutableQuery.Data=&OtherToken;}));
    TestTrue(TEXT("index replacement invalidates reuse"),RejectContext([&](auto& C){C.ImmutableQuery.Index=&OtherToken;}));
    TestTrue(TEXT("generation invalidates reused addresses"),RejectContext([](auto& C){++C.ImmutableQuery.Generation;}));
    TestTrue(TEXT("frame change invalidates reuse"),RejectContext([](auto& C){++C.Frame;}));
    {
        auto Cache=NewCache(); auto Consumer=Entry.Consumer; Consumer.ExactType=&OtherToken; FWorldHit Hit;
        TestTrue(TEXT("unknown exact type is refused"),Cache.TryConsume(Context,Consumer,Entry.Query,1.0,Hit)==EAnalyticPrefetchReuse1075::ExactTypeChanged);
    }
    {
        auto Cache=NewCache(); auto Consumer=Entry.Consumer; Consumer.DeltaBits^=1; FWorldHit Hit;
        TestTrue(TEXT("dt mutation is refused"),Cache.TryConsume(Context,Consumer,Entry.Query,1.0,Hit)==EAnalyticPrefetchReuse1075::DeltaChanged);
    }
    {
        auto Cache=NewCache(); FWorldHit Hit;
        TestTrue(TEXT("search-bound mutation is refused"),Cache.TryConsume(Context,Entry.Consumer,Entry.Query,0.5,Hit)==EAnalyticPrefetchReuse1075::SearchBoundChanged);
    }
    {
        auto Cache=NewCache(); auto Current=Context; Current.NumericContext.VectorControl^=0x2000; FWorldHit Hit;
        TestTrue(TEXT("numeric environment mutation invalidates speculation"),
            Cache.TryConsume(Current,Entry.Consumer,Entry.Query,1.0,Hit)==EAnalyticPrefetchReuse1075::NumericEnvironmentChanged);
    }
    {
        TArray<FAnalyticPrefetchQuery1075> TooMany; TooMany.SetNum(65);
        TArray<FWorldHit> TooManyHits; TooManyHits.SetNum(65);
        FJoinedAnalyticQueryCache1075 Refused(TooMany,TooManyHits,Context);
        TestEqual(TEXT("joined cache refuses more than 64 captures"),Refused.Num(),0);
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedPrefetchRejectsSerialFallback1075,
    "IAmSpeed.Parallel.Analytic1075.PrefetchRejectsSerialFallback",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FIAmSpeedPrefetchRejectsSerialFallback1075::RunTest(const FString& Parameters)
{
    using namespace Speed;
    using namespace Analytic;
    using namespace PrefetchTests1075;
    FAnalyticWorldData Data; FString Reason;
    if (!ParallelTests1075::PiecewiseWorld(Data,Reason)){AddError(Reason);return false;}
    FAnalyticStaticCollisionWorld World(Data,4,2,2);
    FWorldQuery Query; Query.Shape=EQueryShape::Sphere; Query.Radius=3;
    Query.Start=FVector3d(0,0,30); Query.End=FVector3d(0,0,-30);
    Query.bAuthorityOnly=true; Query.bIncludeCompactPatches=true;
    TArray<FWorldQuery> Queries{Query,Query}; TArray<FWorldHit> Hits; Hits.SetNum(2);
    Hits[0].PrimitiveId=0x12345678; const FWorldHit Sentinel=Hits[0];
    FScopedInt Causal(TEXT("p.IAmSpeed.Parallel.AnalyticCausal1075"),1);
    FScopedInt Audit(TEXT("p.IAmSpeed.StaticWorldQuery.Audit"),1);
    if(!Causal.Is(1)||!Audit.Is(1))return false;
    ResetAnalyticCausalStats1075();
    FIndependentQueryBatchStats Stats;
    TestFalse(TEXT("thread-owned observer refuses speculative admission"),World.SweepPrefetchBatch1075(Queries,Hits,&Stats));
    TestTrue(TEXT("precise refusal reason retained"),Stats.Fallback==EIndependentQueryFallback::ThreadOwnedDiagnostics);
    TestTrue(TEXT("refused speculative output stays untouched"),ParallelTests1075::SameHit(Sentinel,Hits[0]));
    TestEqual(TEXT("no speculative world query ran"),SnapshotAnalyticCausalStats1075().Kinds[int32(EAnalyticCausalKind1075::WorldSweep)].Seen,uint64(0));
    TestTrue(TEXT("ordinary API retains its serial diagnostic behavior"),World.SweepIndependentBatch(Queries,Hits,&Stats));
    TestTrue(TEXT("ordinary fallback actually queried the world"),SnapshotAnalyticCausalStats1075().Kinds[int32(EAnalyticCausalKind1075::WorldSweep)].Seen>0);
    {
        FScopedInt AuditOff(TEXT("p.IAmSpeed.StaticWorldQuery.Audit"),0);
        if(!AuditOff.Is(0))return false;
        Hits[0]=Sentinel;
        ResetAnalyticCausalStats1075();
        FAnalyticStaticCollisionWorld Serial(Data,1,2,2);
        TestFalse(TEXT("serial policy refuses speculative execution"),Serial.SweepPrefetchBatch1075(Queries,Hits,&Stats));
        TestTrue(TEXT("serial refusal reason retained"),Stats.Fallback==EIndependentQueryFallback::SerialPolicy);
        {
            FScopedFrameParallelTask Nested;
            TestFalse(TEXT("nested prefetch refuses speculative execution"),World.SweepPrefetchBatch1075(Queries,Hits,&Stats));
            TestTrue(TEXT("nested refusal reason retained"),Stats.Fallback==EIndependentQueryFallback::NestedTask);
        }
        TArray<FWorldQuery> OneQuery{Query}; TArray<FWorldHit> OneHit{Sentinel};
        TestFalse(TEXT("below-grain prefetch refuses speculative execution"),World.SweepPrefetchBatch1075(OneQuery,OneHit,&Stats));
        TestTrue(TEXT("below-grain refusal reason retained"),Stats.Fallback==EIndependentQueryFallback::TooFewQueryItems);
        TestTrue(TEXT("fallback output still untouched"),ParallelTests1075::SameHit(Sentinel,Hits[0]));
        TestTrue(TEXT("below-grain output still untouched"),ParallelTests1075::SameHit(Sentinel,OneHit[0]));
        TestEqual(TEXT("all rejected admission paths execute no world query"),SnapshotAnalyticCausalStats1075().Kinds[int32(EAnalyticCausalKind1075::WorldSweep)].Seen,uint64(0));
    }
    ResetAnalyticCausalStats1075();
    return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedWheelPrefetchNeutralized1075,
    "IAmSpeed.Parallel.Analytic1075.WheelPrefetchNeutralizedUntilNumericCacheProof",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FIAmSpeedWheelPrefetchNeutralized1075::RunTest(const FString& Parameters)
{
    using namespace Speed::Analytic;
    PrefetchTests1075::FScopedInt Requested(TEXT("p.IAmSpeed.Parallel.WheelQueryPrefetch1075"),1);
    if(!Requested.Is(1))return false;
    TestTrue(TEXT("requested value remains observable"),IsWheelSuspensionPrefetch1075Requested());
    TestFalse(TEXT("unproved numeric/cache semantics cannot be activated"),IsWheelSuspensionPrefetch1075Enabled());
    TestTrue(TEXT("explicit runtime reason available"),FCString::Strcmp(WheelSuspensionPrefetch1075Status(),TEXT("DisabledCanonicalCacheAndOutputFEnvUnqualified"))==0);
    ResetAnalyticPrefetchStats1075();
    {TConstArrayView<TObjectPtr<USWheelSubBody>> Empty;FScopedWheelSuspensionPrefetch1075 Scope(Empty,0.0033333333f);}
    TestEqual(TEXT("disabled scope performs no capture or speculation"),SnapshotAnalyticPrefetchStats1075().ScopeAttempts,uint64(0));
    return true;
}

#endif
