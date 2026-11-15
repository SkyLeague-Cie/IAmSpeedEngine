// Mathematical table tests only. No native snapshot, World, owner, actor,
// authority ticket, query serial or PASS is constructed by these fixtures.
namespace
{
static_assert(!AllowsEarlyCache94(C::EInheritedReach91::Unknown));
static_assert(!AllowsEarlyCache94(C::EInheritedReach91::Cleared));
static_assert(AllowsEarlyCache94(C::EInheritedReach91::Invalid));
static_assert(AllowsEarlyCache94(C::EInheritedReach91::Outside));
static_assert(AllowsEarlyCache94(C::EInheritedReach91::Near));
struct FEarlyCache94Fixture
{
    FPreparedWorldSnapshotPtr NoNativeSnapshot;
    Analytic::FWorldQuery Query;
    C::FReferenceCandidateSetPtr Candidates;
    TSharedPtr<FCompleteGeometryProof94,ESPMode::ThreadSafe> Proof;
    C::FDistanceBudget Budget{65536};
    FEarlyCache94Fixture()
    {
        Query.Shape=Analytic::EQueryShape::Sphere;Query.Start=FVector3d(0,0,2);
        Query.End=FVector3d::ZeroVector;Query.Radius=.5;
        Query.bAuthorityOnly=true;Query.bApplyCollisionFilter=true;
    }
    bool Build(bool compact=false)
    {
        TArray<C::FIntervalTensorSurface> surfaces;
        for(const auto& p:{FVector3d(0,0,1),FVector3d(10,10,10),FVector3d(-10,-10,10)})
        {C::FIntervalTensorSurface s;s.ControlPoints.Add(C::Detail::Point(p));surfaces.Add(MoveTemp(s));}
        C::FIndexedIntervalUnionPtr scene;C::FIntervalBezierCenterPath path;
        if(!C::FIndexedIntervalUnion::Build(MakeArrayView(surfaces),Budget,scene)||
            !C::CenterDetail::Linear(Query.Start,Query.End,path)||
            !C::FReferenceCandidateSet::Select(scene,path,Query.Radius,Budget,Candidates,nullptr,false,compact)||!Candidates)return false;
        Proof=MakeShared<FCompleteGeometryProof94,ESPMode::ThreadSafe>();
        Proof->Contact.Status=C::ETensorSphereContactStatus::FirstContactEnclosed;
        Proof->Contact.Leaves.AddDefaulted();Proof->RegularNormals.AddDefaulted();
        Proof->Hit.bHit=true;Proof->Hit.SourceId=123;Proof->Hit.Time=.25;
        return true;
    }
    FCompleteQueryView94 Key(const Analytic::FWorldQuery& q,uint64 numeric=17,uint64 delta=0,uint64 physical=0)const
    {
        return {NoNativeSnapshot,q,numeric,delta?delta:A::Bits(.01),physical?physical:A::Bits(.49),
            Candidates->GetCandidateIndices(),Candidates->GetExcludedIndices()};
    }
    FCompleteQueryView94 EarlyKey(const Analytic::FWorldQuery& q,uint64 numeric=17,uint64 delta=0,uint64 physical=0)const
    {return {NoNativeSnapshot,q,numeric,delta?delta:A::Bits(.01),physical?physical:A::Bits(.49),{}, {}};}
    bool Store(FHistoricalQueryCache94& cache,const Analytic::FWorldQuery& q,bool retain)
    {
        const auto k=Key(q);uint64 bytes=0;
        if(FHistoricalQueryCache94::PrepareStore(k,Proof->Contact,MakeArrayView(Proof->RegularNormals),Budget)!=ECompleteCache94::Hit)return false;
        if(retain&&FHistoricalQueryCache94::PrepareEarlyRetention(k,Candidates,Budget,bytes)!=ECompleteCache94::Hit)return false;
        cache.StoreReserved(k,Proof,retain?Candidates:C::FReferenceCandidateSetPtr{},bytes);return true;
    }
};
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FEarlyHistoricalQueryCache94IdentityTest,
    "IAmSpeed.AnalyticWorld.NativeSolid.EarlyHistoricalQueryCache94.IdentityAndPartition",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FEarlyHistoricalQueryCache94IdentityTest::RunTest(const FString&)
{
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE mathematical table fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FEarlyCache94Fixture f;if(!TestTrue(TEXT("actual immutable selected partition"),f.Build()))return false;
    TestEqual(TEXT("one actual candidate"),f.Candidates->GetCandidateIndices().Num(),1);
    TestEqual(TEXT("two original-order exclusions"),f.Candidates->GetExcludedIndices().Num(),2);
    FHistoricalQueryCache94 cache;TSharedPtr<const FCompleteGeometryProof94,ESPMode::ThreadSafe> found;C::FReferenceCandidateSetPtr selected;
    if(!TestTrue(TEXT("old pointer-free store is unchanged"),f.Store(cache,f.Query,false)))return false;
    TestTrue(TEXT("pointer-free entry is not early eligible"),cache.FindEarly(f.EarlyKey(f.Query),f.Budget,found,selected)==ECompleteCache94::Miss&&!found&&!selected);
    TestTrue(TEXT("same pointer-free entry remains a normal late hit"),cache.Find(f.Key(f.Query),f.Budget,found)==ECompleteCache94::Hit);
    FHistoricalQueryCache94 early;if(!TestTrue(TEXT("bounded pointer retention"),f.Store(early,f.Query,true)))return false;
    const auto* identity=f.Candidates.Get();const auto scene=f.Candidates->GetScene();const auto q=f.Query;
    TestTrue(TEXT("key arrays really alias the immutable producer object"),FHistoricalQueryCache94::CanRetainCandidates(f.Key(q),f.Candidates));
    TArray<int32> copied;copied.Append(f.Candidates->GetCandidateIndices().GetData(),f.Candidates->GetCandidateIndices().Num());
    const FCompleteQueryView94 copiedKey{f.NoNativeSnapshot,q,17,A::Bits(.01),A::Bits(.49),MakeArrayView(copied),f.Candidates->GetExcludedIndices()};
    TestFalse(TEXT("equal reconstructed indices do not authorize pointer retention"),FHistoricalQueryCache94::CanRetainCandidates(copiedKey,f.Candidates));
    f.Candidates.Reset();f.Query.RequiredSourceId=77;
    TestTrue(TEXT("entry owns original key and shares the actual partition after caller release"),
        early.FindEarly(f.EarlyKey(q),f.Budget,found,selected)==ECompleteCache94::Hit&&found==f.Proof&&selected.Get()==identity&&selected->GetScene()==scene);
    TestEqual(TEXT("excluded order retained"),selected->GetExcludedIndices()[0],1);
    TestEqual(TEXT("excluded order retained second"),selected->GetExcludedIndices()[1],2);
    TestTrue(TEXT("original CP payload retained"),selected->GetSurfacesForFixedSource().Num()==1&&
        A::Bits(selected->GetSurfacesForFixedSource()[0].ControlPoints[0].Axis[2].Lower)==A::Bits(1.));
    uint64 actualBytes=0;TestTrue(TEXT("all owned allocations counted with bounded work"),
        selected->AccountOwnedQueryStorageBytes(f.Budget,actualBytes)&&actualBytes>=sizeof(C::FReferenceCandidateSet)+
        uint64(selected->GetCandidateIndices().Num()+selected->GetExcludedIndices().Num())*sizeof(int32)+
        uint64(selected->GetSurfacesForFixedSource().Num())*sizeof(C::FIntervalTensorSurface)+sizeof(C::FIntervalPoint));
    TestTrue(TEXT("FP numeric key mutation misses"),early.FindEarly(f.EarlyKey(q,18),f.Budget,found,selected)==ECompleteCache94::Miss);
    TestTrue(TEXT("delta bits mutation misses"),early.FindEarly(f.EarlyKey(q,17,A::Bits(.02)),f.Budget,found,selected)==ECompleteCache94::Miss);
    TestTrue(TEXT("physical radius mutation misses"),early.FindEarly(f.EarlyKey(q,17,0,A::Bits(.48)),f.Budget,found,selected)==ECompleteCache94::Miss);
    TestTrue(TEXT("restored"),restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FEarlyHistoricalQueryCache94CompleteKeyTest,
    "IAmSpeed.AnalyticWorld.NativeSolid.EarlyHistoricalQueryCache94.AllQueryFieldsAndCollisions",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FEarlyHistoricalQueryCache94CompleteKeyTest::RunTest(const FString&)
{
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE exact-key fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FEarlyCache94Fixture f;if(!TestTrue(TEXT("actual candidate object"),f.Build()))return false;
    FHistoricalQueryCache94 cache;if(!TestTrue(TEXT("initial entry"),f.Store(cache,f.Query,true)))return false;
    TArray<Analytic::FWorldQuery> changed;
    const auto add=[&](auto mutate){auto q=f.Query;mutate(q);changed.Add(q);};
    add([](auto& q){q.Shape=Analytic::EQueryShape::Box;});
    add([](auto& q){q.Start.X=-0.;});add([](auto& q){q.End.X=1.;});
    add([](auto& q){q.Rotation.W=-1.;});add([](auto& q){q.Radius=.6;});
    add([](auto& q){q.HalfExtent.X=1.;});add([](auto& q){q.DomainTolerance=0.;});
    add([](auto& q){q.InitialOverlapTolerance=0.;});add([](auto& q){q.TraceChannel=1;});
    add([](auto& q){q.ObjectTypes=1;});add([](auto& q){q.BlockingObjectTypes=0;});
    add([](auto& q){q.RequiredSourceId=1;});add([](auto& q){q.RequiredSurfaceId=1;});
    add([](auto& q){q.RequiredCanonicalGroupId=1;});add([](auto& q){q.ReferenceNormal.Z=1.;});
    add([](auto& q){q.MinimumReferenceNormalDot=0.;});
    add([](auto& q){q.bAllowEstablishedFaceContactAtBoundary=!q.bAllowEstablishedFaceContactAtBoundary;});
    add([](auto& q){q.bUseFiniteContactDomain=!q.bUseFiniteContactDomain;});
    add([](auto& q){q.bObjectQuery=!q.bObjectQuery;});add([](auto& q){q.bApplyCollisionFilter=!q.bApplyCollisionFilter;});
    add([](auto& q){q.bAuthorityOnly=!q.bAuthorityOnly;});add([](auto& q){q.bExcludeAuthorityEligible=!q.bExcludeAuthorityEligible;});
    add([](auto& q){q.bExcludeTori=!q.bExcludeTori;});add([](auto& q){q.bIncludeCompactPatches=!q.bIncludeCompactPatches;});
    add([](auto& q){q.bIncludeTriangles=!q.bIncludeTriangles;});
    TSharedPtr<const FCompleteGeometryProof94,ESPMode::ThreadSafe> found;C::FReferenceCandidateSetPtr selected;
    for(int32 i=0;i<changed.Num();++i)
        TestTrue(*FString::Printf(TEXT("query field %d cannot reuse original geometry"),i),
            cache.FindEarly(f.EarlyKey(changed[i]),f.Budget,found,selected)==ECompleteCache94::Miss&&!found&&!selected);
    for(uint64 i=1;i<=5;++i){auto q=f.Query;q.RequiredSourceId=i;if(!TestTrue(TEXT("same bucket insertion"),f.Store(cache,q,true)))return false;}
    TestTrue(TEXT("four ways evict instead of returning a collided hit"),cache.FindEarly(f.EarlyKey(f.Query),f.Budget,found,selected)==ECompleteCache94::Miss&&!found&&!selected);
    auto q=f.Query;q.RequiredSourceId=5;
    TestTrue(TEXT("collision winner still matches full key"),cache.FindEarly(f.EarlyKey(q),f.Budget,found,selected)==ECompleteCache94::Hit&&selected==f.Candidates);
    TestTrue(TEXT("restored"),restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FEarlyHistoricalQueryCache94BudgetAndRouteTest,
    "IAmSpeed.AnalyticWorld.NativeSolid.EarlyHistoricalQueryCache94.BudgetAndInheritedRouting",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FEarlyHistoricalQueryCache94BudgetAndRouteTest::RunTest(const FString&)
{
    TestFalse(TEXT("Unknown inherited result never enters early lookup"),AllowsEarlyCache94(C::EInheritedReach91::Unknown));
    TestFalse(TEXT("Cleared inherited result keeps separated route"),AllowsEarlyCache94(C::EInheritedReach91::Cleared));
    FEarlyCache94Fixture f;FHistoricalQueryCache94 cache;
    TSharedPtr<const FCompleteGeometryProof94,ESPMode::ThreadSafe> found;C::FReferenceCandidateSetPtr selected;
    C::FDistanceBudget zero{0};
    TestTrue(TEXT("known insufficient work is not applicable without consume"),
        cache.FindEarly(f.EarlyKey(f.Query),zero,found,selected)==ECompleteCache94::NotApplicable&&zero.Used==0&&!found&&!selected);
    C::FDistanceBudget reserved{152,0,1};
    TestTrue(TEXT("reservation is honored before consuming"),
        cache.FindEarly(f.EarlyKey(f.Query),reserved,found,selected)==ECompleteCache94::NotApplicable&&reserved.Used==0);
    C::FDistanceBudget sticky{65536};sticky.Used=MAX_uint64-1;
    TestTrue(TEXT("attempted work failure is terminal, never downgraded to miss"),
        cache.FindEarly(f.EarlyKey(f.Query),sticky,found,selected)==ECompleteCache94::Failure&&sticky.Used==MAX_uint64&&!found&&!selected);
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE retention fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    if(!TestTrue(TEXT("real retained allocations"),f.Build()))return false;
    const auto key=f.Key(f.Query);uint64 bytes=99;
    TestTrue(TEXT("small retention limit is only not-applicable, with zero published bytes"),
        FHistoricalQueryCache94::PrepareEarlyRetention(key,f.Candidates,f.Budget,bytes,0)==ECompleteCache94::NotApplicable&&bytes==0);
    if(!TestTrue(TEXT("late store remains eligible after early size refusal"),
        FHistoricalQueryCache94::PrepareStore(key,f.Proof->Contact,MakeArrayView(f.Proof->RegularNormals),f.Budget)==ECompleteCache94::Hit))return false;
    cache.StoreReserved(key,f.Proof,{},bytes);
    TestTrue(TEXT("late cache survives rejected early pointer retention"),cache.Find(key,f.Budget,found)==ECompleteCache94::Hit);
    TestTrue(TEXT("rejected retention cannot become an early hit"),cache.FindEarly(f.EarlyKey(f.Query),f.Budget,found,selected)==ECompleteCache94::Miss&&!found&&!selected);
    C::FDistanceBudget noRetentionWork{0};bytes=99;
    TestTrue(TEXT("insufficient retention work is inapplicable without debit"),
        FHistoricalQueryCache94::PrepareEarlyRetention(key,f.Candidates,noRetentionWork,bytes)==ECompleteCache94::NotApplicable&&bytes==0&&noRetentionWork.Used==0);
    TestTrue(TEXT("restored"),restore.Restore());return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCompactHistoricalQueryCache94Test,
    "IAmSpeed.AnalyticWorld.NativeSolid.EarlyHistoricalQueryCache94.CompactPartition",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCompactHistoricalQueryCache94Test::RunTest(const FString&)
{
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE complete-key fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FEarlyCache94Fixture f;if(!TestTrue(TEXT("real compact producer partition"),f.Build(true)&&f.Candidates->HasCompactExclusions()))return false;
    const auto compact=f.Candidates;C::FReferenceCandidateSetPtr legacy;
    if(!TestTrue(TEXT("same actual scene legacy partition"),C::FReferenceCandidateSet::Select(compact->GetScene(),compact->GetPathForFixedSource(),f.Query.Radius,f.Budget,legacy)))return false;
    const FCompleteQueryView94 compactKey=f.Key(f.Query);
    const FCompleteQueryView94 legacyKey{f.NoNativeSnapshot,f.Query,17,A::Bits(.01),A::Bits(.49),legacy->GetCandidateIndices(),legacy->GetExcludedIndices()};
    FHistoricalQueryCache94 cache,legacyCache;TSharedPtr<const FCompleteGeometryProof94,ESPMode::ThreadSafe> found;
    C::FDistanceBudget a{65536},b=a;uint64 ba=0,bb=0;
    if(!TestTrue(TEXT("same logical copy debit with compact representation"),
        FHistoricalQueryCache94::PrepareStore(compactKey,f.Proof->Contact,MakeArrayView(f.Proof->RegularNormals),a)==ECompleteCache94::Hit&&
        FHistoricalQueryCache94::PrepareStore(legacyKey,f.Proof->Contact,MakeArrayView(f.Proof->RegularNormals),b)==ECompleteCache94::Hit&&a.Used==b.Used&&a.Remaining==b.Remaining))return false;
    if(!TestTrue(TEXT("retention validates exact storage identity and accounts fewer real bytes"),
        FHistoricalQueryCache94::PrepareEarlyRetention(compactKey,compact,a,ba)==ECompleteCache94::Hit&&
        FHistoricalQueryCache94::PrepareEarlyRetention(legacyKey,legacy,b,bb)==ECompleteCache94::Hit&&a.Used==b.Used&&bb>=ba+2*sizeof(int32)))return false;
    cache.StoreReserved(compactKey,f.Proof,compact,ba);legacyCache.StoreReserved(legacyKey,f.Proof,legacy,bb);
    C::FReferenceCandidateSetPtr retained;
    TestTrue(TEXT("early result retains original actual compact partition"),cache.FindEarly(f.EarlyKey(f.Query),f.Budget,found,retained)==ECompleteCache94::Hit&&retained==compact&&found==f.Proof);
    for(int32 cap=0;cap<180;++cap)
    {
        C::FDistanceBudget ca{cap,3,1},cb=ca;TSharedPtr<const FCompleteGeometryProof94,ESPMode::ThreadSafe> x,y;
        const auto ra=cache.Find(compactKey,ca,x),rb=legacyCache.Find(legacyKey,cb,y);
        if(!TestTrue(TEXT("late compact equality preserves all reserve and debit boundaries"),ra==rb&&ca.Used==cb.Used&&ca.Remaining==cb.Remaining&&bool(x)==bool(y)))return false;
    }
    TestTrue(TEXT("mixed explicit request can compare stored compact complement without materialization"),cache.Find(legacyKey,f.Budget,found)==ECompleteCache94::Hit);
    TestTrue(TEXT("compact request compares stored explicit complement exactly"),legacyCache.Find(compactKey,f.Budget,found)==ECompleteCache94::Hit);
    TArray<int32> copied;copied.Append(compact->GetCandidateIndices().GetData(),compact->GetCandidateIndices().Num());
    const FCompleteQueryView94 reconstructed{f.NoNativeSnapshot,f.Query,17,A::Bits(.01),A::Bits(.49),MakeArrayView(copied),compact->GetExcludedIndices()};
    TestFalse(TEXT("compact identity cannot be attached to reconstructed candidates"),FHistoricalQueryCache94::CanRetainCandidates(reconstructed,compact));
    TestTrue(TEXT("forged association cannot enter lookup/store"),cache.Find(reconstructed,f.Budget,found)==ECompleteCache94::NotApplicable&&
        FHistoricalQueryCache94::PrepareStore(reconstructed,f.Proof->Contact,MakeArrayView(f.Proof->RegularNormals),f.Budget)==ECompleteCache94::NotApplicable);
    FEarlyCache94Fixture other;if(!TestTrue(TEXT("independent complete scene"),other.Build(true)))return false;
    TestTrue(TEXT("foreign immutable scene identity never uses compact equivalence"),cache.Find(other.Key(other.Query),f.Budget,found)==ECompleteCache94::Miss);
    auto changed=f.Query;changed.Start.X=A::FromBits(A::Bits(changed.Start.X)+1);
    TestTrue(TEXT("one-bit query change still misses early"),cache.FindEarly(f.EarlyKey(changed),f.Budget,found,retained)==ECompleteCache94::Miss);
    C::FReferenceCandidateSetPtr second;
    if(!TestTrue(TEXT("second compact partition of same scene"),C::FReferenceCandidateSet::Select(compact->GetScene(),compact->GetPathForFixedSource(),f.Query.Radius,f.Budget,second,nullptr,false,true)))return false;
    const FCompleteQueryView94 sameScene{f.NoNativeSnapshot,f.Query,17,A::Bits(.01),A::Bits(.49),second->GetCandidateIndices(),second->GetExcludedIndices()};
    TestTrue(TEXT("different query object may reuse same exact Scene/N/candidates"),cache.Find(sameScene,f.Budget,found)==ECompleteCache94::Hit);
    TestTrue(TEXT("original context restored"),restore.Restore());return !HasAnyErrors();
}
