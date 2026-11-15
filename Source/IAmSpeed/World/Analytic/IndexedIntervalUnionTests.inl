#include "IndexedIntervalUnion.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticCompleteUnionTangentTest,
 "IAmSpeed.AnalyticWorld.CompleteUnion.TangentPartitionAndSnapshot",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticCompleteUnionTangentTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture established"),context.Ready))return false;
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    FIntervalTensorSurface touch,later;touch.ControlPoints.Add(Detail::Point(FVector3d(0.25,0.125,0)));later.ControlPoints.Add(Detail::Point(FVector3d(0.75,0,0)));
    TArray<FIntervalTensorSurface> surfaces;surfaces.Add(touch);
    for(int32 i=0;i<2990;++i){FIntervalTensorSurface far;far.ControlPoints.Add(Detail::Point(FVector3d(100.+i,100,100)));surfaces.Add(far);}surfaces.Add(later);
    FDistanceBudget budget{65536};FIndexedIntervalUnionPtr scene;
    if(!TestTrue(TEXT("complete mathematical union snapshot"),FIndexedIntervalUnion::Build(MakeArrayView(surfaces),budget,scene)))return false;
    TestEqual(TEXT("all canonical providers retained"),scene->Num(),2992);
    FIntervalBezierCenterPath path;TestTrue(TEXT("reference path"),CenterDetail::Linear(FVector3d::ZeroVector,FVector3d(1,0,0),path));
    FReferenceCandidateSetPtr proof;const auto before=budget.Used;
    if(!TestTrue(TEXT("complete conservative scan published"),FReferenceCandidateSet::Select(scene,path,0.125,budget,proof)))return false;
    TestEqual(TEXT("2992 scans plus2 single-CP candidate copies, one shared budget"),budget.Used-before,uint64(2994));
    const auto candidates=proof->GetCandidateIndices();const auto excluded=proof->GetExcludedIndices();
    TestTrue(TEXT("strict test keeps the exact tangent and later piercing"),candidates.Num()==2&&candidates[0]==0&&candidates[1]==2991);
    TestEqual(TEXT("every far provider explicitly excluded"),excluded.Num(),2990);
    for(int32 i=0;i<excluded.Num();++i)if(excluded[i]!=i+1){AddError(TEXT("canonical partition has a hole or duplicate"));break;}
    // Changing the caller's scene/path storage cannot mutate the captured proof.
    surfaces[0].ControlPoints[0]=Detail::Point(FVector3d(100,100,100));path.ControlPoints[0]=Detail::Point(FVector3d(100,100,100));
    const auto bounded=proof->Sweep(budget,1./32.,1.e-8,1);
    TestTrue(TEXT("earlier tangent is never skipped to promote later piercing"),bounded.Status==ETensorSphereTimeStatus::Unknown&&bounded.InitialSeparationProved&&bounded.LowerTime<=0.25);
    TArray<FIntervalTensorSurface> direct={touch,later};FDistanceBudget directBudget{8192};FIntervalBezierCenterPath original;CenterDetail::Linear(FVector3d::ZeroVector,FVector3d(1,0,0),original);
    const auto acquired=BoundTensorSphereSweep(MakeArrayView(direct),original,0.125,directBudget,1./32.,1.e-8,1);
    TestTrue(TEXT("candidate subset preserves global unresolved anchor"),acquired.Status==bounded.Status&&AnalyticCertified::Bits(acquired.LowerTime)==AnalyticCertified::Bits(bounded.LowerTime));
    TestTrue(TEXT("original floating context restored"),context.Restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticCompleteUnionRemapTest,
 "IAmSpeed.AnalyticWorld.CompleteUnion.ContactRemapAndRefusals",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticCompleteUnionRemapTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture established"),context.Ready))return false;
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    FIntervalTensorSurface far,plane;far.ControlPoints.Add(Detail::Point(FVector3d(100,100,100)));Detail::PromoteTensor(TimeUnitPlane(),plane);
    TArray<FIntervalTensorSurface> surfaces={far,plane,far};FDistanceBudget budget{262144};FIndexedIntervalUnionPtr scene;
    if(!TestTrue(TEXT("owned complete union"),FIndexedIntervalUnion::Build(MakeArrayView(surfaces),budget,scene)))return false;
    FIntervalBezierCenterPath path;CenterDetail::Linear(FVector3d(0,0,1),FVector3d(0,0,-1),path);FReferenceCandidateSetPtr proof;
    if(!TestTrue(TEXT("captured crossing path"),FReferenceCandidateSet::Select(scene,path,0.25,budget,proof)))return false;
    TestTrue(TEXT("canonical plane index is1"),proof->GetCandidateIndices().Num()==1&&proof->GetCandidateIndices()[0]==1);
    const auto hit=proof->Contact(budget,1./65536.,1.e-7,256,0.01,0.05);
    TestTrue(TEXT("same kernel encloses continuous first contact"),hit.Status==ETensorSphereContactStatus::FirstContactEnclosed);
    if(hit.Status==ETensorSphereContactStatus::FirstContactEnclosed)
    {
        TestTrue(TEXT("bracket includes exact3/8"),hit.LowerTime<=0.375&&hit.UpperTime>=0.375);
        const double normal[3]={0,0,1};for(int32 d=0;d<3;++d)TestTrue(TEXT("point and radial normal enclosures retained"),hit.SurfacePoint.Axis[d].Lower<=0&&hit.SurfacePoint.Axis[d].Upper>=0&&hit.RadialUnitNormal.Axis[d].Lower<=normal[d]&&hit.RadialUnitNormal.Axis[d].Upper>=normal[d]);
    }
    FIntervalBezierCenterPath inside;CenterDetail::Linear(FVector3d::ZeroVector,FVector3d(0,0,1),inside);FReferenceCandidateSetPtr initial;
    TestTrue(TEXT("inside query needs its own selection"),FReferenceCandidateSet::Select(scene,inside,0.25,budget,initial));
    if(initial){const auto r=initial->Sweep(budget,1./64.,1.e-7,1);TestTrue(TEXT("later/initial inside witness remaps to canonical index1"),r.Status==ETensorSphereTimeStatus::InitialInsideTube&&r.InsideWitnessSurfaceIndex==1);}
    if(initial)TestTrue(TEXT("reselection owns scene before clearing its containing output"),FReferenceCandidateSet::Select(initial->GetScene(),path,0.25,budget,initial));
    FDistanceBudget shortBudget{3};FReferenceCandidateSetPtr stale=proof;
    TestFalse(TEXT("partial scan cannot publish"),FReferenceCandidateSet::Select(scene,path,0.25,shortBudget,stale));TestFalse(TEXT("refusal erased old proof"),stale.IsValid());
    FDistanceBudget invalidBudget{1024};auto invalid=surfaces;invalid.Last().DegreeU=17;FIndexedIntervalUnionPtr staleScene=scene;
    TestFalse(TEXT("invalid last provider cannot be silently omitted"),FIndexedIntervalUnion::Build(MakeArrayView(invalid),invalidBudget,staleScene));TestFalse(TEXT("invalid scene erased old output"),staleScene.IsValid());
    TArray<FIntervalTensorSurface> onlyFar={far};FIndexedIntervalUnionPtr farScene;TestTrue(TEXT("far scene"),FIndexedIntervalUnion::Build(MakeArrayView(onlyFar),budget,farScene));FReferenceCandidateSetPtr empty;
    TestTrue(TEXT("full scan proves empty candidates"),FReferenceCandidateSet::Select(farScene,path,0.25,budget,empty));
    if(empty)
    {
        TestEqual(TEXT("all providers excluded"),empty->GetExcludedIndices().Num(),1);
        TestTrue(TEXT("empty proof permits provided-union NoContact"),empty->Contact(budget,1./64.,1.e-7,1,0.01,0.05).Status==ETensorSphereContactStatus::NoContactWithProvidedTensors);
        FDistanceBudget zero{0};TestTrue(TEXT("empty proof cannot bypass budget exhaustion"),empty->Sweep(zero,1./64.,1.e-7,1).Status==ETensorSphereTimeStatus::Unknown);
        const uint32 control=VectorGetControlRegister();VectorSetControlRegister((control&~uint32(0x6000))|0x2000u);bool refused=false;
        try {empty->Sweep(budget,1./64.,1.e-7,1);}catch(const FAnalyticNumericUnknown&){refused=true;}VectorSetControlRegister(control);
        TestTrue(TEXT("empty proof rechecks FP context on execution"),refused);
    }
    TestTrue(TEXT("original floating context restored"),context.Restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticCompleteUnionConsumedTest,
 "IAmSpeed.AnalyticWorld.CompleteUnion.ConsumedRangeIsNotEndpoint",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticCompleteUnionConsumedTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture established"),context.Ready))return false;
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    FIntervalTensorSurface plane;Detail::PromoteTensor(TimeUnitPlane(),plane);TArray<FIntervalTensorSurface> surfaces={plane};FDistanceBudget budget{32768};FIndexedIntervalUnionPtr scene;
    if(!TestTrue(TEXT("complete plane scene"),FIndexedIntervalUnion::Build(MakeArrayView(surfaces),budget,scene)))return false;
    FZeroOffsetKinematicCenterReference returning;BuildZeroOffsetKinematicCenterReference(FVector3d(0,0,1),FVector3d(0,0,-4),FVector3d(0,0,8),1.f,FVector3d::ZeroVector,FVector3d::ZeroVector,returning);
    TestTrue(TEXT("consumed endpoint is clear"),ProveConsumedCenterRangeClear(scene,returning,Interval::Exact(1),0.25,budget,1.e-7,16));
    TestFalse(TEXT("endpoint never licenses advancing across middle contact"),ProveConsumedCenterRangeClear(scene,returning,Interval{0,1,true},0.25,budget,1.e-7,16));
    TestTrue(TEXT("whole short consumed prefix proves strict separation"),ProveConsumedCenterRangeClear(scene,returning,Interval{0,1./16.,true},0.25,budget,1.e-7,16));
    volatile float s=0.5f,s2=s*s;const double actual=1.-4.*double(s)+4.*double(s2);TestEqual(TEXT("independent consumed middle pose on plane"),actual,0.);
    FDistanceBudget zero{0};TestFalse(TEXT("zero budget is unknown, never clear"),ProveConsumedCenterRangeClear(scene,returning,Interval::Exact(1),0.25,zero,1.e-7,1));
    TestFalse(TEXT("invalid normalized range refuses"),ProveConsumedCenterRangeClear(scene,returning,Interval{-1,1,true},0.25,budget,1.e-7,1));
    TestTrue(TEXT("original floating context restored"),context.Restore.Restore());return !HasAnyErrors();
}
namespace
{
FAutoConsoleCommand CompleteUnionUnits(TEXT("Speed.Analytic.RunCompleteUnionUnits"),TEXT("Run three complete-union candidate/exclusion units; no World authority."),
 FConsoleCommandDelegate::CreateLambda([]()
 {
    static bool Used=false;auto& f=FAutomationTestFramework::Get();
    if(!IsRunningCommandlet()||!FParse::Param(FCommandLine::Get(),TEXT("AnalyticCompleteUnionUnits"))||Used||GIsAutomationTesting||GIsSlowTask||GIsPlayInEditorWorld||f.GetCurrentTest()||!f.IsLatentCommandQueueEmpty()){UE_LOG(LogTemp,Error,TEXT("Complete union units refused"));return;}Used=true;
    const TCHAR* classes[]={TEXT("FAnalyticCompleteUnionTangentTest"),TEXT("FAnalyticCompleteUnionRemapTest"),TEXT("FAnalyticCompleteUnionConsumedTest")};
    const TCHAR* names[]={TEXT("IAmSpeed.AnalyticWorld.CompleteUnion.TangentPartitionAndSnapshot"),TEXT("IAmSpeed.AnalyticWorld.CompleteUnion.ContactRemapAndRefusals"),TEXT("IAmSpeed.AnalyticWorld.CompleteUnion.ConsumedRangeIsNotEndpoint")};
    for(int32 i=0;i<3;++i)
    {
        if(GIsAutomationTesting||GIsSlowTask||GIsPlayInEditorWorld||f.GetCurrentTest()||!f.IsLatentCommandQueueEmpty()||!f.ContainsTest(classes[i])){UE_LOG(LogTemp,Error,TEXT("Complete union unit ownership refused"));return;}
        f.SetRequestedTestFilter(EAutomationTestFlags::EngineFilter);f.StartTestByName(classes[i],0,names[i]);
        if(!GIsAutomationTesting||!f.GetCurrentTest()||f.GetCurrentTestFullPath()!=names[i]||!f.IsLatentCommandQueueEmpty()){UE_LOG(LogTemp,Error,TEXT("Complete union unit synchronous ownership refused"));return;}
        FAutomationTestExecutionInfo info;const bool stopped=f.StopTest(info);
        if(GIsAutomationTesting||GIsSlowTask||GIsPlayInEditorWorld||f.GetCurrentTest()||!f.IsLatentCommandQueueEmpty()){UE_LOG(LogTemp,Error,TEXT("Complete union unit final ownership refused"));return;}
        const bool passed=stopped&&info.bSuccessful&&info.GetErrorTotal()==0&&info.GetWarningTotal()==0;
        UE_LOG(LogTemp,Display,TEXT("ANALYTIC_COMPLETE_UNION_UNIT name=%s passed=%d errors=%d warnings=%d"),names[i],passed,info.GetErrorTotal(),info.GetWarningTotal());
    }
 }));
}


#include "FixedShapeQueryBounds.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticFixedQueryBoundsTest,
 "IAmSpeed.AnalyticWorld.CompleteUnion.FixedRaySphereBoxBounds",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticFixedQueryBoundsTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture established"),context.Ready))return false;
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    FWorldQuery q;q.Start=FVector3d::ZeroVector;q.End=FVector3d(1,0,0);
    FIntervalPoint hull;
    if(!TestTrue(TEXT("zero-radius ray volume"),BuildFixedWorldQueryVolumeBounds(q,hull)))return false;
    TArray<FIntervalTensorSurface> surfaces;
    for(auto p:{FVector3d(.5,0,0),FVector3d(2,0,0),FVector3d(0,0,2)})
    {FIntervalTensorSurface surface;surface.ControlPoints.Add(Detail::Point(p));surfaces.Add(surface);}
    FDistanceBudget budget{32768};FIndexedIntervalUnionPtr scene;
    if(!TestTrue(TEXT("complete immutable provider input"),FIndexedIntervalUnion::Build(MakeArrayView(surfaces),budget,scene)))return false;
    FReferenceBoundsCandidateSetPtr partition;
    if(!TestTrue(TEXT("ray selection has no invented positive radius"),FReferenceBoundsCandidateSet::Select(scene,hull,budget,partition)))return false;
    TestTrue(TEXT("ray piercing retained, separated providers excluded"),partition->GetCandidateIndices().Num()==1&&partition->GetCandidateIndices()[0]==0&&partition->GetExcludedIndices().Num()==2);
    q.Shape=EQueryShape::Sphere;q.End=q.Start;q.Radius=2;
    TestTrue(TEXT("stationary sphere volume"),BuildFixedWorldQueryVolumeBounds(q,hull));
    TestTrue(TEXT("tangent sphere partition"),FReferenceBoundsCandidateSet::Select(scene,hull,budget,partition));
    TestTrue(TEXT("exact sphere tangencies are not excluded"),partition&&partition->GetCandidateIndices().Num()==3);
    q.Shape=EQueryShape::Box;q.HalfExtent=FVector3d(2,1,1);q.Rotation=FRotator3d(0,45,0).Quaternion();q.End=FVector3d(10,0,0);
    if(!TestTrue(TEXT("rotated OBB volume"),BuildFixedWorldQueryVolumeBounds(q,hull)))return false;
    for(double time:{0.,.25,.5,1.})for(int32 vertex=0;vertex<8;++vertex)
    {
        const FVector3d corner((vertex&1)?2:-2,(vertex&2)?1:-1,(vertex&4)?1:-1);
        const auto point=FMath::Lerp(q.Start,q.End,time)+q.Rotation.RotateVector(corner);
        for(int32 d=0;d<3;++d)if(!TestTrue(TEXT("independent native OBB corners stay enclosed"),hull.Axis[d].Lower<=point[d]&&point[d]<=hull.Axis[d].Upper))return false;
    }
    TestTrue(TEXT("full box partition"),FReferenceBoundsCandidateSet::Select(scene,hull,budget,partition));
    TestTrue(TEXT("selection can retain its own inputs before clearing output"),FReferenceBoundsCandidateSet::Select(partition->GetScene(),partition->GetQueryHull(),budget,partition));
    FDistanceBudget shortBudget{2};TestFalse(TEXT("budget failure never publishes a partial exclusion"),FReferenceBoundsCandidateSet::Select(scene,hull,shortBudget,partition));
    TestFalse(TEXT("failed selection clears stale partition"),partition.IsValid());
    FIntervalTensorSurface copied;TestTrue(TEXT("budgeted immutable coefficient access"),scene->CopySurfaceForQuery(0,budget,copied));
    TestEqual(TEXT("provider CP retained"),copied.ControlPoints.Num(),1);
    TestFalse(TEXT("invalid provider cannot impersonate clear geometry"),scene->CopySurfaceForQuery(3,budget,copied));
    TestTrue(TEXT("invalid provider clears stale geometry"),copied.ControlPoints.IsEmpty());
    FWorldQuery cancelling;cancelling.Start=FVector3d(1.e16,0,0);cancelling.End=FVector3d(1,0,0);
    TestTrue(TEXT("Lerp DAG enclosure handles cancellation"),BuildFixedWorldQueryVolumeBounds(cancelling,hull));
    const auto actualEnd=FMath::Lerp(cancelling.Start,cancelling.End,1.);
    TestTrue(TEXT("native Lerp cancellation lies outside endpoint min/max yet inside DAG bound"),actualEnd.X==0&&hull.Axis[0].Lower<=actualEnd.X&&actualEnd.X<=hull.Axis[0].Upper);
    FWorldQuery nearUnit;nearUnit.Shape=EQueryShape::Box;nearUnit.HalfExtent=FVector3d(1,1,1);
    const double v=FMath::Sqrt(.5)*1.000001;nearUnit.Rotation=FQuat4d(0,0,v,v);
    if(!TestTrue(TEXT("counterexample uses a quaternion accepted by the native normalized predicate"),nearUnit.Rotation.IsNormalized()))return false;
    if(!TestTrue(TEXT("near-unit OBB retains both representations"),BuildFixedWorldQueryVolumeBounds(nearUnit,hull)))return false;
    Private::FBoxSweepContext axes;axes.Initialize(nearUnit);bool distinct=false;
    for(int32 vertex=0;vertex<8;++vertex)
    {
        const FVector3d corner((vertex&1)?1:-1,(vertex&2)?1:-1,(vertex&4)?1:-1);
        const auto raw=nearUnit.Rotation.RotateVector(corner);
        const auto normalized=axes.Axis[0]*corner.X+axes.Axis[1]*corner.Y+axes.Axis[2]*corner.Z;
        distinct|=raw!=normalized;
        for(int32 d=0;d<3;++d)TestTrue(TEXT("raw corner and normalized-axis corner both enclosed"),
            hull.Axis[d].Lower<=raw[d]&&raw[d]<=hull.Axis[d].Upper&&hull.Axis[d].Lower<=normalized[d]&&normalized[d]<=hull.Axis[d].Upper);
    }
    TestTrue(TEXT("the two native OBB representations actually differ"),distinct);
    FIntervalTensorSurface square;square.DegreeU=square.DegreeV=1;
    square.ControlPoints={Detail::Point(FVector3d(-1,-1,0)),Detail::Point(FVector3d(-1,1,0)),Detail::Point(FVector3d(1,-1,0)),Detail::Point(FVector3d(1,1,0))};
    TArray<FIntervalTensorSurface> squareInput={square};FIndexedIntervalUnionPtr squareScene;
    TestTrue(TEXT("finite-domain counterexample input"),FIndexedIntervalUnion::Build(MakeArrayView(squareInput),budget,squareScene));
    FWorldQuery tolerant;tolerant.bAuthorityOnly=true;
    tolerant.Start=FVector3d(1+.5*tolerant.DomainTolerance,0,1);tolerant.End=FVector3d(tolerant.Start.X,0,-1);
    TestTrue(TEXT("literal geometric ray volume"),BuildFixedWorldQueryVolumeBounds(tolerant,hull));
    TestTrue(TEXT("literal geometric partition"),FReferenceBoundsCandidateSet::Select(squareScene,hull,budget,partition));
    TestTrue(TEXT("literal volume is separated from the actual square"),partition&&partition->GetCandidateIndices().IsEmpty());
    FBoundedPlane plane;plane.HalfExtents=FVector2d(1,1);FWorldHit toleranceHit;
    TestFalse(TEXT("unlabelled finite plane is invalid"),plane.IsValid());
    TestFalse(TEXT("unlabelled finite plane kernel is refused"),FWorldQueryService::EvaluateFinitePlaneGeometry(tolerant,plane,false,toleranceHit));
    TestFalse(TEXT("invalid plane refusal publishes no hit"),toleranceHit.bHit);
    // Local synthetic geometry labels only; this pure kernel test admits no world provider.
    plane.SurfaceId=1;plane.FeatureId=1;
    TestTrue(TEXT("labelled finite plane fixture is valid"),plane.IsValid());
    TestTrue(TEXT("pure geometry keeps authority-only execution"),tolerant.bAuthorityOnly);
    TestFalse(TEXT("pure geometry does not promote provider eligibility"),plane.bAuthorityEligible);
    TestTrue(TEXT("existing tolerance-bearing face kernel evaluates"),FWorldQueryService::EvaluateFinitePlaneGeometry(tolerant,plane,false,toleranceHit));
    TestTrue(TEXT("geometric exclusion alone is NOT native kernel exclusion"),toleranceHit.bHit);
    q.Shape=static_cast<EQueryShape>(255);TestFalse(TEXT("unknown shape has no volume"),BuildFixedWorldQueryVolumeBounds(q,hull));
    TestFalse(TEXT("unknown volume is not marked valid"),Detail::ValidPoint(hull));
    TestTrue(TEXT("original numeric context restored"),context.Restore.Restore());return !HasAnyErrors();
}

#include "FixedShapeQueryBounds.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticCompleteUnionAxisSelectionTest,
 "IAmSpeed.AnalyticWorld.CompleteUnion.ImmutableAxisSelection",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticCompleteUnionAxisSelectionTest::RunTest(const FString&)
{
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture"),context.Ready))return false;
    const uint32 original=VectorGetControlRegister();
    for(uint32 flush:{0u,64u,32768u,32832u})
    {
        VectorSetControlRegister((original&~uint32(0x8040))|flush);
        const auto pointSurface=[](const FVector3d& p){FIntervalTensorSurface s;s.ControlPoints.Add(Detail::Point(p));return s;};
        TArray<FIntervalTensorSurface> surfaces;
        for(const auto& p:{FVector3d(0,0,0),FVector3d(1,0,0),FVector3d(1,1,0),FVector3d(2,0,0),FVector3d(-2,0,0),FVector3d(AnalyticCertified::FromBits(0x3FF0000000000001ULL),0,0)})surfaces.Add(pointSurface(p));
        FDistanceBudget buildBudget{4096};FIndexedIntervalUnionPtr scene;
        if(!TestTrue(TEXT("immutable finite provider scene"),FIndexedIntervalUnion::Build(surfaces,buildBudget,scene)))return false;
        FIntervalBezierCenterPath path;path.ControlPoints.Add(Detail::Point(FVector3d::ZeroVector));
        FReferenceCandidateSetPtr selected;FDistanceBudget selectedBudget{4096};
        if(!TestTrue(TEXT("new axis selector closes complete scene"),FReferenceCandidateSet::Select(scene,path,1,selectedBudget,selected)))return false;
        TArray<int32> expectedCandidates,expectedExcluded;const auto r2=AnalyticCertified::Square(Interval::Exact(1));
        for(int32 i=0;i<surfaces.Num();++i)
        {
            FIntervalPoint box;TestTrue(TEXT("oracle hull"),CandidateDetail::SurfaceHull(surfaces[i],box));
            const auto distance=CandidateDetail::BoxDistanceSquared(path.ControlPoints[0],box);
            TestTrue(TEXT("ordinary oracle valid"),AnalyticCertified::Valid(distance));
            (distance.Lower>r2.Upper?expectedExcluded:expectedCandidates).Add(i);
        }
        const auto candidates=selected->GetCandidateIndices();const auto excluded=selected->GetExcludedIndices();
        bool same=candidates.Num()==expectedCandidates.Num()&&excluded.Num()==expectedExcluded.Num();
        for(int32 i=0;i<candidates.Num()&&same;++i)same&=candidates[i]==expectedCandidates[i];
        for(int32 i=0;i<excluded.Num()&&same;++i)same&=excluded[i]==expectedExcluded[i];
        TestTrue(TEXT("finite ordinary oracle partitions exact in canonical order"),same);
        TestTrue(TEXT("same provider and candidate CP costs"),selectedBudget.Used==uint64(surfaces.Num()+candidates.Num()));
        TestTrue(TEXT("tangent retained and diagonal excluded by full fallback"),candidates.Contains(1)&&excluded.Contains(2));
        auto hull=Detail::Point(FVector3d::ZeroVector);FReferenceBoundsCandidateSetPtr bounds;FDistanceBudget boundsBudget{4096};
        if(!TestTrue(TEXT("validated immutable bounds selector"),FReferenceBoundsCandidateSet::Select(scene,hull,boundsBudget,bounds)))return false;
        TestTrue(TEXT("bounds keeps closed contact and same scan costs"),bounds->GetCandidateIndices().Num()==1&&bounds->GetCandidateIndices()[0]==0&&boundsBudget.Used==uint64(surfaces.Num()));
        const double max=AnalyticCertified::FromBits(0x7FEFFFFFFFFFFFFFULL);
        TArray<FIntervalTensorSurface> maxSurfaces{pointSurface(FVector3d(max,0,0))};FIndexedIntervalUnionPtr maxScene;FDistanceBudget maxBuild{32};
        TestTrue(TEXT("max finite immutable box"),FIndexedIntervalUnion::Build(maxSurfaces,maxBuild,maxScene));
        FIntervalBezierCenterPath maxPath;maxPath.ControlPoints.Add(Detail::Point(FVector3d(max,0,0)));FDistanceBudget maxBudget{2};FReferenceCandidateSetPtr maxResult;
        TestTrue(TEXT("no expanded-hull overflow; exact two-unit candidate floor retained"),FReferenceCandidateSet::Select(maxScene,maxPath,1,maxBudget,maxResult)&&maxBudget.Used==2&&maxBudget.Remaining==0&&maxResult->GetCandidateIndices().Num()==1);
        TArray<FIntervalTensorSurface> hugeSurfaces{pointSurface(FVector3d(1.e200,0,0))};FIndexedIntervalUnionPtr hugeScene;FDistanceBudget hugeBuild{32};
        TestTrue(TEXT("huge finite axis box"),FIndexedIntervalUnion::Build(hugeSurfaces,hugeBuild,hugeScene));FDistanceBudget hugeBudget{1};FReferenceCandidateSetPtr hugeResult;
        TestTrue(TEXT("new strict axis proof closes before an unattempted overflowing square"),FReferenceCandidateSet::Select(hugeScene,path,1,hugeBudget,hugeResult)&&hugeBudget.Used==1&&hugeResult->GetExcludedIndices().Num()==1);
        TArray<FIntervalTensorSurface> arithmeticSurfaces{pointSurface(FVector3d(max,1.e200,0))};FIndexedIntervalUnionPtr arithmeticScene;FDistanceBudget arithmeticBuild{32};
        TestTrue(TEXT("arithmetic fixture scene"),FIndexedIntervalUnion::Build(arithmeticSurfaces,arithmeticBuild,arithmeticScene));
        FIntervalBezierCenterPath opposite;opposite.ControlPoints.Add(Detail::Point(FVector3d(-max,0,0)));FDistanceBudget arithmeticBudget{32};auto denied=selected;
        TestTrue(TEXT("attempted first-axis subtraction failure stays terminal before another separation axis"),!FReferenceCandidateSet::Select(arithmeticScene,opposite,1,arithmeticBudget,denied)&&!denied);
        TArray<FIntervalTensorSurface> twoFar{pointSurface(FVector3d(2,0,0)),pointSurface(FVector3d(3,0,0))};FIndexedIntervalUnionPtr floorScene;FDistanceBudget floorBuild{32};
        TestTrue(TEXT("floor fixture"),FIndexedIntervalUnion::Build(twoFar,floorBuild,floorScene));FDistanceBudget floor{2,5,1};denied=selected;
        TestTrue(TEXT("unchanged provider floor has no partial output or refund"),!FReferenceCandidateSet::Select(floorScene,path,1,floor,denied)&&!denied&&floor.Used==6&&floor.Remaining==1);
    }
    VectorSetControlRegister(original);return true;
}

namespace
{
using namespace Speed::Analytic::Certified;
// Frozen Source66 linear oracle, deliberately independent of BVH/envelope.
    static bool Source66RadiusPredicate(const FIntervalPoint& center,const FIntervalPoint& box,
        double radius,Interval radiusSquared,bool& separate)
    {
        separate=false;auto sum=Interval::Exact(0);
        for(int32 d=0;d<3;++d)
        {
            auto gap=Interval::Exact(0);
            if(center.Axis[d].Lower>box.Axis[d].Upper)
                gap=AnalyticCertified::Subtract(Interval::Exact(center.Axis[d].Lower),Interval::Exact(box.Axis[d].Upper));
            else if(box.Axis[d].Lower>center.Axis[d].Upper)
                gap=AnalyticCertified::Subtract(Interval::Exact(box.Axis[d].Lower),Interval::Exact(center.Axis[d].Upper));
            if(!AnalyticCertified::Valid(gap))return false; // Attempted failure is terminal.
            // NEW necessary separation theorem, before any square is attempted.
            // Strictness retains tangency; no expanded center/overflow fallback.
            if(gap.Lower>radius){separate=true;return true;}
            sum=AnalyticCertified::Add(sum,AnalyticCertified::Square(gap));
            if(!AnalyticCertified::Valid(sum))return false;
        }
        separate=sum.Lower>radiusSquared.Upper;return true;
    }

}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticRetainedBvhPartitionTest,
 "IAmSpeed.AnalyticWorld.CompleteUnion.RetainedBvhCompletePartition",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticRetainedBvhPartitionTest::RunTest(const FString&)
{
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("guarded RNE fixture"),context.Ready))return false;
    const uint32 original=VectorGetControlRegister();
    const auto point=[](const FVector3d& p){FIntervalTensorSurface s;s.ControlPoints.Add(Detail::Point(p));return s;};
    const auto same=[](const auto& a,const TArray<int32>& b)
    {if(a.Num()!=b.Num())return false;for(int32 i=0;i<a.Num();++i)if(a[i]!=b[i])return false;return true;};
    for(uint32 flush:{0u,64u,32768u,32832u})
    {
        VectorSetControlRegister((original&~uint32(0x8040))|flush);
        TArray<FIntervalTensorSurface> surfaces;
        surfaces.Add(point(FVector3d::ZeroVector));surfaces.Add(point(FVector3d(1,0,0)));
        surfaces.Add(point(FVector3d(AnalyticCertified::FromBits(0x3ff0000000000001ULL),0,0)));
        surfaces.Add(point(FVector3d(1,1,0)));surfaces.Add(point(FVector3d(-1,0,0)));
        for(int32 i=5;i<2992;++i)surfaces.Add(point(FVector3d((i%101)*20+40,(i/101)*20+40,(i%7)*20)));
        // Broad interval CP family, not the nominal point used by a proxy mesh.
        surfaces[2991].ControlPoints[0].Axis[0]={-2,2,true};
        surfaces[2991].ControlPoints[0].Axis[1]={-2,2,true};
        surfaces[2991].ControlPoints[0].Axis[2]={-2,2,true};
        FDistanceBudget build{200000};FIndexedIntervalUnionPtr scene;
        if(!TestTrue(TEXT("2992 actual CP hulls indexed"),FIndexedIntervalUnion::Build(surfaces,build,scene)))return false;
        TArray<FIntervalPoint> boxes;for(const auto& surface:surfaces){FIntervalPoint box;TestTrue(TEXT("oracle CP hull"),CandidateDetail::SurfaceHull(surface,box));boxes.Add(box);}
        const double tiny=AnalyticCertified::FromBits(AnalyticCertified::MinNormal);
        for(const auto& center:{Detail::Point(FVector3d::ZeroVector),Detail::Point(FVector3d(42,42,0)),
            Detail::Point(FVector3d(-500,-500,-500)),Detail::Point(FVector3d(1,0,0))})
        for(double radius:{tiny,1.,2.,5000.})
        {
            FIntervalBezierCenterPath path;path.ControlPoints.Add(center);
            const auto r2=AnalyticCertified::Square(Interval::Exact(radius));
            FDistanceBudget actualBudget{200000},oracleBudget{200000};FReferenceCandidateSetPtr selected;FReferenceSelectionWork work;
            TArray<int32> candidates,excluded;
            for(int32 i=0;i<surfaces.Num();++i)
            {
                TestTrue(TEXT("oracle provider debit"),oracleBudget.Consume());bool separate=false;
                TestTrue(TEXT("frozen old radius arithmetic"),Source66RadiusPredicate(center,boxes[i],radius,r2,separate));
                if(separate)excluded.Add(i);else{for(int32 cp=0;cp<surfaces[i].ControlPoints.Num();++cp)TestTrue(TEXT("oracle CP debit"),oracleBudget.Consume());candidates.Add(i);}
            }
            TestTrue(TEXT("BVH radius selection closes"),FReferenceCandidateSet::Select(scene,path,radius,actualBudget,selected,&work));
            if(selected)
            {
                TestTrue(TEXT("exact old canonical candidate partition"),same(selected->GetCandidateIndices(),candidates)&&same(selected->GetExcludedIndices(),excluded));
                TestTrue(TEXT("old provider/CP debits and remaining preserved"),actualBudget.Used==oracleBudget.Used&&actualBudget.Remaining==oracleBudget.Remaining);
                TestTrue(TEXT("explicit BVH, never automatic 2992 grid"),work.UsedBvh&&work.NodesVisited>0);
                if(radius==1. && center.Axis[0].Lower==0 && center.Axis[1].Lower==0)
                    TestTrue(TEXT("localized geometry prunes most expensive radius predicates"),work.ProviderBoundsTested<2992/8&&work.RadiusPredicates<2992/8);
            }
            FReferenceBoundsCandidateSetPtr bounded;FDistanceBudget boundsBudget{200000};FReferenceSelectionWork boundsWork;
            TestTrue(TEXT("closed AABB selection"),FReferenceBoundsCandidateSet::Select(scene,center,boundsBudget,bounded,&boundsWork));
            TArray<int32> bc,be;
            for(int32 i=0;i<boxes.Num();++i){bool separate=false;for(int32 d=0;d<3;++d)separate|=center.Axis[d].Upper<boxes[i].Axis[d].Lower||boxes[i].Axis[d].Upper<center.Axis[d].Lower;(separate?be:bc).Add(i);}
            if(bounded)TestTrue(TEXT("AABB complete exact ordered partition and old costs"),same(bounded->GetCandidateIndices(),bc)&&same(bounded->GetExcludedIndices(),be)&&boundsBudget.Used==2992&&boundsWork.UsedBvh);
        }
        FIntervalBezierCenterPath zero;zero.ControlPoints.Add(Detail::Point(FVector3d::ZeroVector));
        for(const auto initial:{FDistanceBudget{2992},FDistanceBudget{2993},FDistanceBudget{2992,7,1},
            FDistanceBudget{9000,MAX_uint64-2,0},FDistanceBudget{9000,0,-1}})
        {
            auto actual=initial,oracle=initial;FReferenceCandidateSetPtr selected;bool expected=true;
            const auto r2=AnalyticCertified::Square(Interval::Exact(1));
            for(int32 i=0;i<surfaces.Num()&&expected;++i)
            {
                if(!oracle.Consume()){expected=false;break;}bool separate=false;
                if(!Source66RadiusPredicate(zero.ControlPoints[0],boxes[i],1,r2,separate)){expected=false;break;}
                if(!separate)for(int32 cp=0;cp<surfaces[i].ControlPoints.Num();++cp)if(!oracle.Consume()){expected=false;break;}
            }
            const bool got=FReferenceCandidateSet::Select(scene,zero,1,actual,selected);
            TestTrue(TEXT("partial provider/CP/floor/overflow refusals unchanged, no partial publication"),got==expected&&bool(selected)==expected&&actual.Used==oracle.Used&&actual.Remaining==oracle.Remaining&&actual.ConsumptionFloor==oracle.ConsumptionFloor);
        }
        // A single unsafe leaf disables radius skipping BEFORE new arithmetic;
        // the old first attempted overflow must still be a terminal refusal.
        const double max=AnalyticCertified::FromBits(0x7fefffffffffffffULL);
        surfaces[0]=point(FVector3d(max,1.e200,0));FDistanceBudget hugeBuild{200000};FIndexedIntervalUnionPtr huge;
        TestTrue(TEXT("extreme scene still immutable and valid"),FIndexedIntervalUnion::Build(surfaces,hugeBuild,huge));
        FIntervalBezierCenterPath opposite;opposite.ControlPoints.Add(Detail::Point(FVector3d(-max,0,0)));
        FDistanceBudget deniedBudget{200000};FReferenceCandidateSetPtr denied;FReferenceSelectionWork deniedWork;
        TestTrue(TEXT("old numeric failure never hidden by BVH exclusion"),!FReferenceCandidateSet::Select(huge,opposite,1,deniedBudget,denied,&deniedWork)&&!denied&&!deniedWork.UsedBvh&&deniedBudget.Used==1);
        TestTrue(TEXT("selector never changes RNE/masks/FTZ/DAZ controls"),(VectorGetControlRegister()&~uint32(0x3f))==(((original&~uint32(0x8040))|flush)&~uint32(0x3f)));
    }
    VectorSetControlRegister(original);return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticValidatedFinitePlaneGeometryTest,
 "IAmSpeed.AnalyticWorld.CompleteUnion.ValidatedFinitePlaneGeometry",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticValidatedFinitePlaneGeometryTest::RunTest(const FString&)
{
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture"),context.Ready))return false;
    const uint32 original=VectorGetControlRegister();
    const auto same=[](const FWorldHit& a,const FWorldHit& b)
    {
        if(a.bHit!=b.bHit||a.bStartPenetrating!=b.bStartPenetrating||a.bSurfaceNormalMayVary!=b.bSurfaceNormalMayVary||
            a.QueryFeatureKind!=b.QueryFeatureKind||a.SurfaceFeatureKind!=b.SurfaceFeatureKind||a.QueryFeatureIndex!=b.QueryFeatureIndex||
            a.SurfaceFeatureIndex!=b.SurfaceFeatureIndex||a.SourceId!=b.SourceId||a.SurfaceId!=b.SurfaceId||a.FeatureId!=b.FeatureId||
            a.PrimitiveId!=b.PrimitiveId||a.CanonicalGroupId!=b.CanonicalGroupId||a.MaterialId!=b.MaterialId)return false;
        const double x[]={a.Time,a.PenetrationDepth,a.GeometricErrorBoundCm,a.AdditionalResidualAgreementAllowanceCm,
            a.Location.X,a.Location.Y,a.Location.Z,a.Point.X,a.Point.Y,a.Point.Z,a.QueryPoint.X,a.QueryPoint.Y,a.QueryPoint.Z,a.Normal.X,a.Normal.Y,a.Normal.Z};
        const double y[]={b.Time,b.PenetrationDepth,b.GeometricErrorBoundCm,b.AdditionalResidualAgreementAllowanceCm,
            b.Location.X,b.Location.Y,b.Location.Z,b.Point.X,b.Point.Y,b.Point.Z,b.QueryPoint.X,b.QueryPoint.Y,b.QueryPoint.Z,b.Normal.X,b.Normal.Y,b.Normal.Z};
        for(int32 i=0;i<UE_ARRAY_COUNT(x);++i)if(AnalyticCertified::Bits(x[i])!=AnalyticCertified::Bits(y[i]))return false;
        return true;
    };
    for(uint32 buildMode:{0u,64u,32768u,32832u})
    {
        VectorSetControlRegister((original&~uint32(0x8040))|buildMode);
        FBoundedPlane plane;plane.SourceId=19;plane.SurfaceId=11;plane.FeatureId=13;plane.PrimitiveId=17;plane.MaterialId=7;
        plane.HalfExtents=FVector2d(10,10);plane.BlockingChannels=MAX_uint64;
        // Valid SIMPLE, NONCONVEX domain: validity must never imply convexity.
        plane.DomainVertices={FVector2d(-2,-2),FVector2d(2,-2),FVector2d(2,2),FVector2d(0,2),FVector2d(0,0),FVector2d(-2,0)};
        TestTrue(TEXT("nonconvex finite plane is valid"),plane.IsValid());
        auto geometry=FWorldQueryService::ValidateFinitePlaneGeometry(plane);
        if(!TestTrue(TEXT("checked owned immutable geometry"),geometry.IsValid()))return false;
        FWorldQuery ray;ray.bAuthorityOnly=true;ray.bUseFiniteContactDomain=true;ray.DomainTolerance=0;ray.InitialOverlapTolerance=0;
        ray.Start=FVector3d(0,-1,4);ray.End=FVector3d(0,-1,-4);
        TArray<FWorldQuery> queries;queries.Add(ray);
        auto q=ray;q.Start.X=q.End.X=8;queries.Add(q); // finite-domain miss
        q=ray;q.Shape=EQueryShape::Sphere;q.Radius=.25;queries.Add(q);
        q=ray;q.Shape=EQueryShape::Box;q.HalfExtent=FVector3d(.25,.25,.25);queries.Add(q);
        q=ray;q.ReferenceNormal=FVector3d(0,0,-1);q.MinimumReferenceNormalDot=.9;queries.Add(q);
        q=ray;q.bApplyCollisionFilter=true;q.BlockingObjectTypes=0;queries.Add(q);
        q=ray;q.RequiredSourceId=123456;queries.Add(q);
        q=ray;q.bAuthorityOnly=false;queries.Add(q);
        q=ray;q.Shape=EQueryShape::Sphere;q.Radius=0;queries.Add(q);
        q=ray;q.Start.X=std::numeric_limits<double>::quiet_NaN();queries.Add(q);
        for(uint32 useMode:{0u,64u,32768u,32832u})
        {
            VectorSetControlRegister((original&~uint32(0x8040))|useMode);
            for(const auto& query:queries)
            {
                FWorldHit a,b;a.bHit=b.bHit=true;EFinitePlaneValidationPath path=EFinitePlaneValidationPath::Unavailable;
                const bool x=FWorldQueryService::EvaluateFinitePlaneGeometry(query,plane,false,a);
                const bool y=FWorldQueryService::EvaluateValidatedFinitePlaneGeometry(query,geometry,false,b,&path);
                TestTrue(TEXT("generic/certified result and every hit field bit-identical"),x==y&&same(a,b));
                if(query.bAuthorityOnly)TestTrue(TEXT("same controls reuse; other controls perform original validation"),path==(useMode==buildMode?EFinitePlaneValidationPath::ReusedGeometry:EFinitePlaneValidationPath::RevalidatedGeometry));
            }
            for(EQueryShape shape:{EQueryShape::Sphere,EQueryShape::Box})
            {
                q=ray;q.Shape=shape;q.Radius=.25;q.HalfExtent=FVector3d(.25,.25,.25);q.Start=q.End=FVector3d(0,-1,.1);
                FWorldHit a,b;const bool x=FWorldQueryService::EvaluateFinitePlaneGeometry(q,plane,true,a);
                const bool y=FWorldQueryService::EvaluateValidatedFinitePlaneGeometry(q,geometry,true,b);
                TestTrue(TEXT("deepest overlap shares original kernel and fields"),x==y&&same(a,b));
            }
            TestTrue(TEXT("production never changes numeric controls"),(VectorGetControlRegister()&~uint32(0x3f))==(((original&~uint32(0x8040))|useMode)&~uint32(0x3f)));
        }
        VectorSetControlRegister((original&~uint32(0x8040))|buildMode);
        FWorldHit reference;TestTrue(TEXT("original ray hits its finite face"),FWorldQueryService::EvaluateFinitePlaneGeometry(ray,plane,false,reference)&&reference.bHit);
        const auto originalPlane=plane;
        plane.SurfaceId=0;plane.DomainVertices.Reset(); // source mutation cannot alter the owned copy
        FWorldHit owned;TestTrue(TEXT("certificate owns full plane beyond input mutation"),FWorldQueryService::EvaluateValidatedFinitePlaneGeometry(ray,geometry,false,owned)&&same(reference,owned));
        TestFalse(TEXT("invalid mutated input cannot create a new handle"),FWorldQueryService::ValidateFinitePlaneGeometry(plane).IsValid());
        auto crossed=originalPlane;crossed.DomainVertices={FVector2d(-2,-2),FVector2d(2,2),FVector2d(-2,2),FVector2d(2,-2)};
        TestFalse(TEXT("self-intersecting polygon never published"),FWorldQueryService::ValidateFinitePlaneGeometry(crossed).IsValid());
        auto invalid=originalPlane;invalid.Normal.X=std::numeric_limits<double>::quiet_NaN();
        TestFalse(TEXT("nonfinite plane never published"),FWorldQueryService::ValidateFinitePlaneGeometry(invalid).IsValid());
        owned.bHit=true;EFinitePlaneValidationPath path=EFinitePlaneValidationPath::ReusedGeometry;
        TestTrue(TEXT("null handle refuses and clears old hit"),!FWorldQueryService::EvaluateValidatedFinitePlaneGeometry(ray,{},false,owned,&path)&&!owned.bHit&&path==EFinitePlaneValidationPath::Unavailable);
    }
    VectorSetControlRegister(original);return true;
}


IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticFixedSphereSegmentBoxSeparationTest,
 "IAmSpeed.AnalyticWorld.CompleteUnion.FixedSphereSegmentBoxSeparation",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticFixedSphereSegmentBoxSeparationTest::RunTest(const FString&)
{
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("guarded segment proof fixture"),context.Ready))return false;
    const uint32 original=VectorGetControlRegister();ON_SCOPE_EXIT{VectorSetControlRegister(original);};
    const auto point=[](FVector3d p){FIntervalTensorSurface s;s.ControlPoints.Add(Detail::Point(p));return s;};
    TArray<FIntervalTensorSurface> surfaces{point(FVector3d(0,8,0)),point(FVector3d(4,4,0)),point(FVector3d(4,4,1)),point(FVector3d(8,8,0))};
    for(uint32 flush:{0u,64u,32768u,32832u})for(uint32 sticky:{0u,63u})
    {
        VectorSetControlRegister((original&~uint32(0x807f))|flush|sticky);
        FDistanceBudget build{128};FIndexedIntervalUnionPtr scene;
        if(!TestTrue(TEXT("immutable CP scene"),FIndexedIntervalUnion::Build(surfaces,build,scene)))return false;
        for(bool reverse:{false,true})
        {
            FIntervalBezierCenterPath path;
            if(!TestTrue(TEXT("exact authored degree1"),CenterDetail::Linear(reverse?FVector3d(8,8,0):FVector3d::ZeroVector,reverse?FVector3d::ZeroVector:FVector3d(8,8,0),path)))return false;
            FDistanceBudget oldBudget{1024},newBudget{1024};FReferenceCandidateSetPtr oldSet,newSet;FReferenceSelectionWork work;
            if(!TestTrue(TEXT("general oracle unchanged"),FReferenceCandidateSet::Select(scene,path,1.,oldBudget,oldSet)))return false;
            if(!TestTrue(TEXT("strict specialized proof"),FReferenceCandidateSet::SelectFixedLinearSphere(scene,path,1.,newBudget,newSet,&work)))return false;
            TestTrue(TEXT("oblique false positive actually removed, tangent and endpoint retained"),oldSet->GetCandidateIndices().Num()==4&&newSet->GetCandidateIndices().Num()==3&&
                newSet->GetCandidateIndices()[0]==1&&newSet->GetCandidateIndices()[1]==2&&newSet->GetCandidateIndices()[2]==3&&newSet->GetExcludedIndices().Num()==1&&newSet->GetExcludedIndices()[0]==0);
            TestTrue(TEXT("actual bounded segment work, canonical complete partition"),work.SegmentTests==4&&work.SegmentExcluded==1&&work.CandidatesBeforeSegment==4&&work.CandidatesAfterSegment==3&&
                work.SegmentAxes<=12&&work.SegmentPairs<=64&&work.UsedBvh&&work.NodesVisited>0&&newSet->GetCandidateIndices().Num()+newSet->GetExcludedIndices().Num()==scene->Num());
        }
        FIntervalBezierCenterPath stationary;CenterDetail::Linear(FVector3d(4,4,0),FVector3d(4,4,-0.),stationary);
        FDistanceBudget stationaryBudget{1024};FReferenceCandidateSetPtr stationarySet;FReferenceSelectionWork stationaryWork;
        TestTrue(TEXT("zero/negative-zero motion never divides; tangent retained"),FReferenceCandidateSet::SelectFixedLinearSphere(scene,stationary,1.,stationaryBudget,stationarySet,&stationaryWork)&&stationarySet->GetCandidateIndices().Num()==2&&stationaryWork.SegmentUncertain==stationaryWork.SegmentTests);
        // A subnormal authored coefficient is widened by Exact before admission;
        // a direction interval containing zero supplies no time constraint.
        FIntervalBezierCenterPath tiny;const double sub=AnalyticCertified::FromBits(1);
        CenterDetail::Linear(FVector3d(4,4,sub),FVector3d(4,4,0),tiny);
        FDistanceBudget tinyBudget{1024};FReferenceCandidateSetPtr tinySet;FReferenceSelectionWork tinyWork;
        TestTrue(TEXT("subnormal/cross-zero coefficient family is conservative"),FReferenceCandidateSet::SelectFixedLinearSphere(scene,tiny,1.,tinyBudget,tinySet,&tinyWork)&&tinySet->GetCandidateIndices().Num()>=2);
        // A nonzero direction with tiny components must neither divide by zero
        // nor produce a quotient overflow. The helper uses only cross products.
        FIntervalBezierCenterPath minPath;const double mn=AnalyticCertified::FromBits(AnalyticCertified::MinNormal);
        CenterDetail::Linear(FVector3d(0,0,0),FVector3d(mn,mn,0),minPath);
        FIntervalTensorSurface origin;origin.ControlPoints.Add(Detail::Point(FVector3d::ZeroVector));
        FDistanceBudget minBuild{32};FIndexedIntervalUnionPtr minScene;
        if(!TestTrue(TEXT("near-origin smallest-normal fixture"),FIndexedIntervalUnion::Build(MakeArrayView(&origin,1),minBuild,minScene)))return false;
        FDistanceBudget minBudget{1024};FReferenceCandidateSetPtr minSet;FReferenceSelectionWork minWork;
        TestTrue(TEXT("smallest-normal motion reaches helper, never quotient overflow"),FReferenceCandidateSet::SelectFixedLinearSphere(minScene,minPath,1.,minBudget,minSet,&minWork)&&
            minWork.SegmentTests==1&&minWork.SegmentAxes==3&&minWork.SegmentPairs>0&&minWork.SegmentPairs<=16&&minSet->GetCandidateIndices().Num()==1);
    }
    return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticFixedSphereSegmentSelectionAndBudgetTest,
 "IAmSpeed.AnalyticWorld.CompleteUnion.FixedSphereSegmentSelectionAndBudget",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticFixedSphereSegmentSelectionAndBudgetTest::RunTest(const FString&)
{
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE segment budget fixture"),context.Ready))return false;
    FIntervalTensorSurface s;s.ControlPoints.Add(Detail::Point(FVector3d(4,4,0)));FDistanceBudget build{128};FIndexedIntervalUnionPtr scene;
    if(!TestTrue(TEXT("one immutable provider"),FIndexedIntervalUnion::Build(MakeArrayView(&s,1),build,scene)))return false;
    FIntervalBezierCenterPath path;CenterDetail::Linear(FVector3d::ZeroVector,FVector3d(8,8,0),path);
    FDistanceBudget full{1024};FReferenceCandidateSetPtr set;FReferenceSelectionWork work;
    TestTrue(TEXT("closed paid proof"),FReferenceCandidateSet::SelectFixedLinearSphere(scene,path,1.,full,set,&work));
    TestTrue(TEXT("exact local debit includes providers, actual axes/pairs and admitted CP"),full.Used==2+work.SegmentAxes+work.SegmentPairs&&full.Remaining==1024-int32(full.Used));
    for(int32 allowance=1;allowance<int32(full.Used);++allowance)
    {
        FDistanceBudget shortBudget{allowance};auto stale=set;FReferenceSelectionWork partial;
        TestFalse(TEXT("insufficient paid proof never publishes prior candidate set"),FReferenceCandidateSet::SelectFixedLinearSphere(scene,path,1.,shortBudget,stale,&partial));
        TestFalse(TEXT("no partial output or budget refund"),stale.IsValid());TestTrue(TEXT("bounded debit"),shortBudget.Used<=uint64(allowance)&&shortBudget.Remaining>=0);
    }
    FDistanceBudget floor{1024,7,1023};auto stale=set;
    TestFalse(TEXT("floor respected"),FReferenceCandidateSet::SelectFixedLinearSphere(scene,path,1.,floor,stale));TestFalse(TEXT("floor has no output"),stale.IsValid());
    auto malformed=path;malformed.Degree=0;FDistanceBudget bad{1024};
    TestFalse(TEXT("new API refuses degree mismatch"),FReferenceCandidateSet::SelectFixedLinearSphere(scene,malformed,1.,bad,stale));
    malformed=path;malformed.ControlPoints[0].Axis[0].Known=false;bad={1024};
    TestFalse(TEXT("unknown coefficient refused"),FReferenceCandidateSet::SelectFixedLinearSphere(scene,malformed,1.,bad,stale));
    malformed=path;malformed.ControlPoints[0].Axis[0]={AnalyticCertified::FromBits(AnalyticCertified::Infinity),0,true};bad={1024};
    TestFalse(TEXT("nonfinite coefficient refused"),FReferenceCandidateSet::SelectFixedLinearSphere(scene,malformed,1.,bad,stale));
    bad={1024};TestFalse(TEXT("NaN radius refused"),FReferenceCandidateSet::SelectFixedLinearSphere(scene,path,AnalyticCertified::FromBits(AnalyticCertified::Infinity|1),bad,stale));
    // Validate no auxiliary flags escape: original radius predicate runs first
    // on each lane; the helper is an independent save/restore observation.
    FIntervalBezierCenterPath equal;CenterDetail::Linear(FVector3d(4,4,0),FVector3d(4,4,0),equal);
    const uint32 original=VectorGetControlRegister();ON_SCOPE_EXIT{VectorSetControlRegister(original);};
    for(uint32 flush:{0u,64u,32768u,32832u})for(uint32 sticky:{0u,63u})
    {
        const uint32 initial=(original&~uint32(0x807f))|flush|sticky;
        VectorSetControlRegister(initial);FDistanceBudget a{1024};FReferenceCandidateSetPtr oracle;
        TestTrue(TEXT("general FP oracle"),FReferenceCandidateSet::Select(scene,equal,1.,a,oracle));const uint32 expected=VectorGetControlRegister();
        VectorSetControlRegister(initial);FDistanceBudget b{1024};FReferenceCandidateSetPtr actual;
        TestTrue(TEXT("specialized FP parity"),FReferenceCandidateSet::SelectFixedLinearSphere(scene,equal,1.,b,actual));
        TestTrue(TEXT("complete vector state preserved versus original arithmetic"),VectorGetControlRegister()==expected);
    }
    return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCompactCandidatePartitionBudgetTest,
 "IAmSpeed.AnalyticWorld.CompleteUnion.CompactPartitionExactBudget",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCompactCandidatePartitionBudgetTest::RunTest(const FString&)
{
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("guarded RNE fixture"),context.Ready))return false;
    // Complete partial-failure semantics, not merely successful all-or-none spend.
    for(int32 remaining:{-1,0,1,2,8,MAX_int32})for(int32 floor:{-1,0,1,7,MAX_int32})
    for(uint64 used:{0ull,MAX_uint64-2,MAX_uint64-1,MAX_uint64})for(uint64 count:{0ull,1ull,2ull,7ull,9ull})
    {
        FDistanceBudget a{remaining,used,floor},b=a;bool expected=true;
        for(uint64 i=0;i<count;++i)if(!a.Consume()){expected=false;break;}
        const bool actual=CandidateDetail::ConsumeRepeated(b,count);
        if(!TestTrue(TEXT("block equals each original Consume including failure"),actual==expected&&a.Remaining==b.Remaining&&a.Used==b.Used&&a.ConsumptionFloor==b.ConsumptionFloor))return false;
    }
    const uint32 original=VectorGetControlRegister();
    const auto same=[](const auto& a,const auto& b){if(a.Num()!=b.Num())return false;for(int32 i=0;i<a.Num();++i)if(a[i]!=b[i])return false;return true;};
    for(uint32 flush:{0u,64u,32768u,32832u})
    {
        VectorSetControlRegister((original&~uint32(0x8040|0x3f))|flush);
        TArray<FIntervalTensorSurface> surfaces;
        for(int32 i=0;i<19;++i)
        {
            FIntervalTensorSurface s;s.DegreeU=i%3;s.DegreeV=(i/3)%2;
            // Three retained providers have different CP counts. Gaps on both
            // sides make low-budget exhaustion land in a gap or in a CP copy.
            const FVector3d p=(i==2||i==8||i==15)?FVector3d(.25,0,0):FVector3d(100+i,100,100);
            for(int32 k=0;k<(s.DegreeU+1)*(s.DegreeV+1);++k)s.ControlPoints.Add(Detail::Point(p));
            if(i==4)
            {
                s.DegreeU=s.DegreeV=1;s.ControlPoints.Reset();
                for(const auto& corner:{FVector3d(-.2,.8,0),FVector3d(.8,-.2,0),FVector3d(.8,1.8,0),FVector3d(1.8,.8,0)})
                    s.ControlPoints.Add(Detail::Point(corner));
            }
            surfaces.Add(MoveTemp(s));
        }
        FDistanceBudget build{10000};FIndexedIntervalUnionPtr scene;
        if(!TestTrue(TEXT("immutable variable-CP union"),FIndexedIntervalUnion::Build(surfaces,build,scene)))return false;
        FIntervalBezierCenterPath path;if(!CenterDetail::Linear(FVector3d::ZeroVector,FVector3d(.125,0,0),path))return false;
        for(bool edge:{false,true})for(int32 cap=0;cap<=750;++cap)
        {
            FDistanceBudget a{cap,5,3},b=a;FReferenceCandidateSetPtr explicitSet,compactSet;
            FReferenceSelectionWork wa,wb;
            VectorSetControlRegister((original&~uint32(0x8040|0x3f))|flush);
            const bool ra=FReferenceCandidateSet::Select(scene,path,.125,a,explicitSet,&wa,edge,false);const uint32 fa=VectorGetControlRegister();
            VectorSetControlRegister((original&~uint32(0x8040|0x3f))|flush);
            const bool rb=FReferenceCandidateSet::Select(scene,path,.125,b,compactSet,&wb,edge,true);const uint32 fb=VectorGetControlRegister();
            if(!TestTrue(TEXT("result, complete work, null output and sticky bits identical at every cap"),ra==rb&&a.Remaining==b.Remaining&&a.Used==b.Used&&fa==fb&&bool(explicitSet)==bool(compactSet)&&wa.NodesVisited==wb.NodesVisited&&wa.ProviderBoundsTested==wb.ProviderBoundsTested&&wa.RadiusPredicates==wb.RadiusPredicates&&wa.UsedBvh==wb.UsedBvh))return false;
            if(ra)
            {
                if(!TestTrue(TEXT("same ordered complete partition"),same(explicitSet->GetCandidateIndices(),compactSet->GetCandidateIndices())&&same(explicitSet->GetExcludedIndices(),compactSet->GetExcludedIndices())&&compactSet->HasCompactExclusions()))return false;
                const auto view=compactSet->GetExcludedIndices();int32 ordinal=0;
                for(int32 index:view){if(!TestTrue(TEXT("forward iteration and ordinal agree"),index==view[ordinal++]&&view.Contains(index)))return false;}
                for(int32 i:compactSet->GetCandidateIndices())if(!TestFalse(TEXT("no included index appears excluded"),view.Contains(i)))return false;
                if(!TestTrue(TEXT("count and bounds are canonical"),ordinal==view.Num()&&!view.Contains(-1)&&!view.Contains(scene->Num())))return false;
                if(cap==750&&!edge)
                {
                    FDistanceBudget ma{10000},mb{10000};uint64 ba=0,bb=0;
                    if(!TestTrue(TEXT("compact removes actual allocation, same accounting work"),explicitSet->AccountOwnedQueryStorageBytes(ma,ba)&&compactSet->AccountOwnedQueryStorageBytes(mb,bb)&&ma.Used==mb.Used&&ba>=bb+uint64(view.Num())*sizeof(int32)))return false;
                    const auto sa=explicitSet->GetSurfacesForFixedSource(),sb=compactSet->GetSurfacesForFixedSource();
                    if(!TestEqual(TEXT("same copied candidate surfaces"),sa.Num(),sb.Num()))return false;
                    for(int32 surface=0;surface<sa.Num();++surface)
                    {
                        if(!TestTrue(TEXT("same degrees and original CP count"),sa[surface].DegreeU==sb[surface].DegreeU&&sa[surface].DegreeV==sb[surface].DegreeV&&sa[surface].ControlPoints.Num()==sb[surface].ControlPoints.Num()))return false;
                        for(int32 k=0;k<sa[surface].ControlPoints.Num();++k)for(int32 axis=0;axis<3;++axis)
                        {
                            const auto& x=sa[surface].ControlPoints[k].Axis[axis];const auto& y=sb[surface].ControlPoints[k].Axis[axis];
                            if(!TestTrue(TEXT("retained coefficient endpoints are bit identical"),x.Known==y.Known&&AnalyticCertified::Bits(x.Lower)==AnalyticCertified::Bits(y.Lower)&&AnalyticCertified::Bits(x.Upper)==AnalyticCertified::Bits(y.Upper)))return false;
                        }
                    }
                }
            }
        }
        // Invalid counters and Used near overflow retain the original fallback
        // branch and/or its exact partial failure; no compact admission invented.
        for(const auto initial:{FDistanceBudget{10000,0,-1},FDistanceBudget{10000,MAX_uint64-20,0},FDistanceBudget{10000,MAX_uint64-1,0},FDistanceBudget{10000,0,10001}})
        {
            auto a=initial,b=initial;FReferenceCandidateSetPtr x,y;
            const bool rx=FReferenceCandidateSet::Select(scene,path,.125,a,x,nullptr,true,false);
            const bool ry=FReferenceCandidateSet::Select(scene,path,.125,b,y,nullptr,true,true);
            if(!TestTrue(TEXT("invalid and overflow accounting unchanged"),rx==ry&&a.Used==b.Used&&a.Remaining==b.Remaining&&bool(x)==bool(y)))return false;
        }
        FIntervalBezierCenterPath emptyPath;CenterDetail::Linear(FVector3d(-100,-100,-100),FVector3d(-99,-100,-100),emptyPath);
        FDistanceBudget allBudget{10000};FReferenceCandidateSetPtr all;
        if(!TestTrue(TEXT("empty-candidate complement covers whole scene"),FReferenceCandidateSet::Select(scene,emptyPath,.125,allBudget,all,nullptr,false,true)&&all->HasCompactExclusions()&&all->GetCandidateIndices().IsEmpty()&&all->GetExcludedIndices().Num()==scene->Num()))return false;
        const auto borrowed=all->GetExcludedIndices();const auto owner=all;all.Reset();scene.Reset();surfaces.Reset();
        int32 next=0;for(int32 i:borrowed)if(!TestEqual(TEXT("borrow remains valid while actual owner retained"),i,next++))return false;
    }
    VectorSetControlRegister(original);TestTrue(TEXT("original context restored"),context.Restore.Restore());return !HasAnyErrors();
}
