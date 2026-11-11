#include "FinitePlaneTensorIntervals.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticTensorAffineIntervalTest,
 "IAmSpeed.AnalyticWorld.TensorIntervalInput.AffineRounding",
 EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FAnalyticTensorAffineIntervalTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture established"),context.Ready))return false;
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    const double small=AnalyticCertified::FromBits(0x3ca0000000000000ULL); //2^-53
    const double radius=AnalyticCertified::FromBits(0x3c90000000000000ULL); //2^-54
    const double next=AnalyticCertified::FromBits(0x3ff0000000000001ULL);
    FBoundedPlane plane;plane.Origin=FVector3d(1,0,0);plane.AxisU=FVector3d(1,0,0);plane.AxisV=FVector3d(0,1,0);
    plane.SourceId=11;plane.SurfaceId=22;plane.FeatureId=33;plane.PrimitiveId=44;plane.MaterialId=7;plane.ObjectType=2;plane.BlockingChannels=5;
    plane.bQueryCollisionEnabled=true;plane.bRequiresCompactOptIn=true;plane.bAuthorityEligible=false;
    plane.DomainVertices={FVector2d(small,0),FVector2d(1,0),FVector2d(small,1)};
    FPlaneTriangleIntervalNet triangle;const bool built=BuildPlaneTriangleIntervalNet(plane,FIntVector(0,1,2),triangle);
    TestTrue(TEXT("triangle affine coefficients enclosed before binary64 vertex rounding"),built);
    if(built)
    {
        const auto x=triangle.Surface.ControlPoints[0].Axis[0];
        TestTrue(TEXT("interval contains exact unrepresentable1+2^-53"),x.Lower<=1.&&x.Upper>=next);
        TestTrue(TEXT("parent identities and original domain indices preserved"),triangle.SourceId==11&&triangle.SurfaceId==22&&triangle.FeatureId==33&&triangle.PrimitiveId==44&&triangle.OriginalDomainIndices==FIntVector(0,1,2));
        TestTrue(TEXT("policy copied without promotion"),triangle.MaterialId==7&&triangle.ObjectType==2&&triangle.BlockingChannels==5&&triangle.bQueryCollisionEnabled&&triangle.bRequiresCompactOptIn&&!triangle.bAuthorityEligible);
        FIntervalTensorSurface exactPoint;exactPoint.ControlPoints.Add(triangle.Surface.ControlPoints[0]);TArray<FIntervalTensorSurface> points={exactPoint};FDistanceBudget b{128};
        const auto bounded=BoundTensorSphereSweep(MakeArrayView(points),FVector3d(1,0,0),FVector3d(1,0,0),radius,b,1./64.,0.,1);
        TestTrue(TEXT("uncertain coefficient cannot create a false universal inside"),bounded.Status==ETensorSphereTimeStatus::Unknown);
        volatile double rounded=plane.Origin.X+plane.AxisU.X*small;TestEqual(TEXT("naive vertex sum rounds to1"),double(rounded),1.);
        FTensorBezierSurface naive;naive.ControlPoints.Add(FVector3d(rounded,0,0));TArray<FTensorBezierSurface> naivePoints={naive};FDistanceBudget naiveBudget{16};
        const auto wrongModel=BoundTensorSphereSweep(MakeArrayView(naivePoints),FVector3d(1,0,0),FVector3d(1,0,0),radius,naiveBudget,1./64.,0.,1);
        TestTrue(TEXT("rounded vertex would describe a different, inside geometry"),wrongModel.Status==ETensorSphereTimeStatus::InitialInsideTube);
        TestFalse(TEXT("invalid polygon index refused"),BuildPlaneTriangleIntervalNet(plane,FIntVector(0,1,9),triangle));
        TestTrue(TEXT("refusal leaves no partial net or identity"),triangle.Surface.ControlPoints.IsEmpty()&&triangle.PrimitiveId==0);
    }
    TestTrue(TEXT("original floating context restored"),context.Restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticTensorCollapsedTriangleTest,
 "IAmSpeed.AnalyticWorld.TensorIntervalInput.CollapsedTriangle",
 EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FAnalyticTensorCollapsedTriangleTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture established"),context.Ready))return false;
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    FBoundedPlane plane;plane.AxisU=FVector3d(1,0,0);plane.AxisV=FVector3d(0,1,0);
    plane.DomainVertices={FVector2d(0,0),FVector2d(1,0),FVector2d(0,1)};
    FPlaneTriangleIntervalNet triangle;if(!TestTrue(TEXT("generic triangle constructed"),BuildPlaneTriangleIntervalNet(plane,FIntVector(0,1,2),triangle)))return false;
    FDistanceBudget rootWitnessBudget{16};
    const auto rootWitness=BoundTensorDistanceSquaredIntervalCenter(triangle.Surface,Detail::Point(FVector3d(1.125,0,0)),rootWitnessBudget,0.,1);
    TestTrue(TEXT("collapsed vertex supplies a true inside witness without subdividing equivalent v values"),rootWitness.Status!=EDistanceBoundStatus::Unknown&&rootWitness.UpperSquared<0.0625&&rootWitness.SampleUV.X==1.&&rootWitnessBudget.Used==1);
    TArray<FIntervalTensorSurface> surfaces={triangle.Surface};FDistanceBudget budget{131072};
    const auto r=BoundTensorSphereFirstContact(MakeArrayView(surfaces),FVector3d(2,0,0),FVector3d(0,0,0),0.25,budget,1./65536.,1.e-7,256,0.01,0.05);
    TestTrue(TEXT("collapsed original edge retains the true vertex contact"),r.Status==ETensorSphereContactStatus::FirstContactEnclosed);
    if(r.Status==ETensorSphereContactStatus::FirstContactEnclosed)
    {
        TestTrue(TEXT("first contact time3/8 enclosed"),r.LowerTime<=0.375&&r.UpperTime>=0.375);
        const double point[3]={1,0,0},normal[3]={1,0,0};
        for(int32 d=0;d<3;++d)
        {
            TestTrue(TEXT("vertex b is enclosed"),r.SurfacePoint.Axis[d].Lower<=point[d]&&r.SurfacePoint.Axis[d].Upper>=point[d]);
            TestTrue(TEXT("sphere radial normal+X enclosed"),r.RadialUnitNormal.Axis[d].Lower<=normal[d]&&r.RadialUnitNormal.Axis[d].Upper>=normal[d]);
        }
    }
    FDistanceBudget missBudget{4096};const auto miss=BoundTensorSphereSweep(MakeArrayView(surfaces),FVector3d(0.9,0.9,1),FVector3d(0.9,0.9,-1),0.1,missBudget,1./64.,1.e-7,64);
    TestTrue(TEXT("triangle excludes the part of its bounding rectangle outside x+y<=1"),miss.Status==ETensorSphereTimeStatus::NoIntersectionWithProvidedTensors);
    TestTrue(TEXT("original floating context restored"),context.Restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticTensorIntervalFamilyTest,
 "IAmSpeed.AnalyticWorld.TensorIntervalInput.UniversalFamilyAndBudget",
 EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FAnalyticTensorIntervalFamilyTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture established"),context.Ready))return false;
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    FIntervalTensorSurface family;auto cp=Detail::Point(FVector3d::ZeroVector);cp.Axis[2]=Interval{0,1,true};family.ControlPoints.Add(cp);
    TArray<FIntervalTensorSurface> surfaces={family};FDistanceBudget budget{8192};
    const auto mixed=BoundTensorSphereSweep(MakeArrayView(surfaces),FVector3d(0,0,2),FVector3d(0,0,0.5),0.25,budget,1./64.,1.e-7,1);
    TestTrue(TEXT("some fixed family members hit and others miss: no existential choice"),mixed.Status==ETensorSphereTimeStatus::Unknown);
    auto farBowl=ContactUnitBowl();for(auto& p:farBowl.ControlPoints)p.Z+=100.;FIntervalTensorSurface far;
    TestTrue(TEXT("binary64 control net promotes into the same interval core"),Detail::PromoteTensor(farBowl,far));
    const auto threshold=Interval::Exact(1.);FDistanceBudget rootBudget{257};
    const auto root=BoundTensorDistanceSquaredIntervalCenter(far,Detail::Point(FVector3d::ZeroVector),rootBudget,0.,257,&threshold);
    TestTrue(TEXT("strict root separation stops without claiming distance convergence"),root.Status==EDistanceBoundStatus::Bounded&&root.LowerSquared>1.);
    TestEqual(TEXT("threshold decision charges only its actual root"),rootBudget.Used,uint64(1));TestEqual(TEXT("unused shared budget preserved without refund"),rootBudget.Remaining,256);
    TArray<FTensorBezierSurface> binaryNets={ContactUnitBowl(),TimeUnitPlane()};FDistanceBudget insufficient{1};
    const auto noPromotion=BoundTensorSphereSweep(MakeArrayView(binaryNets),FVector3d::ZeroVector,FVector3d::ZeroVector,0.25,insufficient,1./64.,1.e-7,1);
    TestTrue(TEXT("budget1 rejects a multi-provider promotion before any global copying/work"),noPromotion.Status==ETensorSphereTimeStatus::Unknown&&insufficient.Used==0&&insufficient.Remaining==1);
    const auto noContactPromotion=BoundTensorSphereFirstContact(MakeArrayView(binaryNets),FVector3d::ZeroVector,FVector3d::ZeroVector,0.25,insufficient,1./64.,1.e-7,1,0.01,0.05);
    TestTrue(TEXT("contact wrapper shares the same pre-copy budget gate"),noContactPromotion.Status==ETensorSphereContactStatus::Unknown&&insufficient.Used==0&&insufficient.Remaining==1);
    FDistanceBudget empty{0};const auto stop=BoundTensorSphereFirstContact(MakeArrayView(surfaces),FVector3d(0,0,2),FVector3d(0,0,0.5),0.25,empty,1./64.,1.e-7,1,0.01,0.05);
    TestTrue(TEXT("empty shared budget cannot qualify a partial family"),stop.Status==ETensorSphereContactStatus::Unknown&&empty.Used==0);
    family.ControlPoints[0].Axis[0]=Interval{AnalyticCertified::FromBits(1),1.,true};FDistanceBudget malformedBudget{4};
    const auto malformed=BoundTensorDistanceSquaredIntervalCenter(family,Detail::Point(FVector3d::ZeroVector),malformedBudget,0.);
    TestTrue(TEXT("forged subnormal coefficient endpoint refused"),malformed.Status==EDistanceBoundStatus::Unknown);
    TestTrue(TEXT("original floating context restored"),context.Restore.Restore());return !HasAnyErrors();
}
namespace
{
FAutoConsoleCommand TensorIntervalInputUnits(TEXT("Speed.Analytic.RunTensorIntervalInputUnits"),
    TEXT("Run three interval-input and exact finite-triangle model tests, no World admission."),
    FConsoleCommandDelegate::CreateLambda([]()
    {
        static bool Used=false;auto& Framework=FAutomationTestFramework::Get();
        if(!IsRunningCommandlet()||!FParse::Param(FCommandLine::Get(),TEXT("AnalyticTensorIntervalInputUnits"))||Used||GIsAutomationTesting||GIsSlowTask||GIsPlayInEditorWorld||Framework.GetCurrentTest()||!Framework.IsLatentCommandQueueEmpty())
        {UE_LOG(LogTemp,Error,TEXT("Analytic tensor interval input commandlet refused"));return;}
        Used=true;
        const TCHAR* Classes[]={TEXT("FAnalyticTensorAffineIntervalTest"),TEXT("FAnalyticTensorCollapsedTriangleTest"),TEXT("FAnalyticTensorIntervalFamilyTest")};
        const TCHAR* Names[]={TEXT("IAmSpeed.AnalyticWorld.TensorIntervalInput.AffineRounding"),TEXT("IAmSpeed.AnalyticWorld.TensorIntervalInput.CollapsedTriangle"),TEXT("IAmSpeed.AnalyticWorld.TensorIntervalInput.UniversalFamilyAndBudget")};
        for(int32 i=0;i<3;++i)
        {
            if(GIsAutomationTesting||GIsSlowTask||GIsPlayInEditorWorld||Framework.GetCurrentTest()||!Framework.IsLatentCommandQueueEmpty()||!Framework.ContainsTest(Classes[i]))
            {UE_LOG(LogTemp,Error,TEXT("Analytic tensor interval input ownership refused"));return;}
            Framework.SetRequestedTestFilter(EAutomationTestFlags::EngineFilter);Framework.StartTestByName(Classes[i],0,Names[i]);
            if(!GIsAutomationTesting||!Framework.GetCurrentTest()||Framework.GetCurrentTestFullPath()!=Names[i]||!Framework.IsLatentCommandQueueEmpty())
            {UE_LOG(LogTemp,Error,TEXT("Analytic tensor interval input synchronous ownership refused"));return;}
            FAutomationTestExecutionInfo info;const bool stopped=Framework.StopTest(info);
            if(GIsAutomationTesting||GIsSlowTask||GIsPlayInEditorWorld||Framework.GetCurrentTest()||!Framework.IsLatentCommandQueueEmpty())
            {UE_LOG(LogTemp,Error,TEXT("Analytic tensor interval input final ownership refused"));return;}
            const bool passed=stopped&&info.bSuccessful&&info.GetErrorTotal()==0&&info.GetWarningTotal()==0;
            UE_LOG(LogTemp,Display,TEXT("ANALYTIC_TENSOR_INTERVAL_INPUT_UNIT name=%s passed=%d errors=%d warnings=%d"),Names[i],passed,info.GetErrorTotal(),info.GetWarningTotal());
        }
    }));
}


// Frozen pre-heap selector oracle. Bounds, split, witness order, stops and
// budget calls remain the old implementation, independent of the new heap.
namespace Speed::Analytic::Certified::LeafMinimumOracle
{
inline FTensorDistanceBounds LinearScanOriginal(const FIntervalTensorSurface& surface,
    const FIntervalPoint& centerBounds, FDistanceBudget& budget, double requestedSquaredGap,
    int32 maximumNodesThisCall=MAX_int32,const Interval* decisionThresholdSquared=nullptr)
{
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    FTensorDistanceBounds result;
    const uint64 usedBefore=budget.Used;
    if (maximumNodesThisCall<=0 || surface.DegreeU<0 || surface.DegreeV<0 || surface.DegreeU>16 || surface.DegreeV>16 ||
        surface.ControlPoints.Num()!=(surface.DegreeU+1)*(surface.DegreeV+1) ||
        !AnalyticCertified::Finite(requestedSquaredGap) ||
        ((AnalyticCertified::Bits(requestedSquaredGap)&AnalyticCertified::Sign)!=0 &&
         (AnalyticCertified::Bits(requestedSquaredGap)&AnalyticCertified::Magnitude)!=0) ||
        (decisionThresholdSquared&&!AnalyticCertified::Valid(*decisionThresholdSquared)) || !budget.Consume()) return result;
    if (!Detail::ValidPoint(centerBounds))return result;
    Detail::FNode first; first.DegreeU=surface.DegreeU;first.DegreeV=surface.DegreeV;
    for (const auto& p:surface.ControlPoints)first.Net.Add(p);
    if (!Detail::Bounds(first,centerBounds))return result;
    TArray<Detail::FNode> leaves;leaves.Add(MoveTemp(first));
    result.UpperSquared=leaves[0].UpperSquared;
    result.SampleUV=leaves[0].WitnessUV;result.SamplePoint=leaves[0].WitnessPoint;
    for (;;)
    {
        int32 best=0;
        for (int32 i=1;i<leaves.Num();++i)
            if (leaves[i].LowerSquared<leaves[best].LowerSquared)best=i;
        result.LowerSquared=leaves[best].LowerSquared;result.Status=EDistanceBoundStatus::Bounded;
        const auto gap=AnalyticCertified::Subtract(Interval::Exact(result.UpperSquared),Interval::Exact(result.LowerSquared));
        if (!AnalyticCertified::Valid(gap))return FTensorDistanceBounds();
        const bool thresholdDecided=decisionThresholdSquared&&(result.LowerSquared>decisionThresholdSquared->Upper||
            result.UpperSquared<decisionThresholdSquared->Lower);
        // Distance convergence alone cannot answer an unresolved radius threshold.
        // Keep the same nodes/shared budget and refine until a strict decision;
        // an exhausted/invalid attempt still leaves that decision unknown.
        if (gap.Upper<=requestedSquaredGap&&(!decisionThresholdSquared||thresholdDecided))
        {result.Status=EDistanceBoundStatus::Converged;break;}
        // Preserve Converged above when both facts hold; a threshold-only
        // early stop is Bounded, with the same universal lower/upper bounds.
        if(thresholdDecided)break;
        if (budget.Remaining<2 || budget.Used-usedBefore+2>uint64(maximumNodesThisCall))break;
        const auto& node=leaves[best];
        if (node.DegreeU==0 && node.DegreeV==0)break;
        const bool alongU=node.DegreeU>0 && (node.DegreeV==0 || node.DepthU<=node.DepthV);
        Detail::FNode a,b;
        if (!Detail::Split(node,alongU,a,b))break;
        if (!budget.Consume() || !budget.Consume() || !Detail::Bounds(a,centerBounds) || !Detail::Bounds(b,centerBounds))return FTensorDistanceBounds();
        for (const auto* child:{&a,&b})
            if (child->UpperSquared<result.UpperSquared)
            {result.UpperSquared=child->UpperSquared;result.SampleUV=child->WitnessUV;result.SamplePoint=child->WitnessPoint;}
        leaves[best]=MoveTemp(a);leaves.Add(MoveTemp(b));
    }
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    return result;
}

// Isolate heap equivalence from the separately tested schedule change.
inline FTensorDistanceBounds LinearScanScheduled(const FIntervalTensorSurface& surface,
    const FIntervalPoint& centerBounds, FDistanceBudget& budget, double requestedSquaredGap,
    int32 maximumNodesThisCall=MAX_int32,const Interval* decisionThresholdSquared=nullptr)
{
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    FTensorDistanceBounds result;
    const uint64 usedBefore=budget.Used;
    if (maximumNodesThisCall<=0 || surface.DegreeU<0 || surface.DegreeV<0 || surface.DegreeU>16 || surface.DegreeV>16 ||
        surface.ControlPoints.Num()!=(surface.DegreeU+1)*(surface.DegreeV+1) ||
        !AnalyticCertified::Finite(requestedSquaredGap) ||
        ((AnalyticCertified::Bits(requestedSquaredGap)&AnalyticCertified::Sign)!=0 &&
         (AnalyticCertified::Bits(requestedSquaredGap)&AnalyticCertified::Magnitude)!=0) ||
        (decisionThresholdSquared&&!AnalyticCertified::Valid(*decisionThresholdSquared)) || !budget.Consume()) return result;
    if (!Detail::ValidPoint(centerBounds))return result;
    Detail::FNode first; first.DegreeU=surface.DegreeU;first.DegreeV=surface.DegreeV;
    for (const auto& p:surface.ControlPoints)first.Net.Add(p);
    if (!Detail::Bounds(first,centerBounds))return result;
    TArray<Detail::FNode> leaves;leaves.Add(MoveTemp(first));
    result.UpperSquared=leaves[0].UpperSquared;
    result.SampleUV=leaves[0].WitnessUV;result.SamplePoint=leaves[0].WitnessPoint;
    for (;;)
    {
        int32 best=0;
        for (int32 i=1;i<leaves.Num();++i)
            if (leaves[i].LowerSquared<leaves[best].LowerSquared)best=i;
        result.LowerSquared=leaves[best].LowerSquared;result.Status=EDistanceBoundStatus::Bounded;
        const auto gap=AnalyticCertified::Subtract(Interval::Exact(result.UpperSquared),Interval::Exact(result.LowerSquared));
        if (!AnalyticCertified::Valid(gap))return FTensorDistanceBounds();
        const bool thresholdDecided=decisionThresholdSquared&&(result.LowerSquared>decisionThresholdSquared->Upper||
            result.UpperSquared<decisionThresholdSquared->Lower);
        // Distance convergence alone cannot answer an unresolved radius threshold.
        // Keep the same nodes/shared budget and refine until a strict decision;
        // an exhausted/invalid attempt still leaves that decision unknown.
        if (gap.Upper<=requestedSquaredGap&&(!decisionThresholdSquared||thresholdDecided))
        {result.Status=EDistanceBoundStatus::Converged;break;}
        // Preserve Converged above when both facts hold; a threshold-only
        // early stop is Bounded, with the same universal lower/upper bounds.
        if(thresholdDecided)break;
        if (budget.Remaining<2 || budget.Used-usedBefore+2>uint64(maximumNodesThisCall))break;
        const auto& node=leaves[best];
        if (node.DegreeU==0 && node.DegreeV==0)break;
        const bool alongU=Detail::ChooseTensorDistanceSplitU(node);
        Detail::FNode a,b;
        if (!Detail::Split(node,alongU,a,b))break;
        if (!budget.Consume() || !budget.Consume() || !Detail::Bounds(a,centerBounds) || !Detail::Bounds(b,centerBounds))return FTensorDistanceBounds();
        for (const auto* child:{&a,&b})
            if (child->UpperSquared<result.UpperSquared)
            {result.UpperSquared=child->UpperSquared;result.SampleUV=child->WitnessUV;result.SamplePoint=child->WitnessPoint;}
        leaves[best]=MoveTemp(a);leaves.Add(MoveTemp(b));
    }
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    return result;
}

}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticStableDistanceLeafHeapTest,
 "IAmSpeed.AnalyticWorld.TensorDistance.StableLeafMinimum",
 EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FAnalyticStableDistanceLeafHeapTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture"),context.Ready))return false;
    using namespace Speed::Analytic::Certified;
    const uint32 controls=VectorGetControlRegister();
    const double values[]={0.,-0.,AnalyticCertified::FromBits(0x0010000000000000ULL),1.,
        AnalyticCertified::FromBits(0x3ff0000000000001ULL),16.,AnalyticCertified::FromBits(0x7fefffffffffffffULL)};
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        for(int32 pattern=0;pattern<3;++pattern)
        {
            TArray<Detail::FNode> leaves;Detail::FMinLowerLeafHeap heap;
            leaves.AddDefaulted();leaves[0].LowerSquared=values[pattern];heap.PushIndex(0,leaves);
            const auto oracleIndex=[&]() {int32 best=0;for(int32 i=1;i<leaves.Num();++i)if(leaves[i].LowerSquared<leaves[best].LowerSquared)best=i;return best;};
            for(int32 step=0;step<513;++step)
            {
                const int32 best=oracleIndex();
                if(!TestTrue(TEXT("every old-index tie and reallocation matches linear minimum"),heap.Num()==leaves.Num()&&heap.PeekIndex()==best))return false;
                heap.PopRoot(leaves);
                leaves[best].LowerSquared=values[pattern==0?0:(step*5+pattern)%UE_ARRAY_COUNT(values)];
                const int32 next=leaves.AddDefaulted();leaves[next].LowerSquared=values[pattern==0?1:(step*3+pattern+1)%UE_ARRAY_COUNT(values)];
                heap.PushIndex(best,leaves);heap.PushIndex(next,leaves);
            }
            // Drain against an independent live-index set; detects duplicate,
            // missing and stale entries after non-power-of-two growth.
            TArray<bool> alive;alive.Init(true,leaves.Num());
            for(int32 count=leaves.Num();count>0;--count)
            {
                int32 best=INDEX_NONE;for(int32 i=0;i<leaves.Num();++i)if(alive[i]&&(best==INDEX_NONE||leaves[i].LowerSquared<leaves[best].LowerSquared))best=i;
                if(!TestTrue(TEXT("exactly one entry per leaf, stable complete draining"),heap.Num()==count&&heap.PeekIndex()==best))return false;
                heap.PopRoot(leaves);alive[best]=false;
            }
            TestEqual(TEXT("fully drained heap"),heap.Num(),0);
        }
        // The dyadic split stop is still a stop on the chosen leaf.
        Detail::FNode n,a,b;n.DegreeU=1;n.DepthU=52;n.Net={Detail::Point(FVector3d::ZeroVector),Detail::Point(FVector3d(1,0,0))};
        TestFalse(TEXT("depth52 split remains unavailable"),Detail::Split(n,true,a,b));
    }
    TestTrue(TEXT("numeric controls restored"),context.Restore.Restore());return !HasAnyErrors();
}
namespace Speed::Analytic::Certified::LeafMinimumOracle
{
inline FTensorDistanceBounds LinearScanKKT(const FIntervalTensorSurface& surface,
    const FIntervalPoint& centerBounds, FDistanceBudget& budget, double requestedSquaredGap,
    int32 maximumNodesThisCall=MAX_int32,const Interval* decisionThresholdSquared=nullptr)
{
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    FTensorDistanceBounds result;
    const uint64 usedBefore=budget.Used;
    if (maximumNodesThisCall<=0 || surface.DegreeU<0 || surface.DegreeV<0 || surface.DegreeU>16 || surface.DegreeV>16 ||
        surface.ControlPoints.Num()!=(surface.DegreeU+1)*(surface.DegreeV+1) ||
        !AnalyticCertified::Finite(requestedSquaredGap) ||
        ((AnalyticCertified::Bits(requestedSquaredGap)&AnalyticCertified::Sign)!=0 &&
         (AnalyticCertified::Bits(requestedSquaredGap)&AnalyticCertified::Magnitude)!=0) ||
        (decisionThresholdSquared&&!AnalyticCertified::Valid(*decisionThresholdSquared)) || !budget.Consume()) return result;
    if (!Detail::ValidPoint(centerBounds))return result;
    Detail::FNode first; first.DegreeU=surface.DegreeU;first.DegreeV=surface.DegreeV;
    for (const auto& p:surface.ControlPoints)first.Net.Add(p);
    if (!Detail::Bounds(first,centerBounds,decisionThresholdSquared))return result;
    TArray<Detail::FNode> leaves;leaves.Add(MoveTemp(first));
    TArray<uint8> alive;alive.Add(1);
    result.UpperSquared=leaves[0].UpperSquared;
    result.SampleUV=leaves[0].WitnessUV;result.SamplePoint=leaves[0].WitnessPoint;
    for (;;)
    {
        int32 best=INDEX_NONE;
        for(int32 i=0;i<leaves.Num();++i)if(alive[i]&&(best==INDEX_NONE||leaves[i].LowerSquared<leaves[best].LowerSquared))best=i;
        if(best==INDEX_NONE)return FTensorDistanceBounds();
        result.LowerSquared=leaves[best].LowerSquared;result.Status=EDistanceBoundStatus::Bounded;
        const auto gap=AnalyticCertified::Subtract(Interval::Exact(result.UpperSquared),Interval::Exact(result.LowerSquared));
        if (!AnalyticCertified::Valid(gap))return FTensorDistanceBounds();
        const bool thresholdDecided=decisionThresholdSquared&&(result.LowerSquared>decisionThresholdSquared->Upper||
            result.UpperSquared<decisionThresholdSquared->Lower);
        // Distance convergence alone cannot answer an unresolved radius threshold.
        // Keep the same nodes/shared budget and refine until a strict decision;
        // an exhausted/invalid attempt still leaves that decision unknown.
        if (gap.Upper<=requestedSquaredGap&&(!decisionThresholdSquared||thresholdDecided))
        {result.Status=EDistanceBoundStatus::Converged;break;}
        // Preserve Converged above when both facts hold; a threshold-only
        // early stop is Bounded, with the same universal lower/upper bounds.
        if(thresholdDecided)break;
        // A KKT predicate is new work, paid from the SAME global and local
        // allowance. A local cap stop retains valid Bounded values; an actual
        // Consume denial/invalid predicate remains terminal Unknown.
        if(budget.Used-usedBefore+1>uint64(maximumNodesThisCall))break;
        if(!budget.Consume())return FTensorDistanceBounds();
        const auto minimumClass=MinimumDetail::ClassifyMinimum(leaves[best],centerBounds);
        if(minimumClass==MinimumDetail::ECellMinimum::Unknown)return FTensorDistanceBounds();
        if(minimumClass==MinimumDetail::ECellMinimum::Excluded)
        {
            alive[best]=0; // Independent linear live-index oracle.
            leaves[best].Net.Reset(); // Local dead slot; witness copies remain owned by result.
            if(!alive.Contains(uint8(1)))return FTensorDistanceBounds(); // Never infer clear by vacuity.
            continue;
        }
        // Recheck AFTER KKT consumes: the old two-consume guard is stale now.
        if (budget.Remaining<2 || budget.Used-usedBefore+2>uint64(maximumNodesThisCall))break;
        const auto& node=leaves[best];
        if (node.DegreeU==0 && node.DegreeV==0)break;
        const bool alongU=Detail::ChooseTensorDistanceSplitU(node);
        Detail::FNode a,b;
        if (!Detail::Split(node,alongU,a,b))break;
        if (!budget.Consume() || !budget.Consume() || !Detail::Bounds(a,centerBounds,decisionThresholdSquared) || !Detail::Bounds(b,centerBounds,decisionThresholdSquared))return FTensorDistanceBounds();
        // Linear oracle retains the slot until the same successful replacement.
        for (const auto* child:{&a,&b})
            if (child->UpperSquared<result.UpperSquared)
            {result.UpperSquared=child->UpperSquared;result.SampleUV=child->WitnessUV;result.SamplePoint=child->WitnessPoint;}
        leaves[best]=MoveTemp(a);leaves.Add(MoveTemp(b));
        alive[best]=1;alive.Add(1);
    }
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    return result;
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticDistanceLeafHeapParityTest,
 "IAmSpeed.AnalyticWorld.TensorDistance.LeafSelectionBitsAndBudget",
 EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FAnalyticDistanceLeafHeapParityTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture"),context.Ready))return false;
    using namespace Speed::Analytic::Certified;
    const uint32 controls=VectorGetControlRegister();
    const auto same=[](const FTensorDistanceBounds& a,const FTensorDistanceBounds& b)
    {
        if(a.Status!=b.Status||AnalyticCertified::Bits(a.LowerSquared)!=AnalyticCertified::Bits(b.LowerSquared)||AnalyticCertified::Bits(a.UpperSquared)!=AnalyticCertified::Bits(b.UpperSquared)||AnalyticCertified::Bits(a.SampleUV.X)!=AnalyticCertified::Bits(b.SampleUV.X)||AnalyticCertified::Bits(a.SampleUV.Y)!=AnalyticCertified::Bits(b.SampleUV.Y))return false;
        for(int32 d=0;d<3;++d)if(a.SamplePoint.Axis[d].Known!=b.SamplePoint.Axis[d].Known||AnalyticCertified::Bits(a.SamplePoint.Axis[d].Lower)!=AnalyticCertified::Bits(b.SamplePoint.Axis[d].Lower)||AnalyticCertified::Bits(a.SamplePoint.Axis[d].Upper)!=AnalyticCertified::Bits(b.SamplePoint.Axis[d].Upper))return false;
        return true;
    };
    TArray<FIntervalTensorSurface> fixtures;
    FIntervalTensorSurface constant;constant.ControlPoints={Detail::Point(FVector3d(3,4,0))};fixtures.Add(constant);
    FIntervalTensorSurface plane;plane.DegreeU=plane.DegreeV=1;
    for(const FVector3d p:{FVector3d(-2,-2,0),FVector3d(-2,2,0),FVector3d(2,-2,0),FVector3d(2,2,0)})plane.ControlPoints.Add(Detail::Point(p));fixtures.Add(plane);
    auto family=plane;for(auto& p:family.ControlPoints)p.Axis[2]=AnalyticCertified::Interval{-.125,.125,true};fixtures.Add(family);
    FIntervalTensorSurface curve;curve.DegreeU=5;
    for(int32 i=0;i<6;++i)curve.ControlPoints.Add(Detail::Point(FVector3d(i-2.,0,(i%2?1.:-1.)*2.)));fixtures.Add(curve);
    FIntervalTensorSurface saddle;saddle.DegreeU=saddle.DegreeV=3;
    for(int32 i=0;i<4;++i)for(int32 j=0;j<4;++j)saddle.ControlPoints.Add(Detail::Point(FVector3d(i-1.5,j-1.5,(i%2?-1.:1.)*(j%2?-1.:1.))));fixtures.Add(saddle);
    auto malformed=plane;malformed.ControlPoints.Pop();fixtures.Add(malformed);
    auto overflow=plane;for(auto& p:overflow.ControlPoints)p.Axis[0]=AnalyticCertified::Interval::Exact(AnalyticCertified::FromBits(0x7fefffffffffffffULL));fixtures.Add(overflow);
    auto badBound=plane;badBound.ControlPoints[0].Axis[0]=AnalyticCertified::Interval{2.,1.,true};fixtures.Add(badBound);
    auto tooHigh=plane;tooHigh.DegreeU=17;fixtures.Add(tooHigh);
    const int32 caps[]={0,1,2,3,8,31,128,513};
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        for(const auto& surface:fixtures)for(int32 cap:caps)for(int32 mode=0;mode<4;++mode)
        {
            auto center=Detail::Point(FVector3d(.375,-.125,1.));
            if(mode==1)center.Axis[0]=AnalyticCertified::Interval{.25,.5,true};
            if(mode==3)center.Axis[1]=AnalyticCertified::Interval{2.,1.,true};
            const AnalyticCertified::Interval threshold=mode==2?AnalyticCertified::Interval{0.,100.,true}:AnalyticCertified::Interval::Exact(.25);
            FDistanceBudget oldBudget{cap,7,mode==1?FMath::Min(2,cap):0},newBudget=oldBudget;
            const auto a=LeafMinimumOracle::LinearScanKKT(surface,center,oldBudget,mode==0?.000001:0.,cap,mode==0?nullptr:&threshold);
            const auto b=BoundTensorDistanceSquaredIntervalCenter(surface,center,newBudget,mode==0?.000001:0.,cap,mode==0?nullptr:&threshold);
            if(!TestTrue(TEXT("linear oracle under the same schedule and KKT agrees on every result bit and budget/floor stop"),same(a,b)&&oldBudget.Remaining==newBudget.Remaining&&oldBudget.Used==newBudget.Used&&oldBudget.ConsumptionFloor==newBudget.ConsumptionFloor))return false;
        }
    }
    TestTrue(TEXT("numeric controls restored"),context.Restore.Restore());return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticPhysicalDistanceSplitTest,
 "IAmSpeed.AnalyticWorld.TensorDistance.PhysicalDirectionSchedule",
 EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FAnalyticPhysicalDistanceSplitTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture"),context.Ready))return false;
    using namespace Speed::Analytic::Certified;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        FIntervalTensorSurface net;net.DegreeU=2;net.DegreeV=5;
        const double uncertainty=AnalyticCertified::FromBits(0x3d70000000000000ULL); //2^-40
        for(int32 i=0;i<3;++i)for(int32 j=0;j<6;++j)
        {
            auto p=Detail::Point(FVector3d(i-1.,0.,i==1?-1.:1.));
            p.Axis[2]=AnalyticCertified::Add(p.Axis[2],AnalyticCertified::Interval{-uncertainty,uncertainty,true});
            net.ControlPoints.Add(p);
        }
        const auto center=Detail::Point(FVector3d(.25,0,0));
        const auto threshold=AnalyticCertified::Square(AnalyticCertified::Interval::Exact(.1));
        FDistanceBudget oldBudget{5},newBudget{5};
        const auto old=LeafMinimumOracle::LinearScanOriginal(net,center,oldBudget,0.,5,&threshold);
        const auto now=LeafMinimumOracle::LinearScanScheduled(net,center,newBudget,0.,5,&threshold);
        TestTrue(TEXT("balanced schedule remains ambiguous at this bounded work"),old.Status!=EDistanceBoundStatus::Unknown&&old.LowerSquared<=threshold.Upper&&old.UpperSquared>=threshold.Lower&&oldBudget.Used==5);
        TestTrue(TEXT("physical schedule proves a universal fixed-UV inside witness in the same work"),now.Status!=EDistanceBoundStatus::Unknown&&now.UpperSquared<threshold.Lower&&newBudget.Used==5&&now.SampleUV.X==.625);
        // The coefficient boxes above are deliberately independent, not an
        // exact-extrusion certificate; require their uncertainty at the witness.
        TestTrue(TEXT("uncertain geometry stays interval-valued"),now.SamplePoint.Axis[2].Lower<now.SamplePoint.Axis[2].Upper);
        Detail::FNode limits;limits.DegreeU=limits.DegreeV=1;limits.DepthU=52;limits.DepthV=0;
        TestFalse(TEXT("exhausted U selects available V"),Detail::ChooseTensorDistanceSplitU(limits));
        limits.DepthU=0;limits.DepthV=52;TestTrue(TEXT("exhausted V selects available U"),Detail::ChooseTensorDistanceSplitU(limits));
        limits.DepthU=limits.DepthV=52;Detail::FNode a,b;
        TestFalse(TEXT("both exhausted preserve an actual split refusal"),Detail::Split(limits,Detail::ChooseTensorDistanceSplitU(limits),a,b));
        Detail::FNode tied;tied.DegreeU=tied.DegreeV=1;
        for(const FVector3d p:{FVector3d(-1,-1,0),FVector3d(-1,1,0),FVector3d(1,-1,0),FVector3d(1,1,0)})tied.Net.Add(Detail::Point(p));
        TestTrue(TEXT("equal physical scores keep old balanced U"),Detail::ChooseTensorDistanceSplitU(tied));
        tied.DepthU=1;TestFalse(TEXT("equal physical scores keep old balanced V"),Detail::ChooseTensorDistanceSplitU(tied));
        tied.Net[0].Axis[0]=AnalyticCertified::Interval{2.,1.,true};
        TestFalse(TEXT("unavailable score keeps old schedule without certifying geometry"),Detail::ChooseTensorDistanceSplitU(tied));
    }
    TestTrue(TEXT("numeric controls restored"),context.Restore.Restore());return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticCapturedGutterProbe778Test,
 "IAmSpeed.AnalyticWorld.TensorDistance.CapturedGutterProbe778",
 EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FAnalyticCapturedGutterProbe778Test::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture"),context.Ready))return false;
    using namespace Speed::Analytic::Certified;
    // Mathematical regression reconstructed from retained packet090d4115 and
    // actual Fast14 query778. It is NOT a new native snapshot/contact ticket.
    // Record1931 LowerArc0/XPYP SHA6316e057, CPblock4992cc6b; preserve BOTH
    // endpoints of each independent coefficient box, never singletonize.
    static constexpr uint64 endpoints[36][6]={
        {0x0000000000000000ULL,0x0000000000000000ULL,0x40ae000000000000ULL,0x40ae000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL},
        {0x0000000000000000ULL,0x0000000000000000ULL,0x40ae28365eed39e1ULL,0x40ae28365eed39e2ULL,0x0000000000000000ULL,0x0000000000000000ULL},
        {0x0000000000000000ULL,0x0000000000000000ULL,0x40ae506cbdda73c3ULL,0x40ae506cbdda73c4ULL,0x0000000000000000ULL,0x0000000000000000ULL},
        {0x0000000000000000ULL,0x0000000000000000ULL,0x40ae781ececd30b4ULL,0x40ae781ececd30b5ULL,0x4017b0042a803897ULL,0x4017b0042a803898ULL},
        {0x0000000000000000ULL,0x0000000000000000ULL,0x40ae9ec8530920acULL,0x40ae9ec8530920adULL,0x402795c6d9c027bcULL,0x402795c6d9c027bdULL},
        {0x0000000000000000ULL,0x0000000000000000ULL,0x40aec3ef1535754bULL,0x40aec3ef1535754cULL,0x40337ca1866b95ceULL,0x40337ca1866b95cfULL},
        {0x40867fdd7d3ed05eULL,0x40867fdd7d3ed05fULL,0x40ae000000000000ULL,0x40ae000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL},
        {0x40867fdd7d3ed05eULL,0x40867fdd7d3ed05fULL,0x40ae28365eed39e1ULL,0x40ae28365eed39e2ULL,0x0000000000000000ULL,0x0000000000000000ULL},
        {0x40867fdd7d3ed05eULL,0x40867fdd7d3ed05fULL,0x40ae506cbdda73c3ULL,0x40ae506cbdda73c4ULL,0x0000000000000000ULL,0x0000000000000000ULL},
        {0x40867fdd7d3ed05eULL,0x40867fdd7d3ed05fULL,0x40ae781ececd30b4ULL,0x40ae781ececd30b5ULL,0x4017b0042a803897ULL,0x4017b0042a803898ULL},
        {0x40867fdd7d3ed05eULL,0x40867fdd7d3ed05fULL,0x40ae9ec8530920acULL,0x40ae9ec8530920adULL,0x402795c6d9c027bcULL,0x402795c6d9c027bdULL},
        {0x40867fdd7d3ed05eULL,0x40867fdd7d3ed05fULL,0x40aec3ef1535754bULL,0x40aec3ef1535754cULL,0x40337ca1866b95ceULL,0x40337ca1866b95cfULL},
        {0x40967fdd7d3ed05eULL,0x40967fdd7d3ed05fULL,0x40ae000000000000ULL,0x40ae000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL},
        {0x40967fdd7d3ed05eULL,0x40967fdd7d3ed05fULL,0x40ae28365eed39e1ULL,0x40ae28365eed39e2ULL,0x0000000000000000ULL,0x0000000000000000ULL},
        {0x40967fdd7d3ed05eULL,0x40967fdd7d3ed05fULL,0x40ae506cbdda73c3ULL,0x40ae506cbdda73c4ULL,0x0000000000000000ULL,0x0000000000000000ULL},
        {0x40967fdd7d3ed05eULL,0x40967fdd7d3ed05fULL,0x40ae781ececd30b4ULL,0x40ae781ececd30b5ULL,0x4017b0042a803897ULL,0x4017b0042a803898ULL},
        {0x40967fdd7d3ed05eULL,0x40967fdd7d3ed05fULL,0x40ae9ec8530920acULL,0x40ae9ec8530920adULL,0x402795c6d9c027bcULL,0x402795c6d9c027bdULL},
        {0x40967fdd7d3ed05eULL,0x40967fdd7d3ed05fULL,0x40aec3ef1535754bULL,0x40aec3ef1535754cULL,0x40337ca1866b95ceULL,0x40337ca1866b95cfULL},
        {0x40a0dfe61def1c46ULL,0x40a0dfe61def1c47ULL,0x40ae000000000000ULL,0x40ae000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL},
        {0x40a0dfe61def1c46ULL,0x40a0dfe61def1c47ULL,0x40ae28365eed39e1ULL,0x40ae28365eed39e2ULL,0x0000000000000000ULL,0x0000000000000000ULL},
        {0x40a0dfe61def1c46ULL,0x40a0dfe61def1c47ULL,0x40ae506cbdda73c3ULL,0x40ae506cbdda73c4ULL,0x0000000000000000ULL,0x0000000000000000ULL},
        {0x40a0dfe61def1c46ULL,0x40a0dfe61def1c47ULL,0x40ae781ececd30b4ULL,0x40ae781ececd30b5ULL,0x4017b0042a803897ULL,0x4017b0042a803898ULL},
        {0x40a0dfe61def1c46ULL,0x40a0dfe61def1c47ULL,0x40ae9ec8530920acULL,0x40ae9ec8530920adULL,0x402795c6d9c027bcULL,0x402795c6d9c027bdULL},
        {0x40a0dfe61def1c46ULL,0x40a0dfe61def1c47ULL,0x40aec3ef1535754bULL,0x40aec3ef1535754cULL,0x40337ca1866b95ceULL,0x40337ca1866b95cfULL},
        {0x40a67fdd7d3ed05eULL,0x40a67fdd7d3ed05fULL,0x40ae000000000000ULL,0x40ae000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL},
        {0x40a67fdd7d3ed05eULL,0x40a67fdd7d3ed05fULL,0x40ae28365eed39e1ULL,0x40ae28365eed39e2ULL,0x0000000000000000ULL,0x0000000000000000ULL},
        {0x40a67fdd7d3ed05eULL,0x40a67fdd7d3ed05fULL,0x40ae506cbdda73c3ULL,0x40ae506cbdda73c4ULL,0x0000000000000000ULL,0x0000000000000000ULL},
        {0x40a67fdd7d3ed05eULL,0x40a67fdd7d3ed05fULL,0x40ae781ececd30b4ULL,0x40ae781ececd30b5ULL,0x4017b0042a803897ULL,0x4017b0042a803898ULL},
        {0x40a67fdd7d3ed05eULL,0x40a67fdd7d3ed05fULL,0x40ae9ec8530920acULL,0x40ae9ec8530920adULL,0x402795c6d9c027bcULL,0x402795c6d9c027bdULL},
        {0x40a67fdd7d3ed05eULL,0x40a67fdd7d3ed05fULL,0x40aec3ef1535754bULL,0x40aec3ef1535754cULL,0x40337ca1866b95ceULL,0x40337ca1866b95cfULL},
        {0x40ac1fd4dc8e8475ULL,0x40ac1fd4dc8e8476ULL,0x40ae000000000000ULL,0x40ae000000000000ULL,0x0000000000000000ULL,0x0000000000000000ULL},
        {0x40ac1fd4dc8e8475ULL,0x40ac1fd4dc8e8476ULL,0x40ae28365eed39e1ULL,0x40ae28365eed39e2ULL,0x0000000000000000ULL,0x0000000000000000ULL},
        {0x40ac1fd4dc8e8475ULL,0x40ac1fd4dc8e8476ULL,0x40ae506cbdda73c3ULL,0x40ae506cbdda73c4ULL,0x0000000000000000ULL,0x0000000000000000ULL},
        {0x40ac1fd4dc8e8475ULL,0x40ac1fd4dc8e8476ULL,0x40ae781ececd30b4ULL,0x40ae781ececd30b5ULL,0x4017b0042a803897ULL,0x4017b0042a803898ULL},
        {0x40ac1fd4dc8e8475ULL,0x40ac1fd4dc8e8476ULL,0x40ae9ec8530920acULL,0x40ae9ec8530920adULL,0x402795c6d9c027bcULL,0x402795c6d9c027bdULL},
        {0x40ac1fd4dc8e8475ULL,0x40ac1fd4dc8e8476ULL,0x40aec3ef1535754bULL,0x40aec3ef1535754cULL,0x40337ca1866b95ceULL,0x40337ca1866b95cfULL},
    };
    FIntervalTensorSurface net;net.DegreeU=net.DegreeV=5;
    for(const auto& row:endpoints)
    {
        FIntervalPoint p;
        for(int32 d=0;d<3;++d)p.Axis[d]={AnalyticCertified::FromBits(row[2*d]),AnalyticCertified::FromBits(row[2*d+1]),true};
        net.ControlPoints.Add(p);
    }
    const FVector3d start(27.778356667970566,3879.3069625658977,26.60767370092227);
    const FVector3d end(27.778356667970566,3883.5051787030302,2.9777143004932451);
    const double radius=12.510000228881836,timeGap=2.8074707245500296e-6;
    const double pointWidth=.001250028610229492,normalWidth=.00012500000593718141;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        // Pure kernel reservation: native policy/candidate-selection costs are
        // outside this isolated regression and are not relabelled as executed.
        FDistanceBudget budget{1048576};
        const auto result=BoundTensorSphereFirstContact(MakeArrayView(&net,1),start,end,radius,budget,timeGap,0.,8192,pointWidth,normalWidth,24);
        AddInfo(FString::Printf(TEXT("captured_gutter778 flush=%u status=%u failure=%u time_failure=%u used=%llu remaining=%d leaves=%d widths=%d"),flush,uint32(result.Status),uint32(result.Failure),uint32(result.TimeFailure),budget.Used,budget.Remaining,result.Leaves.Num(),result.TemporalWidthRefinements));
        if(!TestTrue(TEXT("captured gutter polynomial closes first contact at the unchanged requested quality"),result.Status==ETensorSphereContactStatus::FirstContactEnclosed&&!result.Leaves.IsEmpty()&&result.CollisionRadiusBits==AnalyticCertified::Bits(radius)))return false;
        TestTrue(TEXT("one finite kernel budget, never refilled"),budget.Used+uint64(budget.Remaining)==1048576&&result.LowerTime>=0&&result.UpperTime<=1&&result.LowerTime<=result.UpperTime);
    }
    TestTrue(TEXT("numeric controls restored"),context.Restore.Restore());return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticGlobalMinimumKKTDomainTest,
 "IAmSpeed.AnalyticWorld.TensorDistance.GlobalMinimumKKTDomain",
 EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FAnalyticGlobalMinimumKKTDomainTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture"),context.Ready))return false;
    using namespace Speed::Analytic::Certified;using E=MinimumDetail::ECellMinimum;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        Detail::FNode line;line.DegreeU=1;line.Net={Detail::Point(FVector3d(0,0,0)),Detail::Point(FVector3d(1,0,0))};
        Detail::FNode a,b;TestTrue(TEXT("exact original-domain restriction"),Detail::Split(line,true,a,b));
        const auto left=Detail::Point(FVector3d(-1,0,0)),right=Detail::Point(FVector3d(2,0,0)),middle=Detail::Point(FVector3d(.5,0,0));
        TestTrue(TEXT("original lower boundary minimum retained"),MinimumDetail::ClassifyMinimum(a,left)==E::Possible);
        TestTrue(TEXT("strict positive derivative excludes internal cell"),MinimumDetail::ClassifyMinimum(b,left)==E::Excluded);
        TestTrue(TEXT("original upper boundary minimum retained"),MinimumDetail::ClassifyMinimum(b,right)==E::Possible);
        TestTrue(TEXT("strict negative derivative excludes internal cell"),MinimumDetail::ClassifyMinimum(a,right)==E::Excluded);
        TestTrue(TEXT("minimum on internal split edge survives both neighbors"),MinimumDetail::ClassifyMinimum(a,middle)==E::Possible&&MinimumDetail::ClassifyMinimum(b,middle)==E::Possible);
        auto uncertain=middle;uncertain.Axis[0]=AnalyticCertified::Interval{-1.,2.,true};
        TestTrue(TEXT("center family cannot be replaced by a convenient center"),MinimumDetail::ClassifyMinimum(a,uncertain)==E::Possible&&MinimumDetail::ClassifyMinimum(b,uncertain)==E::Possible);
        Detail::FNode constant;constant.Net={Detail::Point(FVector3d(1,2,3))};
        TestTrue(TEXT("zero derivatives and singular constant net remain possible"),MinimumDetail::ClassifyMinimum(constant,middle)==E::Possible);
        auto invalid=a;invalid.Net[0].Axis[0]=AnalyticCertified::Interval{2.,1.,true};
        TestTrue(TEXT("invalid gradient input is Unknown, never exclusion"),MinimumDetail::ClassifyMinimum(invalid,middle)==E::Unknown);
        auto badCenter=middle;badCenter.Axis[0]=AnalyticCertified::Interval{2.,1.,true};TestTrue(TEXT("invalid center is Unknown"),MinimumDetail::ClassifyMinimum(a,badCenter)==E::Unknown);
        Detail::FNode plane;plane.DegreeU=plane.DegreeV=1;
        for(const FVector3d p:{FVector3d(0,0,0),FVector3d(0,1,0),FVector3d(1,0,0),FVector3d(1,1,0)})plane.Net.Add(Detail::Point(p));
        Detail::FNode u0,u1,q0,q1;Detail::Split(plane,true,u0,u1);Detail::Split(u0,false,q0,q1);
        const auto corner=Detail::Point(FVector3d(-1,-1,1));
        TestTrue(TEXT("original corner remains represented"),MinimumDetail::ClassifyMinimum(q0,corner)==E::Possible);
        TestTrue(TEXT("artificial V edge is not an original lower edge"),MinimumDetail::ClassifyMinimum(q1,corner)==E::Excluded);
        TestTrue(TEXT("artificial U edge is not an original lower edge"),MinimumDetail::ClassifyMinimum(u1,corner)==E::Excluded);
    }
    TestTrue(TEXT("numeric controls restored"),context.Restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticGlobalMinimumKKTBoundsTest,
 "IAmSpeed.AnalyticWorld.TensorDistance.GlobalMinimumKKTBounds",
 EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FAnalyticGlobalMinimumKKTBoundsTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture"),context.Ready))return false;
    using namespace Speed::Analytic::Certified;
    FIntervalTensorSurface plane;plane.DegreeU=plane.DegreeV=1;
    for(const FVector3d p:{FVector3d(-1,-1,0),FVector3d(-1,1,0),FVector3d(1,-1,0),FVector3d(1,1,0)})plane.ControlPoints.Add(Detail::Point(p));
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        for(int32 family=0;family<2;++family)for(int32 cap:{0,1,2,3,4,5,16,128,1025})for(int32 floor:{0,2})
        {
            auto net=plane;if(family)for(auto& p:net.ControlPoints)p.Axis[2]=AnalyticCertified::Interval{-.125,.125,true};
            FDistanceBudget budget{cap,11,FMath::Min(floor,cap)};const auto center=Detail::Point(FVector3d(.375,-.125,2));
            const auto r=BoundTensorDistanceSquaredIntervalCenter(net,center,budget,1.e-10,cap);
            if(r.Status!=EDistanceBoundStatus::Unknown)
                TestTrue(TEXT("universal bounds contain exact closest distances of represented members"),r.LowerSquared<=(family?3.515625:4.)&&r.UpperSquared>=(family?4.515625:4.));
            TestTrue(TEXT("KKT and splits respect one local/global allowance and floor"),budget.Used-11+uint64(budget.Remaining)==uint64(cap)&&budget.Used-11<=uint64(cap)&&budget.Remaining>=budget.ConsumptionFloor);
            if(!family&&cap==1025&&floor==0)TestTrue(TEXT("dyadic interior minimum closes without losing its internal edges"),r.Status==EDistanceBoundStatus::Converged);
        }
        auto family=plane;for(auto& p:family.ControlPoints)p.Axis[2]=AnalyticCertified::Interval{-.125,.125,true};
        const auto center=Detail::Point(FVector3d(.375,-.125,2));const auto tangent=AnalyticCertified::Interval::Exact(4.);
        FDistanceBudget uncertain{1025};const auto r=BoundTensorDistanceSquaredIntervalCenter(family,center,uncertain,0.,1025,&tangent);
        TestTrue(TEXT("family with members inside/outside never becomes a universal radius verdict"),r.Status==EDistanceBoundStatus::Unknown||(!(r.LowerSquared>4.)&&!(r.UpperSquared<4.)));
    }
    TestTrue(TEXT("numeric controls restored"),context.Restore.Restore());return !HasAnyErrors();
}

// Frozen four-Outward-pairs oracle, independent of extrema selection.
namespace AnalyticCertified::MultiplyExtremaOracle
{
inline Interval OriginalMultiply(Interval a, Interval b)
{
    if (!Valid(a) || !Valid(b)) return {};
    const double av[2] = {a.Lower, a.Upper}, bv[2] = {b.Lower, b.Upper};
    Interval r; bool first = true;
    for (double x : av) for (double y : bv)
    {
        volatile double z = x * y; double lo, hi;
        if (!Outward(z, false, lo) || !Outward(z, true, hi)) return {};
        if (first || lo < r.Lower) r.Lower = lo;
        if (first || hi > r.Upper) r.Upper = hi;
        first = false;
    }
    r.Known = true; return r;
}

}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticIntervalMultiplyExtremaEquivalenceTest,
 "IAmSpeed.AnalyticWorld.TensorIntervalInput.MultiplyExtremaEquivalence",
 EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FAnalyticIntervalMultiplyExtremaEquivalenceTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture"),context.Ready))return false;
    using namespace AnalyticCertified;
    const uint32 original=VectorGetControlRegister();uint64 cases=0;
    const auto compare=[&](Interval a,Interval b,uint32 controls)
    {
        VectorSetControlRegister(controls & ~uint32(0x3f));
        const auto old=MultiplyExtremaOracle::OriginalMultiply(a,b);
        const uint32 oldFlags=VectorGetControlRegister() & 0x3f;
        VectorSetControlRegister(controls & ~uint32(0x3f));
        const auto current=Multiply(a,b);
        const uint32 currentFlags=VectorGetControlRegister() & 0x3f;
        ++cases;
        return old.Known==current.Known && Bits(old.Lower)==Bits(current.Lower) &&
            Bits(old.Upper)==Bits(current.Upper) && oldFlags==currentFlags &&
            (VectorGetControlRegister() & ~uint32(0x3f))==(controls & ~uint32(0x3f));
    };
    // Products of NORMAL operands exercise subnormal results, FTZ and DAZ.
    const uint64 edges[]={0,Sign,MinNormal,MinNormal+1,0x001fffffffffffffULL,
        0x0020000000000000ULL,0x2000000000000000ULL,0x3ca0000000000000ULL,
        0x3fe0000000000000ULL,0x3ff0000000000000ULL,0x3ff0000000000001ULL,
        0x4000000000000000ULL,0x6000000000000000ULL,0x7fd0000000000000ULL,
        0x7feffffffffffffeULL,0x7fefffffffffffffULL};
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        const uint32 controls=(original & ~uint32(0x8040)) | flush;
        bool same=true;
        for(uint64 x:edges)for(uint64 y:edges)for(uint64 sx:{uint64(0),Sign})for(uint64 sy:{uint64(0),Sign})
        {
            const double a=FromBits(x ^ sx),b=FromBits(y ^ sy);
            same &= compare(Interval::Exact(a),Interval::Exact(b),controls);
            // Wide mixed signs place overflow/failure at different products.
            same &= compare(Interval{-FMath::Abs(a),FMath::Abs(a),true},Interval{-FMath::Abs(b),FMath::Abs(b),true},controls);
        }
        uint64 seed=0x5693c0d17ea4882bULL;
        const auto next=[&]()
        {seed^=seed<<13;seed^=seed>>7;seed^=seed<<17;return seed;};
        const auto key=[](uint64 b){return (b & Sign)?~b:(b | Sign);};
        for(int32 i=0;i<4096;++i)
        {
            uint64 v[4];for(auto& bits:v){do{bits=next();}while((bits&Magnitude)>=Infinity ||
                ((bits&Magnitude)!=0 && (bits&Magnitude)<MinNormal));}
            if(key(v[0])>key(v[1]))Swap(v[0],v[1]);if(key(v[2])>key(v[3]))Swap(v[2],v[3]);
            same &= compare(Interval{FromBits(v[0]),FromBits(v[1]),true},Interval{FromBits(v[2]),FromBits(v[3]),true},controls);
        }
        for(const Interval invalid:{Interval{},Interval{2,1,true},Interval{FromBits(1),1,true},
            Interval{-1,FromBits(Sign|1),true},Interval{FromBits(Infinity),FromBits(Infinity),true},
            Interval{FromBits(Infinity|1),1,true}})
        {same &= compare(invalid,Interval::Exact(1),controls);same &= compare(Interval::Exact(1),invalid,controls);}
        // The former immediate Outward refusal at MaxFinite must precede
        // any later overflow. Compare sticky flags as well as null output.
        same &= compare(Interval{1,2,true},Interval{FromBits(0x7fefffffffffffffULL),FromBits(0x7fefffffffffffffULL),true},controls);
        same &= compare(Interval{-2,-1,true},Interval{FromBits(0x7fdfffffffffffffULL),FromBits(0x7fefffffffffffffULL),true},controls);
        TestTrue(TEXT("bit-identical bounds, refusal and sticky flags for this FTZ/DAZ mode"),same);
    }
    UE_LOG(LogTemp,Display,TEXT("[IntervalMultiplyExtremaEquivalence] Cases=%llu Modes=4 NoPhysicalAcceptance=1"),cases);
    TestTrue(TEXT("numeric controls restored"),context.Restore.Restore());return !HasAnyErrors();
}


IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticHalfSum72EquivalenceTest,
 "IAmSpeed.AnalyticWorld.TensorIntervalInput.HalfSumEquivalence",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticHalfSum72EquivalenceTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture"),context.Ready))return false;
    using namespace AnalyticCertified;const uint32 original=VectorGetControlRegister();uint64 cases=0;
    const auto same=[](Interval a,Interval b){return a.Known==b.Known&&Bits(a.Lower)==Bits(b.Lower)&&Bits(a.Upper)==Bits(b.Upper);};
    const auto compare=[&](Interval a,Interval b,uint32 controls)
    {
        VectorSetControlRegister(controls);
        const auto old=Add(MultiplyExtremaOracle::OriginalMultiply(a,Interval::Exact(.5)),
            MultiplyExtremaOracle::OriginalMultiply(b,Interval::Exact(.5)));
        const uint32 oldFlags=VectorGetControlRegister();
        VectorSetControlRegister(controls);const auto current=HalfSum(a,b);const uint32 currentFlags=VectorGetControlRegister();
        VectorSetControlRegister(controls);const auto oldHalf=MultiplyExtremaOracle::OriginalMultiply(a,Interval::Exact(.5));
        const uint32 oldHalfFlags=VectorGetControlRegister();
        VectorSetControlRegister(controls);const auto newHalf=ScaleHalf(a);const uint32 newHalfFlags=VectorGetControlRegister();
        ++cases;return same(old,current)&&oldFlags==currentFlags&&same(oldHalf,newHalf)&&oldHalfFlags==newHalfFlags;
    };
    const uint64 edges[]={0,1,2,3,MinNormal-1,MinNormal,MinNormal+1,MinNormal+2,MinNormal+3,
        0x001fffffffffffffULL,0x0020000000000000ULL,0x0020000000000001ULL,0x3ca0000000000000ULL,
        0x3fe0000000000000ULL,0x3ff0000000000000ULL,0x3ff0000000000001ULL,
        0x4000000000000000ULL,0x6000000000000000ULL,0x7feffffffffffffeULL,0x7fefffffffffffffULL};
    const auto key=[](uint64 b){return (b&Sign)?~b:(b|Sign);};
    for(uint32 flush:{0u,64u,32768u,32832u})for(uint32 sticky:{0u,1u,2u,4u,8u,16u,32u,63u})
    {
        const uint32 controls=(original&~uint32(0x807f))|flush|sticky;bool ok=true;
        for(uint64 x:edges)for(uint64 y:edges)for(uint64 sx:{uint64(0),Sign})for(uint64 sy:{uint64(0),Sign})
        {
            const double a=FromBits(x^sx),b=FromBits(y^sy);
            ok&=compare(Interval::Exact(a),Interval::Exact(b),controls);
            // Unlike Exact(subnormal), these forged subnormal endpoints must
            // still refuse before multiplication, irrespective of DAZ.
            ok&=compare(Interval{a,a,true},Interval{b,b,true},controls);
        }
        const Interval underflow{FromBits(MinNormal+1),FromBits(MinNormal+3),true};
        const Interval unusual[]={Interval{},Interval{2,1,true},Interval{0.,FromBits(Sign),true},
            Interval{FromBits(Sign),0.,true},Interval{FromBits(1),1,true},
            Interval{-1,FromBits(Sign|1),true},Interval{FromBits(Infinity),1,true},
            Interval{FromBits(Infinity|1),1,true},Interval{FromBits(Sign|Infinity|1),1,true}};
        for(const auto bad:unusual)
        {ok&=compare(bad,underflow,controls);ok&=compare(underflow,bad,controls);ok&=compare(bad,bad,controls);}
        uint64 seed=0x8fa6aef2de713bb4ULL;
        const auto next=[&](){seed^=seed<<13;seed^=seed>>7;seed^=seed<<17;return seed;};
        for(int32 i=0;i<4096;++i)
        {
            uint64 v[4];for(auto& x:v){do{x=next();}while((x&Magnitude)>=Infinity||((x&Magnitude)!=0&&(x&Magnitude)<MinNormal));}
            if(key(v[0])>key(v[1]))Swap(v[0],v[1]);if(key(v[2])>key(v[3]))Swap(v[2],v[3]);
            ok&=compare(Interval{FromBits(v[0]),FromBits(v[1]),true},Interval{FromBits(v[2]),FromBits(v[3]),true},controls);
        }
        TestTrue(TEXT("same half-products/half-sum bounds, failure, complete MXCSR and masked sticky union"),ok);
    }
    UE_LOG(LogTemp,Display,TEXT("[IntervalHalfSum72Equivalence] Cases=%llu Modes=4 StickyPresets=8 NoPhysicalAcceptance=1"),cases);
    TestTrue(TEXT("numeric controls restored"),context.Restore.Restore());return !HasAnyErrors();
}


IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTensorThresholdQuickBoundsTest,
    "IAmSpeed.AnalyticWorld.TensorDistance.StrictThresholdCornerAndBox",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FTensorThresholdQuickBoundsTest::RunTest(const FString&)
{
    using namespace Speed::Analytic::Certified;namespace A=::AnalyticCertified;
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE threshold fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        Detail::FNode node;node.DegreeU=node.DegreeV=1;
        node.Net={Detail::Point(FVector3d(0,0,0)),Detail::Point(FVector3d(0,1,0)),Detail::Point(FVector3d(1,0,0)),Detail::Point(FVector3d(1,1,0))};
        auto threshold=Interval::Exact(1.);Detail::FThresholdQuickWitness quick;
        auto center=Detail::Point(FVector3d(.5,.5,3));
        TestTrue(TEXT("whole restricted source separated by convex-hull box"),Detail::TryThresholdCornerAndBox(node,center,&threshold,quick)==Detail::EThresholdQuickResult::Decided&&quick.LowerSquared>1&&quick.UV==FVector2d(0,0));
        center=Detail::Point(FVector3d(0,0,.5));
        TestTrue(TEXT("actual original-corner witness universally inside threshold"),Detail::TryThresholdCornerAndBox(node,center,&threshold,quick)==Detail::EThresholdQuickResult::Decided&&quick.UpperSquared<1&&Detail::ValidPoint(quick.Point));
        Detail::FNode left,right;TestTrue(TEXT("actual tensor split"),Detail::Split(node,true,left,right));
        center=Detail::Point(FVector3d(1,0,.5));
        TestTrue(TEXT("restricted corner belongs to restricted global UV"),Detail::TryThresholdCornerAndBox(right,center,&threshold,quick)==Detail::EThresholdQuickResult::Decided&&quick.UV==FVector2d(right.MinU,right.MinV)&&quick.Point.Axis[0].Lower==right.Net[0].Axis[0].Lower);
        center=Detail::Point(FVector3d(.5,.5,1));
        TestTrue(TEXT("tangency or equality is not an optimistic decision"),Detail::TryThresholdCornerAndBox(node,center,&threshold,quick)==Detail::EThresholdQuickResult::Undecided);
        TestTrue(TEXT("no threshold preserves ordinary path"),Detail::TryThresholdCornerAndBox(node,center,nullptr,quick)==Detail::EThresholdQuickResult::Undecided);
        auto malformed=node;malformed.Net.Last().Axis[2].Known=false;
        TestTrue(TEXT("all coefficients validated before corner success"),Detail::TryThresholdCornerAndBox(malformed,Detail::Point(FVector3d(0,0,.5)),&threshold,quick)==Detail::EThresholdQuickResult::Unknown);
        // Two distant disconnected patches: one clear node never proves the
        // full provider union clear when another has a strict inside witness.
        FIntervalTensorSurface far;far.DegreeU=far.DegreeV=1;far.ControlPoints=node.Net;
        for(auto& cp:far.ControlPoints)cp.Axis[2]=Interval::Exact(10.);
        FIntervalTensorSurface near;near.DegreeU=near.DegreeV=1;near.ControlPoints=node.Net;
        TArray<FIntervalTensorSurface> surfaces{far,near};FDistanceBudget work{100000};
        const auto joined=TimeDetail::Evaluate(MakeArrayView(surfaces),Detail::Point(FVector3d(0,0,.5)),work,threshold,0.,8192,true);
        TestTrue(TEXT("complete provider aggregation retains later inside owner"),joined.Known&&joined.AnyInside&&!joined.AllClear&&joined.SurfaceIndex==1);
        FDistanceBudget empty{0};
        const auto refused=BoundTensorDistanceSquaredIntervalCenter(near,Detail::Point(FVector3d(0,0,.5)),empty,0.,8192,&threshold);
        TestTrue(TEXT("new early decision cannot bypass consumed budget"),refused.Status==EDistanceBoundStatus::Unknown&&empty.Used==0);
    }
    TestTrue(TEXT("numeric context restored"),restore.Restore());return !HasAnyErrors();
}


// Frozen pre-specialization Square uses the independent four-product oracle.
namespace AnalyticCertified::SquareTwoProductsOracle
{
inline Interval OriginalSquare(Interval a)
{
    if(!Valid(a))return {};
    auto left=MultiplyExtremaOracle::OriginalMultiply(Interval::Exact(a.Lower),Interval::Exact(a.Lower));
    auto right=MultiplyExtremaOracle::OriginalMultiply(Interval::Exact(a.Upper),Interval::Exact(a.Upper));
    if(!left.Known||!right.Known)return {};
    const bool crossesZero=a.Lower<=0.0&&a.Upper>=0.0;
    double lo=crossesZero?0.0:(left.Lower<right.Lower?left.Lower:right.Lower);
    if(lo<0.0)lo=0.0;
    return {lo,left.Upper>right.Upper?left.Upper:right.Upper,true};
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticSquareTwoProductsEquivalenceTest,
 "IAmSpeed.AnalyticWorld.TensorIntervalInput.SquareTwoProductsEquivalence",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticSquareTwoProductsEquivalenceTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture"),context.Ready))return false;
    using namespace AnalyticCertified;const uint32 original=VectorGetControlRegister();uint64 cases=0;
    const auto compare=[&](Interval a,uint32 controls)
    {
        VectorSetControlRegister(controls);const auto old=SquareTwoProductsOracle::OriginalSquare(a);
        const uint32 oldFlags=VectorGetControlRegister();
        VectorSetControlRegister(controls);const auto current=Square(a);
        const uint32 currentFlags=VectorGetControlRegister();++cases;
        return old.Known==current.Known&&Bits(old.Lower)==Bits(current.Lower)&&
            Bits(old.Upper)==Bits(current.Upper)&&oldFlags==currentFlags;
    };
    const uint64 edges[]={0,1,2,MinNormal-1,MinNormal,MinNormal+1,
        0x001fffffffffffffULL,0x0020000000000000ULL,0x1ff0000000000000ULL,
        0x2000000000000000ULL,0x2000000000000001ULL,0x3fe0000000000000ULL,
        0x3ff0000000000000ULL,0x3ff0000000000001ULL,0x4000000000000000ULL,
        0x5fefffffffffffffULL,0x5ff0000000000000ULL,0x5ff0000000000001ULL,
        0x6000000000000000ULL,0x7feffffffffffffeULL,0x7fefffffffffffffULL};
    const auto key=[](uint64 b){return(b&Sign)?~b:(b|Sign);};
    for(uint32 flush:{0u,64u,32768u,32832u})for(uint32 sticky:{0u,1u,2u,4u,8u,16u,32u,63u})
    {
        const uint32 controls=(original&~uint32(0x807f))|flush|sticky;bool ok=true;
        for(uint64 x:edges)for(uint64 y:edges)for(uint64 sx:{uint64(0),Sign})for(uint64 sy:{uint64(0),Sign})
        {
            uint64 lo=x^sx,hi=y^sy;if(key(lo)>key(hi))Swap(lo,hi);
            ok&=compare(Interval{FromBits(lo),FromBits(hi),true},controls);
            ok&=compare(Interval::Exact(FromBits(x^sx)),controls);
        }
        const Interval unusual[]={Interval{},Interval{2,1,true},Interval{0.,FromBits(Sign),true},
            Interval{FromBits(Sign),0.,true},Interval{FromBits(1),1,true},
            Interval{-1,FromBits(Sign|1),true},Interval{FromBits(Infinity),1,true},
            Interval{FromBits(Infinity|1),1,true},Interval{FromBits(Sign|Infinity|1),1,true},
            Interval{-1,1,false},
            // First endpoint overflows; the second still underflows.
            Interval{FromBits(Sign|0x6000000000000000ULL),FromBits(MinNormal+1),true},
            Interval{FromBits(Sign|MinNormal|1),FromBits(0x6000000000000000ULL),true}};
        for(const auto a:unusual)ok&=compare(a,controls);
        uint64 seed=0x693b2282e3d985d1ULL;
        const auto next=[&](){seed^=seed<<13;seed^=seed>>7;seed^=seed<<17;return seed;};
        for(int32 i=0;i<4096;++i)
        {
            uint64 v[2];for(auto& b:v){do{b=next();}while((b&Magnitude)>=Infinity||((b&Magnitude)!=0&&(b&Magnitude)<MinNormal));}
            if(key(v[0])>key(v[1]))Swap(v[0],v[1]);
            ok&=compare(Interval{FromBits(v[0]),FromBits(v[1]),true},controls);
        }
        TestTrue(TEXT("same square bounds, refusal and full MXCSR including both endpoint flags"),ok);
    }
    UE_LOG(LogTemp,Display,TEXT("[IntervalSquareTwoProductsEquivalence] Cases=%llu Modes=4 StickyPresets=8 NoPhysicalAcceptance=1"),cases);
    TestTrue(TEXT("numeric controls restored"),context.Restore.Restore());return !HasAnyErrors();
}

// Frozen immediate baseline: four products and integer extrema, no singleton dispatch.
namespace AnalyticCertified::MultiplySingletonOracle
{
inline Interval OriginalFourProducts(Interval a, Interval b)
{
    if (!Valid(a) || !Valid(b)) return {};
    const double av[2] = {a.Lower, a.Upper}, bv[2] = {b.Lower, b.Upper};
    std::uint64_t lowerBits=0,upperBits=0,lowerKey=0,upperKey=0;bool first=true;
    for (double x : av) for (double y : bv)
    {
        // Retain ALL four native products, their order and the original
        // immediate failure point (Outward also refuses +/-MaxFinite).
        // Outward is monotone: only the extrema require its bit expansion.
        volatile double z = x * y;const auto bits=Bits(z);
        if ((bits & Magnitude) >= 0x7fefffffffffffffULL) return {};
        // Native comparisons could erase a subnormal product under DAZ.
        // Integer ordering also keeps the two signed-zero representatives;
        // both expand to exactly the same +/-MinNormal bounds as before.
        const auto key=(bits & Sign)?~bits:(bits | Sign);
        if(first || key<lowerKey){lowerKey=key;lowerBits=bits;}
        if(first || key>upperKey){upperKey=key;upperBits=bits;}
        first=false;
    }
    Interval r;
    r.Known=Outward(FromBits(lowerBits),false,r.Lower) && Outward(FromBits(upperBits),true,r.Upper);
    return r;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticIntervalMultiplySingletonEquivalenceTest,
 "IAmSpeed.AnalyticWorld.TensorIntervalInput.MultiplySingletonEquivalence",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticIntervalMultiplySingletonEquivalenceTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture"),context.Ready))return false;
    using namespace AnalyticCertified;
    const uint32 original=VectorGetControlRegister();uint64 cases=0,pointPoint=0,pointRange=0,rangePoint=0,general=0;
    const auto compare=[&](Interval a,Interval b,uint32 controls)
    {
        VectorSetControlRegister(controls);
        const auto expected=AnalyticCertified::MultiplySingletonOracle::OriginalFourProducts(a,b);
        const uint32 expectedFlags=VectorGetControlRegister();
        VectorSetControlRegister(controls);const auto actual=AnalyticCertified::Multiply(a,b);
        const uint32 actualFlags=VectorGetControlRegister();++cases;
        const bool sa=Bits(a.Lower)==Bits(a.Upper),sb=Bits(b.Lower)==Bits(b.Upper);
        if(sa&&sb)++pointPoint;else if(sa)++pointRange;else if(sb)++rangePoint;else ++general;
        return expected.Known==actual.Known&&Bits(expected.Lower)==Bits(actual.Lower)&&Bits(expected.Upper)==Bits(actual.Upper)&&expectedFlags==actualFlags;
    };
    const auto order=[](uint64 value){return(value&Sign)?~value:(value|Sign);};
    // Normal operands near the normal/subnormal and overflow transitions.
    // Forged subnormal inputs are tested separately, rather than normalized.
    const uint64 edges[]={0,Sign,MinNormal,MinNormal+1,0x001fffffffffffffULL,
        0x0020000000000000ULL,0x2000000000000000ULL,0x3ca0000000000000ULL,
        0x3fe0000000000000ULL,0x3ff0000000000000ULL,0x3ff0000000000001ULL,
        0x4000000000000000ULL,0x6000000000000000ULL,0x7fd0000000000000ULL,
        0x7feffffffffffffeULL,0x7fefffffffffffffULL};
    for(uint32 flush:{0u,64u,32768u,32832u})for(uint32 sticky:{0u,1u,2u,4u,8u,16u,32u,63u})
    {
        const uint32 controls=(original&~uint32(0x807f))|flush|sticky;bool ok=true;
        for(uint64 x:edges)for(uint64 y:edges)for(uint64 sx:{uint64(0),Sign})for(uint64 sy:{uint64(0),Sign})
        {
            const uint64 xb=x^sx,yb=y^sy;uint64 lo=xb,hi=yb;if(order(lo)>order(hi))Swap(lo,hi);
            const Interval scalarX{FromBits(xb),FromBits(xb),true};
            const Interval scalarY{FromBits(yb),FromBits(yb),true};
            const Interval range{FromBits(lo),FromBits(hi),true};
            ok&=compare(scalarX,scalarY,controls);
            ok&=compare(scalarX,range,controls);ok&=compare(range,scalarY,controls);
        }
        const Interval unusual[]={Interval{},Interval{-1,1,false},Interval{2,1,true},
            Interval{0.,FromBits(Sign),true},Interval{FromBits(Sign),0.,true},
            Interval{FromBits(1),FromBits(1),true},Interval{FromBits(Sign|1),FromBits(Sign|1),true},
            Interval{FromBits(MinNormal-1),1,true},Interval{-1,FromBits(Sign|(MinNormal-1)),true},
            Interval{FromBits(Infinity),FromBits(Infinity),true},
            Interval{FromBits(Infinity|1),FromBits(Infinity|1),true},
            Interval{FromBits(Sign|Infinity|1),1,true}};
        for(const auto a:unusual)for(double scalar:{-2.,-1.,0.,.5,1.,2.})
        {ok&=compare(a,Interval::Exact(scalar),controls);ok&=compare(Interval::Exact(scalar),a,controls);}
        // Immediate refusal precedes a different later overflow; a zero result
        // keeps the full original +/-MinNormal Outward enclosure.
        ok&=compare(Interval{1,2,true},Interval::Exact(FromBits(0x7fefffffffffffffULL)),controls);
        ok&=compare(Interval::Exact(FromBits(0x7fefffffffffffffULL)),Interval{1,2,true},controls);
        ok&=compare(Interval{FromBits(Sign|0x7fefffffffffffffULL),FromBits(MinNormal),true},Interval::Exact(.5),controls);
        uint64 seed=0xb8112d7e5e084ec1ULL;
        const auto next=[&](){seed^=seed<<13;seed^=seed>>7;seed^=seed<<17;return seed;};
        for(int32 i=0;i<1024;++i)
        {
            uint64 values[4];for(auto& value:values){do{value=next();}while((value&Magnitude)>=Infinity||((value&Magnitude)!=0&&(value&Magnitude)<MinNormal));}
            if(order(values[0])>order(values[1]))Swap(values[0],values[1]);
            if(order(values[2])>order(values[3]))Swap(values[2],values[3]);
            const Interval rangeA{FromBits(values[0]),FromBits(values[1]),true},rangeB{FromBits(values[2]),FromBits(values[3]),true};
            ok&=compare(rangeA,rangeB,controls);
            ok&=compare(Interval{rangeA.Lower,rangeA.Lower,true},rangeB,controls);
            ok&=compare(rangeA,Interval{rangeB.Upper,rangeB.Upper,true},controls);
        }
        if(!TestTrue(TEXT("identical bounds/refusal/full MXCSR for point-point point-range range-point and general"),ok))return false;
    }
    VectorSetControlRegister(original&~uint32(0x3f));
    const auto zero=AnalyticCertified::Multiply(Interval::Exact(0.),Interval::Exact(7.));
    if(!TestTrue(TEXT("zero product still carries original outward normal allowance"),zero.Known&&Bits(zero.Lower)==(Sign|MinNormal)&&Bits(zero.Upper)==MinNormal))return false;
    const auto six=AnalyticCertified::Multiply(Interval::Exact(2.),Interval::Exact(3.));
    if(!TestTrue(TEXT("an exact scalar product still has both original outward steps"),six.Known&&Bits(six.Lower)==Bits(6.)-1&&Bits(six.Upper)==Bits(6.)+1))return false;
    // Actual dot-product use: constant coordinate/oblique axes multiply
    // interval coordinates. The old and new kernels see the same additions.
    for(uint32 flush:{0u,64u,32768u,32832u})
    {
        const uint32 controls=(original&~uint32(0x807f))|flush;
        const Interval point[3]={{-3.,-2.,true},{.1,.2,true},{17.,18.,true}};
        const double axes[5][3]={{1,0,0},{0,1,0},{0,0,1},{.25,-.75,1.5},{FromBits(MinNormal),-.5,2.}};
        for(const auto& axis:axes)
        {
            VectorSetControlRegister(controls);Interval expected=Interval::Exact(0);
            for(int32 d=0;d<3;++d)expected=Add(expected,AnalyticCertified::MultiplySingletonOracle::OriginalFourProducts(point[d],Interval::Exact(axis[d])));
            const uint32 expectedFlags=VectorGetControlRegister();
            VectorSetControlRegister(controls);Interval actual=Interval::Exact(0);
            for(int32 d=0;d<3;++d)actual=Add(actual,AnalyticCertified::Multiply(point[d],Interval::Exact(axis[d])));
            if(!TestTrue(TEXT("coordinate and oblique dot projections preserve bits and flags"),expected.Known==actual.Known&&Bits(expected.Lower)==Bits(actual.Lower)&&Bits(expected.Upper)==Bits(actual.Upper)&&VectorGetControlRegister()==expectedFlags))return false;
        }
    }
    UE_LOG(LogTemp,Display,TEXT("[IntervalMultiplySingletonEquivalence] Cases=%llu PointPoint=%llu PointRange=%llu RangePoint=%llu General=%llu Modes=4 StickyPresets=8 NoPhysicalAcceptance=1"),cases,pointPoint,pointRange,rangePoint,general);
    TestTrue(TEXT("all dispatch families exercised"),pointPoint>0&&pointRange>0&&rangePoint>0&&general>0);
    TestTrue(TEXT("numeric controls restored"),context.Restore.Restore());return !HasAnyErrors();
}
