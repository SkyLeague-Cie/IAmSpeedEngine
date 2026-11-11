// Pure mathematical regressions; no gameplay scenario or physical admission.
#include "TensorDistanceBounds.h"
namespace ProjectedDistanceTestDetail
{
using namespace Speed::Analytic::Certified;
inline FIntervalTensorSurface AdverseFamily(bool uncertain=false)
{
    FIntervalTensorSurface net;net.DegreeU=net.DegreeV=5;
    const double x[]={5,2,0,0,2,5};
    for(int32 u=0;u<6;++u)for(int32 v=0;v<6;++v)
    {
        auto p=Detail::Point(FVector3d(x[v],-5.+2.*v,0));
        if(uncertain)for(int32 d=1;d<3;++d)
            p.Axis[d]=AnalyticCertified::Add(p.Axis[d],AnalyticCertified::Interval{-.125,.125,true});
        net.ControlPoints.Add(p);
    }
    return net;
}
inline bool Contains(const FIntervalPoint& p,FVector3d actual)
{
    if(!Detail::ValidPoint(p))return false;
    for(int32 d=0;d<3;++d)if(actual[d]<p.Axis[d].Lower||actual[d]>p.Axis[d].Upper)return false;
    return true;
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticProjectedYZFamilyTest,
    "IAmSpeed.AnalyticWorld.TensorDistance.ProjectedYZUniversalFamily",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticProjectedYZFamilyTest::RunTest(const FString&)
{
    using namespace Speed::Analytic::Certified;using namespace ProjectedDistanceTestDetail;
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("full floating context"),context.Ready))return false;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,64u,32768u,32832u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        const auto net=AdverseFamily();const auto center=Detail::Point(FVector3d::ZeroVector);
        const auto threshold=Interval::Exact(.25);FDistanceBudget budget{1000,13};
        const auto result=BoundTensorDistanceSquaredIntervalCenter(net,center,budget,0.,4,&threshold);
        if(!TestTrue(TEXT("projected inside is NOT a true 3D inside witness"),
            result.Status==EDistanceBoundStatus::Bounded&&budget.Used==17&&
            result.UpperSquared>threshold.Upper&&result.SamplePoint.Axis[0].Lower>.9))return false;
        FIntervalPoint realHalf,projectedHalf;
        TestTrue(TEXT("true point evaluated on original family"),ProjectedDistanceDetail::EvaluateFixedUV(net,FVector2d(.5,.5),realHalf));
        const auto projected=ProjectedDistanceDetail::ProjectYZ(net);
        TestTrue(TEXT("relaxation actually discarded nonzero X"),
            ProjectedDistanceDetail::EvaluateFixedUV(projected,FVector2d(.5,.5),projectedHalf)&&
            projectedHalf.Axis[0].Lower<=0&&projectedHalf.Axis[0].Upper>=0&&realHalf.Axis[0].Lower>.9);
        auto family=AdverseFamily(true);FIntervalPoint enclosure;
        if(!TestTrue(TEXT("identical YZ interval boxes do not assert equal real coefficients"),
            ProjectedDistanceDetail::IsUsefulYZFamily(family)&&
            ProjectedDistanceDetail::EvaluateFixedUV(family,FVector2d(.5,.5),enclosure)))return false;
        // Independent real coefficients, including variation along U. Dyadic
        // half-parameter weights and chosen dyadic coefficients make this
        // independent reference sum exactly representable in binary64 here.
        const int32 weight[]={1,5,10,10,5,1};
        for(int32 pattern=0;pattern<4;++pattern)
        {
            FVector3d actual=FVector3d::ZeroVector;
            for(int32 u=0;u<6;++u)for(int32 v=0;v<6;++v)
            {
                const auto& p=family.ControlPoints[u*6+v];const double w=double(weight[u]*weight[v])/1024.;
                const FVector3d chosen(p.Axis[0].Lower,-5.+2.*v+(u==pattern?.0625:-.0625),
                    u<=pattern?.0625:-.0625);
                for(int32 d=0;d<3;++d)actual[d]+=w*chosen[d];
            }
            TestTrue(TEXT("independent coefficient choices fit full fixed-UV enclosure"),Contains(enclosure,actual));
            TestTrue(TEXT("chosen independent coefficients produce a nonzero U-dependent displacement"),actual.Y!=0||actual.Z!=0);
        }
        FIntervalTensorSurface curved;curved.DegreeU=curved.DegreeV=5;
        const double z[]={5,2,0,0,2,5};
        for(int32 u=0;u<6;++u)for(int32 v=0;v<6;++v)
            curved.ControlPoints.Add(Detail::Point(FVector3d(200.*u-500.,-5.+2.*v,z[v])));
        Detail::FNode root;root.DegreeU=root.DegreeV=5;root.Net=curved.ControlPoints;
        TestTrue(TEXT("true 3D root AABB remains ambiguous before relaxation"),Detail::Bounds(root,center)&&root.LowerSquared==0&&root.UpperSquared>.25);
        FDistanceBudget positiveBudget{1000};
        const auto positive=BoundTensorDistanceSquaredIntervalCenter(curved,center,positiveBudget,0.,7,&threshold);
        TestTrue(TEXT("strictly positive projected global lower proves true separation on same allowance"),
            positive.Status!=EDistanceBoundStatus::Unknown&&positive.LowerSquared>threshold.Upper&&
            positive.LowerSquared<=positive.UpperSquared&&positiveBudget.Used==7);
        auto mirrored=curved;
        for(auto& point:mirrored.ControlPoints)Swap(point.Axis[0],point.Axis[1]);
        FDistanceBudget mirroredBudget{1000};
        const auto mirroredDistance=BoundTensorDistanceSquaredIntervalCenter(mirrored,center,mirroredBudget,0.,7,&threshold);
        TestTrue(TEXT("orthogonal extrusion retains strict original-family separation"),
            ProjectedDistanceDetail::ExtrusionAxis(mirrored)==1&&
            mirroredDistance.Status!=EDistanceBoundStatus::Unknown&&mirroredDistance.LowerSquared>threshold.Upper&&
            mirroredDistance.LowerSquared<=mirroredDistance.UpperSquared&&mirroredBudget.Used>0&&mirroredBudget.Used<=7);
        for(const FVector2d uv:{FVector2d(0,0),FVector2d(1,1),FVector2d(.5,.5),
            FVector2d(AnalyticCertified::FromBits(1),.5)})
        {
            FIntervalPoint point;TestTrue(TEXT("endpoints/interior/positive subnormal fixed UV bounded"),ProjectedDistanceDetail::EvaluateFixedUV(family,uv,point));
        }
        for(const FVector2d uv:{FVector2d(-.1,.5),FVector2d(1.1,.5),FVector2d(.5,2),
            FVector2d(AnalyticCertified::FromBits(0x8000000000000001ULL),.5),
            FVector2d(std::numeric_limits<double>::quiet_NaN(),.5)})
        {
            auto point=enclosure;
            TestFalse(TEXT("invalid parameter cannot forge a point"),ProjectedDistanceDetail::EvaluateFixedUV(family,uv,point));
            TestFalse(TEXT("unknown point clears previous output"),Detail::ValidPoint(point));
        }
    }
    return TestTrue(TEXT("original FP controls restored"),context.Restore.Restore())&&!HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticProjectedYZBudgetTest,
    "IAmSpeed.AnalyticWorld.TensorDistance.ProjectedYZBudgetAndRefusal",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticProjectedYZBudgetTest::RunTest(const FString&)
{
    using namespace Speed::Analytic::Certified;using namespace ProjectedDistanceTestDetail;
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("full floating context"),context.Ready))return false;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,64u,32768u,32832u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        const auto net=AdverseFamily();const auto center=Detail::Point(FVector3d::ZeroVector);
        const auto threshold=Interval::Exact(.25);
        for(int32 cap:{1,2,3,4,5,32,258,259,300})
        {
            FDistanceBudget budget{cap+10,37,10};const auto before=budget;
            const auto result=BoundTensorDistanceSquaredIntervalCenter(net,center,budget,0.,cap,&threshold);
            TestTrue(TEXT("projection and fallback spend only ORIGINAL total allowance/floor"),
                budget.Used>=before.Used&&budget.Used-before.Used<=uint64(cap)&&
                budget.Remaining>=10&&budget.Used+uint64(budget.Remaining)==before.Used+uint64(before.Remaining)&&budget.ConsumptionFloor==10);
            if(result.Status!=EDistanceBoundStatus::Unknown)
                TestTrue(TEXT("bounded schedule cannot turn projected inside into false3D inside"),result.LowerSquared<=result.UpperSquared&&result.UpperSquared>threshold.Upper);
        }
        FDistanceBudget tight{104,17,100};
        const auto denied=BoundTensorDistanceSquaredIntervalCenter(net,center,tight,0.,100,&threshold);
        TestTrue(TEXT("actual global Consume denial remains terminal Unknown"),denied.Status==EDistanceBoundStatus::Unknown&&tight.Used==21&&tight.Remaining==100&&tight.ConsumptionFloor==100);
        const auto generousThreshold=Interval::Exact(100);FDistanceBudget rootBudget{1000};
        const auto root=BoundTensorDistanceSquaredIntervalCenter(net,center,rootBudget,0.,500,&generousThreshold);
        TestTrue(TEXT("already decided true root does no optional work"),root.Status!=EDistanceBoundStatus::Unknown&&rootBudget.Used==1);
        auto rejected=net;rejected.ControlPoints[6].Axis[1]=Interval::Exact(-4.);
        FDistanceBudget rejectedBudget{1000};
        const auto fallback=BoundTensorDistanceSquaredIntervalCenter(rejected,center,rejectedBudget,0.,4,&threshold);
        TestTrue(TEXT("rejected optional gate is paid and fallback shares same local allowance"),fallback.Status!=EDistanceBoundStatus::Unknown&&rejectedBudget.Used==3);
    }
    return TestTrue(TEXT("original FP controls restored"),context.Restore.Restore())&&!HasAnyErrors();
}
