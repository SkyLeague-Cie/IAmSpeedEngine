#include "ScalarSphereContactBounds.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticScalarSphereContactTest,
 "IAmSpeed.AnalyticWorld.ScalarContact.ActualConsumersRadiusAndAmbiguity",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticScalarSphereContactTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture established"),context.Ready))return false;
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    const uint32 original=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((original&~uint32(0x8040))|flush);
        auto enclose=[](TArray<FIntervalTensorSurface>& surfaces,const FVector3d& start,double radius,double width)
        {
            FDistanceBudget budget{1048576};FIntervalBezierCenterPath path;
            if(!CenterDetail::Linear(start,FVector3d::ZeroVector,path))return FTensorSphereContactEnclosure{};
            return BoundTensorSphereFirstContact(MakeArrayView(surfaces),path,radius,budget,1./1048576.,1.e-7,512,width,width);
        };
        FIntervalTensorSurface plane;plane.DegreeU=1;plane.DegreeV=1;
        for(const auto& p:TArray<FVector3d>{FVector3d(-2,-2,0),FVector3d(-2,2,0),FVector3d(2,-2,0),FVector3d(2,2,0)})plane.ControlPoints.Add(Detail::Point(p));
        TArray<FIntervalTensorSurface> planar={plane};const auto contact=enclose(planar,FVector3d(0,0,3),2.,0.001);
        if(TestTrue(TEXT("actual first planar contact enclosed"),contact.Status==ETensorSphereContactStatus::FirstContactEnclosed))
        {
            TestTrue(TEXT("producer records actual radius bits"),contact.CollisionRadiusBits==AnalyticCertified::Bits(2.));
            FScalarSphereContact scalar;
            if(TestTrue(TEXT("two distinct radii converted with separate consumers"),ConvertSphereContact(contact,0,FVector3d::ZeroVector,FVector3d(0,0,2),2.,1.,scalar)==EScalarSphereContactStatus::Bounded))
            {
                TestTrue(TEXT("impact witness and current center remain distinct"),scalar.ImpactPoint==FVector3d::ZeroVector&&scalar.Location==FVector3d(0,0,2));
                TestTrue(TEXT("physical response arm retains collision/physical radius gap"),scalar.RadialResponsePoint==FVector3d(0,0,1));
                TestTrue(TEXT("all separate error boxes remain small in this exact reference case"),scalar.SampleEvaluationErrorCm<0.01&&scalar.SurfaceErrorCm<0.01&&scalar.LocationErrorCm<0.01&&scalar.ResponseErrorCm<0.01);
                // Natural caller re-bound aliases its previous output. Parameters
                // must survive output reset, not accidentally become zero.
                TestTrue(TEXT("self re-bound copies vector inputs before clearing output"),ConvertSphereContact(contact,0,scalar.ImpactPoint,scalar.Location,2.,1.,scalar)==EScalarSphereContactStatus::Bounded&&scalar.Location==FVector3d(0,0,2)&&scalar.RadialResponsePoint==FVector3d(0,0,1));
            }
            scalar.Location=FVector3d(9,9,9);
            TestTrue(TEXT("a different radius cannot reuse the enclosure and leaves no stale output"),ConvertSphereContact(contact,0,FVector3d::ZeroVector,FVector3d(0,0,3),3.,1.,scalar)==EScalarSphereContactStatus::Unknown&&scalar.RepresentativeSurfaceIndex==INDEX_NONE&&scalar.Location==FVector3d::ZeroVector);
        }
        // A point polynomial isolates oblique radial normalization without any
        // regular-surface/World claim. The output itself is NOT a hit certificate.
        FIntervalTensorSurface point;point.ControlPoints.Add(Detail::Point(FVector3d::ZeroVector));TArray<FIntervalTensorSurface> onePoint={point};
        const auto oblique=enclose(onePoint,FVector3d(3,3,3),3.,0.001);
        if(TestTrue(TEXT("actual oblique radial first contact enclosed"),oblique.Status==ETensorSphereContactStatus::FirstContactEnclosed))
        {
            const double r=FMath::Sqrt(3.);const FVector3d center(r,r,r),approximation(1.e-5,0,-1.e-5);FScalarSphereContact scalar;
            if(TestTrue(TEXT("actual oblique scalar outputs bounded"),ConvertSphereContact(oblique,0,approximation,center,3.,1.,scalar)==EScalarSphereContactStatus::Bounded))
            {
                const auto stored=(center-approximation).GetSafeNormal();const auto consumed=stored.GetSafeNormal();
                for(int32 d=0;d<3;++d)TestTrue(TEXT("actual stored and consumed normalization bits are independently retained"),AnalyticCertified::Bits(scalar.NormalStored[d])==AnalyticCertified::Bits(stored[d])&&AnalyticCertified::Bits(scalar.NormalConsumed[d])==AnalyticCertified::Bits(consumed[d]));
                TestTrue(TEXT("a biased scalar sample retains nonzero sample error instead of an exact evaluator claim"),scalar.SampleEvaluationErrorCm>=2.e-5&&scalar.SurfaceErrorCm>=2.e-5);
                TestTrue(TEXT("both actual normalized outputs receive distinct finite enclosures"),TimeDetail::NonnegativeFinite(scalar.StoredNormalErrorL1)&&TimeDetail::NonnegativeFinite(scalar.ConsumedNormalErrorL1)&&scalar.StoredNormalErrorL1<0.01&&scalar.ConsumedNormalErrorL1<0.01);
            }
        }
        FIntervalTensorSurface left,right;left.ControlPoints.Add(Detail::Point(FVector3d(-0.6,0,0)));right.ControlPoints.Add(Detail::Point(FVector3d(0.6,0,0)));
        TArray<FIntervalTensorSurface> twoPoints={left,right};const auto competing=enclose(twoPoints,FVector3d(0,0,2),1.,2.);
        if(TestTrue(TEXT("both symmetric first contacts remain in one global enclosure"),competing.Status==ETensorSphereContactStatus::FirstContactEnclosed))
        {
            FScalarSphereContact rejected;
            TestTrue(TEXT("a selected representative cannot hide the opposite true contact normal"),ConvertSphereContact(competing,0,FVector3d(-0.6,0,0),FVector3d(0,0,0.8),1.,0.5,rejected)==EScalarSphereContactStatus::Unknown&&rejected.RepresentativeSurfaceIndex==INDEX_NONE);
        }
    }
    VectorSetControlRegister(original);TestTrue(TEXT("original floating context restored"),context.Restore.Restore());return !HasAnyErrors();
}
namespace
{
FAutoConsoleCommand ScalarSphereContactUnit(TEXT("Speed.Analytic.RunScalarSphereContactUnit"),
 TEXT("One scalar contact consumer/radius/ambiguity unit; no World, actor, save or physical admission."),
 FConsoleCommandDelegate::CreateLambda([]()
 {
    static bool used=false;auto& framework=FAutomationTestFramework::Get();const TCHAR* type=TEXT("FAnalyticScalarSphereContactTest");const TCHAR* name=TEXT("IAmSpeed.AnalyticWorld.ScalarContact.ActualConsumersRadiusAndAmbiguity");
    if(!IsRunningCommandlet()||!FParse::Param(FCommandLine::Get(),TEXT("AnalyticScalarContactUnit"))||used||GIsAutomationTesting||GIsSlowTask||GIsPlayInEditorWorld||framework.GetCurrentTest()||!framework.IsLatentCommandQueueEmpty()||!framework.ContainsTest(type))
    {UE_LOG(LogTemp,Error,TEXT("Scalar sphere contact unit refused"));return;}
    used=true;framework.SetRequestedTestFilter(EAutomationTestFlags::EngineFilter);framework.StartTestByName(type,0,name);
    if(!GIsAutomationTesting||!framework.GetCurrentTest()||framework.GetCurrentTestFullPath()!=name||!framework.IsLatentCommandQueueEmpty())
    {UE_LOG(LogTemp,Error,TEXT("Scalar sphere synchronous test ownership refused"));return;}
    FAutomationTestExecutionInfo info;const bool stopped=framework.StopTest(info);
    if(GIsAutomationTesting||GIsSlowTask||GIsPlayInEditorWorld||framework.GetCurrentTest()||!framework.IsLatentCommandQueueEmpty())
    {UE_LOG(LogTemp,Error,TEXT("Scalar sphere final test ownership refused"));return;}
    const bool passed=stopped&&info.bSuccessful&&info.GetErrorTotal()==0&&info.GetWarningTotal()==0;
    UE_LOG(LogTemp,Display,TEXT("ANALYTIC_SCALAR_CONTACT_UNIT name=%s passed=%d errors=%d warnings=%d"),name,passed,info.GetErrorTotal(),info.GetWarningTotal());
 }));
}
