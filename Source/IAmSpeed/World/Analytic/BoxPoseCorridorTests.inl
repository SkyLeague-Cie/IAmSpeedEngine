// FUTURE PURE-VALUE REGRESSIONS ONLY. Not installed or executed. These fixtures
// define no physical scenario, pose, vehicle profile, timestep or game threshold.
#include "BoxPoseCorridorBounds.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticBoxPoseCorridorFamilyTest,
    "IAmSpeed.AnalyticWorld.GeneralBox.PoseCorridorFamily",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticBoxPoseCorridorFamilyTest::RunTest(const FString&)
{
    using namespace Speed::Analytic::Certified;
    // Included in TensorEvaluationTests.cpp after its existing full x87/MXCSR
    // fixture. It masks exceptions, establishes RNE and restores both units.
    FTensorBoundsTestContext context;
    if(!TestTrue(TEXT("Complete floating context established"),context.Ready))return false;
    const uint32 original=VectorGetControlRegister();
    for(uint32 flush:{0u,64u,32768u,32832u})
    {
        VectorSetControlRegister((original&~(0x6000u|0x8040u))|flush);
        FBoxPoseCorridorInput v;
        v.BeforeCOM=FVector(17.125,-9.75,41.5);v.AfterCOM=FVector(17.375,-9.5,41.625);
        v.BeforeRotation=FQuat(FVector(1,2,3).GetSafeNormal(),.07);
        v.AfterRotation=(FQuat(FVector(3,-1,2).GetSafeNormal(),.003)*v.BeforeRotation).GetNormalized();
        v.LocalCOM=FVector(2.25,-1.5,4.75);v.LocalOffset=FVector(-1.125,2.5,8.25);
        v.LocalRotation=FQuat(FVector(2,3,-1).GetSafeNormal(),.09);
        v.PhysicalHalfExtent=FVector(11.25,7.75,3.125);v.QueryHalfExtent=FVector(11.5,7.5,3.25);
        v.DomainShell=.00003125;v.OverlapShell=.000015625;
        FIntervalPoint hull;
        if(!TestTrue(TEXT("Pure corridor bounds available in each flush mode"),BuildBoxPoseCorridorBounds(v,hull)))return false;
        const auto contains=[&](const FVector& p)
        {
            for(int32 d=0;d<3;++d)
                if(!FMath::IsFinite(p[d])||p[d]<hull.Axis[d].Lower||p[d]>hull.Axis[d].Upper)return false;
            return true;
        };
        // Real native arithmetic independently checks both raw endpoints and
        // sampled raw/normalized chord configurations. Samples are regression
        // evidence; the full-family enclosure relies on interval composition.
        for(int32 i=0;i<=32;++i)
        {
            const double s=double(i)/32.;
            const FVector p=v.BeforeCOM*(1.-s)+v.AfterCOM*s;
            const FQuat raw(v.BeforeRotation.X*(1.-s)+v.AfterRotation.X*s,
                v.BeforeRotation.Y*(1.-s)+v.AfterRotation.Y*s,
                v.BeforeRotation.Z*(1.-s)+v.AfterRotation.Z*s,
                v.BeforeRotation.W*(1.-s)+v.AfterRotation.W*s);
            for(const FQuat& q:{raw,raw.GetNormalized()})
            {
                const FVector origin=p-q.RotateVector(v.LocalCOM);
                const FVector center=origin+q.RotateVector(v.LocalOffset);
                const FQuat body=q*v.LocalRotation;
                for(const FVector& extent:{v.PhysicalHalfExtent,v.QueryHalfExtent})for(int32 k=0;k<8;++k)
                {
                    FVector local;
                    for(int32 d=0;d<3;++d)local[d]=(k&(1<<d))?extent[d]:-extent[d];
                    if(!TestTrue(TEXT("Actual native corner inside complete corridor"),contains(center+body.RotateVector(local))))return false;
                }
            }
        }
        if(!TestTrue(TEXT("Inclusive containment preserves strict outer separation"),BoxPoseCorridorWithin(hull,hull)))return false;
        auto tooSmall=hull;tooSmall.Axis[0].Lower=tooSmall.Axis[0].Upper;
        if(!TestFalse(TEXT("Escaped corridor cannot reuse retained envelope"),BoxPoseCorridorWithin(tooSmall,hull)))return false;
        const auto refuse=[&](const FBoxPoseCorridorInput& invalid,const TCHAR* why)
        {
            auto output=hull;
            return TestFalse(why,BuildBoxPoseCorridorBounds(invalid,output))&&
                TestFalse(TEXT("Unknown clears any prior hull"),Detail::ValidPoint(output));
        };
        auto invalid=v;invalid.BeforeRotation=FQuat::Identity;invalid.AfterRotation=FQuat(0,0,0,-1);
        if(!refuse(invalid,TEXT("Antipodal chord not sign-flipped into acceptance")))return false;
        invalid=v;invalid.PhysicalHalfExtent.X=0;
        if(!refuse(invalid,TEXT("Zero physical extent refused")))return false;
        invalid=v;invalid.DomainShell=-.25;
        if(!refuse(invalid,TEXT("Negative captured shell refused")))return false;
        invalid=v;invalid.LocalRotation=FQuat(0,0,0,0);
        if(!refuse(invalid,TEXT("Invalid native local quaternion refused")))return false;
    }
    return TestTrue(TEXT("Complete original floating context restored"),context.Restore.Restore())&&!HasAnyErrors();
}
