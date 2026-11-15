#include "IAmSpeed/World/Collision/CanonicalFixedSphereWheelProbeMath.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCanonicalFixedSphereWheelProbeMathTest,
 "IAmSpeed.AnalyticWorld.NativeSolid.CanonicalFixedSphereWheelProbeConversions",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCanonicalFixedSphereWheelProbeMathTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;
    if(!TestTrue(TEXT("RNE fixture established"),context.Ready))return false;
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    using namespace FixedWheelProbeMath;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        // Synthetic value fixtures only: no native World/pose/profile or grant.
        FTensorSphereContactEnclosure c;
        c.Status=ETensorSphereContactStatus::FirstContactEnclosed;c.Leaves.AddDefaulted();
        c.LowerTime=c.UpperTime=.5;
        FUniqueFloatSeconds seconds;
        if(!TestTrue(TEXT("exact fixed-probe normalized time uses actual double Delta"),TryUniqueFloatSeconds(c,1./300.,seconds)))return false;
        TestTrue(TEXT("actual downstream conversion agrees bitwise"),
            FloatBits(static_cast<float>(seconds.NormalizedRepresentative*(1./300.)))==seconds.NativeSecondsBits);
        c.LowerTime=.25;c.UpperTime=.75;
        TestFalse(TEXT("bracket crossing native float bins never chooses a midpoint TOI"),TryUniqueFloatSeconds(c,1.,seconds));
        TestTrue(TEXT("failed time conversion clears previous proof"),seconds.NativeSecondsBits==0&&!seconds.Seconds.Known);
        c.LowerTime=c.UpperTime=0;
        TestTrue(TEXT("exact positive-zero endpoint retained without negative outward shell"),TryUniqueFloatSeconds(c,1.,seconds)&&seconds.NativeSecondsBits==0);
        c.LowerTime=c.UpperTime=1;
        const double delta=double(1.f/300.f)-std::numeric_limits<double>::epsilon();
        if(!TestTrue(TEXT("legitimate last endpoint converted without clipping float seconds"),TryUniqueFloatSeconds(c,delta,seconds)))return false;
        TestTrue(TEXT("native endpoint may round above double Delta"),double(seconds.NativeSeconds)>delta);
        c.LowerTime=.25;c.UpperTime=.75;
        TestTrue(TEXT("real +0 Delta maps entire root bracket to native +0"),TryUniqueFloatSeconds(c,0.,seconds)&&
            seconds.NativeSecondsBits==0&&seconds.NormalizedRepresentative==.25&&seconds.Seconds.Lower==0&&seconds.Seconds.Upper==0);
        TestTrue(TEXT("zero Delta keeps geometric lower, distinct from native time"),
            FloatBits(static_cast<float>(seconds.NormalizedRepresentative*0.))==0&&seconds.NormalizedRepresentative!=0);
        TestFalse(TEXT("negative-zero Delta never becomes a positive-zero proof"),TryUniqueFloatSeconds(c,-0.,seconds));
        TestFalse(TEXT("raw subnormal Delta never becomes zero under DAZ"),TryUniqueFloatSeconds(c,AnalyticCertified::FromBits(1),seconds));
        auto empty=c;empty.Leaves.Reset();TestFalse(TEXT("zero Delta still requires retained contact leaves"),TryUniqueFloatSeconds(empty,0.,seconds));
        c.LowerTime=c.UpperTime=1;
        TestFalse(TEXT("native float overflow is unknown"),TryUniqueFloatSeconds(c,std::numeric_limits<double>::max(),seconds));
        c.LowerTime=c.UpperTime=AnalyticCertified::FromBits(0x8000000000000001ULL);
        TestFalse(TEXT("negative raw subnormal is never disguised by DAZ"),TryUniqueFloatSeconds(c,1.,seconds));
        c.LowerTime=c.UpperTime=std::numeric_limits<double>::quiet_NaN();
        TestFalse(TEXT("NaN root bracket unsupported"),TryUniqueFloatSeconds(c,1.,seconds));
        FUniqueQuantizedNormal normal;FDistanceBudget work{256};
        const auto up=Detail::Point(FVector3d(0,0,1));
        if(!TestTrue(TEXT("all up normals share existing quantization outputs"),TryUniqueQuantizedNormal(up,FVector(0,0,10),work,normal)))return false;
        const FVector actual=Speed::QuantizeUnitNormal(FVector(0,0,10));
        TestTrue(TEXT("real scalar quantized and renormalized bits retained separately"),
            SameBits(normal.Quantized,actual)&&SameBits(normal.ReNormalized,actual.GetSafeNormal()));
        const FVector obliqueStored(3,4,5);
        const auto obliqueCone=Detail::Point(FVector3d(obliqueStored.GetSafeNormal()));work={256};
        if(!TestTrue(TEXT("oblique three-component normal has one actual quantization cell"),
            TryUniqueQuantizedNormal(obliqueCone,obliqueStored,work,normal)))return false;
        const FVector obliqueActual=Speed::QuantizeUnitNormal(obliqueStored);
        TestTrue(TEXT("oblique quantized and downstream normalized bits agree"),
            SameBits(normal.Quantized,obliqueActual)&&SameBits(normal.ReNormalized,obliqueActual.GetSafeNormal()));
        work={256};
        TestTrue(TEXT("scalar outside true cone accepted only on identical consumed discrete bits"),
            TryUniqueQuantizedNormal(up,FVector(.00001,0,1),work,normal)&&SameBits(normal.Quantized,actual));
        work={256};
        TestFalse(TEXT("different scalar cell cannot reuse true normal-cell proof"),TryUniqueQuantizedNormal(up,FVector(1,0,0),work,normal));
        TestTrue(TEXT("normal refusal clears previous outputs"),normal.Quantized.IsZero()&&normal.ReNormalized.IsZero());
        auto straddling=up;const double step=double(1.e-3f);
        straddling.Axis[0]={step*.49,step*.51,true};straddling.Axis[2]={.999,1.001,true};work={256};
        TestFalse(TEXT("cone spanning a real normal quantization boundary refuses"),
            TryUniqueQuantizedNormal(straddling,FVector(step*.5,0,1),work,normal));
        FDistanceBudget exhausted{0};
        TestFalse(TEXT("exhausted normal work cannot emit a stale cell"),TryUniqueQuantizedNormal(up,FVector(0,0,1),exhausted,normal));
        TestTrue(TEXT("no prior normal proof after exhausted work"),normal.Quantized.IsZero()&&normal.ReNormalized.IsZero());
        FReferenceDyadicCell decoded;
        TestTrue(TEXT("exact retained quarter cell decoded"),DecodeLeafDyadic(.25,.5,decoded)&&decoded.Depth==2&&decoded.Index==1);
        TestFalse(TEXT("non-dyadic leaf interval refused"),DecodeLeafDyadic(.2,.4,decoded));
        TestFalse(TEXT("depth53 leaf refused"),DecodeLeafDyadic(0.,1./9007199254740992.,decoded));
        FTensorSphereContactLeaf leaf;leaf.SurfaceIndex=0;leaf.MinU=leaf.MinV=.25;leaf.MaxU=leaf.MaxV=.5;
        const auto buildScene=[&](bool singular,bool reverse,FIndexedIntervalUnionPtr& scene)
        {
            FIntervalTensorSurface tensor;tensor.DegreeU=tensor.DegreeV=singular?2:1;
            constexpr double square[3]={.140625,-.234375,.390625};
            for(int32 i=0;i<=tensor.DegreeU;++i)for(int32 j=0;j<=tensor.DegreeV;++j)
                tensor.ControlPoints.Add(Detail::Point(FVector3d(singular?square[i]:double(reverse?-i:i),singular?square[j]:double(j),0.)));
            TArray<FIntervalTensorSurface> tensors;tensors.Add(MoveTemp(tensor));FDistanceBudget create{256};
            return FIndexedIntervalUnion::Build(tensors,create,scene);
        };
        FIndexedIntervalUnionPtr scene;P chart;
        if(!TestTrue(TEXT("regular plane geometry fixture built"),buildScene(false,false,scene)))return false;
        work={512};TestTrue(TEXT("rank2 proven on same interior retained leaf"),ProveRegularFaceLeaf(scene,leaf,work,chart));
        if(!TestTrue(TEXT("opposite orientation geometry fixture built"),buildScene(false,true,scene)))return false;
        work={512};TestTrue(TEXT("rank2 independent of chart orientation"),ProveRegularFaceLeaf(scene,leaf,work,chart));
        if(!TestTrue(TEXT("interior singular geometry fixture built"),buildScene(true,false,scene)))return false;
        work={512};TestFalse(TEXT("interior folded-square singularity never relabelled Face"),ProveRegularFaceLeaf(scene,leaf,work,chart));
        TestTrue(TEXT("rank refusal clears previous chart certificate"),!chart.Axis[0].Known&&!chart.Axis[1].Known&&!chart.Axis[2].Known);
        exhausted={0};TestFalse(TEXT("no rank certificate without work"),ProveRegularFaceLeaf(scene,leaf,exhausted,chart));
    }
    TestTrue(TEXT("floating context restored"),context.Restore.Restore());return !HasAnyErrors();
}
