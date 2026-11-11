#include "IAmSpeed/World/Collision/PreparedNativeAnalyticContactValues.h"
#include "Components/BoxComponent.h"
#include "UObject/StrongObjectPtr.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPreparedNativeAnalyticContactValuesTest,
 "IAmSpeed.AnalyticWorld.NativeSolid.ContactTokenAndExactValues",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FPreparedNativeAnalyticContactValuesTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture established"),context.Ready))return false;
    if(!TestTrue(TEXT("transient token fixture only created on GT"),IsInGameThread()))return false;
    using namespace Speed;using namespace Speed::PreparedExecution;
    const TStrongObjectPtr<UBoxComponent> contact(NewObject<UBoxComponent>()),other(NewObject<UBoxComponent>());
    FPreparedSourceContactBinding binding;binding.SourceId=1;binding.SourceIdentity=contact.Get();binding.ContactIdentity=contact.Get();binding.ContactToken=contact.Get();
    Analytic::FWorldHit input;input.bHit=true;input.SourceId=1;input.SurfaceId=2;input.PrimitiveId=3;input.FeatureId=4;
    input.Location=FVector(0,0,1);input.QueryPoint=FVector::ZeroVector;input.Point=FVector::ZeroVector;input.Normal=FVector(0,0,1);
    input.QueryFeatureKind=EContactFeatureKind::Vertex;input.QueryFeatureIndex=6;input.SurfaceFeatureKind=EContactFeatureKind::Face;input.SurfaceFeatureIndex=2;
    const float seconds=0.123456789f;SHitResult hit;
    TestTrue(TEXT("copy GT Contact and actual feature values at explicit seconds"),BuildNativeAnalyticContactValues(input,seconds,17,binding,hit));
    TestTrue(TEXT("typed token identity retained without lookup"),hit.Component.HasSameIndexAndSerialNumber(binding.ContactToken));
    TestTrue(TEXT("explicit seconds retain exact float bits"),Analytic::Certified::KinematicCenterDetail::Bits(hit.TOI)==Analytic::Certified::KinematicCenterDetail::Bits(seconds));
    TestTrue(TEXT("actual features copied and triangle index not invented"),hit.ContactFeatureThis==input.QueryFeatureKind&&hit.ContactFeatureOther==input.SurfaceFeatureKind&&
        hit.ContactFeatureIndexThis==6&&hit.ContactFeatureIndexOther==2&&hit.FaceIndex==INDEX_NONE);
    auto changed=hit;TestTrue(TEXT("same complete native values compare equal"),SameNativeAnalyticContactValues(hit,changed));
    changed.ContactFeatureIndexThis=5;TestFalse(TEXT("feature index mutation detected"),SameNativeAnalyticContactValues(hit,changed));
    changed=hit;changed.Component=other.Get();TestFalse(TEXT("Contact identity mutation detected"),SameNativeAnalyticContactValues(hit,changed));
    changed=hit;changed.TOI=-0.f;auto zero=hit;zero.TOI=0.f;TestFalse(TEXT("signed seconds mutation detected"),SameNativeAnalyticContactValues(zero,changed));
    changed=hit;changed.ContactPointOther.X=-0.;TestFalse(TEXT("signed witness bit mutation detected"),SameNativeAnalyticContactValues(hit,changed));
    auto absent=binding;absent.ContactToken.Reset();TestFalse(TEXT("absent GT Contact cannot become usable hit values"),BuildNativeAnalyticContactValues(input,seconds,17,absent,changed));
    TestFalse(TEXT("refusal clears prior hit"),changed.bHit);
    TestFalse(TEXT("negative-zero consumed time refused"),BuildNativeAnalyticContactValues(input,-0.f,17,binding,changed));
    input.bStartPenetrating=true;input.PenetrationDepth=0.125;
    TestTrue(TEXT("normal captured support depth retains existing nearest float conversion"),BuildNativeAnalyticContactValues(input,0.f,17,binding,changed)&&
        changed.bStartPenetrating&&changed.PenetrationDepth==0.125f&&changed.TOI==0);
    const uint32 control=VectorGetControlRegister();VectorSetControlRegister(control|uint32(0x8040));
    input.PenetrationDepth=AnalyticCertified::FromBits(0x3800000000000000ULL); // 2^-127: positive float subnormal / flushed zero.
    TestFalse(TEXT("positive depth narrowed to subnormal or FTZ zero is refused"),BuildNativeAnalyticContactValues(input,0.f,17,binding,changed));
    TestFalse(TEXT("depth narrowing refusal clears prior native hit"),changed.bHit);
    input.PenetrationDepth=AnalyticCertified::FromBits(0x3600000000000000ULL); // 2^-159: nearest float zero.
    TestFalse(TEXT("positive depth cannot narrow to invented tangency"),BuildNativeAnalyticContactValues(input,0.f,17,binding,changed));
    input.PenetrationDepth=AnalyticCertified::FromBits(0x7fefffffffffffffULL);
    TestFalse(TEXT("finite support depth overflowing native float remains Unknown"),BuildNativeAnalyticContactValues(input,0.f,17,binding,changed));
    input.PenetrationDepth=AnalyticCertified::FromBits(0x3810000000000000ULL); // Exact float minimum normal.
    TestTrue(TEXT("minimum normal native depth is retained without inflation"),BuildNativeAnalyticContactValues(input,0.f,17,binding,changed)&&
        Analytic::Certified::KinematicCenterDetail::Bits(changed.PenetrationDepth)==0x00800000u);
    VectorSetControlRegister(control);
    TestTrue(TEXT("original floating context restored"),context.Restore.Restore());return !HasAnyErrors();
}
