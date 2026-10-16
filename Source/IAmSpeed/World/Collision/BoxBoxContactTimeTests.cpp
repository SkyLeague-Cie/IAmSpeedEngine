#if WITH_DEV_AUTOMATION_TESTS
#include "IAmSpeed/Base/SUtils.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBoxBoxContactTimeTest,
    "IAmSpeed.Collision.BoxBox.FirstContactBracket",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FBoxBoxContactTimeTest::RunTest(const FString& Parameters)
{
    constexpr float Delta=1.f/300.f;
    constexpr uint8 Substeps=12;
    int32 Cell=0;
    for(float Closing:{300.f,1200.f,2400.f,4600.f})
    for(float Phase:{.01f,.25f,.5f,.75f,.99f})
    for(float Offset:{-30.f,0.f,30.f})
    {
        const float ExactTime=Phase*Delta/float(Substeps);
        const FVector Extent(78,52,18);
        Speed::SBox A(FVector::ZeroVector,Extent,FQuat::Identity,
            FVector(Closing*.5f,0,0),FVector::ZeroVector,FVector::ZeroVector,FVector::ZeroVector);
        Speed::SBox B(FVector(156+Closing*ExactTime,Offset,0),Extent,FQuat::Identity,
            FVector(-Closing*.5f,0,0),FVector::ZeroVector,FVector::ZeroVector,FVector::ZeroVector);
        const SHitResult Hit=A.IntersectNextFrame(B,Delta,Substeps);
        const SHitResult Reverse=B.IntersectNextFrame(A,Delta,Substeps);
        const FString Label=FString::Printf(TEXT("cell%d closing%.0f phase%.2f offset%.0f"),Cell++,Closing,Phase,Offset);
        TestTrue(*(Label+TEXT(" contact found")),Hit.bHit&&Hit.bBlockingHit);
        TestTrue(*(Label+TEXT(" reverse contact found")),Reverse.bHit&&Reverse.bBlockingHit);
        if(!Hit.bHit || !Reverse.bHit)continue;
        // Independent axis-aligned gap at the returned instant: penetration
        // must not be hidden by the same SAT implementation used to find TOI.
        const double Gap=B.WorldCenter.X-A.WorldCenter.X-156.-double(Closing)*Hit.TOI;
        TestTrue(*(Label+TEXT(" no finite penetration at returned TOI")),Gap>=-0.0001);
        TestTrue(*(Label+TEXT(" no material early event")),Gap<0.002);
        TestTrue(*(Label+TEXT(" time within step")),Hit.TOI>=0&&Hit.TOI<=Delta);
        TestTrue(*(Label+TEXT(" reverse same time")),FMath::Abs(Hit.TOI-Reverse.TOI)<1.e-7);
        TestTrue(*(Label+TEXT(" normal B into A")),Hit.ImpactNormal.X<-.999);
        TestTrue(*(Label+TEXT(" reverse opposite normal")),Reverse.ImpactNormal.Equals(-Hit.ImpactNormal,1.e-5));
        TestTrue(*(Label+TEXT(" repeatable witness/time")),
            A.IntersectNextFrame(B,Delta,Substeps).ImpactPoint.Equals(Hit.ImpactPoint,0)&&
            A.IntersectNextFrame(B,Delta,Substeps).TOI==Hit.TOI);
        AddInfo(FString::Printf(TEXT("[BoxBoxContactTime] %s toi=%.9g gapCm=%.9g"),*Label,Hit.TOI,Gap));
    }
    return true;
}
#endif
