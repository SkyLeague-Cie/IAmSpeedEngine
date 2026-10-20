#include "FrameSolveWitness.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "IAmSpeed/SubBodies/Solid/BoxSubBody.h"
#include "Components/BoxComponent.h"
#include "UObject/UObjectGlobals.h"
#include "IAmSpeed/World/Analytic/AnalyticWorldData.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFramePairTokenWitnessTests,
    "IAmSpeed.Simulation.FrameWitness.PairTokenIdentity",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FFramePairTokenWitnessTests::RunTest(const FString&)
{
    using namespace Speed;
    auto* A=NewObject<UBoxSubBody>();auto* B=NewObject<UBoxSubBody>();
    auto* X=NewObject<UBoxComponent>();auto* Y=NewObject<UBoxComponent>();
    FFramePointerRegistry RA,RB;
    const FFramePointerKey Source{1,41,1,2,0},Other{2,97,1,3,0};
    TestTrue(TEXT("left resolver identity"),RA.Register(A,Source));
    TestTrue(TEXT("left other identity"),RA.Register(X,Other));
    TestTrue(TEXT("right resolver identity"),RB.Register(B,Source));
    TestTrue(TEXT("right other identity"),RB.Register(Y,Other));
    TestTrue(TEXT("left sealed"),RA.Seal());TestTrue(TEXT("right sealed"),RB.Seal());
    const uint64 KA=USSubBody::MakePairKey(A,X),KB=USSubBody::MakePairKey(B,Y);
    TestTrue(TEXT("separate UObject pairs have separate raw tokens"),KA!=KB);
    TArray<uint8> RawA,RawB,CanA,CanB;SHitResult HA,HB;
    HA.bHit=HB.bHit=true;HA.TOI=HB.TOI=0.125f;HA.Component=X;HB.Component=Y;
    {
        FFrameSolveWitnessScope Scope(9,RawA,RA);
        FFrameSolveWitnessScope::Record(9,0,0,41,0,KA,HA,false,0,1,A);
        TestTrue(TEXT("actual left resolver makes token known"),Scope.MakeCanonical(CanA));
    }
    {
        FFrameSolveWitnessScope Scope(9,RawB,RB);
        FFrameSolveWitnessScope::Record(9,0,0,41,0,KB,HB,false,0,1,B);
        TestTrue(TEXT("actual right resolver makes token known"),Scope.MakeCanonical(CanB));
    }
    TestTrue(TEXT("equivalent typed pair identities match"),CanA==CanB);
    TestTrue(TEXT("raw tokens and addresses retained"),RawA!=RawB);
    {
        TArray<uint8> Raw,Out;FFrameSolveWitnessScope Scope(9,Raw,RA);
        FFrameSolveWitnessScope::Record(9,0,0,41,0,KB,HA,false,0,1,A);
        TestFalse(TEXT("token not derived from actual resolver is UNKNOWN"),Scope.MakeCanonical(Out));
    }
    {
        TArray<uint8> Raw,Out;FFrameSolveWitnessScope Scope(9,Raw,RA);
        FFrameSolveWitnessScope::Record(9,0,0,41,0,KA,HA,false,0,1);
        TestFalse(TEXT("missing resolver is UNKNOWN"),Scope.MakeCanonical(Out));
    }
    // Exercise the production key formula with distinct UObject IDs. Provider
    // grouping overrides SurfaceId; planar vertex salt applies only with the
    // actual resting world, a fixed surface normal, and a valid vertex feature.
    for(int32 Kind=0;Kind<3;++Kind)for(int32 Variant=0;Variant<5;++Variant)
    {
        SHitResult LA=HA,LB=HB;
        LA.SurfaceId=LB.SurfaceId=71;
        LA.CanonicalGroupId=LB.CanonicalGroupId=Kind==0?0:83;
        LA.bSurfaceNormalMayVary=LB.bSurfaceNormalMayVary=Variant==2;
        LA.ContactFeatureThis=LB.ContactFeatureThis=Variant==3?EContactFeatureKind::Face:EContactFeatureKind::Vertex;
        LA.ContactFeatureIndexThis=LB.ContactFeatureIndexThis=Variant==4?-1:2;
        const bool Resting=Kind==2;
        const auto Key=[&](const USSubBody* S,const SHitResult& H)
        {
            uint64 K=USSubBody::MakePairKey(S,H.Component.Get());
            const uint64 Provider=H.CanonicalGroupId?H.CanonicalGroupId:H.SurfaceId;
            if(Provider)K=Analytic::CombineStableIds(K,Provider);
            if(Resting && H.SurfaceId && !H.bSurfaceNormalMayVary && H.ContactFeatureThis==EContactFeatureKind::Vertex && H.ContactFeatureIndexThis>=0)
                K=Analytic::CombineStableIds(K,0x504c414e00000000ull|uint64(H.ContactFeatureIndexThis+1));
            return K;
        };
        const uint64 LK=Key(A,LA),RK=Key(B,LB);
        TArray<uint8> LR,RR,LC,RC;
        {FFrameSolveWitnessScope S(9,LR,RA);FFrameSolveWitnessScope::Record(9,0,0,41,0,LK,LA,false,0,1,A,Resting);
         TestTrue(TEXT("left annotated native solver key verified"),S.MakeCanonical(LC));}
        {FFrameSolveWitnessScope S(9,RR,RB);FFrameSolveWitnessScope::Record(9,0,0,41,0,RK,LB,false,0,1,B,Resting);
         TestTrue(TEXT("right annotated native solver key verified"),S.MakeCanonical(RC));}
        TestTrue(TEXT("exact annotated semantic pairs match without discarding raw identity"),LC==RC && LR!=RR);
        for(int32 Bad=0;Bad<5;++Bad)
        {
            SHitResult H=LA;uint64 K=LK;const USSubBody* S=A;bool R=Resting;
            if(Bad==0)K^=1; // forged token
            if(Bad==1)S=B;  // wrong and undeclared source
            if(Bad==2)H.CanonicalGroupId=101; // wrong provider
            if(Bad==3)S=nullptr;
            if(Bad==4){H.Component=Y;}
            TArray<uint8> Raw,Out{9,7};const auto Before=Out;
            FFrameSolveWitnessScope Scope(9,Raw,RA);
            FFrameSolveWitnessScope::Record(9,0,0,41,0,K,H,false,0,1,S,R);
            TestFalse(TEXT("invalid annotated token remains UNKNOWN"),Scope.MakeCanonical(Out));
            TestTrue(TEXT("failed normalization preserves original raw and destination"),!Raw.IsEmpty() && Out==Before);
        }
        if(Resting && Variant==0)
        {
            TArray<uint8> Raw,Out;FFrameSolveWitnessScope Scope(9,Raw,RA);
            FFrameSolveWitnessScope::Record(9,0,0,41,0,LK,LA,false,0,1,A,false);
            TestFalse(TEXT("incorrect resting-world annotation is UNKNOWN"),Scope.MakeCanonical(Out));
        }
    }
    return true;
}
#endif
