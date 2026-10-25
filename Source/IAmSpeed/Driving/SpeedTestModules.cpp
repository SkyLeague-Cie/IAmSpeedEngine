#include "SpeedTestModules.h"
#include "IAmSpeed/SubBodies/Solid/RingSubBody.h"
#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"

ASpeedRingObstacle::ASpeedRingObstacle()
{
    PrimaryActorTick.bCanEverTick=false;
    Solid=CreateDefaultSubobject<URingSubBody>(TEXT("RingSolid"));SetRootComponent(Solid);
    Solid->MajorRadius=1400;Solid->TubeRadius=160;
    Visual=CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RingVisual"));Visual->SetupAttachment(Solid);
    Visual->SetMobility(EComponentMobility::Static);Visual->SetCollisionEnabled(ECollisionEnabled::NoCollision);
}
ASpeedRampModule::ASpeedRampModule()
{
    PrimaryActorTick.bCanEverTick=false;
    auto* Root=CreateDefaultSubobject<USceneComponent>(TEXT("RampRoot"));Root->SetMobility(EComponentMobility::Static);SetRootComponent(Root);
    SolidMesh=CreateDefaultSubobject<UStaticMeshComponent>(TEXT("ClosedPrism"));SolidMesh->SetupAttachment(Root);
    SolidMesh->SetMobility(EComponentMobility::Static);SolidMesh->SetSimulatePhysics(false);
    SolidMesh->SetCollisionEnabled(ECollisionEnabled::QueryOnly);SolidMesh->SetCollisionObjectType(ECC_WorldStatic);
    SolidMesh->SetCollisionResponseToAllChannels(ECR_Block);
}
bool ASpeedRampModule::BuildClosedPrism(TArray<FVector>& V,TArray<int32>& T) const
{
    V.Reset();T.Reset();
    if(!FMath::IsFinite(Run)||!FMath::IsFinite(Rise)||!FMath::IsFinite(Width)||!FMath::IsFinite(TableLength)||
        Run<=0||Rise<=0||Width<=0||TableLength<0||Run>20000||Rise>10000||Width>10000||TableLength>20000)return false;
    TArray<FVector2D> P;
    if(TableLength==0)P={{0,0},{Run,0},{Run,Rise}};
    else P={{0,0},{2*Run+TableLength,0},{Run+TableLength,Rise},{Run,Rise}};
    const int32 N=P.Num();
    for(double Y:{-Width*.5,Width*.5})for(const auto& Q:P)V.Add(FVector(Q.X,Y,Q.Y));
    // P is counterclockwise in XZ: -Y side has the forward winding.
    for(int32 I=1;I<N-1;++I){T.Append({0,I,I+1});T.Append({N,N+I+1,N+I});}
    for(int32 I=0;I<N;++I){const int32 J=(I+1)%N;T.Append({I,N+I,N+J,I,N+J,J});}
    return true;
}
