#include "BodyContactWrite.h"
#include "ComponentContactWrite.h"
#include "IAmSpeed/Netcode/client-prediction/PrivateNativeStorageMap.h"
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadNativeObjects.h"
#include "IAmSpeed/SubBodies/SSubBody.h"
namespace Speed::NativeKinematics
{
void FBodyWrite::Hit(USSubBody& Body,const SHitResult& Hit)
{
    Body.CurrentHitStorage() = Hit;
}
void FBodyWrite::FutureHit(USSubBody& Body,const SHitResult& Hit)
{
    Body.FutureHitStorage() = Hit;
}

// Actual owner operation overloads. Legacy/default/custom entries above remain unchanged.
void FBodyWrite::Hit(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USSubBody& Body,const SHitResult& Hit)
{
    Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<Speed::Netcode::Prediction::FPrivateRoadSubBodyValues>(Operation,&Body).CurrentHit = Hit;
}
void FBodyWrite::FutureHit(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USSubBody& Body,const SHitResult& Hit)
{
    Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<Speed::Netcode::Prediction::FPrivateRoadSubBodyValues>(Operation,&Body).FutureHit = Hit;
}

void FBodyWrite::Impulse(USolidSubBody& Body,const FVector& Impulse,const FVector& Point)
{Body.ApplyImpulse(Impulse,Point);}
void FBodyWrite::Impulse(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USolidSubBody& Body,const FVector& Impulse,const FVector& Point)
{
    using namespace Speed::Netcode::Prediction;
    const auto& Values=FPrivateNativeStorageAccess::Part<const FPrivateRoadSubBodyValues>(Operation,&Body);
    (void)FPrivateNativeStorageAccess::Part<const FPrivateRoadSolidValues>(Operation,&Body);
    if(Values.ParentComponent)FComponentWrite::Impulse(Operation,Values.ParentComponent,Impulse,Point,&Body);
}

void FBodyWrite::FakePhysics(USolidSubBody* Body,USolidSubBody* Other,const SHitResult& Hit,const float& DeltaTime,const FFakePhysicsImpactContext* ImpactContext)
{Body->ApplyFakePhysicsOn(*Other,Hit,DeltaTime,ImpactContext);}
void FBodyWrite::FakePhysics(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USolidSubBody* Body,USolidSubBody* Other,const SHitResult& Hit,const float& DeltaTime,const FFakePhysicsImpactContext* ImpactContext)
{
    using namespace Speed::Netcode::Prediction;
    // Both raw adjusted subjects are checked before either is dereferenced
    // and before the private virtual call.
    (void)FPrivateNativeStorageAccess::Part<const FPrivateRoadSubBodyValues>(Operation,Body);
    (void)FPrivateNativeStorageAccess::Part<const FPrivateRoadSolidValues>(Operation,Body);
    (void)FPrivateNativeStorageAccess::Part<const FPrivateRoadSubBodyValues>(Operation,Other);
    (void)FPrivateNativeStorageAccess::Part<const FPrivateRoadSolidValues>(Operation,Other);
    Body->ApplyNativeFakePhysicsOn(Operation,*Other,Hit,DeltaTime,ImpactContext);
}

}
