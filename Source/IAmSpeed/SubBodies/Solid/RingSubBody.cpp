#include "RingSubBody.h"

URingSubBody::URingSubBody(const FObjectInitializer& ObjectInitializer):Super(ObjectInitializer)
{
    SetMobility(EComponentMobility::Static);
    SetCollisionObjectType(ECC_WorldStatic);
    // Chaos collision is not a substitute for the canonical torus provider.
    SetCollisionEnabled(ECollisionEnabled::NoCollision);
}
bool URingSubBody::BuildStaticTorus(uint64 StableSourceId,Speed::Analytic::FTorusSurface& Out,FString& Error) const
{
    Out={};Error.Reset();
    const FTransform WorldPose=GetComponentTransform(); const FVector Scale=WorldPose.GetScale3D();
    if (Mobility!=EComponentMobility::Static || WorldPose.ContainsNaN() || Scale.GetMin()<=0. ||
        !FMath::IsNearlyEqual(Scale.X,Scale.Y,1.e-9) || !FMath::IsNearlyEqual(Scale.X,Scale.Z,1.e-9))
    { Error=TEXT("RingSubBody supports static poses with finite positive uniform scale."); return false; }
    Out.SourceId=Out.SurfaceId=Out.FeatureId=Out.PrimitiveId=StableSourceId;
    Out.Pose=FTransform(WorldPose.GetRotation(),WorldPose.GetTranslation());
    Out.Geometry={MajorRadius*Scale.X,TubeRadius*Scale.X};
    return Out.Finalize(&Error);
}
FBoxSphereBounds URingSubBody::CalcBounds(const FTransform& LocalToWorld) const
{
    const double R=FMath::Max(0.,MajorRadius),r=FMath::Max(0.,TubeRadius);
    return FBoxSphereBounds(FVector::ZeroVector,FVector(r,R+r,R+r),R+r).TransformBy(LocalToWorld);
}
