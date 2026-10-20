#pragma once
#include "SolidSubBody.h"
#include "CylinderGeometry.h"
#include "CylinderSubBody.generated.h"
/** Finite circular solid, local Z axis. The current runtime admission supports
 * prescribed obstacles; Box/Sphere/Wheel contacts use exact analytic geometry.
 * No capsule, polygon mesh or Chaos body substitutes for the cylinder. */
UCLASS(ClassGroup="Collision",editinlinenew,meta=(BlueprintSpawnableComponent))
class IAMSPEED_API UCylinderSubBody : public USolidSubBody
{
    GENERATED_BODY()
public:
    UCylinderSubBody(const FObjectInitializer& Initializer);
    bool Configure(double Radius,double HalfHeight);
    Speed::Geometry::FCylinderGeometry Geometry() const { return {CylinderRadius,CylinderHalfHeight}; }
    void Initialize(ISpeedComponent* Owner) override;
    bool SweepTOI(const float& Remaining,float& TOI) override;
    void AcceptHit() override;
    FPrimitiveSceneProxy* CreateSceneProxy() override { return nullptr; }
    FBoxSphereBounds CalcBounds(const FTransform& Pose) const override;
    // Unreal has no exact cylinder FCollisionShape. No fallback query is legal.
    FCollisionShape GetCollisionShape(float Inflation=0.f) const override { return FCollisionShape(); }
    FMatrix ComputeWorldInvInertiaTensor() const override;
#if !UE_BUILD_SHIPPING
    void AppendEpisodeStateForTesting(TArray<uint8>& Out) const override
    {USolidSubBody::AppendEpisodeStateForTesting(Out);using B=Speed::FEpisodeStateBytesForTesting;B::Add(Out,CylinderRadius);B::Add(Out,CylinderHalfHeight);}
#endif
protected:
    void ResolveCurrentHitPrv(const float& Delta,const float& SimTime) override;
    FMatrix InitInvInertiaTensor() const override;
private:
    UPROPERTY(EditAnywhere,Category="Shape") double CylinderRadius=100.;
    UPROPERTY(EditAnywhere,Category="Shape") double CylinderHalfHeight=100.;
};
