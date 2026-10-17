#pragma once
#include "IAmSpeed/Components/SpeedMovementComponent.h"
#include "IAmSpeed/SubBodies/Configs/SubBodyConfig.h"
#include "PrescribedMotion.h"
#include <memory>
#include "SpeedPrescribedComponent.generated.h"
/** Read and written only by the canonical worker. No game-thread actor tick. */
namespace Speed::Motion { struct FActivation { int32 Stage=INDEX_NONE; bool bInside=false; uint64 Episode=0; }; }
/** Infinite-inertia, explicitly prescribed solid; existing movers retain their
 * ordinary force/impulse paths. Only local standalone course use is admitted. */
UCLASS(ClassGroup="Physics")
class IAMSPEED_API USpeedPrescribedComponent : public USpeedMovementComponent
{
    GENERATED_BODY()
#if WITH_DEV_AUTOMATION_TESTS
    friend class FPrescribedPhysicalComponentTest;
#endif
public:
    USpeedPrescribedComponent(const FObjectInitializer& Initializer);
    bool Configure(USolidSubBody* Body,const SubBodyConfig& Config,
        const Speed::Motion::FPrescribedMotionTrack& Motion,const FTransform& Anchor,int32 Stage,
        std::shared_ptr<const Speed::Motion::FActivation> Activation);
    void BeginPlay() override;
    void OnCreatePhysicsState() override;
    bool ValidateSimulationBindings(FString& Reason) const override;
    bool IsPrescribedMotion() const override { return true; }
    void PrepareCanonicalFrame(const FCanonicalFrameContext& Context) override;
    unsigned int NumFrame() const override { return CurrentFrame; }
    float GetPhysMass() const override { return 0.f; }
    FMatrix ComputeWorldInvInertiaTensor() const override { return FMatrix(ForceInitToZero); }
    FMatrix ComputeWorldInvInertiaTensorOfSubBody(const USSubBody&) const override { return FMatrix(ForceInitToZero); }
    FVector GetNominalGravityAcceleration() const override { return FVector::ZeroVector; }
    SubBodyConfig GetSubBodyConfig(const USSubBody&) const override { return ShapeConfig; }
    const SKinematic& GetKinematicStateForFrame(const unsigned int&) const override { return GetKinematicState(); }
    void SetKinematicState(const SKinematic& State) override;
    void PostPhysicsUpdatePrv(const float&) override {}
    void AppendSimulationSnapshot(TArray<uint8>& Payload) const override;
    bool CanRestoreSimulationSnapshot(TConstArrayView<uint8> Payload) const override;
protected:
    void PostIntegrateKinematics(const float& Delta) override;
    void RestoreSimulationSnapshotPayload(TConstArrayView<uint8> Payload) override;
private:
    void InstallPose(double ActiveTicks,bool bMoving);
    Speed::Motion::FPrescribedMotionTrack Track;
    Speed::Motion::FStageMotionClock Clock;
    std::shared_ptr<const Speed::Motion::FActivation> Gate;
    SubBodyConfig ShapeConfig;
    FTransform AnchorPose=FTransform::Identity;
    uint64 FrameStartTicks=0,LastEpisode=0;
    double FrameElapsed=0.,FrameDelta=1./300.;
    unsigned int CurrentFrame=0;
    int32 StageIndex=INDEX_NONE;
    bool bConfigured=false,bAdmitted=false,bMovingThisFrame=false;
};
