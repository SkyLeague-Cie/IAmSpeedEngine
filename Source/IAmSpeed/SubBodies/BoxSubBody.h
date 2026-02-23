// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "SolidSubBody.h"
#include "UObject/ObjectMacros.h"
#include "BoxSubBody.generated.h"

class USphereSubBody;
class USWheelSubBody;

DECLARE_LOG_CATEGORY_EXTERN(BoxSubBodyLog, Log, All);

/**
 * UBoxSubBody : A sub-body representing a box. It provides functionality for collision detection and response specific for each shape.
 */
UCLASS(ClassGroup = "Collision", hidecategories = (Object, LOD, Lighting, TextureStreaming), editinlinenew, meta = (DisplayName = "Box Collision", BlueprintSpawnableComponent))
class IAMSPEED_API UBoxSubBody : public USolidSubBody
{
    GENERATED_UCLASS_BODY()
	
public:
    virtual void Initialize(ISpeedComponent* InParentComponent) override;

    // --- USubBody overrides ---
    bool SweepTOI(const float& RemainingDelta, const float& TimePassed, float& OutTOI) override;
    void ResolveCurrentHitPrv(const float& delta, const float& TimePassed, const float& SimTime) override;
    SKinematic GetKinematicsFromOwner(const unsigned int& NumFrame) const;
    SKinematic GetKinematicsFromOwnerKS(const SKinematic& CarKinematicState) const;
    SSBox MakeBox(const unsigned int& NumFrame, const float& TimePassed) const;
    SSBox MakeBoxFromKS(const SKinematic& CarKinematicState) const;
    void ResetForFrame(const float& Delta) override;
    void AcceptHit() override;

    static void GetBoxVertices(const FVector& Center, const FQuat& Rot, const FVector& Ext, TArray<FVector>& OutVerts);
    static FVector ComputeBoxSupportPointWS(const FVector& Center, const FQuat& Rot, const FVector& Ext, const FVector& N);
    const TArray<FVector>& GetGroundContacts() const;
    bool IsConcaveGroundContact() const;
    FVector GetGroundPlaneNormal() const;
    float GetGroundPlaneD() const;
    bool GetLatchedEdgeWS(FVector& OutA, FVector& OutB) const;
    bool IsPointSupportedByPersistentContact(const FVector& P, float Margin = 2.f) const;

    bool HasPersistentGroundContact() const;
    void UpdatePersistentGroundContact(const float& delta);
    bool HasPersistentEdgeSupport() const;
    bool IsInEdgeBalance() const;
    int GetBestSupportingFace() const;
    int GetLowestFaceIndex() const;
    FVector GetFaceNormalWS(const int& FaceIndex, const FQuat& HitboxRot) const;
    TArray<FVector> GetFaceVerticesWS(const int& FaceIndex) const;
    bool ComputeEdgeLockAxis(const FVector& N, const TArray<FVector>& SupportPts, FVector& OutLockAxis) const;
    void ApplyImpulse(const FVector& LinearImpulse, const FVector& WorldPoint) override;
    FVector GetCOM() const;
	FVector GetBoxExtent() const;
    FMatrix ComputeWorldInvInertiaTensor() const override;

    FPrimitiveSceneProxy* CreateSceneProxy() override;
    FBoxSphereBounds CalcBounds(const FTransform& LocalToWorld) const override;
    void UpdateBodySetup();
protected:
    virtual FCollisionShape GetCollisionShape(float Inflation = 0.0f) const override;
    FMatrix InitInvInertiaTensor() const override;
#if WITH_EDITOR
    FEngineShowFlags GetShowFlags() const { return ShowFlags; }
    void SetShowFlags(const FEngineShowFlags& InShowFlags);
#endif // WITH_EDITOR
private:
    void ResolveHitVsGround(const float& delta, const float& TimePassed);
    void ResolveHitVsSphere(USphereSubBody& OtherSphere, const float& delta, const float& TimePassed);
    // void ResolveHitVsWheel(USWheelSubBody& OtherWheel, const float& delta, const float& TimePassed);
    void ResolveHitVsBox(UBoxSubBody& OtherBox, const float& delta, const float& TimePassed);

    // --- Internal helpers ---
    bool SweepVsGround(SHitResult& OutHit, const float& Delta, const float& TimePassed, float& OutTOI);
    bool SweepVsSpheres(SHitResult& OutHit, const float& Delta, const float& TimePassed, float& OutTOI);
    bool SweepVsWheels(SHitResult& OutHit, const float& Delta, const float& TimePassed, float& OutTOI);
    bool SweepVsBoxes(SHitResult& OutHit, const float& Delta, const float& TimePassed, float& OutTOI);

    void BuildContactPoints(const FVector& Nworld, const FVector& CenterWS, const FQuat& BoxRotWS, const FVector& Ext, TArray<FVector>& OutPointsWS) const;
    FVector BuildCompositeGroundNormal() const;
    void ResolveWallOrGutter(const float& Dt, const SHitResult& Hit);
    void ResolveDirectGroundSupport(const float& Dt, const SHitResult& Hit);
    void ApplyPersistentSupportConstraint(const float& Dt);
    bool SolveEdgeSupportConstraint(const FVector& SupportN, const TArray<FVector>& SupportPts, const float Dt, const bool bDoFriction /*= false*/,
        const float Mu /*= 0.0f*/);

    static FORCEINLINE FVector SafeNormal(const FVector& v)
    {
        return v.GetSafeNormal();
    }

    static FORCEINLINE float EffectiveMassAlongDir(
        float InvMass,
        const FMatrix& InvI,
        const FVector& r,          // P - COM
        const FVector& dir)        // N or T (unit)
    {
        // denom = InvMass + ( (r x dir)^T * I^-1 * (r x dir) ) projected back onto dir
        // Standard rigid body effective mass along a direction for point impulse.
        const FVector rxDir = FVector::CrossProduct(r, dir);
        const FVector Iinv_rxDir = InvI.TransformVector(rxDir);
        // scalar = (I^-1 (r x dir)) dot (r x dir)
        const float ang = FVector::DotProduct(Iinv_rxDir, rxDir);
        return InvMass + ang;
    }

    // “Contact set” helper: build manifold from support plane normal.
    static FORCEINLINE void BuildSupportManifoldFromNormal(
        const FVector& N,
        const FVector& CenterWS,
        const FQuat& RotWS,
        const FVector& Ext,
        TArray<FVector>& OutPts,
        float ContactEpsilonCm = 0.5f)
    {
        TArray<FVector> Verts;
        GetBoxVertices(CenterWS, RotWS, Ext, Verts);

        float minProj = FLT_MAX;
        for (const FVector& V : Verts)
        {
            minProj = FMath::Min(minProj, FVector::DotProduct(V, N));
        }

        OutPts.Reset();
        for (const FVector& V : Verts)
        {
            const float d = FVector::DotProduct(V, N);
            if (d <= minProj + ContactEpsilonCm)
            {
                OutPts.Add(V);
            }
        }

        if (OutPts.Num() == 0)
        {
            OutPts.Add(CenterWS);
        }
    }

    static FORCEINLINE FVector FaceLocalNormal(int32 FaceIndex)
    {
        switch (FaceIndex)
        {
        case 0: return FVector(+1, 0, 0); // +X
        case 1: return FVector(-1, 0, 0); // -X
        case 2: return FVector(0, +1, 0); // +Y
        case 3: return FVector(0, -1, 0); // -Y
        case 4: return FVector(0, 0, +1); // +Z
        case 5: return FVector(0, 0, -1); // -Z
        default: return FVector::ZeroVector;
        }
    }

    // Full 4 vertices of the face in LOCAL (centered box), any order
    static FORCEINLINE void FaceLocalVertices(int32 FaceIndex, const FVector& Ext, FVector Out[4])
    {
        const float ex = Ext.X;
        const float ey = Ext.Y;
        const float ez = Ext.Z;

        switch (FaceIndex)
        {
        case 0: // +X : x = +ex, y/z varient
            Out[0] = FVector(+ex, -ey, -ez);
            Out[1] = FVector(+ex, -ey, +ez);
            Out[2] = FVector(+ex, +ey, +ez);
            Out[3] = FVector(+ex, +ey, -ez);
            break;

        case 1: // -X
            Out[0] = FVector(-ex, -ey, -ez);
            Out[1] = FVector(-ex, +ey, -ez);
            Out[2] = FVector(-ex, +ey, +ez);
            Out[3] = FVector(-ex, -ey, +ez);
            break;

        case 2: // +Y
            Out[0] = FVector(-ex, +ey, -ez);
            Out[1] = FVector(-ex, +ey, +ez);
            Out[2] = FVector(+ex, +ey, +ez);
            Out[3] = FVector(+ex, +ey, -ez);
            break;

        case 3: // -Y
            Out[0] = FVector(-ex, -ey, -ez);
            Out[1] = FVector(+ex, -ey, -ez);
            Out[2] = FVector(+ex, -ey, +ez);
            Out[3] = FVector(-ex, -ey, +ez);
            break;

        case 4: // +Z
            Out[0] = FVector(-ex, -ey, +ez);
            Out[1] = FVector(+ex, -ey, +ez);
            Out[2] = FVector(+ex, +ey, +ez);
            Out[3] = FVector(-ex, +ey, +ez);
            break;

        case 5: // -Z
            Out[0] = FVector(-ex, -ey, -ez);
            Out[1] = FVector(-ex, +ey, -ez);
            Out[2] = FVector(+ex, +ey, -ez);
            Out[3] = FVector(+ex, -ey, -ez);
            break;

        default:
            Out[0] = Out[1] = Out[2] = Out[3] = FVector::ZeroVector;
            break;
        }
    }

    static FORCEINLINE FVector SafeDir(const FVector& V)
    {
        const float s2 = V.SizeSquared();
        return (s2 > KINDA_SMALL_NUMBER) ? (V / FMath::Sqrt(s2)) : FVector::ZeroVector;
    }

    static FORCEINLINE float PlaneSignedDist(const FVector& P, const FVector& PlaneN, const FVector& PlanePointWS)
    {
        return FVector::DotProduct(P - PlanePointWS, PlaneN);
    }

public:
    static FORCEINLINE bool IsVertexSupportedByGroundPlane(
        const FVector& V,
        const FVector& GroundN,
        float GroundD)
    {
        // Signed distance to plane
        const float dist = FVector::DotProduct(V, GroundN) - GroundD;

        // Tolerances (IMPORTANT)
        constexpr float SupportTol = 1.5f;   // cm above plane -> still supported
        constexpr float PenTol = -0.5f;  // cm inside plane -> definitely supported

        return (dist <= SupportTol);
    }


protected:
    UPROPERTY(EditAnywhere, BlueprintReadOnly, export, Category = Shape)
    FVector BoxExtent = FVector::ZeroVector;
#if WITH_EDITOR
    /** List of all show flags this box component visualizer should respect. */
    FEngineShowFlags ShowFlags;
#endif // WITH_EDITOR
private:
    SHitResult GroundHit;
    SHitResult SphereHit;
    SHitResult WheelHit;
    SHitResult BoxHit;

    TArray<FVector> CurrentGroundContactsWS;
    TArray<FVector> CurrentGroundNormalsWS; // per contact point normal
    TArray<FVector> PrevGroundContactsLS; // local hitbox space (COM frame)
    TArray<FVector> LatchedEdgeContactsLS; // size = 2
    bool bEdgeSupportLatched = false;
    int32 EdgeSupportLatchFrame = -1;
    FVector CompositeGroundNormal; // average normal from all ground contacts this frame
    TMap<TWeakObjectPtr<UPrimitiveComponent>, uint32> HitCountThisFrame;
    uint32 MaxHitsPerComponentPerFrame = 4; // face = 4 corners
    TArray<float> PrevLambdaN;

    bool bGroundContactStable = false; // true if every contact point were stable last frame
    bool bHasGroundContact = false; // true if we have at least one ground contact this frame
    float StableTime = 0.f;

    bool bGroundHitFromSweep = false; // true if ground hit was generated from sweep this frame
    bool bGroundPlaneValid = false;
    FVector GroundPlanePointWS = FVector::ZeroVector;; // a point on the ground plane
    FVector GroundPlaneN = FVector::UpVector;
    float GroundPlaneD = 0.f; // plane equation: dot(X, N) - D = 0
    TWeakObjectPtr<UPrimitiveComponent> GroundComp;
    FVector PrevGroundNormal = FVector::UpVector; // normal from previous frame ground plane
    bool bHadGroundContactPrevFrame = false;
    FVector PrevContactNormal = FVector::UpVector; // average contact normal from previous frame
    bool bHadContactPrevFrame = false;
    float MinSlopCm = 0.05f; // Minimum allowed penetration depth in cm, used as slop in the solver to prevent jittering when resolving penetrations
};
