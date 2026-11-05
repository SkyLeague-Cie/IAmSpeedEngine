// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "SolidSubBody.h"
#include "ShowFlags.h"
#include "IAmSpeed/SubBodies/Common/IBoxSweeper.h"
#include "IAmSpeed/SubBodies/Common/ISolidSweeper.h"
#include "UObject/ObjectMacros.h"
#include "BoxSubBody.generated.h"

class USpeedWorldSubsystem;
class USphereSubBody;
class USWheelSubBody;
namespace Speed { class IStaticCollisionWorld; struct FBoxRestingSupport; }
namespace Speed::Analytic { struct FWorldQuery; }

DECLARE_LOG_CATEGORY_EXTERN(BoxSubBodyLog, Log, All);

/**
 * UBoxSubBody : A sub-body representing a box. It provides functionality for collision detection and response specific for each shape.
 */
UCLASS(ClassGroup = "Collision", hidecategories = (Object, LOD, Lighting, TextureStreaming), editinlinenew, meta = (DisplayName = "Box Collision", BlueprintSpawnableComponent))
class IAMSPEED_API UBoxSubBody : public USolidSubBody, public IBoxSweeper, public ISolidSweeper
{
    GENERATED_UCLASS_BODY()
#if WITH_DEV_AUTOMATION_TESTS
    friend class FIAmSpeedSweeperCandidatesTest;
#endif
	
public:
    const USolidSubBody* GetSolidSweepBody() const override { return this; }
#if !UE_BUILD_SHIPPING
    void AppendEpisodeStateForTesting(TArray<uint8>& Out) const override
    {
        USolidSubBody::AppendEpisodeStateForTesting(Out);
        AppendBoxEpisodeTailForTesting(Out);
    }
#endif
    virtual void Initialize(ISpeedComponent* InParentComponent) override;

    // --- USubBody overrides ---
    bool SweepTOI(const float& RemainingDelta, float& OutTOI) override;
    void ResolveCurrentHitPrv(const float& delta, const float& SimTime) override;
    const Speed::FKinematicState& GetKinematicState() const { return USolidSubBody::GetKinematicState(); }
    SKinematic GetKinematicsFromOwner(const unsigned int& NumFrame) const;
    SKinematic GetKinematicsFromOwnerKS(const SKinematic& CarKinematicState) const;
    SSBox MakeBox() const;
    // Construction-time physical policy; not driven by a test ID or diagnostic.
    void SetProductionNonpenetratingSphereContacts(bool bEnabled) { bProductionNonpenetratingSphereContactsBoxDomainStorage() = bEnabled; }
    bool UsesProductionNonpenetratingSphereContacts() const { return bProductionNonpenetratingSphereContactsBoxDomainStorage(); }

    SSBox MakeBoxFromKS(const SKinematic& CarKinematicState) const;
    void ResetForFrame(const float& Delta) override;
	/** Publishes independently certified passive support, without an impact callback or pose change. */
	void PostPhysicsUpdate() override;
	FVector GetStaticRestingReaction(const Speed::IStaticCollisionWorld& World,
		double* StopAfterSeconds = nullptr) const override;
	double GetStaticSupportStopTimeCandidate() const override;
	bool TryGetStaticContactAcceleration(FVector& Linear, FVector& Angular) const override;
	bool CanResolveRepeatedContact(const SHitResult& Hit) const override;
	bool CanInitializeRepeatedContact(const SHitResult& Hit) const override;
	bool RequiresUnquantizedContactPose() const override;
	bool CanApplyQuantizedPose(const Speed::IStaticCollisionWorld& World, const SKinematic& OwnerPose) const override;
	/** Evaluates the current box pose/filters against a supplied load, without changing any state. */
	bool EvaluateStaticRestingSupport(const Speed::IStaticCollisionWorld& World,
		const FVector& ExternalAcceleration, Speed::FBoxRestingSupport& OutSupport) const;
	bool HasExactStaticRestingSupport(const Speed::IStaticCollisionWorld& World, const FVector& Load) const override;
	bool ProjectEstablishedStaticContact(const float& Delta) override;
	bool RequiresEstablishedStaticContactTransport() const override;
    void AcceptHit() override;

    static void GetBoxVertices(const FVector& Center, const FQuat& Rot, const FVector& Ext, TArray<FVector>& OutVerts);
    static FVector ComputeBoxSupportPointWS(const FVector& Center, const FQuat& Rot, const FVector& Ext, const FVector& N);
    const TArray<FVector>& GetGroundContacts() const;
    const TArray<FVector>& GetPhysicsTickGroundContacts() const;
    int32 GetLastResolvedGroundHitFrame() const { return LastResolvedGroundHitFrameBoxDomainStorage(); }
    /** Rechecks a resolved exact planar provider at the current pose, without publishing contact. */
    bool HasCurrentExactPlanarContact() const;
    bool IsConcaveGroundContact() const;
    FVector GetGroundPlaneNormal() const;
    float GetGroundPlaneD() const;
    bool GetLatchedEdgeWS(FVector& OutA, FVector& OutB) const;
    bool IsPointSupportedByPersistentContact(const FVector& P, float Margin = 2.f) const;

    bool HasPersistentGroundContact() const;
    bool HasPhysicsTickGroundContact() const;
    bool HasRoofSurfaceTraversalSupport() const;
    void UpdatePersistentGroundContact(const float& delta, bool bDirectSupportSolved = false);
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
    virtual bool UsesNonclosingBoxBumpResponse() const { return false; }
	float GetSphereSeparation(const FVector& SphereCenter, float SphereRadius) const;
	bool GatherStaticPenetrationHits(TArray<FHitResult>& OutHits) const;
	void SetBoxExtent(const FVector& InBoxExtent);
    FMatrix ComputeChassisLocalInvInertiaTensor() const;
    FMatrix ComputeWorldInvInertiaTensor() const override;

    FPrimitiveSceneProxy* CreateSceneProxy() override;
    FBoxSphereBounds CalcBounds(const FTransform& LocalToWorld) const override;
    void UpdateBodySetup();
protected:
    // Legacy default preserves the existing lookup point/order. An original
    // epoch subtype may resolve its actual frame-bound IAS source participant.
    virtual const USpeedWorldSubsystem* ResolveEpochBridgeForSweep(UWorld* World,const Speed::IStaticCollisionWorld* StaticWorld) const;
protected:
    virtual FVector& BoxExtentBoxDomainStorage() {return BoxExtent;}
    virtual FVector const& BoxExtentBoxDomainStorage() const {return BoxExtent;}
    virtual bool& bProductionNonpenetratingSphereContactsBoxDomainStorage() {return bProductionNonpenetratingSphereContacts;}
    virtual bool const& bProductionNonpenetratingSphereContactsBoxDomainStorage() const {return bProductionNonpenetratingSphereContacts;}
    virtual TArray<FVector>& CurrentGroundContactsWSBoxDomainStorage() {return CurrentGroundContactsWS;}
    virtual TArray<FVector> const& CurrentGroundContactsWSBoxDomainStorage() const {return CurrentGroundContactsWS;}
    virtual TArray<FVector>& PreviousFrameGroundContactsWSBoxDomainStorage() {return PreviousFrameGroundContactsWS;}
    virtual TArray<FVector> const& PreviousFrameGroundContactsWSBoxDomainStorage() const {return PreviousFrameGroundContactsWS;}
    virtual TArray<FVector>& CurrentGroundNormalsWSBoxDomainStorage() {return CurrentGroundNormalsWS;}
    virtual TArray<FVector> const& CurrentGroundNormalsWSBoxDomainStorage() const {return CurrentGroundNormalsWS;}
    virtual TArray<FVector>& PrevGroundContactsLSBoxDomainStorage() {return PrevGroundContactsLS;}
    virtual TArray<FVector> const& PrevGroundContactsLSBoxDomainStorage() const {return PrevGroundContactsLS;}
    virtual TArray<FVector>& LatchedEdgeContactsLSBoxDomainStorage() {return LatchedEdgeContactsLS;}
    virtual TArray<FVector> const& LatchedEdgeContactsLSBoxDomainStorage() const {return LatchedEdgeContactsLS;}
    virtual bool& bEdgeSupportLatchedBoxDomainStorage() {return bEdgeSupportLatched;}
    virtual bool const& bEdgeSupportLatchedBoxDomainStorage() const {return bEdgeSupportLatched;}
    virtual int32& EdgeSupportLatchFrameBoxDomainStorage() {return EdgeSupportLatchFrame;}
    virtual int32 const& EdgeSupportLatchFrameBoxDomainStorage() const {return EdgeSupportLatchFrame;}
    virtual bool& bRoofSurfaceTraversalLatchedBoxDomainStorage() {return bRoofSurfaceTraversalLatched;}
    virtual bool const& bRoofSurfaceTraversalLatchedBoxDomainStorage() const {return bRoofSurfaceTraversalLatched;}
    virtual int32& LastRoofSurfaceContactFrameBoxDomainStorage() {return LastRoofSurfaceContactFrame;}
    virtual int32 const& LastRoofSurfaceContactFrameBoxDomainStorage() const {return LastRoofSurfaceContactFrame;}
    virtual TWeakObjectPtr<UPrimitiveComponent>& RoofSurfaceCompBoxDomainStorage() {return RoofSurfaceComp;}
    virtual TWeakObjectPtr<UPrimitiveComponent> const& RoofSurfaceCompBoxDomainStorage() const {return RoofSurfaceComp;}
    virtual FVector& CompositeGroundNormalBoxDomainStorage() {return CompositeGroundNormal;}
    virtual FVector const& CompositeGroundNormalBoxDomainStorage() const {return CompositeGroundNormal;}
    virtual TMap<TWeakObjectPtr<UPrimitiveComponent>, uint32>& HitCountThisFrameBoxDomainStorage() {return HitCountThisFrame;}
    virtual TMap<TWeakObjectPtr<UPrimitiveComponent>, uint32> const& HitCountThisFrameBoxDomainStorage() const {return HitCountThisFrame;}
    virtual uint32& MaxHitsPerComponentPerFrameBoxDomainStorage() {return MaxHitsPerComponentPerFrame;}
    virtual uint32 const& MaxHitsPerComponentPerFrameBoxDomainStorage() const {return MaxHitsPerComponentPerFrame;}
    virtual TArray<float>& PrevLambdaNBoxDomainStorage() {return PrevLambdaN;}
    virtual TArray<float> const& PrevLambdaNBoxDomainStorage() const {return PrevLambdaN;}
    virtual bool& bGroundContactStableBoxDomainStorage() {return bGroundContactStable;}
    virtual bool const& bGroundContactStableBoxDomainStorage() const {return bGroundContactStable;}
    virtual int32& LastResolvedGroundHitFrameBoxDomainStorage() {return LastResolvedGroundHitFrame;}
    virtual int32 const& LastResolvedGroundHitFrameBoxDomainStorage() const {return LastResolvedGroundHitFrame;}
    virtual bool& bHasGroundContactBoxDomainStorage() {return bHasGroundContact;}
    virtual bool const& bHasGroundContactBoxDomainStorage() const {return bHasGroundContact;}
    virtual bool& bFreshEdgeRecoverCandidateBoxDomainStorage() {return bFreshEdgeRecoverCandidate;}
    virtual bool const& bFreshEdgeRecoverCandidateBoxDomainStorage() const {return bFreshEdgeRecoverCandidate;}
    virtual float& StableTimeBoxDomainStorage() {return StableTime;}
    virtual float const& StableTimeBoxDomainStorage() const {return StableTime;}
    virtual bool& bGroundHitFromSweepBoxDomainStorage() {return bGroundHitFromSweep;}
    virtual bool const& bGroundHitFromSweepBoxDomainStorage() const {return bGroundHitFromSweep;}
    virtual bool& bGroundPlaneValidBoxDomainStorage() {return bGroundPlaneValid;}
    virtual bool const& bGroundPlaneValidBoxDomainStorage() const {return bGroundPlaneValid;}
    virtual FVector& GroundPlanePointWSBoxDomainStorage() {return GroundPlanePointWS;}
    virtual FVector const& GroundPlanePointWSBoxDomainStorage() const {return GroundPlanePointWS;}
    virtual FVector& GroundPlaneNBoxDomainStorage() {return GroundPlaneN;}
    virtual FVector const& GroundPlaneNBoxDomainStorage() const {return GroundPlaneN;}
    virtual float& GroundPlaneDBoxDomainStorage() {return GroundPlaneD;}
    virtual float const& GroundPlaneDBoxDomainStorage() const {return GroundPlaneD;}
    virtual TWeakObjectPtr<UPrimitiveComponent>& GroundCompBoxDomainStorage() {return GroundComp;}
    virtual TWeakObjectPtr<UPrimitiveComponent> const& GroundCompBoxDomainStorage() const {return GroundComp;}
    virtual uint64& EstablishedSupportSourceIdBoxDomainStorage() {return EstablishedSupportSourceId;}
    virtual uint64 const& EstablishedSupportSourceIdBoxDomainStorage() const {return EstablishedSupportSourceId;}
    virtual uint64& EstablishedSupportSurfaceIdBoxDomainStorage() {return EstablishedSupportSurfaceId;}
    virtual uint64 const& EstablishedSupportSurfaceIdBoxDomainStorage() const {return EstablishedSupportSurfaceId;}
    virtual int32& FreshSupportClampUntilFrameBoxDomainStorage() {return FreshSupportClampUntilFrame;}
    virtual int32 const& FreshSupportClampUntilFrameBoxDomainStorage() const {return FreshSupportClampUntilFrame;}
    virtual FVector& FreshSupportClampNBoxDomainStorage() {return FreshSupportClampN;}
    virtual FVector const& FreshSupportClampNBoxDomainStorage() const {return FreshSupportClampN;}
    virtual TWeakObjectPtr<UPrimitiveComponent>& FreshSupportClampCompBoxDomainStorage() {return FreshSupportClampComp;}
    virtual TWeakObjectPtr<UPrimitiveComponent> const& FreshSupportClampCompBoxDomainStorage() const {return FreshSupportClampComp;}
    virtual FVector& PrevGroundNormalBoxDomainStorage() {return PrevGroundNormal;}
    virtual FVector const& PrevGroundNormalBoxDomainStorage() const {return PrevGroundNormal;}
    virtual bool& bHadGroundContactPrevFrameBoxDomainStorage() {return bHadGroundContactPrevFrame;}
    virtual bool const& bHadGroundContactPrevFrameBoxDomainStorage() const {return bHadGroundContactPrevFrame;}
    virtual FVector& PrevContactNormalBoxDomainStorage() {return PrevContactNormal;}
    virtual FVector const& PrevContactNormalBoxDomainStorage() const {return PrevContactNormal;}
    virtual bool& bHadContactPrevFrameBoxDomainStorage() {return bHadContactPrevFrame;}
    virtual bool const& bHadContactPrevFrameBoxDomainStorage() const {return bHadContactPrevFrame;}
    virtual float& MinSlopCmBoxDomainStorage() {return MinSlopCm;}
    virtual float const& MinSlopCmBoxDomainStorage() const {return MinSlopCm;}
    virtual int32& LastWallOrGutterFrameBoxDomainStorage() {return LastWallOrGutterFrame;}
    virtual int32 const& LastWallOrGutterFrameBoxDomainStorage() const {return LastWallOrGutterFrame;}
    virtual FVector& LastWallOrGutterNBoxDomainStorage() {return LastWallOrGutterN;}
    virtual FVector const& LastWallOrGutterNBoxDomainStorage() const {return LastWallOrGutterN;}
    virtual float& LastWallOrGutterUpDotBoxDomainStorage() {return LastWallOrGutterUpDot;}
    virtual float const& LastWallOrGutterUpDotBoxDomainStorage() const {return LastWallOrGutterUpDot;}
    virtual SHitResult& LastResolvedSphereHitBoxDomainStorage() {return LastResolvedSphereHit;}
    virtual SHitResult const& LastResolvedSphereHitBoxDomainStorage() const {return LastResolvedSphereHit;}
    virtual int32& LastResolvedSphereHitFrameBoxDomainStorage() {return LastResolvedSphereHitFrame;}
    virtual int32 const& LastResolvedSphereHitFrameBoxDomainStorage() const {return LastResolvedSphereHitFrame;}
    bool bProductionNonpenetratingSphereContacts = false;
    virtual FCollisionShape GetCollisionShape(float Inflation = 0.0f) const override;
    FMatrix InitInvInertiaTensor() const override;
#if WITH_EDITOR
    FEngineShowFlags GetShowFlags() const { return ShowFlags; }
    void SetShowFlags(const FEngineShowFlags& InShowFlags);
#endif // WITH_EDITOR
    /*
    * Overriden methods
    */
    bool InternalSweep(UWorld* World, const FVector& Start,
        const FVector& End, SHitResult& OutHit, const float& delta) {
        return ISolidSweeper::InternalSweep(World, Start, End, OutHit, delta);
    }
    virtual ECollisionChannel GetCollisionChannel() const { return USolidSubBody::GetCollisionChannel(); }
    virtual const FCollisionResponseParams& GetResponseParams() const { return USolidSubBody::GetResponseParams(); }
    virtual FCollisionQueryParams BuildTraceParams() const { return USolidSubBody::BuildTraceParams(); }
	bool ShouldSkipSphereSweep(const USphereSubBody& Sphere) const override;
	bool ShouldSkipBoxSweep(const UBoxSubBody& Box) const override;
    bool ComponentHasBeenIgnored(const UPrimitiveComponent& OtherComp) const { return USolidSubBody::ComponentHasBeenIgnored(OtherComp); }
    const TArray<TWeakObjectPtr<UBoxSubBody>>& GetExternalBoxSubBodies() const { return USolidSubBody::GetExternalBoxSubBodies(); }
    const TArray<TWeakObjectPtr<USphereSubBody>>& GetExternalSphereSubBodies() const { return USolidSubBody::GetExternalSphereSubBodies(); }
    const TArray<TWeakObjectPtr<USWheelSubBody>>& GetExternalWheelSubBodies() const { return USolidSubBody::GetExternalWheelSubBodies(); }
private:
    class FLegacyResponseMassAccess;class FNativeResponseMassAccess;
    template<class TAccess> void ResetForFrameMassResponseBody(TAccess& Storage,const float& Delta);
    template<class TAccess> void ResolveCurrentHitPrvMassResponseBody(TAccess& Storage,const float& delta, const float& SimTime);
    template<class TAccess> void ResolveHitVsGroundMassResponseBody(TAccess& Storage,const float& Dt);
    template<class TAccess> void ResolveHitVsSphereMassResponseBody(TAccess& Storage,USphereSubBody& Sphere, const float& delta);
    template<class TAccess> void ResolveHitVsBoxMassResponseBody(TAccess& Storage,
    UBoxSubBody& OtherBox,
    const float& delta
);
    template<class TAccess> void ResolveWallOrGutterMassResponseBody(TAccess& Storage,const float& Dt, const SHitResult& Hit);
    template<class TAccess> void ResolveDirectGroundSupportMassResponseBody(TAccess& Storage,const float& Dt, const SHitResult& Hit);
    template<class TAccess> void ApplyPersistentSupportConstraintMassResponseBody(TAccess& Storage,const float& Dt);
    template<class TAccess> bool SolveEdgeSupportConstraintMassResponseBody(TAccess& Storage,
    const FVector& SupportN,
    const TArray<FVector>& SupportPts,
    const float Dt,
    const bool bDoFriction /*= false*/,
    const float Mu /*= 0.0f*/,
    const bool bAllowTraversalTorque /*= false*/);
    void ResolveNativeBoxCurrentHit(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const float& Delta,const float& SimTime);
    void ResetNativeBoxForFrame(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const float& Delta);
    friend class USpeedBranchBoxSubBody;
    friend class Speed::NativeKinematics::FBodyRead;
    class FLegacyStoredInverseAccess;class FNativeStoredInverseAccess;
    template<class TAccess> static FMatrix RotateStoredInverseInertiaBody(TAccess& Storage,const FQuat& BoxToFrame);
    FMatrix ComputeNativeChassisLocalInvInertiaTensor(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation) const;
    class FLegacyBoxHitAccess;class FNativeBoxHitAccess;class FLegacyRepeatedContactAccess;
    class FLegacyInitialInertiaAccess;class FNativeInitialInertiaAccess;
    class FLegacyBirthInitializeAccess;class FNativeBirthInitializeAccess;class FReferenceBirthInitializeAccess;
    template<class TAccess> void InitializeBirthBody(TAccess& Storage,ISpeedComponent* InParentComponent);
    void InitializeNativeBox(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,ISpeedComponent* Parent);
    void InitializeReferenceBox(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const Speed::Netcode::Prediction::FPrivateReferenceOwnerBirth& Proof,ISpeedComponent* Parent);
    template<class TAccess> FMatrix InitialInverseTensorBody(TAccess& Storage) const;
    FMatrix InitialNativeInverseTensor(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation) const;
    template<class TAccess> void AcceptBoxHitBody(TAccess& Storage);
    template<class TAccess> bool CanInitializeRepeatedContactBody(TAccess& Storage,const SHitResult& Hit) const;
    template<class TAccess> bool CanResolveRepeatedContactBody(TAccess& Storage,const SHitResult& Hit) const;
    void AcceptNativeBoxHit(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation);
#if WITH_DEV_AUTOMATION_TESTS
    friend class FIAmSpeedCurvedPersistentSupportTest;
#endif
    /** Expresses the box-local inverse inertia in a frame, preserving quaternion precision. */
    FMatrix RotateLocalInverseInertia(const FQuat& BoxToFrame) const;
    /** Actual box from an owner COM pose, with shared authoritative collision filters. */
    Speed::Analytic::FWorldQuery MakeSupportBoxQuery(const SKinematic& OwnerPose) const;
    /** Revalidates and publishes passive finite face support; never edits motion or emits an impact. */
    bool PublishExactStaticSupport();
    /** Shares query construction; geometry-only diagnostics do not depend on live velocity. */
    bool EvaluateStaticBoxSupport(const Speed::IStaticCollisionWorld& World,
        const FVector& ExternalAcceleration, Speed::FBoxRestingSupport& OutSupport,
        bool bIncludeMotion, double HorizonSeconds = 0) const;
    /** Resolves a certified simultaneous full-face normal impact as one resultant,
     * avoiding corner-order torque. Rejects spinning/sliding or partial support. */
    bool TryResolveExactFaceImpact();
    /** Resolves certified planar vertex/edge/face witnesses simultaneously;
     * geometry acquisition and position feasibility remain separate phases. */
    bool TryResolveExactPlanarImpact(bool bNotifyImpact = true);
    /** Refines a candidate plane against the owner-integrated vertices and
     * already established unilateral support; returns only a new contact. */
    bool RefineExactPlanarTOI(SHitResult& Hit, float Delta) const;
    /** Projects only the float-TOI roundoff of an arriving edge/face; never
     * settles a finite tilt or edits the live state before contact validation. */
    void ProjectPlanarEventRoundoff(SKinematic& State, const SHitResult& Hit) const;
    bool RefreshVariableNormalGroundSupport(const float Delta);
    /** A finite analytical provider owns this varying support; cached tangent
     * planes must not translate or attract the body to maintain contact. */
    bool UsesNativeAnalyticEstablishedContact() const;
	bool ProjectEstablishedStaticContactImpl(
		const float& Delta, bool bProjectVelocity);
    void ResolveHitVsGround(const float& delta);
    void ResolveHitVsSphere(USphereSubBody& OtherSphere, const float& delta);
    // void ResolveHitVsWheel(USWheelSubBody& OtherWheel, const float& delta);
    void ResolveHitVsBox(UBoxSubBody& OtherBox, const float& delta);

    // --- Internal helpers ---
    bool SweepVsGround(UWorld* World, SHitResult& OutHit, const float& Delta, float& OutTOI);
    bool SweepVsSpheres(UWorld* World, SHitResult& OutHit, const float& Delta, float& OutTOI);
    bool SweepVsWheels(UWorld* World, SHitResult& OutHit, const float& Delta, float& OutTOI);
    bool SweepVsBoxes(UWorld* World, SHitResult& OutHit, const float& Delta, float& OutTOI);

    void BuildContactPoints(const FVector& Nworld, const FVector& CenterWS, const FQuat& BoxRotWS, const FVector& Ext, TArray<FVector>& OutPointsWS) const;
    FVector BuildCompositeGroundNormal() const;
    bool ShouldPromoteWallHitToSupport(const SHitResult& Hit, const FVector& HitN, float HitUpDot, float Dt, SHitResult& OutSupportHit) const;
    bool TryBuildPersistentSphereContact(bool bHasSweepSphereHit, const SHitResult& SweepSphereHit, SHitResult& OutHit, float& OutTOI) const;
    void ResolveWallOrGutter(const float& Dt, const SHitResult& Hit);
    void ResolveDirectGroundSupport(const float& Dt, const SHitResult& Hit);
    void ApplyPersistentSupportConstraint(const float& Dt);
    bool SolveEdgeSupportConstraint(const FVector& SupportN, const TArray<FVector>& SupportPts, const float Dt, const bool bDoFriction /*= false*/,
        const float Mu /*= 0.0f*/, const bool bAllowTraversalTorque /*= false*/);

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
    TArray<FVector> CurrentGroundContactsWS;
    TArray<FVector> PreviousFrameGroundContactsWS;
    TArray<FVector> CurrentGroundNormalsWS; // per contact point normal
    TArray<FVector> PrevGroundContactsLS; // local hitbox space (COM frame)
    TArray<FVector> LatchedEdgeContactsLS; // size = 2
    bool bEdgeSupportLatched = false;
    int32 EdgeSupportLatchFrame = -1;
    bool bRoofSurfaceTraversalLatched = false;
    int32 LastRoofSurfaceContactFrame = INDEX_NONE;
    TWeakObjectPtr<UPrimitiveComponent> RoofSurfaceComp;
    FVector CompositeGroundNormal; // average normal from all ground contacts this frame
    TMap<TWeakObjectPtr<UPrimitiveComponent>, uint32> HitCountThisFrame;
    uint32 MaxHitsPerComponentPerFrame = 4; // face = 4 corners
    TArray<float> PrevLambdaN;

    bool bGroundContactStable = false; // true if every contact point were stable last frame
    int32 LastResolvedGroundHitFrame = INDEX_NONE;
    bool bHasGroundContact = false; // true if we have at least one ground contact this frame
    bool bFreshEdgeRecoverCandidate = false; // one-frame edge contact kept for car-level auto-recover
    float StableTime = 0.f;

    bool bGroundHitFromSweep = false; // true if ground hit was generated from sweep this frame
    bool bGroundPlaneValid = false;
    FVector GroundPlanePointWS = FVector::ZeroVector;; // a point on the ground plane
    FVector GroundPlaneN = FVector::UpVector;
    float GroundPlaneD = 0.f; // plane equation: dot(X, N) - D = 0
    TWeakObjectPtr<UPrimitiveComponent> GroundComp;
    uint64 EstablishedSupportSourceId = 0;
    uint64 EstablishedSupportSurfaceId = 0;
    int32 FreshSupportClampUntilFrame = INDEX_NONE;
    FVector FreshSupportClampN = FVector::UpVector;
    TWeakObjectPtr<UPrimitiveComponent> FreshSupportClampComp;
    FVector PrevGroundNormal = FVector::UpVector; // normal from previous frame ground plane
    bool bHadGroundContactPrevFrame = false;
    FVector PrevContactNormal = FVector::UpVector; // average contact normal from previous frame
    bool bHadContactPrevFrame = false;
    float MinSlopCm = 0.05f; // Minimum allowed penetration depth in cm, used as slop in the solver to prevent jittering when resolving penetrations

	// ======= Debug Netcode =======
    int32 LastWallOrGutterFrame = INDEX_NONE;
    FVector LastWallOrGutterN = FVector::ZeroVector;
    float LastWallOrGutterUpDot = 0.f;
    SHitResult LastResolvedSphereHit;
    int32 LastResolvedSphereHitFrame = INDEX_NONE;

protected:
#if !UE_BUILD_SHIPPING
    void AppendBoxEpisodeTailForTesting(TArray<uint8>& Out) const;
#endif
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
    void AppendCanonicalEpisodeStateForTesting(
        const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,TArray<uint8>& Out,
        Speed::NativeKinematics::FCanonicalMassAppendDiagnostics& Diagnostics) const override;
#endif
};
