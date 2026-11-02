#pragma once
// Exact fields of the SIX original body value structs, not whole-epoch coverage.
// References use the owner's original graph table; unknown/stale is refusal.
// No UObject/raw pointer/padding is serialized. This does not supply a lease,
// authority, a configuration census or restoration of any physical object.
#include "PrivateRoadNativeObjects.h"
#include "IAmSpeed/SubBodies/Solid/CylinderSubBody.h"
#include "IAmSpeed/World/Simulation/ReplayFieldArchive.h"
#include "IAmSpeed/World/Analytic/AnalyticWorldData.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <set>
#include <tuple>
#include <type_traits>

namespace Speed::Netcode::Prediction::OriginalBodyCapture
{
inline constexpr std::size_t StoredResponseChannels=std::extent_v<decltype(FCollisionResponseContainer::EnumArray)>;
// ECC_MAX includes an extra deprecated marker, not a stored channel. Bind to
// the ACTUAL native array extent and prove that profile boundary at compile.
static_assert(StoredResponseChannels==std::size_t(ECC_OverlapAll_Deprecated));

enum class EReferenceRole : uint16 {Component=1,Box,Wheel,WheelObject,NativeWheel,NativeSuspension,StaticSource};
struct FReferenceKey
{
    uint64 Actor=0,Incarnation=0;uint16 Role=0,Index=0;uint64 Source=0;
    auto Tuple()const noexcept{return std::tie(Actor,Incarnation,Role,Index,Source);}
    bool operator<(const FReferenceKey& R)const noexcept{return Tuple()<R.Tuple();}
    bool operator==(const FReferenceKey& R)const noexcept{return Tuple()==R.Tuple();}
};
class FReferences final
{
    std::map<const void*,FReferenceKey> Entries;
    bool Sealed=false;
public:
    // The private owner must enumerate ACTUAL graph identities and keep them
    // alive through capture. Registering numbers is not lifecycle authority.
    bool Add(const void* OriginalAddress,FReferenceKey Key)
    {
        if(Sealed||!OriginalAddress||!Key.Role)return false;
        const bool Static=Key.Role==uint16(EReferenceRole::StaticSource);
        if(Static?(!Key.Source||Key.Actor||Key.Incarnation||Key.Index):(!Key.Actor||!Key.Incarnation||Key.Source))return false;
        const auto [It,Added]=Entries.emplace(OriginalAddress,Key);
        return Added||It->second==Key;
    }
    bool Seal(){if(Sealed||Entries.empty())return false;Sealed=true;return true;}
    std::optional<FReferenceKey> Find(const void* OriginalAddress)const
    {
        if(!Sealed)return {};
        if(!OriginalAddress)return FReferenceKey{};
        const auto It=Entries.find(OriginalAddress);if(It==Entries.end())return {};
        return It->second;
    }
};
class FWriter final
{
    Replay::FFieldWriter Out;
    const FReferences& References;
    void Key(const FReferenceKey& K)
    {Value(K.Actor);Value(K.Incarnation);Value(K.Role);Value(K.Index);Value(K.Source);}
public:
    explicit FWriter(const FReferences& R):References(R){}
    const Replay::FBytes& Bytes()const{return Out.Bytes();}
    template<class T>requires(std::is_integral_v<T>||std::is_enum_v<T>||std::is_floating_point_v<T>)
    void Value(const T& V)
    {
        if constexpr(std::is_floating_point_v<T>)if(!std::isfinite(V))throw Replay::FFieldError("nonfinite original body field");
        Out.Scalar(V);
    }
    void Value(const FVector& V){Value(V.X);Value(V.Y);Value(V.Z);}
    void Value(const FVector2d& V){Value(V.X);Value(V.Y);}
    void Value(const FQuat& V){Value(V.X);Value(V.Y);Value(V.Z);Value(V.W);}
    void Value(const FTransform& V){Value(V.GetTranslation());Value(V.GetRotation());Value(V.GetScale3D());}
    void Value(const FMatrix& V){for(int32 I=0;I<4;++I)for(int32 J=0;J<4;++J)Value(V.M[I][J]);}
    void Value(const FBox3d& V){Value(V.IsValid);Value(V.Min);Value(V.Max);}
    template<class T>void Value(T* V)
    {
        const auto K=References.Find(V);if(!K)throw Replay::FFieldError("unknown original graph reference");Key(*K);
    }
    template<class T>void Value(const TObjectPtr<T>& V){Value(V.Get());}
    template<class T>void Value(const TWeakObjectPtr<T>& V)
    {
        if(V.IsExplicitlyNull()){Key(FReferenceKey{});return;}
        T* P=V.Get();if(!P)throw Replay::FFieldError("stale original graph reference");Value(P);
    }
    template<class T>void Value(const TArray<T>& V)
    {
        if(V.Num()>65536)throw Replay::FFieldError("original body collection bound");
        Value(uint32(V.Num()));for(const auto& Item:V)Value(Item);
    }
    void Value(const FCollisionResponseParams& V)
    {for(std::size_t I=0;I<StoredResponseChannels;++I)Value(V.CollisionResponse.GetResponse(ECollisionChannel(I)));}
    void Value(const Speed::FKinematicState& V)
    {Value(V.Location);Value(V.Velocity);Value(V.Acceleration);Value(V.Rotation);Value(V.AngularVelocity);Value(V.AngularAcceleration);}
    void Value(const SWheelRenderData& V)
    {
        Value(V.RollRotation);Value(V.SteerRotation);Value(V.WorldPosition);
        Value(V.RightAxis);Value(V.UpAxis);Value(V.ForwardAxis);Value(V.SpringOffset);
    }
    void Value(const SHitResult& V)
    {
        Value(V.bHit);Value(V.bBlockingHit);Value(V.Location);Value(V.ImpactPoint);Value(V.ImpactNormal);
        Value(V.TOI);Value(V.FaceIndex);Value(V.Component);Value(V.SubBody);Value(V.bStartPenetrating);
        Value(V.PenetrationDepth);Value(V.GeometricErrorBoundCm);Value(V.bSurfaceNormalMayVary);
        Value(V.ContactPointThis);Value(V.ContactPointOther);Value(V.ContactFeatureThis);Value(V.ContactFeatureOther);
        Value(V.ContactFeatureIndexThis);Value(V.ContactFeatureIndexOther);Value(V.SourceId);Value(V.SurfaceId);
        Value(V.FeatureId);Value(V.PrimitiveId);Value(V.CanonicalGroupId);Value(V.MaterialId);Value(V.FrameTag);
    }
    void Value(const TMap<TWeakObjectPtr<UPrimitiveComponent>,uint32>& V)
    {
        if(V.Num()>65536)throw Replay::FFieldError("original hitcount map bound");
        std::map<FReferenceKey,uint32> Sorted;
        for(const auto& Pair:V)
        {
            if(!Pair.Key.IsExplicitlyNull()&&!Pair.Key.IsValid())throw Replay::FFieldError("stale hitcount reference");
            const auto K=References.Find(Pair.Key.Get());
            if(!K||!Sorted.emplace(*K,Pair.Value).second)throw Replay::FFieldError("unknown or duplicate hitcount identity");
        }
        Value(uint32(Sorted.size()));for(const auto& Pair:Sorted){Key(Pair.first);Value(Pair.second);}
    }
};

inline void AppendPlane(FWriter& A,const Speed::Analytic::FBoundedPlane& V)
{
    A.Value(V.SourceId);A.Value(V.SurfaceId);A.Value(V.FeatureId);A.Value(V.PrimitiveId);
    A.Value(V.MaterialId);A.Value(V.ObjectType);A.Value(V.BlockingChannels);
    A.Value(V.Origin);A.Value(V.Normal);A.Value(V.AxisU);A.Value(V.AxisV);
    A.Value(V.HalfExtents);A.Value(V.DomainVertices);A.Value(V.Bounds);
    A.Value(V.bQueryCollisionEnabled);A.Value(V.bRequiresCompactOptIn);A.Value(V.bAuthorityEligible);
}

inline void Append(FWriter& A,const FPrivateRoadSubBodyValues& V)
{
    A.Value(V.LocalPose);
    A.Value(V.ParentComponent);
    A.Value(V.SubBodyType);
    A.Value(V.CollisionChannel);
    A.Value(V.ResponseParams);
    A.Value(V.AlwaysIgnoredComponents);
    A.Value(V.IgnoredComponents);
    A.Value(V.IgnoredSubBodyTypes);
    A.Value(V.ExternalBoxSubBodies);
    A.Value(V.ExternalSphereSubBodies);
    A.Value(V.ExternalWheelSubBodies);
    A.Value(V.ExternalCylinderSubBodies);
    A.Value(V.Kinematics);
    A.Value(V.FutureHit);
    A.Value(V.CurrentHit);
    A.Value(V.GroundHit);
    A.Value(V.BoxHit);
    A.Value(V.SphereHit);
    A.Value(V.WheelHit);
}

inline void Append(FWriter& A,const FPrivateRoadBoxDomain& V)
{
    A.Value(V.BoxExtent);
    A.Value(V.bProductionNonpenetratingSphereContacts);
    A.Value(V.CurrentGroundContactsWS);
    A.Value(V.PreviousFrameGroundContactsWS);
    A.Value(V.CurrentGroundNormalsWS);
    A.Value(V.PrevGroundContactsLS);
    A.Value(V.LatchedEdgeContactsLS);
    A.Value(V.bEdgeSupportLatched);
    A.Value(V.EdgeSupportLatchFrame);
    A.Value(V.bRoofSurfaceTraversalLatched);
    A.Value(V.LastRoofSurfaceContactFrame);
    A.Value(V.RoofSurfaceComp);
    A.Value(V.CompositeGroundNormal);
    A.Value(V.HitCountThisFrame);
    A.Value(V.MaxHitsPerComponentPerFrame);
    A.Value(V.PrevLambdaN);
    A.Value(V.bGroundContactStable);
    A.Value(V.LastResolvedGroundHitFrame);
    A.Value(V.bHasGroundContact);
    A.Value(V.bFreshEdgeRecoverCandidate);
    A.Value(V.StableTime);
    A.Value(V.bGroundHitFromSweep);
    A.Value(V.bGroundPlaneValid);
    A.Value(V.GroundPlanePointWS);
    A.Value(V.GroundPlaneN);
    A.Value(V.GroundPlaneD);
    A.Value(V.GroundComp);
    A.Value(V.EstablishedSupportSourceId);
    A.Value(V.EstablishedSupportSurfaceId);
    A.Value(V.FreshSupportClampUntilFrame);
    A.Value(V.FreshSupportClampN);
    A.Value(V.FreshSupportClampComp);
    A.Value(V.PrevGroundNormal);
    A.Value(V.bHadGroundContactPrevFrame);
    A.Value(V.PrevContactNormal);
    A.Value(V.bHadContactPrevFrame);
    A.Value(V.MinSlopCm);
    A.Value(V.LastWallOrGutterFrame);
    A.Value(V.LastWallOrGutterN);
    A.Value(V.LastWallOrGutterUpDot);
    A.Value(V.LastResolvedSphereHit);
    A.Value(V.LastResolvedSphereHitFrame);
}

inline void Append(FWriter& A,const FPrivateRoadWheelDomain& V)
{
    A.Value(V.WheelComponent);
    A.Value(V.ChaosWheel);
    A.Value(V.PWheel);
    A.Value(V.PSuspension);
    A.Value(V.WheelMass);
    A.Value(V.SuspensionSpringRate);
    A.Value(V.SuspensionDampingCompressionRatio);
    A.Value(V.SuspensionDampingReboundRatio);
    A.Value(V.bUseSuspensionForceModelOverride);
    A.Value(V.bClampSuspensionForceToPositive);
    A.Value(V.bClampPositiveUntilFirstCompression);
    A.Value(V.bUseEffectiveSuspensionSweepRadius);
    A.Value(V.bUseIncreasingDisplacementAsCompression);
    A.Value(V.bUseCompressionCrossingContactImpulse);
    A.Value(V.GroundForceApplicationMode);
    A.Value(V.ContactImpulseApplicationMode);
    A.Value(V.SuspensionStiffnessForceScale);
    A.Value(V.SuspensionStiffnessReferenceDisplacement);
    A.Value(V.SuspensionPreRestCompressionDampingScale);
    A.Value(V.bClampNegativeSuspensionDisplacement);
    A.Value(V.bDisableSuspensionBumpStopForce);
    A.Value(V.SuspensionSpringRateCmOverride);
    A.Value(V.SuspensionCompressionDampingOverride);
    A.Value(V.SuspensionReboundDampingOverride);
    A.Value(V.SuspensionMaxRaiseOverride);
    A.Value(V.SuspensionMaxDropOverride);
    A.Value(V.ConfiguredStaticSpringCompression);
}

inline void Append(FWriter& A,const FPrivateRoadSphereDomain& V)
{
    A.Value(V.Radius);
}

inline void Append(FWriter& A,const FPrivateRoadSolidValues& V)
{
    A.Value(V.Mass);
    A.Value(V.bIsMainSubBody);
    A.Value(V.bApplyRestForce);
    A.Value(V.EnableFakePhysics);
    A.Value(V.bEnableConstraintOnSubBodyHit);
    A.Value(V.Restitution);
    A.Value(V.SphereBoxRestitutionOverride);
    A.Value(V.SphereBoxFrictionOverride);
    A.Value(V.SphereBoxManifoldFrictionOverride);
    A.Value(V.bUseCoupledContactImpulse);
    A.Value(V.bUsePostNormalFrictionImpulse);
    A.Value(V.bUsePersistentBilateralContact);
    A.Value(V.CoupledContactMaxLinearSpeed);
    A.Value(V.CoupledContactMaxAngularSpeed);
    A.Value(V.CoupledContactMaxAbsLocalNormalY);
    A.Value(V.StaticFriction);
    A.Value(V.DynamicFriction);
    A.Value(V.ImpactThreshold);
    A.Value(V.HitDamping);
    A.Value(V.InvInertiaLocal);
    A.Value(V.idx);
}

inline void Append(FWriter& A,const FPrivateRoadWheelRuntimeValues& V)
{
    A.Value(V.SuspensionForce);
    A.Value(V.bIsJumping);
    A.Value(V.bJumpUnilateralSupport);
    A.Value(V.ConsecutiveGroundFrames);
    A.Value(V.SteeringAngle);
    A.Value(V.Omega);
    A.Value(V.RollAngle);
    A.Value(V.bWasOnGroundPrevFrame);
    A.Value(V.bCountedGroundFrame);
    A.Value(V.bContactVelocityLocked);
    A.Value(V.bCrossedIntoCompressionThisFrame);
    A.Value(V.ForwardAxis);
    A.Value(V.RightAxis);
    A.Value(V.UpAxis);
    A.Value(V.RenderData);
}

template<class T>std::optional<Replay::FBytes> Capture(const FReferences& OriginalReferences,const T& OriginalValues)
{
    try{FWriter W(OriginalReferences);W.Value(uint32(0x31424349));W.Value(uint16(1));Append(W,OriginalValues);return W.Bytes();}
    catch(const Replay::FFieldError&){return {};}
}
}
