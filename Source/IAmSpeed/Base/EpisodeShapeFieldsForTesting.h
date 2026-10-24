#pragma once
#include "EpisodeStateBytesForTesting.h"
#include "SHitResult.h"
#include "PhysicalContactConstraint.h"
#if !UE_BUILD_SHIPPING
namespace Speed {
template<> struct FEpisodeFieldCodec<FKinematicState> {
 static void Write(TArray<uint8>& O,const FKinematicState& X) {
  using S=std::remove_cvref_t<decltype(X.Location.X)>;
  static_assert(std::is_same_v<S,float> || std::is_same_v<S,double>);
  static_assert(std::is_same_v<S,std::remove_cvref_t<decltype(X.Location.Y)>>
   && std::is_same_v<S,std::remove_cvref_t<decltype(X.Location.Z)>>
   && std::is_same_v<S,std::remove_cvref_t<decltype(X.Velocity.X)>>
   && std::is_same_v<S,std::remove_cvref_t<decltype(X.Velocity.Y)>>
   && std::is_same_v<S,std::remove_cvref_t<decltype(X.Velocity.Z)>>
   && std::is_same_v<S,std::remove_cvref_t<decltype(X.Acceleration.X)>>
   && std::is_same_v<S,std::remove_cvref_t<decltype(X.Acceleration.Y)>>
   && std::is_same_v<S,std::remove_cvref_t<decltype(X.Acceleration.Z)>>
   && std::is_same_v<S,std::remove_cvref_t<decltype(X.Rotation.X)>>
   && std::is_same_v<S,std::remove_cvref_t<decltype(X.Rotation.Y)>>
   && std::is_same_v<S,std::remove_cvref_t<decltype(X.Rotation.Z)>>
   && std::is_same_v<S,std::remove_cvref_t<decltype(X.Rotation.W)>>
   && std::is_same_v<S,std::remove_cvref_t<decltype(X.AngularVelocity.X)>>
   && std::is_same_v<S,std::remove_cvref_t<decltype(X.AngularVelocity.Y)>>
   && std::is_same_v<S,std::remove_cvref_t<decltype(X.AngularVelocity.Z)>>
   && std::is_same_v<S,std::remove_cvref_t<decltype(X.AngularAcceleration.X)>>
   && std::is_same_v<S,std::remove_cvref_t<decltype(X.AngularAcceleration.Y)>>
   && std::is_same_v<S,std::remove_cvref_t<decltype(X.AngularAcceleration.Z)>>);
  // Named scalars only: no UE object layout/padding, conversion, normalization,
  // or array-length prefix. Preserve the field codec's existing byte order.
  const S Values[]={X.Location.X,X.Location.Y,X.Location.Z,
   X.Velocity.X,X.Velocity.Y,X.Velocity.Z,
   X.Acceleration.X,X.Acceleration.Y,X.Acceleration.Z,
   X.Rotation.X,X.Rotation.Y,X.Rotation.Z,X.Rotation.W,
   X.AngularVelocity.X,X.AngularVelocity.Y,X.AngularVelocity.Z,
   X.AngularAcceleration.X,X.AngularAcceleration.Y,X.AngularAcceleration.Z};
  static_assert(sizeof(Values)==19*sizeof(S));
  O.Append(reinterpret_cast<const uint8*>(Values),sizeof(Values));
 }
};
template<> struct FEpisodeFieldCodec<SHitResult> { static void Write(TArray<uint8>& O,const SHitResult& X) { FEpisodeStateBytesForTesting::Add(O,X.bHit);FEpisodeStateBytesForTesting::Add(O,X.bBlockingHit);FEpisodeStateBytesForTesting::Add(O,X.Location);FEpisodeStateBytesForTesting::Add(O,X.ImpactPoint);FEpisodeStateBytesForTesting::Add(O,X.ImpactNormal);FEpisodeStateBytesForTesting::Add(O,X.TOI);FEpisodeStateBytesForTesting::Add(O,X.FaceIndex);FEpisodeStateBytesForTesting::Add(O,X.Component);FEpisodeStateBytesForTesting::Add(O,X.SubBody);FEpisodeStateBytesForTesting::Add(O,X.bStartPenetrating);FEpisodeStateBytesForTesting::Add(O,X.PenetrationDepth);FEpisodeStateBytesForTesting::Add(O,X.GeometricErrorBoundCm);FEpisodeStateBytesForTesting::Add(O,X.bSurfaceNormalMayVary);FEpisodeStateBytesForTesting::Add(O,X.ContactPointThis);FEpisodeStateBytesForTesting::Add(O,X.ContactPointOther);FEpisodeStateBytesForTesting::Add(O,X.ContactFeatureThis);FEpisodeStateBytesForTesting::Add(O,X.ContactFeatureOther);FEpisodeStateBytesForTesting::Add(O,X.ContactFeatureIndexThis);FEpisodeStateBytesForTesting::Add(O,X.ContactFeatureIndexOther);FEpisodeStateBytesForTesting::Add(O,X.SourceId);FEpisodeStateBytesForTesting::Add(O,X.SurfaceId);FEpisodeStateBytesForTesting::Add(O,X.FeatureId);FEpisodeStateBytesForTesting::Add(O,X.PrimitiveId);FEpisodeStateBytesForTesting::Add(O,X.CanonicalGroupId);FEpisodeStateBytesForTesting::Add(O,X.MaterialId);FEpisodeStateBytesForTesting::Add(O,X.FrameTag); } };
template<> struct FEpisodeFieldCodec<FPhysicalContactConstraint> { static void Write(TArray<uint8>& O,const FPhysicalContactConstraint& X) { FEpisodeStateBytesForTesting::Add(O,X.Normal);FEpisodeStateBytesForTesting::Add(O,X.ContactPoint);FEpisodeStateBytesForTesting::Add(O,X.OtherComponent);FEpisodeStateBytesForTesting::Add(O,X.SourceSubBody);FEpisodeStateBytesForTesting::Add(O,X.PenetrationDepth);FEpisodeStateBytesForTesting::Add(O,X.TOI);FFramePointerCaptureScope::ObservePairToken(O,X.PairKey,X.SourceSubBody.Get(),X.OtherComponent.Get());FEpisodeStateBytesForTesting::Add(O,X.PairKey);FEpisodeStateBytesForTesting::Add(O,X.FramesSinceSeen);FEpisodeStateBytesForTesting::Add(O,X.bPersistent); } };
}
#endif
