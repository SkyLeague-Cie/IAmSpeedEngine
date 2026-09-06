#pragma once

#include "StaticCollisionWorld.h"
#include "IAmSpeed/Base/Kinematic.h"

namespace Speed
{
enum class EBoxRestingSupportResult : uint8
{
	InvalidQuery, UnsupportedLoad, UnstableCOM, Penetrating, UncertifiedGeometry, NotTouching, Supported
};
/** A feasible four-point normal reaction, expressed per unit owner mass. */
struct IAMSPEED_API FBoxRestingSupport
{
	FVector ResultantAcceleration = FVector::ZeroVector;
	FVector Normal = FVector::ZeroVector;
	FVector Points[4] = { FVector::ZeroVector, FVector::ZeroVector, FVector::ZeroVector, FVector::ZeroVector };
	/** Immutable provider witness for publishing support without manufacturing an impact. */
	Analytic::FWorldHit ProviderWitness;
	double LoadFractions[4] = {};
	EBoxRestingSupportResult Result = EBoxRestingSupportResult::InvalidQuery;
	int32 WitnessIndex = INDEX_NONE;
	double WitnessGapCm = 0.0;
	/** Finite only for Coulomb sliding; marks the physical stop event, not sleep. */
	double StopAfterSeconds = TNumericLimits<double>::Max();
};

/**
 * Certifies a stationary box face against current finite, exact planar geometry.
 * Returns a unilateral, torque-free reaction to a purely normal COM load. This
 * does not change the pose/velocity, acquire sleep, or reuse a previous contact.
 * BoxQuery supplies the actual pose and collision filters, not an inflated box.
 * Partial/curved/penetrating support and tipping COMs are conservatively refused.
 */
IAMSPEED_API bool TryBuildBoxRestingSupport(
	const IStaticCollisionWorld& World,
	const Analytic::FWorldQuery& BoxQuery,
	const FVector& CenterOfMass,
	const FVector& ExternalAcceleration,
	FBoxRestingSupport& OutSupport);

/** Certifies full-face Coulomb sliding under a purely normal COM load.
 * The pressure center balances friction torque and must remain inside the
 * face. Unsupported/tipping motion is rejected, never orientation-locked.
 * A positive horizon also requires continuous finite support along the whole
 * material trajectory (up to its stop); zero evaluates only the current pose. */
IAMSPEED_API bool TryBuildBoxSlidingSupport(
	const IStaticCollisionWorld& World, const Analytic::FWorldQuery& BoxQuery,
	const FVector& CenterOfMass, const FVector& ExternalAcceleration,
	const FVector& TangentialVelocity, double DynamicFriction,
	FBoxRestingSupport& OutSupport, double HorizonSeconds = 0);

/** Certifies resolution-bounded closure of an inelastic micro-rocking impact.
 * Rechecks a complete finite face and a feasible load, without moving the pose.
 * Total incoming kinetic energy must be below the potential energy of half a
 * canonical position step, friction stopping work and corner-motion/tipping
 * energy bounds. Pure sliding/spinning, separating, driven, elastic,
 * unsupported and resolvable motion are refused. No persistent sleep state. */
IAMSPEED_API bool CanStabilizeBoxMicroRocking(
	const IStaticCollisionWorld& World, const Analytic::FWorldQuery& BoxQuery,
	const FKinematicState& Incoming, const FMatrix& WorldInverseInertia,
	double InverseMass, double ImpactRestitution, double DynamicFriction);
}
