#pragma once

#include "CoreMinimal.h"

namespace Speed
{
/** Simultaneous normal response for up to four coplanar rigid-body contacts. */
struct IAMSPEED_API FPlanarNormalImpulse
{
	FVector DeltaVelocity = FVector::ZeroVector;
	FVector DeltaAngularVelocity = FVector::ZeroVector;
	double Impulses[4] = {};
	/** Tangential witness at each input point; zero for a normal-only solve. */
	FVector TangentImpulses[4] = {};
	double MaximumClosingResidual = 0;
	uint16 IterationCount = 0;
	bool bSolved = false;
};

/** Solves the unilateral normal complementarity problem in double precision.
 * Enumerates at most 14 independent active sets (normal contact rank <= 3),
 * instead of applying a clamped impulse to successive corners. Input contacts
 * must already be valid geometry; this function neither moves the body nor
 * adds friction, persistence, sleep or a host-specific preferred face.
 * An optional per-point normal bias also supports the acceleration-level
 * equation (including centripetal acceleration), with restitution zero.
 */
IAMSPEED_API FPlanarNormalImpulse SolvePlanarNormalImpulse(
	TConstArrayView<FVector> Points, const FVector& Normal,
	const FVector& COM, const FVector& Velocity, const FVector& AngularVelocity,
	double InverseMass, const FMatrix& WorldInverseInertia, double Restitution,
	TConstArrayView<double> PointNormalBias = {});

/** Coupled normal/Coulomb response, with accumulated tangential impulses.
 * Re-solves the normal block after friction instead of spending each newly
 * generated normal impulse as another independent friction budget.
 * Plastic edge/face sticking uses a closed impulse-feasibility certificate
 * when available. An edge retains its physically free axial rotation.
 */
IAMSPEED_API FPlanarNormalImpulse SolvePlanarCoulombImpulse(
	TConstArrayView<FVector> Points, const FVector& Normal,
	const FVector& COM, const FVector& Velocity, const FVector& AngularVelocity,
	double InverseMass, const FMatrix& WorldInverseInertia, double Restitution, double Friction);

/** Continuous sliding reaction on one to four supported points. Friction opposes each
 * point's actual slip (including normal-axis spin), not its acceleration.
 * Returns acceleration deltas; rejects unresolved pressure/friction coupling
 * or a possible slip reversal within HorizonSeconds. Never stops the body.
 */
IAMSPEED_API FPlanarNormalImpulse SolvePlanarSlidingReaction(
	TConstArrayView<FVector> Points, const FVector& Normal, const FVector& COM,
	const FVector& Velocity, const FVector& AngularVelocity,
	const FVector& Acceleration, const FVector& AngularAcceleration,
	double InverseMass, const FMatrix& WorldInverseInertia, double Friction, double HorizonSeconds);
}
