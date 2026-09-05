#pragma once

#include "IAmSpeed/World/Analytic/AnalyticWorldQuery.h"

namespace Speed
{

enum class EStaticCollisionBackend : uint8
{
	UnrealLegacy = 0,
	AnalyticHybrid,
	SurfaceAnalytic,
};

class IAMSPEED_API IStaticCollisionWorld
{
public:
	virtual ~IStaticCollisionWorld() = default;
	virtual Analytic::FWorldHit SweepSingle(
		const Analytic::FWorldQuery& Query) const = 0;
	virtual bool HasAuthorityCoverage(
		const Analytic::FWorldQuery& Query) const = 0;
	virtual bool TrySweepAuthority(
		const Analytic::FWorldQuery& Query,
		Analytic::FWorldHit& OutAuthorityHit) const = 0;
	virtual EStaticCollisionBackend GetBackend() const = 0;
};

// Result of projecting a predicted query pose back into the feasible region of
// one already-established static contact. New contacts remain the
// responsibility of continuous collision detection.
struct IAMSPEED_API FEstablishedStaticContactProjection
{
	Analytic::FWorldHit Hit;
	FVector3d Location = FVector3d::ZeroVector;
	double MaximumPenetrationDepth = 0.0;
	double ResidualPenetrationDepth = 0.0;
	uint32 CorrectionIterations = 0;
	bool bContact = false;
	bool bConverged = false;
};

// Projects translation only. The caller owns the rigid-body velocity response
// because it has the mass, inertia and contact-point velocity. Query must be
// restricted to the immutable provider of an established contact.
IAMSPEED_API FEstablishedStaticContactProjection ProjectEstablishedStaticContact(
	const IStaticCollisionWorld& World,
	Analytic::FWorldQuery Query,
	uint32 MaxCorrectionIterations = 4,
	double PenetrationToleranceCm = 1.0e-6);

// Parallel-transports linear velocity between two analytical contact tangent
// spaces, then removes only any residual closing speed contributed by the
// transported linear and rotational point velocities. This preserves
// tangential speed without applying witness-dependent angular impulses.
IAMSPEED_API FVector TransportEstablishedContactVelocity(
	const FVector& LinearVelocity,
	const FVector& RotationalPointVelocity,
	const FVector& PreviousNormal,
	const FVector& CurrentNormal);

class IAMSPEED_API FAnalyticStaticCollisionWorld final
	: public IStaticCollisionWorld
{
public:
	explicit FAnalyticStaticCollisionWorld(
		const Analytic::FAnalyticWorldData& InWorld)
		: QueryService(InWorld)
	{
	}

	Analytic::FWorldHit SweepSingle(
		const Analytic::FWorldQuery& Query) const override
	{
		return QueryService.Sweep(Query);
	}

	bool HasAuthorityCoverage(
		const Analytic::FWorldQuery& Query) const override
	{
		return QueryService.HasAuthorityCoverage(Query);
	}

	bool TrySweepAuthority(
		const Analytic::FWorldQuery& Query,
		Analytic::FWorldHit& OutAuthorityHit) const override
	{
		return QueryService.TrySweepAuthority(Query, OutAuthorityHit);
	}

	EStaticCollisionBackend GetBackend() const override
	{
		return EStaticCollisionBackend::SurfaceAnalytic;
	}

private:
	Analytic::FWorldQueryService QueryService;
};

} // namespace Speed
