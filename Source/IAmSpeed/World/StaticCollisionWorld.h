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
	virtual EStaticCollisionBackend GetBackend() const = 0;
};

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

	EStaticCollisionBackend GetBackend() const override
	{
		return EStaticCollisionBackend::SurfaceAnalytic;
	}

private:
	Analytic::FWorldQueryService QueryService;
};

} // namespace Speed
