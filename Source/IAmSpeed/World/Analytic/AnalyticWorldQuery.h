#pragma once

#include "AnalyticWorldData.h"

namespace Speed::Analytic
{

enum class EQueryShape : uint8
{
	Ray = 0,
	Sphere = 1,
	Box = 2,
};

struct IAMSPEED_API FWorldQuery
{
	EQueryShape Shape = EQueryShape::Ray;
	FVector3d Start = FVector3d::ZeroVector;
	FVector3d End = FVector3d::ZeroVector;
	FQuat4d Rotation = FQuat4d::Identity;
	double Radius = 0.0;
	FVector3d HalfExtent = FVector3d::ZeroVector;
	double DomainTolerance = 1.0e-6;
	uint8 TraceChannel = 0;
	uint64 ObjectTypes = 0;
	bool bObjectQuery = false;
	bool bApplyCollisionFilter = false;
	bool bIncludeTriangles = false;
};

struct IAMSPEED_API FWorldHit
{
	bool bHit = false;
	bool bStartPenetrating = false;
	double Time = 1.0;
	double PenetrationDepth = 0.0;
	FVector3d Location = FVector3d::ZeroVector;
	FVector3d Point = FVector3d::ZeroVector;
	FVector3d Normal = FVector3d::ZeroVector;
	uint64 SurfaceId = 0;
	uint64 FeatureId = 0;
	uint64 PrimitiveId = 0;
	uint32 MaterialId = 0;
};

class IAMSPEED_API FWorldQueryService
{
public:
	explicit FWorldQueryService(const FAnalyticWorldData& InWorld) : World(InWorld) {}

	FWorldHit Sweep(const FWorldQuery& Query) const;

private:
	const FAnalyticWorldData& World;
	static double SupportRadius(const FWorldQuery& Query, const FVector3d& Normal);
	static FWorldHit SweepPlane(const FWorldQuery& Query, const FBoundedPlane& Plane);
	static FWorldHit SweepTriangleFace(
		const FWorldQuery& Query, const FTriangleSurface& Triangle);
	static bool TrianglePassesFilter(
		const FWorldQuery& Query, const FTriangleSurface& Triangle);
	static bool IsBetterHit(const FWorldHit& Candidate, const FWorldHit& Best);
};

} // namespace Speed::Analytic
