#include "StaticCollisionWorld.h"
#if !UE_BUILD_SHIPPING
#include "../Analytic/StaticWorldQueryAudit.h"
#endif

namespace Speed
{

FEstablishedStaticContactProjection ProjectEstablishedStaticContact(
	const IStaticCollisionWorld& World,
	Analytic::FWorldQuery Query,
	const uint32 MaxCorrectionIterations,
	const double PenetrationToleranceCm)
{
	FEstablishedStaticContactProjection Result;
	Result.Location = Query.Start;
	if (Query.RequiredSourceId == 0 || Query.RequiredSurfaceId == 0 ||
		Query.Start != Query.End || MaxCorrectionIterations == 0)
	{
		return Result;
	}

	const double Tolerance = FMath::Max(0.0, PenetrationToleranceCm);
	Query.InitialOverlapTolerance = 0.0;
	for (uint32 Iteration = 0; Iteration < MaxCorrectionIterations; ++Iteration)
	{
		Query.Start = Result.Location;
		Query.End = Result.Location;
		const Analytic::FWorldHit Candidate = World.SweepSingle(Query);
#if !UE_BUILD_SHIPPING
		Analytic::FStaticWorldQueryAudit::RecordEstablishedContactQuery(Query, Candidate, Iteration, false);
#endif
		if (!Candidate.bHit)
		{
			Result.ResidualPenetrationDepth = 0.0;
			Result.bConverged = Result.bContact;
			return Result;
		}
		Result.Hit = Candidate;
		Result.bContact = true;

		const double Depth = Result.Hit.bStartPenetrating
			? FMath::Max(0.0, Result.Hit.PenetrationDepth) : 0.0;
		Result.MaximumPenetrationDepth =
			FMath::Max(Result.MaximumPenetrationDepth, Depth);
		Result.ResidualPenetrationDepth = Depth;
		if (Depth <= Tolerance)
		{
			Result.bConverged = true;
			return Result;
		}

		const FVector3d Normal = Result.Hit.Normal.GetSafeNormal();
		if (Normal.IsNearlyZero())
		{
			return Result;
		}
		Result.Location += Depth * Normal;
		++Result.CorrectionIterations;
	}

	// The loop's final correction has not been measured yet. One validation
	// query keeps the correction count bounded while reporting the true residual.
	Query.Start = Result.Location;
	Query.End = Result.Location;
	const Analytic::FWorldHit ValidationHit = World.SweepSingle(Query);
#if !UE_BUILD_SHIPPING
	Analytic::FStaticWorldQueryAudit::RecordEstablishedContactQuery(Query, ValidationHit, MaxCorrectionIterations, true);
#endif
	if (ValidationHit.bHit)
	{
		Result.Hit = ValidationHit;
		Result.bContact = true;
	}
	Result.ResidualPenetrationDepth =
		ValidationHit.bHit && ValidationHit.bStartPenetrating
		? FMath::Max(0.0, ValidationHit.PenetrationDepth) : 0.0;
	Result.bConverged = Result.ResidualPenetrationDepth <= Tolerance;
	return Result;
}

FVector TransportEstablishedContactVelocity(
	const FVector& LinearVelocity,
	const FVector& RotationalPointVelocity,
	const FVector& PreviousNormal,
	const FVector& CurrentNormal)
{
	const FVector From = PreviousNormal.GetSafeNormal();
	const FVector To = CurrentNormal.GetSafeNormal();
	if (From.IsNearlyZero() || To.IsNearlyZero() ||
		FVector::DotProduct(From, To) <= 0.0f)
	{
		return LinearVelocity;
	}

	FVector Result = FQuat::FindBetweenNormals(From, To).RotateVector(
		LinearVelocity);
	const float NormalSpeed = FVector::DotProduct(
		Result + RotationalPointVelocity, To);
	if (NormalSpeed < 0.0f)
	{
		Result -= NormalSpeed * To;
	}
	return Result;
}

} // namespace Speed
