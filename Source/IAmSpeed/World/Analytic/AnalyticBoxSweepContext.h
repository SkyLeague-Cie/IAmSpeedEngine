#pragma once

#include "AnalyticWorldQuery.h"

namespace Speed::Analytic::Private
{
	/** Query-local OBB geometry shared by triangle and bounds sweeps. */
	struct FBoxSweepContext
	{
		FVector3d Axis[3] = {};
		FVector3d Motion = FVector3d::ZeroVector;
		FVector3d HalfExtent = FVector3d::ZeroVector;
		FVector3d WorldExtent = FVector3d::ZeroVector;
		double DomainTolerance = 0.0;

		FBoxSweepContext() = default;
		explicit FBoxSweepContext(const FWorldQuery& Query) { Initialize(Query); }

		void Initialize(const FWorldQuery& Query, const FVector3d* CachedAxes = nullptr)
		{
			Motion = Query.End - Query.Start;
			HalfExtent = FVector3d(
				FMath::Max(0.0, Query.HalfExtent.X),
				FMath::Max(0.0, Query.HalfExtent.Y),
				FMath::Max(0.0, Query.HalfExtent.Z));
			DomainTolerance = FMath::Max(0.0, Query.DomainTolerance);
			if (CachedAxes)
			{
				Axis[0] = CachedAxes[0];
				Axis[1] = CachedAxes[1];
				Axis[2] = CachedAxes[2];
			}
			else
			{
				Axis[0] = Query.Rotation.RotateVector(FVector3d::ForwardVector).GetSafeNormal();
				Axis[1] = Query.Rotation.RotateVector(FVector3d::RightVector).GetSafeNormal();
				Axis[2] = Query.Rotation.RotateVector(FVector3d::UpVector).GetSafeNormal();
			}
			WorldExtent = Axis[0].GetAbs() * HalfExtent.X +
				Axis[1].GetAbs() * HalfExtent.Y +
				Axis[2].GetAbs() * HalfExtent.Z;
		}
	};

	/** Bounds-only specialization: lazily derive each fixed cross-axis once per query. */
	struct FBoxBoundsSweepContext final : FBoxSweepContext
	{
		struct FProjection
		{
			FVector3d Axis;
			double BoxRadius;
			double MotionDistance;
			double ToleranceRadius;
			bool bDegenerate;
		};

		FBoxBoundsSweepContext() = default;
		explicit FBoxBoundsSweepContext(const FWorldQuery& Query) { Initialize(Query); }
		/** Start another immutable query; never mutate inherited geometry after initialization. */
		void Initialize(const FWorldQuery& Query, const FVector3d* CachedAxes = nullptr)
		{
			FBoxSweepContext::Initialize(Query, CachedAxes);
			PreparedMask = 0;
		}

		/** Same scalar operations as the original per-node SAT; node-dependent terms stay outside. */
		const FProjection& GetCrossAxis(const int32 BoxAxisIndex, const int32 WorldAxisIndex) const
		{
			const int32 Index = 3 * BoxAxisIndex + WorldAxisIndex;
			const uint16 Bit = uint16(1u << Index);
			FProjection& Projection = Projections[Index];
			if ((PreparedMask & Bit) == 0)
			{
				static const FVector3d WorldAxes[3] = {
					FVector3d::ForwardVector, FVector3d::RightVector, FVector3d::UpVector };
				Projection.Axis = FVector3d::CrossProduct(Axis[BoxAxisIndex], WorldAxes[WorldAxisIndex]);
				const double AxisLengthSquared = Projection.Axis.SquaredLength();
				Projection.bDegenerate = AxisLengthSquared <= 1.0e-24;
				if (!Projection.bDegenerate)
				{
					Projection.BoxRadius =
						HalfExtent.X * FMath::Abs(FVector3d::DotProduct(Axis[0], Projection.Axis)) +
						HalfExtent.Y * FMath::Abs(FVector3d::DotProduct(Axis[1], Projection.Axis)) +
						HalfExtent.Z * FMath::Abs(FVector3d::DotProduct(Axis[2], Projection.Axis));
					Projection.MotionDistance = FVector3d::DotProduct(Motion, Projection.Axis);
					Projection.ToleranceRadius = DomainTolerance * FMath::Sqrt(AxisLengthSquared);
				}
				PreparedMask |= Bit;
			}
			return Projection;
		}

	private:
		// No initialization of unused projections: PreparedMask guards every read.
		// The entire context belongs to one query/lane, never to the simulation world.
		mutable FProjection Projections[9];
		mutable uint16 PreparedMask = 0;
	};
	static_assert(sizeof(FBoxBoundsSweepContext) <= 1024, "Bound per-query scratch independently of BVH size");
}
