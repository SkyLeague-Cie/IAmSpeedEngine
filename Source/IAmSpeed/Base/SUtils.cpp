#include "SUtils.h"
#include "Engine/World.h"

using namespace Speed;

SSphere::SSphere(const FVector& InCenter, const float& InRadius, const FVector& Velocity, const FVector& Acceleration) :
	Center(InCenter), Radius(InRadius), Vel(Velocity), Accel(Acceleration)
{
}

bool SSphere::IsInside(const FVector& Point) const
{
	return (FVector::DistSquared(Point, Center) <= FMath::Square(Radius));
}

SHitResult SSphere::IntersectNextFrame(const SSphere& Other, const float& deltaTime) const
{
	if (Radius <= 0.f || Other.Radius <= 0.f)
		return SHitResult();
	const float Epsilon = 1e-4f;
	const int MaxIter = 20;

	auto DistMinusR = [&](float t)
		{
			FVector d =
				(Other.Center + Other.Vel * t + 0.5f * Other.Accel * t * t)
				- (Center + Vel * t + 0.5f * Accel * t * t);
			return d.Size() - (Radius + Other.Radius);
		};

	float f0 = DistMinusR(0.f);
	if (f0 <= 0.f)
	{
		const FVector pA0 = Center;
		const FVector pB0 = Other.Center;

		FVector N = (pA0 - pB0);
		const float distSq = N.SizeSquared();

		if (distSq > KINDA_SMALL_NUMBER)
		{
			N /= FMath::Sqrt(distSq);
		}
		else
		{
			// Centers are (almost) identical -> pick a stable fallback normal
			FVector vRel = Vel - Other.Vel;
			if (vRel.SizeSquared() > KINDA_SMALL_NUMBER)
			{
				N = vRel.GetSafeNormal();
			}
			else
			{
				N = FVector::UpVector; // last resort
			}
		}

		const FVector ImpactPoint = pA0 - N * Radius;

		return SHitResult(true,
			ImpactPoint,
			N,
			0.f
		);
	}

	// ------------- EARLY OUT -------------
	const float Gap0 = FMath::Abs(f0);
	const float MaxClosing = SEarlyOut::MaxRelativeTravel(Vel, Accel, Other.Vel, Other.Accel, deltaTime);
	if (Gap0 > MaxClosing)
		return SHitResult(); // no hit
	// -------------------------------------

	float f1 = DistMinusR(deltaTime);
	if (f1 > 0.f)
		return SHitResult(); // no hit

	float tMin = 0.f;
	float tMax = deltaTime;

	for (int i = 0; i < MaxIter; ++i)
	{
		float tMid = 0.5f * (tMin + tMax);
		float fMid = DistMinusR(tMid);

		if (FMath::Abs(fMid) < Epsilon)
		{
			tMin = tMid;
			break;
		}

		if (fMid > 0.f)
			tMin = tMid;
		else
			tMax = tMid;
	}

	float TOI = 0.5f * (tMin + tMax);

	FVector pA = Center + Vel * TOI + 0.5f * Accel * TOI * TOI;
	FVector pB = Other.Center + Other.Vel * TOI + 0.5f * Other.Accel * TOI * TOI;

	FVector N = (pA - pB).GetSafeNormal();
	FVector ImpactPoint = pA - N * Radius;

	return SHitResult(true,
		ImpactPoint,
		N,
		TOI
	);
}

SHitResult SSphere::IntersectNextFrame(const SBox& Box, const float& deltaTime, const uint8 NbSubSteps) const
{
	auto HitResult = Box.IntersectNextFrame(*this, deltaTime, NbSubSteps);
	if (HitResult.bHit)
	{
		// Flip normal to point into the sphere (self)
		HitResult.ImpactNormal *= -1.0f;
	}
	return HitResult;
}

SHitResult SSphere::IntersectDuringMovement(const SSphere& StaticSphere, const FVector& Start, const FVector& End, const float& delta) const
{
	SHitResult Result;
	if (Radius <= 0.f || StaticSphere.Radius <= 0.f)
		return Result;

	const FVector D = End - Start;
	const FVector C = StaticSphere.Center;
	const float SumR = Radius + StaticSphere.Radius;

	const FVector m = Start - C;

	// ------------- EARLY OUT -------------
	const float Dist0 = m.Size();
	const float MaxTravel = D.Size();
	if (Dist0 > SumR + MaxTravel)
		return SHitResult(); // no hit
	// -------------------------------------

	const float a = FVector::DotProduct(D, D);
	const float b = 2.0f * FVector::DotProduct(m, D);
	const float c = FVector::DotProduct(m, m) - SumR * SumR;

	// Already overlapping at start
	if (c <= 0.0f)
	{
		FVector Normal = (Start - C).GetSafeNormal();
		Result = SHitResult(true,
			Start - Normal * Radius,
			Normal,
			0.0f
		);
		return Result;
	}

	// No movement
	if (a < KINDA_SMALL_NUMBER)
		return Result;

	const float Discriminant = b * b - 4.0f * a * c;
	if (Discriminant < 0.0f)
		return Result;

	const float SqrtD = FMath::Sqrt(Discriminant);

	const float t1 = (-b - SqrtD) / (2.0f * a);
	const float t2 = (-b + SqrtD) / (2.0f * a);

	float t = -1.0f;
	if (t1 >= 0.0f && t1 <= 1.0f)
		t = t1;
	else if (t2 >= 0.0f && t2 <= 1.0f)
		t = t2;

	if (t < 0.0f)
		return Result;

	const FVector ImpactCenter = Start + t * D;
	const FVector Normal = (ImpactCenter - C).GetSafeNormal();
	const FVector ImpactPoint = ImpactCenter - Normal * Radius;

	Result = SHitResult(true,
		ImpactPoint,
		Normal,
		t * delta
	);

	return Result;
}

SHitResult SSphere::IntersectDuringMovement(const SBox& StaticBox,
	const FVector& Start,
	const FVector& End, const float& delta) const
{
	// We sweep THIS sphere center from Start to End, against a STATIC OBB.
	// The OBB pose must come from StaticBox (NOT from the sphere).
	const float R = Radius;
	const FVector C0 = Start;
	const FVector C1 = End;

	// ------------- EARLY OUT -------------
	FVector ContactPoint0;
	const float Sep0 = StaticBox.SphereOBBSeparation(
		StaticBox.Rot,
		StaticBox.WorldCenter,
		C0,
		R,
		&ContactPoint0
	);
	const float MaxTravel = (C1 - C0).Size();
	if (Sep0 > MaxTravel)
		return SHitResult(); // no hit
	// -----------------------------------------

	// Early out: if Start==End, just test overlap at start
	const float lenSq = (C1 - C0).SizeSquared();
	if (lenSq <= KINDA_SMALL_NUMBER)
	{

		if (Sep0 <= 0.f)
		{
			FVector N = (C0 - ContactPoint0);
			if (!N.Normalize())
			{
				// degenerate, pick something stable
				N = StaticBox.UpVector();
			}

			return SHitResult(true,
				ContactPoint0,
				N,
				0.f
			);
		}

		return SHitResult();
	}

	// We can do conservative advancement on f(t)=separation(t),
	// with linear motion of the sphere center.
	const float MaxTime = 1.f; // we parametrize t in [0,1] for the segment

	float t = 0.f;
	float tPrev = 0.f;

	FVector ContactPoint;
	float f = StaticBox.SphereOBBSeparation(
		StaticBox.Rot,
		StaticBox.WorldCenter,
		C0,
		R,
		&ContactPoint
	);

	// If already overlapping at the start
	if (f <= 0.f)
	{
		FVector N = (C0 - ContactPoint);
		if (!N.Normalize())
			N = StaticBox.UpVector();

		return SHitResult(true,
			ContactPoint,
			N,
			0.f
		);
	}

	const FVector dC = (C1 - C0);
	const float vBound = dC.Size(); // speed bound in "units per segment"

	// Safety
	const int MaxIter = 32;
	const float Eps = 1e-4f;

	for (int it = 0; it < MaxIter; ++it)
	{
		if (f <= Eps)
			break;

		// Conservative time step: dt = f / (speed upper bound)
		if (vBound <= KINDA_SMALL_NUMBER)
			break;

		const float dt = f / vBound;
		tPrev = t;
		t = FMath::Min(t + dt, MaxTime);

		const FVector Ct = C0 + dC * t;

		f = StaticBox.SphereOBBSeparation(
			StaticBox.Rot,
			StaticBox.WorldCenter,
			Ct,
			R,
			&ContactPoint
		);

		if (t >= MaxTime)
			break;
	}

	if (f > Eps)
		return SHitResult(); // no contact over the segment

	// Impact normal: from box contact point toward sphere center
	const FVector Ct = C0 + dC * t;
	FVector N = (Ct - ContactPoint);
	if (!N.Normalize())
		N = StaticBox.UpVector();

	return SHitResult(true,
		ContactPoint,
		N,
		t * delta
	);
}




void SSphere::DrawDebug(UWorld* World)
{
	DrawDebugSphere(World, Center, Radius, 12, FColor::Red, false, -1.0f, 0, 2.0);
}

FVector SSphere::AdvancePosition(const float& t) const
{
	return Center + Vel * t + 0.5f * Accel * t * t;
}

Speed::SBox::SBox(const FVector& Center, const FVector& Extent, const FQuat& Rotation, const FVector& Velocity,
	const FVector& Acceleration, const FVector& AngularVelocity, const FVector& AngularAcceleration)
	: WorldCenter(Center), Rot(Rotation), Vel(Velocity), Accel(Acceleration), AngVel(AngularVelocity),
	AngAccel(AngularAcceleration)
{
	FVector CenterBoxCoord = TransformIntoLocalSpace(Center);
	Min = CenterBoxCoord - Extent;
	Max = CenterBoxCoord + Extent;
}

bool Speed::SBox::IsInside(const FVector& Point) const
{
	return (Point.X >= Min.X && Point.X <= Max.X &&
		Point.Y >= Min.Y && Point.Y <= Max.Y &&
		Point.Z >= Min.Z && Point.Z <= Max.Z);
}

bool Speed::SBox::IsAbsoluteInside(const FVector& Point) const
{
	// Transform the point into local space
	FVector LocalPoint = TransformIntoLocalSpace(Point);
	return IsInside(LocalPoint);
}

FVector Speed::SBox::Center() const
{
	return Min + Extent();
}

FVector Speed::SBox::AbsoluteCenter() const
{
	return WorldCenter;
}

FVector Speed::SBox::Extent() const
{
	return (Max - Min) * 0.5f;
}

const TArray<FVector>& Speed::SBox::GetAbsoluteCorners()
{
	if (AbsoluteCorners.Num() == 0)
	{
		// Compute 8 corners of the box
		FVector BoxExtent = Extent();
		FVector LocalCenter = Center();

		TArray<FVector> LocalOffsets = {
			FVector(-1, -1, -1), FVector(1, -1, -1),
			FVector(-1, 1, -1),  FVector(1, 1, -1),
			FVector(-1, -1, 1),  FVector(1, -1, 1),
			FVector(-1, 1, 1),   FVector(1, 1, 1)
		};

		for (const FVector& Offset : LocalOffsets)
		{
			FVector LocalCorner = LocalCenter + Offset * BoxExtent;
			FVector AbsoluteCorner = TransformIntoAbsoluteSpace(LocalCorner);
			AbsoluteCorners.Add(AbsoluteCorner);
		}
	}
	return AbsoluteCorners;
}

FVector Speed::SBox::TransformIntoLocalSpace(const FVector& Point) const
{
	return Rot.UnrotateVector(Point - WorldCenter);
}

FVector Speed::SBox::TransformIntoAbsoluteSpace(const FVector& Point) const
{
	return Rot.RotateVector(Point) + WorldCenter;
}

FVector Speed::SBox::ForwardVector() const
{
	return TransformIntoLocalSpace(FVector(1, 0, 0));
}

FVector Speed::SBox::RightVector() const
{
	return TransformIntoLocalSpace(FVector(0, 1, 0));
}

FVector Speed::SBox::UpVector() const
{
	return TransformIntoLocalSpace(FVector(0, 0, 1));
}

std::optional<FVector> Speed::SBox::Intersect(const SSphere& Sphere) const
{
	if (Extent() == FVector::ZeroVector || Sphere.Radius == 0.0f)
	{
		return std::nullopt;
	}
	FVector SphereCenter = TransformIntoLocalSpace(Sphere.Center);
	FVector BoxCenter = Center();
	FVector BoxExtent = Extent();
	// Calculate the distance between the sphere center and the box center
	FVector Distance = SphereCenter - BoxCenter;
	// Check if the sphere is inside the box
	if (IsInside(SphereCenter))
	{
		return SphereCenter;
	}
	FVector ClosestPoint = FVector::ZeroVector;
	float DistanceSq = 0.0f;
	ClosestPointOnOBB(Rot, AbsoluteCenter(), Sphere.Center, ClosestPoint, DistanceSq);

	// Check if the closest point is inside the sphere
	if (Sphere.IsInside(ClosestPoint))
	{
		return ClosestPoint;
	}
	return std::nullopt;
}

std::optional<FVector> Speed::SBox::Intersect(const SBox& OtherBox) const
{
	if (Extent() == FVector::ZeroVector || OtherBox.Extent() == FVector::ZeroVector)
	{
		return std::nullopt;
	}
	TArray<FVector> InsidePoints;
	for (const FVector& Corner : AbsoluteCorners)
	{
		if (OtherBox.IsAbsoluteInside(Corner))
		{
			InsidePoints.Add(Corner);
		}
	}

	if (InsidePoints.Num() > 0)
	{
		// Return the average of the inside points at the intersection point
		FVector Sum = FVector::ZeroVector;
		for (const FVector& Point : InsidePoints)
		{
			Sum += Point;
		}
		return Sum / InsidePoints.Num();
	}

	return std::nullopt;
}

// ==== Continuous CCD for sphere vs OBB with full kinematics ====
SHitResult Speed::SBox::IntersectNextFrame(const SSphere& Sphere, const float& deltaTime, const uint8 NbSubsteps) const
{
	if (deltaTime <= 0.f) return SHitResult();
	if (Extent() == FVector::ZeroVector || Sphere.Radius == 0.0f)
	{
		return SHitResult();
	}

	// Initial state at t=0
	const FVector Xb0 = AbsoluteCenter();
	const FQuat Qb0 = Rot;
	const FVector Vb0 = Vel;
	const FVector Ab0 = Accel;
	const FVector Wb0 = AngVel;
	const FVector Alphab0 = AngAccel;

	const FVector Xs0 = Sphere.Center;
	const FVector Vs0 = Sphere.Vel;
	const FVector As0 = Sphere.Accel;
	const float Rs = Sphere.Radius;

	// -------- - EARLY OUT -------------
	FVector ContactPoint0;
	const float Sep0 = SphereOBBSeparation(Qb0, Xb0, Xs0, Rs, &ContactPoint0);
	if (Sep0 > 0.f)
	{
		const float MaxLinearClosing = SEarlyOut::MaxRelativeTravel(Vb0, Ab0, Vs0, As0, deltaTime);
		const float BoxAngularRadius = MaxAngularSweepRadius(Extent());
		const float MaxRotClosing = SEarlyOut::MaxAngularTravel(Wb0, Alphab0, BoxAngularRadius, deltaTime);
		const float MaxPossibleClosing = MaxLinearClosing + MaxRotClosing;
		if (Sep0 > MaxPossibleClosing)
			return SHitResult(); // no hit possible
	}
	// ---------------------------------

	auto BoxX = [&](float t) { return AdvancePosition(Xb0, Vb0, Ab0, t); }; // Absolute box center at time t
	auto BoxQ = [&](float t) { return IntegrateRotation(Qb0, Wb0, Alphab0, t); }; // Absolute box rotation at time t
	auto SphereC = [&](float t) { return AdvancePosition(Xs0, Vs0, As0, t); }; // Absolute sphere center at time t

	const float dtStep = deltaTime / NbSubsteps;

	for (uint8 i = 0; i < NbSubsteps; ++i)
	{
		float t = i * dtStep;

		const FVector BoxPos = BoxX(t);
		const FQuat BoxRot = BoxQ(t);
		const FVector SpherePos = SphereC(t);

		FVector ClosestPoint;
		float distSq = 0.f;
		ClosestPointOnOBB(BoxRot, BoxPos, SpherePos, ClosestPoint, distSq);

		if (distSq <= FMath::Square(Rs))
		{
			const FVector Vbt = Vb0 + Ab0 * t; // Box velocity at time t
			const FVector Vst = Vs0 + As0 * t; // Sphere velocity at time t
			const FVector Wbt = Wb0 + Alphab0 * t; // Box angular velocity at time t
			auto ImpactNormal = (ClosestPoint - SpherePos).GetSafeNormal();
			return SHitResult(true, ClosestPoint, ImpactNormal, t);
		}
	}

	return SHitResult();
}

// ==== Continuous CCD for box vs box with full kinematics ====
SHitResult Speed::SBox::IntersectNextFrame(const SBox& Other, const float& deltaTime, const uint8 NbSubsteps) const
{
	if (NbSubsteps == 0 || deltaTime <= 0.f)
		return SHitResult();

	const float RA = MaxAngularSweepRadius(Extent());
	const float RB = MaxAngularSweepRadius(Other.Extent());

	const FVector CA0 = WorldCenter;
	const FVector CB0 = Other.WorldCenter;

	const FVector Delta0 = CB0 - CA0;
	const float Dist0Sq = Delta0.SizeSquared();
	const float SumR = RA + RB;

	// ------------- EARLY OUT -------------
	if (Dist0Sq > FMath::Square(SumR))
	{
		const float Dist0 = FMath::Sqrt(Dist0Sq);
		const float Gap0 = Dist0 - SumR;
		const float MaxLinearClosing = SEarlyOut::MaxRelativeTravel(Vel, Accel, Other.Vel, Other.Accel, deltaTime);
		const float MaxRotClosing = SEarlyOut::MaxAngularTravel(AngVel, AngAccel, RA + RB, deltaTime);
		const float MaxPossibleClosing = MaxLinearClosing + MaxRotClosing;
		if (Gap0 > MaxPossibleClosing)
			return SHitResult(); // no hit possible
	}

	const float dt = deltaTime / float(NbSubsteps);

	auto PoseAt = [&](const SBox& B, float t, FVector& OutC, FQuat& OutQ, FVector& OutW) -> void
		{
			OutC = Speed::SBox::AdvancePosition(B.WorldCenter, B.Vel, B.Accel, t);

			// integrate rotation using B.AngVel / B.AngAccel
			OutQ = Speed::SBox::IntegrateRotation(B.Rot, B.AngVel, B.AngAccel, t);

			// angular velocity at t (used only for SHitResult payload)
			OutW = Speed::SBox::AdvanceAngularVelocity(B.AngVel, B.AngAccel, t);
		};

	auto SAT_Overlap = [&](const FVector& CA, const FQuat& QA,
		const FVector& CB, const FQuat& QB,
		FVector& OutNormal, float& OutMinPen) -> bool
		{
			// Box half extents in local space
			const FVector EA = (Max - Min) * 0.5f; // or use Extent() if it returns half extents
			const FVector EB = (Other.Max - Other.Min) * 0.5f;

			// Orthonormal axes in world
			FVector A0 = QA.GetAxisX();
			FVector A1 = QA.GetAxisY();
			FVector A2 = QA.GetAxisZ();

			FVector B0 = QB.GetAxisX();
			FVector B1 = QB.GetAxisY();
			FVector B2 = QB.GetAxisZ();

			// Rotation matrix expressing B in A
			float R[3][3];
			float AbsR[3][3];
			const float EPS = 1e-6f;

			FVector T = CB - CA;
			// bring T into A's frame for the SAT formulas
			float tA[3] = { FVector::DotProduct(T, A0), FVector::DotProduct(T, A1), FVector::DotProduct(T, A2) };

			FVector Ax[3] = { A0, A1, A2 };
			FVector Bx[3] = { B0, B1, B2 };
			float a[3] = { EA.X, EA.Y, EA.Z };
			float b[3] = { EB.X, EB.Y, EB.Z };

			for (int i = 0; i < 3; ++i)
			{
				for (int j = 0; j < 3; ++j)
				{
					R[i][j] = FVector::DotProduct(Ax[i], Bx[j]);
					AbsR[i][j] = FMath::Abs(R[i][j]) + EPS;
				}
			}

			OutMinPen = FLT_MAX;
			OutNormal = FVector::ZeroVector;

			auto TestAxis = [&](const FVector& AxisWorld, float dist, float ra, float rb) -> bool
				{
					float sep = dist - (ra + rb);
					if (sep > 0.f)
						return false; // separating axis found

					// penetration depth along this axis
					float pen = -sep;
					if (pen < OutMinPen)
					{
						OutMinPen = pen;

						// choose normal direction from B->A
						FVector n = AxisWorld.GetSafeNormal();
						if (FVector::DotProduct(CA - CB, n) < 0.f)
							n = -n;

						OutNormal = n;
					}
					return true;
				};

			// 1) Axes A0,A1,A2
			for (int i = 0; i < 3; ++i)
			{
				float ra = a[i];
				float rb = b[0] * AbsR[i][0] + b[1] * AbsR[i][1] + b[2] * AbsR[i][2];
				float dist = FMath::Abs(tA[i]);
				if (!TestAxis(Ax[i], dist, ra, rb))
					return false;
			}

			// 2) Axes B0,B1,B2
			for (int j = 0; j < 3; ++j)
			{
				float ra = a[0] * AbsR[0][j] + a[1] * AbsR[1][j] + a[2] * AbsR[2][j];
				float rb = b[j];
				float dist = FMath::Abs(tA[0] * R[0][j] + tA[1] * R[1][j] + tA[2] * R[2][j]);
				if (!TestAxis(Bx[j], dist, ra, rb))
					return false;
			}

			// 3) Axes A_i x B_j
			for (int i = 0; i < 3; ++i)
			{
				for (int j = 0; j < 3; ++j)
				{
					FVector Axis = FVector::CrossProduct(Ax[i], Bx[j]);
					float AxisLen2 = Axis.SizeSquared();
					if (AxisLen2 < 1e-8f)
						continue;

					// SAT distance along A_i x B_j
					float dist =
						FMath::Abs(
							tA[(i + 2) % 3] * R[(i + 1) % 3][j] -
							tA[(i + 1) % 3] * R[(i + 2) % 3][j]
						);

					float ra =
						a[(i + 1) % 3] * AbsR[(i + 2) % 3][j] +
						a[(i + 2) % 3] * AbsR[(i + 1) % 3][j];

					float rb =
						b[(j + 1) % 3] * AbsR[i][(j + 2) % 3] +
						b[(j + 2) % 3] * AbsR[i][(j + 1) % 3];

					if (!TestAxis(Axis, dist, ra, rb))
						return false;
				}
			}

			return true; // no separating axis => overlap
		};

	// Substep scan
	for (uint8 step = 0; step <= NbSubsteps; ++step)
	{
		const float t = step * dt;

		FVector CA, CB, WA, WB;
		FQuat QA, QB;
		PoseAt(*this, t, CA, QA, WA);
		PoseAt(Other, t, CB, QB, WB);

		FVector Normal;
		float MinPen = 0.f;

		if (!SAT_Overlap(CA, QA, CB, QB, Normal, MinPen))
			continue;

		// Contact point heuristic:
		// closest point on A to B center and on B to A center, then average
		FVector ClosestA; float DistSqA;
		FVector ClosestB; float DistSqB;
		ClosestPointOnOBB(QA, CA, CB, ClosestA, DistSqA);
		Other.ClosestPointOnOBB(QB, CB, CA, ClosestB, DistSqB);

		FVector ContactPoint = 0.5f * (ClosestA + ClosestB);

		return SHitResult(
			true,
			ContactPoint,
			Normal,              // B -> A
			t                   // TOI in seconds
		);
	}

	return SHitResult();
}


SHitResult Speed::SBox::IntersectDuringMovement(
	const SSphere& StaticSphere,
	const FVector& Start,
	const FVector& End,
	const float& delta
) const
{
	if (Extent() == FVector::ZeroVector || StaticSphere.Radius == 0.0f)
	{
		return SHitResult();
	}
	constexpr int MaxIter = 25;
	constexpr float Eps = 1e-4f;

	const FVector Delta = End - Start;
	const FQuat Q = Rot; // rotation is fixed during sweep

	float t0 = 0.0f;
	float t1 = 1.0f;

	FVector ContactPoint0;
	float f0 = SphereOBBSeparation(
		Q,
		Start,
		StaticSphere.Center,
		StaticSphere.Radius,
		&ContactPoint0
	);

	// Already intersecting at start
	if (f0 <= 0.0f)
	{
		FVector N = (ContactPoint0 - StaticSphere.Center).GetSafeNormal();

		return SHitResult(
			true,
			ContactPoint0,
			N,
			0.0f
		);
	}

	FVector ContactPoint1;
	float f1 = SphereOBBSeparation(
		Q,
		End,
		StaticSphere.Center,
		StaticSphere.Radius,
		&ContactPoint1
	);

	// No collision during sweep
	if (f1 > 0.0f)
	{
		return SHitResult(); // bContact = false
	}

	// Root is bracketed in [t0, t1]
	float tMid = 0.0f;
	FVector ContactMid;

	for (int i = 0; i < MaxIter; ++i)
	{
		tMid = 0.5f * (t0 + t1);
		FVector X = Start + tMid * Delta;

		float fMid = SphereOBBSeparation(
			Q,
			X,
			StaticSphere.Center,
			StaticSphere.Radius,
			&ContactMid
		);

		if (FMath::Abs(fMid) < Eps)
			break;

		if (fMid > 0.0f)
		{
			t0 = tMid;
		}
		else
		{
			t1 = tMid;
		}
	}

	// Final contact
	FVector ImpactPoint = ContactMid;
	FVector Normal = (ImpactPoint - StaticSphere.Center).GetSafeNormal();

	return SHitResult(
		true,
		ImpactPoint,
		Normal,
		tMid * delta
	);
}


void Speed::SBox::DrawDebug(UWorld* World)
{
	auto BoxCenter = Center();
	auto Box = Extent();
	FVector Start = TransformIntoAbsoluteSpace(FVector(Box.X, Box.Y, Box.Z));
	FVector End = TransformIntoAbsoluteSpace(FVector(Box.X, -Box.Y, Box.Z));
	DrawDebugLine(World, Start, End, FColor::Red, false, -1.0f, 0, 2.0);

	Start = TransformIntoAbsoluteSpace(FVector(Box.X, -Box.Y, Box.Z));
	End = TransformIntoAbsoluteSpace(FVector(-Box.X, -Box.Y, Box.Z));
	DrawDebugLine(World, Start, End, FColor::Red, false, -1.0f, 0, 2.0);

	Start = TransformIntoAbsoluteSpace(FVector(-Box.X, -Box.Y, Box.Z));
	End = TransformIntoAbsoluteSpace(FVector(-Box.X, Box.Y, Box.Z));
	DrawDebugLine(World, Start, End, FColor::Red, false, -1.0f, 0, 2.0);

	Start = TransformIntoAbsoluteSpace(FVector(-Box.X, Box.Y, Box.Z));
	End = TransformIntoAbsoluteSpace(FVector(Box.X, Box.Y, Box.Z));
	DrawDebugLine(World, Start, End, FColor::Red, false, -1.0f, 0, 2.0);

	Start = TransformIntoAbsoluteSpace(FVector(Box.X, Box.Y, -Box.Z));
	End = TransformIntoAbsoluteSpace(FVector(Box.X, -Box.Y, -Box.Z));
	DrawDebugLine(World, Start, End, FColor::Red, false, -1.0f, 0, 2.0);

	Start = TransformIntoAbsoluteSpace(FVector(Box.X, -Box.Y, -Box.Z));
	End = TransformIntoAbsoluteSpace(FVector(-Box.X, -Box.Y, -Box.Z));
	DrawDebugLine(World, Start, End, FColor::Red, false, -1.0f, 0, 2.0);

	Start = TransformIntoAbsoluteSpace(FVector(-Box.X, -Box.Y, -Box.Z));
	End = TransformIntoAbsoluteSpace(FVector(-Box.X, Box.Y, -Box.Z));
	DrawDebugLine(World, Start, End, FColor::Red, false, -1.0f, 0, 2.0);

	Start = TransformIntoAbsoluteSpace(FVector(-Box.X, Box.Y, -Box.Z));
	End = TransformIntoAbsoluteSpace(FVector(Box.X, Box.Y, -Box.Z));
	DrawDebugLine(World, Start, End, FColor::Red, false, -1.0f, 0, 2.0);

	Start = TransformIntoAbsoluteSpace(FVector(Box.X, Box.Y, Box.Z));
	End = TransformIntoAbsoluteSpace(FVector(Box.X, Box.Y, -Box.Z));
	DrawDebugLine(World, Start, End, FColor::Red, false, -1.0f, 0, 2.0);

	Start = TransformIntoAbsoluteSpace(FVector(Box.X, -Box.Y, Box.Z));
	End = TransformIntoAbsoluteSpace(FVector(Box.X, -Box.Y, -Box.Z));
	DrawDebugLine(World, Start, End, FColor::Red, false, -1.0f, 0, 2.0);

	Start = TransformIntoAbsoluteSpace(FVector(-Box.X, -Box.Y, Box.Z));
	End = TransformIntoAbsoluteSpace(FVector(-Box.X, -Box.Y, -Box.Z));
	DrawDebugLine(World, Start, End, FColor::Red, false, -1.0f, 0, 2.0);

	Start = TransformIntoAbsoluteSpace(FVector(-Box.X, Box.Y, Box.Z));
	End = TransformIntoAbsoluteSpace(FVector(-Box.X, Box.Y, -Box.Z));
	DrawDebugLine(World, Start, End, FColor::Red, false, -1.0f, 0, 2.0);
}

FVector Speed::SBox::AdvancePosition(const float& t) const
{
	return AdvancePosition(WorldCenter, Vel, Accel, t);
}

FVector Speed::SBox::AdvancePosition(const FVector& Pos0, const FVector& Vel0, const FVector& Accel0, float t)
{
	return Pos0 + Vel0 * t + 0.5f * Accel0 * (t * t);
}

FVector Speed::SBox::AdvanceVelocity(const FVector& Vel0, const FVector& Accel0, float t)
{
	return Vel0 + Accel0 * t;
}

FVector Speed::SBox::AdvanceAngularVelocity(const FVector& AngVel0, const FVector& AngAccel0, float t)
{
	return AngVel0 + AngAccel0 * t;
}

FQuat Speed::SBox::IntegrateRotation(const FQuat& Q0, const FVector& W0, const FVector& Alpha, float t)
{
	// Theta(t) = w0 t + 0.5 alpha t^2 (axis-angle in world space)
	const FVector Theta = W0 * t + 0.5f * Alpha * (t * t);
	const float Angle = Theta.Size();
	if (Angle < 1e-8f)
	{
		return Q0;
	}
	const FVector Axis = Theta / Angle;
	FQuat DQ = FQuat(Axis, Angle);
	return (DQ * Q0).GetNormalized();
}

FVector Speed::SBox::ComputeInertiaTensor(const FVector& HalfExtent, const float& Mass, const float& scale)
{
	FVector Extent = 2 * HalfExtent;
	const float w = Extent.X;
	const float h = Extent.Y;
	const float d = Extent.Z;

	const float Ixx = (1.0f / 12.0f) * Mass * (h * h + d * d);
	const float Iyy = (1.0f / 12.0f) * Mass * (w * w + d * d);
	const float Izz = (1.0f / 12.0f) * Mass * (w * w + h * h);

	return scale * FVector(Ixx, Iyy, Izz);
}

FMatrix Speed::SBox::ConvertBoxInertiaToCOM(const FVector& BoxInertiaDiag, const FVector& d_cm, float MassKg)
{
	// Ibox (diag)
	FMatrix Ibox(EForceInit::ForceInitToZero);
	Ibox.M[0][0] = BoxInertiaDiag.X;
	Ibox.M[1][1] = BoxInertiaDiag.Y;
	Ibox.M[2][2] = BoxInertiaDiag.Z;
	Ibox.M[3][3] = 1.f;

	// ddT
	FMatrix ddT(EForceInit::ForceInitToZero);
	ddT.M[0][0] = d_cm.X * d_cm.X;
	ddT.M[0][1] = d_cm.X * d_cm.Y;
	ddT.M[0][2] = d_cm.X * d_cm.Z;

	ddT.M[1][0] = d_cm.Y * d_cm.X;
	ddT.M[1][1] = d_cm.Y * d_cm.Y;
	ddT.M[1][2] = d_cm.Y * d_cm.Z;

	ddT.M[2][0] = d_cm.Z * d_cm.X;
	ddT.M[2][1] = d_cm.Z * d_cm.Y;
	ddT.M[2][2] = d_cm.Z * d_cm.Z;

	// Ipar = m*(||d||^2*I - ddT)
	const float d2 = d_cm.SizeSquared();

	FMatrix d2I(EForceInit::ForceInitToZero);
	d2I.M[0][0] = d2;
	d2I.M[1][1] = d2;
	d2I.M[2][2] = d2;
	d2I.M[3][3] = 1.f;

	FMatrix Ipar = (d2I + (ddT * -1)) * MassKg;

	return Ibox + Ipar;
}


FVector Speed::SBox::ComputeLocalInverseInertiaTensor(const FVector& Extent, const float& Mass)
{
	FVector InertiaTensor = ComputeInertiaTensor(Extent, Mass);
	return FVector(
		(InertiaTensor.X > SMALL_NUMBER) ? 1.0f / InertiaTensor.X : 0.0f,
		(InertiaTensor.Y > SMALL_NUMBER) ? 1.0f / InertiaTensor.Y : 0.0f,
		(InertiaTensor.Z > SMALL_NUMBER) ? 1.0f / InertiaTensor.Z : 0.0f
	);
}

FVector Speed::SBox::InverseLocalInertiaTensor(const FVector& LocalInertiaTensor)
{
	return FVector(
		(LocalInertiaTensor.X > SMALL_NUMBER) ? 1.0f / LocalInertiaTensor.X : 0.0f,
		(LocalInertiaTensor.Y > SMALL_NUMBER) ? 1.0f / LocalInertiaTensor.Y : 0.0f,
		(LocalInertiaTensor.Z > SMALL_NUMBER) ? 1.0f / LocalInertiaTensor.Z : 0.0f
	);
}

FMatrix Speed::SBox::ComputeWorldInverseInertiaTensor(const FQuat& Rot, const FVector& LocalInvInertiaTensor)
{
	// Convert local inverse inertia tensor (diagonal matrix) to world space
	FMatrix R = FRotationMatrix::Make(Rot);
	FMatrix R_T = R.GetTransposed();
	FMatrix LocalInvInertiaMat = FMatrix::Identity;
	LocalInvInertiaMat.M[0][0] = LocalInvInertiaTensor.X;
	LocalInvInertiaMat.M[1][1] = LocalInvInertiaTensor.Y;
	LocalInvInertiaMat.M[2][2] = LocalInvInertiaTensor.Z;
	return R * LocalInvInertiaMat * R_T;
}

void Speed::SBox::ClosestPointOnOBB(const FQuat& Q, const FVector& X, const FVector& P, FVector& OutClosestWorld, float& OutDistSq) const
{
	// Transform point into the box's local space at pose (Q, X) (Q is box orientation, X is absolute box center)
	const FVector PLocal = Q.UnrotateVector(P - X);
	FVector CLocal(
		FMath::Clamp(PLocal.X, Min.X, Max.X),
		FMath::Clamp(PLocal.Y, Min.Y, Max.Y),
		FMath::Clamp(PLocal.Z, Min.Z, Max.Z)
	);
	OutClosestWorld = Q.RotateVector(CLocal) + X;
	OutDistSq = FVector::DistSquared(OutClosestWorld, P);
}

float Speed::SBox::SphereOBBSeparation(const FQuat& Q, const FVector& X, const FVector& CS, float R, FVector* OutContactPointWorld) const
{
	FVector Closest = FVector::ZeroVector;
	float DistSq = 0.f;
	ClosestPointOnOBB(Q, X, CS, Closest, DistSq);
	const float Dist = FMath::Sqrt(DistSq);
	if (OutContactPointWorld)
	{
		*OutContactPointWorld = Closest;
	}

	return Dist - R;
}

float Speed::SBox::ProjectedRadiusOnNormal(const SBox& Box, const FVector& N)
{
	const FVector Ax = Box.Rot.GetAxisX();
	const FVector Ay = Box.Rot.GetAxisY();
	const FVector Az = Box.Rot.GetAxisZ();

	// Attention: Extent() chez toi = demi-extent ou extent total ?
	// Dans ton SBox, tu stockes Min/Max local; je pars sur demi-extent.
	const FVector E = Box.Extent(); // doit renvoyer les demi-extents (Hx,Hy,Hz)

	return
		E.X * FMath::Abs(FVector::DotProduct(Ax, N)) +
		E.Y * FMath::Abs(FVector::DotProduct(Ay, N)) +
		E.Z * FMath::Abs(FVector::DotProduct(Az, N));
}

float Speed::SBox::EstimateOBBOverlapAlongNormal(const SBox& A, const SBox& B, const FVector& N_BtoA)
{
	const FVector N = N_BtoA.GetSafeNormal();
	if (N.IsNearlyZero()) return 0.f;

	const float rA = ProjectedRadiusOnNormal(A, N);
	const float rB = ProjectedRadiusOnNormal(B, N);

	const FVector d = A.WorldCenter - B.WorldCenter;
	const float dist = FMath::Abs(FVector::DotProduct(d, N));

	return (rA + rB) - dist; // >0 => overlap
}

int Speed::Interpolate(const int& min, const int& max, const float& factor)
{
	float minFloat = min;
	float maxFloat = max;
	return FMath::RoundToInt(FMath::Lerp(minFloat, maxFloat, factor));
}

FVector Speed::RoundVectorToNetQuantize(const FVector& vector, const unsigned int& base)
{
	float X = static_cast<float>(FMath::RoundToInt(vector.X * base)) / base;
	float Y = static_cast<float>(FMath::RoundToInt(vector.Y * base)) / base;
	float Z = static_cast<float>(FMath::RoundToInt(vector.Z * base)) / base;
	return FVector(X, Y, Z);
}

FQuat Speed::RoundQuatToNetQuantize(const FQuat& quat, const FQuat& QRefForHemisphere)
{
	FRotator R = quat.Rotator();
	R.Normalize();
	R.Pitch = FRotator::NormalizeAxis(R.Pitch);
	R.Yaw = FRotator::NormalizeAxis(R.Yaw);
	R.Roll = FRotator::NormalizeAxis(R.Roll);

	const uint16 SP = FRotator::CompressAxisToShort(R.Pitch);
	const uint16 SY = FRotator::CompressAxisToShort(R.Yaw);
	const uint16 SR = FRotator::CompressAxisToShort(R.Roll);

	const float Pitch = FRotator::DecompressAxisFromShort(SP);
	const float Yaw = FRotator::DecompressAxisFromShort(SY);
	const float Roll = FRotator::DecompressAxisFromShort(SR);

	FQuat Out = FRotator(Pitch, Yaw, Roll).Quaternion();
	Out.Normalize();

	if (QuatDot(Out, QRefForHemisphere) < 0.f)
	{
		Out.X *= -1; Out.Y *= -1; Out.Z *= -1; Out.W *= -1;
	}
	return Out;
}

FRotator Speed::CompressAllAxisRotator(const FRotator& rotator)
{
	FRotator compressedRot = rotator;
	compressedRot.Pitch = FRotator::CompressAxisToShort(rotator.Pitch);
	compressedRot.Yaw = FRotator::CompressAxisToShort(rotator.Yaw);
	compressedRot.Roll = FRotator::CompressAxisToShort(rotator.Roll);
	return compressedRot;
}

FRotator Speed::DecompressAllAxisRotator(const FRotator& compressedRotator)
{
	FRotator rot = compressedRotator;
	rot.Pitch = FRotator::DecompressAxisFromShort(compressedRotator.Pitch);
	rot.Yaw = FRotator::DecompressAxisFromShort(compressedRotator.Yaw);
	rot.Roll = FRotator::DecompressAxisFromShort(compressedRotator.Roll);
	return rot;
}

Speed::FKinematicState Speed::FKinematicState::Integrate(const float& delta) const
{
	Speed::FKinematicState NewState;
	NewState.Location = Speed::SBox::AdvancePosition(Location, Velocity, Acceleration, delta);
	NewState.Velocity = Speed::SBox::AdvanceVelocity(Velocity, Acceleration, delta);
	NewState.Acceleration = Acceleration;
	NewState.Rotation = Speed::SBox::IntegrateRotation(Rotation, AngularVelocity, AngularAcceleration, delta);
	NewState.AngularVelocity = Speed::SBox::AdvanceAngularVelocity(AngularVelocity, AngularAcceleration, delta);
	NewState.AngularAcceleration = AngularAcceleration;
	return NewState;
}

bool Speed::FKinematicState::Serialize(FArchive& Ar)
{
	Ar << Location;
	Ar << Velocity;
	Ar << Acceleration;
	Ar << Rotation;
	Ar << AngularVelocity;
	Ar << AngularAcceleration;
	return true;
}
void Speed::FKinematicState::Quantize(const FQuat& PrevRotation)
{
	Location = RoundVectorToNetQuantize(Location, 100);
	Velocity = RoundVectorToNetQuantize(Velocity, 100);
	// Acceleration = RoundVectorToNetQuantize(Acceleration, 1);
	Rotation = RoundQuatToNetQuantize(Rotation, PrevRotation);
	AngularVelocity = RoundVectorToNetQuantize(AngularVelocity, 1e4);
	// AngularAcceleration = RoundVectorToNetQuantize(AngularAcceleration, 100);
}

bool Speed::FKinematicState::NetSerialize(FArchive& Ar, class UPackageMap* Map, bool& bOutSuccess)
{
	Ar << Location;
	Ar << Velocity;
	// Ar << Acceleration;
	if (Ar.IsSaving())
	{
		FRotator R = Rotation.Rotator();          // quat -> rotator
		R.SerializeCompressedShort(Ar);           // 3x int16
	}
	else
	{
		FRotator R;
		R.SerializeCompressedShort(Ar);           // read 3x int16
		Rotation = R.Quaternion();                // rotator -> quat
		Rotation.Normalize();
	}
	Ar << AngularVelocity;
	// Ar << AngularAcceleration;
	bOutSuccess = true;
	return true;
}

FString Speed::FKinematicState::ToString() const
{
	return FString::Printf(TEXT("Location: %s, Velocity: %s, Acceleration: %s, Rotation: %s, AngularVelocity: %s, AngularAcceleration: %s"),
		*Location.ToString(), *Velocity.ToString(), *Acceleration.ToString(),
		*Rotation.ToString(), *AngularVelocity.ToString(), *AngularAcceleration.ToString());
}

Speed::FKinematicState Speed::Lerp(const FKinematicState& A, const FKinematicState& B, float Alpha)
{
	Speed::FKinematicState Result;
	Result.Location = FMath::Lerp(A.Location, B.Location, Alpha);
	Result.Velocity = FMath::Lerp(A.Velocity, B.Velocity, Alpha);
	Result.Acceleration = FMath::Lerp(A.Acceleration, B.Acceleration, Alpha);
	Result.Rotation = FQuat::Slerp(A.Rotation, B.Rotation, Alpha);
	Result.AngularVelocity = FMath::Lerp(A.AngularVelocity, B.AngularVelocity, Alpha);
	Result.AngularAcceleration = FMath::Lerp(A.AngularAcceleration, B.AngularAcceleration, Alpha);
	return Result;
}

FVector Speed::QuantizeUnitNormal(const FVector& n, float q)
{
	FVector nCopy = n;
	if (!nCopy.Normalize())
	{
		return FVector::UpVector;
	}
	nCopy.X = FMath::RoundToFloat(nCopy.X / q) * q;
	nCopy.Y = FMath::RoundToFloat(nCopy.Y / q) * q;
	nCopy.Z = FMath::RoundToFloat(nCopy.Z / q) * q;
	nCopy.Normalize();
	return nCopy;
}

bool SImpulseSolver::ComputeCollisionImpulse(
	const FVector& ContactPoint,
	FVector Normal,

	const FKinematicState& KA, float MassA,
	const FMatrix& InvInertiaA,

	const FKinematicState& KB, float MassB,
	float RadiusB,

	float Restitution,

	FVector& OutImpulseA,
	FVector& OutImpulseB
)
{
	OutImpulseA = FVector::ZeroVector;
	OutImpulseB = FVector::ZeroVector;

	// Safety check
	Normal = Normal.GetSafeNormal();
	if (Normal.IsNearlyZero())
		return false;

	//-------------------------------------------------------
	// 1) Positions & velocities at TOI
	//-------------------------------------------------------
	const FVector PosA = KA.Location;
	const FVector VelA = KA.Velocity;
	const FVector AngA = KA.AngularVelocity;

	const FVector PosB = KB.Location;
	const FVector VelB = KB.Velocity;
	const FVector AngB = KB.AngularVelocity;

	//-------------------------------------------------------
	// 2) Offsets from centers of mass
	//-------------------------------------------------------
	const FVector rA = ContactPoint - PosA;
	const FVector rB = ContactPoint - PosB;

	//-------------------------------------------------------
	// 3) Relative velocity at contact
	//-------------------------------------------------------
	const FVector vA = VelA + FVector::CrossProduct(AngA, rA);
	const FVector vB = VelB + FVector::CrossProduct(AngB, rB);
	const FVector vRel = vB - vA;

	const float vRelN = FVector::DotProduct(vRel, Normal);

	// No impulse needed if objects are separating
	if (vRelN < 0.f)
	{
		// UE_LOG(LogTemp, Warning, TEXT("[CCD]No impulse needed, objects are separating"));
		return false;
	}

	//-------------------------------------------------------
	// 4) Masses and inertias
	//-------------------------------------------------------
	const float invA = 1.f / MassA;
	const float invB = 1.f / MassB;

	// Ball inertia: solid sphere
	const float InertiaB = (2.f / 5.f) * MassB * RadiusB * RadiusB;
	const float InvInertiaB = (InertiaB > KINDA_SMALL_NUMBER) ? 1.f / InertiaB : 0.f;

	//-------------------------------------------------------
	// 5) Effective mass denominator
	//-------------------------------------------------------
	const FVector rA_x_n = FVector::CrossProduct(rA, Normal);
	const FVector rB_x_n = FVector::CrossProduct(rB, Normal);

	const FVector termA = InvInertiaA.TransformVector(rA_x_n);
	const float angA = FVector::DotProduct(rA_x_n, termA);

	const float angB = InvInertiaB * FVector::DotProduct(rB_x_n, rB_x_n);

	float denom = invA + invB + angA + angB;
	if (denom < KINDA_SMALL_NUMBER)
		return false;

	//-------------------------------------------------------
	// 6) Impulse magnitude
	//-------------------------------------------------------
	const float Jmag = -(1.f + Restitution) * vRelN / denom;

	FVector J = Jmag * Normal;

	//-------------------------------------------------------
	// 7) Output impulses
	//-------------------------------------------------------
	OutImpulseA = -J;   // applied to A
	OutImpulseB = J;   // applied to B
	return true;
}

bool SImpulseSolver::ComputeCollisionImpulse(
	const FVector& ContactPoint,
	FVector Normal,                         // MUST point from B -> A
	const FKinematicState& KA, float MassA,
	const FMatrix& InvInertiaA,
	const FKinematicState& KB, float MassB,
	const FMatrix& InvInertiaB,
	float Restitution,
	FVector& OutImpulseA,                   // impulse applied to A  = -J n
	FVector& OutImpulseB                    // impulse applied to B  =  J n
)
{
	OutImpulseA = FVector::ZeroVector;
	OutImpulseB = FVector::ZeroVector;

	Normal = Normal.GetSafeNormal();
	if (Normal.IsNearlyZero())
		return false;

	const float InvMassA = (MassA > KINDA_SMALL_NUMBER) ? 1.f / MassA : 0.f;
	const float InvMassB = (MassB > KINDA_SMALL_NUMBER) ? 1.f / MassB : 0.f;

	// Lever arms
	const FVector rA = ContactPoint - KA.Location;
	const FVector rB = ContactPoint - KB.Location;

	// Velocities at contact
	const FVector vA = KA.Velocity + FVector::CrossProduct(KA.AngularVelocity, rA);
	const FVector vB = KB.Velocity + FVector::CrossProduct(KB.AngularVelocity, rB);

	const FVector vRel = vA - vB;
	const float vRelN = FVector::DotProduct(vRel, Normal);

	// Separating -> no impulse
	if (vRelN >= 0.f)
		return false;

	// Compute angular terms: n · ( (InvI * (r×n)) × r )
	const FVector rAxn = FVector::CrossProduct(rA, Normal);
	const FVector rBxn = FVector::CrossProduct(rB, Normal);

	const FVector invIA_rAxn = InvInertiaA.TransformVector(rAxn);
	const FVector invIB_rBxn = InvInertiaB.TransformVector(rBxn);

	const float angA = FVector::DotProduct(Normal, FVector::CrossProduct(invIA_rAxn, rA));
	const float angB = FVector::DotProduct(Normal, FVector::CrossProduct(invIB_rBxn, rB));

	float denom = InvMassA + InvMassB + angA + angB;
	if (denom <= KINDA_SMALL_NUMBER)
		return false;

	Restitution = FMath::Clamp(Restitution, 0.f, 1.f);

	// Impulse magnitude
	const float j = -(1.f + Restitution) * vRelN / denom;

	const FVector Jn = j * Normal;

	// Contract: OutImpulseA = -J n, OutImpulseB = J n
	OutImpulseA = -Jn;
	OutImpulseB = Jn;

	return true;
}

bool SImpulseSolver::ComputeCollisionImpulse(
	const FVector& ContactPoint,
	const FVector& Normal,

	const FKinematicState& KA,
	float MassA,
	const FMatrix& InvInertiaA,

	const FKinematicState& KB,
	float MassB,
	const FMatrix& InvInertiaB,

	float Restitution,
	float Friction,

	FVector& OutImpulseA,
	FVector& OutImpulseB,
	const float& ImpactThreshold
)
{
	OutImpulseA = FVector::ZeroVector;
	OutImpulseB = FVector::ZeroVector;

	const FVector N = Normal.GetSafeNormal();
	if (N.IsNearlyZero())
		return false;

	// -------------------------------------------------------
	// 1) Inverse masses
	// -------------------------------------------------------
	const float InvMassA = (MassA > KINDA_SMALL_NUMBER) ? 1.f / MassA : 0.f;
	const float InvMassB = (MassB > KINDA_SMALL_NUMBER) ? 1.f / MassB : 0.f;

	// -------------------------------------------------------
	// 2) Lever arms
	// -------------------------------------------------------
	const FVector rA = ContactPoint - KA.Location;
	const FVector rB = ContactPoint - KB.Location;

	// -------------------------------------------------------
	// 3) Velocities at contact
	// -------------------------------------------------------
	const FVector vA = KA.Velocity + FVector::CrossProduct(KA.AngularVelocity, rA);
	const FVector vB = KB.Velocity + FVector::CrossProduct(KB.AngularVelocity, rB);

	// Relative velocity: THIS relative to OTHER
	const FVector vRel = vA - vB;

	const float vRelN = FVector::DotProduct(vRel, N);

	// -------------------------------------------------------
	// 4) Separating -> no impulse
	// -------------------------------------------------------
	if (vRelN >= 0.f)
		return false;

	// -------------------------------------------------------
	// 5) Normal effective mass
	// -------------------------------------------------------
	const FVector rAxN = FVector::CrossProduct(rA, N);
	const FVector rBxN = FVector::CrossProduct(rB, N);

	const FVector invIA_rAxN = InvInertiaA.TransformVector(rAxN);
	const FVector invIB_rBxN = InvInertiaB.TransformVector(rBxN);

	const float angA = FVector::DotProduct(N, FVector::CrossProduct(invIA_rAxN, rA));
	const float angB = FVector::DotProduct(N, FVector::CrossProduct(invIB_rBxN, rB));

	float denomN = InvMassA + InvMassB + angA + angB;
	if (denomN <= KINDA_SMALL_NUMBER)
		return false;

	// -------------------------------------------------------
	// 6) Restitution gating (important)
	// -------------------------------------------------------
	// Disable restitution for resting / sliding contacts
	const float RestitutionVelThreshold = ImpactThreshold; // cm/s
	float e = (FMath::Abs(vRelN) > RestitutionVelThreshold)
		? FMath::Clamp(Restitution, 0.f, 1.f)
		: 0.f;

	// -------------------------------------------------------
	// 7) Normal impulse
	// -------------------------------------------------------
	const float jn = -(1.f + e) * vRelN / denomN;
	const FVector Jn = jn * N;

	// -------------------------------------------------------
	// 8) Tangential impulse (friction)
	// -------------------------------------------------------
	FVector Jt = FVector::ZeroVector;

	const FVector vRelT = vRel - vRelN * N;
	const float vRelTmag = vRelT.Size();

	if (vRelTmag > KINDA_SMALL_NUMBER && Friction > 0.f)
	{
		const FVector T = vRelT / vRelTmag;

		const FVector rAxT = FVector::CrossProduct(rA, T);
		const FVector rBxT = FVector::CrossProduct(rB, T);

		const FVector invIA_rAxT = InvInertiaA.TransformVector(rAxT);
		const FVector invIB_rBxT = InvInertiaB.TransformVector(rBxT);

		const float angTA = FVector::DotProduct(T, FVector::CrossProduct(invIA_rAxT, rA));
		const float angTB = FVector::DotProduct(T, FVector::CrossProduct(invIB_rBxT, rB));

		float denomT = InvMassA + InvMassB + angTA + angTB;
		if (denomT > KINDA_SMALL_NUMBER)
		{
			float jt = -vRelTmag / denomT;

			// Coulomb clamp
			const float jtMax = Friction * jn;
			jt = FMath::Clamp(jt, -jtMax, jtMax);

			Jt = jt * T;
		}
	}

	// -------------------------------------------------------
	// 9) Total impulse
	// -------------------------------------------------------
	const FVector J = Jn + Jt;

	OutImpulseA = J;
	OutImpulseB = -J;
	// UE_LOG(LogTemp, Warning,
	//	TEXT("vRelN=%.3f InvMassA=%.6f InvMassB=%.6f angA=%.6f angB=%.6f denomN=%.6f jn=%.3f"),
	//	vRelN, InvMassA, InvMassB, angA, angB, denomN, jn);
	return true;
}

