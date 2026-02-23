#pragma once

#pragma warning(push, 0)
#include "CoreMinimal.h"
#pragma warning(pop)

#include <optional>
#include <limits>

#include "SHitResult.h"
#include "Kinematic.h"

class UWorld;
class USSubBody;


namespace Speed
{
	struct SBox;

	/**
	* Geometric Sphere Collision
	* This class is used to handle sphere collision in the game.
	*/
	struct SSphere
	{
		SSphere(const FVector& InCenter, const float& InRadius, const FVector& Velocity, const FVector& Acceleration);

		bool IsInside(const FVector& Point) const;
		SHitResult IntersectNextFrame(const SSphere& Sphere, const float& deltaTime) const;
		SHitResult IntersectNextFrame(const SBox& Box, const float& deltaTime, const uint8 nbSubSteps) const;
		// Sweep this sphere from Start to End against a static sphere
		SHitResult IntersectDuringMovement(const SSphere& StaticSphere, const FVector& Start, const FVector& End, const float& deltaTime) const;
		SHitResult IntersectDuringMovement(const SBox& StaticBox, const FVector& Start, const FVector& End, const float& deltaTime) const;


		void DrawDebug(UWorld* World);

		// Attributes
		FVector Center = FVector::ZeroVector;
		float Radius = 0.0;

		// Kinematics
		FVector Vel = FVector::ZeroVector; // velocity in absolute space
		FVector Accel = FVector::ZeroVector; // acceleration in absolute space

		// compute new position after time t with constant velocity Vel and constant acceleration Accel.
		FVector AdvancePosition(const float& t) const;
	};

	/**
	* Geometric Box Collision
	* This class is used to handle box collision in the game.
	*/
	struct SBox
	{
		SBox(const FVector& Center, const FVector& Extent, const FQuat& Rotation, const FVector& Velocity, const FVector& Acceleration,
			const FVector& AngularVelocityRadian, const FVector& AngularAccelerationRadian);

		// Check if Point (in local space) is inside this box
		bool IsInside(const FVector& Point) const;
		// Check if Point (in absolute space) is inside this box
		bool IsAbsoluteInside(const FVector& Point) const;
		FVector Center() const;
		FVector AbsoluteCenter() const;
		FVector Extent() const;
		const TArray<FVector>& GetAbsoluteCorners();

		// Return the impact point with a sphere if it exists
		std::optional<FVector> Intersect(const SSphere& Sphere) const;
		// Return the impact point with another box if it exists
		std::optional<FVector> Intersect(const SBox& OtherBox) const;
		// Return the impact point in the next frame with a sphere if it exists
		SHitResult IntersectNextFrame(const SSphere& Sphere, const float& deltaTime, const uint8 NbSubsteps) const;
		// Return the impact point in the next frame with a box if it exists
		SHitResult IntersectNextFrame(const SBox& Box, const float& deltaTime, const uint8 NbSubsteps) const;
		// Sweep this box from Start to End against a static sphere
		SHitResult IntersectDuringMovement(const SSphere& StaticSphere, const FVector& Start, const FVector& End, const float& DeltaTime) const;


		FVector TransformIntoLocalSpace(const FVector& Point) const;
		FVector TransformIntoAbsoluteSpace(const FVector& Point) const;
		FVector ForwardVector() const;
		FVector RightVector() const;
		FVector UpVector() const;

		void DrawDebug(UWorld* World);

		FVector WorldCenter = FVector::ZeroVector; // Absolute Center of the Box
		FVector Min = FVector::ZeroVector; // "minimum" vector in local space
		FVector Max = FVector::ZeroVector; // "maximum" vector in local space
		FQuat Rot = FQuat::Identity;

		// Kinematics
		FVector Vel = FVector::ZeroVector; // velocity in absolute space
		FVector Accel = FVector::ZeroVector; // acceleration in absolute space
		FVector AngVel = FVector::ZeroVector; // angular velocity in rad/s
		FVector AngAccel = FVector::ZeroVector; // angular acceleration in rad/s^2

		TArray<FVector> AbsoluteCorners;

		// Signed "separation" between sphere and OBB at pose (Q, X) for a given sphere center CS and radius R.
		// Negative => penetration; Zero => touching; Positive => separated.
		float SphereOBBSeparation(const FQuat& Q, const FVector& X, const FVector& CS, float R, FVector* OutContactPointWorld = nullptr) const;

		// Helpers
		// compute new position after time t with constant velocity Vel and constant acceleration Accel.
		FVector AdvancePosition(const float& t) const;
		// Compute new position after time t with initial position (Absolute Center) Pos0, initial velocity Vel0 and constant acceleration Accel0.
		static FVector AdvancePosition(const FVector& Pos0, const FVector& Vel0, const FVector& Accel0, float t);
		// Compute new velocity after time t with initial velocity Vel0 and constant acceleration Accel0.
		static FVector AdvanceVelocity(const FVector& Vel0, const FVector& Accel0, float t);
		// Compute new angular velocity after time t with initial angular velocity AngVel0 and constant angular acceleration AngAccel0.
		static FVector AdvanceAngularVelocity(const FVector& AngVelRadian0, const FVector& AngAccelRadian0, float t);
		// Compute new rotation after time t with initial rotation Q0, initial angular velocity W0 and constant angular acceleration Alpha.
		static FQuat IntegrateRotation(const FQuat& Q0, const FVector& W0, const FVector& Alpha, float t);

		static FVector ComputeInertiaTensor(const FVector& HalfExtent, const float& Mass, const float& scale = 1.0);
		static FMatrix ConvertBoxInertiaToCOM(const FVector& BoxInertiaDiag, const FVector& d_cm, float MassKg);
		static FVector ComputeLocalInverseInertiaTensor(const FVector& Extent, const float& Mass);
		static FVector InverseLocalInertiaTensor(const FVector& LocalInertiaTensor);
		static FMatrix ComputeWorldInverseInertiaTensor(const FQuat& Rot, const FVector& LocalInvInertiaTensor);
		static float ProjectedRadiusOnNormal(const SBox& Box, const FVector& N);
		static float EstimateOBBOverlapAlongNormal(const SBox& A, const SBox& B, const FVector& N_BtoA);

		static FORCEINLINE float MaxAngularSweepRadius(const FVector& BoxExtent)
		{
			return BoxExtent.Size();
		}
	private:
		// Compute closest point on OBB (with pose Rot, AbsCenter) to a given world point P.
		// Returns closest point (OutClosestWorld) in world space and squared distance (OutDistSq).
		void ClosestPointOnOBB(const FQuat& Rot, const FVector& AbsCenter, const FVector& P, FVector& OutClosestWorld, float& OutDistSq) const;
	};

	// Linear interpolation between min and max with factor in [0, 1]
	int Interpolate(const int& min, const int& max, const float& factor);
	FORCEINLINE float QuatDot(const FQuat& A, const FQuat& B)
	{
		return A.X * B.X + A.Y * B.Y + A.Z * B.Z + A.W * B.W;
	}
	FVector RoundVectorToNetQuantize(const FVector& vector, const unsigned int& base);
	FQuat RoundQuatToNetQuantize(const FQuat& quat, const FQuat& QRefForHemisphere);
	FRotator CompressAllAxisRotator(const FRotator& rotator);
	FRotator DecompressAllAxisRotator(const FRotator& compressedRotator);

	FVector QuantizeUnitNormal(const FVector& n, float q = 1e-3f);

	struct SEarlyOut
	{
		static FORCEINLINE float MaxTravel1D(const FVector& Vel, const FVector& Accel, const float& Dt)
		{
			// Compute the maximum travel along one axis with initial velocity Vel, constant acceleration Accel and delta time Dt.
			// This is used to early out of collision checks when the distance between two objects is greater than the maximum travel.
			// The formula is: d = v * t + 0.5 * a * t^2
			return Vel.Size() * Dt + 0.5f * Accel.Size() * Dt * Dt;
		}

		static FORCEINLINE float MaxRelativeTravel(const FVector& VelA, const FVector& AccelA, const FVector& VelB, const FVector& AccelB, const float& Dt)
		{
			// Compute the maximum relative travel between two objects A and B with initial velocities VelA and VelB, constant accelerations AccelA and AccelB and delta time Dt.
			// This is used to early out of collision checks when the distance between two objects is greater than the maximum relative travel.
			return MaxTravel1D(VelA - VelB, AccelA - AccelB, Dt);
		}

		static FORCEINLINE float MaxAngularTravel(const FVector& AngVelRadian, const FVector& AngAccelRadian, const float& Radius, const float& Dt)
		{
			// Compute the maximum angular travel with initial angular velocity AngVelRadian, constant angular acceleration AngAccelRadian and delta time Dt.
			// This is used to early out of collision checks when the distance between two objects is greater than the maximum angular travel.
			return (AngVelRadian.Size() * Dt + 0.5f * AngAccelRadian.Size() * Dt * Dt) * Radius;
		}
	};

	struct SImpulseSolver
	{
		// Computes a symmetric collision impulse between object A and B.
		// Returns false if no impulse must be applied (separating or degenerate case).
		static bool ComputeCollisionImpulse(
			const FVector& ContactPoint,
			FVector Normal,                                 // MUST point from B -> A

			const FKinematicState& KA, float MassA,
			const FMatrix& InvInertiaA,                     // inverse inertia tensor (world-space) for A

			const FKinematicState& KB, float MassB,
			float RadiusB,                                  // used for solid-sphere inertia

			float Restitution,

			FVector& OutImpulseA,                           // impulse applied to A  = -J n
			FVector& OutImpulseB                            // impulse applied to B  = J n
		);

		// Computes a symmetric collision impulse between object A and B.
		// Returns false if no impulse must be applied (separating or degenerate case).
		static bool ComputeCollisionImpulse(
			const FVector& ContactPoint,
			FVector Normal,                                 // MUST point from B -> A
			const FKinematicState& KA, float MassA,
			const FMatrix& InvInertiaA,                     // inverse inertia tensor (world-space) for A
			const FKinematicState& KB, float MassB,
			const FMatrix& InvInertiaB,                     // inverse inertia tensor (world-space) for B
			float Restitution,
			FVector& OutImpulseA,                           // impulse applied to A  = -J n
			FVector& OutImpulseB                            // impulse applied to B  = J n
		);

		static bool ComputeCollisionImpulse(
			const FVector& ContactPoint,
			const FVector& Normal,                 // MUST point from Other -> This

			// THIS body
			const FKinematicState& KA,
			float MassA,
			const FMatrix& InvInertiaA,

			// OTHER body
			const FKinematicState& KB,
			float MassB,
			const FMatrix& InvInertiaB,

			float Restitution,                      // [0..1]
			float Friction,                         // Coulomb ?

			// Output
			FVector& OutImpulseA,                   // impulse to apply to THIS
			FVector& OutImpulseB,                   // impulse to apply to OTHER
			const float& ImpactThreshold			// impact threshold (normal velocity below which restitution is disabled, in cm/s)
		);
	};

	struct SimUtils
	{
		// Default physics FPS used for interpolation and extrapolation when the engine fixed delta time is not set (e.g. in editor or when using variable time step).
		static constexpr unsigned short int DefaultEnginePhysicsFPS = 120;
		// Compute the number of frames to simulate for a given simulation time and FPS, adding 1 to be aligned with netcode local frame counting (which starts at 1).
		static int32 ComputeNumFrameFromSimTime(const unsigned int& FPS, const float& SimTime)
		{
			return FMath::CeilToInt32(SimTime * FPS) + 1; // +1 to be aligned with netcode local frame counting (starts at 1)
		}

		static unsigned short int ComputePhysicsFPS(const float& EngineFixedDeltaTime)
		{
			if (EngineFixedDeltaTime > 0.0)
			{
				return static_cast<unsigned short int>(1.0f / EngineFixedDeltaTime);
			}
			return DefaultEnginePhysicsFPS;
		}
	};
	
}

using SSBox = Speed::SBox;
using SSphere = Speed::SSphere;