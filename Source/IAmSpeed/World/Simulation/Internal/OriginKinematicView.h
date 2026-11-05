#pragma once
#include "IAmSpeed/Base/Kinematic.h"
namespace Speed::NativeKinematics
{
struct FOriginKinematicView
	{
		const SKinematic& COM;
		const FVector OriginToCOM;
		FOriginKinematicView(const SKinematic& State, const FVector& LocalCOM)
			: COM(State), OriginToCOM(State.Rotation.RotateVector(LocalCOM)) {}
		FVector Location() const { return COM.Location - OriginToCOM; }
		FVector Velocity() const { return COM.Velocity - FVector::CrossProduct(COM.AngularVelocity, OriginToCOM); }
		FVector Acceleration() const
		{
			return COM.Acceleration - FVector::CrossProduct(COM.AngularAcceleration, OriginToCOM)
				- FVector::CrossProduct(COM.AngularVelocity, FVector::CrossProduct(COM.AngularVelocity, OriginToCOM));
		}
	};
}
