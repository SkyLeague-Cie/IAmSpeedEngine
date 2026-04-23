#pragma once
#include "IAmSpeed/Base/SUtils.h"

class UBoxSubBody;
class USphereSubBody;
class USWheelSubBody;

class ISweeper
{
public:
	virtual bool InternalSweep(UWorld* World, const FVector& Start, const FVector& End, SHitResult& OutHit, const float& delta) = 0;
	virtual const Speed::FKinematicState& GetKinematicState() const = 0;

	virtual FCollisionShape GetCollisionShape(float Inflation = 0.0f) const = 0;
	virtual ECollisionChannel GetCollisionChannel() const = 0;
	virtual const FCollisionResponseParams& GetResponseParams() const = 0;
	virtual FCollisionQueryParams BuildTraceParams() const = 0;
protected:
	virtual bool SweepVsGround(UWorld* World, SHitResult& OutHit, const float& DeltaTime, float& OutTOI) = 0;
	virtual bool SweepVsBoxes(UWorld* World, SHitResult& OutHit, const float& DeltaTime, float& OutTOI) = 0;
	virtual bool SweepVsSpheres(UWorld* World, SHitResult& OutHit, const float& Delta, float& OutTOI) = 0;
	virtual bool SweepVsWheels(UWorld* World, SHitResult& OutHit, const float& Delta, float& OutTOI) = 0;

	virtual const TArray<TWeakObjectPtr<UBoxSubBody>> GetExternalBoxSubBodies() const = 0;
	virtual const TArray<TWeakObjectPtr<USphereSubBody>> GetExternalSphereSubBodies() const = 0;
	virtual const TArray<TWeakObjectPtr<USWheelSubBody>> GetExternalWheelSubBodies() const = 0;
	virtual bool ComponentHasBeenIgnored(const UPrimitiveComponent& OtherComp) const = 0;
public:
	virtual ~ISweeper() = default;
};