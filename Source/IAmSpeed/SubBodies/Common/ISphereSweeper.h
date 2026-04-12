#pragma once
#include "ISweeper.h"

class IAMSPEED_API ISphereSweeper : public ISweeper
{
protected:
	bool SweepVsGround(UWorld* World, SHitResult& OutHit, const float& DeltaTime, float& OutTOI) override;
	bool SweepVsBoxes(UWorld* World, SHitResult& OutHit, const float& DeltaTime, float& OutTOI) override;
	bool SweepVsSpheres(UWorld* World, SHitResult& OutHit, const float& Delta, float& OutTOI) override;
	bool SweepVsWheels(UWorld* World, SHitResult& OutHit, const float& Delta, float& OutTOI) override;

	virtual SSphere MakeSphere() const;

	virtual float GetRadiusWithMargin() const = 0;
public:
	virtual ~ISphereSweeper() = default;
};