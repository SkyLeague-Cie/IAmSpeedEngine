#pragma once
#include "ISweeper.h"

class IAMSPEED_API IBoxSweeper : public ISweeper
{
protected:
	bool SweepVsGround(UWorld* World, SHitResult& OutHit, const float& DeltaTime, float& OutTOI) override;
	bool SweepVsBoxes(UWorld* World, SHitResult& OutHit, const float& DeltaTime, float& OutTOI) override;
	bool SweepVsSpheres(UWorld* World, SHitResult& OutHit, const float& Delta, float& OutTOI) override;
	bool SweepVsWheels(UWorld* World, SHitResult& OutHit, const float& Delta, float& OutTOI) override;

	virtual SSBox MakeBox() const;

	virtual FVector GetBoxExtent() const = 0;
public:
	virtual ~IBoxSweeper() = default;
};
