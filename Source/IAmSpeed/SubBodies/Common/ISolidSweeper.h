#pragma once

#include "ISweeper.h"

class USolidSubBody;
class IAMSPEED_API ISolidSweeper : public ISweeper
{
public:
    virtual const USolidSubBody* GetSolidSweepBody() const { return nullptr; }
	bool InternalSweep(UWorld* World, const FVector& Start, const FVector& End, SHitResult& OutHit, const float& delta) override;
};
