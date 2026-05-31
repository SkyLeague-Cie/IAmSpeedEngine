#pragma once

#include "ISweeper.h"

class ISolidSweeper : public ISweeper
{
public:
	bool InternalSweep(UWorld* World, const FVector& Start, const FVector& End, SHitResult& OutHit, const float& delta) override;
};
