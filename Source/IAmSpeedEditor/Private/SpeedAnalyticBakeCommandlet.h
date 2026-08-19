#pragma once

#include "CoreMinimal.h"
#include "Commandlets/Commandlet.h"
#include "SpeedAnalyticBakeCommandlet.generated.h"

UCLASS()
class USpeedAnalyticBakeCommandlet : public UCommandlet
{
	GENERATED_BODY()

public:
	USpeedAnalyticBakeCommandlet();
	virtual int32 Main(const FString& Params) override;
};
