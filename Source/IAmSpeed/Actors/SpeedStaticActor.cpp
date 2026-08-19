#include "SpeedStaticActor.h"

#include "IAmSpeed/World/Analytic/SpeedAnalyticSourceComponent.h"

ASpeedStaticActor::ASpeedStaticActor()
{
	PrimaryActorTick.bCanEverTick = false;
	AnalyticSource = CreateDefaultSubobject<USpeedAnalyticSourceComponent>(
		TEXT("AnalyticSource"));
}

bool ASpeedStaticActor::ShouldIncludeAnalyticStaticMesh(
	const UStaticMeshComponent& Component) const
{
	return true;
}

void ASpeedStaticActor::GatherAnalyticStaticMeshes(
	TArray<UStaticMeshComponent*>& OutComponents) const
{
	if (!AnalyticSource)
	{
		OutComponents.Reset();
		return;
	}
	AnalyticSource->GatherOwnedStaticMeshes(OutComponents);
	OutComponents.RemoveAll([this](const UStaticMeshComponent* Component)
	{
		return !Component || !ShouldIncludeAnalyticStaticMesh(*Component);
	});
}
