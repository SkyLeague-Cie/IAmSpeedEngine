#pragma once
#include "CoreMinimal.h"
#include "UObject/StrongObjectPtr.h"
#include "IAmSpeed/World/Analytic/AnalyticWorldData.h"
class ASpeedSimulation;
class AFastSimulation;
class UWorld;
class UPrimitiveComponent;

struct FIsolatedStaticSource
{
    uint64 SourceId = 0;
    FTransform Transform;
    uint8 ObjectType = 0;
};
struct FIsolatedStaticWorldSeed
{
    TSharedPtr<const Speed::Analytic::FAnalyticWorldData> Geometry;
    TArray<FIsolatedStaticSource> Sources;
};

/** Minimal host: own UWorld, driver, registry, static provider and source proxies.
 * No BeginPlay, render/async ticking, live registry or game mode is admitted.
 * Its canonical calls are synchronous on exactly one admitted owner lane.
 * Close/destroy only on GT AFTER that owner has acknowledged suspension/join.
 */
class IAMSPEED_API FIsolatedSimulationContext final
{
public:
    static TUniquePtr<FIsolatedSimulationContext> Create(const FIsolatedStaticWorldSeed& Seed);
    ~FIsolatedSimulationContext();
    UWorld* GetWorld() const { return World.Get(); }
    AFastSimulation* GetDriver() const { return Driver; }
    UPrimitiveComponent* ResolveStaticSource(uint64 SourceId) const;
    uint64 ResolveStaticId(const UPrimitiveComponent* Component) const;
    void Close();
private:
    FIsolatedSimulationContext() = default;
    TStrongObjectPtr<UWorld> World;
    AFastSimulation* Driver = nullptr;
    TMap<uint64, TWeakObjectPtr<UPrimitiveComponent>> Sources;
    TMap<const UPrimitiveComponent*, uint64> SourceIds;
};
