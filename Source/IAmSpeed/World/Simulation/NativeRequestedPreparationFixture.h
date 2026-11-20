#pragma once
#include "IAmSpeed/Actors/SpeedCar.h"
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include <atomic>
#include "NativeRequestedPreparationFixture.generated.h"

// Comparison instrumentation only. This fixture never overrides SupportsV2,
// never constructs a native permission and never qualifies a game profile.
UCLASS(Transient, NotBlueprintable)
class UNativeRequestedPreparationProbe final : public USpeedWheeledComponent
{
    GENERATED_BODY()
public:
    bool CompareCanonicalRequestedPreparationV2(const Speed::FNativeRequestedPreparationContextV2&) const override;
    mutable std::atomic<uint32> ComparisonCalls{0}, ComparisonThread{0};
    std::atomic<bool> RejectComparison{false}, ThrowComparison{false};
};
UCLASS(Transient, NotBlueprintable, NotPlaceable)
class ANativeRequestedPreparationCar final : public ASpeedCar
{
    GENERATED_BODY()
public:
    explicit ANativeRequestedPreparationCar(const FObjectInitializer&);
};
