#pragma once

#include "CoreMinimal.h"
#include "Physics/NetworkPhysicsComponent.h"
#include "SafeNetworkPhysicsComponent.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(SafeNetworkPhysicsLog, Log, All);

UCLASS()
class IAMSPEED_API USafeNetworkPhysicsComponent : public UNetworkPhysicsComponent
{
	GENERATED_BODY()

public:
	virtual void InitializeComponent() override;
	virtual void UninitializeComponent() override;
	virtual void BeginDestroy() override;

private:
	bool ShouldBypassNetworkPhysicsLifecycle() const;
	void LogBypass(const TCHAR* Stage) const;
	void LogLifecycle(const TCHAR* Stage) const;
	void PrepareAsyncShutdown(const TCHAR* Stage);

	bool bInitializedNetworkPhysicsLifecycle = false;
};
