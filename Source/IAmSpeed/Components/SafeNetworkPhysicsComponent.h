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
	virtual bool CallRemoteFunction(UFunction* Function, void* Parameters, FOutParmRec* OutParms, FFrame* Stack) override;

private:
	bool ShouldBypassNetworkPhysicsLifecycle() const;
	void LogBypass(const TCHAR* Stage) const;
	void LogLifecycle(const TCHAR* Stage) const;
	void PrepareAsyncShutdown(const TCHAR* Stage);

	bool bInitializedNetworkPhysicsLifecycle = false;
#if !(UE_BUILD_SHIPPING)
	uint32 PrivateRPCTraceCallSerial = 0;
	uint32 PrivateRPCTraceLines = 0;
	int32 PrivateRPCTraceLastCount = INDEX_NONE;
#endif
};
