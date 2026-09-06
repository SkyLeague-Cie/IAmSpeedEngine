#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

DECLARE_LOG_CATEGORY_EXTERN(SpeedPhysicsLog, Log, All);

class IAmSpeed : public IModuleInterface
{
public:
	/** IModuleInterface implementation */
	void StartupModule();
	void ShutdownModule();
};
