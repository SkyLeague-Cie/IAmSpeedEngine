#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

class IAmSpeed : public IModuleInterface
{
public:
	/** IModuleInterface implementation */
	void StartupModule();
	void ShutdownModule();
};
