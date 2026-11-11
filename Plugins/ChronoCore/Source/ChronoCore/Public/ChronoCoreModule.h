// Project Chrono core module for Unreal. See Plugins/ChronoCore/README.md.

#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleInterface.h"

CHRONOCORE_API DECLARE_LOG_CATEGORY_EXTERN(LogChronoCore, Log, All);

class FChronoCoreModule : public IModuleInterface
{
public:
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};
