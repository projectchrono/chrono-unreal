// Project Chrono vehicle module for Unreal. See Plugins/ChronoCore/README.md.

#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleInterface.h"

CHRONOVEHICLE_API DECLARE_LOG_CATEGORY_EXTERN(LogChronoVehicle, Log, All);

class FChronoVehicleModule : public IModuleInterface
{
public:
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

	/// Directory that chrono::vehicle::GetVehicleDataFile() resolves relative paths against
	/// (<ProjectContentDir>/ChronoData/vehicle/).
	static FString GetVehicleDataDirectory();
};
