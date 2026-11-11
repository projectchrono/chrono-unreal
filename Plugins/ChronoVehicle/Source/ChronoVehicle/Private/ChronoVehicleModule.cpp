// Project Chrono vehicle module for Unreal. See Plugins/ChronoCore/README.md.

#include "ChronoVehicleModule.h"

#include "Misc/Paths.h"
#include "Modules/ModuleManager.h"

#include "ChronoIncludeBegin.h"
#include "chrono_vehicle/ChVehicleDataPath.h"
#include "ChronoIncludeEnd.h"

DEFINE_LOG_CATEGORY(LogChronoVehicle);

FString FChronoVehicleModule::GetVehicleDataDirectory()
{
	return FPaths::ConvertRelativePathToFull(FPaths::ProjectContentDir() / TEXT("ChronoData/vehicle/"));
}

void FChronoVehicleModule::StartupModule()
{
	const FString DataDir = GetVehicleDataDirectory();
	chrono::vehicle::SetVehicleDataPath(std::string(TCHAR_TO_UTF8(*DataDir)));
	UE_LOG(LogChronoVehicle, Log, TEXT("Chrono vehicle data path: %s"), *DataDir);
}

void FChronoVehicleModule::ShutdownModule()
{
}

IMPLEMENT_MODULE(FChronoVehicleModule, ChronoVehicle)
