// Project Chrono core module for Unreal. See Plugins/ChronoCore/README.md.

#include "ChronoCoreModule.h"

#include "Modules/ModuleManager.h"

// Version macros only - safe to include without the ChronoIncludeBegin/End guards.
#include "chrono/ChVersion.h"

DEFINE_LOG_CATEGORY(LogChronoCore);

void FChronoCoreModule::StartupModule()
{
	UE_LOG(LogChronoCore, Log, TEXT("Chrono version %hs (core module compiled from source)"), CHRONO_VERSION);
}

void FChronoCoreModule::ShutdownModule()
{
}

IMPLEMENT_MODULE(FChronoCoreModule, ChronoCore)
