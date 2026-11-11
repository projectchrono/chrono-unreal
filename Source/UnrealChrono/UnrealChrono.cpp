#include "Modules/ModuleManager.h"
#include "Misc/Paths.h"
#include "ShaderCore.h"

class FUnrealChronoModule : public FDefaultGameModuleImpl
{
public:
	virtual void StartupModule() override
	{
		FDefaultGameModuleImpl::StartupModule();
		// Project shaders (terrain deformation): /UnrealChrono/... -> <project>/Shaders/...
		AddShaderSourceDirectoryMapping(TEXT("/UnrealChrono"), FPaths::Combine(FPaths::ProjectDir(), TEXT("Shaders")));
	}
};

IMPLEMENT_PRIMARY_GAME_MODULE(FUnrealChronoModule, UnrealChrono, "UnrealChrono");
