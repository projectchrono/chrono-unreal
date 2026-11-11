// Project Chrono 10.0.0 vehicle module compiled from source. See Plugins/ChronoCore/README.md.

using System.IO;
using UnrealBuildTool;

public class ChronoVehicle : ModuleRules
{
	public ChronoVehicle(ReadOnlyTargetRules Target) : base(Target)
	{
		// Same third-party compilation model as ChronoCore.
		PCHUsage = PCHUsageMode.NoPCHs;
		bUseUnity = false;
		bEnableExceptions = true;
		bUseRTTI = true;
		IWYUSupport = IWYUSupport.None;

		bWarningsAsErrors = false;
		CppCompileWarningSettings.ShadowVariableWarningLevel = WarningLevel.Off;
		CppCompileWarningSettings.UndefinedIdentifierWarningLevel = WarningLevel.Off;
		CppCompileWarningSettings.UnreachableCodeWarningLevel = WarningLevel.Off;
		CppCompileWarningSettings.UnsafeTypeCastWarningLevel = WarningLevel.Off;
		CppCompileWarningSettings.DeprecationWarningLevel = WarningLevel.Off;
		CppCompileWarningSettings.ShortenSizeTToIntWarningLevel = WarningLevel.Off;
		CppCompileWarningSettings.SwitchUnhandledEnumeratorWarningLevel = WarningLevel.Off;
		CppCompileWarningSettings.DeterministicWarningLevel = WarningLevel.Off;

		string ThirdPartyDir = Path.Combine(ModuleDirectory, "ThirdParty");

		// chrono_vehicle/ and chrono_thirdparty/stb; chrono/, Eigen etc. come from ChronoCore's public system include paths.
		PublicSystemIncludePaths.Add(ThirdPartyDir);

		PublicDependencyModuleNames.AddRange(new string[] { "Core", "ChronoCore" });

		// Private definitions of the Chrono_vehicle CMake target (src/chrono_vehicle/CMakeLists.txt).
		PrivateDefinitions.AddRange(new string[]
		{
			"CH_API_COMPILE_VEHICLE",   // CH_VEHICLE_API -> __declspec(dllexport) inside this module
			"NOMINMAX=",
		});
	}
}
