// Project Chrono 10.0.0 core module compiled from source. See Plugins/ChronoCore/README.md.

using System.IO;
using UnrealBuildTool;

public class ChronoCore : ModuleRules
{
	public ChronoCore(ReadOnlyTargetRules Target) : base(Target)
	{
		// Third-party code: no PCH, no unity files (unrelated Chrono/Bullet TUs reuse static helper names),
		// and Chrono needs C++ exceptions and RTTI (throw, dynamic_cast, typeid).
		PCHUsage = PCHUsageMode.NoPCHs;
		bUseUnity = false;
		bEnableExceptions = true;
		bUseRTTI = true;
		IWYUSupport = IWYUSupport.None;

		// Chrono, Bullet, HACD and Eigen are not warning-clean under Unreal's strict defaults.
		// The verbatim sources under ThirdParty/ must not be edited, so relax the warnings UBT promotes to errors.
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

		// Registered as *system* include paths so MSVC treats Chrono/Eigen headers as external code
		// (/external:I + /external:W0) both for this module and for every consumer.
		// Mirrors the include directories of Chrono's CMake target Chrono_core.
		PublicSystemIncludePaths.AddRange(new string[]
		{
			ThirdPartyDir,                                                    // chrono/, chrono_thirdparty/
			Path.Combine(ThirdPartyDir, "eigen"),                             // Eigen/, unsupported/Eigen/
			Path.Combine(ThirdPartyDir, "chrono", "collision", "bullet"),     // LinearMath/, BulletCollision/
			Path.Combine(ThirdPartyDir, "chrono", "collision", "gimpact"),    // GIMPACT/, ConvexDecomposition/
			Path.Combine(ThirdPartyDir, "chrono_thirdparty", "HACD"),
			Path.Combine(ThirdPartyDir, "chrono_thirdparty", "HACDv2"),
		});

		PublicDependencyModuleNames.AddRange(new string[] { "Core" });

		// Chaos collision backend (Private/ChaosCollision, Private/Simulation). The public headers only forward-declare
		// Chaos types, so consumers do not inherit these dependencies. Deliberately NOT "Engine": the game module passes
		// World->GetPhysicsScene() / its solver in.
		PrivateDependencyModuleNames.AddRange(new string[] { "Chaos", "ChaosCore", "PhysicsCore" });

		// Private definitions of the Chrono_core CMake target (src/chrono/CMakeLists.txt).
		PrivateDefinitions.AddRange(new string[]
		{
			"CH_API_COMPILE",            // ChApi -> __declspec(dllexport) inside this module
			"BT_THREADSAFE",
			"BP_USE_FIXEDPOINT_INT_32",
			// <windows.h>/<winsock2.h> are pulled in by ChSocket, Bullet and chrono_thirdparty/filesystem.
			// Defined empty (trailing '=') to stay identical to Unreal's own definition in Windows/MinWindows.h.
			// WIN32_LEAN_AND_MEAN is deliberately NOT defined: HACD/hacdMicroAllocator.cpp needs timeGetTime() from <mmsystem.h>.
			"NOMINMAX=",
		});

		// Definitions that must be seen by every consumer because they affect types declared in Chrono headers.
		PublicDefinitions.AddRange(new string[]
		{
			"BT_USE_DOUBLE_PRECISION",   // Bullet in double precision (CH_USE_BULLET_DOUBLE=ON); cbtScalar is in headers
			"CH_IGNORE_DEPRECATED",      // CH_DEPRECATED expands to nothing
			"_CRT_SECURE_NO_DEPRECATE",
			"_SCL_SECURE_NO_DEPRECATE",
		});

		if (Target.LinkType == TargetLinkType.Monolithic)
		{
			// Everything ends up in one binary: make ChApiEXPORT/ChApiIMPORT expand to nothing.
			PublicDefinitions.Add("CH_STATIC");
		}

		if (Target.Platform == UnrealTargetPlatform.Win64)
		{
			PublicSystemLibraries.Add("Ws2_32.lib");   // chrono/utils/ChSocket.cpp
			PublicSystemLibraries.Add("Winmm.lib");    // timeGetTime() in chrono_thirdparty/HACD/hacdMicroAllocator.cpp
		}
	}
}
