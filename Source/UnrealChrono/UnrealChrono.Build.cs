using UnrealBuildTool;

public class UnrealChrono : ModuleRules
{
	public UnrealChrono(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core", "CoreUObject", "Engine", "InputCore",
			"RenderCore", "RHI", "Renderer", "Projects", "DeveloperSettings",
			"Landscape",
			"Chaos", "ChaosCore", "PhysicsCore",
			// Project Chrono compiled from source (Plugins/ChronoCore, Plugins/ChronoVehicle)
			"ChronoCore", "ChronoVehicle",
		});

		// The sources include each other without folder prefixes (e.g. "Conversions.h", "Terrain/ChaosTerrain.h")
		PublicIncludePaths.AddRange(new string[]
		{
			"UnrealChrono/ChronoCarPhysics",
			"UnrealChrono/ChronoCarPhysics/Terrain",
			"UnrealChrono/ChronoCarPhysics/Tools",
			"UnrealChrono/ChronoCarPhysics/TestCar",
			"UnrealChrono/Tools",
			"UnrealChrono/UECarPhysics",
			"UnrealChrono/TerrainDeform",
			"UnrealChrono/Benchmark",
		});

		bEnableExceptions = true;
		bUseRTTI = true;
		PrivateDefinitions.Add("CHAOS_INCLUDE_LEVEL_1=1");

		SetupModulePhysicsSupport(Target);
	}
}
