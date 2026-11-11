using UnrealBuildTool;

public class UnrealChronoTarget : TargetRules
{
	public UnrealChronoTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Game;
		DefaultBuildSettings = BuildSettingsVersion.Latest;
		IncludeOrderVersion = EngineIncludeOrderVersion.Latest;
		bOverrideBuildEnvironment = true;
		ExtraModuleNames.Add("UnrealChrono");
	}
}
