using UnrealBuildTool;

public class UnrealChronoEditorTarget : TargetRules
{
	public UnrealChronoEditorTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Editor;
		DefaultBuildSettings = BuildSettingsVersion.Latest;
		IncludeOrderVersion = EngineIncludeOrderVersion.Latest;
		bOverrideBuildEnvironment = true;
		ExtraModuleNames.Add("UnrealChrono");
	}
}
