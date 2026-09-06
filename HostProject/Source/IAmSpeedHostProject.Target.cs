using UnrealBuildTool;
using System.Collections.Generic;

public class IAmSpeedHostProjectTarget : TargetRules
{
	public IAmSpeedHostProjectTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Game;
		DefaultBuildSettings = BuildSettingsVersion.Latest;
		IncludeOrderVersion = EngineIncludeOrderVersion.Latest;
		ExtraModuleNames.Add("IAmSpeedHostProject");
	}
}
