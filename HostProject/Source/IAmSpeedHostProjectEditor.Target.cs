using UnrealBuildTool;
using System.Collections.Generic;

public class IAmSpeedHostProjectEditorTarget : TargetRules
{
	public IAmSpeedHostProjectEditorTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Editor;
		DefaultBuildSettings = BuildSettingsVersion.Latest;
		IncludeOrderVersion = EngineIncludeOrderVersion.Latest;

		// CI builds keep Windows resource outputs in this project's Intermediate
		// tree instead of sharing or rewriting the installed Engine's RC files.
		if (Target.Arguments?.HasOption("-SLPrivateProjectResources") == true)
		{
			WindowsPlatform.bSharedResourceFiles = false;
		}

		ExtraModuleNames.Add("IAmSpeedHostProject");
	}
}
