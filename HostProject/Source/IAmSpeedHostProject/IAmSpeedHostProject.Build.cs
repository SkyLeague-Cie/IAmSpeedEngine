using UnrealBuildTool;

public class IAmSpeedHostProject : ModuleRules
{
	public IAmSpeedHostProject(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		PublicDependencyModuleNames.AddRange(new[] {
			"Core", "CoreUObject", "Engine", "IAmSpeed"
		});
	}
}
