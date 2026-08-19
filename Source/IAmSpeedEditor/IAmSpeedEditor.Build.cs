using UnrealBuildTool;

public class IAmSpeedEditor : ModuleRules
{
	public IAmSpeedEditor(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core", "CoreUObject", "Engine", "IAmSpeed"
		});
		PrivateDependencyModuleNames.AddRange(new string[]
		{
			"AssetRegistry", "MeshDescription", "StaticMeshDescription", "UnrealEd"
		});
	}
}
