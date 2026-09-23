// Fill out your copyright notice in the Description page of Project Settings.

using System.IO;
using UnrealBuildTool;

public class IAmSpeed : ModuleRules
{
	public IAmSpeed(ReadOnlyTargetRules Target) : base(Target)
	{
			PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
			// Canonical publication contains source/publisher C++ exceptions before
			// returning to UE. Module-wide unwind/codegen policy, validated separately
			// on each supported target; not an input-only compiler switch.
			bEnableExceptions = true;
		// Link the OS acquisition API directly; no Unreal input dispatch module
		// or render-frame polling participates in this producer.
		if (Target.Platform == UnrealTargetPlatform.Win64 && Target.Type != TargetType.Server)
		{
			string GameInputSdk = Path.Combine(EngineDirectory, "Plugins", "Runtime", "GameInput", "Source",
				"GameInputWindowsLibrary", "ThirdParty");
			PublicSystemIncludePaths.Add(GameInputSdk);
			PublicAdditionalLibraries.Add(Path.Combine(GameInputSdk, "Binaries", "x64", "GameInput.lib"));
		}

		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore",
			"EnhancedInput", "ChaosVehicles", "PhysicsCore", "Chaos", "ChaosVehiclesCore", "Landscape"});
        PublicIncludePaths.AddRange(new string[] { Path.Combine(ModuleDirectory, "Actors") });
        PublicIncludePaths.AddRange(new string[] { Path.Combine(ModuleDirectory, "Base") });
        PublicIncludePaths.AddRange(new string[] { Path.Combine(ModuleDirectory, "Components") });
        PublicIncludePaths.AddRange(new string[] { Path.Combine(ModuleDirectory, "SubBodies") });
        PublicIncludePaths.AddRange(new string[] { Path.Combine(ModuleDirectory, "World") });
		PublicIncludePaths.AddRange(new string[] { Path.Combine(ModuleDirectory, "World", "Analytic") });
    }
}
