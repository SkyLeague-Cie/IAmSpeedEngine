// Fill out your copyright notice in the Description page of Project Settings.

using System.IO;
using UnrealBuildTool;

public class IAmSpeed : ModuleRules
{
	public IAmSpeed(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore",
			"EnhancedInput", "ChaosVehicles", "PhysicsCore", "Chaos", "ChaosVehiclesCore"});
        PublicIncludePaths.AddRange(new string[] { Path.Combine(ModuleDirectory, "Actors") });
        PublicIncludePaths.AddRange(new string[] { Path.Combine(ModuleDirectory, "Base") });
        PublicIncludePaths.AddRange(new string[] { Path.Combine(ModuleDirectory, "Components") });
        PublicIncludePaths.AddRange(new string[] { Path.Combine(ModuleDirectory, "SubBodies") });
        PublicIncludePaths.AddRange(new string[] { Path.Combine(ModuleDirectory, "World") });
    }
}
