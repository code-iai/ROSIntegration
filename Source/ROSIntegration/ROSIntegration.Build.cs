// Copyright 1998-2017 Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

using System;
using System.IO;



public class ROSIntegration : ModuleRules
{
    private string ModulePath
    {
        get { return ModuleDirectory; }
    }


    private string ThirdPartyPath
    {
        get { return Path.GetFullPath(Path.Combine(ModulePath, "../../ThirdParty/")); }
    }

    public ROSIntegration(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

        string BSONPath = Path.Combine(ThirdPartyPath, "bson");

        // Console.WriteLine("");
        Console.WriteLine("BSONPath: " + BSONPath);


        PublicIncludePaths.AddRange(
			new string[] {
				"ROSIntegration/Public"
				// ... add public include paths required here ...
			}
			);
				
		
		PrivateIncludePaths.AddRange(
			new string[] {
				"ROSIntegration/Private",
				// ... add other private include paths required here ...
			}
			);
			
		
		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
				// ... add other public dependencies that you statically link with here ...
			}
			);
			
		
		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"CoreUObject",
				"Engine",
				"Slate",
				"SlateCore",
				// ... add private dependencies that you statically link with here ...	
			}
			);
		
		
		DynamicallyLoadedModuleNames.AddRange(
			new string[]
			{
				// ... add any modules that your module loads dynamically here ...
			}
			);



        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            Console.WriteLine("Using Windows BSON files");
            PublicAdditionalLibraries.Add(Path.Combine(BSONPath, "lib", "bson-static-1.0.lib"));
            PublicIncludePaths.Add(Path.Combine(BSONPath, "include", "windows")); 
        }

    }
}
