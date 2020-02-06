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
		get { return Path.GetFullPath(Path.Combine(ModulePath, "..", "..", "ThirdParty")); }
	}

	public ROSIntegration(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

		string BSONPath = Path.Combine(ThirdPartyPath, "bson");

		// Console.WriteLine("");
		Console.WriteLine("BSONPath: " + BSONPath);

		// Include std::string functions for rapidjson
#if UE_4_19_OR_LATER // works at least for 4.18.3, but not for 4.17.3 and below
		PublicDefinitions.Add("RAPIDJSON_HAS_STDSTRING=1");
#else
		Definitions.Add("RAPIDJSON_HAS_STDSTRING=1");
#endif

    		PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "Public"));
		
		PrivateIncludePaths.Add(Path.Combine(ModuleDirectory, "Private"));
    		PrivateIncludePaths.Add(Path.Combine(ModuleDirectory, "Private/rosbridge2cpp"));

		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
				"Sockets",
				"Networking"
				// ... add other public dependencies that you statically link with here ...
			}
		);


		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"CoreUObject",
				"Engine",
				"Sockets",
				"Networking"
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
		else if (Target.Platform == UnrealTargetPlatform.Linux)
		{
			Console.WriteLine("Using Linux BSON files");
			PublicAdditionalLibraries.Add(Path.Combine(BSONPath, "lib", "libbson-1.0.a"));
			PublicIncludePaths.Add(Path.Combine(BSONPath, "include", "linux"));
		}
		else if (Target.Platform == UnrealTargetPlatform.Mac)
		{
			Console.WriteLine("Using macOS BSON files");
			PublicAdditionalLibraries.Add(Path.Combine(BSONPath, "lib", "libbson-static-1.0.a"));
			PublicIncludePaths.Add(Path.Combine(BSONPath, "include", "mac"));
		}
	}
}
