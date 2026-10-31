// Shader-registration module, deliberately separate from the Thesis game module.
//
// WHY: modules that register global shader types (IMPLEMENT_GLOBAL_SHADER) must
// load at PostConfigInit and CANNOT be hot-patched — Live Coding patch DLLs
// re-run the shader-type static initializers and crash the editor with
// "Shader type was loaded too late". Keeping every IMPLEMENT_GLOBAL_SHADER
// here means Live Coding edits to gameplay code in the Thesis module never
// touch this module, so the editor no longer crashes on in-editor compiles.
//
// RULE: any new compute shader's .h goes in Public/, its .cpp (with
// IMPLEMENT_GLOBAL_SHADER) in Private/. If you EDIT files in this module,
// close the editor and build from the IDE — this module still cannot be
// live-coded.

using UnrealBuildTool;

public class ThesisShaders : ModuleRules
{
	public ThesisShaders(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "RenderCore", "RHI" });

		PrivateDependencyModuleNames.AddRange(new string[] { "Renderer", "Projects" });
	}
}
