// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;
using System.Collections.Generic;

public class ThesisEditorTarget : TargetRules
{
	public ThesisEditorTarget( TargetInfo Target) : base(Target)
	{
		Type = TargetType.Editor;
		DefaultBuildSettings = BuildSettingsVersion.V6;
		IncludeOrderVersion = EngineIncludeOrderVersion.Unreal5_7;
		// Keep translation units independent of Git's modified-file set.
		// Adaptive standalone compilation changed /fp:fast planner scores.
		bUseAdaptiveUnityBuild = false;
		ExtraModuleNames.AddRange( new string[] { "Thesis", "ThesisShaders" } );
	}
}
