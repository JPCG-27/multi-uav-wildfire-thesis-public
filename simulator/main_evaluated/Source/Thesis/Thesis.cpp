// Copyright Epic Games, Inc. All Rights Reserved.

#include "Thesis.h"
#include "Modules/ModuleManager.h"

// Shader registration and the /ThesisShaders source-directory mapping live in
// the separate ThesisShaders module (PostConfigInit, never live-coded). This
// game module is plain gameplay code and is safe to hot-patch.
class FThesisModule : public FDefaultGameModuleImpl
{
public:
	virtual void StartupModule() override
	{
		// A build dependency is not enough to guarantee that the shader module's
		// static IMPLEMENT_GLOBAL_SHADER registrations run before first dispatch.
		FModuleManager::LoadModuleChecked<IModuleInterface>(TEXT("ThesisShaders"));
		UE_LOG(LogTemp, Log, TEXT("Thesis: ensured ThesisShaders module is loaded."));
		FDefaultGameModuleImpl::StartupModule();
	}
};

IMPLEMENT_PRIMARY_GAME_MODULE(FThesisModule, Thesis, "Thesis");
