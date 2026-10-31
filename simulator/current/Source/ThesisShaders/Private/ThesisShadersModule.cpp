#include "Modules/ModuleManager.h"
#include "Misc/Paths.h"
#include "ShaderCore.h"

/**
 * Loads at PostConfigInit (see Thesis.uproject) so the global shader types in
 * this module register before the engine initializes shader types, and so the
 * /ThesisShaders virtual path exists before any shader compilation.
 */
class FThesisShadersModule : public IModuleInterface
{
public:
    virtual void StartupModule() override
    {
        const FString ShaderDir = FPaths::Combine(FPaths::ProjectDir(), TEXT("Shaders"));
        // Maps virtual path "/ThesisShaders/Foo.usf" to <ProjectDir>/Shaders/Foo.usf.
        AddShaderSourceDirectoryMapping(TEXT("/ThesisShaders"), ShaderDir);
        UE_LOG(LogTemp, Log, TEXT("ThesisShaders: mapped /ThesisShaders to %s"), *ShaderDir);
    }
};

IMPLEMENT_MODULE(FThesisShadersModule, ThesisShaders);
