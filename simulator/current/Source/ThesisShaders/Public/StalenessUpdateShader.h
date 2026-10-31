#pragma once

#include "CoreMinimal.h"
#include "GlobalShader.h"
#include "ShaderParameterStruct.h"
#include "RenderGraphResources.h"

/**
 * Per-tick staleness increment. Every cell's staleness is increased by 1.
 * Observation resets are done separately (TODO: server Bayesian update).
 */
class THESISSHADERS_API FStalenessUpdateCS : public FGlobalShader
{
public:
    DECLARE_GLOBAL_SHADER(FStalenessUpdateCS);
    SHADER_USE_PARAMETER_STRUCT(FStalenessUpdateCS, FGlobalShader);

    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2D<float>, StalenessOut)
        SHADER_PARAMETER(FIntPoint, TextureSize)
    END_SHADER_PARAMETER_STRUCT()

    static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Params)
    {
        return IsFeatureLevelSupported(Params.Platform, ERHIFeatureLevel::SM5)
            || IsFeatureLevelSupported(Params.Platform, ERHIFeatureLevel::SM6);
    }
};
