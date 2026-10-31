#pragma once

#include "CoreMinimal.h"
#include "GlobalShader.h"
#include "ShaderParameterStruct.h"
#include "RenderGraphResources.h"

/**
 * 3x3 wind-biased convolution on log-odds belief textures. Ping-ponged on
 * the CPU side. INVARIANT: deterministic (no atomics, fixed sample order,
 * weights set once per scenario).
 */
class THESISSHADERS_API FPixelBleedSpreadCS : public FGlobalShader
{
public:
    DECLARE_GLOBAL_SHADER(FPixelBleedSpreadCS);
    SHADER_USE_PARAMETER_STRUCT(FPixelBleedSpreadCS, FGlobalShader);

    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER_RDG_TEXTURE_SRV(Texture2D<float>, InputBelief)
        SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2D<float>, OutputBelief)
        SHADER_PARAMETER(FIntPoint, TextureSize)
        SHADER_PARAMETER(float, DecayPerTick)
        SHADER_PARAMETER_ARRAY(FVector4f, Weights, [3])
    END_SHADER_PARAMETER_STRUCT()

    static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Params)
    {
        return IsFeatureLevelSupported(Params.Platform, ERHIFeatureLevel::SM5)
            || IsFeatureLevelSupported(Params.Platform, ERHIFeatureLevel::SM6);
    }
};
