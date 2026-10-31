#pragma once

#include "CoreMinimal.h"
#include "GlobalShader.h"
#include "ShaderParameterStruct.h"
#include "RenderGraphResources.h"

/**
 * Grid-wide staleness statistics (mean/max staleness, coverage) for the
 * metrics recorder. Two passes, both with sequential fixed-order loops —
 * no atomics, no cross-thread reductions (determinism invariant).
 * See Shaders/GridStatsReduce.usf.
 */
class THESISSHADERS_API FGridStatsColumnReduceCS : public FGlobalShader
{
public:
    DECLARE_GLOBAL_SHADER(FGridStatsColumnReduceCS);
    SHADER_USE_PARAMETER_STRUCT(FGridStatsColumnReduceCS, FGlobalShader);

    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER_RDG_TEXTURE_SRV(Texture2D<float>, InputStaleness)
        SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<FVector4f>, ColumnPartials)
        SHADER_PARAMETER(FIntPoint, TextureSize)
        SHADER_PARAMETER(float, CurrentTickF)
    END_SHADER_PARAMETER_STRUCT()

    static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Params)
    {
        return IsFeatureLevelSupported(Params.Platform, ERHIFeatureLevel::SM5)
            || IsFeatureLevelSupported(Params.Platform, ERHIFeatureLevel::SM6);
    }
};

class THESISSHADERS_API FGridStatsFinalReduceCS : public FGlobalShader
{
public:
    DECLARE_GLOBAL_SHADER(FGridStatsFinalReduceCS);
    SHADER_USE_PARAMETER_STRUCT(FGridStatsFinalReduceCS, FGlobalShader);

    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FVector4f>, ColumnPartialsIn)
        SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<FVector4f>, StatsOut)
        SHADER_PARAMETER(int32, NumColumns)
    END_SHADER_PARAMETER_STRUCT()

    static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Params)
    {
        return IsFeatureLevelSupported(Params.Platform, ERHIFeatureLevel::SM5)
            || IsFeatureLevelSupported(Params.Platform, ERHIFeatureLevel::SM6);
    }
};
