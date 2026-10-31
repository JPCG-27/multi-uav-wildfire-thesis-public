#pragma once

#include "CoreMinimal.h"
#include "GlobalShader.h"
#include "ShaderParameterStruct.h"
#include "RenderGraphResources.h"

class THESISSHADERS_API FObservationIngestCS : public FGlobalShader
{
public:
    DECLARE_GLOBAL_SHADER(FObservationIngestCS);
    SHADER_USE_PARAMETER_STRUCT(FObservationIngestCS, FGlobalShader);

    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint4>, CellUpdates)
        SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2D<float>, FireBeliefOut)
        SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2D<float>, PersonBeliefOut)
        SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2D<float>, StalenessOut)
        SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2D<uint>, PersonObservationTickOut)
        SHADER_PARAMETER(FIntPoint, TextureSize)
        SHADER_PARAMETER(uint32, NumUpdates)
    END_SHADER_PARAMETER_STRUCT()

    static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Params)
    {
        return IsFeatureLevelSupported(Params.Platform, ERHIFeatureLevel::SM5)
            || IsFeatureLevelSupported(Params.Platform, ERHIFeatureLevel::SM6);
    }
};
