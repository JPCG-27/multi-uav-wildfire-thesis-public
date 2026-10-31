#pragma once

#include "CoreMinimal.h"
#include "GlobalShader.h"
#include "ShaderParameterStruct.h"
#include "RenderGraphResources.h"

class THESISSHADERS_API FPriorityComputeCS : public FGlobalShader
{
public:
    DECLARE_GLOBAL_SHADER(FPriorityComputeCS);
    SHADER_USE_PARAMETER_STRUCT(FPriorityComputeCS, FGlobalShader);

    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER_RDG_TEXTURE_SRV(Texture2D<float>, InputFireBelief)
        SHADER_PARAMETER_RDG_TEXTURE_SRV(Texture2D<float>, InputPersonBelief)
        SHADER_PARAMETER_RDG_TEXTURE_SRV(Texture2D<float>, InputStaleness)
        SHADER_PARAMETER_RDG_TEXTURE_SRV(Texture2D<uint>, InputPersonObservationTick)
        SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2D<float>, OutputPriority)
        SHADER_PARAMETER(FIntPoint, TextureSize)
        SHADER_PARAMETER(float, WeightStaleness)
        SHADER_PARAMETER(float, WeightDanger)
        SHADER_PARAMETER(float, WeightPerson)
        SHADER_PARAMETER(float, StalenessSaturationTicks)
        SHADER_PARAMETER(float, ProbabilityScale)
        SHADER_PARAMETER(float, MonitoringUrgencyTau)
        SHADER_PARAMETER(float, UnseenExplorationBonus)
        SHADER_PARAMETER(uint32, CurrentTick)
        SHADER_PARAMETER(uint32, SeparatePersonFreshness)
    END_SHADER_PARAMETER_STRUCT()

    static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Params)
    {
        return IsFeatureLevelSupported(Params.Platform, ERHIFeatureLevel::SM5)
            || IsFeatureLevelSupported(Params.Platform, ERHIFeatureLevel::SM6);
    }
};
