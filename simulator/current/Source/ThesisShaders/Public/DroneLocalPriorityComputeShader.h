#pragma once

#include "CoreMinimal.h"
#include "GlobalShader.h"
#include "ShaderParameterStruct.h"
#include "RenderGraphResources.h"

class THESISSHADERS_API FDroneLocalPriorityComputeCS : public FGlobalShader
{
public:
    DECLARE_GLOBAL_SHADER(FDroneLocalPriorityComputeCS);
    SHADER_USE_PARAMETER_STRUCT(FDroneLocalPriorityComputeCS, FGlobalShader);

    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint4>, CellStateA)
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint4>, CellStateB)
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, OwnedMask)
        SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float2>, OutputPlanning)
        SHADER_PARAMETER(uint32, NumCells)
        SHADER_PARAMETER(uint32, Width)
        SHADER_PARAMETER(uint32, Height)
        SHADER_PARAMETER(uint32, CurrentTick)
        SHADER_PARAMETER(float, WeightStaleness)
        SHADER_PARAMETER(float, WeightDanger)
        SHADER_PARAMETER(float, WeightPerson)
        SHADER_PARAMETER(float, StalenessSaturationTicks)
        SHADER_PARAMETER(float, ProbabilityScale)
        SHADER_PARAMETER(float, MonitoringUrgencyTau)
        SHADER_PARAMETER(float, UnseenExplorationBonus)
        SHADER_PARAMETER(float, FirePriorityRampMultiplier)
        SHADER_PARAMETER(float, FireMonitoringUrgencyTau)
        SHADER_PARAMETER(float, FireMinimumUrgency)
        SHADER_PARAMETER(float, NeighborPriorityWeight)
        SHADER_PARAMETER(uint32, FireDangerDiscoveryDelayTicks)
        SHADER_PARAMETER(uint32, FireDangerDiscoveryRampTicks)
        SHADER_PARAMETER(uint32, DroneDangerDiscoveryDelayTicks)
        SHADER_PARAMETER(uint32, DroneDangerDiscoveryRampTicks)
    END_SHADER_PARAMETER_STRUCT()

    static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Params)
    {
        return IsFeatureLevelSupported(Params.Platform, ERHIFeatureLevel::SM5)
            || IsFeatureLevelSupported(Params.Platform, ERHIFeatureLevel::SM6);
    }
};
