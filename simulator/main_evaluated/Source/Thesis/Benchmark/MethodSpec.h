#pragma once

#include "CoreMinimal.h"

enum class EInformationSource : uint8
{
    BeliefFromSensors,
    GroundTruthOracle
};

enum class ERegionRegime : uint8
{
    StaticUniform,
    Dynamic
};

struct THESIS_API FMethodSpec
{
    EInformationSource Info = EInformationSource::BeliefFromSensors;
    ERegionRegime Regime = ERegionRegime::Dynamic;
    FName PartitionStrategy = FName(TEXT("ZoneRegionGrowing"));
    FName PlannerStrategy = FName(TEXT("GreedyFov"));

    static bool FromMethodId(const FString& MethodId, FMethodSpec& OutSpec)
    {
        if (MethodId.Equals(TEXT("mine_dynamic"), ESearchCase::IgnoreCase))
        {
            OutSpec = { EInformationSource::BeliefFromSensors, ERegionRegime::Dynamic, FName(TEXT("ZoneRegionGrowing")), FName(TEXT("GreedyFov")) };
            return true;
        }
        if (MethodId.Equals(TEXT("mine_static"), ESearchCase::IgnoreCase))
        {
            OutSpec = { EInformationSource::BeliefFromSensors, ERegionRegime::StaticUniform, FName(TEXT("Uniform")), FName(TEXT("GreedyFov")) };
            return true;
        }
        if (MethodId.Equals(TEXT("random_dynamic"), ESearchCase::IgnoreCase))
        {
            OutSpec = { EInformationSource::BeliefFromSensors, ERegionRegime::Dynamic, FName(TEXT("Random")), FName(TEXT("Random")) };
            return true;
        }
        if (MethodId.Equals(TEXT("random_static"), ESearchCase::IgnoreCase))
        {
            OutSpec = { EInformationSource::BeliefFromSensors, ERegionRegime::StaticUniform, FName(TEXT("Uniform")), FName(TEXT("Random")) };
            return true;
        }
        if (MethodId.Equals(TEXT("oracle_dynamic"), ESearchCase::IgnoreCase))
        {
            OutSpec = { EInformationSource::GroundTruthOracle, ERegionRegime::Dynamic, FName(TEXT("OracleAllocation")), FName(TEXT("OracleTarget")) };
            return true;
        }
        if (MethodId.Equals(TEXT("oracle_static"), ESearchCase::IgnoreCase))
        {
            OutSpec = { EInformationSource::GroundTruthOracle, ERegionRegime::StaticUniform, FName(TEXT("Uniform")), FName(TEXT("OracleTarget")) };
            return true;
        }
        return false;
    }

    static const TCHAR* ToConfigString(EInformationSource Source)
    {
        return Source == EInformationSource::GroundTruthOracle ? TEXT("oracle") : TEXT("belief");
    }

    static const TCHAR* ToConfigString(ERegionRegime RegimeValue)
    {
        return RegimeValue == ERegionRegime::StaticUniform ? TEXT("static_uniform") : TEXT("dynamic");
    }
};
