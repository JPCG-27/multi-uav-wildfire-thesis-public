#include "DronePlannerFactory.h"
#include "DronePlannerBase.h"
#include "GreedyFovPlanner.h"
#include "RandomPlanner.h"
#include "OracleTargetPlanner.h"

namespace
{
    struct FPlannerRegistryRow
    {
        FName Name;
        TSubclassOf<UDronePlannerBase> Class;
    };

    static const TArray<FPlannerRegistryRow>& GetPlannerRegistry()
    {
        static const TArray<FPlannerRegistryRow> Registry = {
            { FName(TEXT("GreedyFov")), UGreedyFovPlanner::StaticClass() },
            { FName(TEXT("Random")), URandomPlanner::StaticClass() },
            { FName(TEXT("OracleTarget")), UOracleTargetPlanner::StaticClass() },
        };
        return Registry;
    }
}

TSubclassOf<UDronePlannerBase> FDronePlannerFactory::CreatePlannerClass(FName StrategyName)
{
    for (const FPlannerRegistryRow& Row : GetPlannerRegistry())
    {
        if (Row.Name == StrategyName)
        {
            return Row.Class;
        }
    }
    return nullptr;
}

FName FDronePlannerFactory::DefaultStrategyName()
{
    return FName(TEXT("GreedyFov"));
}

TArray<FName> FDronePlannerFactory::RegisteredNames()
{
    TArray<FName> Names;
    for (const FPlannerRegistryRow& Row : GetPlannerRegistry())
    {
        Names.Add(Row.Name);
    }
    return Names;
}
