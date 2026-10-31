#pragma once

#include "CoreMinimal.h"
#include "Templates/SubclassOf.h"

class UDronePlannerBase;

class THESIS_API FDronePlannerFactory
{
public:
    static TSubclassOf<UDronePlannerBase> CreatePlannerClass(FName StrategyName);
    static FName DefaultStrategyName();
    static TArray<FName> RegisteredNames();
};
