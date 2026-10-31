#pragma once

#include "CoreMinimal.h"
#include "DronePlannerBase.h"
#include "OracleTargetPlanner.generated.h"

UCLASS(Blueprintable)
class THESIS_API UOracleTargetPlanner : public UDronePlannerBase
{
    GENERATED_BODY()

public:
    virtual void Replan(ADrone& Drone, int32 CurrentTick) override;
};
