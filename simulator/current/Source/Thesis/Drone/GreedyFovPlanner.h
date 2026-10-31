#pragma once

#include "CoreMinimal.h"
#include "DronePlannerBase.h"
#include "GreedyFovPlanner.generated.h"

/**
 * Default planner: delegates to the built-in FOV-aware greedy target selection
 * with commitment margin and territory-confined pathing (ADrone::RunGreedyFovReplan).
 * Behaviour is identical to the pre-refactor inline planner.
 */
UCLASS(Blueprintable)
class THESIS_API UGreedyFovPlanner : public UDronePlannerBase
{
    GENERATED_BODY()

public:
    virtual void Replan(ADrone& Drone, int32 CurrentTick) override;
};
