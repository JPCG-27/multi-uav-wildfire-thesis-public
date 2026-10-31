#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "DronePlannerBase.generated.h"

class ADrone;

/**
 * Abstract drone planner strategy (mirrors the UDroneSensorBase pattern).
 *
 * Called from ADrone::TickDrone when a replan trigger fires (arrival, cadence,
 * ownership change, fallback timer). Implementations select a target and plan
 * a path for the drone — either by delegating to the built-in
 * ADrone::RunGreedyFovReplan, or by implementing custom selection logic.
 *
 * INVARIANT: implementations must be deterministic (explicit tie-breaks, no
 * wall-clock or random state) and must only read the drone's own local belief
 * (OwnGrid) — never server/global priority. They run inside the drone's
 * parallel phase, so they must not touch other drones or non-owned grid cells.
 *
 * Selection: ASimulationManager::DronePlannerClass (pushed to every drone via
 * ApplyManagerConfiguration), or per-drone PlannerClass in the editor.
 */
UCLASS(Abstract, Blueprintable)
class THESIS_API UDronePlannerBase : public UObject
{
    GENERATED_BODY()

public:
    virtual void Replan(ADrone& Drone, int32 CurrentTick) PURE_VIRTUAL(UDronePlannerBase::Replan, );
};
