#include "OracleTargetPlanner.h"
#include "Drone.h"
#include "SimulationManager.h"

void UOracleTargetPlanner::Replan(ADrone& Drone, int32 CurrentTick)
{
    ASimulationManager* Manager = ASimulationManager::StaticInstance;
    int32 TargetCell = INDEX_NONE;
    float Score = 0.0f;

    if (Manager && Manager->FindOracleTargetForDrone(Drone, CurrentTick, TargetCell, Score) &&
        Drone.PlanOracleTargetCell(TargetCell, CurrentTick, Score))
    {
        return;
    }
}
