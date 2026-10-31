#include "FullyRandomPlanner.h"
#include "Drone.h"
#include "SimulationManager.h"

void UFullyRandomPlanner::Replan(ADrone& Drone, int32 CurrentTick)
{
    const ASimulationManager* Manager = ASimulationManager::StaticInstance;
    Drone.RunFullyRandomReplan(CurrentTick, Manager ? Manager->GetBenchmarkRunSeed() : 0);
}
