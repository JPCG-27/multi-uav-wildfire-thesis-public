#include "RandomPlanner.h"
#include "Drone.h"
#include "SimulationManager.h"

void URandomPlanner::Replan(ADrone& Drone, int32 CurrentTick)
{
    const ASimulationManager* Manager = ASimulationManager::StaticInstance;
    const int32 RunSeed = Manager ? Manager->GetBenchmarkRunSeed() : 0;
    Drone.RunRandomReplan(CurrentTick, RunSeed);
}
