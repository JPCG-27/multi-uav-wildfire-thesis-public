#include "GreedyFovPlanner.h"
#include "Drone.h"

void UGreedyFovPlanner::Replan(ADrone& Drone, int32 CurrentTick)
{
    Drone.RunGreedyFovReplan(CurrentTick);
}
