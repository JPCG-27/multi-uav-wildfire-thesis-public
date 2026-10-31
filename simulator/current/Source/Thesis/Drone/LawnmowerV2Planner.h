#pragma once

#include "CoreMinimal.h"
#include "DronePlannerBase.h"
#include "LawnmowerV2Planner.generated.h"

/** FOV-spaced, persistent-coverage lane planner. */
UCLASS(Blueprintable)
class THESIS_API ULawnmowerV2Planner : public UDronePlannerBase
{
    GENERATED_BODY()
public:
    virtual void Replan(ADrone& Drone, int32 CurrentTick) override;

    static int32 ComputeLaneStride(float ViewRadiusCells, float HorizontalFOVDegrees,
                                   int32 GridWidth, int32 GridHeight);
    static TArray<int32> BuildSweepOrder(const TArray<int32>& OwnedCells, int32 GridWidth,
                                         int32 GridHeight, int32 LaneStride,
                                         bool bStartFromTop, bool bStartFromRight);
    static bool IsWaypointAvailable(int32 Cell, const TArray<uint8>* GlobalCompleted,
                                    const TArray<uint8>& LocalCompleted);
};
