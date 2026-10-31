#pragma once

#include "CoreMinimal.h"
#include "DroneSensorBase.h"
#include "PerfectDroneSensor.generated.h"

UCLASS(Blueprintable)
class THESIS_API UPerfectDroneSensor : public UDroneSensorBase
{
    GENERATED_BODY()

public:
virtual void Sense(int32 CurrentTick,
                   const Grid* GridData,
                   const FVector& DroneWorldPos,
                   float CellSizeUU,
                   const FVector& GridOriginWorld,
                   int32 FacingDir,
                   float HorizontalFOVDegrees,
                   float MaxViewDistanceUU,
                   TArray<FObservation>& OutObservations) override;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Sensor")
    float FirePositiveLLR = 3.0f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Sensor")
    float FireNegativeLLR = -0.3f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Sensor")
    float PersonPositiveLLR = 2.5f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Sensor")
    float PersonNegativeLLR = -0.2f;

    /** When true (default), a POSITIVE person observation additionally requires
     *  at least one person in the cell to be physically inside the view cone
     *  (world-space check), instead of counting anyone in a cell whose centre is
     *  visible. Without this gate a person can be "detected" up to half a cell
     *  diagonal outside the camera footprint, which inflates detection at coarse
     *  cell sizes. Belief, staleness, and negative observations stay cell-based.
     *  Set false to reproduce the legacy cell-centre behaviour. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Sensor")
    bool bWorldSpacePersonGating = true;
};