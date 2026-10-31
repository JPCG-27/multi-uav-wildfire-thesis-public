#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "DroneSensorBase.generated.h"

struct FObservation;
struct FHumanSnapshot;
class Grid;

UCLASS(Abstract, Blueprintable)
class THESIS_API UDroneSensorBase : public UObject
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
                       TArray<FObservation>& OutObservations) PURE_VIRTUAL(UDroneSensorBase::Sense, );

    /** Ground-truth person snapshots used to gate POSITIVE person observations
     *  on the person's physical position being inside the view cone (world-space
     *  sensing). Injected by the manager each tick; may be null (legacy cell-
     *  centre behaviour). INVARIANT: read-only during the drone parallel phase —
     *  humans update only after the drone barrier, so this is race-free. */
    void SetPersonGroundTruth(const TArray<FHumanSnapshot>* InSnapshots)
    {
        PersonGroundTruth = InSnapshots;
    }

protected:
    const TArray<FHumanSnapshot>* PersonGroundTruth = nullptr;
};