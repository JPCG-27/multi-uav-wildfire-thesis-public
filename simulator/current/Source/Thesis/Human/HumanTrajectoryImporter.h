#pragma once

#include "CoreMinimal.h"
#include "HumanTypes.h"

/**
 * Loads Tools/generate_humans.py output (humans.csv) into flat keyframe
 * storage with per-tick ranges — mirror of UFireDataImporter's role for fire.
 *
 * Layout: Keyframes is sorted by tick (the generator writes ticks in order;
 * the importer verifies). TickStartIndex[t] .. TickStartIndex[t+1] is the
 * half-open range of keyframes for tick t. Replay at any tick is one
 * contiguous scan — no per-tick searching.
 */
class THESIS_API FHumanTrajectoryImporter
{
public:
    struct FResult
    {
        TArray<FHumanKeyframe> Keyframes;
        TArray<int32> TickStartIndex;   // size = MaxTick + 2 (sentinel at end)
        TArray<int32> AppearTick;       // per person: tick of first keyframe
        int32 MaxTick = -1;
        int32 NumPersons = 0;
    };

    /** Returns false on missing file / malformed header / non-monotonic ticks. */
    static bool ImportFromCSV(const FString& FilePath, FResult& Out);
};
