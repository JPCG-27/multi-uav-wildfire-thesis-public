#pragma once

#include "CoreMinimal.h"

struct FObservation
{
    int32 CellIndex = INDEX_NONE;
    float FireLogOddsUpdate = 0.0f;
    float PersonLogOddsUpdate = 0.0f;
    int32 Tick = 0;
    // Explicit modality: a fire-only observation must not refresh person evidence.
    // Defaults preserve existing ideal/custom sensors, including negative evidence.
    bool bHasPersonEvidence = true;
};
