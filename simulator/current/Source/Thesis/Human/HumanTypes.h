#pragma once

#include "CoreMinimal.h"

/** Mobility/health states — must match Tools/generate_humans.py. */
namespace EHumanState
{
    inline constexpr uint8 Healthy  = 0;
    inline constexpr uint8 Injured  = 1;
    inline constexpr uint8 Immobile = 2;
    inline constexpr uint8 Dead     = 3;
}

/** One row of humans.csv: a person's pose at a tick (cell-space coords). */
struct THESIS_API FHumanKeyframe
{
    int32 Tick = 0;
    int32 PersonId = INDEX_NONE;
    float X = 0.0f;       // cell space (cell = floor(X)); world = origin + X * CellSize
    float Y = 0.0f;
    int32 GroupId = INDEX_NONE;
    uint8 State = EHumanState::Healthy;
};

/** Current pose of one person, rebuilt each tick by AHumanManager::ApplyTick. */
struct THESIS_API FHumanSnapshot
{
    int32 PersonId = INDEX_NONE;
    FVector2f PosCell = FVector2f::ZeroVector;
    int32 GroupId = INDEX_NONE;
    uint8 State = EHumanState::Healthy;
};
