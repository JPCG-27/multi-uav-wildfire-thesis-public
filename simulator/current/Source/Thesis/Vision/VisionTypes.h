#pragma once

#include "CoreMinimal.h"
#include "VisionTypes.generated.h"

/**
 * Detection-cache behaviour of the vision bridge (PLAN.md C.3).
 *
 * Live ML inference (and GPU rasterisation) is not bit-deterministic, which
 * would break the per-tick determinism invariant. The cache restores it:
 * Record runs talk to the live server and write every response to
 * Saved/VisionCache/<dir>/; Replay runs read only the cache (no server, no
 * pixel readback) and are therefore fully reproducible — the dissertation
 * numbers run in Replay.
 */
UENUM(BlueprintType)
enum class EVisionCacheMode : uint8
{
    /** Query the live server every time; cache untouched. */
    Off,
    /** Query the live server and record every response to the cache. */
    Record,
    /** Never touch the server; read responses from the cache only. */
    Replay
};

/** One detection from the vision model. Coordinates normalized to [0,1] image space. */
struct THESIS_API FVisionDetection
{
    int32 ClassId = 0;          // COCO class (0 = person)
    float Confidence = 0.0f;
    float X0 = 0.0f, Y0 = 0.0f; // top-left
    float X1 = 0.0f, Y1 = 0.0f; // bottom-right
};
