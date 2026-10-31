#pragma once

#include "CoreMinimal.h"
#include "VisionTypes.h"
#include "YoloGeometricMetrics.h"

/** Pure scoring helpers for the fixed-path cached-detection temporal reference.
 *  The caller supplies detections projected with the destination frame's
 *  processing-time camera/terrain geometry. This class never changes the
 *  operational detection stream. */
namespace ChanceNullScorer
{
    struct FOffset
    {
        int32 FractionPercent = 0; // stable replicate identity across cameras/runs
        int32 Frames = 0;          // derived from this camera's actual frame count
    };

    struct FPerson
    {
        int32 Id = INDEX_NONE;
        int32 Cell = INDEX_NONE;
        int32 AppearanceTick = INDEX_NONE;
    };

    struct FFrame
    {
        int32 CameraId = INDEX_NONE;
        int32 CaptureTick = INDEX_NONE;
        int32 ProcessingTick = INDEX_NONE;
        FTransform ProcessingCameraTransform = FTransform::Identity;
        float ProcessingCameraFOV = 0.0f;
        TArray<FVisionDetection> Detections;
        TArray<FPerson> VisiblePeople; // sorted by stable person ID
        TArray<int32> OriginalDetectionCells;
        TArray<int32> OriginalMatchedPersonIds; // per detection, INDEX_NONE if unmatched
        int32 OriginalTruePositives = 0;
        int32 OriginalFalsePositives = 0;
        int32 OriginalMissed = 0;
        int32 OriginalNonPerson = 0;
        int32 OriginalPersonClaimCount = 0;
        int32 OriginalOffGridRejections = 0;
    };

    struct FFrameScore
    {
        int32 CameraId = INDEX_NONE;
        int32 OffsetFrames = 0;
        int32 ReplicateIndex = INDEX_NONE;
        int32 SourceCameraId = INDEX_NONE;
        int32 SourceCaptureTick = INDEX_NONE;
        int32 CaptureTick = INDEX_NONE;
        int32 ProcessingTick = INDEX_NONE;
        int32 DetectionCount = 0;
        int32 PersonClaimCount = 0;
        int32 TruePositives = 0;
        int32 FalsePositives = 0;
        int32 Missed = 0;
        int32 NonPerson = 0;
        int32 PersonsInView = 0;
        int32 OffGridRejections = 0;
        TArray<FVisionDetection> SourceDetections; // complete payload in source order
        TArray<int32> MatchedPersonIds; // per detection; preserves payload order
        TArray<int32> DetectionCells;
    };

    struct FPersonScore
    {
        int32 PersonId = INDEX_NONE;
        int32 AppearanceTick = INDEX_NONE;
        int32 FirstMatchProcessingTick = INDEX_NONE;
        int32 MatchCount = 0;
    };

    /** Select unique circular rotations at fractions 0.10 through 0.90. */
    THESIS_API TArray<FOffset> BuildOffsets(int32 FrameCount, bool bPilot = false);

    /** Scores one source payload against one destination frame's eligible people.
     *  ProjectDetection must report whether INDEX_NONE means an off-grid point. */
    THESIS_API FFrameScore ScoreFrame(
        const FFrame& Destination,
        const FFrame& Source,
        int32 OffsetFrames,
        int32 GridWidth,
        float PersonConfidenceThreshold,
        const TFunction<int32(const FVisionDetection&, bool&)>& ProjectDetection);

    /** Aggregates frame match vectors into stable per-person first-match and count rows. */
    THESIS_API bool AggregatePeople(
        const TArray<FFrame>& Frames,
        const TArray<FFrameScore>& Scores,
        const TArray<FPerson>& AllPeople,
        TArray<FPersonScore>& OutPeople);

    /** Exact no-shuffle gate: compare ordered projected cells and winning person IDs
     *  as well as all frame counters, including confidence/payload order via the
     *  unchanged source detection array. */
    THESIS_API bool MatchesOriginal(const FFrame& Destination, const FFrameScore& Score);
}
