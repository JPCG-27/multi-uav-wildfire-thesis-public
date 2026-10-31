#pragma once

#include "CoreMinimal.h"
#include "HumanManager.h"
#include "Dom/JsonObject.h"

struct THESIS_API FYoloMaskPersonStats
{
    int32 PersonId = INDEX_NONE;
    FIntRect AmodalBBoxXYXY = FIntRect(FIntPoint::ZeroValue, FIntPoint::ZeroValue);
    FIntRect VisibleBBoxXYXY = FIntRect(FIntPoint::ZeroValue, FIntPoint::ZeroValue);
    int64 AmodalPixels = 0;
    int64 VisiblePixels = 0;
};

/** One offline training frame. Ground truth is deliberately kept out of the
 *  vision bridge and all online belief interfaces. */
struct THESIS_API FYoloDatasetFrame
{
    FString CameraId;
    FString ScenarioFamilyId;
    FString ViewSource = TEXT("planner_trajectory");
    int32 Tick = INDEX_NONE;
    int32 PoseTick = INDEX_NONE;
    int32 Width = 0;
    int32 Height = 0;
    int32 HumanVisualSeed = 0;
    float SecondsPerTick = 1.0f;
    FTransform CameraTransform = FTransform::Identity;
    float HorizontalFovDegrees = 0.0f;
    float DepthToleranceUU = 0.1f;
    TArray<FColor> Rgb;
    TArray<float> OpaqueSceneDepthUU;
    TArray<FHumanVisualCaptureRecord> Persons;
    TArray<TArray<float>> PersonDepthUU;
    TArray<FString> QualityExclusions;
};

/** Deterministic CPU post-process and atomic native output for sar-export-v1. */
class THESIS_API FYoloDatasetExporter
{
public:
    bool BeginRun(const FString& OutputRoot, const FString& InDatasetId,
        const FString& InScenarioFamilyId, const TMap<FString, FString>& InSourceHashes,
        FString& OutError);
    bool WriteFrame(const FYoloDatasetFrame& Frame, FString& OutError);
    bool CompleteRun(const TSharedPtr<FJsonObject>& Provenance, FString& OutError);
    void AbortRun();

    static bool BuildVisibleMask(const FYoloDatasetFrame& Frame, TArray<uint32>& OutMask,
        TArray<FYoloMaskPersonStats>& OutPersonStats, FString& OutError);

private:
    FString RunDirectory;
    FString ScenarioFamilyId;
    FString DatasetId;
    TMap<FString, FString> SourceHashes;
    TArray<FString> FramePaths;
    bool bActive = false;

    static bool SaveBytesAtomic(const FString& Path, const TArray64<uint8>& Bytes);
    static bool SaveTextAtomic(const FString& Path, const FString& Text, bool bReplaceExisting = false);
};
