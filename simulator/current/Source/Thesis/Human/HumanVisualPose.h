#pragma once

#include "CoreMinimal.h"
#include "ReferenceSkeleton.h"

// Presentation only. Never feeds human trajectories, truth, or drone belief.
enum class EHumanVisualPose : uint8
{
    Standing, Walking, Running, Waving, Crouching, Seated, Lying, Count
};

struct THESIS_API FHumanVisualRig
{
    TArray<FTransform> Reference;
    TArray<int32> Parents;
    int32 UpperArm[2] = { INDEX_NONE, INDEX_NONE };
    int32 ForeArm[2] = { INDEX_NONE, INDEX_NONE };
    int32 Hand[2] = { INDEX_NONE, INDEX_NONE };
    int32 Thigh[2] = { INDEX_NONE, INDEX_NONE };
    int32 Calf[2] = { INDEX_NONE, INDEX_NONE };
    int32 Foot[2] = { INDEX_NONE, INDEX_NONE };
    int32 Toe[2] = { INDEX_NONE, INDEX_NONE };
    int32 Head = INDEX_NONE;
    FVector Forward = FVector::ForwardVector;
    FVector Right = FVector::RightVector;
    float ForwardYaw = 0.0f;
    bool bSupported = false;

    void Initialize(const FReferenceSkeleton& Skeleton);
    // Always rebuilds from the reference pose; no accumulated transforms,
    // Animation Blueprint ticks, physics, cloth, or wall-clock inputs.
    void Evaluate(EHumanVisualPose Pose, double SimSeconds, uint32 PersonSeed,
        TArray<FTransform>& OutLocal, FBox& OutBoneBounds) const;
    static const TCHAR* Name(EHumanVisualPose Pose);
    static bool Parse(const FString& Name, EHumanVisualPose& OutPose);
};
