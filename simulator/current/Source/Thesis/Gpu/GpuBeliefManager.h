#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "RenderGraphResources.h"
#include "RHIResources.h"
#include "RHIGPUReadback.h"
#include "RenderCommandFence.h"
#include "Templates/RefCounting.h"
#include "GpuBeliefManager.generated.h"

struct IPooledRenderTarget;
class UTextureRenderTarget2D;
struct FObservation;  // forward declaration (from DroneTypes.h)

/** Canonical row-major readback of one actual GPU texture at a simulation
 * checkpoint. Bytes exclude GPU row padding; unsupported/unproduced fields are
 * represented with bAvailable=false and an explicit reason. */
struct THESIS_API FGpuAuditTextureSnapshot
{
    FString Name;
    FString Format;
    int32 Width = 0;
    int32 Height = 0;
    int32 SourceTick = -1;
    bool bAvailable = false;
    FString UnavailableReason;
    TArray<uint8> Bytes;
};

UCLASS()
class THESIS_API UGpuBeliefManager : public UWorldSubsystem
{
    GENERATED_BODY()

public:
    void InitializeTextures(int32 InWidth, int32 InHeight);
    void ResetTextures();
    void DispatchStalenessUpdate(int32 CurrentTick = -1);
    void DispatchSpread(int32 CurrentTick = -1);
    void DispatchPriority(int32 CurrentTick, bool bSeparatePersonFreshness = false);
    void SwapPingPong();

    /** Applies a batch of observations to the server belief textures. */
    void DispatchObservationIngest(const TArray<FObservation>& Observations);

    /** Request a one-shot priority readback for debug visualization. */
    void RequestPriorityReadback(int32 CurrentTick);
    bool TryGetPriorityReadback(TArray<float>& OutPriorityData, int32& OutSourceTick);

    /** Synchronously reads actual GPU-owned audit fields at a fixed checkpoint.
     *  The caller chooses the checkpoint tick; this method fences the render
     *  queue and GPU, then returns tightly packed row-major bytes. This is an
     *  opt-in diagnostic and intentionally stalls the simulation tick. */
    bool CaptureAuditSnapshot(int32 SourceTick,
                              TArray<FGpuAuditTextureSnapshot>& OutFields,
                              FString& OutError);

    /** Grid-wide staleness stats (metrics). Two-pass fixed-order reduction —
     *  see GridStatsReduce.usf. Dispatch at tick T, consume a few ticks later. */
    void DispatchGridStats(int32 CurrentTick);

    /** Blocks (fence + flush) until the last dispatched stats are ready, then
     *  returns mean/max staleness and coverage fraction. False if none pending. */
    bool ConsumeGridStats(float& OutMeanStaleness, float& OutMaxStaleness,
                          float& OutCoverageFraction, int32& OutSourceTick);

    /** True if a stats dispatch is awaiting consumption. */
    int32 GetPendingGridStatsTick() const { return GridStatsTick; }

    int32 GetWidth() const { return Width; }
    int32 GetHeight() const { return Height; }

    virtual void Initialize(FSubsystemCollectionBase& Collection) override;
    virtual void Deinitialize() override;

private:
    int32 Width = 0;
    int32 Height = 0;
    int32 ReadIndex = 0;
    int32 WriteIndex = 1;

    TRefCountPtr<IPooledRenderTarget> FireBelief[2];
    TRefCountPtr<IPooledRenderTarget> PersonBelief[2];
    TRefCountPtr<IPooledRenderTarget> Staleness;
    // Integer observation tick + 1; zero means no person evidence has arrived.
    TRefCountPtr<IPooledRenderTarget> PersonObservationTick;
    TRefCountPtr<IPooledRenderTarget> Priority;

    // Priority debug readback
    TUniquePtr<FRHIGPUTextureReadback> PriorityReadback;
    FRenderCommandFence PriorityReadbackFence;
    int32 PriorityReadbackTick = -1;

    // Grid stats readback
    TUniquePtr<FRHIGPUBufferReadback> GridStatsReadback;
    TRefCountPtr<FRDGPooledBuffer> GridStatsResultBuffer;
    FRenderCommandFence GridStatsFence;
    int32 GridStatsTick = -1;
    int32 LastStalenessDispatchTick = -1;
    int32 LastSpreadDispatchTick = -1;
    int32 LastPriorityDispatchTick = -1;

    UPROPERTY()
    TObjectPtr<UTextureRenderTarget2D> HeatFieldRT = nullptr;

    bool bAllocated = false;

    void EnsureAllocated_RenderThread(FRHICommandListImmediate& RHICmdList);
};
