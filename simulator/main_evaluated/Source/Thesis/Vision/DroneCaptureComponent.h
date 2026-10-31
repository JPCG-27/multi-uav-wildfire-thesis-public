#pragma once

#include "CoreMinimal.h"
#include "Components/SceneCaptureComponent2D.h"
#include "RenderCommandFence.h"
#include "DroneCaptureComponent.generated.h"

class UTextureRenderTarget2D;
class UPrimitiveComponent;
class FDroneSensorTimeExtension;
class FRHIGPUTextureReadback;

/**
 * Per-drone downward camera capture — the image source for the external
 * vision model (PLAN.md C.2/C.3).
 *
 * Determinism measures: explicit captures only (never per-frame), temporal AA
 * and motion blur disabled, manual exposure, tick-derived material time,
 * distance-field AO and refraction excluded from sensor views. Project settings
 * use conventional meshes, resident textures and hard local-light shadows.
 * Live pixel repeatability must still be validated for each rendering setup;
 * cached detection replay does not establish live rendering determinism.
 *
 * RGB copies are queued after capture and fenced at the scheduled consumer.
 * The consumer blocks until that image is ready: a slow frame is acceptable,
 * an inconsistent tick is not.
 */
UCLASS(ClassGroup = (Thesis), meta = (BlueprintSpawnableComponent))
class THESIS_API UDroneCaptureComponent : public USceneCaptureComponent2D
{
    GENERATED_BODY()

public:
    UDroneCaptureComponent();

    /** Allocates the render target and configures the camera. Call once. */
    void InitCapture(int32 ResX, int32 ResY, float HorizontalFOVDegrees, float PitchDegrees);

    /** Configure exposure/bloom. Manual exposure is independent of frame history;
     *  ExposureBias is compensation in stops, not a physical camera EV100.
     *  Auto exposure is an explicit look-development option. */
    void SetExposure(bool bAutoExposure, float ExposureBias, float BloomIntensity, float ManualEV100 = 12.0f);

    /** Toggle the explicitly changed-appearance profile used by fast experiments. */
    void SetFastUnshadowedRendering(bool bEnabled);
    void SetDeterministicFirstCaptureWarmup(bool bEnabled) { bWarmFirstCapture = bEnabled; }

    /** Renders the scene from the drone's current pose. Game thread only. */
    void CaptureTick(int32 Tick, bool bRender = true, float SecondsPerTick = 1.0f);

    /** Blocking read of the last capture. Returns false if nothing captured. */
    bool ReadLastCapture(TArray<FColor>& OutPixels, int32& OutWidth, int32& OutHeight, int32& OutTick);

    /** Borrow the immutable cached pixels without another full image copy.
     *  Game thread only; valid until the next capture, resize, or component destruction. */
    bool ReadLastCaptureView(const TArray<FColor>*& OutPixels, int32& OutWidth, int32& OutHeight, int32& OutTick);

    /** Blocking auxiliary SCS_SceneDepth capture at the last RGB camera pose/time.
     *  When OnlyComponent is supplied, only that primitive is rendered (amodal
     *  per-person pass). Labels are kept outside the vision bridge. */
    bool ReadDepthPass(const UPrimitiveComponent* OnlyComponent, TArray<float>& OutDepth,
        int32& OutWidth, int32& OutHeight);

    /** Dump every capture as PNG to Saved/Captures/ (debug/inspection only). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Capture")
    bool bSavePngForDebug = false;

    /** Optional run-specific folder. Empty uses Saved/Captures. */
    FString DebugOutputDirectory;

    int32 GetLastCaptureTick() const { return LastCaptureTick; }

private:
    bool bWarmFirstCapture = false;
    bool bFirstCaptureWarmed = false;
    UPROPERTY()
    TObjectPtr<UTextureRenderTarget2D> CaptureTarget;

    UPROPERTY()
    TObjectPtr<UTextureRenderTarget2D> DepthCaptureTarget;

    int32 LastCaptureTick = -1;
    TSharedPtr<FDroneSensorTimeExtension, ESPMode::ThreadSafe> SensorTimeExtension;

    // One pending GPU copy per capture component. The pixel cache is kept
    // separately so debug export can edit its local copy without changing it.
    TSharedPtr<FRHIGPUTextureReadback, ESPMode::ThreadSafe> CaptureReadback;
    FRenderCommandFence CaptureReadbackFence;
    TArray<FColor> CachedCapturePixels;
    int32 PendingReadbackWidth = 0;
    int32 PendingReadbackHeight = 0;
    int32 PendingReadbackTick = -1;
    bool bCaptureReadbackPending = false;
    bool bHasSavedFastProfileShowFlags = false;
    bool bSavedDynamicShadows = true;
    bool bSavedAmbientOcclusion = true;
    bool bSavedBloom = true;

    void SavePng(int32 Tick);
    void QueueCaptureReadback(int32 Tick);
    bool ConsumeCaptureReadback();

protected:
    virtual void BeginDestroy() override;
};
