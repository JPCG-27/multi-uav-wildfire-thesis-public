#include "DroneCaptureComponent.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Components/PrimitiveComponent.h"
#include "TextureResource.h"
#include "ImageUtils.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/FileManager.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "SceneViewExtension.h"
#include "SceneView.h"
#include "GameTime.h"
#include "RenderGraphUtils.h"
#include "RHIGPUReadback.h"
#include "SceneTexturesConfig.h"
#include "PixelFormat.h"
#include "RenderingThread.h"
#include "HAL/IConsoleManager.h"
#include "DumpGPU.h"
#include "PipelineStateCache.h"
#if WITH_EDITOR
#include "AssetCompilingManager.h"
#include "ShaderCompiler.h"
#endif

// Local to this sensor, never globally registered on spectator views. Setup
// copies time on the game thread before renderer creation; render callbacks
// never read mutable component state.
class FDroneSensorTimeExtension final : public ISceneViewExtension
{
public:
    struct FGBufferReadback
    {
        FString Channel;
        FIntRect Rect;
        EPixelFormat Format = PF_Unknown;
        FIntPoint SourceExtent = FIntPoint::ZeroValue;
        TUniquePtr<FRHIGPUTextureReadback> Readback;
    };

    FGameTime CaptureTime = FGameTime::CreateUndilated(0.0, 0.0f);
    uint32 CaptureFrame = 0;

    void ArmGBufferDiagnostic(const FString& OutputDir, const FString& OwnerName, int32 Tick, uint32 Frame, FIntPoint Extent,
        bool bOnlyMismatchProbe, const FString& CaptureLabel)
    {
        check(IsInGameThread());
        bGBufferArmed = true;
        bGBufferQueued = false;
        GBufferError.Empty();
        GBufferOutputDir = OutputDir;
        GBufferOwnerName = OwnerName;
        GBufferTick = Tick;
        GBufferFrame = Frame;
        GBufferExtent = Extent;
        GBufferCaptureLabel = CaptureLabel;
        GBufferReadbacks.Reset();

        // Preserve the legacy ROIs by default. The targeted mode captures only
        // the small region around the two known HDR mismatch pixels.
        TArray<FIntRect> Probes;
        if (bOnlyMismatchProbe)
        {
            Probes.Add(FIntRect(536, 533, 542, 538));
        }
        else
        {
            Probes.Add(FIntRect(199, 259, 204, 264));
            Probes.Add(FIntRect(691, 255, 722, 284));
        }
        GBufferProbeCount = Probes.Num();
        const TCHAR* Channels[] = { TEXT("GBufferA"), TEXT("GBufferB"), TEXT("GBufferC"), TEXT("GBufferD"), TEXT("SceneColorBasePass") };
        for (const TCHAR* Channel : Channels)
        {
        for (const FIntRect& Probe : Probes)
            {
                FGBufferReadback& Slot = GBufferReadbacks.AddDefaulted_GetRef();
                Slot.Channel = Channel;
                Slot.Rect = Probe;
                Slot.Readback = MakeUnique<FRHIGPUTextureReadback>(FName(*FString::Printf(
                    TEXT("Thesis%s_%s_t%d"), Channel, *OwnerName, Tick)));
            }
        }
    }

    bool SaveGBufferDiagnostic()
    {
        check(IsInGameThread());
        if (!bGBufferArmed) return true;

        // Submit and wait for the GPU copy fences, then make the completed bytes visible to the game thread.
        ENQUEUE_RENDER_COMMAND(ThesisGBufferReadbackFence)([](FRHICommandListImmediate& RHICmdList)
        {
            RHICmdList.SubmitAndBlockUntilGPUIdle();
        });
        FlushRenderingCommands();
        if (!bGBufferQueued)
        {
            UE_LOG(LogTemp, Error, TEXT("Thesis GBuffer diagnostic failed tick=%d drone=%s: no valid base-pass copies (%s)."),
                GBufferTick, *GBufferOwnerName, *GBufferError);
            bGBufferArmed = false;
            return false;
        }

        for (const FGBufferReadback& Slot : GBufferReadbacks)
        {
            if (!Slot.Readback->IsReady())
            {
                UE_LOG(LogTemp, Error, TEXT("Thesis GBuffer GPU readback fence did not complete: %s."), *Slot.Channel);
                bGBufferArmed = false;
                return false;
            }
            int32 RowPitchPixels = 0;
            int32 BufferHeight = 0;
            const uint8* Source = static_cast<const uint8*>(Slot.Readback->Lock(RowPitchPixels, &BufferHeight));
            const FPixelFormatInfo& Info = GPixelFormats[Slot.Format];
            if (!Source || Info.BlockSizeX != 1 || Info.BlockSizeY != 1 || Info.BlockBytes <= 0 ||
                RowPitchPixels < Slot.Rect.Width() || BufferHeight < Slot.Rect.Height())
            {
                if (Source) Slot.Readback->Unlock();
                UE_LOG(LogTemp, Error, TEXT("Thesis GBuffer invalid native readback: %s format=%s pitch=%d height=%d."),
                    *Slot.Channel, Info.Name, RowPitchPixels, BufferHeight);
                bGBufferArmed = false;
                return false;
            }

            const int32 RowBytes = Slot.Rect.Width() * Info.BlockBytes;
            TArray64<uint8> Bytes;
            Bytes.SetNumUninitialized(static_cast<int64>(RowBytes) * Slot.Rect.Height());
            for (int32 Y = 0; Y < Slot.Rect.Height(); ++Y)
            {
                FMemory::Memcpy(Bytes.GetData() + static_cast<int64>(Y) * RowBytes,
                    Source + static_cast<int64>(Y) * RowPitchPixels * Info.BlockBytes, RowBytes);
            }
            Slot.Readback->Unlock();

            IFileManager::Get().MakeDirectory(*GBufferOutputDir, true);
            const FString Stem = FString::Printf(TEXT("%s_t%06d_f%06u%s_%s_x%d_y%d_w%d_h%d"),
                *GBufferOwnerName, GBufferTick, GBufferFrame,
                GBufferCaptureLabel.IsEmpty() ? TEXT("") : *FString::Printf(TEXT("_%s"), *GBufferCaptureLabel), *Slot.Channel,
                Slot.Rect.Min.X, Slot.Rect.Min.Y, Slot.Rect.Width(), Slot.Rect.Height());
            const FString RawPath = FPaths::Combine(GBufferOutputDir, Stem + TEXT(".raw"));
            const FString MetaPath = FPaths::Combine(GBufferOutputDir, Stem + TEXT(".raw.meta"));
            const FString Metadata = FString::Printf(
                TEXT("tick=%d\nframe=%u\ndrone=%s\ncapture_phase=%s\nchannel=%s\nformat_enum=%d\nformat_name=%s\nblock_width=%d\nblock_height=%d\nbytes_per_block=%d\nsource_extent_x=%d\nsource_extent_y=%d\nview_rect_x=%d\nview_rect_y=%d\nview_rect_width=%d\nview_rect_height=%d\nroi_x=%d\nroi_y=%d\nroi_width=%d\nroi_height=%d\nrow_pitch_pixels=%d\nrow_padding_saved=false\nproducer=PostRenderBasePassDeferred_RenderThread\n"),
                GBufferTick, GBufferFrame, *GBufferOwnerName, *GBufferCaptureLabel, *Slot.Channel, static_cast<int32>(Slot.Format), Info.Name,
                Info.BlockSizeX, Info.BlockSizeY, Info.BlockBytes, Slot.SourceExtent.X, Slot.SourceExtent.Y,
                GBufferViewRect.Min.X, GBufferViewRect.Min.Y, GBufferViewRect.Width(), GBufferViewRect.Height(),
                Slot.Rect.Min.X, Slot.Rect.Min.Y, Slot.Rect.Width(), Slot.Rect.Height(), RowPitchPixels);
            if (!FFileHelper::SaveArrayToFile(Bytes, *RawPath) || !FFileHelper::SaveStringToFile(Metadata, *MetaPath))
            {
                UE_LOG(LogTemp, Error, TEXT("Thesis GBuffer diagnostic failed to write %s."), *RawPath);
                bGBufferArmed = false;
                return false;
            }
        }

        bGBufferArmed = false;
        return true;
    }

    virtual void PostRenderBasePassDeferred_RenderThread(FRDGBuilder& GraphBuilder, FSceneView& View,
        const FRenderTargetBindingSlots& RenderTargets,
        TRDGUniformBufferRef<FSceneTextureUniformParameters> SceneTextures) override
    {
        if (!bGBufferArmed || bGBufferQueued) return;
        if (!SceneTextures || !View.Family || View.Family->FrameNumber != GBufferFrame ||
            View.UnscaledViewRect.Min.X != 0 || View.UnscaledViewRect.Min.Y != 0 ||
            View.UnscaledViewRect.Width() != GBufferExtent.X || View.UnscaledViewRect.Height() != GBufferExtent.Y)
        {
            GBufferError = TEXT("view/frame/scene-texture uniform mismatched capture");
            UE_LOG(LogTemp, Error, TEXT("Thesis GBuffer diagnostic rejected view frame=%u rect=%d,%d-%d,%d expected frame=%u extent=%dx%d; RT slots=%d."),
                View.Family ? View.Family->FrameNumber : 0, View.UnscaledViewRect.Min.X, View.UnscaledViewRect.Min.Y,
                View.UnscaledViewRect.Max.X, View.UnscaledViewRect.Max.Y, GBufferFrame, GBufferExtent.X, GBufferExtent.Y,
                RenderTargets.GetActiveCount());
            return;
        }

        GBufferViewRect = View.UnscaledViewRect;
        FRDGTextureRef UniformTextures[] = { (*SceneTextures)->GBufferATexture, (*SceneTextures)->GBufferBTexture, (*SceneTextures)->GBufferCTexture, nullptr, nullptr };
        FRDGTextureRef Textures[] = { nullptr, nullptr, nullptr, nullptr, nullptr };
        const TCHAR* ChannelNames[] = { TEXT("GBufferA"), TEXT("GBufferB"), TEXT("GBufferC"), TEXT("GBufferD"), TEXT("SceneColorBasePass") };
        const TCHAR* SlotNames[] = { TEXT("GBufferA"), TEXT("GBufferB"), TEXT("GBufferC"), TEXT("GBufferD"), TEXT("SceneColor") };
        RenderTargets.Enumerate([&](FRenderTargetBinding Binding)
        {
            FRDGTextureRef Bound = Binding.GetTexture();
            if (!Bound) return;
            UE_LOG(LogTemp, Display, TEXT("Thesis GBuffer base-pass slot: texture=%s format=%s(%d) extent=%dx%d."),
                Bound->Name, GPixelFormats[Bound->Desc.Format].Name, static_cast<int32>(Bound->Desc.Format),
                Bound->Desc.Extent.X, Bound->Desc.Extent.Y);
            for (int32 Channel = 0; Channel < UE_ARRAY_COUNT(Textures); ++Channel)
            {
                if ((UniformTextures[Channel] && Bound == UniformTextures[Channel]) ||
                    (FString(Bound->Name).Contains(SlotNames[Channel]) && !Textures[Channel]))
                {
                    Textures[Channel] = Bound;
                }
            }
        });

        // Prefer the scene-texture uniform when it references a real active
        // attachment; fall back to named render-target slots if it is opaque.
        for (int32 Channel = 0; Channel < UE_ARRAY_COUNT(Textures); ++Channel)
        {
            FRDGTextureRef Texture = Textures[Channel];
            if (!Texture || Texture->Desc.Extent != GBufferExtent)
            {
                GBufferError = FString::Printf(TEXT("GBuffer %s absent from capture-sized render-target slots"), ChannelNames[Channel]);
                UE_LOG(LogTemp, Error, TEXT("Thesis GBuffer rejected %s: uniform=%s extent=%dx%d, slot match absent (color slots=%d)."),
                    ChannelNames[Channel], UniformTextures[Channel] ? UniformTextures[Channel]->Name : TEXT("<null>"),
                    UniformTextures[Channel] ? UniformTextures[Channel]->Desc.Extent.X : 0,
                    UniformTextures[Channel] ? UniformTextures[Channel]->Desc.Extent.Y : 0, RenderTargets.GetActiveCount());
                return;
            }
            const FPixelFormatInfo& Info = GPixelFormats[Texture->Desc.Format];
            if (Info.BlockSizeX != 1 || Info.BlockSizeY != 1 || Info.BlockBytes <= 0)
            {
                GBufferError = FString::Printf(TEXT("unsupported native format %s"), Info.Name);
                UE_LOG(LogTemp, Error, TEXT("Thesis GBuffer rejected %s format=%s."), ChannelNames[Channel], Info.Name);
                return;
            }
            UE_LOG(LogTemp, Display, TEXT("Thesis GBuffer source drone=%s tick=%d frame=%u channel=%s texture=%s format=%s(%d) extent=%dx%d."),
                *GBufferOwnerName, GBufferTick, GBufferFrame, ChannelNames[Channel], Texture->Name, Info.Name,
                static_cast<int32>(Texture->Desc.Format), Texture->Desc.Extent.X, Texture->Desc.Extent.Y);
        }

        // Validate every ROI before queuing any readback, so an error cannot
        // leave RDG lambdas holding readback objects that the next tick destroys.
        for (const FGBufferReadback& Slot : GBufferReadbacks)
        {
            if (Slot.Rect.Min.X < View.UnscaledViewRect.Min.X || Slot.Rect.Min.Y < View.UnscaledViewRect.Min.Y ||
                Slot.Rect.Max.X > View.UnscaledViewRect.Max.X || Slot.Rect.Max.Y > View.UnscaledViewRect.Max.Y)
            {
                GBufferError = TEXT("probe ROI outside view rect");
                return;
            }
        }
        for (int32 Channel = 0; Channel < UE_ARRAY_COUNT(Textures); ++Channel)
        {
            FRDGTextureRef Texture = Textures[Channel];
            for (int32 Probe = 0; Probe < GBufferProbeCount; ++Probe)
            {
                FGBufferReadback& Slot = GBufferReadbacks[Channel * GBufferProbeCount + Probe];
                Slot.Format = Texture->Desc.Format;
                Slot.SourceExtent = Texture->Desc.Extent;
                AddEnqueueCopyPass(GraphBuilder, Slot.Readback.Get(), Texture,
                    FResolveRect(Slot.Rect.Min.X, Slot.Rect.Min.Y, Slot.Rect.Max.X, Slot.Rect.Max.Y));
            }
        }
        bGBufferQueued = true;
    }

    virtual void SetupViewFamily(FSceneViewFamily& Family) override
    {
        Family.Time = CaptureTime;
        // Shader sampling can use these counters independently of Time.
        // Scene captures must use the simulation tick, not editor frame count.
        Family.FrameNumber = CaptureFrame;
        Family.FrameCounter = CaptureFrame;
    }

private:
    bool bGBufferArmed = false;
    bool bGBufferQueued = false;
    FString GBufferError;
    FString GBufferOutputDir;
    FString GBufferOwnerName;
    int32 GBufferTick = INDEX_NONE;
    uint32 GBufferFrame = 0;
    FIntPoint GBufferExtent = FIntPoint::ZeroValue;
    FIntRect GBufferViewRect;
    FString GBufferCaptureLabel;
    int32 GBufferProbeCount = 0;
    TArray<FGBufferReadback> GBufferReadbacks;
};

UDroneCaptureComponent::UDroneCaptureComponent()
{
    PrimaryComponentTick.bCanEverTick = false;

    // Explicit captures only — the sim decides when to render, never the frame loop.
    bCaptureEveryFrame = false;
    bCaptureOnMovement = false;
    bAlwaysPersistRenderingState = true;

    CaptureSource = SCS_FinalColorLDR;

    // Strip temporal effects so a single capture doesn't depend on frame history.
    ShowFlags.SetTemporalAA(false);
    ShowFlags.SetMotionBlur(false);

    // A sensor must not inherit the spectator's exposure history or artistic
    // local contrast compression. Keep natural albedo and shadows readable.
    PostProcessSettings.bOverride_ColorContrast = true;
    PostProcessSettings.ColorContrast = FVector4(1, 1, 1, 1);
    PostProcessSettings.bOverride_ColorSaturation = true;
    PostProcessSettings.ColorSaturation = FVector4(1, 1, 1, 1);
    PostProcessSettings.bOverride_LocalExposureHighlightContrastScale = true;
    PostProcessSettings.LocalExposureHighlightContrastScale = 1.0f;
    PostProcessSettings.bOverride_LocalExposureShadowContrastScale = true;
    PostProcessSettings.LocalExposureShadowContrastScale = 1.0f;
    // These temporal screen-space effects are unsuitable for sparse captures.
    PostProcessSettings.bOverride_DynamicGlobalIlluminationMethod = true;
    PostProcessSettings.DynamicGlobalIlluminationMethod = EDynamicGlobalIlluminationMethod::None;
    PostProcessSettings.bOverride_ReflectionMethod = true;
    PostProcessSettings.ReflectionMethod = EReflectionMethod::None;
    ShowFlags.SetScreenSpaceReflections(false);
    SetExposure(/*bAutoExposure=*/false, /*ExposureBias=*/0.0f, /*BloomIntensity=*/0.15f);
}

void UDroneCaptureComponent::InitCapture(int32 ResX, int32 ResY, float HorizontalFOVDegrees, float PitchDegrees)
{
    // Retire any queued copy before UpdateResourceImmediate can replace the
    // texture it references.
    if (bCaptureReadbackPending)
    {
        ConsumeCaptureReadback();
    }
    // UE's reusable texture readback assumes a fixed extent for its staging
    // texture. Recreate it whenever the render target itself is reinitialized.
    CaptureReadback.Reset();
    CachedCapturePixels.Reset();
    PendingReadbackTick = -1;
    // The DFAO bent-normal buffer varies between identical captures even with
    // identical depth/GBuffer inputs. Exclude that input from sensor lighting;
    // retain skylight, screen-space AO and cast shadows. Set this at runtime
    // so serialized component defaults cannot re-enable it for the sensor.
    ShowFlags.SetDistanceFieldAO(false);
    // Exclude unstable heat-haze distortion; retain the fire/smoke sprites.
    ShowFlags.SetRefraction(false);

    const bool bDiagnosticBaseColor = FParse::Param(FCommandLine::Get(), TEXT("ThesisSensorDiagnosticBaseColor"));
    const bool bDiagnosticNormal = FParse::Param(FCommandLine::Get(), TEXT("ThesisSensorDiagnosticNormal"));
    const bool bDiagnosticSceneColorHDR = FParse::Param(FCommandLine::Get(), TEXT("ThesisSensorDiagnosticSceneColorHDR"));
    const bool bDiagnosticSceneColorSceneDepth = FParse::Param(FCommandLine::Get(), TEXT("ThesisSensorDiagnosticSceneColorSceneDepth"));
    const bool bDiagnosticGBuffer = FParse::Param(FCommandLine::Get(), TEXT("ThesisSensorDiagnosticGBuffer"));
    if (bDiagnosticGBuffer && !bDiagnosticSceneColorSceneDepth)
    {
        UE_LOG(LogTemp, Error, TEXT("Thesis GBuffer diagnostic refused: -ThesisSensorDiagnosticGBuffer requires -ThesisSensorDiagnosticSceneColorSceneDepth."));
    }
    const int32 DiagnosticFlagCount = static_cast<int32>(bDiagnosticBaseColor) + static_cast<int32>(bDiagnosticNormal) + static_cast<int32>(bDiagnosticSceneColorHDR) + static_cast<int32>(bDiagnosticSceneColorSceneDepth);
    const bool bExactFloatDiagnostic = DiagnosticFlagCount == 1 && (bDiagnosticNormal || bDiagnosticSceneColorHDR || bDiagnosticSceneColorSceneDepth);
    if (DiagnosticFlagCount > 1)
    {
        UE_LOG(LogTemp, Error, TEXT("Thesis sensor diagnostic refused: BaseColor, Normal, SceneColorHDR and SceneColorSceneDepth flags are mutually exclusive; retaining configured CaptureSource=%d."), static_cast<int32>(CaptureSource));
    }
    else if (DiagnosticFlagCount == 1)
    {
        CaptureSource = bDiagnosticBaseColor ? SCS_BaseColor : (bDiagnosticNormal ? SCS_Normal : (bDiagnosticSceneColorHDR ? SCS_SceneColorHDR : SCS_SceneColorSceneDepth));
        const TCHAR* DiagnosticName = bDiagnosticBaseColor ? TEXT("SCS_BaseColor") : (bDiagnosticNormal ? TEXT("SCS_Normal") : (bDiagnosticSceneColorHDR ? TEXT("SCS_SceneColorHDR") : TEXT("SCS_SceneColorSceneDepth")));
        UE_LOG(LogTemp, Warning, TEXT("Thesis sensor diagnostic: CaptureSource=%d (%s); this is a renderer isolation image, not a scientific YOLO input."),
            static_cast<int32>(CaptureSource), DiagnosticName);
        if (bDiagnosticSceneColorHDR || bDiagnosticSceneColorSceneDepth)
        {
            UE_LOG(LogTemp, Warning, TEXT("Thesis sensor %s diagnostic disables the full post-processing chain; it is not a FinalColorLDR capture."), DiagnosticName);
            if (bDiagnosticSceneColorSceneDepth)
            {
                UE_LOG(LogTemp, Warning, TEXT("SceneColorSceneDepth stores scene depth in the sidecar alpha channel."));
            }
        }
    }

    if (!CaptureTarget)
    {
        CaptureTarget = NewObject<UTextureRenderTarget2D>(this, TEXT("DroneCaptureRT"));
    }
    CaptureTarget->InitCustomFormat(
        FMath::Max(16, ResX), FMath::Max(16, ResY),
        bExactFloatDiagnostic ? PF_A32B32G32R32F : PF_B8G8R8A8,
        /*bForceLinearGamma=*/bExactFloatDiagnostic);
    CaptureTarget->UpdateResourceImmediate(true);

    TextureTarget = CaptureTarget;
    FOVAngle = FMath::Clamp(HorizontalFOVDegrees, 5.0f, 170.0f);
    SetRelativeRotation(FRotator(PitchDegrees, 0.0f, 0.0f));
}

void UDroneCaptureComponent::SetExposure(bool bAutoExposure, float ExposureBias, float BloomIntensity, float ManualEV100)
{
    // Keep the exposure pass enabled even for manual metering. Disabling this
    // show flag bypasses compensation entirely, making ExposureBias ineffective.
    // AEM_Manual below fixes exposure without temporal eye adaptation.
    ShowFlags.SetEyeAdaptation(true);
    PostProcessSettings.bOverride_AutoExposureMethod = true;
    PostProcessSettings.AutoExposureMethod = bAutoExposure ? AEM_Histogram : AEM_Manual;
    PostProcessSettings.bOverride_AutoExposureApplyPhysicalCameraExposure = true;
    PostProcessSettings.AutoExposureApplyPhysicalCameraExposure = true;
    // ISO 100 and f/8 give EV100 = log2(64 * shutter speed). This separates
    // the fixed daylight reference from the user's exposure compensation.
    PostProcessSettings.bOverride_CameraISO = true;
    PostProcessSettings.CameraISO = 100.0f;
    PostProcessSettings.bOverride_DepthOfFieldFstop = true;
    PostProcessSettings.DepthOfFieldFstop = 8.0f;
    PostProcessSettings.bOverride_CameraShutterSpeed = true;
    PostProcessSettings.CameraShutterSpeed = FMath::Pow(2.0f, FMath::Clamp(ManualEV100, -4.0f, 20.0f)) / 64.0f;
    ShowFlags.SetDepthOfField(false);
    PostProcessSettings.bOverride_AutoExposureBiasCurve = true;
    PostProcessSettings.AutoExposureBiasCurve = nullptr;
    // A single sporadic capture has no frame history, so adapt essentially
    // instantly; otherwise eye adaptation sits at an unconverged default.
    PostProcessSettings.bOverride_AutoExposureSpeedUp = true;
    PostProcessSettings.AutoExposureSpeedUp = 100.0f;
    PostProcessSettings.bOverride_AutoExposureSpeedDown = true;
    PostProcessSettings.AutoExposureSpeedDown = 100.0f;
    PostProcessSettings.bOverride_AutoExposureBias = true;
    PostProcessSettings.AutoExposureBias = ExposureBias;
    PostProcessSettings.bOverride_BloomIntensity = true;
    PostProcessSettings.BloomIntensity = FMath::Max(0.0f, BloomIntensity);
    PostProcessSettings.bOverride_BloomThreshold = true;
    PostProcessSettings.BloomThreshold = 1.0f;
}

void UDroneCaptureComponent::SetFastUnshadowedRendering(bool bEnabled)
{
    if (bEnabled)
    {
        if (!bHasSavedFastProfileShowFlags)
        {
            bSavedDynamicShadows = ShowFlags.DynamicShadows;
            bSavedAmbientOcclusion = ShowFlags.AmbientOcclusion;
            bSavedBloom = ShowFlags.Bloom;
            bHasSavedFastProfileShowFlags = true;
        }

        ShowFlags.SetDynamicShadows(false);
        ShowFlags.SetAmbientOcclusion(false);
        ShowFlags.SetBloom(false);
        return;
    }

    if (bHasSavedFastProfileShowFlags)
    {
        ShowFlags.SetDynamicShadows(bSavedDynamicShadows);
        ShowFlags.SetAmbientOcclusion(bSavedAmbientOcclusion);
        ShowFlags.SetBloom(bSavedBloom);
        bHasSavedFastProfileShowFlags = false;
    }
}

void UDroneCaptureComponent::CaptureTick(int32 Tick, bool bRender, float SecondsPerTick)
{
    if (!bRender)
    {
        // Replay advances the same capture schedule, without submitting a
        // scene that no consumer will read.
        CachedCapturePixels.Reset();
        PendingReadbackTick = -1;
        LastCaptureTick = Tick;
        return;
    }
    if (!CaptureTarget)
    {
        return;
    }

    // Reset any view history: a capture's appearance cannot depend on the
    // spectator frame rate or how often this camera happened to render.
    // A view state is needed for Unreal's physical exposure pass, but must
    // start fresh for this observation. Destroy uses deferred render cleanup.
    for (FSceneViewStateReference& State : ViewStates)
    {
        State.Destroy();
    }
    ViewStates.Empty();
    if (!SensorTimeExtension)
    {
        SensorTimeExtension = MakeShared<FDroneSensorTimeExtension, ESPMode::ThreadSafe>();
        SceneViewExtensions.Add(SensorTimeExtension);
    }
    SensorTimeExtension->CaptureTime = FGameTime::CreateUndilated(
        static_cast<double>(Tick) * SecondsPerTick, SecondsPerTick);
    SensorTimeExtension->CaptureFrame = static_cast<uint32>(Tick);
    const bool bGBufferFlag = FParse::Param(FCommandLine::Get(), TEXT("ThesisSensorDiagnosticGBuffer"));
    const bool bDumpGpuFlag = FParse::Param(FCommandLine::Get(), TEXT("ThesisSensorDiagnosticDumpGPU"));
    int32 DumpGpuDroneIndex = 0;
    FParse::Value(FCommandLine::Get(), TEXT("ThesisSensorDiagnosticDumpDrone="), DumpGpuDroneIndex);
    int32 DumpGpuTick = 1;
    FParse::Value(FCommandLine::Get(), TEXT("ThesisSensorDiagnosticDumpTick="), DumpGpuTick);
    const bool bGBufferDiagnostic = bGBufferFlag &&
        FParse::Param(FCommandLine::Get(), TEXT("ThesisSensorDiagnosticSceneColorSceneDepth")) &&
        CaptureSource == SCS_SceneColorSceneDepth &&
        !FParse::Param(FCommandLine::Get(), TEXT("ThesisSensorDiagnosticBaseColor")) &&
        !FParse::Param(FCommandLine::Get(), TEXT("ThesisSensorDiagnosticNormal")) &&
        !FParse::Param(FCommandLine::Get(), TEXT("ThesisSensorDiagnosticSceneColorHDR"));
    if (bGBufferFlag && !bGBufferDiagnostic)
    {
        UE_LOG(LogTemp, Error, TEXT("Thesis GBuffer diagnostic requires SceneColorSceneDepth as the sole capture-source diagnostic."));
    }
    if (bDumpGpuFlag && !bGBufferDiagnostic)
    {
        UE_LOG(LogTemp, Error, TEXT("Thesis DumpGPU diagnostic requires GBuffer plus SceneColorSceneDepth diagnostics."));
    }
    if (bDumpGpuFlag && DumpGpuDroneIndex < 0)
    {
        UE_LOG(LogTemp, Error, TEXT("Thesis DumpGPU diagnostic refused: DumpDrone index must be nonnegative, got %d."), DumpGpuDroneIndex);
    }
    FString CaptureOwnerName;
    FString DumpGpuOwnerName;
    if (bGBufferDiagnostic)
    {
        const AActor* OwnerActor = GetOwner();
        CaptureOwnerName = OwnerActor ? OwnerActor->GetName() : TEXT("Drone");
        DumpGpuOwnerName = FString::Printf(TEXT("Drone_%d"), DumpGpuDroneIndex);
        if (bDumpGpuFlag && DumpGpuDroneIndex >= 0 && CaptureOwnerName == DumpGpuOwnerName && Tick == DumpGpuTick)
        {
            // Drain older view commands before arming this sensor's RDG dump/readback state.
            FlushRenderingCommands();
        }
        const FString OwnerName = CaptureOwnerName;
        const FString OutputDir = DebugOutputDirectory.IsEmpty()
            ? FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("Captures"))
            : DebugOutputDirectory;
        SensorTimeExtension->ArmGBufferDiagnostic(OutputDir, OwnerName, Tick, static_cast<uint32>(Tick),
            FIntPoint(CaptureTarget->SizeX, CaptureTarget->SizeY),
            FParse::Param(FCommandLine::Get(), TEXT("ThesisSensorDiagnosticGBufferMismatchROI")),
            FParse::Param(FCommandLine::Get(), TEXT("ThesisSensorDiagnosticRepeatSameTick")) ? TEXT("first") : TEXT(""));
    }
    bCameraCutThisFrame = true;
#if WITH_DUMPGPU
    TUniquePtr<UE::RenderCore::DumpGPU::FDumpScope> DumpGpuScope;
    static bool bDumpGpuCaptureStarted = false;
    if (bDumpGpuFlag && bGBufferDiagnostic && DumpGpuDroneIndex >= 0 && !bDumpGpuCaptureStarted &&
        CaptureOwnerName == DumpGpuOwnerName && Tick == DumpGpuTick)
    {
        const IConsoleManager& Console = IConsoleManager::Get();
        const IConsoleVariable* RootCVar = Console.FindConsoleVariable(TEXT("r.DumpGPU.Root"));
        const IConsoleVariable* TextureCVar = Console.FindConsoleVariable(TEXT("r.DumpGPU.Texture"));
        const IConsoleVariable* BufferCVar = Console.FindConsoleVariable(TEXT("r.DumpGPU.Buffer"));
        const IConsoleVariable* PassParametersCVar = Console.FindConsoleVariable(TEXT("r.DumpGPU.PassParameters"));
        const IConsoleVariable* UploadCVar = Console.FindConsoleVariable(TEXT("r.DumpGPU.Upload"));
        const IConsoleVariable* ExploreCVar = Console.FindConsoleVariable(TEXT("r.DumpGPU.Explore"));
        const IConsoleVariable* FixedTickRateCVar = Console.FindConsoleVariable(TEXT("r.DumpGPU.FixedTickRate"));
        const IConsoleVariable* DelayCVar = Console.FindConsoleVariable(TEXT("r.DumpGPU.Delay"));
        const IConsoleVariable* FrameDelayCVar = Console.FindConsoleVariable(TEXT("r.DumpGPU.FrameDelay"));
        const IConsoleVariable* DirectoryCVar = Console.FindConsoleVariable(TEXT("r.DumpGPU.Directory"));
        const FString DumpRoot = RootCVar ? RootCVar->GetString() : FString();
        const FString DumpDirectory = DirectoryCVar ? DirectoryCVar->GetString() : FString();
        const bool bMetadataOnlyShadowMode = DumpRoot == TEXT("*Shadow*") &&
            TextureCVar && TextureCVar->GetInt() == 1 && BufferCVar && BufferCVar->GetInt() == 1;
        const bool bShadowMaskBinaryMode = DumpRoot == TEXT("*CompositeVirtualShadowMapMask*") &&
            TextureCVar && TextureCVar->GetInt() == 2 && BufferCVar && BufferCVar->GetInt() == 1;
        const bool bProjectionFullDumpMode = DumpRoot == TEXT("VirtualShadowMapProjection(Input:Substrate)") &&
            TextureCVar && TextureCVar->GetInt() == 2 && BufferCVar && BufferCVar->GetInt() == 2;
        const bool bSkyLightingDumpMode = DumpRoot == TEXT("ReflectionEnvironmentAndSky*") &&
            TextureCVar && TextureCVar->GetInt() == 2 && BufferCVar && BufferCVar->GetInt() == 2;
        const bool bDumpSettingsValid = !DumpDirectory.IsEmpty() && (bMetadataOnlyShadowMode || bShadowMaskBinaryMode || bProjectionFullDumpMode || bSkyLightingDumpMode) &&
            PassParametersCVar && PassParametersCVar->GetInt() == 1 && UploadCVar && UploadCVar->GetInt() == 0 &&
            ExploreCVar && ExploreCVar->GetInt() == 0 && FixedTickRateCVar && FixedTickRateCVar->GetFloat() == 0.0f &&
            DelayCVar && DelayCVar->GetFloat() == 0.0f && FrameDelayCVar && FrameDelayCVar->GetInt() == 0;
        if (!bDumpSettingsValid)
        {
            UE_LOG(LogTemp, Error, TEXT("Thesis DumpGPU diagnostic refused: expected Directory plus Shadow metadata, composite-mask binary, projection full-binary, or sky-lighting full-binary mode, with PassParameters=1 Upload=0 Explore=0 FixedTickRate=0 Delay=0 FrameDelay=0; got Directory='%s' Root='%s' Texture=%d Buffer=%d PassParameters=%d Upload=%d Explore=%d FixedTickRate=%g Delay=%g FrameDelay=%d."),
                *DumpDirectory, *DumpRoot, TextureCVar ? TextureCVar->GetInt() : INDEX_NONE,
                BufferCVar ? BufferCVar->GetInt() : INDEX_NONE, PassParametersCVar ? PassParametersCVar->GetInt() : INDEX_NONE,
                UploadCVar ? UploadCVar->GetInt() : INDEX_NONE, ExploreCVar ? ExploreCVar->GetInt() : INDEX_NONE,
                FixedTickRateCVar ? FixedTickRateCVar->GetFloat() : -1.0f, DelayCVar ? DelayCVar->GetFloat() : -1.0f,
                FrameDelayCVar ? FrameDelayCVar->GetInt() : INDEX_NONE);
        }
        else
        {
            // Finish any earlier view work before this one-frame, filtered RDG capture.
            FlushRenderingCommands();
            if (bProjectionFullDumpMode)
            {
                UE_LOG(LogTemp, Display, TEXT("Thesis DumpGPU projection diagnostic uses Texture=2 and Buffer=2: UE 5.7 AddDumpBufferPass reads r.DumpGPU.Texture for buffer binaries, so this captures all projection-pass input texture and buffer payloads."));
            }
            UE_LOG(LogTemp, Display, TEXT("Thesis DumpGPU diagnostic armed: target=%s actual=%s tick=%d Root=%s directory=%s."),
                *DumpGpuOwnerName, *CaptureOwnerName, Tick, *DumpRoot, *DumpDirectory);
            DumpGpuScope = MakeUnique<UE::RenderCore::DumpGPU::FDumpScope>();
            bDumpGpuCaptureStarted = true;
        }
    }
#else
    if (bDumpGpuFlag)
    {
        UE_LOG(LogTemp, Error, TEXT("Thesis DumpGPU diagnostic unavailable: this engine build has WITH_DUMPGPU=0."));
    }
#endif
    if (bWarmFirstCapture && !bFirstCaptureWarmed)
    {
        // Trigger view-specific material permutations at this SAME simulation
        // tick, then wait for their resources before producing the observation.
        // This priming render is never sent to the detector or the recorder.
        CaptureScene();
        FlushRenderingCommands();
#if WITH_EDITOR
        FAssetCompilingManager::Get().FinishAllCompilation();
        if (GShaderCompilingManager) GShaderCompilingManager->FinishAllCompilation();
#endif
        ENQUEUE_RENDER_COMMAND(ThesisCameraWaitForPipelines)(
            [](FRHICommandListImmediate&) { PipelineStateCache::WaitForAllTasks(); });
        FlushRenderingCommands();
        GetWorld()->SendAllEndOfFrameUpdates();
        FlushRenderingCommands();
        for (FSceneViewStateReference& State : ViewStates) State.Destroy();
        ViewStates.Empty();
        // CaptureScene consumes the cut flag. The real observation must start
        // with the same fresh-view state as every subsequent capture.
        bCameraCutThisFrame = true;
        bFirstCaptureWarmed = true;
    }
    CaptureScene();
    QueueCaptureReadback(Tick);
    const bool bRepeatSameTickDiagnostic = FParse::Param(FCommandLine::Get(), TEXT("ThesisSensorDiagnosticRepeatSameTick"));
    if (bRepeatSameTickDiagnostic)
    {
        const AActor* OwnerActor = GetOwner();
        const bool bSceneColorDepthDiagnostic =
            CaptureSource == SCS_SceneColorSceneDepth &&
            FParse::Param(FCommandLine::Get(), TEXT("ThesisSensorDiagnosticSceneColorSceneDepth")) &&
            !FParse::Param(FCommandLine::Get(), TEXT("ThesisSensorDiagnosticBaseColor")) &&
            !FParse::Param(FCommandLine::Get(), TEXT("ThesisSensorDiagnosticNormal")) &&
            !FParse::Param(FCommandLine::Get(), TEXT("ThesisSensorDiagnosticSceneColorHDR"));
        const bool bTargetCameraAndTick = Tick == 1 && OwnerActor && OwnerActor->GetName() == TEXT("Drone_0");
        if (bTargetCameraAndTick && !bSceneColorDepthDiagnostic)
        {
            UE_LOG(LogTemp, Error, TEXT("Same-tick repeat diagnostic requires an exact SceneColorSceneDepth capture for Drone_0 at tick 1."));
        }
        else if (bTargetCameraAndTick)
        {
            // Preserve the first readback, then render again with the same
            // simulation tick/time and unchanged world state. The blocking
            // readback drains the first capture before the second is submitted.
            FTextureRenderTargetResource* Resource = CaptureTarget->GameThread_GetRenderTargetResource();
            TArray<FLinearColor> FirstPixels;
            const FReadSurfaceDataFlags ReadFlags(RCM_MinMax);
            if (!Resource || !Resource->ReadLinearColorPixels(FirstPixels, ReadFlags) ||
                FirstPixels.Num() != CaptureTarget->SizeX * CaptureTarget->SizeY)
            {
                UE_LOG(LogTemp, Error, TEXT("Same-tick repeat diagnostic failed to read the first capture at tick %d."), Tick);
            }
            else
            {
                const int64 ExpectedBytes = static_cast<int64>(FirstPixels.Num()) * sizeof(FLinearColor);
                TArray64<uint8> FirstBytes;
                FirstBytes.SetNumUninitialized(ExpectedBytes);
                FMemory::Memcpy(FirstBytes.GetData(), FirstPixels.GetData(), ExpectedBytes);
                const FString Dir = DebugOutputDirectory.IsEmpty()
                    ? FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("Captures"))
                    : DebugOutputDirectory;
                IFileManager::Get().MakeDirectory(*Dir, true);
                const FString FirstPath = FPaths::Combine(Dir, TEXT("Drone_0_t000001_same_tick_first.rgba32f"));
                const FString FirstMetadataPath = FirstPath + TEXT(".meta");
                const FString FirstMetadata = FString::Printf(
                    TEXT("width=%d\nheight=%d\nformat=PF_A32B32G32R32F\nchannels=FLinearColor RGBA\nbytes_per_pixel=%d\ncompression=none\nread_flags=RCM_MinMax\ncapture_source=SCS_SceneColorSceneDepth\nsimulation_tick=%d\ncapture_frame=%u\n"),
                    CaptureTarget->SizeX, CaptureTarget->SizeY, static_cast<int32>(sizeof(FLinearColor)), Tick, SensorTimeExtension->CaptureFrame);
                if (!FFileHelper::SaveArrayToFile(FirstBytes, *FirstPath) ||
                    !FFileHelper::SaveStringToFile(FirstMetadata, *FirstMetadataPath))
                {
                    UE_LOG(LogTemp, Error, TEXT("Same-tick repeat diagnostic failed to save first capture: %s."), *FirstPath);
                }
                else
                {
                    UE_LOG(LogTemp, Display, TEXT("Same-tick repeat diagnostic saved first frame at unchanged simulation tick %d: %s."), Tick, *FirstPath);
                }
                const FString GBufferOutputDir = DebugOutputDirectory.IsEmpty()
                    ? FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("Captures"))
                    : DebugOutputDirectory;
                if (bGBufferDiagnostic)
                {
                    if (!SensorTimeExtension->SaveGBufferDiagnostic())
                    {
                        UE_LOG(LogTemp, Error, TEXT("Same-tick repeat diagnostic could not save first-capture GBuffer for tick %d."), Tick);
                    }
                    else
                    {
                        SensorTimeExtension->ArmGBufferDiagnostic(GBufferOutputDir, CaptureOwnerName, Tick,
                            static_cast<uint32>(Tick), FIntPoint(CaptureTarget->SizeX, CaptureTarget->SizeY),
                            FParse::Param(FCommandLine::Get(), TEXT("ThesisSensorDiagnosticGBufferMismatchROI")), TEXT("second"));
                    }
                }
                // Match the view-state reset performed before the first render.
                for (FSceneViewStateReference& State : ViewStates)
                {
                    State.Destroy();
                }
                ViewStates.Empty();
                bCameraCutThisFrame = true;
                CaptureScene();
                // The repeat diagnostic already blocks on and saves the first
                // image. Queue only the final, consumer-visible image.
                QueueCaptureReadback(Tick);
                UE_LOG(LogTemp, Display, TEXT("Same-tick repeat diagnostic submitted second Drone_0 capture at unchanged simulation tick %d and capture frame %u."),
                    Tick, SensorTimeExtension->CaptureFrame);
            }
        }
    }
    if (bGBufferDiagnostic && !SensorTimeExtension->SaveGBufferDiagnostic())
    {
        UE_LOG(LogTemp, Error, TEXT("Thesis GBuffer diagnostic failed for capture tick %d."), Tick);
    }
#if WITH_DUMPGPU
    DumpGpuScope.Reset();
#endif
    LastCaptureTick = Tick;

    if (bSavePngForDebug)
    {
        SavePng(Tick);
    }
}

bool UDroneCaptureComponent::ReadLastCapture(TArray<FColor>& OutPixels, int32& OutWidth, int32& OutHeight, int32& OutTick)
{
    const TArray<FColor>* Pixels = nullptr;
    if (!ReadLastCaptureView(Pixels, OutWidth, OutHeight, OutTick)) return false;
    OutPixels = *Pixels;
    return true;
}

bool UDroneCaptureComponent::ReadLastCaptureView(const TArray<FColor>*& OutPixels, int32& OutWidth, int32& OutHeight, int32& OutTick)
{
    check(IsInGameThread());
    OutPixels = nullptr;
    if (!CaptureTarget || LastCaptureTick < 0)
    {
        return false;
    }

    // This opt-out supports same-build byte-for-byte A/B checks against the
    // original synchronous ReadPixels path.
    const bool bLegacyReadback = FParse::Param(FCommandLine::Get(), TEXT("ThesisLegacyCaptureReadback"));
    // PF_B8G8R8A8 is the normal sensor format. For legacy comparisons and
    // float diagnostic targets, retain Unreal's full conversion path exactly.
    if (bLegacyReadback || CaptureTarget->GetFormat() != PF_B8G8R8A8)
    {
        FTextureRenderTargetResource* Resource = CaptureTarget->GameThread_GetRenderTargetResource();
        if (!Resource || !Resource->ReadPixels(CachedCapturePixels)) return false;
    }
    else
    {
        if (!bCaptureReadbackPending && CachedCapturePixels.Num() == CaptureTarget->SizeX * CaptureTarget->SizeY &&
            PendingReadbackTick == LastCaptureTick)
        {
            // The cache already contains this exact capture.
        }
        else
        {
            if (!ConsumeCaptureReadback() || PendingReadbackTick != LastCaptureTick) return false;
        }
    }

    OutWidth = CaptureTarget->SizeX;
    OutHeight = CaptureTarget->SizeY;
    OutTick = LastCaptureTick;
    OutPixels = &CachedCapturePixels;
    return true;
}

void UDroneCaptureComponent::QueueCaptureReadback(int32 Tick)
{
    check(IsInGameThread());
    if (FParse::Param(FCommandLine::Get(), TEXT("ThesisLegacyCaptureReadback")))
    {
        if (bCaptureReadbackPending) ConsumeCaptureReadback();
        CachedCapturePixels.Reset();
        PendingReadbackTick = -1;
        return;
    }
    if (!CaptureTarget || CaptureTarget->GetFormat() != PF_B8G8R8A8) return;

    // A prior image can remain pending if it had no consumer. Retire it before
    // reusing the single per-component staging readback.
    if (bCaptureReadbackPending && !ConsumeCaptureReadback())
    {
        UE_LOG(LogTemp, Error, TEXT("Thesis camera readback could not retire prior image before tick %d."), Tick);
        CachedCapturePixels.Reset();
        PendingReadbackTick = -1;
        return;
    }
    // A new capture invalidates the old image even when target lookup or the
    // later GPU copy fails. This also disambiguates two captures at one tick.
    CachedCapturePixels.Reset();
    PendingReadbackTick = -1;
    if (!CaptureReadback)
    {
        CaptureReadback = MakeShared<FRHIGPUTextureReadback, ESPMode::ThreadSafe>(TEXT("Thesis.DroneCaptureReadback"));
    }

    FTextureRenderTargetResource* Resource = CaptureTarget->GameThread_GetRenderTargetResource();
    if (!Resource) return;
    const int32 Width = CaptureTarget->SizeX;
    const int32 Height = CaptureTarget->SizeY;
    TSharedPtr<FRHIGPUTextureReadback, ESPMode::ThreadSafe> Readback = CaptureReadback;
    ENQUEUE_RENDER_COMMAND(ThesisQueueDroneCaptureReadback)(
        [Resource, Readback](FRHICommandListImmediate& RHICmdList)
        {
            // Resolve the texture on the render thread, after CaptureScene's
            // queued render has initialized or updated the target resource.
            const FTextureRHIRef& Texture = Resource->GetShaderResourceTexture();
            if (Texture.IsValid())
            {
                // Register the external render target with RDG so its state
                // transition to CopySrc and restoration are tracked correctly.
                FRDGBuilder GraphBuilder(RHICmdList);
                FRDGTextureRef SourceTexture = GraphBuilder.RegisterExternalTexture(
                    CreateRenderTarget(Texture.GetReference(), TEXT("DroneCaptureReadbackSource")));
                AddEnqueueCopyPass(GraphBuilder, Readback.Get(), SourceTexture);
                GraphBuilder.Execute();
            }
        });
    CaptureReadbackFence.BeginFence();
    PendingReadbackWidth = Width;
    PendingReadbackHeight = Height;
    PendingReadbackTick = Tick;
    bCaptureReadbackPending = true;
}

bool UDroneCaptureComponent::ConsumeCaptureReadback()
{
    check(IsInGameThread());
    if (!bCaptureReadbackPending || !CaptureReadback) return false;

    // Do not leave an older same-tick cache available if this copy fails.
    CachedCapturePixels.Reset();

    const bool bLegacyConsume = FParse::Param(FCommandLine::Get(), TEXT("ThesisLegacyCaptureConsume"));
    if (bLegacyConsume) CaptureReadbackFence.Wait();
    struct FCaptureCopyResult
    {
        TArray<FColor> Pixels;
        bool bSuccess = false;
    };
    TSharedRef<FCaptureCopyResult, ESPMode::ThreadSafe> Result = MakeShared<FCaptureCopyResult, ESPMode::ThreadSafe>();
    // Retain the staging CPU allocation across frames instead of allocating a
    // fresh image at every consumer. Ownership returns after the fence below.
    if (!bLegacyConsume) Result->Pixels = MoveTemp(CachedCapturePixels);
    TSharedPtr<FRHIGPUTextureReadback, ESPMode::ThreadSafe> Readback = CaptureReadback;
    const int32 Width = PendingReadbackWidth;
    const int32 Height = PendingReadbackHeight;
    ENQUEUE_RENDER_COMMAND(ThesisConsumeDroneCaptureReadback)(
        [Readback, Result, Width, Height](FRHICommandListImmediate& RHICmdList)
        {
            // The caller consumes at the same scheduled tick; GPU latency may
            // slow that tick, but can never move delivery to a later tick.
            if (!Readback->IsReady())
            {
                RHICmdList.SubmitAndBlockUntilGPUIdle();
                if (!Readback->IsReady()) return;
            }
            int32 RowPitchPixels = 0;
            int32 BufferHeight = 0;
            const uint8* Source = static_cast<const uint8*>(Readback->Lock(RowPitchPixels, &BufferHeight));
            if (!Source) return;
            if (RowPitchPixels >= Width && BufferHeight >= Height)
            {
                Result->Pixels.SetNumUninitialized(Width * Height);
                for (int32 Y = 0; Y < Height; ++Y)
                {
                    // UE 5.7's PF_B8G8R8A8 ReadPixels conversion copies each
                    // row byte-for-byte into FColor, including alpha.
                    FMemory::Memcpy(Result->Pixels.GetData() + Y * Width,
                        Source + static_cast<int64>(Y) * RowPitchPixels * sizeof(FColor),
                        static_cast<SIZE_T>(Width) * sizeof(FColor));
                }
                Result->bSuccess = true;
            }
            Readback->Unlock();
        });
    if (bLegacyConsume)
    {
        FlushRenderingCommands();
    }
    else
    {
        // Queue and consume commands use the same ordered render pipe. A
        // single fence after the CPU copy covers both; GPU readiness is still
        // checked and blocked above at the original simulation consumer tick.
        FRenderCommandFence CopyComplete;
        CopyComplete.BeginFence();
        CopyComplete.Wait();
    }

    bCaptureReadbackPending = false;
    if (!Result->bSuccess || Result->Pixels.Num() != Width * Height)
    {
        UE_LOG(LogTemp, Error, TEXT("Thesis camera GPU readback failed at tick %d (%dx%d)."), PendingReadbackTick, Width, Height);
        return false;
    }
    CachedCapturePixels = MoveTemp(Result->Pixels);
    return true;
}

void UDroneCaptureComponent::BeginDestroy()
{
    check(IsInGameThread());
    if (bCaptureReadbackPending)
    {
        // Ensure the render command has resolved the texture and submitted its
        // staging copy before deferring readback destruction to the RT.
        CaptureReadbackFence.Wait();
    }

    TSharedPtr<FRHIGPUTextureReadback, ESPMode::ThreadSafe> Readback = MoveTemp(CaptureReadback);
    if (Readback)
    {
        const bool bWaitForGPU = bCaptureReadbackPending;
        ENQUEUE_RENDER_COMMAND(ThesisReleaseDroneCaptureReadback)(
            [Readback = MoveTemp(Readback), bWaitForGPU](FRHICommandListImmediate& RHICmdList) mutable
            {
                if (bWaitForGPU)
                {
                    RHICmdList.SubmitAndBlockUntilGPUIdle();
                }
                // Captured readback is released here on the render thread,
                // after any in-flight GPU copy has completed.
            });
        FlushRenderingCommands();
    }
    bCaptureReadbackPending = false;
    CachedCapturePixels.Reset();
    Super::BeginDestroy();
}

bool UDroneCaptureComponent::ReadDepthPass(const UPrimitiveComponent* OnlyComponent,
    TArray<float>& OutDepth, int32& OutWidth, int32& OutHeight)
{
    if (!CaptureTarget || LastCaptureTick < 0 || !SensorTimeExtension)
    {
        return false;
    }

    if (!DepthCaptureTarget || DepthCaptureTarget->SizeX != CaptureTarget->SizeX || DepthCaptureTarget->SizeY != CaptureTarget->SizeY)
    {
        DepthCaptureTarget = NewObject<UTextureRenderTarget2D>(this, TEXT("DroneDepthCaptureRT"));
        DepthCaptureTarget->InitCustomFormat(CaptureTarget->SizeX, CaptureTarget->SizeY, PF_R32_FLOAT, true);
        DepthCaptureTarget->ClearColor = FLinearColor(0.0f, 0.0f, 0.0f, 0.0f);
        DepthCaptureTarget->UpdateResourceImmediate(true);
    }

    // Keep the original RGB setup intact. The source reads linear scene depth
    // into R; unlike DeviceDepth it needs no packed-depth decode.
    const ESceneCaptureSource PreviousSource = CaptureSource;
    UTextureRenderTarget2D* PreviousTarget = TextureTarget;
    const ESceneCapturePrimitiveRenderMode PreviousMode = PrimitiveRenderMode;
    const FEngineShowFlags PreviousShowFlags = ShowFlags;
    TArray<TWeakObjectPtr<UPrimitiveComponent>> PreviousComponents = ShowOnlyComponents;
    TArray<TObjectPtr<AActor>> PreviousActors = ShowOnlyActors;

    CaptureSource = SCS_SceneDepth;
    TextureTarget = DepthCaptureTarget;
    ShowFlags.SetAntiAliasing(false);
    ShowFlags.SetTemporalAA(false);
    ShowFlags.SetMotionBlur(false);
    ShowFlags.SetPostProcessing(false);
    ShowFlags.SetFog(false);
    ShowFlags.SetVolumetricFog(false);
    ShowFlags.SetAtmosphere(false);
    ShowFlags.SetTranslucency(false);
    if (OnlyComponent)
    {
        ClearShowOnlyComponents();
        ShowOnlyActors.Reset();
        PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_UseShowOnlyList;
        ShowOnlyComponent(const_cast<UPrimitiveComponent*>(OnlyComponent));
    }

    for (FSceneViewStateReference& State : ViewStates)
    {
        State.Destroy();
    }
    ViewStates.Empty();
    bCameraCutThisFrame = true;
    CaptureScene();
    FlushRenderingCommands();

    bool bSuccess = false;
    FTextureRenderTargetResource* Resource = DepthCaptureTarget->GameThread_GetRenderTargetResource();
    TArray<FLinearColor> Pixels;
    const FReadSurfaceDataFlags ReadFlags(RCM_MinMax);
    if (Resource && Resource->ReadLinearColorPixels(Pixels, ReadFlags) &&
        Pixels.Num() == DepthCaptureTarget->SizeX * DepthCaptureTarget->SizeY)
    {
        OutDepth.SetNumUninitialized(Pixels.Num());
        for (int32 Index = 0; Index < Pixels.Num(); ++Index)
        {
            OutDepth[Index] = Pixels[Index].R;
        }
        OutWidth = DepthCaptureTarget->SizeX;
        OutHeight = DepthCaptureTarget->SizeY;
        bSuccess = true;
    }

    // Restore the capture component even when readback fails. Rebuild its
    // prior show-only list through the API so render proxies are notified.
    if (OnlyComponent)
    {
        ClearShowOnlyComponents();
        ShowOnlyActors.Reset();
        for (const TWeakObjectPtr<UPrimitiveComponent>& Component : PreviousComponents)
        {
            if (Component.IsValid()) ShowOnlyComponent(Component.Get());
        }
        ShowOnlyActors = MoveTemp(PreviousActors);
    }
    PrimitiveRenderMode = PreviousMode;
    CaptureSource = PreviousSource;
    TextureTarget = PreviousTarget;
    ShowFlags = PreviousShowFlags;
    for (FSceneViewStateReference& State : ViewStates)
    {
        State.Destroy();
    }
    ViewStates.Empty();
    bCameraCutThisFrame = true;
    return bSuccess;
}

void UDroneCaptureComponent::SavePng(int32 Tick)
{
    const bool bDiagnosticBaseColor = FParse::Param(FCommandLine::Get(), TEXT("ThesisSensorDiagnosticBaseColor"));
    const bool bDiagnosticNormal = FParse::Param(FCommandLine::Get(), TEXT("ThesisSensorDiagnosticNormal"));
    const bool bDiagnosticSceneColorHDR = FParse::Param(FCommandLine::Get(), TEXT("ThesisSensorDiagnosticSceneColorHDR"));
    const bool bDiagnosticSceneColorSceneDepth = FParse::Param(FCommandLine::Get(), TEXT("ThesisSensorDiagnosticSceneColorSceneDepth"));
    const int32 DiagnosticFlagCount = static_cast<int32>(bDiagnosticBaseColor) + static_cast<int32>(bDiagnosticNormal) + static_cast<int32>(bDiagnosticSceneColorHDR) + static_cast<int32>(bDiagnosticSceneColorSceneDepth);
    const bool bExactFloatDiagnostic = DiagnosticFlagCount == 1 && (bDiagnosticNormal || bDiagnosticSceneColorHDR || bDiagnosticSceneColorSceneDepth);
    if (bExactFloatDiagnostic)
    {
        FTextureRenderTargetResource* Resource = CaptureTarget
            ? CaptureTarget->GameThread_GetRenderTargetResource()
            : nullptr;
        TArray<FLinearColor> FloatPixels;
        const FReadSurfaceDataFlags ReadFlags(RCM_MinMax);
        if (!Resource || !Resource->ReadLinearColorPixels(FloatPixels, ReadFlags) ||
            FloatPixels.Num() != CaptureTarget->SizeX * CaptureTarget->SizeY)
        {
            UE_LOG(LogTemp, Error, TEXT("Thesis sensor diagnostic failed: exact float readback failed at tick %d."), Tick);
            return;
        }

        const int64 ExpectedBytes = static_cast<int64>(FloatPixels.Num()) * sizeof(FLinearColor);
        TArray64<uint8> RawFloatBytes;
        RawFloatBytes.SetNumUninitialized(ExpectedBytes);
        FMemory::Memcpy(RawFloatBytes.GetData(), FloatPixels.GetData(), ExpectedBytes);

        const FString Dir = DebugOutputDirectory.IsEmpty()
            ? FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("Captures"))
            : DebugOutputDirectory;
        IFileManager::Get().MakeDirectory(*Dir, true);
        const AActor* OwnerActor = GetOwner();
        const FString BaseName = FString::Printf(TEXT("%s_t%06d"),
            OwnerActor ? *OwnerActor->GetName() : TEXT("Drone"), Tick);
        const FString RawPath = FPaths::Combine(Dir, BaseName + TEXT(".rgba32f"));
        const FString MetadataPath = FPaths::Combine(Dir, BaseName + TEXT(".rgba32f.meta"));
        const TCHAR* CaptureSourceName = bDiagnosticNormal ? TEXT("SCS_Normal") : (bDiagnosticSceneColorHDR ? TEXT("SCS_SceneColorHDR") : TEXT("SCS_SceneColorSceneDepth"));
        FString Metadata = FString::Printf(
            TEXT("width=%d\nheight=%d\nformat=PF_A32B32G32R32F\nchannels=FLinearColor RGBA\nbytes_per_pixel=%d\ncompression=none\nread_flags=RCM_MinMax\ncapture_source=%s\n"),
            CaptureTarget->SizeX, CaptureTarget->SizeY, static_cast<int32>(sizeof(FLinearColor)), CaptureSourceName);
        if (FParse::Param(FCommandLine::Get(), TEXT("ThesisSensorDiagnosticRepeatSameTick")))
        {
            Metadata += FString::Printf(TEXT("simulation_tick=%d\ncapture_frame=%u\n"), Tick, SensorTimeExtension ? SensorTimeExtension->CaptureFrame : 0u);
        }
        if (!FFileHelper::SaveArrayToFile(RawFloatBytes, *RawPath) ||
            !FFileHelper::SaveStringToFile(Metadata, *MetadataPath))
        {
            UE_LOG(LogTemp, Error, TEXT("Thesis sensor diagnostic failed: exact float sidecar write failed at tick %d."), Tick);
            return;
        }
    }

    TArray<FColor> Pixels;
    int32 W = 0, H = 0, SrcTick = -1;
    if (!ReadLastCapture(Pixels, W, H, SrcTick))
    {
        return;
    }

    for (FColor& P : Pixels)
    {
        P.A = 255;
    }

    TArray64<uint8> Png;
    FImageUtils::PNGCompressImageArray(W, H, Pixels, Png);

    const FString Dir = DebugOutputDirectory.IsEmpty()
        ? FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("Captures"))
        : DebugOutputDirectory;
    IFileManager::Get().MakeDirectory(*Dir, true);

    const AActor* OwnerActor = GetOwner();
    const FString Name = FString::Printf(TEXT("%s_t%06d.png"),
        OwnerActor ? *OwnerActor->GetName() : TEXT("Drone"), Tick);
    FFileHelper::SaveArrayToFile(Png, *FPaths::Combine(Dir, Name));
}
