#pragma once

#include "SceneView.h"
#include "SceneViewExtension.h"
#include "SimulationManager.h"

// Applied after level/camera blending, only to the player's live view. Keeping
// this out of world post-process volumes preserves sensor capture calibration.
inline void ApplyLiveViewPresentation(FPostProcessSettings& Settings, bool bCapture, float ExposureCompensation)
{
    if (bCapture) return;
    Settings.AutoExposureBias += FMath::Clamp(ExposureCompensation, -5.0f, 5.0f);
    Settings.BloomIntensity = FMath::Min(Settings.BloomIntensity, 0.15f);
    Settings.BloomThreshold = FMath::Max(Settings.BloomThreshold, 1.0f);
    Settings.LocalExposureHighlightContrastScale = 1.0f;
    Settings.LocalExposureShadowContrastScale = 1.0f;
}

class FLiveViewPresentationExtension final : public FWorldSceneViewExtension
{
public:
    FLiveViewPresentationExtension(const FAutoRegister& AutoRegister, UWorld* World,
        ASimulationManager* InManager)
        : FWorldSceneViewExtension(AutoRegister, World), Manager(InManager) {}

    virtual void SetupView(FSceneViewFamily& Family, FSceneView& View) override
    {
        if (const ASimulationManager* Owner = Manager.Get(); Owner && Owner->bCorrectLiveViewLighting)
            ApplyLiveViewPresentation(View.FinalPostProcessSettings,
                View.bIsSceneCapture || View.bIsReflectionCapture, Owner->LiveViewExposureCompensation);
    }

private:
    TWeakObjectPtr<ASimulationManager> Manager;
};
