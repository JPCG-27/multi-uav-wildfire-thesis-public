#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "../World/LiveViewPresentation.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLiveViewPresentationTest, "Thesis.Environment.LiveViewPresentation",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLiveViewPresentationTest::RunTest(const FString& Parameters)
{
    FPostProcessSettings Level;
    Level.AutoExposureBias = 0.15f;
    Level.BloomIntensity = 1.65f;
    Level.BloomThreshold = 0.85f;
    Level.LocalExposureHighlightContrastScale = 0.35f;
    Level.LocalExposureShadowContrastScale = 0.20f;
    FPostProcessSettings Capture = Level;
    ApplyLiveViewPresentation(Capture, true, -1.0f);
    TestEqual(TEXT("Capture exposure unchanged"), Capture.AutoExposureBias, Level.AutoExposureBias);
    TestEqual(TEXT("Capture bloom unchanged"), Capture.BloomIntensity, Level.BloomIntensity);
    TestEqual(TEXT("Capture local exposure unchanged"), Capture.LocalExposureShadowContrastScale, Level.LocalExposureShadowContrastScale);

    ApplyLiveViewPresentation(Level, false, -1.0f);
    TestTrue(TEXT("Live exposure reduced by one stop"), FMath::IsNearlyEqual(Level.AutoExposureBias, -0.85f));
    TestEqual(TEXT("Live bloom capped"), Level.BloomIntensity, 0.15f);
    TestEqual(TEXT("Ordinary surfaces excluded from bloom"), Level.BloomThreshold, 1.0f);
    TestEqual(TEXT("Shadows no longer lifted"), Level.LocalExposureShadowContrastScale, 1.0f);
    Level.BloomIntensity = 0.0f;
    ApplyLiveViewPresentation(Level, false, 0.0f);
    TestEqual(TEXT("Authored disabled bloom stays disabled"), Level.BloomIntensity, 0.0f);
    return true;
}
#endif
