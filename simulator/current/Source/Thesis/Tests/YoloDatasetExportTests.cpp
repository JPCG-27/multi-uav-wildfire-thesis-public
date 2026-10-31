#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"
#include "../Vision/YoloDatasetExporter.h"
#include <limits>

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FYoloDatasetDepthMaskTest,
    "Thesis.Dataset.DepthMask",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FYoloDatasetDepthMaskTest::RunTest(const FString& Parameters)
{
    FYoloDatasetFrame Frame;
    Frame.Width = 3;
    Frame.Height = 2;
    Frame.DepthToleranceUU = 0.1f;
    Frame.Rgb.Init(FColor::Black, 6);
    Frame.OpaqueSceneDepthUU.Init(100000000.0f, 6);

    FHumanVisualCaptureRecord A;
    A.PersonId = 0;
    A.Pose = TEXT("standing");
    FHumanVisualCaptureRecord B;
    B.PersonId = 1;
    B.Pose = TEXT("crouching");
    Frame.Persons = { A, B };

    TArray<float> DepthA;
    DepthA.Init(100000000.0f, 6);
    DepthA[0] = 1000.0f;
    DepthA[1] = 1000.0f;
    DepthA[2] = 1000.0f;
    DepthA[3] = 1000.0f;
    TArray<float> DepthB;
    DepthB.Init(100000000.0f, 6);
    DepthB[1] = 500.0f;
    DepthB[2] = 2000.0f;
    DepthB[4] = 1000.0f;
    Frame.PersonDepthUU = { DepthA, DepthB };
    Frame.OpaqueSceneDepthUU[3] = 100.0f; // Opaque object in front of person A.

    TArray<uint32> Mask;
    TArray<FYoloMaskPersonStats> Stats;
    FString Error;
    TestTrue(TEXT("depth inputs build a visible instance mask"),
        FYoloDatasetExporter::BuildVisibleMask(Frame, Mask, Stats, Error));
    TestTrue(TEXT("visible support ignores hidden person pixels"), Error.IsEmpty());
    TestEqual(TEXT("one mask value per pixel"), Mask.Num(), 6);
    TestEqual(TEXT("person A owns nearest visible overlap"), Mask[0], 1u);
    TestEqual(TEXT("person B wins the nearer overlap"), Mask[1], 2u);
    TestEqual(TEXT("person A remains nearer than person B"), Mask[2], 1u);
    TestEqual(TEXT("opaque scene depth hides person A"), Mask[3], 0u);
    TestEqual(TEXT("person B is visible in the final pixel"), Mask[4], 2u);
    TestEqual(TEXT("background stays zero"), Mask[5], 0u);
    TestEqual(TEXT("person A amodal pixel count"), Stats[0].AmodalPixels, 4ll);
    TestEqual(TEXT("person A visible pixel count"), Stats[0].VisiblePixels, 2ll);
    TestEqual(TEXT("person A amodal max X is exclusive"), Stats[0].AmodalBBoxXYXY.Max.X, 3);
    TestEqual(TEXT("person A visible max Y is exclusive"), Stats[0].VisibleBBoxXYXY.Max.Y, 1);
    TestEqual(TEXT("person B visible count includes only assigned pixels"), Stats[1].VisiblePixels, 2ll);

    TestTrue(TEXT("same-size reused statistics are reset"), FYoloDatasetExporter::BuildVisibleMask(Frame, Mask, Stats, Error));
    TestEqual(TEXT("same-size reuse does not accumulate pixels"), Stats[0].VisiblePixels, 2ll);
    Frame.OpaqueSceneDepthUU[0] = std::numeric_limits<float>::quiet_NaN();
    TestFalse(TEXT("nonfinite scene depth fails closed"), FYoloDatasetExporter::BuildVisibleMask(Frame, Mask, Stats, Error));
    Frame.OpaqueSceneDepthUU[0] = 100000000.0f;
    Frame.PersonDepthUU[0][0] = -1.0f;
    TestFalse(TEXT("invalid person depth fails closed"), FYoloDatasetExporter::BuildVisibleMask(Frame, Mask, Stats, Error));
    Frame.PersonDepthUU[0][0] = 1000.0f;

    // Reuse the output arrays to catch stale per-person stats across frames.
    Frame.Persons.Reset();
    Frame.PersonDepthUU.Reset();
    TestTrue(TEXT("empty-person frame is valid"), FYoloDatasetExporter::BuildVisibleMask(Frame, Mask, Stats, Error));
    TestTrue(TEXT("no-person mask contains only background"), Mask.Contains(0u) && Mask.Num() == 6);
    TestTrue(TEXT("no-person frame has no retained statistics"), Stats.IsEmpty());

    return true;
}

#endif
