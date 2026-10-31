#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"
#include "../World/VisionProjection.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FVisionProjectionFrustumMembershipTest,
    "Thesis.Vision.ProjectionFrustumMembership",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FVisionProjectionFrustumMembershipTest::RunTest(const FString& Parameters)
{
    const FTransform Camera(FRotator(-57.0f, 31.0f, 9.0f), FVector(120.0f, -340.0f, 1800.0f));
    const FVector Origin = Camera.GetLocation();
    const FVector Forward = Camera.GetUnitAxis(EAxis::X);
    const FVector Right = Camera.GetUnitAxis(EAxis::Y);
    const FVector Up = Camera.GetUnitAxis(EAxis::Z);
    constexpr float FOV = 58.0f;
    constexpr float Distance = 5000.0f;
    const float HalfExtent = Distance * FMath::Tan(FMath::DegreesToRadians(FOV * 0.5f));

    float U = 0.0f, V = 0.0f;
    TestTrue(TEXT("A point at the optical centre is inside the tilted frustum"),
        ThesisVisionProjection::ProjectWorldPointToImage(
            Origin, Forward, Right, Up, FOV, Origin + Forward * Distance, U, V));
    TestTrue(TEXT("Optical centre projects to image centre"),
        FMath::IsNearlyEqual(U, 0.5f) && FMath::IsNearlyEqual(V, 0.5f));

    // This is a corner of the projected ground-plane AABB, outside the actual
    // camera quadrilateral. The previous AABB-only footprint accepted it.
    const FVector OutsideAabbCorner = Origin + Forward * Distance
        + Right * (HalfExtent * 1.25f) + Up * (HalfExtent * 1.25f);
    TestFalse(TEXT("A projected AABB corner outside the tilted frustum is rejected"),
        ThesisVisionProjection::ProjectWorldPointToImage(
            Origin, Forward, Right, Up, FOV, OutsideAabbCorner, U, V));

    const FVector InsideEdge = Origin + Forward * Distance
        + Right * (HalfExtent * 0.8f) - Up * (HalfExtent * 0.7f);
    TestTrue(TEXT("A terrain point within the tilted frustum is accepted"),
        ThesisVisionProjection::ProjectWorldPointToImage(
            Origin, Forward, Right, Up, FOV, InsideEdge, U, V));

    TestFalse(TEXT("Points behind the camera are rejected"),
        ThesisVisionProjection::ProjectWorldPointToImage(
            Origin, Forward, Right, Up, FOV, Origin - Forward * Distance, U, V));
    return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
