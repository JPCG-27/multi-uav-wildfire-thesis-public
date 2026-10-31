#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "../World/ProceduralTerrain.h"
#include "../Vision/DroneCaptureComponent.h"
#include "Engine/World.h"
#include "Engine/Engine.h"
#include "RenderingThread.h"
#include "ProceduralMeshComponent.h"
#if WITH_EDITOR
#include "AssetCompilingManager.h"
#endif

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FEnvironmentEdgeRenderTest, "Thesis.Environment.EdgeDepthCoverage",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FEnvironmentEdgeRenderTest::RunTest(const FString& Parameters)
{
    const UWorld::InitializationValues Initialization = UWorld::InitializationValues().AllowAudioPlayback(false)
        .CreatePhysicsScene(false).CreateNavigation(false).CreateAISystem(false).ShouldSimulatePhysics(false);
    UWorld* World = UWorld::CreateWorld(EWorldType::Game, false, FName(TEXT("EnvironmentEdgeFixture")),
        nullptr, true, ERHIFeatureLevel::Num, &Initialization);
    GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
    AProceduralTerrain* Terrain = World->SpawnActor<AProceduralTerrain>();
    Terrain->RenderTileQuads = 0; // Inspect the complete canonical mesh below.
    Terrain->PresentationMarginMeters = 0;
    Terrain->VerticesPerCell = 3; // Non-binary spacing catches rounding regressions.
    Terrain->Profile = ETerrainProfile::Foothills;
    Terrain->PresentationMarginCells = 0;
    Terrain->BuildTerrain(10, 10, FVector::ZeroVector, 1000, 9340);
    const TArray<FProcMeshVertex> OriginalVertices = Terrain->Mesh->GetProcMeshSection(0)->ProcVertexBuffer;
    Terrain->PresentationMarginCells = 12;
    Terrain->BuildTerrain(10, 10, FVector::ZeroVector, 1000, 9340);
    const TArray<FProcMeshVertex>& ExtendedVertices = Terrain->Mesh->GetProcMeshSection(0)->ProcVertexBuffer;
    bool bInteriorExact = true;
    for (int32 Y = 0; Y <= 30; ++Y)
    {
        for (int32 X = 0; X <= 30; ++X)
        {
            const FProcMeshVertex& A = OriginalVertices[Y * 31 + X];
            const FProcMeshVertex& B = ExtendedVertices[(Y + 36) * 103 + X + 36];
            bInteriorExact &= A.Position.Equals(B.Position, 0.0) && A.Normal.Equals(B.Normal, 0.0);
        }
    }
    TestTrue(TEXT("Interior relief vertices and normals unchanged by border"), bInteriorExact);
    const TArray<FProcMeshVertex> NearVertices = ExtendedVertices;
    Terrain->PresentationMarginMeters = 1000;
    Terrain->BuildTerrain(10, 10, FVector::ZeroVector, 1000, 9340);
    const TArray<FProcMeshVertex>& DistantVertices = Terrain->Mesh->GetProcMeshSection(0)->ProcVertexBuffer;
    bool bNearMeshExact = true;
    for (int32 Y = 0; Y < 103; ++Y)
        for (int32 X = 0; X < 103; ++X)
        {
            const FProcMeshVertex& A = NearVertices[Y * 103 + X];
            const FProcMeshVertex& B = DistantVertices[(Y + 32) * 167 + X + 32];
            bNearMeshExact &= A.Position.Equals(B.Position, 0.0) && A.Normal.Equals(B.Normal, 0.0);
        }
    TestTrue(TEXT("Kilometre apron preserves the entire detailed mesh exactly"), bNearMeshExact);
    TestTrue(TEXT("Distant apron starts one kilometre outside grid"),
        FMath::IsNearlyEqual(DistantVertices[0].Position.X, -100000.0, 0.1));
    TestTrue(TEXT("Distant apron ends one kilometre beyond grid"),
        FMath::IsNearlyEqual(DistantVertices.Last().Position.X, 110000.0, 0.1));
    Terrain->PresentationMarginMeters = 0;
    Terrain->VerticesPerCell = 2;
    Terrain->RenderTileQuads = 64; // Exercise production culling tiles in the depth checks.
    Terrain->Profile = ETerrainProfile::Flat;
    Terrain->PresentationMarginCells = 0;
    Terrain->BuildTerrain(10, 10, FVector::ZeroVector, 1000, 9340);
    AActor* CameraActor = World->SpawnActor<AActor>();
    UDroneCaptureComponent* Camera = NewObject<UDroneCaptureComponent>(CameraActor);
    CameraActor->SetRootComponent(Camera);
    Camera->RegisterComponent();
    Camera->InitCapture(128, 128, 60, -90);
    Camera->SetWorldRotation(FRotator(-90, 0, 0));
    Camera->SetWorldLocation(FVector(0, 5000, 8000));
#if WITH_EDITOR
    FAssetCompilingManager::Get().FinishAllCompilation();
#endif
    auto Read = [&](TArray<float>& Depth)
    {
        World->SendAllEndOfFrameUpdates();
        FlushRenderingCommands();
        Camera->CaptureTick(1, true, 1.0f);
        int32 W = 0, H = 0;
        TestTrue(TEXT("Scene depth readback"), Camera->ReadDepthPass(nullptr, Depth, W, H));
        TestEqual(TEXT("Depth width"), W, 128);
        TestEqual(TEXT("Depth height"), H, 128);
    };
    const auto IsGround = [](float D) { return FMath::IsFinite(D) && FMath::IsNearlyEqual(D, 8000.0f, 2.0f); };
    TArray<float> Before;
    Read(Before);
    TestTrue(TEXT("Control reproduces missing ground outside grid"), Before.ContainsByPredicate([&](float D) { return !IsGround(D); }));
    Terrain->PresentationMarginCells = 12;
    Terrain->BuildTerrain(10, 10, FVector::ZeroVector, 1000, 9340);
#if WITH_EDITOR
    FAssetCompilingManager::Get().FinishAllCompilation();
#endif
    const FVector2D Positions[] = { {0, 5000}, {10000, 5000}, {5000, 0}, {5000, 10000}, {0, 0} };
    for (const FVector2D& Position : Positions)
    {
        Camera->SetWorldLocation(FVector(Position.X, Position.Y, 8000));
        TArray<float> Depth, Repeated;
        Read(Depth);
        TestTrue(TEXT("Extended terrain covers the full edge/corner capture"), Depth.Num() == 128 * 128 &&
            !Depth.ContainsByPredicate([&](float D) { return !IsGround(D); }));
        Read(Repeated);
        TestTrue(TEXT("Repeated edge depth exact"), Depth == Repeated);
    }
    // A near-horizontal edge view sees past the old 120 m apron. The camera's
    // upper ray still points down, so a ground pixel is expected everywhere.
    Camera->SetWorldLocation(FVector(0, 5000, 8000));
    Camera->SetWorldRotation(FRotator(-35, 180, 0));
    const auto IsObliqueGround = [](float D) { return FMath::IsFinite(D) && D > 0.0f && D < 200000.0f; };
    TArray<float> ObliqueBefore;
    Read(ObliqueBefore);
    TestTrue(TEXT("Oblique control sees beyond the old apron"),
        ObliqueBefore.ContainsByPredicate([&](float D) { return !IsObliqueGround(D); }));
    Terrain->PresentationMarginMeters = 1000;
    Terrain->BuildTerrain(10, 10, FVector::ZeroVector, 1000, 9340);
    const float Yaws[] = {180, 0, -90, 90, 225};
    for (int32 Index = 0; Index < UE_ARRAY_COUNT(Positions); ++Index)
    {
        Camera->SetWorldLocation(FVector(Positions[Index].X, Positions[Index].Y, 8000));
        Camera->SetWorldRotation(FRotator(-35, Yaws[Index], 0));
        TArray<float> Depth, Repeated;
        Read(Depth);
        TestTrue(TEXT("Kilometre apron covers oblique edge/corner view"), Depth.Num() == 128 * 128 &&
            !Depth.ContainsByPredicate([&](float D) { return !IsObliqueGround(D); }));
        Read(Repeated);
        TestTrue(TEXT("Repeated oblique depth exact"), Depth == Repeated);
    }
    Camera->DestroyComponent();
    GEngine->DestroyWorldContext(World);
    World->DestroyWorld(false);
    FlushRenderingCommands();
    return true;
}
#endif
