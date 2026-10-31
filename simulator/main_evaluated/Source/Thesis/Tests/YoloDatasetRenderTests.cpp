#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "../Vision/DroneCaptureComponent.h"
#include "../Vision/YoloDatasetExporter.h"
#include "Engine/World.h"
#include "Engine/Engine.h"
#include "Engine/StaticMesh.h"
#include "Components/StaticMeshComponent.h"
#include "RenderingThread.h"
#if WITH_EDITOR
#include "AssetCompilingManager.h"
#endif

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FYoloDatasetDepthRenderTest, "Thesis.Dataset.DepthRendering",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FYoloDatasetDepthRenderTest::RunTest(const FString& Parameters)
{
    const UWorld::InitializationValues Initialization = UWorld::InitializationValues().AllowAudioPlayback(false)
        .CreatePhysicsScene(false).CreateNavigation(false).CreateAISystem(false).ShouldSimulatePhysics(false);
    UWorld* World = UWorld::CreateWorld(EWorldType::Game, false, FName(TEXT("DatasetDepthFixture")),
        nullptr, true, ERHIFeatureLevel::Num, &Initialization);
    GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
    UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
    if (!TestNotNull(TEXT("Fixture cube"), Cube))
    {
        GEngine->DestroyWorldContext(World);
        World->DestroyWorld(false);
        return false;
    }
    auto AddCube = [&](const FVector& Position, const FVector& Scale)
    {
        AActor* Actor = World->SpawnActor<AActor>();
        UStaticMeshComponent* Mesh = NewObject<UStaticMeshComponent>(Actor);
        Actor->SetRootComponent(Mesh);
        Mesh->SetStaticMesh(Cube);
        Mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        Mesh->RegisterComponent();
        Mesh->SetWorldLocation(Position);
        Mesh->SetWorldScale3D(Scale);
        return Mesh;
    };
    UStaticMeshComponent* Person = AddCube(FVector::ZeroVector, FVector(1));
    UStaticMeshComponent* BorderPerson = AddCube(FVector(0, 340, 0), FVector(1));
    UStaticMeshComponent* Occluder = AddCube(FVector(-200, 0, 0), FVector(1, 3, 3));
    Occluder->SetVisibility(false);
    AActor* CameraActor = World->SpawnActor<AActor>();
    UDroneCaptureComponent* Camera = NewObject<UDroneCaptureComponent>(CameraActor);
    CameraActor->SetRootComponent(Camera);
    Camera->RegisterComponent();
    Camera->InitCapture(128, 128, 60, 0);
    Camera->SetWorldLocation(FVector(-600, 0, 0));
    Camera->SetWorldRotation(FRotator::ZeroRotator);
#if WITH_EDITOR
    FAssetCompilingManager::Get().FinishAllCompilation();
#endif
    World->SendAllEndOfFrameUpdates();
    FlushRenderingCommands();
    Camera->CaptureTick(1, true, 1.0f);

    FYoloDatasetFrame Frame;
    Frame.Width = Frame.Height = 128;
    Frame.Rgb.Init(FColor::Black, 128 * 128);
    FHumanVisualCaptureRecord P;
    P.PersonId = 0;
    Frame.Persons.Add(P);
    P.PersonId = 1;
    Frame.Persons.Add(P);
    Frame.PersonDepthUU.SetNum(2);
    int32 W = 0, H = 0;
    TestTrue(TEXT("Full-scene depth readback"), Camera->ReadDepthPass(nullptr, Frame.OpaqueSceneDepthUU, W, H));
    TestTrue(TEXT("Isolated first subject"), Camera->ReadDepthPass(Person, Frame.PersonDepthUU[0], W, H));
    TestTrue(TEXT("Isolated border subject"), Camera->ReadDepthPass(BorderPerson, Frame.PersonDepthUU[1], W, H));
    TestEqual(TEXT("Width"), W, 128);
    if (Frame.OpaqueSceneDepthUU.Num() == 128 * 128)
        TestTrue(TEXT("Depth is linear view distance in UU"), FMath::IsNearlyEqual(Frame.OpaqueSceneDepthUU[64 * 128 + 64], 550.0f, 0.1f));
    TArray<uint32> Mask;
    TArray<FYoloMaskPersonStats> Stats;
    FString Error;
    TestTrue(TEXT("Unoccluded mask construction"), FYoloDatasetExporter::BuildVisibleMask(Frame, Mask, Stats, Error));
    if (Stats.Num() == 2)
    {
        TestTrue(TEXT("First subject has pixels"), Stats[0].VisiblePixels > 0);
        TestEqual(TEXT("Unoccluded subject support"), Stats[0].VisiblePixels, Stats[0].AmodalPixels);
        TestTrue(TEXT("Border subject has pixels"), Stats[1].VisiblePixels > 0);
        TestTrue(TEXT("Border box is clipped at image edge"), Stats[1].VisibleBBoxXYXY.Max.X == 128 || Stats[1].VisibleBBoxXYXY.Min.X == 0);
    }
    TArray<float> Repeated;
    Camera->ReadDepthPass(Person, Repeated, W, H);
    TestTrue(TEXT("Repeated isolated depth is exact"), Repeated == Frame.PersonDepthUU[0]);
    TArray<FColor> Before, After;
    int32 Tick = 0;
    Camera->ReadLastCapture(Before, W, H, Tick);
    Camera->CaptureTick(1, true, 1.0f);
    Camera->ReadLastCapture(After, W, H, Tick);
    TestTrue(TEXT("Auxiliary passes preserve repeated production RGB"), !Before.IsEmpty() && Before == After);

    Occluder->SetVisibility(true);
    World->SendAllEndOfFrameUpdates();
    FlushRenderingCommands();
    Camera->ReadDepthPass(nullptr, Frame.OpaqueSceneDepthUU, W, H);
    TestTrue(TEXT("Occluded mask construction"), FYoloDatasetExporter::BuildVisibleMask(Frame, Mask, Stats, Error));
    if (Stats.Num() == 2)
    {
        TestEqual(TEXT("Opaque occluder removes all visible pixels"), Stats[0].VisiblePixels, int64(0));
        TestTrue(TEXT("Hidden subject retains amodal support"), Stats[0].AmodalPixels > 0);
    }
    Frame.Persons.Reset();
    Frame.PersonDepthUU.Reset();
    TestTrue(TEXT("No-subject scene accepted"), FYoloDatasetExporter::BuildVisibleMask(Frame, Mask, Stats, Error));
    TestTrue(TEXT("No-subject mask is empty background"), Mask.Num() == 128 * 128 && !Mask.ContainsByPredicate([](uint32 ID) { return ID != 0; }));
    Camera->DestroyComponent();
    GEngine->DestroyWorldContext(World);
    World->DestroyWorld(false);
    FlushRenderingCommands();
    return true;
}
#endif
