#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "../Vision/DroneCaptureComponent.h"
#include "Engine/World.h"
#include "Engine/Engine.h"
#include "Engine/StaticMesh.h"
#include "Components/StaticMeshComponent.h"
#include "Components/DirectionalLightComponent.h"
#include "Engine/TextureRenderTarget2D.h"
#include "TextureResource.h"
#include "RenderingThread.h"
#if WITH_EDITOR
#include "AssetCompilingManager.h"
#endif

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDroneCameraQueuedReadbackTest, "Thesis.Camera.QueuedReadbackDeterminism",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FDroneCameraQueuedReadbackTest::RunTest(const FString& Parameters)
{
    const UWorld::InitializationValues Initialization = UWorld::InitializationValues().AllowAudioPlayback(false)
        .CreatePhysicsScene(false).CreateNavigation(false).CreateAISystem(false).ShouldSimulatePhysics(false);
    UWorld* World = UWorld::CreateWorld(EWorldType::Game, false, FName(TEXT("CameraReadbackFixture")),
        nullptr, true, ERHIFeatureLevel::Num, &Initialization);
    if (!TestNotNull(TEXT("Fixture world"), World)) return false;
    GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);

    UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
    if (!TestNotNull(TEXT("Fixture cube"), Cube))
    {
        GEngine->DestroyWorldContext(World);
        World->DestroyWorld(false);
        FlushRenderingCommands();
        return false;
    }

    AActor* SubjectActor = World->SpawnActor<AActor>();
    UStaticMeshComponent* Subject = NewObject<UStaticMeshComponent>(SubjectActor);
    SubjectActor->SetRootComponent(Subject);
    Subject->SetStaticMesh(Cube);
    Subject->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Subject->RegisterComponent();
    Subject->SetWorldLocation(FVector::ZeroVector);
    Subject->SetWorldScale3D(FVector(1.0f));

    // A lit subject makes the moved-geometry assertion meaningful rather than
    // comparing two black frames from an otherwise unlit fixture world.
    AActor* LightActor = World->SpawnActor<AActor>();
    UDirectionalLightComponent* Light = NewObject<UDirectionalLightComponent>(LightActor);
    LightActor->SetRootComponent(Light);
    Light->SetMobility(EComponentMobility::Movable);
    Light->SetIntensity(100000.0f);
    Light->RegisterComponent();
    Light->SetWorldRotation(FRotator(-30.0f, 0.0f, 0.0f));

    AActor* CameraActor = World->SpawnActor<AActor>();
    UDroneCaptureComponent* Camera = NewObject<UDroneCaptureComponent>(CameraActor);
    CameraActor->SetRootComponent(Camera);
    Camera->RegisterComponent();
    Camera->InitCapture(131, 67, 60.0f, 0.0f);
    Camera->SetExposure(false, 0.0f, 0.0f, 12.0f);
    Camera->SetWorldLocation(FVector(-600.0f, 0.0f, 0.0f));
    Camera->SetWorldRotation(FRotator::ZeroRotator);
#if WITH_EDITOR
    FAssetCompilingManager::Get().FinishAllCompilation();
#endif
    World->SendAllEndOfFrameUpdates();
    FlushRenderingCommands();

    auto ReadDirectPixels = [&](TArray<FColor>& OutPixels)
    {
        FTextureRenderTargetResource* Resource = Camera->TextureTarget
            ? Camera->TextureTarget->GameThread_GetRenderTargetResource()
            : nullptr;
        return Resource && Resource->ReadPixels(OutPixels);
    };
    auto CompareQueuedWithDirect = [&](const TCHAR* Label, int32 ExpectedWidth, int32 ExpectedHeight, int32 ExpectedTick)
    {
        TArray<FColor> Direct;
        const bool bDirectRead = ReadDirectPixels(Direct);
        TestTrue(FString::Printf(TEXT("%s direct ReadPixels succeeds"), Label), bDirectRead);
        TArray<FColor> Queued;
        int32 Width = 0;
        int32 Height = 0;
        int32 Tick = -1;
        const bool bQueuedRead = Camera->ReadLastCapture(Queued, Width, Height, Tick);
        TestTrue(FString::Printf(TEXT("%s queued read succeeds"), Label), bQueuedRead);
        TestEqual(FString::Printf(TEXT("%s width"), Label), Width, ExpectedWidth);
        TestEqual(FString::Printf(TEXT("%s height"), Label), Height, ExpectedHeight);
        TestEqual(FString::Printf(TEXT("%s tick"), Label), Tick, ExpectedTick);
        TestEqual(FString::Printf(TEXT("%s pixel count"), Label), Queued.Num(), Direct.Num());
        if (Queued.Num() == Direct.Num())
        {
            for (int32 Pixel = 0; Pixel < Direct.Num(); ++Pixel)
            {
                if (Queued[Pixel] != Direct[Pixel])
                {
                    AddError(FString::Printf(TEXT("%s pixel %d differs: queued=(%u,%u,%u,%u) direct=(%u,%u,%u,%u)"),
                        Label, Pixel, Queued[Pixel].R, Queued[Pixel].G, Queued[Pixel].B, Queued[Pixel].A,
                        Direct[Pixel].R, Direct[Pixel].G, Direct[Pixel].B, Direct[Pixel].A));
                    break;
                }
            }
        }
        return Queued;
    };

    Camera->CaptureTick(1, true, 1.0f);
    TArray<FColor> First = CompareQueuedWithDirect(TEXT("Odd-sized capture"), 131, 67, 1);
    TestTrue(TEXT("Odd-sized capture contains pixels"), First.Num() == 131 * 67);
    const TArray<FColor> FirstImage = First;

    // The caller owns its returned array. Editing it must not alter the
    // component's cached capture used by subsequent consumers.
    if (!First.IsEmpty()) First[0] = FColor::Magenta;
    TArray<FColor> CachedAgain;
    int32 Width = 0, Height = 0, Tick = -1;
    TestTrue(TEXT("Repeated cached read succeeds"), Camera->ReadLastCapture(CachedAgain, Width, Height, Tick));
    TArray<FColor> DirectAgain;
    TestTrue(TEXT("Direct comparison read succeeds"), ReadDirectPixels(DirectAgain));
    TestTrue(TEXT("Editing returned pixels does not mutate cached image"), CachedAgain == DirectAgain);
    const TArray<FColor>* BorrowedPixels = nullptr;
    TestTrue(TEXT("Borrowed read succeeds"), Camera->ReadLastCaptureView(BorrowedPixels, Width, Height, Tick));
    TestTrue(TEXT("Borrowed pixels match the direct GPU read"), BorrowedPixels && *BorrowedPixels == DirectAgain);
    const FColor* BorrowedStorage = BorrowedPixels ? BorrowedPixels->GetData() : nullptr;
    TestTrue(TEXT("Repeated borrowed read succeeds"), Camera->ReadLastCaptureView(BorrowedPixels, Width, Height, Tick));
    TestTrue(TEXT("Repeated borrowed reads share immutable storage"), BorrowedPixels && BorrowedPixels->GetData() == BorrowedStorage);

    TArray<float> Depth;
    TestTrue(TEXT("Depth pass succeeds between RGB reads"), Camera->ReadDepthPass(nullptr, Depth, Width, Height));
    TestTrue(TEXT("Depth dimensions match RGB"), Width == 131 && Height == 67 && Depth.Num() == 131 * 67);
    TArray<FColor> AfterDepth;
    TestTrue(TEXT("RGB read after depth succeeds"), Camera->ReadLastCapture(AfterDepth, Width, Height, Tick));
    TestTrue(TEXT("Depth capture preserves cached RGB"), AfterDepth == DirectAgain);

    Subject->SetWorldLocation(FVector(0.0f, 180.0f, 0.0f));
    World->SendAllEndOfFrameUpdates();
    Camera->CaptureTick(2, true, 1.0f);
    const TArray<FColor> MovedImage = CompareQueuedWithDirect(TEXT("Moved geometry"), 131, 67, 2);
    TestTrue(TEXT("Moved geometry produces a new image"), MovedImage != FirstImage);

    // Leave the old-size copy pending when the render target is replaced.
    Camera->CaptureTick(3, true, 1.0f);
    Camera->InitCapture(17, 19, 60.0f, 0.0f);
    Camera->SetExposure(false, 0.0f, 0.0f, 12.0f);
    Camera->SetWorldLocation(FVector(-600.0f, 0.0f, 0.0f));
    Camera->SetWorldRotation(FRotator::ZeroRotator);
    World->SendAllEndOfFrameUpdates();
    Camera->CaptureTick(4, true, 1.0f);
    CompareQueuedWithDirect(TEXT("Resized capture"), 17, 19, 4);

    Camera->CaptureTick(4, false, 1.0f);
    TArray<FColor> ReplayPixels;
    TestFalse(TEXT("Same-tick replay cannot expose prior live RGB"), Camera->ReadLastCapture(ReplayPixels, Width, Height, Tick));
    TestFalse(TEXT("Replay cannot borrow prior live RGB"), Camera->ReadLastCaptureView(BorrowedPixels, Width, Height, Tick));
    TestNull(TEXT("Failed borrowed read clears its output"), BorrowedPixels);

    // Destroy with a submitted copy that no consumer has requested yet.
    Camera->CaptureTick(5, true, 1.0f);
    Camera->DestroyComponent();
    Camera->ConditionalBeginDestroy();
    SubjectActor->Destroy();
    GEngine->DestroyWorldContext(World);
    World->DestroyWorld(false);
    FlushRenderingCommands();
    return true;
}
#endif
