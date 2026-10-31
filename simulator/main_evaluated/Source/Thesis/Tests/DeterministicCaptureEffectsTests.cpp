#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "../World/DeterministicCaptureEffects.h"
#include "Engine/World.h"
#include "Materials/MaterialInterface.h"
#include "ProceduralMeshComponent.h"
#include "../Vision/DroneCaptureComponent.h"
#include "Engine/Engine.h"
#include "Engine/StaticMesh.h"
#include "Components/StaticMeshComponent.h"
#include "Components/DirectionalLightComponent.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDeterministicCaptureEffectsTest,
    "Thesis.Fire.DeterministicCaptureEffects",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FDeterministicCaptureEffectsTest::RunTest(const FString& Parameters)
{
    const UWorld::InitializationValues Init = UWorld::InitializationValues()
        .AllowAudioPlayback(false).CreatePhysicsScene(false).CreateNavigation(false)
        .CreateAISystem(false).ShouldSimulatePhysics(false);
    UWorld* World = UWorld::CreateWorld(EWorldType::Game, false,
        FName(TEXT("DeterministicCaptureEffectsFixture")), nullptr, true,
        ERHIFeatureLevel::Num, &Init);
    if (!TestNotNull(TEXT("Fixture world exists"), World)) return false;
    GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);

    AActor* Owner = World->SpawnActor<AActor>();
    USceneComponent* Root = NewObject<USceneComponent>(Owner, TEXT("EffectsRoot"));
    Owner->SetRootComponent(Root);
    Root->RegisterComponent();
    UDeterministicCaptureEffectsComponent* Effects =
        NewObject<UDeterministicCaptureEffectsComponent>(Owner, TEXT("CaptureEffects"));
    Owner->AddInstanceComponent(Effects);
    Effects->RegisterComponent();

    const FVector FlameAnchor(100, 200, 30);
    const FVector SmokeAnchor(140, 240, 30);
    const FVector2D Wind(1.0, 0.25);
    Effects->Initialize(FlameAnchor, SmokeAnchor, 17, 90210, 0.8f, 10, 40,
        Wind, 75.0f, 1.0f, 1.0f, 1.0f, 2, true, true);

    TArray<UPrimitiveComponent*> Primitives;
    Effects->GetEffectPrimitives(Primitives);
    TestEqual(TEXT("One flame and one shared smoke plume are created"), Primitives.Num(), 2);
    if (Primitives.Num() != 2)
    {
        World->DestroyWorld(false);
        return false;
    }

    TestTrue(TEXT("Flame mesh is visible when enabled"), Primitives[0]->IsVisible());
    TestTrue(TEXT("Smoke mesh is visible when enabled"), Primitives[1]->IsVisible());
    TestTrue(TEXT("Smoke uses a translucent material"),
        Primitives[1]->GetMaterial(0) && Primitives[1]->GetMaterial(0)->GetBlendMode() == BLEND_Translucent);
    if (UProceduralMeshComponent* Smoke = Cast<UProceduralMeshComponent>(Primitives[1]))
    {
        const FProcMeshSection* Section = Smoke->GetProcMeshSection(0);
        bool bHasFeatheredAlpha = false;
        if (Section)
            for (const FProcMeshVertex& Vertex : Section->ProcVertexBuffer)
                bHasFeatheredAlpha |= Vertex.Color.A > 0 && Vertex.Color.A < 255;
        TestTrue(TEXT("Smoke plume vertices carry partial opacity for soft edges"), bHasFeatheredAlpha);
    }

    const auto Update = [&](int32 Tick)
    {
        Effects->UpdateAtTick(FlameAnchor, SmokeAnchor, Tick, Wind, 75.0f,
            1.0f, 1.0f, 1.0f, 2, true, true);
    };
    Update(44);
    const FTransform AFlame = Primitives[0]->GetComponentTransform();
    const FTransform ASmoke = Primitives[1]->GetComponentTransform();
    Update(45);
    const FTransform BFlame = Primitives[0]->GetComponentTransform();
    const FTransform BSmoke = Primitives[1]->GetComponentTransform();
    Update(44);
    TestTrue(TEXT("Returning to tick A restores the exact flame transform"),
        Primitives[0]->GetComponentTransform().Equals(AFlame, 0.0f));
    TestTrue(TEXT("Returning to tick A restores the exact smoke transform"),
        Primitives[1]->GetComponentTransform().Equals(ASmoke, 0.0f));
    TestFalse(TEXT("Tick B changes the flame presentation"), BFlame.Equals(AFlame, 0.0f));
    TestFalse(TEXT("Tick B advances the plume"), BSmoke.Equals(ASmoke, 0.0f));

    Effects->UpdateAtTick(FlameAnchor, SmokeAnchor, 45, Wind, 75.0f,
        1.0f, 1.0f, 1.0f, 2, false, false);
    TestFalse(TEXT("Flame can be disabled independently"), Primitives[0]->IsVisible());
    TestFalse(TEXT("Smoke can be disabled independently"), Primitives[1]->IsVisible());

    Effects->UpdateAtTick(FlameAnchor, SmokeAnchor, 40, Wind, 75.0f,
        1.0f, 1.0f, 1.0f, 2, true, true);
    AActor* CameraActor = World->SpawnActor<AActor>();
    // Dark smoke must be tested against an illuminated background; on a black
    // empty world its absorption would be invisible even with correct opacity.
    AActor* BackdropActor = World->SpawnActor<AActor>();
    UStaticMeshComponent* Backdrop = NewObject<UStaticMeshComponent>(BackdropActor);
    BackdropActor->SetRootComponent(Backdrop);
    Backdrop->SetStaticMesh(LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube")));
    Backdrop->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Backdrop->RegisterComponent();
    Backdrop->SetWorldLocation(FVector(1500, 0, 1300));
    Backdrop->SetWorldScale3D(FVector(1, 50, 50));
    AActor* LightActor = World->SpawnActor<AActor>();
    UDirectionalLightComponent* Light = NewObject<UDirectionalLightComponent>(LightActor);
    LightActor->SetRootComponent(Light);
    Light->SetIntensity(100000.0f);
    Light->RegisterComponent();
    Light->SetWorldRotation(FRotator(-10, 0, 0));
    UDroneCaptureComponent* Camera = NewObject<UDroneCaptureComponent>(CameraActor);
    CameraActor->SetRootComponent(Camera);
    Camera->RegisterComponent();
    Camera->InitCapture(96, 96, 70.0f, 0.0f);
    // Match the production v3 profile, including its disabled temporal shadows.
    Camera->SetFastUnshadowedRendering(true);
    Camera->SetDeterministicFirstCaptureWarmup(true);
    Camera->SetExposure(false, 0.0f, 0.0f, 12.0f);
    Camera->SetWorldLocation(FVector(-2200.0f, 0.0f, 350.0f));
    Camera->SetWorldRotation(FRotator::ZeroRotator);
    auto Capture = [&](int32 Tick, TArray<FColor>& Pixels)
    {
        World->SendAllEndOfFrameUpdates();
        Camera->CaptureTick(Tick, true, 0.1f);
        int32 Width = 0, Height = 0, CapturedTick = -1;
        return Camera->ReadLastCapture(Pixels, Width, Height, CapturedTick)
            && Width == 96 && Height == 96 && CapturedTick == Tick;
    };
    TArray<FColor> PixelsA, PixelsB, PixelsAAgain;
    TestTrue(TEXT("First live scene capture succeeds"), Capture(40, PixelsA));
    int32 VisiblePixels = 0;
    for (const FColor& Pixel : PixelsA)
        VisiblePixels += (Pixel.R > 12 || Pixel.G > 12 || Pixel.B > 12) ? 1 : 0;
    TestTrue(TEXT("Capture contains rendered fire and smoke pixels"), VisiblePixels > 20);
    Effects->UpdateAtTick(FlameAnchor, SmokeAnchor, 45, Wind, 75.0f,
        1.0f, 1.0f, 1.0f, 2, true, true);
    TestTrue(TEXT("Second live scene capture succeeds"), Capture(45, PixelsB));
    Effects->UpdateAtTick(FlameAnchor, SmokeAnchor, 40, Wind, 75.0f,
        1.0f, 1.0f, 1.0f, 2, true, true);
    TestTrue(TEXT("A-B-A replay capture succeeds"), Capture(40, PixelsAAgain));
    TestTrue(TEXT("A-B-A captures reproduce identical pixels"), PixelsAAgain == PixelsA);

    TArray<FColor> SmokeOnly, FlameOnly;
    Effects->UpdateAtTick(FlameAnchor, SmokeAnchor, 40, Wind, 75.0f,
        1.0f, 1.0f, 1.0f, 2, false, true);
    TestTrue(TEXT("Smoke-only capture succeeds"), Capture(40, SmokeOnly));
    Effects->UpdateAtTick(FlameAnchor, SmokeAnchor, 40, Wind, 75.0f,
        1.0f, 1.0f, 1.0f, 2, true, false);
    TestTrue(TEXT("Flame-only capture succeeds"), Capture(40, FlameOnly));
    TestTrue(TEXT("Flame contributes to captured pixels"), SmokeOnly != PixelsA);
    TestTrue(TEXT("Smoke contributes to captured pixels"), FlameOnly != PixelsA);

    GEngine->DestroyWorldContext(World);
    World->DestroyWorld(false);
    return true;
}
#endif
