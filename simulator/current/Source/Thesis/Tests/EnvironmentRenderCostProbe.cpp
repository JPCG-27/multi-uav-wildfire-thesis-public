#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "../World/ForestDresser.h"
#include "../World/ProceduralTerrain.h"
#include "../World/Grid.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/HierarchicalInstancedStaticMeshComponent.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/World.h"
#include "Engine/Engine.h"
#include "HAL/PlatformTime.h"
#include "HAL/IConsoleManager.h"
#include "Misc/ScopeExit.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "RenderingThread.h"
#include "RHICommandList.h"
#if WITH_EDITOR
#include "AssetCompilingManager.h"
#include "ShaderCompiler.h"
#endif

// Opt-in profiling fixture; timings are diagnostics, never pass/fail thresholds.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FEnvironmentRenderCostProbe, "Thesis.Environment.RenderCostProbe",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::PerfFilter)

bool FEnvironmentRenderCostProbe::RunTest(const FString& Parameters)
{
    const auto Initialization = UWorld::InitializationValues().AllowAudioPlayback(false)
        .CreatePhysicsScene(false).CreateNavigation(false).CreateAISystem(false).ShouldSimulatePhysics(false);
    UWorld* World = UWorld::CreateWorld(EWorldType::Game, false, TEXT("RenderCostFixture"),
        nullptr, true, ERHIFeatureLevel::Num, &Initialization);
    GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
    auto* Terrain = World->SpawnActor<AProceduralTerrain>();
    Terrain->VerticesPerCell = 3;
    auto* Forest = World->SpawnActor<AForestDresser>();
    AActor* SunActor = World->SpawnActor<AActor>();
    auto* Sun = NewObject<UDirectionalLightComponent>(SunActor);
    SunActor->SetRootComponent(Sun);
    Sun->SetMobility(EComponentMobility::Movable);
    Sun->SetIntensity(75000.0f);
    Sun->SetWorldRotation(FRotator(-70, -45, 0));
    Sun->RegisterComponent();
    AActor* CameraActor = World->SpawnActor<AActor>();
    auto* Camera = NewObject<USceneCaptureComponent2D>(CameraActor);
    CameraActor->SetRootComponent(Camera);
    Camera->bCaptureEveryFrame = false;
    Camera->bCaptureOnMovement = false;
    Camera->bAlwaysPersistRenderingState = true;
    Camera->CaptureSource = SCS_FinalColorLDR;
    Camera->FOVAngle = 90;
    Camera->RegisterComponent();
    UTextureRenderTarget2D* Target = NewObject<UTextureRenderTarget2D>(Camera);
    Target->InitCustomFormat(640, 360, PF_B8G8R8A8, false);
    Target->UpdateResourceImmediate(true);
    Camera->TextureTarget = Target;
    Grid Cells(100, 100);
    auto DrainGPU = []()
    {
        ENQUEUE_RENDER_COMMAND(DrainEnvironmentProbe)([](FRHICommandListImmediate& RHICmdList)
        {
            RHICmdList.SubmitAndBlockUntilGPUIdle();
        });
        FlushRenderingCommands();
    };
    auto Build = [&](int32 ForestTile, int32 TerrainTile, bool bSmallShadows)
    {
        Terrain->RenderTileQuads = TerrainTile;
        Forest->HISMSpatialTileSideCells = ForestTile;
        Forest->bGroundCoverCastsShadow = bSmallShadows;
        Forest->bSurfaceDebrisCastsShadow = bSmallShadows;
        Terrain->BuildTerrain(100, 100, FVector::ZeroVector, 300, 27);
        Forest->BuildForest(&Cells, FVector::ZeroVector, 300, 27, Terrain);
        Camera->SetWorldLocation(FVector(15000, 15000, Terrain->GetHeightAtCell(50, 50) + 8000));
        Camera->SetWorldRotation(FRotator(-45, 0, 0));
#if WITH_EDITOR
        FAssetCompilingManager::Get().FinishAllCompilation();
        if (GShaderCompilingManager) GShaderCompilingManager->FinishAllCompilation();
#endif
        World->SendAllEndOfFrameUpdates();
        DrainGPU();
    };
    struct FVariant { const TCHAR* Name; int32 ForestTile; int32 TerrainTile; bool bSmallShadows; };
    const FVariant Variants[] = {
        { TEXT("baseline"), 0, 0, true },
        { TEXT("shadow_only"), 0, 0, false },
        { TEXT("forest64"), 64, 0, false },
        { TEXT("terrain_only"), 0, 64, false },
        { TEXT("combined64"), 64, 64, false },
        { TEXT("no_coarse"), 0, 0, false },
        { TEXT("nanite_baseline"), 0, 0, true },
        { TEXT("nanite_optimized"), 0, 64, false },
    };
    IConsoleVariable* Coarse = IConsoleManager::Get().FindConsoleVariable(TEXT("r.Shadow.Virtual.NonNanite.IncludeInCoarsePages"));
    const int32 OriginalCoarse = Coarse->GetInt();
    ON_SCOPE_EXIT { Coarse->Set(OriginalCoarse, ECVF_SetByCode); };
    IConsoleVariable* Nanite = IConsoleManager::Get().FindConsoleVariable(TEXT("r.Nanite"));
    const int32 OriginalNanite = Nanite->GetInt();
    ON_SCOPE_EXIT { Nanite->SetWithCurrentPriority(OriginalNanite); };
    FString RequestedVariant = TEXT("baseline");
    FParse::Value(FCommandLine::Get(), TEXT("ThesisRenderProbeVariant="), RequestedVariant);
    for (const FVariant& Variant : Variants)
    {
        // Use a fresh process per variant: rebuilding into the same scene inside
        // one automation frame can retain renderer invalidation/history state.
        if (RequestedVariant != Variant.Name) continue;
        Nanite->SetWithCurrentPriority(RequestedVariant.StartsWith(TEXT("nanite_")) ? 1 : 0);
        Coarse->Set(FCString::Strcmp(Variant.Name, TEXT("no_coarse")) == 0 ? 0 : OriginalCoarse, ECVF_SetByCode);
        Build(Variant.ForestTile, Variant.TerrainTile, Variant.bSmallShadows);
        UE_LOG(LogTemp, Display, TEXT("Environment render probe BEGIN %s"), Variant.Name);
        TArray<double> Samples;
        for (int32 Frame = 0; Frame < 10; ++Frame)
        {
            const double Start = FPlatformTime::Seconds();
            Camera->CaptureScene();
            DrainGPU();
            if (Frame >= 3) Samples.Add((FPlatformTime::Seconds() - Start) * 1000.0);
        }
        Samples.Sort();
        TArray<UHierarchicalInstancedStaticMeshComponent*> Components;
        Forest->GetComponents(Components);
        int32 ShadowInstances = 0;
        for (auto* Component : Components)
            if (Component->CastShadow) ShadowInstances += Component->GetInstanceCount();
        UE_LOG(LogTemp, Display, TEXT("Environment render probe END %s: median %.3f ms, min %.3f ms, max %.3f ms, %d instances, %d shadow instances, %d foliage components"),
            Variant.Name, Samples[Samples.Num() / 2], Samples[0], Samples.Last(),
            Forest->NumInstancesSpawned, ShadowInstances, Components.Num());
    }
    Camera->DestroyComponent();
    GEngine->DestroyWorldContext(World);
    World->DestroyWorld(false);
    DrainGPU();
    return true;
}
#endif
