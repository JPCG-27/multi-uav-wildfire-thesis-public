#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "../World/ForestDresser.h"
#include "../World/ForestDresserBatching.h"
#include "../World/Grid.h"
#include "../Vision/DroneCaptureComponent.h"
#include "Components/HierarchicalInstancedStaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "Engine/Engine.h"
#include "HAL/IConsoleManager.h"
#include "Misc/ScopeExit.h"
#include "RenderingThread.h"
#if WITH_EDITOR
#include "AssetCompilingManager.h"
#endif

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FForestVisibilityRenderTest, "Thesis.Forest.TriangleBudgetRendering",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FForestVisibilityRenderTest::RunTest(const FString& Parameters)
{
    IConsoleVariable* Budget = IConsoleManager::Get().FindConsoleVariable(TEXT("foliage.MaxTrianglesToRender"));
    if (!TestNotNull(TEXT("Engine foliage budget"), Budget)) return false;
    const int32 OriginalBudget = Budget->GetInt();
    ON_SCOPE_EXIT { Budget->Set(OriginalBudget, ECVF_SetByCode); FlushRenderingCommands(); };
    Budget->Set(240, ECVF_SetByCode);
    const auto Initialization = UWorld::InitializationValues().AllowAudioPlayback(false)
        .CreatePhysicsScene(false).CreateNavigation(false).CreateAISystem(false).ShouldSimulatePhysics(false);
    UWorld* World = UWorld::CreateWorld(EWorldType::Game, false, TEXT("ForestVisibilityFixture"),
        nullptr, true, ERHIFeatureLevel::Num, &Initialization);
    GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
    UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
    AForestDresser* Forest = World->SpawnActor<AForestDresser>();
    Forest->bAutoPopulateFromThesisFlora = false;
    Forest->bPreferFullyResidentMeshes = false;
    Forest->TreeMesh = Cube;
    Forest->PresentationMarginCells = 0;
    Forest->HISMSpatialTileSideCells = 4;
    Forest->ForestCoverage = 1.0f;
    Forest->MaxTreesPerCell = 4;
    Forest->MaxBushesPerCell = 0;
    Forest->GroundCoverPerCell = 0;
    Forest->SurfaceDebrisPerCell = 0;
    Forest->TreeScaleMin = Forest->TreeScaleMax = 3.0f;
    Grid Cells(8, 8);
    Forest->BuildForest(&Cells, FVector::ZeroVector, 1000.0f, 9340, nullptr);
    TArray<UHierarchicalInstancedStaticMeshComponent*> Chunks;
    Forest->GetComponents(Chunks);
    TestTrue(TEXT("Low budget splits the real forest builder into multiple chunks"), Chunks.Num() > 1);
    TArray<FTransform> Transforms;
    for (auto* Chunk : Chunks)
    {
        TestTrue(TEXT("Every cube chunk stays below the triangle cap"), Chunk->GetInstanceCount() * 12 < 240);
        for (int32 I = 0; I < Chunk->GetInstanceCount(); ++I)
        {
            FTransform Transform;
            Chunk->GetInstanceTransform(I, Transform, true);
            if (I == 0)
            {
                const int32 TileX = ForestDresserBatching::GetSpatialTileCoordinate(
                    FMath::FloorToInt(Transform.GetLocation().X / 1000.0f), 4);
                const int32 TileY = ForestDresserBatching::GetSpatialTileCoordinate(
                    FMath::FloorToInt(Transform.GetLocation().Y / 1000.0f), 4);
                for (int32 J = 1; J < Chunk->GetInstanceCount(); ++J)
                {
                    FTransform Other;
                    Chunk->GetInstanceTransform(J, Other, true);
                    TestEqual(TEXT("Each HISM component contains one spatial tile"),
                        ForestDresserBatching::MakeSpatialTileKey(
                            ForestDresserBatching::GetSpatialTileCoordinate(
                                FMath::FloorToInt(Other.GetLocation().X / 1000.0f), 4),
                            ForestDresserBatching::GetSpatialTileCoordinate(
                                FMath::FloorToInt(Other.GetLocation().Y / 1000.0f), 4)),
                        ForestDresserBatching::MakeSpatialTileKey(TileX, TileY));
                }
            }
            Transforms.Add(Transform);
        }
        Chunk->SetVisibility(false);
    }
    TestEqual(TEXT("All generated instances retained"), Transforms.Num(), Forest->NumInstancesSpawned);
    // Reproduce the old single-component implementation using exactly the
    // transforms from the real builder, then compare it to bounded components.
    AActor* ControlActor = World->SpawnActor<AActor>();
    auto* Control = NewObject<UHierarchicalInstancedStaticMeshComponent>(ControlActor);
    ControlActor->SetRootComponent(Control);
    Control->SetStaticMesh(Cube);
    Control->RegisterComponent();
    Control->AddInstances(Transforms, false, true);
    Control->BuildTreeIfOutdated(false, true);
    AActor* CameraActor = World->SpawnActor<AActor>();
    auto* Camera = NewObject<UDroneCaptureComponent>(CameraActor);
    CameraActor->SetRootComponent(Camera);
    Camera->RegisterComponent();
    Camera->InitCapture(128, 128, 60, -90);
    Camera->SetWorldLocation(FVector(4000, 4000, 10000));
    Camera->SetWorldRotation(FRotator(-90, 0, 0));
#if WITH_EDITOR
    FAssetCompilingManager::Get().FinishAllCompilation();
#endif
    auto Read = [&]()
    {
        World->SendAllEndOfFrameUpdates();
        FlushRenderingCommands();
        Camera->CaptureTick(1, true, 1.0f);
        TArray<float> Depth;
        int32 W = 0, H = 0;
        TestTrue(TEXT("Forest depth readback"), Camera->ReadDepthPass(nullptr, Depth, W, H));
        return Depth;
    };
    Budget->Set(100000000, ECVF_SetByCode);
    const TArray<float> Reference = Read();
    TestTrue(TEXT("Reference contains visible trees"), Reference.ContainsByPredicate(
        [](float D) { return D > 0.0f && D < 11000.0f; }));
    Budget->Set(240, ECVF_SetByCode);
    const TArray<float> Dropped = Read();
    TestTrue(TEXT("Single-component control reproduces disappearing trees"), Reference != Dropped);
    Control->SetVisibility(false);
    for (auto* Chunk : Chunks) Chunk->SetVisibility(true);
    const TArray<float> Bounded = Read();
    TestTrue(TEXT("Bounded chunks reproduce all reference pixels under the low budget"), Reference == Bounded);
    TestTrue(TEXT("Repeated bounded depth is exact"), Bounded == Read());
    // Every cell reference must still resolve after a component boundary.
    int32 Anchors = 0;
    for (int32 Y = 0; Y < 8; ++Y)
        for (int32 X = 0; X < 8; ++X)
        {
            TArray<FVector> Locations;
            Anchors += Forest->GetVegetationFireAnchors(X, Y, Locations, 100);
            Forest->ScorchCell(X, Y);
        }
    TestEqual(TEXT("Cell references retain every tree across chunk boundaries"), Anchors, Transforms.Num());
    for (auto* Chunk : Chunks)
        for (float Value : Chunk->PerInstanceSMCustomData)
            TestEqual(TEXT("Scorch reaches instances in every chunk"), Value, 1.0f);
    Forest->ResetScorch();
    for (auto* Chunk : Chunks)
        for (float Value : Chunk->PerInstanceSMCustomData)
            TestEqual(TEXT("Scorch reset reaches every chunk"), Value, 0.0f);
    TArray<float> Fuel;
    for (int32 Y = 0; Y < 8; ++Y)
        for (int32 X = 0; X < 8; ++X) Fuel.Add(Cells.getFloraDensity(X, Y));
    auto SortTransforms = [](TArray<FTransform>& Values)
    {
        Values.Sort([](const FTransform& A, const FTransform& B)
        {
            const FVector AL = A.GetLocation(), BL = B.GetLocation();
            if (AL.X != BL.X) return AL.X < BL.X;
            if (AL.Y != BL.Y) return AL.Y < BL.Y;
            if (AL.Z != BL.Z) return AL.Z < BL.Z;
            const FQuat AR = A.GetRotation(), BR = B.GetRotation();
            if (AR.X != BR.X) return AR.X < BR.X;
            if (AR.Y != BR.Y) return AR.Y < BR.Y;
            if (AR.Z != BR.Z) return AR.Z < BR.Z;
            if (AR.W != BR.W) return AR.W < BR.W;
            const FVector AS = A.GetScale3D(), BS = B.GetScale3D();
            if (AS.X != BS.X) return AS.X < BS.X;
            if (AS.Y != BS.Y) return AS.Y < BS.Y;
            return AS.Z < BS.Z;
        });
    };
    TArray<FTransform> ExpectedTransforms = Transforms;
    SortTransforms(ExpectedTransforms);
    Budget->Set(100000000, ECVF_SetByCode);
    Forest->BuildForest(&Cells, FVector::ZeroVector, 1000.0f, 9340, nullptr);
    Forest->GetComponents(Chunks);
    TestEqual(TEXT("High budget still keeps the four spatial tiles separately cullable"), Chunks.Num(), 4);
    TArray<FTransform> RebuiltTransforms;
    for (auto* Chunk : Chunks)
        for (int32 I = 0; I < Chunk->GetInstanceCount(); ++I)
        {
            FTransform Transform;
            Chunk->GetInstanceTransform(I, Transform, true);
            RebuiltTransforms.Add(Transform);
        }
    SortTransforms(RebuiltTransforms);
    TestEqual(TEXT("Rebuild preserves total instance count"), RebuiltTransforms.Num(), ExpectedTransforms.Num());
    for (int32 I = 0; I < FMath::Min(RebuiltTransforms.Num(), ExpectedTransforms.Num()); ++I)
        TestTrue(TEXT("Changing chunk boundaries preserves exact seeded placement"),
            ExpectedTransforms[I].Equals(RebuiltTransforms[I], 0.0));
    for (int32 Y = 0; Y < 8; ++Y)
        for (int32 X = 0; X < 8; ++X)
            TestEqual(TEXT("Visual chunk budget cannot change simulation fuel"), Cells.getFloraDensity(X, Y), Fuel[Y * 8 + X]);

    Forest->TreeMesh = nullptr;
    Forest->GroundCoverMeshes.Reset();
    Forest->GroundCoverMeshes.Add(Cube);
    Forest->MaxTreesPerCell = 0;
    Forest->GroundCoverPerCell = 1;
    Forest->bGroundCoverCastsShadow = false;
    Forest->BuildForest(&Cells, FVector::ZeroVector, 1000.0f, 9340, nullptr);
    Chunks.Reset();
    Forest->GetComponents(Chunks);
    TestTrue(TEXT("Ground cover instances are still rendered in spatial batches"), Chunks.Num() > 0);
    for (auto* Chunk : Chunks)
        TestFalse(TEXT("Optional small-ground-cover shadow control is applied"), Chunk->CastShadow);

    Camera->DestroyComponent();
    GEngine->DestroyWorldContext(World);
    World->DestroyWorld(false);
    FlushRenderingCommands();
    return true;
}
#endif
