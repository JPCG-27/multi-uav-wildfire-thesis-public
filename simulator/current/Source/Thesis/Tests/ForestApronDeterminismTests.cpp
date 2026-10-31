#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "../World/ForestDresser.h"
#include "../World/Grid.h"
#include "../World/ProceduralTerrain.h"
#include "../Vision/DroneCaptureComponent.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/HierarchicalInstancedStaticMeshComponent.h"
#include "Engine/DirectionalLight.h"
#include "Engine/Engine.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "HAL/FileManager.h"
#include "ImageUtils.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "RenderingThread.h"

namespace
{
uint32 ForestApronFloatBits(float Value)
{
    uint32 Bits = 0;
    FMemory::Memcpy(&Bits, &Value, sizeof(Bits));
    return Bits;
}

FString ForestApronTransformKey(const UHierarchicalInstancedStaticMeshComponent* Component,
                                const FTransform& Transform)
{
    const FVector Location = Transform.GetLocation();
    const FQuat Rotation = Transform.GetRotation();
    const FVector Scale = Transform.GetScale3D();
    return FString::Printf(TEXT("%s|%08x|%08x|%08x|%08x|%08x|%08x|%08x|%08x|%08x|%08x"),
        Component->GetStaticMesh() ? *Component->GetStaticMesh()->GetPathName() : TEXT("<none>"),
        ForestApronFloatBits(Location.X), ForestApronFloatBits(Location.Y), ForestApronFloatBits(Location.Z),
        ForestApronFloatBits(Rotation.X), ForestApronFloatBits(Rotation.Y),
        ForestApronFloatBits(Rotation.Z), ForestApronFloatBits(Rotation.W),
        ForestApronFloatBits(Scale.X), ForestApronFloatBits(Scale.Y), ForestApronFloatBits(Scale.Z));
}

void CollectForestApronKeys(AForestDresser* Forest, const AProceduralTerrain* Terrain,
                            float CellSizeUU, float GridWidthUU, float GridHeightUU,
                            float DetailedMarginUU, float ApronUU, TArray<FString>& InGrid,
                            TArray<FString>& FarApron, TArray<FString>& AllKeys,
                            int32& OutInstanceCount, int32& OutBadFarBounds,
                            int32& OutVisibilityMismatches)
{
    TArray<UHierarchicalInstancedStaticMeshComponent*> Components;
    Forest->GetComponents(Components);
    OutInstanceCount = 0;
    OutBadFarBounds = 0;
    OutVisibilityMismatches = 0;
    for (const UHierarchicalInstancedStaticMeshComponent* Component : Components)
    {
        if (!Component) continue;
        if (Component->bVisibleInSceneCaptureOnly != Forest->bVisibleOnlyToDroneCaptures)
            ++OutVisibilityMismatches;
        for (int32 Index = 0; Index < Component->GetInstanceCount(); ++Index)
        {
            FTransform Transform;
            if (!Component->GetInstanceTransform(Index, Transform, true)) continue;
            const FString Key = ForestApronTransformKey(Component, Transform);
            ++OutInstanceCount;
            AllKeys.Add(Key);
            const FVector P = Transform.GetLocation();
            if (P.X >= 0.0f && P.X < GridWidthUU && P.Y >= 0.0f && P.Y < GridHeightUU)
            {
                InGrid.Add(Key);
            }
            else if (P.X < -DetailedMarginUU || P.X >= GridWidthUU + DetailedMarginUU
                     || P.Y < -DetailedMarginUU || P.Y >= GridHeightUU + DetailedMarginUU)
            {
                FarApron.Add(Key);
                if (!FMath::IsFinite(P.X) || !FMath::IsFinite(P.Y) || !FMath::IsFinite(P.Z))
                    ++OutBadFarBounds;
                if (P.X < -ApronUU - 0.1f || P.X > GridWidthUU + ApronUU + 0.1f
                    || P.Y < -ApronUU - 0.1f || P.Y > GridHeightUU + ApronUU + 0.1f)
                    ++OutBadFarBounds;
                if (Component->GetStaticMesh()
                    && Component->GetStaticMesh()->GetPathName() == TEXT("/Engine/BasicShapes/Cube.Cube"))
                {
                    const float Ground = Terrain->GetHeightAtCell(P.X / CellSizeUU, P.Y / CellSizeUU);
                    if (FMath::Abs(P.Z - Ground) > 1.0f)
                        ++OutBadFarBounds;
                }
            }
        }
    }
}

void SortKeys(TArray<FString>& Keys)
{
    Keys.Sort();
}

bool SaveCapturePng(UDroneCaptureComponent* Camera, const FString& Path)
{
    TArray<FColor> Pixels;
    int32 Width = 0, Height = 0, Tick = -1;
    if (!Camera->ReadLastCapture(Pixels, Width, Height, Tick) || Pixels.Num() != Width * Height)
        return false;
    TArray64<uint8> Png;
    FImageUtils::PNGCompressImageArray(Width, Height, Pixels, Png);
    IFileManager::Get().MakeDirectory(*FPaths::GetPath(Path), true);
    return FFileHelper::SaveArrayToFile(Png, *Path);
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FForestApronDeterminismTest,
    "Thesis.Forest.ApronDeterminismAndBoundary",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FForestApronDeterminismTest::RunTest(const FString& Parameters)
{
    const auto Initialization = UWorld::InitializationValues().AllowAudioPlayback(false)
        .CreatePhysicsScene(false).CreateNavigation(false).CreateAISystem(false).ShouldSimulatePhysics(false);
    UWorld* World = UWorld::CreateWorld(EWorldType::Game, false, TEXT("ForestApronFixture"),
        nullptr, true, ERHIFeatureLevel::Num, &Initialization);
    if (!TestNotNull(TEXT("Apron fixture world"), World)) return false;
    FWorldContext& Context = GEngine->CreateNewWorldContext(EWorldType::Game);
    Context.SetCurrentWorld(World);

    UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
    AProceduralTerrain* Terrain = World->SpawnActor<AProceduralTerrain>();
    AForestDresser* Forest = World->SpawnActor<AForestDresser>();
    if (!TestNotNull(TEXT("Terrain"), Terrain) || !TestNotNull(TEXT("Forest"), Forest)
        || !TestNotNull(TEXT("Cube stand-in"), Cube))
    {
        GEngine->DestroyWorldContext(World);
        World->DestroyWorld(false);
        return false;
    }

    constexpr int32 GridW = 8;
    constexpr int32 GridH = 8;
    constexpr float CellSizeUU = 300.0f;
    constexpr float GridWidthUU = GridW * CellSizeUU;
    constexpr float GridHeightUU = GridH * CellSizeUU;
    constexpr float ApronMeters = 1000.0f;
    constexpr float ApronUU = ApronMeters * 100.0f;
    constexpr int32 DetailMarginCells = 12;
    const int32 Seed = 71231;

    Terrain->Profile = ETerrainProfile::Foothills;
    Terrain->SlopeIntensity = 0.8f;
    Terrain->PresentationMarginCells = DetailMarginCells;
    Terrain->PresentationMarginMeters = ApronMeters;
    Terrain->RenderTileQuads = 64;
    Terrain->VerticesPerCell = 1;
    Terrain->BuildTerrain(GridW, GridH, FVector::ZeroVector, CellSizeUU, Seed);

    Forest->bAutoPopulateFromThesisFlora = false;
    Forest->bPreferFullyResidentMeshes = false;
    Forest->TreeMeshes.Add(Cube);
    Forest->ForestCoverage = 0.82f;
    Forest->NoiseScaleCells = 12.0f;
    Forest->MaxTreesPerCell = 4;
    Forest->MaxBushesPerCell = 0;
    Forest->GroundCoverPerCell = 0;
    Forest->SurfaceDebrisPerCell = 0;
    Forest->PresentationMarginCells = DetailMarginCells;
    Forest->PresentationApronMaxPatches = 0;
    Forest->bVisibleOnlyToDroneCaptures = true;

    Grid Cells(GridW, GridH);
    Forest->BuildForest(&Cells, FVector::ZeroVector, CellSizeUU, Seed, Terrain);

    TArray<float> BaselineFuel;
    BaselineFuel.Reserve(GridW * GridH);
    TArray<TArray<FVector>> BaselineAnchors;
    BaselineAnchors.SetNum(GridW * GridH);
    for (int32 Y = 0; Y < GridH; ++Y)
        for (int32 X = 0; X < GridW; ++X)
        {
            BaselineFuel.Add(Cells.getFloraDensity(X, Y));
            Forest->GetVegetationFireAnchors(X, Y, BaselineAnchors[Y * GridW + X], 32);
        }

    const float DetailedMarginUU = DetailMarginCells * CellSizeUU;
    TArray<FString> BaselineInGrid, BaselineFarApron, BaselineAll;
    int32 BaselineCount = 0, BaselineBadBounds = 0, BaselineVisibilityMismatches = 0;
    CollectForestApronKeys(Forest, Terrain, CellSizeUU, GridWidthUU, GridHeightUU,
        DetailedMarginUU, ApronUU, BaselineInGrid, BaselineFarApron, BaselineAll,
        BaselineCount, BaselineBadBounds, BaselineVisibilityMismatches);
    SortKeys(BaselineInGrid);
    SortKeys(BaselineFarApron);
    SortKeys(BaselineAll);
    TestEqual(TEXT("Zero far-apron budget leaves only the preexisting detailed edge band"),
        BaselineFarApron.Num(), 0);
    TestEqual(TEXT("Existing forest chunks keep scene-capture visibility policy"),
        BaselineVisibilityMismatches, 0);

    Forest->PresentationApronMaxPatches = 256;
    Forest->BuildForest(&Cells, FVector::ZeroVector, CellSizeUU, Seed, Terrain);
    TArray<FString> ApronInGrid, ApronFar, ApronAll;
    int32 ApronCount = 0, ApronBadBounds = 0, ApronVisibilityMismatches = 0;
    CollectForestApronKeys(Forest, Terrain, CellSizeUU, GridWidthUU, GridHeightUU,
        DetailedMarginUU, ApronUU, ApronInGrid, ApronFar, ApronAll,
        ApronCount, ApronBadBounds, ApronVisibilityMismatches);
    SortKeys(ApronInGrid);
    SortKeys(ApronFar);
    SortKeys(ApronAll);

    TestTrue(TEXT("Far terrain apron receives sparse deterministic flora"),
        ApronFar.Num() > 0);
    TestTrue(TEXT("Far-apron scatter stays within the configured 1000m terrain bounds"),
        ApronBadBounds == 0 && ApronFar.Num() <= 256 * 32);
    TestEqual(TEXT("Far-apron scatter keeps scene-capture visibility policy"),
        ApronVisibilityMismatches, 0);
    TestTrue(TEXT("Apron adds only a bounded number of instances"),
        ApronCount <= BaselineCount + 256 * 32);
    TestEqual(TEXT("Apron placement does not alter in-grid instance count"),
        ApronInGrid.Num(), BaselineInGrid.Num());
    TestTrue(TEXT("All original in-grid mesh transforms are bit-identical"),
        ApronInGrid == BaselineInGrid);

    for (int32 Y = 0; Y < GridH; ++Y)
        for (int32 X = 0; X < GridW; ++X)
        {
            const int32 CellIndex = Y * GridW + X;
            TestEqual(TEXT("Apron placement cannot alter simulation fuel"),
                Cells.getFloraDensity(X, Y), BaselineFuel[CellIndex]);
            TArray<FVector> Anchors;
            Forest->GetVegetationFireAnchors(X, Y, Anchors, 32);
            TestTrue(TEXT("Apron instances do not enter interior fire-anchor mapping"),
                Anchors == BaselineAnchors[CellIndex]);
        }

    const TArray<FString> FirstApronKeys = ApronFar;
    Forest->BuildForest(&Cells, FVector::ZeroVector, CellSizeUU, Seed, Terrain);
    TArray<FString> RebuiltInGrid, RebuiltApron, RebuiltAll;
    int32 RebuiltCount = 0, RebuiltBadBounds = 0, RebuiltVisibilityMismatches = 0;
    CollectForestApronKeys(Forest, Terrain, CellSizeUU, GridWidthUU, GridHeightUU,
        DetailedMarginUU, ApronUU, RebuiltInGrid, RebuiltApron, RebuiltAll,
        RebuiltCount, RebuiltBadBounds, RebuiltVisibilityMismatches);
    SortKeys(RebuiltApron);
    SortKeys(RebuiltAll);
    TestTrue(TEXT("Same seed reproduces exact far-apron instance transforms"),
        FirstApronKeys == RebuiltApron);
    TestTrue(TEXT("Warm layout-cache replay reproduces every flora transform"),
        ApronAll == RebuiltAll);
    for (int32 Y = 0; Y < GridH; ++Y)
        for (int32 X = 0; X < GridW; ++X)
        {
            const int32 CellIndex = Y * GridW + X;
            TestEqual(TEXT("Warm layout-cache replay republishes exact fuel density"),
                Cells.getFloraDensity(X, Y), BaselineFuel[CellIndex]);
            TArray<FVector> Anchors;
            Forest->GetVegetationFireAnchors(X, Y, Anchors, 32);
            TestTrue(TEXT("Warm layout-cache replay restores fire-anchor mappings"),
                Anchors == BaselineAnchors[CellIndex]);
        }

    Forest->ScorchCell(0, 0);
    Forest->ResetScorch();
    TArray<FVector> ResetAnchors;
    Forest->GetVegetationFireAnchors(0, 0, ResetAnchors, 32);
    TestTrue(TEXT("Scorch reset leaves cached-layout fire anchors intact"),
        ResetAnchors == BaselineAnchors[0]);

    Forest->TreeZOffsetUU += 37.0f;
    Forest->BuildForest(&Cells, FVector::ZeroVector, CellSizeUU, Seed, Terrain);
    TArray<FString> ChangedInGrid, ChangedApron, ChangedAll;
    int32 ChangedCount = 0, ChangedBadBounds = 0, ChangedVisibilityMismatches = 0;
    CollectForestApronKeys(Forest, Terrain, CellSizeUU, GridWidthUU, GridHeightUU,
        DetailedMarginUU, ApronUU, ChangedInGrid, ChangedApron, ChangedAll,
        ChangedCount, ChangedBadBounds, ChangedVisibilityMismatches);
    SortKeys(ChangedAll);
    TestTrue(TEXT("Changing tree Z offset invalidates the cached transform layout"),
        ChangedAll != ApronAll);
    Forest->TreeZOffsetUU -= 37.0f;
    Forest->BuildForest(&Cells, FVector::ZeroVector, CellSizeUU, Seed, Terrain);
    TArray<FString> RestoredInGrid, RestoredApron, RestoredAll;
    int32 RestoredCount = 0, RestoredBadBounds = 0, RestoredVisibilityMismatches = 0;
    CollectForestApronKeys(Forest, Terrain, CellSizeUU, GridWidthUU, GridHeightUU,
        DetailedMarginUU, ApronUU, RestoredInGrid, RestoredApron, RestoredAll,
        RestoredCount, RestoredBadBounds, RestoredVisibilityMismatches);
    SortKeys(RestoredAll);
    TestTrue(TEXT("Restored input restores the original exact layout deterministically"),
        RestoredAll == ApronAll);

    // Use the imported Geres flora when available for a visual review image.
    // This remains a presentation-only fixture; fire/human simulation is absent.
    UStaticMesh* Beech = LoadObject<UStaticMesh>(nullptr,
        TEXT("/Game/EuropeanBeech/Geometry/SimpleWind/SM_EuropeanBeech_Forest_02.SM_EuropeanBeech_Forest_02"));
    AForestDresser* VisualForest = World->SpawnActor<AForestDresser>();
    if (Beech && VisualForest)
    {
        VisualForest->bAutoPopulateFromThesisFlora = true;
        VisualForest->bPreferFullyResidentMeshes = false;
        VisualForest->ForestCoverage = 0.92f;
        VisualForest->NoiseScaleCells = 12.0f;
        VisualForest->MaxTreesPerCell = 1;
        VisualForest->MaxBushesPerCell = 0;
        VisualForest->GroundCoverPerCell = 0;
        VisualForest->SurfaceDebrisPerCell = 0;
        VisualForest->PresentationMarginCells = DetailMarginCells;
        // Keep the render review dense enough that several real tree samples
        // appear in each camera footprint; the assertion fixture above still
        // tests a small deterministic budget for speed.
        VisualForest->PresentationApronMaxPatches = 4096;
        VisualForest->BuildForest(&Cells, FVector::ZeroVector, CellSizeUU, Seed, Terrain);

        ADirectionalLight* Sun = World->SpawnActor<ADirectionalLight>();
        if (Sun)
        {
            Sun->SetActorRotation(FRotator(-45.0f, 35.0f, 0.0f));
            Sun->GetLightComponent()->SetIntensity(75000.0f);
        }
        AActor* CameraActor = World->SpawnActor<AActor>();
        UDroneCaptureComponent* Camera = NewObject<UDroneCaptureComponent>(CameraActor);
        CameraActor->SetRootComponent(Camera);
        Camera->RegisterComponent();
        Camera->InitCapture(640, 480, 70.0f, -25.0f);
        Camera->SetWorldLocation(FVector(0.0f, GridHeightUU * 0.5f, 8000.0f));
        Camera->SetWorldRotation(FRotator(-38.0f, 180.0f, 0.0f));
        Camera->SetExposure(false, 0.0f, 0.0f, 12.0f);
        // Flush newly-built HISM and terrain render state before the first
        // capture; otherwise an offscreen automation world can read the
        // render target before those components have reached the scene.
        World->SendAllEndOfFrameUpdates();
        FlushRenderingCommands();
        Camera->CaptureTick(1, true, 1.0f);
        TArray<FColor> Pixels;
        int32 ReadW = 0, ReadH = 0, ReadTick = -1;
        const bool bRead = Camera->ReadLastCapture(Pixels, ReadW, ReadH, ReadTick);
        TestTrue(TEXT("Far-apron visual review capture"), bRead && Pixels.Num() == ReadW * ReadH);
        if (bRead)
        {
            int32 NonBlackPixels = 0;
            for (const FColor& Pixel : Pixels)
                if (Pixel.R > 8 || Pixel.G > 8 || Pixel.B > 8) ++NonBlackPixels;
            AddInfo(FString::Printf(TEXT("Edge apron review image: %d nonblack pixels of %d"), NonBlackPixels, Pixels.Num()));
            TestTrue(TEXT("Edge apron review image contains rendered scene content"), NonBlackPixels > Pixels.Num() / 100);
            const FString ImagePath = FPaths::Combine(FPaths::ProjectSavedDir(),
                TEXT("LiveModeQA/forest_apron_visual/edge_apron.png"));
            TestTrue(TEXT("Save far-apron review image"), SaveCapturePng(Camera, ImagePath));
            Camera->SetWorldLocation(FVector(-ApronUU * 0.5f, GridHeightUU * 0.5f, 8000.0f));
            Camera->SetWorldRotation(FRotator(-38.0f, 180.0f, 0.0f));
            World->SendAllEndOfFrameUpdates();
            FlushRenderingCommands();
            Camera->CaptureTick(2, true, 1.0f);
            TArray<FColor> FarPixels;
            int32 FarW = 0, FarH = 0, FarTick = -1;
            const bool bFarRead = Camera->ReadLastCapture(FarPixels, FarW, FarH, FarTick);
            int32 FarNonBlackPixels = 0;
            if (bFarRead)
                for (const FColor& Pixel : FarPixels)
                    if (Pixel.R > 8 || Pixel.G > 8 || Pixel.B > 8) ++FarNonBlackPixels;
            AddInfo(FString::Printf(TEXT("Far apron review image: %d nonblack pixels of %d"), FarNonBlackPixels, FarPixels.Num()));
            TestTrue(TEXT("Far apron review image contains rendered scene content"),
                bFarRead && FarPixels.Num() == FarW * FarH && FarNonBlackPixels > FarPixels.Num() / 100);
            TestTrue(TEXT("Save far-apron horizon image"),
                SaveCapturePng(Camera, FPaths::Combine(FPaths::ProjectSavedDir(),
                    TEXT("LiveModeQA/forest_apron_visual/far_apron.png"))));
        }
    }
    else
    {
        AddWarning(TEXT("European Beech asset missing; skipped optional real-mesh screenshot."));
    }

    GEngine->DestroyWorldContext(World);
    World->DestroyWorld(false);
    FlushRenderingCommands();
    return true;
}
#endif

