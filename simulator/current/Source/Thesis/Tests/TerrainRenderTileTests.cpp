#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "../World/ProceduralTerrain.h"
#include "ProceduralMeshComponent.h"
#include "Engine/World.h"
#include "Engine/Engine.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTerrainRenderTileTest, "Thesis.Terrain.RenderTileGeometry",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTerrainRenderTileTest::RunTest(const FString& Parameters)
{
    const auto Initialization = UWorld::InitializationValues().AllowAudioPlayback(false)
        .CreatePhysicsScene(false).CreateNavigation(false).CreateAISystem(false).ShouldSimulatePhysics(false);
    UWorld* World = UWorld::CreateWorld(EWorldType::Game, false, TEXT("TerrainTilesFixture"),
        nullptr, true, ERHIFeatureLevel::Num, &Initialization);
    GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
    auto* Terrain = World->SpawnActor<AProceduralTerrain>();
    Terrain->RenderTileQuads = 0;
    Terrain->BuildTerrain(21, 17, FVector::ZeroVector, 1000, 9340);
    const FProcMeshSection Original = *Terrain->Mesh->GetProcMeshSection(0);
    TMap<FVector, int32> VertexIndices;
    for (int32 I = 0; I < Original.ProcVertexBuffer.Num(); ++I)
        VertexIndices.Add(Original.ProcVertexBuffer[I].Position, I);
    TSet<FIntVector> OriginalTriangles;
    for (int32 I = 0; I < Original.ProcIndexBuffer.Num(); I += 3)
        OriginalTriangles.Add(FIntVector(Original.ProcIndexBuffer[I], Original.ProcIndexBuffer[I + 1], Original.ProcIndexBuffer[I + 2]));
    Terrain->RenderTileQuads = 64;
    Terrain->BuildTerrain(21, 17, FVector::ZeroVector, 1000, 9340);
    TArray<UProceduralMeshComponent*> Tiles;
    Terrain->GetComponents(Tiles);
    TestTrue(TEXT("Terrain exposes independent culling primitives"), Tiles.Num() > 1);
    TSet<FIntVector> TiledTriangles;
    bool bSamplesExact = true, bTrianglesExact = true, bBoundsTight = true;
    for (auto* Tile : Tiles)
    {
        const FProcMeshSection* Section = Tile->GetProcMeshSection(0);
        if (!Section) { bSamplesExact = false; continue; }
        const FVector Extent = Section->SectionLocalBox.GetSize();
        bBoundsTight &= Extent.X <= 12800.1 && Extent.Y <= 12800.1;
        for (const FProcMeshVertex& V : Section->ProcVertexBuffer)
        {
            const int32* I = VertexIndices.Find(V.Position);
            if (!I) { bSamplesExact = false; continue; }
            const FProcMeshVertex& Ref = Original.ProcVertexBuffer[*I];
            bSamplesExact &= V.Normal.Equals(Ref.Normal, 0) && V.UV0.Equals(Ref.UV0, 0) && V.Color == Ref.Color;
        }
        for (int32 I = 0; I < Section->ProcIndexBuffer.Num(); I += 3)
        {
            FIntVector Triangle;
            for (int32 J = 0; J < 3; ++J)
                Triangle[J] = VertexIndices.FindChecked(Section->ProcVertexBuffer[Section->ProcIndexBuffer[I + J]].Position);
            bTrianglesExact &= OriginalTriangles.Contains(Triangle) && !TiledTriangles.Contains(Triangle);
            TiledTriangles.Add(Triangle);
        }
        TestTrue(TEXT("All tiles share the same burn-mask material"), Tile->GetMaterial(0) == Terrain->Mesh->GetMaterial(0));
    }
    TestTrue(TEXT("Tile bounds stay within 128 metres including distant apron"), bBoundsTight);
    TestTrue(TEXT("All boundary positions/normals/UVs/colors exact"), bSamplesExact);
    TestTrue(TEXT("Every triangle retains its winding without overlaps"), bTrianglesExact);
    TestEqual(TEXT("No triangles removed or added"), TiledTriangles.Num(), OriginalTriangles.Num());
    Terrain->bRenderVisuals = false;
    Terrain->bVisibleOnlyToDroneCaptures = false;
    Terrain->BuildTerrain(21, 17, FVector::ZeroVector, 1000, 9340);
    Terrain->GetComponents(Tiles);
    TestEqual(TEXT("Headless rebuild releases additional render tiles"), Tiles.Num(), 1);
    TestEqual(TEXT("Headless root has no render sections"), Terrain->Mesh->GetNumSections(), 0);
    GEngine->DestroyWorldContext(World);
    World->DestroyWorld(false);
    return true;
}
#endif
