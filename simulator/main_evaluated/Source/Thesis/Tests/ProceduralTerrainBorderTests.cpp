#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"
#include "../World/ProceduralTerrain.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FProceduralTerrainPresentationBorderTest,
    "Thesis.Terrain.PresentationBorderCoordinates",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FProceduralTerrainPresentationBorderTest::RunTest(const FString& Parameters)
{
    constexpr int32 GridCells = 17;
    constexpr int32 Margin = 12;
    for (const int32 Resolution : {1, 2, 3, 4})
    {
        const int32 GridVertices = GridCells * Resolution;
        const int32 MarginVertices = Margin * Resolution;
        TestEqual(TEXT("Apron begins at negative margin"),
                  AProceduralTerrain::PresentationVertexCellCoordinate(0, Resolution, Margin),
                  -static_cast<float>(Margin));
        TestEqual(TEXT("Simulation grid far edge remains at its original coordinate"),
                  AProceduralTerrain::PresentationVertexCellCoordinate(
                      GridVertices + MarginVertices, Resolution, Margin),
                  static_cast<float>(GridCells));
        TestEqual(TEXT("Apron extends beyond the far grid edge"),
                  AProceduralTerrain::PresentationVertexCellCoordinate(
                      GridVertices + MarginVertices * 2, Resolution, Margin),
                  static_cast<float>(GridCells + Margin));

        for (int32 GridVertex = 0; GridVertex <= GridVertices; ++GridVertex)
        {
            const float Before = static_cast<float>(GridVertex) * (1.0f / static_cast<float>(Resolution));
            const float WithApron = AProceduralTerrain::PresentationVertexCellCoordinate(
                GridVertex + MarginVertices, Resolution, Margin);
            TestEqual(TEXT("Interior vertex coordinate remains exact"), WithApron, Before);
        }

        for (int32 Vertex = 0; Vertex < GridVertices + MarginVertices * 2; ++Vertex)
        {
            const float A = AProceduralTerrain::PresentationVertexCellCoordinate(Vertex, Resolution, Margin);
            const float B = AProceduralTerrain::PresentationVertexCellCoordinate(Vertex + 1, Resolution, Margin);
            TestTrue(TEXT("Terrain samples remain evenly joined across border"),
                     FMath::IsNearlyEqual(B - A, 1.0f / Resolution, KINDA_SMALL_NUMBER));
        }
    }
    return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
