#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "../World/ForestDresserBatching.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FForestDresserBatchingTest,
    "Thesis.Forest.HISMChunkTriangleBudget",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FForestDresserBatchingTest::RunTest(const FString& Parameters)
{
	using ForestDresserBatching::ComputeInstancesPerComponent;
	const int32 ChunkSize = ComputeInstancesPerComponent(550781, 100000000);
	TestEqual(TEXT("Chunk leaves half the engine triangle budget for traversal overhead"), ChunkSize, 90);
	TestTrue(TEXT("Chunk worst-case triangle count stays below the full traversal cap"),
		int64(ChunkSize) * 550781 < 100000000);
	TestEqual(TEXT("A single oversized instance is isolated in its own component"),
		ComputeInstancesPerComponent(60000000, 100000000), 1);
	TestEqual(TEXT("Zero triangle budget still produces bounded single-instance components"),
		ComputeInstancesPerComponent(550781, 0), 1);
	TestEqual(TEXT("Unknown triangle data is kept isolated conservatively"),
		ComputeInstancesPerComponent(0, 100000000), 1);
	const auto LastInFirstChunk = ForestDresserBatching::GetChunkAddress(89, ChunkSize);
	TestEqual(TEXT("Last slot of first chunk maps to component zero"), LastInFirstChunk.ComponentOrdinal, 0);
	TestEqual(TEXT("Last slot keeps its local instance index"), LastInFirstChunk.InstanceOrdinal, 89);
	const auto FirstInSecondChunk = ForestDresserBatching::GetChunkAddress(90, ChunkSize);
	TestEqual(TEXT("Next tree maps into a new bounded component"), FirstInSecondChunk.ComponentOrdinal, 1);
	TestEqual(TEXT("New component custom-data and scorch index starts at zero"), FirstInSecondChunk.InstanceOrdinal, 0);
	TestEqual(TEXT("Positive cells map to their spatial tile"),
		ForestDresserBatching::GetSpatialTileCoordinate(31, 16), 1);
	TestEqual(TEXT("Negative presentation cells use floor division"),
		ForestDresserBatching::GetSpatialTileCoordinate(-1, 16), -1);
	TestEqual(TEXT("Negative tile coordinates produce stable keys"),
		ForestDresserBatching::MakeSpatialTileKey(-1, 2), ForestDresserBatching::MakeSpatialTileKey(-1, 2));
	TestTrue(TEXT("Adjacent spatial tiles have distinct component keys"),
		ForestDresserBatching::MakeSpatialTileKey(0, 0) != ForestDresserBatching::MakeSpatialTileKey(1, 0));
	return true;
}
#endif
