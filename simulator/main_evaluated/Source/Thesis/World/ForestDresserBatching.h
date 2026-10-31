#pragma once

#include "CoreMinimal.h"

namespace ForestDresserBatching
{
	/** Floor-divide cell coordinates so exterior negative cells map consistently. */
	inline int32 GetSpatialTileCoordinate(int32 CellCoordinate, int32 TileSideCells)
	{
		const int32 SafeSide = FMath::Max(1, TileSideCells);
		const int32 Quotient = CellCoordinate / SafeSide;
		const int32 Remainder = CellCoordinate % SafeSide;
		return (Remainder < 0) ? Quotient - 1 : Quotient;
	}

	/** Stable key for a signed X/Y tile pair. */
	inline int64 MakeSpatialTileKey(int32 TileX, int32 TileY)
	{
		return static_cast<int64>((static_cast<uint64>(static_cast<uint32>(TileX)) << 32)
			| static_cast<uint32>(TileY));
	}

	struct FChunkAddress
	{
		int32 ComponentOrdinal = 0;
		int32 InstanceOrdinal = 0;
	};

	inline FChunkAddress GetChunkAddress(int32 MeshInstanceOrdinal, int32 InstancesPerComponent)
	{
		const int32 SafeChunkSize = FMath::Max(1, InstancesPerComponent);
		return { MeshInstanceOrdinal / SafeChunkSize, MeshInstanceOrdinal % SafeChunkSize };
	}

	/** Keep each HISM traversal comfortably below Unreal's foliage triangle cap. */
	inline int32 ComputeInstancesPerComponent(int64 MaxTrianglesPerInstance, int64 TriangleBudget)
	{
		if (MaxTrianglesPerInstance <= 0 || TriangleBudget <= 0)
		{
			return 1;
		}

		// HISM counts all sections and may count both sides of an LOD transition.
		// Keep two maximum-LOD copies strictly below the engine's strict (<) cap.
		const int64 SafeTriangleBudget = (TriangleBudget - 1) / 2;
		return static_cast<int32>(FMath::Clamp<int64>(
			SafeTriangleBudget / MaxTrianglesPerInstance, 1, MAX_int32));
	}
}
