#include "RandomPartition.h"
#include "Math/RandomStream.h"

namespace
{
    static int32 MixPartitionSeed(int32 RunSeed, int32 Tick)
    {
        uint32 X = static_cast<uint32>(RunSeed) ^ 0xA511E9B3u;
        X ^= static_cast<uint32>(Tick) * 0x9E3779B1u;
        X ^= X >> 16;
        X *= 0x7FEB352Du;
        X ^= X >> 15;
        X *= 0x846CA68Bu;
        X ^= X >> 16;
        return static_cast<int32>(X & 0x7fffffffu);
    }
}

TArray<uint16> FRandomPartition::ComputePartition(
    const TArray<TObjectPtr<ADrone>>& ActiveDrones,
    const TArray<FVector2f>& DronePositionsCellSpace,
    int32 Tick,
    const FPartitionParams& Params)
{
    const int32 W = Params.GridWidth;
    const int32 H = Params.GridHeight;
    const int32 N = FMath::Min(ActiveDrones.Num(), DronePositionsCellSpace.Num());

    TArray<uint16> Owners;
    Owners.Init(0xFFFFu, W * H);
    if (W <= 0 || H <= 0 || N <= 0)
    {
        return Owners;
    }

    const int32 TileSide = FMath::Max(2, FMath::FloorToInt(FMath::Min(W, H) * Params.BackgroundTileFraction));
    const int32 TilesX = FMath::DivideAndRoundUp(W, TileSide);
    const int32 TilesY = FMath::DivideAndRoundUp(H, TileSide);
    TArray<uint16> TileOwners;
    TileOwners.SetNumUninitialized(TilesX * TilesY);

    FRandomStream Stream(MixPartitionSeed(Params.RunSeed, Tick));
    for (int32 Tile = 0; Tile < TileOwners.Num(); ++Tile)
    {
        TileOwners[Tile] = static_cast<uint16>(Stream.RandRange(0, N - 1));
    }

    for (int32 Y = 0; Y < H; ++Y)
    {
        const int32 TY = Y / TileSide;
        for (int32 X = 0; X < W; ++X)
        {
            const int32 TX = X / TileSide;
            Owners[Y * W + X] = TileOwners[TY * TilesX + TX];
        }
    }

    UE_LOG(LogTemp, Log,
           TEXT("Thesis Partition tick %d: seeded random tile assignment (%dx%d tiles, seed=%d)."),
           Tick, TilesX, TilesY, MixPartitionSeed(Params.RunSeed, Tick));
    return Owners;
}
