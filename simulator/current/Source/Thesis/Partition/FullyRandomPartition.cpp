#include "FullyRandomPartition.h"

#include "Math/RandomStream.h"

namespace
{
    int32 MixSeed(int32 RunSeed, int32 Tick)
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

    struct FFrontierEntry
    {
        int32 Cell = INDEX_NONE;
        uint16 Owner = 0;
    };
}

TArray<uint16> FFullyRandomPartition::ComputeOwners(
    int32 Width, int32 Height, int32 DroneCount, int32 RunSeed, int32 Tick)
{
    TArray<uint16> Owners;
    if (Width <= 0 || Height <= 0)
    {
        return Owners;
    }

    const int64 CellCount64 = static_cast<int64>(Width) * static_cast<int64>(Height);
    if (CellCount64 > MAX_int32)
    {
        return Owners;
    }
    const int32 CellCount = static_cast<int32>(CellCount64);
    Owners.Init(0xFFFFu, CellCount);
    if (DroneCount <= 0 || CellCount == 0)
    {
        return Owners;
    }

    // 0xFFFF is the unowned sentinel, so valid owner labels are 0..0xFFFE.
    const int32 OwnerCount = FMath::Min3(DroneCount, CellCount, 0xFFFF);
    if (OwnerCount <= 0)
    {
        return Owners;
    }

    FRandomStream Stream(MixSeed(RunSeed, Tick));

    // Partial Fisher-Yates gives distinct uniformly sampled seed cells.
    TArray<int32> CellOrder;
    CellOrder.SetNumUninitialized(CellCount);
    for (int32 Cell = 0; Cell < CellCount; ++Cell)
    {
        CellOrder[Cell] = Cell;
    }

    // Randomize owner labels independently of cell seed order.
    TArray<uint16> OwnerOrder;
    OwnerOrder.SetNumUninitialized(OwnerCount);
    for (int32 Owner = 0; Owner < OwnerCount; ++Owner)
    {
        OwnerOrder[Owner] = static_cast<uint16>(Owner);
    }
    for (int32 I = OwnerCount - 1; I > 0; --I)
    {
        const int32 J = Stream.RandRange(0, I);
        OwnerOrder.Swap(I, J);
    }

    TArray<FFrontierEntry> Frontier;
    Frontier.Reserve(static_cast<int32>(FMath::Min<int64>(static_cast<int64>(CellCount) * 2, MAX_int32)));
    for (int32 I = 0; I < OwnerCount; ++I)
    {
        const int32 J = I + Stream.RandRange(0, CellCount - I - 1);
        CellOrder.Swap(I, J);
        const int32 SeedCell = CellOrder[I];
        const uint16 Owner = OwnerOrder[I];
        Owners[SeedCell] = Owner;

        const int32 X = SeedCell % Width;
        const int32 Y = SeedCell / Width;
        if (X > 0) Frontier.Add({SeedCell - 1, Owner});
        if (X + 1 < Width) Frontier.Add({SeedCell + 1, Owner});
        if (Y > 0) Frontier.Add({SeedCell - Width, Owner});
        if (Y + 1 < Height) Frontier.Add({SeedCell + Width, Owner});
    }

    int32 AssignedCount = OwnerCount;
    while (AssignedCount < CellCount && !Frontier.IsEmpty())
    {
        const int32 EntryIndex = Stream.RandRange(0, Frontier.Num() - 1);
        const FFrontierEntry Entry = Frontier[EntryIndex];
        Frontier.RemoveAtSwap(EntryIndex, 1, EAllowShrinking::No);
        if (Owners[Entry.Cell] != 0xFFFFu)
        {
            continue;
        }

        Owners[Entry.Cell] = Entry.Owner;
        ++AssignedCount;

        const int32 X = Entry.Cell % Width;
        const int32 Y = Entry.Cell / Width;
        if (X > 0 && Owners[Entry.Cell - 1] == 0xFFFFu) Frontier.Add({Entry.Cell - 1, Entry.Owner});
        if (X + 1 < Width && Owners[Entry.Cell + 1] == 0xFFFFu) Frontier.Add({Entry.Cell + 1, Entry.Owner});
        if (Y > 0 && Owners[Entry.Cell - Width] == 0xFFFFu) Frontier.Add({Entry.Cell - Width, Entry.Owner});
        if (Y + 1 < Height && Owners[Entry.Cell + Width] == 0xFFFFu) Frontier.Add({Entry.Cell + Width, Entry.Owner});
    }

    return Owners;
}

TArray<uint16> FFullyRandomPartition::ComputePartition(
    const TArray<TObjectPtr<ADrone>>& ActiveDrones,
    const TArray<FVector2f>& DronePositionsCellSpace,
    int32 Tick,
    const FPartitionParams& Params)
{
    // Drone positions are intentionally ignored: only the active drone count
    // controls the random territory count.
    return ComputeOwners(Params.GridWidth, Params.GridHeight, ActiveDrones.Num(), Params.RunSeed, Tick);
}
