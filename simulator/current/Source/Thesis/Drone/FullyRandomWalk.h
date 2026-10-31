#pragma once
#include "CoreMinimal.h"
#include "Math/RandomStream.h"

namespace FullyRandomWalk
{
    // Uniform over legal neighbouring cells, with no priority or visit bias.
    template <typename FIsOwned>
    int32 ChooseStep(int32 Width, int32 Height, int32 Start, int32 RunSeed,
                     int32 DroneId, int32 Tick, FIsOwned IsOwned)
    {
        if (Width <= 0 || Height <= 0 || Start < 0 ||
            static_cast<int64>(Start) >= static_cast<int64>(Width) * Height) return INDEX_NONE;
        uint32 Seed = static_cast<uint32>(RunSeed) ^ 0xD1B54A35u;
        Seed ^= (static_cast<uint32>(DroneId) + 1u) * 0x9E3779B1u;
        Seed ^= (static_cast<uint32>(Tick) + 17u) * 0x85EBCA6Bu;
        Seed ^= Seed >> 16;
        FRandomStream Stream(static_cast<int32>(Seed & 0x7fffffffu));
        constexpr int32 DX[8] = {1,1,0,-1,-1,-1,0,1};
        constexpr int32 DY[8] = {0,-1,-1,-1,0,1,1,1};
        const int32 X = Start % Width, Y = Start / Width;
        const bool bInside = IsOwned(Start);
        TArray<int32, TInlineAllocator<8>> Candidates;
        for (int32 Dir = 0; Dir < 8; ++Dir)
        {
            const int32 NX = X + DX[Dir], NY = Y + DY[Dir];
            if (NX < 0 || NY < 0 || NX >= Width || NY >= Height) continue;
            const int32 Cell = NY * Width + NX;
            if (!bInside || IsOwned(Cell)) Candidates.Add(Cell);
        }
        return Candidates.IsEmpty() ? Start : Candidates[Stream.RandRange(0, Candidates.Num() - 1)];
    }
}
