#pragma once

#include "CoreMinimal.h"

namespace FirePresentation
{
    // A budgeted prefix stays connected within each fire region. Regions take
    // turns so a large first ignition cannot consume every visible component.
    // Earliest ignition seeds each flood fill; inputs may arrive in any order.
    inline TArray<int32> ConnectedCellOrder(const TArray<int32>& Cells, int32 Width,
        int32 Height, const TArray<int32>& IgnitionTicks)
    {
        TArray<int32> Seeds = Cells;
        Seeds.Sort([&](int32 A, int32 B)
        {
            const int32 TA = IgnitionTicks.IsValidIndex(A) ? IgnitionTicks[A] : MAX_int32;
            const int32 TB = IgnitionTicks.IsValidIndex(B) ? IgnitionTicks[B] : MAX_int32;
            return TA != TB ? TA < TB : A < B;
        });
        TSet<int32> Active;
        for (int32 Cell : Seeds)
            if (Cell >= 0 && Width > 0 && Cell < Width * Height) Active.Add(Cell);
        TSet<int32> Visited;
        TArray<TArray<int32>> Regions;
        for (int32 Seed : Seeds)
        {
            if (!Active.Contains(Seed) || Visited.Contains(Seed)) continue;
            TArray<int32>& Queue = Regions.AddDefaulted_GetRef();
            Queue.Add(Seed);
            Visited.Add(Seed);
            for (int32 Head = 0; Head < Queue.Num(); ++Head)
            {
                const int32 Cell = Queue[Head];
                const int32 X = Cell % Width, Y = Cell / Width;
                const int32 Neighbours[] = {
                    X > 0 ? Cell - 1 : INDEX_NONE,
                    X + 1 < Width ? Cell + 1 : INDEX_NONE,
                    Y > 0 ? Cell - Width : INDEX_NONE,
                    Y + 1 < Height ? Cell + Width : INDEX_NONE };
                for (int32 Neighbour : Neighbours)
                    if (Active.Contains(Neighbour) && !Visited.Contains(Neighbour))
                    {
                        Visited.Add(Neighbour);
                        Queue.Add(Neighbour);
                    }
            }
        }
        TArray<int32> Result;
        Result.Reserve(Active.Num());
        TArray<int32> ActiveRegions, NextRegions;
        for (int32 Region = 0; Region < Regions.Num(); ++Region) ActiveRegions.Add(Region);
        for (int32 Depth = 0; !ActiveRegions.IsEmpty(); ++Depth)
        {
            NextRegions.Reset();
            for (int32 Region : ActiveRegions)
            {
                Result.Add(Regions[Region][Depth]);
                if (Depth + 1 < Regions[Region].Num()) NextRegions.Add(Region);
            }
            Swap(ActiveRegions, NextRegions);
        }
        return Result;
    }

    struct FNiagaraStepPlan
    {
        bool bReconstruct;
        int32 StepsToAdvance;
        int32 TargetSteps;
    };

    // Decide whether a component can continue its current deterministic loop.
    // Keep absolute age as well as the modulo target so crossing a whole loop
    // always resets its particle seed/history, including large tick jumps.
    inline FNiagaraStepPlan PlanNiagaraSteps(bool bIncremental, bool bCompatibleState,
        bool bForceReconstruction, int32 Tick, int32 PreviousTick,
        int64 AgeSteps, int64 PreviousAgeSteps, int32 PreviousTargetSteps,
        int32 PeriodSteps, int32 WarmupSteps)
    {
        const int32 TargetSteps = WarmupSteps + static_cast<int32>(AgeSteps % PeriodSteps);
        const bool bCrossedLoop = PreviousAgeSteps >= 0
            && AgeSteps / PeriodSteps != PreviousAgeSteps / PeriodSteps;
        const bool bReconstruct = !bIncremental || !bCompatibleState || bForceReconstruction
            || PreviousTick < 0 || Tick < PreviousTick
            || PreviousAgeSteps < 0 || AgeSteps < PreviousAgeSteps || bCrossedLoop
            || PreviousTargetSteps < 0 || TargetSteps < PreviousTargetSteps;
        return { bReconstruct,
                 bReconstruct ? TargetSteps : TargetSteps - PreviousTargetSteps,
                 TargetSteps };
    }

    // Stable visual variation keyed only by scene identity. This is deliberately
    // independent of FRandomStream, frame time and Niagara's shared RNG state.
    inline float ClusterScaleVariation(int32 Cell, int32 Cluster)
    {
        uint32 Value = static_cast<uint32>(Cell) * 0x9E3779B9u
            ^ static_cast<uint32>(Cluster + 1) * 0x85EBCA6Bu;
        Value ^= Value >> 16;
        Value *= 0x7FEB352Du;
        Value ^= Value >> 15;
        Value *= 0x846CA68Bu;
        Value ^= Value >> 16;
        const float Unit = static_cast<float>(Value >> 8) * (1.0f / 16777215.0f);
        return 0.86f + Unit * 0.28f;
    }

    struct FSmokeLayerProfile
    {
        float Scale;
        float Intensity;
        float NoiseStrength;
    };

    // Lower smoke is denser and more compact; upper layers spread, thin and
    // break up. Layer indices are fixed by the pool, so this remains repeatable.
    inline FSmokeLayerProfile SmokeLayerProfile(int32 LayerIndex, int32 LayerCount)
    {
        const float Height = LayerCount > 1
            ? FMath::Clamp(static_cast<float>(LayerIndex) / static_cast<float>(LayerCount - 1), 0.0f, 1.0f)
            : 0.0f;
        return { 1.0f + 0.50f * Height,
                 1.0f - 0.50f * Height,
                 0.45f + 0.35f * Height };
    }

    // A plume cross-section at height H has travelled for H / rise-speed.
    // Bound the offset so a weak updraft cannot place a layer kilometres away.
    // No accumulated integration, random stream or wall-clock input.
    inline FVector SmokeWindOffset(float HeightUU, const FVector2D& Direction,
        float HorizontalSpeed, float RiseSpeed, float MaxDriftUU)
    {
        if (HeightUU <= 0.0f || RiseSpeed <= KINDA_SMALL_NUMBER ||
            HorizontalSpeed <= 0.0f || MaxDriftUU <= 0.0f)
        {
            return FVector::ZeroVector;
        }
        const FVector2D Drift = Direction.GetSafeNormal() *
            FMath::Min(MaxDriftUU, HeightUU / RiseSpeed * HorizontalSpeed);
        return FVector(Drift.X, Drift.Y, 0.0f);
    }
}
