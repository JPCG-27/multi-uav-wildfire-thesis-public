#include "FireVisualsManager.h"
#include "DeterministicCaptureEffects.h"
#include "FireVisualProfile.h"
#include "FirePresentationMath.h"
#include "Grid.h"
#include "ProceduralTerrain.h"
#include "ForestDresser.h"
#include "NiagaraSystem.h"
#include "NiagaraComponent.h"
#include "NiagaraEmitter.h"
#include "NiagaraParameterStore.h"
#include "NiagaraSpriteRendererProperties.h"
#include "NiagaraMeshRendererProperties.h"
#include "NiagaraLightRendererProperties.h"
#include "NiagaraSystemInstanceController.h"
#include "NiagaraGpuComputeDispatchInterface.h"
#include "NiagaraFunctionLibrary.h"
#include "Particles/ParticleSystem.h"
#include "Particles/ParticleSystemComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Components/PointLightComponent.h"
#include "Materials/MaterialInterface.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "ProceduralMeshComponent.h"
#include "UObject/UnrealType.h"
#include "RenderingThread.h"
#include "ProfilingDebugging/CsvProfiler.h"
#if WITH_EDITOR
#include "NiagaraScriptSource.h"
#include "NiagaraGraph.h"
#include "NiagaraNodeFunctionCall.h"
#include "NiagaraScript.h"
#endif

namespace
{
    bool TryGetAuthoredFloatParameter(const UNiagaraSystem* System, FName ParameterName, float& OutValue)
    {
        if (!System || ParameterName.IsNone())
        {
            return false;
        }

        const FNiagaraVariable Parameter(FNiagaraTypeDefinition::GetFloatDef(), ParameterName);
        const FNiagaraParameterStore& ExposedParameters = System->GetExposedParameters();
        if (ExposedParameters.IndexOf(Parameter) == INDEX_NONE)
        {
            return false;
        }

        OutValue = ExposedParameters.GetParameterValueOrDefault<float>(Parameter, 1.0f);
        return true;
    }

    UMaterialInterface* GetVertexColorMaterial()
    {
        static TWeakObjectPtr<UMaterialInterface> CachedMaterial;
        if (CachedMaterial.IsValid())
        {
            return CachedMaterial.Get();
        }

        UMaterialInterface* Material = LoadObject<UMaterialInterface>(
            nullptr, TEXT("/Engine/EngineDebugMaterials/VertexColorMaterial.VertexColorMaterial"));
        CachedMaterial = Material;
        return Material;
    }

    void AddTwoSidedTri(TArray<int32>& Triangles, int32 A, int32 B, int32 C)
    {
        Triangles.Append({ A, B, C, C, B, A });
    }

    void AddFlameMesh(TArray<FVector>& Vertices, TArray<int32>& Triangles, TArray<FLinearColor>& Colors)
    {
        constexpr int32 Sides = 12;
        const int32 BaseStart = Vertices.Num();
        const float BaseRadius = 210.0f;
        const float MidRadius = 95.0f;
        const float MidZ = 290.0f;
        const float TipZ = 720.0f;

        for (int32 I = 0; I < Sides; ++I)
        {
            const float Angle = 2.0f * PI * static_cast<float>(I) / static_cast<float>(Sides);
            const float C = FMath::Cos(Angle);
            const float S = FMath::Sin(Angle);
            Vertices.Add(FVector(C * BaseRadius, S * BaseRadius, 0.0f));
            Colors.Add(FLinearColor(1.0f, 0.08f, 0.01f, 1.0f));
            Vertices.Add(FVector(C * MidRadius, S * MidRadius, MidZ));
            Colors.Add(FLinearColor(1.0f, 0.42f, 0.02f, 1.0f));
        }

        const int32 Tip = Vertices.Num();
        Vertices.Add(FVector(0.0f, 0.0f, TipZ));
        Colors.Add(FLinearColor(1.0f, 0.92f, 0.18f, 1.0f));

        for (int32 I = 0; I < Sides; ++I)
        {
            const int32 Next = (I + 1) % Sides;
            const int32 Base0 = BaseStart + I * 2;
            const int32 Mid0 = Base0 + 1;
            const int32 Base1 = BaseStart + Next * 2;
            const int32 Mid1 = Base1 + 1;

            AddTwoSidedTri(Triangles, Base0, Base1, Mid0);
            AddTwoSidedTri(Triangles, Mid0, Base1, Mid1);
            AddTwoSidedTri(Triangles, Mid0, Mid1, Tip);
        }
    }

    void AddSmokePuff(TArray<FVector>& Vertices, TArray<int32>& Triangles, TArray<FLinearColor>& Colors,
                      const FVector& Center, const FVector& Radius, const FLinearColor& Color)
    {
        const int32 Start = Vertices.Num();
        Vertices.Append({
            Center + FVector( Radius.X, 0.0f,     0.0f),
            Center + FVector(-Radius.X, 0.0f,     0.0f),
            Center + FVector(0.0f,      Radius.Y, 0.0f),
            Center + FVector(0.0f,     -Radius.Y, 0.0f),
            Center + FVector(0.0f,      0.0f,     Radius.Z),
            Center + FVector(0.0f,      0.0f,    -Radius.Z)
        });
        for (int32 I = 0; I < 6; ++I)
        {
            Colors.Add(Color);
        }

        AddTwoSidedTri(Triangles, Start + 0, Start + 2, Start + 4);
        AddTwoSidedTri(Triangles, Start + 2, Start + 1, Start + 4);
        AddTwoSidedTri(Triangles, Start + 1, Start + 3, Start + 4);
        AddTwoSidedTri(Triangles, Start + 3, Start + 0, Start + 4);
        AddTwoSidedTri(Triangles, Start + 2, Start + 0, Start + 5);
        AddTwoSidedTri(Triangles, Start + 1, Start + 2, Start + 5);
        AddTwoSidedTri(Triangles, Start + 3, Start + 1, Start + 5);
        AddTwoSidedTri(Triangles, Start + 0, Start + 3, Start + 5);
    }

    void AddSmokeMesh(TArray<FVector>& Vertices, TArray<int32>& Triangles, TArray<FLinearColor>& Colors)
    {
        AddSmokePuff(Vertices, Triangles, Colors,
            FVector(-90.0f,  35.0f, 230.0f), FVector(190.0f, 150.0f, 170.0f),
            FLinearColor(0.22f, 0.22f, 0.22f, 1.0f));
        AddSmokePuff(Vertices, Triangles, Colors,
            FVector( 85.0f, -40.0f, 500.0f), FVector(260.0f, 210.0f, 220.0f),
            FLinearColor(0.32f, 0.32f, 0.32f, 1.0f));
        AddSmokePuff(Vertices, Triangles, Colors,
            FVector(-35.0f,  75.0f, 820.0f), FVector(340.0f, 280.0f, 260.0f),
            FLinearColor(0.42f, 0.42f, 0.42f, 1.0f));
    }

    FVector DeterministicSmokeLayerOffset(int32 Cell, int32 LayerIdx, float RadiusUU, float ZStepUU)
    {
        if (LayerIdx <= 0 || RadiusUU <= 0.0f)
        {
            return FVector::ZeroVector;
        }

        const float Angle = FMath::DegreesToRadians(
            static_cast<float>((Cell * 37 + LayerIdx * 113) % 360));
        const float Ring = RadiusUU * (0.45f + 0.18f * static_cast<float>(LayerIdx % 4));
        return FVector(FMath::Cos(Angle) * Ring,
                       FMath::Sin(Angle) * Ring,
                       static_cast<float>(LayerIdx) * ZStepUU);
    }

    FVector DeterministicClusterOffset(int32 Cell, int32 ClusterIdx, float RadiusUU)
    {
        if (ClusterIdx <= 0 || RadiusUU <= 0.0f)
        {
            return FVector::ZeroVector;
        }

        const float Angle = FMath::DegreesToRadians(
            static_cast<float>((Cell * 71 + ClusterIdx * 137) % 360));
        const float Ring = RadiusUU * (0.25f + 0.19f * static_cast<float>((ClusterIdx + Cell) % 4));
        return FVector(FMath::Cos(Angle) * Ring,
                       FMath::Sin(Angle) * Ring,
                       0.0f);
    }

    uint32 StableCandidateRank(int32 Cell, int32 ClusterIdx)
    {
        uint32 Value = static_cast<uint32>(Cell) * 0x9E3779B9u
                     ^ static_cast<uint32>(ClusterIdx + 1) * 0x85EBCA6Bu;
        Value ^= Value >> 16;
        Value *= 0x7FEB352Du;
        Value ^= Value >> 15;
        Value *= 0x846CA68Bu;
        return Value ^ (Value >> 16);
    }

    uint64 CandidateKey(int32 Cell, int32 ClusterIdx)
    {
        return (static_cast<uint64>(static_cast<uint32>(Cell)) << 32)
             | static_cast<uint32>(ClusterIdx);
    }

    UNiagaraSystem* TryLoadNiagara(const TCHAR* Path, const TCHAR* Label)
    {
        if (UNiagaraSystem* System = LoadObject<UNiagaraSystem>(nullptr, Path))
        {
            UE_LOG(LogTemp, Log, TEXT("Thesis fire visuals: auto-loaded %s VFX %s."), Label, Path);
            return System;
        }
        return nullptr;
    }

    UParticleSystem* TryLoadCascade(const TCHAR* Path, const TCHAR* Label)
    {
        if (UParticleSystem* System = LoadObject<UParticleSystem>(nullptr, Path))
        {
            UE_LOG(LogTemp, Log, TEXT("Thesis fire visuals: auto-loaded %s VFX %s."), Label, Path);
            return System;
        }
        return nullptr;
    }
}

AFireVisualsManager::AFireVisualsManager()
{
    PrimaryActorTick.bCanEverTick = false;   // driven by ApplyTick (server slot)

    USceneComponent* Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    SetRootComponent(Root);
}

void AFireVisualsManager::Init(const Grid* WorldGrid, const FVector& GridOriginWorld, float CellSizeUU,
                               int32 BurnDurationTicks, AProceduralTerrain* InTerrain,
                               AForestDresser* InForest)
{
    if (!WorldGrid)
    {
        return;
    }

    GridW = WorldGrid->Width;
    // Init owns presentation reconfiguration; captures reuse these compiled copies.
    DeterministicSystems.Reset();
    PreviewNiagaraStates.Reset();
    CaptureNiagaraStates.Reset();
    CaptureReuseAdvances = 0;
    CaptureReuseResets = 0;
    CaptureReuseNoOps = 0;
    GridH = WorldGrid->Height;
    GridOrigin = GridOriginWorld;
    CellSize = CellSizeUU;
    Terrain = InTerrain;
    Forest = InForest;

    ApplyVisualProfile(BurnDurationTicks);
    // Only the separate capture-hidden viewport actor opts into wall-clock
    // Niagara updates. The sensor actor keeps the profile's tick-driven path.
    if (bUseRealtimeNiagaraForPreview)
    {
        bDriveNiagaraAgeFromSimulationTick = false;
        bIncrementalNiagaraPreviewStepping = false;
    }
    ResetPools();

    IgniteEvents.Reset();
    ScorchEvents.Reset();
    SmoulderEndEvents.Reset();
    IgniteCursor = 0;
    ScorchCursor = 0;
    SmoulderEndCursor = 0;
    LastAppliedTick = -1;
    BurningCells.Reset();
    SmoulderingCells.Reset();
    PoolCells.Reset();
    PoolClusterIndices.Reset();
    PoolSmouldering.Reset();
    PreviewNiagaraStates.Reset();

    CellFuel.SetNumUninitialized(GridW * GridH);
    CellIgnitionTick.Init(-1, GridW * GridH);

    // One read-only pass over the authoritative fire plan builds a separate,
    // presentation-only lifecycle. Nothing here writes back into the grid.
    for (int32 y = 0; y < GridH; ++y)
    {
        for (int32 x = 0; x < GridW; ++x)
        {
            const int32 Cell = y * GridW + x;
            CellFuel[Cell] = WorldGrid->getFloraDensity(x, y);

            const int32 FirstTick = WorldGrid->getFirstFireTick(x, y);
            if (FirstTick >= 0)
            {
                CellIgnitionTick[Cell] = FirstTick;
                IgniteEvents.Add({ FirstTick, Cell });
                if (bUseFiniteVisualLifetime)
                {
                    const int32 ScorchTick = FirstTick + VisualFlameDurationTicks;
                    ScorchEvents.Add({ ScorchTick, Cell });
                    // This legacy deadline now schedules scorch only. Fire truth
                    // has no burn-end event; presentation must not invent one.
                }
            }
        }
    }

    auto ByTickThenCell = [](const FFireEvent& A, const FFireEvent& B)
    {
        return (A.Tick != B.Tick) ? (A.Tick < B.Tick) : (A.Cell < B.Cell);
    };
    IgniteEvents.Sort(ByTickThenCell);
    ScorchEvents.Sort(ByTickThenCell);
    SmoulderEndEvents.Sort(ByTickThenCell);

    if (!bLoadVfxOnlyForYoloCapture)
    {
        EnsureVfxReady();
    }
    else
    {
        DeactivateAllVisuals(/*bResetAssignments=*/true);
    }

    UE_LOG(LogTemp, Log,
           TEXT("Thesis fire visuals: %d ignition events; scorch delay=%d ticks, legacy smoulder=%d ticks, delayed scorch=%d; pool budget=%d modular clusters, %d clusters/cell, %d smoke layers/cluster, %d lights, core scale=%.2f (width=%.2f height=%.2f), outer scale=%.2f, smoke scale=%.2f, yolo-demand=%d, realtime-preview=%d."),
           IgniteEvents.Num(), VisualFlameDurationTicks, VisualSmoulderDurationTicks,
           bUseFiniteVisualLifetime ? 1 : 0, MaxFireVfx, FireClustersPerCell, SmokeLayerCount, MaxFireLights,
           VfxScaleMultiplier, FlameWidthScaleMultiplier, FlameHeightScaleMultiplier,
           OuterFlameScaleMultiplier, SmokeVfxScaleMultiplier,
           bLoadVfxOnlyForYoloCapture ? 1 : 0, bUseRealtimeNiagaraForPreview ? 1 : 0);
}

void AFireVisualsManager::ApplyVisualProfile(int32 BurnDurationTicks)
{
    VisualFlameDurationTicks = FMath::Max(1, BurnDurationTicks);
    VisualSmoulderDurationTicks = 30;
    bUseFiniteVisualLifetime = true;
    bEnableOuterFlameLayer = OuterFlameSystem != nullptr;
    bEnableEmberLayer = true;
    bDriveNiagaraAgeFromSimulationTick = true;
    VisualSecondsPerTick = 0.10f;
    SmoulderSmokeScale = 0.70f;
    SmoulderEmberScale = 0.35f;
    VisualWindDirection = FVector2D(1.0f, 0.0f);
    VisualWindHorizontalSpeed = 100.0f;
    VisualSmokeRiseSpeed = 45.0f;
    NiagaraSpawnRateParameter = TEXT("User.SpawnRate");
    NiagaraSpawnRateScaleParameter = TEXT("User.SpawnRateScale");
    NiagaraIntensityParameter = TEXT("User.Intensity");
    NiagaraNoiseStrengthParameter = TEXT("User.NoiseStrength");
    NiagaraWindParameter = TEXT("User.Wind");

    if (!VisualProfile)
    {
        return;
    }

    if (VisualProfile->FlameSystem)         FlameSystem = VisualProfile->FlameSystem;
    if (VisualProfile->OuterFlameSystem)    OuterFlameSystem = VisualProfile->OuterFlameSystem;
    if (VisualProfile->EmberSystem)         EmberSystem = VisualProfile->EmberSystem;
    if (VisualProfile->SmokeSystem)         SmokeSystem = VisualProfile->SmokeSystem;
    if (VisualProfile->FlameSystemCascade)  FlameSystemCascade = VisualProfile->FlameSystemCascade;
    if (VisualProfile->SmokeSystemCascade)  SmokeSystemCascade = VisualProfile->SmokeSystemCascade;

    bEnableOuterFlameLayer = VisualProfile->bEnableOuterFlameLayer;
    bEnableEmberLayer = VisualProfile->bEnableEmberLayer;
    bUseFiniteVisualLifetime = VisualProfile->bUseFiniteVisualLifetime;
    VisualFlameDurationTicks = FMath::Max(
        1,
        VisualProfile->FlameDurationOverrideTicks > 0
            ? VisualProfile->FlameDurationOverrideTicks
            : BurnDurationTicks);
    VisualSmoulderDurationTicks = FMath::Max(0, VisualProfile->SmoulderDurationTicks);
    VisualSecondsPerTick = FMath::Max(0.001f, VisualProfile->VisualSecondsPerTick);
    bDriveNiagaraAgeFromSimulationTick = VisualProfile->bDriveNiagaraAgeFromSimulationTick;
    SmoulderSmokeScale = FMath::Max(0.01f, VisualProfile->SmoulderSmokeScale);
    SmoulderEmberScale = FMath::Max(0.0f, VisualProfile->SmoulderEmberScale);
    VisualWindDirection = VisualProfile->WindDirectionWorldXY.GetSafeNormal();
    VisualWindHorizontalSpeed = FMath::Max(0.0f, VisualProfile->WindHorizontalSpeedUUPerSecond);
    VisualSmokeRiseSpeed = FMath::Max(0.0f, VisualProfile->SmokeRiseSpeedUUPerSecond);
    NiagaraSpawnRateParameter = VisualProfile->SpawnRateParameter;
    NiagaraSpawnRateScaleParameter = VisualProfile->SpawnRateScaleParameter;
    NiagaraIntensityParameter = VisualProfile->IntensityParameter;
    NiagaraNoiseStrengthParameter = VisualProfile->NoiseStrengthParameter;
    NiagaraWindParameter = VisualProfile->WindParameter;
}

FVector AFireVisualsManager::CellCenterWorld(int32 Cell) const
{
    const int32 X = Cell % GridW;
    const int32 Y = Cell / GridW;
    const float CX = X + 0.5f;
    const float CY = Y + 0.5f;
    return GroundWorldAtCellFraction(CX, CY);
}

FVector AFireVisualsManager::GroundWorldAtCellFraction(float CellX, float CellY) const
{
    const float Z = Terrain.IsValid() ? Terrain->GetHeightAtCell(CellX, CellY) : GridOrigin.Z;
    return FVector(GridOrigin.X + CellX * CellSize, GridOrigin.Y + CellY * CellSize, Z);
}

FVector AFireVisualsManager::ClusterWorldLocation(int32 Cell, int32 ClusterIdx) const
{
    const int32 X = Cell % GridW;
    const int32 Y = Cell / GridW;

    // Each connected-front cell receives one terrain-sampled primary support.
    // Secondary clusters remain anchored to individual vegetation instances.
    if (bConnectedFireFront && ClusterIdx == 0)
    {
        const FVector Offset = DeterministicClusterOffset(Cell + 739, 0, CellSize * 0.12f);
        return GroundWorldAtCellFraction(X + 0.5f + Offset.X / CellSize,
                                         Y + 0.5f + Offset.Y / CellSize);
    }

    TArray<FVector> Anchors;
    if (Forest.IsValid())
    {
        Forest->GetVegetationFireAnchors(X, Y, Anchors, FMath::Max(1, FireClustersPerCell));
    }

    FVector Base;
    if (Anchors.Num() > 0)
    {
        Base = Anchors[ClusterIdx % Anchors.Num()];
    }
    else
    {
        const FVector Center = CellCenterWorld(Cell);
        Base = Center + DeterministicClusterOffset(Cell, ClusterIdx, FireClusterRadiusUU);
    }

    const FVector AroundBase = DeterministicClusterOffset(Cell + 101, ClusterIdx + 1, FireClusterRadiusUU * 0.35f);
    const float CellX = (Base.X + AroundBase.X - GridOrigin.X) / FMath::Max(CellSize, KINDA_SMALL_NUMBER);
    const float CellY = (Base.Y + AroundBase.Y - GridOrigin.Y) / FMath::Max(CellSize, KINDA_SMALL_NUMBER);
    return GroundWorldAtCellFraction(FMath::Clamp(CellX, static_cast<float>(X), static_cast<float>(X + 1)),
                                     FMath::Clamp(CellY, static_cast<float>(Y), static_cast<float>(Y + 1)));
}

void AFireVisualsManager::EnsureDefaultVfxAssets()
{
    if (!bAutoLoadDefaultVfxAssets)
    {
        return;
    }

    if (!FlameSystem && !FlameSystemCascade)
    {
        const TCHAR* NiagaraCoreFlamePaths[] = {
            TEXT("/Game/Vefects/Free_Fire/Shared/Particles/NS_Fire_Simple.NS_Fire_Simple"),
            TEXT("/Game/Vefects/Free_Fire/Shared/Particles/NS_Fire_Small_Simple.NS_Fire_Small_Simple"),
            TEXT("/Game/Vefects/Free_Fire/Shared/Particles/NS_Fire_Small.NS_Fire_Small"),
            TEXT("/Game/Vefects/Free_Fire/Shared/Particles/NS_Fire_Medium.NS_Fire_Medium"),
            TEXT("/Game/NiagaraExamples/FX_Misc/NS_Fire.NS_Fire"),
            TEXT("/NiagaraFluids/Templates/Gas/2D/Systems/Grid2D_Gas_SmokeFire.Grid2D_Gas_SmokeFire"),
            TEXT("/NiagaraFluids/Templates/Gas/3D/Systems/Grid3D_Gas_Fire.Grid3D_Gas_Fire"),
        };
        for (const TCHAR* Path : NiagaraCoreFlamePaths)
        {
            if ((FlameSystem = TryLoadNiagara(Path, TEXT("core flame"))) != nullptr)
            {
                break;
            }
        }
        if (!FlameSystem)
        {
            const TCHAR* CascadeFlamePaths[] = {
                TEXT("/Game/StarterContent/Particles/P_Fire.P_Fire"),
            };
            for (const TCHAR* Path : CascadeFlamePaths)
            {
                if ((FlameSystemCascade = TryLoadCascade(Path, TEXT("core flame"))) != nullptr)
                {
                    break;
                }
            }
        }
    }

    if (bEnableOuterFlameLayer && !OuterFlameSystem)
    {
        const TCHAR* NiagaraOuterFlamePaths[] = {
            TEXT("/Game/Vefects/Free_Fire/Shared/Particles/NS_Fire_Floor_02.NS_Fire_Floor_02"),
            TEXT("/Game/Vefects/Free_Fire/Shared/Particles/NS_Fire_Floor_01.NS_Fire_Floor_01"),
            TEXT("/Game/Vefects/Free_Fire/Shared/Particles/NS_Fire_Medium.NS_Fire_Medium"),
            TEXT("/Game/Vefects/Free_Fire/Shared/Particles/NS_Fire_Big_Simple.NS_Fire_Big_Simple"),
        };
        for (const TCHAR* Path : NiagaraOuterFlamePaths)
        {
            if ((OuterFlameSystem = TryLoadNiagara(Path, TEXT("outer flame"))) != nullptr)
            {
                break;
            }
        }
    }

    if (bEnableEmberLayer && !EmberSystem)
    {
        const TCHAR* NiagaraEmberPaths[] = {
            TEXT("/Game/NiagaraExamples/FX_Sparks/NS_Spark_Continuous.NS_Spark_Continuous"),
            TEXT("/Game/NiagaraExamples/FX_Sparks/NS_Spark_Impact_Looping.NS_Spark_Impact_Looping"),
            TEXT("/Game/NiagaraExamples/FX_Footstep/NS_Footstep_Fire.NS_Footstep_Fire"),
            TEXT("/Game/NiagaraExamples/FX_NDC/NS_NDC_Footsteps_Fire.NS_NDC_Footsteps_Fire"),
        };
        for (const TCHAR* Path : NiagaraEmberPaths)
        {
            if ((EmberSystem = TryLoadNiagara(Path, TEXT("ember"))) != nullptr)
            {
                break;
            }
        }
    }

    if (!SmokeSystem && !SmokeSystemCascade)
    {
        const TCHAR* NiagaraSmokePaths[] = {
            TEXT("/Game/NiagaraExamples/FX_Smoke/NS_Smoke_Plume.NS_Smoke_Plume"),
            TEXT("/Game/NiagaraExamples/FX_Smoke/NS_Chimney_Smoke.NS_Chimney_Smoke"),
            TEXT("/Game/Vefects/Free_Fire/Shared/Particles/NS_Fire_Big_Smoke.NS_Fire_Big_Smoke"),
            TEXT("/Game/Vefects/Free_Fire/Shared/Particles/NS_Fire_Medium_Smoke.NS_Fire_Medium_Smoke"),
            TEXT("/Game/Vefects/Free_Fire/Shared/Particles/NS_Fire_Floor_02_Smoke.NS_Fire_Floor_02_Smoke"),
            TEXT("/Game/Vefects/Free_Fire/Shared/Particles/NS_Fire_Floor_01_Smoke.NS_Fire_Floor_01_Smoke"),
            TEXT("/NiagaraFluids/Templates/Gas/2D/Systems/Grid2D_Gas_Smoke.Grid2D_Gas_Smoke"),
            TEXT("/NiagaraFluids/Templates/Gas/3D/Systems/Grid3D_Gas_Smoke.Grid3D_Gas_Smoke"),
        };
        for (const TCHAR* Path : NiagaraSmokePaths)
        {
            if ((SmokeSystem = TryLoadNiagara(Path, TEXT("smoke"))) != nullptr)
            {
                break;
            }
        }
        if (!SmokeSystem)
        {
            const TCHAR* CascadeSmokePaths[] = {
                TEXT("/Game/StarterContent/Particles/P_Smoke.P_Smoke"),
            };
            for (const TCHAR* Path : CascadeSmokePaths)
            {
                if ((SmokeSystemCascade = TryLoadCascade(Path, TEXT("smoke"))) != nullptr)
                {
                    break;
                }
            }
        }
    }

    if (!FlameSystem && !FlameSystemCascade && !SmokeSystem && !SmokeSystemCascade)
    {
        if (bUseProceduralFallbackVfx)
        {
            UE_LOG(LogTemp, Log,
                   TEXT("Thesis fire visuals: no Niagara/Cascade fire/smoke VFX found; using procedural fallback meshes."));
        }
        else
        {
            UE_LOG(LogTemp, Warning,
                   TEXT("Thesis fire visuals: no default fire/smoke VFX found. Enable NiagaraFluids or assign FlameSystem/SmokeSystem on the FireVisualsManager."));
        }
    }
}

UNiagaraSystem* AFireVisualsManager::GetDeterministicSystem(UNiagaraSystem* Source)
{
    if (!Source) return nullptr;
    if (const TObjectPtr<UNiagaraSystem>* Existing = DeterministicSystems.Find(Source)) return Existing->Get();

    const bool bUseClusterLocalSpace = Source->GetPathName().Contains(TEXT("/Game/Vefects/Free_Fire/"))
        && Source->GetName().StartsWith(TEXT("NS_Fire_"))
        && !Source->GetName().Contains(TEXT("Smoke"));
    const bool bUseClusterLocalSpaceForSystem = bUseClusterLocalSpace;

#if !WITH_EDITOR
    if (bUseClusterLocalSpace && (bConnectedFireFront
        || !FMath::IsNearlyEqual(BundledFlameSpriteSizeMultiplier, 1.0f)))
    {
        UE_LOG(LogTemp, Fatal, TEXT("Thesis fire visuals: bake flame sprite dimensions into %s before cooking, set BundledFlameSpriteSizeMultiplier to 1 and disable bConnectedFireFront. Runtime sprite-size adaptation requires editor Niagara compilation."), *Source->GetPathName());
        return nullptr;
    }
    // Emitter.Determinism is a compile-time constant, not just a runtime seed
    // setting. A cooked executable cannot repair nondeterministic bytecode.
    bool bCookedDeterministic = Source->NeedsDeterminism();
    for (const FNiagaraEmitterHandle& Handle : Source->GetEmitterHandles())
    {
        if (const FVersionedNiagaraEmitterData* Data = Handle.GetEmitterData())
        {
            bCookedDeterministic &= Data->bDeterminism
                && Data->SimTarget == ENiagaraSimTarget::CPUSim
                && (!bUseClusterLocalSpaceForSystem || Data->bLocalSpace);
        }
    }
    if (!bCookedDeterministic)
    {
        UE_LOG(LogTemp, Fatal, TEXT("Thesis fire visuals: cooked Niagara asset %s is incompatible with tick-driven fire rendering. Prepare CPU emitters with determinism enabled and collision modules disabled; bundled fire emitters also require LocalSpace enabled. Recook the asset."), *Source->GetPathName());
        return nullptr;
    }
#endif

    UNiagaraSystem* Runtime = DuplicateObject<UNiagaraSystem>(Source, this);
    if (!Runtime) return nullptr;
    Runtime->SetFlags(RF_Transient);
    Runtime->ClearFlags(RF_Public | RF_Standalone);
    // These system settings are protected UPROPERTYs, with getters but no
    // runtime setters. Change only our transient copy through UE reflection.
    FBoolProperty* Determinism = FindFProperty<FBoolProperty>(UNiagaraSystem::StaticClass(), TEXT("bDeterminism"));
    if (!Determinism)
    {
        UE_LOG(LogTemp, Error, TEXT("Thesis fire visuals: Niagara determinism property unavailable for %s."), *Source->GetName());
        return nullptr;
    }
    Determinism->SetPropertyValue_InContainer(Runtime, true);
    if (FIntProperty* Seed = FindFProperty<FIntProperty>(UNiagaraSystem::StaticClass(), TEXT("RandomSeed")))
        Seed->SetPropertyValue_InContainer(Runtime, 0);
    if (FBoolProperty* FixedDelta = FindFProperty<FBoolProperty>(UNiagaraSystem::StaticClass(), TEXT("bFixedTickDelta")))
        FixedDelta->SetPropertyValue_InContainer(Runtime, false);
    Runtime->SetWarmupTime(0.0f); // The bounded, explicit warmup below owns all steps.

    for (FNiagaraEmitterHandle& Handle : Runtime->GetEmitterHandles())
    {
        if (bUseClusterLocalSpace && (Handle.GetName() == TEXT("NE_Lights")
            || (!bEnableBundledHeatDistortion && Handle.GetName() == TEXT("NE_Heat"))))
        {
            // Pooled lights already own illumination; do not simulate invisible
            // particle lights. Distortion is unnecessary in the overview view.
            Handle.SetIsEnabled(false, *Runtime, /*bRecompileIfChanged=*/false);
        }
        FVersionedNiagaraEmitter Emitter = Handle.GetInstance();
        if (!Emitter.Emitter) continue;
        // Normally DuplicateObject copied owned emitters as well. Explicitly
        // duplicate externally-owned instances before changing any properties.
        if (!Emitter.Emitter->IsIn(Runtime))
        {
            Emitter.Emitter = DuplicateObject<UNiagaraEmitter>(Emitter.Emitter, Runtime);
            Handle.SetInstance(Emitter);
        }
        if (FVersionedNiagaraEmitterData* Data = Handle.GetEmitterData())
        {
            Data->bDeterminism = true;
            Data->RandomSeed = 0;
            if (bUseClusterLocalSpaceForSystem)
            {
                const bool bWasLocalSpace = Data->bLocalSpace;
                // Fire emitters stay anchored to a fixed cell/cluster during
                // their deterministic lifetime. Keep particles in that local
                // frame so the per-cluster non-uniform transform controls the
                // visible footprint without spawning more systems.
                Data->bLocalSpace = true;
                UE_LOG(LogTemp, Display,
                    TEXT("Thesis flame emitter %s/%s: LocalSpace %d -> 1 for cluster-scaled presentation."),
                    *Source->GetName(), *Handle.GetName().ToString(), bWasLocalSpace ? 1 : 0);
            }
#if WITH_EDITOR
            // GPU particle allocation/collision history varied between identical
            // captures. Compile only our private presentation copy for CPU use;
            // grid and belief-field computation stays on the GPU.
            Data->SimTarget = ENiagaraSimTarget::CPUSim;
            if (bUseClusterLocalSpace && Handle.GetName().ToString().Contains(TEXT("Flame")))
            {
                // At the production 300 UU cell size, scale quads enough to
                // overlap adjacent primary supports. Cap the derived factor so
                // larger imported grid cells cannot create unbounded sprites.
                const float ConnectedFootprintScale = bConnectedFireFront
                    ? FMath::Clamp(2.5f * FMath::Max(CellSize, 0.0f) / 300.0f, 1.0f, 3.0f)
                    : 1.0f;
                // Adjust spawn dimensions, not the evolving particle buffer or
                // update curve: each particle's lifetime scaling remains authored.
                TArray<UNiagaraScript*> Scripts;
                Data->GetScripts(Scripts);
                TSet<UNiagaraScript*> Visited;
                for (UNiagaraScript* Script : Scripts)
                {
                    if (!Script || Visited.Contains(Script)) continue;
                    Visited.Add(Script);
                    if (!Script->IsIn(Runtime))
                    {
                        UE_LOG(LogTemp, Fatal, TEXT("Thesis fire visuals: refusing to resize non-private Niagara script %s."), *Script->GetPathName());
                        return nullptr;
                    }
                    TArray<FNiagaraVariable> Parameters;
                    Script->RapidIterationParameters.GetParameters(Parameters);
                    for (const FNiagaraVariable& Parameter : Parameters)
                    {
                        const FString Name = Parameter.GetName().ToString();
                        if (!Name.Contains(TEXT(".InitializeParticle.")) || !Name.Contains(TEXT("Sprite Size"))) continue;
                        const float Factor = FMath::Clamp(BundledFlameSpriteSizeMultiplier, 0.1f, 10.0f)
                            * ConnectedFootprintScale;
                        if (Parameter.GetType() == FNiagaraTypeDefinition::GetFloatDef())
                        {
                            const float Size = Script->RapidIterationParameters.GetParameterValue<float>(Parameter);
                            Script->RapidIterationParameters.SetParameterValue(Size * Factor, Parameter);
                            UE_LOG(LogTemp, Display, TEXT("Thesis flame sprite %s: %.2f -> %.2f"), *Name, Size, Size * Factor);
                        }
                        else if (Parameter.GetType() == FNiagaraTypeDefinition::GetVec2Def())
                        {
                            const FVector2f Size = Script->RapidIterationParameters.GetParameterValue<FVector2f>(Parameter);
                            Script->RapidIterationParameters.SetParameterValue(Size * Factor, Parameter);
                            UE_LOG(LogTemp, Display, TEXT("Thesis flame sprite %s: (%.2f, %.2f) -> (%.2f, %.2f)"), *Name, Size.X, Size.Y, Size.X * Factor, Size.Y * Factor);
                        }
                    }
                }
            }
            if (UNiagaraScriptSource* ScriptSource = Cast<UNiagaraScriptSource>(Data->GraphSource))
            {
                check(ScriptSource->IsIn(Runtime));
                if (ScriptSource->NodeGraph)
                {
                    for (UEdGraphNode* Node : ScriptSource->NodeGraph->Nodes)
                    {
                        UNiagaraNodeFunctionCall* Call = Cast<UNiagaraNodeFunctionCall>(Node);
                        if (Call && Call->FunctionScript && Call->FunctionScript->GetPathName() == TEXT("/Niagara/Modules/Collision/Collision.Collision"))
                        {
                            Call->SetEnabledState(ENodeEnabledState::Disabled, true);
                            UE_LOG(LogTemp, Display, TEXT("Sensor fire: disabled view-dependent collision in %s/%s"),
                                *Source->GetName(), *Handle.GetName().ToString());
                        }
                    }
                }
            }
#endif
            for (UNiagaraRendererProperties* Renderer : Data->GetRenderers())
            {
                if (UNiagaraLightRendererProperties* Light = Cast<UNiagaraLightRendererProperties>(Renderer))
                {
                    // Simple particle lights introduce a second, unstable light
                    // list. The tick-driven PointLight pool supplies fire light.
                    Light->SetIsEnabled(false);
                }
                if (UNiagaraSpriteRendererProperties* Sprite = Cast<UNiagaraSpriteRendererProperties>(Renderer))
                {
                    // Particle allocation order is not a blending order.
                    // Preserve authored sorts, but sort unsorted sprites by depth.
                    if (Sprite->SortMode == ENiagaraSortMode::None)
                        Sprite->SortMode = ENiagaraSortMode::ViewDepth;
                    Sprite->SortPrecision = ENiagaraRendererSortPrecision::High;
                }
            }
        }
    }
#if WITH_EDITOR
    // Changing bDeterminism through reflection does not send an editor property
    // event. Explicit forcing dirties the private system/emitter graphs and
    // rebuilds the Emitter.Determinism static branches in CPU/GPU bytecode.
    // Waiting alone only finishes jobs that happened to be already queued.
    if (!Runtime->RequestCompile(/*bForce=*/true) && !Runtime->HasActiveCompilations())
    {
        UE_LOG(LogTemp, Fatal, TEXT("Thesis fire visuals: could not compile deterministic Niagara copy for %s. Shader compilation must be available for this editor capture run."), *Source->GetPathName());
        return nullptr;
    }
    // Asset/derived shader readiness may block, but must not select which
    // simulation tick first renders a working particle system.
    Runtime->WaitForCompilationComplete(/*bIncludingGPUShaders=*/true, /*bShowProgress=*/false);
#endif
    if (!Runtime->IsReadyToRun())
    {
        UE_LOG(LogTemp, Fatal, TEXT("Thesis fire visuals: deterministic capture copy of %s failed to compile or initialize."), *Source->GetPathName());
        return nullptr;
    }

    // Asset flame emission was authored for a much lower exposure than the
    // outdoor scene. Adapt only the known flame materials, never the shared
    // originals or the smoke. Do this after compilation's MIC generation.
    auto DaylightFlameMaterial = [&](UMaterialInterface* Material) -> UMaterialInterface*
    {
        if (!Material || !Material->GetPathName().Contains(TEXT("/Vefects/Free_Fire/"))
            || !Material->GetName().StartsWith(TEXT("MI_VFX_Fire"))) return Material;
        float Emissive = 0;
        const FMaterialParameterInfo Parameter(TEXT("Emissive_Intensity"));
        if (!Material->GetScalarParameterValue(Parameter, Emissive) || Emissive <= 0) return Material;
        UMaterialInstanceDynamic* Adapted = UMaterialInstanceDynamic::Create(Material, Runtime);
        // Several overlapping tongues contribute at each pixel in front mode.
        // Reduce individual emission so overlap retains orange detail.
        const float EmissionScale = BundledFlameEmissiveMultiplier * (bConnectedFireFront ? 0.5f : 1.0f);
        Adapted->SetScalarParameterValue(TEXT("Emissive_Intensity"), Emissive * EmissionScale);
        UE_LOG(LogTemp, Display, TEXT("Thesis flame material %s: emission %.3f -> %.3f"),
            *Material->GetName(), Emissive, Emissive * EmissionScale);
        return Adapted;
    };
    for (FNiagaraEmitterHandle& Handle : Runtime->GetEmitterHandles())
    {
        if (FVersionedNiagaraEmitterData* Data = Handle.GetEmitterData())
        {
            for (UNiagaraRendererProperties* Renderer : Data->GetRenderers())
            {
                if (UNiagaraSpriteRendererProperties* Sprite = Cast<UNiagaraSpriteRendererProperties>(Renderer))
                {
                    UMaterialInterface* Adapted = DaylightFlameMaterial(Sprite->Material);
                    if (Adapted != Sprite->Material)
                    {
                        Sprite->Material = Adapted;
#if WITH_EDITORONLY_DATA
                        Sprite->MICMaterial = nullptr;
#endif
                    }
                }
                else if (UNiagaraMeshRendererProperties* Mesh = Cast<UNiagaraMeshRendererProperties>(Renderer))
                {
                    for (auto& Override : Mesh->OverrideMaterials)
                        Override.ExplicitMat = DaylightFlameMaterial(Override.ExplicitMat);
                }
            }
        }
    }

    float AuthoredSpawnRate = 1.0f;
    if (TryGetAuthoredFloatParameter(Source, NiagaraSpawnRateParameter, AuthoredSpawnRate))
    {
        UE_LOG(LogTemp, Display,
            TEXT("Thesis fire visuals: %s exposes %s with authored rate %.3f particles/sec."),
            *Source->GetPathName(), *NiagaraSpawnRateParameter.ToString(), AuthoredSpawnRate);
    }
    else
    {
        UE_LOG(LogTemp, Display,
            TEXT("Thesis fire visuals: %s has no exposed %s; spawn-rate scaling will try %s."),
            *Source->GetPathName(), *NiagaraSpawnRateParameter.ToString(),
            *NiagaraSpawnRateScaleParameter.ToString());
    }
    DeterministicSystems.Add(Source, Runtime);
    return Runtime;
}

UNiagaraSystem* AFireVisualsManager::GetRealtimePreviewSystem(UNiagaraSystem* Source)
{
    // The existing capture copy owns all fire appearance adapters (cluster
    // local-space, sprite footprint, materials, sorting, and light removal).
    // Each AFireVisualsManager has its own transient cache, so this preview
    // copy is private to the hidden-from-capture actor. Its NiagaraComponents
    // still use ordinary world ticking because bDriveNiagaraAgeFromSimulationTick
    // was disabled for this actor after the profile was applied.
    return GetDeterministicSystem(Source);
}

void AFireVisualsManager::HideVfxComponent(USceneComponent* Component)
{
    if (!Component) return;
    Component->SetVisibility(false);
    if (UNiagaraComponent* Niagara = Cast<UNiagaraComponent>(Component))
    {
        if (bUseRealtimeNiagaraForPreview)
        {
            // Hidden slots must not continue wall-clock particle simulation.
            Niagara->Deactivate();
            Niagara->SetComponentTickEnabled(false);
        }
        else if (bDriveNiagaraAgeFromSimulationTick)
        {
            // Preserve particles and simulated age between captures. Deactivate /
            // Activate can reset a Niagara instance even when its pool key survived.
            Component->SetComponentTickEnabled(false);
        }
        PreviewNiagaraStates.Remove(Niagara);
        return;
    }
    Component->Deactivate();
}

void AFireVisualsManager::AdvanceNiagaraVisual(UNiagaraComponent* Component, int32 Tick,
    int32 IgnitionTick, int32 Cell, int32 ClusterIndex, int32 LayerIndex, bool bForceReconstruction,
    float EffectiveEmission, float EffectIntensity, float EffectiveNoiseStrength,
    const FVector& EffectiveWind)
{
    CSV_SCOPED_TIMING_STAT_EXCLUSIVE(FirePreview_ManualNiagara);
    if (!bCanonicalVfxReconstruction || !bReuseDeterministicCaptureState)
    {
        // Any ordinary presentation tick mutates this component's Niagara
        // history, so it cannot remain eligible for a later capture hit.
        CaptureNiagaraStates.Remove(Component);
    }
    // Presentation loops after eight seconds, with one second of warmup. This
    // bounds first-use/reassignment cost independently of scenario duration.
    // It changes particle phase only: ignition, burnout and smoke lifecycle
    // still come from the original deterministic fire events above.
    constexpr float CanonicalStepSeconds = 1.0f / 30.0f;
    constexpr int32 CanonicalPeriodSteps = 240;
    constexpr int32 CanonicalWarmupSteps = 30;
    constexpr float FastStepSeconds = 1.0f / 15.0f;
    constexpr int32 FastPeriodSteps = 60;
    constexpr int32 FastWarmupSteps = 15;
    const bool bUseFastCanonicalCapture = bCanonicalVfxReconstruction && bFastDeterministicCapture;
    const float StepSeconds = bUseFastCanonicalCapture ? FastStepSeconds : CanonicalStepSeconds;
    const int32 StepsPerSecond = bUseFastCanonicalCapture ? 15 : 30;
    const int32 PeriodSteps = bUseFastCanonicalCapture ? FastPeriodSteps : CanonicalPeriodSteps;
    const int32 WarmupSteps = bUseFastCanonicalCapture ? FastWarmupSteps : CanonicalWarmupSteps;
    const int64 AgeTicks = FMath::Max<int64>(0, static_cast<int64>(Tick) - IgnitionTick);
    const int64 AgeSteps = static_cast<int64>(static_cast<double>(AgeTicks) * VisualSecondsPerTick * StepsPerSecond);
    const uint32 Seed = (StableCandidateRank(Cell, ClusterIndex)
        ^ (static_cast<uint32>(LayerIndex + 1) * 0x9e3779b9u)) & 0x3fffffffu;
    const FTransform ComponentTransform = Component->GetComponentTransform();
    const int32 TargetSteps = WarmupSteps + static_cast<int32>(AgeSteps % PeriodSteps);
    const bool bUseCaptureStateReuse = bReuseDeterministicCaptureState && bUseFastCanonicalCapture;
    const FCaptureNiagaraState* CaptureState = bUseCaptureStateReuse
        ? CaptureNiagaraStates.Find(Component) : nullptr;
    const bool bCaptureStateCompatible = CaptureState && CaptureState->bValid
        && CaptureState->Asset.Get() == Component->GetAsset()
        && CaptureState->ComponentTransform.GetTranslation() == ComponentTransform.GetTranslation()
        && CaptureState->ComponentTransform.GetRotation() == ComponentTransform.GetRotation()
        && CaptureState->ComponentTransform.GetScale3D() == ComponentTransform.GetScale3D()
        && CaptureState->Wind == EffectiveWind
        && CaptureState->Emission == EffectiveEmission
        && CaptureState->Intensity == EffectIntensity
        && CaptureState->NoiseStrength == EffectiveNoiseStrength
        && CaptureState->Seed == Seed && CaptureState->Cell == Cell
        && CaptureState->ClusterIndex == ClusterIndex && CaptureState->LayerIndex == LayerIndex
        && CaptureState->IgnitionTick == IgnitionTick
        && Component->IsActive() && Component->IsPaused() && !Component->IsComponentTickEnabled();

    FPreviewNiagaraState* State = PreviewNiagaraStates.Find(Component);
    const bool bIncremental = bIncrementalNiagaraPreviewStepping && !bCanonicalVfxReconstruction;
    const bool bCompatibleState = State && State->bValid && State->Seed == Seed
        && State->Cell == Cell && State->ClusterIndex == ClusterIndex
        && State->LayerIndex == LayerIndex && State->IgnitionTick == IgnitionTick;
    const FirePresentation::FNiagaraStepPlan Plan = bUseCaptureStateReuse
        ? FirePresentation::PlanNiagaraSteps(
            true, bCaptureStateCompatible, bForceReconstruction, Tick,
            CaptureState ? CaptureState->LastTick : INDEX_NONE, AgeSteps,
            CaptureState ? CaptureState->LastAgeSteps : -1,
            CaptureState ? CaptureState->TargetSteps : INDEX_NONE, PeriodSteps, WarmupSteps)
        : FirePresentation::PlanNiagaraSteps(
            bIncremental, bCompatibleState, bForceReconstruction, Tick,
            State ? State->LastTick : INDEX_NONE, AgeSteps,
            State ? State->LastAgeSteps : -1,
            State ? State->LastTargetSteps : INDEX_NONE, PeriodSteps, WarmupSteps);

    if (bUseCaptureStateReuse)
    {
        if (Plan.bReconstruct) ++CaptureReuseResets;
        else if (Plan.StepsToAdvance > 0) ++CaptureReuseAdvances;
        else ++CaptureReuseNoOps;
    }

    if (Plan.bReconstruct)
    {
        // Activate before asking for the controller: a newly registered
        // Niagara component may not own a system instance until activation.
        Component->SetRandomSeedOffset(static_cast<int32>(Seed));
        Component->Activate(/*bReset=*/true);
        // Solo prevents shared world-manager simulation. Activation can
        // re-enable component ticking, so disable it after every reset.
        Component->SetComponentTickEnabled(false);
    }

    if (Plan.StepsToAdvance > 0)
    {
        Component->SetPaused(false);
    }
    const auto Controller = Component->GetSystemInstanceController();
    if (Controller.IsValid())
    {
        if (Plan.StepsToAdvance > 0)
        {
            // Fixed-count ManualTick is deterministic and has no wall-time age
            // budget. Preview and compatible v2 captures advance only elapsed steps.
            Component->AdvanceSimulation(Plan.StepsToAdvance, StepSeconds);
            Controller->WaitForConcurrentTickAndFinalize();
        }
        // Niagara's world manager can advance active non-solo instances even
        // when their component tick is disabled. Keep deterministic sensor
        // instances frozen between explicit simulation-tick advances.
        Component->SetPaused(true);
        FPreviewNiagaraState& NewState = PreviewNiagaraStates.FindOrAdd(Component);
        NewState.Seed = Seed;
        NewState.LastTick = Tick;
        NewState.LastAgeSteps = AgeSteps;
        NewState.LastTargetSteps = Plan.TargetSteps;
        NewState.Cell = Cell;
        NewState.ClusterIndex = ClusterIndex;
        NewState.LayerIndex = LayerIndex;
        NewState.IgnitionTick = IgnitionTick;
        NewState.bValid = true;
        if (bUseCaptureStateReuse)
        {
            FCaptureNiagaraState& NewCaptureState = CaptureNiagaraStates.FindOrAdd(Component);
            NewCaptureState.Asset = Component->GetAsset();
            NewCaptureState.ComponentTransform = ComponentTransform;
            NewCaptureState.Wind = EffectiveWind;
            NewCaptureState.Emission = EffectiveEmission;
            NewCaptureState.Intensity = EffectIntensity;
            NewCaptureState.NoiseStrength = EffectiveNoiseStrength;
            NewCaptureState.Seed = Seed;
            NewCaptureState.LastTick = Tick;
            NewCaptureState.LastAgeSteps = AgeSteps;
            NewCaptureState.TargetSteps = TargetSteps;
            NewCaptureState.Cell = Cell;
            NewCaptureState.ClusterIndex = ClusterIndex;
            NewCaptureState.LayerIndex = LayerIndex;
            NewCaptureState.IgnitionTick = IgnitionTick;
            NewCaptureState.bValid = true;
        }
    }
    else
    {
        UE_LOG(LogTemp, Fatal, TEXT("Thesis fire visuals: capture particle system %s did not initialize after activation."), *Component->GetName());
    }
}

USceneComponent* AFireVisualsManager::MakeVfxComponent(UNiagaraSystem* Niagara, UParticleSystem* Cascade, bool bFlameFallback)
{
    USceneComponent* C = nullptr;
    if (Niagara && bUseRealtimeNiagaraForPreview) Niagara = GetRealtimePreviewSystem(Niagara);
    else if (Niagara && bDriveNiagaraAgeFromSimulationTick) Niagara = GetDeterministicSystem(Niagara);
    if (Niagara)
    {
        UNiagaraComponent* N = NewObject<UNiagaraComponent>(this);
        N->SetAsset(Niagara);
        N->SetAutoActivate(false);
        if (bDriveNiagaraAgeFromSimulationTick)
        {
            N->SetForceSolo(true);
            N->SetAllowScalability(false);
            N->SetAgeUpdateMode(ENiagaraAgeUpdateMode::TickDeltaTime);
        }
        C = N;
    }
    else if (Cascade && !bDriveNiagaraAgeFromSimulationTick)
    {
        UParticleSystemComponent* P = NewObject<UParticleSystemComponent>(this);
        P->SetTemplate(Cascade);
        P->SetAutoActivate(false);
        C = P;
    }
    else if (bUseProceduralFallbackVfx)
    {
        C = MakeProceduralFallbackComponent(bFlameFallback);
    }

    if (C)
    {
        C->RegisterComponent();
        C->AttachToComponent(GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
        if (UPrimitiveComponent* Prim = Cast<UPrimitiveComponent>(C))
        {
            Prim->SetHiddenInSceneCapture(bHideFromSceneCaptures);
            Prim->SetVisibleInSceneCaptureOnly(bVfxOnlyInDroneCaptures);
            if (bUseRealtimeNiagaraForPreview)
            {
                // Sensor and reflection views must not see or receive hidden
                // shadows from wall-clock presentation particles.
                Prim->bVisibleInReflectionCaptures = false;
                Prim->bVisibleInRealTimeSkyCaptures = false;
                Prim->bVisibleInRayTracing = false;
                Prim->SetCastHiddenShadow(false);
                Prim->SetAffectDynamicIndirectLighting(false);
                Prim->SetAffectIndirectLightingWhileHidden(false);
                Prim->MarkRenderStateDirty();
            }
        }
        C->SetVisibility(false);
        if (bDriveNiagaraAgeFromSimulationTick) C->SetComponentTickEnabled(false);
    }
    return C;
}

USceneComponent* AFireVisualsManager::MakeProceduralFallbackComponent(bool bFlame)
{
    UProceduralMeshComponent* Mesh = NewObject<UProceduralMeshComponent>(this);
    if (!Mesh)
    {
        return nullptr;
    }

    Mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Mesh->SetCanEverAffectNavigation(false);
    Mesh->bUseAsyncCooking = false;

    TArray<FVector> Vertices;
    TArray<int32> Triangles;
    TArray<FLinearColor> Colors;

    if (bFlame)
    {
        AddFlameMesh(Vertices, Triangles, Colors);
    }
    else
    {
        AddSmokeMesh(Vertices, Triangles, Colors);
    }

    TArray<FVector> Normals;
    TArray<FVector2D> UVs;
    Normals.Init(FVector::UpVector, Vertices.Num());
    UVs.Init(FVector2D::ZeroVector, Vertices.Num());

    Mesh->CreateMeshSection_LinearColor(
        0, Vertices, Triangles, Normals, UVs, Colors, TArray<FProcMeshTangent>(),
        /*bCreateCollision=*/false);

    if (UMaterialInterface* Material = GetVertexColorMaterial())
    {
        Mesh->SetMaterial(0, Material);
    }

    return Mesh;
}

void AFireVisualsManager::ResetPools()
{
    PreviewNiagaraStates.Reset();
    auto DestroyPool = [](auto& Pool)
    {
        for (auto& Component : Pool)
        {
            if (Component)
            {
                Component->DestroyComponent();
            }
        }
        Pool.Reset();
    };

    DestroyPool(FlamePool);
    DestroyPool(OuterFlamePool);
    DestroyPool(EmberPool);
    DestroyPool(SmokePool);
    DestroyPool(LightPool);
    DestroyPool(FastShaderPool);
    PoolCells.Reset();
    PoolClusterIndices.Reset();
    PoolSmouldering.Reset();
}

void AFireVisualsManager::EnsurePools()
{
    const int32 VfxCount = FMath::Max(0, MaxFireVfx);
    if (!bUseFastShaderV3)
    {
        while (FlamePool.Num() < VfxCount)
        {
            FlamePool.Add(MakeVfxComponent(FlameSystem.Get(), FlameSystemCascade.Get(), /*bFlameFallback=*/true));
        }

        const int32 OuterCount = bEnableOuterFlameLayer ? VfxCount : 0;
        while (OuterFlamePool.Num() > OuterCount)
        {
            if (USceneComponent* Component = OuterFlamePool.Pop()) Component->DestroyComponent();
        }
        UNiagaraSystem* OuterSystem = OuterFlameSystem ? OuterFlameSystem.Get() : FlameSystem.Get();
        while (OuterFlamePool.Num() < OuterCount)
        {
            OuterFlamePool.Add(MakeVfxComponent(OuterSystem, FlameSystemCascade.Get(), /*bFlameFallback=*/true));
        }

        const int32 EmberCount = (bEnableEmberLayer && EmberSystem) ? VfxCount : 0;
        while (EmberPool.Num() > EmberCount)
        {
            if (USceneComponent* Component = EmberPool.Pop()) Component->DestroyComponent();
        }
        while (EmberPool.Num() < EmberCount)
        {
            EmberPool.Add(MakeVfxComponent(EmberSystem.Get(), nullptr, /*bFlameFallback=*/true));
        }

        const int32 SmokeCount = VfxCount * FMath::Max(1, SmokeLayerCount);
        while (SmokePool.Num() < SmokeCount)
        {
            SmokePool.Add(MakeVfxComponent(SmokeSystem.Get(), SmokeSystemCascade.Get(), /*bFlameFallback=*/false));
        }
    }

    const int32 LightCount = bFireLights ? FMath::Max(0, MaxFireLights) : 0;
    while (LightPool.Num() < LightCount)
    {
        UPointLightComponent* L = NewObject<UPointLightComponent>(this);
        L->RegisterComponent();
        L->AttachToComponent(GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
        L->SetMobility(EComponentMobility::Movable);
        L->SetLightColor(FLinearColor(1.0f, 0.42f, 0.10f), true);
        L->SetAttenuationRadius(LightRadiusUU);
        L->SetSourceRadius(LightSourceRadiusUU);
        L->SetSoftSourceRadius(LightSourceRadiusUU * 0.5f);
        L->SetVolumetricScatteringIntensity(LightVolumetricScatteringIntensity);
        L->SetIntensity(0.0f);
        L->SetCastShadows(LightPool.Num() < MaxShadowCastingFireLights);
        LightPool.Add(L);
    }

    const int32 SlotCount = FMath::Max(VfxCount, LightCount);
    const int32 OldSlotCount = PoolCells.Num();
    PoolCells.SetNum(SlotCount);
    PoolClusterIndices.SetNum(SlotCount);
    PoolSmouldering.SetNum(SlotCount);
    for (int32 SlotIdx = OldSlotCount; SlotIdx < SlotCount; ++SlotIdx)
    {
        PoolCells[SlotIdx] = INDEX_NONE;
        PoolClusterIndices[SlotIdx] = INDEX_NONE;
        PoolSmouldering[SlotIdx] = 0;
    }
}

void AFireVisualsManager::EnsureFastShaderPool(int32 ActiveSlotCount)
{
    const int32 Needed = FMath::Clamp(ActiveSlotCount, 0, FMath::Max(0, MaxFireVfx));
    while (FastShaderPool.Num() < Needed)
    {
        UDeterministicCaptureEffectsComponent* Effect = NewObject<UDeterministicCaptureEffectsComponent>(this);
        if (!Effect)
        {
            break;
        }
        Effect->RegisterComponent();
        Effect->SetVisibleInSceneCaptureOnly(bVfxOnlyInDroneCaptures);
        Effect->Deactivate();
        FastShaderPool.Add(Effect);
    }
}

void AFireVisualsManager::EnsureVfxReady()
{
    if (bUseFastShaderV3)
    {
        EnsurePools();
        return;
    }
    EnsureDefaultVfxAssets();
    EnsurePools();
}

void AFireVisualsManager::DeactivateAllVisuals(bool bResetAssignments)
{
    auto DeactivatePool = [this](const TArray<TObjectPtr<USceneComponent>>& Pool)
    {
        for (const TObjectPtr<USceneComponent>& C : Pool)
        {
            if (!C)
            {
                continue;
            }
            HideVfxComponent(C.Get());
        }
    };

    DeactivatePool(FlamePool);
    DeactivatePool(OuterFlamePool);
    DeactivatePool(EmberPool);
    DeactivatePool(SmokePool);
    for (const TObjectPtr<UDeterministicCaptureEffectsComponent>& Effect : FastShaderPool)
    {
        if (Effect)
        {
            Effect->Deactivate();
        }
    }

    for (const TObjectPtr<UPointLightComponent>& L : LightPool)
    {
        if (!L)
        {
            continue;
        }
        L->SetIntensity(0.0f);
        L->SetVisibility(false);
    }

    if (bResetAssignments)
    {
        PoolCells.Init(INDEX_NONE, PoolCells.Num());
        PoolClusterIndices.Init(INDEX_NONE, PoolClusterIndices.Num());
        PoolSmouldering.Init(0, PoolSmouldering.Num());
        CaptureNiagaraStates.Reset();
    }
}

void AFireVisualsManager::SetVfxCaptureOnly(bool bCaptureOnly)
{
    bVfxOnlyInDroneCaptures = bCaptureOnly;
    auto Apply = [bCaptureOnly](const TArray<TObjectPtr<USceneComponent>>& Pool)
    {
        for (const TObjectPtr<USceneComponent>& C : Pool)
        {
            if (UPrimitiveComponent* Prim = Cast<UPrimitiveComponent>(C.Get()))
            {
                Prim->SetVisibleInSceneCaptureOnly(bCaptureOnly);
            }
        }
    };
    Apply(FlamePool);
    Apply(OuterFlamePool);
    Apply(EmberPool);
    Apply(SmokePool);
    for (const TObjectPtr<UDeterministicCaptureEffectsComponent>& Effect : FastShaderPool)
    {
        if (Effect) Effect->SetVisibleInSceneCaptureOnly(bCaptureOnly);
    }
}

void AFireVisualsManager::ApplyTick(int32 Tick)
{
    const bool bRealtimePreviewRewind = bUseRealtimeNiagaraForPreview && Tick < LastAppliedTick;
    if (bRealtimePreviewRewind)
    {
        // A replay rewind starts new live particle instances instead of
        // retaining wall-clock history from the previous presentation pass.
        DeactivateAllVisuals(/*bResetAssignments=*/true);
    }
    AdvanceFireState(Tick);
    if (bLoadVfxOnlyForYoloCapture
        && (bReuseDeterministicCaptureState || bUseFastShaderV3))
    {
        // Capture-only profiles keep visual components parked between sensor
        // captures. Fire/scorch bookkeeping still advances.
        return;
    }
    EnsureVfxReady();
    if (bRealtimePreviewRewind || (Tick % FMath::Max(1, ReassignEveryNTicks)) == 0)
    {
        ReassignPool(Tick);
    }
    UpdateFlicker(Tick);
}

void AFireVisualsManager::AdvanceFireState(int32 Tick)
{
    if (GridW <= 0)
    {
        return;
    }

    // Replays support both forward progress and a reset to tick 0.
    if (Tick < LastAppliedTick)
    {
        IgniteCursor = 0;
        ScorchCursor = 0;
        SmoulderEndCursor = 0;
        BurningCells.Reset();
        SmoulderingCells.Reset();
        if (bScorchedGroundFeedback)
        {
            if (Terrain.IsValid()) Terrain->ResetScorch();
            if (Forest.IsValid())  Forest->ResetScorch();
        }
    }
    LastAppliedTick = Tick;

    while (IgniteCursor < IgniteEvents.Num() && IgniteEvents[IgniteCursor].Tick <= Tick)
    {
        const int32 Cell = IgniteEvents[IgniteCursor].Cell;
        BurningCells.Add(Cell);
        SmoulderingCells.Remove(Cell);
        if (!bUseFiniteVisualLifetime && bScorchedGroundFeedback)
        {
            // Compatibility mode has no visual burnout event, so retain the
            // former ignition-time scorch behaviour.
            ScorchCellVisuals(Cell);
        }
        ++IgniteCursor;
    }

    while (ScorchCursor < ScorchEvents.Num() && ScorchEvents[ScorchCursor].Tick <= Tick)
    {
        const int32 Cell = ScorchEvents[ScorchCursor].Cell;
        // IsFireActive remains true after ignition in the current simulator.
        // Retain flames (including for old profiles with finite lifetime enabled)
        // and use this deadline only for the existing delayed scorch feedback.
        if (bScorchedGroundFeedback)
        {
            ScorchCellVisuals(Cell);
        }
        ++ScorchCursor;
    }

    while (SmoulderEndCursor < SmoulderEndEvents.Num()
           && SmoulderEndEvents[SmoulderEndCursor].Tick <= Tick)
    {
        SmoulderingCells.Remove(SmoulderEndEvents[SmoulderEndCursor].Cell);
        ++SmoulderEndCursor;
    }
    // Batched terrain scorch upload (full vertex-buffer rewrite — modest cadence).
    if (bScorchedGroundFeedback && Terrain.IsValid()
        && (Tick % FMath::Max(1, ScorchFlushEveryNTicks)) == 0)
    {
        Terrain->FlushScorch();
    }
}

void AFireVisualsManager::PrepareForYoloCapture(int32 Tick, const FVector& CaptureWorldLocation, float CaptureRadiusUU)
{
    bCanonicalVfxReconstruction = true;
    CaptureReuseAdvances = 0;
    CaptureReuseResets = 0;
    CaptureReuseNoOps = 0;
    PreviewNiagaraStates.Reset();
    AdvanceFireState(Tick);
    EnsureVfxReady();
    ReassignPool(Tick, &CaptureWorldLocation, CaptureRadiusUU);
    UpdateFlicker(Tick);
    if (bDriveNiagaraAgeFromSimulationTick || bUseFastShaderV3)
    {
        // Visibility/intensity changes recreate light proxies. Publish them in
        // pool order instead of the world's pointer-keyed end-of-frame set,
        // keeping scene light indices (and shadow sampling) reproducible.
        for (const auto& Light : LightPool)
            if (Light && Light->IsRegistered()) Light->DoDeferredRenderUpdates_Concurrent();

        // Equal-distance translucency otherwise falls back to renderer IDs,
        // which depend on scene-proxy allocation. Preserve pool order for
        // ties and give each component an explicit back-to-front priority.
        TArray<UPrimitiveComponent*> SortedVisuals;
        auto AddPool = [&SortedVisuals](const TArray<TObjectPtr<USceneComponent>>& Pool)
        {
            for (const auto& Component : Pool)
                if (UPrimitiveComponent* Primitive = Cast<UPrimitiveComponent>(Component.Get()))
                    SortedVisuals.Add(Primitive);
        };
        AddPool(FlamePool);
        AddPool(OuterFlamePool);
        AddPool(EmberPool);
        AddPool(SmokePool);
        for (int32 EffectSlot = 0; EffectSlot < FastShaderPool.Num(); ++EffectSlot)
        {
            if (FastShaderPool[EffectSlot] && PoolCells.IsValidIndex(EffectSlot)
                && PoolCells[EffectSlot] != INDEX_NONE)
            {
                FastShaderPool[EffectSlot]->GetEffectPrimitives(SortedVisuals);
            }
        }
        SortedVisuals.StableSort([&CaptureWorldLocation](const UPrimitiveComponent& A, const UPrimitiveComponent& B)
        {
            return FVector::DistSquared(A.GetComponentLocation(), CaptureWorldLocation)
                > FVector::DistSquared(B.GetComponentLocation(), CaptureWorldLocation);
        });
        for (int32 Index = 0; Index < SortedVisuals.Num(); ++Index)
            SortedVisuals[Index]->SetTranslucentSortPriority(Index);

        // Publish finalized simulation/proxy updates before the sensor render.
        // The dispatcher flush has no real camera depth, so capture effects
        // must not depend on scene-depth collision or wall-clock inputs.
        if (!bUseFastShaderV3)
        {
            if (FNiagaraGpuComputeDispatchInterface* Dispatch = FNiagaraGpuComputeDispatchInterface::Get(GetWorld()))
            {
                Dispatch->FlushPendingTicks_GameThread();
            }
        }
        // Publish the ordered active-pool update once before the sensor render.
        FlushRenderingCommands();
    }
    if (FParse::Param(FCommandLine::Get(), TEXT("ThesisCameraReuseStats")))
    {
        int32 ActiveEffectSlots = 0;
        const int32 EffectSlotBudget = FMath::Min(PoolCells.Num(), FMath::Max(0, MaxFireVfx));
        for (int32 SlotIdx = 0; SlotIdx < FMath::Min(EffectSlotBudget, FastShaderPool.Num()); ++SlotIdx)
        {
            if (PoolCells.IsValidIndex(SlotIdx) && PoolCells[SlotIdx] != INDEX_NONE)
                ++ActiveEffectSlots;
        }
        UE_LOG(LogTemp, Log,
               TEXT("[ThesisCameraReuseStats] tick=%d continued_components=%d reset_components=%d unchanged=%d fast_shader_v3=%d allocated_effects=%d active_effects=%d slot_budget=%d"),
               Tick, CaptureReuseAdvances, CaptureReuseResets, CaptureReuseNoOps,
               bUseFastShaderV3 ? 1 : 0, bUseFastShaderV3 ? FastShaderPool.Num() : 0,
               bUseFastShaderV3 ? ActiveEffectSlots : 0,
               bUseFastShaderV3 ? EffectSlotBudget : 0);
    }
}

void AFireVisualsManager::EndYoloCapture()
{
    if (bLoadVfxOnlyForYoloCapture)
    {
        // Retain logical assignments. The next capture can reactivate the same
        // slots at their tick-derived age instead of reshuffling the fire front.
        DeactivateAllVisuals(/*bResetAssignments=*/false);
        if (bDriveNiagaraAgeFromSimulationTick)
            for (const auto& Light : LightPool)
                if (Light && Light->IsRegistered()) Light->DoDeferredRenderUpdates_Concurrent();
    }
    bCanonicalVfxReconstruction = false;
    PreviewNiagaraStates.Reset();
    // The next preview tick starts from its deterministic seed after a capture
    // has modified component assignment and particle history.
}

void AFireVisualsManager::ReassignPool(int32 Tick, const FVector* FocusWorld, float FocusRadiusUU)
{
    CSV_SCOPED_TIMING_STAT_EXCLUSIVE(FirePreview_ReassignPool);
    struct FClusterCandidate
    {
        int32 Cell = INDEX_NONE;
        int32 ClusterIdx = 0;
        bool bSmouldering = false;
    };

    TArray<int32> SortedFlames = BurningCells.Array();
    TArray<int32> SortedSmoulder = SmoulderingCells.Array();
    if (FocusWorld && FocusRadiusUU > 0.0f)
    {
        const float RadiusSq = FMath::Square(FocusRadiusUU);
        const auto InFocus = [&](int32 Cell)
        {
            return FVector::DistSquared2D(CellCenterWorld(Cell), *FocusWorld) <= RadiusSq;
        };
        SortedFlames = SortedFlames.FilterByPredicate(InFocus);
        SortedSmoulder = SortedSmoulder.FilterByPredicate(InFocus);
    }
    // Canonicalize only the candidates that survived the unchanged camera
    // radius test. Sorting after filtering preserves the same in-focus order.
    SortedFlames.Sort();
    SortedSmoulder.Sort();

    const int32 Slots = PoolCells.Num();
    const int32 EffectSlotBudget = bUseFastShaderV3 ? FMath::Min(Slots, FMath::Max(0, MaxFireVfx)) : 0;
    const int32 ClustersPerCell = FMath::Max(1, FireClustersPerCell);
    TArray<FClusterCandidate> Candidates;
    Candidates.Reserve((SortedFlames.Num() + SortedSmoulder.Num()) * ClustersPerCell);
    for (int32 ClusterIdx = 0; ClusterIdx < ClustersPerCell; ++ClusterIdx)
    {
        for (const int32 Cell : SortedFlames)
        {
            Candidates.Add({ Cell, ClusterIdx, false });
        }
        for (const int32 Cell : SortedSmoulder)
        {
            Candidates.Add({ Cell, ClusterIdx, true });
        }
    }

    TMap<int32, int32> ConnectedRanks;
    if (bConnectedFireFront)
    {
        const TArray<int32> Order = FirePresentation::ConnectedCellOrder(
            SortedFlames, GridW, GridH, CellIgnitionTick);
        for (int32 Rank = 0; Rank < Order.Num(); ++Rank) ConnectedRanks.Add(Order[Rank], Rank);
    }
    // Primary supports fill connected patches first; detail keeps its stable
    // hash order. Above budget, each ignition region retains a connected prefix.
    Candidates.Sort([this, &ConnectedRanks](const FClusterCandidate& A, const FClusterCandidate& B)
    {
        if (A.bSmouldering != B.bSmouldering)
        {
            return !A.bSmouldering;
        }
        if (bConnectedFireFront && A.ClusterIdx != B.ClusterIdx)
        {
            // Give every burning cell its primary support before spending the
            // bounded pool on secondary campfire detail clusters.
            return A.ClusterIdx < B.ClusterIdx;
        }
        if (bConnectedFireFront && A.ClusterIdx == 0 && !A.bSmouldering)
        {
            return ConnectedRanks.FindChecked(A.Cell) < ConnectedRanks.FindChecked(B.Cell);
        }
        const uint32 RankA = StableCandidateRank(A.Cell, A.ClusterIdx);
        const uint32 RankB = StableCandidateRank(B.Cell, B.ClusterIdx);
        if (RankA != RankB) return RankA < RankB;
        if (A.Cell != B.Cell) return A.Cell < B.Cell;
        return A.ClusterIdx < B.ClusterIdx;
    });
    if (Candidates.Num() > Slots)
    {
        Candidates.SetNum(Slots);
    }

    TSet<uint64> EligibleKeys;
    for (const FClusterCandidate& Candidate : Candidates)
    {
        EligibleKeys.Add(CandidateKey(Candidate.Cell, Candidate.ClusterIdx));
    }

    TArray<FClusterCandidate> Chosen;
    Chosen.SetNum(Slots);
    TSet<uint64> UsedKeys;

    // Keep a candidate in its existing component slot whenever possible.
    for (int32 SlotIdx = 0; SlotIdx < Slots; ++SlotIdx)
    {
        const int32 OldCell = PoolCells.IsValidIndex(SlotIdx) ? PoolCells[SlotIdx] : INDEX_NONE;
        const int32 OldCluster = PoolClusterIndices.IsValidIndex(SlotIdx)
            ? PoolClusterIndices[SlotIdx]
            : INDEX_NONE;
        if (OldCell == INDEX_NONE || OldCluster == INDEX_NONE)
        {
            continue;
        }

        const uint64 Key = CandidateKey(OldCell, OldCluster);
        if (EligibleKeys.Contains(Key) && !UsedKeys.Contains(Key))
        {
            Chosen[SlotIdx] = { OldCell, OldCluster, SmoulderingCells.Contains(OldCell) };
            UsedKeys.Add(Key);
        }
    }

    int32 CandidateCursor = 0;
    for (int32 SlotIdx = 0; SlotIdx < Slots; ++SlotIdx)
    {
        if (Chosen[SlotIdx].Cell != INDEX_NONE)
        {
            continue;
        }
        while (CandidateCursor < Candidates.Num()
               && UsedKeys.Contains(CandidateKey(Candidates[CandidateCursor].Cell,
                                                 Candidates[CandidateCursor].ClusterIdx)))
        {
            ++CandidateCursor;
        }
        if (CandidateCursor < Candidates.Num())
        {
            Chosen[SlotIdx] = Candidates[CandidateCursor++];
            UsedKeys.Add(CandidateKey(Chosen[SlotIdx].Cell, Chosen[SlotIdx].ClusterIdx));
        }
    }

    // Smoke is shared by all flame clusters belonging to a cell. Pick the
    // lowest assigned cluster for ownership, but place the shared plume over
    // the mean burning-cluster position instead of one potentially distant tree.
    // Three-metre burning cells should feed a common plume, not stack a full
    // plume every three metres. Group neighbouring cells for smoke only.
    const int32 SmokeTileSide = bConnectedFireFront
        ? FMath::Clamp(FMath::FloorToInt(600.0f / FMath::Max(CellSize, 1.0f)), 1, 8) : 1;
    const auto SmokeGroup = [&](int32 Cell)
    {
        return ((Cell / GridW) / SmokeTileSide) * ((GridW + SmokeTileSide - 1) / SmokeTileSide)
            + (Cell % GridW) / SmokeTileSide;
    };
    TMap<int32, uint64> SmokeOwnerKeyByGroup;
    const int32 SmokeVisualSlotCount = bUseFastShaderV3 ? EffectSlotBudget : FlamePool.Num();
    struct FSmokeAnchor { FVector Sum = FVector::ZeroVector; int32 Count = 0; };
    TMap<int32, FSmokeAnchor> SmokeAnchors;
    TArray<FVector> ClusterPositions;
    ClusterPositions.Init(FVector::ZeroVector, Chosen.Num());
    for (int32 SlotIdx = 0; SlotIdx < Chosen.Num(); ++SlotIdx)
        if (Chosen[SlotIdx].Cell != INDEX_NONE)
            ClusterPositions[SlotIdx] = ClusterWorldLocation(Chosen[SlotIdx].Cell, Chosen[SlotIdx].ClusterIdx);
    if (bShareSmokeAcrossCellClusters)
    {
        for (int32 SlotIdx = 0; SlotIdx < Chosen.Num(); ++SlotIdx)
        {
            const FClusterCandidate& Candidate = Chosen[SlotIdx];
            if (Candidate.Cell == INDEX_NONE || SlotIdx >= SmokeVisualSlotCount)
            {
                continue;
            }
            const int32 Group = SmokeGroup(Candidate.Cell);
            const uint64 Key = CandidateKey(Candidate.Cell, Candidate.ClusterIdx);
            uint64* ExistingOwner = SmokeOwnerKeyByGroup.Find(Group);
            if (!ExistingOwner || Key < *ExistingOwner)
            {
                SmokeOwnerKeyByGroup.Add(Group, Key);
            }
        }
        // Cell/cluster order, never retained pool-slot order, determines sums.
        TArray<int32> SmokeAnchorSlots;
        for (int32 SlotIdx = 0; SlotIdx < FMath::Min(Chosen.Num(), SmokeVisualSlotCount); ++SlotIdx)
            if (Chosen[SlotIdx].Cell != INDEX_NONE) SmokeAnchorSlots.Add(SlotIdx);
        SmokeAnchorSlots.Sort([&](int32 A, int32 B)
        {
            return CandidateKey(Chosen[A].Cell, Chosen[A].ClusterIdx)
                 < CandidateKey(Chosen[B].Cell, Chosen[B].ClusterIdx);
        });
        for (int32 SlotIdx : SmokeAnchorSlots)
        {
            FSmokeAnchor& Anchor = SmokeAnchors.FindOrAdd(SmokeGroup(Chosen[SlotIdx].Cell));
            Anchor.Sum += ClusterPositions[SlotIdx];
            ++Anchor.Count;
        }
    }

    if (bUseFastShaderV3)
    {
        int32 RequiredPoolSize = 0;
        for (int32 SlotIdx = 0; SlotIdx < FMath::Min(EffectSlotBudget, Chosen.Num()); ++SlotIdx)
        {
            if (Chosen[SlotIdx].Cell != INDEX_NONE)
            {
                RequiredPoolSize = SlotIdx + 1;
            }
        }
        EnsureFastShaderPool(RequiredPoolSize);
    }

    TArray<int32> SlotsToUpdate;
    if (bUseFastShaderV3)
    {
        for (int32 SlotIdx = 0; SlotIdx < PoolCells.Num(); ++SlotIdx)
        {
            const bool bWasActive = PoolCells[SlotIdx] != INDEX_NONE;
            const bool bWillBeActive = Chosen.IsValidIndex(SlotIdx) && Chosen[SlotIdx].Cell != INDEX_NONE;
            if (bWasActive || bWillBeActive)
            {
                SlotsToUpdate.Add(SlotIdx);
            }
        }
    }
    else
    {
        SlotsToUpdate.Reserve(PoolCells.Num());
        for (int32 SlotIdx = 0; SlotIdx < PoolCells.Num(); ++SlotIdx)
        {
            SlotsToUpdate.Add(SlotIdx);
        }
    }

    for (const int32 SlotIdx : SlotsToUpdate)
    {
        const int32 Cell = Chosen.IsValidIndex(SlotIdx) ? Chosen[SlotIdx].Cell : INDEX_NONE;
        const int32 ClusterIdx = Chosen.IsValidIndex(SlotIdx) ? Chosen[SlotIdx].ClusterIdx : INDEX_NONE;
        const bool bSmouldering = Chosen.IsValidIndex(SlotIdx) && Chosen[SlotIdx].bSmouldering;
        const bool bAssignmentChanged = PoolCells[SlotIdx] != Cell
            || !PoolClusterIndices.IsValidIndex(SlotIdx)
            || PoolClusterIndices[SlotIdx] != ClusterIdx;
        const bool bPhaseChanged = PoolSmouldering.IsValidIndex(SlotIdx)
            && (PoolSmouldering[SlotIdx] != static_cast<uint8>(bSmouldering));
        PoolCells[SlotIdx] = Cell;
        if (PoolClusterIndices.IsValidIndex(SlotIdx))
        {
            PoolClusterIndices[SlotIdx] = ClusterIdx;
        }
        if (PoolSmouldering.IsValidIndex(SlotIdx))
        {
            PoolSmouldering[SlotIdx] = bSmouldering ? 1 : 0;
        }

        const bool bActive = (Cell != INDEX_NONE);
        const bool bFlameActive = bActive && !bSmouldering;
        const FVector ClusterBase = ClusterPositions[SlotIdx];
        const FSmokeAnchor* SmokeAnchor = bActive ? SmokeAnchors.Find(SmokeGroup(Cell)) : nullptr;
        const FVector SmokeBase = SmokeAnchor && SmokeAnchor->Count > 0
            ? SmokeAnchor->Sum / SmokeAnchor->Count : ClusterBase;
        const float FuelScale = (bActive && bScaleWithFuel && CellFuel.IsValidIndex(Cell))
            ? FMath::Lerp(0.5f, 1.4f, FMath::Clamp(CellFuel[Cell], 0.0f, 1.0f))
            : 1.0f;
        const float ClusterFuelScale = FuelScale * (bActive
            ? FirePresentation::ClusterScaleVariation(Cell, ClusterIdx)
            : 1.0f);
        const float FlameVfxScale = ClusterFuelScale * FMath::Max(0.01f, VfxScaleMultiplier);
        const float OuterVfxScale = ClusterFuelScale * FMath::Max(0.01f, OuterFlameScaleMultiplier);
        const float EmberPhaseScale = bSmouldering ? SmoulderEmberScale : 1.0f;
        const float SmokePhaseScale = bSmouldering ? SmoulderSmokeScale : 1.0f;
        const float EmberVfxScale = ClusterFuelScale * FMath::Max(0.01f, EmberScaleMultiplier) * EmberPhaseScale;
        const float SmokeVfxScale = ClusterFuelScale * FMath::Max(0.01f, SmokeVfxScaleMultiplier) * SmokePhaseScale;
        const int32 IgnitionTick = (bActive && CellIgnitionTick.IsValidIndex(Cell))
            ? CellIgnitionTick[Cell]
            : Tick;
        const FVector2D WindXY = VisualWindDirection * VisualWindHorizontalSpeed;

        const FName SmokeLayerName(TEXT("Smoke"));
        auto ApplyToVfx = [&](USceneComponent* C, bool bLayerActive, const FVector& Location,
                              float Scale, float YawDegrees, FName LayerName, int32 LayerIndex,
                              float LayerIntensityScale, float LayerNoiseStrength,
                              float WidthScale, float HeightScale)
        {
            if (!C) return;
            C->SetVisibility(bLayerActive);
            if (bLayerActive)
            {
                C->SetWorldLocation(Location);
                C->SetWorldRotation(FRotator(0.0f, YawDegrees, 0.0f));
                C->SetWorldScale3D(FVector(Scale * WidthScale, Scale * WidthScale, Scale * HeightScale));
                float CaptureEmission = -1.0f;
                float CaptureIntensity = -1.0f;
                float CaptureNoiseStrength = -1.0f;
                FVector CaptureWind = FVector::ZeroVector;
                if (UNiagaraComponent* N = Cast<UNiagaraComponent>(C))
                {
                    const bool bSmokeLayer = (LayerName == SmokeLayerName);
                    const FName EffectiveSpawnRateParameter = NiagaraSpawnRateParameter;
                    const float PhaseIntensity = bSmouldering ? 0.35f : 1.0f;
                    const float EffectIntensity = ClusterFuelScale * PhaseIntensity * LayerIntensityScale;
                    float AuthoredSpawnRate = 1.0f;
                    float EffectiveEmissionValue = -1.0f;
                    const bool bHasAuthoredSpawnRate = TryGetAuthoredFloatParameter(
                        N->GetAsset(), EffectiveSpawnRateParameter, AuthoredSpawnRate);
                    if (bHasAuthoredSpawnRate)
                    {
                        // SpawnRate is particles/second, so scale the system's
                        // authored emission rate instead of replacing it with ~1.
                        EffectiveEmissionValue = AuthoredSpawnRate * EffectIntensity;
                        N->SetVariableFloat(EffectiveSpawnRateParameter, EffectiveEmissionValue);
                    }
                    else
                    {
                        float AuthoredSpawnRateScale = 1.0f;
                        if (TryGetAuthoredFloatParameter(N->GetAsset(), NiagaraSpawnRateScaleParameter,
                                                         AuthoredSpawnRateScale))
                        {
                            EffectiveEmissionValue = AuthoredSpawnRateScale * EffectIntensity;
                            N->SetVariableFloat(NiagaraSpawnRateScaleParameter, EffectiveEmissionValue);
                        }
                    }

                    float AuthoredIntensity = 1.0f;
                    if (TryGetAuthoredFloatParameter(N->GetAsset(), NiagaraIntensityParameter, AuthoredIntensity))
                    {
                        CaptureIntensity = AuthoredIntensity * EffectIntensity;
                        N->SetVariableFloat(NiagaraIntensityParameter, CaptureIntensity);
                    }
                    CaptureEmission = EffectiveEmissionValue;

                    if (bSmokeLayer && !NiagaraNoiseStrengthParameter.IsNone())
                    {
                        float AuthoredNoiseStrength = 1.0f;
                        if (TryGetAuthoredFloatParameter(N->GetAsset(), NiagaraNoiseStrengthParameter,
                                                         AuthoredNoiseStrength))
                        {
                            CaptureNoiseStrength = AuthoredNoiseStrength * LayerNoiseStrength;
                            N->SetVariableFloat(NiagaraNoiseStrengthParameter,
                                CaptureNoiseStrength);
                        }
                    }
                    if (!NiagaraWindParameter.IsNone())
                    {
                        CaptureWind = FVector(WindXY.X, WindXY.Y,
                            bSmokeLayer ? VisualSmokeRiseSpeed : VisualSmokeRiseSpeed * 0.33f);
                        N->SetVariableVec3(NiagaraWindParameter, CaptureWind);
                    }
                }
                if (UNiagaraComponent* N = Cast<UNiagaraComponent>(C); N && bDriveNiagaraAgeFromSimulationTick)
                {
                    AdvanceNiagaraVisual(N, Tick, IgnitionTick, Cell, ClusterIdx, LayerIndex,
                                         bAssignmentChanged || bPhaseChanged, CaptureEmission,
                                         CaptureIntensity, CaptureNoiseStrength, CaptureWind);
                }
                else
                {
                    const bool bNeedsReset = bAssignmentChanged || bPhaseChanged;
                    if (bUseRealtimeNiagaraForPreview && Cast<UNiagaraComponent>(C))
                    {
                        // Hidden live-preview pool slots are disabled to avoid
                        // simulating unused systems; enable them when reused.
                        C->SetComponentTickEnabled(true);
                    }
                    if (bNeedsReset || !C->IsActive())
                    {
                        C->Activate(/*bReset=*/bNeedsReset);
                    }
                }
            }
            else
            {
                HideVfxComponent(C);
            }
        };
        if (FlamePool.IsValidIndex(SlotIdx))
        {
            ApplyToVfx(FlamePool[SlotIdx], bFlameActive,
                       ClusterBase + FVector(0.0f, 0.0f, FlameZOffsetUU),
                       FlameVfxScale,
                       static_cast<float>((Cell * 11 + ClusterIdx * 53) % 360),
                       TEXT("Core"), 0, 1.0f, 1.0f,
                       FlameWidthScaleMultiplier, FlameHeightScaleMultiplier);
        }

        if (OuterFlamePool.IsValidIndex(SlotIdx))
        {
            ApplyToVfx(OuterFlamePool[SlotIdx], bFlameActive && bEnableOuterFlameLayer,
                       ClusterBase + DeterministicClusterOffset(Cell + 17, ClusterIdx + 3, FireClusterRadiusUU * 0.18f)
                           + FVector(0.0f, 0.0f, FlameZOffsetUU + 8.0f),
                       OuterVfxScale,
                       static_cast<float>((Cell * 19 + ClusterIdx * 71) % 360),
                       TEXT("Outer"), 1, 1.0f, 1.0f,
                       FlameWidthScaleMultiplier, FlameHeightScaleMultiplier);
        }

        if (EmberPool.IsValidIndex(SlotIdx))
        {
            ApplyToVfx(EmberPool[SlotIdx], bActive && bEnableEmberLayer && EmberPhaseScale > 0.0f,
                       ClusterBase + FVector(0.0f, 0.0f, FlameZOffsetUU + 35.0f),
                       EmberVfxScale,
                       static_cast<float>((Cell * 23 + ClusterIdx * 97) % 360),
                       TEXT("Ember"), 2, 1.0f, 1.0f, 1.0f, 1.0f);
        }

        const int32 SmokeLayers = FMath::Max(1, SmokeLayerCount);
        const uint64* SmokeOwnerKey = bActive ? SmokeOwnerKeyByGroup.Find(SmokeGroup(Cell)) : nullptr;
        const bool bSmokeOwner = bActive && (!bShareSmokeAcrossCellClusters
            || (SmokeOwnerKey && *SmokeOwnerKey == CandidateKey(Cell, ClusterIdx)));

        if (bUseFastShaderV3 && SlotIdx < EffectSlotBudget && FastShaderPool.IsValidIndex(SlotIdx))
        {
            UDeterministicCaptureEffectsComponent* Effect = FastShaderPool[SlotIdx];
            if (Effect)
            {
                const bool bEffectActive = bFlameActive || bSmokeOwner;
                if (bEffectActive)
                {
                    const float EffectIntensity = ClusterFuelScale * (bSmouldering ? 0.35f : 1.0f);
                    const float EffectFlameScale = FlameVfxScale * (bSmouldering ? 0.0f : 1.0f);
                    const float EffectSmokeScale = SmokeVfxScale * (bSmouldering ? 0.70f : 1.0f);
                    const bool bNeedsReset = bAssignmentChanged || bPhaseChanged;
                    if (bNeedsReset)
                    {
                        Effect->Initialize(
                            ClusterBase + FVector(0.0f, 0.0f, FlameZOffsetUU),
                            SmokeBase + FVector(0.0f, 0.0f, SmokeZOffsetUU),
                            Cell, static_cast<int32>(StableCandidateRank(Cell, ClusterIdx)), ClusterFuelScale,
                            IgnitionTick, Tick, VisualWindDirection, VisualWindHorizontalSpeed,
                            EffectFlameScale, EffectSmokeScale, EffectIntensity, SmokeLayers,
                            bFlameActive, bSmokeOwner);
                    }
                    else
                    {
                        Effect->UpdateAtTick(
                            ClusterBase + FVector(0.0f, 0.0f, FlameZOffsetUU),
                            SmokeBase + FVector(0.0f, 0.0f, SmokeZOffsetUU),
                            Tick, VisualWindDirection, VisualWindHorizontalSpeed,
                            EffectFlameScale, EffectSmokeScale, EffectIntensity,
                            SmokeLayers, bFlameActive, bSmokeOwner);
                    }
                }
                else
                {
                    Effect->Deactivate();
                }
            }
        }

        for (int32 LayerIdx = 0; LayerIdx < SmokeLayers; ++LayerIdx)
        {
            const int32 SmokeIdx = SlotIdx * SmokeLayers + LayerIdx;
            if (!SmokePool.IsValidIndex(SmokeIdx))
            {
                continue;
            }

            FVector LayerOffset = bSmokeOwner
                ? DeterministicSmokeLayerOffset(Cell + ClusterIdx * 97, LayerIdx, SmokeLayerRadiusUU, SmokeLayerZStepUU)
                : FVector::ZeroVector;
            if (bSmokeOwner)
            {
                LayerOffset += FirePresentation::SmokeWindOffset(
                    FMath::Max(0.0f, LayerOffset.Z), VisualWindDirection,
                    VisualWindHorizontalSpeed, VisualSmokeRiseSpeed, SmokeMaxWindDriftUU);
            }
            const float LayerYaw = static_cast<float>((Cell * 29 + LayerIdx * 67 + ClusterIdx * 13) % 360);
            const FirePresentation::FSmokeLayerProfile LayerProfile =
                FirePresentation::SmokeLayerProfile(LayerIdx, SmokeLayers);
            const float LayerScale = SmokeVfxScale * LayerProfile.Scale;
            ApplyToVfx(SmokePool[SmokeIdx], bSmokeOwner,
                       SmokeBase + FVector(0.0f, 0.0f, SmokeZOffsetUU) + LayerOffset,
                       LayerScale,
                       LayerYaw,
                       TEXT("Smoke"), 3 + LayerIdx,
                       LayerProfile.Intensity, LayerProfile.NoiseStrength, 1.0f, 1.0f);
        }

        if (LightPool.IsValidIndex(SlotIdx) && LightPool[SlotIdx])
        {
            UPointLightComponent* L = LightPool[SlotIdx];
            L->SetVisibility(bActive);
            L->SetLightColor(FLinearColor(1.0f, 0.42f, 0.10f), true);
            L->SetAttenuationRadius(LightRadiusUU);
            L->SetSourceRadius(LightSourceRadiusUU);
            L->SetSoftSourceRadius(LightSourceRadiusUU * 0.5f);
            L->SetVolumetricScatteringIntensity(LightVolumetricScatteringIntensity);
            L->SetCastShadows(SlotIdx < MaxShadowCastingFireLights);
            if (bActive)
            {
                L->SetWorldLocation(ClusterBase + FVector(0, 0, LightZOffsetUU));
                L->SetIntensity(LightIntensity * ClusterFuelScale * (bSmouldering ? 0.08f : 1.0f));
            }
            else
            {
                L->SetIntensity(0.0f);
            }
        }
    }

}

void AFireVisualsManager::UpdateFlicker(int32 Tick)
{
    // Deterministic flicker: pure function of (tick, cell) — no wall clock.
    for (int32 SlotIdx = 0; SlotIdx < LightPool.Num(); ++SlotIdx)
    {
        UPointLightComponent* L = LightPool.IsValidIndex(SlotIdx) ? LightPool[SlotIdx].Get() : nullptr;
        const int32 Cell = PoolCells.IsValidIndex(SlotIdx) ? PoolCells[SlotIdx] : INDEX_NONE;
        if (!L || Cell == INDEX_NONE)
        {
            continue;
        }
        const int32 ClusterIdx = PoolClusterIndices.IsValidIndex(SlotIdx) ? PoolClusterIndices[SlotIdx] : 0;
        const float Phase = static_cast<float>(((Cell + ClusterIdx * 23) % 17)) * 0.37f;
        const float Flicker = 0.80f
            + 0.14f * FMath::Sin(Tick * 0.9f + Phase)
            + 0.06f * FMath::Sin(Tick * 2.3f + Phase * 1.7f);
        const float FuelScale = bScaleWithFuel
            && CellFuel.IsValidIndex(Cell)
            ? FMath::Lerp(0.5f, 1.4f, FMath::Clamp(CellFuel[Cell], 0.0f, 1.0f))
            : 1.0f;
        const float ClusterFuelScale = FuelScale * FirePresentation::ClusterScaleVariation(Cell, ClusterIdx);
        const bool bSmouldering = PoolSmouldering.IsValidIndex(SlotIdx)
            && PoolSmouldering[SlotIdx] != 0;
        L->SetIntensity(LightIntensity * ClusterFuelScale * Flicker * (bSmouldering ? 0.08f : 1.0f));
    }
}

void AFireVisualsManager::ScorchCellVisuals(int32 Cell)
{
    const int32 X = Cell % GridW;
    const int32 Y = Cell / GridW;
    if (Terrain.IsValid()) Terrain->ScorchCell(X, Y);
    if (Forest.IsValid())  Forest->ScorchCell(X, Y);
}
