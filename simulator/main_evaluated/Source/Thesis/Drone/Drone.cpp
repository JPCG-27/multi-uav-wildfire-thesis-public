#include "Drone.h"
#include "DroneSensorBase.h"
#include "PerfectDroneSensor.h"
#include "../Human/HumanTypes.h"
#include "DronePlannerBase.h"
#include "GreedyFovPlanner.h"
#include "OracleTargetPlanner.h"
#include "RandomPlanner.h"
#include "DroneCaptureComponent.h"
#include "../World/PriorityModel.h"
#include "../World/ProceduralTerrain.h"
#include "Components/SceneComponent.h"
#include "Camera/CameraComponent.h"
#include "Engine/Engine.h"
#include "DroneLocalPriorityComputeShader.h"
#include "RenderGraphBuilder.h"
#include "RenderGraphUtils.h"
#include "RenderingThread.h"
#include "GlobalShader.h"
#include "ShaderParameterStruct.h"
#include "DataDrivenShaderPlatformInfo.h"
#include "Math/RandomStream.h"

ADrone::ADrone()
{
    PrimaryActorTick.bCanEverTick = false;
    USceneComponent* Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    SetRootComponent(Root);
}


void ADrone::ApplyManagerConfiguration(
    float InSpeedUUPerTick,
    float InHorizontalFOVDegrees,
    float InMaxViewDistanceUU,
    int32 InReplanCadenceTicks,
    float InCommitmentMargin,
    float InDistancePenaltyPerCell,
    int32 InTopKPathBonusCandidates,
    float InArrivalRadiusUU,
    int32 InFallbackReplanTicks,
    float InDetourBudgetFraction,
    bool bInUseFastRoutePlanner,
    float InFastRouteFOVGainWeight,
    bool bInUseTargetCellScoring,
    bool bInUseDroneLocalGpuPriority,
    int32 InDroneLocalPriorityRequestIntervalTicks,
    int32 InMaxDroneLocalPriorityCacheAgeTicks,
    float InNeighborPriorityWeight,
    TSubclassOf<UDroneSensorBase> InSensorClass,
    TSubclassOf<UDronePlannerBase> InPlannerClass,
    float InFirePositiveLLR,
    float InFireNegativeLLR,
    float InPersonPositiveLLR,
    float InPersonNegativeLLR,
    float InPriorityWeightStaleness,
    float InPriorityWeightDanger,
    float InPriorityWeightPerson,
    float InPriorityStalenessSaturationTicks,
    float InPriorityProbabilityScale,
    float InPriorityMonitoringUrgencyTau,
    float InPriorityUnseenExplorationBonus,
    float InPriorityFireRampMultiplier,
    float InPriorityFireMonitoringUrgencyTau,
    float InPriorityFireMinimumUrgency,
    int32 InPriorityFireDiscoveryDelayTicks,
    int32 InPriorityFireDiscoveryRampTicks,
    int32 InPriorityPersonDiscoveryDelayTicks,
    int32 InPriorityPersonDiscoveryRampTicks,
    float InLoadWeightStaleness,
    float InLoadWeightDanger,
    float InLoadWeightPerson)
{
    SpeedUUPerTick = FMath::Max(0.0f, InSpeedUUPerTick);
    HorizontalFOVDegrees = FMath::Clamp(InHorizontalFOVDegrees, 1.0f, 360.0f);
    MaxViewDistanceUU = FMath::Max(1.0f, InMaxViewDistanceUU);
    ReplanCadenceTicks = FMath::Max(1, InReplanCadenceTicks);
    CommitmentMargin = FMath::Max(0.0f, InCommitmentMargin);
    DistancePenaltyPerCell = FMath::Max(0.0f, InDistancePenaltyPerCell);
    TopKPathBonusCandidates = FMath::Max(1, InTopKPathBonusCandidates);
    ArrivalRadiusUU = FMath::Max(1.0f, InArrivalRadiusUU);
    FallbackReplanTicks = FMath::Max(1, InFallbackReplanTicks);
    DetourBudgetFraction = FMath::Max(0.0f, InDetourBudgetFraction);
    bUseFastRoutePlanner = bInUseFastRoutePlanner;
    FastRouteFOVGainWeight = FMath::Max(0.0f, InFastRouteFOVGainWeight);
    bUseTargetCellScoring = bInUseTargetCellScoring;
    bUseDroneLocalGpuPriority = bInUseDroneLocalGpuPriority;
    DroneLocalPriorityRequestIntervalTicks = FMath::Max(1, InDroneLocalPriorityRequestIntervalTicks);
    MaxDroneLocalPriorityCacheAgeTicks = FMath::Max(0, InMaxDroneLocalPriorityCacheAgeTicks);
    NeighborPriorityWeight = FMath::Max(0.0f, InNeighborPriorityWeight);

    PriorityWeightStaleness = FMath::Max(0.0f, InPriorityWeightStaleness);
    PriorityWeightDanger = FMath::Max(0.0f, InPriorityWeightDanger);
    PriorityWeightPerson = FMath::Max(0.0f, InPriorityWeightPerson);
    PriorityStalenessSaturationTicks = FMath::Max(1.0f, InPriorityStalenessSaturationTicks);
    PriorityProbabilityScale = FMath::Max(0.0f, InPriorityProbabilityScale);
    PriorityMonitoringUrgencyTau = FMath::Max(1.0f, InPriorityMonitoringUrgencyTau);
    PriorityUnseenExplorationBonus = FMath::Max(0.0f, InPriorityUnseenExplorationBonus);
    PriorityFireRampMultiplier = FMath::Max(0.0f, InPriorityFireRampMultiplier);
    PriorityFireMonitoringUrgencyTau = FMath::Max(1.0f, InPriorityFireMonitoringUrgencyTau);
    PriorityFireMinimumUrgency = FMath::Clamp(InPriorityFireMinimumUrgency, 0.0f, 1.0f);
    PriorityFireDiscoveryDelayTicks = FMath::Max(0, InPriorityFireDiscoveryDelayTicks);
    PriorityFireDiscoveryRampTicks = FMath::Max(1, InPriorityFireDiscoveryRampTicks);
    PriorityPersonDiscoveryDelayTicks = FMath::Max(0, InPriorityPersonDiscoveryDelayTicks);
    PriorityPersonDiscoveryRampTicks = FMath::Max(1, InPriorityPersonDiscoveryRampTicks);
    LoadWeightStaleness = FMath::Max(0.0f, InLoadWeightStaleness);
    LoadWeightDanger = FMath::Max(0.0f, InLoadWeightDanger);
    LoadWeightPerson = FMath::Max(0.0f, InLoadWeightPerson);
    CachedFirePositiveLLR = InFirePositiveLLR;
    CachedFireNegativeLLR = InFireNegativeLLR;
    CachedPersonPositiveLLR = InPersonPositiveLLR;
    CachedPersonNegativeLLR = InPersonNegativeLLR;

    const bool bSensorClassChanged = InSensorClass && InSensorClass != SensorClass;
    SensorClass = InSensorClass;
    if (!Sensor || bSensorClassChanged)
    {
        Sensor = SensorClass
            ? NewObject<UDroneSensorBase>(this, SensorClass)
            : NewObject<UPerfectDroneSensor>(this);
    }
    if (Sensor)
    {
        Sensor->SetPersonGroundTruth(PersonGroundTruth);
    }

    if (UPerfectDroneSensor* PerfectSensor = Cast<UPerfectDroneSensor>(Sensor))
    {
        PerfectSensor->FirePositiveLLR = InFirePositiveLLR;
        PerfectSensor->FireNegativeLLR = InFireNegativeLLR;
        PerfectSensor->PersonPositiveLLR = InPersonPositiveLLR;
        PerfectSensor->PersonNegativeLLR = InPersonNegativeLLR;
        // Keep the parallel-path sensing duplicate in sync with the sensor's
        // world-space gating flag (see SensePerfectFromLocation).
        bWorldSpacePersonGatingCached = PerfectSensor->bWorldSpacePersonGating;
    }

    const bool bPlannerClassChanged = InPlannerClass && InPlannerClass != PlannerClass;
    if (InPlannerClass)
    {
        PlannerClass = InPlannerClass;
    }
    if (!Planner || bPlannerClassChanged)
    {
        Planner = PlannerClass
            ? NewObject<UDronePlannerBase>(this, PlannerClass)
            : NewObject<UGreedyFovPlanner>(this);
    }

    if (CellSizeCached > 0.0f)
    {
        PrecomputeFOVFootprint();
    }

    ResetLocalGpuPriorityCache();
    bForceReplan = true;
}

void ADrone::ConfigureTerrainFlight(AProceduralTerrain* InTerrain,
                                    float InMinimumAltitudeAboveGroundUU,
                                    float InCruiseAltitudeAboveGroundUU,
                                    float InCameraForwardOffsetUU,
                                    float InCameraDownOffsetUU)
{
    TerrainProvider = InTerrain;
    MinimumAltitudeAboveGroundUU = FMath::Max(0.0f, InMinimumAltitudeAboveGroundUU);
    CruiseAltitudeAboveGroundUU = FMath::Max(MinimumAltitudeAboveGroundUU, InCruiseAltitudeAboveGroundUU);
    CameraForwardOffsetUU = FMath::Max(0.0f, InCameraForwardOffsetUU);
    CameraDownOffsetUU = FMath::Max(0.0f, InCameraDownOffsetUU);

    SetActorLocation(ApplyTerrainFlightAltitude(GetActorLocation(), true));

    if (CaptureComponent)
    {
        CaptureComponent->SetRelativeLocation(FVector(CameraForwardOffsetUU, 0.0f, -CameraDownOffsetUU));
        CaptureComponent->HideActorComponents(this, true);
    }
    if (ViewCamera)
    {
        ViewCamera->SetRelativeLocation(FVector(CameraForwardOffsetUU, 0.0f, -CameraDownOffsetUU));
    }
}

void ADrone::BeginPlay()
{
    Super::BeginPlay();

    if (SensorClass)
        Sensor = NewObject<UDroneSensorBase>(this, SensorClass);
    else
        Sensor = NewObject<UPerfectDroneSensor>(this);
    if (Sensor)
        Sensor->SetPersonGroundTruth(PersonGroundTruth);

    if (PlannerClass)
        Planner = NewObject<UDronePlannerBase>(this, PlannerClass);
    else
        Planner = NewObject<UGreedyFovPlanner>(this);
}

void ADrone::SetPersonGroundTruth(const TArray<FHumanSnapshot>* InSnapshots)
{
    PersonGroundTruth = InSnapshots;
    if (Sensor)
    {
        Sensor->SetPersonGroundTruth(InSnapshots);
    }
}

void ADrone::InitFromManager(Grid* InSharedGrid, int32 InDroneIndex,
                             const FVector& GridOriginWorld, float CellSizeUU)
{
    DroneIndex = InDroneIndex;
    GridOriginCached = GridOriginWorld;
    CellSizeCached = CellSizeUU;

    if (!InSharedGrid) return;

    SharedGrid = InSharedGrid;

    PrivateObservationBuffer.Reset();
    OwnedCells.Reset();
    OwnedCellMask.Init(0, InSharedGrid->Width * InSharedGrid->Height);
    TargetCellIndex = INDEX_NONE;
    TicksSinceReplan = 0;
    LastReplanTick = -1;
    CurrentTargetScore = 0.0f;
    CurrentPath.Empty();
    CurrentPathIndex = 0;
    bForceReplan = true;
    CurrentFacingDir = 0;   // per-scenario sensing state; must reset or it leaks
                            // across in-process scenario swaps (reused drone actor).
    ResetLocalGpuPriorityCache();

    OwnGrid = MakeUnique<Grid>(InSharedGrid->Width, InSharedGrid->Height);
    OwnGrid->Initialize();
    for (int32 y = 0; y < InSharedGrid->Height; ++y)
    {
        for (int32 x = 0; x < InSharedGrid->Width; ++x)
        {
            OwnGrid->setFirstFireTick(x, y, InSharedGrid->getFirstFireTick(x, y));
            OwnGrid->setFloraDensity(x, y, InSharedGrid->getFloraDensity(x, y));
        }
    }

    PrecomputeFOVFootprint();
}

// ---------------------------------------------------------------------------
//  FOV footprint pre‑computation
// ---------------------------------------------------------------------------
void ADrone::PrecomputeFOVFootprint()
{
    for (int32 d = 0; d < 8; ++d)
        FOVOffsets[d].Reset();

    static constexpr int32 DirX[8] =
    {
         1,  1,  0, -1,
        -1, -1,  0,  1
    };

    static constexpr int32 DirY[8] =
    {
         0, -1, -1, -1,
         0,  1,  1,  1
    };

    const float MaxDistCells =
        MaxViewDistanceUU / CellSizeCached;

    const float CosHalfFOV =
        FMath::Cos(
            FMath::DegreesToRadians(
                HorizontalFOVDegrees * 0.5f));

    const int32 Radius =
        FMath::CeilToInt(MaxDistCells);

    for (int32 d = 0; d < 8; ++d)
    {
        const FVector2D Forward(
            static_cast<float>(DirX[d]),
            static_cast<float>(DirY[d]));

        const FVector2D Fwd =
            Forward.GetSafeNormal();

        for (int32 oy = -Radius; oy <= Radius; ++oy)
        {
            for (int32 ox = -Radius; ox <= Radius; ++ox)
            {
                if (ox == 0 && oy == 0)
                    continue;

                const FVector2D Delta(
                    static_cast<float>(ox),
                    static_cast<float>(oy));

                const float Dist =
                    Delta.Length();

                if (Dist <= KINDA_SMALL_NUMBER)
                    continue;

                if (Dist > MaxDistCells)
                    continue;

                const FVector2D Dir =
                    Delta / Dist;

                const float CosAngle =
                    FVector2D::DotProduct(Fwd, Dir);

                if (CosAngle < CosHalfFOV)
                    continue;

                FOVOffsets[d].Add(FIntPoint(ox, oy));
            }
        }
    }
}


// ---------------------------------------------------------------------------
//  Coordinate / direction helpers
// ---------------------------------------------------------------------------
void ADrone::CellIndexToXY(int32 Idx, int32& OutX, int32& OutY) const
{
    if (!OwnGrid) { OutX = OutY = 0; return; }
    OutX = Idx % OwnGrid->Width;
    OutY = Idx / OwnGrid->Width;
}

int32 ADrone::DirectionIndexFromDelta(int32 dx, int32 dy) const
{
    // 0=E, 1=NE, 2=N, 3=NW, 4=W, 5=SW, 6=S, 7=SE.
    // Grid/world Y increases downward in cell space, so north is dy == -1.
    if      (dx ==  1 && dy ==  0) return 0; // E
    else if (dx ==  1 && dy == -1) return 1; // NE
    else if (dx ==  0 && dy == -1) return 2; // N
    else if (dx == -1 && dy == -1) return 3; // NW
    else if (dx == -1 && dy ==  0) return 4; // W
    else if (dx == -1 && dy ==  1) return 5; // SW
    else if (dx ==  0 && dy ==  1) return 6; // S
    else if (dx ==  1 && dy ==  1) return 7; // SE
    return 0;
}

int32 ADrone::DirectionIndex(int32 FromCell, int32 ToCell) const
{
    int32 fx, fy, tx, ty;
    CellIndexToXY(FromCell, fx, fy);
    CellIndexToXY(ToCell, tx, ty);
    return DirectionIndexFromDelta(tx - fx, ty - fy);
}

int32 ADrone::GetCurrentCellIndex() const
{
    return GetCurrentCellIndexFromLocation(GetActorLocation());
}

int32 ADrone::GetCurrentCellIndexFromLocation(const FVector& WorldLocation) const
{
    if (!OwnGrid || CellSizeCached <= 0.0f)
    {
        return INDEX_NONE;
    }

    const FVector& Pos = WorldLocation;
    const int32 X = FMath::Clamp(
        FMath::FloorToInt((Pos.X - GridOriginCached.X) / CellSizeCached),
        0,
        OwnGrid->Width - 1);
    const int32 Y = FMath::Clamp(
        FMath::FloorToInt((Pos.Y - GridOriginCached.Y) / CellSizeCached),
        0,
        OwnGrid->Height - 1);
    return Y * OwnGrid->Width + X;
}

// ---------------------------------------------------------------------------
//  Priority helpers
// ---------------------------------------------------------------------------
float ADrone::ComputeCellPriority(int32 CellIndex, int32 CurrentTick) const
{
    if (!OwnGrid) return 0.0f;
    int32 X, Y;
    CellIndexToXY(CellIndex, X, Y);
    const int32 LastObs = OwnGrid->getLastObservedTick(X, Y);
    const float Staleness = (LastObs < 0)
        ? static_cast<float>(CurrentTick)
        : static_cast<float>(CurrentTick - LastObs);
    float StalenessTerm = FMath::Max(0.0f, Staleness / PriorityStalenessSaturationTicks);
    if (LastObs < 0)
    {
        StalenessTerm += PriorityUnseenExplorationBonus;
    }

    const int32 LastPersonObs = bSeparatePersonFreshness
        ? OwnGrid->getLastPersonObservedTick(X, Y) : LastObs;
    const float PersonStaleness = bSeparatePersonFreshness
        ? ((LastPersonObs < 0) ? static_cast<float>(CurrentTick) : static_cast<float>(CurrentTick - LastPersonObs))
        : Staleness;
    const float GeneralUrgency = 1.0f - FMath::Exp(-PersonStaleness / PriorityMonitoringUrgencyTau);
    const float FireUrgency = FMath::Max(
        PriorityFireMinimumUrgency,
        1.0f - FMath::Exp(-Staleness / PriorityFireMonitoringUrgencyTau));

    const float FireGate = ThesisPriority::DiscoveryGateWithParams(
        OwnGrid->getFirstFireDetectionTick(X, Y),
        CurrentTick,
        PriorityFireDiscoveryDelayTicks,
        PriorityFireDiscoveryRampTicks);
    const float PersonGate = ThesisPriority::DiscoveryGateWithParams(
        OwnGrid->getFirstPersonDetectionTick(X, Y),
        CurrentTick,
        PriorityPersonDiscoveryDelayTicks,
        PriorityPersonDiscoveryRampTicks);

    const float DangerTerm = ThesisPriority::PositiveEvidence(OwnGrid->getFireBelief(X, Y))
        * PriorityProbabilityScale
        * FireUrgency
        * FireGate
        * PriorityFireRampMultiplier;
    const float PersonTerm = ThesisPriority::PositiveEvidence(OwnGrid->getPersonBelief(X, Y))
        * PriorityProbabilityScale
        * GeneralUrgency
        * PersonGate;

    return PriorityWeightStaleness * StalenessTerm
         + PriorityWeightDanger    * DangerTerm
         + PriorityWeightPerson    * PersonTerm;
}

float ADrone::ComputeCellLoadPriority(int32 CellIndex, int32 CurrentTick) const
{
    if (!OwnGrid) return 0.0f;
    int32 X, Y;
    CellIndexToXY(CellIndex, X, Y);
    const int32 LastObs = OwnGrid->getLastObservedTick(X, Y);
    const float Staleness = (LastObs < 0)
        ? static_cast<float>(CurrentTick)
        : static_cast<float>(CurrentTick - LastObs);
    const float StalenessTerm = FMath::Clamp(Staleness / PriorityStalenessSaturationTicks, 0.0f, 1.0f);
    const float DangerTerm = ThesisPriority::PositiveEvidence(OwnGrid->getFireBelief(X, Y)) * PriorityProbabilityScale;
    const float PersonTerm = ThesisPriority::PositiveEvidence(OwnGrid->getPersonBelief(X, Y)) * PriorityProbabilityScale;

    float Priority = LoadWeightStaleness * StalenessTerm
                   + LoadWeightDanger    * DangerTerm
                   + LoadWeightPerson    * PersonTerm;
    if (LastObs < 0)
        Priority += PriorityUnseenExplorationBonus;
    return Priority;
}

float ADrone::ComputeCellDetectionMass(int32 CellIndex) const
{
    if (!OwnGrid) return 0.0f;
    int32 X, Y;
    CellIndexToXY(CellIndex, X, Y);
    const float DangerTerm = ThesisPriority::PositiveEvidence(OwnGrid->getFireBelief(X, Y)) * PriorityProbabilityScale;
    const float PersonTerm = ThesisPriority::PositiveEvidence(OwnGrid->getPersonBelief(X, Y)) * PriorityProbabilityScale;
    return LoadWeightDanger * DangerTerm + LoadWeightPerson * PersonTerm;
}

namespace
{
    struct FDroneLocalPriorityCellA
    {
        uint32 FireLogOddsBits;
        uint32 PersonLogOddsBits;
        uint32 LastObservedTickBits;
        uint32 FirstFireDetectionTickBits;
    };

    struct FDroneLocalPriorityCellB
    {
        uint32 FirstPersonDetectionTickBits;
        uint32 LastPersonObservedTickBits;
        uint32 Padding1;
        uint32 Padding2;
    };

    struct FDroneLocalPlanningOutput
    {
        float Priority;
        float NeighborhoodScore;
    };

    static uint32 FloatToBits(float Value)
    {
        uint32 Bits = 0;
        FMemory::Memcpy(&Bits, &Value, sizeof(uint32));
        return Bits;
    }

    static uint32 IntToBits(int32 Value)
    {
        return static_cast<uint32>(Value);
    }
}

void ADrone::ResetLocalGpuPriorityCache()
{
    // The readback object may be referenced by queued render-thread commands.
    // Flush before destroying it during reset/reinit/teardown.
    if (bLocalGpuPriorityRequestInFlight || bLocalGpuPriorityCopyQueued)
    {
        FlushRenderingCommands();
    }

    CachedLocalGpuPriority.Reset();
    CachedLocalGpuNeighborhoodScore.Reset();
    CachedLocalGpuPriorityTick = -1;
    LastLocalGpuPriorityRequestTick = -999999;
    PendingLocalGpuPriorityTick = -1;
    PendingLocalGpuPriorityCopyTick = -1;
    bLocalGpuPriorityRequestInFlight = false;
    bLocalGpuPriorityCopyQueued = false;
    PendingLocalPriorityCpuCopy.Reset();
    LocalPriorityReadback.Reset();
}

bool ADrone::TryConsumeLocalGpuPriority(TArray<float>& OutPriorityCache,
                                        TArray<float>& OutNeighborhoodScoreCache,
                                        int32 CurrentTick)
{
    if (!bUseDroneLocalGpuPriority || !OwnGrid)
    {
        return false;
    }

    const int32 Total = OwnGrid->Width * OwnGrid->Height;
    if (Total <= 0)
    {
        return false;
    }

    // Important: FRHIGPUBufferReadback::Lock()/Unlock() must be called on the
    // rendering thread in UE 5.x. The first patch called Lock() here from the
    // game thread, which triggers Assertion failed: IsInRenderingThread().
    if (LocalPriorityReadback && bLocalGpuPriorityRequestInFlight && !bLocalGpuPriorityCopyQueued)
    {
        if (LocalPriorityReadbackFence.IsFenceComplete())
        {
            const SIZE_T NumBytes = static_cast<SIZE_T>(Total) * sizeof(FDroneLocalPlanningOutput);
            TSharedPtr<FLocalGpuPlanningReadback, ESPMode::ThreadSafe> CpuCopy = MakeShared<FLocalGpuPlanningReadback, ESPMode::ThreadSafe>();
            FRHIGPUBufferReadback* Readback = LocalPriorityReadback.Get();

            ENQUEUE_RENDER_COMMAND(ThesisDroneLocalPriorityConsumeReadback)(
                [Readback, CpuCopy, NumBytes, Total](FRHICommandListImmediate& RHICmdList)
                {
                    if (!Readback || !Readback->IsReady())
                    {
                        return;
                    }

                    const void* Data = Readback->Lock(static_cast<uint32>(NumBytes));
                    if (!Data)
                    {
                        return;
                    }

                    CpuCopy->Priority.SetNumUninitialized(Total);
                    CpuCopy->NeighborhoodScore.SetNumUninitialized(Total);

                    const FDroneLocalPlanningOutput* Src = static_cast<const FDroneLocalPlanningOutput*>(Data);
                    for (int32 i = 0; i < Total; ++i)
                    {
                        CpuCopy->Priority[i] = Src[i].Priority;
                        CpuCopy->NeighborhoodScore[i] = Src[i].NeighborhoodScore;
                    }

                    Readback->Unlock();
                });

            PendingLocalPriorityCpuCopy = CpuCopy;
            PendingLocalGpuPriorityCopyTick = PendingLocalGpuPriorityTick;
            bLocalGpuPriorityCopyQueued = true;
            bLocalGpuPriorityRequestInFlight = false;
            PendingLocalGpuPriorityTick = -1;
            LocalPriorityCpuCopyFence.BeginFence();
        }
    }

    if (bLocalGpuPriorityCopyQueued && LocalPriorityCpuCopyFence.IsFenceComplete())
    {
        if (PendingLocalPriorityCpuCopy.IsValid() &&
            PendingLocalPriorityCpuCopy->Priority.Num() == Total &&
            PendingLocalPriorityCpuCopy->NeighborhoodScore.Num() == Total)
        {
            CachedLocalGpuPriority = PendingLocalPriorityCpuCopy->Priority;
            CachedLocalGpuNeighborhoodScore = PendingLocalPriorityCpuCopy->NeighborhoodScore;
            CachedLocalGpuPriorityTick = PendingLocalGpuPriorityCopyTick;
        }

        PendingLocalPriorityCpuCopy.Reset();
        PendingLocalGpuPriorityCopyTick = -1;
        bLocalGpuPriorityCopyQueued = false;
    }

    if (CachedLocalGpuPriority.Num() != Total ||
        CachedLocalGpuNeighborhoodScore.Num() != Total ||
        CachedLocalGpuPriorityTick < 0)
    {
        return false;
    }

    if (MaxDroneLocalPriorityCacheAgeTicks > 0 &&
        CurrentTick - CachedLocalGpuPriorityTick > MaxDroneLocalPriorityCacheAgeTicks)
    {
        return false;
    }

    OutPriorityCache = CachedLocalGpuPriority;
    OutNeighborhoodScoreCache = CachedLocalGpuNeighborhoodScore;
    return true;
}

void ADrone::RequestLocalGpuPriority(int32 CurrentTick)
{
    if (!bUseDroneLocalGpuPriority || !OwnGrid)
    {
        return;
    }

    const int32 W = OwnGrid->Width;
    const int32 H = OwnGrid->Height;
    const int32 Total = W * H;
    if (Total <= 0)
    {
        return;
    }

    if (bLocalGpuPriorityRequestInFlight || bLocalGpuPriorityCopyQueued)
    {
        return;
    }

    const int32 Interval = FMath::Max(1, DroneLocalPriorityRequestIntervalTicks);
    if (CurrentTick - LastLocalGpuPriorityRequestTick < Interval)
    {
        return;
    }

    TArray<FDroneLocalPriorityCellA> PackedA;
    TArray<FDroneLocalPriorityCellB> PackedB;
    TArray<uint32> PackedOwnedMask;
    PackedA.SetNumUninitialized(Total);
    PackedB.SetNumUninitialized(Total);
    PackedOwnedMask.Init(0u, Total);

    for (const int32 Cell : OwnedCells)
    {
        if (Cell >= 0 && Cell < Total)
        {
            PackedOwnedMask[Cell] = 1u;
        }
    }

    for (int32 Y = 0; Y < H; ++Y)
    {
        for (int32 X = 0; X < W; ++X)
        {
            const int32 Idx = Y * W + X;
            PackedA[Idx].FireLogOddsBits = FloatToBits(OwnGrid->getFireBelief(X, Y));
            PackedA[Idx].PersonLogOddsBits = FloatToBits(OwnGrid->getPersonBelief(X, Y));
            PackedA[Idx].LastObservedTickBits = IntToBits(OwnGrid->getLastObservedTick(X, Y));
            PackedA[Idx].FirstFireDetectionTickBits = IntToBits(OwnGrid->getFirstFireDetectionTick(X, Y));

            PackedB[Idx].FirstPersonDetectionTickBits = IntToBits(OwnGrid->getFirstPersonDetectionTick(X, Y));
            PackedB[Idx].LastPersonObservedTickBits = IntToBits(bSeparatePersonFreshness
                ? OwnGrid->getLastPersonObservedTick(X, Y) : OwnGrid->getLastObservedTick(X, Y));
            PackedB[Idx].Padding1 = 0;
            PackedB[Idx].Padding2 = 0;
        }
    }

    if (!LocalPriorityReadback)
    {
        LocalPriorityReadback = MakeUnique<FRHIGPUBufferReadback>(TEXT("Thesis.DroneLocalPriorityReadback"));
    }

    FRHIGPUBufferReadback* Readback = LocalPriorityReadback.Get();
    const uint32 NumCells = static_cast<uint32>(Total);
    const uint32 TickValue = static_cast<uint32>(FMath::Max(CurrentTick, 0));

    // Copy configurable UObject member values on the game thread before entering the
    // render-command lambda. Do not capture `this` across threads.
    const float WeightStalenessValue = PriorityWeightStaleness;
    const float WeightDangerValue = PriorityWeightDanger;
    const float WeightPersonValue = PriorityWeightPerson;
    const float StalenessSaturationTicksValue = PriorityStalenessSaturationTicks;
    const float ProbabilityScaleValue = PriorityProbabilityScale;
    const float MonitoringUrgencyTauValue = PriorityMonitoringUrgencyTau;
    const float UnseenExplorationBonusValue = PriorityUnseenExplorationBonus;
    const float FirePriorityRampMultiplierValue = PriorityFireRampMultiplier;
    const float FireMonitoringUrgencyTauValue = PriorityFireMonitoringUrgencyTau;
    const float FireMinimumUrgencyValue = PriorityFireMinimumUrgency;
    const uint32 FireDangerDiscoveryDelayTicksValue = static_cast<uint32>(PriorityFireDiscoveryDelayTicks);
    const uint32 FireDangerDiscoveryRampTicksValue = static_cast<uint32>(PriorityFireDiscoveryRampTicks);
    const uint32 DroneDangerDiscoveryDelayTicksValue = static_cast<uint32>(PriorityPersonDiscoveryDelayTicks);
    const uint32 DroneDangerDiscoveryRampTicksValue = static_cast<uint32>(PriorityPersonDiscoveryRampTicks);

    ENQUEUE_RENDER_COMMAND(ThesisDroneLocalPriorityCompute)(
        [PackedA = MoveTemp(PackedA),
         PackedB = MoveTemp(PackedB),
         PackedOwnedMask = MoveTemp(PackedOwnedMask),
         Readback,
         NumCells,
         WidthValue = static_cast<uint32>(W),
         HeightValue = static_cast<uint32>(H),
         TickValue,
         NeighborWeightValue = NeighborPriorityWeight,
         WeightStalenessValue,
         WeightDangerValue,
         WeightPersonValue,
         StalenessSaturationTicksValue,
         ProbabilityScaleValue,
         MonitoringUrgencyTauValue,
         UnseenExplorationBonusValue,
         FirePriorityRampMultiplierValue,
         FireMonitoringUrgencyTauValue,
         FireMinimumUrgencyValue,
         FireDangerDiscoveryDelayTicksValue,
         FireDangerDiscoveryRampTicksValue,
         DroneDangerDiscoveryDelayTicksValue,
         DroneDangerDiscoveryRampTicksValue](FRHICommandListImmediate& RHICmdList) mutable
        {
            if (!Readback || NumCells == 0)
            {
                return;
            }

            FRDGBuilder GraphBuilder(RHICmdList);

            FRDGBufferRef CellStateA = CreateStructuredBuffer(
                GraphBuilder,
                TEXT("Thesis.DroneLocalPriority.CellStateA"),
                sizeof(FDroneLocalPriorityCellA),
                NumCells,
                PackedA.GetData(),
                PackedA.Num() * sizeof(FDroneLocalPriorityCellA));

            FRDGBufferRef CellStateB = CreateStructuredBuffer(
                GraphBuilder,
                TEXT("Thesis.DroneLocalPriority.CellStateB"),
                sizeof(FDroneLocalPriorityCellB),
                NumCells,
                PackedB.GetData(),
                PackedB.Num() * sizeof(FDroneLocalPriorityCellB));

            FRDGBufferRef OwnedMask = CreateStructuredBuffer(
                GraphBuilder,
                TEXT("Thesis.DroneLocalPriority.OwnedMask"),
                sizeof(uint32),
                NumCells,
                PackedOwnedMask.GetData(),
                PackedOwnedMask.Num() * sizeof(uint32));

            FRDGBufferRef OutputPlanning = GraphBuilder.CreateBuffer(
                FRDGBufferDesc::CreateStructuredDesc(sizeof(FDroneLocalPlanningOutput), NumCells),
                TEXT("Thesis.DroneLocalPriority.OutputPlanning"));

            FDroneLocalPriorityComputeCS::FParameters* Params =
                GraphBuilder.AllocParameters<FDroneLocalPriorityComputeCS::FParameters>();
            Params->CellStateA = GraphBuilder.CreateSRV(FRDGBufferSRVDesc(CellStateA));
            Params->CellStateB = GraphBuilder.CreateSRV(FRDGBufferSRVDesc(CellStateB));
            Params->OwnedMask = GraphBuilder.CreateSRV(FRDGBufferSRVDesc(OwnedMask));
            Params->OutputPlanning = GraphBuilder.CreateUAV(FRDGBufferUAVDesc(OutputPlanning));
            Params->NumCells = NumCells;
            Params->Width = WidthValue;
            Params->Height = HeightValue;
            Params->CurrentTick = TickValue;
            Params->WeightStaleness = WeightStalenessValue;
            Params->WeightDanger = WeightDangerValue;
            Params->WeightPerson = WeightPersonValue;
            Params->StalenessSaturationTicks = StalenessSaturationTicksValue;
            Params->ProbabilityScale = ProbabilityScaleValue;
            Params->MonitoringUrgencyTau = MonitoringUrgencyTauValue;
            Params->UnseenExplorationBonus = UnseenExplorationBonusValue;
            Params->FirePriorityRampMultiplier = FirePriorityRampMultiplierValue;
            Params->FireMonitoringUrgencyTau = FireMonitoringUrgencyTauValue;
            Params->FireMinimumUrgency = FireMinimumUrgencyValue;
            Params->NeighborPriorityWeight = NeighborWeightValue;
            Params->FireDangerDiscoveryDelayTicks = FireDangerDiscoveryDelayTicksValue;
            Params->FireDangerDiscoveryRampTicks = FireDangerDiscoveryRampTicksValue;
            Params->DroneDangerDiscoveryDelayTicks = DroneDangerDiscoveryDelayTicksValue;
            Params->DroneDangerDiscoveryRampTicks = DroneDangerDiscoveryRampTicksValue;

            TShaderMapRef<FDroneLocalPriorityComputeCS> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
            FComputeShaderUtils::AddPass(
                GraphBuilder,
                RDG_EVENT_NAME("Thesis.DroneLocalPriorityCompute"),
                Shader,
                Params,
                FIntVector(FMath::DivideAndRoundUp(static_cast<int32>(NumCells), 64), 1, 1));

            AddEnqueueCopyPass(GraphBuilder, Readback, OutputPlanning, static_cast<uint32>(NumCells * sizeof(FDroneLocalPlanningOutput)));
            GraphBuilder.Execute();
        });

    LocalPriorityReadbackFence.BeginFence();
    bLocalGpuPriorityRequestInFlight = true;
    PendingLocalGpuPriorityTick = CurrentTick;
    LastLocalGpuPriorityRequestTick = CurrentTick;
}

FVector ADrone::GetCellWorldCenter(int32 CellIndex) const
{
    return GetCellWorldCenterForPlanning(CellIndex, GetActorLocation().Z, true);
}

FVector ADrone::GetCellWorldCenterForPlanning(int32 CellIndex,
                                              float ReferenceZ,
                                              bool bUseTerrainAdjustedCenter) const
{
    int32 X, Y;
    CellIndexToXY(CellIndex, X, Y);
    const float WorldX = GridOriginCached.X + (X + 0.5f) * CellSizeCached;
    const float WorldY = GridOriginCached.Y + (Y + 0.5f) * CellSizeCached;
    const FVector Center(WorldX, WorldY, ReferenceZ);
    return bUseTerrainAdjustedCenter ? ApplyTerrainFlightAltitude(Center, true) : Center;
}

float ADrone::GetTerrainHeightAtWorldXY(float WorldX, float WorldY) const
{
    if (!TerrainProvider.IsValid() || CellSizeCached <= 0.0f)
    {
        return GridOriginCached.Z;
    }

    const float CellX = (WorldX - GridOriginCached.X) / CellSizeCached;
    const float CellY = (WorldY - GridOriginCached.Y) / CellSizeCached;
    return TerrainProvider->GetHeightAtCell(CellX, CellY);
}

float ADrone::GetTerrainFlightReferenceZAtWorldXY(float WorldX, float WorldY) const
{
    float HighestZ = GetTerrainHeightAtWorldXY(WorldX, WorldY);
    if (!TerrainProvider.IsValid() || CellSizeCached <= 0.0f)
    {
        return HighestZ;
    }

    const float ProbeRadiusUU = FMath::Clamp(CellSizeCached * 0.35f, 100.0f, 500.0f);
    const FVector2D Offsets[] = {
        FVector2D( ProbeRadiusUU, 0.0f),
        FVector2D(-ProbeRadiusUU, 0.0f),
        FVector2D(0.0f,  ProbeRadiusUU),
        FVector2D(0.0f, -ProbeRadiusUU),
        FVector2D( ProbeRadiusUU,  ProbeRadiusUU),
        FVector2D( ProbeRadiusUU, -ProbeRadiusUU),
        FVector2D(-ProbeRadiusUU,  ProbeRadiusUU),
        FVector2D(-ProbeRadiusUU, -ProbeRadiusUU)
    };

    for (const FVector2D& Offset : Offsets)
    {
        HighestZ = FMath::Max(
            HighestZ,
            GetTerrainHeightAtWorldXY(WorldX + Offset.X, WorldY + Offset.Y));
    }
    return HighestZ;
}

FVector ADrone::ApplyTerrainFlightAltitude(const FVector& InLocation, bool bUseCruiseAltitude) const
{
    FVector Out = InLocation;
    const float GroundZ = GetTerrainFlightReferenceZAtWorldXY(Out.X, Out.Y);
    const float TargetAgl = bUseCruiseAltitude ? CruiseAltitudeAboveGroundUU : MinimumAltitudeAboveGroundUU;
    Out.Z = FMath::Max(Out.Z, GroundZ + FMath::Max(0.0f, TargetAgl));
    if (bUseCruiseAltitude)
    {
        Out.Z = GroundZ + CruiseAltitudeAboveGroundUU;
    }
    return Out;
}

// ---------------------------------------------------------------------------
//  Ownership update
// ---------------------------------------------------------------------------
void ADrone::SetOwnedCells(TArray<int32>&& InOwnedCells, const TArray<int32>& NewlyOwnedCells)
{
    const int32 Total = OwnGrid ? OwnGrid->Width * OwnGrid->Height : 0;
    if (OwnedCellMask.Num() != Total)
    {
        OwnedCellMask.Init(0, Total);
    }
    else
    {
        for (const int32 Cell : OwnedCells)
        {
            if (Cell >= 0 && Cell < Total)
            {
                OwnedCellMask[Cell] = 0;
            }
        }
    }

    for (const int32 Cell : InOwnedCells)
    {
        if (Cell >= 0 && Cell < Total)
        {
            OwnedCellMask[Cell] = 1;
        }
    }

    // Ordinary planners are territory-confined.  The benchmark oracle is a
    // global upper-bound policy, so partition changes must not invalidate its
    // direct intercept flight.
    const bool bPlannerIsOracle = Planner && Planner->IsA<UOracleTargetPlanner>();
    bool bInvalidatePlan = false;

    if (!bPlannerIsOracle && TargetCellIndex != INDEX_NONE && !IsOwnedCell(TargetCellIndex))
    {
        bInvalidatePlan = true;
    }

    if (!bInvalidatePlan && CurrentPath.Num() > 0)
    {
        for (int32 Cell : CurrentPath)
        {
            if (!IsOwnedCell(Cell))
            {
                bInvalidatePlan = true;
                break;
            }
        }
    }

    OwnedCells = MoveTemp(InOwnedCells);

    // Refresh this drone's local belief only for cells it just acquired from
    // the server-side CPU aggregate. Existing owned cells already carry this
    // drone's local updates, so re-copying them every partition is wasted work.
    if (OwnGrid && SharedGrid)
    {
        for (const int32 Cell : NewlyOwnedCells)
        {
            if (Cell < 0 || Cell >= OwnGrid->Width * OwnGrid->Height)
                continue;

            int32 X = 0;
            int32 Y = 0;
            CellIndexToXY(Cell, X, Y);

            OwnGrid->setFireBelief(X, Y, SharedGrid->getFireBelief(X, Y));
            OwnGrid->setPersonBelief(X, Y, SharedGrid->getPersonBelief(X, Y));
            OwnGrid->setLastObservedTick(X, Y, SharedGrid->getLastObservedTick(X, Y));
            OwnGrid->setLastPersonObservedTick(X, Y, SharedGrid->getLastPersonObservedTick(X, Y));
            OwnGrid->setFirstFireDetectionTick(X, Y, SharedGrid->getFirstFireDetectionTick(X, Y));
            OwnGrid->setFirstPersonDetectionTick(X, Y, SharedGrid->getFirstPersonDetectionTick(X, Y));
        }
    }

    if (bInvalidatePlan)
    {
        TargetCellIndex = INDEX_NONE;
        CurrentPath.Empty();
        CurrentPathIndex = 0;
        CurrentTargetScore = 0.0f;
        bForceReplan = true;
    }
    else
    {
        // Ownership still changed, so we still want a refresh soon.
        bForceReplan = true;
    }
}

float ADrone::GetCachedCpuPlanningPriority(int32 CellIndex, int32 CurrentTick)
{
    if (!OwnGrid || CellIndex < 0 || CellIndex >= OwnGrid->Width * OwnGrid->Height)
    {
        return 0.0f;
    }

    const int32 Total = OwnGrid->Width * OwnGrid->Height;
    if (ScratchCpuPlanningPriority.Num() != Total ||
        ScratchCpuPlanningPriorityTick.Num() != Total)
    {
        ScratchCpuPlanningPriority.SetNumUninitialized(Total);
        ScratchCpuPlanningPriorityTick.Init(MIN_int32, Total);
    }

    if (ScratchCpuPlanningPriorityTick[CellIndex] != CurrentTick)
    {
        ScratchCpuPlanningPriority[CellIndex] = ComputeCellPriority(CellIndex, CurrentTick);
        ScratchCpuPlanningPriorityTick[CellIndex] = CurrentTick;
    }
    return ScratchCpuPlanningPriority[CellIndex];
}

bool ADrone::ComputePathToTarget(int32 StartCell, int32 TargetCell,
                                 const TArray<float>* PriorityCache,
                                 int32 CurrentTick,
                                 bool bUseCpuPriorityFallback,
                                 bool bTargetCellScoring)
{
    const int32 W     = OwnGrid->Width;
    const int32 H     = OwnGrid->Height;
    const int32 Total = W * H;

    CurrentPath.Empty();

    auto ReadPriority = [&](int32 Cell) -> float
    {
        if (PriorityCache && PriorityCache->IsValidIndex(Cell))
        {
            return (*PriorityCache)[Cell];
        }
        return bUseCpuPriorityFallback ? GetCachedCpuPlanningPriority(Cell, CurrentTick) : 0.0f;
    };

    if (StartCell == TargetCell)
    {
        CurrentPath.Add(TargetCell);
        return true;
    }

    auto IsOwned = [&](int32 Cell) -> bool
    {
        return IsOwnedCell(Cell);
    };

    if (!IsOwned(TargetCell))
        return false;

    // ------------------------------------------------------------
    // Start / target coordinates
    // ------------------------------------------------------------
    int32 SX = 0;
    int32 SY = 0;
    int32 TX = 0;
    int32 TY = 0;

    CellIndexToXY(StartCell, SX, SY);
    CellIndexToXY(TargetCell, TX, TY);

    // ------------------------------------------------------------
    // BFS shortest distance to target over owned territory
    //
    // StartCell may be outside ownership right after repartition.
    // Every other traversed cell must stay inside ownership.
    // ------------------------------------------------------------
    TArray<int32>& DistToTarget = ScratchPathDist;   // per-drone reuse (no per-call alloc)
    TArray<int32>& TouchedDistCells = ScratchPathTouchedDistCells;
    if (DistToTarget.Num() != Total)
    {
        DistToTarget.Init(INT32_MAX, Total);
        TouchedDistCells.Reset();
    }
    else
    {
        for (const int32 Cell : TouchedDistCells)
        {
            if (Cell >= 0 && Cell < Total)
            {
                DistToTarget[Cell] = INT32_MAX;
            }
        }
        TouchedDistCells.Reset();
    }
    auto SetPathDist = [&](int32 Cell, int32 Dist)
    {
        if (DistToTarget[Cell] == INT32_MAX)
        {
            TouchedDistCells.Add(Cell);
        }
        DistToTarget[Cell] = Dist;
    };

    TArray<int32>& BFSQueue = ScratchPathBFSQueue;   // per-drone reuse (no per-call alloc)
    BFSQueue.Reset();                                // clear previous call's contents, keep capacity
    BFSQueue.Reserve(Total);

    SetPathDist(TargetCell, 0);
    BFSQueue.Add(TargetCell);

    int32 ShortestDist = (StartCell == TargetCell) ? 0 : INT32_MAX;
    int32 MaxDistanceToCompute = INT32_MAX;

    for (int32 BFSIdx = 0; BFSIdx < BFSQueue.Num(); ++BFSIdx)
    {
        const int32 Cur = BFSQueue[BFSIdx];
        const int32 CurDist = DistToTarget[Cur];

        if (CurDist >= MaxDistanceToCompute)
        {
            continue;
        }

        int32 CX = 0;
        int32 CY = 0;
        CellIndexToXY(Cur, CX, CY);

        for (int32 d = 0; d < 8; ++d)
        {
            const int32 NX = CX + DirOffsetsX[d];
            const int32 NY = CY + DirOffsetsY[d];

            if (NX < 0 || NY < 0 || NX >= W || NY >= H)
                continue;

            const int32 NC = NY * W + NX;

            if (NC != StartCell && !IsOwned(NC))
                continue;

            if (DistToTarget[NC] != INT32_MAX)
                continue;

            const int32 NewDist = CurDist + 1;
            SetPathDist(NC, NewDist);

            if (NC == StartCell && ShortestDist == INT32_MAX)
            {
                ShortestDist = NewDist;
                MaxDistanceToCompute = FMath::Max(
                    1,
                    FMath::CeilToInt(static_cast<float>(ShortestDist) *
                                     (1.0f + FMath::Max(0.0f, DetourBudgetFraction))));
            }

            BFSQueue.Add(NC);
        }
    }

    if (ShortestDist == INT32_MAX)
        return false;

    const int32 MaxSteps = FMath::Max(
        1,
        FMath::CeilToInt(static_cast<float>(ShortestDist) *
                         (1.0f + FMath::Max(0.0f, DetourBudgetFraction))));

    // ------------------------------------------------------------
    // Fast planner
    //
    // The previous layered DP is still kept below as an opt-in/reference path,
    // but it is expensive because it expands many cells for many layers and
    // reconstructs parents repeatedly to avoid revisits. In practice most replans
    // only need a good route, not an exhaustive best-detour route.
    //
    // This greedy distance-field planner is O(path_length * 8 * visible_offsets).
    // It always moves through owned cells that can still reach the target inside
    // the step budget, and uses priority/FOV gain only to choose among valid next
    // steps.
    // ------------------------------------------------------------
    if (bUseFastRoutePlanner)
    {
        TArray<bool>& Visited = ScratchPathVisited;   // per-drone reuse (no per-call alloc)
        TArray<int32>& VisitedCells = ScratchPathVisitedCells;
        if (Visited.Num() != Total)
        {
            Visited.Init(false, Total);
            VisitedCells.Reset();
        }
        else
        {
            for (const int32 Cell : VisitedCells)
            {
                if (Cell >= 0 && Cell < Total)
                {
                    Visited[Cell] = false;
                }
            }
            VisitedCells.Reset();
        }
        auto MarkVisited = [&](int32 Cell)
        {
            if (!Visited[Cell])
            {
                Visited[Cell] = true;
                VisitedCells.Add(Cell);
            }
        };
        MarkVisited(StartCell);

        int32 Cur = StartCell;
        int32 StepsUsed = 0;
        CurrentPath.Reset();
        CurrentPath.Reserve(ShortestDist);

        while (Cur != TargetCell && StepsUsed < MaxSteps)
        {
            int32 CX = 0;
            int32 CY = 0;
            CellIndexToXY(Cur, CX, CY);

            int32 BestNext = INDEX_NONE;
            int32 BestDir = 0;
            float BestScore = -FLT_MAX;
            int32 BestDist = INT32_MAX;

            for (int32 Dout = 0; Dout < 8; ++Dout)
            {
                const int32 NX = CX + DirOffsetsX[Dout];
                const int32 NY = CY + DirOffsetsY[Dout];

                if (NX < 0 || NY < 0 || NX >= W || NY >= H)
                    continue;

                const int32 NCell = NY * W + NX;
                if (!IsOwned(NCell))
                    continue;

                if (Visited[NCell] && NCell != TargetCell)
                    continue;

                const int32 NDist = DistToTarget[NCell];
                if (NDist == INT32_MAX)
                    continue;

                const int32 RemainingAfterThis = MaxSteps - (StepsUsed + 1);
                if (NDist > RemainingAfterThis)
                    continue;

                float RouteReward = 0.0f;
                if (bTargetCellScoring)
                {
                    if (NCell == TargetCell)
                    {
                        RouteReward = ReadPriority(TargetCell);
                    }
                }
                else
                {
                    for (const FIntPoint& Off : FOVOffsets[Dout])
                    {
                        const int32 OX = NX + Off.X;
                        const int32 OY = NY + Off.Y;
                        if (OX < 0 || OY < 0 || OX >= W || OY >= H)
                            continue;
                        RouteReward += ReadPriority(OY * W + OX);
                    }
                }

                // Distance dominates to guarantee progress. Priority/FOV gain decides among
                // similarly good moves and allows small detours when the budget permits them.
                const float Score =
                    -100000.0f * static_cast<float>(NDist) +
                    (bTargetCellScoring
                        ? RouteReward
                        : ReadPriority(NCell) + FastRouteFOVGainWeight * RouteReward);

                if (Score > BestScore ||
                    (FMath::IsNearlyEqual(Score, BestScore, 1e-4f) && NDist < BestDist) ||
                    (FMath::IsNearlyEqual(Score, BestScore, 1e-4f) && NDist == BestDist && NCell < BestNext))
                {
                    BestScore = Score;
                    BestDist = NDist;
                    BestNext = NCell;
                    BestDir = Dout;
                }
            }

            if (BestNext == INDEX_NONE)
                break;

            CurrentPath.Add(BestNext);
            MarkVisited(BestNext);
            Cur = BestNext;
            CurrentFacingDir = BestDir;
            ++StepsUsed;
        }

        if (Cur == TargetCell)
            return true;

        // If the greedy route gets boxed in, fall through to the robust DP/fallback
        // implementation below. This should be rare unless the territory is highly concave.
        CurrentPath.Empty();
    }

    // ------------------------------------------------------------
    // Precompute directional FOV gain per cell and arrival direction
    // ------------------------------------------------------------
    TArray<float>& CellDirGain = ScratchPathCellDirGain;
    CellDirGain.SetNumUninitialized(Total * 8);

    for (int32 Cell = 0; Cell < Total; ++Cell)
    {
        if (!IsOwned(Cell))
        {
            for (int32 d = 0; d < 8; ++d)
                CellDirGain[Cell * 8 + d] = -FLT_MAX;

            continue;
        }

        int32 CX = 0;
        int32 CY = 0;
        CellIndexToXY(Cell, CX, CY);

        for (int32 d = 0; d < 8; ++d)
        {
            float Gain = 0.0f;
            if (bTargetCellScoring)
            {
                if (Cell == TargetCell)
                {
                    Gain = ReadPriority(TargetCell);
                }
            }
            else
            {
                for (const FIntPoint& Off : FOVOffsets[d])
                {
                    const int32 OX = CX + Off.X;
                    const int32 OY = CY + Off.Y;

                    if (OX < 0 || OY < 0 || OX >= W || OY >= H)
                        continue;

                    Gain += ReadPriority(OY * W + OX);
                }
            }

            CellDirGain[Cell * 8 + d] = Gain;
        }
    }

    // ------------------------------------------------------------
    // Layered DP with Manhattan budget
    //
    // Important:
    // A candidate transition is rejected if V already exists in the
    // current reconstructed path to U. This prevents planned paths like:
    //
    // A -> B -> A -> B -> ...
    //
    // while still allowing longer useful detours toward the target.
    // ------------------------------------------------------------
    TArray<float>& PrevGain = ScratchPathPrevGain;
    TArray<float>& CurrGain = ScratchPathCurrGain;

    PrevGain.Init(-FLT_MAX, Total);
    CurrGain.Init(-FLT_MAX, Total);

    TArray<int32>& ActivePrev = ScratchPathActivePrev;
    TArray<int32>& ActiveCurr = ScratchPathActiveCurr;

    ActivePrev.Reset();
    ActiveCurr.Reset();
    ActivePrev.Reserve(Total);
    ActiveCurr.Reserve(Total);

    ActivePrev.Add(StartCell);
    PrevGain[StartCell] = 0.0f;

    TArray<TArray<int32>>& ParentLayers = ScratchPathParentLayers;
    if (ParentLayers.Num() < MaxSteps + 1)
    {
        ParentLayers.SetNum(MaxSteps + 1);
    }

    ParentLayers[0].Init(INDEX_NONE, Total);
    ParentLayers[0][StartCell] = StartCell;

    constexpr float GainEpsilon = 1e-6f;
    constexpr float StepPenalty = 0.01f;

    int32 BestTargetStep = INDEX_NONE;
    float BestTargetGain = -FLT_MAX;

    auto PathAlreadyContainsCell =
        [&](int32 Step, int32 CellAtStep, int32 QueryCell) -> bool
    {
        int32 TraceCell = CellAtStep;

        for (int32 TraceStep = Step; TraceStep >= 0; --TraceStep)
        {
            if (TraceCell == QueryCell)
                return true;

            if (TraceStep == 0)
                break;

            if (!ParentLayers.IsValidIndex(TraceStep))
                break;

            if (!ParentLayers[TraceStep].IsValidIndex(TraceCell))
                break;

            const int32 Prev =
                ParentLayers[TraceStep][TraceCell];

            if (Prev == INDEX_NONE)
                break;

            if (Prev == TraceCell)
                break;

            TraceCell = Prev;
        }

        return false;
    };

    for (int32 Step = 1; Step <= MaxSteps; ++Step)
    {
        CurrGain.Init(-FLT_MAX, Total);
        ActiveCurr.Reset();

        ParentLayers[Step].Init(INDEX_NONE, Total);

        for (const int32 U : ActivePrev)
        {
            const float UGain = PrevGain[U];

            if (UGain <= -FLT_MAX * 0.5f)
                continue;

            int32 UX = 0;
            int32 UY = 0;
            CellIndexToXY(U, UX, UY);

            for (int32 Dout = 0; Dout < 8; ++Dout)
            {
                const int32 VX = UX + DirOffsetsX[Dout];
                const int32 VY = UY + DirOffsetsY[Dout];

                if (VX < 0 || VY < 0 || VX >= W || VY >= H)
                    continue;

                const int32 V = VY * W + VX;

                if (!IsOwned(V))
                    continue;

                // Do not allow this planned path to revisit a cell.
                if (PathAlreadyContainsCell(Step - 1, U, V))
                    continue;

                // Must still be possible to reach the target within the remaining budget.
                const int32 VDist = DistToTarget[V];
                if (VDist == INT32_MAX)
                    continue;

                const int32 RemainingAfterThis =
                    MaxSteps - Step;

                if (VDist > RemainingAfterThis)
                    continue;

                const float RawStepGain =
                    CellDirGain[V * 8 + Dout];

                if (RawStepGain <= -FLT_MAX * 0.5f)
                    continue;

                // Small step penalty discourages spending the whole budget unless it is useful.
                const float StepGain =
                    RawStepGain - StepPenalty * static_cast<float>(Step);

                const float NewGain =
                    UGain + StepGain;

                if (NewGain > CurrGain[V] + GainEpsilon)
                {
                    if (CurrGain[V] <= -FLT_MAX * 0.5f)
                        ActiveCurr.Add(V);

                    CurrGain[V] = NewGain;
                    ParentLayers[Step][V] = U;
                }
                else if (FMath::IsNearlyEqual(NewGain, CurrGain[V], GainEpsilon))
                {
                    const int32 ExistingParent =
                        ParentLayers[Step][V];

                    if (ExistingParent == INDEX_NONE || U < ExistingParent)
                    {
                        ParentLayers[Step][V] = U;
                    }
                }
            }
        }

        if (CurrGain[TargetCell] > -FLT_MAX * 0.5f)
        {
            if (BestTargetStep == INDEX_NONE ||
                CurrGain[TargetCell] > BestTargetGain + GainEpsilon ||
                (FMath::IsNearlyEqual(CurrGain[TargetCell], BestTargetGain, GainEpsilon) &&
                 Step < BestTargetStep))
            {
                BestTargetStep = Step;
                BestTargetGain = CurrGain[TargetCell];
            }
        }

        Swap(PrevGain, CurrGain);
        Swap(ActivePrev, ActiveCurr);

        if (ActivePrev.IsEmpty())
            break;
    }

    // ------------------------------------------------------------
    // Reconstruct best budgeted no-repeat path
    // ------------------------------------------------------------
    if (BestTargetStep != INDEX_NONE)
    {
        TArray<int32>& RevPath = ScratchPathRevPath;
        RevPath.Reset();
        RevPath.Reserve(BestTargetStep);

        int32 Cur = TargetCell;

        bool bValidTrace = true;

        for (int32 Step = BestTargetStep; Step > 0; --Step)
        {
            RevPath.Add(Cur);

            const int32 Prev =
                ParentLayers[Step][Cur];

            if (Prev == INDEX_NONE)
            {
                bValidTrace = false;
                break;
            }

            Cur = Prev;
        }

        if (bValidTrace && Cur == StartCell)
        {
            CurrentPath.Reserve(RevPath.Num());

            for (int32 i = RevPath.Num() - 1; i >= 0; --i)
                CurrentPath.Add(RevPath[i]);

            return true;
        }
    }

    // ------------------------------------------------------------
    // Fallback: shortest territory-confined path
    // ------------------------------------------------------------
    TArray<int32>& Parent = ScratchPathFallbackParent;
    Parent.Init(-1, Total);

    TArray<int32>& Queue = ScratchPathFallbackQueue;
    Queue.Reset();
    Queue.Reserve(Total);

    Parent[StartCell] = StartCell;
    Queue.Add(StartCell);

    for (int32 QIdx = 0; QIdx < Queue.Num(); ++QIdx)
    {
        const int32 Cur = Queue[QIdx];

        if (Cur == TargetCell)
            break;

        int32 CX = 0;
        int32 CY = 0;
        CellIndexToXY(Cur, CX, CY);

        for (int32 d = 0; d < 8; ++d)
        {
            const int32 NX = CX + DirOffsetsX[d];
            const int32 NY = CY + DirOffsetsY[d];

            if (NX < 0 || NY < 0 || NX >= W || NY >= H)
                continue;

            const int32 NC = NY * W + NX;

            if (NC != TargetCell && !IsOwned(NC))
                continue;

            if (Parent[NC] != -1)
                continue;

            Parent[NC] = Cur;
            Queue.Add(NC);
        }
    }

    if (Parent[TargetCell] == -1)
        return false;

    TArray<int32>& RevPath = ScratchPathRevPath;
    RevPath.Reset();

    for (int32 Cur = TargetCell; Cur != StartCell; Cur = Parent[Cur])
        RevPath.Add(Cur);

    CurrentPath.Reserve(RevPath.Num());

    for (int32 i = RevPath.Num() - 1; i >= 0; --i)
        CurrentPath.Add(RevPath[i]);

    return true;
}

// Simplified movement: always move at full speed, and check arrival after the move.
void ADrone::MoveAlongPath()
{
    bool bMoved = false;
    const FVector NewLocation = MoveAlongPathFromLocation(GetActorLocation(), bMoved, true);
    if (bMoved)
    {
        SetActorLocation(NewLocation);
    }
}

FVector ADrone::MoveAlongPathFromLocation(const FVector& CurrentLocation,
                                          bool& bOutMoved,
                                          bool bUseTerrainAdjustedTarget)
{
    if (CurrentPathIndex >= CurrentPath.Num())
    {
        CurrentPath.Empty();
        CurrentPathIndex = 0;
        bForceReplan = true;
        bOutMoved = false;
        return CurrentLocation;
    }

    const int32 TargetCell = CurrentPath[CurrentPathIndex];
    const FVector TargetLoc = GetCellWorldCenterForPlanning(TargetCell, CurrentLocation.Z, bUseTerrainAdjustedTarget);
    FVector CurrentLoc = CurrentLocation;

    const FVector Delta = TargetLoc - CurrentLoc;
    const float Dist2D = Delta.Size2D();

    if (Dist2D <= KINDA_SMALL_NUMBER)
    {
        ++CurrentPathIndex;
        bOutMoved = true;
        return bUseTerrainAdjustedTarget ? ApplyTerrainFlightAltitude(TargetLoc, true) : TargetLoc;
    }

    const FVector Dir = Delta / Dist2D;

    // Update facing direction from movement direction.
    // Convention:
    // 0 = E, 1 = NE, 2 = N, 3 = NW, 4 = W, 5 = SW, 6 = S, 7 = SE
    {
        const float AX = FMath::Abs(Dir.X);
        const float AY = FMath::Abs(Dir.Y);

        if (AX < 0.25f && AY < 0.25f)
        {
            // Leave CurrentFacingDir unchanged for tiny motion.
        }
        else if (AX >= AY)
        {
            if (Dir.X >= 0.0f)
                CurrentFacingDir = (Dir.Y < 0.0f) ? 1 : (Dir.Y > 0.0f ? 7 : 0); // NE / SE / E
            else
                CurrentFacingDir = (Dir.Y < 0.0f) ? 3 : (Dir.Y > 0.0f ? 5 : 4); // NW / SW / W
        }
        else
        {
            if (Dir.Y < 0.0f)
                CurrentFacingDir = (Dir.X >= 0.0f) ? 1 : 3; // NE / NW
            else
                CurrentFacingDir = (Dir.X >= 0.0f) ? 7 : 5; // SE / SW
        }

        // Pure vertical cases.
        if (AX < 0.25f)
        {
            if (Dir.Y < 0.0f) CurrentFacingDir = 2; // N
            if (Dir.Y > 0.0f) CurrentFacingDir = 6; // S
        }

        // Pure horizontal cases.
        if (AY < 0.25f)
        {
            if (Dir.X > 0.0f) CurrentFacingDir = 0; // E
            if (Dir.X < 0.0f) CurrentFacingDir = 4; // W
        }
    }

    const float Step = FMath::Min(SpeedUUPerTick, Dist2D);

    if (Step >= Dist2D - KINDA_SMALL_NUMBER)
    {
        ++CurrentPathIndex;
        bOutMoved = true;
        return bUseTerrainAdjustedTarget ? ApplyTerrainFlightAltitude(TargetLoc, true) : TargetLoc;
    }
    else
    {
        CurrentLoc += Dir * Step;
        bOutMoved = true;
        return bUseTerrainAdjustedTarget ? ApplyTerrainFlightAltitude(CurrentLoc, true) : CurrentLoc;
    }
}

// ---------------------------------------------------------------------------
//  Replan (global target + path)
// ---------------------------------------------------------------------------
void ADrone::RunGreedyFovReplan(int32 CurrentTick)
{
    // The GPU-local priority flag is the single switch governing both this
    // (sequential) entry and the parallel entry (TickDroneParallelCpu, which always
    // passes false). When the flag is off, both use the deterministic CPU priority
    // fallback and produce identical plans.
    RunGreedyFovReplanFromLocation(CurrentTick, GetActorLocation(), bUseDroneLocalGpuPriority, true);
}

void ADrone::RunGreedyFovReplanFromLocation(int32 CurrentTick,
                                            const FVector& DronePos,
                                            bool bAllowLocalGpuPriority,
                                            bool bUseTerrainAdjustedCenters)
{
    if (!OwnGrid || !SharedGrid || OwnGrid->Width <= 0)
        return;

    const int32 W = OwnGrid->Width;
    const int32 H = OwnGrid->Height;
    const int32 Total = W * H;

    const float CellSize =
        FMath::Max(CellSizeCached, 1.0f);

    // ------------------------------------------------------------
    // Priority cache
    // ------------------------------------------------------------
    // Important: planning never reads the server/global priority field.
    // The drone first tries to consume its own asynchronous GPU priority
    // result, computed from OwnGrid. If that result is not ready, it falls
    // back to the exact same drone-local priority model on the CPU.
    TArray<float>& PriorityCache = ScratchReplanPriority;              // per-drone reuse (no per-replan alloc)
    TArray<float>& NeighborhoodScoreCache = ScratchReplanNeighborhood; // per-drone reuse (no per-replan alloc)
    const bool bUsedLocalGpuPriority =
        bAllowLocalGpuPriority &&
        TryConsumeLocalGpuPriority(PriorityCache, NeighborhoodScoreCache, CurrentTick);

    if (bAllowLocalGpuPriority)
    {
        RequestLocalGpuPriority(CurrentTick);
    }

    auto IsOwned = [&](int32 Cell) -> bool
    {
        return IsOwnedCell(Cell);
    };
    auto ReadPriority = [&](int32 Cell) -> float
    {
        return bUsedLocalGpuPriority
            ? (PriorityCache.IsValidIndex(Cell) ? PriorityCache[Cell] : 0.0f)
            : GetCachedCpuPlanningPriority(Cell, CurrentTick);
    };

    if (!bUsedLocalGpuPriority || NeighborhoodScoreCache.Num() != Total)
    {
        if (NeighborhoodScoreCache.Num() != Total)
        {
            NeighborhoodScoreCache.SetNumUninitialized(Total);
        }

        for (const int32 Idx : OwnedCells)
        {
            if (Idx < 0 || Idx >= Total)
                continue;

            const int32 CX = Idx % W;
            const int32 CY = Idx / W;
            float NeighborhoodScore = 0.0f;

            for (int32 dy = -1; dy <= 1; ++dy)
            {
                const int32 NY = CY + dy;
                if (NY < 0 || NY >= H)
                    continue;

                for (int32 dx = -1; dx <= 1; ++dx)
                {
                    const int32 NX = CX + dx;
                    if (NX < 0 || NX >= W)
                        continue;

                    const int32 NCell = NY * W + NX;
                    if (!IsOwned(NCell))
                        continue;

                    const float Weight = (dx == 0 && dy == 0) ? 1.0f : NeighborPriorityWeight;
                    NeighborhoodScore += Weight * ReadPriority(NCell);
                }
            }

            NeighborhoodScoreCache[Idx] = NeighborhoodScore;
        }
    }

    // ------------------------------------------------------------
    // Current cell
    // ------------------------------------------------------------
    const int32 CellX =
        FMath::Clamp(
            FMath::FloorToInt(
                (DronePos.X - GridOriginCached.X) / CellSize),
            0,
            W - 1);

    const int32 CellY =
        FMath::Clamp(
            FMath::FloorToInt(
                (DronePos.Y - GridOriginCached.Y) / CellSize),
            0,
            H - 1);

    const int32 StartCell =
        CellY * W + CellX;

    // ------------------------------------------------------------
    // If outside owned territory:
    // move toward nearest owned cell immediately.
    // ------------------------------------------------------------
    if (!IsOwned(StartCell))
    {
        int32 BestOwned = INDEX_NONE;
        float BestDistSq = FLT_MAX;

        for (const int32 Cell : OwnedCells)
        {
            const FVector Center =
                GetCellWorldCenterForPlanning(Cell, DronePos.Z, bUseTerrainAdjustedCenters);

            const float DistSq =
                FVector::DistSquared2D(
                    DronePos,
                    Center);

            if (DistSq < BestDistSq)
            {
                BestDistSq = DistSq;
                BestOwned = Cell;
            }
        }

        if (BestOwned != INDEX_NONE)
        {
            CurrentPath.Empty();
            CurrentPath.Add(BestOwned);

            CurrentPathIndex = 0;
            TargetCellIndex = BestOwned;
            CurrentTargetScore = 0.0f;

            LastReplanTick = CurrentTick;

            return;
        }
    }

    // ------------------------------------------------------------
    // Candidate set
    // ------------------------------------------------------------
    if (OwnedCells.Num() == 0)
    {
        TargetCellIndex = INDEX_NONE;
        CurrentTargetScore = 0.0f;

        CurrentPath.Empty();
        CurrentPathIndex = 0;

        return;
    }

    struct FScored
    {
        int32 Idx = INDEX_NONE;
        float PrimaryScore = -FLT_MAX;
        int32 ScoreKey = 0;
        uint32 TieKey = 0;
    };

    auto Quantize4 = [](float Value) -> int32
    {
        return FMath::RoundToInt(Value * 10000.0f);
    };

    auto TieKeyFn =
        [DroneIdx = GetLogicalDroneId()](int32 CellIdx) -> uint32
    {
        constexpr uint32 TieHashPrime = 2654435761u;

        return
            (static_cast<uint32>(CellIdx) ^
            (static_cast<uint32>(DroneIdx) * 0x9E3779B1u))
            * TieHashPrime;
    };

    auto IsBetterCandidate = [](const FScored& A, const FScored& B) -> bool
    {
        if (A.ScoreKey != B.ScoreKey)
            return A.ScoreKey > B.ScoreKey;

        if (A.TieKey != B.TieKey)
            return A.TieKey < B.TieKey;

        return A.Idx < B.Idx;
    };

    const int32 MaxCandidatesToPath =
        FMath::Clamp(TopKPathBonusCandidates, 1, 128);

    TArray<FScored> Scored;
    Scored.Reserve(MaxCandidatesToPath + 1);

    FScored PreviousTargetCandidate;

    // ------------------------------------------------------------
    // Score candidates
    //
    // The expensive 3x3 neighborhood score is produced by
    // DroneLocalPriorityCompute.usf when the GPU cache is available. The CPU
    // fallback above computes the same values only when the async GPU result is
    // not ready yet. We keep only a small top-K set instead of sorting every
    // owned cell on 300x300 grids.
    // ------------------------------------------------------------
    for (const int32 Idx : OwnedCells)
    {
        if (Idx == StartCell || Idx < 0 || Idx >= Total)
            continue;

        const float NeighborhoodScore = NeighborhoodScoreCache.IsValidIndex(Idx)
            ? NeighborhoodScoreCache[Idx]
            : -FLT_MAX;

        if (!FMath::IsFinite(NeighborhoodScore) || NeighborhoodScore <= -FLT_MAX * 0.5f)
            continue;

        FScored S;
        S.Idx = Idx;
        S.PrimaryScore = bUseTargetCellScoring ? ReadPriority(Idx) : NeighborhoodScore;
        S.ScoreKey = Quantize4(S.PrimaryScore);
        S.TieKey = TieKeyFn(Idx);

        if (Idx == TargetCellIndex)
        {
            PreviousTargetCandidate = S;
        }

        // Early-out: once the top-K set is full, a candidate that does not beat
        // the current worst kept (Scored.Last(), the K-th best) cannot enter it,
        // so skip the O(K) insertion scan. Bit-identical to the full scan because
        // IsBetterCandidate is a total order (ties broken by cell index), and the
        // previous-target capture above still runs for every owned cell.
        if (Scored.Num() >= MaxCandidatesToPath && !IsBetterCandidate(S, Scored.Last()))
        {
            continue;
        }

        int32 InsertAt = 0;
        while (InsertAt < Scored.Num() && !IsBetterCandidate(S, Scored[InsertAt]))
        {
            ++InsertAt;
        }

        if (InsertAt < MaxCandidatesToPath)
        {
            Scored.Insert(S, InsertAt);
            if (Scored.Num() > MaxCandidatesToPath)
            {
                Scored.Pop(EAllowShrinking::No);
            }
        }
    }

    if (Scored.Num() == 0)
    {
        TargetCellIndex = INDEX_NONE;
        CurrentTargetScore = 0.0f;
        CurrentPath.Empty();
        CurrentPathIndex = 0;
        return;
    }

    // ------------------------------------------------------------
    // Commitment margin
    // ------------------------------------------------------------
    if (TargetCellIndex != INDEX_NONE && PreviousTargetCandidate.Idx == TargetCellIndex)
    {
        const float BestScore = Scored[0].PrimaryScore;
        const float PrevScore = PreviousTargetCandidate.PrimaryScore;

        if (BestScore - PrevScore < CommitmentMargin)
        {
            int32 ExistingIndex = INDEX_NONE;
            for (int32 i = 0; i < Scored.Num(); ++i)
            {
                if (Scored[i].Idx == TargetCellIndex)
                {
                    ExistingIndex = i;
                    break;
                }
            }

            if (ExistingIndex > 0)
            {
                Scored.RemoveAt(ExistingIndex, 1, EAllowShrinking::No);
                Scored.Insert(PreviousTargetCandidate, 0);
            }
            else if (ExistingIndex == INDEX_NONE)
            {
                Scored.Insert(PreviousTargetCandidate, 0);
                if (Scored.Num() > MaxCandidatesToPath + 1)
                {
                    Scored.Pop(EAllowShrinking::No);
                }
            }
        }
    }

    // ------------------------------------------------------------
    // Try candidates in order
    // ------------------------------------------------------------
    for (const FScored& S : Scored)
    {
        if (ComputePathToTarget(
                StartCell,
                S.Idx,
                bUsedLocalGpuPriority ? &PriorityCache : nullptr,
                CurrentTick,
                !bUsedLocalGpuPriority,
                bUseTargetCellScoring))
        {
            TargetCellIndex =
                S.Idx;

            CurrentTargetScore =
                S.PrimaryScore;

            CurrentPathIndex = 0;

            LastReplanTick =
                CurrentTick;

            return;
        }
    }

    // ------------------------------------------------------------
    // Failed
    // ------------------------------------------------------------
    TargetCellIndex = INDEX_NONE;

    CurrentTargetScore = 0.0f;

    CurrentPath.Empty();
    CurrentPathIndex = 0;
}

bool ADrone::PlanDirectTargetCell(int32 TargetCell, int32 CurrentTick, float Score)
{
    if (!OwnGrid || !SharedGrid || TargetCell < 0 || TargetCell >= OwnGrid->Width * OwnGrid->Height)
    {
        return false;
    }

    const int32 StartCell = GetCurrentCellIndex();
    if (StartCell == INDEX_NONE)
    {
        return false;
    }

    if (!ComputePathToTarget(StartCell, TargetCell, nullptr, CurrentTick, false))
    {
        return false;
    }

    TargetCellIndex = TargetCell;
    CurrentTargetScore = Score;
    CurrentPathIndex = 0;
    LastReplanTick = CurrentTick;
    return true;
}

bool ADrone::PlanOracleTargetCell(int32 TargetCell, int32 CurrentTick, float Score)
{
    if (!OwnGrid || TargetCell < 0 || TargetCell >= OwnGrid->Width * OwnGrid->Height)
    {
        return false;
    }

    TargetCellIndex = TargetCell;
    CurrentTargetScore = Score;
    CurrentPath.Reset(1);
    CurrentPath.Add(TargetCell);
    CurrentPathIndex = 0;
    LastReplanTick = CurrentTick;
    return true;
}

void ADrone::RunRandomReplan(int32 CurrentTick, int32 RunSeed)
{
    RunRandomReplanFromLocation(CurrentTick, GetActorLocation(), RunSeed);
}

void ADrone::RunRandomReplanFromLocation(int32 CurrentTick,
                                         const FVector& DroneWorldPos,
                                         int32 RunSeed)
{
    if (!OwnGrid || OwnedCells.Num() == 0)
    {
        TargetCellIndex = INDEX_NONE;
        CurrentTargetScore = 0.0f;
        CurrentPath.Empty();
        CurrentPathIndex = 0;
        return;
    }

    const int32 StartCell = GetCurrentCellIndexFromLocation(DroneWorldPos);
    if (StartCell == INDEX_NONE)
    {
        TargetCellIndex = INDEX_NONE;
        CurrentTargetScore = 0.0f;
        CurrentPath.Empty();
        CurrentPathIndex = 0;
        return;
    }

    if (!IsOwnedCell(StartCell))
    {
        int32 BestOwned = INDEX_NONE;
        float BestDistSq = FLT_MAX;

        for (const int32 Cell : OwnedCells)
        {
            if (Cell < 0 || Cell >= OwnGrid->Width * OwnGrid->Height)
            {
                continue;
            }

            const FVector Center = GetCellWorldCenterForPlanning(Cell, DroneWorldPos.Z, false);
            const float DistSq = FVector::DistSquared2D(DroneWorldPos, Center);

            if (DistSq < BestDistSq || (FMath::IsNearlyEqual(DistSq, BestDistSq) && Cell < BestOwned))
            {
                BestDistSq = DistSq;
                BestOwned = Cell;
            }
        }

        if (BestOwned != INDEX_NONE)
        {
            CurrentPath.Empty();
            CurrentPath.Add(BestOwned);
            CurrentPathIndex = 0;
            TargetCellIndex = BestOwned;
            CurrentTargetScore = 0.0f;
            LastReplanTick = CurrentTick;
            return;
        }

        TargetCellIndex = INDEX_NONE;
        CurrentTargetScore = 0.0f;
        CurrentPath.Empty();
        CurrentPathIndex = 0;
        return;
    }

    const int32 W = OwnGrid->Width;
    const int32 H = OwnGrid->Height;
    const int32 Total = W * H;

    TArray<int32>& VisitMarker = ScratchRandomVisitMarker;
    TArray<int32>& RandomVisitedCells = ScratchRandomVisitedCells;
    if (VisitMarker.Num() != Total)
    {
        VisitMarker.Init(0, Total);
        RandomVisitedCells.Reset();
    }
    else
    {
        for (const int32 Cell : RandomVisitedCells)
        {
            if (Cell >= 0 && Cell < Total)
            {
                VisitMarker[Cell] = 0;
            }
        }
        RandomVisitedCells.Reset();
    }
    auto MarkRandomVisited = [&](int32 Cell)
    {
        if (VisitMarker[Cell] == 0)
        {
            VisitMarker[Cell] = 1;
            RandomVisitedCells.Add(Cell);
        }
    };

    TArray<int32>& Queue = ScratchPathBFSQueue;
    Queue.Reset();
    Queue.Reserve(OwnedCells.Num() + 1);
    MarkRandomVisited(StartCell);
    Queue.Add(StartCell);

    TArray<int32>& Candidates = ScratchRandomCandidates;
    Candidates.Reset(OwnedCells.Num());

    for (int32 QueueIdx = 0; QueueIdx < Queue.Num(); ++QueueIdx)
    {
        const int32 Cur = Queue[QueueIdx];
        if (Cur != StartCell && IsOwnedCell(Cur))
        {
            Candidates.Add(Cur);
        }

        int32 CX = 0;
        int32 CY = 0;
        CellIndexToXY(Cur, CX, CY);
        for (int32 Dir = 0; Dir < 8; ++Dir)
        {
            const int32 NX = CX + DirOffsetsX[Dir];
            const int32 NY = CY + DirOffsetsY[Dir];
            if (NX < 0 || NY < 0 || NX >= W || NY >= H)
            {
                continue;
            }

            const int32 NC = NY * W + NX;
            if (VisitMarker[NC] != 0)
            {
                continue;
            }
            if (NC != StartCell && !IsOwnedCell(NC))
            {
                continue;
            }

            MarkRandomVisited(NC);
            Queue.Add(NC);
        }
    }

    if (Candidates.Num() == 0 && IsOwnedCell(StartCell))
    {
        Candidates.Add(StartCell);
    }

    if (Candidates.Num() == 0)
    {
        TargetCellIndex = INDEX_NONE;
        CurrentTargetScore = 0.0f;
        CurrentPath.Empty();
        CurrentPathIndex = 0;
        return;
    }

    uint32 Seed = static_cast<uint32>(RunSeed);
    Seed ^= static_cast<uint32>(GetLogicalDroneId() + 1) * 0x9E3779B1u;
    Seed ^= static_cast<uint32>(CurrentTick + 17) * 0x85EBCA6Bu;
    Seed ^= Seed >> 16;
    FRandomStream Stream(static_cast<int32>(Seed & 0x7fffffffu));

    const int32 Target = Candidates[Stream.RandRange(0, Candidates.Num() - 1)];
    if (ComputePathToTarget(StartCell, Target, nullptr, CurrentTick, false))
    {
        TargetCellIndex = Target;
        CurrentTargetScore = 0.0f;
        CurrentPathIndex = 0;
        LastReplanTick = CurrentTick;
        return;
    }

    TargetCellIndex = INDEX_NONE;
    CurrentTargetScore = 0.0f;
    CurrentPath.Empty();
    CurrentPathIndex = 0;
}

// ---------------------------------------------------------------------------
//  Tick
// ---------------------------------------------------------------------------
bool ADrone::SupportsParallelCpuTick() const
{
    return Sensor && Sensor->IsA<UPerfectDroneSensor>();
}

void ADrone::ApplyParallelCpuTickResult(const FDroneParallelTickResult& Result)
{
    if (Result.bDidRun && Result.bHasNewLocation)
    {
        SetActorLocation(ApplyTerrainFlightAltitude(Result.NewLocation, true));
    }
}

void ADrone::SensePerfectFromLocation(int32 CurrentTick,
                                      const FVector& DroneWorldPos,
                                      TArray<FObservation>& OutObservations,
                                      TArray<FIdealSensorWitnessEvent>* OutWitnessEvents) const
{
    OutObservations.Reset();

    if (!SharedGrid || CellSizeCached <= 0.0f)
    {
        return;
    }

    static constexpr int32 DirX[8] = { 1, 1, 0, -1, -1, -1, 0, 1 };
    static constexpr int32 DirY[8] = { 0, -1, -1, -1, 0, 1, 1, 1 };

    const int32 FacingDir = ((CurrentFacingDir % 8) + 8) % 8;
    const FVector2D Forward(static_cast<float>(DirX[FacingDir]), static_cast<float>(DirY[FacingDir]));
    const FVector2D Fwd = Forward.GetSafeNormal();

    const float DroneGX = (DroneWorldPos.X - GridOriginCached.X) / CellSizeCached;
    const float DroneGY = (DroneWorldPos.Y - GridOriginCached.Y) / CellSizeCached;
    const float MaxDistCells = MaxViewDistanceUU / CellSizeCached;
    const float CosHalfFOV = FMath::Cos(FMath::DegreesToRadians(HorizontalFOVDegrees * 0.5f));

    const int32 SearchRadius = FMath::CeilToInt(MaxDistCells) + 2;
    const int32 BaseX = FMath::FloorToInt(DroneGX);
    const int32 BaseY = FMath::FloorToInt(DroneGY);

    const int32 MinX = FMath::Clamp(BaseX - SearchRadius, 0, SharedGrid->Width - 1);
    const int32 MaxX = FMath::Clamp(BaseX + SearchRadius, 0, SharedGrid->Width - 1);
    const int32 MinY = FMath::Clamp(BaseY - SearchRadius, 0, SharedGrid->Height - 1);
    const int32 MaxY = FMath::Clamp(BaseY + SearchRadius, 0, SharedGrid->Height - 1);

    for (int32 Y = MinY; Y <= MaxY; ++Y)
    {
        for (int32 X = MinX; X <= MaxX; ++X)
        {
            const FVector2D CellCenter(static_cast<float>(X) + 0.5f,
                                       static_cast<float>(Y) + 0.5f);
            const FVector2D Delta = CellCenter - FVector2D(DroneGX, DroneGY);
            const float Dist = Delta.Length();

            if (Dist <= KINDA_SMALL_NUMBER || Dist > MaxDistCells)
            {
                continue;
            }

            const FVector2D DirToCell = Delta / Dist;
            if (FVector2D::DotProduct(Fwd, DirToCell) < CosHalfFOV)
            {
                continue;
            }

            const int32 FirstFire = SharedGrid->getFirstFireTick(X, Y);
            const bool bFireHere = (FirstFire >= 0) && (CurrentTick >= FirstFire);
            const int32 NumHumans = (SharedGrid->getRealPriorityAndNumHumans(X, Y) >> 16) & 0xFFFF;
            bool bPersonHere = NumHumans > 0;
            int32 GateWitnessPersonId = INDEX_NONE;

            // INVARIANT (sensor realism): world-space person gating — a POSITIVE
            // person observation requires a person physically inside the view
            // cone, not merely inside an observed cell. This function is the
            // thread-safe DUPLICATE of UPerfectDroneSensor::Sense used by the
            // parallel-CPU path (the one benchmarks actually run); keep the two
            // gates in sync. See PerfectDroneSensor.cpp for the full rationale.
            if (bPersonHere && bWorldSpacePersonGatingCached && PersonGroundTruth)
            {
                bPersonHere = false;
                for (const FHumanSnapshot& S : *PersonGroundTruth)
                {
                    if (S.PersonId == INDEX_NONE ||
                        S.State == EHumanState::Dead)
                        continue;   // matches the per-cell count semantics

                    const int32 PX = FMath::Clamp(
                        static_cast<int32>(S.PosCell.X), 0, SharedGrid->Width - 1);
                    const int32 PY = FMath::Clamp(
                        static_cast<int32>(S.PosCell.Y), 0, SharedGrid->Height - 1);
                    if (PX != X || PY != Y)
                        continue;

                    const FVector2D PDelta(
                        static_cast<float>(S.PosCell.X) - DroneGX,
                        static_cast<float>(S.PosCell.Y) - DroneGY);
                    const float PDist = PDelta.Length();
                    if (PDist <= KINDA_SMALL_NUMBER || PDist > MaxDistCells)
                        continue;
                    if (FVector2D::DotProduct(Fwd, PDelta / PDist) < CosHalfFOV)
                        continue;

                    bPersonHere = true;
                    GateWitnessPersonId = S.PersonId;
                    break;
                }
            }

            // Score-only diagnostic: record only the first truth snapshot that
            // actually makes the existing binary positive gate true. This is
            // not identity recognition and never feeds observations or planning.
            if (OutWitnessEvents && !bExternalPersonSensing &&
                bPersonHere && GateWitnessPersonId != INDEX_NONE)
            {
                FIdealSensorWitnessEvent& Event = OutWitnessEvents->AddDefaulted_GetRef();
                Event.SensingTick = CurrentTick;
                Event.SnapshotTick = CurrentTick - 1;
                Event.DroneIndex = GetLogicalDroneId();
                Event.PersonId = GateWitnessPersonId;
                Event.CellIndex = Y * SharedGrid->Width + X;
            }

            FObservation Obs;
            Obs.bHasPersonEvidence = true;
            Obs.CellIndex = Y * SharedGrid->Width + X;
            Obs.Tick = CurrentTick;
            Obs.FireLogOddsUpdate = bFireHere ? CachedFirePositiveLLR : CachedFireNegativeLLR;
            Obs.PersonLogOddsUpdate = bPersonHere ? CachedPersonPositiveLLR : CachedPersonNegativeLLR;
            OutObservations.Add(Obs);
        }
    }
}

void ADrone::FilterInternalPersonEvidence(TArray<FObservation>& Observations) const
{
    if (bExternalPersonSensing)
    {
        // Filter after sensing so custom sensors retain their call/RNG order.
        // Remove negative as well as positive oracle evidence before either
        // local belief or the server observation buffer can consume it.
        for (FObservation& Observation : Observations)
        {
            Observation.PersonLogOddsUpdate = 0.0f;
            Observation.bHasPersonEvidence = false;
        }
    }
}

void ADrone::TickDroneParallelCpu(int32 CurrentTick,
                                  const FVector& SnapshotLocation,
                                  bool bUseRandomPlanner,
                                  int32 RunSeed,
                                  bool bProfileTiming,
                                  bool bRecordIdealSensorWitnesses,
                                  FDroneParallelTickResult& OutResult)
{
    OutResult = FDroneParallelTickResult();
    FDroneTickProfile Profile;
    const double TotalStart = bProfileTiming ? FPlatformTime::Seconds() : 0.0;
    auto BeginProfilePhase = [bProfileTiming]() -> double
    {
        return bProfileTiming ? FPlatformTime::Seconds() : 0.0;
    };
    auto EndProfilePhase = [bProfileTiming](double StartSeconds) -> float
    {
        return bProfileTiming ? static_cast<float>((FPlatformTime::Seconds() - StartSeconds) * 1000.0) : 0.0f;
    };

    if (!OwnGrid || !SharedGrid)
    {
        LastTickProfile = Profile;
        return;
    }

    double PhaseStart = BeginProfilePhase();
    if (TargetCellIndex != INDEX_NONE && !IsOwnedCell(TargetCellIndex))
    {
        TargetCellIndex = INDEX_NONE;
        CurrentPath.Empty();
        CurrentPathIndex = 0;
        CurrentTargetScore = 0.0f;
        bForceReplan = true;
    }

    if (CurrentPath.Num() > 0)
    {
        for (int32 Cell : CurrentPath)
        {
            if (!IsOwnedCell(Cell))
            {
                CurrentPath.Empty();
                CurrentPathIndex = 0;
                TargetCellIndex = INDEX_NONE;
                CurrentTargetScore = 0.0f;
                bForceReplan = true;
                break;
            }
        }
    }
    Profile.ValidateMs = EndProfilePhase(PhaseStart);

    TArray<FObservation>& Obs = ScratchObservations;
    Obs.Reset();
    PhaseStart = BeginProfilePhase();
    SensePerfectFromLocation(CurrentTick, SnapshotLocation, Obs,
        bRecordIdealSensorWitnesses ? &OutResult.IdealSensorWitnessEvents : nullptr);
    FilterInternalPersonEvidence(Obs);
    Profile.SenseMs = EndProfilePhase(PhaseStart);
    Profile.ObservationCount = Obs.Num();

    PhaseStart = BeginProfilePhase();
    for (const FObservation& O : Obs)
    {
        int32 X, Y;
        CellIndexToXY(O.CellIndex, X, Y);

        constexpr float MinLogOdds = -8.0f;
        constexpr float MaxLogOdds =  8.0f;
        OwnGrid->setFireBelief(
            X, Y,
            FMath::Clamp(OwnGrid->getFireBelief(X, Y) + O.FireLogOddsUpdate,
                         MinLogOdds,
                         MaxLogOdds));
        OwnGrid->setPersonBelief(
            X, Y,
            FMath::Clamp(OwnGrid->getPersonBelief(X, Y) + O.PersonLogOddsUpdate,
                         MinLogOdds,
                         MaxLogOdds));
        OwnGrid->setLastObservedTick(X, Y, CurrentTick);
        OwnGrid->RecordPersonObservation(O);

        if (OwnGrid->getFireBelief(X, Y) > 0.0f && OwnGrid->getFirstFireDetectionTick(X, Y) < 0)
            OwnGrid->setFirstFireDetectionTick(X, Y, CurrentTick);

        if (OwnGrid->getPersonBelief(X, Y) > 0.0f && OwnGrid->getFirstPersonDetectionTick(X, Y) < 0)
            OwnGrid->setFirstPersonDetectionTick(X, Y, CurrentTick);

        PrivateObservationBuffer.Add(O);
    }
    Profile.LocalBeliefMs = EndProfilePhase(PhaseStart);

    PhaseStart = BeginProfilePhase();
    bool bShouldReplan = bForceReplan;
    bForceReplan = false;

    if (CurrentPath.IsEmpty())
        bShouldReplan = true;

    if (!CurrentPath.IsEmpty() && CurrentPathIndex >= CurrentPath.Num())
        bShouldReplan = true;

    if (!bShouldReplan)
    {
        if (TargetCellIndex == INDEX_NONE)
            bShouldReplan = true;
        else if (!IsOwnedCell(TargetCellIndex))
            bShouldReplan = true;
        else if (!bUseRandomPlanner)
        {
            TicksSinceReplan++;
            if (TicksSinceReplan >= ReplanCadenceTicks ||
                (CurrentTick - LastReplanTick) >= FallbackReplanTicks)
            {
                bShouldReplan = true;
            }
        }
    }
    Profile.ReplanDecisionMs = EndProfilePhase(PhaseStart);

    if (bShouldReplan)
    {
        PhaseStart = BeginProfilePhase();
        if (bUseRandomPlanner)
        {
            RunRandomReplanFromLocation(CurrentTick, SnapshotLocation, RunSeed);
        }
        else
        {
            RunGreedyFovReplanFromLocation(CurrentTick, SnapshotLocation, false, false);
        }
        TicksSinceReplan = 0;
        Profile.ReplanMs = EndProfilePhase(PhaseStart);
        Profile.bReplanned = true;
    }

    bool bMoved = false;
    PhaseStart = BeginProfilePhase();
    const FVector NewLocation = MoveAlongPathFromLocation(SnapshotLocation, bMoved, false);
    Profile.MoveMs = EndProfilePhase(PhaseStart);
    Profile.TotalMs = EndProfilePhase(TotalStart);
    Profile.bValid = bProfileTiming;
    OutResult.bDidRun = true;
    OutResult.bHasNewLocation = true;
    OutResult.NewLocation = bMoved ? NewLocation : SnapshotLocation;
    OutResult.Profile = Profile;
    LastTickProfile = Profile;
}

void ADrone::TickDrone(int32 CurrentTick, bool bProfileTiming)
{
    FDroneTickProfile Profile;
    const double TotalStart = bProfileTiming ? FPlatformTime::Seconds() : 0.0;
    auto BeginProfilePhase = [bProfileTiming]() -> double
    {
        return bProfileTiming ? FPlatformTime::Seconds() : 0.0;
    };
    auto EndProfilePhase = [bProfileTiming](double StartSeconds) -> float
    {
        return bProfileTiming ? static_cast<float>((FPlatformTime::Seconds() - StartSeconds) * 1000.0) : 0.0f;
    };

    if (!OwnGrid || !Sensor)
    {
        LastTickProfile = Profile;
        return;
    }

    SetActorLocation(ApplyTerrainFlightAltitude(GetActorLocation(), false));

    const bool bPlannerIsOracle = Planner && Planner->IsA<UOracleTargetPlanner>();

    // Fast ownership lookup for stale-target validation.
    // If the current target is no longer owned, kill the plan immediately.
    double PhaseStart = BeginProfilePhase();
    if (!bPlannerIsOracle && TargetCellIndex != INDEX_NONE && !IsOwnedCell(TargetCellIndex))
    {
        TargetCellIndex = INDEX_NONE;
        CurrentPath.Empty();
        CurrentPathIndex = 0;
        CurrentTargetScore = 0.0f;
        bForceReplan = true;
    }

    // If the current path contains any cell that is no longer owned,
    // discard it before sensing/movement this tick.
    if (!bPlannerIsOracle && CurrentPath.Num() > 0)
    {
        for (int32 Cell : CurrentPath)
        {
            if (!IsOwnedCell(Cell))
            {
                CurrentPath.Empty();
                CurrentPathIndex = 0;
                TargetCellIndex = INDEX_NONE;
                CurrentTargetScore = 0.0f;
                bForceReplan = true;
                break;
            }
        }
    }
    Profile.ValidateMs = EndProfilePhase(PhaseStart);

    // 1. Sense
    TArray<FObservation>& Obs = ScratchObservations;
    Obs.Reset();
    PhaseStart = BeginProfilePhase();
    Sensor->Sense(CurrentTick,
                  SharedGrid,
                  GetActorLocation(),
                  CellSizeCached,
                  GridOriginCached,
                  CurrentFacingDir,
                  HorizontalFOVDegrees,
                  MaxViewDistanceUU,
                  Obs);
    FilterInternalPersonEvidence(Obs);
    Profile.SenseMs = EndProfilePhase(PhaseStart);
    Profile.ObservationCount = Obs.Num();

    // 2. Update local belief
    PhaseStart = BeginProfilePhase();
    for (const FObservation& O : Obs)
    {
        int32 X, Y;
        CellIndexToXY(O.CellIndex, X, Y);

        constexpr float MinLogOdds = -8.0f;
        constexpr float MaxLogOdds =  8.0f;
        OwnGrid->setFireBelief(
            X, Y,
            FMath::Clamp(OwnGrid->getFireBelief(X, Y) + O.FireLogOddsUpdate,
                         MinLogOdds,
                         MaxLogOdds));
        OwnGrid->setPersonBelief(
            X, Y,
            FMath::Clamp(OwnGrid->getPersonBelief(X, Y) + O.PersonLogOddsUpdate,
                         MinLogOdds,
                         MaxLogOdds));
        OwnGrid->setLastObservedTick(X, Y, CurrentTick);
        OwnGrid->RecordPersonObservation(O);

        if (OwnGrid->getFireBelief(X, Y) > 0.0f && OwnGrid->getFirstFireDetectionTick(X, Y) < 0)
            OwnGrid->setFirstFireDetectionTick(X, Y, CurrentTick);

        if (OwnGrid->getPersonBelief(X, Y) > 0.0f && OwnGrid->getFirstPersonDetectionTick(X, Y) < 0)
            OwnGrid->setFirstPersonDetectionTick(X, Y, CurrentTick);

        PrivateObservationBuffer.Add(O);
    }
    Profile.LocalBeliefMs = EndProfilePhase(PhaseStart);

    // 3. Replan triggers
    PhaseStart = BeginProfilePhase();
    bool bShouldReplan = bPlannerIsOracle || bForceReplan;
    bForceReplan = false;
    const bool bPlannerUsesCadenceReplan = !(Planner && Planner->IsA<URandomPlanner>());
    bool bHoldCompletedOracleTarget = false;

    if (CurrentPath.IsEmpty())
    {
        bShouldReplan = true;
    }

    if (!bShouldReplan && !CurrentPath.IsEmpty() && CurrentPathIndex >= CurrentPath.Num())
    {
        const int32 CurrentCell = GetCurrentCellIndex();
        const bool bOracleCanHoldCurrentTarget =
            bPlannerIsOracle &&
            TargetCellIndex != INDEX_NONE &&
            TargetCellIndex == CurrentCell &&
            IsOwnedCell(TargetCellIndex) &&
            LastReplanTick >= 0 &&
            (CurrentTick - LastReplanTick) < ReplanCadenceTicks;

        if (bOracleCanHoldCurrentTarget)
        {
            bHoldCompletedOracleTarget = true;
        }
        else
        {
            bShouldReplan = true;
        }
    }

    if (bHoldCompletedOracleTarget)
    {
        // Oracle v4 sets bShouldReplan before this branch, so it never holds a
        // stale target.  Keep this legacy branch for unchanged non-oracle tick
        // flow and for compatibility with custom planner subclasses.
    }

    if (!bShouldReplan)
    {
        if (TargetCellIndex == INDEX_NONE)
            bShouldReplan = true;
        else
        {
            // If the target drifted outside ownership after a partition update,
            // force a fresh plan immediately.
            if (!bPlannerIsOracle && !IsOwnedCell(TargetCellIndex))
            {
                bShouldReplan = true;
            }
            else if (bPlannerUsesCadenceReplan)
            {
                TicksSinceReplan++;
                if (TicksSinceReplan >= ReplanCadenceTicks ||
                    (CurrentTick - LastReplanTick) >= FallbackReplanTicks)
                {
                    bShouldReplan = true;
                }
            }
        }
    }
    Profile.ReplanDecisionMs = EndProfilePhase(PhaseStart);

    if (bShouldReplan)
    {
        PhaseStart = BeginProfilePhase();
        if (Planner)
        {
            Planner->Replan(*this, CurrentTick);
        }
        else
        {
            RunGreedyFovReplan(CurrentTick);
        }
        TicksSinceReplan = 0;
        Profile.ReplanMs = EndProfilePhase(PhaseStart);
        Profile.bReplanned = true;
    }

    // 4. Move along path
    PhaseStart = BeginProfilePhase();
    if (!bHoldCompletedOracleTarget)
    {
        MoveAlongPath();
    }
    Profile.MoveMs = EndProfilePhase(PhaseStart);
    Profile.TotalMs = EndProfilePhase(TotalStart);
    Profile.bValid = bProfileTiming;
    LastTickProfile = Profile;
}

void ADrone::ApplyExternalObservations(const TArray<FObservation>& Observations, int32 CurrentTick)
{
    if (!OwnGrid)
    {
        return;
    }

    for (const FObservation& O : Observations)
    {
        int32 X, Y;
        CellIndexToXY(O.CellIndex, X, Y);
        if (X < 0 || X >= OwnGrid->Width || Y < 0 || Y >= OwnGrid->Height)
        {
            continue;
        }

        constexpr float MinLogOdds = -8.0f;
        constexpr float MaxLogOdds =  8.0f;
        OwnGrid->setFireBelief(
            X, Y,
            FMath::Clamp(OwnGrid->getFireBelief(X, Y) + O.FireLogOddsUpdate, MinLogOdds, MaxLogOdds));
        OwnGrid->setPersonBelief(
            X, Y,
            FMath::Clamp(OwnGrid->getPersonBelief(X, Y) + O.PersonLogOddsUpdate, MinLogOdds, MaxLogOdds));
        OwnGrid->setLastObservedTick(X, Y, CurrentTick);
        OwnGrid->RecordPersonObservation(O);

        if (OwnGrid->getFireBelief(X, Y) > 0.0f && OwnGrid->getFirstFireDetectionTick(X, Y) < 0)
            OwnGrid->setFirstFireDetectionTick(X, Y, CurrentTick);
        if (OwnGrid->getPersonBelief(X, Y) > 0.0f && OwnGrid->getFirstPersonDetectionTick(X, Y) < 0)
            OwnGrid->setFirstPersonDetectionTick(X, Y, CurrentTick);

    }
}

void ADrone::ConfigureCapture(bool bEnable, int32 ResX, int32 ResY,
                              float CaptureFOVDegrees, float PitchDegrees, bool bSavePng)
{
    if (!bEnable)
    {
        bCaptureEnabled = false;
        if (CaptureComponent)
        {
            CaptureComponent->Deactivate();
        }
        return;
    }

    if (!CaptureComponent)
    {
        CaptureComponent = NewObject<UDroneCaptureComponent>(this, TEXT("DroneCapture"));
        CaptureComponent->RegisterComponent();
        CaptureComponent->AttachToComponent(GetRootComponent(),
            FAttachmentTransformRules::KeepRelativeTransform);
    }

    CaptureComponent->SetRelativeLocation(FVector(CameraForwardOffsetUU, 0.0f, -CameraDownOffsetUU));
    CaptureComponent->HideActorComponents(this, true);
    CaptureComponent->Activate();
    bCaptureEnabled = true;
    CaptureComponent->bSavePngForDebug = bSavePng;
    CaptureComponent->InitCapture(ResX, ResY, CaptureFOVDegrees, PitchDegrees);
}

bool ADrone::HasActiveCapture() const
{
    return bCaptureEnabled && CaptureComponent && CaptureComponent->IsActive();
}

void ADrone::ConfigureViewCamera(float FOVDegrees, float PitchDegrees)
{
    if (!ViewCamera)
    {
        ViewCamera = NewObject<UCameraComponent>(this, TEXT("DroneViewCamera"));
        ViewCamera->RegisterComponent();
        ViewCamera->AttachToComponent(GetRootComponent(),
            FAttachmentTransformRules::KeepRelativeTransform);
    }
    ViewCamera->SetRelativeLocation(FVector(CameraForwardOffsetUU, 0.0f, -CameraDownOffsetUU));
    ViewCamera->SetFieldOfView(FMath::Clamp(FOVDegrees, 5.0f, 170.0f));
    ViewCamera->SetRelativeRotation(FRotator(PitchDegrees, 0.0f, 0.0f));
}

TArray<FObservation> ADrone::ConsumePrivateObservationBuffer()
{
    TArray<FObservation> Out = MoveTemp(PrivateObservationBuffer);
    PrivateObservationBuffer.Empty();
    return Out;
}

void ADrone::AppendPrivateObservationsTo(TArray<FObservation>& OutObservations)
{
    OutObservations.Append(PrivateObservationBuffer);
    PrivateObservationBuffer.Reset();
}
