#pragma once

#include "CoreMinimal.h"

namespace ThesisPriority
{
    // ---------- Planning weights (drone & server priority) ----------
    inline constexpr float WeightStaleness          = 1.0f;
    inline constexpr float WeightDanger             = 0.35f;
    inline constexpr float WeightPerson             = 0.30f;
    inline constexpr float StalenessSaturationTicks = 300.0f;
    inline constexpr float ProbabilityScale         = 10.0f;
    inline constexpr float MonitoringUrgencyTau     = 50.0f;

    inline constexpr float UnseenExplorationBonus       = 3.0f;

    // Drone-local fire zones should become urgent much faster than ordinary
    // stale/unknown cells. Person evidence keeps the slower discovery curve.
    inline constexpr float FirePriorityRampMultiplier     = 3.0f;
    inline constexpr float FireMonitoringUrgencyTau       = 12.0f;
    inline constexpr float FireMinimumUrgency             = 0.35f;
    inline constexpr int32 FireDangerDiscoveryDelayTicks  = 3;
    inline constexpr int32 FireDangerDiscoveryRampTicks   = 12;

    inline constexpr int32 DroneDangerDiscoveryDelayTicks = 30;
    inline constexpr int32 DroneDangerDiscoveryRampTicks  = 50;

    // ---------- Load‑balancing weights (fire‑heavy, CAPPED staleness for partition) ----------
    inline constexpr float LoadWeightStaleness = 1.0f;    // original staleness weight
    inline constexpr float LoadWeightDanger    = 350.0f;  // ~1000× original danger weight (0.35 → 350)
    inline constexpr float LoadWeightPerson    = 300.0f;  // ~1000× original person weight (0.30 → 300)

    // Adaptive-zone partition (region growing on zone adjacency graph)
    inline constexpr float BackgroundTileFraction      = 1.0f / 15.0f;
    inline constexpr float MaxFireZoneFraction         = 1.0f / 5.0f;
    inline constexpr float ConfirmedDetectionThreshold = 0.5f * 350.0f * 10.0f;
    inline constexpr int32 WeightConfirmed             = 5;
    inline constexpr int32 WeightExpected              = 3;
    inline constexpr int32 WeightNormal                = 1;

    FORCEINLINE float LogOddsToProb(float L)
    {
        return 1.0f / (1.0f + FMath::Exp(-L));
    }

    FORCEINLINE float PositiveEvidence(float LogOdds)
    {
        return FMath::Max(0.0f, 2.0f * LogOddsToProb(LogOdds) - 1.0f);
    }

    FORCEINLINE float MonitoringUrgency(float StalenessTicks)
    {
        return 1.0f - FMath::Exp(-StalenessTicks / MonitoringUrgencyTau);
    }

    FORCEINLINE float DiscoveryGateWithParams(
        int32 FirstDetectionTick,
        int32 CurrentTick,
        int32 DelayTicks,
        int32 RampTicks)
    {
        if (FirstDetectionTick < 0) return 0.0f;
        const float Age = static_cast<float>(CurrentTick - FirstDetectionTick);
        if (Age < static_cast<float>(DelayTicks)) return 0.0f;
        const float Ramp = static_cast<float>(FMath::Max(RampTicks, 1));
        return FMath::Clamp((Age - static_cast<float>(DelayTicks)) / Ramp, 0.0f, 1.0f);
    }

    FORCEINLINE float DiscoveryGate(int32 FirstDetectionTick, int32 CurrentTick)
    {
        return DiscoveryGateWithParams(
            FirstDetectionTick,
            CurrentTick,
            DroneDangerDiscoveryDelayTicks,
            DroneDangerDiscoveryRampTicks);
    }

    // ---- Server static priority (no urgency/gate) ----
    FORCEINLINE float ComputeStaticPriority(float FireLogOdds, float PersonLogOdds)
    {
        const float DangerTerm = PositiveEvidence(FireLogOdds)   * ProbabilityScale;
        const float PersonTerm = PositiveEvidence(PersonLogOdds) * ProbabilityScale;
        return WeightDanger * DangerTerm + WeightPerson * PersonTerm;
    }

    // ---- Partition load: CAPPED staleness (unchanged) ----
    FORCEINLINE float ComputeLoadPriority(float StalenessTicks,
                                          float FireLogOdds,
                                          float PersonLogOdds)
    {
        const float StalenessTerm = FMath::Clamp(StalenessTicks / StalenessSaturationTicks, 0.0f, 1.0f);
        const float DangerTerm    = PositiveEvidence(FireLogOdds)   * ProbabilityScale;
        const float PersonTerm    = PositiveEvidence(PersonLogOdds) * ProbabilityScale;

        return LoadWeightStaleness * StalenessTerm
             + LoadWeightDanger    * DangerTerm
             + LoadWeightPerson    * PersonTerm;
    }

    FORCEINLINE float ComputeDetectionLoad(float FireLogOdds, float PersonLogOdds)
    {
        const float DangerTerm = PositiveEvidence(FireLogOdds)   * ProbabilityScale;
        const float PersonTerm = PositiveEvidence(PersonLogOdds) * ProbabilityScale;
        return LoadWeightDanger * DangerTerm + LoadWeightPerson * PersonTerm;
    }

    // ---- Server‑side priority (C++ reference, now uncapped staleness) ----
    FORCEINLINE float ComputePriority(float StalenessTicks,
                                      float FireLogOdds,
                                      float PersonLogOdds)
    {
        // Uncapped staleness (no clamp)
        const float StalenessTerm = FMath::Max(0.0f, StalenessTicks / StalenessSaturationTicks);
        const float Urgency       = MonitoringUrgency(StalenessTicks);
        const float DangerTerm    = PositiveEvidence(FireLogOdds)   * ProbabilityScale * Urgency;
        const float PersonTerm    = PositiveEvidence(PersonLogOdds) * ProbabilityScale * Urgency;
        return WeightStaleness * StalenessTerm
             + WeightDanger    * DangerTerm
             + WeightPerson    * PersonTerm;
    }

    // ---- Drone‑local priority (uncapped staleness, with unseen bonus, discovery gate) ----
    FORCEINLINE float ComputeDronePriority(float StalenessTicks,
                                           int32 LastObservedTick,
                                           int32 FirstFireDetectionTick,
                                           int32 FirstPersonDetectionTick,
                                           int32 CurrentTick,
                                           float FireLogOdds,
                                           float PersonLogOdds)
    {
        // Uncapped staleness term
        float StalenessTerm = FMath::Max(0.0f, StalenessTicks / StalenessSaturationTicks);
        if (LastObservedTick < 0)
            StalenessTerm += UnseenExplorationBonus;

        const float GeneralUrgency = MonitoringUrgency(StalenessTicks);
        const float FireUrgency = FMath::Max(
            FireMinimumUrgency,
            1.0f - FMath::Exp(-StalenessTicks / FMath::Max(FireMonitoringUrgencyTau, 1.0f)));

        const float FireGate = DiscoveryGateWithParams(
            FirstFireDetectionTick,
            CurrentTick,
            FireDangerDiscoveryDelayTicks,
            FireDangerDiscoveryRampTicks);
        const float PersonGate = DiscoveryGate(FirstPersonDetectionTick, CurrentTick);

        const float DangerTerm = PositiveEvidence(FireLogOdds)
            * ProbabilityScale
            * FireUrgency
            * FireGate
            * FirePriorityRampMultiplier;
        const float PersonTerm = PositiveEvidence(PersonLogOdds)
            * ProbabilityScale
            * GeneralUrgency
            * PersonGate;

        return WeightStaleness * StalenessTerm
             + WeightDanger    * DangerTerm
             + WeightPerson    * PersonTerm;
    }
}