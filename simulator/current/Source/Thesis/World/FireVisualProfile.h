#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "FireVisualProfile.generated.h"

class UNiagaraSystem;
class UParticleSystem;

/**
 * Presentation-only configuration for wildfire rendering.
 *
 * This asset deliberately contains no simulation inputs. It may change which
 * effects are drawn and how old they look, but it never changes fire truth,
 * observations, beliefs, planning, exposure, partitioning, or simulation time.
 */
UCLASS(BlueprintType)
class THESIS_API UFireVisualProfile : public UPrimaryDataAsset
{
    GENERATED_BODY()

public:
    // Explicit assets win over the manager's fields and the optional auto-loader.
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Assets")
    TObjectPtr<UNiagaraSystem> FlameSystem;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Assets")
    TObjectPtr<UNiagaraSystem> OuterFlameSystem;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Assets")
    TObjectPtr<UNiagaraSystem> EmberSystem;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Assets")
    TObjectPtr<UNiagaraSystem> SmokeSystem;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Assets")
    TObjectPtr<UParticleSystem> FlameSystemCascade;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Assets")
    TObjectPtr<UParticleSystem> SmokeSystemCascade;

    /** Avoids doubling a flame asset merely because no dedicated outer asset exists. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Layers")
    bool bEnableOuterFlameLayer = false;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Layers")
    bool bEnableEmberLayer = true;

    /**
     * Legacy setting retained for saved profiles. Selects delayed rather than
     * ignition-time scorch feedback; it no longer extinguishes active fire.
     */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Lifecycle")
    bool bUseFiniteVisualLifetime = true;

    /** Legacy scorch delay. Zero uses SimulationManager's BurnDurationTicks.
     *  Does not limit flame lifetime while the simulator reports fire. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Lifecycle", meta = (ClampMin = "0"))
    int32 FlameDurationOverrideTicks = 0;

    /** Reserved legacy value; active simulator fire never enters visual smoulder. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Lifecycle", meta = (ClampMin = "0"))
    int32 SmoulderDurationTicks = 30;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Lifecycle", meta = (ClampMin = "0.01", ClampMax = "4.0"))
    float SmoulderSmokeScale = 0.70f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Lifecycle", meta = (ClampMin = "0.0", ClampMax = "4.0"))
    float SmoulderEmberScale = 0.35f;

    /** Maps deterministic simulation ticks to Niagara presentation age. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Timing", meta = (ClampMin = "0.001", ClampMax = "10.0"))
    float VisualSecondsPerTick = 0.10f;

    /** Seek Niagara systems to tick-derived age instead of relying on wall-clock age. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Timing")
    bool bDriveNiagaraAgeFromSimulationTick = true;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Wind")
    FVector2D WindDirectionWorldXY = FVector2D(1.0f, 0.0f);

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Wind", meta = (ClampMin = "0.0", ClampMax = "10000.0"))
    float WindHorizontalSpeedUUPerSecond = 100.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Wind", meta = (ClampMin = "0.0", ClampMax = "10000.0"))
    float SmokeRiseSpeedUUPerSecond = 45.0f;

    // Niagara packs expose different user parameter names. NAME_None disables
    // a binding, so adapting another pack no longer requires C++ edits.
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Niagara Bindings")
    FName SpawnRateParameter = TEXT("User.SpawnRate");

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Niagara Bindings")
    FName SpawnRateScaleParameter = TEXT("User.SpawnRateScale");

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Niagara Bindings")
    FName IntensityParameter = TEXT("User.Intensity");

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Niagara Bindings")
    FName NoiseStrengthParameter = TEXT("User.NoiseStrength");

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Niagara Bindings")
    FName WindParameter = TEXT("User.Wind");
};
