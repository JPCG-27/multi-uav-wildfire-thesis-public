#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Components/PrimitiveComponent.h"
#include "DeterministicCaptureEffects.generated.h"

class UMaterialInterface;
class UMaterial;
class UMaterialInstanceDynamic;
class UProceduralMeshComponent;

/**
 * Tick-indexed, presentation-only fire and smoke for deterministic captures.
 * Call Initialize/UpdateAtTick from the owning presentation manager; this
 * component deliberately has no actor tick and never reads world time.
 */
UCLASS(ClassGroup = (Rendering), meta = (BlueprintSpawnableComponent))
class THESIS_API UDeterministicCaptureEffectsComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    UDeterministicCaptureEffectsComponent();

    UPROPERTY(EditAnywhere, Category = "Capture Effects")
    TObjectPtr<UMaterialInterface> FlameMaterial;

    UPROPERTY(EditAnywhere, Category = "Capture Effects")
    TObjectPtr<UMaterialInterface> SmokeMaterial;

    UFUNCTION(BlueprintCallable, Category = "Capture Effects")
    void Initialize(const FVector& FlameWorldLocation, const FVector& SmokeWorldLocation,
        int32 InCellId, int32 InSeed,
        float InFuel, int32 InIgnitionTick, int32 CurrentTick,
        const FVector2D& WindDirection, float WindSpeedUUPerSecond,
        float FlameScale, float SmokeScale, float Intensity,
        int32 SmokeLayerCount, bool bEnableFlame, bool bEnableSmoke);

    UFUNCTION(BlueprintCallable, Category = "Capture Effects")
    void UpdateAtTick(const FVector& FlameWorldLocation, const FVector& SmokeWorldLocation,
        int32 CurrentTick, const FVector2D& WindDirection,
        float WindSpeedUUPerSecond, float FlameScale, float SmokeScale,
        float Intensity, int32 SmokeLayerCount, bool bEnableFlame, bool bEnableSmoke);

    virtual void Deactivate() override;

    UFUNCTION(BlueprintCallable, Category = "Capture Effects")
    void SetVisibleInSceneCaptureOnly(bool bVisibleOnlyInCapture);

    void GetEffectPrimitives(TArray<UPrimitiveComponent*>& OutPrimitives) const;
    virtual void OnComponentDestroyed(bool bDestroyingHierarchy) override;

private:
    UPROPERTY(Transient)
    TObjectPtr<UProceduralMeshComponent> FlameMesh;

    UPROPERTY(Transient)
    TObjectPtr<UProceduralMeshComponent> SmokeMesh;

    UPROPERTY(Transient)
    TObjectPtr<UMaterialInstanceDynamic> FlameMID;

    UPROPERTY(Transient)
    TObjectPtr<UMaterialInstanceDynamic> SmokeMID;

    UPROPERTY(Transient)
    TObjectPtr<UMaterial> GeneratedMaterial;

    FVector FlameLocation = FVector::ZeroVector;
    FVector SmokeLocation = FVector::ZeroVector;
    int32 CellId = INDEX_NONE;
    int32 Seed = 0;
    int32 IgnitionTick = 0;
    int32 ConfiguredSmokeLayerCount = 2;
    float Fuel = 1.0f;
    bool bGeometryCreated = false;
    bool bCaptureOnly = false;

    void EnsureGeometry();
    void ApplyTick(const FVector& FlameWorldLocation, const FVector& SmokeWorldLocation,
        int32 CurrentTick, const FVector2D& WindDirection,
        float WindSpeedUUPerSecond, float FlameScale, float SmokeScale,
        float Intensity, int32 SmokeLayerCount, bool bEnableFlame, bool bEnableSmoke);
};
