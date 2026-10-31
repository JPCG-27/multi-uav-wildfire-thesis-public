#pragma once

#include "CoreMinimal.h"
#include "RandomPlanner.h"
#include "FullyRandomPlanner.generated.h"

/** Versioned seeded random walk; no target/path reward or shortest-path search. */
UCLASS()
class THESIS_API UFullyRandomPlanner : public URandomPlanner
{
    GENERATED_BODY()
public:
    virtual void Replan(ADrone& Drone, int32 CurrentTick) override;
};
