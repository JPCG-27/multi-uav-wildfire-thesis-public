#pragma once

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"
#include "SimulationHUD.generated.h"

UCLASS()
class THESIS_API ASimulationHUD : public AHUD
{
    GENERATED_BODY()

public:
    virtual void DrawHUD() override;
    virtual void NotifyHitBoxClick(FName BoxName) override;

private:
    void DrawPriorityTextOverlay();
    void DrawViewButtons();
    class ASimulationManager* GetSimManager() const;
};