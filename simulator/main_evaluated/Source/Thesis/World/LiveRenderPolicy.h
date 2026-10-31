#pragma once
#include "CoreMinimal.h"
class ASimulationManager;

namespace LiveRenderPolicy
{
    inline bool CanUseNanite(bool bRequested, bool bDroneCaptures, bool bSuppressedViewport)
    {
        // Vision bridge and dataset export consume pixels only through the
        // manager's drone-capture path. Their flags alone do not render a view.
        return bRequested && !bDroneCaptures && !bSuppressedViewport;
    }

    void Register(ASimulationManager* Manager);
    void Update();
    void Unregister(ASimulationManager* Manager);
}
