#pragma once

#include "CoreMinimal.h"

/** Pure pinhole projection shared by live vision accounting and geometry tests. */
namespace ThesisVisionProjection
{
    inline bool ProjectWorldPointToImage(const FVector& CameraOrigin,
                                        const FVector& CameraForward,
                                        const FVector& CameraRight,
                                        const FVector& CameraUp,
                                        float FOVDegrees,
                                        const FVector& WorldPoint,
                                        float& OutU,
                                        float& OutV)
    {
        OutU = 0.0f;
        OutV = 0.0f;
        const FVector ToPoint = WorldPoint - CameraOrigin;
        const float Forward = FVector::DotProduct(ToPoint, CameraForward);
        if (Forward <= KINDA_SMALL_NUMBER)
        {
            return false;
        }

        const float TanHalfFOV = FMath::Tan(FMath::DegreesToRadians(FOVDegrees * 0.5f));
        if (!FMath::IsFinite(TanHalfFOV) || TanHalfFOV <= KINDA_SMALL_NUMBER)
        {
            return false;
        }

        const float Right = FVector::DotProduct(ToPoint, CameraRight);
        const float Up = FVector::DotProduct(ToPoint, CameraUp);
        OutU = 0.5f * ((Right / (Forward * TanHalfFOV)) + 1.0f);
        OutV = 0.5f * ((-Up / (Forward * TanHalfFOV)) + 1.0f);
        return OutU >= 0.0f && OutU <= 1.0f && OutV >= 0.0f && OutV <= 1.0f;
    }
}
