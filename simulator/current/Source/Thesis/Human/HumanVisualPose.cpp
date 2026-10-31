#include "HumanVisualPose.h"

namespace
{
void ComponentPose(const TArray<FTransform>& Local, const TArray<int32>& Parents, TArray<FTransform>& Out)
{
    Out.SetNum(Local.Num());
    for (int32 I = 0; I < Local.Num(); ++I)
    {
        Out[I] = Parents[I] == INDEX_NONE ? Local[I] : Local[I] * Out[Parents[I]];
    }
}
}

const TCHAR* FHumanVisualRig::Name(EHumanVisualPose Pose)
{
    static const TCHAR* Names[] = { TEXT("standing"), TEXT("walking"), TEXT("running"),
        TEXT("waving"), TEXT("crouching"), TEXT("seated"), TEXT("lying") };
    return static_cast<uint8>(Pose) < static_cast<uint8>(EHumanVisualPose::Count)
        ? Names[static_cast<uint8>(Pose)] : TEXT("invalid");
}

bool FHumanVisualRig::Parse(const FString& Value, EHumanVisualPose& OutPose)
{
    for (uint8 I = 0; I < static_cast<uint8>(EHumanVisualPose::Count); ++I)
    {
        if (Value.Equals(Name(static_cast<EHumanVisualPose>(I)), ESearchCase::IgnoreCase))
        {
            OutPose = static_cast<EHumanVisualPose>(I);
            return true;
        }
    }
    return false;
}

void FHumanVisualRig::Initialize(const FReferenceSkeleton& Skeleton)
{
    Reference = Skeleton.GetRefBonePose();
    Parents.SetNum(Reference.Num());
    for (int32 I = 0; I < Parents.Num(); ++I) Parents[I] = Skeleton.GetParentIndex(I);
    auto Find = [&](const TCHAR* Mixamo, const TCHAR* Unreal) -> int32
    {
        for (int32 I = 0; I < Skeleton.GetNum(); ++I)
        {
            const FString Bone = Skeleton.GetBoneName(I).ToString();
            if (Bone.EndsWith(Mixamo) || Bone.Equals(Unreal)) return I;
        }
        return INDEX_NONE;
    };
    UpperArm[0] = Find(TEXT("LeftArm"), TEXT("upperarm_l"));
    UpperArm[1] = Find(TEXT("RightArm"), TEXT("upperarm_r"));
    ForeArm[0] = Find(TEXT("LeftForeArm"), TEXT("lowerarm_l"));
    ForeArm[1] = Find(TEXT("RightForeArm"), TEXT("lowerarm_r"));
    Hand[0] = Find(TEXT("LeftHand"), TEXT("hand_l"));
    Hand[1] = Find(TEXT("RightHand"), TEXT("hand_r"));
    Thigh[0] = Find(TEXT("LeftUpLeg"), TEXT("thigh_l"));
    Thigh[1] = Find(TEXT("RightUpLeg"), TEXT("thigh_r"));
    Calf[0] = Find(TEXT("LeftLeg"), TEXT("calf_l"));
    Calf[1] = Find(TEXT("RightLeg"), TEXT("calf_r"));
    Foot[0] = Find(TEXT("LeftFoot"), TEXT("foot_l"));
    Foot[1] = Find(TEXT("RightFoot"), TEXT("foot_r"));
    TArray<FTransform> Component;
    ComponentPose(Reference, Parents, Component);
    Toe[0] = Find(TEXT("LeftToeBase"), TEXT("ball_l"));
    Toe[1] = Find(TEXT("RightToeBase"), TEXT("ball_r"));
    Head = Find(TEXT("Head"), TEXT("head"));
    if (Foot[0] != INDEX_NONE && Toe[0] != INDEX_NONE)
    {
        Forward = (Component[Toe[0]].GetLocation() - Component[Foot[0]].GetLocation()).GetSafeNormal2D();
        if (Forward.IsNearlyZero()) Forward = FVector::ForwardVector;
    }
    Right = FVector::CrossProduct(FVector::UpVector, Forward).GetSafeNormal();
    ForwardYaw = FMath::RadiansToDegrees(FMath::Atan2(Forward.Y, Forward.X));
    bSupported = true;
    for (int32 Side = 0; Side < 2; ++Side)
    {
        bSupported &= UpperArm[Side] != INDEX_NONE && ForeArm[Side] != INDEX_NONE && Hand[Side] != INDEX_NONE
            && Thigh[Side] != INDEX_NONE && Calf[Side] != INDEX_NONE && Foot[Side] != INDEX_NONE;
    }
}

void FHumanVisualRig::Evaluate(EHumanVisualPose Pose, double SimSeconds, uint32 PersonSeed,
    TArray<FTransform>& OutLocal, FBox& OutBoneBounds) const
{
    OutLocal = Reference;
    TArray<FTransform> Component;
    ComponentPose(OutLocal, Parents, Component);
    auto Aim = [&](int32 Bone, int32 Child, const FVector& Target)
    {
        if (Bone == INDEX_NONE || Child == INDEX_NONE) return;
        const FVector Direction = (Component[Child].GetLocation() - Component[Bone].GetLocation()).GetSafeNormal();
        if (Direction.IsNearlyZero()) return;
        FTransform Changed = Component[Bone];
        Changed.SetRotation((FQuat::FindBetweenNormals(Direction, Target.GetSafeNormal()) * Changed.GetRotation()).GetNormalized());
        const FQuat LocalRotation = Parents[Bone] == INDEX_NONE ? Changed.GetRotation()
            : Changed.GetRelativeTransform(Component[Parents[Bone]]).GetRotation();
        // Rotations only: preserve authored bone lengths/scales exactly, even
        // on imported rigs whose root carries a non-unit conversion scale.
        OutLocal[Bone].SetRotation(LocalRotation.GetNormalized());
        ComponentPose(OutLocal, Parents, Component);
    };
    const double Period = Pose == EHumanVisualPose::Running ? 0.72 : 1.3;
    const double Phase = FMath::Fmod(FMath::Max(0.0, SimSeconds) / Period + (PersonSeed % 997u) / 997.0, 1.0);
    const double Swing = FMath::Sin(Phase * 2.0 * UE_PI);
    const FVector Up = FVector::UpVector;
    for (int32 Side = 0; Side < 2; ++Side)
    {
        const double Sign = Side == 0 ? -1.0 : 1.0;
        const FVector Out = Right * Sign;
        FVector Arm = Out * 0.18 - Up;
        FVector LowerArm = Forward * 0.1 - Up;
        FVector UpperLeg = -Up;
        FVector LowerLeg = -Up;
        if (Pose == EHumanVisualPose::Walking || Pose == EHumanVisualPose::Running)
        {
            const bool bRun = Pose == EHumanVisualPose::Running;
            const double Stride = Swing * Sign * (bRun ? 0.85 : 0.42);
            UpperLeg = Forward * Stride - Up;
            LowerLeg = -Forward * FMath::Max(0.0, -Stride) * (bRun ? 2.0 : 0.7) - Up;
            Arm = -Forward * Stride + Out * 0.12 - Up;
            LowerArm = Forward * (bRun ? 1.8 : 0.25) - Up * (bRun ? 0.2 : 1.0);
        }
        else if (Pose == EHumanVisualPose::Waving && Side == 1)
        {
            Arm = Out * 0.6 + Up * 0.8;
            LowerArm = Out * (0.25 * Swing) + Up;
        }
        else if (Pose == EHumanVisualPose::Crouching || Pose == EHumanVisualPose::Seated)
        {
            const bool bSit = Pose == EHumanVisualPose::Seated;
            UpperLeg = Forward + Out * 0.18 - Up * (bSit ? 0.03 : 0.6);
            LowerLeg = bSit ? Forward * 0.9 - Up * 0.35 : -Forward * 0.65 - Up;
            Arm = Forward * 0.6 + Out * 0.15 - Up;
            LowerArm = Forward - Up * 0.15;
        }
        else if (Pose == EHumanVisualPose::Lying)
        {
            Arm = Out * 0.3 - Up;
            LowerArm = Out * 0.25 - Up;
            UpperLeg = Out * 0.12 - Up;
        }
        Aim(UpperArm[Side], ForeArm[Side], Arm);
        Aim(ForeArm[Side], Hand[Side], LowerArm);
        Aim(Thigh[Side], Calf[Side], UpperLeg);
        Aim(Calf[Side], Foot[Side], LowerLeg);
        Aim(Foot[Side], Toe[Side], Forward);
    }
    OutBoneBounds = FBox(ForceInit);
    // Ignore stationary IK helper/root bones: they would leave a crouched body
    // floating above the ground. These are contact bounds, not visibility labels.
    for (int32 Side = 0; Side < 2; ++Side)
    {
        for (int32 Bone : { UpperArm[Side], ForeArm[Side], Hand[Side], Thigh[Side], Calf[Side], Foot[Side], Toe[Side] })
            if (Bone != INDEX_NONE) OutBoneBounds += Component[Bone].GetLocation();
    }
    if (Head != INDEX_NONE) OutBoneBounds += Component[Head].GetLocation();
}
