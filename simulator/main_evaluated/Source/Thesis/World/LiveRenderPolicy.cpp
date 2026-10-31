#include "LiveRenderPolicy.h"
#include "SimulationManager.h"
#include "HAL/IConsoleManager.h"

namespace LiveRenderPolicy
{
    namespace
    {
        // r.Nanite recreates scene proxies globally. Coordinate all PIE worlds:
        // a sensor world always takes precedence over a presentation-only world.
        TArray<TWeakObjectPtr<ASimulationManager>> Managers;
        TOptional<int32> OriginalNanite;
        TOptional<int32> OriginalProxyDelay;
        uint32 LastDecisionHash = MAX_uint32;
    }

    void Update()
    {
        check(IsInGameThread());
        IConsoleVariable* Nanite = IConsoleManager::Get().FindConsoleVariable(TEXT("r.Nanite"));
        if (!Nanite || !OriginalNanite.IsSet()) return;
        Managers.RemoveAll([](const auto& Entry) { return !Entry.IsValid(); });
        bool bSensorWorld = false, bLiveRequested = false, bImmediateProxies = false;
        uint32 DecisionHash = HashCombine(GetTypeHash(Nanite->GetInt()),
            HashCombine(GetTypeHash(OriginalNanite.GetValue()), GetTypeHash(Managers.Num())));
        for (const auto& Entry : Managers)
        {
            const ASimulationManager* M = Entry.Get();
            if (!M) continue;
            const bool bEligible = CanUseNanite(M->bUseNaniteForLiveView, M->bEnableDroneCaptures,
                M->bSuppressBenchmarkViewport);
            bLiveRequested |= bEligible;
            bSensorWorld |= M->bEnableDroneCaptures;
            bImmediateProxies |= M->bEnableDroneCaptures && M->CameraRenderProfile == TEXT("fast_shader_v3");
            const uint32 Flags = (M->bUseNaniteForLiveView ? 1u : 0u) |
                (M->bEnableDroneCaptures ? 2u : 0u) |
                (M->bEnableVisionBridge ? 4u : 0u) |
                (M->bEnableYoloDatasetExport ? 8u : 0u) |
                (M->bSuppressBenchmarkViewport ? 16u : 0u);
            DecisionHash = HashCombine(DecisionHash, GetTypeHash(M->GetFName()));
            DecisionHash = HashCombine(DecisionHash, GetTypeHash(M->GetWorld() ? M->GetWorld()->GetFName() : NAME_None));
            DecisionHash = HashCombine(DecisionHash, GetTypeHash(Flags));
        }
        // V3 must block on required pipelines rather than omit unready meshes
        // from initial captures. Coordinate this global setting across worlds.
        if (OriginalProxyDelay.IsSet())
            if (IConsoleVariable* ProxyDelay = IConsoleManager::Get().FindConsoleVariable(TEXT("r.PSOPrecache.ProxyCreationWhenPSOReady")))
            {
                const int32 Value = bImmediateProxies ? 0 : OriginalProxyDelay.GetValue();
                if (ProxyDelay->GetInt() != Value)
                    ProxyDelay->Set(Value, static_cast<EConsoleVariableFlags>(ProxyDelay->GetFlags() & ECVF_SetByMask));
            }
        const int32 Desired = Managers.IsEmpty() ? OriginalNanite.GetValue() : (bLiveRequested && !bSensorWorld ? 1 : 0);
        const int32 Before = Nanite->GetInt();
        if (Nanite->GetInt() != Desired)
        {
            Nanite->SetWithCurrentPriority(Desired);
            const TCHAR* Reason = Managers.IsEmpty() ? TEXT("restored editor setting") :
                bSensorWorld ? TEXT("deterministic sensor profile") :
                Desired ? TEXT("live presentation profile") : TEXT("Nanite disabled by manager settings");
            UE_LOG(LogTemp, Display, TEXT("Thesis rendering: Nanite=%d (%s)."), Desired,
                Reason);
        }
        DecisionHash = HashCombine(DecisionHash, GetTypeHash(Desired));
        if (DecisionHash != LastDecisionHash)
        {
            TArray<FString> ManagerStates;
            for (const auto& Entry : Managers)
            {
                const ASimulationManager* M = Entry.Get();
                if (M)
                {
                    const bool bEligible = CanUseNanite(M->bUseNaniteForLiveView, M->bEnableDroneCaptures,
                        M->bSuppressBenchmarkViewport);
                    ManagerStates.Add(FString::Printf(TEXT("%s[request=%d captures=%d vision=%d dataset=%d viewport_suppressed=%d eligible=%d]"),
                        *M->GetPathName(), M->bUseNaniteForLiveView, M->bEnableDroneCaptures,
                        M->bEnableVisionBridge, M->bEnableYoloDatasetExport, M->bSuppressBenchmarkViewport, bEligible));
                }
            }
            UE_LOG(LogTemp, Display,
                TEXT("Thesis r.Nanite policy: current=%d desired=%d after=%d flags=0x%08x original=%d sensor_world=%d live_requested=%d managers={%s}"),
                Before, Desired, Nanite->GetInt(), static_cast<uint32>(Nanite->GetFlags()),
                OriginalNanite.GetValue(), bSensorWorld, bLiveRequested, *FString::Join(ManagerStates, TEXT(", ")));
            LastDecisionHash = DecisionHash;
        }
        if (Managers.IsEmpty())
        {
            OriginalNanite.Reset();
            OriginalProxyDelay.Reset();
            LastDecisionHash = MAX_uint32;
        }
    }

    void Register(ASimulationManager* Manager)
    {
        check(IsInGameThread());
        if (!OriginalNanite.IsSet())
            if (IConsoleVariable* Nanite = IConsoleManager::Get().FindConsoleVariable(TEXT("r.Nanite")))
                OriginalNanite = Nanite->GetInt();
        if (!OriginalProxyDelay.IsSet())
            if (IConsoleVariable* ProxyDelay = IConsoleManager::Get().FindConsoleVariable(TEXT("r.PSOPrecache.ProxyCreationWhenPSOReady")))
                OriginalProxyDelay = ProxyDelay->GetInt();
        Managers.AddUnique(Manager);
        UE_LOG(LogTemp, Display,
            TEXT("Thesis render policy register: manager=%s world=%s request=%d captures=%d vision=%d dataset=%d viewport_suppressed=%d"),
            Manager ? *Manager->GetPathName() : TEXT("<null>"),
            Manager && Manager->GetWorld() ? *Manager->GetWorld()->GetPathName() : TEXT("<no world>"),
            Manager ? Manager->bUseNaniteForLiveView : 0,
            Manager ? Manager->bEnableDroneCaptures : 0,
            Manager ? Manager->bEnableVisionBridge : 0,
            Manager ? Manager->bEnableYoloDatasetExport : 0,
            Manager ? Manager->bSuppressBenchmarkViewport : 0);
        Update();
    }

    void Unregister(ASimulationManager* Manager)
    {
        check(IsInGameThread());
        UE_LOG(LogTemp, Display, TEXT("Thesis render policy unregister: manager=%s"),
            Manager ? *Manager->GetPathName() : TEXT("<null>"));
        Managers.Remove(Manager);
        Update();
    }
}
