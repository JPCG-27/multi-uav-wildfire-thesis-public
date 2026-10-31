#include "SimulationHUD.h"
#include "SimulationManager.h"
#include "Drone.h"
#include "Engine/Canvas.h"
#include "CanvasItem.h"
#include "Engine/Engine.h"
#include "GameFramework/PlayerController.h"

void ASimulationHUD::DrawHUD()
{
    if (const ASimulationManager* Sim = GetSimManager(); Sim && Sim->bSuppressBenchmarkViewport)
    {
        return;
    }
    Super::DrawHUD();
    DrawPriorityTextOverlay();
    DrawViewButtons();
}

void ASimulationHUD::DrawViewButtons()
{
    ASimulationManager* Sim = GetSimManager();
    if (!Sim || !Canvas) return;

    const int32 NumDrones = Sim->RegisteredDrones.Num();
    if (NumDrones == 0) return;

    UFont* Font = GEngine ? GEngine->GetSmallFont() : nullptr;
    if (!Font) return;

    const float BtnH = 28.0f;
    const float Pad = 6.0f;
    const float PrevW = 34.0f, NextW = 34.0f, MainW = 120.0f;
    const float TotalW = PrevW + MainW + NextW + 2.0f * Pad;
    float X = Canvas->SizeX - TotalW - 16.0f;
    const float Y = 16.0f;

    const int32 Viewed = Sim->GetViewedDroneIndex();
    const FString MainLabel = (Viewed == INDEX_NONE)
        ? FString(TEXT("View: Overview"))
        : FString::Printf(TEXT("View: Drone %d"), Viewed);

    auto DrawButton = [&](float BX, float BW, const FString& Label, FName HitName)
    {
        FCanvasTileItem Tile(FVector2D(BX, Y), FVector2D(BW, BtnH),
                             FLinearColor(0.05f, 0.05f, 0.05f, 0.65f));
        Tile.BlendMode = SE_BLEND_Translucent;
        Canvas->DrawItem(Tile);

        FCanvasTextItem Text(FVector2D(BX + BW * 0.5f, Y + BtnH * 0.5f),
                             FText::FromString(Label), Font, FLinearColor::White);
        Text.EnableShadow(FLinearColor::Black);
        Text.bCentreX = true;
        Text.bCentreY = true;
        Canvas->DrawItem(Text);

        AddHitBox(FVector2D(BX, Y), FVector2D(BW, BtnH), HitName, /*bConsumesInput=*/true);
    };

    DrawButton(X, PrevW, TEXT("<"), TEXT("ViewPrevDrone"));
    X += PrevW + Pad;
    DrawButton(X, MainW, MainLabel, TEXT("ViewToggle"));
    X += MainW + Pad;
    DrawButton(X, NextW, TEXT(">"), TEXT("ViewNextDrone"));
}

void ASimulationHUD::NotifyHitBoxClick(FName BoxName)
{
    Super::NotifyHitBoxClick(BoxName);

    ASimulationManager* Sim = GetSimManager();
    if (!Sim) return;

    if (BoxName == TEXT("ViewPrevDrone"))      Sim->ViewPreviousDrone();
    else if (BoxName == TEXT("ViewNextDrone")) Sim->ViewNextDrone();
    else if (BoxName == TEXT("ViewToggle"))    Sim->ToggleDroneView();
}

void ASimulationHUD::DrawPriorityTextOverlay()
{
    ASimulationManager* Sim = GetSimManager();
    if (!Sim) return;

    if (Sim->DebugPrioritySource == EDebugPrioritySource::None) return;

    const int32 Step = FMath::Max(Sim->DebugPriorityDisplayStep, 1);
    const FVector GridOrigin = Sim->GetGridOriginWorld();
    const float CellSize = Sim->CellSizeUU;
    const int32 GridW = Sim->GridWidth;
    const int32 GridH = Sim->GridHeight;
    const int32 CurrentTick = Sim->GetCurrentTick();

    UFont* SmallFont = GEngine->GetSmallFont();
    if (!SmallFont) return;

    APlayerController* PC = GetWorld()->GetFirstPlayerController();
    if (!PC) return;

    for (int32 CY = 0; CY < GridH; CY += Step)
    {
        for (int32 CX = 0; CX < GridW; CX += Step)
        {
            float Priority = 0.0f;

            switch (Sim->DebugPrioritySource)
            {
                case EDebugPrioritySource::GroundTruthPriority:
                    Priority = Sim->ComputeGroundTruthPriority(CX, CY, CurrentTick);
                    break;

                case EDebugPrioritySource::DroneLocalPriority:
                {
                    // Each cell uses its OWNER drone's view (mirrors DrawPriorityHeatmap).
                    const int32 OwnerIdx = Sim->DebugGetOwner(CX, CY);
                    if (OwnerIdx >= 0)
                    {
                        const TArray<TObjectPtr<ADrone>>& Drones = Sim->RegisteredDrones;
                        if (Drones.IsValidIndex(OwnerIdx))
                        {
                            if (ADrone* D = Drones[OwnerIdx].Get())
                            {
                                Priority = D->ComputeCellPriority(CY * GridW + CX, CurrentTick);
                            }
                        }
                    }
                    break;
                }

                case EDebugPrioritySource::FireGroundTruth:
                    if (Sim->IsFireActive(CX, CY, CurrentTick))
                        Priority = 1.0f;
                    else
                        continue;
                    break;

                case EDebugPrioritySource::ServerPriority:
                {
                    const TArray<float>& Cached = Sim->GetCachedServerPriority();
                    if (Cached.Num() == GridW * GridH)
                        Priority = Cached[CY * GridW + CX];
                    break;
                }

                default: break;
            }

            if (Priority <= 0.0f) continue;

            // World position slightly above the heatmap boxes
            FVector WorldCenter(
                GridOrigin.X + (CX + 0.5f * Step) * CellSize,
                GridOrigin.Y + (CY + 0.5f * Step) * CellSize,
                GridOrigin.Z + Sim->DebugPriorityZOffset + 50.0f
            );

            FVector2D ScreenPos;
            if (PC->ProjectWorldLocationToScreen(WorldCenter, ScreenPos, false))
            {
                if (ScreenPos.X < 0 || ScreenPos.X > Canvas->SizeX ||
                    ScreenPos.Y < 0 || ScreenPos.Y > Canvas->SizeY) continue;

                FString Text = FString::Printf(TEXT("%.2f"), Priority);

                FCanvasTextItem TextItem(ScreenPos, FText::FromString(Text), SmallFont, FLinearColor::White);
                TextItem.EnableShadow(FLinearColor::Black);
                TextItem.Scale = FVector2D(0.8f, 0.8f);
                TextItem.bCentreX = true;
                TextItem.bCentreY = true;
                Canvas->DrawItem(TextItem);
            }
        }
    }
}

ASimulationManager* ASimulationHUD::GetSimManager() const
{
    return ASimulationManager::StaticInstance;
}
