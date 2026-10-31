#include "GpuBeliefManager.h"
#include "PixelBleedSpreadShader.h"
#include "PriorityComputeShader.h"
#include "StalenessUpdateShader.h"
#include "ObservationIngestShader.h"
#include "GridStatsShader.h"
#include "DroneTypes.h"          // for FObservation

#include "RenderGraphBuilder.h"
#include "RenderGraphUtils.h"
#include "RenderingThread.h"
#include "RenderTargetPool.h"
#include "RHICommandList.h"
#include "RHIStaticStates.h"
#include "RHIGPUReadback.h"
#include "RenderCommandFence.h"
#include "GlobalShader.h"
#include "ShaderParameterStruct.h"
#include "DataDrivenShaderPlatformInfo.h"
#include "../World/PriorityModel.h"

namespace
{
    constexpr float kDefaultDecayPerTick = 0.02f;
    constexpr float kDefaultWeights[9] = {
        1.0f / 16.0f, 2.0f / 16.0f, 1.0f / 16.0f,
        2.0f / 16.0f, 4.0f / 16.0f, 2.0f / 16.0f,
        1.0f / 16.0f, 2.0f / 16.0f, 1.0f / 16.0f,
    };

    FPooledRenderTargetDesc MakeR32FDesc(int32 W, int32 H)
    {
        return FPooledRenderTargetDesc::Create2DDesc(
            FIntPoint(W, H), PF_R32_FLOAT, FClearValueBinding::Black,
            TexCreate_None, TexCreate_ShaderResource | TexCreate_UAV, false);
    }
}

void UGpuBeliefManager::Initialize(FSubsystemCollectionBase& Collection)
{
    Super::Initialize(Collection);
}

void UGpuBeliefManager::Deinitialize()
{
    FlushRenderingCommands();

    Width = 0;
    Height = 0;
    bAllocated = false;

    TRefCountPtr<IPooledRenderTarget> Fire0 = MoveTemp(FireBelief[0]);
    TRefCountPtr<IPooledRenderTarget> Fire1 = MoveTemp(FireBelief[1]);
    TRefCountPtr<IPooledRenderTarget> Person0 = MoveTemp(PersonBelief[0]);
    TRefCountPtr<IPooledRenderTarget> Person1 = MoveTemp(PersonBelief[1]);
    TRefCountPtr<IPooledRenderTarget> Stale = MoveTemp(Staleness);
    TRefCountPtr<IPooledRenderTarget> PersonTick = MoveTemp(PersonObservationTick);
    TRefCountPtr<IPooledRenderTarget> Prio = MoveTemp(Priority);
    TUniquePtr<FRHIGPUTextureReadback> PrioReadback = MoveTemp(PriorityReadback);

    ENQUEUE_RENDER_COMMAND(ThesisGpuBeliefRelease)(
        [Fire0, Fire1, Person0, Person1, Stale, PersonTick, Prio,
         PrioReadback = MoveTemp(PrioReadback)](FRHICommandListImmediate&) mutable {});
    FlushRenderingCommands();

    Super::Deinitialize();
}

void UGpuBeliefManager::InitializeTextures(int32 InWidth, int32 InHeight)
{
    if (InWidth <= 0 || InHeight <= 0) return;
    if (Width == InWidth && Height == InHeight && bAllocated) return;

    Width = InWidth;
    Height = InHeight;
    bAllocated = false;
    ReadIndex = 0;
    WriteIndex = 1;
}

void UGpuBeliefManager::ResetTextures()
{
    FlushRenderingCommands();

    bAllocated = false;
    ReadIndex = 0;
    WriteIndex = 1;
    PriorityReadbackTick = -1;
    GridStatsTick = -1;
    GridStatsResultBuffer.SafeRelease();

    TRefCountPtr<IPooledRenderTarget> Fire0 = MoveTemp(FireBelief[0]);
    TRefCountPtr<IPooledRenderTarget> Fire1 = MoveTemp(FireBelief[1]);
    TRefCountPtr<IPooledRenderTarget> Person0 = MoveTemp(PersonBelief[0]);
    TRefCountPtr<IPooledRenderTarget> Person1 = MoveTemp(PersonBelief[1]);
    TRefCountPtr<IPooledRenderTarget> Stale = MoveTemp(Staleness);
    TRefCountPtr<IPooledRenderTarget> PersonTick = MoveTemp(PersonObservationTick);
    TRefCountPtr<IPooledRenderTarget> Prio = MoveTemp(Priority);
    TUniquePtr<FRHIGPUTextureReadback> PrioReadback = MoveTemp(PriorityReadback);

    ENQUEUE_RENDER_COMMAND(ThesisGpuBeliefReset)(
        [Fire0, Fire1, Person0, Person1, Stale, PersonTick, Prio,
         PrioReadback = MoveTemp(PrioReadback)](FRHICommandListImmediate&) mutable {});

    FlushRenderingCommands();
}

void UGpuBeliefManager::EnsureAllocated_RenderThread(FRHICommandListImmediate& RHICmdList)
{
    if (bAllocated || Width <= 0 || Height <= 0) return;

    const FPooledRenderTargetDesc Desc = MakeR32FDesc(Width, Height);

    GRenderTargetPool.FindFreeElement(RHICmdList, Desc, FireBelief[0],   TEXT("Thesis.FireBelief0"));
    GRenderTargetPool.FindFreeElement(RHICmdList, Desc, FireBelief[1],   TEXT("Thesis.FireBelief1"));
    GRenderTargetPool.FindFreeElement(RHICmdList, Desc, PersonBelief[0], TEXT("Thesis.PersonBelief0"));
    GRenderTargetPool.FindFreeElement(RHICmdList, Desc, PersonBelief[1], TEXT("Thesis.PersonBelief1"));
    GRenderTargetPool.FindFreeElement(RHICmdList, Desc, Staleness,       TEXT("Thesis.Staleness"));
    GRenderTargetPool.FindFreeElement(RHICmdList, Desc, Priority,        TEXT("Thesis.Priority"));

    const FPooledRenderTargetDesc PersonTickDesc = FPooledRenderTargetDesc::Create2DDesc(
        FIntPoint(Width, Height), PF_R32_UINT, FClearValueBinding::Black,
        TexCreate_None, TexCreate_ShaderResource | TexCreate_UAV, false);
    GRenderTargetPool.FindFreeElement(RHICmdList, PersonTickDesc, PersonObservationTick, TEXT("Thesis.PersonObservationTick"));
    // Pooled allocations may contain prior-run data.  Every belief input and
    // derived field must begin at zero after a reset, not just the new modality
    // clock, otherwise a later scenario can consume stale evidence.
    FRDGBuilder ClearGraph(RHICmdList);
    FRDGTextureRef Fire0Tex = ClearGraph.RegisterExternalTexture(FireBelief[0]);
    FRDGTextureRef Fire1Tex = ClearGraph.RegisterExternalTexture(FireBelief[1]);
    FRDGTextureRef Person0Tex = ClearGraph.RegisterExternalTexture(PersonBelief[0]);
    FRDGTextureRef Person1Tex = ClearGraph.RegisterExternalTexture(PersonBelief[1]);
    FRDGTextureRef StalenessTex = ClearGraph.RegisterExternalTexture(Staleness);
    FRDGTextureRef PriorityTex = ClearGraph.RegisterExternalTexture(Priority);
    FRDGTextureRef PersonTickTex = ClearGraph.RegisterExternalTexture(PersonObservationTick);

    AddClearUAVPass(ClearGraph, ClearGraph.CreateUAV(FRDGTextureUAVDesc(Fire0Tex)), 0.0f);
    AddClearUAVPass(ClearGraph, ClearGraph.CreateUAV(FRDGTextureUAVDesc(Fire1Tex)), 0.0f);
    AddClearUAVPass(ClearGraph, ClearGraph.CreateUAV(FRDGTextureUAVDesc(Person0Tex)), 0.0f);
    AddClearUAVPass(ClearGraph, ClearGraph.CreateUAV(FRDGTextureUAVDesc(Person1Tex)), 0.0f);
    AddClearUAVPass(ClearGraph, ClearGraph.CreateUAV(FRDGTextureUAVDesc(StalenessTex)), 0.0f);
    AddClearUAVPass(ClearGraph, ClearGraph.CreateUAV(FRDGTextureUAVDesc(PriorityTex)), 0.0f);
    AddClearUAVPass(ClearGraph, ClearGraph.CreateUAV(FRDGTextureUAVDesc(PersonTickTex)), 0u);
    ClearGraph.Execute();
    bAllocated = true;
}

void UGpuBeliefManager::DispatchStalenessUpdate()
{
    if (Width <= 0 || Height <= 0) return;
    const FIntPoint Size(Width, Height);

    ENQUEUE_RENDER_COMMAND(ThesisStalenessUpdate)(
        [this, Size](FRHICommandListImmediate& RHICmdList)
        {
            EnsureAllocated_RenderThread(RHICmdList);
            if (!bAllocated) return;

            FRDGBuilder GraphBuilder(RHICmdList);
            FRDGTextureRef OutTex = GraphBuilder.RegisterExternalTexture(Staleness, TEXT("Thesis.Staleness.UAV"));
            FStalenessUpdateCS::FParameters* Params = GraphBuilder.AllocParameters<FStalenessUpdateCS::FParameters>();
            Params->StalenessOut = GraphBuilder.CreateUAV(FRDGTextureUAVDesc(OutTex));
            Params->TextureSize = Size;

            TShaderMapRef<FStalenessUpdateCS> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
            FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("Thesis.StalenessUpdate"), Shader, Params,
                                         FComputeShaderUtils::GetGroupCount(Size, FIntPoint(8, 8)));
            GraphBuilder.Execute();
        });
}

void UGpuBeliefManager::DispatchSpread()
{
    if (Width <= 0 || Height <= 0) return;
    const int32 ReadIdx = ReadIndex;
    const int32 WriteIdx = WriteIndex;
    const FIntPoint Size(Width, Height);

    ENQUEUE_RENDER_COMMAND(ThesisSpread)(
        [this, ReadIdx, WriteIdx, Size](FRHICommandListImmediate& RHICmdList)
        {
            EnsureAllocated_RenderThread(RHICmdList);
            if (!bAllocated) return;

            FRDGBuilder GraphBuilder(RHICmdList);
            FRDGTextureRef FireInTex   = GraphBuilder.RegisterExternalTexture(FireBelief[ReadIdx],    TEXT("Thesis.FireBelief.In"));
            FRDGTextureRef FireOutTex  = GraphBuilder.RegisterExternalTexture(FireBelief[WriteIdx],   TEXT("Thesis.FireBelief.Out"));
            FRDGTextureRef PersonInTex = GraphBuilder.RegisterExternalTexture(PersonBelief[ReadIdx],  TEXT("Thesis.PersonBelief.In"));
            FRDGTextureRef PersonOutTex= GraphBuilder.RegisterExternalTexture(PersonBelief[WriteIdx], TEXT("Thesis.PersonBelief.Out"));

            FVector4f FireWeights[3] = {
                FVector4f(kDefaultWeights[0], kDefaultWeights[1], kDefaultWeights[2], kDefaultWeights[3]),
                FVector4f(kDefaultWeights[4], kDefaultWeights[5], kDefaultWeights[6], kDefaultWeights[7]),
                FVector4f(kDefaultWeights[8], 0.0f, 0.0f, 0.0f)
            };

            FVector4f CopyWeights[3] = {
                FVector4f(0.0f, 0.0f, 0.0f, 0.0f),
                FVector4f(1.0f, 0.0f, 0.0f, 0.0f),
                FVector4f(0.0f, 0.0f, 0.0f, 0.0f)
            };

            TShaderMapRef<FPixelBleedSpreadCS> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));

            FPixelBleedSpreadCS::FParameters* FireParams = GraphBuilder.AllocParameters<FPixelBleedSpreadCS::FParameters>();
            FireParams->InputBelief  = GraphBuilder.CreateSRV(FRDGTextureSRVDesc::Create(FireInTex));
            FireParams->OutputBelief = GraphBuilder.CreateUAV(FRDGTextureUAVDesc(FireOutTex));
            FireParams->TextureSize  = Size;
            FireParams->DecayPerTick = kDefaultDecayPerTick;
            for (int32 j = 0; j < 3; ++j)
            {
                FireParams->Weights[j] = FireWeights[j];
            }
            FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("Thesis.PixelBleedSpread.Fire"), Shader, FireParams,
                                         FComputeShaderUtils::GetGroupCount(Size, FIntPoint(8, 8)));

            // Person belief is not spatially spread, but it must be copied into the
            // write texture so the fire/person ping-pong indices stay coherent.
            FPixelBleedSpreadCS::FParameters* PersonParams = GraphBuilder.AllocParameters<FPixelBleedSpreadCS::FParameters>();
            PersonParams->InputBelief  = GraphBuilder.CreateSRV(FRDGTextureSRVDesc::Create(PersonInTex));
            PersonParams->OutputBelief = GraphBuilder.CreateUAV(FRDGTextureUAVDesc(PersonOutTex));
            PersonParams->TextureSize  = Size;
            PersonParams->DecayPerTick = 0.0f;
            for (int32 j = 0; j < 3; ++j)
            {
                PersonParams->Weights[j] = CopyWeights[j];
            }
            FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("Thesis.PixelBleedSpread.CopyPerson"), Shader, PersonParams,
                                         FComputeShaderUtils::GetGroupCount(Size, FIntPoint(8, 8)));

            GraphBuilder.Execute();
        });
}

void UGpuBeliefManager::DispatchPriority(int32 CurrentTick, bool bSeparatePersonFreshness)
{
    if (Width <= 0 || Height <= 0) return;
    const int32 FireIdx   = WriteIndex;
    const int32 PersonIdx = WriteIndex;
    const FIntPoint Size(Width, Height);

    ENQUEUE_RENDER_COMMAND(ThesisPriority)(
        [this, FireIdx, PersonIdx, Size, CurrentTick, bSeparatePersonFreshness](FRHICommandListImmediate& RHICmdList)
        {
            EnsureAllocated_RenderThread(RHICmdList);
            if (!bAllocated) return;

            FRDGBuilder GraphBuilder(RHICmdList);
            FRDGTextureRef FireTex   = GraphBuilder.RegisterExternalTexture(FireBelief[FireIdx], TEXT("Thesis.FireBelief.Read"));
            FRDGTextureRef PersonTex = GraphBuilder.RegisterExternalTexture(PersonBelief[PersonIdx], TEXT("Thesis.PersonBelief.Read"));
            FRDGTextureRef StaleTex  = GraphBuilder.RegisterExternalTexture(Staleness, TEXT("Thesis.Staleness.Read"));
            FRDGTextureRef PrioTex   = GraphBuilder.RegisterExternalTexture(Priority, TEXT("Thesis.Priority.Write"));
            FRDGTextureRef PersonTickTex = GraphBuilder.RegisterExternalTexture(PersonObservationTick, TEXT("Thesis.PersonObservationTick.Read"));

            FPriorityComputeCS::FParameters* Params = GraphBuilder.AllocParameters<FPriorityComputeCS::FParameters>();
            Params->InputFireBelief   = GraphBuilder.CreateSRV(FRDGTextureSRVDesc::Create(FireTex));
            Params->InputPersonBelief = GraphBuilder.CreateSRV(FRDGTextureSRVDesc::Create(PersonTex));
            Params->InputStaleness    = GraphBuilder.CreateSRV(FRDGTextureSRVDesc::Create(StaleTex));
            Params->InputPersonObservationTick = GraphBuilder.CreateSRV(FRDGTextureSRVDesc::Create(PersonTickTex));
            Params->SeparatePersonFreshness = bSeparatePersonFreshness ? 1u : 0u;
            Params->OutputPriority    = GraphBuilder.CreateUAV(FRDGTextureUAVDesc(PrioTex));
            Params->TextureSize       = Size;
            Params->WeightStaleness          = ThesisPriority::WeightStaleness;
            Params->WeightDanger             = ThesisPriority::WeightDanger;
            Params->WeightPerson             = ThesisPriority::WeightPerson;
            Params->StalenessSaturationTicks = ThesisPriority::StalenessSaturationTicks;
            Params->ProbabilityScale         = ThesisPriority::ProbabilityScale;
            Params->MonitoringUrgencyTau     = ThesisPriority::MonitoringUrgencyTau;
            Params->UnseenExplorationBonus   = ThesisPriority::UnseenExplorationBonus;
            Params->CurrentTick              = static_cast<uint32>(CurrentTick);

            TShaderMapRef<FPriorityComputeCS> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
            FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("Thesis.PriorityCompute"), Shader, Params,
                                         FComputeShaderUtils::GetGroupCount(Size, FIntPoint(8, 8)));
            GraphBuilder.Execute();
        });
}

void UGpuBeliefManager::SwapPingPong()
{
    Swap(ReadIndex, WriteIndex);
}

// ---------- Observation Ingestion ----------

void UGpuBeliefManager::DispatchObservationIngest(const TArray<FObservation>& Observations)
{
    if (Observations.Num() == 0 || Width <= 0 || Height <= 0) return;

    // -----------------------------------------------------------------
    // 1. Aggregate per-cell deltas on the CPU – no atomics needed later
    // -----------------------------------------------------------------
    TMap<int32, TPair<float, float>> CellDeltas;  // CellIndex -> (FireLLR, PersonLLR)
    CellDeltas.Reserve(FMath::Min(Observations.Num(), Width * Height));

    TMap<int32, uint32> PersonTicks;
    for (const FObservation& O : Observations)
    {
        TPair<float, float>& Delta = CellDeltas.FindOrAdd(O.CellIndex, TPair<float, float>(0.0f, 0.0f));
        Delta.Key   += O.FireLogOddsUpdate;
        Delta.Value += O.PersonLogOddsUpdate;
        if (O.bHasPersonEvidence && O.Tick >= 0)
        {
            uint32& TickPlusOne = PersonTicks.FindOrAdd(O.CellIndex);
            TickPlusOne = FMath::Max(TickPlusOne, static_cast<uint32>(O.Tick) + 1u);
        }
    }

    // Pack into a structured buffer (one entry per unique cell)
    struct FPackedUpdate
    {
        uint32 CellIdx;
        float  FireDelta;
        float  PersonDelta;
        uint32 PersonTickPlusOne;
    };
    TArray<FPackedUpdate> Packed;
    Packed.Reserve(CellDeltas.Num());
    for (const auto& It : CellDeltas)
    {
        Packed.Add({ (uint32)It.Key, It.Value.Key, It.Value.Value, PersonTicks.FindRef(It.Key) });
    }

    // Apply to the current read index (will be used by the next spread)
    const int32 FireIdx   = ReadIndex;
    const int32 PersonIdx = ReadIndex;
    const FIntPoint Size(Width, Height);

    ENQUEUE_RENDER_COMMAND(ThesisObservationIngest)(
        [this, Packed = MoveTemp(Packed), FireIdx, PersonIdx, Size](FRHICommandListImmediate& RHICmdList)
        {
            EnsureAllocated_RenderThread(RHICmdList);
            if (!bAllocated) return;

            FRDGBuilder GraphBuilder(RHICmdList);

            FRDGBufferRef UpdateBuffer = CreateStructuredBuffer(
                GraphBuilder,
                TEXT("Thesis.CellUpdates"),
                sizeof(FPackedUpdate),
                Packed.Num(),
                Packed.GetData(),
                Packed.Num() * sizeof(FPackedUpdate));

            FRDGTextureRef FireUAV   = GraphBuilder.RegisterExternalTexture(FireBelief[FireIdx],   TEXT("Thesis.FireBelief.UAV"));
            FRDGTextureRef PersonUAV = GraphBuilder.RegisterExternalTexture(PersonBelief[PersonIdx], TEXT("Thesis.PersonBelief.UAV"));
            FRDGTextureRef StaleUAV  = GraphBuilder.RegisterExternalTexture(Staleness,             TEXT("Thesis.Staleness.UAV"));

            FRDGTextureRef PersonTickUAV = GraphBuilder.RegisterExternalTexture(PersonObservationTick, TEXT("Thesis.PersonObservationTick.UAV"));
            FObservationIngestCS::FParameters* Params =
                GraphBuilder.AllocParameters<FObservationIngestCS::FParameters>();
            Params->CellUpdates       = GraphBuilder.CreateSRV(FRDGBufferSRVDesc(UpdateBuffer));
            Params->FireBeliefOut     = GraphBuilder.CreateUAV(FRDGTextureUAVDesc(FireUAV));
            Params->PersonBeliefOut   = GraphBuilder.CreateUAV(FRDGTextureUAVDesc(PersonUAV));
            Params->StalenessOut      = GraphBuilder.CreateUAV(FRDGTextureUAVDesc(StaleUAV));
            Params->TextureSize       = Size;
            Params->PersonObservationTickOut = GraphBuilder.CreateUAV(FRDGTextureUAVDesc(PersonTickUAV));
            Params->NumUpdates        = static_cast<uint32>(Packed.Num());

            TShaderMapRef<FObservationIngestCS> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
            const int32 ThreadGroups = FMath::DivideAndRoundUp(Packed.Num(), 64);
            FComputeShaderUtils::AddPass(
                GraphBuilder,
                RDG_EVENT_NAME("Thesis.ObservationIngest"),
                Shader,
                Params,
                FIntVector(ThreadGroups, 1, 1));

            GraphBuilder.Execute();
        });
}
// ---------- Priority Debug Readback ----------

void UGpuBeliefManager::RequestPriorityReadback(int32 CurrentTick)
{
    if (Width <= 0 || Height <= 0) return;

    ENQUEUE_RENDER_COMMAND(ThesisPriorityReadback)(
        [this](FRHICommandListImmediate& RHICmdList)
        {
            if (!bAllocated || !Priority.IsValid()) return;

            if (!PriorityReadback)
            {
                PriorityReadback = MakeUnique<FRHIGPUTextureReadback>(TEXT("Thesis.PriorityReadback"));
            }

            FRHITexture* PrioRHI = Priority->GetRHI();
            if (PrioRHI)
            {
                RHICmdList.Transition(FRHITransitionInfo(PrioRHI, ERHIAccess::Unknown, ERHIAccess::CopySrc));
                PriorityReadback->EnqueueCopy(RHICmdList, PrioRHI);
            }
        });

    PriorityReadbackFence.BeginFence();
    PriorityReadbackTick = CurrentTick;
}

bool UGpuBeliefManager::TryGetPriorityReadback(TArray<float>& OutPriorityData, int32& OutSourceTick)
{
    if (!PriorityReadback) return false;
    if (!PriorityReadbackFence.IsFenceComplete()) return false;
    if (!PriorityReadback->IsReady()) return false;

    int32 RowPitchInPixels = 0;
    const void* Data = PriorityReadback->Lock(RowPitchInPixels);
    if (!Data || RowPitchInPixels <= 0)
    {
        if (Data)
        {
            PriorityReadback->Unlock();
        }
        return false;
    }

    OutPriorityData.SetNumUninitialized(Width * Height);
    const float* Src = static_cast<const float*>(Data);
    if (RowPitchInPixels == Width)
    {
        FMemory::Memcpy(OutPriorityData.GetData(), Src, static_cast<SIZE_T>(Width) * Height * sizeof(float));
    }
    else
    {
        for (int32 y = 0; y < Height; ++y)
        {
            FMemory::Memcpy(OutPriorityData.GetData() + y * Width,
                            Src + y * RowPitchInPixels,
                            static_cast<SIZE_T>(Width) * sizeof(float));
        }
    }
    PriorityReadback->Unlock();

    OutSourceTick = PriorityReadbackTick;
    PriorityReadbackTick = -1;
    return true;
}

// ---------- Grid stats (metrics) ----------

void UGpuBeliefManager::DispatchGridStats(int32 CurrentTick)
{
    if (Width <= 0 || Height <= 0) return;

    const FIntPoint Size(Width, Height);
    GridStatsTick = CurrentTick;

    ENQUEUE_RENDER_COMMAND(ThesisGridStats)(
        [this, Size, CurrentTick](FRHICommandListImmediate& RHICmdList)
        {
            EnsureAllocated_RenderThread(RHICmdList);
            if (!bAllocated) return;

            if (!GridStatsReadback)
            {
                GridStatsReadback = MakeUnique<FRHIGPUBufferReadback>(TEXT("Thesis.GridStatsReadback"));
            }

            FRDGBuilder GraphBuilder(RHICmdList);

            FRDGTextureRef StaleTex = GraphBuilder.RegisterExternalTexture(Staleness, TEXT("Thesis.Staleness.Stats"));

            FRDGBufferRef PartialsBuffer = GraphBuilder.CreateBuffer(
                FRDGBufferDesc::CreateStructuredDesc(sizeof(FVector4f), Size.X),
                TEXT("Thesis.GridStats.Partials"));
            FRDGBufferRef ResultBuffer = GraphBuilder.CreateBuffer(
                FRDGBufferDesc::CreateStructuredDesc(sizeof(FVector4f), 1),
                TEXT("Thesis.GridStats.Result"));

            // Pass 1: one thread per column, sequential rows.
            FGridStatsColumnReduceCS::FParameters* ColParams =
                GraphBuilder.AllocParameters<FGridStatsColumnReduceCS::FParameters>();
            ColParams->InputStaleness = GraphBuilder.CreateSRV(FRDGTextureSRVDesc::Create(StaleTex));
            ColParams->ColumnPartials = GraphBuilder.CreateUAV(FRDGBufferUAVDesc(PartialsBuffer));
            ColParams->TextureSize    = Size;
            ColParams->CurrentTickF   = static_cast<float>(CurrentTick);

            TShaderMapRef<FGridStatsColumnReduceCS> ColShader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
            FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("Thesis.GridStats.Columns"), ColShader, ColParams,
                                         FIntVector(FMath::DivideAndRoundUp(Size.X, 64), 1, 1));

            // Pass 2: single thread over column partials.
            FGridStatsFinalReduceCS::FParameters* FinParams =
                GraphBuilder.AllocParameters<FGridStatsFinalReduceCS::FParameters>();
            FinParams->ColumnPartialsIn = GraphBuilder.CreateSRV(FRDGBufferSRVDesc(PartialsBuffer));
            FinParams->StatsOut         = GraphBuilder.CreateUAV(FRDGBufferUAVDesc(ResultBuffer));
            FinParams->NumColumns       = Size.X;

            TShaderMapRef<FGridStatsFinalReduceCS> FinShader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
            FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("Thesis.GridStats.Final"), FinShader, FinParams,
                                         FIntVector(1, 1, 1));

            GraphBuilder.QueueBufferExtraction(ResultBuffer, &GridStatsResultBuffer);
            GraphBuilder.Execute();

            if (GridStatsResultBuffer.IsValid())
            {
                FRHIBuffer* ResultRHI = GridStatsResultBuffer->GetRHI();
                if (ResultRHI)
                {
                    RHICmdList.Transition(FRHITransitionInfo(ResultRHI, ERHIAccess::Unknown, ERHIAccess::CopySrc));
                    GridStatsReadback->EnqueueCopy(RHICmdList, ResultRHI, sizeof(FVector4f));
                }
            }
        });

    GridStatsFence.BeginFence();
}

bool UGpuBeliefManager::ConsumeGridStats(float& OutMeanStaleness, float& OutMaxStaleness,
                                         float& OutCoverageFraction, int32& OutSourceTick)
{
    if (GridStatsTick < 0 || !GridStatsReadback) return false;

    // Deterministic consumption: the caller decides the consumption tick; we
    // block until the GPU result is ready: slow frame, correct sim/metrics.
    GridStatsFence.Wait();

    // FRHIGPUBufferReadback::Lock asserts IsInRenderingThread() — unlike the
    // texture readbacks used elsewhere, the buffer lock must happen on the
    // render thread. Capture-by-reference is safe: FlushRenderingCommands
    // blocks this (game) thread until the lambda has executed.
    FVector4f Stats(0.0f, 0.0f, 0.0f, 0.0f);
    bool bGotData = false;
    ENQUEUE_RENDER_COMMAND(ThesisGridStatsRead)(
        [this, &Stats, &bGotData](FRHICommandListImmediate& RHICmdList)
        {
            if (GridStatsReadback.IsValid() && !GridStatsReadback->IsReady())
            {
                // A render-command fence is not a GPU completion fence. This
                // sample is due now, so finish its GPU work on this tick.
                RHICmdList.BlockUntilGPUIdle();
            }
            if (GridStatsReadback.IsValid() && GridStatsReadback->IsReady())
            {
                if (const void* Data = GridStatsReadback->Lock(sizeof(FVector4f)))
                {
                    Stats = *static_cast<const FVector4f*>(Data);
                    GridStatsReadback->Unlock();
                    bGotData = true;
                }
            }
        });
    FlushRenderingCommands();

    if (!bGotData)
    {
        UE_LOG(LogTemp, Fatal, TEXT("Thesis grid stats: readback failed after GPU completion."));
        GridStatsTick = -1;
        return false;
    }

    const float TotalCells = static_cast<float>(Width) * static_cast<float>(Height);
    if (TotalCells <= 0.0f) return false;

    OutMeanStaleness    = Stats.X / TotalCells;
    OutMaxStaleness     = Stats.Y;
    OutCoverageFraction = Stats.Z / TotalCells;
    OutSourceTick       = GridStatsTick;
    GridStatsTick = -1;
    return true;
}
