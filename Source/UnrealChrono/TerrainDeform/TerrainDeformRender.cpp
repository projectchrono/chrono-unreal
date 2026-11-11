#include "TerrainDeformRender.h"

#include "DataDrivenShaderPlatformInfo.h"
#include "Engine/World.h"
#include "GlobalShader.h"
#include "RenderGraphBuilder.h"
#include "RenderGraphEvent.h"
#include "RenderGraphUtils.h"
#include "RenderUtils.h"
#include "RHIGPUReadback.h"
#include "RHIStaticStates.h"
#include "SceneViewExtensionContext.h"
#include "ShaderParameterStruct.h"
#include "TerrainDeformPhysicsCache.h"
#include "TextureResource.h"

// Shows up as "TerrainDeform" in `stat gpu`.
DECLARE_GPU_STAT(TerrainDeform);

// Live tuning. Read every frame on the render thread; defaults match FTerrainDeformSettings.
static TAutoConsoleVariable<float> CVarCompression(TEXT("TerrainDeform.Compression"), 0.4f,
	TEXT("Fraction of displaced soil volume that is compacted away (0 = exactly volume preserving, 1 = no berms)."), ECVF_RenderThreadSafe);
static TAutoConsoleVariable<float> CVarForwardBias(TEXT("TerrainDeform.ForwardBias"), 0.f,
	TEXT("0 = berm material goes straight out to the sides, 1 = nearest point on the footprint contour (can pile up ahead)."), ECVF_RenderThreadSafe);
static TAutoConsoleVariable<float> CVarReposeAngle(TEXT("TerrainDeform.ReposeAngle"), 35.f,
	TEXT("Angle of repose in degrees for slope relaxation."), ECVF_RenderThreadSafe);
static TAutoConsoleVariable<float> CVarTreadDepth(TEXT("TerrainDeform.TreadDepth"), 2.f,
	TEXT("How far tread lugs press below the track floor, in cm (0 disables tread)."), ECVF_RenderThreadSafe);
static TAutoConsoleVariable<int32> CVarEnabled(TEXT("TerrainDeform.Enabled"), 1,
	TEXT("0 = run no deformation passes at all (for timing the system on and off in the same scene)."), ECVF_RenderThreadSafe);
static TAutoConsoleVariable<int32> CVarPhysicsFeedback(TEXT("TerrainDeform.PhysicsFeedback"), 1,
	TEXT("1 = read the deformation back to the CPU every frame so tire physics sees ruts and berms (0 = visual only)."), ECVF_RenderThreadSafe);
static TAutoConsoleVariable<int32> CVarRelaxIterations(TEXT("TerrainDeform.RelaxIterations"), 8,
	TEXT("Slope relaxation passes per stamp per frame (0 disables)."), ECVF_RenderThreadSafe);

BEGIN_SHADER_PARAMETER_STRUCT(FTerrainDeformCSParameters, )
	SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2D<int>, DeltaTex)
	SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2D<float>, HeightOut)
	SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2D<float4>, NormalOut)
	SHADER_PARAMETER_TEXTURE(Texture2D, TreadTexture)
	SHADER_PARAMETER_SAMPLER(SamplerState, TreadSampler)
	SHADER_PARAMETER(uint32, WindowSize)
	SHADER_PARAMETER(float, TexelSize)
	SHADER_PARAMETER(FIntPoint, TileOrigin)
	SHADER_PARAMETER(FIntPoint, TileSize)
	SHADER_PARAMETER(FVector2f, StampCenter)
	SHADER_PARAMETER(FVector2f, StampForward)
	SHADER_PARAMETER(float, StampHalfLength)
	SHADER_PARAMETER(float, StampHalfWidth)
	SHADER_PARAMETER(float, StampSinkage)
	SHADER_PARAMETER(float, StampSegmentHalf)
	SHADER_PARAMETER(float, StampPathStart)
	SHADER_PARAMETER(float, TreadDepth)
	SHADER_PARAMETER(float, TreadWidthCm)
	SHADER_PARAMETER(float, TreadRepeatCm)
	SHADER_PARAMETER(float, Compression)
	SHADER_PARAMETER(float, ForwardBias)
	SHADER_PARAMETER(int32, ReposeThreshold)
END_SHADER_PARAMETER_STRUCT()

// Seeds the fine layer from the coarse one; separate struct so no pass binds the same texture twice.
BEGIN_SHADER_PARAMETER_STRUCT(FTerrainDeformInitParameters, )
	SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2D<int>, DeltaTex)
	SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2D<int>, CoarseDelta)
	SHADER_PARAMETER(uint32, WindowSize)
	SHADER_PARAMETER(float, TexelSize)
	SHADER_PARAMETER(uint32, CoarseWindowSize)
	SHADER_PARAMETER(float, CoarseTexelSize)
	SHADER_PARAMETER(FIntPoint, TileOrigin)
	SHADER_PARAMETER(FIntPoint, TileSize)
END_SHADER_PARAMETER_STRUCT()

template <typename TParameters>
class TTerrainDeformCS : public FGlobalShader
{
public:
	using FParameters = TParameters;
	TTerrainDeformCS() = default;
	TTerrainDeformCS(const ShaderMetaType::CompiledShaderInitializerType& Initializer) : FGlobalShader(Initializer) {}
	static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
	{
		return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
	}
};
using FTerrainDeformCS = TTerrainDeformCS<FTerrainDeformCSParameters>;
using FTerrainDeformInitBase = TTerrainDeformCS<FTerrainDeformInitParameters>;

class FTerrainDeformStampCS : public FTerrainDeformCS
{
	DECLARE_GLOBAL_SHADER(FTerrainDeformStampCS);
	SHADER_USE_PARAMETER_STRUCT(FTerrainDeformStampCS, FTerrainDeformCS);
};
class FTerrainDeformTreadCS : public FTerrainDeformCS
{
	DECLARE_GLOBAL_SHADER(FTerrainDeformTreadCS);
	SHADER_USE_PARAMETER_STRUCT(FTerrainDeformTreadCS, FTerrainDeformCS);
};
class FTerrainDeformRelaxCS : public FTerrainDeformCS
{
	DECLARE_GLOBAL_SHADER(FTerrainDeformRelaxCS);
	SHADER_USE_PARAMETER_STRUCT(FTerrainDeformRelaxCS, FTerrainDeformCS);
};
class FTerrainDeformClearCS : public FTerrainDeformCS
{
	DECLARE_GLOBAL_SHADER(FTerrainDeformClearCS);
	SHADER_USE_PARAMETER_STRUCT(FTerrainDeformClearCS, FTerrainDeformCS);
};
class FTerrainDeformResolveCS : public FTerrainDeformCS
{
	DECLARE_GLOBAL_SHADER(FTerrainDeformResolveCS);
	SHADER_USE_PARAMETER_STRUCT(FTerrainDeformResolveCS, FTerrainDeformCS);
};
class FTerrainDeformInitFromCoarseCS : public FTerrainDeformInitBase
{
	DECLARE_GLOBAL_SHADER(FTerrainDeformInitFromCoarseCS);
	SHADER_USE_PARAMETER_STRUCT(FTerrainDeformInitFromCoarseCS, FTerrainDeformInitBase);
};

IMPLEMENT_GLOBAL_SHADER(FTerrainDeformStampCS, "/UnrealChrono/TerrainDeform/TerrainDeform.usf", "StampCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FTerrainDeformTreadCS, "/UnrealChrono/TerrainDeform/TerrainDeform.usf", "TreadCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FTerrainDeformRelaxCS, "/UnrealChrono/TerrainDeform/TerrainDeform.usf", "RelaxCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FTerrainDeformClearCS, "/UnrealChrono/TerrainDeform/TerrainDeform.usf", "ClearCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FTerrainDeformResolveCS, "/UnrealChrono/TerrainDeform/TerrainDeform.usf", "ResolveCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FTerrainDeformInitFromCoarseCS, "/UnrealChrono/TerrainDeform/TerrainDeform.usf", "InitFromCoarseCS", SF_Compute);

FTerrainDeformViewExtension::FTerrainDeformViewExtension(const FAutoRegister& AutoRegister, const UWorld* InWorld, const FTerrainDeformSettings& InSettings)
	: FSceneViewExtensionBase(AutoRegister)
	, World(InWorld)
	, Settings(InSettings)
{
}

FTerrainDeformViewExtension::~FTerrainDeformViewExtension()
{
	// A copy task still reads the mapped staging texture; it must finish before the readback objects go away.
	for (FPhysicsReadback& Slot : PhysicsReadbacks)
	{
		Slot.CopyTask.Wait();
	}
}

void FTerrainDeformViewExtension::SetOutputResources(int32 Layer, FTextureRenderTargetResource* InHeight, FTextureRenderTargetResource* InNormal)
{
	check(Layer >= 0 && Layer < FTerrainDeformSettings::NumLayers);
	ENQUEUE_RENDER_COMMAND(TerrainDeformSetOutputs)([this, Layer, InHeight, InNormal](FRHICommandListImmediate&)
	{
		Layers[Layer].HeightOut = InHeight;
		Layers[Layer].NormalOut = InNormal;
	});
}

void FTerrainDeformViewExtension::SetTreadTexture(FTextureReferenceRHIRef InTreadReference)
{
	ENQUEUE_RENDER_COMMAND(TerrainDeformSetTread)([this, InTreadReference](FRHICommandListImmediate&)
	{
		TreadReference = InTreadReference;
	});
}

void FTerrainDeformViewExtension::Submit(FTerrainDeformFrame&& Frame)
{
	FScopeLock Lock(&FrameLock);
	for (int32 L = 0; L < FTerrainDeformSettings::NumLayers; ++L)
	{
		LatestCenterTexel[L] = Frame.CenterTexel[L];
	}
	if (bHasPending)
	{
		// Render fell behind (or stamps arrived from a worker): merge so no stamps are dropped, oldest first; the latest window centres win.
		Pending.Stamps.Append(MoveTemp(Frame.Stamps));
		Frame.Stamps = MoveTemp(Pending.Stamps);
	}
	Pending = MoveTemp(Frame);
	bHasPending = true;
}

void FTerrainDeformViewExtension::AppendStamps(TArray<FTerrainDeformStamp>&& Stamps)
{
	if (Stamps.Num() == 0)
	{
		return;
	}
	FScopeLock Lock(&FrameLock);
	Pending.Stamps.Append(MoveTemp(Stamps));
	bHasPending = true;
}

bool FTerrainDeformViewExtension::IsActiveThisFrame_Internal(const FSceneViewExtensionContext& Context) const
{
	return World.IsValid() && Context.GetWorld() == World.Get();
}

void FTerrainDeformViewExtension::PreRenderViewFamily_RenderThread(FRDGBuilder& GraphBuilder, FSceneViewFamily& InViewFamily)
{
	PollVolumeReadback();
	PollPhysicsReadbacks();

	if (CVarEnabled.GetValueOnRenderThread() == 0)
	{
		// Switched off (for A/B timing): run no passes, drop queued stamps and hide the deformation from physics.
		// Existing deformation stays on screen as it was.
		{
			FScopeLock Lock(&FrameLock);
			Pending = FTerrainDeformFrame();
			bHasPending = false;
		}
		FTerrainDeformPhysicsCache::Clear();
		return;
	}

	Settings.Compression = CVarCompression.GetValueOnRenderThread();
	Settings.ForwardBias = CVarForwardBias.GetValueOnRenderThread();
	Settings.ReposeAngleDeg = CVarReposeAngle.GetValueOnRenderThread();
	Settings.RelaxIterations = CVarRelaxIterations.GetValueOnRenderThread();
	Settings.TreadDepthCm = CVarTreadDepth.GetValueOnRenderThread();

	FRHITexture* HeightRHI[FTerrainDeformSettings::NumLayers] = {};
	FRHITexture* NormalRHI[FTerrainDeformSettings::NumLayers] = {};
	for (int32 L = 0; L < FTerrainDeformSettings::NumLayers; ++L)
	{
		HeightRHI[L] = Layers[L].HeightOut ? Layers[L].HeightOut->GetRenderTargetTexture() : nullptr;
		NormalRHI[L] = Layers[L].NormalOut ? Layers[L].NormalOut->GetRenderTargetTexture() : nullptr;
		if (!HeightRHI[L] || !NormalRHI[L] || !EnumHasAnyFlags(HeightRHI[L]->GetFlags(), TexCreate_UAV) || !EnumHasAnyFlags(NormalRHI[L]->GetFlags(), TexCreate_UAV))
		{
			// Not created yet, or created without UAV support: skip rather than crash RDG validation.
			return;
		}
	}

	FTerrainDeformFrame Frame;
	{
		FScopeLock Lock(&FrameLock);
		if (!bHasPending && Layers[0].bInitialized)
		{
			return;
		}
		Frame = MoveTemp(Pending);
		Pending = FTerrainDeformFrame();
		bHasPending = false;
		// Stamps can arrive between two Submits, so the centres always come from the latest Submit.
		for (int32 L = 0; L < FTerrainDeformSettings::NumLayers; ++L)
		{
			Frame.CenterTexel[L] = LatestCenterTexel[L];
		}
	}

	RDG_EVENT_SCOPE_STAT(GraphBuilder, TerrainDeform, "TerrainDeform");

	const FGlobalShaderMap* ShaderMap = GetGlobalShaderMap(GMaxRHIFeatureLevel);
	FRHITexture* TreadRHI = TreadReference.IsValid() ? static_cast<FRHITexture*>(TreadReference.GetReference()) : nullptr;
	const bool bHaveTread = TreadRHI != nullptr;
	{
		// Log once per change so a missing tread texture shows up in the log instead of failing silently.
		static int32 LastLoggedState = -1;
		const int32 State = bHaveTread ? 1 : 3;
		if (State != LastLoggedState)
		{
			LastLoggedState = State;
			UE_LOG(LogTemp, Display, TEXT("TerrainDeform: tread state = %s"), State == 1 ? TEXT("texture reference bound") : TEXT("NO tread texture"));
		}
	}
	if (!TreadRHI)
	{
		TreadRHI = GBlackTexture->TextureRHI;
	}
	FRHISamplerState* TreadSampler = TStaticSamplerState<SF_Bilinear, AM_Clamp, AM_Wrap>::GetRHI();

	// Register every layer's textures up front: the fine layer reads the coarse delta.
	FRDGTextureRef DeltaRef[FTerrainDeformSettings::NumLayers] = {};
	FRDGTextureUAVRef DeltaUAV[FTerrainDeformSettings::NumLayers] = {};
	FRDGTextureUAVRef HeightUAV[FTerrainDeformSettings::NumLayers] = {};
	FRDGTextureUAVRef NormalUAV[FTerrainDeformSettings::NumLayers] = {};
	for (int32 L = 0; L < FTerrainDeformSettings::NumLayers; ++L)
	{
		const int32 N = Settings.Layers[L].WindowSize;
		if (Layers[L].Delta.IsValid())
		{
			DeltaRef[L] = GraphBuilder.RegisterExternalTexture(Layers[L].Delta, TEXT("TerrainDeform.Delta"));
		}
		else
		{
			const FRDGTextureDesc Desc = FRDGTextureDesc::Create2D(FIntPoint(N, N), PF_R32_SINT, FClearValueBinding::None, TexCreate_ShaderResource | TexCreate_UAV);
			DeltaRef[L] = GraphBuilder.CreateTexture(Desc, TEXT("TerrainDeform.Delta"));
		}
		DeltaUAV[L] = GraphBuilder.CreateUAV(DeltaRef[L]);
		HeightUAV[L] = GraphBuilder.CreateUAV(GraphBuilder.RegisterExternalTexture(CreateRenderTarget(HeightRHI[L], TEXT("TerrainDeform.Height"))));
		NormalUAV[L] = GraphBuilder.CreateUAV(GraphBuilder.RegisterExternalTexture(CreateRenderTarget(NormalRHI[L], TEXT("TerrainDeform.Normal"))));
	}

	auto Groups = [](FIntPoint Size) { return FIntVector(FMath::DivideAndRoundUp(Size.X, 8), FMath::DivideAndRoundUp(Size.Y, 8), 1); };

	for (int32 L = 0; L < FTerrainDeformSettings::NumLayers; ++L)
	{
		const FTerrainDeformLayerConfig& Config = Settings.Layers[L];
		FLayerState& State = Layers[L];
		const int32 N = Config.WindowSize;
		const int32 Half = N / 2;
		const float TexelSize = Config.TexelSize;
		const int32 TileSize = Config.StampTileSize;
		const FIntPoint Center = Frame.CenterTexel[L];

		auto MakeParams = [&](FIntPoint Origin, FIntPoint Size)
		{
			FTerrainDeformCSParameters* P = GraphBuilder.AllocParameters<FTerrainDeformCSParameters>();
			P->DeltaTex = DeltaUAV[L];
			P->HeightOut = HeightUAV[L];
			P->NormalOut = NormalUAV[L];
			P->TreadTexture = TreadRHI;
			P->TreadSampler = TreadSampler;
			P->WindowSize = N;
			P->TexelSize = TexelSize;
			P->TileOrigin = Origin;
			P->TileSize = Size;
			P->TreadDepth = FMath::Max(Settings.TreadDepthCm, 0.f);
			P->TreadWidthCm = FMath::Max(Settings.TreadWidthCm, 1.f);
			P->TreadRepeatCm = FMath::Max(Settings.TreadRepeatCm, 1.f);
			P->Compression = FMath::Clamp(Settings.Compression, 0.f, 1.f);
			P->ForwardBias = FMath::Clamp(Settings.ForwardBias, 0.f, 1.f);
			P->ReposeThreshold = FMath::RoundToInt(FMath::Tan(FMath::DegreesToRadians(Settings.ReposeAngleDeg)) * TexelSize * 1000.f);
			return P;
		};

		// Re-initialise a rectangle: the coarse layer starts flat; the fine layer is seeded from the coarse layer so
		// old tracks stay visible when its window scrolls onto them. Outputs are refreshed to match.
		auto RefreshRect = [&](FIntPoint Origin, FIntPoint Size)
		{
			if (Size.X <= 0 || Size.Y <= 0) { return; }
			if (L == FTerrainDeformSettings::CoarseLayer)
			{
				TShaderMapRef<FTerrainDeformClearCS> Clear(ShaderMap);
				FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("TerrainDeform.Clear"), Clear, MakeParams(Origin, Size), Groups(Size));
			}
			else
			{
				const FTerrainDeformLayerConfig& Coarse = Settings.Layers[FTerrainDeformSettings::CoarseLayer];
				FTerrainDeformInitParameters* IP = GraphBuilder.AllocParameters<FTerrainDeformInitParameters>();
				IP->DeltaTex = DeltaUAV[L];
				IP->CoarseDelta = DeltaUAV[FTerrainDeformSettings::CoarseLayer];
				IP->WindowSize = N;
				IP->TexelSize = TexelSize;
				IP->CoarseWindowSize = Coarse.WindowSize;
				IP->CoarseTexelSize = Coarse.TexelSize;
				IP->TileOrigin = Origin;
				IP->TileSize = Size;
				TShaderMapRef<FTerrainDeformInitFromCoarseCS> Init(ShaderMap);
				FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("TerrainDeform.InitFromCoarse"), Init, IP, Groups(Size));
			}
			TShaderMapRef<FTerrainDeformResolveCS> Resolve(ShaderMap);
			FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("TerrainDeform.ResolveRefreshed"), Resolve, MakeParams(Origin, Size), Groups(Size));
		};

		if (!State.bInitialized)
		{
			RefreshRect(Center - FIntPoint(Half, Half), FIntPoint(N, N));
			State.bInitialized = true;
		}
		else
		{
			const FIntPoint D = Center - State.LastCenterTexel;
			if (FMath::Abs(D.X) >= N || FMath::Abs(D.Y) >= N)
			{
				RefreshRect(Center - FIntPoint(Half, Half), FIntPoint(N, N));
			}
			else
			{
				if (D.X > 0)      { RefreshRect(FIntPoint(State.LastCenterTexel.X + Half, State.LastCenterTexel.Y - Half), FIntPoint(D.X, N)); }
				else if (D.X < 0) { RefreshRect(FIntPoint(Center.X - Half, State.LastCenterTexel.Y - Half), FIntPoint(-D.X, N)); }
				if (D.Y > 0)      { RefreshRect(FIntPoint(Center.X - Half, State.LastCenterTexel.Y + Half), FIntPoint(N, D.Y)); }
				else if (D.Y < 0) { RefreshRect(FIntPoint(Center.X - Half, Center.Y - Half), FIntPoint(N, -D.Y)); }
			}
		}
		State.LastCenterTexel = Center;

		const FIntPoint WinMin = Center - FIntPoint(Half, Half);
		const FIntPoint WinMax = Center + FIntPoint(Half, Half);

		for (const FTerrainDeformStamp& Stamp : Frame.Stamps)
		{
			const FIntPoint CenterTexel(FMath::FloorToInt(Stamp.Center.X / TexelSize), FMath::FloorToInt(Stamp.Center.Y / TexelSize));
			const FIntPoint Origin = CenterTexel - FIntPoint(TileSize / 2, TileSize / 2);
			// Keep a 1 texel border inside the window so normal taps stay valid.
			if (Origin.X < WinMin.X + 1 || Origin.Y < WinMin.Y + 1 ||
				Origin.X + TileSize > WinMax.X - 1 || Origin.Y + TileSize > WinMax.Y - 1)
			{
				continue;
			}

			FTerrainDeformCSParameters* P = MakeParams(Origin, FIntPoint(TileSize, TileSize));
			P->StampCenter = Stamp.Center;
			P->StampForward = Stamp.Forward.GetSafeNormal();
			P->StampHalfLength = FMath::Max(Stamp.HalfLength, 1.f);
			P->StampHalfWidth = FMath::Max(Stamp.HalfWidth, 1.f);
			P->StampSinkage = Stamp.Sinkage;
			P->StampSegmentHalf = FMath::Max(Stamp.SegmentHalf, 0.f);
			P->StampPathStart = Stamp.PathStart;

			TShaderMapRef<FTerrainDeformStampCS> StampCS(ShaderMap);
			FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("TerrainDeform.Stamp"), StampCS, P, Groups(FIntPoint(TileSize, TileSize)));

			TShaderMapRef<FTerrainDeformRelaxCS> RelaxCS(ShaderMap);
			for (int32 Iter = 0; Iter < Settings.RelaxIterations; ++Iter)
			{
				FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("TerrainDeform.Relax"), RelaxCS, MakeParams(Origin, FIntPoint(TileSize, TileSize)), Groups(FIntPoint(TileSize, TileSize)));
			}

			// Tread goes in after relaxation so the slope pass can't smooth the lugs away. Fine layer only.
			if (Config.bTread && bHaveTread && Settings.TreadDepthCm > 0.f && Stamp.SegmentHalf > 0.f)
			{
				TShaderMapRef<FTerrainDeformTreadCS> TreadCS(ShaderMap);
				FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("TerrainDeform.Tread"), TreadCS, P, Groups(FIntPoint(TileSize, TileSize)));
			}

			const FIntPoint RIn = Origin - FIntPoint(1, 1);
			const FIntPoint RSize(TileSize + 2, TileSize + 2);
			TShaderMapRef<FTerrainDeformResolveCS> ResolveCS(ShaderMap);
			FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("TerrainDeform.Resolve"), ResolveCS, MakeParams(RIn, RSize), Groups(RSize));
		}
	}

	// Optional debug check: sum of every delta texel of one layer (0 for a volume-preserving run with Compression = 0).
	if (!VolumeReadback)
	{
		const int32 RequestedLayer = VolumeRequestedLayer.Exchange(-1);
		if (RequestedLayer >= 0 && RequestedLayer < FTerrainDeformSettings::NumLayers)
		{
			VolumeReadback = MakeUnique<FRHIGPUTextureReadback>(TEXT("TerrainDeform.Volume"));
			VolumeReadbackLayer = RequestedLayer;
			AddEnqueueCopyPass(GraphBuilder, VolumeReadback.Get(), DeltaRef[RequestedLayer]);
		}
	}

	// Physics feedback: copy the fine-layer region around the vehicle into a small staging texture and read it back.
	// The window is toroidal, so the region may wrap around the texture edge: copy it in up to four pieces.
	if (CVarPhysicsFeedback.GetValueOnRenderThread() == 0)
	{
		// Feedback off: drop the last snapshot so the physics stops seeing stale deformation.
		FTerrainDeformPhysicsCache::Clear();
	}
	else
	{
		FPhysicsReadback* Free = nullptr;
		for (FPhysicsReadback& Slot : PhysicsReadbacks)
		{
			if (!Slot.bPending) { Free = &Slot; break; }
		}
		if (Free)
		{
			const int32 W = PhysicsRegionTexels;
			const int32 N = Settings.Layers[FTerrainDeformSettings::FineLayer].WindowSize;
			const FIntPoint Origin = Frame.CenterTexel[FTerrainDeformSettings::FineLayer] - FIntPoint(W / 2, W / 2);
			const int32 Sx = ((Origin.X % N) + N) % N;
			const int32 Sy = ((Origin.Y % N) + N) % N;
			const int32 W1 = FMath::Min(W, N - Sx);
			const int32 W2 = W - W1;
			const int32 H1 = FMath::Min(W, N - Sy);
			const int32 H2 = W - H1;

			FRDGTextureRef Stage = GraphBuilder.CreateTexture(
				FRDGTextureDesc::Create2D(FIntPoint(W, W), PF_R32_SINT, FClearValueBinding::None, TexCreate_ShaderResource), TEXT("TerrainDeform.PhysicsStage"));
			FRDGTextureRef FineDelta = DeltaRef[FTerrainDeformSettings::FineLayer];
			auto CopyPiece = [&](int32 SrcX, int32 SrcY, int32 Width, int32 Height, int32 DstX, int32 DstY)
			{
				if (Width <= 0 || Height <= 0) { return; }
				FRHICopyTextureInfo Info;
				Info.Size = FIntVector(Width, Height, 1);
				Info.SourcePosition = FIntVector(SrcX, SrcY, 0);
				Info.DestPosition = FIntVector(DstX, DstY, 0);
				AddCopyTexturePass(GraphBuilder, FineDelta, Stage, Info);
			};
			CopyPiece(Sx, Sy, W1, H1, 0, 0);
			CopyPiece(0, Sy, W2, H1, W1, 0);
			CopyPiece(Sx, 0, W1, H2, 0, H1);
			CopyPiece(0, 0, W2, H2, W1, H1);

			if (!Free->Readback)
			{
				Free->Readback = MakeUnique<FRHIGPUTextureReadback>(TEXT("TerrainDeform.Physics"));
			}
			AddEnqueueCopyPass(GraphBuilder, Free->Readback.Get(), Stage);
			Free->OriginTexel = Origin;
			Free->Serial = ++PhysicsSerial;
			Free->bPending = true;
		}
	}

	for (int32 L = 0; L < FTerrainDeformSettings::NumLayers; ++L)
	{
		GraphBuilder.QueueTextureExtraction(DeltaRef[L], &Layers[L].Delta);
	}
}

void FTerrainDeformViewExtension::PollPhysicsReadbacks()
{
	// The copy of a ready readback (2.3 MB, plus the snapshot allocation) runs on a worker so it stays off the render
	// thread. The render thread only maps the staging texture, and unmaps it once the worker is done: Lock / Unlock
	// need the immediate command list, reading the mapped memory in between does not.
	bool bCopyInFlight = false;
	for (FPhysicsReadback& Slot : PhysicsReadbacks)
	{
		if (!Slot.bCopying)
		{
			continue;
		}
		if (Slot.CopyTask.IsCompleted())
		{
			Slot.Readback->Unlock();
			Slot.bCopying = false;
			Slot.bPending = false;
		}
		else
		{
			bCopyInFlight = true;
		}
	}

	// Publish only the newest completed readback; older ones are simply released. One copy at a time keeps the
	// published snapshots in order (an older copy can never overwrite a newer one).
	FPhysicsReadback* Newest = nullptr;
	for (FPhysicsReadback& Slot : PhysicsReadbacks)
	{
		if (Slot.bPending && !Slot.bCopying && Slot.Readback && Slot.Readback->IsReady() && (!Newest || Slot.Serial > Newest->Serial))
		{
			Newest = &Slot;
		}
	}
	if (!Newest || bCopyInFlight)
	{
		return;
	}

	const int32 W = PhysicsRegionTexels;
	int32 RowPitch = 0;
	const int32* Data = static_cast<const int32*>(Newest->Readback->Lock(RowPitch));
	if (Data)
	{
		const FIntPoint Origin = Newest->OriginTexel;
		const float TexelSizeCm = Settings.Layers[FTerrainDeformSettings::FineLayer].TexelSize;
		Newest->bCopying = true;
		Newest->CopyTask = UE::Tasks::Launch(TEXT("TerrainDeform.PhysicsCopy"), [Data, RowPitch, W, Origin, TexelSizeCm]()
		{
			TSharedRef<FTerrainDeformPhysicsSnapshot, ESPMode::ThreadSafe> Snapshot = MakeShared<FTerrainDeformPhysicsSnapshot, ESPMode::ThreadSafe>();
			Snapshot->OriginTexel = Origin;
			Snapshot->Size = W;
			Snapshot->TexelSizeCm = TexelSizeCm;
			Snapshot->Data.SetNumUninitialized(W * W);
			for (int32 Y = 0; Y < W; ++Y)
			{
				FMemory::Memcpy(Snapshot->Data.GetData() + (int64)Y * W, Data + (int64)Y * RowPitch, W * sizeof(int32));
			}
			FTerrainDeformPhysicsCache::Publish(Snapshot);
		});
	}
	else
	{
		Newest->Readback->Unlock();
		Newest->bPending = false;
	}

	// Free the older completed slots that were skipped (the newest is freed when its copy finishes).
	for (FPhysicsReadback& Slot : PhysicsReadbacks)
	{
		if (&Slot != Newest && Slot.bPending && !Slot.bCopying && Slot.Readback && Slot.Readback->IsReady())
		{
			Slot.bPending = false;
		}
	}
}

void FTerrainDeformViewExtension::PollVolumeReadback()
{
	if (!VolumeReadback || !VolumeReadback->IsReady())
	{
		return;
	}

	const int32 N = Settings.Layers[VolumeReadbackLayer].WindowSize;
	int32 RowPitch = 0;
	const int32* Data = static_cast<const int32*>(VolumeReadback->Lock(RowPitch));
	int64 Sum = 0;
	int32 MinV = 0, MaxV = 0, NonZero = 0;
	if (Data)
	{
		for (int32 Y = 0; Y < N; ++Y)
		{
			const int32* Row = Data + (int64)Y * RowPitch;
			for (int32 X = 0; X < N; ++X)
			{
				const int32 V = Row[X];
				Sum += V;
				MinV = FMath::Min(MinV, V);
				MaxV = FMath::Max(MaxV, V);
				NonZero += (V != 0);
			}
		}
	}
	VolumeReadback->Unlock();
	VolumeReadback.Reset();

	LastMin = MinV;
	LastMax = MaxV;
	LastNonZero = NonZero;
	LastVolume = Sum;
	bHasVolume = true;
}

void FTerrainDeformViewExtension::RequestVolumeCheck(int32 Layer)
{
	VolumeRequestedLayer = Layer;
}

bool FTerrainDeformViewExtension::GetLastStats(int64& OutSum, int32& OutMin, int32& OutMax, int32& OutNonZero) const
{
	OutSum = LastVolume;
	OutMin = LastMin;
	OutMax = LastMax;
	OutNonZero = LastNonZero;
	return bHasVolume;
}
