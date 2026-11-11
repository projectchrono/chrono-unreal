#pragma once

#include "CoreMinimal.h"
#include "SceneViewExtension.h"
#include "TerrainDeformTypes.h"
#include "RenderGraphResources.h"
#include "RHIResources.h"
#include "Tasks/Task.h"

class FTextureRenderTargetResource;
class FTextureResource;
class FRHIGPUTextureReadback;
class UWorld;

/** Data handed from the game thread to the render thread once per frame. */
struct FTerrainDeformFrame
{
	TArray<FTerrainDeformStamp> Stamps;
	/** Per-layer window centre in absolute texel coordinates (snapped). */
	FIntPoint CenterTexel[FTerrainDeformSettings::NumLayers] = { FIntPoint::ZeroValue, FIntPoint::ZeroValue };
};

/** Render-thread owner of the persistent simulation state. */
class FTerrainDeformViewExtension : public FSceneViewExtensionBase
{
public:
	FTerrainDeformViewExtension(const FAutoRegister& AutoRegister, const UWorld* InWorld, const FTerrainDeformSettings& InSettings);
	virtual ~FTerrainDeformViewExtension() override;

	/** Called from the game thread. The render targets must have UAV enabled. */
	void SetOutputResources(int32 Layer, FTextureRenderTargetResource* InHeight, FTextureRenderTargetResource* InNormal);
	/**
	 * Tread depth map (white = rubber), as the texture's stable reference handle. The engine keeps the handle valid
	 * when the texture's resource is swapped (async compile, streaming), unlike a raw resource pointer. May be null.
	 */
	void SetTreadTexture(FTextureReferenceRHIRef InTreadReference);
	/** Any thread. Window centres plus the stamps that are ready; stamps still queued from earlier frames are kept. */
	void Submit(FTerrainDeformFrame&& Frame);
	/** Any thread. Stamps only (e.g. from a worker task); they use the window centres of the latest Submit. */
	void AppendStamps(TArray<FTerrainDeformStamp>&& Stamps);

	/** Debug: asks for a readback of one layer's delta window; the sum is available a few frames later. */
	void RequestVolumeCheck(int32 Layer);
	/** Same readback: sum, min / max texel (0.1 mm units) and the count of non-zero texels. False until complete. */
	bool GetLastStats(int64& OutSum, int32& OutMin, int32& OutMax, int32& OutNonZero) const;

	// ISceneViewExtension
	virtual void SetupViewFamily(FSceneViewFamily& InViewFamily) override {}
	virtual void SetupView(FSceneViewFamily& InViewFamily, FSceneView& InView) override {}
	virtual void BeginRenderViewFamily(FSceneViewFamily& InViewFamily) override {}
	virtual void PreRenderViewFamily_RenderThread(FRDGBuilder& GraphBuilder, FSceneViewFamily& InViewFamily) override;
	virtual bool IsActiveThisFrame_Internal(const FSceneViewExtensionContext& Context) const override;

private:
	/** Render-thread state of one window. */
	struct FLayerState
	{
		TRefCountPtr<IPooledRenderTarget> Delta;
		FTextureRenderTargetResource* HeightOut = nullptr;
		FTextureRenderTargetResource* NormalOut = nullptr;
		FIntPoint LastCenterTexel = FIntPoint::ZeroValue;
		bool bInitialized = false;
	};

	void PollVolumeReadback();
	void PollPhysicsReadbacks();

	/** Fine-layer region read back for the physics side, in texels per side. ~5.8 m at 0.75 cm. */
	static constexpr int32 PhysicsRegionTexels = 768;
	static constexpr int32 PhysicsReadbackSlots = 3;
	struct FPhysicsReadback
	{
		TUniquePtr<FRHIGPUTextureReadback> Readback;
		FIntPoint OriginTexel = FIntPoint::ZeroValue;
		uint64 Serial = 0;
		bool bPending = false;
		/** The staging texture stays mapped (Lock) while a worker copies it into the physics snapshot; the render thread unmaps it when that is done. */
		UE::Tasks::FTask CopyTask;
		bool bCopying = false;
	};
	FPhysicsReadback PhysicsReadbacks[PhysicsReadbackSlots];
	uint64 PhysicsSerial = 0;

	TUniquePtr<FRHIGPUTextureReadback> VolumeReadback;
	int32 VolumeReadbackLayer = 0;
	TAtomic<int32> VolumeRequestedLayer{-1};
	TAtomic<bool> bHasVolume{false};
	TAtomic<int64> LastVolume{0};
	TAtomic<int32> LastMin{0};
	TAtomic<int32> LastMax{0};
	TAtomic<int32> LastNonZero{0};

	TWeakObjectPtr<const UWorld> World;
	FTerrainDeformSettings Settings;

	FCriticalSection FrameLock;
	FTerrainDeformFrame Pending;
	bool bHasPending = false;
	/** Window centres of the latest Submit; a frame built from AppendStamps alone uses these. */
	FIntPoint LatestCenterTexel[FTerrainDeformSettings::NumLayers] = { FIntPoint::ZeroValue, FIntPoint::ZeroValue };

	// Render-thread only.
	FLayerState Layers[FTerrainDeformSettings::NumLayers];
	FTextureReferenceRHIRef TreadReference;
};
