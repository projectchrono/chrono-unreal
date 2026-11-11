#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "TerrainDeformTypes.h"
#include "Tasks/Task.h"
#include "TerrainDeformSubsystem.generated.h"

class FTerrainDeformViewExtension;
class UTextureRenderTarget2D;
class UTexture2D;
class UMaterialParameterCollection;

/**
 * Owns the GPU height-delta windows (coarse long-memory layer + fine tread-detail layer). Vehicles call
 * AddTrackStamp (any thread); the landscape material samples the render targets through the parameter collection.
 */
UCLASS()
class UNREALCHRONO_API UTerrainDeformSubsystem : public UTickableWorldSubsystem
{
	GENERATED_BODY()

public:
	virtual bool ShouldCreateSubsystem(UObject* Outer) const override;
	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;
	virtual void Tick(float DeltaTime) override;
	virtual TStatId GetStatId() const override;

	/** Thread-safe. One footprint, as given. */
	void AddStamp(const FTerrainDeformStamp& Stamp);

	/**
	 * Thread-safe, only queues. A footprint belonging to a moving source (e.g. one wheel). A worker task remembers the
	 * source's last position and sweeps the footprint along the travel since then, so tracks are continuous at any speed.
	 * The stamps reach the GPU about a frame later than they would if swept inline.
	 * Forward is replaced by the travel direction once the source moves. SourceId must be unique per wheel.
	 *
	 * A source that is not moving adds nothing (its rut already exists from when it arrived, and re-stamping a
	 * jittering stopped wheel only digs and heaps soil around it), unless bAllowWhenStationary says the wheel is
	 * spinning in place, i.e. really digging in.
	 */
	void AddTrackStamp(int32 SourceId, const FTerrainDeformStamp& Stamp, bool bAllowWhenStationary = false);

	/** Script / test entry point for AddStamp. Center and Forward are world XY (cm). */
	UFUNCTION(BlueprintCallable, Category = "TerrainDeform")
	void AddDebugStamp(FVector2D Center, FVector2D Forward, float HalfLength, float HalfWidth, float Sinkage);

	/** World location the windows follow. If never set, the first player's view target is used. */
	UFUNCTION(BlueprintCallable, Category = "TerrainDeform")
	void SetFocusLocation(FVector WorldLocation);

	/** Debug: sums every texel of one layer's delta window (0 = coarse, 1 = fine). Result arrives a few frames later. */
	void RequestVolumeCheck(int32 Layer);
	/** False until a requested check has completed. Sum is in 0.1 mm texel units (0 = volume preserved). */
	bool GetLastVolumeStats(int64& OutSum, int32& OutMin, int32& OutMax, int32& OutNonZero) const;

	const FTerrainDeformSettings& GetSettings() const { return Settings; }

private:
	FTerrainDeformSettings Settings;

	UPROPERTY(Transient)
	TObjectPtr<UTextureRenderTarget2D> HeightTexture[FTerrainDeformSettings::NumLayers];
	UPROPERTY(Transient)
	TObjectPtr<UTextureRenderTarget2D> NormalTexture[FTerrainDeformSettings::NumLayers];
	UPROPERTY(Transient)
	TObjectPtr<UTexture2D> TreadTexture;
	UPROPERTY(Transient)
	TObjectPtr<UMaterialParameterCollection> ParameterCollection;

	TSharedPtr<FTerrainDeformViewExtension, ESPMode::ThreadSafe> ViewExtension;

	/** A moving source's footprint as the vehicle reported it; the worker turns it into swept stamps. */
	struct FTrackRequest
	{
		int32 SourceId = 0;
		FTerrainDeformStamp Stamp;
		bool bAllowWhenStationary = false;
	};

	/** Worker task: sweeps each request along its source's travel and hands the stamps to the render thread. */
	void SweepTrackRequests(TArray<FTrackRequest>&& Requests);

	FCriticalSection StampLock;
	/** Stamps that need no sweeping (AddStamp). */
	TArray<FTerrainDeformStamp> PendingStamps;
	TArray<FTrackRequest> PendingTrackRequests;

	/** Owned by the worker task (one runs at a time, so no lock). */
	TMap<int32, FVector2f> LastTrackPosition;
	/** Distance each source has travelled, so the tread pattern lines up along the whole track. */
	TMap<int32, float> TrackPathDistance;
	UE::Tasks::FTask SweepTask;

	FVector FocusLocation = FVector::ZeroVector;
	bool bHasFocus = false;
};
