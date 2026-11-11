#pragma once

#include "CoreMinimal.h"
#include "Engine/DeveloperSettings.h"
#include "TerrainDeformSettings.generated.h"

class UTextureRenderTarget2D;
class UTexture2D;
class UMaterialParameterCollection;

/**
 * Asset bindings for the deformation windows. The landscape material samples these assets directly
 * (landscape material instances can't take per-instance runtime textures), so the subsystem renders
 * into the same render targets and drives the same parameter collection.
 */
UCLASS(Config = Game, DefaultConfig, meta = (DisplayName = "Terrain Deform"))
class UNREALCHRONO_API UTerrainDeformSettings : public UDeveloperSettings
{
	GENERATED_BODY()

public:
	UTerrainDeformSettings();

	/** Coarse layer (long memory): R32F, UAV-enabled. Falls back to a transient target if unset or the wrong size. */
	UPROPERTY(Config, EditAnywhere, Category = "Assets")
	TSoftObjectPtr<UTextureRenderTarget2D> HeightTarget;

	/** Coarse layer: RGBA8, UAV-enabled. */
	UPROPERTY(Config, EditAnywhere, Category = "Assets")
	TSoftObjectPtr<UTextureRenderTarget2D> NormalTarget;

	/** Fine layer (tread detail around the vehicle): R32F. */
	UPROPERTY(Config, EditAnywhere, Category = "Assets")
	TSoftObjectPtr<UTextureRenderTarget2D> FineHeightTarget;

	/** Fine layer: RGBA8. */
	UPROPERTY(Config, EditAnywhere, Category = "Assets")
	TSoftObjectPtr<UTextureRenderTarget2D> FineNormalTarget;

	/** Tread depth map: white = rubber presses in, black = groove. Width = tire tread width, length = one repeat. */
	UPROPERTY(Config, EditAnywhere, Category = "Assets")
	TSoftObjectPtr<UTexture2D> TreadTexture;

	/** Per-layer parameters (DeformCenter, DeformLayers) are written here each frame. */
	UPROPERTY(Config, EditAnywhere, Category = "Assets")
	TSoftObjectPtr<UMaterialParameterCollection> ParameterCollection;

};
