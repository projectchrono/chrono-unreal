#include "TerrainDeformSettings.h"

UTerrainDeformSettings::UTerrainDeformSettings()
{
	CategoryName = TEXT("Plugins");
	HeightTarget = TSoftObjectPtr<UTextureRenderTarget2D>(FSoftObjectPath(TEXT("/Game/TerrainDeform/RT_DeformHeight.RT_DeformHeight")));
	NormalTarget = TSoftObjectPtr<UTextureRenderTarget2D>(FSoftObjectPath(TEXT("/Game/TerrainDeform/RT_DeformNormal.RT_DeformNormal")));
	FineHeightTarget = TSoftObjectPtr<UTextureRenderTarget2D>(FSoftObjectPath(TEXT("/Game/TerrainDeform/RT_DeformFineHeight.RT_DeformFineHeight")));
	FineNormalTarget = TSoftObjectPtr<UTextureRenderTarget2D>(FSoftObjectPath(TEXT("/Game/TerrainDeform/RT_DeformFineNormal.RT_DeformFineNormal")));
	TreadTexture = TSoftObjectPtr<UTexture2D>(FSoftObjectPath(TEXT("/Game/TerrainDeform/T_TreadHeight.T_TreadHeight")));
	ParameterCollection = TSoftObjectPtr<UMaterialParameterCollection>(FSoftObjectPath(TEXT("/Game/TerrainDeform/MPC_TerrainDeform.MPC_TerrainDeform")));
}
