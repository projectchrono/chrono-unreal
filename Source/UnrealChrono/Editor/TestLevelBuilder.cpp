#include "TestLevelBuilder.h"

#include "Engine/World.h"
#include "Materials/MaterialInterface.h"
#include "PhysicalMaterials/PhysicalMaterial.h"

#if WITH_EDITOR
#include "Landscape.h"
#include "LandscapeInfo.h"
#include "LandscapeProxy.h"
#include "LandscapeLayerInfoObject.h"
#endif

ALandscape* UTestLevelBuilder::CreateTestLandscape(UWorld* World, UMaterialInterface* Material, UPhysicalMaterial* PhysMaterial,
	int32 ComponentCount, float QuadSizeCm, float ZScale, float DuneAmplitudeCm, float FlatRadiusCm)
{
#if WITH_EDITOR
	if (!World)
	{
		return nullptr;
	}

	constexpr int32 SectionsPerComponent = 2;
	constexpr int32 QuadsPerSection = 63;
	const int32 QuadsPerComponent = SectionsPerComponent * QuadsPerSection;
	const int32 Size = ComponentCount * QuadsPerComponent + 1;

	// Height samples: 32768 is z = 0, one unit is ZScale / 128 cm
	const float CmPerUnit = ZScale / 128.f;
	TArray<uint16> Heights;
	Heights.SetNumUninitialized(Size * Size);
	for (int32 Y = 0; Y < Size; ++Y)
	{
		for (int32 X = 0; X < Size; ++X)
		{
			const float Px = (X - 0.5f * (Size - 1)) * QuadSizeCm;
			const float Py = (Y - 0.5f * (Size - 1)) * QuadSizeCm;
			const float R = FMath::Sqrt(Px * Px + Py * Py);
			const float Blend = FMath::SmoothStep(FlatRadiusCm, FlatRadiusCm * 2.f, R);
			const float Dunes = 0.5f * FMath::Sin(Px * 0.0013f + 0.7f) + 0.35f * FMath::Sin(Py * 0.0020f) + 0.25f * FMath::Sin((Px + Py) * 0.0035f + 1.9f);
			const float HeightCm = Blend * DuneAmplitudeCm * Dunes;
			Heights[Y * Size + X] = (uint16)FMath::Clamp(FMath::RoundToInt(32768.f + HeightCm / CmPerUnit), 0, 65535);
		}
	}

	// A landscape without edit layers keys its import data with the default (zero) guid
	TMap<FGuid, TArray<uint16>> HeightData;
	HeightData.Add(FGuid(), MoveTemp(Heights));
	TMap<FGuid, TArray<FLandscapeImportLayerInfo>> LayerData;
	LayerData.Add(FGuid(), TArray<FLandscapeImportLayerInfo>());

	const FVector Scale(QuadSizeCm, QuadSizeCm, ZScale);
	const FVector Offset(-0.5 * (Size - 1) * QuadSizeCm, -0.5 * (Size - 1) * QuadSizeCm, 0.0);

	ALandscape* Landscape = World->SpawnActor<ALandscape>(Offset, FRotator::ZeroRotator);
	if (!Landscape)
	{
		return nullptr;
	}
	Landscape->LandscapeMaterial = Material;
	Landscape->DefaultPhysMaterial = PhysMaterial;
	Landscape->SetActorRelativeScale3D(Scale);
	Landscape->StaticLightingLOD = 0;

	Landscape->Import(FGuid::NewGuid(), 0, 0, Size - 1, Size - 1, SectionsPerComponent, QuadsPerSection, HeightData, TEXT(""), LayerData,
		ELandscapeImportAlphamapType::Additive, TArrayView<const FLandscapeLayer>());

	if (ULandscapeInfo* Info = Landscape->GetLandscapeInfo())
	{
		Info->UpdateLayerInfoMap(Landscape);
	}
	Landscape->SetActorLabel(TEXT("TestLandscape"));
	return Landscape;
#else
	return nullptr;
#endif
}
