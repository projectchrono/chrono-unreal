#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "TestLevelBuilder.generated.h"

class ALandscape;
class UMaterialInterface;
class UPhysicalMaterial;

// Editor helpers used by Scripts/create_test_level.py (Python cannot create a Landscape on its own).
// Outside the editor the functions do nothing.
UCLASS()
class UNREALCHRONO_API UTestLevelBuilder : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	// Spawns a Nanite landscape centred on the origin: ComponentCount x ComponentCount components of 126 quads, QuadSizeCm per
	// quad, with a flat pad around the origin and gentle dunes further out. Returns null outside the editor.
	UFUNCTION(BlueprintCallable, Category = "UnrealChrono|Editor")
	static ALandscape* CreateTestLandscape(UWorld* World, UMaterialInterface* Material, UPhysicalMaterial* PhysMaterial,
		int32 ComponentCount = 8, float QuadSizeCm = 100.f, float ZScale = 100.f, float DuneAmplitudeCm = 250.f, float FlatRadiusCm = 4000.f);
};
