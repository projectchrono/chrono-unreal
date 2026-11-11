#pragma once

#include "CoreMinimal.h"
#include "VehicleCommon.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "VehiclePhysicalMaterial.generated.h"

UCLASS(BlueprintType, Blueprintable, ClassGroup = Physics, hidecategories = Object, meta = (DisplayName = "Vehicle Physical Material"))
class UNREALCHRONO_API UVehiclePhysicalMaterial : public UPhysicalMaterial
{
	GENERATED_BODY()

public:

	UVehiclePhysicalMaterial();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Material")
	ETerrainType MaterialType = ETerrainType::Asphalt;

	/** Material stiffness (Young's modulus, typically in Pascals) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mechanical Properties", meta = (ClampMin = "0.0"))
	float YoungsModulus = 1e5f;

	/** Poissons ratio) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mechanical Properties", meta = (ClampMin = "0.0"))
	float PoissonRatio = 0.3f;

	/** how much resistance to the tire rolling */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Friction", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float RollingFriction = 0.001f;

	/** Once I'm slipping, how much resistance do I get?*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Friction", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float SpinningFriction = 0.001f;

	ETerrainType GetTerrainType();
};