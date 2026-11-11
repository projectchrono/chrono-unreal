#pragma once

#include "CoreMinimal.h"
#include "Tools/Common.h"
#include "Tools/Conversions.h"
#include "VehicleCommon.generated.h"


#define CH_PI = 3.14159265358979323846264338327;

UENUM(BlueprintType)
enum class EWheelLocation : uint8
{
	FrontLeft		UMETA(DisplayName = "FrontLeft"),
	FrontRight		UMETA(DisplayName = "FrontRight"),
	RearLeft		UMETA(DisplayName = "RearLeft"),
	RearRight		UMETA(DisplayName = "RearRight")
};


UENUM(BlueprintType)
enum class ETerrainType : uint8
{
	Grass			UMETA(DisplayName = "Grass"),
	Sand			UMETA(DisplayName = "Sand"),
	Dirt			UMETA(DisplayName = "Dirt"),
	Concrete		UMETA(DisplayName = "Concrete"),
	Asphalt			UMETA(DisplayName = "Asphalt"),
	Gravel			UMETA(DisplayName = "Gravel"),
	Mud				UMETA(DisplayName = "Mud"),
	Rock			UMETA(DisplayName = "Rock"),
	Snow			UMETA(DisplayName = "Snow"),
	Ice				UMETA(DisplayName = "Ice"),
	WetAsphalt		UMETA(DisplayName = "WetAsphalt"),
	Glass			UMETA(DisplayName = "Glass"),
	Wood			UMETA(DisplayName = "Wood")
};

/** Map of terrain types to their respective friction values */
static const TMap<ETerrainType, float> TerrainFrictionMap = {
	{ETerrainType::Grass, 0.5f},
	{ETerrainType::Sand, 0.4f},
	{ETerrainType::Dirt, 0.6f},
	{ETerrainType::Concrete, 0.9f},
	{ETerrainType::Asphalt, 0.8f},
	{ETerrainType::Gravel, 0.6f},
	{ETerrainType::Mud, 0.35f},
	{ETerrainType::Rock, 0.85f},
	{ETerrainType::Snow, 0.25f},
	{ETerrainType::Ice, 0.15f},
	{ETerrainType::WetAsphalt, 0.65f},
	{ETerrainType::Glass, 0.65f},
	{ETerrainType::Wood, 0.75f}
};



/// Enum for available tire models.
UENUM(BlueprintType)
enum class ETireModelType : uint8
{
    RIGID = (int)chrono::vehicle::TireModelType::RIGID,       ///< rigid tire (cylindrical)
    RIGID_MESH = (int)chrono::vehicle::TireModelType::RIGID_MESH,  ///< rigid tire (mesh)
    FIALA = (int)chrono::vehicle::TireModelType::FIALA,       ///< Fiala tire
    ANCF = (int)chrono::vehicle::TireModelType::ANCF,        ///< ANCF shell element-based tire
    REISSNER = (int)chrono::vehicle::TireModelType::REISSNER,    ///< Reissner 6-field shell element-based tire
    FEA = (int)chrono::vehicle::TireModelType::FEA,         ///< FEA co-rotational tire
    PAC89 = (int)chrono::vehicle::TireModelType::PAC89,       ///< Pacejka 89 (magic formula) tire
    TMEASY = (int)chrono::vehicle::TireModelType::TMEASY,      ///< Tire Model Made Easy tire (G. Rill)
    PAC02 = (int)chrono::vehicle::TireModelType::PAC02,        ///< Pacejka 02 (magic formula) tire
	DISCRETE
};

UENUM(BlueprintType)
enum class ETireCollisionType : uint8
{
    SINGLE_POINT,
    FOUR_POINTS,
    ENVELOPE
};
