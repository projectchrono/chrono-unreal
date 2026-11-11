#include "VehiclePhysicalMaterial.h"

UVehiclePhysicalMaterial::UVehiclePhysicalMaterial()
{

}

ETerrainType UVehiclePhysicalMaterial::GetTerrainType()
{
	return (ETerrainType)SurfaceType.GetValue();
}

