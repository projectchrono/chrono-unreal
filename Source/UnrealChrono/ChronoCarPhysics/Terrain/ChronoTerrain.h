#define _SILENCE_CXX17_ITERATOR_BASE_CLASS_DEPRECATION_WARNING

#pragma once

#include "ChronoTypes.h"
#include <Common.h>
#include "../Tools/MathLibrary.h"
#include "TerrainTypes.h"
#include <mutex>
#include <filesystem>
#include <algorithm>
#include <atomic>
#include <unordered_map>
#include "Conversions.h"

using std::shared_ptr;

class NGLandscape;

typedef ChVector3<float> ColorValue;
typedef float AlphaValue;
typedef std::pair<ColorValue, AlphaValue> PixelValue;
typedef std::pair<int, int> InputMapRange;
typedef std::pair<int, int> OutputMapRange;

namespace fs = std::filesystem;

struct MapCoordinates
{
public:
	int X, Y;
	MapCoordinates(const int XInit, const int YInit) : X(XInit), Y(YInit) {}
	bool operator==(const MapCoordinates& Coordinate) const
	{
		return Coordinate.X == X && Coordinate.Y == Y;
	}
};

//Texture height maps
class UEHeightFunctor : public chrono::vehicle::ChTerrain::HeightFunctor
{
public:
	UEHeightFunctor(float* CombinedMapDataInit, int UEHalfMapSizeInit, int TerrainMapSizeInit, float MinHeightInit, float MaxHeightInit, chrono::ChSystem* ChronoSystemInit)
		: CombinedMapData(CombinedMapDataInit), UEHalfMapSize(UEHalfMapSizeInit), TerrainMapSize(TerrainMapSizeInit), MinHeight(MinHeightInit), 
		MaxHeight(MaxHeightInit), ChronoSystem(ChronoSystemInit) {}

	virtual ~UEHeightFunctor() {}
	float* CombinedMapData;
	int UEHalfMapSize, TerrainMapSize;
	float MinHeight, MaxHeight;
	chrono::ChSystem * ChronoSystem;

	virtual double operator()(const ChVector3<>& loc)
	{
		const int XCoord = std::clamp((int)MapRange(-UEHalfMapSize, UEHalfMapSize, 0, TerrainMapSize, loc.x()), 0, TerrainMapSize);
		const int YCoord = std::clamp((int)MapRange(-UEHalfMapSize, UEHalfMapSize, 0, TerrainMapSize, -loc.y()), 0, TerrainMapSize);	

		const int PixelNumber = (YCoord * TerrainMapSize + XCoord) * 4;		

		if(PixelNumber > TerrainMapSize * TerrainMapSize * 4) return 0;

		const float Height = MapRangeFast(0, 1, MinHeight, MaxHeight, CombinedMapData[PixelNumber + 3]);
	
		return Height;
	}
};

class UENormalFunctor : public chrono::vehicle::ChTerrain::NormalFunctor
{
public:
	UENormalFunctor(float* CombinedMapDataInit, int UEHalfMapSizeInit, int TerrainMapSizeInit, chrono::ChSystem* ChronoSystemInit)
		: CombinedMapData(CombinedMapDataInit), UEHalfMapSize(UEHalfMapSizeInit), TerrainMapSize(TerrainMapSizeInit), ChronoSystem(ChronoSystemInit) {
	}

	virtual ~UENormalFunctor() {}
	float* CombinedMapData;
	int UEHalfMapSize, TerrainMapSize;
	chrono::ChSystem* ChronoSystem;

	virtual ChVector3<> operator()(const ChVector3<>& loc)
	{
		const int XCoord = std::clamp((int)MapRange(-UEHalfMapSize, UEHalfMapSize, 0, TerrainMapSize, loc.x()), 0, TerrainMapSize);
		const int YCoord = std::clamp((int)MapRange(-UEHalfMapSize, UEHalfMapSize, 0, TerrainMapSize, -loc.y()), 0, TerrainMapSize);

		const int PixelNumber = (YCoord * TerrainMapSize + XCoord) * 4;

		if (PixelNumber > TerrainMapSize * TerrainMapSize * 4) return ChVector3<>(0, 0, 1);

		return ChVector3<>(CombinedMapData[PixelNumber + 0], CombinedMapData[PixelNumber + 1], CombinedMapData[PixelNumber + 2]);
	}
};

class UEFrictionFunctor : public chrono::vehicle::ChTerrain::FrictionFunctor
{
public:
	UEFrictionFunctor(float* FrictionMapDataInit, int UEHalfMapSizeInit, int TerrainMapSizeInit)
		: FrictionMapData(FrictionMapDataInit), UEHalfMapSize(UEHalfMapSizeInit), TerrainMapSize(TerrainMapSizeInit) {}

	virtual ~UEFrictionFunctor() {}

	virtual float operator()(const ChVector3<>& loc)
	{
		const int XCoord = std::clamp((int)MapRange(-UEHalfMapSize, UEHalfMapSize, 0, TerrainMapSize, loc.x()), 0, TerrainMapSize);
		const int YCoord = std::clamp((int)MapRange(-UEHalfMapSize, UEHalfMapSize, 0, TerrainMapSize, -loc.y()), 0, TerrainMapSize);

		const int PixelNumber = (YCoord * TerrainMapSize + XCoord) * 4;

		if (PixelNumber > TerrainMapSize * TerrainMapSize * 4) return 0.7f;

		return FrictionMapData[PixelNumber];
	}

	float* FrictionMapData;
	int UEHalfMapSize, TerrainMapSize;
};

class MapHashFunction
{
public:
	size_t operator()(const MapCoordinates& Coordinate) const
	{
		return Coordinate.X * 31 + Coordinate.Y; //prime number modulus to minimize collisions
	}
};

//Terrain of the Bullet collision backend: a chrono::vehicle::RigidTerrain made of the MapData mesh patches plus
//terrain sensing (height / normal / friction for the TMeasy style tires) through Chrono's own ray cast against the
//Bullet collision world. That world holds the patches and the level geometry gathered from Unreal
//(FBulletStaticCollision), so the tires see exactly what the rigid bodies collide with. Runs entirely on the
//physics thread, no Unreal world access.
class ChronoTerrain : public chrono::vehicle::RigidTerrain, public INextGenTerrain
{
public:	

	ChronoTerrain(chrono::ChSystem* System, const char* MapDirectory, ETireModelType TireModelInit, std::vector<FTerrainMaterialEntry> MaterialsInit = {})
				: RigidTerrain(System), TireModel(TireModelInit), Materials(std::move(MaterialsInit))
	{
		ChronoSystem = System;
	};
	~ChronoTerrain() 
	{
		
	}	

	/// Get the (detrended) root mean square of the tracks, height offset is not considered [m]
	double GetRMS() { return m_rms; }

	/// Get the International Roughness Index, estimated from unevenness and waviness [m/km]
	double GetIRI() { return m_iri; }

	bool LoadMaps();
	ChTerrain* GetTerrain();	
	
	std::shared_ptr<Patch> AddNewTerrainPatch(   std::shared_ptr<chrono::ChContactMaterial> material,  ///< [in] contact material
		const chrono::ChCoordsys<>& position,                 ///< [in] patch location and orientation
		const std::string& mesh_file,                 ///< [in] filename of the input mesh (OBJ)
		bool connected_mesh = true,                   ///< [in] use connected contact mesh?
		double sweep_sphere_radius = 0,               ///< [in] radius of sweep sphere
		bool visualization = true,
		bool Serialize = false, 
		const fs::path TerrainFile = fs::path{});
	
	bool LoadCombinedMap(const char * FileName, int& OutWidth, int& OutHeight);	
	bool LoadFrictionMap(const char * FileName, int& OutWidth, int& OutHeight);

	// chrono::vehicle::ChTerrain - terrain sensing through the Bullet collision world (patches + gathered level
	// geometry). Falls back to the RigidTerrain patch lookup when the ray misses.
	virtual double GetHeight(const ChVector3d& loc) const override;
	virtual ChVector3d GetNormal(const ChVector3d& loc) const override;
	virtual float GetCoefficientFriction(const ChVector3d& loc) const override;
	virtual void GetProperties(const ChVector3d& loc, ChVector3d& point, double& height, ChVector3d& normal, float& friction) const override;

	// INextGenTerrain
	virtual bool QueryTerrain(const ChVector3d& Loc, FTerrainQueryResult& Out) const override;

	//Ray extents around the query point (metres)
	double RayUpMetres = 2.0;
	double RayDownMetres = 6.0;

	std::shared_ptr<UEHeightFunctor> HeightFunc;
	std::shared_ptr<UENormalFunctor> NormalFunc;
	std::shared_ptr<UEFrictionFunctor> FrictionFunc;

	TMap<TireKey, TireTerrainData> TireData;

	static ETerrainType PhysMaterialToTerrain();

private:
	double m_rms;                      ///< (detrended) root mean square of the uneven tracks
	double m_iri;                      ///< International Roughness Index estimated from unevenness and waviness*/

	void SetupCollision();

	chrono::ChSystem* ChronoSystem;	
	std::unordered_map< MapCoordinates, PixelValue, MapHashFunction> SparseImageMap;

	const char * MapDirectory;		

	ETireModelType TireModel;

	std::vector<FTerrainMaterialEntry> Materials;

	long long TotalMapRange;
	int TerrainMapSize = 65536; //Size of the texture map. 64k map by default
	int HalfTerrainMapSize;
	float ResolutionFactor;	
	
	float MetresTotalRange;
	InputMapRange InputCoordRange; //Unreal input range of coordinates. -1, 40 means -1metre X and 40 metres y direction
	OutputMapRange OutputCoordRange;	

	float * CombinedMapData;
	std::atomic_flag mapLock = ATOMIC_FLAG_INIT;
	
};
