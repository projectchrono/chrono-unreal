#include "ChronoTerrain.h"

#include "ChronoIncludeBegin.h"
#include "chrono/serialization/ChArchiveBinary.h"
#include "ChronoIncludeEnd.h"

#ifndef TINYEXR_IMPLEMENTATION
#define TINYEXR_IMPLEMENTATION
#endif
//#include "tinyexr.h"
#include <omp.h>
#include <filesystem>

//#pragma optimize("", off)

namespace fs = std::filesystem;

using namespace chrono;
using namespace chrono::vehicle;

bool ChronoTerrain::LoadMaps()
{
	//int OutHeight, OutWidth;	
	//bool MapLoaded = LoadCombinedMap(CombinedMapName, OutWidth, OutHeight) && OutWidth == OutHeight;
	//MapLoaded |= LoadFrictionMap(FrictionMapName, OutWidth, OutHeight) && OutWidth == OutHeight;
	
	return true;
}

chrono::vehicle::ChTerrain* ChronoTerrain::GetTerrain()
{
	return this;
}

std::shared_ptr<chrono::vehicle::RigidTerrain::Patch> ChronoTerrain::AddNewTerrainPatch(std::shared_ptr<chrono::ChContactMaterial> material, const chrono::ChCoordsys<>& position, const std::string& mesh_file, bool connected_mesh,
                                                                                        double sweep_sphere_radius, bool visualization, bool Serialize, const fs::path TerrainFile)
{
	auto Patch = AddPatch(material, position, mesh_file, connected_mesh, sweep_sphere_radius, visualization);	
	
	if (Serialize)
	{		
		
		auto MeshPatch = std::dynamic_pointer_cast<chrono::vehicle::RigidTerrain::MeshPatch>(Patch);
		std::string filename = TerrainFile.string() + "_serialized.bin";
		std::ofstream file(filename, std::ios::binary);
		chrono::ChArchiveOutBinary archive(file);
		archive << CHNVP(MeshPatch->m_trimesh);
		file.close();		
	}
	return Patch;
	
}

//-----------------------------------------------------------------------------
// Terrain sensing through the Bullet collision world
//-----------------------------------------------------------------------------

bool ChronoTerrain::QueryTerrain(const ChVector3d& Loc, FTerrainQueryResult& Out) const
{
	Out = FTerrainQueryResult();

	const ChVector3d Vertical = ChWorldFrame::Vertical();
	const ChVector3d From = Loc + Vertical * RayUpMetres;
	const ChVector3d To = Loc - Vertical * RayDownMetres;

	//Bullet ray cast in the TERRAIN_QUERY_GROUP family: a body answers the query only if its family mask contains that
	//bit. The vehicle chassis (VEHICLE_MASK) never does, the level geometry (STATIC_MASK) and the RigidTerrain patches
	//(default mask, all families) do
	auto BulletSystem = ChronoSystem ? std::dynamic_pointer_cast<ChCollisionSystemBullet>(ChronoSystem->GetCollisionSystem()) : nullptr;
	if (BulletSystem)
	{
		ChCollisionSystem::ChRayhitResult Hit;
		if (BulletSystem->RayHit(From, To, Hit, (short)TERRAIN_QUERY_GROUP, (short)0x7FFF) && Hit.hit)
		{
			Out.bHit = true;
			Out.Point = Hit.abs_hitPoint;
			Out.Normal = Hit.abs_hitNormal;
			Out.Normal.Normalize();
			if (Vdot(Out.Normal, Vertical) < 0)
			{
				Out.Normal = -Out.Normal;
			}
			if (Hit.hitModel && Hit.hitModel->GetNumShapes() > 0)
			{
				const std::shared_ptr<ChCollisionShape>& Shape = Hit.hitModel->GetShapeInstance(0).shape;
				if (Shape && Shape->GetMaterial())
				{
					Out.Friction = Shape->GetMaterial()->GetSlidingFriction();
				}
			}
			return true;
		}
	}

	//No Bullet hit: fall back to the RigidTerrain patch lookup (also covers the case where the collision system is
	//not initialized yet)
	ChVector3d Point;
	double Height;
	ChVector3d Normal;
	float Friction;
	if (FindPoint(Loc, Point, Height, Normal, Friction))
	{
		Out.bHit = true;
		Out.Point = Point;
		Out.Normal = Normal;
		Out.Friction = Friction;
		return true;
	}
	return false;
}

double ChronoTerrain::GetHeight(const ChVector3d& loc) const
{
	if (m_height_fun)
	{
		return (*m_height_fun)(loc);
	}
	FTerrainQueryResult Result;
	return QueryTerrain(loc, Result) ? ChWorldFrame::Height(Result.Point) : 0.0;
}

ChVector3d ChronoTerrain::GetNormal(const ChVector3d& loc) const
{
	if (m_normal_fun)
	{
		return (*m_normal_fun)(loc);
	}
	FTerrainQueryResult Result;
	return QueryTerrain(loc, Result) ? Result.Normal : ChWorldFrame::Vertical();
}

float ChronoTerrain::GetCoefficientFriction(const ChVector3d& loc) const
{
	if (m_friction_fun)
	{
		return (*m_friction_fun)(loc);
	}
	FTerrainQueryResult Result;
	return QueryTerrain(loc, Result) ? Result.Friction : 0.8f;
}

void ChronoTerrain::GetProperties(const ChVector3d& loc, ChVector3d& point, double& height, ChVector3d& normal, float& friction) const
{
	if (m_height_fun || m_normal_fun || m_friction_fun)
	{
		RigidTerrain::GetProperties(loc, point, height, normal, friction);
		return;
	}

	FTerrainQueryResult Result;
	if (QueryTerrain(loc, Result))
	{
		point = Result.Point;
		height = ChWorldFrame::Height(Result.Point);
		normal = Result.Normal;
		friction = Result.Friction;
		return;
	}

	point = loc;
	ChWorldFrame::Project(point);
	height = 0.0;
	normal = ChWorldFrame::Vertical();
	friction = 0.8f;
}

void ChronoTerrain::SetupCollision()
{
}

bool ChronoTerrain::LoadCombinedMap(const char * FileName, int& OutWidth, int& OutHeight)
{
	TerrainMapSize = OutWidth;
	HalfTerrainMapSize = TerrainMapSize / 2;	

	HeightFunc = std::make_shared<UEHeightFunctor>(CombinedMapData, 128, TerrainMapSize, 0, 1, ChronoSystem);
	NormalFunc = std::make_shared<UENormalFunctor>(CombinedMapData, 128, TerrainMapSize, ChronoSystem);	

	RegisterHeightFunctor(HeightFunc);
	RegisterNormalFunctor(NormalFunc);	
	
	return true; 
}


bool ChronoTerrain::LoadFrictionMap(const char *FileName, int& OutWidth, int& OutHeight)
{
	return true;
}

ETerrainType ChronoTerrain::PhysMaterialToTerrain()
{
	return ETerrainType::Asphalt;	
}

//#pragma optimize("", on)
