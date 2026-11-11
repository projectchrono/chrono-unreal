#pragma once

#include "ChronoTypes.h"
#include <Common.h>
#include "ChronoVehicle.h"
#include "ChronoSimulation.h"
#include "Terrain/TerrainTypes.h"
#include "../Tools/VehicleSnapshot.h"
#include "../Benchmark/BackendComparisonTypes.h"
#include <memory>
#include <string>
#include <vector>

class TestCar;
class Driver;

class ChronoTerrain;
class ChaosTerrain;

//Everything FChronoVehicleWorld::Build needs, gathered on the game thread (plain data, no UObject access on the
//physics thread).
struct FChronoVehicleSetup
{
	//Collision backend the hosting FChronoSimulation was created with
	EChronoCollisionBackend Backend = EChronoCollisionBackend::Chaos;

	//Vehicle model parameters (suspension geometry, engine maps, tire model, ...)
	std::shared_ptr<FChVehicleSetup> Vehicle;

	//Chassis start pose (chrono frame, metres)
	ChCoordsysd StartPose = ChCoordsysd(ChVector3d(0, 0, 0), ChQuaterniond(1, 0, 0, 0));

	//Driver mass (lbs) added to the vehicle mass
	double DriverMassLbs = 180.0;

	//Bullet only: directory with the RigidTerrain mesh patches (data/vehicle/MapData/<Map>)
	std::string MapDataDir;

	//Adds a flat rigid terrain box patch at this height (chrono metres) - used by the automation tests, which have
	//no level geometry. Ignored when bAddFlatPatch is false.
	bool bAddFlatPatch = false;
	double FlatPatchHeight = 0.0;
	double FlatPatchSize = 1000.0;

	//Unreal physical materials the tires may drive on (identity + surface type), captured on the game thread. The
	//terrain resolves the surface type of a hit by pointer comparison only.
	std::vector<FTerrainMaterialEntry> Materials;

	//Bullet only: static colliders are streamed in/out of the system per grid cell (FStaticCollisionGrid)
	bool bStreamStaticColliders = false;
	int StreamingBudgetPerStep = 128;

	//Bullet only: threads of the Bullet collision system
	int CollisionThreads = 8;

	//Backend comparison: when non empty the driver inputs come from this script (indexed by the time since the vehicle was
	//built) instead of the game thread, and every substep is recorded into Recording.
	TArray<BackendCompare::FScriptKey> InputScript;
	TSharedPtr<BackendCompare::FRunRecording> Recording;

};

//Physics-thread owner of the Chrono vehicle model. Installed as the FChronoSimulation stepper: every substep it
//synchronizes driver / terrain / vehicle / trailer, advances them and steps the hosted system; every physics step
//it publishes an immutable FVehicleSnapshot payload that the Unreal actors read on the game thread.
//
//All methods run on the physics thread (from FChronoSimulation commands or the sim callback) and never touch
//UObjects.
class FChronoVehicleWorld : public IChronoStepper
{
public:
	FChronoVehicleWorld();
	virtual ~FChronoVehicleWorld() override;

	//Creates vehicle, driver, terrain and trailer inside the hosted system. Returns false (and logs) on failure.
	bool Build(chrono::ChSystem& System, const FChronoVehicleSetup& InSetup);
	bool IsBuilt() const { return Vehicle != nullptr; }

	// IChronoStepper
	virtual void SubStep(FChronoStepContext& Context) override;
	virtual TSharedPtr<const FChronoSnapshotPayload> MakePayload(const chrono::ChSystem& System) override;

	//Commands (enqueue through FChronoSimulation)
	void RequestShift(int Direction);
	void ToggleAxleDiffLock(bool FrontAxle, bool Locked);
	void ToggleCenterDiffLock(bool Locked);
	void SetStreamStaticColliders(bool bStream, int BudgetPerStep);

	TestCar* GetVehicle() const { return Vehicle.get(); }
	std::shared_ptr<chrono::vehicle::ChTerrain> GetTerrain() const { return Terrain; }
	const INextGenTerrain* GetTerrainQuery() const { return TerrainQuery; }
	const FChronoVehicleSetup& GetSetup() const { return Setup; }

private:
	void BuildTerrain(chrono::ChSystem& System);
	bool FindTerrainMaps(chrono::ChContactMethod ContactMethod, const std::string& MapDirectory);
	void FillTires(FVehicleSnapshot& Snapshot) const;

	FChronoVehicleSetup Setup;

	//Chrono time of the first substep after the build (comparison scripts / recording start there)
	double ScriptStartTime = -1.0;
	void Record(double LocalTimeEnd);
	void ProbeTerrain();
	bool bProbed = false;

	std::unique_ptr<TestCar> Vehicle;
	std::shared_ptr<Driver> VehicleDriver;
	//ChaosTerrain (Chaos backend) or ChronoTerrain (Bullet backend)
	std::shared_ptr<chrono::vehicle::ChTerrain> Terrain;
	const INextGenTerrain* TerrainQuery = nullptr;
	std::shared_ptr<ChronoTerrain> BulletTerrain;
	std::vector<std::shared_ptr<chrono::vehicle::RigidTerrain::Patch>> Patches;
};
