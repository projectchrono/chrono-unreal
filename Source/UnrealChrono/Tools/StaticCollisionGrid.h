#pragma once

#include "CoreMinimal.h"
#include "DataTypes.h"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"
#include <atomic>
#include <memory>
#include <vector>

namespace chrono
{
	class ChSystem;
	class ChBody;
}

using namespace PhysicsDataTypes;

//A collider body stored within a grid cell along with its location and extent in Unreal space for debug visualization
struct FStaticCollisionBody
{
	std::shared_ptr<chrono::ChBody> Shape;
	FVector Location = FVector::ZeroVector;
	FVector Extent = FVector(50.f, 50.f, 50.f);
	FQuat Rotation = FQuat::Identity;
};

//A single cell of the static collision grid. Holds a range into the flat body list of the grid
struct FStaticCollisionCell
{
	double CenterX = 0.0; //chrono metres
	double CenterY = 0.0;
	int32 FirstBody = 0;
	int32 NumBodies = 0;
	//Only ever touched by the physics thread
	bool Active = false;
	int32 ApplyCursor = 0;
};

//Cell/grid based streaming of the baked static collision. All bodies are created up front but only the
//bodies of the cells close enough to the vehicle are attached to the chrono system, everything else stays
//out of the solver and out of the bullet collision world. This keeps the per step cost proportional to the
//colliders around the vehicle instead of the 180k colliders of the whole map.
//
//Threading contract:
//	*	Build/Reset are called from the game thread while the physics thread is not running
//	*	SetVehicleLocation is safe from any thread (lock free)
//	*	Evaluate runs on the streaming worker thread, it never touches chrono, it only decides which cells
//		should be active and pushes them onto a small guarded queue
//	*	ApplyPending must only be called from the physics thread (it mutates the chrono system)
class FStaticCollisionGrid
{
public:
	static FStaticCollisionGrid& Get();

	//Cell size, activation radius and hysteresis are all in metres (chrono units)
	void SetTuning(double CellSizeMetres, double ActivationRadiusMetres, double HysteresisMetres);
	void SetEnableChronoVisual(bool bEnable) { bEnableChronoVisual.store(bEnable, std::memory_order_relaxed); }
	bool GetEnableChronoVisual() const { return bEnableChronoVisual.load(std::memory_order_relaxed); }

	//Buckets every collider by its world XY position. Must be called before the physics thread starts
	void Build(const std::vector<std::shared_ptr<FCollisionModel>>& Colliders);
	void Reset();

	bool IsBuilt() const { return bBuilt.load(std::memory_order_acquire); }
	int32 GetCellCount() const { return Cells.Num(); }
	int32 GetBodyCount() const { return CellBodies.Num(); }
	int32 GetActiveBodyCount() const { return ActiveBodies.load(std::memory_order_relaxed); }
	int32 GetActiveCellCount() const { return ActiveCells.load(std::memory_order_relaxed); }

	//Lock free publish of the position the streaming decisions are based on
	void SetVehicleLocation(double X, double Y, double Z);

	//Decides which cells should be active. Cheap, branch light pass over the cell centres
	void Evaluate();

	//Applies at most MaxBodyOps enable/disable operations, returns the number of operations performed
	int32 ApplyPending(chrono::ChSystem* System, int32 MaxBodyOps);
	//Drains the whole queue, used for the initial state on begin play
	void ApplyAllPending(chrono::ChSystem* System);

private:
	static void ActivateBody(chrono::ChSystem* System, const std::shared_ptr<chrono::ChBody>& Body);
	static void DeactivateBody(chrono::ChSystem* System, const std::shared_ptr<chrono::ChBody>& Body);

	TArray<FStaticCollisionCell> Cells;
	TArray<std::shared_ptr<chrono::ChBody>> CellBodies;

	//Worker thread owned copy of the last decision per cell
	TArray<uint8> DesiredActive;

	//Guards the grid structure itself (cells/bodies) against Build/Reset while a thread is using it
	mutable FCriticalSection StructureLock;
	//Guarded hand over to the physics thread
	mutable FCriticalSection PendingLock;
	TArray<int32> PendingCells;
	TArray<uint8> PendingState;
	int32 PendingHead = 0;

	//Physics thread owned partial application state
	int32 InProgressCell = INDEX_NONE;
	uint8 InProgressState = 0;

	std::atomic<double> VehicleX{ 0.0 };
	std::atomic<double> VehicleY{ 0.0 };
	std::atomic<double> VehicleZ{ 0.0 };
	std::atomic<int32> ActiveBodies{ 0 };
	std::atomic<int32> ActiveCells{ 0 };

	double CellSize = 64.0;
	double CellHalfDiagonal = 64.0 * 0.70710678;
	double ActivationRadius = 250.0;
	double Hysteresis = 50.0;
	std::atomic_bool bBuilt{ false };
	std::atomic_bool bEnableChronoVisual{ true };
};

//Periodically asks the grid which cells should be active. Deliberately does no chrono work at all
class FStaticCollisionGridThread : public FRunnable
{
public:
	FStaticCollisionGridThread(FString Name, float IntervalSecondsInit);
	virtual ~FStaticCollisionGridThread() override;

	bool Init() override;
	uint32 Run() override;
	void Stop() override;
	void Exit() override;

	std::atomic_bool Running = false;

private:
	FRunnableThread* WorkerThread = nullptr;
	std::atomic_bool KeepRunning = true;
	float IntervalSeconds = 0.1f;
};
