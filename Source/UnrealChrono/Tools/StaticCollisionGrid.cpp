#include "StaticCollisionGrid.h"
#include "ChronoTypes.h"
#include "HAL/PlatformProcess.h"
#include "Conversions.h"

#if WITH_EDITOR
#include "Chaos/DebugDrawQueue.h"
#endif

FStaticCollisionGrid& FStaticCollisionGrid::Get()
{
	static FStaticCollisionGrid Instance;
	return Instance;
}

void FStaticCollisionGrid::SetTuning(double CellSizeMetres, double ActivationRadiusMetres, double HysteresisMetres)
{
	CellSize = FMath::Max(1.0, CellSizeMetres);
	CellHalfDiagonal = CellSize * 0.70710678118;
	ActivationRadius = FMath::Max(0.0, ActivationRadiusMetres);
	Hysteresis = FMath::Max(0.0, HysteresisMetres);
}

void FStaticCollisionGrid::Build(const std::vector<std::shared_ptr<FCollisionModel>>& Colliders)
{
	Reset();

	const int32 Num = (int32)Colliders.size();
	if (Num == 0) return;

	FScopeLock StructureScope(&StructureLock);

	//First pass, bucket every collider index by its cell. The map is only used while building,
	//the runtime pass works on the flat arrays below
	TMap<int64, int32> CellLookup;
	CellLookup.Reserve(Num / 8 + 1);
	TArray<TArray<int32>> Buckets;

	for (int32 i = 0; i < Num; ++i)
	{
		const std::shared_ptr<FCollisionModel>& Model = Colliders[i];
		if (!Model || !Model->Shape) continue;

		//Bucket by the shape's world-space mesh transform. For instanced static meshes this is each
		//instance's world position; for landscape patches this is the collision component world position
		const double X = Model->ColliderInfo.Position.x();
		const double Y = Model->ColliderInfo.Position.y();
		const int32 CellX = (int32)FMath::FloorToDouble(X / CellSize);
		const int32 CellY = (int32)FMath::FloorToDouble(Y / CellSize);
		const int64 Key = ((int64)CellX << 32) | (uint32)CellY;

		int32* Existing = CellLookup.Find(Key);
		int32 CellIndex;
		if (Existing)
		{
			CellIndex = *Existing;
		}
		else
		{
			FStaticCollisionCell NewCell;
			NewCell.CenterX = ((double)CellX + 0.5) * CellSize;
			NewCell.CenterY = ((double)CellY + 0.5) * CellSize;
			CellIndex = Cells.Add(NewCell);
			Buckets.AddDefaulted();
			CellLookup.Add(Key, CellIndex);
		}

		Buckets[CellIndex].Add(i);
	}

	//Second pass, flatten the buckets so every cell owns a contiguous range of bodies.
	//Contiguous ranges keep the enable/disable pass cache friendly
	CellBodies.Reserve(Num);
	for (int32 CellIndex = 0; CellIndex < Cells.Num(); ++CellIndex)
	{
		Cells[CellIndex].FirstBody = CellBodies.Num();
		for (const int32 ColliderIndex : Buckets[CellIndex])
		{
			CellBodies.Add(Colliders[ColliderIndex]->Shape);			
		}
		Cells[CellIndex].NumBodies = CellBodies.Num() - Cells[CellIndex].FirstBody;
	}

	DesiredActive.Init(0, Cells.Num());
	PendingState.Init(0, Cells.Num());
	bBuilt.store(Cells.Num() > 0, std::memory_order_release);

	UE_LOG(LogTemp, Log, TEXT("StaticCollisionGrid: %d colliders bucketed into %d cells of %.0fm (activation radius %.0fm)"),
		CellBodies.Num(), Cells.Num(), CellSize, ActivationRadius);
}

void FStaticCollisionGrid::Reset()
{
	//Take the structure lock first so neither the worker nor the physics thread can be inside the grid
	bBuilt.store(false, std::memory_order_release);
	FScopeLock StructureScope(&StructureLock);
	FScopeLock Lock(&PendingLock);
	Cells.Reset();
	CellBodies.Reset();
	DesiredActive.Reset();
	PendingCells.Reset();
	PendingState.Reset();
	PendingHead = 0;
	InProgressCell = INDEX_NONE;
	InProgressState = 0;
	ActiveBodies.store(0, std::memory_order_relaxed);
	ActiveCells.store(0, std::memory_order_relaxed);
}

void FStaticCollisionGrid::SetVehicleLocation(double X, double Y, double Z)
{
	VehicleX.store(X, std::memory_order_relaxed);
	VehicleY.store(Y, std::memory_order_relaxed);
	VehicleZ.store(Z, std::memory_order_relaxed);
}

void FStaticCollisionGrid::Evaluate()
{
	FScopeLock StructureScope(&StructureLock);
	if (!bBuilt.load(std::memory_order_acquire)) return;

	const double VX = VehicleX.load(std::memory_order_relaxed);
	const double VY = VehicleY.load(std::memory_order_relaxed);

	//Hysteresis, a cell has to move further out than it came in before it is dropped again.
	//This stops cells thrashing when the vehicle drives along a cell border
	const double EnterDistance = ActivationRadius + CellHalfDiagonal;
	const double ExitDistance = ActivationRadius + Hysteresis + CellHalfDiagonal;
	const double EnterDistanceSq = EnterDistance * EnterDistance;
	const double ExitDistanceSq = ExitDistance * ExitDistance;

	TArray<int32, TInlineAllocator<64>> Changed;
	const int32 NumCells = Cells.Num();
	const FStaticCollisionCell* CellData = Cells.GetData();
	uint8* DesiredData = DesiredActive.GetData();

	//Purely horizontal test, the map is only ~200m tall so the height adds nothing useful here
	for (int32 i = 0; i < NumCells; ++i)
	{
		const double DX = CellData[i].CenterX - VX;
		const double DY = CellData[i].CenterY - VY;
		const double DistSq = DX * DX + DY * DY;

		const bool bWasActive = DesiredData[i] != 0;
		const bool bDesired = bWasActive ? (DistSq <= ExitDistanceSq) : (DistSq <= EnterDistanceSq);

		if (bDesired != bWasActive)
		{
			DesiredData[i] = bDesired ? 1 : 0;
			Changed.Add(i);
		}
	}

	if (Changed.Num() == 0) return;

	FScopeLock Lock(&PendingLock);

	//Compact the queue instead of shifting on every pop
	if (PendingHead > 0)
	{
		PendingCells.RemoveAt(0, PendingHead, EAllowShrinking::No);
		PendingHead = 0;
	}

	for (const int32 CellIndex : Changed)
	{
		PendingState[CellIndex] = DesiredActive[CellIndex];
		PendingCells.Add(CellIndex);
	}
}

int32 FStaticCollisionGrid::ApplyPending(chrono::ChSystem* System, int32 MaxBodyOps)
{
	if (!System || MaxBodyOps <= 0) return 0;

	//Only contended on begin play/end play, in the steady state this is an uncontended lock per step
	FScopeLock StructureScope(&StructureLock);
	if (!bBuilt.load(std::memory_order_acquire)) return 0;

	int32 Ops = 0;
	int32 CellDelta = 0;
	int32 BodyDelta = 0;

	while (Ops < MaxBodyOps)
	{
		//Pick up whatever cell we were in the middle of, otherwise pop the next queued one
		if (InProgressCell == INDEX_NONE)
		{
			FScopeLock Lock(&PendingLock);
			if (PendingHead >= PendingCells.Num()) break;

			InProgressCell = PendingCells[PendingHead++];
			if (!Cells.IsValidIndex(InProgressCell))
			{
				//Stale entry left over from a rebuilt grid, drop it
				InProgressCell = INDEX_NONE;
				continue;
			}
			InProgressState = PendingState[InProgressCell];
			//A freshly popped cell always starts from the beginning. Activate/Deactivate are idempotent so
			//a cell that flipped state while it was half applied is simply re-applied from the start
			Cells[InProgressCell].ApplyCursor = 0;
		}

		FStaticCollisionCell& Cell = Cells[InProgressCell];
		const bool bActivate = InProgressState != 0;

		//Already in the requested state, nothing to do
		if (Cell.Active == bActivate)
		{
			InProgressCell = INDEX_NONE;
			continue;
		}

		while (Cell.ApplyCursor < Cell.NumBodies && Ops < MaxBodyOps)
		{
			const std::shared_ptr<chrono::ChBody>& Body = CellBodies[Cell.FirstBody + Cell.ApplyCursor];
			if (bActivate)
			{
				ActivateBody(System, Body);
#if WITH_EDITOR
				if (bEnableChronoVisual.load(std::memory_order_relaxed))
				{
					Chaos::FDebugDrawQueue::GetInstance().DrawDebugBox(Conversions::ToUnrealPos(Body->GetPos()),
						FVector(10,10,10), FQuat::Identity,FColor::Yellow,false,5.0f,1,1.0f	);
				}
#endif
			}
			else
			{
				DeactivateBody(System, Body);
			}
			++Cell.ApplyCursor;
			++Ops;
		}

		if (Cell.ApplyCursor >= Cell.NumBodies)
		{
			//Cell fully switched over
			Cell.ApplyCursor = 0;
			if (Cell.Active != bActivate)
			{
				Cell.Active = bActivate;
				CellDelta += bActivate ? 1 : -1;
				BodyDelta += bActivate ? Cell.NumBodies : -Cell.NumBodies;
			}
			InProgressCell = INDEX_NONE;
		}
	}

	if (CellDelta != 0) ActiveCells.fetch_add(CellDelta, std::memory_order_relaxed);
	if (BodyDelta != 0) ActiveBodies.fetch_add(BodyDelta, std::memory_order_relaxed);

	return Ops;
}

void FStaticCollisionGrid::ApplyAllPending(chrono::ChSystem* System)
{
	while (ApplyPending(System, MAX_int32) > 0) {}
}

//Streaming a cell in/out actually adds/removes the bodies from the
//chrono system body list, so ChSystem::Update (and the solver setup) never iterates the inactive/fixed
//static colliders at all - this is what makes the cost proportional to the colliders around the vehicle
//instead of the whole map. This must only ever run on the physics thread, between solver steps, which is
//guaranteed because ApplyPending is only called from the physics step.
void FStaticCollisionGrid::ActivateBody(chrono::ChSystem* System, const std::shared_ptr<chrono::ChBody>& Body)
{
	if (!System || !Body) return;


	//Add the body back into the system body list. ChSystem::AddBody asserts the body is detached, so guard
	//on the current system to keep the operation idempotent (ApplyPending may re-run a half applied cell)
	if (Body->GetSystem() == nullptr)
	{
		System->AddBody(Body);
	}

	//ChSystem::AddBody does not bind the collision model, so register it explicitly with the (live)
	//collision system. Guard on HasImplementation so a re-applied cell never double adds a model
	if (const std::shared_ptr<chrono::ChCollisionSystem>& CollSys = System->GetCollisionSystem())
	{
		const std::shared_ptr<chrono::ChCollisionModel>& Model = Body->GetCollisionModel();
		if (Model && CollSys->IsInitialized() && !Model->HasImplementation())
		{
			CollSys->Add(Model);
		}
	}
}

void FStaticCollisionGrid::DeactivateBody(chrono::ChSystem* System, const std::shared_ptr<chrono::ChBody>& Body)
{
	if (!System || !Body) return;


	//Remove the body from the system body list. ChSystem::RemoveBody also unbinds the collision model from
	//the collision system, so the bullet broadphase only ever sees the nearby colliders. Guard on the
	//current system so a re-applied cell never removes a body twice
	if (Body->GetSystem() != nullptr)
	{
		System->RemoveBody(Body);
	}
}

//-----------------------------------------------------------------------------

FStaticCollisionGridThread::FStaticCollisionGridThread(FString Name, float IntervalSecondsInit)
{
	IntervalSeconds = FMath::Max(0.01f, IntervalSecondsInit);
	WorkerThread = FRunnableThread::Create(this, *Name, 0, TPri_BelowNormal);
}

FStaticCollisionGridThread::~FStaticCollisionGridThread()
{
	if (WorkerThread)
	{
		WorkerThread->Kill(true);
		delete WorkerThread;
		WorkerThread = nullptr;
	}
}

bool FStaticCollisionGridThread::Init()
{
	return true;
}

uint32 FStaticCollisionGridThread::Run()
{
	Running = true;
	while (KeepRunning)
	{
		FStaticCollisionGrid::Get().Evaluate();
		FPlatformProcess::Sleep(IntervalSeconds);
	}
	Running = false;
	return 0;
}

void FStaticCollisionGridThread::Stop()
{
	KeepRunning = false;
}

void FStaticCollisionGridThread::Exit()
{
	KeepRunning = false;
}
