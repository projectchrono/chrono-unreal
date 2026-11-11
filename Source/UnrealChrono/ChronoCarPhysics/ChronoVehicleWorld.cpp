#include "ChronoVehicleWorld.h"

#include "TestCar/TestCar.h"
#include "Driver.h"
#include "Terrain/ChronoTerrain.h"
#include "Terrain/ChaosTerrain.h"
#include "Tools/Conversions.h"
#include "../Tools/StaticCollisionGrid.h"

#include <filesystem>

DEFINE_LOG_CATEGORY_STATIC(LogChronoVehicleWorld, Log, All);

using namespace chrono;
using namespace chrono::vehicle;

FChronoVehicleWorld::FChronoVehicleWorld()
{
}

FChronoVehicleWorld::~FChronoVehicleWorld()
{
	//Destruction order matters: the trailer / driver / terrain reference the vehicle and the system. The stepper is
	//destroyed on the physics thread while the system is still alive (FChronoSimCallback::DestroySystem_Internal),
	//so the subsystems can still remove their bodies from it.
	VehicleDriver.reset();
	Patches.clear();
	TerrainQuery = nullptr;
	BulletTerrain.reset();
	Terrain.reset();
	Vehicle.reset();
}

bool FChronoVehicleWorld::Build(ChSystem& System, const FChronoVehicleSetup& InSetup)
{
	Setup = InSetup;

	if (!Setup.Vehicle)
	{
		UE_LOG(LogChronoVehicleWorld, Error, TEXT("FChronoVehicleWorld::Build: no vehicle setup"));
		return false;
	}

	try
	{
		//Collision backend. The hosting FChronoSimulation created the system with the requested backend already,
		//this only makes sure the vehicle records the same type (and sets up Bullet's threads).
		const ChCollisionSystem::Type CollisionType = Setup.Backend == EChronoCollisionBackend::Chaos
			? ChCollisionSystem::Type::CHAOS
			: ChCollisionSystem::Type::BULLET;
		if (!System.GetCollisionSystem() || ChronoVehicle::DetectCollisionSystemType(&System) != CollisionType)
		{
			System.SetCollisionSystemType(CollisionType);
		}
		if (CollisionType == ChCollisionSystem::Type::BULLET && System.GetCollisionSystem())
		{
			System.GetCollisionSystem()->SetNumThreads(Setup.CollisionThreads);
		}

		//Vehicle on the hosted system
		Vehicle = std::make_unique<TestCar>("TestCar", &System);
		Vehicle->VehicleSetup = Setup.Vehicle;
		Vehicle->SetCollisionSystemType(CollisionType);

		//Driver
		VehicleDriver = chrono_types::make_shared<Driver>(Vehicle.get(), Setup.DriverMassLbs);

		//Terrain: ChaosTerrain senses the Unreal level through the Chaos scene, ChronoTerrain the Bullet world
		BuildTerrain(System);

		Vehicle->Create();
		Vehicle->Initialize(Setup.StartPose, 0.0);

		if (Setup.Recording)
		{
			using namespace BackendCompare;
			FScopeLock ScopeLock(&Setup.Recording->Lock);
			Setup.Recording->BodyNames = { TEXT("chassis"), TEXT("spindle_FL"), TEXT("spindle_FR"), TEXT("spindle_RL"), TEXT("spindle_RR") };
			Setup.Recording->BodyMass = { Vehicle->GetChassisBody()->GetMass(), Vehicle->FL_Wheel->GetSpindle()->GetMass(), Vehicle->FR_Wheel->GetSpindle()->GetMass(),
				Vehicle->RL_Wheel->GetSpindle()->GetMass(), Vehicle->RR_Wheel->GetSpindle()->GetMass() };
			Setup.Recording->ExtraNames = { TEXT("speed_ms"), TEXT("engine_rpm"), TEXT("gear"), TEXT("terrain_friction_FL"), TEXT("terrain_height_FL_m"), TEXT("terrain_type_FL") };
		}
		Vehicle->SetTerrain(Terrain);

		VehicleDriver->Init();

		//Never enable ChVehicle's realtime throttling here: the step runs inside Chaos' fixed rate callback and must not
		//spin-wait on wall clock time.
		Vehicle->EnableRealtime(false);
	}
	catch (const std::exception& Ex)
	{
		UE_LOG(LogChronoVehicleWorld, Error, TEXT("FChronoVehicleWorld::Build: exception while building the vehicle: %hs"), Ex.what());
		VehicleDriver.reset();
		TerrainQuery = nullptr;
		BulletTerrain.reset();
		Terrain.reset();
		Vehicle.reset();
		return false;
	}

	UE_LOG(LogChronoVehicleWorld, Log, TEXT("FChronoVehicleWorld::Build: vehicle built on the %s backend (%d bodies in the system)"),
		Setup.Backend == EChronoCollisionBackend::Chaos ? TEXT("Chaos") : TEXT("Bullet"), (int32)System.GetBodies().size());
	return true;
}

void FChronoVehicleWorld::BuildTerrain(ChSystem& System)
{
	if (Setup.Backend == EChronoCollisionBackend::Chaos && !Setup.bAddFlatPatch)
	{
		//The Unreal level is the terrain: no Chrono side geometry at all
		std::shared_ptr<ChaosTerrain> Chaos = std::make_shared<ChaosTerrain>(&System, Setup.Materials);
		TerrainQuery = Chaos.get();
		Terrain = Chaos;
		return;
	}

	//Bullet (or a test patch on either backend): RigidTerrain patches + Bullet ray casts
	BulletTerrain = std::make_shared<ChronoTerrain>(&System, Setup.MapDataDir.c_str(), Setup.Vehicle->TireModel, Setup.Materials);
	TerrainQuery = BulletTerrain.get();
	Terrain = BulletTerrain;

	ChContactMaterialData minfo;
	minfo.mu = 0.9f;
	minfo.cr = 0.01f;
	minfo.Y = 2e7f;
	auto patch_mat = minfo.CreateMaterial(System.GetContactMethod());

	if (Setup.bAddFlatPatch)
	{
		const ChCoordsysd PatchPos(ChVector3d(0, 0, Setup.FlatPatchHeight), ChQuaterniond(1, 0, 0, 0));
		std::shared_ptr<RigidTerrain::Patch> Patch = BulletTerrain->AddPatch(patch_mat, PatchPos, Setup.FlatPatchSize, Setup.FlatPatchSize, 1.0, false, 1.0, false);
		if (Patch)
		{
			Patches.push_back(Patch);
		}
	}

	if (Setup.Backend == EChronoCollisionBackend::Bullet && !Setup.MapDataDir.empty())
	{
		FindTerrainMaps(System.GetContactMethod(), Setup.MapDataDir);
	}

	BulletTerrain->Initialize();
}

bool FChronoVehicleWorld::FindTerrainMaps(ChContactMethod ContactMethod, const std::string& MapDirectory)
{
	if (!BulletTerrain)
	{
		return false;
	}

	namespace fs = std::filesystem;

	ChContactMaterialData minfo;
	minfo.mu = 0.9f;
	minfo.cr = 0.01f;
	minfo.Y = 2e7f;
	std::shared_ptr<ChContactMaterial> patch_mat = minfo.CreateMaterial(ContactMethod);

	const fs::path BaseDir(MapDirectory);
	if (!fs::exists(BaseDir) || !fs::is_directory(BaseDir))
	{
		return false;
	}

	std::vector<fs::path> TerrainFiles;
	std::vector<fs::path> SerializedTerrainFiles;
	const std::string SerializedSuffix = "_serialized.bin";

	for (const auto& Entry : fs::directory_iterator(BaseDir))
	{
		if (!Entry.is_regular_file())
		{
			continue;
		}

		const std::string FileName = Entry.path().filename().string();

		if (FileName.ends_with(SerializedSuffix))
		{
			SerializedTerrainFiles.push_back(Entry.path());
			continue;
		}

		if (FileName.rfind("Terrain", 0) == 0)
		{
			fs::path SerializedPath = Entry.path().parent_path() / (FileName + SerializedSuffix);
			if (!fs::exists(SerializedPath))
			{
				TerrainFiles.push_back(Entry.path());
			}
			// If serialized exists, it will be picked up on its own iteration
		}
	}

	const ChCoordsysd StartPos(ChVector3d(0, 0, 0), ChQuaterniond(1, 0, 0, 0));

	for (const auto& SerializedTerrainFile : SerializedTerrainFiles)
	{
		std::shared_ptr<RigidTerrain::Patch> Patch = BulletTerrain->AddNewTerrainPatch(patch_mat, StartPos, SerializedTerrainFile.string().c_str(), true, 0.08, false);
		if (Patch)
		{
			Patches.push_back(Patch);
		}
	}

	for (const auto& TerrainFile : TerrainFiles)
	{
		std::shared_ptr<RigidTerrain::Patch> Patch = BulletTerrain->AddNewTerrainPatch(patch_mat, StartPos, TerrainFile.string().c_str(), true, 0.08, true, true, TerrainFile);
		if (Patch)
		{
			Patches.push_back(Patch);
		}
	}

	return !Patches.empty();
}

void FChronoVehicleWorld::SubStep(FChronoStepContext& Context)
{
	if (!Vehicle)
	{
		Context.System.DoStepDynamics(Context.Dt);
		return;
	}

	//Static collision streaming (Bullet). The grid decides on its own worker thread which cells should be active,
	//here we only attach/detach a limited amount of bodies so a step never stalls on a big batch
	if (Setup.bStreamStaticColliders)
	{
		FStaticCollisionGrid::Get().ApplyPending(&Context.System, Setup.StreamingBudgetPerStep);
	}

	const double Time = Context.Time;

	if (ScriptStartTime < 0.0)
	{
		ScriptStartTime = Time;
	}
	const double LocalTime = Time - ScriptStartTime;
	if (Setup.Recording && !bProbed && LocalTime >= 0.5)
	{
		bProbed = true;
		ProbeTerrain();
	}

	FChronoTickInputs Inputs = Context.Inputs;
	if (Setup.InputScript.Num() > 0)
	{
		//Scripted driver (backend comparison): the inputs are a function of the simulation time only
		const TArray<BackendCompare::FScriptKey>& Keys = Setup.InputScript;
		BackendCompare::FScriptKey Key = Keys[0];
		if (LocalTime >= Keys.Last().Time)
		{
			Key = Keys.Last();
		}
		else if (LocalTime > Keys[0].Time)
		{
			for (int32 i = 0; i + 1 < Keys.Num(); ++i)
			{
				if (LocalTime >= Keys[i].Time && LocalTime < Keys[i + 1].Time)
				{
					const float Alpha = (float)((LocalTime - Keys[i].Time) / FMath::Max(Keys[i + 1].Time - Keys[i].Time, 1e-9f));
					Key.Throttle = FMath::Lerp(Keys[i].Throttle, Keys[i + 1].Throttle, Alpha);
					Key.Steering = FMath::Lerp(Keys[i].Steering, Keys[i + 1].Steering, Alpha);
					Key.Braking = FMath::Lerp(Keys[i].Braking, Keys[i + 1].Braking, Alpha);
					break;
				}
			}
		}
		Inputs.Throttle = Key.Throttle;
		Inputs.Steering = Key.Steering;
		Inputs.Braking = Key.Braking;
		Inputs.Flags &= ~VehicleInputFlags::Handbrake;
	}
	VehicleDriver->SetInputs(Inputs);
	VehicleDriver->Synchronize(Time);
	Terrain->Synchronize(Time);
	Vehicle->Synchronize(Time, VehicleDriver->GetInputs(), *Terrain);
	Vehicle->UpdateVehicleData();

	//Publish the chassis position for the streaming worker (lock free)
	if (Setup.bStreamStaticColliders)
	{
		if (const auto& Chassis = Vehicle->GetChassis())
		{
			const ChVector3d& ChassisPos = Chassis->GetPos();
			FStaticCollisionGrid::Get().SetVehicleLocation(ChassisPos.x(), ChassisPos.y(), ChassisPos.z());
		}
	}

	VehicleDriver->Advance(Context.Dt);
	Terrain->Advance(Context.Dt);
	Vehicle->Advance(Context.Dt);

	//The vehicle does not own the system, so it never steps it (see ChronoVehicle ctor)
	Context.System.DoStepDynamics(Context.Dt);

	if (Setup.Recording)
	{
		Record(LocalTime + Context.Dt);
	}
}

//Queries the terrain below a fixed grid of points (the same on every run) and stores the answers in the recording: the
//terrain sensing of the two backends can then be compared point by point, independent of where the vehicle is.
void FChronoVehicleWorld::ProbeTerrain()
{
	if (!TerrainQuery || !Setup.Recording)
	{
		return;
	}
	TArray<double> Probe;
	for (int32 Row = 0; Row < 3; ++Row)
	{
		const double Y = (Row - 1) * 2.0 + 0.037;
		for (double X = -1.93; X <= 48.0; X += 0.25)
		{
			FTerrainQueryResult R;
			const bool bHit = TerrainQuery->QueryTerrain(ChVector3d(X, Y, 0.5), R) && R.bHit;
			const double Values[BackendCompare::TerrainProbeStride] = { X, Y, bHit ? 1.0 : 0.0, bHit ? R.Point.z() : 0.0, bHit ? R.Normal.x() : 0.0, bHit ? R.Normal.y() : 0.0, bHit ? R.Normal.z() : 0.0,
				bHit ? (double)R.Friction : 0.0, bHit ? (double)(int32)R.TerrainType : -1.0 };
			Probe.Append(Values, BackendCompare::TerrainProbeStride);
		}
	}
	FScopeLock ScopeLock(&Setup.Recording->Lock);
	Setup.Recording->TerrainProbe = MoveTemp(Probe);
}

//Writes one sample of the backend comparison recording (physics thread): chassis + the four spindles
void FChronoVehicleWorld::Record(double LocalTimeEnd)
{
	using namespace BackendCompare;
	FRunRecording& Rec = *Setup.Recording;
	if (Rec.NumBodies() == 0)
	{
		return;
	}

	const std::shared_ptr<ChBody> Bodies[5] =
	{
		Vehicle->GetChassisBody(),
		Vehicle->FL_Wheel->GetSpindle(), Vehicle->FR_Wheel->GetSpindle(), Vehicle->RL_Wheel->GetSpindle(), Vehicle->RR_Wheel->GetSpindle()
	};
	double Data[5 * BodyStride];
	for (int32 b = 0; b < 5; ++b)
	{
		double* D = Data + b * BodyStride;
		const ChVector3d P = Bodies[b]->GetPos();
		const ChQuaterniond Q = Bodies[b]->GetRot();
		const ChVector3d V = Bodies[b]->GetPosDt();
		const ChVector3d W = Bodies[b]->GetAngVelParent();
		D[0] = P.x(); D[1] = P.y(); D[2] = P.z();
		D[3] = Q.e0(); D[4] = Q.e1(); D[5] = Q.e2(); D[6] = Q.e3();
		D[7] = V.x(); D[8] = V.y(); D[9] = V.z();
		D[10] = W.x(); D[11] = W.y(); D[12] = W.z();
	}
	//What the front left tire's terrain query returns on this backend (same ray as the tire's contact model)
	FTerrainQueryResult TerrainHit;
	if (TerrainQuery)
	{
		TerrainQuery->QueryTerrain(Bodies[1]->GetPos(), TerrainHit);
	}
	const double Extra[6] = { Vehicle->m_VehicleData.Speed, Vehicle->m_VehicleData.RPM * 30.0 / PI, (double)Vehicle->m_VehicleData.CurrentGear,
		TerrainHit.bHit ? (double)TerrainHit.Friction : -1.0, TerrainHit.bHit ? TerrainHit.Point.z() : -999.0, TerrainHit.bHit ? (double)(int32)TerrainHit.TerrainType : -1.0 };
	Rec.AddSample(LocalTimeEnd, Data, Extra);
}

namespace
{
	FTransform ToUE(const ChFrame<>& Frame)
	{
		return Conversions::ToUETransform(ChCoordsysd(Frame.GetPos(), Frame.GetRot()));
	}

	bool IsFinite(const ChVector3d& V)
	{
		return std::isfinite(V.x()) && std::isfinite(V.y()) && std::isfinite(V.z());
	}


}

void FChronoVehicleWorld::FillTires(FVehicleSnapshot& Snapshot) const
{
	const ChVehicleData& Data = Vehicle->m_VehicleData;

	FTireSnapshot& FL = Snapshot.Tires[VehicleWheel::FL];
	FTireSnapshot& FR = Snapshot.Tires[VehicleWheel::FR];
	FTireSnapshot& RL = Snapshot.Tires[VehicleWheel::RL];
	FTireSnapshot& RR = Snapshot.Tires[VehicleWheel::RR];

	FL.SlipAngle = Data.FL_SlipAngle;           FR.SlipAngle = Data.FR_SlipAngle;
	RL.SlipAngle = Data.RL_SlipAngle;           RR.SlipAngle = Data.RR_SlipAngle;
	FL.LongitudinalSlip = Data.FL_LongitudinalSlip; FR.LongitudinalSlip = Data.FR_LongitudinalSlip;
	RL.LongitudinalSlip = Data.RL_LongitudinalSlip; RR.LongitudinalSlip = Data.RR_LongitudinalSlip;
	FL.CamberAngle = Data.FL_CamberAngle;       FR.CamberAngle = Data.FR_CamberAngle;
	RL.CamberAngle = Data.RL_CamberAngle;       RR.CamberAngle = Data.RR_CamberAngle;
	FL.Deflection = Data.FL_TireDeflection;     FR.Deflection = Data.FR_TireDeflection;
	RL.Deflection = Data.RL_TireDeflection;     RR.Deflection = Data.RR_TireDeflection;
	FL.ContactPoint = Conversions::ToUnrealPos(Data.FL_ContactPoint);
	FR.ContactPoint = Conversions::ToUnrealPos(Data.FR_ContactPoint);
	RL.ContactPoint = Conversions::ToUnrealPos(Data.RL_ContactPoint);
	RR.ContactPoint = Conversions::ToUnrealPos(Data.RR_ContactPoint);

	//Contact state straight from the tire model (single point / envelope tires do their own terrain query); the surface
	//type and friction come from one extra terrain query at the contact point (physics thread, same ray cast the
	//tires use)
	const std::shared_ptr<ChTire> Tires[VehicleWheel::Num] = { Vehicle->FL_Tire, Vehicle->FR_Tire, Vehicle->RL_Tire, Vehicle->RR_Tire };
	const std::shared_ptr<ChWheel> Wheels[VehicleWheel::Num] = { Vehicle->FL_Wheel, Vehicle->FR_Wheel, Vehicle->RL_Wheel, Vehicle->RR_Wheel };
	for (int32 i = 0; i < VehicleWheel::Num; ++i)
	{
		FTireSnapshot& Tire = Snapshot.Tires[i];
		ChVector3d QueryPoint;
		bool bHaveQueryPoint = false;

		if (auto ForceTire = std::dynamic_pointer_cast<ChForceElementTire>(Tires[i]))
		{
			const ChForceElementTire::ContactData& Contact = ForceTire->ReportTireContactData();
			Tire.bInContact = Contact.in_contact;
			if (Contact.in_contact)
			{
				Tire.TerrainImpactPoint = Conversions::ToUnrealPos(Contact.frame.pos);
				Tire.TerrainNormal = Conversions::ToUnrealNormal(Contact.frame.rot.GetAxisZ());
				Tire.TerrainHeight = Contact.frame.pos.z() * CH2UE_SCALE;
				QueryPoint = Contact.frame.pos;
				bHaveQueryPoint = true;
			}
		}
		else if (Wheels[i] && Wheels[i]->GetSpindle())
		{
			//Rigid (contact based) tires: query straight below the spindle
			QueryPoint = Wheels[i]->GetSpindle()->GetPos();
			bHaveQueryPoint = true;
		}

		//Discrete tire: hand the ground heightfield the contact model used to the game thread (visual tire deformation)
		if (auto DiscreteTire = std::dynamic_pointer_cast<ChDiscreteTire>(Tires[i]))
		{
			const ChDiscreteTire::FGroundView Ground = DiscreteTire->GetGroundView();
			if (Ground.bValid && Ground.Profile && Ground.Profile->numColumns > 0 && Ground.Profile->numStations > 1)
			{
				const DiscreteTire::GroundProfile& Profile = *Ground.Profile;
				Tire.bHasGround = true;
				Tire.GroundColumns = Profile.numColumns;
				Tire.GroundStations = Profile.numStations;
				Tire.GroundReachCm = (float)(std::abs(Profile.stationX.front()) * CH2UE_SCALE);
				Tire.GroundHalfWidthCm = (float)(std::abs(Profile.colY.back()) * CH2UE_SCALE);
				Tire.GroundAxisX = Conversions::ToUnrealNormal(Ground.HeadingX);
				Tire.GroundAxisY = Conversions::ToUnrealNormal(Ground.Axle);
				Tire.GroundAxisZ = Conversions::ToUnrealNormal(Ground.HeadingZ);
				Tire.GroundHeightsCm.SetNumUninitialized((int32)Profile.z.size());
				for (size_t k = 0; k < Profile.z.size(); ++k)
				{
					Tire.GroundHeightsCm[(int32)k] = (float)(Profile.z[k] * CH2UE_SCALE);
				}
			}
		}

		if (bHaveQueryPoint && TerrainQuery)
		{
			FTerrainQueryResult Result;
			if (TerrainQuery->QueryTerrain(QueryPoint, Result))
			{
				Tire.TerrainFriction = Result.Friction;
				Tire.TerrainType = Result.TerrainType;
				if (!Tire.bInContact)
				{
					Tire.TerrainImpactPoint = Conversions::ToUnrealPos(Result.Point);
					Tire.TerrainNormal = Conversions::ToUnrealNormal(Result.Normal);
					Tire.TerrainHeight = ChWorldFrame::Height(Result.Point) * CH2UE_SCALE;
				}
			}
		}
	}

}

TSharedPtr<const FChronoSnapshotPayload> FChronoVehicleWorld::MakePayload(const ChSystem& System)
{
	TSharedRef<FVehicleSnapshot> Snapshot = MakeShared<FVehicleSnapshot>();
	Snapshot->SimTime = System.GetChTime();

	if (!Vehicle || !Vehicle->GetChassis())
	{
		return Snapshot;
	}

	const ChFrame<> ChassisFrame = Vehicle->GetChassis()->GetBody()->GetFrameRefToAbs();
	if (!IsFinite(ChassisFrame.GetPos()))
	{
		return Snapshot;   //bValid stays false
	}

	Snapshot->bValid = true;
	Snapshot->Body = Conversions::ToUETransform(ChCoordsysd(Vehicle->GetPos(), Vehicle->GetRot()));

	const ChFrame<> ChassisInv = ChassisFrame.GetInverse();
	const std::shared_ptr<ChWheel> Wheels[VehicleWheel::Num] =
	{
		Vehicle->GetWheel(0, VehicleSide::LEFT), Vehicle->GetWheel(0, VehicleSide::RIGHT),
		Vehicle->GetWheel(1, VehicleSide::LEFT), Vehicle->GetWheel(1, VehicleSide::RIGHT)
	};
	for (int32 i = 0; i < VehicleWheel::Num; ++i)
	{
		if (Wheels[i] && Wheels[i]->GetSpindle())
		{
			Snapshot->Wheels[i] = ToUE(ChassisInv * Wheels[i]->GetSpindle()->GetFrameRefToAbs());
		}
	}

	const ChVehicleData& Data = Vehicle->m_VehicleData;
	Snapshot->SpeedMS = Data.Speed;
	Snapshot->EngineSpeed = Data.RPM;
	Snapshot->EngineTorque = Data.Torque;
	Snapshot->EngineLoad = Data.Load;
	Snapshot->BrakeTorque = Data.BrakeInput;
	Snapshot->CurrentGear = Data.CurrentGear;
	if (VehicleDriver)
	{
		Snapshot->Throttle = (float)VehicleDriver->GetThrottle();
		Snapshot->Steering = (float)VehicleDriver->GetSteering();
		Snapshot->Braking = (float)VehicleDriver->GetBraking();
	}

	FillTires(*Snapshot);

	return Snapshot;
}

void FChronoVehicleWorld::RequestShift(int Direction)
{
	if (VehicleDriver)
	{
		VehicleDriver->RequestShift(Direction);
	}
}

void FChronoVehicleWorld::ToggleAxleDiffLock(bool FrontAxle, bool Locked)
{
	if (Vehicle)
	{
		Vehicle->ToggleAxleDiffLock(FrontAxle, Locked);
	}
}

void FChronoVehicleWorld::ToggleCenterDiffLock(bool Locked)
{
	if (Vehicle)
	{
		Vehicle->ToggleCenterDiffLock(Locked);
	}
}

void FChronoVehicleWorld::SetStreamStaticColliders(bool bStream, int BudgetPerStep)
{
	Setup.bStreamStaticColliders = bStream;
	Setup.StreamingBudgetPerStep = BudgetPerStep;
}
