#include "PhysicsVehicle.h"
#include "TerrainDeformSubsystem.h"
#include "TerrainDeformPhysicsCache.h"
#include "Engine/CollisionProfile.h"
#include "Engine/Engine.h"
#include "HAL/IConsoleManager.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "InputCoreTypes.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "UObject/ConstructorHelpers.h"

// Test aid: > 0 drives with full throttle, logs the vehicle once per second and, after that many seconds, takes a screenshot and quits
static TAutoConsoleVariable<float> CVarAutoDrive(TEXT("UnrealChrono.AutoDrive"), 0.f,
	TEXT("Test aid. > 0: full throttle after 3 s, state logged once per second; after this many seconds a screenshot is taken and the game quits. 0 = off."));

// Test aid: > 0 puts the camera low and close behind the car (this many cm) so the tracks and berms are seen at a grazing angle
static TAutoConsoleVariable<float> CVarGroundCam(TEXT("UnrealChrono.GroundCam"), 0.f,
	TEXT("Test aid. > 0: follow camera this many cm behind the car, ~60 cm above the ground, to inspect tracks and berms. 0 = off."));

namespace
{
	// Hardpoints of the reduced double wishbone suspension, Unreal cm relative to the axle centre (left side, as for the
	// Range Rover Sport this car is derived from). Order of the Chrono ChDoubleWishboneReduced::PointId enum.
	struct FHardpoint { int32 Id; FVector Location; };

	void BuildSuspensionLocations(std::map<int, Vector>& OutMap)
	{
		// The PointId enumerators of chrono::vehicle::ChDoubleWishboneReduced are protected: mirror them in declaration order
		enum Point { SPINDLE = 0, UPRIGHT, UCA_F, UCA_B, UCA_U, LCA_F, LCA_B, LCA_U, SHOCK_C, SHOCK_U, TIEROD_C, TIEROD_U };
		const FHardpoint Points[] =
		{
			{ Point::SPINDLE,  FVector(3.5560, -90.9701, -2.6289) },
			{ Point::UPRIGHT,  FVector(3.5560, -80.7974, -2.6289) },
			{ Point::UCA_F,    FVector(35.0012, -46.2026, 22.5552) },
			{ Point::UCA_B,    FVector(7.7978, -46.2026, 22.5552) },
			{ Point::UCA_U,    FVector(3.5560, -71.5264, 21.5900) },
			{ Point::LCA_F,    FVector(22.3266, -30.7086, 0.0) },
			{ Point::LCA_B,    FVector(-22.3266, -30.7086, 0.0) },
			{ Point::LCA_U,    FVector(3.5560, -78.6384, -11.8110) },
			{ Point::SHOCK_C,  FVector(-10.3886, -71.6026, 32.3088) },
			{ Point::SHOCK_U,  FVector(-10.3886, -78.6384, -3.8354) },
			{ Point::TIEROD_C, FVector(22.3266, -41.6052, 5.8674) },
			{ Point::TIEROD_U, FVector(17.0282, -82.1106, -0.9271) },
		};
		for (const FHardpoint& P : Points)
		{
			OutMap[P.Id] = Conversions::ToChronoVec(P.Location);
		}
	}

	// Wheel placement of the visual meshes before the first snapshot arrives (cm, chassis frame)
	const FVector WheelRestLocations[VehicleWheel::Num] =
	{
		FVector(153.5, -91.0, 38.0), FVector(153.5, 91.0, 38.0), FVector(-135.0, -91.0, 40.7), FVector(-135.0, 91.0, 40.7)
	};
}

APhysicsVehicle::APhysicsVehicle()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bStartWithTickEnabled = false;

	static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeFinder(TEXT("/Engine/BasicShapes/Cube.Cube"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> CylinderFinder(TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
	static ConstructorHelpers::FObjectFinder<UMaterialInterface> MaterialFinder(TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));

	// The root is the Chrono chassis reference frame, driven every frame from the snapshot
	USceneComponent* Frame = CreateDefaultSubobject<USceneComponent>(TEXT("ChassisFrame"));
	RootComponent = Frame;

	const auto MakeBox = [&](const TCHAR* Name, USceneComponent* Parent, const FVector& Location, const FVector& SizeCm) -> UStaticMeshComponent*
	{
		UStaticMeshComponent* Box = CreateDefaultSubobject<UStaticMeshComponent>(Name);
		Box->SetupAttachment(Parent);
		Box->SetStaticMesh(CubeFinder.Object);
		if (MaterialFinder.Succeeded())
		{
			Box->SetMaterial(0, MaterialFinder.Object);
		}
		Box->SetRelativeLocation(Location);
		Box->SetRelativeScale3D(SizeCm / 100.0);
		Box->SetCollisionEnabled(ECollisionEnabled::NoCollision);   //the physics comes from Chrono
		return Box;
	};

	// Lower body slab x [-2.1, 2.4] m, y +-0.85 m, z [0.45, 1.05] m and a cabin on top
	VehicleBody = MakeBox(TEXT("VehicleBody"), Frame, FVector(15, 0, 75), FVector(450, 170, 60));
	MakeBox(TEXT("Cabin"), Frame, FVector(-20, 0, 130), FVector(220, 160, 50));

	const TCHAR* WheelNames[VehicleWheel::Num] = { TEXT("FrontLeftWheel"), TEXT("FrontRightWheel"), TEXT("RearLeftWheel"), TEXT("RearRightWheel") };
	const TCHAR* TireNames[VehicleWheel::Num] = { TEXT("FrontLeftTire"), TEXT("FrontRightTire"), TEXT("RearLeftTire"), TEXT("RearRightTire") };
	const TCHAR* LugNames[VehicleWheel::Num] = { TEXT("FrontLeftLug"), TEXT("FrontRightLug"), TEXT("RearLeftLug"), TEXT("RearRightLug") };
	TObjectPtr<USceneComponent>* WheelComps[VehicleWheel::Num] = { &FrontLeftWheel, &FrontRightWheel, &RearLeftWheel, &RearRightWheel };
	const FChWheelSetup WheelDefaults;
	const float TireRadiusCm = (float)(WheelDefaults.GetTireRadius() * CH2UE_SCALE);
	const float TireWidthCm = (float)(WheelDefaults.TireWidth * CH2UE_SCALE);
	for (int32 i = 0; i < VehicleWheel::Num; ++i)
	{
		// Wheel component = the spindle frame (the snapshot drives it, the axle is its local Y)
		USceneComponent* Wheel = CreateDefaultSubobject<USceneComponent>(WheelNames[i]);
		Wheel->SetupAttachment(Frame);
		Wheel->SetRelativeLocation(WheelRestLocations[i]);
		*WheelComps[i] = Wheel;

		// The engine cylinder stands along Z (100 cm high, 100 cm across): lay it along the axle (Y)
		UStaticMeshComponent* Tire = CreateDefaultSubobject<UStaticMeshComponent>(TireNames[i]);
		Tire->SetupAttachment(Wheel);
		Tire->SetStaticMesh(CylinderFinder.Object);
		if (MaterialFinder.Succeeded())
		{
			Tire->SetMaterial(0, MaterialFinder.Object);
		}
		Tire->SetRelativeRotation(FRotator(0.f, 0.f, 90.f));
		Tire->SetRelativeScale3D(FVector(TireRadiusCm * 2.f / 100.f, TireRadiusCm * 2.f / 100.f, TireWidthCm / 100.f));
		Tire->SetCollisionEnabled(ECollisionEnabled::NoCollision);

		// A marker across the tread so the wheel rotation is visible
		MakeBox(LugNames[i], Wheel, FVector(TireRadiusCm * 0.75f, 0, 0), FVector(TireRadiusCm * 0.5f, TireWidthCm + 4.f, TireRadiusCm * 0.15f));
	}

	CameraBoom = CreateDefaultSubobject<USpringArmComponent>(TEXT("CameraBoom"));
	CameraBoom->SetupAttachment(Frame);
	CameraBoom->SetRelativeLocation(FVector(0, 0, 150));
	CameraBoom->SetRelativeRotation(FRotator(-12.f, 0.f, 0.f));
	CameraBoom->TargetArmLength = 900.f;
	CameraBoom->bInheritPitch = false;
	CameraBoom->bInheritRoll = false;
	CameraBoom->bDoCollisionTest = false;
	CameraBoom->bEnableCameraRotationLag = true;
	CameraBoom->CameraRotationLagSpeed = 6.f;
	FollowCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("FollowCamera"));
	FollowCamera->SetupAttachment(CameraBoom, USpringArmComponent::SocketName);

	AutoPossessPlayer = EAutoReceiveInput::Disabled;
}

void APhysicsVehicle::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);
	// Input is polled from the keys directly (UpdateInputs), no bindings or input assets needed
}

USceneComponent* APhysicsVehicle::GetWheelComponent(int32 WheelIndex) const
{
	switch (WheelIndex)
	{
	case VehicleWheel::FL: return FrontLeftWheel;
	case VehicleWheel::FR: return FrontRightWheel;
	case VehicleWheel::RL: return RearLeftWheel;
	default: return RearRightWheel;
	}
}

bool APhysicsVehicle::IsPlayerControlled() const
{
	const APlayerController* PC = Cast<APlayerController>(GetController());
	return PC && PC->IsLocalController();
}

void APhysicsVehicle::Build()
{
	ChVehicleSetup = FChVehicleSetup();
	UE_LOG(LogTemp, Log, TEXT("APhysicsVehicle::Build: tire model %d"), (int32)TireModel);
	ChVehicleSetup.TireModel = TireModel;
	ChVehicleSetup.TireCollisionType = chrono::vehicle::ChTire::CollisionType::ENVELOPE;
	BuildSuspensionLocations(ChVehicleSetup.FrontSuspensionLocations);
	BuildSuspensionLocations(ChVehicleSetup.RearSuspensionLocations);

	// Rear axle: a bit softer than the front, the rest of the parameters are the FDoubleWishboneParams defaults
	ChVehicleSetup.Rear.SpringCoefficient = 150000.0;
	ChVehicleSetup.Rear.DampingCoefficient = 15000.0;
	ChVehicleSetup.Rear.AxleInertia = 0.33;
}

void APhysicsVehicle::UpdateInputs()
{
	Inputs.Throttle = Inputs.Brake = Inputs.Handbrake = Inputs.Steering = 0.f;

	const APlayerController* PC = Cast<APlayerController>(GetController());
	if (!PC)
	{
		return;
	}

	if (CVarAutoDrive.GetValueOnGameThread() > 0.f)
	{
		// Let the car settle at rest for 3 s, then full throttle
		Inputs.Throttle = AutoDriveTime >= 3.f ? 1.f : 0.f;
		return;
	}

	const auto Down = [PC](const FKey& A, const FKey& B) { return PC->IsInputKeyDown(A) || PC->IsInputKeyDown(B); };
	Inputs.Throttle = Down(EKeys::W, EKeys::Up) ? 1.f : 0.f;
	Inputs.Brake = Down(EKeys::S, EKeys::Down) ? 1.f : 0.f;
	Inputs.Handbrake = PC->IsInputKeyDown(EKeys::SpaceBar) ? 1.f : 0.f;
	Inputs.Steering = (Down(EKeys::D, EKeys::Right) ? 1.f : 0.f) - (Down(EKeys::A, EKeys::Left) ? 1.f : 0.f);

	// R toggles reverse: out of reverse into drive, otherwise down into reverse (only valid from first gear)
	const bool bReverseDown = PC->IsInputKeyDown(EKeys::R);
	if (bReverseDown && !bReverseKeyWasDown)
	{
		Inputs.Shift = CurrentGear == 0 ? 1 : -1;
	}
	bReverseKeyWasDown = bReverseDown;
}

void APhysicsVehicle::BeginPlay()
{
	Super::BeginPlay();

	// Colour the primitives (BasicShapeMaterial has a "Color" parameter)
	TInlineComponentArray<UStaticMeshComponent*> Meshes(this);
	for (UStaticMeshComponent* Mesh : Meshes)
	{
		const FString Name = Mesh->GetName();
		FLinearColor Color = FLinearColor(0.7f, 0.05f, 0.05f);
		if (Name.Contains(TEXT("Tire"))) Color = FLinearColor(0.02f, 0.02f, 0.02f);
		else if (Name.Contains(TEXT("Lug"))) Color = FLinearColor(0.9f, 0.9f, 0.9f);
		else if (Name.Contains(TEXT("Cabin"))) Color = FLinearColor(0.1f, 0.12f, 0.2f);
		if (UMaterialInstanceDynamic* MID = Mesh->CreateAndSetMaterialInstanceDynamic(0))
		{
			MID->SetVectorParameterValue(TEXT("Color"), Color);
		}
	}
}

void APhysicsVehicle::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	UpdateInputs();

	if (const float GroundCam = CVarGroundCam.GetValueOnGameThread(); GroundCam > 0.f && CameraBoom)
	{
		CameraBoom->TargetArmLength = GroundCam;
		CameraBoom->SetRelativeLocation(FVector(0, 0, 25));
		CameraBoom->SetRelativeRotation(FRotator(-4.f, 12.f, 0.f));
	}

	const float AutoDrive = CVarAutoDrive.GetValueOnGameThread();
	if (AutoDrive != 0.f && GetController())
	{
		AutoDriveTime += DeltaTime;
		if (AutoDriveTime - AutoDriveLogTime >= 1.f)
		{
			AutoDriveLogTime = AutoDriveTime;
			const FVector Loc = GetActorLocation();
			UE_LOG(LogTemp, Display, TEXT("AutoDrive thr=%.1f brk=%.1f str=%.1f t=%.1f speed=%.1f km/h gear=%d rpm=%.0f pos=(%.0f, %.0f, %.0f) valid=%d"),
				Inputs.Throttle, Inputs.Brake, Inputs.Steering, AutoDriveTime, GetSpeedKPH(), CurrentGear, Snapshot.EngineSpeed * 30.f / PI, Loc.X, Loc.Y, Loc.Z, Snapshot.bValid ? 1 : 0);
		}
		if (AutoDrive > 0.f && !bAutoDriveShot && AutoDriveTime >= AutoDrive - 2.f)
		{
			bAutoDriveShot = true;
			GEngine->Exec(GetWorld(), TEXT("HighResShot 1280x720"));
		}
		if (AutoDrive > 0.f && AutoDriveTime >= AutoDrive + 1.f)
		{
			GEngine->Exec(GetWorld(), TEXT("quit"));
		}
	}
}

void APhysicsVehicle::ApplySnapshot(const FVehicleSnapshot& NewSnapshot)
{
	Snapshot = NewSnapshot;
	if (!Snapshot.bValid)
	{
		return;
	}

	CurrentGear = Snapshot.CurrentGear;

	// Chassis frame, then the four spindle frames relative to it
	SetActorTransform(Snapshot.Body);
	for (int32 i = 0; i < VehicleWheel::Num; ++i)
	{
		GetWheelComponent(i)->SetRelativeLocationAndRotation(Snapshot.Wheels[i].GetLocation(), Snapshot.Wheels[i].GetRotation());
	}

	UpdateTireDeformation();
}

void APhysicsVehicle::UpdateTireDeformation()
{
	UTerrainDeformSubsystem* Deform = GetWorld() ? GetWorld()->GetSubsystem<UTerrainDeformSubsystem>() : nullptr;
	if (!Deform)
	{
		return;
	}

	const float RadiusCm = (float)(ChVehicleSetup.Wheels.GetTireRadius() * CH2UE_SCALE);
	const float TireWidthCm = (float)(ChVehicleSetup.Wheels.TireWidth * CH2UE_SCALE);

	for (int32 i = 0; i < VehicleWheel::Num; ++i)
	{
		const FTireSnapshot& TireSnap = Snapshot.Tires[i];

		// Only the discrete tire publishes the ground heightfield the stamp needs
		if (!TireSnap.bHasGround || !TireSnap.bInContact)
		{
			continue;
		}

		// Soft ground only
		switch (TireSnap.TerrainType)
		{
		case ETerrainType::Sand:
		case ETerrainType::Dirt:
		case ETerrainType::Mud:
		case ETerrainType::Snow:
		case ETerrainType::Grass:
		case ETerrainType::Gravel:
			break;
		default:
			continue;
		}

		// The ground the physics sensed already includes earlier deformation, but the stamp wants the depth below the
		// ORIGINAL surface, so add the cached deformation back in. Without this a second pass through a rut would read
		// as almost no sinkage.
		const FVector2D ImpactXY(TireSnap.TerrainImpactPoint.X, TireSnap.TerrainImpactPoint.Y);
		float DeformCm = 0.f;
		FVector2D DeformSlope;
		FTerrainDeformPhysicsCache::Sample(ImpactXY, DeformCm, DeformSlope);
		// Same policy the terrain query used (berms only by default), so subtracting it recovers the original.
		const float OriginalGroundZ = (float)TireSnap.TerrainImpactPoint.Z - FTerrainDeformPhysicsCache::ToPhysicsHeightCm(DeformCm);

		const float TyreBottomZ = (float)GetWheelComponent(i)->GetComponentLocation().Z - RadiusCm + (float)(TireSnap.Deflection * CH2UE_SCALE);
		const float SinkageCm = FMath::Clamp(OriginalGroundZ - TyreBottomZ, 0.f, TireDeformMaxSinkageCm);
		if (SinkageCm <= 0.1f)
		{
			continue;
		}

		// Contact patch half length from the tyre circle cut at the sinkage depth; half width = the real tread width.
		const float ChordHalf = FMath::Sqrt(FMath::Max(2.f * RadiusCm * SinkageCm - SinkageCm * SinkageCm, 0.f));
		FTerrainDeformStamp Stamp;
		Stamp.Center = FVector2f((float)ImpactXY.X, (float)ImpactXY.Y);
		Stamp.Forward = FVector2f((float)TireSnap.GroundAxisX.X, (float)TireSnap.GroundAxisX.Y);
		Stamp.HalfLength = FMath::Clamp(ChordHalf, 6.f, FMath::Max(TireSnap.GroundReachCm, 6.f));
		Stamp.HalfWidth = TireWidthCm > 1.f ? 0.5f * TireWidthCm : FMath::Max(TireSnap.GroundHalfWidthCm, 5.f);
		Stamp.Sinkage = SinkageCm;
		// One source per wheel: the subsystem sweeps the footprint along each wheel's travel. A stopped wheel adds nothing
		// unless it is spinning (digging in): re-stamping a rocking stationary car only heaps soil around it.
		const bool bSpinning = FMath::Abs(TireSnap.LongitudinalSlip) > TireDeformSpinSlip;
		Deform->AddTrackStamp((int32)(GetUniqueID() * 8u + (uint32)i), Stamp, bSpinning);
	}
}
