#pragma once

#include "CoreMinimal.h"
#include <Common.h>
#include "Conversions.h"
#include "ChronoVehicle.h"
#include "../Tools/VehicleSnapshot.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/SpringArmComponent.h"
#include "Camera/CameraComponent.h"
#include "Components/StaticMeshComponent.h"
#include "PhysicsVehicle.generated.h"

class UMaterialInstanceDynamic;

// The Unreal side of the Chrono test car. A pawn made only of engine primitive shapes (cubes for the body, cylinders for
// the wheels) that is driven by the immutable snapshots ACarPhysicsWorld pumps from the physics thread. It also gathers
// the (code defined) Chrono vehicle parameters and feeds the terrain deformation system with one stamp per wheel.
//
// Controls: W throttle, S brake, A/D steer, Space handbrake, R toggle reverse / drive.
UCLASS()
class UNREALCHRONO_API APhysicsVehicle : public APawn
{
	GENERATED_BODY()

public:
	APhysicsVehicle();

	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;
	virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;

	// Fills ChVehicleSetup with the hardpoints, masses, wheel and engine data (called by ACarPhysicsWorld::Init on the game thread)
	void Build();

	// Applies the newest immutable state published by the physics thread (called by ACarPhysicsWorld every frame)
	void ApplySnapshot(const FVehicleSnapshot& NewSnapshot);
	const FVehicleSnapshot& GetSnapshot() const { return Snapshot; }

	// True while a local player controller drives this vehicle
	bool IsPlayerControlled() const;
	const FDriverInputs& GetInputs() const { return Inputs; }
	// Clears the one-shot inputs (gear shifts) after the physics world consumed them
	void ConsumeInput() { Inputs.Shift = 0; }

	UFUNCTION(BlueprintPure, Category = "VehicleData")
		float GetSpeedKPH() const { return Snapshot.SpeedMS * MetresPerSecondToKPH; }

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "PhysicsVehicle")
		TObjectPtr<UStaticMeshComponent> VehicleBody;
	// Spindle frames (driven by the snapshot); the tire cylinder and its marker are children
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "PhysicsVehicle")
		TObjectPtr<USceneComponent> FrontLeftWheel;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "PhysicsVehicle")
		TObjectPtr<USceneComponent> FrontRightWheel;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "PhysicsVehicle")
		TObjectPtr<USceneComponent> RearLeftWheel;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "PhysicsVehicle")
		TObjectPtr<USceneComponent> RearRightWheel;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Camera")
		TObjectPtr<USpringArmComponent> CameraBoom;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Camera")
		TObjectPtr<UCameraComponent> FollowCamera;

	// Tire model of the Chrono car. Discrete (terramechanics, the only one that deforms the terrain) is the default;
	// RIGID (cylinder) and TMEASY are also available. Any other value falls back to Discrete.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PhysicsVehicle")
		ETireModelType TireModel = ETireModelType::DISCRETE;

	// Deepest rut a tire stamps into the terrain deformation window [cm].
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire Deformation", meta = (ClampMin = "0.0", Units = "cm"))
		float TireDeformMaxSinkageCm = 25.f;
	// A wheel that is not travelling only deforms the ground while it is spinning in place (digging in): |longitudinal slip| above this.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tire Deformation", meta = (ClampMin = "0.0"))
		float TireDeformSpinSlip = 0.15f;

	FChVehicleSetup ChVehicleSetup;

private:
	void UpdateInputs();
	void UpdateTireDeformation();
	USceneComponent* GetWheelComponent(int32 WheelIndex) const;

	FDriverInputs Inputs;
	float AutoDriveTime = 0.f;
	float AutoDriveLogTime = 0.f;
	bool bAutoDriveShot = false;
	bool bReverseKeyWasDown = false;
	int32 CurrentGear = 1;   //0 == reverse, 1+ == forward gears

	//Latest vehicle state from the physics thread (a default constructed snapshot before the first step)
	FVehicleSnapshot Snapshot;
};
