#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "UnrealChronoGameMode.generated.h"

// No default pawn: ACarPhysicsWorld possesses the placed APhysicsVehicle when it starts the simulation.
UCLASS()
class UNREALCHRONO_API AUnrealChronoGameMode : public AGameModeBase
{
	GENERATED_BODY()

public:
	AUnrealChronoGameMode();
};
