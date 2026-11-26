#pragma once

#include "GameFramework/WheeledVehicle.h"
#include "MonsterTruckPawn.generated.h"

// Team5 truck presentation on the project's cm-based PhysX vehicle manager.
UCLASS()
class AMonsterTruckPawn : public AWheeledVehicle
{
	GENERATED_BODY()
public:
	AMonsterTruckPawn();
	void BeginPlay() override;
	void Tick(float DeltaTime) override;
private:
	void UpdateTruckVisuals();
	class UStaticMeshComponent* TruckWheels[4]{};
	class UStaticMeshComponent* Axles[2]{};
};
