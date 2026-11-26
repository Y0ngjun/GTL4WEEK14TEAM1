#include "EnginePCH.h"
#include "GameFramework/Actor/MonsterTruckPawn.h"
#include "Asset/AssetManager.h"
#include "Components/BoxComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Components/WheeledVehicleMovementComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "Camera/CameraComponent.h"

AMonsterTruckPawn::AMonsterTruckPawn()
{
	auto* Movement = GetVehicleMovementComponent();
	Movement->Mass = 3280.0f;
	Movement->COMOffset = FVector(-16.0f, 0.0f, -15.0f);
	const FVector Centers[] = {{179.8f,-141.1f,-75.4f},{179.8f,141.4f,-75.4f},
	                           {-151.1f,-141.1f,-75.4f},{-151.1f,141.4f,-75.4f}};
	for (int32 Index = 0; Index < 4; ++Index)
	{
		auto& Setup = Movement->WheelSetups[Index];
		Setup.AdditionalOffset = Centers[Index];
		Setup.Wheel.ShapeRadius = 82.7f;
		Setup.Wheel.ShapeWidth = 105.2f;
		Setup.Wheel.Mass = 120.0f;
		Setup.Wheel.SuspensionMaxRaise = 35.0f;
		Setup.Wheel.SuspensionMaxDrop = 45.0f;
		Setup.Wheel.FrictionScale = 1.2f;
		Setup.Wheel.MaxBrakeTorque = 6000.0f;
		Setup.Wheel.MaxHandBrakeTorque = 12000.0f;
	}
	Movement->EngineSetup.TorqueCurve.Keys.Empty();
	Movement->EngineSetup.TorqueCurve.AddKey(0.0f, 1800.0f);
	Movement->EngineSetup.TorqueCurve.AddKey(3000.0f, 2600.0f);
	Movement->EngineSetup.TorqueCurve.AddKey(4500.0f, 1800.0f);
	Movement->NotifySetupChanged();
	GetChassis()->BoxExtent = FVector(212.0f, 53.0f, 20.0f);
	// Hide the standard car's presentation; its wheel roots still drive physics.
	for (UActorComponent* Component : GetComponents())
	{
		if (auto* Mesh = Cast<UStaticMeshComponent>(Component))
			Mesh->SetVisibility(false);
		if (auto* Arm = Cast<USpringArmComponent>(Component))
		{
			Arm->TargetArmLength = 880.0f;
			Arm->TargetOffset = FVector(0.0f,0.0f,117.0f);
			Arm->bEnableCameraLag = true;
			Arm->CameraLagSpeed = 7.0f;
		}
		if (auto* Camera = Cast<UCameraComponent>(Component)) Camera->SetFieldOfView(66.0f);
	}
	auto Visual = [this](const char* Name, const char* Asset, const FVector& Location)
	{
		auto* Mesh = CreateDefaultSubobject<UStaticMeshComponent>(Name);
		Mesh->SetupAttachment(GetChassis());
		Mesh->SetCollisionProfileName(FName("NoCollision"));
		Mesh->SetStaticMesh(UAssetManager::GetAssetByPath<UStaticMesh>(Asset));
		Mesh->SetRelativeLocation(Location);
		Mesh->SetRelativeScale3D(FVector(100.0f,100.0f,100.0f));
		return Mesh;
	};
	Visual("CabAndFrame", "FBX/Truck/Cab.obj", FVector::ZeroVector);
	Visual("DumpBed", "FBX/Truck/DumpBed.obj", FVector(-110.0f,0.0f,30.0f));
	const char* Names[] = {"TruckWheelFL","TruckWheelFR","TruckWheelRL","TruckWheelRR"};
	for (int32 Index = 0; Index < 4; ++Index)
		TruckWheels[Index] = Visual(Names[Index], Index % 2 == 0 ? "FBX/Truck/WheelLeft.obj" : "FBX/Truck/WheelRight.obj", Centers[Index]);
	Axles[0] = Visual("FrontAxle", "FBX/Truck/AxleFront.obj", FVector(179.8f,0.0f,-75.4f));
	Axles[1] = Visual("RearAxle", "FBX/Truck/AxleRear.obj", FVector(-151.1f,0.0f,-75.4f));
}

void AMonsterTruckPawn::BeginPlay()
{
	Super::BeginPlay();
	UpdateTruckVisuals();
}

void AMonsterTruckPawn::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	UpdateTruckVisuals();
}

void AMonsterTruckPawn::UpdateTruckVisuals()
{
	const auto& States = GetVehicleMovementComponent()->GetWheelStates();
	for (int32 Index = 0; Index < States.Num() && Index < 4; ++Index)
		if (TruckWheels[Index])
			TruckWheels[Index]->SetRelativeLocationAndRotation(States[Index].LocalPose.Location, States[Index].LocalPose.Rotation.ToFRotator());
	for (int32 Axle = 0; Axle < 2; ++Axle)
		if (States.Num() >= Axle*2+2 && Axles[Axle])
			Axles[Axle]->SetRelativeLocation((States[Axle*2].LocalPose.Location + States[Axle*2+1].LocalPose.Location)*0.5f);
}
