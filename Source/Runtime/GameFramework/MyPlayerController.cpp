#include "EnginePCH.h"
#include "GameFramework/MyPlayerController.h"
#include "Camera/CameraModifier_Photo.h"
#include "Camera/PlayerCameraManager.h"
#include "UI/ViewfinderWidget.h"
#include "GameFramework/Actor/MyCharacter.h"
#include "GameFramework/Actor/KurisuCharacter.h"
#include "GameFramework/WheeledVehicle.h"
#include "Components/WheeledVehicleMovementComponent.h"

namespace
{
	constexpr float PhotoZoomSpeedMm = 40.0f; // Z/X를 누르고 있을 때 초당 렌즈 길이 변화
	constexpr float WheelNotch = 120.0f;      // WM_MOUSEWHEEL 한 칸
}

void AMyPlayerController::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (!IsPhotoModeActive()) return;
	const float Zoom = static_cast<float>(IsKeyDown(static_cast<int32>(EKeyCode::X))) -
		static_cast<float>(IsKeyDown(static_cast<int32>(EKeyCode::Z)));
	if (Zoom != 0.0f)
		GetPhotoModifier(false)->AdjustZoom(Zoom * PhotoZoomSpeedMm * DeltaTime);
}

void AMyPlayerController::ProcessPlayerInput(float DeltaTime)
{
	UCameraModifier_Photo* Photo = GetPhotoModifier(false);
	if (!Photo || !Photo->IsPhotoModeActive())
	{
		Super::ProcessPlayerInput(DeltaTime);
		return;
	}

	const auto Axis = [this](EKeyCode Positive, EKeyCode Negative)
	{
		return static_cast<float>(IsKeyDown(static_cast<int32>(Positive))) -
			static_cast<float>(IsKeyDown(static_cast<int32>(Negative)));
	};
	// 캐릭터에는 이동 입력을 주지 않으므로 그 자리에 서 있고, ControlRotation도 그대로라 나오면 원래 시점으로 돌아간다
	const FVector Move(Axis(EKeyCode::W, EKeyCode::S), Axis(EKeyCode::D, EKeyCode::A), Axis(EKeyCode::E, EKeyCode::Q));
	Photo->AddFreeCameraInput(Move, MouseAxisX * LookSensitivity, MouseAxisY * LookSensitivity,
		IsKeyDown(static_cast<int32>(EKeyCode::Shift)));
	MouseAxisX = MouseAxisY = 0.0f;
}

void AMyPlayerController::InputAxis(EGameInputAxis Axis, float Delta)
{
	Super::InputAxis(Axis, Delta);

	if (Axis == EGameInputAxis::MouseWheel && Delta != 0.0f && IsPhotoModeActive())
		GetPhotoModifier(false)->AdjustFocus(Delta / WheelNotch);
}

UCameraModifier_Photo* AMyPlayerController::GetPhotoModifier(bool bCreate)
{
	APlayerCameraManager* PCM = GetCameraManager();
	if (!PCM) return nullptr;

	auto* Modifier = static_cast<UCameraModifier_Photo*>(
		PCM->FindCameraModifierByClass(UCameraModifier_Photo::StaticClass()));
	if (!Modifier && bCreate)
	{
		Modifier = NewObject<UCameraModifier_Photo>(PCM);
		PCM->AddCameraModifier(Modifier);

		// 뷰파인더는 모디파이어 상태를 매 프레임 직접 읽으므로 여기서 포인터를 들고 있지 않는다.
		// PIE가 끝나면 뷰포트가 위젯을 지운다
		UViewfinderWidget* Viewfinder = NewObject<UViewfinderWidget>();
		Viewfinder->SetOwningWorld(GetWorld());
		Viewfinder->AddToViewport(100);
	}
	return Modifier;
}

bool AMyPlayerController::IsPhotoModeActive()
{
	UCameraModifier_Photo* Photo = GetPhotoModifier(false);
	return Photo && Photo->IsPhotoModeActive();
}

bool AMyPlayerController::HandlePhotoKey(int32 Key)
{
	if (Key == static_cast<int32>(EKeyCode::C))
	{
		if (UCameraModifier_Photo* Photo = GetPhotoModifier(true))
			Photo->TogglePhotoMode();
		return true;
	}

	if (!IsPhotoModeActive()) return false;
	UCameraModifier_Photo* Photo = GetPhotoModifier(false);

	// 위/아래: 조리개를 열고 조인다. 오른쪽/왼쪽: 셔터를 빠르게/느리게
	if (Key == static_cast<int32>(EKeyCode::Up))    { Photo->StepAperture(-1); return true; }
	if (Key == static_cast<int32>(EKeyCode::Down))  { Photo->StepAperture(+1); return true; }
	if (Key == static_cast<int32>(EKeyCode::Right)) { Photo->StepShutter(+1);  return true; }
	if (Key == static_cast<int32>(EKeyCode::Left))  { Photo->StepShutter(-1);  return true; }
	// F: AF를 다시 켜거나 끈다 (휠을 돌리면 MF로 넘어간다)
	if (Key == static_cast<int32>(EKeyCode::F))     { Photo->ToggleAutoFocus(); return true; }
	// G: 자동 노출(AE)과 수동 노출을 바꾼다
	if (Key == static_cast<int32>(EKeyCode::G))     { Photo->ToggleAutoExposure(); return true; }
	// H: 35mm 한 컷(3:2)과 하프 프레임(3:4 세로)을 바꾼다
	if (Key == static_cast<int32>(EKeyCode::H))     { Photo->ToggleHalfFrame(); return true; }

	if (Key == static_cast<int32>(EKeyCode::LButton))
	{
		Photo->TriggerShutter();
		return true;
	}
	// 카메라를 들고 있는 동안 우클릭 공격, 점프, E 슬라이드·R 구르기는 막는다 (Q/E는 카메라 위아래로 쓴다)
	if (Key == static_cast<int32>(EKeyCode::RButton) || Key == static_cast<int32>(EKeyCode::Space) ||
		Key == static_cast<int32>(EKeyCode::E) || Key == static_cast<int32>(EKeyCode::R))
		return true;
	return false;
}

void AMyPlayerController::InputKey(int32 Key, bool bDown)
{
	// Super가 bKeyDown을 갱신하므로, 눌림 순간 판정은 호출 전에 이전 상태를 잡아 둔다
	const bool bJustPressed = bDown && !IsKeyDown(Key);
	const bool bJustReleased = !bDown && IsKeyDown(Key);
	Super::InputKey(Key, bDown);

	// 차량: Space 핸드브레이크 (누르는 동안). WASD는 AddMovementInput으로 이동 컴포넌트가 받는다.
	if (Key == static_cast<int32>(EKeyCode::Space) && (bJustPressed || bJustReleased) &&
		(!IsPhotoModeActive() || bJustReleased))
	{
		if (AWheeledVehicle* Vehicle = Cast<AWheeledVehicle>(GetPawn()))
		{
			if (UWheeledVehicleMovementComponent* VehicleMovement = Vehicle->GetVehicleMovementComponent())
				VehicleMovement->SetHandbrakeInput(bDown);
		}
	}


	if (!bJustPressed)
		return;

	if (HandlePhotoKey(Key)) return;

	if (Key == static_cast<int32>(EKeyCode::LButton))
	{
		if (auto* Kurisu = Cast<AKurisuCharacter>(GetPawn())) { Kurisu->Jab(); }
		else if (AMyCharacter* MyCharacter = Cast<AMyCharacter>(GetPawn()))
		{
			MyCharacter->Attack();
		}
	}

	if (auto* Kurisu = Cast<AKurisuCharacter>(GetPawn())) {
		if (Key == static_cast<int32>(EKeyCode::RButton)) Kurisu->Punch();
		if (Key == static_cast<int32>(EKeyCode::R)) Kurisu->Roll();
		if (Key == static_cast<int32>(EKeyCode::E)) Kurisu->Slide();
	}

	if (Key == static_cast<int32>(EKeyCode::Space))
	{
		// UE 템플릿처럼 모든 ACharacter가 점프한다. Jump는 virtual이라 AMyCharacter의 override(애님 IsJumping)도 그대로 탄다.
		if (ACharacter* Character = Cast<ACharacter>(GetPawn()))
		{
			Character->Jump();
		}
	}

	// 차량: R 차 되돌리기 (뒤집혔을 때)
	if (Key == static_cast<int32>(EKeyCode::R))
	{
		if (AWheeledVehicle* Vehicle = Cast<AWheeledVehicle>(GetPawn()))
		{
			if (UWheeledVehicleMovementComponent* VehicleMovement = Vehicle->GetVehicleMovementComponent())
				VehicleMovement->ResetVehicle();
		}
	}
}
