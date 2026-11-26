#pragma once

#include "GameFramework/PlayerController.h"
#include "MyPlayerController.generated.h"

UCLASS()
class AMyPlayerController : public APlayerController
{
	GENERATED_BODY()

public:
	void Tick(float DeltaTime) override;
	void ProcessPlayerInput(float DeltaTime) override;
	void InputAxis(EGameInputAxis Axis, float Delta) override;
	class UCameraModifier_Photo* GetPhotoModifier(bool bCreate);
	bool IsPhotoModeActive();
	bool HandlePhotoKey(int32 Key);
	virtual void InputKey(int32 Key, bool bDown) override;
};
