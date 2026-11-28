#pragma once

#include "Components/ActorComponent.h"
#include "PathFollowingComponent.generated.h"

enum class ENavigationPathFollowStatus : uint8
{
	Idle,
	Following,
	Reached,
	Blocked,
};

// 한 층의 보행 경로를 따라갈 수평 입력 방향을 계산한다.
UCLASS()
class UPathFollowingComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	bool SetPath(const TArray<FVector>& WorldPath, float InAcceptanceRadius);
	void Reset();
	bool IsFollowing() const { return Status == ENavigationPathFollowStatus::Following; }
	ENavigationPathFollowStatus GetStatus() const { return Status; }

	FVector Update(const FVector& CurrentLocation, float DeltaTime);

private:
	TArray<FVector> PathPoints;
	int32 CurrentPointIndex = 0;
	float AcceptanceRadiusSquared = 0.0f;
	float ProgressReferenceDistance = -1.0f;
	float TimeWithoutProgress = 0.0f;
	ENavigationPathFollowStatus Status = ENavigationPathFollowStatus::Idle;
};
