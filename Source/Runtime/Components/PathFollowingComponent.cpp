#include "EnginePCH.h"
#include "Components/PathFollowingComponent.h"

#include <cmath>

bool UPathFollowingComponent::SetPath(const TArray<FVector>& WorldPath, float InAcceptanceRadius)
{
	if (WorldPath.IsEmpty() || !std::isfinite(InAcceptanceRadius) || InAcceptanceRadius <= 0.0f)
		return false;

	PathPoints = WorldPath;
	CurrentPointIndex = 0;
	AcceptanceRadiusSquared = InAcceptanceRadius * InAcceptanceRadius;
	ProgressReferenceDistance = -1.0f;
	TimeWithoutProgress = 0.0f;
	Status = ENavigationPathFollowStatus::Following;
	return true;
}

void UPathFollowingComponent::Reset()
{
	PathPoints.Reset();
	CurrentPointIndex = 0;
	ProgressReferenceDistance = -1.0f;
	TimeWithoutProgress = 0.0f;
	Status = ENavigationPathFollowStatus::Idle;
}

FVector UPathFollowingComponent::Update(const FVector& CurrentLocation, float DeltaTime)
{
	if (!IsFollowing())
		return FVector::ZeroVector;

	constexpr float MinProgressDistance = 5.0f; // cm
	constexpr float BlockedTimeLimit = 1.25f; // 초
	while (PathPoints.IsValidIndex(CurrentPointIndex))
	{
		FVector ToPoint = PathPoints[CurrentPointIndex] - CurrentLocation;
		// 경로점은 바닥에 있고 액터 원점은 캡슐 중심이므로 XY 거리로 비교한다.
		ToPoint.Z = 0.0f;
		const float DistanceSquared = ToPoint.SizeSquared();
		if (DistanceSquared > AcceptanceRadiusSquared)
		{
			const float Distance = std::sqrt(DistanceSquared);
			// 제자리에서 흔들려도, 이전 기준보다 목표에 가까워져야 진행으로 인정한다.
			if (ProgressReferenceDistance < 0.0f || ProgressReferenceDistance - Distance >= MinProgressDistance)
			{
				ProgressReferenceDistance = Distance;
				TimeWithoutProgress = 0.0f;
			}
			else if (DeltaTime > 0.0f)
			{
				TimeWithoutProgress += DeltaTime;
			}

			if (TimeWithoutProgress >= BlockedTimeLimit)
			{
				Reset();
				Status = ENavigationPathFollowStatus::Blocked;
				return FVector::ZeroVector;
			}
			return ToPoint / Distance;
		}

		// 시작점이나 중복 점이 허용 반경 안에 있으면 같은 프레임에 건너뛴다.
		++CurrentPointIndex;
		ProgressReferenceDistance = -1.0f;
	}

	Reset();
	Status = ENavigationPathFollowStatus::Reached;
	return FVector::ZeroVector;
}
