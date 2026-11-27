#pragma once

#include "Containers/Array.h"
#include "Math/Vector.h"

// 한 층의 보행 경로를 따라갈 수평 입력 방향을 계산한다.
class FNavigationPathFollower
{
public:
	bool SetPath(const TArray<FVector>& WorldPath, float InAcceptanceRadius);
	void Reset();
	bool IsFollowing() const { return PathPoints.IsValidIndex(CurrentPointIndex); }

	FVector Update(const FVector& CurrentLocation);

private:
	TArray<FVector> PathPoints;
	int32 CurrentPointIndex = 0;
	float AcceptanceRadiusSquared = 0.0f;
};
