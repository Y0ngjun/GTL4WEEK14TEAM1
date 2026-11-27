#include "EnginePCH.h"
#include "Navigation/NavigationPathFollower.h"

#include <cmath>

bool FNavigationPathFollower::SetPath(const TArray<FVector>& WorldPath, float InAcceptanceRadius)
{
	if (WorldPath.IsEmpty() || !std::isfinite(InAcceptanceRadius) || InAcceptanceRadius <= 0.0f)
		return false;

	PathPoints = WorldPath;
	CurrentPointIndex = 0;
	AcceptanceRadiusSquared = InAcceptanceRadius * InAcceptanceRadius;
	return true;
}

void FNavigationPathFollower::Reset()
{
	PathPoints.Reset();
	CurrentPointIndex = 0;
}

FVector FNavigationPathFollower::Update(const FVector& CurrentLocation)
{
	while (IsFollowing())
	{
		FVector ToPoint = PathPoints[CurrentPointIndex] - CurrentLocation;
		// 경로점은 바닥에 있고 액터 원점은 캡슐 중심이므로 XY 거리로 비교한다.
		ToPoint.Z = 0.0f;
		if (ToPoint.SizeSquared() > AcceptanceRadiusSquared)
			return ToPoint.GetSafeNormal();

		// 시작점이나 중복 점이 허용 반경 안에 있으면 같은 프레임에 건너뛴다.
		++CurrentPointIndex;
	}

	Reset();
	return FVector::ZeroVector;
}
