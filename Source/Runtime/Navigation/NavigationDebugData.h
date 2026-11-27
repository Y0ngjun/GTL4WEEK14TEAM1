#pragma once

#include "Containers/Array.h"
#include "Math/Vector.h"

struct FNavigationDebugTriangle
{
	FVector A;
	FVector B;
	FVector C;
};

struct FNavigationDebugSegment
{
	FVector Start;
	FVector End;
};

// 좌표는 모두 Kurisu 월드 공간(cm, Z-up)이다.
struct FNavigationDebugData
{
	TArray<FNavigationDebugTriangle> SurfaceTriangles;
	TArray<FNavigationDebugSegment> BoundarySegments; // 걸어서 건널 수 있는 내부 경계는 제외한다.
	TArray<FVector> PathPoints; // 출발점에서 목적지 순서. 완전한 경로가 없으면 비어 있다.
	FVector RequestedGoal;
	FVector ProjectedGoal;
	bool bHasRequestedGoal = false;
	bool bHasProjectedGoal = false;
};
