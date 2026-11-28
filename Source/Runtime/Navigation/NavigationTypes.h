#pragma once

#include "Containers/Array.h"
#include "Math/Vector.h"

struct FNavAgentProperties
{
	// 월드 단위는 cm이며 경사는 각도(도)다.
	float Radius = 50.0f;
	float Height = 180.0f;
	float MaxStepHeight = 30.0f;
	float MaxSlope = 45.0f;
};

struct FNavigationBuildResult
{
	int MeshComponentCount = 0;
	int WalkableTriangleCount = 0;
	int TriangleCount = 0;
	int GridSizeX = 0;
	int GridSizeZ = 0;
	int WalkableSpanCount = 0;
	int SpanCount = 0;
	int FilteredWalkableSpanCount = 0;
	int ErodedWalkableSpanCount = 0;
	int ContourCount = 0;
	int PolygonCount = 0;
	int PolygonMeshVertexCount = 0;
	const char* FailedAt = nullptr;
};

enum class ENavigationQueryResult : uint8
{
	Success,
	NoNavigationData,
	NoPath,
	Error,
};

struct FNavigationPathResult
{
	ENavigationQueryResult Status = ENavigationQueryResult::NoNavigationData;
	TArray<FVector> PathPoints; // Kurisu 월드 좌표. 완전한 경로일 때만 채운다.
	int CorridorPolygonCount = 0;
	const char* FailedAt = nullptr;
};
