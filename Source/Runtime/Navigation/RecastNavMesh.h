#pragma once

#include "Navigation/NavigationTypes.h"

class dtNavMesh;
struct FNavigationDebugData;

// Recast/Detour 의존성을 런타임 내비게이션 시스템 뒤에 숨긴다.
class FRecastNavMesh
{
public:
	FRecastNavMesh();
	~FRecastNavMesh();

	// 입력 삼각형은 이미 Recast 좌표계(Y-up)로 변환되어 있어야 한다.
	FNavigationBuildResult Build(const float* Vertices, int VertexCount, const int* Triangles, int TriangleCount,
		const FNavAgentProperties& Agent, bool bBuildHeightDetail);
	FNavigationPathResult FindPath(const FVector& WorldStart, const FVector& WorldGoal,
		FNavigationDebugData* DebugData = nullptr) const;
	bool GatherDebugGeometry(FNavigationDebugData& DebugData) const;
	bool CopyFrom(const FRecastNavMesh& Source);
	void Reset();
	bool HasData() const { return NavMesh != nullptr; }

private:
	bool LoadTile(const unsigned char* Data, int DataSize);

	TArray<uint8> TileData;
	TUniquePtr<dtNavMesh> NavMesh;
};
