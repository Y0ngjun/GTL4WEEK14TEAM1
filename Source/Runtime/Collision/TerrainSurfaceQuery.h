#pragma once

#include "Math/Vector.h"
#include "Math/Vector2.h"
#include "Containers/Array.h"

//**********************************************************
// 언리얼의 Chaos/HeightField에 대응되는 객체 데이터가 아직 존재하지 않음
// 이로 인해, 쿼리 함수를 임시 파일에 추가해두고, 추후 구현시 해당 파일쪽으로 병합하는 방향으로 계획
// 구현 확인 후, 이 파일 삭제하고 해당 파일로 옮겨서 책임 병합.
//**********************************************************

namespace TerrainSurface
{
	// 월드 XY의 표면 조회. WorldOrigin.Z는 샘플값 0에 대응하는 높이
	bool TryGetSurfaceAtWorldXY(const TArray<uint16>& Samples, int32 NumSamplesX, int32 NumSamplesY,
	                            const FVector& WorldOrigin, const FVector2& CellSize, float HeightScale,
	                            const FVector2& WorldXY, float& OutWorldZ, FVector& OutWorldNormal);
}