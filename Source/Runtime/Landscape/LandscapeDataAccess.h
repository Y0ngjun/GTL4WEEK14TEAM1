#pragma once

#include "Core/Types.h"
#include "Containers/Array.h"
#include "Math/Vector.h"
#include "Math/Vector2.h"
#include "Math/Matrix.h"

class ULandscapeComponent;

// 높이 데이터 자체
struct FLandscapeHeightfieldData
{
	int32 SizeX = 0; // 정점
	int32 SizeY = 0; // 정점

	TArray<uint16> Heights;

	bool IsValid() const;

	// 2차원 좌표를 배열 인덱스로 변환
	int32 GetSampleIndex(int32 X, int32 Y) const;
	uint16 GetRawHeight(int32 X, int32 Y) const;
};

// Navigation에 전달할 데이터
// 지형 전체 LOD 0의 월드 좌표 
// WorldVertices: 엔진 월드 좌표, cm, Z-up
struct FLandscapeNavigationGeometry
{
	TArray<FVector> WorldVertices;
	TArray<uint32> Indices;

	void Reset()
	{
		WorldVertices.Reset();
		Indices.Reset();
	}
};

namespace LandscapeDataAccess
{
	// 언리얼과 같은 높이 디코딩 규칙.
	float GetLocalHeight(uint16 Height);

	// 전체 지형 기준 로컬 정점.
	// 반환: (X, Y, 디코딩한 높이)
	FVector GetLocalVertex(const FLandscapeHeightfieldData& Data, int32 X, int32 Y);

	// 양수 스케일로 월드 XY에 정렬된 격자의 높이와 면 노멀 조회
	// 전체 격자는 루트 행렬, 청크는 배치가 반영된 청크 월드 행렬
	bool TryGetSurfaceAtWorldXY(const FLandscapeHeightfieldData& Data, const FMatrix& LandscapeToWorld,
	                           const FVector2& WorldXY, float& OutWorldZ, FVector& OutWorldNormal);

	// 전체 지형 기준 0~1 UV.
	FVector2 GetLandscapeUV(const FLandscapeHeightfieldData& Data, int32 X, int32 Y);

	// 전체 원본 격자의 이웃 높이로 로컬 탄젠트 공간 계산.
	// 청크 경계에서도 같은 원본 이웃을 사용한다.
	void GetLocalTangentVectors(const FLandscapeHeightfieldData& Data, int32 X, int32 Y, FVector& OutTangentX, FVector& OutTangentY, FVector& OutNormal);

	// NumQuadsX/Y는 정점 수가 아니라 셀 수다.
	// 실패하면 OutIndices는 비어 있다.
	bool BuildGridIndices(int32 NumQuadsX, int32 NumQuadsY, TArray<uint32>& OutIndices);


	// 전체 원본 지형의 LOD 0을 월드 좌표로 내보낸다.
	// LandscapeToWorld는 지형 루트의 월드 행렬이다.
	// 청크의 월드 행렬을 넣으면 안 된다.
	// 실패하면 OutGeometry는 비어 있다.
	bool ExportNavigationGeometry(const FLandscapeHeightfieldData& Data, const FMatrix& LandscapeToWorld, FLandscapeNavigationGeometry& OutGeometry);
}

class FLandscapeComponentDataInterface
{
public:
	// 청크단위 조회
	explicit FLandscapeComponentDataInterface(const ULandscapeComponent& InComponent);

	uint16 GetHeight(int32 LocalX, int32 LocalY) const;
	float GetLocalHeight(int32 LocalX, int32 LocalY) const;

	FVector GetLocalVertex(int32 LocalX, int32 LocalY) const;
	FVector GetWorldVertex(int32 LocalX, int32 LocalY) const;

private:
	const ULandscapeComponent& Component;
};
