#include "EnginePCH.h"
#include "LandscapeDataAccess.h"
#include "LandscapeComponent.h"

#include <cassert>
#include <cmath>
#include <limits>

namespace
{
	bool IsFiniteVector(const FVector& Value)
	{
		return std::isfinite(Value.X) && std::isfinite(Value.Y) && std::isfinite(Value.Z);
	}

	bool IsSupportedTransform(const FMatrix& Matrix)
	{
		for (int32 Row = 0; Row < 4; ++Row)
		{
			for (int32 Column = 0; Column < 4; ++Column)
			{
				if (!std::isfinite(Matrix.M[Row][Column]))
				{
					return false;
				}
					
			}
		}

		// 위치 변환용 아핀 행렬인지 검사한다.
		constexpr float Tolerance = 0.000001f;

		// 행렬의 마지막 열이 대략 (0, 0, 0, 1)인지 검사
		if (std::abs(Matrix.M[0][3]) > Tolerance || std::abs(Matrix.M[1][3]) > Tolerance || std::abs(Matrix.M[2][3]) > Tolerance || std::abs(Matrix.M[3][3] - 1.0f) > Tolerance)
		{
			return false;
		}

		// 공간의 방향을 유지하는 변환만 허용
		const float Determinant = Matrix.Determinant();

		return std::isfinite(Determinant) && Determinant > 0.0f;
	}
}

bool FLandscapeHeightfieldData::IsValid() const
{
	// 최소크기가 2*2인 이유: 삼각형 두 개로 이루어진 셀 하나를 만들려면 정점 4개가 필요하디
	if (SizeX < 2 || SizeY < 2)
	{
		return false;
	}
		

	const int64 ExpectedCount = int64(SizeX) * int64(SizeY);
	const int64 MaxCount = (std::numeric_limits<int32>::max)();

	return ExpectedCount <= MaxCount && ExpectedCount == int64(Heights.Num());
}

int32 FLandscapeHeightfieldData::GetSampleIndex(int32 X, int32 Y) const
{
	assert(X >= 0 && X < SizeX);
	assert(Y >= 0 && Y < SizeY);

	const int64 Index = int64(Y) * SizeX + X;

	assert(Index >= 0 && Index < int64(Heights.Num()));

	return int32(Index);
}

uint16 FLandscapeHeightfieldData::GetRawHeight(int32 X, int32 Y) const
{
	return Heights[GetSampleIndex(X, Y)];
}

namespace
{
	// uint16 높이 데이터에서 로컬 높이 0을 나타내는 값
	constexpr int32 LandscapeHeightZeroPoint = 32768;

	// 로컬 높이 1에 해당하는 저장값의 변화량
	constexpr float LandscapeHeightUnitsPerLocalUnit = 128.0f;
}

namespace LandscapeDataAccess
{
	float GetLocalHeight(uint16 Height)
	{
		return float(int32(Height) - LandscapeHeightZeroPoint) / LandscapeHeightUnitsPerLocalUnit;
	}

	// 격자 좌표와 높이를 묶어 정점 하나를 만든다.
	FVector GetLocalVertex(const FLandscapeHeightfieldData& Data, int32 X, int32 Y)
	{
		return FVector(float(X), float(Y), GetLocalHeight(Data.GetRawHeight(X, Y)));
	}

	bool TryGetSurfaceAtWorldXY(const FLandscapeHeightfieldData& Data, const FMatrix& LandscapeToWorld,
	                           const FVector2& WorldXY, float& OutWorldZ, FVector& OutWorldNormal)
	{
		if (!Data.IsValid() || !IsSupportedTransform(LandscapeToWorld))
		{
			return false;
		}

		// 월드 축에 정렬된 격자의 간격과 높이 배율
		const auto& Matrix = LandscapeToWorld.M;
		const float CellSizeX = Matrix[0][0];
		const float CellSizeY = Matrix[1][1];
		const float HeightScale = Matrix[2][2];
		if (CellSizeX <= 0.0f || CellSizeY <= 0.0f || HeightScale <= 0.0f ||
		    Matrix[0][1] != 0.0f || Matrix[0][2] != 0.0f || Matrix[1][0] != 0.0f ||
		    Matrix[1][2] != 0.0f || Matrix[2][0] != 0.0f || Matrix[2][1] != 0.0f)
		{
			return false;
		}

		const FVector WorldOrigin(Matrix[3][0], Matrix[3][1], Matrix[3][2]);
		const float LastX = float(Data.SizeX - 1);
		const float LastY = float(Data.SizeY - 1);
		const float MaxWorldX = WorldOrigin.X + LastX * CellSizeX;
		const float MaxWorldY = WorldOrigin.Y + LastY * CellSizeY;
		if (!std::isfinite(MaxWorldX) || !std::isfinite(MaxWorldY) ||
		    !(WorldXY.X >= WorldOrigin.X && WorldXY.X <= MaxWorldX &&
		      WorldXY.Y >= WorldOrigin.Y && WorldXY.Y <= MaxWorldY))
		{
			return false;
		}

		// 마지막 샘플 경계의 마지막 셀 선택
		const float GridX = WorldXY.X == MaxWorldX ? LastX : FMath::Min((WorldXY.X - WorldOrigin.X) / CellSizeX, LastX);
		const float GridY = WorldXY.Y == MaxWorldY ? LastY : FMath::Min((WorldXY.Y - WorldOrigin.Y) / CellSizeY, LastY);
		const int32 CellX = FMath::Min(FMath::FloorToInt(GridX), Data.SizeX - 2);
		const int32 CellY = FMath::Min(FMath::FloorToInt(GridY), Data.SizeY - 2);
		const float U = GridX - float(CellX);
		const float V = GridY - float(CellY);

		const float H00 = GetLocalHeight(Data.GetRawHeight(CellX, CellY));
		const float H11 = GetLocalHeight(Data.GetRawHeight(CellX + 1, CellY + 1));
		float DeltaX;
		float DeltaY;

		// BuildGridIndices와 같은 00–11 대각선의 삼각형 선택
		if (U >= V)
		{
			const float H10 = GetLocalHeight(Data.GetRawHeight(CellX + 1, CellY));
			DeltaX = H10 - H00;
			DeltaY = H11 - H10;
		}
		else
		{
			const float H01 = GetLocalHeight(Data.GetRawHeight(CellX, CellY + 1));
			DeltaX = H11 - H01;
			DeltaY = H01 - H00;
		}

		// 같은 삼각형 평면의 월드 높이와 단위 노멀
		OutWorldZ = WorldOrigin.Z + (H00 + U * DeltaX + V * DeltaY) * HeightScale;
		OutWorldNormal = FVector(-DeltaX * HeightScale / CellSizeX,
		                         -DeltaY * HeightScale / CellSizeY, 1.0f).GetSafeNormal();
		return true;
	}

	FVector2 GetLandscapeUV(const FLandscapeHeightfieldData& Data, int32 X, int32 Y)
	{
		assert(Data.SizeX >= 2 && Data.SizeY >= 2);
		assert(X >= 0 && X < Data.SizeX);
		assert(Y >= 0 && Y < Data.SizeY);

		// Size - 1 인 이유: 마지막 정점의 UV를 정확히 1로 하기 위해 
		return FVector2(float(X) / float(Data.SizeX - 1), float(Y) / float(Data.SizeY - 1));
	}

	// 정점 주변 높이로 지형의 기울기를 구하고, 그 기울기로 노멀과 탄젠트를 계산
	// 반환값은 로컬 공간 기준
	void GetLocalTangentVectors(const FLandscapeHeightfieldData& Data, int32 X, int32 Y, FVector& OutTangentX,  FVector& OutTangentY,  FVector& OutNormal)
	{
		assert(Data.SizeX >= 2 && Data.SizeY >= 2);
		assert(X >= 0 && X < Data.SizeX);
		assert(Y >= 0 && Y < Data.SizeY);

		// 지형 끝에서는 자기 자신 사용
		const int32 Left = X > 0 ? X - 1 : X;
		const int32 Right = X + 1 < Data.SizeX ? X + 1 : X;
		const int32 Down = Y > 0 ? Y - 1 : Y;
		const int32 Up = Y + 1 < Data.SizeY ? Y + 1 : Y;

		const float HeightLeft = GetLocalHeight(Data.GetRawHeight(Left, Y));

		const float HeightRight = GetLocalHeight(Data.GetRawHeight(Right, Y));

		const float HeightDown = GetLocalHeight(Data.GetRawHeight(X, Down));

		const float HeightUp = GetLocalHeight(Data.GetRawHeight(X, Up));

		const float Dx = (HeightRight - HeightLeft) / float(Right - Left);

		const float Dy = (HeightUp - HeightDown) / float(Up - Down);

		OutNormal = FVector(-Dx, -Dy, 1.0f).GetSafeNormal();

		OutTangentX = FVector(1.0f, 0.0f, Dx).GetSafeNormal();

		OutTangentY = FVector::Cross(OutNormal, OutTangentX).GetSafeNormal();
	}

	// NumQuadsX, NumQuadsY는 셀 수
	bool BuildGridIndices(int32 NumQuadsX, int32 NumQuadsY, TArray<uint32>& OutIndices)
	{
		OutIndices.Reset();

		if (NumQuadsX < 1 || NumQuadsY < 1)
			return false;

		const int64 NumVerticesX = int64(NumQuadsX) + 1;
		const int64 NumVerticesY = int64(NumQuadsY) + 1;

		const int64 VertexCount = NumVerticesX * NumVerticesY;
		const int64 MaxCount = (std::numeric_limits<int32>::max)();

		if (VertexCount > MaxCount)
		{
			return false;
		}
			

		// VertexCount 검사를 통과했으므로 이 곱은 int64 범위 안이다.
		const int64 IndexCount = int64(NumQuadsX) * int64(NumQuadsY) * 6;

		if (IndexCount > MaxCount)
		{
			return false;
		}
			
		OutIndices.Reserve(uint32(IndexCount));

		const uint32 Stride = uint32(NumVerticesX);

		for (int32 Y = 0; Y < NumQuadsY; ++Y)
		{
			for (int32 X = 0; X < NumQuadsX; ++X)
			{
				const uint32 A = uint32(Y) * Stride + uint32(X);
				const uint32 B = A + 1;
				const uint32 C = A + Stride;
				const uint32 D = C + 1;

				OutIndices.Add(A);
				OutIndices.Add(B);
				OutIndices.Add(D);

				OutIndices.Add(A);
				OutIndices.Add(D);
				OutIndices.Add(C);
			}
		}

		return true;
	}

	// Navigation에서 활용
	bool ExportNavigationGeometry(const FLandscapeHeightfieldData& Data, const FMatrix& LandscapeToWorld, FLandscapeNavigationGeometry& OutGeometry)
	{
		OutGeometry.Reset();

		if (!Data.IsValid() || !IsSupportedTransform(LandscapeToWorld))
		{
			return false;
		}

		if (!BuildGridIndices(Data.SizeX - 1, Data.SizeY - 1, OutGeometry.Indices))
		{
			return false;
		}

		OutGeometry.WorldVertices.Reserve(uint32(Data.Heights.Num()));

		for (int32 Y = 0; Y < Data.SizeY; ++Y)
		{
			for (int32 X = 0; X < Data.SizeX; ++X)
			{
				// 전체 지형의 LOD 0 삼각형 데이터를 반환
				const FVector LocalPosition = GetLocalVertex(Data, X, Y);

				const FVector WorldPosition = LandscapeToWorld.TransformPosition(LocalPosition);

				if (!IsFiniteVector(WorldPosition))
				{
					OutGeometry.Reset();
					return false;
				}

				OutGeometry.WorldVertices.Add(WorldPosition);
			}
		}

		return true;
	}
}

FLandscapeComponentDataInterface::FLandscapeComponentDataInterface(const ULandscapeComponent& InComponent) : Component(InComponent)
{
}

uint16 FLandscapeComponentDataInterface::GetHeight(int32 LocalX, int32 LocalY) const
{
	return Component.GetHeightfieldData().GetRawHeight(LocalX, LocalY);
}

float FLandscapeComponentDataInterface::GetLocalHeight(int32 LocalX, int32 LocalY) const
{
	return LandscapeDataAccess::GetLocalHeight(GetHeight(LocalX, LocalY));
}

FVector FLandscapeComponentDataInterface::GetLocalVertex(int32 LocalX, int32 LocalY) const
{

	return FVector(float(LocalX), float(LocalY), GetLocalHeight(LocalX, LocalY));
}

FVector FLandscapeComponentDataInterface::GetWorldVertex(int32 LocalX, int32 LocalY) const
{
	return Component.GetWorldMatrix().TransformPosition(GetLocalVertex(LocalX, LocalY));
}
