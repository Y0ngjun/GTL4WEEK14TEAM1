#include "EnginePCH.h"
#include "TerrainSurfaceQuery.h"
#include <cmath>

namespace TerrainSurface
{
	bool TryGetSurfaceAtWorldXY(const TArray<uint16>& Samples, int32 NumSamplesX, int32 NumSamplesY,
	                            const FVector& WorldOrigin, const FVector2& CellSize, float HeightScale,
	                            const FVector2& WorldXY, float& OutWorldZ, FVector& OutWorldNormal)
	{
		// 격자 입력 검증
		if (NumSamplesX < 2 || NumSamplesY < 2 || !(CellSize.X > 0.0f && CellSize.Y > 0.0f) ||
		    !std::isfinite(WorldOrigin.Z) || !std::isfinite(HeightScale))
		{
			return false;
		}

		if (Samples.Num() != static_cast<int64>(NumSamplesX) * NumSamplesY)
		{
			return false;
		}

		const float LastX = static_cast<float>(NumSamplesX - 1);
		const float LastY = static_cast<float>(NumSamplesY - 1);
		const float MaxWorldX = WorldOrigin.X + LastX * CellSize.X;
		const float MaxWorldY = WorldOrigin.Y + LastY * CellSize.Y;
		if (!std::isfinite(MaxWorldX) || !std::isfinite(MaxWorldY) ||
		    !(WorldXY.X >= WorldOrigin.X && WorldXY.X <= MaxWorldX &&
		      WorldXY.Y >= WorldOrigin.Y && WorldXY.Y <= MaxWorldY))
		{
			return false;
		}

		// 마지막 샘플 경계를 마지막 셀의 끝으로 처리
		const float GridX = WorldXY.X == MaxWorldX ? LastX : FMath::Min((WorldXY.X - WorldOrigin.X) / CellSize.X, LastX);
		const float GridY = WorldXY.Y == MaxWorldY ? LastY : FMath::Min((WorldXY.Y - WorldOrigin.Y) / CellSize.Y, LastY);
		const int32 CellX = FMath::Min(FMath::FloorToInt(GridX), NumSamplesX - 2);
		const int32 CellY = FMath::Min(FMath::FloorToInt(GridY), NumSamplesY - 2);
		const float U = GridX - static_cast<float>(CellX);
		const float V = GridY - static_cast<float>(CellY);

		// 행 우선 순서의 샘플을 실수로 읽기
		const auto ReadSample = [&](int32 X, int32 Y)
		{
			const uint32 Index = static_cast<uint32>(Y * NumSamplesX + X);
			return static_cast<float>(Samples[Index]);
		};

		const float H00 = ReadSample(CellX, CellY);
		const float H11 = ReadSample(CellX + 1, CellY + 1);
		float DeltaX;
		float DeltaY;

		// 00–11 대각선을 기준으로 삼각형 선택
		if (U >= V)
		{
			const float H10 = ReadSample(CellX + 1, CellY);
			DeltaX = H10 - H00;
			DeltaY = H11 - H10;
		}
		else
		{
			const float H01 = ReadSample(CellX, CellY + 1);
			DeltaX = H11 - H01;
			DeltaY = H01 - H00;
		}

		// 같은 삼각형 평면에서 월드 높이와 단위 노멀 계산
		OutWorldZ = WorldOrigin.Z + (H00 + U * DeltaX + V * DeltaY) * HeightScale;
		OutWorldNormal = FVector(-DeltaX * HeightScale / CellSize.X,
		                         -DeltaY * HeightScale / CellSize.Y, 1.0f)
		                     .GetSafeNormal();
		return true;
	}
}