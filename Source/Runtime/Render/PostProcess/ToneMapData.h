#pragma once

struct alignas(16) FToneMapData
{
	float InvGamma = 1.0f / 2.2f;
	float bGammaCorretion = 1.0f;
	float Exposure = 1.0f; // 톤매핑 전에 곱하는 노출 배율
	float bFilmic = 0.0f;  // 1이면 ACES 필름 곡선, 0이면 기존처럼 잘라낸다 (노출 기능을 켠 카메라만 1)
};

static_assert(sizeof(FToneMapData) == 16);
