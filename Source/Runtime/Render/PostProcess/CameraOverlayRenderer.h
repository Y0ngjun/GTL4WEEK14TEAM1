#pragma once
#include "Render/Renderer.h"
#include "Render/PipelineState.h"

struct FSceneView;

struct alignas(16) FCameraOverlayData
{
	FVector4 FadeColor;
	FVector2 ViewSize;
	float VignetteIntensity;
	float VignettePower;
	float LetterboxAmount;
	float LetterboxAspect;
	FVector2 TanHalfFov;      // 화면 가장자리까지의 tan(반화각). 0이면 렌즈 비네팅 끔
	float PupilRadius;        // 경통 반지름 1 기준 동공 반지름 = 최대 개방 N / 현재 N
	float LensVignetteAmount; // 경통 깊이 k
	float BarrelRadius;       // 최대 개방 동공 반지름 1 기준 경통 반지름 (>= 1)
	float Padding;
};
static_assert(sizeof(FCameraOverlayData) == 64);

class FCameraOverlayRenderer
{
public:
	void Init(FRenderer* InRenderer);
	bool OnRender(const FSceneView& View, uint32 Width, uint32 Height, bool bVignette, bool bOverlay);

private:
	FRenderer* Renderer = nullptr;
	FShaderProgram* Shader = nullptr;
	FPipelineState PipelineState;
	TUniquePtr<FConstantBuffer> ConstantBuffer;
};
