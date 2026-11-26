#pragma once

#include "Render/RenderEnums.h"
#include "Render/ShowFlags.h"

// 뷰 하나의 씬 렌더링 옵션. 에디터와 게임 뷰포트가 같은 설정을 전달한다.
// 무엇을 보일지(표시 여부)는 ShowFlags가, 어떻게 그릴지(그림자·감마 등 품질)는 나머지 값이 담는다.
struct FSceneRenderOptions
{
	FEngineShowFlags ShowFlags{ EShowFlagInitMode::Game };

	EPcfMode PcfMode = EPcfMode::Sample8;
	float ShadowDepthBias = 0.0001f;
	bool bCacheShadows = true; // r.Shadow.CacheWholeSceneShadows: 바뀌지 않은 그림자 타일은 다시 그리지 않는다
	ELightCullingMode LightCullingMode = ELightCullingMode::Tiled;
	bool bDecalCulling = true;
	bool bDecalNarrowCulling = true;
	bool bDecalBVHCulling = true;
	bool bDecalOBBTree = true;
	float SceneDepthRange = 1.0f;
	uint32 DirectionalShadowMapSize = 0;
	uint32 PointShadowMapSize = 0;
	uint32 SpotShadowMapSize = 0;

	bool bGammaCorrection = true;
	float DisplayGamma = 2.2f;
	float ExposureCompensation = 0.0f; // 스톱 단위. +1이면 2배 밝게 (UE PostProcess의 Exposure Compensation)
	bool bUsePreviewSkybox = false;

	ELightingModel LightingModel = ELightingModel::Gouraud;
};
