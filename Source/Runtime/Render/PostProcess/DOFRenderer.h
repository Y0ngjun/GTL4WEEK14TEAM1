#pragma once

#include "Core/Types.h"

class FConstantBuffer;
class FTexture2D;
struct FPipelineState;
struct FSceneView;

class FDOFRenderer
{
public:
	void Init();
	bool OnRenderVisualize(const FSceneView& View, FTexture2D& Depth);

	bool Prepare(const FSceneView& InView, uint32 Width, uint32 Height);
	bool OnRenderForeground(FTexture2D& SceneColor, FTexture2D& SceneDepth);
	bool OnRenderBackground(FTexture2D& SceneColor, FTexture2D& SceneDepth);
	bool OnRenderFinalize(FTexture2D& SceneColor, FTexture2D& SceneDepth);

	FTexture2D* GetForegroundOutput() const;

	FTexture2D* GetBackgroundOutput() const;

private:
	struct FDOFConstants
	{
		float A;
		float B;
		uint32 bIsPerspective;
		float FocalDistance;

		float SceneColorInvWidth;
		float SceneColorInvHeight;
		float CoC;
		float MaxForegroundRadiusPixels;

		int32 BladeCount;
		float MaxBackgroundRadiusPixels;
		float BokehRoundness;
		float MaxDepthBlurRadiusPixels;

		float InvDepthBlurHalfDistance;
		float FocusRange;
		int32 Padding[2];
	};

	FPipelineState* ForegroundPipelineState;
	FPipelineState* BackgroundPipelineState;
	FPipelineState* FinalizePipelineState;

	TUniquePtr<FConstantBuffer> Constants;

	TUniquePtr<FTexture2D> ForegroundOutput;
	TUniquePtr<FTexture2D> BackgroundOutput;
};
