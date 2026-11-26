#pragma once

#include "Core/Types.h"
#include "Containers/Array.h"
#include "Render/RenderCommand.h"
#include "Render/Texture2D.h"

// 뷰포트의 색상, 깊이 리소스를 관리한다.
struct FSceneViewTargets
{
	uint32 Width = 0, Height = 0;

	TUniquePtr<FTexture2D> SceneColors[2];
	int32 CurrentSceneColorIndex = 0;

	TUniquePtr<FTexture2D> Depth;

	void Reset()
	{
		Width = 0;
		Height = 0;

		for (int32 i = 0; i < 2; i++)
		{
			SceneColors[i].reset();
		}
		CurrentSceneColorIndex = 0;

		Depth.reset();
	}

	bool IsValidFor(uint32 InWidth, uint32 InHeight) const
	{
		return Width == InWidth && Height == InHeight;
	}

	bool Resize(uint32 InWidth, uint32 InHeight)
	{
		if (InWidth == 0 || InHeight == 0)
		{
			return false;
		}

		if (IsValidFor(InWidth, InHeight))
		{
			return true;
		}

		for (int32 i = 0; i < 2; i++)
		{
			FTextureDesc Desc{};
			Desc.Width = InWidth;
			Desc.Height = InHeight;
			Desc.BindFlags = ETextureBindFlags::RenderTarget | ETextureBindFlags::ShaderResource;
			Desc.Format = ETextureFormat::R8G8B8A8_UNorm;

			SceneColors[i] = RenderCommand::CreateTexture2D(Desc);
		}
		CurrentSceneColorIndex = 0;

		// DSV로 쓰고 SRV로 읽을 수 있도록 typeless 리소스로 만든다.
		FTextureDesc DepthDesc{};
		DepthDesc.Width = InWidth;
		DepthDesc.Height = InHeight;
		DepthDesc.Format = ETextureFormat::R24G8_Typeless;
		DepthDesc.BindFlags = ETextureBindFlags::DepthStencil | ETextureBindFlags::ShaderResource;
		Depth = RenderCommand::CreateTexture2D(DepthDesc);

		if (!SceneColors[0] || !SceneColors[1] || !Depth)
		{
			Reset();
			return false;
		}

		Width = InWidth;
		Height = InHeight;

		return true;
	}

	inline void SwapSceneColor()
	{
		CurrentSceneColorIndex = 1 - CurrentSceneColorIndex;
	}

	inline FTexture2D* GetFrontSceneColor() const
	{
		return SceneColors[CurrentSceneColorIndex].get();
	}

	inline FTexture2D* GetBackSceneColor() const
	{
		return SceneColors[1 - CurrentSceneColorIndex].get();
	}

	inline FTexture2D* GetDepth() const
	{
		return Depth.get();
	}
};
