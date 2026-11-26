#include "EnginePCH.h"
#include "Render/PostProcess/FXAARenderer.h"
#include "Render/RenderCommand.h"
#include "Render/RenderResourceManager.h"
#include "Render/Texture2D.h"

static_assert(sizeof(FFXAAData) == 16);

// Shader와 상수 버퍼를 준비한다.
void FFXAARenderer::Init(FRenderer* InRenderer)
{
	Renderer = InRenderer;
	Shader = FRenderResourceManager::GetShaderProgram("EngineResources/Shaders/FxaaShader.hlsl");

	// depth buffer를 사용하지 않고, 별도의 shader를 통해 렌더링을 처리한다.
	FXAAPipelineState.Shader = Shader;
	FXAAPipelineState.Topology = EPrimitiveTopology::TriangleList;
	FXAAPipelineState.RasterizerState = ERasterizerState::SolidNone;
	FXAAPipelineState.BlendState = EBlendState::Opaque;
	FXAAPipelineState.DepthStencilState = EDepthStencilState::Disabled;

	ConstantBuffer = RenderCommand::CreateConstantBuffer(sizeof(FFXAAData));

	assert(Shader && Shader->VertexShader && Shader->PixelShader &&
	       ConstantBuffer && ConstantBuffer->GetBuffer());
}

bool FFXAARenderer::OnRender(FTexture2D* ColorTarget)
{
	if (!ColorTarget || !ConstantBuffer)
		return false;
	const uint32 Width = ColorTarget->GetWidth();
	const uint32 Height = ColorTarget->GetHeight();
	FFXAAData Data{};
	Data.InvWidth = 1.0f / Width;
	Data.InvHeight = 1.0f / Height;

	RenderCommand::BindPipelineState(FXAAPipelineState);
	RenderCommand::BindVertexBuffer(nullptr);
	RenderCommand::BindSamplerState(0, ESamplerState::LinearClamp, EShaderBindFlagBits::Pixel);
	RenderCommand::UpdateBufferData(ConstantBuffer.get(), &Data, sizeof(Data));
	RenderCommand::BindConstantBuffer(0, ConstantBuffer.get(), EShaderBindFlagBits::Pixel);
	RenderCommand::BindShaderResource(0, ColorTarget, EShaderBindFlagBits::Pixel);
	RenderCommand::Draw(3);
	RenderCommand::BindShaderResource(0, static_cast<FTexture2D*>(nullptr), EShaderBindFlagBits::Pixel);
	return true;
}
