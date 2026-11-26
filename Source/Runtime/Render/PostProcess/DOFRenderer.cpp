#include "EnginePCH.h"
#include "Render/PostProcess/DOFRenderer.h"
#include "Math/EngineMath.h"
#include "Render/RenderCommand.h"
#include "Render/Texture2D.h"
#include "Render/RenderResourceManager.h"
#include "Render/PipelineState.h"
#include "Render/Buffer.h"
#include "Render/SceneView.h"

FTexture2D* FDOFRenderer::GetForegroundOutput() const
{
	return ForegroundOutput.get();
}

FTexture2D* FDOFRenderer::GetBackgroundOutput() const
{
	return BackgroundOutput.get();
}

void FDOFRenderer::Init()
{
	ForegroundPipelineState = FRenderResourceManager::GetPSO(EPSOType::DOF_Foreground);
	BackgroundPipelineState = FRenderResourceManager::GetPSO(EPSOType::DOF_Background);
	FinalizePipelineState = FRenderResourceManager::GetPSO(EPSOType::DOF_Finalize);

	Constants = RenderCommand::CreateConstantBuffer(sizeof(FDOFConstants));
}

bool FDOFRenderer::Prepare(const FSceneView& InView, uint32 Width, uint32 Height)
{
	float SensorWidth = InView.FinalPostProcessSettings.DepthOfFieldSensorWidth;
	float Fstop = InView.FinalPostProcessSettings.DepthOfFieldFstop;
	float FocalDistance = InView.FinalPostProcessSettings.DepthOfFieldFocalDistance;
	float MinFstop = InView.FinalPostProcessSettings.DepthOfFieldMinFstop;
	int32 BladeCount = InView.FinalPostProcessSettings.DepthOfFieldBladeCount;

	if (SensorWidth <= 0.f || Fstop <= 0.f)
	{
		return false;
	}

	const float MMToUU = 0.1f;

	// ī�޶� ������ ���̸� ���Ѵ�.
	const float SensorHeight = SensorWidth / InView.AspectRatio * MMToUU;

	// ���� ������ �Ÿ��� ���Ѵ�.
	const float f = (SensorHeight * 0.5f) / tan(FMath::DegreesToRadians(InView.FieldOfView * 0.5f));
	if (f >= FocalDistance)
	{
		return false;
	}

	// �� ���Ÿ� �帲 ������ ���Ѵ�.
	const float CoC = (f * f) / (Fstop * (FocalDistance - f));

	// CoC�� �ȼ� ��ǥ�� �ٲ۴�.
	const float CoCPixel = (CoC * 0.5f) / SensorHeight * Height;

	FDOFConstants ConstantsData{};
	ConstantsData.A = InView.ProjectionMatrix[0][2];
	ConstantsData.B = InView.ProjectionMatrix[3][2];
	ConstantsData.bIsPerspective = InView.bIsPerspective;
	ConstantsData.FocalDistance = FocalDistance;
	ConstantsData.SceneColorInvWidth = 1.f / Width;
	ConstantsData.SceneColorInvHeight = 1.f / Height;
	const float EffectWeight = std::clamp(InView.FinalPostProcessSettings.DepthOfFieldEffectWeight, 0.0f, 1.0f);
	ConstantsData.CoC = CoCPixel * EffectWeight;
	ConstantsData.MaxForegroundRadiusPixels = 3.f * EffectWeight;
	ConstantsData.MaxBackgroundRadiusPixels = FMath::Max(0.f, InView.FinalPostProcessSettings.DepthOfFieldMaxBackgroundRadiusPixels) * EffectWeight;
	ConstantsData.BladeCount = BladeCount <= 0 ? 0 : FMath::Clamp(BladeCount, 4, 16);
	ConstantsData.BokehRoundness = InView.FinalPostProcessSettings.BokehRoundness();
	ConstantsData.MaxDepthBlurRadiusPixels = FMath::Max(0.f, InView.FinalPostProcessSettings.DepthOfFieldDepthBlurRadius) * InView.FinalPostProcessSettings.GetDOFDepthBlurRadiusRatio(Width) * EffectWeight;
	ConstantsData.FocusRange = std::max(0.0f, InView.FinalPostProcessSettings.DepthOfFieldFocusRange);
	ConstantsData.InvDepthBlurHalfDistance = 1.f / (FMath::Max(0.000001f, InView.FinalPostProcessSettings.DepthOfFieldDepthBlurAmount) * 100000.f);

	RenderCommand::UpdateBufferData<FDOFConstants>(Constants.get(), &ConstantsData);

	constexpr ETextureFormat OutputFormat = ETextureFormat::R16G16B16A16_Float;
	const uint32 OutputWidth = std::max(1u, Width / 2);
	const uint32 OutputHeight = std::max(1u, Height / 2);

	if (!ForegroundOutput || ForegroundOutput->GetWidth() != OutputWidth || ForegroundOutput->GetHeight() != OutputHeight || ForegroundOutput->GetTextureFormat() != OutputFormat)
	{
		FTextureDesc Desc{};
		Desc.Width = OutputWidth;
		Desc.Height = OutputHeight;
		Desc.Format = OutputFormat;
		Desc.BindFlags = ETextureBindFlags::RenderTarget | ETextureBindFlags::ShaderResource;

		ForegroundOutput = RenderCommand::CreateTexture2D(Desc);
	}

	if (!BackgroundOutput || BackgroundOutput->GetWidth() != OutputWidth || BackgroundOutput->GetHeight() != OutputHeight || BackgroundOutput->GetTextureFormat() != OutputFormat)
	{
		FTextureDesc Desc{};
		Desc.Width = OutputWidth;
		Desc.Height = OutputHeight;
		Desc.Format = OutputFormat;
		Desc.BindFlags = ETextureBindFlags::RenderTarget | ETextureBindFlags::ShaderResource;

		BackgroundOutput = RenderCommand::CreateTexture2D(Desc);
	}

	return ForegroundOutput && BackgroundOutput;
}

bool FDOFRenderer::OnRenderForeground(FTexture2D& SceneColor, FTexture2D& SceneDepth)
{
	ID3D11ShaderResourceView* SRVs[] = {
		SceneColor.GetSRV(),
		SceneDepth.GetSRV()
	};

	RenderCommand::BindPipelineState(*ForegroundPipelineState);
	RenderCommand::BindConstantBuffer(0, Constants.get(), EShaderBindFlagBits::Pixel);
	RenderCommand::BindShaderResources(0, SRVs, 2, EShaderBindFlagBits::Pixel);
	RenderCommand::BindSamplerState(0, ESamplerState::LinearClamp, EShaderBindFlagBits::Pixel);
	RenderCommand::BindSamplerState(1, ESamplerState::PointClamp, EShaderBindFlagBits::Pixel);
	RenderCommand::BindVertexBuffer(nullptr);
	RenderCommand::Draw(3);

	RenderCommand::UnbindShaderResources(0, 2, EShaderBindFlagBits::Pixel);

	return true;
}

bool FDOFRenderer::OnRenderBackground(FTexture2D& SceneColor, FTexture2D& SceneDepth)
{
	ID3D11ShaderResourceView* SRVs[] = {
	    SceneColor.GetSRV(),
	    SceneDepth.GetSRV()
	};

	RenderCommand::BindPipelineState(*BackgroundPipelineState);
	RenderCommand::BindConstantBuffer(0, Constants.get(), EShaderBindFlagBits::Pixel);
	RenderCommand::BindShaderResources(0, SRVs, 2, EShaderBindFlagBits::Pixel);
	RenderCommand::BindSamplerState(0, ESamplerState::LinearClamp, EShaderBindFlagBits::Pixel);
	RenderCommand::BindSamplerState(1, ESamplerState::PointClamp, EShaderBindFlagBits::Pixel);
	RenderCommand::BindVertexBuffer(nullptr);
	RenderCommand::Draw(3);

	RenderCommand::UnbindShaderResources(0, 2, EShaderBindFlagBits::Pixel);

	return true;
}

bool FDOFRenderer::OnRenderFinalize(FTexture2D& SceneColor, FTexture2D& SceneDepth)
{
	ID3D11ShaderResourceView* SRVs[] = {
		SceneColor.GetSRV(),
		SceneDepth.GetSRV(),
	    ForegroundOutput->GetSRV(),
	    BackgroundOutput->GetSRV(),
	};

	RenderCommand::BindPipelineState(*FinalizePipelineState);
	RenderCommand::BindConstantBuffer(0, Constants.get(), EShaderBindFlagBits::Pixel);
	RenderCommand::BindShaderResources(0, SRVs, 4, EShaderBindFlagBits::Pixel);
	RenderCommand::BindSamplerState(0, ESamplerState::LinearClamp, EShaderBindFlagBits::Pixel);
	RenderCommand::BindSamplerState(1, ESamplerState::PointClamp, EShaderBindFlagBits::Pixel);
	RenderCommand::BindVertexBuffer(nullptr);
	RenderCommand::Draw(3);

	RenderCommand::UnbindShaderResources(0, 4, EShaderBindFlagBits::Pixel);

	return true;
}


bool FDOFRenderer::OnRenderVisualize(const FSceneView& View, FTexture2D& Depth)
{
 if (!Prepare(View, Depth.GetWidth(), Depth.GetHeight())) return false;
 auto* Shader = FRenderResourceManager::GetShaderProgram("EngineResources/Shaders/DOFVisualize.hlsl");
 if (!Shader) return false;
 FPipelineState Pipeline{};
 Pipeline.Shader = Shader; Pipeline.Topology = EPrimitiveTopology::TriangleList;
 Pipeline.RasterizerState = ERasterizerState::SolidNone;
 Pipeline.BlendState = EBlendState::Opaque; Pipeline.DepthStencilState = EDepthStencilState::Disabled;
 RenderCommand::BindPipelineState(Pipeline);
 RenderCommand::BindConstantBuffer(0, Constants.get(), EShaderBindFlagBits::Pixel);
 RenderCommand::BindShaderResource(0, &Depth, EShaderBindFlagBits::Pixel);
 RenderCommand::BindSamplerState(0, ESamplerState::PointClamp, EShaderBindFlagBits::Pixel);
 RenderCommand::BindVertexBuffer(nullptr); RenderCommand::Draw(3);
 RenderCommand::UnbindShaderResources(0, 1, EShaderBindFlagBits::Pixel);
 return true;
}
