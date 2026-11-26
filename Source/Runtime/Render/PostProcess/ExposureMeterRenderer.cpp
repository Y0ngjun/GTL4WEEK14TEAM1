#include "EnginePCH.h"
#include "Render/PostProcess/ExposureMeterRenderer.h"
#include "Render/RenderCommand.h"
#include "Render/RenderDevice.h"
#include "Render/RenderResourceManager.h"
#include "Render/Texture2D.h"
#include "Render/PostProcess/PostProcessSettings.h"
#include "Core/EngineTimer.h"

#include <DirectXPackedVector.h>
#include <algorithm>
#include <cmath>

void FExposureMeterRenderer::Init(FRenderer* InRenderer)
{
	Renderer = InRenderer;
	Shader = FRenderResourceManager::GetShaderProgram("EngineResources/Shaders/ExposureMeter.hlsl");

	PipelineState.Shader = Shader;
	PipelineState.Topology = EPrimitiveTopology::TriangleList;
	PipelineState.RasterizerState = ERasterizerState::SolidNone;
	PipelineState.BlendState = EBlendState::Opaque;
	PipelineState.DepthStencilState = EDepthStencilState::Disabled;

	LastMip = static_cast<uint32>(std::log2(static_cast<float>(TargetSize)));
}

bool FExposureMeterRenderer::CreateResources(FViewState& State) const
{
	// R = 가중 log2 휘도, G = 가중치. 두 값을 같이 평균내야 가중 평균을 되찾을 수 있다
	FTextureDesc Desc{};
	Desc.Width = TargetSize;
	Desc.Height = TargetSize;
	Desc.Format = ETextureFormat::R16G16_Float;
	Desc.BindFlags = ETextureBindFlags::RenderTarget | ETextureBindFlags::ShaderResource;
	State.Target = RenderCommand::CreateTexture2D(Desc);

	// 엔진 텍스처는 mip 자동 생성 텍스처에 RTV를 만들지 않으므로, 평균용 mip 체인은 따로 두고 복사해 넣는다
	ID3D11Device* Device = FRenderDevice::Get()->GetDevice();
	D3D11_TEXTURE2D_DESC MipDesc{};
	MipDesc.Width = TargetSize;
	MipDesc.Height = TargetSize;
	MipDesc.MipLevels = 0;
	MipDesc.ArraySize = 1;
	MipDesc.Format = DXGI_FORMAT_R16G16_FLOAT;
	MipDesc.SampleDesc.Count = 1;
	MipDesc.Usage = D3D11_USAGE_DEFAULT;
	MipDesc.BindFlags = D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_RENDER_TARGET;
	MipDesc.MiscFlags = D3D11_RESOURCE_MISC_GENERATE_MIPS;
	Device->CreateTexture2D(&MipDesc, nullptr, State.MipChain.GetAddressOf());
	if (State.MipChain)
		Device->CreateShaderResourceView(State.MipChain.Get(), nullptr, State.MipChainSRV.GetAddressOf());

	D3D11_TEXTURE2D_DESC StagingDesc{};
	StagingDesc.Width = 1;
	StagingDesc.Height = 1;
	StagingDesc.MipLevels = 1;
	StagingDesc.ArraySize = 1;
	StagingDesc.Format = DXGI_FORMAT_R16G16_FLOAT;
	StagingDesc.SampleDesc.Count = 1;
	StagingDesc.Usage = D3D11_USAGE_STAGING;
	StagingDesc.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
	for (auto& Texture : State.Staging)
		Device->CreateTexture2D(&StagingDesc, nullptr, Texture.GetAddressOf());

	return State.Target && State.MipChain && State.MipChainSRV;
}

FExposureMeterRenderer::FViewState* FExposureMeterRenderer::FindOrCreateState(const void* ViewKey)
{
	if (!ViewKey || !Shader)
		return nullptr;

	const float Now = EngineTimer::GetTotalTime();
	EvictStaleStates(Now);

	auto It = ViewStates.find(ViewKey);
	if (It == ViewStates.end())
	{
		FViewState NewState;
		if (!CreateResources(NewState))
			return nullptr;
		It = ViewStates.emplace(ViewKey, std::move(NewState)).first;
	}
	It->second.LastUsedTime = Now;
	return &It->second;
}

void FExposureMeterRenderer::EvictStaleStates(float Now)
{
	for (auto It = ViewStates.begin(); It != ViewStates.end();)
	{
		// 시간이 되돌아간 경우(타이머 초기화)도 오래된 것으로 보지 않는다
		if (Now - It->second.LastUsedTime > StaleSeconds)
			It = ViewStates.erase(It);
		else
			++It;
	}
}

FTexture2D* FExposureMeterRenderer::GetTarget(const void* ViewKey)
{
	FViewState* State = FindOrCreateState(ViewKey);
	return State ? State->Target.get() : nullptr;
}

bool FExposureMeterRenderer::Render(const void* ViewKey, FTexture2D* SceneHDR)
{
	auto It = ViewStates.find(ViewKey);
	if (It == ViewStates.end() || !Shader || !SceneHDR)
		return false;
	FViewState& State = It->second;
	if (!State.Target || !State.MipChain || !State.Staging[State.WriteIndex])
		return false;

	RenderCommand::BindPipelineState(PipelineState);
	RenderCommand::BindVertexBuffer(nullptr);
	RenderCommand::BindSamplerState(0, ESamplerState::LinearClamp, EShaderBindFlagBits::Pixel);
	RenderCommand::BindShaderResource(0, SceneHDR, EShaderBindFlagBits::Pixel);
	RenderCommand::Draw(3);
	RenderCommand::UnbindShaderResources(0, 1, EShaderBindFlagBits::Pixel);

	ID3D11DeviceContext* Context = FRenderDevice::Get()->GetContext();
	Context->CopySubresourceRegion(State.MipChain.Get(), 0, 0, 0, 0, State.Target->GetRawPtr(), 0, nullptr);
	Context->GenerateMips(State.MipChainSRV.Get());
	Context->CopySubresourceRegion(State.Staging[State.WriteIndex].Get(), 0, 0, 0, 0, State.MipChain.Get(), LastMip, nullptr);
	State.bStagingPending[State.WriteIndex] = true;
	State.WriteIndex = (State.WriteIndex + 1) % NumStaging;

	ReadBack(State);
	return true;
}

void FExposureMeterRenderer::ReadBack(FViewState& State)
{
	// 다음에 쓸 칸이 가장 오래된 복사본이다. 아직 GPU가 안 끝냈으면 기다리지 않고 다음 프레임에 다시 본다
	ID3D11DeviceContext* Context = FRenderDevice::Get()->GetContext();
	const uint32 ReadIndex = State.WriteIndex;
	if (!State.bStagingPending[ReadIndex])
		return;

	D3D11_MAPPED_SUBRESOURCE Mapped{};
	if (FAILED(Context->Map(State.Staging[ReadIndex].Get(), 0, D3D11_MAP_READ, D3D11_MAP_FLAG_DO_NOT_WAIT, &Mapped)))
		return;

	const uint16* Half = static_cast<const uint16*>(Mapped.pData);
	const float WeightedLogL = DirectX::PackedVector::XMConvertHalfToFloat(Half[0]);
	const float Weight = DirectX::PackedVector::XMConvertHalfToFloat(Half[1]);
	Context->Unmap(State.Staging[ReadIndex].Get(), 0);
	State.bStagingPending[ReadIndex] = false;

	if (Weight > 0.0f)
	{
		// 로그 평균(기하 평균)이라 하늘 같은 아주 밝은 부분이 평균을 혼자 끌고 가지 않는다
		State.AverageLuminance = std::exp2(WeightedLogL / Weight);
		State.bHasReading = true;
	}
}

float FExposureMeterRenderer::UpdateAdaptedEV100(const void* ViewKey, const FPostProcessSettings& Settings, float DisplayGamma)
{
	const float MinEV = std::min(Settings.AutoExposureMinEV100, Settings.AutoExposureMaxEV100);
	const float MaxEV = std::max(Settings.AutoExposureMinEV100, Settings.AutoExposureMaxEV100);

	FViewState* State = FindOrCreateState(ViewKey);
	if (!State)
		return std::clamp(Settings.GetCameraEV100(), MinEV, MaxEV);

	// 첫 측정이 오기 전에는 카메라에 설정된 값을 목표로 둔다
	const float TargetEV = std::clamp(State->bHasReading ? FPostProcessSettings::GetMeteredEV100(State->AverageLuminance, DisplayGamma)
	                                                     : Settings.GetCameraEV100(),
	                                  MinEV, MaxEV);

	const float Now = EngineTimer::GetTotalTime();
	if (!State->bHasAdapted)
	{
		// 처음 켰을 때 검은 화면에서 서서히 밝아지지 않게 바로 목표에 맞춘다
		State->AdaptedEV100 = TargetEV;
		State->bHasAdapted = true;
	}
	else if (Now != State->LastAdaptTime)
	{
		// 탭 전환 등으로 시간이 크게 튀면 한 번에 끝까지 넘어가 버리므로 막는다
		const float Dt = std::clamp(Now - State->LastAdaptTime, 0.0f, 0.1f);
		// 목표 EV가 작다 = 장면이 어두워서 화면을 밝혀야 한다
		const float Speed = TargetEV < State->AdaptedEV100 ? Settings.AutoExposureSpeedUp : Settings.AutoExposureSpeedDown;
		// 남은 거리의 일정 비율씩 다가가서, 프레임레이트가 달라도 같은 시간에 같은 만큼 적응한다
		State->AdaptedEV100 += (TargetEV - State->AdaptedEV100) * (1.0f - std::exp(-Dt * Speed));
	}
	State->LastAdaptTime = Now;
	return State->AdaptedEV100;
}

bool FExposureMeterRenderer::GetReading(const void* ViewKey, float& Luminance, float& EV) const
{
	const auto It = ViewStates.find(ViewKey);
	if (It == ViewStates.end() || !It->second.bHasReading || !It->second.bHasAdapted) return false;
	Luminance = It->second.AverageLuminance;
	EV = It->second.AdaptedEV100;
	return true;
}
