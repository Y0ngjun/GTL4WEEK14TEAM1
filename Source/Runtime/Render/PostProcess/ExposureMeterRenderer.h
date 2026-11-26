#pragma once

#include "Core/Types.h"
#include "Render/PipelineState.h"

#include <d3d11.h>
#include <wrl/client.h>
#include <unordered_map>

class FRenderer;
class FTexture2D;
struct FPostProcessSettings;

// 카메라 노출계. HDR 장면의 중앙 중점 평균 휘도를 GPU에서 구하고 몇 프레임 뒤에 CPU로 읽어 온다.
// 뷰(뷰포트)마다 보는 장면이 다르므로 측정 타깃·읽기 대기열·적응 상태를 뷰 키마다 따로 둔다.
// 키는 뷰가 계속 쓰는 렌더 타깃 묶음(FSceneViewTargets)의 주소다. UE도 눈 적응 상태를 뷰 상태(FSceneViewState)에 둔다.
class FExposureMeterRenderer
{
public:
	void Init(FRenderer* InRenderer);

	// 측정 패스의 렌더 타깃. 처음 보는 뷰면 만든다. FSceneRenderer가 그래프에 import해서 바인딩한다
	FTexture2D* GetTarget(const void* ViewKey);

	// 현재 바인딩된 그 뷰의 Target에 측정값을 쓰고, 평균을 만들어 읽기 대기열에 넣는다
	bool Render(const void* ViewKey, FTexture2D* SceneHDR);

	// 눈 적응: 측정값이 가리키는 EV100 쪽으로 그 뷰의 EV100을 시간에 따라 조금씩 옮기고 돌려준다.
	// DisplayGamma는 톤맵이 거는 감마(보정을 껐으면 1). 같은 시각에 두 번 불러도 한 번만 움직인다
	float UpdateAdaptedEV100(const void* ViewKey, const FPostProcessSettings& Settings, float DisplayGamma);

	bool GetReading(const void* ViewKey, float& Luminance, float& EV) const;

private:
	static constexpr uint32 TargetSize = 256;
	// GPU가 앞서 달리므로 3프레임 전에 복사한 것을 읽으면 기다리지 않는다
	static constexpr uint32 NumStaging = 3;
	// 이 시간(초) 동안 쓰이지 않은 뷰 상태는 버린다 (닫힌 에셋 에디터·썸네일 뷰 등)
	static constexpr float StaleSeconds = 5.0f;

	struct FViewState
	{
		TUniquePtr<FTexture2D> Target;
		Microsoft::WRL::ComPtr<ID3D11Texture2D> MipChain;
		Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> MipChainSRV;
		Microsoft::WRL::ComPtr<ID3D11Texture2D> Staging[NumStaging];
		bool bStagingPending[NumStaging]{};
		uint32 WriteIndex = 0;

		float AverageLuminance = 0.18f;
		bool bHasReading = false;
		float AdaptedEV100 = 0.0f;
		bool bHasAdapted = false;
		float LastAdaptTime = -1.0f;
		float LastUsedTime = 0.0f;
	};

	FViewState* FindOrCreateState(const void* ViewKey);
	bool CreateResources(FViewState& State) const;
	void ReadBack(FViewState& State);
	void EvictStaleStates(float Now);

	FRenderer* Renderer = nullptr;
	FShaderProgram* Shader = nullptr;
	FPipelineState PipelineState;
	uint32 LastMip = 0;
	std::unordered_map<const void*, FViewState> ViewStates;
};
