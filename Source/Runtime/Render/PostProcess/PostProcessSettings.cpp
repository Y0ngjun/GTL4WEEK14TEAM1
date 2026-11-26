#include "EnginePCH.h"
#include "Render/PostProcess/PostProcessSettings.h"
#include "Math/EngineMath.h"

#include <algorithm>
#include <cmath>

bool FPostProcessSettings::HasDOFEffect() const
{
	return DepthOfFieldFocalDistance > 0.f;
}

void FPostProcessSettings::Blend(const FPostProcessSettings& Other, float Alpha)
{
	Alpha = std::clamp(Alpha, 0.0f, 1.0f);
	const FPostProcessSettings Defaults;
	// Blend resolved values even when only one camera overrides the property.
	// FinalPostProcessSettings must consume the blended value during the transition.
#define BLEND_CAMERA_FIELD(Name, Value) \
	do { \
		const auto From = bOverride_##Name ? Name : Defaults.Name; \
		const auto To = Other.bOverride_##Name ? Other.Name : Defaults.Name; \
		Name = (Value); \
		bOverride_##Name = Alpha <= 0.0f ? bOverride_##Name : \
		                   (Alpha >= 1.0f ? Other.bOverride_##Name : (bOverride_##Name || Other.bOverride_##Name)); \
	} while (false)
#define BLEND_CAMERA_FLOAT(Name) BLEND_CAMERA_FIELD(Name, FMath::Lerp(From, To, Alpha))
#define BLEND_CAMERA_STEP(Name) BLEND_CAMERA_FIELD(Name, Alpha < 0.5f ? From : To)
	BLEND_CAMERA_FLOAT(LensVignetteAmount);
	BLEND_CAMERA_FLOAT(LensMaxAperture);
	BLEND_CAMERA_FLOAT(LensBarrelRadius);
	BLEND_CAMERA_FLOAT(LensFlareIntensity);
	BLEND_CAMERA_FLOAT(LensFlareThreshold);
	BLEND_CAMERA_STEP(LensFlareGhostCount);
	BLEND_CAMERA_FLOAT(LensFlareGhostDispersal);
	BLEND_CAMERA_FLOAT(LensFlareHaloWidth);
	BLEND_CAMERA_FLOAT(LensFlareHaloIntensity);
	BLEND_CAMERA_FLOAT(LensFlareBokehSize);
	BLEND_CAMERA_FLOAT(LensFlareChromaticDistortion);
	BLEND_CAMERA_FLOAT(LensFlareDirtIntensity);
	BLEND_CAMERA_FLOAT(CameraShutterSpeed);
	BLEND_CAMERA_FLOAT(CameraISO);
	BLEND_CAMERA_STEP(bApplyPhysicalCameraExposure);
	BLEND_CAMERA_STEP(bAutoExposure);
	BLEND_CAMERA_FLOAT(AutoExposureMinEV100);
	BLEND_CAMERA_FLOAT(AutoExposureMaxEV100);
	BLEND_CAMERA_FLOAT(AutoExposureSpeedUp);
	BLEND_CAMERA_FLOAT(AutoExposureSpeedDown);
	BLEND_CAMERA_FLOAT(AutoExposureBias);
#undef BLEND_CAMERA_STEP
#undef BLEND_CAMERA_FLOAT
#undef BLEND_CAMERA_FIELD

	// A disabled DOF endpoint has no lens to interpolate. Keep the active lens
	// and fade its blur strength instead of switching the entire effect at 50%.
	const bool bFromDOF = HasDOFEffect();
	const bool bToDOF = Other.HasDOFEffect();
	FPostProcessSettings FromDOF = !bFromDOF && bToDOF ? Other : *this;
	FPostProcessSettings ToDOF = !bToDOF && bFromDOF ? *this : Other;
	if (!bFromDOF) FromDOF.DepthOfFieldEffectWeight = 0.0f;
	if (!bToDOF) ToDOF.DepthOfFieldEffectWeight = 0.0f;
#define BLEND_DOF_FLOAT(Name) Name = FMath::Lerp(FromDOF.Name, ToDOF.Name, Alpha)
	BLEND_DOF_FLOAT(DepthOfFieldFocalDistance);
	BLEND_DOF_FLOAT(DepthOfFieldFstop);
	BLEND_DOF_FLOAT(DepthOfFieldSensorWidth);
	BLEND_DOF_FLOAT(DepthOfFieldMaxBackgroundRadiusPixels);
	BLEND_DOF_FLOAT(DepthOfFieldDepthBlurRadius);
	BLEND_DOF_FLOAT(DepthOfFieldFocusRange);
	BLEND_DOF_FLOAT(DepthOfFieldEffectWeight);
	BLEND_DOF_FLOAT(DepthOfFieldMinFstop);
	BLEND_DOF_FLOAT(DepthOfFieldDepthBlurAmount);
#undef BLEND_DOF_FLOAT
	DepthOfFieldBladeCount = Alpha < 0.5f ? FromDOF.DepthOfFieldBladeCount : ToDOF.DepthOfFieldBladeCount;
	if ((Alpha <= 0.0f && !bFromDOF) || (Alpha >= 1.0f && !bToDOF))
		DepthOfFieldFocalDistance = 0.0f;
}

void FPostProcessSettings::OverrideBlend(const FPostProcessSettings& Src, float Weight)
{
	Weight = std::clamp(Weight, 0.0f, 1.0f);
	if (Weight <= 0.0f) return;

	auto BlendFloat = [Weight](bool bOverride, float& Dest, float Value)
	{
		if (bOverride) Dest += (Value - Dest) * Weight;
	};

	BlendFloat(Src.bOverride_LensVignetteAmount, LensVignetteAmount, Src.LensVignetteAmount);
	BlendFloat(Src.bOverride_LensMaxAperture, LensMaxAperture, Src.LensMaxAperture);
	BlendFloat(Src.bOverride_LensBarrelRadius, LensBarrelRadius, Src.LensBarrelRadius);
	BlendFloat(Src.bOverride_LensFlareIntensity, LensFlareIntensity, Src.LensFlareIntensity);
	BlendFloat(Src.bOverride_LensFlareThreshold, LensFlareThreshold, Src.LensFlareThreshold);
	if (Src.bOverride_LensFlareGhostCount && Weight >= 0.5f)
		LensFlareGhostCount = Src.LensFlareGhostCount;
	BlendFloat(Src.bOverride_LensFlareGhostDispersal, LensFlareGhostDispersal, Src.LensFlareGhostDispersal);
	BlendFloat(Src.bOverride_LensFlareHaloWidth, LensFlareHaloWidth, Src.LensFlareHaloWidth);
	BlendFloat(Src.bOverride_LensFlareHaloIntensity, LensFlareHaloIntensity, Src.LensFlareHaloIntensity);
	BlendFloat(Src.bOverride_LensFlareBokehSize, LensFlareBokehSize, Src.LensFlareBokehSize);
	BlendFloat(Src.bOverride_LensFlareChromaticDistortion, LensFlareChromaticDistortion, Src.LensFlareChromaticDistortion);
	BlendFloat(Src.bOverride_LensFlareDirtIntensity, LensFlareDirtIntensity, Src.LensFlareDirtIntensity);
	BlendFloat(Src.bOverride_CameraShutterSpeed, CameraShutterSpeed, Src.CameraShutterSpeed);
	BlendFloat(Src.bOverride_CameraISO, CameraISO, Src.CameraISO);
	// 켜고 끄는 값은 섞을 수 없으니 절반을 넘으면 넘어간다
	if (Src.bOverride_bApplyPhysicalCameraExposure && Weight >= 0.5f)
		bApplyPhysicalCameraExposure = Src.bApplyPhysicalCameraExposure;
	if (Src.bOverride_bAutoExposure && Weight >= 0.5f)
		bAutoExposure = Src.bAutoExposure;
	BlendFloat(Src.bOverride_AutoExposureMinEV100, AutoExposureMinEV100, Src.AutoExposureMinEV100);
	BlendFloat(Src.bOverride_AutoExposureMaxEV100, AutoExposureMaxEV100, Src.AutoExposureMaxEV100);
	BlendFloat(Src.bOverride_AutoExposureSpeedUp, AutoExposureSpeedUp, Src.AutoExposureSpeedUp);
	BlendFloat(Src.bOverride_AutoExposureSpeedDown, AutoExposureSpeedDown, Src.AutoExposureSpeedDown);
	BlendFloat(Src.bOverride_AutoExposureBias, AutoExposureBias, Src.AutoExposureBias);
}

namespace
{
	// 씬 라이트가 물리 단위가 아니라서 "지금 화면 밝기 = 맑은 날 낮"이라고 정한 기준점
	constexpr float SunnySixteenFstop = 16.0f;
	constexpr float SunnySixteenShutter = 200.0f;
	constexpr float SunnySixteenISO = 200.0f;
	// 노출계가 평균 밝기를 맞추려는 목표(톤매핑 전 선형 값). 실제 18%로 두면 이 씬은 원래보다 어둡게 맞춰져서,
	// 한 스톱 올려 원래 게임 화면 밝기에 가깝게 맞췄다
	constexpr float MiddleGray = 0.36f;

	float GetReferenceEV100()
	{
		static const float Reference = FPostProcessSettings::ComputeEV100(SunnySixteenFstop, SunnySixteenShutter, SunnySixteenISO);
		return Reference;
	}

	// ToneMapShader.hlsl의 ACESFilm과 같은 상수여야 화면 밝기를 거꾸로 풀 수 있다
	constexpr float AcesA = 2.51f, AcesB = 0.03f, AcesC = 2.43f, AcesD = 0.59f, AcesE = 0.14f;

	float ACESFilm(float X)
	{
		return std::clamp((X * (AcesA * X + AcesB)) / (X * (AcesC * X + AcesD) + AcesE), 0.0f, 1.0f);
	}

	// y = (aX²+bX)/(cX²+dX+e)를 X에 대한 2차식으로 놓고 양수 근을 고른다
	float InverseACESFilm(float Y)
	{
		Y = std::clamp(Y, 0.0f, 0.99f);
		const float A = AcesA - AcesC * Y;
		const float B = AcesB - AcesD * Y;
		return (-B + std::sqrt(B * B + 4.0f * A * AcesE * Y)) / (2.0f * A);
	}

	// 밝기는 감마 1(보정 없음) 화면 기준으로 맞춰 뒀다. 감마 g에서도 같은 화면 밝기가 나오려면
	// 톤매핑 결과가 그 밝기의 g제곱이어야 하므로, 그 값이 나오는 선형 목표를 거꾸로 구한다
	float GetMiddleGrayForGamma(float DisplayGamma)
	{
		if (DisplayGamma <= 1.0f) return MiddleGray;
		return InverseACESFilm(std::pow(ACESFilm(MiddleGray), DisplayGamma));
	}
}

float FPostProcessSettings::ComputeEV100(float Fstop, float ShutterSpeed, float ISO)
{
	// EV100 = log2(N^2 / t) - log2(ISO / 100), t = 1 / ShutterSpeed
	return std::log2(Fstop * Fstop * ShutterSpeed * 100.0f / ISO);
}

float FPostProcessSettings::GetMeteredEV100(float AverageLuminance, float DisplayGamma)
{
	// 화면 밝기 = 평균 휘도 x 2^(기준EV - EV) 이므로, 이것이 중간 회색이 되는 EV를 거꾸로 푼다
	return GetReferenceEV100() + std::log2(std::max(AverageLuminance, 1e-6f) / GetMiddleGrayForGamma(DisplayGamma));
}

float FPostProcessSettings::GetCameraEV100() const
{
	return ComputeEV100(std::max(DepthOfFieldFstop, 0.5f), std::max(CameraShutterSpeed, 1.0f), std::max(CameraISO, 1.0f));
}

float FPostProcessSettings::GetPhysicalCameraExposureScale() const
{
	if (!bApplyPhysicalCameraExposure) return 1.0f;
	return EV100ToExposureScale(GetCameraEV100());
}

float FPostProcessSettings::EV100ToExposureScale(float EV100)
{
	// EV가 1 커지면(한 스톱 조이면) 빛이 절반이 된다
	return std::exp2(GetReferenceEV100() - EV100);
}

float FPostProcessSettings::GetExposureScale(float AdaptedEV100) const
{
	// Bias는 EV를 낮추는 쪽이 밝아지는 쪽이라 빼 준다. UE처럼 수동 카메라에도 똑같이 얹어 두 방식의 밝기를 맞춘다
	if (bAutoExposure) return EV100ToExposureScale(AdaptedEV100 - AutoExposureBias);
	if (bApplyPhysicalCameraExposure) return EV100ToExposureScale(GetCameraEV100() - AutoExposureBias);
	return 1.0f;
}
