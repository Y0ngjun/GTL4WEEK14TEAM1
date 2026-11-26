#include "EnginePCH.h"
#include "Render/PostProcess/FinalPostProcessSettings.h"
#include "Math/EngineMath.h"
#include <algorithm>

void FFinalPostProcessSettings::OverridePostProcessSettings(const FPostProcessSettings& InSettings, float Weight)
{
	Blend(InSettings, std::clamp(Weight, 0.0f, 1.0f));
}

float FFinalPostProcessSettings::BokehRoundness() const
{
	return 1.f - FMath::Clamp((DepthOfFieldFstop - DepthOfFieldMinFstop) / DepthOfFieldFstop, 0.f, 1.f);
}

float FFinalPostProcessSettings::GetDOFDepthBlurRadiusRatio(float Width) const
{
	return Width / 1920.f;
}
