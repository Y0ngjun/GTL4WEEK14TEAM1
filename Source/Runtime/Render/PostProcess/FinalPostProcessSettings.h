#pragma once

#include "Render/PostProcess/PostProcessSettings.h"
#include "FinalPostProcessSettings.generated.h"

USTRUCT()
struct FFinalPostProcessSettings : public FPostProcessSettings
{
	GENERATED_BODY()

public:
	void OverridePostProcessSettings(const FPostProcessSettings& InSettings, float Weight = 1.0f);

	float BokehRoundness() const;

	float GetDOFDepthBlurRadiusRatio(float Width) const;
};
