#pragma once

#include "Containers/Array.h"
#include "Math/Vector.h"

#include <span>

struct FClothVirtualParticleSettings
{
	static constexpr float DefaultSpacing = 25.0f;
	static constexpr float MinSpacing = 0.1f;
	static constexpr int32 MaxSubdivisions = 16;

	bool bEnabled = false;
	float Spacing = DefaultSpacing;
	int32 Subdivisions = MaxSubdivisions;

	FClothVirtualParticleSettings Sanitized() const;
	bool operator==(const FClothVirtualParticleSettings&) const = default;
};

struct FClothVirtualParticle
{
	uint32 A, B, C, WeightIndex;

	bool operator==(const FClothVirtualParticle&) const = default;
};

struct FClothVirtualParticleData
{
	static constexpr int32 MaxParticles = 2048;

	TArray<FClothVirtualParticle> Particles;
	TArray<FVector> Weights;
	bool bBudgetLimited = false;
};

FClothVirtualParticleData BuildClothVirtualParticles(
    std::span<const FVector> RestPositions,
    std::span<const uint32> TriangleIndices,
    FClothVirtualParticleSettings Settings);
