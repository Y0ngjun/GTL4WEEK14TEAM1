#include "ClothVirtualParticles.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <vector>

FClothVirtualParticleSettings FClothVirtualParticleSettings::Sanitized() const
{
	return {
		bEnabled,
		std::isfinite(Spacing) ? std::max(Spacing, MinSpacing) : DefaultSpacing,
		std::clamp(Subdivisions, 1, MaxSubdivisions)
	};
}

namespace
{
	using FTriangleKey = std::array<uint32, 3>;
	using FEdgeKey = std::array<uint32, 2>;
	using FWeightKey = std::array<int32, 3>;

	struct FTriangle
	{
		FTriangleKey Vertices;
		double AreaSquared;
		double LongestEdgeSquared;
		int32 Subdivisions;
	};

	struct FEdge
	{
		FEdgeKey Vertices;
		uint32 ThirdVertex;
		double LengthSquared;
		int32 Subdivisions;
	};

	int32 Subdivide(double LengthSquared, const FClothVirtualParticleSettings& Settings)
	{
		const double Ratio = std::sqrt(LengthSquared) / Settings.Spacing;
		// Import noise near an integer multiple of the spacing should not add a subdivision.
		return static_cast<int32>(std::clamp(std::ceil(Ratio - 1.e-6), 1.0,
		                                      static_cast<double>(Settings.Subdivisions)));
	}

	bool IsFinite(const FVector& Position)
	{
		return std::isfinite(Position.X) && std::isfinite(Position.Y) && std::isfinite(Position.Z);
	}

	double SquaredLength(double X, double Y, double Z)
	{
		return X * X + Y * Y + Z * Z;
	}

	double EdgeLengthSquared(const FVector& A, const FVector& B)
	{
		return SquaredLength(static_cast<double>(B.X) - A.X,
		                     static_cast<double>(B.Y) - A.Y,
		                     static_cast<double>(B.Z) - A.Z);
	}

	double TriangleAreaSquared(const FVector& A, const FVector& B, const FVector& C)
	{
		const double ABX = static_cast<double>(B.X) - A.X;
		const double ABY = static_cast<double>(B.Y) - A.Y;
		const double ABZ = static_cast<double>(B.Z) - A.Z;
		const double ACX = static_cast<double>(C.X) - A.X;
		const double ACY = static_cast<double>(C.Y) - A.Y;
		const double ACZ = static_cast<double>(C.Z) - A.Z;
		return SquaredLength(ABY * ACZ - ABZ * ACY, ABZ * ACX - ABX * ACZ,
		                     ABX * ACY - ABY * ACX);
	}
}

FClothVirtualParticleData BuildClothVirtualParticles(
    std::span<const FVector> RestPositions,
    std::span<const uint32> TriangleIndices,
    FClothVirtualParticleSettings Settings)
{
	FClothVirtualParticleData Result;
	Settings = Settings.Sanitized();
	if (!Settings.bEnabled || Settings.Subdivisions == 1)
		return Result;

	std::vector<FTriangle> Faces;
	constexpr uint32 MaxIndex = std::numeric_limits<uint16>::max();
	for (size_t Index = 0; Index + 2 < TriangleIndices.size(); Index += 3)
	{
		FTriangleKey Vertices{TriangleIndices[Index], TriangleIndices[Index + 1], TriangleIndices[Index + 2]};
		std::sort(Vertices.begin(), Vertices.end());
		const auto [A, B, C] = Vertices;
		if (A == B || B == C || C > MaxIndex || C >= RestPositions.size())
			continue;

		const FVector& P0 = RestPositions[A];
		const FVector& P1 = RestPositions[B];
		const FVector& P2 = RestPositions[C];
		if (!IsFinite(P0) || !IsFinite(P1) || !IsFinite(P2))
			continue;

		const double Longest = std::max({EdgeLengthSquared(P0, P1), EdgeLengthSquared(P1, P2),
		                                 EdgeLengthSquared(P2, P0)});
		const double Area = TriangleAreaSquared(P0, P1, P2);
		const double MinCross = Longest * 1.e-8;
		if (Area <= MinCross * MinCross)
			continue;

		const int32 Subdivisions = Subdivide(Longest, Settings);
		if (Subdivisions > 1)
			Faces.push_back({Vertices, Area, Longest, Subdivisions});
	}
	if (Faces.empty())
		return Result;

	std::sort(Faces.begin(), Faces.end(), [](const FTriangle& A, const FTriangle& B)
	{
		return A.AreaSquared == B.AreaSquared ? A.Vertices < B.Vertices : A.AreaSquared > B.AreaSquared;
	});
	Faces.erase(std::unique(Faces.begin(), Faces.end(), [](const FTriangle& A, const FTriangle& B)
	{
		return A.Vertices == B.Vertices;
	}), Faces.end());

	std::map<FWeightKey, uint32> WeightIndices;
	const auto Add = [&](const FTriangleKey& Vertices, int32 I, int32 J, int32 K)
	{
		if (Result.Particles.Num() == FClothVirtualParticleData::MaxParticles)
		{
			Result.bBudgetLimited = true;
			return false;
		}
		const int32 Divisor = std::gcd(std::gcd(I, J), K);
		const FWeightKey Key{I / Divisor, J / Divisor, K / Divisor};
		const auto [Weight, bNew] = WeightIndices.try_emplace(Key, static_cast<uint32>(Result.Weights.Num()));
		if (bNew)
		{
			const float Sum = static_cast<float>(I + J + K);
			Result.Weights.Add(FVector(I / Sum, J / Sum, K / Sum));
		}
		Result.Particles.Add(FClothVirtualParticle{Vertices[0], Vertices[1], Vertices[2], Weight->second});
		return true;
	};

	for (const FTriangle& Face : Faces)
		if (!Add(Face.Vertices, 1, 1, 1))
			return Result;

	std::map<FEdgeKey, FEdge> UniqueEdges;
	for (const FTriangle& Face : Faces)
	{
		const auto [A, B, C] = Face.Vertices;
		const auto AddEdge = [&](uint32 First, uint32 Second, uint32 Third)
		{
			const FEdgeKey Key{First, Second};
			const double Length = EdgeLengthSquared(RestPositions[First], RestPositions[Second]);
			const int32 Subdivisions = Subdivide(Length, Settings);
			if (Subdivisions == 1)
				return;
			const auto [It, bNew] = UniqueEdges.try_emplace(Key, FEdge{Key, Third, Length, Subdivisions});
			if (!bNew)
				It->second.ThirdVertex = std::min(It->second.ThirdVertex, Third);
		};
		AddEdge(A, B, C);
		AddEdge(B, C, A);
		AddEdge(A, C, B);
	}
	std::vector<FEdge> Edges;
	Edges.reserve(UniqueEdges.size());
	for (const auto& Edge : UniqueEdges)
		Edges.push_back(Edge.second);

	std::sort(Edges.begin(), Edges.end(), [](const FEdge& A, const FEdge& B)
	{
		return A.LengthSquared == B.LengthSquared ? A.Vertices < B.Vertices : A.LengthSquared > B.LengthSquared;
	});
	for (const FEdge& Edge : Edges)
	{
		const int32 N = Edge.Subdivisions;
		const FTriangleKey Vertices{Edge.Vertices[0], Edge.Vertices[1], Edge.ThirdVertex};
		for (int32 K = N / 2, Other = N / 2 + 1; K > 0 || Other < N; --K, ++Other)
		{
			if (K > 0 && !Add(Vertices, N - K, K, 0))
				return Result;
			if (Other < N && !Add(Vertices, N - Other, Other, 0))
				return Result;
		}
	}

	std::sort(Faces.begin(), Faces.end(), [](const FTriangle& A, const FTriangle& B)
	{
		return A.LongestEdgeSquared == B.LongestEdgeSquared ? A.Vertices < B.Vertices
		                                                   : A.LongestEdgeSquared > B.LongestEdgeSquared;
	});
	for (const FTriangle& Face : Faces)
	{
		const int32 N = Face.Subdivisions;
		for (int32 I = 1; I < N - 1; ++I)
		{
			for (int32 J = 1; J < N - I; ++J)
			{
				const int32 K = N - I - J;
				if (I == J && J == K)
					continue; // The centroid has already been emitted.
				if (!Add(Face.Vertices, I, J, K))
					return Result;
			}
		}
	}
	return Result;
}
