#include "EnginePCH.h"
#include "ClothPaintTool.h"
#include "Components/SkeletalMeshComponent.h"
#include "Editor/Viewports/EditorViewportClient.h"
#include "Render/RenderCommand.h"
#include "Render/RenderResourceManager.h"
#include "Render/SceneView.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

namespace
{
	constexpr float StrokeSpacing = 4.0f;
	constexpr float VisibilityTolerance = 0.05f;
	constexpr int32 MaxUndoSteps = 100;

	bool ResolvePaintTarget(const USkeletalMesh& Mesh, int32 SectionIndex, FClothMaxDistanceTarget& Out)
	{
		if (!Mesh.IsSkeletalMeshAsset() || Mesh.AssetNeedsReload())
			return false;

		for (const FClothingSectionBinding& Binding : Mesh.GetClothingBindings())
		{
			if (Binding.RenderLOD != 0 || Binding.SectionIndex != SectionIndex)
				continue;

			const FClothingAsset* Asset = Mesh.GetClothingAsset(Binding.ClothingAssetIndex);
			if (!Asset || !Asset->LODs.IsValidIndex(Binding.ClothLODIndex))
				return false;

			Out = {0, SectionIndex, Asset->Id, Binding.ClothLODIndex,
			       Asset->LODs[Binding.ClothLODIndex].SourceSectionSignature};
			return !Out.SectionSignature.empty();
		}
		return false;
	}

	TArray<float> MeshDistances(const TArray<FVector>& Positions,
	                            const TArray<TArray<int32>>& Neighbors,
	                            const TArray<int32>& Seeds)
	{
		TArray<float> Distances;
		Distances.Init(std::numeric_limits<float>::infinity(), Positions.Num());
		using FQueueItem = std::pair<float, int32>;
		std::priority_queue<FQueueItem, std::vector<FQueueItem>, std::greater<FQueueItem>> Queue;
		for (int32 Seed : Seeds)
		{
			Distances[Seed] = 0.0f;
			Queue.emplace(0.0f, Seed);
		}

		while (!Queue.empty())
		{
			const auto [Distance, Vertex] = Queue.top();
			Queue.pop();
			if (Distance > Distances[Vertex])
				continue;

			for (int32 Neighbor : Neighbors[Vertex])
			{
				const float Candidate = Distance + (Positions[Neighbor] - Positions[Vertex]).Size();
				if (Candidate < Distances[Neighbor])
				{
					Distances[Neighbor] = Candidate;
					Queue.emplace(Candidate, Neighbor);
				}
			}
		}
		return Distances;
	}
}

bool FClothPaintTool::Begin(USkeletalMesh& Mesh, const USkeletalMeshComponent& Component,
                           int32 SectionIndex, FString& OutError)
{
	End();
	OutError.clear();
	FClothMaxDistanceTarget NewTarget;
	if (!ResolvePaintTarget(Mesh, SectionIndex, NewTarget))
	{
		OutError = "Bind a Cloth Asset to the selected section before painting.";
		return false;
	}

	const FClothingLODData& Cloth = *Mesh.FindClothingLOD(0, SectionIndex);
	const auto& LOD = Mesh.GetRenderData().LODRenderData[0];
	FClothingLODData RenderPose;
	RenderPose.RestPositions.Reserve(LOD.BindVertices.Num());
	for (const FVertex& Vertex : LOD.BindVertices)
		RenderPose.RestPositions.Add(Vertex.Position);
	RenderPose.SkinWeights = LOD.SkinWeights;

	if (!Component.BuildClothPose(RenderPose, RenderPositions) ||
	    !Component.BuildClothPose(Cloth, SimPositions))
	{
		End();
		OutError = "Could not build the preview pose for Cloth Paint.";
		return false;
	}

	const FMatrix ToWorld = Component.GetWorldMatrix();
	for (FVector& Position : RenderPositions)
		Position = ToWorld.TransformPosition(Position);
	for (FVector& Position : SimPositions)
		Position = ToWorld.TransformPosition(Position);

	RenderToSim = Cloth.RenderToSimVertex;
	RestPositions = Cloth.RestPositions;
	Values = Cloth.MaxDistances;
	Neighbors.SetNum(Values.Num());
	SimTriangles.SetNum(Values.Num());
	for (int32 Index = 0; Index < Cloth.TriangleIndices.Num(); Index += 3)
	{
		for (int32 Corner = 0; Corner < 3; ++Corner)
		{
			const int32 Vertex = static_cast<int32>(Cloth.TriangleIndices[Index + Corner]);
			const int32 Neighbor = static_cast<int32>(Cloth.TriangleIndices[Index + (Corner + 1) % 3]);
			Neighbors[Vertex].AddUnique(Neighbor);
			Neighbors[Neighbor].AddUnique(Vertex);
		}
	}

	TArray<int32> TriangleIds;
	TriangleIds.Reserve(LOD.Indices.Num() / 3);
	for (int32 Section = 0; Section < LOD.Sections.Num(); ++Section)
	{
		const auto& RenderSection = LOD.Sections[Section];
		const uint64 EndIndex = static_cast<uint64>(RenderSection.StartIndex) + RenderSection.IndexCount;
		if (EndIndex > static_cast<uint64>(LOD.Indices.Num()))
			continue;

		for (uint32 Index = RenderSection.StartIndex; Index + 2 < EndIndex; Index += 3)
		{
			const uint32 A = LOD.Indices[Index];
			const uint32 B = LOD.Indices[Index + 1];
			const uint32 C = LOD.Indices[Index + 2];
			if (!RenderPositions.IsValidIndex(A) || !RenderPositions.IsValidIndex(B) ||
			    !RenderPositions.IsValidIndex(C))
				continue;

			const FVector Normal = (RenderPositions[B] - RenderPositions[A])
			                           .Cross(RenderPositions[C] - RenderPositions[A]).GetSafeNormal();
			if (Normal.SizeSquared() == 0.0f)
				continue;

			const auto Triangle = static_cast<int32>(Triangles.Add({Section, {A, B, C}, Normal}));
			TriangleIds.Add(Triangle);
			if (Section == SectionIndex)
			{
				SectionTriangles.Add(Triangle);
				for (uint32 Vertex : Triangles[Triangle].Vertices)
					SimTriangles[RenderToSim[Vertex]].Add(Triangle);
			}
		}
	}

	if (SectionTriangles.IsEmpty())
	{
		End();
		OutError = "The selected Cloth section has no paintable triangles in this pose.";
		return false;
	}

	TriangleBVH.Build({TriangleIds.GetData(), static_cast<size_t>(TriangleIds.Num())});
	Target = std::move(NewTarget);
	ResetDisplayRange();
	return true;
}

void FClothPaintTool::End()
{
	Target = {};
	CancelInteraction();
	ClearHover();
	TriangleBVH.Clear();
	Triangles.Empty();
	SectionTriangles.Empty();
	RenderPositions.Empty();
	SimPositions.Empty();
	RenderToSim.Empty();
	RestPositions.Empty();
	Neighbors.Empty();
	SimTriangles.Empty();
	Values.Empty();
	SurfaceVertices.Empty();
	Markers.Empty();
	bSurfaceDirty = true;
}

bool FClothPaintTool::SameTarget(const FClothMaxDistanceTarget& A, const FClothMaxDistanceTarget& B)
{
	return A.RenderLOD == B.RenderLOD && A.SectionIndex == B.SectionIndex &&
	       A.ClothingAssetId == B.ClothingAssetId && A.ClothLODIndex == B.ClothLODIndex &&
	       A.SectionSignature == B.SectionSignature;
}

bool FClothPaintTool::IsTargetCurrent(const USkeletalMesh& Mesh) const
{
	FClothMaxDistanceTarget Current;
	return IsActive() && ResolvePaintTarget(Mesh, Target.SectionIndex, Current) && SameTarget(Current, Target);
}

void FClothPaintTool::ForgetSection(int32 SectionIndex)
{
	for (int32 Index = Histories.Num() - 1; Index >= 0; --Index)
		if (Histories[Index].Target.SectionIndex == SectionIndex)
			Histories.RemoveAtSwap(Index);
}

FClothPaintTool::FHit FClothPaintTool::Pick(const FRay& Ray, float& OutDistance) const
{
	FHit Hit;
	const FPreparedRay Prepared(Ray);
	TriangleBVH.TraceClosest(
	    [&](const FBox& Bounds, float& OutT)
	    {
		    return RayIntersectsAABB(Prepared, Bounds.Min, Bounds.Max, OutT);
	    },
	    [&](int32 Index, float& InOutDistance)
	    {
		    const auto& V = Triangles[Index].Vertices;
		    float Distance;
		    if (!RayIntersectsTriangle(Ray, RenderPositions[V[0]], RenderPositions[V[1]],
		                               RenderPositions[V[2]], Distance) ||
		        Distance < 0.0f || Distance >= InOutDistance)
			    return false;

		    InOutDistance = Distance;
		    Hit = {Index, Ray.PointAt(Distance)};
		    return true;
	    }, OutDistance);
	return Hit;
}

FClothPaintTool::FHit FClothPaintTool::PickCloth(const FEditorViewportClient& Viewport, const FVector2& Mouse) const
{
	const FRay Ray = Viewport.DeprojectScreenToWorld(Mouse);
	float Distance = std::numeric_limits<float>::max();
	const FHit Hit = Pick(Ray, Distance);
	if (Hit.Triangle == INDEX_NONE)
		return {};

	const FTriangle& Triangle = Triangles[Hit.Triangle];
	if (Triangle.SectionIndex != Target.SectionIndex ||
	    (Settings.bIgnoreBackFaces && FVector::Dot(Triangle.Normal, Ray.Direction) >= 0.0f))
		return {};

	return Hit;
}

bool FClothPaintTool::IsVisible(const FEditorViewportClient& Viewport, const FMatrix& ViewProjection,
                                int32 Vertex) const
{
	const FVector& Position = SimPositions[Vertex];
	const FVector2 Screen = WorldToScreen(Position, ViewProjection, Viewport.GetWidth(), Viewport.GetHeight());
	if (Screen.X < 0.0f || Screen.Y < 0.0f ||
	    Screen.X >= Viewport.GetWidth() || Screen.Y >= Viewport.GetHeight())
		return false;

	const FRay Ray = Viewport.DeprojectScreenToWorld(Screen);
	if (Settings.bIgnoreBackFaces &&
	    std::none_of(SimTriangles[Vertex].begin(), SimTriangles[Vertex].end(),
	                 [&](int32 Index) { return FVector::Dot(Triangles[Index].Normal, Ray.Direction) < 0.0f; }))
		return false;

	float Distance = FVector::Dot(Position - Ray.Origin, Ray.Direction) - VisibilityTolerance;
	return Distance > 0.0f && Pick(Ray, Distance).Triangle == INDEX_NONE;
}

int32 FClothPaintTool::ClosestVertex(const FHit& Hit) const
{
	if (Hit.Triangle == INDEX_NONE)
		return INDEX_NONE;

	int32 Closest = INDEX_NONE;
	float MinDistance = std::numeric_limits<float>::max();
	for (uint32 RenderVertex : Triangles[Hit.Triangle].Vertices)
	{
		const float Distance = (RenderPositions[RenderVertex] - Hit.Position).SizeSquared();
		if (Distance < MinDistance)
		{
			MinDistance = Distance;
			Closest = RenderToSim[RenderVertex];
		}
	}
	return Closest;
}

void FClothPaintTool::UpdateHover(const FEditorViewportClient& Viewport, const FVector2& Mouse)
{
	Hover = PickCloth(Viewport, Mouse);
}

bool FClothPaintTool::GetHoveredValue(float& OutValue) const
{
	const int32 Vertex = ClosestVertex(Hover);
	if (Vertex == INDEX_NONE)
		return false;

	OutValue = (bStrokeActive ? Draft : Values)[Vertex];
	return true;
}

void FClothPaintTool::Press(const FEditorViewportClient& Viewport, const FVector2& Mouse,
                            bool bShift, bool bCtrl, USkeletalMesh& Mesh, FString& OutError)
{
	if (Hover.Triangle == INDEX_NONE)
		return;

	switch (Settings.Tool)
	{
	case EClothPaintTool::Fill:
		Fill(ClosestVertex(Hover), Mesh, OutError);
		break;
	case EClothPaintTool::Gradient:
		SetGradientPoint(Hover, bCtrl);
		break;
	case EClothPaintTool::Brush:
	case EClothPaintTool::Smooth:
		Draft = Values;
		bStrokeActive = true;
		LastStamp = Mouse;
		bHasLastStamp = true;
		Stamp(Viewport, Hover, bShift);
		break;
	}
}

void FClothPaintTool::Drag(const FEditorViewportClient& Viewport, const FVector2& Mouse, bool bShift)
{
	if (!bStrokeActive)
		return;

	if (!bHasLastStamp)
	{
		LastStamp = Mouse;
		bHasLastStamp = true;
		Stamp(Viewport, Hover, bShift);
		return;
	}

	const FVector2 Offset = Mouse - LastStamp;
	const float Length = std::hypot(Offset.X, Offset.Y);
	const int32 Steps = static_cast<int32>(Length / StrokeSpacing);
	if (Steps == 0)
		return;

	const FVector2 Step = Offset * (StrokeSpacing / Length);
	for (int32 Index = 0; Index < Steps; ++Index)
	{
		LastStamp = LastStamp + Step;
		Stamp(Viewport, PickCloth(Viewport, LastStamp), bShift);
	}
}

void FClothPaintTool::Release(USkeletalMesh& Mesh, FString& OutError)
{
	if (bStrokeActive)
		Commit(Mesh, OutError);
}

void FClothPaintTool::CancelStroke()
{
	if (bStrokeActive)
		bSurfaceDirty = true;
	Draft.Empty();
	bStrokeActive = false;
	bHasLastStamp = false;
}

bool FClothPaintTool::CancelInteraction()
{
	const bool bHadInteraction = bStrokeActive || !GradientStartPoints.IsEmpty() || !GradientEndPoints.IsEmpty();
	CancelStroke();
	GradientStartPoints.Empty();
	GradientEndPoints.Empty();
	return bHadInteraction;
}

void FClothPaintTool::SetTool(EClothPaintTool Tool)
{
	if (Settings.Tool == Tool)
		return;
	CancelInteraction();
	Settings.Tool = Tool;
}

void FClothPaintTool::Stamp(const FEditorViewportClient& Viewport, const FHit& Hit, bool bErase)
{
	if (Hit.Triangle == INDEX_NONE)
		return;

	const float Radius = Settings.Radius;
	const float InnerRadius = Radius * (1.0f - Settings.Falloff);
	const float RadiusSquared = Radius * Radius;
	const bool bSmooth = Settings.Tool == EClothPaintTool::Smooth;
	const float PaintValue = bErase ? 0.0f : Settings.PaintValue;
	const FMatrix ViewProjection = Viewport.GetViewProjectionMatrix();
	TArray<FClothMaxDistanceEdit> Changes;
	for (int32 Vertex = 0; Vertex < SimPositions.Num(); ++Vertex)
	{
		const float DistanceSquared = (SimPositions[Vertex] - Hit.Position).SizeSquared();
		if (DistanceSquared > RadiusSquared || !IsVisible(Viewport, ViewProjection, Vertex))
			continue;

		const float OldValue = Draft[Vertex];
		float TargetValue = PaintValue;
		if (bSmooth)
		{
			if (OldValue == 0.0f || Neighbors[Vertex].IsEmpty())
				continue;

			double Sum = 0.0;
			for (int32 Neighbor : Neighbors[Vertex])
				Sum += Draft[Neighbor];
			TargetValue = static_cast<float>(Sum / Neighbors[Vertex].Num());
		}

		const float Distance = std::sqrt(DistanceSquared);
		const float Weight = Distance <= InnerRadius
		                         ? 1.0f
		                         : (Radius - Distance) / (Radius - InnerRadius);
		const float Alpha = Settings.Strength * Weight;
		if (Alpha <= 0.0f)
			continue;

		float NewValue = std::lerp(OldValue, TargetValue, Alpha);
		if (!bSmooth && TargetValue == 0.0f && NewValue < 0.01f)
			NewValue = 0.0f;

		if (NewValue != OldValue)
			Changes.Add({Vertex, NewValue});
	}

	for (const FClothMaxDistanceEdit& Change : Changes)
		Draft[Change.SimVertexIndex] = Change.MaxDistance;
	bSurfaceDirty |= !Changes.IsEmpty();
}

void FClothPaintTool::Fill(int32 Seed, USkeletalMesh& Mesh, FString& OutError)
{
	Draft = Values;
	TArray<uint8> Visited;
	Visited.Init(0, Values.Num());
	TArray<int32> Queue;
	Queue.Add(Seed);
	Visited[Seed] = 1;
	const float SeedValue = Values[Seed];

	for (int32 Index = 0; Index < Queue.Num(); ++Index)
	{
		const int32 Vertex = Queue[Index];
		Draft[Vertex] = Settings.FillValue;
		for (int32 Neighbor : Neighbors[Vertex])
		{
			if (!Visited[Neighbor] &&
			    std::abs(Values[Neighbor] - SeedValue) <= Settings.FillThreshold)
			{
				Visited[Neighbor] = 1;
				Queue.Add(Neighbor);
			}
		}
	}
	Commit(Mesh, OutError);
}

void FClothPaintTool::SetGradientPoint(const FHit& Hit, bool bEnd)
{
	const int32 Vertex = ClosestVertex(Hit);
	if (Vertex == INDEX_NONE)
		return;

	auto& Points = bEnd ? GradientEndPoints : GradientStartPoints;
	auto& OtherPoints = bEnd ? GradientStartPoints : GradientEndPoints;
	OtherPoints.Remove(Vertex);
	Points.AddUnique(Vertex);
}

bool FClothPaintTool::CanApplyGradient() const
{
	return IsActive() && Settings.Tool == EClothPaintTool::Gradient &&
	       !GradientStartPoints.IsEmpty() && !GradientEndPoints.IsEmpty();
}

void FClothPaintTool::ApplyGradient(USkeletalMesh& Mesh, FString& OutError)
{
	if (!CanApplyGradient())
		return;

	const auto FromStart = MeshDistances(RestPositions, Neighbors, GradientStartPoints);
	const auto FromEnd = MeshDistances(RestPositions, Neighbors, GradientEndPoints);
	Draft = Values;
	bool bHasConnectedPoints = false;
	for (int32 Vertex = 0; Vertex < Values.Num(); ++Vertex)
	{
		if (!std::isfinite(FromStart[Vertex]) || !std::isfinite(FromEnd[Vertex]))
			continue;

		bHasConnectedPoints = true;
		const float Total = FromStart[Vertex] + FromEnd[Vertex];
		const float Fraction = Total > 0.0f ? FromStart[Vertex] / Total : 0.0f;
		Draft[Vertex] = std::lerp(Settings.GradientStart, Settings.GradientEnd, Fraction);
	}

	if (!bHasConnectedPoints)
	{
		Draft.Empty();
		OutError = "Select Gradient start and end vertices on the same connected Cloth surface.";
		return;
	}

	Commit(Mesh, OutError);
	if (OutError.empty())
	{
		GradientStartPoints.Empty();
		GradientEndPoints.Empty();
	}
}

FClothPaintTool::FHistory* FClothPaintTool::FindHistory()
{
	return const_cast<FHistory*>(std::as_const(*this).FindHistory());
}

const FClothPaintTool::FHistory* FClothPaintTool::FindHistory() const
{
	for (const FHistory& History : Histories)
		if (SameTarget(History.Target, Target))
			return &History;
	return nullptr;
}

void FClothPaintTool::Commit(USkeletalMesh& Mesh, FString& OutError)
{
	OutError.clear();
	TArray<FVertexChange> Changes;
	TArray<FClothMaxDistanceEdit> Edits;
	for (int32 Vertex = 0; Vertex < Draft.Num(); ++Vertex)
	{
		if (Values[Vertex] != Draft[Vertex])
		{
			Changes.Add({Vertex, Values[Vertex], Draft[Vertex]});
			Edits.Add({Vertex, Draft[Vertex]});
		}
	}

	if (!Edits.IsEmpty() &&
	    Mesh.ApplyClothMaxDistanceEdits(Target, {Edits.GetData(), static_cast<size_t>(Edits.Num())}, OutError))
	{
		for (const FClothMaxDistanceEdit& Edit : Edits)
			Values[Edit.SimVertexIndex] = Edit.MaxDistance;

		FHistory* History = FindHistory();
		if (!History)
		{
			Histories.Add({Target, {}, {}});
			History = &Histories.Last();
		}
		History->Undo.Add(std::move(Changes));
		if (History->Undo.Num() > MaxUndoSteps)
			History->Undo.RemoveAt(0);
		History->Redo.Empty();
		ResetDisplayRange();
	}
	CancelStroke();
}

bool FClothPaintTool::CanUndo(bool bRedo) const
{
	const FHistory* History = FindHistory();
	return IsActive() && !bStrokeActive && History && !(bRedo ? History->Redo : History->Undo).IsEmpty();
}

void FClothPaintTool::Undo(USkeletalMesh& Mesh, bool bRedo, FString& OutError)
{
	if (!CanUndo(bRedo))
		return;

	FHistory* History = FindHistory();
	auto& Source = bRedo ? History->Redo : History->Undo;
	auto& Destination = bRedo ? History->Undo : History->Redo;
	TArray<FClothMaxDistanceEdit> Edits;
	for (const FVertexChange& Change : Source.Last())
		Edits.Add({Change.Vertex, bRedo ? Change.After : Change.Before});

	if (!Mesh.ApplyClothMaxDistanceEdits(Target, {Edits.GetData(), static_cast<size_t>(Edits.Num())}, OutError))
		return;

	for (const FClothMaxDistanceEdit& Edit : Edits)
		Values[Edit.SimVertexIndex] = Edit.MaxDistance;
	Destination.Add(std::move(Source.Last()));
	Source.RemoveLast();
	ResetDisplayRange();
}

void FClothPaintTool::ResetDisplayRange()
{
	const auto [Min, Max] = std::minmax_element(Values.begin(), Values.end());
	AutoMin = *Min;
	AutoMax = *Max;
	if (AutoMax == AutoMin)
	{
		AutoMin = 0.0f;
		AutoMax = std::max(1.0f, AutoMax);
	}
	bSurfaceDirty = true;
}

void FClothPaintTool::UpdateDrawVertices()
{
	const float Min = Settings.bAutoRange ? AutoMin : Settings.ViewMin;
	const float Max = Settings.bAutoRange ? AutoMax : Settings.ViewMax;
	const FVector Range(Min, Max, Settings.Opacity);
	if (!bSurfaceDirty && Range == PreviousViewSettings)
		return;

	PreviousViewSettings = Range;
	SurfaceVertices.Reset();
	SurfaceVertices.Reserve(SectionTriangles.Num() * 3);
	const auto& VisibleValues = bStrokeActive ? Draft : Values;
	for (int32 Triangle : SectionTriangles)
	{
		for (uint32 Vertex : Triangles[Triangle].Vertices)
		{
			const float Value = VisibleValues[RenderToSim[Vertex]];
			const float Fraction = std::clamp((Value - Min) / std::max(Max - Min, 0.001f), 0.0f, 1.0f);
			const float Gray = std::lerp(0.05f, 1.0f, Fraction);
			SurfaceVertices.Add({RenderPositions[Vertex], {Gray, Gray, Gray, Settings.Opacity}});
		}
	}
}

bool FClothPaintTool::EnsureVertexBuffer(TUniquePtr<FVertexBuffer>& Buffer,
                                        uint32& Capacity, uint32 VertexCount)
{
	if (Buffer && Capacity >= VertexCount)
		return true;

	const uint32 NewCapacity = std::max(1024u, std::max(VertexCount, Capacity + Capacity / 2));
	Buffer = RenderCommand::CreateDynamicVertexBuffer(NewCapacity * sizeof(FOverlayVertex),
	                                                  sizeof(FOverlayVertex));
	Capacity = Buffer ? NewCapacity : 0;
	return Buffer != nullptr;
}

void FClothPaintTool::Draw(const FSceneView& View)
{
	if (!IsActive())
		return;

	FShaderProgram* Shader = FRenderResourceManager::GetShaderProgram("EngineResources/Shaders/ClothPaintOverlay.hlsl");
	if (!Shader)
		return;

	if (!ViewBuffer)
		ViewBuffer = RenderCommand::CreateConstantBuffer(sizeof(FMatrix));
	if (!ViewBuffer)
		return;

	RenderCommand::UpdateBufferData(ViewBuffer.get(), &View.ViewProjectionMatrix, sizeof(FMatrix));
	FPipelineState Pipeline;
	Pipeline.Shader = Shader;
	Pipeline.RasterizerState = ERasterizerState::SolidNone;
	Pipeline.DepthStencilState = EDepthStencilState::ReadOnly;
	Pipeline.BlendState = EBlendState::AlphaBlend;

	const bool bUpdateSurface = bSurfaceDirty;
	const FVector OldViewSettings = PreviousViewSettings;
	UpdateDrawVertices();
	if (Settings.Opacity > 0.0f &&
	    EnsureVertexBuffer(SurfaceBuffer, SurfaceCapacity, SurfaceVertices.Num()))
	{
		if (bUpdateSurface || OldViewSettings != PreviousViewSettings)
			RenderCommand::UpdateBufferData(SurfaceBuffer.get(), SurfaceVertices.GetData(),
			                                SurfaceVertices.Num() * sizeof(FOverlayVertex));
		Pipeline.Topology = EPrimitiveTopology::TriangleList;
		RenderCommand::BindPipelineState(Pipeline);
		RenderCommand::BindVertexBuffer(SurfaceBuffer.get());
		RenderCommand::BindConstantBuffer(0, ViewBuffer.get(), EShaderBindFlagBits::Vertex);
		RenderCommand::Draw(SurfaceVertices.Num());
		bSurfaceDirty = false;
	}

	Markers.Reset();
	const auto AddLine = [this](const FVector& A, const FVector& B, const FVector4& Color)
	{
		Markers.Add({A, Color});
		Markers.Add({B, Color});
	};
	const auto AddPoint = [&](const FVector& Position, float Radius, const FVector4& Color)
	{
		AddLine(Position - FVector(Radius, 0.0f, 0.0f), Position + FVector(Radius, 0.0f, 0.0f), Color);
		AddLine(Position - FVector(0.0f, Radius, 0.0f), Position + FVector(0.0f, Radius, 0.0f), Color);
		AddLine(Position - FVector(0.0f, 0.0f, Radius), Position + FVector(0.0f, 0.0f, Radius), Color);
	};

	const auto& VisibleValues = bStrokeActive ? Draft : Values;
	if (Settings.bShowPins)
	{
		for (int32 Index = 0; Index < VisibleValues.Num(); ++Index)
			if (VisibleValues[Index] == 0.0f)
				AddPoint(SimPositions[Index], 0.75f, FVector4(1.0f, 0.65f, 0.1f, 1.0f));
	}
	for (int32 Vertex : GradientStartPoints)
		AddPoint(SimPositions[Vertex], 1.5f, FVector4(0.2f, 1.0f, 0.3f, 1.0f));
	for (int32 Vertex : GradientEndPoints)
		AddPoint(SimPositions[Vertex], 1.5f, FVector4(1.0f, 0.25f, 0.2f, 1.0f));

	if (Hover.Triangle != INDEX_NONE)
	{
		if (Settings.Tool == EClothPaintTool::Brush || Settings.Tool == EClothPaintTool::Smooth)
		{
			const FVector& Normal = Triangles[Hover.Triangle].Normal;
			const FVector Axis = std::abs(Normal.Z) < 0.9f
			                         ? FVector(0.0f, 0.0f, 1.0f) : FVector(0.0f, 1.0f, 0.0f);
			const FVector Tangent = Normal.Cross(Axis).GetSafeNormal();
			const FVector Bitangent = Normal.Cross(Tangent);
			constexpr int32 Segments = 40;
			FVector Previous = Hover.Position + Tangent * Settings.Radius;
			for (int32 Index = 1; Index <= Segments; ++Index)
			{
				const float Angle = 6.283185307f * Index / Segments;
				const FVector Next = Hover.Position +
				    (Tangent * std::cos(Angle) + Bitangent * std::sin(Angle)) * Settings.Radius;
				AddLine(Previous, Next, FVector4(0.1f, 0.85f, 1.0f, 1.0f));
				Previous = Next;
			}
		}
		else
		{
			AddPoint(SimPositions[ClosestVertex(Hover)], 1.5f, FVector4(0.1f, 0.85f, 1.0f, 1.0f));
		}
	}

	if (!Markers.IsEmpty() && EnsureVertexBuffer(MarkerBuffer, MarkerCapacity, Markers.Num()))
	{
		RenderCommand::UpdateBufferData(MarkerBuffer.get(), Markers.GetData(), Markers.Num() * sizeof(FOverlayVertex));
		Pipeline.Topology = EPrimitiveTopology::LineList;
		RenderCommand::BindPipelineState(Pipeline);
		RenderCommand::BindVertexBuffer(MarkerBuffer.get());
		RenderCommand::BindConstantBuffer(0, ViewBuffer.get(), EShaderBindFlagBits::Vertex);
		RenderCommand::Draw(Markers.Num());
	}
}

void FClothPaintTool::FillFromBone(USkeletalMesh& Mesh, int32 BoneIndex, bool bClear, FString& OutError)
{
 if (!IsTargetCurrent(Mesh) || bStrokeActive) { OutError = "Select a current Cloth section first."; return; }
 const auto* Cloth = Mesh.FindClothingLOD(0, Target.SectionIndex);
 const auto& Bones = Mesh.GetRefSkeleton();
 if (!Cloth || (!bClear && !Bones.IsValidIndex(BoneIndex))) { OutError = "Select a bone first."; return; }
 Draft = Values;
 for (int32 Vertex = 0; Vertex < Draft.Num(); ++Vertex) {
  if (bClear) { Draft[Vertex] = 0.0f; continue; }
  if (!Cloth->SkinWeights.IsValidIndex(Vertex)) continue;
  float Weight = 0.0f;
  const auto& Skin = Cloth->SkinWeights[Vertex];
  for (int32 Influence = 0; Influence < MaxBoneInfluences; ++Influence) {
   int32 Bone = Skin.BoneIndex[Influence];
   for (int32 Guard = 0; Bones.IsValidIndex(Bone) && Guard < Bones.Num(); ++Guard) {
    if (Bone == BoneIndex) { Weight += Skin.Weight[Influence]; break; }
    Bone = Bones[Bone].ParentIndex;
   }
  }
  Draft[Vertex] = Settings.PaintValue * std::clamp(Weight, 0.0f, 1.0f);
 }
 Commit(Mesh, OutError);
}
