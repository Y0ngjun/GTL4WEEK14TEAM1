#pragma once

#include "Collision/Ray.h"
#include "Math/BVH.h"
#include "Render/SkeletalMesh.h"

class FEditorViewportClient;
class USkeletalMeshComponent;
struct FSceneView;

enum class EClothPaintTool : uint8
{
	Brush,
	Fill,
	Gradient,
	Smooth
};

struct FClothPaintSettings
{
	EClothPaintTool Tool = EClothPaintTool::Brush;
	float PaintValue = 25.0f;
	float Radius = 25.0f;
	float Strength = 1.0f;
	float Falloff = 0.5f;
	float FillValue = 25.0f;
	float FillThreshold = 0.0f;
	float GradientStart = 0.0f;
	float GradientEnd = 100.0f;
	float ViewMin = 0.0f;
	float ViewMax = 100.0f;
	float Opacity = 0.75f;
	bool bIgnoreBackFaces = true;
	bool bAutoRange = true;
	bool bShowPins = true;
};

class FClothPaintTool
{
public:
	FClothPaintSettings Settings;

	bool Begin(USkeletalMesh& Mesh, const USkeletalMeshComponent& Component, int32 SectionIndex, FString& OutError);
	void End();
	bool IsActive() const { return Target.SectionIndex != INDEX_NONE; }
	bool IsTargetCurrent(const USkeletalMesh& Mesh) const;
	bool IsPainting() const { return bStrokeActive; }
	void CancelStroke();
	bool CancelInteraction();
	void ForgetSection(int32 SectionIndex);
	void ClearHistory() { Histories.Empty(); }

	void UpdateHover(const FEditorViewportClient& Viewport, const FVector2& Mouse);
	bool GetHoveredValue(float& OutValue) const;
	void ClearHover() { Hover = {}; bHasLastStamp = false; }
	void Press(const FEditorViewportClient& Viewport, const FVector2& Mouse, bool bShift, bool bCtrl,
	           USkeletalMesh& Mesh, FString& OutError);
	void Drag(const FEditorViewportClient& Viewport, const FVector2& Mouse, bool bShift);
	void Release(USkeletalMesh& Mesh, FString& OutError);
	bool CanApplyGradient() const;
	void ApplyGradient(USkeletalMesh& Mesh, FString& OutError);
	void SetTool(EClothPaintTool Tool);
	void FillFromBone(USkeletalMesh& Mesh, int32 BoneIndex, bool bClear, FString& OutError);
	bool CanUndo(bool bRedo) const;
	void Undo(USkeletalMesh& Mesh, bool bRedo, FString& OutError);
	void Draw(const FSceneView& View);

private:
	struct FTriangle
	{
		int32 SectionIndex;
		uint32 Vertices[3];
		FVector Normal;
	};

	struct FHit
	{
		int32 Triangle = INDEX_NONE;
		FVector Position = FVector::ZeroVector;
	};

	struct FVertexChange
	{
		int32 Vertex;
		float Before;
		float After;
	};

	struct FOverlayVertex
	{
		FVector Position;
		FVector4 Color;
	};
	static_assert(offsetof(FOverlayVertex, Color) == sizeof(float) * 3);

	struct FHistory
	{
		FClothMaxDistanceTarget Target;
		TArray<TArray<FVertexChange>> Undo;
		TArray<TArray<FVertexChange>> Redo;
	};

	static bool SameTarget(const FClothMaxDistanceTarget& A, const FClothMaxDistanceTarget& B);
	FHistory* FindHistory();
	const FHistory* FindHistory() const;
	FHit Pick(const FRay& Ray, float& OutDistance) const;
	FHit PickCloth(const FEditorViewportClient& Viewport, const FVector2& Mouse) const;
	bool IsVisible(const FEditorViewportClient& Viewport, const FMatrix& ViewProjection, int32 Vertex) const;
	int32 ClosestVertex(const FHit& Hit) const;
	void Stamp(const FEditorViewportClient& Viewport, const FHit& Hit, bool bErase);
	void SetGradientPoint(const FHit& Hit, bool bEnd);
	void Fill(int32 Seed, USkeletalMesh& Mesh, FString& OutError);
	void Commit(USkeletalMesh& Mesh, FString& OutError);
	void UpdateDrawVertices();
	void ResetDisplayRange();
	bool EnsureVertexBuffer(TUniquePtr<FVertexBuffer>& Buffer, uint32& Capacity, uint32 VertexCount);

	FClothMaxDistanceTarget Target;
	TArray<FVector> RenderPositions;
	TArray<FVector> SimPositions;
	TArray<int32> RenderToSim;
	TArray<FTriangle> Triangles;
	TArray<int32> SectionTriangles;
	TBVH<int32> TriangleBVH{[this](int32 Index)
	{
		const FTriangle& Triangle = Triangles[Index];
		const FVector& A = RenderPositions[Triangle.Vertices[0]];
		const FVector& B = RenderPositions[Triangle.Vertices[1]];
		const FVector& C = RenderPositions[Triangle.Vertices[2]];
		FBox Bounds{A, A};
		Bounds.Expand(FBox{B, B});
		Bounds.Expand(FBox{C, C});
		// A ray parallel to a triangle's zero-width bounds can run on a slab boundary.
		Bounds.Min -= FVector(0.01f);
		Bounds.Max += FVector(0.01f);
		return Bounds;
	}};
	TArray<TArray<int32>> Neighbors;
	TArray<TArray<int32>> SimTriangles;
	TArray<FVector> RestPositions;
	TArray<float> Values;
	TArray<float> Draft;
	TArray<int32> GradientStartPoints;
	TArray<int32> GradientEndPoints;
	TArray<FHistory> Histories;
	FHit Hover;
	FVector2 LastStamp;
	float AutoMin = 0.0f;
	float AutoMax = 1.0f;
	bool bStrokeActive = false;
	bool bHasLastStamp = false;
	bool bSurfaceDirty = true;

	TArray<FOverlayVertex> SurfaceVertices;
	TArray<FOverlayVertex> Markers;
	FVector PreviousViewSettings = FVector::ZeroVector;
	TUniquePtr<FVertexBuffer> SurfaceBuffer;
	TUniquePtr<FVertexBuffer> MarkerBuffer;
	TUniquePtr<FConstantBuffer> ViewBuffer;
	uint32 SurfaceCapacity = 0;
	uint32 MarkerCapacity = 0;
};
