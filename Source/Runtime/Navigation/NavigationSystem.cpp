#include "EnginePCH.h"
#include "Navigation/NavigationSystem.h"
#include "Navigation/RecastNavMesh.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/Level.h"
#include "Engine/World.h"
#include "GameFramework/Pawn.h"

#include <cmath>

namespace
{
	bool AppendStaticMesh(const UStaticMeshComponent& Component, TArray<float>& Vertices, TArray<int>& Triangles)
	{
		const FStaticMeshData* Mesh = Component.GetMeshData();
		if (!Mesh || Mesh->Indices.IsEmpty() || Mesh->Indices.Num() % 3 != 0 || Mesh->Vertices.IsEmpty())
			return false;

		const FMatrix World = Component.GetWorldMatrix();
		const float Determinant = World.Determinant();
		if (!std::isfinite(Determinant) || Determinant == 0.0f)
			return false;
		for (const uint32 Index : Mesh->Indices)
			if (Index >= static_cast<uint32>(Mesh->Vertices.Num()))
				return false;

		const int OldFloatCount = Vertices.Num();
		const int BaseVertex = OldFloatCount / 3;
		for (const FVertex& Vertex : Mesh->Vertices)
		{
			const FVector Position = World.TransformPosition(Vertex.Position);
			if (!std::isfinite(Position.X) || !std::isfinite(Position.Y) || !std::isfinite(Position.Z))
			{
				Vertices.SetNum(OldFloatCount);
				return false;
			}
			Vertices.Add(Position.X);
			Vertices.Add(Position.Z);
			Vertices.Add(Position.Y);
		}
		// Y/Z 교환은 면을 뒤집는다. 월드 변환에 미러가 있으면 두 반전이 상쇄된다.
		const bool bFlipWinding = Determinant > 0.0f;
		for (int Index = 0; Index < Mesh->Indices.Num(); Index += 3)
		{
			Triangles.Add(BaseVertex + static_cast<int>(Mesh->Indices[Index]));
			Triangles.Add(BaseVertex + static_cast<int>(Mesh->Indices[Index + (bFlipWinding ? 2 : 1)]));
			Triangles.Add(BaseVertex + static_cast<int>(Mesh->Indices[Index + (bFlipWinding ? 1 : 2)]));
		}
		return true;
	}
}

UNavigationSystemV1::UNavigationSystemV1()
	: NavMesh(MakeUnique<FRecastNavMesh>())
{
}

UNavigationSystemV1::~UNavigationSystemV1() = default;

UNavigationSystemV1* UNavigationSystemV1::GetCurrent(UWorld* World)
{
	return World ? World->GetNavigationSystem() : nullptr;
}

FNavigationBuildResult UNavigationSystemV1::Build(const FNavAgentProperties& Agent)
{
	Clear();
	const auto* World = Cast<UWorld>(GetOuter());
	if (!World)
	{
		FNavigationBuildResult Result;
		Result.FailedAt = "no navigation world";
		return Result;
	}
	TArray<float> Vertices;
	TArray<int> Triangles;
	int MeshComponentCount = 0;
	for (const ULevel* Level : World->GetLevels())
	{
		if (!Level)
			continue;
		for (const AActor* Actor : Level->GetActors())
		{
			if (!Actor || !Actor->GetActorEnableCollision() || Cast<APawn>(Actor) || World->IsPendingDestroy(Actor))
				continue;
			for (const auto* Component : Actor->GetComponents())
			{
				const auto* Mesh = Cast<UStaticMeshComponent>(Component);
				if (Mesh && Mesh->IsRegistered() && Mesh->CanEverAffectNavigation() &&
					!Mesh->IsEditorIcon() && !Mesh->IsSimulatingPhysics() && AppendStaticMesh(*Mesh, Vertices, Triangles))
					++MeshComponentCount;
			}
		}
	}
	FNavigationBuildResult Result = NavMesh->Build(Vertices.GetData(), Vertices.Num() / 3,
		Triangles.GetData(), Triangles.Num() / 3, Agent, true);
	Result.MeshComponentCount = MeshComponentCount;
	if (!Result.FailedAt && !NavMesh->GatherDebugGeometry(DebugData))
	{
		DebugData = {};
		KRS_LOG(Warning, "[NavMesh] Could not build navigation debug geometry.");
	}
	return Result;
}

FNavigationPathResult UNavigationSystemV1::FindPathToLocationSynchronously(const FVector& Start, const FVector& Goal)
{
	return NavMesh->FindPath(Start, Goal, &DebugData);
}

bool UNavigationSystemV1::IsNavigationBuilt() const
{
	return NavMesh->HasData();
}

void UNavigationSystemV1::Clear()
{
	NavMesh->Reset();
	DebugData = {};
}

bool UNavigationSystemV1::CopyBuiltDataFrom(const UNavigationSystemV1& Source)
{
	if (!NavMesh->CopyFrom(*Source.NavMesh))
	{
		DebugData = {};
		return false;
	}
	DebugData = {};
	DebugData.SurfaceTriangles = Source.DebugData.SurfaceTriangles;
	DebugData.BoundarySegments = Source.DebugData.BoundarySegments;
	return true;
}
