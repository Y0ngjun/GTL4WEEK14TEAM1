#include "EnginePCH.h"
#include "Navigation/RecastNavMesh.h"
#include "Navigation/NavigationDebugData.h"

#include <Recast.h>
#include <DetourAlloc.h>
#include <DetourCommon.h>
#include <DetourNavMeshBuilder.h>
#include <DetourNavMeshQuery.h>
#include <cmath>
#include <cstring>

namespace
{
	constexpr float CellSize = 25.0f;
	constexpr float CellHeight = 10.0f;
	constexpr unsigned short WalkFlag = 1;
	constexpr int MaxGridCells = 4 * 1024 * 1024;
	constexpr int MaxSearchNodes = 2048;
	constexpr int MaxPathPolygons = 512;

	FVector ToWorldPosition(const float* Position)
	{
		return FVector(Position[0], Position[2], Position[1]);
	}

	bool IsFinite(const FVector& Position)
	{
		return std::isfinite(Position.X) && std::isfinite(Position.Y) && std::isfinite(Position.Z);
	}
}

FRecastNavMesh::FRecastNavMesh() = default;
FRecastNavMesh::~FRecastNavMesh() = default;

void FRecastNavMesh::Reset()
{
	NavMesh.reset();
	TileData.Empty();
}

bool FRecastNavMesh::LoadTile(const unsigned char* Data, int DataSize)
{
	auto* Tile = static_cast<unsigned char*>(dtAlloc(DataSize, DT_ALLOC_PERM));
	if (!Tile)
		return false;

	std::memcpy(Tile, Data, DataSize);
	auto NewMesh = MakeUnique<dtNavMesh>();
	if (dtStatusFailed(NewMesh->init(Tile, DataSize, DT_TILE_FREE_DATA)))
	{
		dtFree(Tile);
		return false;
	}
	NavMesh = std::move(NewMesh);
	return true;
}

bool FRecastNavMesh::CopyFrom(const FRecastNavMesh& Source)
{
	if (this == &Source)
		return true;

	Reset();
	if (!Source.HasData())
		return true;
	if (!LoadTile(Source.TileData.GetData(), Source.TileData.Num()))
		return false;

	TileData = Source.TileData;
	return true;
}

FNavigationBuildResult FRecastNavMesh::Build(const float* Vertices, int VertexCount, const int* Triangles,
	int TriangleCount, const FNavAgentProperties& Agent, bool bBuildHeightDetail)
{
	Reset();
	FNavigationBuildResult Result;
	Result.TriangleCount = TriangleCount;
	if (VertexCount < 3 || TriangleCount < 1)
	{
		Result.FailedAt = "no navigation geometry";
		return Result;
	}
	if (!std::isfinite(Agent.Height) || Agent.Height < CellHeight * 3 || Agent.Height >= RC_SPAN_MAX_HEIGHT * CellHeight ||
		!std::isfinite(Agent.Radius) || Agent.Radius < 0.0f || Agent.Radius > 127 * CellSize ||
		!std::isfinite(Agent.MaxStepHeight) || Agent.MaxStepHeight < 0.0f || Agent.MaxStepHeight >= RC_SPAN_MAX_HEIGHT * CellHeight ||
		!std::isfinite(Agent.MaxSlope) || Agent.MaxSlope < 0.0f || Agent.MaxSlope >= 90.0f)
	{
		Result.FailedAt = "invalid agent properties";
		return Result;
	}
	const int WalkableHeight = static_cast<int>(std::ceil(Agent.Height / CellHeight));
	const int WalkableClimb = static_cast<int>(std::floor(Agent.MaxStepHeight / CellHeight));
	const int WalkableRadius = static_cast<int>(std::ceil(Agent.Radius / CellSize));

	float BoundsMin[3];
	float BoundsMax[3];
	rcCalcBounds(Vertices, VertexCount, BoundsMin, BoundsMax);
	BoundsMax[1] += CellHeight;
	const float SizeX = (BoundsMax[0] - BoundsMin[0]) / CellSize;
	const float SizeZ = (BoundsMax[2] - BoundsMin[2]) / CellSize;
	const float Height = (BoundsMax[1] - BoundsMin[1]) / CellHeight;
	constexpr int MaxGridAxis = 4096;
	if (!std::isfinite(SizeX) || !std::isfinite(SizeZ) || !std::isfinite(Height) ||
		SizeX > MaxGridAxis || SizeZ > MaxGridAxis || Height >= RC_SPAN_MAX_HEIGHT)
	{
		Result.FailedAt = "navigation bounds too large for a single tile";
		return Result;
	}
	rcCalcGridSize(BoundsMin, BoundsMax, CellSize, &Result.GridSizeX, &Result.GridSizeZ);
	const int ColumnCount = Result.GridSizeX * Result.GridSizeZ;
	if (ColumnCount <= 0 || ColumnCount > MaxGridCells)
	{
		Result.FailedAt = "invalid navigation grid size";
		return Result;
	}

	rcContext Context(false);
	TArray<unsigned char> Areas;
	Areas.SetNum(TriangleCount);
	rcMarkWalkableTriangles(&Context, Agent.MaxSlope, Vertices, VertexCount, Triangles, TriangleCount, Areas.GetData());
	for (const auto Area : Areas)
		Result.WalkableTriangleCount += Area == RC_WALKABLE_AREA;

	rcHeightfield Heightfield;
	if (!rcCreateHeightfield(&Context, Heightfield, Result.GridSizeX, Result.GridSizeZ,
		BoundsMin, BoundsMax, CellSize, CellHeight))
	{
		Result.FailedAt = "rcCreateHeightfield";
		return Result;
	}
	if (!rcRasterizeTriangles(&Context, Vertices, VertexCount, Triangles, Areas.GetData(),
		TriangleCount, Heightfield, WalkableClimb))
	{
		Result.FailedAt = "rcRasterizeTriangles";
		return Result;
	}
	for (int Column = 0; Column < ColumnCount; ++Column)
	{
		for (const rcSpan* Span = Heightfield.spans[Column]; Span; Span = Span->next)
		{
			++Result.SpanCount;
			Result.WalkableSpanCount += Span->area == RC_WALKABLE_AREA;
		}
	}

	rcFilterLedgeSpans(&Context, WalkableHeight, WalkableClimb, Heightfield);
	rcFilterWalkableLowHeightSpans(&Context, WalkableHeight, Heightfield);
	rcCompactHeightfield Compact;
	if (!rcBuildCompactHeightfield(&Context, WalkableHeight, WalkableClimb, Heightfield, Compact))
	{
		Result.FailedAt = "rcBuildCompactHeightfield";
		return Result;
	}
	Result.FilteredWalkableSpanCount = Compact.spanCount;
	if (!rcErodeWalkableArea(&Context, WalkableRadius, Compact))
	{
		Result.FailedAt = "rcErodeWalkableArea";
		return Result;
	}
	for (int Index = 0; Index < Compact.spanCount; ++Index)
		Result.ErodedWalkableSpanCount += Compact.areas[Index] != RC_NULL_AREA;
	if (Result.ErodedWalkableSpanCount == 0)
	{
		Result.FailedAt = "no walkable area";
		return Result;
	}

	// 한 타일을 빌드하므로 타일 가장자리와 작은 영역 제거 임계값은 0이다.
	if (!rcBuildRegionsMonotone(&Context, Compact, 0, 0, 0))
	{
		Result.FailedAt = "rcBuildRegionsMonotone";
		return Result;
	}
	rcContourSet Contours;
	if (!rcBuildContours(&Context, Compact, 1.3f, 0, Contours))
	{
		Result.FailedAt = "rcBuildContours";
		return Result;
	}
	Result.ContourCount = Contours.nconts;

	rcPolyMesh Polygons;
	if (!rcBuildPolyMesh(&Context, Contours, DT_VERTS_PER_POLYGON, Polygons) || Polygons.npolys == 0)
	{
		Result.FailedAt = "rcBuildPolyMesh";
		return Result;
	}
	Result.PolygonCount = Polygons.npolys;
	Result.PolygonMeshVertexCount = Polygons.nverts;
	for (int Index = 0; Index < Polygons.npolys; ++Index)
		Polygons.flags[Index] = Polygons.areas[Index] != RC_NULL_AREA ? WalkFlag : 0;

	rcPolyMeshDetail Detail;
	dtNavMeshCreateParams Params{};
	if (bBuildHeightDetail)
	{
		if (!rcBuildPolyMeshDetail(&Context, Polygons, Compact, CellSize * 6.0f, CellHeight, Detail))
		{
			Result.FailedAt = "rcBuildPolyMeshDetail";
			return Result;
		}
		Params.detailMeshes = Detail.meshes;
		Params.detailVerts = Detail.verts;
		Params.detailVertsCount = Detail.nverts;
		Params.detailTris = Detail.tris;
		Params.detailTriCount = Detail.ntris;
	}
	Params.verts = Polygons.verts;
	Params.vertCount = Polygons.nverts;
	Params.polys = Polygons.polys;
	Params.polyAreas = Polygons.areas;
	Params.polyFlags = Polygons.flags;
	Params.polyCount = Polygons.npolys;
	Params.nvp = Polygons.nvp;
	dtVcopy(Params.bmin, Polygons.bmin);
	dtVcopy(Params.bmax, Polygons.bmax);
	Params.cs = Polygons.cs;
	Params.ch = Polygons.ch;
	Params.walkableHeight = Agent.Height;
	Params.walkableClimb = Agent.MaxStepHeight;
	Params.walkableRadius = Agent.Radius;
	Params.buildBvTree = true;

	unsigned char* Data = nullptr;
	int DataSize = 0;
	if (!dtCreateNavMeshData(&Params, &Data, &DataSize))
	{
		Result.FailedAt = "dtCreateNavMeshData";
		return Result;
	}
	const std::unique_ptr<unsigned char, decltype(&dtFree)> ScopedData(Data, dtFree);
	if (!LoadTile(Data, DataSize))
	{
		Result.FailedAt = "dtNavMesh::init";
		return Result;
	}
	// 초기 데이터는 보존하고, Detour가 연결을 작성하는 버퍼는 각 월드가 독립적으로 소유한다.
	TileData.SetNum(DataSize);
	std::memcpy(TileData.GetData(), Data, DataSize);
	return Result;
}

FNavigationPathResult FRecastNavMesh::FindPath(const FVector& WorldStart, const FVector& WorldGoal,
	FNavigationDebugData* DebugData) const
{
	if (DebugData)
	{
		DebugData->PathPoints.Reset();
		DebugData->RequestedGoal = WorldGoal;
		DebugData->bHasRequestedGoal = IsFinite(WorldGoal);
		DebugData->bHasProjectedGoal = false;
	}

	FNavigationPathResult Result;
	if (!NavMesh)
		return Result;

	Result.Status = ENavigationQueryResult::Error;
	if (!IsFinite(WorldStart) || !IsFinite(WorldGoal))
	{
		Result.FailedAt = "invalid path endpoints";
		return Result;
	}
	dtNavMeshQuery Query;
	if (dtStatusFailed(Query.init(NavMesh.get(), MaxSearchNodes)))
	{
		Result.FailedAt = "dtNavMeshQuery::init";
		return Result;
	}
	dtQueryFilter Filter;
	Filter.setIncludeFlags(WalkFlag);
	const float Start[] = {WorldStart.X, WorldStart.Z, WorldStart.Y};
	const float Goal[] = {WorldGoal.X, WorldGoal.Z, WorldGoal.Y};
	const float SearchHalfExtents[] = {75.0f, 100.0f, 75.0f};
	dtPolyRef StartRef = 0;
	dtPolyRef GoalRef = 0;
	float StartOnMesh[3];
	float GoalOnMesh[3];
	const dtStatus StartStatus = Query.findNearestPoly(Start, SearchHalfExtents, &Filter, &StartRef, StartOnMesh);
	const dtStatus GoalStatus = Query.findNearestPoly(Goal, SearchHalfExtents, &Filter, &GoalRef, GoalOnMesh);
	if (dtStatusFailed(StartStatus) || dtStatusFailed(GoalStatus))
	{
		Result.FailedAt = "findNearestPoly";
		return Result;
	}
	if (DebugData && GoalRef)
	{
		DebugData->ProjectedGoal = ToWorldPosition(GoalOnMesh);
		DebugData->bHasProjectedGoal = true;
	}
	if (!StartRef || !GoalRef)
	{
		Result.Status = ENavigationQueryResult::NoPath;
		Result.FailedAt = "findNearestPoly (no polygon in range)";
		return Result;
	}

	dtPolyRef Corridor[MaxPathPolygons];
	const dtStatus PathStatus = Query.findPath(StartRef, GoalRef, StartOnMesh, GoalOnMesh, &Filter,
		Corridor, &Result.CorridorPolygonCount, MaxPathPolygons);
	if (dtStatusFailed(PathStatus) || dtStatusDetail(PathStatus, DT_BUFFER_TOO_SMALL | DT_OUT_OF_NODES))
	{
		Result.FailedAt = "findPath (query failed)";
		return Result;
	}
	if (dtStatusDetail(PathStatus, DT_PARTIAL_RESULT) || Result.CorridorPolygonCount == 0 ||
		Corridor[Result.CorridorPolygonCount - 1] != GoalRef)
	{
		Result.Status = ENavigationQueryResult::NoPath;
		Result.FailedAt = "findPath (no complete corridor)";
		return Result;
	}

	float Points[MaxPathPolygons * 3];
	int PointCount = 0;
	const dtStatus StraightStatus = Query.findStraightPath(StartOnMesh, GoalOnMesh, Corridor,
		Result.CorridorPolygonCount, Points, nullptr, nullptr, &PointCount, MaxPathPolygons);
	if (dtStatusFailed(StraightStatus) || dtStatusDetail(StraightStatus, DT_PARTIAL_RESULT | DT_BUFFER_TOO_SMALL) ||
		PointCount == 0)
	{
		Result.FailedAt = "findStraightPath (no complete path)";
		return Result;
	}
	Result.PathPoints.Reserve(PointCount);
	for (int Index = 0; Index < PointCount; ++Index)
		Result.PathPoints.Add(ToWorldPosition(&Points[Index * 3]));
	if (DebugData)
		DebugData->PathPoints = Result.PathPoints;
	Result.Status = ENavigationQueryResult::Success;
	return Result;
}

bool FRecastNavMesh::GatherDebugGeometry(FNavigationDebugData& DebugData) const
{
	DebugData.SurfaceTriangles.Reset();
	DebugData.BoundarySegments.Reset();
	const dtNavMesh* Mesh = NavMesh.get();
	if (!Mesh)
		return false;
	const dtMeshTile* Tile = Mesh->getTile(0);
	if (!Tile || !Tile->header)
		return false;

	dtNavMeshQuery Query;
	if (dtStatusFailed(Query.init(Mesh, MaxSearchNodes)))
		return false;
	dtQueryFilter Filter;
	Filter.setIncludeFlags(WalkFlag);
	const dtPolyRef RefBase = Mesh->getPolyRefBase(Tile);
	for (int Index = 0; Index < Tile->header->polyCount; ++Index)
	{
		const dtPoly& Polygon = Tile->polys[Index];
		if (Polygon.getType() != DT_POLYTYPE_GROUND || (Polygon.flags & WalkFlag) == 0)
			continue;
		const dtPolyDetail& Detail = Tile->detailMeshes[Index];
		for (int TriangleIndex = 0; TriangleIndex < Detail.triCount; ++TriangleIndex)
		{
			const unsigned char* Triangle = &Tile->detailTris[(Detail.triBase + TriangleIndex) * 4];
			FVector WorldVertices[3];
			for (int Corner = 0; Corner < 3; ++Corner)
			{
				const int Vertex = Triangle[Corner];
				const float* Position = Vertex < Polygon.vertCount
					? &Tile->verts[Polygon.verts[Vertex] * 3]
					: &Tile->detailVerts[(Detail.vertBase + Vertex - Polygon.vertCount) * 3];
				WorldVertices[Corner] = ToWorldPosition(Position);
			}
			DebugData.SurfaceTriangles.Add(FNavigationDebugTriangle{WorldVertices[0], WorldVertices[2], WorldVertices[1]});
		}

		float Segments[DT_VERTS_PER_POLYGON * 6];
		int SegmentCount = 0;
		const dtStatus Status = Query.getPolyWallSegments(RefBase | Index, &Filter,
			Segments, nullptr, &SegmentCount, DT_VERTS_PER_POLYGON);
		if (dtStatusFailed(Status) || dtStatusDetail(Status, DT_BUFFER_TOO_SMALL))
			return false;
		for (int SegmentIndex = 0; SegmentIndex < SegmentCount; ++SegmentIndex)
		{
			const float* Segment = &Segments[SegmentIndex * 6];
			DebugData.BoundarySegments.Add(FNavigationDebugSegment{ToWorldPosition(Segment), ToWorldPosition(Segment + 3)});
		}
	}
	return true;
}
