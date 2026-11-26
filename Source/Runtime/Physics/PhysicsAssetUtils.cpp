#include "EnginePCH.h"
#include "Physics/PhysicsAssetUtils.h"
#include "Physics/PhysicsAsset.h"
#include "Render/SkeletalMesh.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <vector>

namespace
{
	struct FBoneBox
	{
		FVector Min = FVector(FLT_MAX, FLT_MAX, FLT_MAX);
		FVector Max = FVector(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		int32 Count = 0;
		bool IsValid() const { return Count > 0 && Min.X <= Max.X; }
		void Add(const FVector& P)
		{
			for (int Axis = 0; Axis < 3; ++Axis)
			{
				Min[Axis] = std::min(Min[Axis], P[Axis]);
				Max[Axis] = std::max(Max[Axis], P[Axis]);
			}
			++Count;
		}
		// UE CalcBoneInfoLength와 같이 상자 반 대각선 길이(BoxExtent.Size())로 본 크기를 잰다.
		float HalfDiagonal() const { return IsValid() ? ((Max - Min) * 0.5f).Length() : 0.0f; }
	};

	// 본 하나에 모인 정점들 (본 공간). 상자는 크기 판정(MinBoneSize)에, 점들은 캡슐 맞춤에 쓴다.
	struct FBonePoints
	{
		std::vector<FVector> Points;
		FBoneBox Box;
		void Add(const FVector& P)
		{
			Points.push_back(P);
			Box.Add(P);
		}
	};

	// 형상을 축 방향 캡슐로 근사한 것 (관절 축·겹침 판정용): 축 방향(단위 벡터), 중심, 반지름, 반구를 포함한 반길이
	struct FFitCapsule
	{
		FVector Axis = FVector(0, 0, 1);
		FVector Center = FVector::ZeroVector;
		float Radius = 1.0f;
		float HalfHeight = 1.0f;
	};

	FVector AxisVector(int32 Axis)
	{
		FVector V = FVector::ZeroVector;
		V[Axis] = 1.0f;
		return V;
	}

	// 이 메시의 "뼈 길이 방향" 본 로컬 축 (UE 리그는 +X). 부모 → 자식 본 오프셋이 가장 많이 향하는 부호 있는 로컬 축을 다수결로 고른다.
	// 임포트 방식·리그에 따라 Y나 Z일 수 있어서 X로 고정하지 않는다. OutSecondary는 그다음 축(X→Y, Y→Z, Z→X).
	void FindBoneLengthAxis(const TArray<FBoneInfo>& Bones, FVector& OutTwist, FVector& OutSecondary)
	{
		int32 Votes[6] = {}; // +X, -X, +Y, -Y, +Z, -Z
		for (const FBoneInfo& Bone : Bones)
		{
			if (Bone.ParentIndex < 0)
				continue;
			const FVector& Offset = Bone.LocalBindPose.Location; // 부모 본 공간의 자식 원점
			if (Offset.SizeSquared() < 1e-6f)
				continue;
			int32 Axis = 0;
			for (int32 Candidate = 1; Candidate < 3; ++Candidate)
				if (std::abs(Offset[Candidate]) > std::abs(Offset[Axis]))
					Axis = Candidate;
			++Votes[Axis * 2 + (Offset[Axis] < 0.0f ? 1 : 0)];
		}
		const int32 Best = static_cast<int32>(std::max_element(Votes, Votes + 6) - Votes);
		const int32 Axis = Best / 2;
		OutTwist = AxisVector(Axis) * (Best % 2 ? -1.0f : 1.0f);
		OutSecondary = AxisVector((Axis + 1) % 3);
	}

	// 상자의 가장 긴 변 방향 (형상 측정 축)
	FVector LongestBoxAxis(const FBoneBox& Box)
	{
		const FVector Size = Box.Max - Box.Min;
		return AxisVector(Size.X >= Size.Y && Size.X >= Size.Z ? 0 : (Size.Y >= Size.Z ? 1 : 2));
	}

	// 로컬 Z(형상의 긴 축)를 Dir 방향으로 돌리는 회전
	FQuat RotationZTo(const FVector& Dir)
	{
		const FVector Z(0, 0, 1);
		const float Cos = std::clamp(Z.Dot(Dir), -1.0f, 1.0f);
		if (Cos > 0.9999f)
			return FQuat::Identity();
		if (Cos < -0.9999f)
			return FQuat::MakeFromAxisAngle(FVector(1, 0, 0), 3.14159265359f);
		return FQuat::MakeFromAxisAngle(Z.Cross(Dir), std::acos(Cos));
	}

	// ① 범위 재기 (형상과 무관). Rotation의 로컬 축 기준으로 잰 중심과 반 크기 (UE BoxCenter / BoxExtent).
	//    로컬 Z = 축 방향, 로컬 X·Y = 축에 수직인 두 방향.
	struct FBoneBounds
	{
		FQuat Rotation = FQuat::Identity();
		FVector Axis = FVector(0, 0, 1);
		FVector Center = FVector::ZeroVector;
		FVector HalfExtent = FVector::ZeroVector;
	};

	FBoneBounds MeasureBounds(const std::vector<FVector>& Points, const FVector& Dir)
	{
		FBoneBounds Bounds;
		Bounds.Rotation = RotationZTo(Dir);
		Bounds.Axis = Dir;
		const FVector LocalAxes[3] = {Bounds.Rotation.RotateVector(FVector(1, 0, 0)), Bounds.Rotation.RotateVector(FVector(0, 1, 0)), Dir};
		FVector Min(FLT_MAX, FLT_MAX, FLT_MAX), Max(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		for (const FVector& P : Points)
			for (int32 Axis = 0; Axis < 3; ++Axis)
			{
				const float D = P.Dot(LocalAxes[Axis]);
				Min[Axis] = std::min(Min[Axis], D);
				Max[Axis] = std::max(Max[Axis], D);
			}
		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			Bounds.Center += LocalAxes[Axis] * ((Min[Axis] + Max[Axis]) * 0.5f);
			Bounds.HalfExtent[Axis] = std::max((Max[Axis] - Min[Axis]) * 0.5f, 0.5f);
		}
		return Bounds;
	}

	// ② 형상 만들기. 각자 BodySetup에 형상을 넣고, 관절·겹침 판정용 캡슐 근사를 돌려준다.
	FFitCapsule AddCapsule(UBodySetup& Setup, const FBoneBounds& Bounds, const FName& Name)
	{
		// UE CreateCollisionFromBone(EFG_Sphyl): 반 크기가 가장 큰 축을 캡슐 축으로 삼는다.
		//   Radius = 나머지 두 반 크기 중 큰 값 × 1.01, Length(원기둥 부분) = 긴 축 반 크기 × 1.01
		const FVector& E = Bounds.HalfExtent;
		int32 LongAxis = 2;
		if (E.X > E.Z && E.X > E.Y)
			LongAxis = 0;
		else if (E.Y > E.Z && E.Y > E.X)
			LongAxis = 1;
		const FVector AxisDir = Bounds.Rotation.RotateVector(AxisVector(LongAxis)).GetSafeNormal();

		FFitCapsule Fit;
		Fit.Axis = AxisDir;
		Fit.Center = Bounds.Center;
		Fit.Radius = std::max(E[(LongAxis + 1) % 3], E[(LongAxis + 2) % 3]) * 1.01f;
		const float Length = E[LongAxis] * 1.01f;
		Fit.HalfHeight = Length * 0.5f + Fit.Radius;
		FKSphylElem Capsule;
		Capsule.Name = Name;
		Capsule.Center = Bounds.Center;
		Capsule.Rotation = (LongAxis == 2 ? Bounds.Rotation : RotationZTo(AxisDir)).ToFRotator();
		Capsule.Radius = Fit.Radius;
		Capsule.Length = Length;
		Setup.AggGeom.SphylElems.Add(Capsule);
		return Fit;
	}

	FFitCapsule AddBox(UBodySetup& Setup, const FBoneBounds& Bounds, const FName& Name)
	{
		FKBoxElem Box;
		Box.Name = Name;
		Box.Center = Bounds.Center;
		Box.Rotation = Bounds.Rotation.ToFRotator();
		Box.X = Bounds.HalfExtent.X * 2.0f;	// 변 길이
		Box.Y = Bounds.HalfExtent.Y * 2.0f;
		Box.Z = Bounds.HalfExtent.Z * 2.0f;
		Setup.AggGeom.BoxElems.Add(Box);
		FFitCapsule Fit;
		Fit.Axis = Bounds.Axis;
		Fit.Center = Bounds.Center;
		Fit.Radius = std::max(Bounds.HalfExtent.X, Bounds.HalfExtent.Y);
		Fit.HalfHeight = std::max(Bounds.HalfExtent.Z, Fit.Radius);
		return Fit;
	}

	FFitCapsule AddSphere(UBodySetup& Setup, const FBoneBounds& Bounds, const FName& Name)
	{
		FKSphereElem Sphere;
		Sphere.Name = Name;
		Sphere.Center = Bounds.Center;
		Sphere.Radius = std::max({Bounds.HalfExtent.X, Bounds.HalfExtent.Y, Bounds.HalfExtent.Z});	// UE: 반 크기 최대값
		Setup.AggGeom.SphereElems.Add(Sphere);
		FFitCapsule Fit;
		Fit.Axis = Bounds.Axis;
		Fit.Center = Bounds.Center;
		Fit.Radius = Sphere.Radius;
		Fit.HalfHeight = Sphere.Radius;
		return Fit;
	}

	// 두 선분 사이 최단 거리 (캡슐 겹침 판정)
	float SegmentDistance(const FVector& P1, const FVector& Q1, const FVector& P2, const FVector& Q2)
	{
		const FVector D1 = Q1 - P1, D2 = Q2 - P2, R = P1 - P2;
		const float A = D1.Dot(D1), E = D2.Dot(D2), F = D2.Dot(R);
		float S = 0.0f, T = 0.0f;
		if (A <= 1e-6f && E <= 1e-6f)
			return (P1 - P2).Size();
		if (A <= 1e-6f)
			T = std::clamp(F / E, 0.0f, 1.0f);
		else
		{
			const float C = D1.Dot(R);
			if (E <= 1e-6f)
				S = std::clamp(-C / A, 0.0f, 1.0f);
			else
			{
				const float B = D1.Dot(D2), Denom = A * E - B * B;
				S = Denom > 1e-6f ? std::clamp((B * F - C * E) / Denom, 0.0f, 1.0f) : 0.0f;
				T = (B * S + F) / E;
				if (T < 0.0f)
				{
					T = 0.0f;
					S = std::clamp(-C / A, 0.0f, 1.0f);
				}
				else if (T > 1.0f)
				{
					T = 1.0f;
					S = std::clamp((B - C) / A, 0.0f, 1.0f);
				}
			}
		}
		return ((P1 + D1 * S) - (P2 + D2 * T)).Size();
	}

	// 바인드 포즈의 본 → 컴포넌트 행렬 (행 벡터: Local * Parent)
	std::vector<FMatrix> ComputeBindCS(const TArray<FBoneInfo>& Bones)
	{
		std::vector<FMatrix> BindCS(Bones.Num());
		for (int32 Index = 0; Index < Bones.Num(); ++Index)
		{
			const FMatrix Local = Bones[Index].LocalBindPose.GetLocalMatrix();
			BindCS[Index] = Bones[Index].ParentIndex < 0 ? Local : Local * BindCS[Bones[Index].ParentIndex];
		}
		return BindCS;
	}

	std::vector<std::vector<int32>> BuildChildren(const TArray<FBoneInfo>& Bones)
	{
		std::vector<std::vector<int32>> Children(Bones.Num());
		for (int32 Index = 0; Index < Bones.Num(); ++Index)
			if (Bones[Index].ParentIndex >= 0)
				Children[Bones[Index].ParentIndex].push_back(Index);
		return Children;
	}

	// 형상 맞춤용 삼각형 캐시 (UE FSkinnedBoneTriangleCache, bAlwaysUseVertices).
	// 정점은 가중치가 최대인 본(동률이면 모두)에 영향을 준다. 삼각형은 영향 정점이 하나라도 있는 본마다 그 본의 소유가 된다.
	std::vector<std::vector<int32>> BuildTrianglesOfBone(const FSkeletalMeshLODRenderData& LOD, int32 NumBones)
	{
		std::vector<std::vector<int32>> TrianglesOfBone(NumBones);
		const int32 NumVerts = std::min(LOD.BindVertices.Num(), LOD.SkinWeights.Num());
		auto ForEachInfluencedBone = [&](uint32 V, auto&& Func)
		{
			if (static_cast<int32>(V) >= NumVerts)
				return;
			const FSkinWeight& Weight = LOD.SkinWeights[V];
			float MaxWeight = 0.0f;
			for (int32 K = 0; K < MaxBoneInfluences; ++K)
				MaxWeight = std::max(MaxWeight, Weight.Weight[K]);
			if (MaxWeight <= 0.0f)
				return;
			for (int32 K = 0; K < MaxBoneInfluences; ++K)
				if (Weight.Weight[K] >= MaxWeight && Weight.BoneIndex[K] < NumBones)
					Func(static_cast<int32>(Weight.BoneIndex[K]));
		};
		const int32 NumTriangles = LOD.Indices.Num() / 3;
		for (int32 Tri = 0; Tri < NumTriangles; ++Tri)
		{
			int32 Owners[3 * MaxBoneInfluences];
			int32 NumOwners = 0;
			for (int32 Corner = 0; Corner < 3; ++Corner)
				ForEachInfluencedBone(LOD.Indices[Tri * 3 + Corner], [&](int32 Bone)
				{
					if (std::find(Owners, Owners + NumOwners, Bone) == Owners + NumOwners)
						Owners[NumOwners++] = Bone;
				});
			for (int32 Owner = 0; Owner < NumOwners; ++Owner)
				TrianglesOfBone[Owners[Owner]].push_back(Tri);
		}
		return TrianglesOfBone;
	}

	// 바디 본과, 바디가 없는 자손 본들(바디 본에서 멈춤)의 삼각형 정점을 바디 본 공간으로 모은다 (UE bIncludeChildBones, GetRelevantBoneIndices).
	// 정점 중복은 본 하나 안에서만 없앤다 (UE와 같음). 비어 있을 수 있다.
	template <typename FHasBody>
	FBonePoints GatherTrianglePoints(const TArray<FBoneInfo>& Bones, const FSkeletalMeshLODRenderData& LOD,
	                                 const std::vector<std::vector<int32>>& TrianglesOfBone, const std::vector<std::vector<int32>>& Children,
	                                 int32 BodyBone, FHasBody&& HasBody)
	{
		FBonePoints Result;
		const FMatrix& ToBone = Bones[BodyBone].InverseBindPose;
		std::vector<int32> Stack{BodyBone};
		std::vector<bool> bSeen(LOD.BindVertices.Num(), false);
		while (!Stack.empty())
		{
			const int32 Current = Stack.back();
			Stack.pop_back();
			std::fill(bSeen.begin(), bSeen.end(), false);
			for (const int32 Tri : TrianglesOfBone[Current])
				for (int32 Corner = 0; Corner < 3; ++Corner)
				{
					const uint32 V = LOD.Indices[Tri * 3 + Corner];
					if (V >= bSeen.size() || bSeen[V])
						continue;
					bSeen[V] = true;
					Result.Add(ToBone.TransformPosition(LOD.BindVertices[V].Position));
				}
			for (const int32 Child : Children[Current])
				if (!HasBody(Child))
					Stack.push_back(Child);
		}
		return Result;
	}

	// ① 범위 재기 → ② 형상 만들기 (UE CreateCollisionFromBone과 같은 순서). 측정 축은 본 공간 정점 상자의 가장 긴 변.
	FFitCapsule FitShape(UBodySetup& Setup, const FBonePoints& FitPoints, EPhysAssetFitGeomType GeomType, const FName& ShapeName)
	{
		const FBoneBounds Bounds = MeasureBounds(FitPoints.Points, LongestBoxAxis(FitPoints.Box));
		switch (GeomType)
		{
		case EPhysAssetFitGeomType::EFG_Box:
			return AddBox(Setup, Bounds, ShapeName);
		case EPhysAssetFitGeomType::EFG_Sphere:
			return AddSphere(Setup, Bounds, ShapeName);
		default:
			return AddCapsule(Setup, Bounds, ShapeName);
		}
	}

	void ApplyDefaultBodySettings(USkeletalBodySetup& Setup)
	{
		// 질량은 형상 부피 × 밀도. 래그돌이 가벼워 튀지 않게 약간의 감쇠를 둔다.
		Setup.DefaultInstance.bOverrideMass = false;
		Setup.DefaultInstance.LinearDamping = 0.05f;
		Setup.DefaultInstance.AngularDamping = 0.1f;
		// 관절 사슬은 반복을 늘려야 덜 늘어난다 (UE 래그돌 권장값 수준).
		Setup.DefaultInstance.PositionSolverIterationCount = 16;
		Setup.DefaultInstance.VelocitySolverIterationCount = 4;
		Setup.MarkSaved();
	}

	// 관절은 자식 본 원점에 두고, 축은 자식 본 축을 그대로 쓴다: Twist = 뼈 길이 방향 축, Secondary = 그다음 축.
	// 부모 쪽 프레임은 같은 프레임을 부모 본 공간으로 옮긴 것이라 바인드 포즈에서 두 프레임이 겹친다 (UE SnapConstraintToBone).
	void InitConstraintSetup(FConstraintInstance& Joint, const FVector& Twist, const FVector& Secondary,
	                         const FMatrix& ChildBindCS, const FMatrix& ParentBindCS, const FPhysAssetCreateParams& Params)
	{
		Joint.Pos1 = FVector::ZeroVector;
		Joint.PriAxis1 = Twist;
		Joint.SecAxis1 = Secondary;
		// 같은 관절을 부모 바디 공간으로: 자식 본 → 부모 본 = BindCS[child] * BindCS[parent]⁻¹
		const FMatrix ChildToParent = ChildBindCS * ParentBindCS.Inverse();
		Joint.Pos2 = ChildToParent.TransformPosition(FVector::ZeroVector);
		Joint.PriAxis2 = ChildToParent.TransformVector(Twist).GetSafeNormal();
		Joint.SecAxis2 = ChildToParent.TransformVector(Secondary).GetSafeNormal();
		FConstraintProfileProperties& Profile = Joint.ProfileInstance;
		Profile.ConeLimit.Swing1Motion = Params.AngularConstraintMode;
		Profile.ConeLimit.Swing2Motion = Params.AngularConstraintMode;
		Profile.ConeLimit.Swing1LimitDegrees = Params.SwingLimitDegrees;
		Profile.ConeLimit.Swing2LimitDegrees = Params.SwingLimitDegrees;
		Profile.TwistLimit.TwistMotion = Params.AngularConstraintMode;
		Profile.TwistLimit.TwistLimitDegrees = Params.TwistLimitDegrees;
		Profile.bDisableCollision = true;
	}

	// 바디가 있는 가장 가까운 조상 본 (없으면 -1)
	int32 FindParentBodyBone(const UPhysicsAsset& Asset, const TArray<FBoneInfo>& Bones, int32 Bone)
	{
		int32 Parent = Bones[Bone].ParentIndex;
		while (Parent >= 0 && Asset.FindBodyIndex(FName(Bones[Parent].Name)) < 0)
			Parent = Bones[Parent].ParentIndex;
		return Parent;
	}

	int32 FindConstraintOfChild(const UPhysicsAsset& Asset, const FName& ChildBone)
	{
		for (int32 Index = 0; Index < Asset.ConstraintSetup.Num(); ++Index)
			if (Asset.ConstraintSetup[Index] && Asset.ConstraintSetup[Index]->DefaultInstance.ConstraintBone1 == ChildBone)
				return Index;
		return -1;
	}

	// 자식 본 Bone과 부모 본 ParentBone 사이 관절을 (없으면 만들고) 바인드 포즈에 맞춰 다시 잡는다.
	void SetupBoneConstraint(UPhysicsAsset& Asset, const TArray<FBoneInfo>& Bones, const std::vector<FMatrix>& BindCS,
	                         int32 Bone, int32 ParentBone, const FPhysAssetCreateParams& Params)
	{
		const FName ChildName(Bones[Bone].Name);
		const FName ParentName(Bones[ParentBone].Name);
		UPhysicsConstraintTemplate* Template = nullptr;
		const int32 Existing = FindConstraintOfChild(Asset, ChildName);
		if (Existing >= 0)
		{
			Template = Asset.ConstraintSetup[Existing];
			Template->DefaultInstance.ConstraintBone2 = ParentName;
		}
		else
			Template = Asset.AddConstraint(ChildName, ParentName);
		if (!Template)
			return;
		FVector Twist, Secondary;
		FindBoneLengthAxis(Bones, Twist, Secondary);
		InitConstraintSetup(Template->DefaultInstance, Twist, Secondary, BindCS[Bone], BindCS[ParentBone], Params);
	}
}

bool FPhysicsAssetUtils::CreateFromSkeletalMesh(UPhysicsAsset& PhysicsAsset, const USkeletalMesh& Mesh, const FPhysAssetCreateParams& Params)
{
	const TArray<FBoneInfo>& Bones = Mesh.GetRefSkeleton();
	const auto& LODs = Mesh.GetRenderData().LODRenderData;
	if (Bones.IsEmpty() || LODs.IsEmpty())
		return false;
	const FSkeletalMeshLODRenderData& LOD = LODs[0];
	const int32 NumBones = Bones.Num();

	// 1. 바인드 포즈의 본 → 컴포넌트 행렬
	const std::vector<FMatrix> BindCS = ComputeBindCS(Bones);

	// 2. 정점마다 가장 큰 가중치의 본에, 그 본 공간 위치로 모은다 (UE EVW_DominantWeight).
	std::vector<FBonePoints> Boxes(NumBones);
	for (int32 V = 0; V < LOD.BindVertices.Num() && V < LOD.SkinWeights.Num(); ++V)
	{
		const FSkinWeight& Weight = LOD.SkinWeights[V];
		int32 Dominant = -1;
		float Best = 0.0f;
		for (int32 K = 0; K < MaxBoneInfluences; ++K)
			if (Weight.Weight[K] > Best)
			{
				Best = Weight.Weight[K];
				Dominant = Weight.BoneIndex[K];
			}
		if (Dominant < 0 || Dominant >= NumBones)
			continue;
		Boxes[Dominant].Add(Bones[Dominant].InverseBindPose.TransformPosition(LOD.BindVertices[V].Position));
	}

	// 3. 작은 본은 부모에 합친다 (UE CreateFromSkeletalMeshInternal). 자식이 부모보다 뒤에 있으므로 뒤에서부터 처리한다.
	//    크기는 상자를 다시 재지 않고 자기 크기 + 합쳐진 자손 크기의 합(MergedSizes)으로 판정한다.
	//    점은 형상 맞춤용으로만 부모 본 공간으로 옮긴다. 합쳐진 본의 자기 점(OwnPoints)은 강제 루트 바디용으로 남겨 둔다.
	constexpr float MinWeldSize = 1.e-4f; // UE KINDA_SMALL_NUMBER
	std::vector<std::vector<FVector>> OwnPoints(NumBones);
	std::vector<float> MergedSizes(NumBones, 0.0f);
	for (int32 Index = 0; Index < NumBones; ++Index)
	{
		OwnPoints[Index] = Boxes[Index].Points;
		MergedSizes[Index] = Boxes[Index].Box.HalfDiagonal();
	}
	for (int32 Index = NumBones - 1; Index >= 0; --Index)
	{
		const float Size = MergedSizes[Index];
		const int32 Parent = Bones[Index].ParentIndex;
		if (Size >= Params.MinBoneSize || Size < MinWeldSize || Parent < 0)
			continue;
		MergedSizes[Parent] += Size;
		const FMatrix ToParent = Bones[Index].LocalBindPose.GetLocalMatrix();
		for (const FVector& P : Boxes[Index].Points)
			Boxes[Parent].Add(ToParent.TransformPosition(P));
		Boxes[Index] = FBonePoints();
		for (const FVector& P : OwnPoints[Index])
			Boxes[Index].Add(P);
	}

	// 루트 바디는 하나여야 한다: 맨 처음 바디의 부모를 둔 바디가 또 있으면 그 부모를 크기와 상관없이 바디로 만든다 (UE ForcedRootBoneIndex).
	int32 ForcedRoot = -1;
	int32 FirstParent = -1;
	for (int32 Index = 0; Index < NumBones; ++Index)
	{
		if (MergedSizes[Index] <= Params.MinBoneSize)
			continue;
		const int32 Parent = Bones[Index].ParentIndex;
		if (Parent < 0)
			break;
		if (FirstParent < 0)
			FirstParent = Parent;
		else if (Parent == FirstParent)
		{
			ForcedRoot = Parent;
			break;
		}
	}

	std::vector<bool> bHasBody(NumBones, false);
	for (int32 Index = 0; Index < NumBones; ++Index)
		bHasBody[Index] = (MergedSizes[Index] > Params.MinBoneSize || Index == ForcedRoot) && !Boxes[Index].Points.empty();

	const std::vector<std::vector<int32>> Children = BuildChildren(Bones);
	const std::vector<std::vector<int32>> TrianglesOfBone = BuildTrianglesOfBone(LOD, NumBones);
	// 아무 정점도 없으면 합쳐진 정점(Boxes)을 쓴다.
	auto GatherFitPoints = [&](int32 BodyBone)
	{
		FBonePoints Result = GatherTrianglePoints(Bones, LOD, TrianglesOfBone, Children, BodyBone, [&](int32 Bone) { return bHasBody[Bone]; });
		return Result.Points.empty() ? Boxes[BodyBone] : Result;
	};

	// 4. 바디
	PhysicsAsset.ClearAll();
	PhysicsAsset.PreviewMeshPath = Mesh.GetPath();
	std::vector<int32> BodyOfBone(NumBones, -1);
	std::vector<FFitCapsule> Fits;
	std::vector<int32> BoneOfBody;
	for (int32 Index = 0; Index < NumBones; ++Index)
	{
		if (!bHasBody[Index])
			continue;
		USkeletalBodySetup* Setup = PhysicsAsset.AddBody(FName(Bones[Index].Name));
		if (!Setup)
			continue;
		const FFitCapsule Fit = FitShape(*Setup, GatherFitPoints(Index), Params.GeomType, FName(Bones[Index].Name));
		ApplyDefaultBodySettings(*Setup);
		BodyOfBone[Index] = static_cast<int32>(BoneOfBody.size());
		BoneOfBody.push_back(Index);
		Fits.push_back(Fit);
	}
	if (BoneOfBody.empty())
		return false;

	// 5. 관절: 각 바디를 가장 가까운 조상 바디에 잇는다 (UE SnapTransformsToDefault).
	if (Params.bCreateConstraints)
	{
		FVector Twist, Secondary;
		FindBoneLengthAxis(Bones, Twist, Secondary);
		for (size_t Body = 0; Body < BoneOfBody.size(); ++Body)
		{
			const int32 Bone = BoneOfBody[Body];
			int32 ParentBone = Bones[Bone].ParentIndex;
			while (ParentBone >= 0 && BodyOfBone[ParentBone] < 0)
				ParentBone = Bones[ParentBone].ParentIndex;
			if (ParentBone < 0)
				continue;
			UPhysicsConstraintTemplate* Template = PhysicsAsset.AddConstraint(FName(Bones[Bone].Name), FName(Bones[ParentBone].Name));
			if (!Template)
				continue;
			InitConstraintSetup(Template->DefaultInstance, Twist, Secondary, BindCS[Bone], BindCS[ParentBone], Params);
		}
	}

	// 6. 바인드 포즈에서 겹친 캡슐 쌍은 충돌을 끈다 (관절로 이은 쌍은 관절이 이미 끈다).
	if (Params.bDisableCollisionsOfOverlappingBodies)
	{
		struct FSegment
		{
			FVector A, B;
			float Radius;
		};
		std::vector<FSegment> Segments;
		for (size_t Body = 0; Body < BoneOfBody.size(); ++Body)
		{
			const FFitCapsule& Fit = Fits[Body];
			const FVector Axis = Fit.Axis * (Fit.HalfHeight - Fit.Radius);
			const FMatrix& ToComponent = BindCS[BoneOfBody[Body]];
			Segments.push_back({ToComponent.TransformPosition(Fit.Center - Axis), ToComponent.TransformPosition(Fit.Center + Axis), Fit.Radius});
		}
		for (size_t I = 0; I < Segments.size(); ++I)
			for (size_t J = I + 1; J < Segments.size(); ++J)
				if (SegmentDistance(Segments[I].A, Segments[I].B, Segments[J].A, Segments[J].B) < Segments[I].Radius + Segments[J].Radius)
					PhysicsAsset.DisableCollision(static_cast<int32>(I), static_cast<int32>(J));
	}
	PhysicsAsset.UpdateBodySetupIndexMap();
	return true;
}

int32 FPhysicsAssetUtils::CreateNewBody(UPhysicsAsset& PhysicsAsset, const USkeletalMesh& Mesh, int32 BoneIndex, const FPhysAssetCreateParams& Params)
{
	const TArray<FBoneInfo>& Bones = Mesh.GetRefSkeleton();
	const auto& LODs = Mesh.GetRenderData().LODRenderData;
	if (BoneIndex < 0 || BoneIndex >= Bones.Num() || LODs.IsEmpty())
		return -1;
	const FSkeletalMeshLODRenderData& LOD = LODs[0];
	const std::vector<FMatrix> BindCS = ComputeBindCS(Bones);
	const FName BoneName(Bones[BoneIndex].Name);

	// 이 본과, 바디가 없는 자손 본들의 정점에 맞춘다 (UE MakeNewBody → CreateCollisionFromBone).
	const FBonePoints Points = GatherTrianglePoints(Bones, LOD, BuildTrianglesOfBone(LOD, Bones.Num()), BuildChildren(Bones), BoneIndex,
		[&](int32 Bone) { return PhysicsAsset.FindBodyIndex(FName(Bones[Bone].Name)) >= 0; });
	if (Points.Points.empty())
		return -1;

	USkeletalBodySetup* Setup = nullptr;
	int32 BodyIndex = PhysicsAsset.FindBodyIndex(BoneName);
	if (BodyIndex >= 0)
	{
		// 이미 있으면 형상만 새로 맞춘다 (UE: 기존 단순 충돌을 지우고 다시 만든다).
		Setup = PhysicsAsset.SkeletalBodySetups[BodyIndex];
		Setup->AggGeom.EmptyElements();
	}
	else
	{
		Setup = PhysicsAsset.AddBody(BoneName);
		if (!Setup)
			return -1;
		ApplyDefaultBodySettings(*Setup);
		BodyIndex = PhysicsAsset.FindBodyIndex(BoneName);
	}
	FitShape(*Setup, Points, Params.GeomType, BoneName);

	// 부모 바디로의 관절, 그리고 이제 이 바디가 가장 가까운 조상 바디가 된 자식 바디들의 관절을 맞춘다.
	if (Params.bCreateConstraints)
	{
		const int32 ParentBone = FindParentBodyBone(PhysicsAsset, Bones, BoneIndex);
		if (ParentBone >= 0)
			SetupBoneConstraint(PhysicsAsset, Bones, BindCS, BoneIndex, ParentBone, Params);
		for (USkeletalBodySetup* Other : PhysicsAsset.SkeletalBodySetups)
		{
			if (!Other)
				continue;
			const int32 OtherBone = Mesh.FindBoneIndex(Other->BoneName.ToString());
			if (OtherBone >= 0 && OtherBone != BoneIndex && FindParentBodyBone(PhysicsAsset, Bones, OtherBone) == BoneIndex)
				SetupBoneConstraint(PhysicsAsset, Bones, BindCS, OtherBone, BoneIndex, Params);
		}
	}
	PhysicsAsset.UpdateBodySetupIndexMap();
	return BodyIndex;
}

int32 FPhysicsAssetUtils::AddPrimitive(UPhysicsAsset& PhysicsAsset, const FName& BoneName, EPhysAssetFitGeomType GeomType)
{
	int32 BodyIndex = PhysicsAsset.FindBodyIndex(BoneName);
	if (BodyIndex < 0)
	{
		USkeletalBodySetup* Setup = PhysicsAsset.AddBody(BoneName);
		if (!Setup)
			return -1;
		ApplyDefaultBodySettings(*Setup);
		BodyIndex = PhysicsAsset.FindBodyIndex(BoneName);
	}
	// UE FPhysicsAssetEditor::AddNewPrimitive: 본 원점에 DefaultPrimSize(15) 기준 크기로 둔다.
	constexpr float DefaultPrimSize = 15.0f;
	FKAggregateGeom& Geom = PhysicsAsset.SkeletalBodySetups[BodyIndex]->AggGeom;
	switch (GeomType)
	{
	case EPhysAssetFitGeomType::EFG_Box:
	{
		FKBoxElem Box;
		Box.Name = BoneName;
		Box.X = Box.Y = Box.Z = 0.5f * DefaultPrimSize;
		Geom.BoxElems.Add(Box);
		break;
	}
	case EPhysAssetFitGeomType::EFG_Sphere:
	{
		FKSphereElem Sphere;
		Sphere.Name = BoneName;
		Sphere.Radius = DefaultPrimSize;
		Geom.SphereElems.Add(Sphere);
		break;
	}
	default:
	{
		FKSphylElem Capsule;
		Capsule.Name = BoneName;
		Capsule.Radius = DefaultPrimSize;
		Capsule.Length = DefaultPrimSize;
		Geom.SphylElems.Add(Capsule);
		break;
	}
	}
	return BodyIndex;
}

int32 FPhysicsAssetUtils::CreateNewConstraint(UPhysicsAsset& PhysicsAsset, const USkeletalMesh& Mesh, int32 ParentBodyIndex, int32 ChildBodyIndex, const FPhysAssetCreateParams& Params)
{
	const int32 NumBodies = PhysicsAsset.SkeletalBodySetups.Num();
	if (ParentBodyIndex < 0 || ChildBodyIndex < 0 || ParentBodyIndex >= NumBodies || ChildBodyIndex >= NumBodies || ParentBodyIndex == ChildBodyIndex)
		return -1;
	const FName ParentName = PhysicsAsset.SkeletalBodySetups[ParentBodyIndex]->BoneName;
	const FName ChildName = PhysicsAsset.SkeletalBodySetups[ChildBodyIndex]->BoneName;
	const int32 ParentBone = Mesh.FindBoneIndex(ParentName.ToString());
	const int32 ChildBone = Mesh.FindBoneIndex(ChildName.ToString());
	if (ParentBone < 0 || ChildBone < 0)
		return -1;
	// 같은 두 바디의 관절이 이미 있으면 그것을 돌려준다.
	const int32 Existing = PhysicsAsset.FindConstraintIndex(ChildName, ParentName);
	if (Existing >= 0)
		return Existing;
	UPhysicsConstraintTemplate* Template = PhysicsAsset.AddConstraint(ChildName, ParentName);
	if (!Template)
		return -1;

	// UE MakeUniqueNewConstraintName: UserConstraint, UserConstraint_0, UserConstraint_1, ...
	auto IsNameUsed = [&](const FName& Name)
	{
		for (const UPhysicsConstraintTemplate* Other : PhysicsAsset.ConstraintSetup)
			if (Other && Other != Template && Other->DefaultInstance.JointName == Name)
				return true;
		return false;
	};
	FName JointName("UserConstraint");
	for (int32 Suffix = 0; IsNameUsed(JointName); ++Suffix)
		JointName = FName(FString("UserConstraint_") + std::to_string(Suffix));
	Template->DefaultInstance.JointName = JointName;

	// UE InitConstraintSetup: 자식 본 원점에 맞추고(SnapConstraintToBone) 두 바디 충돌을 끈다.
	const TArray<FBoneInfo>& Bones = Mesh.GetRefSkeleton();
	const std::vector<FMatrix> BindCS = ComputeBindCS(Bones);
	FVector Twist, Secondary;
	FindBoneLengthAxis(Bones, Twist, Secondary);
	InitConstraintSetup(Template->DefaultInstance, Twist, Secondary, BindCS[ChildBone], BindCS[ParentBone], Params);
	return PhysicsAsset.ConstraintSetup.Num() - 1;
}
