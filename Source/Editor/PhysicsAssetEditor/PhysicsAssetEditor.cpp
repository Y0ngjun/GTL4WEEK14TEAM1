#include "EnginePCH.h"
#include "PhysicsAssetEditor.h"

#include <algorithm>
#include <cfloat>
#include <cmath>

#include "Asset/AssetManager.h"
#include "Asset/AssetUtils.h"
#include "Collision/Ray.h"
#include "Core/Misc/Paths.h"
#include "Editor/Viewports/EditorViewportClient.h"
#include "PhysicsAssetEditorTab.h"
#include "PhysicsBodyRenderer.h"
#include "Render/SceneView.h"
#include "Components/SkeletalMeshComponent.h"
#include "Components/SceneComponent.h"
#include "Components/BoxComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Render/Mesh.h"
#include "Engine/World.h"
#include "Physics/PhysicsAsset.h"
#include "Physics/BodyInstance.h"
#include "Physics/PhysScene.h"
#include "Physics/PhysicsMath.h"
#include <PxPhysicsAPI.h>
#include "Render/LineBatcher.h"
#include "Render/SkeletalMesh.h"
#include "Render/Texture2D.h"
#include "UObject/UObjectGlobals.h"
#include "imgui.h"

namespace
{
	// 본 월드 행렬에서 스케일을 뺀 것 = 바디 프레임 (실행 바디와 같은 규칙). 행 i = 로컬 i축.
	FMatrix BoneFrameWithoutScale(const USkeletalMeshComponent& Component, int32 BoneIndex)
	{
		FMatrix Frame = Component.GetBoneWorldMatrix(BoneIndex);
		for (int32 Row = 0; Row < 3; ++Row)
		{
			const FVector Axis = FVector(Frame.M[Row][0], Frame.M[Row][1], Frame.M[Row][2]).GetSafeNormal();
			Frame.M[Row][0] = Axis.X;
			Frame.M[Row][1] = Axis.Y;
			Frame.M[Row][2] = Axis.Z;
		}
		return Frame;
	}

	int32 PrimCount(const FKAggregateGeom& Geom, EPhysPrimType Type)
	{
		switch (Type)
		{
		case EPhysPrimType::Sphere: return Geom.SphereElems.Num();
		case EPhysPrimType::Sphyl: return Geom.SphylElems.Num();
		case EPhysPrimType::Box: return Geom.BoxElems.Num();
		case EPhysPrimType::Convex: return Geom.ConvexElems.Num();
		default: return 0;
		}
	}

	// (Type, Index)가 유효하면 그대로 두고, 아니면 첫 형상(구 → 캡슐 → 박스 → 컨벡스)으로 바꾼다. 형상이 하나도 없으면 false.
	bool ResolvePrim(const FKAggregateGeom& Geom, EPhysPrimType& Type, int32& Index)
	{
		if (Index >= 0 && Index < PrimCount(Geom, Type))
			return true;
		for (const EPhysPrimType Candidate : {EPhysPrimType::Sphere, EPhysPrimType::Sphyl, EPhysPrimType::Box, EPhysPrimType::Convex})
			if (PrimCount(Geom, Candidate) > 0)
			{
				Type = Candidate;
				Index = 0;
				return true;
			}
		Type = EPhysPrimType::None;
		Index = -1;
		return false;
	}

	FVector FrameOrigin(const FMatrix& Frame)
	{
		return FVector(Frame.M[3][0], Frame.M[3][1], Frame.M[3][2]);
	}

	FVector FrameAxis(const FMatrix& Frame, int32 Row)
	{
		return FVector(Frame.M[Row][0], Frame.M[Row][1], Frame.M[Row][2]).GetSafeNormal();
	}

	// 관절의 두 프레임을 월드로: Frame1 = 자식 본(Bone1) 쪽, Frame2 = 부모 본(Bone2) 쪽. 본을 못 찾으면 false.
	bool GetConstraintWorldFrames(const USkeletalMeshComponent& Component, const FConstraintInstance& Constraint, FMatrix& OutFrame1, FMatrix& OutFrame2)
	{
		const int32 ChildBone = Component.FindBoneIndex(Constraint.ConstraintBone1.ToString());
		const int32 ParentBone = Component.FindBoneIndex(Constraint.ConstraintBone2.ToString());
		if (ChildBone < 0 || ParentBone < 0)
			return false;
		const float Scale = Component.GetWorldScale3D().X;
		FTransform Ref1 = Constraint.GetRefFrame(EConstraintFrame::Frame1);
		FTransform Ref2 = Constraint.GetRefFrame(EConstraintFrame::Frame2);
		Ref1.Location = Ref1.Location * Scale;	// 실행 관절도 위치에만 컴포넌트 스케일을 곱한다
		Ref2.Location = Ref2.Location * Scale;
		OutFrame1 = Ref1.GetLocalMatrix() * BoneFrameWithoutScale(Component, ChildBone);
		OutFrame2 = Ref2.GetLocalMatrix() * BoneFrameWithoutScale(Component, ParentBone);
		return true;
	}

	// 관절 각도 제한 그리기 (UE FConstraintInstance::DrawConstraintImp)
	constexpr float SelectedConstraintSize = 20.0f;	// 선택한 관절 축 길이와 같다
	constexpr float ConstraintLimitSize = 5.0f;			// 선택 안 한 관절의 원뿔·부채꼴 반지름
	constexpr float SelectedConstraintLimitSize = 18.0f;	// 선택한 관절 (축 길이 20보다 조금 짧게)
	constexpr int32 ConeLimitSides = 32;
	constexpr float Pi = 3.14159265359f;

	// UE HelpBuildAngle: Free = 180°, Locked = 0, Limited = 제한 각도
	float LimitAngle(EAngularConstraintMotion Motion, float LimitDegrees)
	{
		if (Motion == ACM_Free)
			return Pi;
		if (Motion == ACM_Locked)
			return 0.0f;
		return LimitDegrees * Pi / 180.0f;
	}

	// UE CalcConeVert: X축 둘레의 타원 원뿔 위 단위 방향. Angle1은 Y 쪽, Angle2는 Z 쪽으로 벌어지는 반각.
	FVector CalcConeVert(float Angle1, float Angle2, float Azimuth)
	{
		const float A1 = std::clamp(Angle1, 0.01f, Pi - 0.01f);
		const float A2 = std::clamp(Angle2, 0.01f, Pi - 0.01f);
		const float SinX = std::sin(0.5f * A1), SinY = std::sin(0.5f * A2);
		const float Phi = std::atan2(std::sin(Azimuth) * SinY, std::cos(Azimuth) * SinX);
		const float SinPhi = std::sin(Phi), CosPhi = std::cos(Phi);
		const float RSq = SinX * SinX * SinY * SinY / (SinX * SinX * SinPhi * SinPhi + SinY * SinY * CosPhi * CosPhi);
		const float R = std::sqrt(RSq), Sqr = std::sqrt(1.0f - RSq);
		return FVector(1.0f - 2.0f * RSq, 2.0f * Sqr * R * CosPhi, 2.0f * Sqr * R * SinPhi);
	}

	// 꼭짓점 Origin, 축 X(Y·Z는 각도 방향)인 원뿔 옆면 삼각형 (UE DrawCone)
	void AddConeTriangles(TArray<FVector>& Out, const FVector& Origin, const FVector& X, const FVector& Y, const FVector& Z,
	                      float Angle1, float Angle2, float Length)
	{
		FVector Prev;
		for (int32 Side = 0; Side <= ConeLimitSides; ++Side)
		{
			const FVector V = CalcConeVert(Angle1, Angle2, 2.0f * Pi * Side / ConeLimitSides);
			const FVector P = Origin + (X * V.X + Y * V.Y + Z * V.Z) * Length;
			if (Side > 0)
			{
				Out.Add(Origin);
				Out.Add(Prev);
				Out.Add(P);
			}
			Prev = P;
		}
	}

	// 광선(Origin + Direction * t, Direction 단위벡터)과 형상의 첫 교차 거리 t. 안 맞으면 음수.
	float RaySphere(const FVector& Origin, const FVector& Direction, const FVector& Center, float Radius)
	{
		const FVector ToOrigin = Origin - Center;
		const float B = ToOrigin.Dot(Direction);
		const float C = ToOrigin.Dot(ToOrigin) - Radius * Radius;
		const float Discriminant = B * B - C;
		if (Discriminant < 0.0f)
			return -1.0f;
		const float Root = std::sqrt(Discriminant);
		const float Near = -B - Root;
		return Near >= 0.0f ? Near : (-B + Root >= 0.0f ? 0.0f : -1.0f);	// 안에서 쏘면 0
	}

	// 로컬 공간의 축 정렬 박스 (-HalfExtent ~ HalfExtent). 슬랩 방식.
	float RayBox(const FVector& Origin, const FVector& Direction, const FVector& HalfExtent)
	{
		float Enter = 0.0f;
		float Exit = FLT_MAX;
		const float O[3] = {Origin.X, Origin.Y, Origin.Z};
		const float D[3] = {Direction.X, Direction.Y, Direction.Z};
		const float H[3] = {HalfExtent.X, HalfExtent.Y, HalfExtent.Z};
		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			if (std::abs(D[Axis]) < 1e-8f)
			{
				if (O[Axis] < -H[Axis] || O[Axis] > H[Axis])
					return -1.0f;
				continue;
			}
			float T0 = (-H[Axis] - O[Axis]) / D[Axis];
			float T1 = (H[Axis] - O[Axis]) / D[Axis];
			if (T0 > T1)
				std::swap(T0, T1);
			Enter = std::max(Enter, T0);
			Exit = std::min(Exit, T1);
			if (Enter > Exit)
				return -1.0f;
		}
		return Enter;
	}

	// 로컬 공간의 캡슐: 축 = Z, 원기둥 반길이 HalfLength, 반지름 Radius. 원기둥 옆면 + 양 끝 반구.
	float RayCapsule(const FVector& Origin, const FVector& Direction, float HalfLength, float Radius)
	{
		float Best = -1.0f;
		const auto Consider = [&Best](float T)
		{
			if (T >= 0.0f && (Best < 0.0f || T < Best))
				Best = T;
		};

		const float A = Direction.X * Direction.X + Direction.Y * Direction.Y;
		if (A > 1e-8f)
		{
			const float B = Origin.X * Direction.X + Origin.Y * Direction.Y;
			const float C = Origin.X * Origin.X + Origin.Y * Origin.Y - Radius * Radius;
			const float Discriminant = B * B - A * C;
			if (Discriminant >= 0.0f)
			{
				const float T = (-B - std::sqrt(Discriminant)) / A;
				const float Z = Origin.Z + Direction.Z * T;
				if (Z >= -HalfLength && Z <= HalfLength)
					Consider(C <= 0.0f && std::abs(Origin.Z) <= HalfLength ? 0.0f : T);
			}
		}
		Consider(RaySphere(Origin, Direction, FVector(0.0f, 0.0f, HalfLength), Radius));
		Consider(RaySphere(Origin, Direction, FVector(0.0f, 0.0f, -HalfLength), Radius));
		return Best;
	}

	// 관절 프레임의 세 축 (X 빨강 = Twist축, Y 초록, Z 파랑). PIE 관절 디버그와 같은 색.
	void DrawConstraintFrame(FLineBatcher& Lines, const FMatrix& Frame, float Length)
	{
		static const FVector4 AxisColors[3] = {{1.0f, 0.15f, 0.15f, 1.0f}, {0.2f, 1.0f, 0.2f, 1.0f}, {0.25f, 0.45f, 1.0f, 1.0f}};
		const FVector Origin = FrameOrigin(Frame);
		for (int32 Row = 0; Row < 3; ++Row)
		{
			const FVector Axis = FVector(Frame.M[Row][0], Frame.M[Row][1], Frame.M[Row][2]).GetSafeNormal();
			Lines.AddLine(Origin, Origin + Axis * Length, AxisColors[Row]);
		}
	}
}

FPhysicsAssetEditor::FPhysicsAssetEditor(UEditorEngine& Engine, FEditorManager& Manager, const FString& Path)
	: FSkeletalPreviewEditor(Engine, Manager, Path)
{
}

FPhysicsAssetEditor::~FPhysicsAssetEditor()
{
	delete PhysicsAsset;
	PhysicsAsset = nullptr;
}

bool FPhysicsAssetEditor::IsPhysicsAssetPath(const FString& Path)
{
	FString Extension = FPaths::FromUtf8(Path).extension().string();
	std::transform(Extension.begin(), Extension.end(), Extension.begin(), [](unsigned char C) { return static_cast<char>(std::tolower(C)); });
	return Extension == ".physicsasset";
}

FString FPhysicsAssetEditor::ResolvePreviewMeshPath() const
{
	if (PhysicsAsset && !PhysicsAsset->PreviewMeshPath.empty())
		return PhysicsAsset->PreviewMeshPath;

	// 과도기 경로 Content/Physics/PhysicsAssets/<메시 경로>.physicsasset 이면 메시 경로를 되찾을 수 있다.
	const FString FilePath = FAssetUtils::ResolveAssetFilePath(GetAssetPath());
	if (FilePath.empty())
		return {};
	const std::filesystem::path File = FPaths::FromUtf8(FilePath);
	const std::filesystem::path Root = FPaths::ProjectContentDir() / "Physics" / "PhysicsAssets";
	std::error_code Error;
	std::filesystem::path Relative = std::filesystem::relative(File, Root, Error);
	if (Error || Relative.empty() || *Relative.begin() == "..")
		return {};
	Relative.replace_extension();	// <메시 경로>.physicsasset → <메시 경로>
	return FPaths::ToUtf8(std::filesystem::path(Relative.generic_wstring()));
}

bool FPhysicsAssetEditor::Save()
{
	if (!PhysicsAsset)
		return false;
	BodyGizmo.EndDrag();
	const FString FilePath = FAssetUtils::ResolveAssetFilePath(GetAssetPath());
	const std::filesystem::path File = FPaths::FromUtf8(FilePath);
	if (FilePath.empty() || !PhysicsAsset->SaveToFile(File))
	{
		KRS_LOG(Error, "[Physics] Failed to save physics asset: {}", GetAssetPath());
		return false;
	}
	KRS_LOG(Info, "[Physics] Physics asset saved: {}", GetAssetPath());

	// 에디터는 파일에서 따로 읽은 사본을 고친다. 저장한 파일이 메시에 연결된 에셋(과도기 경로)이면
	// 메시가 들고 있는 에셋도 파일에서 다시 읽어, 게임에서 쓰는 래그돌이 바로 새 값을 쓰게 한다.
	if (Mesh && FPaths::HasProject() && !Mesh->GetPath().empty())
	{
		std::filesystem::path Assigned = FPaths::ProjectContentDir() / "Physics" / "PhysicsAssets" / FPaths::FromUtf8(Mesh->GetPath());
		Assigned += ".physicsasset";
		std::error_code Error;
		if (std::filesystem::equivalent(File, Assigned, Error) && !Error)
			Mesh->RevertPhysicsAsset();
	}
	return true;
}

bool FPhysicsAssetEditor::InitializeResources()
{
	// 1. 물리 에셋 읽기
	const FString FilePath = FAssetUtils::ResolveAssetFilePath(GetAssetPath());
	PhysicsAsset = NewObject<UPhysicsAsset>();
	if (!PhysicsAsset || FilePath.empty() || !PhysicsAsset->LoadFromFile(FPaths::FromUtf8(FilePath)))
	{
		KRS_LOG(Warning, "[Physics] Failed to open physics asset: {}", GetAssetPath());
		return false;
	}

	// 2. 미리보기 메시 읽기
	PreviewMeshPath = ResolvePreviewMeshPath();
	if (PreviewMeshPath.empty())
	{
		KRS_LOG(Warning, "[Physics] Physics asset has no preview mesh. Recreate it from a skeletal mesh: {}", GetAssetPath());
		return false;
	}
	const FString MeshFilePath = FAssetUtils::ResolveAssetFilePath(PreviewMeshPath);
	FString Error;
	Mesh = UAssetManager::LoadSkeletalMesh(MeshFilePath.empty() ? PreviewMeshPath : MeshFilePath, Error);
	if (!Mesh)
	{
		KRS_LOG(Warning, "[Physics] Physics asset preview mesh failed to load: {} ({})", PreviewMeshPath, Error);
		return false;
	}

	// 3. 프리뷰 씬에 메시 컴포넌트를 띄운다
	if (!InitializePreview())
		return false;

	// 4. 이 에디터의 주인공은 바디이므로 본은 기본으로 끈다 (뷰포트 메뉴에서 다시 켤 수 있다).
	SetBoneDrawMode(EBoneDrawMode::None);

	// 5. 바디 기즈모가 따라갈 빈 컴포넌트. 축은 바디 형상 기준(Local)으로 보여 준다.
	BodyGizmoTarget = Cast<USceneComponent>(PreviewScene.AddComponent(USceneComponent::StaticClass()));
	if (!BodyGizmoTarget)
		return false;
	BodyGizmoTarget->DetachFromParent();
	BodyGizmo.SetSpace(EGizmoSpace::Local);
	BodyGizmo.SetScreenScale(0.6f);	// 바디는 작으므로 레벨 기즈모보다 작게

	// 6. 미리보기 물리 (UE처럼 항상 켜 두고 바디는 키네마틱으로 둔다)
	InitializePreviewPhysics();
	return true;
}

void FPhysicsAssetEditor::OnRenderOverlay(const FSceneView& View)
{
	if (!PhysicsAsset || !MeshComponent)
		return;
	if (!BodyRenderer)
		BodyRenderer = MakeUnique<FPhysicsBodyRenderer>();
	if (!BodyRenderer->Begin(View.ViewProjectionMatrix, View.ViewLocation))
		return;

	// 선과 같은 색, 면은 옅게. 무언가 선택돼 있으면 나머지는 더 옅게. 시뮬레이션 중 잡은 바디는 선택처럼 강조한다.
	const int32 HighlightBodyIndex = SimGrab.BodyIndex >= 0 ? SimGrab.BodyIndex : SelectedBodyIndex;
	const bool bAnySelected = HighlightBodyIndex >= 0 || SelectedConstraintIndex >= 0;
	const FVector4 BodyColor = bAnySelected ? FVector4(0.3f, 0.8f, 1.0f, 0.12f) : FVector4(0.3f, 0.8f, 1.0f, 0.3f);
	const FVector4 SelectedBodyColor(1.0f, 0.85f, 0.2f, 0.45f);
	const FVector Scale = MeshComponent->GetWorldScale3D();
	for (int32 BodyIndex = 0; BodyIndex < PhysicsAsset->SkeletalBodySetups.Num(); ++BodyIndex)
	{
		const USkeletalBodySetup* Setup = PhysicsAsset->SkeletalBodySetups[BodyIndex];
		if (!Setup)
			continue;
		const int32 BoneIndex = MeshComponent->FindBoneIndex(Setup->BoneName.ToString());
		if (BoneIndex < 0)
			continue;
		BodyRenderer->DrawBody(*Setup, BoneFrameWithoutScale(*MeshComponent, BoneIndex), Scale,
		                       BodyIndex == HighlightBodyIndex ? SelectedBodyColor : BodyColor);
	}

	// 드래그 궤적: 손 목표 위치에 찍은 빨간 점이 GrabTrailLifetime 동안 옅어지며 남는다 (UE SimMouseMove의 DrawDebugPoint).
	for (const FGrabTrailPoint& Point : GrabTrail)
	{
		const float Fade = 1.0f - std::clamp(Point.Age / GrabTrailLifetime, 0.0f, 1.0f);
		BodyRenderer->DrawSphere(Point.Location, GrabTrailPointRadius, FVector4(1.0f, 0.1f, 0.1f, 0.8f * Fade));
	}

	// 관절 각도 제한 (UE와 같이 부모 쪽 Frame2 기준). 모든 관절을 작게, 선택한 관절은 크고 진하게 그린다.
	//   빨강 원뿔 = Swing1(Y 쪽)·Swing2(Z 쪽) 제한: 자식 X축(뼈)이 꺾일 수 있는 범위
	//   초록 부채꼴 = Twist 제한: X축을 중심으로 Y축이 돌 수 있는 범위 (YZ 평면)
	for (int32 ConstraintIndex = 0; ConstraintIndex < PhysicsAsset->ConstraintSetup.Num(); ++ConstraintIndex)
	{
		const UPhysicsConstraintTemplate* Template = PhysicsAsset->ConstraintSetup[ConstraintIndex];
		FMatrix Frame1, Frame2;
		if (Template && GetConstraintWorldFrames(*MeshComponent, Template->DefaultInstance, Frame1, Frame2))
		{
			const bool bSelected = ConstraintIndex == SelectedConstraintIndex;
			const FConstraintProfileProperties& Profile = Template->DefaultInstance.ProfileInstance;
			const FVector Origin = FrameOrigin(Frame2);
			const FVector X = FrameAxis(Frame2, 0), Y = FrameAxis(Frame2, 1), Z = FrameAxis(Frame2, 2);
			const float Length = bSelected ? SelectedConstraintLimitSize : ConstraintLimitSize;
			const float Alpha = bSelected ? 0.4f : 0.3f;
			const FVector4 SwingColor(1.0f, 0.2f, 0.2f, Alpha);
			const FVector4 TwistColor(0.2f, 1.0f, 0.2f, Alpha);

			const FConeConstraint& Cone = Profile.ConeLimit;
			if (!(Cone.Swing1Motion == ACM_Locked && Cone.Swing2Motion == ACM_Locked))
			{
				if (Cone.Swing1Motion == ACM_Free && Cone.Swing2Motion == ACM_Free)
					BodyRenderer->DrawSphere(Origin, Length, SwingColor);
				else
				{
					TArray<FVector> Triangles;
					AddConeTriangles(Triangles, Origin, X, Y, Z,
					                 LimitAngle(Cone.Swing1Motion, Cone.Swing1LimitDegrees), LimitAngle(Cone.Swing2Motion, Cone.Swing2LimitDegrees), Length);
					BodyRenderer->DrawTriangles(Triangles, SwingColor);
				}
			}

			const FTwistConstraint& Twist = Profile.TwistLimit;
			if (Twist.TwistMotion != ACM_Locked)
			{
				// UE: 축을 (Y, Z, X)로 바꾼 원뿔에 Angle2 = 0 → Y를 가운데로 ±Twist만큼 벌어진 평평한 부채꼴
				TArray<FVector> Triangles;
				AddConeTriangles(Triangles, Origin, Y, Z, X, LimitAngle(Twist.TwistMotion, Twist.TwistLimitDegrees), 0.0f, Length);
				BodyRenderer->DrawTriangles(Triangles, TwistColor);
			}
		}
	}
}

void FPhysicsAssetEditor::OnRenderLine(FLineBatcher& LineBatcher)
{
	FSkeletalPreviewEditor::OnRenderLine(LineBatcher);
	if (!PhysicsAsset || !MeshComponent)
		return;

	// 바디: 스케일을 뺀 본 월드 행렬 위에 형상을 그린다. 형상 크기는 컴포넌트 스케일로 키운다 (실행 바디와 같은 규칙).
	const FVector Scale = MeshComponent->GetWorldScale3D();
	// 무언가 선택돼 있으면 나머지는 흐리게, 선택한 바디(시뮬레이션 중에는 잡은 바디)는 노랗게 그린다.
	const int32 HighlightBodyIndex = SimGrab.BodyIndex >= 0 ? SimGrab.BodyIndex : SelectedBodyIndex;
	const bool bAnySelected = HighlightBodyIndex >= 0 || SelectedConstraintIndex >= 0;
	const FVector4 BodyColor = bAnySelected ? FVector4(0.2f, 0.45f, 0.6f, 1.0f) : FVector4(0.3f, 0.8f, 1.0f, 1.0f);
	const FVector4 SelectedBodyColor(1.0f, 0.85f, 0.2f, 1.0f);
	for (int32 BodyIndex = 0; BodyIndex < PhysicsAsset->SkeletalBodySetups.Num(); ++BodyIndex)
	{
		USkeletalBodySetup* Setup = PhysicsAsset->SkeletalBodySetups[BodyIndex];
		if (!Setup)
			continue;
		const int32 BoneIndex = MeshComponent->FindBoneIndex(Setup->BoneName.ToString());
		if (BoneIndex < 0)
			continue;

		Setup->DrawDebug(LineBatcher, BoneFrameWithoutScale(*MeshComponent, BoneIndex), Scale,
		                 BodyIndex == HighlightBodyIndex ? SelectedBodyColor : BodyColor);
	}

	// 잡는 중: 손(마우스 목표)에서 바디의 잡은 지점까지 고무줄을 빨간 선으로 그린다.
	if (SimGrab.Joint && SimGrab.Handle)
	{
		physx::PxRigidActor* Actor0 = nullptr;
		physx::PxRigidActor* Actor1 = nullptr;
		SimGrab.Joint->getActors(Actor0, Actor1);
		if (Actor1)
		{
			const physx::PxVec3 HandPoint = SimGrab.Handle->getGlobalPose().p;
			const physx::PxVec3 BodyPoint = Actor1->getGlobalPose().transform(SimGrab.Joint->getLocalPose(physx::PxJointActorIndex::eACTOR1).p);
			LineBatcher.AddLine(PhysicsMath::FromPx(HandPoint), PhysicsMath::FromPx(BodyPoint), FVector4(1.0f, 0.1f, 0.1f, 1.0f));
		}
	}

	// 관절: 각 본 공간에 저장된 관절 프레임(GetRefFrame)을 본 월드 프레임에 얹는다.
	// Frame1 = 자식 본(Bone1) 쪽, Frame2 = 부모 본(Bone2) 쪽. 정상이면 두 프레임이 같은 자리에 겹친다.
	const FVector4 ConstraintColor(0.0f, 0.0f, 0.0f, 1.0f);	// 관절 표시(연결선·선택 구)는 검은색으로 통일
	for (int32 ConstraintIndex = 0; ConstraintIndex < PhysicsAsset->ConstraintSetup.Num(); ++ConstraintIndex)
	{
		const UPhysicsConstraintTemplate* Template = PhysicsAsset->ConstraintSetup[ConstraintIndex];
		if (!Template)
			continue;
		const bool bSelected = ConstraintIndex == SelectedConstraintIndex;
		FMatrix Frame1, Frame2;
		if (!GetConstraintWorldFrames(*MeshComponent, Template->DefaultInstance, Frame1, Frame2))
			continue;

		// 두 프레임의 축은 원뿔 반지름의 10% 길이로 짧게 그린다 (UE FrameTransformRenderSize = Length * 0.1).
		const float AxisLength = (bSelected ? SelectedConstraintLimitSize : ConstraintLimitSize) * 0.8f;
		DrawConstraintFrame(LineBatcher, Frame1, AxisLength);
		DrawConstraintFrame(LineBatcher, Frame2, AxisLength);
		LineBatcher.AddLine(FrameOrigin(Frame1), FrameOrigin(Frame2), ConstraintColor);
		if (!bSelected)
			continue;

		// 현재 자세 표시 (UE dial): 부모 프레임 위치에서
		//   빨강 = 자식 X축 (Swing 원뿔 안에 있어야 한다), 초록 = 자식 Y축을 부모 YZ 평면에 투영 (Twist 부채꼴 안에 있어야 한다)
		const FVector Origin = FrameOrigin(Frame2);
		const FVector ParentX = FrameAxis(Frame2, 0);
		const FVector ChildY = FrameAxis(Frame1, 1);
		const FVector TwistDial = (ChildY - ParentX * ChildY.Dot(ParentX)).GetSafeNormal();
		//LineBatcher.AddLine(Origin, Origin + FrameAxis(Frame1, 0) * SelectedConstraintSize, FVector4(1.0f, 0.2f, 0.2f, 1.0f));
		//LineBatcher.AddLine(Origin, Origin + TwistDial * SelectedConstraintSize, FVector4(0.2f, 1.0f, 0.2f, 1.0f));
	}
}

void FPhysicsAssetEditor::OnBoneSelected()
{
	BodyGizmo.EndDrag();
	SelectedBodyIndex = -1;
	SelectedPrimType = EPhysPrimType::None;
	SelectedPrimIndex = -1;
	SelectedConstraintIndex = -1;
}

int32 FPhysicsAssetEditor::GetBodyBoneIndex(int32 BodyIndex) const
{
	if (!PhysicsAsset || !Mesh || BodyIndex < 0 || BodyIndex >= PhysicsAsset->SkeletalBodySetups.Num() || !PhysicsAsset->SkeletalBodySetups[BodyIndex])
		return -1;
	return Mesh->FindBoneIndex(PhysicsAsset->SkeletalBodySetups[BodyIndex]->BoneName.ToString());
}

int32 FPhysicsAssetEditor::GetConstraintBoneIndex(int32 ConstraintIndex) const
{
	if (!PhysicsAsset || !Mesh || ConstraintIndex < 0 || ConstraintIndex >= PhysicsAsset->ConstraintSetup.Num() || !PhysicsAsset->ConstraintSetup[ConstraintIndex])
		return -1;
	// 관절은 자식 본(Bone1) 원점에 있다.
	return Mesh->FindBoneIndex(PhysicsAsset->ConstraintSetup[ConstraintIndex]->DefaultInstance.ConstraintBone1.ToString());
}

bool FPhysicsAssetEditor::GenerateAllBodies()
{
	if (!PhysicsAsset || !Mesh)
		return false;
	// 바디 번호가 모두 바뀌므로 시뮬레이션·드래그·선택을 먼저 푼다.
	StopSimulation();
	BodyGizmo.EndDrag();
	SelectBody(-1);
	if (!FPhysicsAssetUtils::CreateFromSkeletalMesh(*PhysicsAsset, *Mesh, CreateParams))
	{
		KRS_LOG(Warning, "[Physics] Generate All Bodies failed: {}", GetAssetPath());
		return false;
	}
	PhysicsAsset->InvalidatePhysicsData(); // Dirty 표시 + 실행 바디 재생성
	KRS_LOG(Info, "[Physics] Generated {} bodies, {} constraints", PhysicsAsset->SkeletalBodySetups.Num(), PhysicsAsset->ConstraintSetup.Num());
	return true;
}

bool FPhysicsAssetEditor::AddBodyToSelectedBone()
{
	if (!PhysicsAsset || !Mesh || bSimulating)
		return false;
	const int32 BoneIndex = SelectedBodyIndex >= 0 ? GetBodyBoneIndex(SelectedBodyIndex) : GetSelectedBoneIndex();
	if (BoneIndex < 0)
		return false;
	BodyGizmo.EndDrag();
	const int32 BodyIndex = FPhysicsAssetUtils::CreateNewBody(*PhysicsAsset, *Mesh, BoneIndex, CreateParams);
	if (BodyIndex < 0)
	{
		KRS_LOG(Warning, "[Physics] Add Bodies failed: no vertices weighted to bone {}", Mesh->GetRefSkeleton()[BoneIndex].Name);
		return false;
	}
	PhysicsAsset->InvalidatePhysicsData();
	SelectBody(BodyIndex);
	return true;
}

bool FPhysicsAssetEditor::AddPrimitive(int32 BoneIndex, EPhysAssetFitGeomType GeomType)
{
	if (!PhysicsAsset || !Mesh || bSimulating || BoneIndex < 0 || BoneIndex >= Mesh->GetRefSkeleton().Num())
		return false;
	BodyGizmo.EndDrag();
	const int32 BodyIndex = FPhysicsAssetUtils::AddPrimitive(*PhysicsAsset, FName(Mesh->GetRefSkeleton()[BoneIndex].Name), GeomType);
	if (BodyIndex < 0)
		return false;
	PhysicsAsset->InvalidatePhysicsData();
	// 방금 더한 형상(해당 종류 배열의 마지막)을 고른다.
	const FKAggregateGeom& Geom = PhysicsAsset->SkeletalBodySetups[BodyIndex]->AggGeom;
	const EPhysPrimType NewType = GeomType == EPhysAssetFitGeomType::EFG_Box ? EPhysPrimType::Box
	                              : GeomType == EPhysAssetFitGeomType::EFG_Sphere ? EPhysPrimType::Sphere
	                                                                              : EPhysPrimType::Sphyl;
	SelectBody(BodyIndex, NewType, PrimCount(Geom, NewType) - 1);
	return true;
}

bool FPhysicsAssetEditor::DeleteBody(int32 BodyIndex)
{
	if (!PhysicsAsset || bSimulating || BodyIndex < 0 || BodyIndex >= PhysicsAsset->SkeletalBodySetups.Num())
		return false;
	BodyGizmo.EndDrag();
	const int32 BoneIndex = GetBodyBoneIndex(BodyIndex);
	PhysicsAsset->RemoveBody(BodyIndex);
	PhysicsAsset->InvalidatePhysicsData();
	// 번호가 밀렸으므로 선택을 풀고 그 본만 고른다.
	SelectBone(BoneIndex);
	return true;
}

bool FPhysicsAssetEditor::DeleteSelected()
{
	if (!PhysicsAsset || bSimulating)
		return false;
	if (SelectedConstraintIndex >= 0)
	{
		const int32 BoneIndex = GetConstraintBoneIndex(SelectedConstraintIndex);
		BodyGizmo.EndDrag();
		PhysicsAsset->RemoveConstraint(SelectedConstraintIndex);
		PhysicsAsset->InvalidatePhysicsData();
		SelectBone(BoneIndex);
		return true;
	}
	if (SelectedBodyIndex < 0 || SelectedBodyIndex >= PhysicsAsset->SkeletalBodySetups.Num() || !PhysicsAsset->SkeletalBodySetups[SelectedBodyIndex])
		return false;

	// 형상이 둘 이상이면 고른 형상만 지우고, 마지막 하나면 바디째 지운다.
	FKAggregateGeom& Geom = PhysicsAsset->SkeletalBodySetups[SelectedBodyIndex]->AggGeom;
	EPhysPrimType PrimType = SelectedPrimType;
	int32 PrimIndex = SelectedPrimIndex;
	const int32 Total = Geom.SphereElems.Num() + Geom.SphylElems.Num() + Geom.BoxElems.Num() + Geom.ConvexElems.Num();
	if (Total <= 1 || !ResolvePrim(Geom, PrimType, PrimIndex))
		return DeleteBody(SelectedBodyIndex);

	BodyGizmo.EndDrag();
	switch (PrimType)
	{
	case EPhysPrimType::Sphere: Geom.SphereElems.RemoveAt(PrimIndex, 1); break;
	case EPhysPrimType::Sphyl: Geom.SphylElems.RemoveAt(PrimIndex, 1); break;
	case EPhysPrimType::Box: Geom.BoxElems.RemoveAt(PrimIndex, 1); break;
	case EPhysPrimType::Convex: Geom.ConvexElems.RemoveAt(PrimIndex, 1); break;
	default: break;
	}
	PhysicsAsset->InvalidatePhysicsData();
	SelectBody(SelectedBodyIndex);	// 남은 형상 중 첫 형상을 고른다
	return true;
}

bool FPhysicsAssetEditor::CreateConstraintBetween(int32 ParentBodyIndex, int32 ChildBodyIndex)
{
	if (!PhysicsAsset || !Mesh || bSimulating)
		return false;
	BodyGizmo.EndDrag();
	const int32 ConstraintIndex = FPhysicsAssetUtils::CreateNewConstraint(*PhysicsAsset, *Mesh, ParentBodyIndex, ChildBodyIndex, CreateParams);
	if (ConstraintIndex < 0)
		return false;
	PhysicsAsset->InvalidatePhysicsData();
	SelectConstraint(ConstraintIndex);
	return true;
}

void FPhysicsAssetEditor::SelectBody(int32 BodyIndex, EPhysPrimType PrimType, int32 PrimIndex)
{
	// 본 선택을 먼저 바꾸고(OnBoneSelected가 선택을 비운다) 그다음 바디를 고른다.
	SelectBone(GetBodyBoneIndex(BodyIndex));
	SelectedBodyIndex = GetBodyBoneIndex(BodyIndex) >= 0 ? BodyIndex : -1;
	SelectedPrimType = EPhysPrimType::None;
	SelectedPrimIndex = -1;
	if (SelectedBodyIndex >= 0 && PhysicsAsset && PhysicsAsset->SkeletalBodySetups[SelectedBodyIndex])
	{
		// 지정한 형상이 유효하면 그것을, 아니면 첫 형상을 고른다.
		ResolvePrim(PhysicsAsset->SkeletalBodySetups[SelectedBodyIndex]->AggGeom, PrimType, PrimIndex);
		SelectedPrimType = PrimType;
		SelectedPrimIndex = PrimIndex;
	}
}

void FPhysicsAssetEditor::SelectConstraint(int32 ConstraintIndex)
{
	SelectBone(GetConstraintBoneIndex(ConstraintIndex));
	SelectedConstraintIndex = GetConstraintBoneIndex(ConstraintIndex) >= 0 ? ConstraintIndex : -1;
}

bool FPhysicsAssetEditor::DrawViewportToolbarLeft()
{
	// 아이콘(Engine/Icons/Toolbar) + 글자 버튼. 시뮬레이션 중에는 Stop 아이콘과 초록 배경.
	static UTexture2D* SimulateIcon = UAssetManager::GetAssetByPath<UTexture2D>("Engine/Icons/Toolbar/Simulate.png");
	static UTexture2D* StopIcon = UAssetManager::GetAssetByPath<UTexture2D>("Engine/Icons/Toolbar/Stop.png");
	UTexture2D* Icon = bSimulating ? StopIcon : SimulateIcon;
	const char* Label = bSimulating ? "Stop" : "Simulate";

	const float Height = ImGui::GetFrameHeight();
	const float IconSize = Height - ImGui::GetStyle().FramePadding.y * 2.0f;
	const float Gap = ImGui::GetStyle().FramePadding.x * 0.6f;
	const float Width = ImGui::GetStyle().FramePadding.x * 2.0f + IconSize + Gap + ImGui::CalcTextSize(Label).x;

	if (bSimulating)
		ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.55f, 0.25f, 1.0f));
	const ImVec2 Pos = ImGui::GetCursorScreenPos();
	const bool bClicked = ImGui::Button("##PhysicsSimulate", ImVec2(Width, Height));
	if (bSimulating)
		ImGui::PopStyleColor();

	ImDrawList* DrawList = ImGui::GetWindowDrawList();
	float X = Pos.x + ImGui::GetStyle().FramePadding.x;
	const float CenterY = Pos.y + Height * 0.5f;
	if (Icon && Icon->GetResource() && Icon->GetResource()->GetSRV())
		DrawList->AddImage((ImTextureID)Icon->GetResource()->GetSRV(), {X, CenterY - IconSize * 0.5f}, {X + IconSize, CenterY + IconSize * 0.5f});
	X += IconSize + Gap;
	DrawList->AddText({X, CenterY - ImGui::GetTextLineHeight() * 0.5f}, ImGui::GetColorU32(ImGuiCol_Text), Label);

	if (ImGui::IsItemHovered())
		ImGui::SetTooltip(bSimulating ? "Stop the simulation and restore the pose" : "Simulate the physics asset as a ragdoll");
	if (bClicked)
		ToggleSimulation();
	return true;
}

void FPhysicsAssetEditor::ToggleSimulation()
{
	if (bSimulating)
		StopSimulation();
	else
		StartSimulation();
}

void FPhysicsAssetEditor::InitializePreviewPhysics()
{
	// UE Persona 미리보기 씬(ShouldSimulatePhysics = true)처럼 이 에디터의 미리보기 월드는 물리를 항상 돌린다.
	// 바디는 늘 있고 평소에는 키네마틱으로 포즈를 따라간다. Simulate는 시뮬레이션 여부만 바꾼다.
	UWorld* World = PreviewScene.GetWorld();
	if (!World || !Mesh || !MeshComponent || !PhysicsAsset)
		return;
	World->SetPreviewPhysicsEnabled(true);

	// 바닥 (UE 미리보기 씬의 바닥 메시): 메시 바인드 AABB 아래면에 윗면이 오는 얇고 넓은 판.
	// 보이는 판은 기본 Cube(한 변 100)를 납작하게 늘린 것이고, 충돌은 같은 크기의 박스가 맡는다.
	constexpr float FloorHalfSize = 500.0f;		// 10m x 10m
	constexpr float FloorHalfThickness = 5.0f;
	const FBox& Bounds = Mesh->GetRenderData().LODRenderData[0].BindAABB;
	const FVector MeshLocation = MeshComponent->GetWorldLocation();
	const float FloorTop = MeshLocation.Z + Bounds.Min.Z * MeshComponent->GetWorldScale3D().Z;
	const FVector FloorCenter(MeshLocation.X, MeshLocation.Y, FloorTop - FloorHalfThickness);

	FloorComponent = Cast<UBoxComponent>(PreviewScene.AddComponent(UBoxComponent::StaticClass()));
	if (FloorComponent)
	{
		FloorComponent->bDrawOnlyIfSelected = true;
		FloorComponent->BoxExtent = FVector(FloorHalfSize, FloorHalfSize, FloorHalfThickness);
		FloorComponent->SetRelativeLocation(FloorCenter);
		// 도형 기본 프리셋(OverlapAllDynamic)은 겹침만 하고 막지 않는다 → 바닥·벽용 BlockAll
		FloorComponent->SetCollisionProfileName(FName("BlockAll"));
		FloorComponent->RecreatePhysicsState();
	}
	FloorMeshComponent = Cast<UStaticMeshComponent>(PreviewScene.AddComponent(UStaticMeshComponent::StaticClass()));
	if (FloorMeshComponent)
	{
		constexpr float CubeSize = 100.0f;
		FloorMeshComponent->SetStaticMesh(UAssetManager::GetAssetByPath<UStaticMesh>("Cube"));
		FloorMeshComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		// 윗면을 그리드보다 아주 조금 내려 겹쳐 깜빡이지 않게 한다.
		FloorMeshComponent->SetRelativeLocation(FloorCenter - FVector(0.0f, 0.0f, 0.1f));
		FloorMeshComponent->SetRelativeScale3D(FVector(FloorHalfSize * 2.0f, FloorHalfSize * 2.0f, FloorHalfThickness * 2.0f) / CubeSize);
	}

	// 편집 중인 사본으로 바디·관절을 만든다. 에셋을 고치면 InvalidatePhysicsData가 이 바디를 다시 만든다.
	MeshComponent->SetPhysicsAsset(PhysicsAsset);
	MeshComponent->GetBodyInstance().bSimulatePhysics = false;
	MeshComponent->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	MeshComponent->RecreatePhysicsState();
}

bool FPhysicsAssetEditor::StartSimulation()
{
	if (bSimulating || !MeshComponent)
		return false;
	BodyGizmo.EndDrag();
	BodyGizmo.SetTarget(nullptr);
	// 시뮬레이션 중에는 편집하지 않으므로 바디·관절 선택을 푼다.
	SelectBody(-1);

	// 바디가 없으면(바디 없는 에셋에서 생성한 직후 등) 지금 만든다.
	if (MeshComponent->Bodies.IsEmpty())
		MeshComponent->RecreatePhysicsState();
	if (MeshComponent->Bodies.IsEmpty())
	{
		KRS_LOG(Warning, "[Physics] Simulate failed: no bodies for {}", GetAssetPath());
		return false;
	}
	// 시뮬레이션 중 컴포넌트가 루트 바디를 따라 움직이므로 끌 때 되돌릴 위치를 저장한다 (UE ResetTM).
	SimulationResetTransform = MeshComponent->GetWorldMatrix();
	// UE EnableSimulation(true): SetSimulatePhysics(true) + 깨우기. Kinematic으로 지정한 바디는 그대로 키네마틱이다.
	MeshComponent->SetAllBodiesSimulatePhysics(true);
	bSimulating = true;
	return true;
}

void FPhysicsAssetEditor::StopSimulation()
{
	if (!bSimulating)
		return;
	ReleaseSimulationGrab();
	bSimulating = false;
	if (!MeshComponent)
		return;
	// UE EnableSimulation(false): 시뮬레이션을 끄고 레퍼런스 포즈로 돌린다.
	// 바디를 다시 만들어 속도까지 지우고 바인드 포즈 위치에 둔다.
	MeshComponent->SetAllBodiesSimulatePhysics(false);
	// 컴포넌트를 시작 위치로 되돌린 뒤 바디를 그 위치에 다시 만든다 (UE SetWorldTransform(ResetTM)).
	MeshComponent->SetWorldTransform(SimulationResetTransform, false);
	MeshComponent->RecreatePhysicsState();
	MeshComponent->RefreshBoneTransformsNow();
}

void FPhysicsAssetEditor::UpdateSimulationGrab(float DeltaTime)
{
	// 잡고 있는 중: 우클릭을 떼거나 Esc면 놓는다.
	if (SimGrab.BodyIndex >= 0)
	{
		if (!ImGui::IsMouseDown(ImGuiMouseButton_Right) || PointerInput.bCancel)
		{
			ReleaseSimulationGrab();
			return;
		}
		// 드래그: 잡은 지점을 지나고 화면과 평행한 평면(법선 = 카메라 앞 방향)과 마우스 광선이 만나는 점으로 손을 옮긴다.
		// 손이 마우스 바로 아래를 따라가고, 카메라에서의 깊이는 잡을 때 그대로다 (UE SimGrab과 같은 평면 이동).
		const FVector2 MousePosition(PointerInput.MouseUV.X * ViewportWidth, PointerInput.MouseUV.Y * ViewportHeight);
		const FRay Ray = ViewportClient->DeprojectScreenToWorld(MousePosition);
		const FVector Direction = Ray.Direction.GetSafeNormal();
		const FVector PlaneNormal = ViewportClient->GetViewForward();
		const float Facing = Direction.Dot(PlaneNormal);
		// 평면과 만나지 않으면(광선이 평면과 나란하거나 뒤쪽) 마우스 목표는 이전 값을 그대로 쓴다.
		if (std::fabs(Facing) >= 1.0e-4f)
		{
			const float Along = (SimGrab.Location - Ray.Origin).Dot(PlaneNormal) / Facing;
			if (Along > 0.0f)
				SimGrab.TargetLocation = Ray.Origin + Direction * Along;
		}
		GrabTrail.Add({SimGrab.TargetLocation, 0.0f});
		// UE PhysicsHandle 보간: 손은 마우스 목표로 바로 가지 않고 매 프레임 DeltaTime × InterpolationSpeed 비율만큼 다가간다.
		// 손이 순간이동하지 않으므로 스프링 힘이 튀지 않는다.
		const float Alpha = std::clamp(DeltaTime * GrabInterpolationSpeed, 0.0f, 1.0f);
		SimGrab.CurrentLocation = SimGrab.CurrentLocation + (SimGrab.TargetLocation - SimGrab.CurrentLocation) * Alpha;
		// 키네마틱 목표로 옮겨야 다음 시뮬레이션 스텝에서 손이 그 위치로 움직이며 스프링이 바디를 당긴다.
		SimGrab.Handle->setKinematicTarget(physx::PxTransform(PhysicsMath::ToPx(SimGrab.CurrentLocation)));
		physx::PxRigidActor* Actor0 = nullptr;
		physx::PxRigidActor* Actor1 = nullptr;
		SimGrab.Joint->getActors(Actor0, Actor1);
		if (physx::PxRigidDynamic* BodyActor = Actor1 ? Actor1->is<physx::PxRigidDynamic>() : nullptr)
			BodyActor->wakeUp();
		return;
	}

	// 잡기 시작: 뷰포트 위에서 Ctrl을 누른 채 우클릭한 프레임만
	if (!CanProcessViewportInput() || !MeshComponent || !PhysicsAsset || !PointerInput.bHovered ||
	    ViewportWidth == 0 || ViewportHeight == 0 || !ImGui::GetIO().KeyCtrl ||
	    !ImGui::IsMouseClicked(ImGuiMouseButton_Right))
	{
		return;
	}

	// 바디 피킹과 같은 마우스 광선으로 맞은 바디와 그 지점을 찾는다.
	const FVector2 MousePosition(PointerInput.MouseUV.X * ViewportWidth, PointerInput.MouseUV.Y * ViewportHeight);
	float Distance = 0.0f;
	const int32 BodyIndex = PickBody(MousePosition, &Distance);
	if (BodyIndex < 0)
		return;
	// 키네마틱 바디는 끌려가지 않으므로 시뮬레이션 바디만 잡는다.
	const USkeletalBodySetup* Setup = PhysicsAsset->SkeletalBodySetups[BodyIndex];
	FBodyInstance* Body = Setup ? MeshComponent->FindBodyInstance(Setup->BoneName) : nullptr;
	physx::PxRigidDynamic* BodyActor = Body && Body->GetActor() ? Body->GetActor()->is<physx::PxRigidDynamic>() : nullptr;
	UWorld* World = PreviewScene.GetWorld();
	FPhysScene* Scene = World ? World->GetPhysicsScene() : nullptr;
	if (!BodyActor || !Body->IsInstanceSimulatingPhysics() || !Scene)
		return;

	const FRay Ray = ViewportClient->DeprojectScreenToWorld(MousePosition);
	SimGrab.BodyIndex = BodyIndex;
	SimGrab.Location = Ray.Origin + Ray.Direction.GetSafeNormal() * Distance;
	SimGrab.TargetLocation = SimGrab.CurrentLocation = SimGrab.Location;

	// 손: 잡은 지점에 둔 형상 없는 키네마틱 액터 (충돌하지 않고, 우리가 옮길 때만 움직인다)
	physx::PxPhysics& Physics = *Scene->GetPhysics();
	const physx::PxVec3 GrabPoint = PhysicsMath::ToPx(SimGrab.Location);
	SimGrab.Handle = Physics.createRigidDynamic(physx::PxTransform(GrabPoint));
	SimGrab.Handle->setRigidBodyFlag(physx::PxRigidBodyFlag::eKINEMATIC, true);
	Scene->GetPhysXScene()->addActor(*SimGrab.Handle);

	// 고무줄: 손 원점과 바디의 잡은 지점을 잇는 D6 관절. 위치 축은 자유로 두고 스프링 드라이브로 끌어당긴다.
	// 바디 쪽 프레임 회전을 바디 회전의 역으로 두어 두 프레임 축이 월드에서 일치하게 한다.
	const physx::PxTransform BodyPose = BodyActor->getGlobalPose();
	const physx::PxTransform BodyFrame(BodyPose.transformInv(GrabPoint), BodyPose.q.getConjugate());
	SimGrab.Joint = physx::PxD6JointCreate(Physics, SimGrab.Handle, physx::PxTransform(physx::PxIdentity), BodyActor, BodyFrame);
	if (!SimGrab.Joint)
	{
		ReleaseSimulationGrab();
		return;
	}
	for (const physx::PxD6Axis::Enum Axis : {physx::PxD6Axis::eX, physx::PxD6Axis::eY, physx::PxD6Axis::eZ,
	                                         physx::PxD6Axis::eTWIST, physx::PxD6Axis::eSWING1, physx::PxD6Axis::eSWING2})
		SimGrab.Joint->setMotion(Axis, physx::PxD6Motion::eFREE);
	// UE 물리 에셋 에디터 기본값 (HandleLinearStiffness 750, HandleLinearDamping 200).
	// 가속도 드라이브라 바디 질량과 상관없이 같은 세기로 당긴다.
	constexpr float HandleStiffness = 750.0f;
	constexpr float HandleDamping = 200.0f;
	const physx::PxD6JointDrive Drive(HandleStiffness, HandleDamping, PX_MAX_F32, true);
	SimGrab.Joint->setDrive(physx::PxD6Drive::eX, Drive);
	SimGrab.Joint->setDrive(physx::PxD6Drive::eY, Drive);
	SimGrab.Joint->setDrive(physx::PxD6Drive::eZ, Drive);
	SimGrab.Joint->setDrivePosition(physx::PxTransform(physx::PxIdentity));
	BodyActor->wakeUp();
}

void FPhysicsAssetEditor::ReleaseSimulationGrab()
{
	// 관절을 먼저 풀고 손을 지운다 (release가 씬에서도 뺀다).
	if (SimGrab.Joint)
		SimGrab.Joint->release();
	if (SimGrab.Handle)
		SimGrab.Handle->release();
	SimGrab = FSimulationGrab();
}

void FPhysicsAssetEditor::ShutdownResources()
{
	StopSimulation();
	if (MeshComponent)
		MeshComponent->DestroyPhysicsState();
	if (FloorComponent)
		FloorComponent->DestroyPhysicsState();
	if (UWorld* World = PreviewScene.GetWorld())
		World->SetPreviewPhysicsEnabled(false);
	FloorComponent = nullptr;
	FloorMeshComponent = nullptr;

	BodyGizmo.EndDrag();
	BodyGizmo.SetTarget(nullptr);
	BodyGizmoTarget = nullptr;
	FSkeletalPreviewEditor::ShutdownResources();
}

void FPhysicsAssetEditor::TickAfterWorld(float DeltaTime)
{
	// 드래그 궤적 점을 늙히고 수명이 지난 점을 지운다 (놓은 뒤에도 잠깐 남는다).
	for (FGrabTrailPoint& Point : GrabTrail)
		Point.Age += DeltaTime;
	while (!GrabTrail.IsEmpty() && GrabTrail[0].Age >= GrabTrailLifetime)
		GrabTrail.RemoveAt(0);

	if (bSimulating)
	{
		// 시뮬레이션 중에는 편집하지 않는다 (UE와 같다).
		BodyGizmo.EndDrag();
		BodyGizmo.SetTarget(nullptr);
		UpdateSimulationGrab(DeltaTime);
		return;
	}
	UpdateGizmoModeKeys();
	UpdateDeleteKey();
	// 기즈모가 마우스를 쓰지 않았을 때만 바디를 피킹하고, 마지막에 기즈모를 선택한 바디로 옮긴다.
	const bool bGizmoUsedMouse = SelectedConstraintIndex >= 0 ? UpdateConstraintGizmo() : UpdateBodyGizmo();
	if (!bGizmoUsedMouse)
		UpdateBodyPicking();
	SyncBodyGizmo();
}

void FPhysicsAssetEditor::SetGizmoMode(EGizmoMode Mode)
{
	BodyGizmo.EndDrag();
	BodyGizmo.SetMode(Mode);
	SyncBodyGizmo();
}

void FPhysicsAssetEditor::UpdateGizmoModeKeys()
{
	// 스켈레탈 메시 에디터처럼 W/E/R. 우클릭 비행 중(카메라 이동 W/E)이나 글자 입력 중에는 바꾸지 않는다.
	const ImGuiIO& IO = ImGui::GetIO();
	if (!PointerInput.bFocused || !PointerInput.bHovered || CameraInput.bCaptured || IO.WantTextInput ||
	    ImGui::IsMouseDown(ImGuiMouseButton_Right) || BodyGizmo.IsUsing())
		return;
	// 관절은 UE처럼 이동·회전만 있다 → R은 무시하고, 스페이스바 순환에서도 크기를 건너뛴다.
	const bool bConstraintSelected = SelectedConstraintIndex >= 0;
	if (ImGui::IsKeyPressed(ImGuiKey_W, false))
		SetGizmoMode(EGizmoMode::Location);
	else if (ImGui::IsKeyPressed(ImGuiKey_E, false))
		SetGizmoMode(EGizmoMode::Rotation);
	else if (ImGui::IsKeyPressed(ImGuiKey_R, false) && !bConstraintSelected)
		SetGizmoMode(EGizmoMode::Scale);
	else if (ImGui::IsKeyPressed(ImGuiKey_Space, false))
	{
		// UE 스페이스바: 이동 → 회전 → 크기 → 이동
		const EGizmoMode Mode = BodyGizmo.GetMode();
		if (Mode == EGizmoMode::Location)
			SetGizmoMode(EGizmoMode::Rotation);
		else if (Mode == EGizmoMode::Rotation && !bConstraintSelected)
			SetGizmoMode(EGizmoMode::Scale);
		else
			SetGizmoMode(EGizmoMode::Location);
	}
}

void FPhysicsAssetEditor::UpdateDeleteKey()
{
	const ImGuiIO& IO = ImGui::GetIO();
	if (!PointerInput.bFocused || !PointerInput.bHovered || CameraInput.bCaptured || IO.WantTextInput ||
	    ImGui::IsMouseDown(ImGuiMouseButton_Right) || BodyGizmo.IsUsing())
		return;
	if (ImGui::IsKeyPressed(ImGuiKey_Delete, false))
		DeleteSelected();
}

bool FPhysicsAssetEditor::UpdateBodyGizmo()
{
	USkeletalBodySetup* Setup = (PhysicsAsset && SelectedBodyIndex >= 0 && SelectedBodyIndex < PhysicsAsset->SkeletalBodySetups.Num())
	                                ? PhysicsAsset->SkeletalBodySetups[SelectedBodyIndex]
	                                : nullptr;
	if (!CanProcessViewportInput() || !BodyGizmo.GetTarget() || !Setup)
	{
		BodyGizmo.EndDrag();
		return false;
	}

	FKAggregateGeom& Geom = Setup->AggGeom;
	const bool bWasUsing = BodyGizmo.IsUsing();
	// 기즈모가 붙은 형상 (선택한 형상, 무효면 첫 형상)
	EPhysPrimType PrimType = SelectedPrimType;
	int32 PrimIndex = SelectedPrimIndex;
	if (!ResolvePrim(Geom, PrimType, PrimIndex))
	{
		BodyGizmo.EndDrag();
		return false;
	}

	// Esc: 드래그 시작 값으로 되돌린다.
	if (PointerInput.bCancel)
	{
		if (bWasUsing && DragStart.BodyIndex == SelectedBodyIndex)
		{
			if (PrimType == EPhysPrimType::Sphere)
			{
				Geom.SphereElems[PrimIndex].Center = DragStart.Center;
				Geom.SphereElems[PrimIndex].Radius = DragStart.Radius;
			}
			else if (PrimType == EPhysPrimType::Sphyl)
			{
				Geom.SphylElems[PrimIndex].Center = DragStart.Center;
				Geom.SphylElems[PrimIndex].Rotation = DragStart.Rotation;
				Geom.SphylElems[PrimIndex].Radius = DragStart.Radius;
				Geom.SphylElems[PrimIndex].Length = DragStart.Length;
			}
			else if (PrimType == EPhysPrimType::Box)
			{
				Geom.BoxElems[PrimIndex].Center = DragStart.Center;
				Geom.BoxElems[PrimIndex].Rotation = DragStart.Rotation;
				Geom.BoxElems[PrimIndex].X = DragStart.BoxSize.X;
				Geom.BoxElems[PrimIndex].Y = DragStart.BoxSize.Y;
				Geom.BoxElems[PrimIndex].Z = DragStart.BoxSize.Z;
			}
			else if (PrimType == EPhysPrimType::Convex)
				Geom.ConvexElems[PrimIndex].Transform = DragStart.ConvexTransform;
		}
		BodyGizmo.EndDrag();
		return false;
	}

	if ((!PointerInput.bHovered && !bWasUsing) || CameraInput.bCaptured)
	{
		BodyGizmo.EndDrag();
		return false;
	}

	SyncBodyGizmo();
	const int32 Width = static_cast<int32>(ViewportClient->GetWidth());
	const int32 Height = static_cast<int32>(ViewportClient->GetHeight());
	const FVector2 MousePosition(PointerInput.MouseUV.X * Width, PointerInput.MouseUV.Y * Height);
	const FRay MouseRay = ViewportClient->DeprojectScreenToWorld(MousePosition);

	// 클릭한 프레임에만 시작하고, 버튼을 놓는 프레임의 마지막 마우스 위치도 반영한다.
	const bool bUpdateDrag = bWasUsing || (PointerInput.bMousePressed && PointerInput.bMouseDown);
	BodyGizmo.Update(MouseRay, MousePosition, ViewportClient->GetViewProjectionMatrix(),
	                 Width, Height, bUpdateDrag, ViewportClient->GetViewLocation(), !ViewportClient->IsPerspective());

	if (!bWasUsing && BodyGizmo.IsUsing())
	{
		// 드래그 시작 값 저장. 본 프레임은 드래그 동안 고정한다 (미리보기 포즈는 움직이지 않는다).
		const int32 BoneIndex = MeshComponent->FindBoneIndex(Setup->BoneName.ToString());
		DragStart = FShapeDragStart();
		DragStart.BodyIndex = SelectedBodyIndex;
		DragStart.BoneFrameInverse = BoneFrameWithoutScale(*MeshComponent, BoneIndex).Inverse();
		if (PrimType == EPhysPrimType::Sphere)
		{
			DragStart.Center = Geom.SphereElems[PrimIndex].Center;
			DragStart.Radius = Geom.SphereElems[PrimIndex].Radius;
		}
		else if (PrimType == EPhysPrimType::Sphyl)
		{
			DragStart.Center = Geom.SphylElems[PrimIndex].Center;
			DragStart.Rotation = Geom.SphylElems[PrimIndex].Rotation;
			DragStart.Radius = Geom.SphylElems[PrimIndex].Radius;
			DragStart.Length = Geom.SphylElems[PrimIndex].Length;
		}
		else if (PrimType == EPhysPrimType::Box)
		{
			DragStart.Center = Geom.BoxElems[PrimIndex].Center;
			DragStart.Rotation = Geom.BoxElems[PrimIndex].Rotation;
			DragStart.BoxSize = FVector(Geom.BoxElems[PrimIndex].X, Geom.BoxElems[PrimIndex].Y, Geom.BoxElems[PrimIndex].Z);
		}
		else if (PrimType == EPhysPrimType::Convex)
			DragStart.ConvexTransform = Geom.ConvexElems[PrimIndex].Transform;
	}

	if (BodyGizmo.IsUsing())
	{
		ApplyBodyGizmo();
		if (!PointerInput.bMouseDown)
		{
			BodyGizmo.EndDrag();
			// 놓을 때 한 번만 에셋을 Dirty로 만들고 실행 바디를 다시 만든다 (드래그 중에는 그리기만 바뀐다).
			PhysicsAsset->InvalidatePhysicsData();
		}
	}

	return bWasUsing || BodyGizmo.IsUsing() || BodyGizmo.GetHoveredAxis() >= 0;
}

void FPhysicsAssetEditor::ApplyBodyGizmo()
{
	if (!PhysicsAsset || DragStart.BodyIndex != SelectedBodyIndex || SelectedBodyIndex < 0 ||
	    SelectedBodyIndex >= PhysicsAsset->SkeletalBodySetups.Num() || !PhysicsAsset->SkeletalBodySetups[SelectedBodyIndex])
		return;
	FKAggregateGeom& Geom = PhysicsAsset->SkeletalBodySetups[SelectedBodyIndex]->AggGeom;
	EPhysPrimType PrimType = SelectedPrimType;
	int32 PrimIndex = SelectedPrimIndex;
	if (!ResolvePrim(Geom, PrimType, PrimIndex))
		return;

	// 기즈모 대상의 월드 Transform → 본 프레임 기준 형상 로컬 (그리기 규칙의 역)
	const FMatrix TargetWorld = FTransform(BodyGizmoTarget->GetWorldRotation(), BodyGizmoTarget->GetWorldLocation()).GetLocalMatrix();
	const FMatrix ShapeLocal = TargetWorld * DragStart.BoneFrameInverse;
	const FVector NewCenter = FrameOrigin(ShapeLocal);
	const FRotator NewRotation = MatrixToRotator(ShapeLocal);
	// 크기 모드: 기즈모 대상의 축별 배율(시작 1). 한 번에 한 축만 바뀌므로 곱으로 합쳐도 그 축 배율이 된다.
	const FVector S = BodyGizmoTarget->GetRelativeScale3D();
	constexpr float MinSize = 0.01f;	// AggregateGeom의 ClampMin과 같다

	const EGizmoMode Mode = BodyGizmo.GetMode();
	if (PrimType == EPhysPrimType::Sphere)
	{
		FKSphereElem& Sphere = Geom.SphereElems[PrimIndex];
		if (Mode == EGizmoMode::Location)
			Sphere.Center = NewCenter;
		else if (Mode == EGizmoMode::Scale)
			Sphere.Radius = FMath::Max(DragStart.Radius * S.X * S.Y * S.Z, MinSize);
		// 구는 회전이 없다
	}
	else if (PrimType == EPhysPrimType::Sphyl)
	{
		FKSphylElem& Capsule = Geom.SphylElems[PrimIndex];
		if (Mode == EGizmoMode::Location)
			Capsule.Center = NewCenter;
		else if (Mode == EGizmoMode::Rotation)
			Capsule.Rotation = NewRotation;
		else if (Mode == EGizmoMode::Scale)
		{
			// 캡슐 축은 형상 로컬 Z: X/Y는 반지름, Z는 원기둥 길이
			Capsule.Radius = FMath::Max(DragStart.Radius * S.X * S.Y, MinSize);
			Capsule.Length = FMath::Max(DragStart.Length * S.Z, 0.0f);
		}
	}
	else if (PrimType == EPhysPrimType::Box)
	{
		FKBoxElem& Box = Geom.BoxElems[PrimIndex];
		if (Mode == EGizmoMode::Location)
			Box.Center = NewCenter;
		else if (Mode == EGizmoMode::Rotation)
			Box.Rotation = NewRotation;
		else if (Mode == EGizmoMode::Scale)
		{
			Box.X = FMath::Max(DragStart.BoxSize.X * S.X, MinSize);
			Box.Y = FMath::Max(DragStart.BoxSize.Y * S.Y, MinSize);
			Box.Z = FMath::Max(DragStart.BoxSize.Z * S.Z, MinSize);
		}
	}
	else if (PrimType == EPhysPrimType::Convex)
	{
		// 컨벡스는 꼭짓점을 그대로 두고 Transform의 위치·회전만 바꾼다
		FTransform& Transform = Geom.ConvexElems[PrimIndex].Transform;
		if (Mode == EGizmoMode::Location)
			Transform.Location = NewCenter;
		else if (Mode == EGizmoMode::Rotation)
			Transform.Rotation = NewRotation.Quaternion();
	}
}

void FPhysicsAssetEditor::UpdateBodyPicking()
{
	// 본 피킹(UpdateBonePicking)과 같은 조건: 뷰포트 위에서 좌클릭이 시작된 프레임만
	if (!CanProcessViewportInput() || !Mesh || !MeshComponent || !PhysicsAsset ||
	    !PointerInput.bHovered || !PointerInput.bMousePressed || PointerInput.bCancel ||
	    CameraInput.bCaptured || ViewportWidth == 0 || ViewportHeight == 0)
	{
		return;
	}

	const FVector2 MousePosition(PointerInput.MouseUV.X * ViewportWidth, PointerInput.MouseUV.Y * ViewportHeight);
	// 관절은 바디 안쪽에 있어 바디를 먼저 보면 고를 수 없다 → 관절 표시 근처를 눌렀으면 관절을 고른다.
	if (const int32 Constraint = PickConstraint(MousePosition); Constraint >= 0)
	{
		SelectConstraint(Constraint);
		return;
	}
	EPhysPrimType PrimType = EPhysPrimType::None;
	int32 PrimIndex = -1;
	const int32 Body = PickBody(MousePosition, nullptr, &PrimType, &PrimIndex);
	if (Body >= 0)
		SelectBody(Body, PrimType, PrimIndex);
	else if (IsBoneVisualizerVisible())
		UpdateBonePicking();	// 바디가 안 맞으면 본을 고른다 (본도 안 맞으면 선택 해제)
	else
		SelectBone(-1);			// 빈 곳 클릭 = 선택 해제
}

bool FPhysicsAssetEditor::UpdateConstraintGizmo()
{
	UPhysicsConstraintTemplate* Template = (PhysicsAsset && SelectedConstraintIndex < PhysicsAsset->ConstraintSetup.Num())
	                                           ? PhysicsAsset->ConstraintSetup[SelectedConstraintIndex]
	                                           : nullptr;
	if (!CanProcessViewportInput() || !BodyGizmo.GetTarget() || !Template || !MeshComponent)
	{
		BodyGizmo.EndDrag();
		return false;
	}
	FConstraintInstance& Instance = Template->DefaultInstance;
	const bool bWasUsing = BodyGizmo.IsUsing();

	// Esc: 드래그 시작 값으로 되돌린다.
	if (PointerInput.bCancel)
	{
		if (bWasUsing && ConstraintDragStart.ConstraintIndex == SelectedConstraintIndex)
		{
			Instance.SetRefFrame(EConstraintFrame::Frame1, ConstraintDragStart.Frame1Local);
			Instance.SetRefFrame(EConstraintFrame::Frame2, ConstraintDragStart.Frame2Local);
		}
		BodyGizmo.EndDrag();
		return false;
	}

	if ((!PointerInput.bHovered && !bWasUsing) || CameraInput.bCaptured)
	{
		BodyGizmo.EndDrag();
		return false;
	}

	SyncBodyGizmo();
	const int32 Width = static_cast<int32>(ViewportClient->GetWidth());
	const int32 Height = static_cast<int32>(ViewportClient->GetHeight());
	const FVector2 MousePosition(PointerInput.MouseUV.X * Width, PointerInput.MouseUV.Y * Height);
	const FRay MouseRay = ViewportClient->DeprojectScreenToWorld(MousePosition);

	const bool bUpdateDrag = bWasUsing || (PointerInput.bMousePressed && PointerInput.bMouseDown);
	BodyGizmo.Update(MouseRay, MousePosition, ViewportClient->GetViewProjectionMatrix(),
	                 Width, Height, bUpdateDrag, ViewportClient->GetViewLocation(), !ViewportClient->IsPerspective());

	if (!bWasUsing && BodyGizmo.IsUsing())
	{
		// 드래그 시작 값 저장. 본 프레임은 드래그 동안 고정한다 (미리보기 포즈는 움직이지 않는다).
		const int32 ChildBone = MeshComponent->FindBoneIndex(Instance.ConstraintBone1.ToString());
		const int32 ParentBone = MeshComponent->FindBoneIndex(Instance.ConstraintBone2.ToString());
		FMatrix Frame1, Frame2;
		if (ChildBone < 0 || ParentBone < 0 || !GetConstraintWorldFrames(*MeshComponent, Instance, Frame1, Frame2))
		{
			BodyGizmo.EndDrag();
			return true;
		}
		ConstraintDragStart = FConstraintDragStart();
		ConstraintDragStart.ConstraintIndex = SelectedConstraintIndex;
		ConstraintDragStart.Frame1Local = Instance.GetRefFrame(EConstraintFrame::Frame1);
		ConstraintDragStart.Frame2Local = Instance.GetRefFrame(EConstraintFrame::Frame2);
		ConstraintDragStart.Frame1World = Frame1;
		ConstraintDragStart.Frame2World = Frame2;
		ConstraintDragStart.ChildBoneInverse = BoneFrameWithoutScale(*MeshComponent, ChildBone).Inverse();
		ConstraintDragStart.ParentBoneInverse = BoneFrameWithoutScale(*MeshComponent, ParentBone).Inverse();
		// 기즈모는 Frame2에 붙어 있다 (SyncBodyGizmo)
		ConstraintDragStart.TargetInverse = Frame2.Inverse();
		ConstraintDragStart.Scale = MeshComponent->GetWorldScale3D().X;
	}

	if (BodyGizmo.IsUsing())
	{
		ApplyConstraintGizmo();
		if (!PointerInput.bMouseDown)
		{
			BodyGizmo.EndDrag();
			PhysicsAsset->InvalidatePhysicsData();
		}
	}

	return bWasUsing || BodyGizmo.IsUsing() || BodyGizmo.GetHoveredAxis() >= 0;
}

void FPhysicsAssetEditor::ApplyConstraintGizmo()
{
	if (!PhysicsAsset || ConstraintDragStart.ConstraintIndex != SelectedConstraintIndex || SelectedConstraintIndex < 0 ||
	    SelectedConstraintIndex >= PhysicsAsset->ConstraintSetup.Num() || !PhysicsAsset->ConstraintSetup[SelectedConstraintIndex])
		return;
	const EGizmoMode Mode = BodyGizmo.GetMode();
	if (Mode != EGizmoMode::Location && Mode != EGizmoMode::Rotation)
		return;
	FConstraintInstance& Instance = PhysicsAsset->ConstraintSetup[SelectedConstraintIndex]->DefaultInstance;

	// 기즈모 변화량 Delta = 시작⁻¹ × 현재. 두 프레임 모두 같은 변화량을 받아 함께 움직인다 (UE 기본: 부모·자식 프레임 모두).
	const FMatrix TargetWorld = FTransform(BodyGizmoTarget->GetWorldRotation(), BodyGizmoTarget->GetWorldLocation()).GetLocalMatrix();
	const FMatrix Delta = ConstraintDragStart.TargetInverse * TargetWorld;
	const float Scale = ConstraintDragStart.Scale > 0.0f ? ConstraintDragStart.Scale : 1.0f;

	// 새 월드 프레임 → 본 로컬. 위치는 그리기 때 컴포넌트 스케일을 곱했으므로 다시 나눈다.
	const auto ToLocal = [&](const FMatrix& StartWorld, const FMatrix& BoneInverse)
	{
		const FMatrix Local = StartWorld * Delta * BoneInverse;
		return FTransform(MatrixToRotator(Local), FrameOrigin(Local) / Scale);
	};
	Instance.SetRefFrame(EConstraintFrame::Frame1, ToLocal(ConstraintDragStart.Frame1World, ConstraintDragStart.ChildBoneInverse));
	Instance.SetRefFrame(EConstraintFrame::Frame2, ToLocal(ConstraintDragStart.Frame2World, ConstraintDragStart.ParentBoneInverse));
}

void FPhysicsAssetEditor::SyncBodyGizmo()
{
	// 관절을 골랐으면 부모 쪽 Frame2에 기즈모를 둔다. UE처럼 크기 모드에서는 기즈모가 없다.
	if (SelectedConstraintIndex >= 0)
	{
		const UPhysicsConstraintTemplate* Template = (PhysicsAsset && SelectedConstraintIndex < PhysicsAsset->ConstraintSetup.Num())
		                                                 ? PhysicsAsset->ConstraintSetup[SelectedConstraintIndex]
		                                                 : nullptr;
		FMatrix Frame1, Frame2;
		if (!BodyGizmoTarget || !Template || !MeshComponent || BodyGizmo.GetMode() == EGizmoMode::Scale ||
		    !GetConstraintWorldFrames(*MeshComponent, Template->DefaultInstance, Frame1, Frame2))
		{
			BodyGizmo.SetTarget(nullptr);
			return;
		}
		if (BodyGizmo.IsUsing())
			return;
		BodyGizmoTarget->SetTransform(FTransform(MatrixToRotator(Frame2), FrameOrigin(Frame2)));
		BodyGizmo.SetTarget(BodyGizmoTarget);
		return;
	}

	const USkeletalBodySetup* Setup = (PhysicsAsset && SelectedBodyIndex >= 0 && SelectedBodyIndex < PhysicsAsset->SkeletalBodySetups.Num())
	                                      ? PhysicsAsset->SkeletalBodySetups[SelectedBodyIndex]
	                                      : nullptr;
	const int32 BoneIndex = Setup && MeshComponent ? MeshComponent->FindBoneIndex(Setup->BoneName.ToString()) : -1;
	if (!BodyGizmoTarget || BoneIndex < 0)
	{
		BodyGizmo.SetTarget(nullptr);
		return;
	}
	// 드래그 중에는 기즈모가 대상 Transform을 쥐고 있으므로 덮어쓰지 않는다.
	if (BodyGizmo.IsUsing())
		return;

	// 형상 프레임은 그리기(DrawDebug, DrawBody)와 같은 규칙: 형상 로컬(Rotation, Center) × 스케일 뺀 본 월드.
	// 선택한 형상에 둔다 (무효면 첫 형상: 구 → 캡슐 → 박스 → 컨벡스 순). 형상이 없으면 본 위치에 둔다.
	const FMatrix BoneFrame = BoneFrameWithoutScale(*MeshComponent, BoneIndex);
	const FKAggregateGeom& Geom = Setup->AggGeom;
	EPhysPrimType PrimType = SelectedPrimType;
	int32 PrimIndex = SelectedPrimIndex;
	FMatrix ShapeFrame = BoneFrame;
	if (ResolvePrim(Geom, PrimType, PrimIndex))
	{
		if (PrimType == EPhysPrimType::Sphere)
			ShapeFrame = FMatrix::MakeTranslation(Geom.SphereElems[PrimIndex].Center) * BoneFrame;
		else if (PrimType == EPhysPrimType::Sphyl)
			ShapeFrame = FTransform(Geom.SphylElems[PrimIndex].Rotation, Geom.SphylElems[PrimIndex].Center).GetLocalMatrix() * BoneFrame;
		else if (PrimType == EPhysPrimType::Box)
			ShapeFrame = FTransform(Geom.BoxElems[PrimIndex].Rotation, Geom.BoxElems[PrimIndex].Center).GetLocalMatrix() * BoneFrame;
		else if (PrimType == EPhysPrimType::Convex)
			ShapeFrame = Geom.ConvexElems[PrimIndex].Transform.GetLocalMatrix() * BoneFrame;
	}

	BodyGizmoTarget->SetTransform(FTransform(MatrixToRotator(ShapeFrame), FrameOrigin(ShapeFrame)));
	BodyGizmo.SetTarget(BodyGizmoTarget);
}

int32 FPhysicsAssetEditor::PickConstraint(const FVector2& MousePosition) const
{
	const FRay Ray = ViewportClient->DeprojectScreenToWorld(MousePosition);
	const FVector RayDirection = Ray.Direction.GetSafeNormal();
	if (RayDirection.SizeSquared() < 0.5f)
		return -1;

	// 관절 위치(자식 쪽 프레임 원점)와 광선 사이 거리로 고른다.
	// 허용 반경은 선택 구 크기(3)와, 멀리 있어도 화면에서 일정하게 잡히도록 광선 거리의 2.5% 중 큰 값.
	int32 Best = -1;
	float BestScore = 1.0f;
	for (int32 ConstraintIndex = 0; ConstraintIndex < PhysicsAsset->ConstraintSetup.Num(); ++ConstraintIndex)
	{
		const UPhysicsConstraintTemplate* Template = PhysicsAsset->ConstraintSetup[ConstraintIndex];
		FMatrix Frame1, Frame2;
		if (!Template || !GetConstraintWorldFrames(*MeshComponent, Template->DefaultInstance, Frame1, Frame2))
			continue;
		const FVector ToPoint = FrameOrigin(Frame1) - Ray.Origin;
		const float Along = ToPoint.Dot(RayDirection);
		if (Along <= 0.0f)
			continue;
		const float Distance = (ToPoint - RayDirection * Along).Size();
		const float Radius = std::max(3.0f, Along * 0.025f);
		const float Score = Distance / Radius;	// 1 미만이면 허용 반경 안. 가장 가운데 가까운 것을 고른다.
		if (Score < BestScore)
		{
			BestScore = Score;
			Best = ConstraintIndex;
		}
	}
	return Best;
}

int32 FPhysicsAssetEditor::PickBody(const FVector2& MousePosition, float* OutDistance, EPhysPrimType* OutPrimType, int32* OutPrimIndex) const
{
	// 레벨 에디터 피킹과 같은 마우스 광선 (원근·직교 모두 처리)
	const FRay Ray = ViewportClient->DeprojectScreenToWorld(MousePosition);
	const FVector RayStart = Ray.Origin;
	const FVector RayDirection = Ray.Direction.GetSafeNormal();
	if (RayDirection.SizeSquared() < 0.5f)
		return -1;

	// 그리기(UBodySetup::DrawDebug)와 같은 규칙으로 형상을 놓고 가장 가까운 것을 고른다.
	const FVector Scale = MeshComponent->GetWorldScale3D();
	int32 BestBody = -1;
	EPhysPrimType BestPrimType = EPhysPrimType::None;
	int32 BestPrimIndex = -1;
	float BestDistance = FLT_MAX;
	for (int32 BodyIndex = 0; BodyIndex < PhysicsAsset->SkeletalBodySetups.Num(); ++BodyIndex)
	{
		const USkeletalBodySetup* Setup = PhysicsAsset->SkeletalBodySetups[BodyIndex];
		if (!Setup)
			continue;
		const int32 BoneIndex = MeshComponent->FindBoneIndex(Setup->BoneName.ToString());
		if (BoneIndex < 0)
			continue;
		const FMatrix BoneFrame = BoneFrameWithoutScale(*MeshComponent, BoneIndex);

		const auto Consider = [&](float Distance, EPhysPrimType PrimType, int32 PrimIndex)
		{
			if (Distance >= 0.0f && Distance < BestDistance)
			{
				BestDistance = Distance;
				BestBody = BodyIndex;
				BestPrimType = PrimType;
				BestPrimIndex = PrimIndex;
			}
		};
		// 형상 로컬 공간으로 광선을 옮긴다. 프레임은 스케일이 없으므로 거리가 그대로 유지된다.
		const auto ToLocal = [&](const FMatrix& Local, FVector& OutOrigin, FVector& OutDirection)
		{
			const FMatrix Inverse = Local.Inverse();
			OutOrigin = Inverse.TransformPosition(RayStart);
			OutDirection = Inverse.TransformVector(RayDirection).GetSafeNormal();
		};

		const FKAggregateGeom& Geom = Setup->AggGeom;
		for (int32 Index = 0; Index < Geom.SphereElems.Num(); ++Index)
		{
			const FKSphereElem& Sphere = Geom.SphereElems[Index];
			if (Sphere.Radius > 0)
				Consider(RaySphere(RayStart, RayDirection, BoneFrame.TransformPosition(Sphere.Center), Sphere.Radius * Scale.X), EPhysPrimType::Sphere, Index);
		}
		for (int32 Index = 0; Index < Geom.BoxElems.Num(); ++Index)
		{
			const FKBoxElem& Box = Geom.BoxElems[Index];
			if (Box.X <= 0 || Box.Y <= 0 || Box.Z <= 0)
				continue;
			FVector Origin, Direction;
			ToLocal(FTransform(Box.Rotation, Box.Center).GetLocalMatrix() * BoneFrame, Origin, Direction);
			Consider(RayBox(Origin, Direction, FVector(Box.X * 0.5f, Box.Y * 0.5f, Box.Z * 0.5f)), EPhysPrimType::Box, Index);
		}
		for (int32 Index = 0; Index < Geom.SphylElems.Num(); ++Index)
		{
			const FKSphylElem& Capsule = Geom.SphylElems[Index];
			if (Capsule.Radius <= 0 || Capsule.Length < 0)
				continue;
			FVector Origin, Direction;
			ToLocal(FTransform(Capsule.Rotation, Capsule.Center).GetLocalMatrix() * BoneFrame, Origin, Direction);
			Consider(RayCapsule(Origin, Direction, Capsule.Length * 0.5f * Scale.X, Capsule.Radius * Scale.X), EPhysPrimType::Sphyl, Index);
		}
	}
	if (OutDistance)
		*OutDistance = BestDistance;
	if (OutPrimType)
		*OutPrimType = BestPrimType;
	if (OutPrimIndex)
		*OutPrimIndex = BestPrimIndex;
	return BestBody;
}

TUniquePtr<IEditorTab> FPhysicsAssetEditor::CreateTab()
{
	return MakeUnique<FPhysicsAssetEditorTab>(*this);
}

bool FPhysicsAssetEditor::IsDirty() const
{
	return PhysicsAsset && PhysicsAsset->IsDirty();
}

bool FPhysicsAssetEditor::ConfirmClose()
{
	if (!IsDirty() || bCloseConfirmed)
		return true;
	bClosePrompt = true;
	return false;
}

void FPhysicsAssetEditor::DrawClosePrompt()
{
	if (bClosePrompt)
	{
		ImGui::OpenPopup("Unsaved Physics Asset");
		bClosePrompt = false;
		bCloseSaveFailed = false;
	}
	if (!ImGui::BeginPopupModal("Unsaved Physics Asset", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
		return;
	ImGui::TextUnformatted("Save physics asset changes before closing?");
	if (ImGui::Button("Save and Close"))
	{
		bCloseSaveFailed = !Save();
		if (!bCloseSaveFailed)
		{
			ImGui::CloseCurrentPopup();
			Close();
		}
	}
	ImGui::SameLine();
	if (ImGui::Button("Discard"))
	{
		// 편집 대상은 이 에디터가 파일에서 읽은 사본이므로 저장 없이 닫으면 변경이 버려진다.
		bCloseConfirmed = true;
		ImGui::CloseCurrentPopup();
		Close();
	}
	ImGui::SameLine();
	if (ImGui::Button("Cancel"))
		ImGui::CloseCurrentPopup();
	if (bCloseSaveFailed)
		ImGui::TextUnformatted("Could not save the physics asset.");
	ImGui::EndPopup();
}
