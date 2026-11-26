#pragma once

#include "Editor/SkeletalPreviewEditor/SkeletalPreviewEditor.h"
#include "Editor/Gizmo/Gizmo.h"
#include "Physics/PhysicsAssetUtils.h"

class UPhysicsAsset;
class FPhysicsBodyRenderer;
namespace physx
{
	class PxRigidDynamic;
	class PxD6Joint;
}

// 바디 안의 형상(Primitive) 종류. 형상은 (종류, 종류별 배열 인덱스)로 가리킨다.
enum class EPhysPrimType : uint8
{
	None,
	Sphere,
	Sphyl,
	Box,
	Convex,
};

// UE Physics Asset Editor. .physicsasset 파일 하나를 열고, 그 에셋의 미리보기 스켈레탈 메시를 뷰포트에 띄운다.
// 지금은 메시 미리보기와 개수 표시까지만 있다 (바디·관절 그리기, 본 트리, 디테일은 다음 단계).
class FPhysicsAssetEditor : public FSkeletalPreviewEditor
{
public:
	FPhysicsAssetEditor(UEditorEngine& Engine, FEditorManager& Manager, const FString& Path);
	~FPhysicsAssetEditor() override;

	static bool IsPhysicsAssetPath(const FString& Path);

	const char* GetEditorTypeName() const override { return "Physics Asset"; }
	bool UsesPreviewSkybox() const override { return true; }
	bool IsDirty() const override;
	// 저장 안 한 변경이 있으면 닫기를 멈추고 확인 창(Save and Close / Discard / Cancel)을 띄운다 (다른 에셋 에디터와 같다).
	bool ConfirmClose() override;
	void DrawClosePrompt();
	// 편집 중인 에셋을 열었던 파일에 저장한다. 그 파일이 메시에 연결된 에셋이면 메시 쪽 에셋도 다시 읽어 실행 바디에 반영한다.
	bool Save();
	// 부모가 그리는 본(기본은 꺼 둠) 위에 바디 형상을 그린다.
	void OnRenderLine(FLineBatcher& LineBatcher) override;
	// 바디를 반투명 면으로 칠한다 (선은 그 위에 테두리로 그린다).
	void OnRenderOverlay(const FSceneView& View) override;
	// 선택한 바디의 첫 형상 위에 기즈모를 띄운다. 드래그하면 형상의 위치(Center)·회전(Rotation)·크기를 바꾼다.
	const FGizmo* GetPreviewGizmo() const override { return BodyGizmo.GetTarget() ? &BodyGizmo : nullptr; }

	UPhysicsAsset* GetPhysicsAsset() const { return PhysicsAsset; }
	const FString& GetPreviewMeshPath() const { return PreviewMeshPath; }

	// Tools 패널의 생성 옵션. Generate All Bodies가 이 값으로 바디·관절을 모두 다시 만든다 (UE Tools > Generate All Bodies).
	FPhysAssetCreateParams& GetCreateParams() { return CreateParams; }
	bool GenerateAllBodies();
	// UE Tools > Add Bodies / Re-generate Bodies: 선택한 본(바디를 골랐으면 그 바디의 본)에 메시에 맞춘 바디를 만들고 고른다.
	bool AddBodyToSelectedBone();
	// UE 트리 우클릭 Add Primitive: 본에 기본 형상을 더한다 (바디가 없으면 만든다). 관절은 만들지 않는다.
	bool AddPrimitive(int32 BoneIndex, EPhysAssetFitGeomType GeomType);
	// UE Delete: 바디 번호가 바뀌므로 시뮬레이션 중에는 막는다. DeleteBody는 바디와 거기 붙은 관절을 모두 지운다.
	bool DeleteBody(int32 BodyIndex);
	// Delete 키: 관절을 골랐으면 그 관절, 아니면 고른 형상을 지운다 (바디의 마지막 형상이면 바디째 지운다).
	bool DeleteSelected();
	// UE 바디 우클릭 Create New Constraint With: ParentBody(우클릭한 바디)와 ChildBody(고른 바디)를 관절로 잇고 고른다.
	bool CreateConstraintBetween(int32 ParentBodyIndex, int32 ChildBodyIndex);

	// UE Simulate: 늘 있는 키네마틱 바디를 시뮬레이션으로 바꿔 래그돌로 떨어뜨린다. 끄면 바인드 포즈로 돌아온다.
	// 시뮬레이션 중에는 기즈모·피킹을 쓰지 않는다.
	bool IsSimulating() const { return bSimulating; }
	void ToggleSimulation();

	// 트리·뷰포트·디테일이 함께 보는 선택. 바디와 관절 중 하나만 고른다 (본만 고르면 둘 다 -1).
	int32 GetSelectedBodyIndex() const { return SelectedBodyIndex; }
	int32 GetSelectedConstraintIndex() const { return SelectedConstraintIndex; }
	// 바디를 고른다. PrimType/PrimIndex가 유효하면 그 형상을, 아니면 첫 형상(구 → 캡슐 → 박스 → 컨벡스)을 고른다.
	void SelectBody(int32 BodyIndex, EPhysPrimType PrimType = EPhysPrimType::None, int32 PrimIndex = -1);
	EPhysPrimType GetSelectedPrimType() const { return SelectedPrimType; }
	int32 GetSelectedPrimIndex() const { return SelectedPrimIndex; }
	void SelectConstraint(int32 ConstraintIndex);
	// 바디·관절이 붙은 본 번호 (없으면 -1)
	int32 GetBodyBoneIndex(int32 BodyIndex) const;
	int32 GetConstraintBoneIndex(int32 ConstraintIndex) const;

	// W/E/R 또는 뷰포트 단축키로 바꾼다 (스켈레탈 메시 에디터와 같다).
	void SetGizmoMode(EGizmoMode Mode);
	EGizmoMode GetGizmoMode() const { return BodyGizmo.GetMode(); }
	bool IsGizmoDragging() const { return BodyGizmo.IsUsing(); }
	// 뷰포트 왼쪽 위 이동/회전/크기 버튼
	int32 GetViewportTransformMode() const override { return static_cast<int32>(BodyGizmo.GetMode()); }
	void SetViewportTransformMode(int32 Mode) override { SetGizmoMode(static_cast<EGizmoMode>(Mode)); }
	// 뷰포트 툴바 왼쪽 Simulate / Stop 버튼 (UE 물리 에셋 에디터 뷰포트의 Simulate)
	bool DrawViewportToolbarLeft() override;

protected:
	bool InitializeResources() override;
	void ShutdownResources() override;
	TUniquePtr<IEditorTab> CreateTab() override;
	// 본을 고르면(트리·뷰포트) 바디·관절 선택은 푼다.
	void OnBoneSelected() override;
	// 뷰포트 클릭으로 바디를 고른다 (본 그리기가 켜져 있고 바디가 안 맞으면 본을 고른다).
	void TickAfterWorld(float DeltaTime) override;
	// 기즈모 드래그나 시뮬레이션 잡기 중에는 카메라를 움직이지 않는다.
	bool IsViewportInteractionBusy() const override { return BodyGizmo.IsUsing() || SimGrab.BodyIndex >= 0; }
	void CancelViewportInteraction() override { BodyGizmo.EndDrag(); }

private:
	// 마우스 위치에서 쏜 광선에 가장 먼저 맞는 바디 번호 (없으면 -1). Convex는 아직 고르지 않는다.
	// OutDistance: 광선 시작점에서 맞은 지점까지 거리 (맞은 지점 = 시작점 + 방향 × 거리)
	// OutPrimType/OutPrimIndex: 맞은 바디 안에서 광선에 가장 먼저 맞은 형상
	int32 PickBody(const FVector2& MousePosition, float* OutDistance = nullptr, EPhysPrimType* OutPrimType = nullptr, int32* OutPrimIndex = nullptr) const;
	// 마우스 광선에 가장 가까운 관절 번호 (자식 쪽 프레임 위치 기준, 없으면 -1). 관절은 바디 안에 있으므로 바디보다 먼저 고른다.
	int32 PickConstraint(const FVector2& MousePosition) const;
	void UpdateBodyPicking();
	// 선택한 바디의 첫 형상 위치·회전으로 기즈모 대상을 옮긴다. 선택이 없으면 기즈모를 끈다.
	void SyncBodyGizmo();
	// 기즈모 입력을 처리한다. 기즈모가 마우스를 썼으면 true (그 프레임은 바디 피킹을 하지 않는다).
	bool UpdateBodyGizmo();
	// 드래그 중인 기즈모 대상 Transform을 형상 값으로 되돌려 쓴다.
	void ApplyBodyGizmo();
	// 관절 기즈모 (UE와 같이 이동·회전만, 부모 쪽 Frame2 위치에 띄운다). 드래그하면 두 프레임을 함께 옮긴다.
	bool UpdateConstraintGizmo();
	void ApplyConstraintGizmo();
	// 뷰포트가 포커스일 때 W/E/R로 기즈모 모드를 바꾼다.
	void UpdateGizmoModeKeys();
	// 뷰포트가 포커스일 때 Delete 키로 선택을 지운다.
	void UpdateDeleteKey();

	// 미리보기 월드 물리를 켜고 바닥과 키네마틱 바디를 만든다 (에디터를 열 때 한 번).
	void InitializePreviewPhysics();
	bool StartSimulation();
	void StopSimulation();
	// 시뮬레이션 중 Ctrl+우클릭 드래그로 바디 잡기 (UE SimMousePress). 누르면 맞은 바디·지점을 찾고, 떼면 놓는다.
	void UpdateSimulationGrab(float DeltaTime);
	void ReleaseSimulationGrab();

	// 에셋에 저장된 미리보기 메시 경로. 없으면(이전 파일) 과도기 저장 경로에서 메시 경로를 되찾는다.
	FString ResolvePreviewMeshPath() const;

	// 파일에서 읽은 편집 대상. 이 에디터가 소유한다.
	UPhysicsAsset* PhysicsAsset = nullptr;
	FString PreviewMeshPath;
	FPhysAssetCreateParams CreateParams;
	// 처음 그릴 때 만든다 (셰이더·기본 메시가 준비된 뒤)
	TUniquePtr<FPhysicsBodyRenderer> BodyRenderer;
	int32 SelectedBodyIndex = -1;
	// 선택한 바디 안의 형상. 기즈모는 이 형상에 붙고 이 형상만 편집한다.
	EPhysPrimType SelectedPrimType = EPhysPrimType::None;
	int32 SelectedPrimIndex = -1;
	int32 SelectedConstraintIndex = -1;
	// 기즈모가 따라갈 빈 컴포넌트 (스켈레탈 메시 에디터의 BoneGizmoTarget과 같은 방식)
	USceneComponent* BodyGizmoTarget = nullptr;
	FGizmo BodyGizmo;
	// UE 미리보기 씬 바닥: 보이는 판(FloorMeshComponent)과 같은 크기의 충돌 박스(FloorComponent). 메시 발밑에 둔다.
	class UBoxComponent* FloorComponent = nullptr;
	class UStaticMeshComponent* FloorMeshComponent = nullptr;
	bool bSimulating = false;
	// 드래그 궤적: 손 목표 위치를 매 프레임 찍고 GrabTrailLifetime 동안 옅어지며 그린다 (UE DrawDebugPoint 0.3초).
	struct FGrabTrailPoint
	{
		FVector Location;
		float Age = 0.0f;
	};
	TArray<FGrabTrailPoint> GrabTrail;
	static constexpr float GrabTrailLifetime = 0.3f;
	static constexpr float GrabTrailPointRadius = 1.5f;
	// UE PhysicsHandle InterpolationSpeed 기본값. 클수록 손이 마우스를 빨리 따라간다.
	static constexpr float GrabInterpolationSpeed = 50.0f;
	// 닫기 확인 창: 띄울 차례인지, Discard로 저장 없이 닫기로 했는지, 저장이 실패했는지
	bool bClosePrompt = false;
	bool bCloseConfirmed = false;
	bool bCloseSaveFailed = false;
	// Simulate 시작 때 메시 컴포넌트 월드 행렬. Stop에서 이 위치로 되돌린다 (UE ResetTM).
	FMatrix SimulationResetTransform = FMatrix::Identity;
	// 잡은 바디(-1이면 잡지 않음)와 잡은 지점(월드). 드래그는 이 지점을 지나는 화면 평행 평면 위에서 한다.
	// UE PhysicsHandle처럼 잡은 지점에 보이지 않는 키네마틱 액터(손)를 두고, 스프링 관절로 바디의 그 지점과 잇는다.
	struct FSimulationGrab
	{
		int32 BodyIndex = -1;
		FVector Location;
		// 마우스가 가리키는 목표와 손의 실제 위치. 손은 목표로 보간해 다가간다 (UE TargetTransform / CurrentTransform).
		FVector TargetLocation;
		FVector CurrentLocation;
		physx::PxRigidDynamic* Handle = nullptr;
		physx::PxD6Joint* Joint = nullptr;
	} SimGrab;
	// 드래그 시작 때의 형상 값. 크기 배율은 이 값에 곱하고, Esc를 누르면 이 값으로 되돌린다.
	struct FShapeDragStart
	{
		int32 BodyIndex = -1;
		FMatrix BoneFrameInverse;
		FVector Center;
		FRotator Rotation;
		FTransform ConvexTransform;
		float Radius = 0.0f;
		float Length = 0.0f;
		FVector BoxSize;
	} DragStart;
	// 관절 드래그 시작 값. 두 프레임의 시작 월드 행렬과 기즈모 시작 행렬의 역으로 드래그 변화량을 두 프레임에 곱한다.
	struct FConstraintDragStart
	{
		int32 ConstraintIndex = -1;
		FTransform Frame1Local;
		FTransform Frame2Local;
		FMatrix Frame1World;
		FMatrix Frame2World;
		FMatrix ChildBoneInverse;
		FMatrix ParentBoneInverse;
		FMatrix TargetInverse;
		float Scale = 1.0f;
	} ConstraintDragStart;
};
