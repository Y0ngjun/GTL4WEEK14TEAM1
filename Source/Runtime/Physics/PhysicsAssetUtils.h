#pragma once

#include "Core/Types.h"
#include "Physics/ConstraintInstance.h"

class UPhysicsAsset;
class USkeletalMesh;

// UE EPhysAssetFitGeomType. 본마다 만들 형상 (순서도 UE와 같다). Convex는 추후 추가.
enum class EPhysAssetFitGeomType : uint8
{
	EFG_Box,
	EFG_Sphyl,	// Capsule
	EFG_Sphere,
};

// UE FPhysAssetCreateParams의 축약판
struct FPhysAssetCreateParams
{
	// 본마다 만들 형상 (UE 기본 Capsule)
	EPhysAssetFitGeomType GeomType = EPhysAssetFitGeomType::EFG_Sphyl;
	// 본에 가중치가 실린 정점 범위 상자의 반 대각선 길이가 이 값 이하인 본은 바디를 만들지 않고 부모 바디에 합친다 (cm).
	// UE 기본값 20cm
	float MinBoneSize = 20.0f;
	// 생성되는 관절의 기본 각도 한계 (도). UE처럼 생성 창에서는 받지 않고 FConstraintInstance 기본값(45)을 쓴다.
	float SwingLimitDegrees = 45.0f;
	float TwistLimitDegrees = 45.0f;
	bool bCreateConstraints = true;
	// 생성되는 관절의 Swing1·Swing2·Twist 모드 (UE AngularConstraintMode, 기본 Limited)
	EAngularConstraintMotion AngularConstraintMode = ACM_Limited;
	// 바인드 포즈에서 서로 겹치는 바디끼리 충돌을 끈다 (겹친 채 시작하면 래그돌이 튄다).
	bool bDisableCollisionsOfOverlappingBodies = true;
};

namespace FPhysicsAssetUtils
{
	// UE FPhysicsAssetUtils::CreateFromSkeletalMesh. 기존 내용을 지우고 본마다 캡슐 바디와 부모 바디로의 D6 관절을 만든다.
	// 바디 형상은 본 공간에서 그 본이 가장 크게 영향을 주는 정점들의 범위에 맞춘다. 실패하면 false.
	bool CreateFromSkeletalMesh(UPhysicsAsset& PhysicsAsset, const USkeletalMesh& Mesh, const FPhysAssetCreateParams& Params = FPhysAssetCreateParams());

	// UE Tools > Add Bodies (FPhysicsAssetEditor::MakeNewBody). 본 하나에 메시에 맞춘 바디를 만든다 (있으면 형상만 다시 맞춘다).
	// bCreateConstraints면 가장 가까운 조상 바디로 관절을 잇고, 이 바디 아래로 들어온 자식 바디들의 관절도 이 바디로 다시 잇는다.
	// 맞출 정점이 없으면 -1. 성공하면 바디 번호.
	int32 CreateNewBody(UPhysicsAsset& PhysicsAsset, const USkeletalMesh& Mesh, int32 BoneIndex, const FPhysAssetCreateParams& Params);
	// UE 트리 우클릭 Add Primitive (AddNewPrimitive). 본에 바디가 없으면 만들고, 본 원점에 기본 크기 형상을 하나 더한다. 관절은 만들지 않는다.
	int32 AddPrimitive(UPhysicsAsset& PhysicsAsset, const FName& BoneName, EPhysAssetFitGeomType GeomType);
	// UE Create New Constraint With (MakeNewConstraint). 두 바디를 관절로 잇는다: 이름 UserConstraint_N, 자식 본 원점에 맞추고 충돌 끔.
	// 같은 두 바디 관절이 이미 있으면 그 번호. 실패하면 -1.
	int32 CreateNewConstraint(UPhysicsAsset& PhysicsAsset, const USkeletalMesh& Mesh, int32 ParentBodyIndex, int32 ChildBodyIndex, const FPhysAssetCreateParams& Params);
}
