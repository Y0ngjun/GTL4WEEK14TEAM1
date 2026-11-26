#pragma once

#include "Editor/AssetEditor/AssetEditorPanel.h"
#include "Editor/PhysicsAssetEditor/PhysicsAssetEditor.h"

// UE Physics Asset Editor의 Details. 바디와 관절(Constraint·Angular Limits)을 보여 준다.
// UE와 같은 카테고리 이름으로 기본 항목만 둔다:
//   Physics    - Physics Type
//   Body Setup - Primitives(형상 위치·회전·크기), Bone Name(읽기 전용)
// 아무것도 안 고르면 에셋 정보를 보여 준다. 값을 바꾸면 에셋을 Dirty로 만들고 실행 바디를 다시 만든다.
class FPhysicsAssetDetailsPanel : public TAssetEditorPanel<FPhysicsAssetEditor>
{
public:
	explicit FPhysicsAssetDetailsPanel(FPhysicsAssetEditor& InEditor);

protected:
	void DrawContent() override;

private:
	void DrawAssetInfo(const UPhysicsAsset& Asset);
	void DrawBodyDetails(UPhysicsAsset& Asset, int32 BodyIndex);
	// 관절: Constraint(이름·자식/부모 본, 읽기 전용), Angular Limits(Swing1·Swing2·Twist의 Motion·각도)
	void DrawConstraintDetails(UPhysicsAsset& Asset, int32 ConstraintIndex);
};
