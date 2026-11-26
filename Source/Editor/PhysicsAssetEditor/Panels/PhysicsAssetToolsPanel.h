#pragma once

#include "Editor/AssetEditor/AssetEditorPanel.h"
#include "Editor/PhysicsAssetEditor/PhysicsAssetEditor.h"

// UE Physics Asset Editor의 Tools 탭. 생성 옵션(Body Creation / Constraint Creation)을 고르고
// Generate All Bodies로 현재 에셋의 바디·관절을 프리뷰 메시 기준으로 모두 다시 만든다.
class FPhysicsAssetToolsPanel : public TAssetEditorPanel<FPhysicsAssetEditor>
{
public:
	explicit FPhysicsAssetToolsPanel(FPhysicsAssetEditor& InEditor);

protected:
	void DrawContent() override;
};
