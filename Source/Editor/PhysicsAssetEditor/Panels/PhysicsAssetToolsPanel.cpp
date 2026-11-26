#include "EnginePCH.h"
#include "PhysicsAssetToolsPanel.h"

#include "Editor/PhysicsAssetEditor/PhysicsAssetCreator.h"
#include "imgui.h"

FPhysicsAssetToolsPanel::FPhysicsAssetToolsPanel(FPhysicsAssetEditor& InEditor)
	: TAssetEditorPanel<FPhysicsAssetEditor>(InEditor, "Tools", "PhysicsAssetPanel")
{
}

void FPhysicsAssetToolsPanel::DrawContent()
{
	FPhysicsAssetCreator::DrawCreateParams(Editor.GetCreateParams());

	ImGui::Spacing();
	// UE처럼 오른쪽 아래에 버튼을 둔다. 프리뷰 메시가 없으면 만들 수 없다.
	// 선택에 따라 버튼이 바뀐다 (UE): 바디 선택 = Re-generate Bodies, 본만 선택 = Add Bodies, 그 밖 = Generate All Bodies.
	const bool bBodySelected = Editor.GetSelectedBodyIndex() >= 0;
	const bool bBoneOnly = !bBodySelected && Editor.GetSelectedConstraintIndex() < 0 && Editor.GetSelectedBoneIndex() >= 0;
	const bool bSingle = bBodySelected || bBoneOnly;
	const char* Label = bBodySelected ? "Re-generate Bodies" : (bBoneOnly ? "Add Bodies" : "Generate All Bodies");
	const float Width = ImGui::CalcTextSize(Label).x + ImGui::GetStyle().FramePadding.x * 2.0f;
	ImGui::SetCursorPosX(std::max(ImGui::GetCursorPosX(), ImGui::GetContentRegionMax().x - Width));
	ImGui::BeginDisabled(!Editor.GetMesh() || !Editor.GetPhysicsAsset() || (bSingle && Editor.IsSimulating()));
	if (ImGui::Button(Label))
	{
		if (bSingle)
			Editor.AddBodyToSelectedBone();
		else
			Editor.GenerateAllBodies();
	}
	ImGui::EndDisabled();
}
