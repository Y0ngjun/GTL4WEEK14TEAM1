#pragma once

#include "Editor/SkeletalPreviewEditor/SkeletalPreviewEditor.h"
#include "Editor/Gizmo/Gizmo.h"
#include "Editor/Viewports/EditorViewportClient.h"
#include "ClothPaintTool.h"
#include "Physics/Cloth/ClothVirtualParticles.h"

#include <optional>

struct FClothingAsset;
struct FClothingLODData;

class FSkeletalMeshEditor : public FSkeletalPreviewEditor
{
public:
	FSkeletalMeshEditor(UEditorEngine& InEditorEngine, FEditorManager& InEditorManager,
	                    const FString& InAssetPath, USkeletalMesh& InMesh)
	    : FSkeletalPreviewEditor(InEditorEngine, InEditorManager, InAssetPath)
	{
		Mesh = &InMesh;
	}

	const char* GetEditorTypeName() const override { return "SkeletalMesh"; }
	bool UsesPreviewSkybox() const override { return true; }
	bool IsDirty() const override;
	bool Save();
	bool RevertToSaved();
	const FString& GetAssetMessage() const { return AssetMessage; }
	const FGizmo* GetPreviewGizmo() const override { return !IsClothPaintMode() && BoneGizmo.GetTarget() ? &BoneGizmo : nullptr; }

	// 선택한 본의 프리뷰 포즈를 부모 기준 로컬 변환으로 조회·수정합니다.
	bool GetSelectedBoneLocalTransform(FTransform& OutTransform) const;
	void SetSelectedBoneLocalTransform(const FTransform& Transform);
	void ResetPose();
	void ResetBone(int32 BoneIndex);

	void CommitBonePoseUndo(bool bInteractionActive);
	void UndoPoseEdit();
	void RedoPoseEdit();
	bool CanUndoPoseEdit() const { return !PoseUndoStack.IsEmpty(); }
	bool CanRedoPoseEdit() const { return !PoseRedoStack.IsEmpty(); }

	void SetGizmoMode(EGizmoMode Mode);
	void SetGizmoSpace(EGizmoSpace Space);
	EGizmoMode GetGizmoMode() const { return BoneGizmo.GetMode(); }
	EGizmoSpace GetGizmoSpace() const { return BoneGizmo.GetSpace(); }
	bool IsGizmoDragging() const { return BoneGizmo.IsUsing(); }

	// 에셋 브라우저에서 선택한 애니메이션 시퀀스 에디터를 연다.
	void OpenAnimationAsset(const FString& InAssetPath);

	bool IsClothPreviewEnabled() const;
	void SetClothPreviewEnabled(bool bEnabled);
	void RefreshClothPreview();
	FClothVirtualParticleSettings GetClothVirtualParticleSettingsForEdit(const FClothingAsset& Asset) const;
	void StageClothVirtualParticleSettings(const FString& AssetId, FClothVirtualParticleSettings Settings);
	bool CommitClothVirtualParticleSettings();
	TSharedPtr<const FClothVirtualParticleData> GetClothVirtualParticlesForDetails(
	    int32 Section, const FClothingLODData& LOD) const;
	void OnRenderLine(FLineBatcher& Lines) override;
	bool HasDepthTestedOverlay() const override { return IsClothPaintMode(); }
	void OnRenderDepthTestedOverlay(const FSceneView& View) override;
	void OpenCapsuleCapeFixture();
	void OpenPaintedCapeFixture(bool bReference);

	int32 GetSelectedClothSection() const { return SelectedClothSection; }
	void SetSelectedClothSection(int32 Section);
	void ClothBindingChanged(int32 Section);
	bool CanPaintCloth() const;
	bool IsClothPaintMode() const { return ClothPaint.IsActive(); }
	void SetClothPaintMode(bool bEnabled);
	FClothPaintSettings& GetClothPaintSettings() { return ClothPaint.Settings; }
	void SetClothPaintTool(EClothPaintTool Tool) { ClothPaint.SetTool(Tool); }
	void SetClothPaintInput(bool bShift, bool bCtrl, bool bConfirm);
	bool GetHoveredClothValue(float& OutValue) const { return ClothPaint.GetHoveredValue(OutValue); }
	bool CanApplyClothGradient() const { return ClothPaint.CanApplyGradient(); }
	void ApplyClothGradient();
	void FillClothFromSelectedBone(bool bClear = false);
	bool IsClothPainting() const { return ClothPaint.IsPainting(); }
	bool CanUndoEdit(bool bRedo) const;
	void UndoEdit(bool bRedo);

protected:
	// 이 에디터에 연결된 탭을 생성합니다.
	TUniquePtr<IEditorTab> CreateTab() override;
	bool InitializeResources() override; // InitializePreview() 후 BoneGizmoTarget 생성
	void ShutdownResources() override;   // 기즈모 정리 후 Super

	void TickAfterWorld(float DeltaTime) override;
	bool IsViewportInteractionBusy() const override { return BoneGizmo.IsUsing() || ClothPaint.IsPainting(); }
	void OnBoneSelected() override
	{
		BoneGizmo.EndDrag();
		SyncBoneGizmo();
	}
	void CancelViewportInteraction() override { BoneGizmo.EndDrag(); ClothPaint.CancelStroke(); }

private:
	void SyncBoneGizmo();
	bool UpdateBoneGizmo();
	void ApplyBoneGizmoTransform();
	FQuat GetBoneWorldRotation(int32 BoneIndex) const;
	TArray<FTransform> CaptureBonePose() const;
	void ApplyBonePose(const TArray<FTransform>& Pose);
	void ResetBonePoseHistory();
	void TickClothPaint();

public:
	bool bShowClothPins = false;
	bool bShowClothWire = false;
	bool bShowClothCollision = false;
	bool bShowClothVirtualParticles = false;

private:
	struct FPendingVirtualParticleEdit
	{
		FString AssetId;
		FClothVirtualParticleSettings Settings;
	};

	std::optional<FPendingVirtualParticleEdit> PendingVirtualParticleEdit;
	USceneComponent* BoneGizmoTarget = nullptr;
	FGizmo BoneGizmo;
	FTransform DragStartBoneTransform;
	FMatrix DragParentInverse;
	FQuat DragParentRotation;

	TArray<TArray<FTransform>> PoseUndoStack;
	TArray<TArray<FTransform>> PoseRedoStack;
	TArray<FTransform> CommittedBonePose;
	FString AssetMessage;
	bool bPoseUndoPending = false;
	int32 SelectedClothSection = INDEX_NONE;
	FClothPaintTool ClothPaint;
	EViewportMode PreviousClothPaintViewMode = EViewportMode::Solid;
	bool bRestoreClothSimulation = false;
	bool bRestoreMeshVisible = true;
	bool bRestoreWeightHeatmap = false;
	bool bPaintShift = false;
	bool bPaintCtrl = false;
	bool bPaintConfirm = false;

	static constexpr int32 MaxUndoSteps = 100;
};
