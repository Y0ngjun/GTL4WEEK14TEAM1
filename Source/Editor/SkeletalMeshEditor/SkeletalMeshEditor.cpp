#include "EnginePCH.h"
#include "Editor/SkeletalMeshEditor/SkeletalMeshEditor.h"
#include "Editor/AssetEditor/EditorManager.h"
#include "SkeletalMeshEditorTab.h"
#include "Asset/AssetManager.h"
#include "Asset/FbxImporter/FbxMeshBake.h"
#include "Components/SkeletalMeshComponent.h"
#include "Core/Misc/Paths.h"
#include "Physics/PhysicsAsset.h"
#include "Render/SkeletalMesh.h"
#include "Render/LineBatcher.h"
#include "Engine/World.h"
#include "UObject/UObjectGlobals.h"

#include <algorithm>
#include <utility>
#include <vector>

namespace
{
	void DrawTriangle(FLineBatcher& Lines, const FVector& A, const FVector& B,
	                  const FVector& C, const FVector4& Color)
	{
		Lines.AddLine(A, B, Color);
		Lines.AddLine(B, C, Color);
		Lines.AddLine(C, A, Color);
	}

	void DrawClothSectionHighlight(FLineBatcher& Lines, const USkeletalMeshComponent& Component,
	                               const FSkeletalMeshLODRenderData& LOD, const FSkeletalMeshSection& Section,
	                               const FMatrix& ToWorld)
	{
		const uint64 EndIndex = static_cast<uint64>(Section.StartIndex) + Section.IndexCount;
		if (EndIndex > static_cast<uint64>(LOD.Indices.Num()))
			return;

		FClothingLODData PoseInput;
		PoseInput.RestPositions.Reserve(Section.IndexCount);
		PoseInput.SkinWeights.Reserve(Section.IndexCount);
		for (uint32 Index = Section.StartIndex; Index < EndIndex; ++Index)
		{
			const uint32 Vertex = LOD.Indices[Index];
			if (!LOD.BindVertices.IsValidIndex(Vertex) || !LOD.SkinWeights.IsValidIndex(Vertex))
				return;

			PoseInput.RestPositions.Add(LOD.BindVertices[Vertex].Position);
			PoseInput.SkinWeights.Add(LOD.SkinWeights[Vertex]);
		}

		TArray<FVector> Positions;
		if (!Component.BuildClothPose(PoseInput, Positions))
			return;

		const auto& ClothVertices = Component.GetClothRenderVertices();
		for (int32 Index = 0; Index < Positions.Num(); ++Index)
		{
			const uint32 Vertex = LOD.Indices[Section.StartIndex + Index];
			if (ClothVertices.IsValidIndex(Vertex) && ClothVertices[Vertex].Position.W != 0.0f)
			{
				const FVector4& Position = ClothVertices[Vertex].Position;
				Positions[Index] = FVector(Position.X, Position.Y, Position.Z);
			}
			Positions[Index] = ToWorld.TransformPosition(Positions[Index]);
		}

		const FVector4 Color(1.0f, 0.65f, 0.1f, 1.0f);
		for (int32 Index = 0; Index + 2 < Positions.Num(); Index += 3)
			DrawTriangle(Lines, Positions[Index], Positions[Index + 1], Positions[Index + 2], Color);
	}

	void DrawClothSimulation(FLineBatcher& Lines, const FClothingLODData& Cloth,
	                         TArray<FVector>& Positions, const FMatrix& ToWorld,
	                         bool bShowPins, bool bShowWire)
	{
		for (FVector& Position : Positions)
			Position = ToWorld.TransformPosition(Position);

		if (bShowPins && Positions.Num() == Cloth.MaxDistances.Num())
		{
			const FVector4 Color(1.0f, 0.2f, 0.2f, 1.0f);
			for (int32 Index = 0; Index < Positions.Num(); ++Index)
			{
				if (Cloth.MaxDistances[Index] == 0.0f)
					Lines.AddSphere(Positions[Index], 1.5f, Color, 8);
			}
		}

		if (bShowWire)
		{
			const FVector4 Color(0.2f, 0.8f, 1.0f, 1.0f);
			const auto& Indices = Cloth.TriangleIndices;
			for (int32 Index = 0; Index + 2 < Indices.Num(); Index += 3)
			{
				const uint32 A = Indices[Index];
				const uint32 B = Indices[Index + 1];
				const uint32 C = Indices[Index + 2];
				if (Positions.IsValidIndex(A) && Positions.IsValidIndex(B) && Positions.IsValidIndex(C))
					DrawTriangle(Lines, Positions[A], Positions[B], Positions[C], Color);
			}
		}
	}

	void DrawClothVirtualParticles(FLineBatcher& Lines, const FClothVirtualParticleData& Samples,
	                               const TArray<FVector>& Positions, const FMatrix& ToWorld)
	{
		const FVector4 Color(1.0f, 0.35f, 0.85f, 1.0f);
		const FVector Axes[] = {FVector(0.9f, 0, 0), FVector(0, 0.9f, 0), FVector(0, 0, 0.9f)};
		for (const auto& Particle : Samples.Particles)
		{
			if (!Positions.IsValidIndex(Particle.A) || !Positions.IsValidIndex(Particle.B) ||
			    !Positions.IsValidIndex(Particle.C) || !Samples.Weights.IsValidIndex(Particle.WeightIndex))
				continue;

			const FVector& W = Samples.Weights[Particle.WeightIndex];
			const FVector Center = ToWorld.TransformPosition(
			    Positions[Particle.A] * W.X + Positions[Particle.B] * W.Y + Positions[Particle.C] * W.Z);
			for (const FVector& Axis : Axes)
				Lines.AddLine(Center - Axis, Center + Axis, Color);
		}
	}

	constexpr char CapsuleCapeMeshPath[] = "FBX/Flagg/CapsuleCape.skeletalmesh";
	constexpr char PaintedCapeMeshPath[] = "FBX/Flagg/PaintedCape.skeletalmesh";
	constexpr char PaintedCapeReferenceMeshPath[] = "FBX/Flagg/PaintedCapeReference.skeletalmesh";

	bool WriteCapsuleCapePhysicsAsset(const std::filesystem::path& Path, const FBoneInfo& Root,
	                                  const TArray<FVector>& RestPositions)
	{
		FBox Bounds{RestPositions[0], RestPositions[0]};
		for (const FVector& Position : RestPositions)
			Bounds.Expand(FBox{Position, Position});

		TUniquePtr<UPhysicsAsset> Physics(NewObject<UPhysicsAsset>());
		Physics->PreviewMeshPath = CapsuleCapeMeshPath;
		TUniquePtr<USkeletalBodySetup> Body(Physics->AddBody(FName(Root.Name)));
		if (!Body)
			return false;

		// Flagg의 세로 천 아래에 Y축 방향으로 캡슐을 놓는다.
		const FVector Center = (Bounds.Min + Bounds.Max) * 0.5f;
		const float Width = Bounds.Max.Y - Bounds.Min.Y;
		FKSphylElem Capsule;
		Capsule.Radius = FMath::Max(5.0f, Width * 0.1f);
		Capsule.Length = Width * 0.4f;
		Capsule.Center = Root.InverseBindPose.TransformPosition(
		    FVector(Center.X, Center.Y, Bounds.Min.Z - Capsule.Radius - 8.0f));
		const FQuat AlongY = FQuat::MakeFromAxisAngle(FVector(1.0f, 0.0f, 0.0f), -1.57079633f);
		Capsule.Rotation = (Root.LocalBindPose.Rotation.UnitInverse() * AlongY).ToFRotator();
		Body->AggGeom.SphylElems.Add(Capsule);

		FKSphereElem Sphere;
		Sphere.Radius = FMath::Max(3.0f, Width * 0.055f);
		Sphere.Center = Root.InverseBindPose.TransformPosition(
		    FVector(Center.X, Center.Y + Width * 0.38f, Bounds.Min.Z - Sphere.Radius - 8.0f));
		Body->AggGeom.SphereElems.Add(Sphere);
		return Physics->SaveToFile(Path);
	}

	bool CreateClothFixture(const char* FixturePath, const char* ClothName, bool bCapsule,
	                        bool bPaintedReference, FString& Error)
	{
		const auto ContentDir = FPaths::ProjectContentDir();
		const auto MeshPath = ContentDir / FixturePath;
		std::error_code FileError;
		const bool bMeshExists = std::filesystem::exists(MeshPath, FileError);
		if (FileError)
		{
			Error = "Cannot access Cloth fixture: " + FileError.message();
			return false;
		}
		if (bMeshExists)
			return true;

		const FString BakePath = FPaths::ToUtf8(MeshPath.parent_path() / "Flagg.fbx.bin");
		FFbxBakedMesh Bake;
		if (!FFbxMeshBake::Read(BakePath, Bake, Error))
			return false;
		if (Bake.Kind != EFbxMeshKind::Skeletal)
		{
			Error = "The Cloth fixture requires the Flagg skeletal mesh bake.";
			return false;
		}

		FSkeletalMeshAssetData Document;
		Document.BakedMeshPath = SkeletalMeshAssetFile::ProjectContentReference(BakePath);
		if (Document.BakedMeshPath.empty())
		{
			Error = "The Flagg bake must be inside the project Content directory.";
			return false;
		}

		TUniquePtr<USkeletalMesh> Fixture(NewObject<USkeletalMesh>());
		if (!Fixture->Build(std::move(Bake.SkeletalMesh), Error))
			return false;

		Fixture->SetPath(FixturePath);
		Fixture->ApplyAssetData(std::move(Document));
		const auto& LODs = Fixture->GetRenderData().LODRenderData;
		const auto& Bones = Fixture->GetRefSkeleton();
		if (!LODs.IsValidIndex(0) || !LODs[0].Sections.IsValidIndex(0) || Bones.IsEmpty())
		{
			Error = "The Cloth fixture requires a root bone and LOD 0 section 0.";
			return false;
		}

		const auto& LOD = LODs[0];
		std::vector<float> MaxDistances(LOD.BindVertices.Num(), 300.0f);
		std::vector<uint32> WeldGroups;
		if (!bCapsule)
		{
			const auto& Section = LOD.Sections[0];
			const auto SectionIndices = std::span(LOD.Indices.GetData() + Section.StartIndex, Section.IndexCount);
			float Top = LOD.BindVertices[SectionIndices.front()].Position.Z;
			float Bottom = Top;
			for (uint32 Vertex : SectionIndices)
			{
				const float Height = LOD.BindVertices[Vertex].Position.Z;
				Top = std::max(Height, Top);
				Bottom = std::min(Height, Bottom);
			}

			// The Flagg bake has matching skinned vertices on both sides of a UV seam.
			WeldGroups.resize(LOD.BindVertices.Num());
			for (int32 Vertex = 0; Vertex < LOD.BindVertices.Num(); ++Vertex)
			{
				const FVector& Position = LOD.BindVertices[Vertex].Position;
				const FSkinWeight& Skin = LOD.SkinWeights[Vertex];
				WeldGroups[Vertex] = Vertex;
				for (int32 Candidate = 0; Candidate < Vertex; ++Candidate)
				{
					const FVector& Other = LOD.BindVertices[Candidate].Position;
					const FSkinWeight& OtherSkin = LOD.SkinWeights[Candidate];
					if (Position.X == Other.X && Position.Y == Other.Y && Position.Z == Other.Z &&
					    std::equal(std::begin(Skin.BoneIndex), std::end(Skin.BoneIndex), std::begin(OtherSkin.BoneIndex)) &&
					    std::equal(std::begin(Skin.Weight), std::end(Skin.Weight), std::begin(OtherSkin.Weight)))
					{
						WeldGroups[Vertex] = WeldGroups[Candidate];
						break;
					}
				}
			}
			if (bPaintedReference && Top > Bottom)
			{
				for (uint32 Vertex : SectionIndices)
					MaxDistances[Vertex] = 100.0f * (Top - LOD.BindVertices[Vertex].Position.Z) / (Top - Bottom);
			}
		}

		const int32 AssetIndex = Fixture->CreateClothingAssetFromSection(
		    0, 0, ClothName, MaxDistances, Error, WeldGroups);
		if (AssetIndex < 0 || !Fixture->BindClothingSection(0, 0, AssetIndex, 0, Error))
			return false;

		if (bCapsule)
		{
			auto* Cloth = Fixture->GetClothingAsset(AssetIndex);
			Cloth->bUsePhysicsAssetCollision = true;
			const auto PhysicsPath = MeshPath.parent_path() / "CapsuleCape.physicsasset";
			const bool bPhysicsExists = std::filesystem::exists(PhysicsPath, FileError);
			if (FileError || (!bPhysicsExists && !WriteCapsuleCapePhysicsAsset(PhysicsPath, Bones[0],
			                                                                  Cloth->LODs[0].RestPositions)))
			{
				Error = "Cannot create CapsuleCape.physicsasset.";
				return false;
			}
			if (!Fixture->AssignPhysicsAssetPath(FPaths::ToUtf8(PhysicsPath), Error))
				return false;
		}

		if (!Fixture->SaveSkeletalMeshAsset(Error))
			return false;

		UAssetManager::Get().RegisterAsset(FixturePath, Fixture.release());
		return true;
	}
}

bool FSkeletalMeshEditor::IsDirty() const
{
	return Mesh && (Mesh->IsAssetDirty() || PendingVirtualParticleEdit.has_value());
}

bool FSkeletalMeshEditor::Save()
{
	if (!Mesh || !CommitClothVirtualParticleSettings())
		return false;
	const bool Saved = Mesh->SaveSkeletalMeshAsset(AssetMessage);
	if (!Saved)
		KRS_LOG(Error, "Skeletal Mesh save failed: {}", AssetMessage);
	return Saved;
}

bool FSkeletalMeshEditor::RevertToSaved()
{
	if (!Mesh ||  !Mesh->RevertSkeletalMeshAsset(AssetMessage))
		return false;

	PendingVirtualParticleEdit.reset();
	SetClothPaintMode(false);
	ClothPaint.ClearHistory();
	SetSelectedClothSection(INDEX_NONE);
	RefreshClothPreview();
	return true;
}

bool FSkeletalMeshEditor::InitializeResources()
{
	if (!InitializePreview())
		return false;
	BoneGizmoTarget = Cast<USceneComponent>(PreviewScene.AddComponent(USceneComponent::StaticClass()));
	if (!BoneGizmoTarget)
		return false;
	BoneGizmoTarget->DetachFromParent();
	ResetBonePoseHistory();
	return true;
}

void FSkeletalMeshEditor::ShutdownResources()
{
	ClothPaint.End();
	ClothPaint.ClearHistory();
	PoseUndoStack.Empty();
	PoseRedoStack.Empty();
	CommittedBonePose.Empty();
	bPoseUndoPending = false;
	BoneGizmo.EndDrag();
	BoneGizmo.SetTarget(nullptr);
	BoneGizmoTarget = nullptr;
	FSkeletalPreviewEditor::ShutdownResources();
}

void FSkeletalMeshEditor::TickAfterWorld(float DeltaTime)
{
	if (IsClothPaintMode())
	{
		TickClothPaint();
		return;
	}

	// 기즈모가 입력을 쓰지 않았을 때만 본을 피킹하고, 마지막에 기즈모를 선택한 본 위치로 맞춘다
	if (!UpdateBoneGizmo())
		UpdateBonePicking();
	SyncBoneGizmo();
}

TUniquePtr<IEditorTab> FSkeletalMeshEditor::CreateTab()
{
	return MakeUnique<FSkeletalMeshEditorTab>(*this);
}

void FSkeletalMeshEditor::SetGizmoMode(EGizmoMode Mode)
{
	BoneGizmo.EndDrag();
	BoneGizmo.SetMode(Mode);
	SyncBoneGizmo();
}

void FSkeletalMeshEditor::SetGizmoSpace(EGizmoSpace Space)
{
	BoneGizmo.EndDrag();
	BoneGizmo.SetSpace(Space);
	SyncBoneGizmo();
}

void FSkeletalMeshEditor::OpenAnimationAsset(const FString& InAssetPath)
{
	if (!InAssetPath.empty())
	{
		EditorManager.OpenEditor(InAssetPath);
	}
}

bool FSkeletalMeshEditor::IsClothPreviewEnabled() const
{
	const UWorld* World = PreviewScene.GetWorld();
	return World && World->IsPreviewClothEnabled();
}

void FSkeletalMeshEditor::SetClothPreviewEnabled(bool bEnabled)
{
	if (UWorld* World = PreviewScene.GetWorld(); World && MeshComponent)
		World->SetPreviewClothEnabled(*MeshComponent, bEnabled);
}

void FSkeletalMeshEditor::RefreshClothPreview()
{
	UWorld* World = PreviewScene.GetWorld();
	if (!World || !MeshComponent || !World->IsPreviewClothEnabled())
		return;

	World->UnregisterClothComponent(*MeshComponent);
	World->RegisterClothComponent(*MeshComponent);
}

FClothVirtualParticleSettings FSkeletalMeshEditor::GetClothVirtualParticleSettingsForEdit(
    const FClothingAsset& Asset) const
{
	if (PendingVirtualParticleEdit && PendingVirtualParticleEdit->AssetId == Asset.Id)
		return PendingVirtualParticleEdit->Settings;
	return Asset.Config.GetVirtualParticleSettings();
}

void FSkeletalMeshEditor::StageClothVirtualParticleSettings(
    const FString& AssetId, FClothVirtualParticleSettings Settings)
{
	if (PendingVirtualParticleEdit && PendingVirtualParticleEdit->AssetId != AssetId &&
	    !CommitClothVirtualParticleSettings())
		return;
	PendingVirtualParticleEdit = FPendingVirtualParticleEdit{AssetId, Settings};
}

bool FSkeletalMeshEditor::CommitClothVirtualParticleSettings()
{
	if (!PendingVirtualParticleEdit)
		return true;

	const auto Edit = std::move(*PendingVirtualParticleEdit);
	PendingVirtualParticleEdit.reset();
	if (Mesh && Mesh->SetClothVirtualParticleSettings(Edit.AssetId, Edit.Settings))
	{
		AssetMessage.clear();
		return true;
	}
	AssetMessage = "Could not apply the virtual particle settings to the selected Cloth Asset.";
	return false;
}

TSharedPtr<const FClothVirtualParticleData> FSkeletalMeshEditor::GetClothVirtualParticlesForDetails(
    int32 Section, const FClothingLODData& LOD) const
{
	const auto* World = PreviewScene.GetWorld();
	TSharedPtr<const FClothVirtualParticleData> Applied;
	if (World && Mesh && MeshComponent && Mesh->FindClothingLOD(0, Section) == &LOD &&
	    World->GetPreviewClothVirtualParticles(*MeshComponent, Section, Applied))
		return Applied;
	return LOD.VirtualParticles;
}

void FSkeletalMeshEditor::SetSelectedClothSection(int32 Section)
{
	if (SelectedClothSection != Section)
		SetClothPaintMode(false);
	SelectedClothSection = Section;
}

void FSkeletalMeshEditor::ClothBindingChanged(int32 Section)
{
	if (Section == SelectedClothSection)
		SetClothPaintMode(false);
	ClothPaint.ForgetSection(Section);
}

bool FSkeletalMeshEditor::CanPaintCloth() const
{
	return Mesh && MeshComponent && ViewportClient && Mesh->IsSkeletalMeshAsset() &&
	       !Mesh->AssetNeedsReload() && Mesh->FindClothingLOD(0, SelectedClothSection);
}

void FSkeletalMeshEditor::SetClothPaintMode(bool bEnabled)
{
	if (bEnabled == IsClothPaintMode())
		return;

	if (!bEnabled)
	{
		ClothPaint.End();
		SetMeshVisible(bRestoreMeshVisible);
		SetWeightHeatmap(bRestoreWeightHeatmap);
		if (ViewportClient)
			ViewportClient->SetViewportMode(PreviousClothPaintViewMode);
		SetClothPreviewEnabled(bRestoreClothSimulation);
		return;
	}

	if (!CanPaintCloth())
		return;

	BoneGizmo.EndDrag();
	CommitBonePoseUndo(false);
	MeshComponent->RefreshBoneTransformsNow();
	if (!ClothPaint.Begin(*Mesh, *MeshComponent, SelectedClothSection, AssetMessage))
		return;

	bRestoreClothSimulation = IsClothPreviewEnabled();
	bRestoreMeshVisible = IsMeshVisible();
	bRestoreWeightHeatmap = IsWeightHeatmapEnabled();
	PreviousClothPaintViewMode = ViewportClient->GetViewportMode();
	SetClothPreviewEnabled(false);
	SetMeshVisible(true);
	SetWeightHeatmap(false);
	ViewportClient->SetViewportMode(EViewportMode::Solid);
}

void FSkeletalMeshEditor::SetClothPaintInput(bool bShift, bool bCtrl, bool bConfirm)
{
	bPaintShift = bShift;
	bPaintCtrl = bCtrl;
	bPaintConfirm = bConfirm;
}

void FSkeletalMeshEditor::TickClothPaint()
{
	const bool bConfirm = std::exchange(bPaintConfirm, false);
	if (!Mesh || !ClothPaint.IsTargetCurrent(*Mesh))
	{
		ClothBindingChanged(SelectedClothSection);
		return;
	}

	if (!CanProcessViewportInput())
	{
		ClothPaint.CancelStroke();
		ClothPaint.ClearHover();
		return;
	}

	if (PointerInput.bCancel)
	{
		if (!ClothPaint.CancelInteraction())
			SetClothPaintMode(false);
		return;
	}

	if (bConfirm && !CameraInput.bCaptured)
		ApplyClothGradient();

	if (CameraInput.bCaptured || !PointerInput.bHovered)
	{
		ClothPaint.ClearHover();
		if (!PointerInput.bMouseDown)
			ClothPaint.Release(*Mesh, AssetMessage);
		return;
	}

	const FVector2 Mouse(PointerInput.MouseUV.X * ViewportWidth, PointerInput.MouseUV.Y * ViewportHeight);
	ClothPaint.UpdateHover(*ViewportClient, Mouse);
	if (PointerInput.bMousePressed && PointerInput.bMouseDown)
		ClothPaint.Press(*ViewportClient, Mouse, bPaintShift, bPaintCtrl, *Mesh, AssetMessage);
	else if (PointerInput.bMouseDown)
		ClothPaint.Drag(*ViewportClient, Mouse, bPaintShift);
	else
		ClothPaint.Release(*Mesh, AssetMessage);
}

void FSkeletalMeshEditor::ApplyClothGradient()
{
	if (Mesh && ClothPaint.IsTargetCurrent(*Mesh))
		ClothPaint.ApplyGradient(*Mesh, AssetMessage);
}

bool FSkeletalMeshEditor::CanUndoEdit(bool bRedo) const
{
	if (IsClothPaintMode())
		return ClothPaint.CanUndo(bRedo);
	return bRedo ? CanRedoPoseEdit() : CanUndoPoseEdit();
}

void FSkeletalMeshEditor::UndoEdit(bool bRedo)
{
	if (IsClothPaintMode())
	{
		if (Mesh && ClothPaint.IsTargetCurrent(*Mesh))
			ClothPaint.Undo(*Mesh, bRedo, AssetMessage);
	}
	else if (bRedo)
	{
		RedoPoseEdit();
	}
	else
	{
		UndoPoseEdit();
	}
}

void FSkeletalMeshEditor::OnRenderDepthTestedOverlay(const FSceneView& View)
{
	ClothPaint.Draw(View);
}

void FSkeletalMeshEditor::OpenCapsuleCapeFixture()
{
	AssetMessage.clear();
	if (CreateClothFixture(CapsuleCapeMeshPath, "CapsuleCape", true, false, AssetMessage))
		EditorManager.OpenEditor(CapsuleCapeMeshPath);
	else
		KRS_LOG(Error, "[Cloth] {}", AssetMessage);
}

void FSkeletalMeshEditor::OpenPaintedCapeFixture(bool bReference)
{
	const char* Path = bReference ? PaintedCapeReferenceMeshPath : PaintedCapeMeshPath;
	const char* Name = bReference ? "PaintedCapeReference" : "PaintedCape";
	AssetMessage.clear();
	if (CreateClothFixture(Path, Name, false, bReference, AssetMessage))
		EditorManager.OpenEditor(Path);
	else
		KRS_LOG(Error, "[Cloth] {}", AssetMessage);
}

void FSkeletalMeshEditor::OnRenderLine(FLineBatcher& Lines)
{
	if (IsClothPaintMode())
		return;

	FSkeletalPreviewEditor::OnRenderLine(Lines);

	const UWorld* World = PreviewScene.GetWorld();
	if (!World || !Mesh || !MeshComponent)
		return;

	const auto& LODs = Mesh->GetRenderData().LODRenderData;
	if (!LODs.IsValidIndex(0) || !LODs[0].Sections.IsValidIndex(SelectedClothSection))
		return;

	const FMatrix ToWorld = MeshComponent->GetWorldMatrix();
	const auto& LOD = LODs[0];
	const auto* Cloth = Mesh->FindClothingLOD(0, SelectedClothSection);
	bool bDrewSimulationWire = false;
	if (Cloth && (bShowClothPins || bShowClothWire || bShowClothCollision || bShowClothVirtualParticles))
	{
		if (bShowClothCollision)
			World->DrawClothCollisionDebug(Lines, MeshComponent, SelectedClothSection);

		if (bShowClothPins || bShowClothWire || bShowClothVirtualParticles)
		{
			FMatrix ClothToWorld = ToWorld;
			auto Positions = World->ReadPreviewClothPositions(*MeshComponent, SelectedClothSection, &ClothToWorld);
			const bool bHasSimPositions = !Positions.IsEmpty();
			if (bHasSimPositions || MeshComponent->BuildClothPose(*Cloth, Positions))
			{
				if (bShowClothVirtualParticles)
				{
					TSharedPtr<const FClothVirtualParticleData> Samples;
					const bool bHasInstance =
					    World->GetPreviewClothVirtualParticles(*MeshComponent, SelectedClothSection, Samples);
					if (!bHasInstance)
						Samples = Cloth->VirtualParticles;
					if (Samples && (bHasSimPositions || !bHasInstance))
						DrawClothVirtualParticles(Lines, *Samples, Positions, ClothToWorld);
				}
				if (bShowClothPins || bShowClothWire)
					DrawClothSimulation(Lines, *Cloth, Positions, ToWorld, bShowClothPins, bShowClothWire);
				bDrewSimulationWire = bShowClothWire;
			}
		}
	}

	if (IsMeshVisible() && !bDrewSimulationWire)
		DrawClothSectionHighlight(Lines, *MeshComponent, LOD, LOD.Sections[SelectedClothSection], ToWorld);
}

void FSkeletalMeshEditor::SyncBoneGizmo()
{
	if (!MeshComponent || !BoneGizmoTarget || SelectedBoneIndex < 0 || SelectedBoneIndex >= BoneItems.Num() ||
	    BoneDrawMode == EBoneDrawMode::None)
	{
		BoneGizmo.SetTarget(nullptr);
		return;
	}

	if (BoneGizmo.IsUsing())
	{
		return;
	}

	const FMatrix BoneWorld = MeshComponent->GetBoneWorldMatrix(SelectedBoneIndex);
	const FVector Location = BoneWorld.TransformPosition(FVector::ZeroVector);
	const FRotator Rotation = GetBoneWorldRotation(SelectedBoneIndex).ToFRotator();

	// 스케일은 조작 배율로만 사용합니다. 부모의 비균일 스케일과 전단을 축 회전에 섞지 않습니다.
	BoneGizmoTarget->SetTransform(FTransform(Rotation, Location));
	BoneGizmo.SetTarget(BoneGizmoTarget);
}

FQuat FSkeletalMeshEditor::GetBoneWorldRotation(int32 BoneIndex) const
{
	FQuat Rotation = FQuat::Identity();
	const TArray<FBoneInfo>& Bones = Mesh->GetRefSkeleton();
	while (BoneIndex >= 0)
	{
		Rotation = MeshComponent->GetBoneLocalTransform(BoneIndex).Rotation * Rotation;
		BoneIndex = Bones[BoneIndex].ParentIndex;
	}

	return (MeshComponent->GetWorldRotation().Quaternion() * Rotation).Normalized();
}

bool FSkeletalMeshEditor::UpdateBoneGizmo()
{
	if (
	    // 공통 뷰포트 입력 조건을 만족하지 않으면 입력을 무시
	    !CanProcessViewportInput() ||

	    // BoneGizmo의 대상이 없으면 입력을 무시
	    !BoneGizmo.GetTarget())
	{
		BoneGizmo.EndDrag();
		return false;
	}

	if (PointerInput.bCancel)
	{
		if (BoneGizmo.IsUsing())
		{
			MeshComponent->SetBoneLocalTransform(SelectedBoneIndex, DragStartBoneTransform);
			MeshComponent->RefreshBoneTransformsNow();
		}
		BoneGizmo.EndDrag();
		return false;
	}

	const bool bWasUsing = BoneGizmo.IsUsing();
	if ((!PointerInput.bHovered && !bWasUsing) || CameraInput.bCaptured)
	{
		BoneGizmo.EndDrag();
		return false;
	}

	SyncBoneGizmo();
	const int32 Width = static_cast<int32>(ViewportClient->GetWidth());
	const int32 Height = static_cast<int32>(ViewportClient->GetHeight());
	const FVector2 MousePosition(PointerInput.MouseUV.X * Width, PointerInput.MouseUV.Y * Height);
	const FRay MouseRay = ViewportClient->DeprojectScreenToWorld(MousePosition);

	// 클릭한 프레임에만 시작하고, 버튼을 놓는 프레임의 마지막 마우스 위치도 반영합니다.
	const bool bUpdateDrag = bWasUsing || (PointerInput.bMousePressed && PointerInput.bMouseDown);
	BoneGizmo.Update(MouseRay, MousePosition, ViewportClient->GetViewProjectionMatrix(),
	                 Width, Height, bUpdateDrag, ViewportClient->GetViewLocation(), !ViewportClient->IsPerspective());

	if (!bWasUsing && BoneGizmo.IsUsing())
	{
		const int32 ParentIndex = Mesh->GetRefSkeleton()[SelectedBoneIndex].ParentIndex;
		const FMatrix ParentWorld = ParentIndex >= 0
		                                ? MeshComponent->GetBoneWorldMatrix(ParentIndex)
		                                : MeshComponent->GetWorldMatrix();

		// 스케일이 0인 부모의 역행렬로 이동을 계산할 수 없습니다.
		if (BoneGizmo.GetMode() == EGizmoMode::Location && fabsf(ParentWorld.Determinant()) < 1e-12f)
		{
			BoneGizmo.EndDrag();
			return true;
		}

		DragStartBoneTransform = MeshComponent->GetBoneLocalTransform(SelectedBoneIndex);
		DragParentRotation = GetBoneWorldRotation(ParentIndex);
		if (BoneGizmo.GetMode() == EGizmoMode::Location)
		{
			DragParentInverse = ParentWorld.Inverse();
		}
	}

	if (BoneGizmo.IsUsing())
	{
		ApplyBoneGizmoTransform();
		if (!PointerInput.bMouseDown)
		{
			BoneGizmo.EndDrag();
		}
	}

	return bWasUsing || BoneGizmo.IsUsing() || BoneGizmo.GetHoveredAxis() >= 0;
}

void FSkeletalMeshEditor::ApplyBoneGizmoTransform()
{
	FTransform Local = DragStartBoneTransform;
	switch (BoneGizmo.GetMode())
	{
	case EGizmoMode::Location:
		Local.Location = DragParentInverse.TransformPosition(BoneGizmoTarget->GetWorldLocation());
		break;
	case EGizmoMode::Rotation:
		Local.Rotation = (DragParentRotation.Inverse() *
		                  BoneGizmoTarget->GetRelativeRotation().Quaternion())
		                     .Normalized();
		break;
	case EGizmoMode::Scale:
		Local.Scale = DragStartBoneTransform.Scale * BoneGizmoTarget->GetRelativeScale3D();
		break;
	}

	// 조작한 성분만 변경하고 나머지는 드래그 시작 시점의 로컬 값을 유지합니다.
	MeshComponent->SetBoneLocalTransform(SelectedBoneIndex, Local);
	MeshComponent->RefreshBoneTransformsNow();
	bPoseUndoPending = true;
}

bool FSkeletalMeshEditor::GetSelectedBoneLocalTransform(FTransform& OutTransform) const
{
	if (!MeshComponent || SelectedBoneIndex < 0 || SelectedBoneIndex >= BoneItems.Num())
	{
		return false;
	}

	OutTransform = MeshComponent->GetBoneLocalTransform(SelectedBoneIndex);
	return true;
}

void FSkeletalMeshEditor::SetSelectedBoneLocalTransform(const FTransform& Transform)
{
	if (!MeshComponent || SelectedBoneIndex < 0 || SelectedBoneIndex >= BoneItems.Num())
	{
		return;
	}

	// 원본 애셋 대신 프리뷰 컴포넌트의 포즈만 수정합니다. 본 갱신은 다음 World Tick에서 수행합니다.
	BoneGizmo.EndDrag();
	MeshComponent->SetBoneLocalTransform(SelectedBoneIndex, Transform);
	bPoseUndoPending = true;
}

void FSkeletalMeshEditor::ResetPose()
{
	if (!MeshComponent)
	{
		return;
	}

	BoneGizmo.EndDrag();
	MeshComponent->ResetPose();
	bPoseUndoPending = true;
}

void FSkeletalMeshEditor::ResetBone(int32 BoneIndex)
{
	if (!Mesh)
	{
		return;
	}

	if (!MeshComponent)
	{
		return;
	}

	if (BoneIndex < 0 || BoneIndex >= Mesh->GetRefSkeleton().Num())
	{
		return;
	}

	BoneGizmo.EndDrag();
	FTransform Transform = Mesh->GetRefSkeleton()[BoneIndex].LocalBindPose;
	MeshComponent->SetBoneLocalTransform(BoneIndex, Transform);
	bPoseUndoPending = true;
}

namespace
{
	bool AreBonePosesEqual(const TArray<FTransform>& A, const TArray<FTransform>& B)
	{
		if (A.Num() != B.Num())
		{
			return false;
		}
		for (int32 i = 0; i < A.Num(); ++i)
		{
			if (A[i].Location.X != B[i].Location.X || A[i].Location.Y != B[i].Location.Y || A[i].Location.Z != B[i].Location.Z ||
			    A[i].Rotation.X != B[i].Rotation.X || A[i].Rotation.Y != B[i].Rotation.Y || A[i].Rotation.Z != B[i].Rotation.Z || A[i].Rotation.W != B[i].Rotation.W ||
			    A[i].Scale.X != B[i].Scale.X || A[i].Scale.Y != B[i].Scale.Y || A[i].Scale.Z != B[i].Scale.Z)
			{
				return false;
			}
		}
		return true;
	}
}

TArray<FTransform> FSkeletalMeshEditor::CaptureBonePose() const
{
	TArray<FTransform> Pose;
	if (!Mesh || !MeshComponent)
	{
		return Pose;
	}

	const int32 NumBones = Mesh->GetRefSkeleton().Num();
	Pose.Reserve(NumBones);
	for (int32 i = 0; i < NumBones; ++i)
	{
		Pose.Add(MeshComponent->GetBoneLocalTransform(i));
	}
	return Pose;
}

void FSkeletalMeshEditor::ApplyBonePose(const TArray<FTransform>& Pose)
{
	if (!Mesh || !MeshComponent)
	{
		return;
	}

	BoneGizmo.EndDrag();
	const int32 NumBones = std::min(static_cast<int32>(Pose.Num()), static_cast<int32>(Mesh->GetRefSkeleton().Num()));
	for (int32 i = 0; i < NumBones; ++i)
	{
		MeshComponent->SetBoneLocalTransform(i, Pose[i]);
	}
	MeshComponent->RefreshBoneTransformsNow();
	SyncBoneGizmo();
}

void FSkeletalMeshEditor::ResetBonePoseHistory()
{
	PoseUndoStack.Empty();
	PoseRedoStack.Empty();
	CommittedBonePose = CaptureBonePose();
	bPoseUndoPending = false;
}

void FSkeletalMeshEditor::CommitBonePoseUndo(bool bInteractionActive)
{
	if (!bPoseUndoPending || bInteractionActive || !MeshComponent)
	{
		return;
	}
	bPoseUndoPending = false;

	TArray<FTransform> Current = CaptureBonePose();
	if (AreBonePosesEqual(Current, CommittedBonePose))
	{
		return;
	}

	PoseUndoStack.Add(std::move(CommittedBonePose));
	if (PoseUndoStack.Num() > MaxUndoSteps)
	{
		PoseUndoStack.RemoveAt(0);
	}
	PoseRedoStack.Empty();
	CommittedBonePose = std::move(Current);
}

void FSkeletalMeshEditor::UndoPoseEdit()
{
	CommitBonePoseUndo(false);
	if (PoseUndoStack.IsEmpty() || !MeshComponent)
	{
		return;
	}

	PoseRedoStack.Add(std::move(CommittedBonePose));
	CommittedBonePose = std::move(PoseUndoStack[PoseUndoStack.Num() - 1]);
	PoseUndoStack.RemoveAt(PoseUndoStack.Num() - 1);
	ApplyBonePose(CommittedBonePose);
}

void FSkeletalMeshEditor::RedoPoseEdit()
{
	CommitBonePoseUndo(false);
	if (PoseRedoStack.IsEmpty() || !MeshComponent)
	{
		return;
	}

	PoseUndoStack.Add(std::move(CommittedBonePose));
	CommittedBonePose = std::move(PoseRedoStack[PoseRedoStack.Num() - 1]);
	PoseRedoStack.RemoveAt(PoseRedoStack.Num() - 1);
	ApplyBonePose(CommittedBonePose);
}

void FSkeletalMeshEditor::FillClothFromSelectedBone(bool bClear)
{
 if (!Mesh || !CanPaintCloth()) return;
 if (!IsClothPaintMode()) SetClothPaintMode(true);
 ClothPaint.FillFromBone(*Mesh, GetSelectedBoneIndex(), bClear, AssetMessage);
}
