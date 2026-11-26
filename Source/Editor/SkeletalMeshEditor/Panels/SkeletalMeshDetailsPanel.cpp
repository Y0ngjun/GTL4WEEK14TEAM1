#include "EnginePCH.h"
#include "SkeletalMeshDetailsPanel.h"
#include "Editor/SkeletalMeshEditor/SkeletalMeshEditor.h"
#include "Math/Transform.h"
#include "Core/Misc/Paths.h"
#include "Render/SkeletalMesh.h"
#include <algorithm>
#include <vector>
#include "Editor/Details/PropertyEditor.h"

#include <array>
#include <cmath>
#include <map>
#include <numeric>
#include <tuple>

namespace
{
	std::vector<uint32> BuildClothWeldGroups(
	    const FSkeletalMeshLODRenderData& LOD, int32 SectionIndex)
	{
		using FBoneIndices = std::array<uint8, MaxBoneInfluences>;
		using FBoneWeights = std::array<float, MaxBoneInfluences>;
		using FWeldKey =
		    std::tuple<float, float, float, FBoneIndices, FBoneWeights>;

		const auto& Section = LOD.Sections[SectionIndex];
		const int32 VertexCount = LOD.BindVertices.Num();
		const uint64 EndIndex =
		    static_cast<uint64>(Section.StartIndex) + Section.IndexCount;

		if (LOD.SkinWeights.Num() != VertexCount ||
		    EndIndex > static_cast<uint64>(LOD.Indices.Num()))
		{
			return {};
		}

		std::vector<uint32> WeldGroups(VertexCount);
		std::iota(WeldGroups.begin(), WeldGroups.end(), uint32{0});

		std::map<FWeldKey, uint32> RepresentativeByKey;
		for (uint64 Index = Section.StartIndex; Index < EndIndex; ++Index)
		{
			const uint32 Vertex = LOD.Indices[Index];
			if (Vertex >= static_cast<uint32>(VertexCount))
				return {};

			const FVector& Position = LOD.BindVertices[Vertex].Position;
			const FSkinWeight& Skin = LOD.SkinWeights[Vertex];

			FBoneIndices BoneIndices{};
			FBoneWeights Weights{};
			for (int32 Influence = 0; Influence < MaxBoneInfluences; ++Influence)
			{
				Weights[Influence] = Skin.Weight[Influence];
				if (Weights[Influence] > 0.0f)
					BoneIndices[Influence] = Skin.BoneIndex[Influence];
			}

			if (!std::isfinite(Position.X) ||
			    !std::isfinite(Position.Y) ||
			    !std::isfinite(Position.Z) ||
			    !std::all_of(Weights.begin(), Weights.end(),
			                 [](float Weight)
			                 { return std::isfinite(Weight); }))
			{
				return {};
			}

			const FWeldKey Key{
			    Position.X, Position.Y, Position.Z, BoneIndices, Weights};
			const auto Group = RepresentativeByKey.try_emplace(Key, Vertex).first;
			WeldGroups[Vertex] = Group->second;
		}

		return WeldGroups;
	}
}

FSkeletalMeshDetailsPanel::FSkeletalMeshDetailsPanel(FSkeletalMeshEditor& InEditor)
    : FSkeletalMeshEditorPanel(InEditor, "Details")
{
}

void FSkeletalMeshDetailsPanel::DrawAssetSettings()
{
	auto* Mesh = Editor.GetMesh();
	if (!Mesh || !Mesh->IsSkeletalMeshAsset())
		return;
	if (Mesh->AssetNeedsReload())
	{
		ImGui::TextWrapped("Restart the editor to use the reimported mesh.");
		return;
	}
	if (!Editor.GetAssetMessage().empty())
		ImGui::TextWrapped("%s", Editor.GetAssetMessage().c_str());
	if (!EditError.empty())
		ImGui::TextWrapped("%s", EditError.c_str());

	const bool bPaintMode = Editor.IsClothPaintMode();
	ImGui::BeginDisabled(bPaintMode);
	const auto& PhysicsPath = Mesh->GetPhysicsAssetPath();
	if (ImGui::BeginCombo("Physics Asset", PhysicsPath.empty() ? "None" : PhysicsPath.c_str()))
	{
		if (ImGui::Selectable("None", PhysicsPath.empty()))
			if (Mesh->AssignPhysicsAssetPath({}, EditError))
				Editor.RefreshClothPreview();
		std::error_code Error;
		for (std::filesystem::recursive_directory_iterator It(FPaths::ProjectContentDir(), Error), End;
		     !Error && It != End; It.increment(Error))
		{
			if (It->path().extension() != ".physicsasset" || !It->is_regular_file(Error))
				continue;
			const auto Path = FPaths::ToUtf8(It->path());
			const auto Reference = SkeletalMeshAssetFile::ProjectContentReference(Path);
			if (!Reference.empty() && ImGui::Selectable(Reference.c_str(), Reference == PhysicsPath))
				if (Mesh->AssignPhysicsAssetPath(Path, EditError))
					Editor.RefreshClothPreview();
		}
		ImGui::EndCombo();
	}
	ImGui::EndDisabled();

	if (!ImGui::CollapsingHeader("Cloth", bPaintMode ? ImGuiTreeNodeFlags_DefaultOpen : 0))
		return;

	ImGui::BeginDisabled(bPaintMode);
	bool bSimulate = Editor.IsClothPreviewEnabled();
	if (ImGui::Checkbox("Simulate Cloth", &bSimulate))
		Editor.SetClothPreviewEnabled(bSimulate);

	ImGui::SameLine();
	ImGui::BeginDisabled(!Editor.IsClothPreviewEnabled());
	if (ImGui::Button("Reset Cloth"))
		Editor.RefreshClothPreview();
	ImGui::EndDisabled();

	ImGui::Checkbox("Pinned Points", &Editor.bShowClothPins);
	ImGui::Checkbox("Simulation Wireframe", &Editor.bShowClothWire);
	ImGui::Checkbox("Collision Shapes", &Editor.bShowClothCollision);
	ImGui::Checkbox("Virtual Particles", &Editor.bShowClothVirtualParticles);
	ImGui::EndDisabled();

	constexpr int32 RenderLOD = 0;
	const auto& LODs = Mesh->GetRenderData().LODRenderData;
	if (!LODs.IsValidIndex(RenderLOD) || LODs[RenderLOD].Sections.IsEmpty())
		return;

	const auto& LOD = LODs[RenderLOD];
	int32 ClothSection = Editor.GetSelectedClothSection();

	ImGui::TextUnformatted("Render LOD 0");
	const FString Preview = LOD.Sections.IsValidIndex(ClothSection)
	                            ? std::to_string(ClothSection)
	                            : "Select a section";

	if (ImGui::BeginCombo("Section", Preview.c_str()))
	{
		for (int32 Index = 0; Index < LOD.Sections.Num(); ++Index)
		{
			if (!ImGui::Selectable(std::to_string(Index).c_str(), Index == ClothSection))
				continue;

			ClothSection = Index;
			Editor.SetSelectedClothSection(Index);
			ClothAsset = INDEX_NONE;
			ClothLOD = 0;

			for (const auto& Binding : Mesh->GetClothingBindings())
			{
				if (Binding.RenderLOD == RenderLOD && Binding.SectionIndex == Index)
				{
					ClothAsset = Binding.ClothingAssetIndex;
					ClothLOD = Binding.ClothLODIndex;
					break;
				}
			}
		}
		ImGui::EndCombo();
	}

	if (!LOD.Sections.IsValidIndex(ClothSection))
		return;

	bool bPaint = Editor.IsClothPaintMode();
	ImGui::BeginDisabled(!Editor.CanPaintCloth());
	if (ImGui::Checkbox("Paint Cloth", &bPaint))
		Editor.SetClothPaintMode(bPaint);
	ImGui::EndDisabled();
	if (Editor.IsClothPaintMode())
	{
		for (const FClothingSectionBinding& Binding : Mesh->GetClothingBindings())
		{
			if (Binding.RenderLOD == RenderLOD && Binding.SectionIndex == ClothSection)
			{
				if (const FClothingAsset* Asset = Mesh->GetClothingAsset(Binding.ClothingAssetIndex))
					ImGui::Text("Cloth Asset  %s", Asset->Name.c_str());
				break;
			}
		}
		DrawClothPaintSettings();
		return;
	}

	const auto& MaterialSlots = Mesh->GetMaterialSlots();
	const int32 MaterialOverride = Mesh->GetSectionMaterialOverride(ClothSection);
	const uint32 ImportedSlot = LOD.Sections[ClothSection].MaterialSlotIndex;
	const FString ImportedMaterial = ImportedSlot < MaterialSlots.size()
	                                     ? "Imported (" + MaterialSlots[ImportedSlot].Name + ")"
	                                     : "Imported";
	const char* MaterialPreview = MaterialSlots.IsValidIndex(MaterialOverride)
	                                  ? MaterialSlots[MaterialOverride].Name.c_str()
	                                  : ImportedMaterial.c_str();
	if (ImGui::BeginCombo("Section Material", MaterialPreview))
	{
		if (ImGui::Selectable("Use Imported Material", MaterialOverride == INDEX_NONE))
			Mesh->SetSectionMaterialOverride(ClothSection, INDEX_NONE);
		for (int32 Slot = 0; Slot < MaterialSlots.Num(); ++Slot)
		{
			ImGui::PushID(Slot);
			const auto& Name = MaterialSlots[Slot].Name;
			const FString Label = Name.empty() ? "Slot " + std::to_string(Slot) : Name;
			if (ImGui::Selectable(Label.c_str(), Slot == MaterialOverride))
				Mesh->SetSectionMaterialOverride(ClothSection, Slot);
			ImGui::PopID();
		}
		ImGui::EndCombo();
	}

	ImGui::DragFloat("New Cloth Max Distance", &ClothMaxDistance,
	                 0.2f, 0.0f, 10000.0f, "%.2f", ImGuiSliderFlags_AlwaysClamp);

	if (ImGui::Button("Create Cloth from Section"))
	{
		const std::vector<uint32> WeldGroups =
		    BuildClothWeldGroups(LOD, ClothSection);
		std::vector<float> MaxDistances(
		    LOD.BindVertices.Num(), ClothMaxDistance);

		// 기존 Cloth Paint 값을 새 용접 그룹으로 옮긴다.
		// 그룹 내 MaxDistance는 최솟값을 사용해 기존 고정점(0)을 보존한다.
		const FClothingLODData* Previous = Mesh->FindClothingLOD(RenderLOD, ClothSection);
		if (Previous && !WeldGroups.empty() &&
		    Previous->RenderToSimVertex.Num() == LOD.BindVertices.Num())
		{
			std::vector<float> GroupMin(
			    WeldGroups.size(), std::numeric_limits<float>::infinity());

			for (int32 Vertex = 0; Vertex < LOD.BindVertices.Num(); ++Vertex)
			{
				const int32 OldSim = Previous->RenderToSimVertex[Vertex];
				if (!Previous->MaxDistances.IsValidIndex(OldSim))
					continue;

				float& Minimum = GroupMin[WeldGroups[Vertex]];
				Minimum = std::min(Minimum, Previous->MaxDistances[OldSim]);
			}

			for (size_t Vertex = 0; Vertex < WeldGroups.size(); ++Vertex)
			{
				const float Minimum = GroupMin[WeldGroups[Vertex]];
				if (std::isfinite(Minimum))
					MaxDistances[Vertex] = Minimum;
			}
		}

		const int32 Index = Mesh->CreateClothingAssetFromSection(
		    RenderLOD, ClothSection,
		    "Cloth " + std::to_string(Mesh->GetClothingAssetCount() + 1),
		    MaxDistances, EditError, WeldGroups);

		if (Index >= 0)
		{
			ClothAsset = Index;
			ClothLOD = 0;
			if (Mesh->BindClothingSection(RenderLOD, ClothSection, Index, 0, EditError))
			{
				Editor.ClothBindingChanged(ClothSection);
				Editor.RefreshClothPreview();
			}
		}
	}

	const auto* Selected = Mesh->GetClothingAsset(ClothAsset);
	if (ImGui::BeginCombo("Cloth Asset", Selected ? Selected->Name.c_str() : "None"))
	{
		for (int32 Index = 0; Index < Mesh->GetClothingAssetCount(); ++Index)
		{
			ImGui::PushID(Index);
			if (ImGui::Selectable(Mesh->GetClothingAsset(Index)->Name.c_str(), ClothAsset == Index))
			{
				ClothAsset = Index;
				ClothLOD = 0;
			}
			ImGui::PopID();
		}
		ImGui::EndCombo();
	}
	auto* Cloth = Mesh->GetClothingAsset(ClothAsset);
	if (!Cloth)
		return;

	ImGui::PushID(ClothAsset);
	char Name[256]{};
	std::snprintf(Name, sizeof(Name), "%s", Cloth->Name.c_str());
	bool Changed = ImGui::InputText("Name", Name, sizeof(Name));
	if (Changed)
		Cloth->Name = Name;

	bool bConfigChanged = false;
	if (FPropertyEditor::BeginPropertyGrid("ClothConfigGrid"))
	{
		const UScriptStruct& ConfigStruct = *FClothConfig::StaticStruct();
		for (const char* Member : {
		         "GravityZ",
		         "SolverFrequency",
		         "Damping",
		         "VerticalStiffness",
		         "HorizontalStiffness",
		         "BendingStiffness",
		         "ShearingStiffness",
		         "Friction",
		         "CollisionThickness",
		         "bEnableContinuousCollision"})
			bConfigChanged |= FPropertyEditor::DrawStructMember(Mesh, &Cloth->Config, ConfigStruct, Member);

		FClothConfig Draft = Cloth->Config;
		Draft.SetVirtualParticleSettings(Editor.GetClothVirtualParticleSettingsForEdit(*Cloth));
		const bool bToggled = FPropertyEditor::DrawStructMember(Mesh, &Draft, ConfigStruct,
		                                                        "bEnableVirtualParticles", nullptr, nullptr, "Enable Virtual Particles");
		ImGui::BeginDisabled(!Draft.bEnableVirtualParticles);
		const bool bSpacingChanged = FPropertyEditor::DrawStructMember(Mesh, &Draft, ConfigStruct,
		                                                               "VirtualParticleSpacing", nullptr, nullptr, "Virtual Particle Spacing (cm)");
		const bool bSubdivisionsChanged = FPropertyEditor::DrawStructMember(Mesh, &Draft, ConfigStruct,
		                                                                    "MaxVirtualParticleSubdivisions", nullptr, nullptr, "Max Virtual Particle Subdivisions");
		ImGui::EndDisabled();
		if (bToggled || bSpacingChanged || bSubdivisionsChanged)
			Editor.StageClothVirtualParticleSettings(Cloth->Id, Draft.GetVirtualParticleSettings());
		if (bToggled)
			Editor.CommitClothVirtualParticleSettings();
		FPropertyEditor::EndPropertyGrid();
	}
	if (bConfigChanged)
	{
		Mesh->MarkAssetDirty();
		bClothConfigRefreshPending = true;
	}

	if (ImGui::Checkbox("Use Physics Asset Collision", &Cloth->bUsePhysicsAssetCollision))
	{
		Mesh->MarkAssetDirty();
		bClothConfigRefreshPending = true;
	}

	ImGui::PopID();
	if (Changed)
		Mesh->MarkAssetDirty();

	ImGui::DragInt("Cloth LOD", &ClothLOD, 1.0f, 0, std::max(0, Cloth->LODs.Num() - 1), "%d", ImGuiSliderFlags_AlwaysClamp);
	if (Cloth->LODs.IsValidIndex(ClothLOD))
	{
		const auto& Data = Cloth->LODs[ClothLOD];
		const auto Samples = Editor.GetClothVirtualParticlesForDetails(ClothSection, Data);
		const int32 RealCount = Data.RestPositions.Num();
		const int32 VirtualCount = Samples ? Samples->Particles.Num() : 0;
		ImGui::Text("Real Particle Count  %d", RealCount);
		ImGui::Text("Virtual Particle Count  %d", VirtualCount);
		ImGui::Text("Total Collision Sample Count  %d", RealCount + VirtualCount);
		if (Samples && Samples->bBudgetLimited)
			ImGui::TextWrapped("Virtual particle limit reached (%d per Cloth LOD).", FClothVirtualParticleData::MaxParticles);
	}
	ImGui::TextUnformatted(Mesh->FindClothingLOD(0, ClothSection) ? "Section bound" : "Section uses default skinning");
	if (ImGui::Button("Bind Section") &&
	    Mesh->BindClothingSection(0, ClothSection, ClothAsset, ClothLOD, EditError))
	{
		Editor.ClothBindingChanged(ClothSection);
		Editor.RefreshClothPreview();
	}
	ImGui::SameLine();
	if (ImGui::Button("Unbind Section") &&
	    Mesh->UnbindClothingSection(RenderLOD, ClothSection))
	{
		EditError.clear();
		Editor.ClothBindingChanged(ClothSection);
		Editor.RefreshClothPreview();
	}
	ImGui::Separator();
}

void FSkeletalMeshDetailsPanel::DrawClothPaintSettings()
{
	ImGui::SeparatorText("Max Distance");
	FClothPaintSettings& Settings = Editor.GetClothPaintSettings();
	if (ImGui::Button("Fill from Selected Bone")) Editor.FillClothFromSelectedBone();
 ImGui::SameLine();
 if (ImGui::Button("Clear Cloth Weights")) Editor.FillClothFromSelectedBone(true);
	constexpr const char* Tools[] = {"Brush", "Fill", "Gradient", "Smooth"};
	int Tool = static_cast<int>(Settings.Tool);
	if (ImGui::Combo("Tool", &Tool, Tools, IM_ARRAYSIZE(Tools)))
		Editor.SetClothPaintTool(static_cast<EClothPaintTool>(Tool));

	const auto DragDistance = [](const char* Label, float& Value, float Min = 0.0f, float Max = 10000.0f)
	{
		ImGui::DragFloat(Label, &Value, 0.2f, Min, Max, "%.2f cm", ImGuiSliderFlags_AlwaysClamp);
	};

	switch (Settings.Tool)
	{
	case EClothPaintTool::Brush:
		DragDistance("Paint Value", Settings.PaintValue);
		ImGui::TextDisabled("Shift + drag to erase.");
		break;
	case EClothPaintTool::Fill:
		DragDistance("Fill Value", Settings.FillValue);
		DragDistance("Threshold", Settings.FillThreshold);
		break;
	case EClothPaintTool::Gradient:
		DragDistance("Start Value", Settings.GradientStart);
		DragDistance("End Value", Settings.GradientEnd);
		ImGui::TextWrapped("Click start points. Ctrl + click end points.");
		ImGui::BeginDisabled(!Editor.CanApplyClothGradient());
		if (ImGui::Button("Apply Gradient"))
			Editor.ApplyClothGradient();
		ImGui::EndDisabled();
		break;
	case EClothPaintTool::Smooth:
		break;
	}

	if (Settings.Tool == EClothPaintTool::Brush || Settings.Tool == EClothPaintTool::Smooth)
	{
		DragDistance("Radius", Settings.Radius, 0.1f);
		ImGui::SliderFloat("Strength", &Settings.Strength, 0.0f, 1.0f, "%.2f", ImGuiSliderFlags_AlwaysClamp);
		ImGui::SliderFloat("Falloff", &Settings.Falloff, 0.0f, 1.0f, "%.2f", ImGuiSliderFlags_AlwaysClamp);
	}
	ImGui::Checkbox("Ignore Back Faces", &Settings.bIgnoreBackFaces);

	if (ImGui::TreeNode("Mask Display"))
	{
		ImGui::Checkbox("Auto Range", &Settings.bAutoRange);
		ImGui::BeginDisabled(Settings.bAutoRange);
		DragDistance("View Min", Settings.ViewMin, 0.0f, Settings.ViewMax - 0.01f);
		DragDistance("View Max", Settings.ViewMax, Settings.ViewMin + 0.01f);
		ImGui::EndDisabled();
		ImGui::SliderFloat("Opacity", &Settings.Opacity, 0.0f, 1.0f, "%.2f", ImGuiSliderFlags_AlwaysClamp);
		ImGui::Checkbox("Pinned Points", &Settings.bShowPins);
		ImGui::TreePop();
	}
}

void FSkeletalMeshDetailsPanel::DrawContent()
{
	DrawAssetSettings();
	if (!ImGui::IsAnyItemActive())
		Editor.CommitClothVirtualParticleSettings();
	if (bClothConfigRefreshPending && !ImGui::IsAnyItemActive())
	{
		bClothConfigRefreshPending = false;
		Editor.RefreshClothPreview();
	}

	if (Editor.IsClothPaintMode())
		return;

	const int32 BoneIndex = Editor.GetSelectedBoneIndex();

	if (ImGui::Button("Reset Transform"))
	{
		Editor.ResetBone(BoneIndex);
	}
	ImGui::SameLine();
	if (ImGui::Button("Reset All Transform"))
	{
		Editor.ResetPose();
	}

	ImGui::Separator();

	FTransform Transform;
	if (!Editor.GetSelectedBoneLocalTransform(Transform))
	{
		ImGui::TextDisabled("Select a bone in Bone Hierarchy.");
		return;
	}

	// 선택이 바뀌면 이전 본의 편집 상태가 새 본에 이어지지 않도록 ID를 구분합니다.
	ImGui::PushID(BoneIndex);
	bool bChanged = false;
	// 레벨 디테일 패널과 같은 X/Y/Z 축 컨트롤을 쓴다 (축 버튼을 누르면 그 축만 초기화).
	bChanged |= FPropertyEditor::DrawVector3("Location", Transform.Location.V, 0.0f);
	// 드래그 중 값이 튀지 않도록 마지막 오일러 값을 기억한다
	static FRotationConversionCache RotationCache;
	FRotator Euler = RotationCache.QuatToRotator(Transform.Rotation);
	if (FPropertyEditor::DrawRotator("Rotation", Euler))
	{
		Transform.Rotation = RotationCache.RotatorToQuat(Euler);
		bChanged = true;
	}
	bChanged |= FPropertyEditor::DrawVector3("Scale", Transform.Scale.V, 1.0f);
	ImGui::PopID();

	if (bChanged)
	{
		Editor.SetSelectedBoneLocalTransform(Transform);
	}
}
