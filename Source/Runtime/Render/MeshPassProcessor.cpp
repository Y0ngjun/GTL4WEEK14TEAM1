#include "EnginePCH.h"
#include "Render/MeshPassProcessor.h"
#include "Render/SceneRenderData.h"
#include "Render/SceneView.h"
#include "Render/RenderResourceManager.h"
#include "Render/Mesh.h"
#include "Engine/PrimitiveSceneProxy.h"
#include "Components/StaticMeshComponent.h"
#include "Asset/AssetManager.h"
#include "Core/EngineTimer.h"
#include <algorithm>
#include <cstring>

static const FAABB ConservativeBounds = FAABB(FVector(0.0f, 0.0f, 0.0f), FVector(FLT_MAX / 4, FLT_MAX / 4, FLT_MAX / 4));

namespace
{
	float GetNormalEncoding(UTexture2D* Texture)
	{
		if (!Texture || !Texture->GetResource() || !Texture->GetResource()->GetSRV())
			return 0.0f;
		switch (Texture->GetResource()->GetFormat())
		{
		case DXGI_FORMAT_BC5_UNORM:
			return 2.0f;
		case DXGI_FORMAT_BC5_SNORM:
			return 3.0f;
		default:
			return 1.0f;
		}
	}

	const FMatrix& PacketWorld(const FRenderPacket& Packet)
	{
		if (Packet.Proxy)
			return Packet.Proxy->GetLocalToWorld();
		return Packet.Model ? *Packet.Model : FMatrix::Identity;
	}

	uint64 GroupKey(const FStaticDrawGroup& Group)
	{
		return (static_cast<uint64>(Group.Material->SortID) << 47) | (static_cast<uint64>(Group.Mesh->SortID) << 31);
	}

	bool BuildBatch(FMeshDrawBatch& Batch, const FVertexFactory* VertexFactory,
	                UMaterial* Material, EMeshPassType Pass, const FSceneView& View, ELightingModel LightingModel,
	                const void* OverrideParams = nullptr, uint32 OverrideSize = 0)
	{
		if (!VertexFactory || !VertexFactory->IsValid() || !Material)
			return false;
		Batch.VertexFactory = VertexFactory;
		const EVertexFactoryType VFType = VertexFactory->GetType();
		const bool bUsesUberShader = Pass == EMeshPassType::Forward || Pass == EMeshPassType::WorldNormal;
		if (bUsesUberShader)
		{
			const bool bWorldNormal = Pass == EMeshPassType::WorldNormal;
			FPipelineState* Base = FRenderResourceManager::GetPSO(bWorldNormal
			                                                          ? EPSOType::StaticMesh_WorldNormal
			                                                          : EPSOType::StaticMesh_Forward);
			// 뷰포트가 전역 Unlit이거나 머티리얼 자체가 Unlit이면 Unlit 셰이더 사용
			const ELightingModel EffectiveLighting =
			    (LightingModel == ELightingModel::Unlit || Material->IsUnlit()) ? ELightingModel::Unlit : LightingModel;
			FShaderProgram* Shader = bWorldNormal ? (Base ? Base->Shader : nullptr)
			                                      : FRenderResourceManager::GetForwardShaderProgram(EffectiveLighting);
			if (!Base || !Shader || !Shader->VertexShader || !Shader->PixelShader)
				return false;
			Batch.Pipeline = *Base;
			Batch.Pipeline.Shader = Shader;
			if (Material->bTwoSided)
				Batch.Pipeline.RasterizerState = ERasterizerState::SolidNone;
			Batch.bIsMasked = Material->IsMasked();
			Batch.bUseObjectSlots = false;
			Batch.bBindViewConstants = false;
			Batch.ObjectBindingSlot = 0;
			Batch.ObjectStages = EShaderBindFlagBits::Vertex | EShaderBindFlagBits::Pixel;
			Batch.MaterialSlot = 2;
			Batch.MaterialStages = EShaderBindFlagBits::Vertex | EShaderBindFlagBits::Pixel;
			Batch.SamplerStages = Batch.MaterialStages;
			Batch.bForceWireframe = !bWorldNormal && View.bIsWireframe;

			FForwardMaterialConstants Params{};
			Params.BaseColor = Material->BaseColor;
			Params.SpecularColorShininess = FVector4(Material->SpecularColor.X,
			                                         Material->SpecularColor.Y, Material->SpecularColor.Z, std::max(1.0f, Material->Shininess));
			UTexture2D* Color = Material->Textures.IsEmpty() ? nullptr : Material->Textures[0];
			UTexture2D* Normal = Material->Textures.Num() > 1 ? Material->Textures[1] : nullptr;
			const bool bHasColor = Color && Color->GetResource() && Color->GetResource()->GetSRV();
			// y: Masked 알파 클립 임계값 (< 0이면 클립하지 않는다)
			const float MaskClip = Material->IsMasked() ? std::max(0.0f, Material->OpacityMaskClipValue) : -1.0f;
			Params.SurfaceParams = FVector4(Material->bFlipNormalY ? 1.0f : 0.0f, MaskClip, bHasColor ? 1.0f : 0.0f,
			                                GetNormalEncoding(Normal));
			const FVector2 UV = Material->UVScrollSpeed * EngineTimer::GetTotalTime();
			Params.UVTransform = FVector4(UV.X, UV.Y, 1.0f, 1.0f);
			std::memcpy(Batch.MaterialData.data(), &Params, sizeof(Params));
			Batch.MaterialDataSize = sizeof(Params);
		}
		else
		{
			// 그리는 쪽(Pass + VertexFactory)이 머티리얼 속성(BlendMode, bTwoSided)을 보고 PSO를 결정한다
			const bool bIsParticleSpriteOrTrail =
			    VFType == EVertexFactoryType::SpriteParticle || VFType == EVertexFactoryType::Trail;
			const bool bIsBillboard = Material->PSOType == EPSOType::Billboard;
			const bool bAdditive = Material->BlendMode == EMaterialBlendMode::Additive;
			EPSOType Type = EPSOType::StaticMesh_Translucent;
			if (bIsBillboard)
				Type = EPSOType::Billboard;
			else if (bIsParticleSpriteOrTrail)
				Type = bAdditive ? EPSOType::Particle_Additive : EPSOType::Particle_AlphaBlend;
			else if (Material->IsOpaqueOrMasked())
				Type = View.bIsWireframe ? EPSOType::StaticMesh_Wireframe : EPSOType::StaticMesh_Opaque;
			else
				Type = bAdditive ? EPSOType::StaticMesh_Additive : EPSOType::StaticMesh_Translucent;
			FPipelineState* Pipeline = FRenderResourceManager::GetPSO(Type);
			if (!Pipeline || !Pipeline->Shader)
				return false;
			Batch.Pipeline = *Pipeline;
			Batch.bForceWireframe = View.bIsWireframe;
			if (!bIsBillboard && !bIsParticleSpriteOrTrail && Material->bTwoSided)
				Batch.Pipeline.RasterizerState = ERasterizerState::SolidNone;
			if (!bIsBillboard && !bIsParticleSpriteOrTrail)
			{
				FStaticMeshMaterialParams Params{};
				Params.BaseColor = Material->BaseColor;
				Params.UVOffset = Material->UVScrollSpeed * EngineTimer::GetTotalTime();
				if (Material->IsTranslucent())
				{
					// 반투명 셰이더: x = 컬러 텍스처 사용 여부. 없으면 BaseColor만 쓴다
					UTexture2D* Color = Material->Textures.IsEmpty() ? nullptr : Material->Textures[0];
					Params.TextureFlags.X = Color && Color->GetResource() && Color->GetResource()->GetSRV() ? 1.0f : 0.0f;
				}
				else
				{
					Params.TextureFlags.X = GetNormalEncoding(Material->Textures.Num() > 1 ? Material->Textures[1] : nullptr);
				}
				Params.TextureFlags.Y = Material->bFlipNormalY ? 1.0f : 0.0f;
				std::memcpy(Batch.MaterialData.data(), &Params, sizeof(Params));
				Batch.MaterialDataSize = sizeof(Params);
			}
			else if (OverrideParams && OverrideSize <= Batch.MaterialData.size())
			{
				std::memcpy(Batch.MaterialData.data(), OverrideParams, OverrideSize);
				Batch.MaterialDataSize = OverrideSize;
				if (bIsParticleSpriteOrTrail)
					Batch.MaterialStages = EShaderBindFlagBits::Vertex | EShaderBindFlagBits::Pixel;
			}
		}
		Batch.Sampler = Material->SamplerState;
		Batch.Textures.Reserve(Material->Textures.Num());
		for (int32 Index = 0; Index < Material->Textures.Num(); ++Index)
		{
			// Forward 조명은 t8 이상을 전역 리소스에 사용한다.
			if (bUsesUberShader && Index >= 8)
				break;
			UTexture2D* Texture = Material->Textures[Index];
			EShaderBindFlagBits Stages = EShaderBindFlagBits::Pixel;
			if (bUsesUberShader && Index == 1)
				Stages = EShaderBindFlagBits::Vertex | EShaderBindFlagBits::Pixel;
			if (bUsesUberShader && Index == 0 &&
			    (!Texture || !Texture->GetResource() || !Texture->GetResource()->GetSRV()))
				Texture = nullptr;
			if (bUsesUberShader && Index == 1 && GetNormalEncoding(Texture) <= 0.5f)
				Texture = nullptr;
			Batch.Textures.Add({Texture, Stages});
		}
		// 파티클 스프라이트/트레일 셰이더는 항상 t0를 샘플링하므로 텍스처가 비어 있으면 흰 텍스처로 대체한다
		if (!bUsesUberShader && (VFType == EVertexFactoryType::SpriteParticle || VFType == EVertexFactoryType::Trail))
		{
			UTexture2D* Primary = Batch.Textures.IsEmpty() ? nullptr : Batch.Textures[0].Texture;
			if (!Primary || !Primary->GetResource() || !Primary->GetResource()->GetSRV())
			{
				UTexture2D* White = UAssetManager::GetAssetByPath<UTexture2D>("WhiteTexture");
				if (Batch.Textures.IsEmpty())
					Batch.Textures.Add({White, EShaderBindFlagBits::Pixel});
				else
					Batch.Textures[0].Texture = White;
			}
		}
		return true;
	}

	// ShadowDepth.hlsl의 cbuffer ShadowMaskParams(b1)와 같은 배치
	struct alignas(16) FShadowMaskParams
	{
		float MaskClip = -1.0f;  // < 0이면 클립하지 않는다
		float BaseAlpha = 1.0f;  // 머티리얼 BaseColor.a
		float bHasTexture = 0.0f; // 1이면 t0의 알파를 곱한다
		float Padding = 0.0f;
		FVector2 UVOffset{};
		FVector2 UVPadding{};
	};
	static_assert(sizeof(FShadowMaskParams) == 32);

	// 그림자 배치에 Masked 알파 클립 값을 싣는다. Masked가 아니면 클립을 끈 값을 싣는다
	// (b1을 항상 채워야 앞 드로우가 남긴 상수 버퍼를 읽지 않는다)
	void ApplyShadowMask(FMeshDrawBatch& Batch, const UMaterial* Material)
	{
		FShadowMaskParams Params{};
		Batch.Textures.Reset();
		if (Material && Material->IsMasked())
		{
			UTexture2D* Color = Material->GetBaseColorTexture();
			const bool bHasTexture = Color && Color->GetResource() && Color->GetResource()->GetSRV();
			Params.MaskClip = std::max(0.0f, Material->OpacityMaskClipValue);
			Params.BaseAlpha = Material->BaseColor.W;
			Params.bHasTexture = bHasTexture ? 1.0f : 0.0f;
			Params.UVOffset = Material->UVScrollSpeed * EngineTimer::GetTotalTime();
			Batch.Textures.Add({bHasTexture ? Color : nullptr, EShaderBindFlagBits::Pixel});
			Batch.Sampler = Material->SamplerState;
			Batch.SamplerStages = EShaderBindFlagBits::Pixel;
		}
		std::memcpy(Batch.MaterialData.data(), &Params, sizeof(Params));
		Batch.MaterialDataSize = sizeof(Params);
		Batch.MaterialSlot = 1;
		Batch.MaterialStages = EShaderBindFlagBits::Pixel;
	}

	// 패스 셰이더를 배치의 정점 타입에 맞는 변형으로 바꾼다. 변형이 없으면 그리지 않는다
	bool ApplyVertexFactory(FMeshDrawBatch& Batch)
	{
		if (!Batch.VertexFactory)
			return false;
		FShaderProgram* Shader = FRenderResourceManager::ResolveShader(Batch.Pipeline.Shader, Batch.VertexFactory->GetType());
		if (!Shader)
			return false;
		Batch.Pipeline.Shader = Shader;
		return true;
	}

	FMeshDrawBatch::FItem BuildItem(const FMatrix& World, const FSceneView& View,
	                                EMeshPassType Pass, uint32 Slot, uint32 StartIndex, uint32 IndexCount,
	                                const void* ObjectKey, bool bReceiveShadow, bool bReceivesDecals, const FAABB& Bounds,
	                                const FPrimitiveSceneProxy* Proxy)
	{
		FMeshDrawBatch::FItem Item{};
		Item.ObjectSlot = Slot;
		Item.StartIndex = StartIndex;
		Item.IndexCount = IndexCount;
		Item.ObjectKey = ObjectKey;
		Item.bReceivesDecals = bReceivesDecals;
		Item.Bounds = Bounds;
		Item.Proxy = Proxy;
		if (Pass == EMeshPassType::Forward || Pass == EMeshPassType::WorldNormal)
		{
			FForwardObjectConstants Constants{};
			Constants.World = World;
			Constants.View = View.ViewMatrix;
			Constants.Projection = View.ProjectionMatrix;
			Constants.NormalMatrix = World.Inverse().GetTransposed();
			Constants.ShadowFlags.X = static_cast<float>(bReceiveShadow);
			// 카메라·조명 방향처럼 에디터 아이콘으로 쓰는 메시는 항상 표면 색으로 표시한다.
			const UStaticMeshComponent* MeshComponent = Proxy ? Cast<UStaticMeshComponent>(Proxy->GetComponent()) : nullptr;
			Constants.ShadowFlags.W = MeshComponent && MeshComponent->IsEditorIcon() ? 1.0f : 0.0f;
			std::memcpy(Item.ObjectData.data(), &Constants, sizeof(Constants));
			Item.ObjectDataSize = sizeof(Constants);
		}
		else
		{
			const FPerObjectConstants Constants = MakePerObjectConstants(World, bReceiveShadow);
			std::memcpy(Item.ObjectData.data(), &Constants, sizeof(Constants));
			Item.ObjectDataSize = sizeof(Constants);
		}
		return Item;
	}

	FMeshDrawBatch::FItem BuildBatchItem(const FMatrix& World, const FMatrix& View, const FMatrix& Projection,
	                                     uint32 Slot, uint32 StartIndex, uint32 IndexCount,
	                                     const void* ObjectKey)
	{
		FMeshDrawBatch::FItem Item{};
		Item.ObjectSlot = Slot;
		Item.StartIndex = StartIndex;
		Item.IndexCount = IndexCount;
		Item.ObjectKey = ObjectKey;

		FForwardObjectConstants Constants{};
		Constants.World = World;
		Constants.View = View;
		Constants.Projection = Projection;
		Constants.NormalMatrix = World.Inverse().GetTransposed();
		std::memcpy(Item.ObjectData.data(), &Constants, sizeof(Constants));
		Item.ObjectDataSize = sizeof(Constants);

		return Item;
	}

	FMeshDrawBatch::FItem MakeShadowItem(const FPrimitiveSceneProxy* Proxy, uint32 StartIndex, uint32 IndexCount,
	                                     const FVector& PreTranslation)
	{
		FMeshDrawBatch::FItem Item{};
		Item.StartIndex = StartIndex;
		Item.IndexCount = IndexCount;
		Item.ObjectKey = Proxy;

		// ShadowDepth.hlsl은 법선(NormalMatrix)으로 기울기 bias를 구하므로 오브젝트 상수 전체를 올린다
		FPerObjectConstants Constants = MakePerObjectConstants(Proxy->GetLocalToWorld());
		Constants.World.M[3][0] += PreTranslation.X;
		Constants.World.M[3][1] += PreTranslation.Y;
		Constants.World.M[3][2] += PreTranslation.Z;
		std::memcpy(Item.ObjectData.data(), &Constants, sizeof(Constants));
		Item.ObjectDataSize = sizeof(Constants);
		return Item;
	}
}

bool FMeshPassProcessor::Build(const FSceneView& View, const FRenderQueue& Queue,
                               const FSceneRenderData* SceneData, EMeshPassType Pass,
                               TArray<FMeshDrawBatch>& Out, ELightingModel LightingModel) const
{
	Out.Reset();
	if (Pass != EMeshPassType::Translucent && Pass != EMeshPassType::BillboardOpaque && SceneData)
	{
		TArray<const FStaticDrawGroup*> Groups;
		for (const FSceneRenderData::FGatherChunk& Chunk : SceneData->GatherChunks)
			for (const FStaticDrawGroup& Group : Chunk.Groups)
				if (!Group.Items.empty())
					Groups.Add(&Group);

		std::sort(Groups.begin(), Groups.end(),
		          [](const FStaticDrawGroup* A, const FStaticDrawGroup* B)
		          { return GroupKey(*A) < GroupKey(*B); }); // 재질, 메시 순으로 정렬

		for (const FStaticDrawGroup* Group : Groups)
		{
			FMeshDrawBatch Batch; // Render에게 전달하는 최종 구조체
			if (!BuildBatch(Batch, Group->Mesh->GetVertexFactory(), Group->Material, Pass, View, LightingModel))
				return false;
			if (!ApplyVertexFactory(Batch))
				continue;

			Batch.Items.Reserve(static_cast<uint32>(Group->Items.size()));

			for (const FStaticDrawItem& Item : Group->Items)
				Batch.Items.Add(BuildItem(Item.Proxy->GetLocalToWorld(), View, Pass, Item.Slot,
				                          Item.StartIndex, Item.IndexCount, Item.Proxy, Item.Proxy->bReceiveShadow,
				                          Item.Proxy->bReceivesDecals, Item.Proxy->GetBounds(), Item.Proxy));
			Out.Add(std::move(Batch));
		}
	}
	const uint32 Begin = Pass == EMeshPassType::Translucent ? Queue.GetFirstTranslucentIndex() : 0;
	const uint32 End = Pass == EMeshPassType::Translucent ? Queue.Num() : Queue.GetFirstTranslucentIndex();
	Out.Reserve(Out.Num() + End - Begin);
	for (uint32 Index = Begin; Index < End; ++Index)
	{
		const FRenderPacket& Packet = Queue[Index];
		const bool bBillboard = Packet.Material && Packet.Material->PSOType == EPSOType::Billboard;
		if ((Pass == EMeshPassType::BillboardOpaque) != bBillboard)
			continue;
		if (Pass == EMeshPassType::WorldNormal &&
		    (!Packet.Material || !Packet.Material->IsOpaqueOrMasked() || bBillboard))
			continue;
		FMeshDrawBatch Batch;
		if (!BuildBatch(Batch, GetPacketVertexFactory(Packet), Packet.Material, Pass, View,
		                LightingModel, Packet.MaterialParamData, Packet.MaterialParamDataSize))
			return false;
		if (!ApplyVertexFactory(Batch))
			continue;

		const uint32 IndexCount = Packet.IndexCount ? Packet.IndexCount : Batch.VertexFactory->GetIndexBuffer()->GetIndexCount();
		const void* ObjectKey = Packet.Proxy ? static_cast<const void*>(Packet.Proxy) : static_cast<const void*>(&Packet);

		Batch.Items.Add(BuildItem(PacketWorld(Packet), View, Pass, Packet.Slot,
		                          Packet.StartIndex, IndexCount, ObjectKey, !Packet.Proxy || Packet.Proxy->bReceiveShadow,
		                          !Packet.Proxy || Packet.Proxy->bReceivesDecals,
		                          Packet.Proxy ? Packet.Proxy->GetBounds() : ConservativeBounds, Packet.Proxy));
		Batch.Items.Last().InstanceCount = Packet.InstanceCount;

		// 웨이트 히트맵: Forward 패스의 오브젝트 상수 빈 칸(ShadowFlags.y)에 본 번호 + 1을 싣는다
		if (Pass == EMeshPassType::Forward && Packet.DebugWeightBone >= 0)
		{
			auto* Constants = reinterpret_cast<FForwardObjectConstants*>(Batch.Items.Last().ObjectData.data());
			Constants->ShadowFlags.Y = static_cast<float>(Packet.DebugWeightBone + 1);
		}

		Out.Add(std::move(Batch));
	}
	return true;
}

bool FMeshPassProcessor::Build(const TArray<FPrimitiveSceneProxy*>& ShadowMeshes, EPSOType PSOType, TArray<FMeshDrawBatch>& OutBatches,
                               const FVector& PreTranslation)
{
	OutBatches.Reset();

	const FPipelineState* Pipeline = FRenderResourceManager::GetPSO(PSOType);
	if (!Pipeline || !Pipeline->Shader || !Pipeline->Shader->VertexShader)
		return false;

	FRenderQueue DynamicPackets;
	for (FPrimitiveSceneProxy* Proxy : ShadowMeshes)
	{
		if (!Proxy || !Proxy->GetRenderState())
			continue;

		if (!Proxy->GetMesh() && Proxy->HasDynamicMeshElements())
		{
			DynamicPackets.Reset();
			Proxy->GetDynamicMeshElements(DynamicPackets, FViewContext());
			for (const FRenderPacket& Packet : DynamicPackets)
			{
				const FVertexFactory* VertexFactory = GetPacketVertexFactory(Packet);
				if (!VertexFactory || !VertexFactory->IsValid() || Packet.IndexCount == 0)
					continue;
				if (Packet.Material && Packet.Material->IsTranslucent())
					continue;
				FMeshDrawBatch Batch{};
				Batch.Pipeline = *Pipeline;
				// 두께 없는 양면 메시(천, 잎)는 광원 쪽에서 뒷면만 보일 수 있으므로 컬링하지 않는다
				if (Packet.Material && Packet.Material->bTwoSided)
					Batch.Pipeline.RasterizerState = ERasterizerState::SolidNone;
				Batch.VertexFactory = VertexFactory;
				if (!ApplyVertexFactory(Batch))
					continue;
				Batch.bUseObjectSlots = false;
				Batch.bBindViewConstants = true;
				Batch.ObjectBindingSlot = 2;
				Batch.ObjectStages = EShaderBindFlagBits::Vertex;
				ApplyShadowMask(Batch, Packet.Material);
				Batch.Items.Add(MakeShadowItem(Proxy, Packet.StartIndex, Packet.IndexCount, PreTranslation));
				Batch.Items.Last().InstanceCount = Packet.InstanceCount;
				OutBatches.Add(std::move(Batch));
			}
			continue;
		}

		UStaticMesh* Mesh = Proxy->GetMesh();
		if (!Mesh || !Proxy->GetRenderState())
			continue;

		const FMeshRenderState& MeshState = *Proxy->GetRenderState();

		FMeshDrawBatch Batch{};
		Batch.Pipeline = *Pipeline;
		Batch.VertexFactory = Mesh->GetVertexFactory();
		Batch.bUseObjectSlots = false;
		Batch.bBindViewConstants = true;
		Batch.ObjectBindingSlot = 2;
		Batch.ObjectStages = EShaderBindFlagBits::Vertex;

		if (!Batch.VertexFactory->IsValid())
		{
			return false;
		}
		if (!ApplyVertexFactory(Batch))
			continue;
		ApplyShadowMask(Batch, nullptr);
		// 양면 섹션은 컬링을 끈 배치로 따로 모은다
		FMeshDrawBatch TwoSidedBatch = Batch;
		TwoSidedBatch.Pipeline.RasterizerState = ERasterizerState::SolidNone;

		for (int32 Index = 0; Index < MeshState.Sections.Num(); ++Index)
		{
			const FCachedMeshSection& Section = Proxy->GetSection(Index);
			if (Section.IndexCount == 0)
				continue;
			// 반투명 섹션은 그림자를 드리우지 않는다
			if (Section.Material && Section.Material->IsTranslucent())
				continue;

			// Masked 섹션은 머티리얼 텍스처와 클립 값이 섹션마다 달라서 배치를 따로 만든다
			if (Section.Material && Section.Material->IsMasked())
			{
				FMeshDrawBatch MaskedBatch = Section.Material->bTwoSided ? TwoSidedBatch : Batch;
				MaskedBatch.Items.Reset();
				ApplyShadowMask(MaskedBatch, Section.Material);
				MaskedBatch.Items.Add(MakeShadowItem(Proxy, Section.StartIndex, Section.IndexCount, PreTranslation));
				OutBatches.Add(std::move(MaskedBatch));
				continue;
			}

			FMeshDrawBatch& Target = (Section.Material && Section.Material->bTwoSided) ? TwoSidedBatch : Batch;
			Target.Items.Add(MakeShadowItem(Proxy, Section.StartIndex, Section.IndexCount, PreTranslation));
		}

		if (!Batch.Items.IsEmpty())
			OutBatches.Add(std::move(Batch));
		if (!TwoSidedBatch.Items.IsEmpty())
			OutBatches.Add(std::move(TwoSidedBatch));
	}
	return true;
}
