#pragma once

#include "UObject/Object.h"
#include "UObject/Property.h"
#include "Render/Renderer.h"

#include <filesystem>
#include <functional>

#include "AssetManager.generated.h"

enum class EAssetType
{
	None,
	Texture,
	Font,
	StaticMesh,
	SkeletalMesh,
	Material
};

class UStaticMesh;
class USkeletalMesh;
class USoundWave;
class UAnimSequence;
class UBlendSpace1D;
class UParticleSystem;
class UPhysicalMaterial;
struct FSkeletalMaterialSlot;

namespace fs = std::filesystem;

UCLASS()
class UAssetManager : public UObject
{
	GENERATED_BODY()
private:
	UAssetManager() = default;

	UAssetManager(const UAssetManager& src) = delete;
	UAssetManager& operator=(const UAssetManager& src) = delete;

	using FAssetLoadProgress = std::function<void(int32 Loaded, int32 Total, const FString& Path)>;

public:
	static UAssetManager& Get();
	static void Init(const FAssetLoadProgress& OnProgress = nullptr);
	static void Shutdown();

	// KeyPrefix가 있으면 키와 경로를 KeyPrefix + 상대 경로로 등록한다 (엔진 콘텐츠의 "Engine/").
	void ScanAssets(const fs::path& AssetRoot, const FAssetLoadProgress& OnProgress, const FString& KeyPrefix = {});
	void LoadAsset(const FString& Key, const FString& Path);

	void CreateDefaultTextures();
	void CreateDefaultMeshes();
	void CreateDefaultMaterial();
	void CreateBillboardMaterial();
	void CreateParticleMaterial();
	void CreateEditorArrowMaterial();

	// 경로는 "Content/Models/a.obj"와 "Models/a.obj"를 같은 에셋으로 본다.
	// 아직 로드되지 않았으면 여기서 로드하므로 GPU 자원을 만들 수 있는 메인 스레드에서만 부른다.
	template <typename T>
	static T* GetAssetByPath(const FString& Path)
	{
		URenderAsset* Asset = Get().FindOrLoadAsset(Path);

		if (Asset == nullptr || !Asset->IsA<T>())
		{
			return nullptr;
		}
		return Cast<T>(Asset);
	}

	// 로드를 일으키지 않고 이미 로드된 에셋만 찾는다
	static URenderAsset* FindLoadedAsset(const FString& Path);

	// 스캔으로 등록됐지만 아직 로드되지 않은 에셋의 타입. 그 외에는 None
	static EAssetType GetUnloadedAssetType(const FString& Path);
	static TArray<FString> GetUnloadedAssetPaths(EAssetType Type);

	void RegisterAsset(const FString& Key, URenderAsset* Asset);
	// 디스크에서 새로 만든 에셋 파일의 키 -> 실제 경로를 등록한다 (스캔 뒤에 생긴 .mat 등)
	void RegisterAssetPath(const FString& Key, const FString& FullPath);

	// ---------- 머티리얼 에셋(.mat, JSON) ----------
	UMaterial* LoadMaterial(const FString& InPath);
	// Path가 비면 Material의 경로에 저장한다. 저장한 머티리얼은 에셋(인스턴스 아님)으로 등록된다.
	static bool SaveMaterial(UMaterial* Material, const FString& Path, FString& OutError);
	// 기본값으로 .mat 파일을 만들고 등록한다
	static UMaterial* CreateMaterialAsset(const FString& Path, FString& OutError);
	// 콘텐츠의 .mat(로드 여부 무관)과 내장 머티리얼 키. .mat을 앞에 둔다
	static TArray<FString> GetAllMaterialAssetPaths();

	// UStaticMesh* GetMesh(FString InName);
	// UStaticMesh* GetMesh(EPrimitiveType Type); // 오버로드(수정 중)

	UTexture2D* LoadTexture(const FString& InPath, bool bGenerateMips = true);
	UFont* LoadFontAtlas(const FString& JsonPath, const FString& AtlasTexturePath);
	USoundWave* LoadSound(const FString& Path);
	static UStaticMesh* LoadObjStaticMesh(const FString& Path);
	static URenderAsset* LoadFbxMesh(const FString& Path, FString& OutError);
	static USkeletalMesh* LoadSkeletalMeshAsset(const FString& Path, FString& OutError);
	static USkeletalMesh* LoadSkeletalMesh(const FString& Path, FString& OutError);
	static bool CreateSkeletalMeshAsset(const FString& BakePath, FString& OutPath, FString& OutError);
	static UAnimSequence* LoadAnimSequence(const FString& Path);
	static UBlendSpace1D* LoadBlendSpace(const FString& Path);
	static UParticleSystem* LoadParticleSystem(const FString& Path);
	static UPhysicalMaterial* LoadPhysicalMaterial(const FString& Path);
	static void RegisterPhysicalMaterial(UPhysicalMaterial* Material);

	// 원본 좌표축이 엔진과 다른 메시를 로드 후 돌린다 (정점·법선·AABB·삼각형 BVH·GPU 정점 버퍼를 다시 만든다).
	// Rotate는 회전(길이·손잡이 보존)이어야 한다.
	static bool ReorientStaticMesh(UStaticMesh& Mesh, const std::function<FVector(const FVector&)>& Rotate);

	// OBJ 스태틱 메시(키가 .obj로 끝나는 것)마다 부른다. 아직 로드되지 않은 메시는 먼저 로드한다. 기본 도형·기즈모 메시는 제외된다.
	static void ForEachObjStaticMesh(const std::function<void(const FString& Key, UStaticMesh& Mesh)>& Func);
	// OwnerMeshPath가 프로젝트 콘텐츠의 메시면 옆 Materials/ 폴더의 .mat을 쓰거나 처음 한 번 만든다 (UE 임포트와 같다).
	// 비어 있으면(내장 도형 등) 디스크에 쓰지 않고 메모리 인스턴스를 만든다.
	static bool BuildMaterialsFromSlots(const TArray<FStaticMaterialSlot>& Slots, TArray<UMaterial*>& OutMaterials, const FString& OwnerMeshPath = {});
	static bool BuildMaterialsFromSlots(const TArray<FSkeletalMaterialSlot>& Slots, TArray<UMaterial*>& OutMaterials, const FString& OwnerMeshPath = {});

private:
	URenderAsset* FindOrLoadAsset(const FString& Path);

	// 키는 Assets 기준 상대 경로. 로드 가능한 파일의 키 -> 실제 파일 경로
	TMap<FString, FString> AssetPathMap;
	TMap<FString, URenderAsset*> AssetMap;
	TMap<FString, USoundWave*> SoundMap;
	TMap<FString, UAnimSequence*> AnimSequenceMap;
	TMap<FString, UBlendSpace1D*> BlendSpaceMap;
	TMap<FString, UParticleSystem*> ParticleSystemMap;
	TMap<FString, UPhysicalMaterial*> PhysicalMaterialMap;

	// TMap<FString, UStaticMesh*> MeshMap;	// KEY: FILE NAME OR 쉐입 첫글자 대문자
	TMap<FString, TUniquePtr<FShader>> ShaderMap; // KEY: FILE NAME

	// TMap<FString, UTexture2D*> TextureMap;
};
