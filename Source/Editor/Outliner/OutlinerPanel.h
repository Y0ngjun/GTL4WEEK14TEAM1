#pragma once

#include "Editor/EditorUI/EditorPanel.h"
#include "Editor/KurisuEd/EditorSelection.h"

#include <functional>

class UWorld;
class ULevel;
class AActor;
class UActorComponent;
class UPrimitiveComponent;
class USceneComponent;
class FEditorUndoHistory;

using DeleteActorCallback = std::function<void(AActor*)>;
using DeleteActorsCallback = std::function<void(const TArray<AActor*>&)>;

class FOutlinerPanel : public IEditorPanel
{
public:
	FOutlinerPanel() = default;
	~FOutlinerPanel() = default;

	// 선택 상태의 원본. 아웃라이너는 읽고 바꿀 뿐 들고 있지 않는다. 바뀐 결과는 Selection.OnChanged로 퍼진다.
	void SetSelection(FEditorSelection* InSelection) { Selection = InSelection ? InSelection : &LocalSelection; }
	void SetDeleteActorCallback(DeleteActorCallback InCallback) { DeleteCallback = std::move(InCallback); }
	void SetDeleteActorsCallback(DeleteActorsCallback InCallback) { MultiDeleteCallback = std::move(InCallback); }
	void SetUndoHistory(FEditorUndoHistory* InHistory) { UndoHistory = InHistory; }

public:
	bool Init() override;
	void Tick(float DeltaTime) override;
	void OnRender() override;
	const char* GetPanelName() const override { return "Outliner"; }

	void SetWorld(UWorld* InWorld) { World = InWorld; };

	void DrawActors(ULevel* Level);
	void DrawActorNode(AActor* Actor);
	void SelectActor(AActor* Actor, bool bAdditiveToggle = false, bool bRangeSelect = false);
	void SelectActors(const TArray<AActor*>& Actors);
	void OnActorDestroyed(AActor* Actor);

	AActor* GetSelectedActor() const { return Selection->GetPrimaryActor(); }
	const TArray<AActor*>& GetSelectedActors() const { return Selection->GetActors(); }
	bool IsActorSelected(const AActor* Actor) const;
	bool IsFocused() const { return bFocused; }

private:
	void PruneInvalidSelections(ULevel* Level);

	UWorld* World = nullptr;
	// SetSelection 전에도 안전하게 쓰도록 자기 몫을 하나 둔다.
	FEditorSelection LocalSelection;
	FEditorSelection* Selection = &LocalSelection;
	TArray<AActor*> VisibleActorOrder;

	DeleteActorCallback DeleteCallback;
	DeleteActorsCallback MultiDeleteCallback;
	FEditorUndoHistory* UndoHistory = nullptr;
	AActor* PendingDeleteActor = nullptr;
	TArray<AActor*> PendingDeleteActors;
	bool bFocused = false;
	AActor* RenamingActor = nullptr;
	char RenameBuffer[256] = {};
	bool bFocusRename = false;
	TMap<AActor*, TArray<AActor*>> ActorChildren;
	USceneComponent* PendingAttachChild = nullptr;
	USceneComponent* PendingAttachParent = nullptr;
	bool bPendingDetach = false;

	// 검색어. 이름에 포함되거나(대소문자 무시) 자손 중에 포함된 액터만 보인다.
	char SearchBuffer[128] = {};
	bool IsFiltering() const { return SearchBuffer[0] != '\0'; }
	bool MatchesSearch(AActor* Actor) const;
};
