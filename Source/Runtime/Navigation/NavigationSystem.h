#pragma once

#include "UObject/Object.h"
#include "Navigation/NavigationDebugData.h"
#include "Navigation/NavigationTypes.h"
#include "NavigationSystem.generated.h"

class FRecastNavMesh;
class UWorld;

// 월드 하나의 내비게이션 메시와 경로 질의를 관리한다.
UCLASS()
class UNavigationSystemV1 : public UObject
{
	GENERATED_BODY()

public:
	UNavigationSystemV1();
	virtual ~UNavigationSystemV1() override;

	static UNavigationSystemV1* GetCurrent(UWorld* World);
	FNavigationBuildResult Build(const FNavAgentProperties& Agent = {});
	FNavigationPathResult FindPathToLocationSynchronously(const FVector& Start, const FVector& Goal);
	bool IsNavigationBuilt() const;
	const FNavigationDebugData& GetDebugData() const { return DebugData; }

	void Clear();
	bool CopyBuiltDataFrom(const UNavigationSystemV1& Source);

private:
	TUniquePtr<FRecastNavMesh> NavMesh;
	FNavigationDebugData DebugData;
};
