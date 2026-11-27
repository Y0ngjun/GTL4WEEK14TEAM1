#pragma once

#include "Components/PrimitiveComponent.h"
#include "LandscapeDataAccess.h"
#include "LandscapeComponent.generated.h"

UCLASS()
class ULandscapeComponent : public UPrimitiveComponent
{
	GENERATED_BODY()

public:
	// 전체 원본에서 이 컴포넌트에 해당하는 영역을 복사한다.
	bool Initialize(const FLandscapeHeightfieldData& Source, int32 InSectionBaseX, int32 InSectionBaseY, int32 InComponentSizeQuads);

	int32 GetSectionBaseX() const { return SectionBaseX; }
	int32 GetSectionBaseY() const { return SectionBaseY; }
	int32 GetComponentSizeQuads() const { return ComponentSizeQuads; }

	const FLandscapeHeightfieldData& GetHeightfieldData() const
	{
		return Heightfield;
	}

private:
	int32 SectionBaseX = 0;
	int32 SectionBaseY = 0;
	int32 ComponentSizeQuads = 0;

	FLandscapeHeightfieldData Heightfield;
};