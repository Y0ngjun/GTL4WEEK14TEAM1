#include "EnginePCH.h"
#include "LandscapeComponent.h"

bool ULandscapeComponent::Initialize(const FLandscapeHeightfieldData& Source, int32 InSectionBaseX, int32 InSectionBaseY, int32 InComponentSizeQuads)
{
	if (!Source.IsValid() || InSectionBaseX < 0 || InSectionBaseY < 0 || InComponentSizeQuads < 1)
	{
		return false;
	}

	// 마지막 정점까지 원본 범위 안에 있어야 한다.
	if (int64(InSectionBaseX) + InComponentSizeQuads >= Source.SizeX || int64(InSectionBaseY) + InComponentSizeQuads >= Source.SizeY)
	{
		return false;
	}

	SectionBaseX = InSectionBaseX;
	SectionBaseY = InSectionBaseY;
	ComponentSizeQuads = InComponentSizeQuads;

	const int32 NumVertices = ComponentSizeQuads + 1;

	Heightfield.SizeX = NumVertices;
	Heightfield.SizeY = NumVertices;
	Heightfield.Heights.SetNum(NumVertices * NumVertices);

	for (int32 LocalY = 0; LocalY < NumVertices; ++LocalY)
	{
		for (int32 LocalX = 0; LocalX < NumVertices; ++LocalX)
		{
			const int32 SourceX = SectionBaseX + LocalX;
			const int32 SourceY = SectionBaseY + LocalY;
			
			Heightfield.Heights[LocalY * NumVertices + LocalX] = Source.GetRawHeight(SourceX, SourceY);
		}
	}

	// 샘플 간격과 높이 배율은 부모 루트의 스케일을 상속한다.
	SetRelativeLocation(FVector(float(SectionBaseX), float(SectionBaseY), 0.0f));

	return true;
}