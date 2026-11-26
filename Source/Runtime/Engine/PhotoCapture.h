#pragma once

#include "Core/Types.h"

#include <atomic>

class FTexture2D;

// 셔터를 누르면 다음에 그려지는 화면 한 장에서 뷰파인더 프레임 영역만 잘라 PNG로 저장한다.
// 사진 모드 인스턴스가 요청을 소유하고, 해당 뷰포트만 그 요청을 처리한다.
class FPhotoCapture
{
public:
	// FrameHeightFraction: 화면 높이 대비 프레임 높이, FrameAspect: 프레임 가로/세로
	void Request(float FrameHeightFraction, float FrameAspect);

	// UI를 그리기 직전에 부른다. 요청이 있으면 FinalColor를 읽어 저장한다
	void ExecutePending(FTexture2D* FinalColor);

	// 지금까지 파일로 저장을 끝낸 장 수 (저장은 다른 스레드에서 끝난다)
	static uint32 GetSavedCount() { return SavedCount.load(); }

private:
	bool bPending = false;
	float PendingFrameHeight = 1.0f;
	float PendingFrameAspect = 1.5f;
	static inline std::atomic<uint32> SavedCount{ 0 };
};
