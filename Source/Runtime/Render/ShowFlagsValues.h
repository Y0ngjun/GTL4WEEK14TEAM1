// 표시(Show) 플래그 목록. UE의 ShowFlagsValues.inl과 같은 자리다.
// 여기 한 줄을 추가하면 FEngineShowFlags 멤버, 에디터 Show 메뉴, editor.ini 저장/불러오기가 함께 생긴다.
// 이 파일은 SHOWFLAG 매크로를 정의한 뒤에만 포함한다. #pragma once를 두지 않는다.
//
// SHOWFLAG(이름, 게임 기본값, 에디터 기본값, 분류, 메뉴 아이콘, 메뉴 표시 이름)
//   Render 분류는 Show 메뉴에 나오지 않고 Render 메뉴가 바꾼다. 저장은 한다.
//   Hidden 분류는 뷰 모드나 코드가 매 프레임 정하므로 메뉴에 나오지 않고 저장하지도 않는다.

#ifndef SHOWFLAG
#error "SHOWFLAG(Name, bGameDefault, bEditorDefault, Category, Icon, DisplayName) must be defined before including ShowFlagsValues.h"
#endif

// ---------- Common ----------
SHOWFLAG(Bounds,             false, false, Common,      "Bounds",       "Bounding Boxes")
SHOWFLAG(Decals,             true,  true,  Common,      "Decals",       "Decals")
// 에디터 선(그리드·바운드·디버그 선) 전체. 끄면 배치 선을 그리지 않는다
SHOWFLAG(Grid,               false, true,  Common,      "Grid",         "Grid")
SHOWFLAG(Particles,          true,  true,  Common,      "Particles",    "Particles")
SHOWFLAG(SkeletalMeshes,     true,  true,  Common,      "SkeletalMesh", "Skeletal Meshes")
// 씬의 메시 전체 (끄면 프리미티브 패스를 건너뛴다)
SHOWFLAG(StaticMeshes,       true,  true,  Common,      "StaticMesh",   "Static Meshes")

// ---------- Physics: PIE에서만 그린다 ----------
SHOWFLAG(PhysicsBodies,      false, false, Physics,     "StaticMesh",   "Physics Bodies")
SHOWFLAG(PhysicsConstraints, false, false, Physics,     "SkeletalMesh", "Physics Constraints")
SHOWFLAG(ClothCollision,     false, false, Physics,     "SkeletalMesh", "Cloth Collision")

// ---------- Post Process ----------
SHOWFLAG(CameraOverlay,      true,  true,  PostProcess, "Camera",       "Camera Fade / Letterbox")
SHOWFLAG(Vignette,           true,  true,  PostProcess, "Lighting",     "Vignette")
SHOWFLAG(DepthOfField,       true,  true,  PostProcess, "Camera",       "Depth of Field")
SHOWFLAG(LensFlares,         true,  true,  PostProcess, "Lighting",     "Lens Flares")

// ---------- Developer ----------
SHOWFLAG(CascadeBounds,      false, false, Developer,   "Cascade",      "Cascade Bounds")
SHOWFLAG(ObjectUUID,         false, false, Developer,   "Pivot",        "Object UUID")
SHOWFLAG(RootBone,           false, false, Developer,   "SkeletalMesh", "Root Bone")

// ---------- Render: Show 메뉴 대신 Render 메뉴가 바꾼다 ----------
SHOWFLAG(AntiAliasing,       false, false, Render,      nullptr,        "Anti-Aliasing (FXAA)")
// 후처리 값(bAutoExposure)이 자동 노출을 원해도, 이 뷰에서 끄면 노출을 고정해 조명 작업을 할 수 있다
SHOWFLAG(EyeAdaptation,      true,  true,  Render,      nullptr,        "Eye Adaptation")

// ---------- Hidden: 뷰 모드·코드가 매 프레임 정한다. 메뉴에 나오지 않고 저장하지 않는다 ----------
SHOWFLAG(Fog,                   true,  true,  Hidden, nullptr, "Fog")
SHOWFLAG(VisualizeSceneDepth,   false, false, Hidden, nullptr, "Visualize Scene Depth")
SHOWFLAG(VisualizeCascades,     false, false, Hidden, nullptr, "Visualize Shadow Cascades")
SHOWFLAG(VisualizeWorldNormal,  false, false, Hidden, nullptr, "Visualize World Normal")
SHOWFLAG(VisualizeLightCulling, false, false, Hidden, nullptr, "Visualize Light Culling")

SHOWFLAG(VisualizeDepthOfField, false, false, Developer, "Camera", "Visualize Depth of Field")
