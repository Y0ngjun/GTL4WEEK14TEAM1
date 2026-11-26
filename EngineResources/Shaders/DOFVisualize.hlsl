#include "DOFCommon.hlsli"

cbuffer Constants : register(b0)
{
    float A;
    float B;
    uint bIsPerspective;
    float FocalDistance;

    float SceneColorInvWidth;
    float SceneColorInvHeight;
    float CoC;
    float MaxForegroundRadiusPixels;

    int BladeCount;
    float MaxBackgroundRadiusPixels;
    float BokehRoundness;
    float MaxDepthBlurRadiusPixels;

    float InvDepthBlurHalfDistance;
    float FocusRange;
    int2 Padding;
};


Texture2D<float> DepthTexture : register(t0);
SamplerState PointSampler : register(s0);
struct VSOutput { float4 Position : SV_POSITION; float2 UV : TEXCOORD0; };
VSOutput mainVS(uint ID : SV_VertexID) {
 VSOutput O; O.UV = float2((ID << 1) & 2, ID & 2);
 O.Position = float4(O.UV * float2(2,-2) + float2(-1,1),0,1); return O;
}
float4 mainPS(VSOutput I) : SV_Target {
 float Z = ConvertFromDeviceZ(DepthTexture.Sample(PointSampler,I.UV),A,B,bIsPerspective);
 float C = ComputeCoCRadius(Z,FocalDistance,CoC,MaxDepthBlurRadiusPixels,InvDepthBlurHalfDistance,FocusRange,MaxBackgroundRadiusPixels);
 float Amount = saturate(abs(C) / max(C < 0 ? MaxForegroundRadiusPixels : MaxBackgroundRadiusPixels, 0.001f));
 return float4(lerp(float3(0,0.6,0), C < 0 ? float3(0,0.2,1) : float3(1,0.15,0), Amount),1);
}
