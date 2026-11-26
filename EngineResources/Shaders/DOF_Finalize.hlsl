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

Texture2D<float4> SceneColorTexture : register(t0);
Texture2D<float> SceneDepthTexture : register(t1);
Texture2D<float4> ForegroundSceneColorTexture : register(t2);
Texture2D<float4> BackgroundSceneColorTexture : register(t3);

SamplerState LinearClampSampler : register(s0);
SamplerState PointClampSampler : register(s1);

struct VSOutput
{
    float4 Position : SV_Position;
    float2 UV : TEXCOORD0;
};

VSOutput mainVS(uint VertexID : SV_VertexID)
{
    float2 UV = float2((VertexID << 1) & 2, VertexID & 2);

    VSOutput Output;
    Output.Position = float4(UV * float2(2, -2) + float2(-1, 1), 0, 1);
    Output.UV = UV;

    return Output;
}

float4 mainPS(VSOutput Input) : SV_Target
{
    float4 SceneColor = SceneColorTexture.Sample(LinearClampSampler, Input.UV);
    float4 ForegroundSceneColor = ForegroundSceneColorTexture.Sample(LinearClampSampler, Input.UV);
    float4 BackgroundSceneColor = BackgroundSceneColorTexture.Sample(LinearClampSampler, Input.UV);
    
    float DeviceZ = SceneDepthTexture.Sample(PointClampSampler, Input.UV);
    float ViewDepth = ConvertFromDeviceZ(DeviceZ, A, B, bIsPerspective);
    
    float CoCRadius = ComputeCoCRadius(ViewDepth, FocalDistance, CoC, MaxDepthBlurRadiusPixels, InvDepthBlurHalfDistance, FocusRange, MaxBackgroundRadiusPixels);
 
    float4 FinalColor = SceneColor;
    
    float BackgroundAlphaWeight = saturate(CoCRadius - 0.5f);
    float BackgroundAlpha = BackgroundSceneColor.a * BackgroundAlphaWeight;
    FinalColor.rgb = FinalColor.rgb * (1.f - BackgroundAlpha) + BackgroundSceneColor.rgb * BackgroundAlphaWeight;
    
    FinalColor.rgb = FinalColor.rgb * (1.f - ForegroundSceneColor.a) + ForegroundSceneColor.rgb;
    
    return FinalColor;
}
