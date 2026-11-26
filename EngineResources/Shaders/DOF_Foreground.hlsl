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
    float4 FinalColor = float4(0.f, 0.f, 0.f, 0.f);
    
    float2 TexelSize = float2(SceneColorInvWidth, SceneColorInvHeight);
    
    int SumCount = 0;
    float TotalWeight = 0.f;
    
    const int MaxRingNumber = 3;
    
    const float PixelPerRing = MaxForegroundRadiusPixels / float(MaxRingNumber);
    for (int RingNumber = 0; RingNumber <= MaxRingNumber; RingNumber++)
    {
        int SamplesInRing = GetSamplesInRing(RingNumber);

        float RadiusPixel = PixelPerRing * RingNumber;
        for (int Index = 0; Index < SamplesInRing; Index++)
        {
            float2 Offset = GetRingSampleOffset(RingNumber, Index, BladeCount, BokehRoundness) * RadiusPixel * TexelSize;
            
            float AroundDeviceZ = SceneDepthTexture.Sample(PointClampSampler, Input.UV + Offset);
            float AroundViewDepth = ConvertFromDeviceZ(AroundDeviceZ, A, B, bIsPerspective);
            float AroundCoCRadius = ComputeCoCRadius(AroundViewDepth, FocalDistance, CoC, MaxDepthBlurRadiusPixels, InvDepthBlurHalfDistance, FocusRange, MaxBackgroundRadiusPixels);
            
            float AbsAroundCoCRadius = abs(AroundCoCRadius);
            if (AroundCoCRadius < 0.f && RadiusPixel <= AbsAroundCoCRadius)
            {
                float4 AroundColor = SceneColorTexture.Sample(LinearClampSampler, Input.UV + Offset);
                FinalColor.rgb += AroundColor.rgb;
                TotalWeight += 1.f;
            }
            
            SumCount++;
        }
    }
    
    if (TotalWeight > 0)
    {
        FinalColor.rgb /= TotalWeight;
    }
    
    FinalColor.a = TotalWeight / float(SumCount);
    FinalColor.rgb *= FinalColor.a;
    
    return FinalColor;
}
