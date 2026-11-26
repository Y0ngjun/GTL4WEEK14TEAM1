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
    float DeviceZ = SceneDepthTexture.Sample(PointClampSampler, Input.UV);
    float ViewDepth = ConvertFromDeviceZ(DeviceZ, A, B, bIsPerspective);
    
    float CoCRadius = ComputeCoCRadius(ViewDepth, FocalDistance, CoC, MaxDepthBlurRadiusPixels, InvDepthBlurHalfDistance, FocusRange, MaxBackgroundRadiusPixels);
    CoCRadius = min(CoCRadius, MaxBackgroundRadiusPixels);
    
    if (CoCRadius <= 0.f)
    {
        return float4(0.f, 0.f, 0.f, 0.f);
    }
    
    float4 FinalColor = float4(0.f, 0.f, 0.f, 0.f);
    float2 TexelSize = float2(SceneColorInvWidth, SceneColorInvHeight);

    const int MaxRingNumber = CoCRadius;
    float TotalWeight = 0.f;
    for (int RingNumber = 0; RingNumber <= MaxRingNumber; RingNumber++)
    {
        int SamplesInRing = GetSamplesInRing(RingNumber);

        float RadiusFactor = RingNumber / float(MaxRingNumber);
        for (int Index = 0; Index < SamplesInRing; Index++)
        {
            float2 ApertureOffset = GetRingSampleOffset(RingNumber, Index, BladeCount, BokehRoundness);
            float2 Offset = ApertureOffset * TexelSize * CoCRadius * RadiusFactor;

            float AroundDeviceZ = SceneDepthTexture.SampleLevel(PointClampSampler, Input.UV + Offset, 0);
            float AroundViewDepth = ConvertFromDeviceZ(AroundDeviceZ, A, B, bIsPerspective);

            float DepthDelta = max(ViewDepth - AroundViewDepth, 0.f);
            float Weight = 1.f - smoothstep(0.f, 1.f, DepthDelta);

            float4 AroundColor = SceneColorTexture.SampleLevel(LinearClampSampler, Input.UV + Offset, 0);

            if (Weight < 1.f)
            {
                float AroundCoCRadius = ComputeCoCRadius(AroundViewDepth, FocalDistance, CoC, MaxDepthBlurRadiusPixels, InvDepthBlurHalfDistance, FocusRange, MaxBackgroundRadiusPixels);

                if (AroundCoCRadius > 0.f)
                {
                    float SourceRadiusPixels = min(AroundCoCRadius, MaxBackgroundRadiusPixels);
                    float SampleDistancePixels = length(Offset / TexelSize);
                    float DirectionalSourceRadius = SourceRadiusPixels * length(ApertureOffset);
                    float SourceCoverage = saturate(DirectionalSourceRadius - SampleDistancePixels);

                    Weight = max(Weight, SourceCoverage);
                }
            }

            FinalColor.rgb += AroundColor.rgb * Weight;
            TotalWeight += Weight;
        }
    }

    if (TotalWeight > 0.f)
    {
        FinalColor.rgb /= TotalWeight;
        FinalColor.a = 1.f;
    }
    
    return FinalColor;
}
