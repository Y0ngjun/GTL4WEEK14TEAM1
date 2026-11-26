#ifndef DOF_COMMON_HLSLI
#define DOF_COMMON_HLSLI

float ConvertFromDeviceZ(float DeviceZ, float A, float B, uint bIsPerspective)
{
    if (bIsPerspective != 0)
    {
        return B / (DeviceZ - A);
    }

    return (DeviceZ - B) / A;
}

int GetSamplesInRing(int RingNumber)
{
    const int SampleCountPerRingNumber = 8;

    if (RingNumber == 0)
    {
        return 1;
    }

    return SampleCountPerRingNumber * RingNumber;
}

float2 GetRingSampleOffset(int RingNumber, int Index, int BokehBladeCount, float BokehRoundness)
{
    const float PI = 3.141592;
    const float PI2 = PI * 2.f;
    
    const int SamplesInRing = GetSamplesInRing(RingNumber);
    const float RadianStep = PI2 / float(SamplesInRing);
    const bool bIsEvenRing = (RingNumber % 2) == 0;
    const float StartRadian = bIsEvenRing ? RadianStep * 0.5f : 0.f;
    const float CurrentRadian = StartRadian + RadianStep * Index;

    if (BokehBladeCount == 0 || BokehRoundness >= 1.f)
    {
        return float2(cos(CurrentRadian), sin(CurrentRadian));
    }

    float SectorAngle = PI2 / float(BokehBladeCount);
    float HalfSectorAngle = SectorAngle * 0.5f;
    float AngleInSector = fmod(CurrentRadian, SectorAngle);
    float AngleFromSideNormal = AngleInSector - HalfSectorAngle;

    // Equal steps along a straight blade, blended back to equal angles as it rounds.
    float SideFraction = AngleInSector / SectorAngle;
    float SidePosition = lerp(-1.f, 1.f, SideFraction) * tan(HalfSectorAngle);
    float EqualSideAngle = atan(SidePosition);
    float SampleAngleFromSideNormal = lerp(EqualSideAngle, AngleFromSideNormal, BokehRoundness);
    float SampleRadian = CurrentRadian + (SampleAngleFromSideNormal - AngleFromSideNormal);

    float2 Direction = float2(cos(SampleRadian), sin(SampleRadian));
    float RadiusScale = cos(HalfSectorAngle) / cos(SampleAngleFromSideNormal);
    RadiusScale = lerp(RadiusScale, 1.f, BokehRoundness);

    return Direction * RadiusScale;
}

float ComputeCoCRadius(float ViewDepth, float FocalDistance, float CoC, float MaxDepthBlurRadiusPixels, float InvDepthBlurHalfDistance, float FocusRange = 0.0f, float MaxRadius = 16.0f)
{
    if (ViewDepth > 0.f)
    {
        float LensCoC = CoC * (ViewDepth - FocalDistance) / ViewDepth;
        if (FocusRange > 0.0f) LensCoC = clamp((ViewDepth - FocalDistance) / FocusRange, -1.0f, 1.0f) * MaxRadius;
        float DepthRatio = ViewDepth * InvDepthBlurHalfDistance;
        float DepthRadius = MaxDepthBlurRadiusPixels * (1.f - exp2(-DepthRatio));

        float Radius = max(abs(LensCoC), DepthRadius);
        return LensCoC < 0.f ? -Radius : Radius;
    }

    return 0.f;
}

#endif
