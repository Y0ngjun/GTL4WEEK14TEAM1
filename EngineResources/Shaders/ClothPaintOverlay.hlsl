#pragma pack_matrix(row_major)

cbuffer ViewConstants : register(b0)
{
    matrix ViewProjection;
};

struct PaintVertex
{
    float3 Position : POSITION;
    float4 Color : COLOR;
};

struct PaintPixel
{
    float4 Position : SV_POSITION;
    float4 Color : COLOR;
};

PaintPixel mainVS(PaintVertex Input)
{
    PaintPixel Output;
    Output.Position = mul(float4(Input.Position, 1.0f), ViewProjection);
    // Match the preview depth while allowing for CPU/GPU skinning roundoff.
    Output.Position.z -= 0.000001f * Output.Position.w;
    Output.Color = Input.Color;
    return Output;
}

float4 mainPS(PaintPixel Input) : SV_TARGET
{
    return Input.Color;
}
