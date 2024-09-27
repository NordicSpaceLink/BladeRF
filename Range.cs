namespace NordicSpaceLink.BladeRF;

public readonly struct Range
{
    public float Min { get; }
    public float Max { get; }
    public float Step { get; }

    public Range(float min, float max, float step)
    {
        Min = min;
        Max = max;
        Step = step;
    }
}