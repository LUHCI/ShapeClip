using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;


/// <summary>
/// Describes how well we've detected a shape clip
/// </summary>
public enum DetectionState
{
    /// <summary>
    /// We think this is a shape clip, but are not sure because it has more than four vertices
    /// </summary>
    Candidate,

    /// <summary>
    /// This is a shape clip, but we couldn't detect the two LDRs, so the orientation is not complete
    /// </summary>
    SemiOriented,

    /// <summary>
    /// This is a shape clip of which we've found the LDRs, so it's orientation should be correct
    /// </summary>
    FullyOriented
}

/// <summary>
/// Simple two-component vector structure
/// </summary>
public struct Vec2F
{
    public Vec2F(OpenCvSharp.CvSize size) : this(size.Width, size.Height) { }

    public Vec2F(float x, float y)
    {
        this.X = x;
        this.Y = y;
    }

    public float[] ToArray()
    {
        return new float[] { X, Y };
    }

    public float Length
    {
        get
        {
            return (float)Math.Sqrt(X * X + Y * Y);
        }
    }

    public Vec2F Scale(Vec2F factor)
    {
        return new Vec2F(X * factor.X, Y * factor.Y);
    }

    public override string ToString()
    {
        return String.Format("[{0}, {1}]", X, Y);
    }

    public readonly float X, Y;
}

/// <summary>
/// Represents a single shape clip on a plane
/// </summary>
class ShapeClip
{
    /// <summary>
    /// The detection state of this shape clip
    /// </summary>
    public DetectionState DetectionState { get; private set; }

    /// <summary>
    /// The center position of the shape clip
    /// </summary>
    public Vec2F Position { get; private set; }

    /// <summary>
    /// The orientation of the shape clip relative to the X-Axis
    /// </summary>
    public double Angle { get; private set; }

    /// <summary>
    /// The bounding box of the shape clip
    /// </summary>
    public Vec2F BoundingBox { get; private set; }

    public ShapeClip(DetectionState state, Vec2F position, Vec2F boundingBox, double angle)
    {
        this.DetectionState = state;
        this.Position = position;
        this.BoundingBox = boundingBox;
        this.Angle = angle;
    }

}
