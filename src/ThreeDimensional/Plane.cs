using Microsoft.Xna.Framework;
using System;

namespace Geometry.ThreeDimensional
{
    // The plane doesn't really have a local origin
    // Everything is relative to the "maths" origin
    public class Plane
    {
        /// <summary>
        /// Plane normal.  Points (x) on the plane satisfy: Vector3.Dot(n, x) = d
        /// </summary>
        public Vector3 Normal { get; set; }

        /// <summary>
        /// D = Vector3.Dot(n, p) for a given point (p) on the plane (distance along normal)
        /// </summary>
        public float D { get; set; }

        /// <summary>
        /// 2D halfwidth for visual representation of plane
        /// </summary>
        public float HalfWidth { get; set; }

        /// <summary>
        /// Private constructor for use by the XNB deserialiser
        /// </summary>
        private Plane() { }

        /// <summary>
        /// Given 3 noncollinear points [ordered Counter-Clockwise] compute the plane equation
        /// (3 or more points are said to be collinear if they lie on a single straight line)
        /// </summary>
        /// <param name="a">Point 1 (As Vector3)</param>
        /// <param name="b">Point 2 (As Vector3)</param>
        /// <param name="c">Point 3 (As Vector3)</param>
        public Plane(Vector3 a, Vector3 b, Vector3 c)
        {
            Normal = Vector3.Normalize(Vector3.Cross(b - a, c - a));
            D = Vector3.Dot(Normal, a);
            HalfWidth = 1.0f;
        }

        /// <summary>
        /// If we have a point available that we know lies on the plane we can compute the distance 
        /// directly from this point and the plane normal using: d = Vector3.Dot(n, p)
        /// </summary>
        /// <param name="n">Plane normal</param>
        /// <param name="p">Point that lies on the plane</param>
        public Plane(Vector3 n, Vector3 p)
        {
            Normal = Vector3.Normalize(n);
            D = Vector3.Dot(Normal, p);
            HalfWidth = 1.0f;
        }

        public float DistanceToPoint(Vector3 point)
        {
            return Vector3.Dot(Normal, point) - D;
        }

        /// <summary>
        /// Closest point on plane to point (projects point on to the plane)
        /// </summary>
        public Vector3 ClosestPtPointPlane(Vector3 point)
        {
            float t = Vector3.Dot(Normal, point) - D;
            return point - t * Normal;
        }

        public static Vector3 IntersectionOf3Planes(Plane p1, Plane p2, Plane p3)
        {
            Vector3 u = Vector3.Cross(p2.Normal, p3.Normal);
            float denom = Vector3.Dot(p1.Normal, u);

            if (Math.Abs(denom) < ToolBox.EPSILON)
            {
                // Planes do not intersect at a point
                return Vector3.Zero;
            }

            return (p1.D * u + Vector3.Cross(p1.Normal, p3.D * p2.Normal - p2.D * p3.Normal)) / denom;
        }

        private static Matrix RotationFrom(Vector3 normal)
        {
            normal.Normalize();

            Matrix rot = Matrix.Identity;
            rot.Backward = normal;
            rot.Up = Vector3.Cross(Vector3.UnitY, normal);

            if (rot.Up.LengthSquared() < ToolBox.EPSILON)
            {
                rot.Up = Vector3.Cross(-Vector3.UnitZ, normal);
            }

            rot.Up.Normalize();
            rot.Right = Vector3.Cross(rot.Up, normal);
            rot.Right.Normalize();

            return rot;
        }
    }
}