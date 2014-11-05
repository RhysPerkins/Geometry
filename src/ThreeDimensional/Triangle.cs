using Microsoft.Xna.Framework;
using System;

namespace Geometry.ThreeDimensional
{
    public class Triangle
    {
        public Vector3 A { get; private set; }

        public Vector3 B { get; private set; }

        public Vector3 C { get; private set; }

        public Vector3[] Edges { get; private set; }

        public Vector3 Normal { get; private set; }

        public Plane Plane { get; private set; }

        public Vector4 Colour { get; private set; }

        /// <summary>
        /// Private constructor for use by the XNB deserialiser
        /// </summary>
        private Triangle() { }

        /// <summary>
        /// Given 3 noncollinear Points [ordered Counter-Clockwise] create a Triangle
        /// (3 or more points are said to be collinear if they lie on a single straight line)
        /// </summary>
        /// <param name="a">Point 1 (As Vector3)</param>
        /// <param name="b">Point 2 (As Vector3)</param>
        /// <param name="c">Point 3 (As Vector3)</param>
        public Triangle(Vector3 a, Vector3 b, Vector3 c)
        {
            // Normal and Plane must change if a, b or c are updated
            A = a;
            B = b;
            C = c;

            Edges = new Vector3[3];
            Edges[0] = b - a;
            Edges[1] = c - b;
            Edges[2] = a - c;

            // Same as Plane method of creating a normal
            Normal = Vector3.Cross(b - a, c - a);
            Normal.Normalize();

            Plane = new Plane(a, b, c);
        }

        public void CalculateSlab(Vector3 axis, ref float min, ref float max)
        {
            float v0 = Vector3.Dot(A, axis);
            float v1 = Vector3.Dot(B, axis);
            float v2 = Vector3.Dot(C, axis);

            min = ToolBox.Min3(v0, v1, v2);
            max = ToolBox.Max3(v0, v1, v2);
        }

        public Vector3 ClosestPtPointTriangle(Vector3 point)
        {
            Vector3 ab = B - A;
            Vector3 ac = C - A;

            float v;
            float w;

            // Check if point in vertex region outside A
            Vector3 ap = point - A;
            float d1 = Vector3.Dot(ab, ap);
            float d2 = Vector3.Dot(ac, ap);

            if (d1 <= 0.0f && d2 <= 0.0f)
            {
                // Barycentric coordinates (1,0,0)
                return A;
            }

            // Check if point in vertex region outside B
            Vector3 bp = point - B;
            float d3 = Vector3.Dot(ab, bp);
            float d4 = Vector3.Dot(ac, bp);

            if (d3 >= 0.0f && d4 <= d3)
            {
                // Barycentric coordinates (0,1,0)
                return B;
            }

            // Check if point in edge region of AB, if so return projection of point onto AB
            float vc = (d1 * d4) - (d3 * d2);

            if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
            {
                v = d1 / (d1 - d3);

                // Barycentric coordinates (1-v,v,0)
                return A + v * ab;
            }

            // Check if point in vertex region outside C
            Vector3 cp = point - C;
            float d5 = Vector3.Dot(ab, cp);
            float d6 = Vector3.Dot(ac, cp);

            if (d6 >= 0.0f && d5 <= d6)
            {
                // Barycentric coordinates (0,0,1)
                return C;
            }

            // Check if point in edge region of AC, if so return projection of point onto AC
            float vb = (d5 * d2) - (d1 * d6);

            if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
            {
                w = d2 / (d2 - d6);

                // Barycentric coordinates (1-w,0,w)
                return A + w * ac;
            }

            // Check if point in edge region of BC, if so return projection of point onto BC
            float va = (d3 * d6) - (d5 * d4);

            if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
            {
                w = (d4 - d3) / ((d4 - d3) + (d5 - d6));

                // Barycentric coordinates (0,1-w,w)
                return B + w * (C - B);
            }

            // Check if point is inside face region.  Compute Q through its barycentric coordinates (u,v,w)
            float denom = 1.0f / (va + vb + vc);
            v = vb * denom;
            w = vc * denom;

            // u*a + v*b + w*c || u = va * denom = 1.0f - v - w
            return A + ab * v + ac * w;
        }

        /// <summary>
        /// Determines whether the order of the specified vertices is counter-clockwise with respect to a normal.
        /// </summary>
        /// <param name="n">The triangle normal.</param>
        /// <returns>Returns a value indicating whether the vertices are specified in counter-clockwise order.</returns>
        public bool IsTriangleCCW(Vector3 n)
        {
            // > 0 for counterclockwise  
            // = 0 for none (degenerate)  
            // < 0 for clockwise
            Vector3 c1 = A - B;
            Vector3 c2 = A - C;
            c1 = Vector3.Cross(c1, c2);

            return Vector3.Dot(n, c1) > ToolBox.EPSILON;
        }

        /// <summary>
        /// Get the minimum bounding circle of a triangle
        /// </summary>
        /// <param name="a">Triangle point (vertex) A</param>
        /// <param name="b">Triangle point (vertex) B</param>
        /// <param name="c">Triangle point (vertex) C</param>
        public void MinimumBoundingCircle(Vector2 a, Vector2 b, Vector2 c)
        {
            // If the triangle is obtuse, then the diameter of the bounding circle is the longest edge
            // Otherwise, the bounding circle is the circumcircle
            Vector2 centre = Vector2.Zero;
            float radius;

            float dotABAB = Vector2.Dot(b - a, b - a);
            float dotABAC = Vector2.Dot(b - a, c - a);
            float dotACAC = Vector2.Dot(c - a, c - a);
            float d = 2.0f * (dotABAB * dotACAC - dotABAC * dotABAC);
            Vector2 referencePt = a;

            if (Math.Abs(d) <= ToolBox.EPSILON)
            {
                // TODO:
                // a, b, and c lie on a line. Circle center is center of AABB of the
                // points, and radius is distance from circle center to AABB corner
            }
            else
            {
                float s = (dotABAB * dotACAC - dotACAC * dotABAC) / d;
                float t = (dotACAC * dotABAB - dotABAB * dotABAC) / d;

                // s controls height over AC, t over AB, (1-s-t) over BC
                if (s <= 0.0f)
                {
                    centre = 0.5f * (a + c);
                }
                else if (t <= 0.0f)
                {
                    centre = 0.5f * (a + b);
                }
                else if (s + t >= 1.0f)
                {
                    centre = 0.5f * (b + c);
                    referencePt = b;
                }
                else
                {
                    centre = a + s * (b - a) + t * (c - a);
                }
            }

            radius = (float)Math.Sqrt(Vector2.Dot(centre - referencePt, centre - referencePt));
        }

        /// <summary>
        /// Test if point (p) lies inside the CCW 3D triangle (ABC) [Page 204]
        /// </summary>
        /// <param name="point">Point to test</param>
        /// <returns></returns>
        public bool PointInTriangle(Vector3 point)
        {
            // Translate point and triangle so that point lies at origin
            Vector3 a0 = A - point;
            Vector3 b0 = B - point;
            Vector3 c0 = C - point;

            float ab = Vector3.Dot(a0, b0);
            float ac = Vector3.Dot(a0, c0);
            float bc = Vector3.Dot(b0, c0);
            float cc = Vector3.Dot(c0, c0);

            // Make sure plane normals for pab and pbc point in the same direction
            if (bc * ac - cc * ab < 0.0f)
                return false;

            // Make sure plane normals for pab and pca point in the same direction
            float bb = Vector3.Dot(b0, b0);

            if (ab * bc - ac * bb < 0.0f)
                return false;

            // Otherwise p must be in (or on) the triangle
            return true;
        }

        public Triangle Transform(Matrix transform)
        {
            return new Triangle(
                Vector3.Transform(A, transform),
                Vector3.Transform(B, transform),
                Vector3.Transform(C, transform)
                );
        }
    }
}