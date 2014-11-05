using Microsoft.Xna.Framework;
using System;

namespace Geometry.ThreeDimensional
{
    // [Overlap Resolution] 
    // position += contact.normal * contact.penetration;

    // [Watch Out For]
    // • Degenerate axes are axes with a length of 0
    // • Test for degenerate axes using squared length to avoid square root

    public static class CollisionDetection
    {
        public static bool AABBAABB(AABB a, AABB b, ref Contact contact)
        {
            // Minimum Translation Vector
            // ==========================
            float mtvDistance = float.MaxValue;             // Set current minimum distance (max float value so next value is always less)
            Vector3 mtvAxis = new Vector3();                // Axis along which to travel with the minimum distance

            // Axes of potential separation
            // ============================
            // - Each shape must be projected on these axes to test for intersection:
            //          
            // (1, 0, 0)                    A0 (= B0) [X Axis]
            // (0, 1, 0)                    A1 (= B1) [Y Axis]
            // (0, 0, 1)                    A1 (= B2) [Z Axis]

            // [X Axis]
            if (!TestAxisStatic(Vector3.UnitX, a.MinPoint.X, a.MaxPoint.X, b.MinPoint.X, b.MaxPoint.X, ref mtvAxis, ref mtvDistance))
                return false;

            // [Y Axis]
            if (!TestAxisStatic(Vector3.UnitY, a.MinPoint.Y, a.MaxPoint.Y, b.MinPoint.Y, b.MaxPoint.Y, ref mtvAxis, ref mtvDistance))
                return false;

            // [Z Axis]
            if (!TestAxisStatic(Vector3.UnitZ, a.MinPoint.Z, a.MaxPoint.Z, b.MinPoint.Z, b.MaxPoint.Z, ref mtvAxis, ref mtvDistance))
                return false;

            contact.isIntersecting = true;

            // Calculate Minimum Translation Vector (MTV) [normal * penetration]
            contact.nEnter = Vector3.Normalize(mtvAxis);

            // Multiply the penetration depth by itself plus a small increment
            // When the penetration is resolved using the MTV, it will no longer intersect
            contact.penetration = (float)Math.Sqrt(mtvDistance) * 1.001f;

            return true;
        }

        public static bool AABBOBB(AABB a, OBB b, ref Contact contact)
        {
            // Minimum Translation Vector
            // ==========================
            float mtvDistance = float.MaxValue;             // Set current minimum distance (max float value so next value is always less)
            Vector3 mtvAxis = new Vector3();                // Axis along which to travel with the minimum distance

            Vector3 axis = new Vector3();
            float minA;
            float maxA;
            float minB;
            float maxB;

            // Axes of potential separation
            // ============================
            // - Each shape must be projected on these axes to test for intersection:
            //          
            // (1, 0, 0)                    A0
            // (0, 1, 0)                    A1
            // (0, 0, 1)                    A2
            // 
            // obb.axis(0)                  B0
            // obb.axis(1)                  B1
            // obb.axis(2)                  B2
            // 
            // (1, 0, 0) x obb.axis(0)      A0 x B0
            // (1, 0, 0) x obb.axis(1)      A0 x B1
            // (1, 0, 0) x obb.axis(2)      A0 x B2
            // 
            // (0, 1, 0) x obb.axis(0)      A1 x B0
            // (0, 1, 0) x obb.axis(1)      A1 x B1
            // (0, 1, 0) x obb.axis(2)      A1 x B2
            // 
            // (0, 0, 1) x obb.axis(0)      A2 x B0
            // (0, 0, 1) x obb.axis(1)      A2 x B1
            // (0, 0, 1) x obb.axis(2)      A2 x B2

            // A0, A1, A2
            for (int i = 0; i < 3; ++i)
            {
                axis = a.Axes[i];

                minA = float.MaxValue;
                maxA = float.MinValue;
                minB = float.MaxValue;
                maxB = float.MinValue;

                a.CalculateSlab(axis, ref minA, ref maxA);
                b.CalculateSlab(axis, ref minB, ref maxB);

                if (!TestAxisStatic(axis, minA, maxA, minB, maxB, ref mtvAxis, ref mtvDistance))
                    return false;
            }

            // B0, B1, B2
            for (int i = 0; i < 3; ++i)
            {
                axis = b.Axes[i];

                minA = float.MaxValue;
                maxA = float.MinValue;
                minB = float.MaxValue;
                maxB = float.MinValue;

                a.CalculateSlab(axis, ref minA, ref maxA);
                b.CalculateSlab(axis, ref minB, ref maxB);

                if (!TestAxisStatic(axis, minA, maxA, minB, maxB, ref mtvAxis, ref mtvDistance))
                    return false;
            }

            // Remaining cross product axes
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    axis = Vector3.Cross(a.Axes[i], b.Axes[j]);

                    minA = float.MaxValue;
                    maxA = float.MinValue;
                    minB = float.MaxValue;
                    maxB = float.MinValue;

                    a.CalculateSlab(axis, ref minA, ref maxA);
                    b.CalculateSlab(axis, ref minB, ref maxB);

                    if (!TestAxisStatic(axis, minA, maxA, minB, maxB, ref mtvAxis, ref mtvDistance))
                        return false;
                }
            }

            contact.isIntersecting = true;

            // Calculate Minimum Translation Vector (MTV) [normal * penetration]
            contact.nEnter = Vector3.Normalize(mtvAxis);

            // Multiply the penetration depth by itself plus a small increment
            // When the penetration is resolved using the MTV, it will no longer intersect
            contact.penetration = (float)Math.Sqrt(mtvDistance) * 1.001f;

            return true;
        }

        public static bool AABBPlane(AABB a, Plane b, ref Contact contact)
        {
            // Compute the projection interval radius of b onto L(t) = b.c + t * p.n (calc extents distance relative to plane normal)          
            float r =
                Math.Abs(a.Extents.X * b.Normal.X) +
                Math.Abs(a.Extents.Y * b.Normal.Y) +
                Math.Abs(a.Extents.Z * b.Normal.Z);

            // Compute distance (s) of AABB center from plane 
            float distance = Vector3.Dot(b.Normal, a.Position) - b.D;
            distance = Math.Abs(distance);

            // Intersection occurs when distance s falls within [-r,+r] interval
            if (distance < r)
            {
                contact.nEnter = b.Normal;
                contact.penetration = r - distance;
                return true;
            }

            return false;
        }

        public static bool AABBRay(AABB a, Ray b, ref Contact contact)
        {
            float tMin = 0.0f;

            // Maximum distance ray can travel (makes it a segment, otherwise it would be infinite)
            float tMax = b.Length;

            // For all 3 slabs (x, y, z) [A slab is the space between a pair of parallel planes]
            for (int i = 0; i < 3; ++i)
            {
                if (Math.Abs(b.Direction.Index(i)) < ToolBox.EPSILON)
                {
                    // Ray is parallel to slab.  No hit if origin (p in book) not within slab
                    if (b.Origin.Index(i) < a.MinPoint.Index(i) || b.Origin.Index(i) > a.MaxPoint.Index(i))
                        return false;
                }
                else
                {
                    // Compute intersection t value of ray with near and far plane of slab
                    float ood = 1.0f / b.Direction.Index(i);
                    float t1 = (a.MinPoint.Index(i) - b.Origin.Index(i)) * ood;
                    float t2 = (a.MaxPoint.Index(i) - b.Origin.Index(i)) * ood;

                    // Make (t1) be intersection with near plane, (t2) with far plane
                    if (t1 > t2)
                        ToolBox.Swap(ref t1, ref t2);

                    // Compute the intersection of slab intersection intervals
                    tMin = Math.Max(tMin, t1);
                    tMax = Math.Min(tMax, t2);

                    // Exit with no collision as soon as slab intersection becomes empty
                    if (tMin > tMax)
                        return false;
                }
            }

            // Ray intersects all 3 slabs
            contact.point = b.Origin + tMin * b.Direction;
            contact.nEnter = a.GetNormalFromPoint(contact.point);

            // Intersection distance
            contact.tEnter = tMin;
            contact.tExit = tMax;

            return true;
        }

        public static bool AABBSphere(AABB a, Sphere b, ref Contact contact)
        {
            // Find point (p) on AABB closest to sphere centre
            Vector3 p = a.ClosestPtPointAABB(b.Position);

            // Find the directional vector between the sphere and this AABB
            Vector3 d = p - b.Position;

            // Find  distance between centre of sphere and centre of AABB
            float distance = d.Length();

            if (distance > b.Radius)
                return false;

            // Check to see if axis is degenerate
            if (distance < 0.001f)
                return false;

            contact.nEnter = Vector3.Normalize(d);
            contact.penetration = (b.Radius - distance) * 1.001f;
            contact.point = p;

            return true;
        }

        public static bool AABBTriangle(AABB a, Triangle b, ref Contact contact)
        {
            // Minimum Translation Vector
            // ==========================
            float mtvDistance = float.MaxValue;             // Set current minimum distance (max float value so next value is always less)
            Vector3 mtvAxis = new Vector3();                // Axis along which to travel with the minimum distance

            Vector3 axis = new Vector3();
            float minA;
            float maxA;
            float minB;
            float maxB;

            // Axes of potential separation
            // ============================
            // - Each shape must be projected on these axes to test for intersection:
            //    
            // Tri.Normal                   N
            //
            // (1, 0, 0)                    A0
            // (0, 1, 0)                    A1
            // (0, 0, 1)                    A2
            //
            // (1, 0, 0) x Tri.Edges[0]     A0 x E0 = (0, -Tri.Edges[0].Z, Tri.Edges[0].Y)
            // (1, 0, 0) x Tri.Edges[1]     A0 x E1 = (0, -Tri.Edges[1].Z, Tri.Edges[1].Y)
            // (1, 0, 0) x Tri.Edges[2]     A0 x E2 = (0, -Tri.Edges[2].Z, Tri.Edges[2].Y)
            //
            // (0, 1, 0) x Tri.Edges[0]     A1 x E0 = (Tri.Edges[0].Z, 0, -Tri.Edges[0].X)
            // (0, 1, 0) x Tri.Edges[1]     A1 x E1 = (Tri.Edges[1].Z, 0, -Tri.Edges[1].X)
            // (0, 1, 0) x Tri.Edges[2]     A1 x E2 = (Tri.Edges[2].Z, 0, -Tri.Edges[2].X)
            //
            // (0, 0, 1) x Tri.Edges[0]     A2 x E0 = (-Tri.Edges[0].Y, Tri.Edges[0].X, 0)
            // (0, 0, 1) x Tri.Edges[1]     A2 x E1 = (-Tri.Edges[1].Y, Tri.Edges[1].X, 0)
            // (0, 0, 1) x Tri.Edges[2]     A2 x E2 = (-Tri.Edges[2].Y, Tri.Edges[2].X, 0)

            // N
            axis = b.Normal;

            minA = float.MaxValue;
            maxA = float.MinValue;
            minB = float.MaxValue;
            maxB = float.MinValue;

            a.CalculateSlab(axis, ref minA, ref maxA);
            b.CalculateSlab(axis, ref minB, ref maxB);

            if (!TestAxisStatic(axis, minA, maxA, minB, maxB, ref mtvAxis, ref mtvDistance))
                return false;

            // A0, A1, A2
            for (int i = 0; i < 3; ++i)
            {
                axis = a.Axes[i];

                minA = float.MaxValue;
                maxA = float.MinValue;
                minB = float.MaxValue;
                maxB = float.MinValue;

                a.CalculateSlab(axis, ref minA, ref maxA);
                b.CalculateSlab(axis, ref minB, ref maxB);

                if (!TestAxisStatic(axis, minA, maxA, minB, maxB, ref mtvAxis, ref mtvDistance))
                    return false;
            }

            // A0 x E0, A0 x E1, A0 x E2
            for (int i = 0; i < 3; ++i)
            {
                axis.X = 0;
                axis.Y = -b.Edges[i].Z;
                axis.Z = b.Edges[i].Y;

                minA = float.MaxValue;
                maxA = float.MinValue;
                minB = float.MaxValue;
                maxB = float.MinValue;

                a.CalculateSlab(axis, ref minA, ref maxA);
                b.CalculateSlab(axis, ref minB, ref maxB);

                if (!TestAxisStatic(axis, minA, maxA, minB, maxB, ref mtvAxis, ref mtvDistance))
                    return false;
            }

            // A1 x E0, A1 x E1, A1 x E2
            for (int i = 0; i < 3; ++i)
            {
                axis.X = b.Edges[i].Z;
                axis.Y = 0;
                axis.Z = -b.Edges[i].X;

                minA = float.MaxValue;
                maxA = float.MinValue;
                minB = float.MaxValue;
                maxB = float.MinValue;

                a.CalculateSlab(axis, ref minA, ref maxA);
                b.CalculateSlab(axis, ref minB, ref maxB);

                if (!TestAxisStatic(axis, minA, maxA, minB, maxB, ref mtvAxis, ref mtvDistance))
                    return false;
            }

            // A2 x E0, A2 x E1, A2 x E2
            for (int i = 0; i < 3; ++i)
            {
                axis.X = -b.Edges[i].Y;
                axis.Y = b.Edges[i].X;
                axis.Z = 0;

                minA = float.MaxValue;
                maxA = float.MinValue;
                minB = float.MaxValue;
                maxB = float.MinValue;

                a.CalculateSlab(axis, ref minA, ref maxA);
                b.CalculateSlab(axis, ref minB, ref maxB);

                if (!TestAxisStatic(axis, minA, maxA, minB, maxB, ref mtvAxis, ref mtvDistance))
                    return false;
            }

            contact.isIntersecting = true;

            // Calculate Minimum Translation Vector (MTV) [normal * penetration]
            contact.nEnter = Vector3.Normalize(mtvAxis);

            // Multiply the penetration depth by itself plus a small increment
            // When the penetration is resolved using the MTV, it will no longer intersect
            contact.penetration = (float)Math.Sqrt(mtvDistance) * 1.001f;

            return true;
        }

        public static bool OBBOBB(OBB a, OBB b, ref Contact contact)
        {
            // Minimum Translation Vector
            // ==========================
            float mtvDistance = float.MaxValue;             // Set current minimum distance (max float value so next value is always less)
            Vector3 mtvAxis = new Vector3();                // Axis along which to travel with the minimum distance

            Vector3 axis = new Vector3();
            float minA;
            float maxA;
            float minB;
            float maxB;

            // Axes of potential separation
            // ============================
            // - Each shape must be projected on these axes to test for intersection:
            //          
            // (1, 0, 0)                    A0
            // (0, 1, 0)                    A1
            // (0, 0, 1)                    A2
            // 
            // obb.axis(0)                  B0
            // obb.axis(1)                  B1
            // obb.axis(2)                  B2
            // 
            // (1, 0, 0) x obb.axis(0)      A0 x B0
            // (1, 0, 0) x obb.axis(1)      A0 x B1
            // (1, 0, 0) x obb.axis(2)      A0 x B2
            // 
            // (0, 1, 0) x obb.axis(0)      A1 x B0
            // (0, 1, 0) x obb.axis(1)      A1 x B1
            // (0, 1, 0) x obb.axis(2)      A1 x B2
            // 
            // (0, 0, 1) x obb.axis(0)      A2 x B0
            // (0, 0, 1) x obb.axis(1)      A2 x B1
            // (0, 0, 1) x obb.axis(2)      A2 x B2

            // A0, A1, A2
            for (int i = 0; i < 3; ++i)
            {
                axis = a.Axes[i];

                minA = float.MaxValue;
                maxA = float.MinValue;
                minB = float.MaxValue;
                maxB = float.MinValue;

                a.CalculateSlab(axis, ref minA, ref maxA);
                b.CalculateSlab(axis, ref minB, ref maxB);

                if (!TestAxisStatic(axis, minA, maxA, minB, maxB, ref mtvAxis, ref mtvDistance))
                    return false;
            }

            // B0, B1, B2
            for (int i = 0; i < 3; ++i)
            {
                axis = b.Axes[i];

                minA = float.MaxValue;
                maxA = float.MinValue;
                minB = float.MaxValue;
                maxB = float.MinValue;

                a.CalculateSlab(axis, ref minA, ref maxA);
                b.CalculateSlab(axis, ref minB, ref maxB);

                if (!TestAxisStatic(axis, minA, maxA, minB, maxB, ref mtvAxis, ref mtvDistance))
                    return false;
            }

            // Remaining cross product axes
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    axis = Vector3.Cross(a.Axes[i], b.Axes[j]);

                    minA = float.MaxValue;
                    maxA = float.MinValue;
                    minB = float.MaxValue;
                    maxB = float.MinValue;

                    a.CalculateSlab(axis, ref minA, ref maxA);
                    b.CalculateSlab(axis, ref minB, ref maxB);

                    if (!TestAxisStatic(axis, minA, maxA, minB, maxB, ref mtvAxis, ref mtvDistance))
                        return false;
                }
            }

            contact.isIntersecting = true;

            // Calculate Minimum Translation Vector (MTV) [normal * penetration]
            contact.nEnter = Vector3.Normalize(mtvAxis);

            // Multiply the penetration depth by itself plus a small increment
            // When the penetration is resolved using the MTV, it will no longer intersect
            contact.penetration = (float)Math.Sqrt(mtvDistance) * 1.001f;

            return true;
        }

        public static bool OBBPlane(OBB a, Plane b, ref Contact contact)
        {
            // Compute the projection interval radius of b onto L(t) = b.c + t * p.n
            float r =
                a.Extents.X * Math.Abs(Vector3.Dot(b.Normal, a.Axes[0])) +
                a.Extents.Y * Math.Abs(Vector3.Dot(b.Normal, a.Axes[1])) +
                a.Extents.Z * Math.Abs(Vector3.Dot(b.Normal, a.Axes[2]));

            // Compute the distance of the OBB centre from plane
            float distance = Vector3.Dot(b.Normal, a.Position) - b.D;

            // Intersection occurs when distance (s) falls with [-r, +r] interval
            if (Math.Abs(distance) > r)
                return false;

            contact.isIntersecting = true;
            contact.nEnter = b.Normal;
            contact.penetration = r - distance;

            return true;
        }

        public static bool OBBRay(OBB a, Ray b, ref Contact contact)
        {
            // Inverse of 3x3 OBB matrix is used to transform ray from world space to OBB space
            Matrix rotation_Inverse = Matrix.Transpose(a.Rotation);

            // Transform the ray from world space into OBB space 
            // Ray's Origin and OBB are translated to the origin
            Vector3 origin_Projected = Vector3.Transform(b.Origin - a.Position, rotation_Inverse);
            Vector3 direction_Projected = Vector3.Transform(b.Direction, rotation_Inverse);

            // Now the test is exactly the same as ray/AABB
            float tMin = 0.0f;

            // Maximum distance ray can travel (makes it a segment, otherwise it would be infinite)
            float tMax = float.MaxValue;

            float ood = 0.0f;
            float t1 = 0.0f;
            float t2 = 0.0f;

            // For all 3 slabs (x, y, z) [A slab is the space between a pair of parallel planes]
            for (int i = 0; i < 3; ++i)
            {
                if (Math.Abs(direction_Projected.Index(i)) < ToolBox.EPSILON)
                {
                    // Ray is parallel to slab.  No hit if origin (p in book) not within slab
                    if (origin_Projected.Index(i) < -a.Extents.Index(i) || origin_Projected.Index(i) > a.Extents.Index(i))
                        return false;
                }
                else
                {
                    // Compute intersection t value of ray with near and far plane of slab
                    ood = 1.0f / direction_Projected.Index(i);
                    t1 = (-a.Extents.Index(i) - origin_Projected.Index(i)) * ood;
                    t2 = (a.Extents.Index(i) - origin_Projected.Index(i)) * ood;

                    // Make (t1) be intersection with near plane, (t2) with far plane
                    if (t1 > t2)
                        ToolBox.Swap(ref t1, ref t2);

                    // Compute the intersection of slab intersection intervals
                    tMin = Math.Max(tMin, t1);
                    tMax = Math.Min(tMax, t2);

                    // Exit with no collision as soon as slab intersection becomes empty
                    if (tMin > tMax)
                        return false;
                }
            }

            // Ray intersects all 3 slabs
            contact.point = b.Origin + tMin * b.Direction;
            contact.nEnter = a.GetNormalFromPoint(contact.point);
            contact.tEnter = tMin;
            contact.tExit = tMax;

            return true;
        }

        public static bool OBBSphere(OBB a, Sphere b, ref Contact contact)
        {
            // Find point p on OBB closest to sphere centre
            Vector3 p = a.ClosestPtPointOBB(b.Position);

            // Find the directional vector between the sphere and this OBB
            Vector3 d = b.Position - p;

            // Find square distance between centre of sphere and centre of OBB
            float distance_Squared = d.LengthSquared();

            if (distance_Squared > b.Radius * b.Radius)
                return false;

            // Calculate the normal using the directional vector
            if (d != Vector3.Zero)
                d.Normalize();

            contact.nEnter = -d;
            contact.penetration = (b.Radius * b.Radius) - distance_Squared;

            return true;
        }

        public static bool OBBTriangle(OBB a, Triangle b, ref Contact contact)
        {
            // Minimum Translation Vector
            // ==========================
            float mtvDistance = float.MaxValue;             // Set current minimum distance (max float value so next value is always less)
            Vector3 mtvAxis = new Vector3();                // Axis along which to travel with the minimum distance

            Vector3 axis = new Vector3();
            float minA;
            float maxA;
            float minB;
            float maxB;

            // Axes of potential separation
            // ============================
            // - Each shape must be projected on these axes to test for intersection:
            //
            // Triangle.Normal              N
            //
            // (1, 0, 0)                    A0
            // (0, 1, 0)                    A1
            // (0, 0, 1)                    A2
            //
            // (1, 0, 0) x Triangle.E0      A0 x E0
            // (1, 0, 0) x Triangle.E1      A0 x E1
            // (1, 0, 0) x Triangle.E2      A0 x E2
            //
            // (0, 1, 0) x Triangle.E0      A1 x E0
            // (0, 1, 0) x Triangle.E1      A1 x E1
            // (0, 1, 0) x Triangle.E2      A1 x E2
            //
            // (0, 0, 1) x Triangle.E0      A2 x E0
            // (0, 0, 1) x Triangle.E1      A2 x E1
            // (0, 0, 1) x Triangle.E2      A2 x E2

            // N
            axis = b.Normal;

            minA = float.MaxValue;
            maxA = float.MinValue;
            minB = float.MaxValue;
            maxB = float.MinValue;

            a.CalculateSlab(axis, ref minA, ref maxA);
            b.CalculateSlab(axis, ref minB, ref maxB);

            if (!TestAxisStatic(axis, minA, maxA, minB, maxB, ref mtvAxis, ref mtvDistance))
                return false;

            // A0, A1, A2
            for (int i = 0; i < 3; ++i)
            {
                axis = a.Axes[i];

                minA = float.MaxValue;
                maxA = float.MinValue;
                minB = float.MaxValue;
                maxB = float.MinValue;

                a.CalculateSlab(axis, ref minA, ref maxA);
                b.CalculateSlab(axis, ref minB, ref maxB);

                if (!TestAxisStatic(axis, minA, maxA, minB, maxB, ref mtvAxis, ref mtvDistance))
                    return false;
            }

            // Remaining cross product axes
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    axis = Vector3.Cross(a.Axes[i], b.Edges[j]);

                    minA = float.MaxValue;
                    maxA = float.MinValue;
                    minB = float.MaxValue;
                    maxB = float.MinValue;

                    a.CalculateSlab(axis, ref minA, ref maxA);
                    b.CalculateSlab(axis, ref minB, ref maxB);

                    if (!TestAxisStatic(axis, minA, maxA, minB, maxB, ref mtvAxis, ref mtvDistance))
                        return false;
                }
            }

            contact.isIntersecting = true;

            // Calculate Minimum Translation Vector (MTV) [normal * penetration]
            contact.nEnter = Vector3.Normalize(mtvAxis);

            // Multiply the penetration depth by itself plus a small increment
            // When the penetration is resolved using the MTV, it will no longer intersect
            contact.penetration = (float)Math.Sqrt(mtvDistance) * 1.001f;

            return true;
        }

        public static bool PlaneRay(Plane a, Ray b, ref Contact contact)
        {
            float dot = Vector3.Dot(a.Normal, b.Origin);

            // Check to see if the ray origin is in the -ve half space of the plane
            // Remove this statement if I want to register intersections for both sides of Plane
            if (dot - a.D < 0)
            {
                return false;
            }

            // Compute the distance (t) for the ray intersecting the plane 
            float distance = (a.D - dot) / Vector3.Dot(a.Normal, b.Direction);

            // If distance (t) in [0, maximum ray length] compute and return intersection point (q)
            if (distance >= 0.0f && distance <= float.MaxValue)
            {
                contact.tEnter = distance;
                contact.point = b.Origin + distance * b.Direction;
                contact.nEnter = a.Normal;

                return true;
            }

            // Else t is +INF, -INF, NaN, or not in [0, maximum ray length], so no intersection
            return false;
        }

        public static bool PlaneSphere(Plane a, Sphere b, ref Contact contact)
        {
            // For a Normalized Plane (|p.n| = 1)
            // Evaluating the plane equation for a point gives the signed distance of the point to the plane
            float distance = Vector3.Dot(b.Position, a.Normal) - a.D;

            // Determine whether sphere intersects negative halfspace of plane (p)
            // -ve halfspace of plane is considered solid
            if (distance > b.Radius)
                return false;

            //contact.location = Centre - p.Normal * (distance + Radius);
            contact.nEnter = a.Normal;
            contact.penetration = b.Radius - distance;

            return true;
        }

        public static bool RaySphere(Ray a, Sphere s, ref Contact contact)
        {
            Vector3 m = a.Origin - s.Position;
            float b = Vector3.Dot(m, a.Direction);
            float c = Vector3.Dot(m, m) - s.Radius * s.Radius;

            // Early exit if:
            // - Ray origin outside Sphere (c > 0) AND 
            // - Ray pointing away from Sphere (b > 0)  
            if (c > 0.0f && b > 0.0f)
                return false;

            // Discriminant (Quadratic equation) = b [Squared] - c
            float discr = b * b - c;

            // A negative discriminant corresponds to Ray missing Sphere
            if (discr < 0.0f)
                return false;

            // Now Ray must hit Sphere

            // Compute smallest value of intersection (t)
            float sqrtDiscr = (float)Math.Sqrt(discr);

            float t0 = -b - sqrtDiscr;
            float t1 = -b + sqrtDiscr;

            // If (t) is negative, Ray started inside Sphere so clamp (t) to zero
            if (t0 < 0.0f)
                t0 = 0.0f;

            // Compute intersection point (q)
            contact.point = a.Origin + t0 * a.Direction;
            contact.nEnter = Vector3.Normalize(contact.point - s.Position);
            contact.tEnter = t0;
            contact.tExit = t1;

            return true;
        }

        public static bool RayTriangle(Ray a, Triangle b, ref Contact contact)
        {
            // First test to see if ray intersects the triangle plane 
            // All necessary contact data will be saved in the plane intersection test
            if (PlaneRay(b.Plane, a, ref contact))
            {
                // Test to see if the intersection point is inside the triangle
                if (b.PointInTriangle(contact.point))
                    return true;
            }

            return false;
        }

        public static bool SphereSphere(Sphere a, Sphere b, ref Contact contact)
        {
            // Calculate squared distance between centres
            Vector3 v = a.Position - b.Position;
            float distance_Squared = Vector3.Dot(v, v);

            float radius_Sum = a.Radius + b.Radius;
            float radius_Sum_Squared = radius_Sum * radius_Sum;

            // Spheres intersect if squared distance is less than squared sum of radii
            if (distance_Squared < radius_Sum_Squared)
            {
                if (v != Vector3.Zero)
                    v.Normalize();

                contact.nEnter = v;
                contact.penetration = radius_Sum - (float)Math.Sqrt(distance_Squared);
                contact.point = a.Position + v * 0.5f;

                return true;
            }

            return false;
        }

        public static bool SphereTriangle(Sphere a, Triangle b, ref Contact contact)
        {
            // Find point p on triangle ABC closest to sphere centre
            Vector3 p = b.ClosestPtPointTriangle(a.Position);

            // Sphere and triangle intersect if the (squared) distance from sphere centre
            // to point p is less than the (squared) sphere radius
            Vector3 v = p - a.Position;
            float distance_Squared = Vector3.Dot(v, v);

            if (distance_Squared <= a.Radius * a.Radius)
            {
                // Check for degenerate axis
                if (distance_Squared > 0.00001f)
                {
                    contact.nEnter = -v;
                    contact.penetration = a.Radius - (float)Math.Sqrt(distance_Squared);
                    return true;
                }
            }

            return false;
        }

        private static bool TestAxisStatic(Vector3 axis, float minA, float maxA, float minB, float maxB, ref Vector3 mtvAxis, ref float mtvDistance)
        {
            // Separating Axis Theorem
            // =======================
            // - Two convex shapes only overlap if they overlap on all axes of separation
            // - In order to create accurate responses we need to find the collision vector (Minimum Translation Vector)   
            // - The collision vector is made from a vector and a scalar, 
            //   - The vector value is the axis associated with the smallest penetration
            //   - The scalar value is the smallest penetration value
            // - Find if the two boxes intersect along a single axis
            // - Compute the intersection interval for that axis
            // - Keep the smallest intersection/penetration value
            float axisLengthSquared = Vector3.Dot(axis, axis);

            // If the axis is degenerate then ignore
            if (axisLengthSquared < 1.0e-8f)
                return true;

            // Calculate the two possible overlap ranges
            // Either we overlap on the left or the right sides
            float d0 = (maxB - minA);   // 'Left' side
            float d1 = (maxA - minB);   // 'Right' side

            // Intervals do not overlap, so no intersection
            if (d0 <= 0.0f || d1 <= 0.0f)
                return false;

            // Find out if we overlap on the 'right' or 'left' of the object.
            float overlap = (d0 < d1) ? d0 : -d1;

            // The mtd vector for that axis
            Vector3 sep = axis * (overlap / axisLengthSquared);

            // The mtd vector length squared
            float sepLengthSquared = Vector3.Dot(sep, sep);

            // If that vector is smaller than our computed Minimum Translation Distance use that vector as our current MTV distance
            if (sepLengthSquared < mtvDistance)
            {
                mtvDistance = sepLengthSquared;
                mtvAxis = sep;
            }

            return true;
        }
    }
}