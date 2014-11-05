using Microsoft.Xna.Framework;
using System;

namespace Geometry.ThreeDimensional
{
    /* 
     * There are six possible types of contact between two AABBs:
     * 
     * - Point-Face          
     * - Edge-Edge
     * 
     * - Point-Point   // No obvious way of calculating Normal [Can be ignored so that further interpenetration will take place later, negating this type of contact]              
     * - Point-Edge    // No obvious way of calculating Normal [Can be ignored so that further interpenetration will take place later, negating this type of contact]
     * 
     * - Edge-Face     // This contact is unstable and should be avoided [Can be replaced with 2 Point-Face OR 2 Edge-Edge contacts]          
     * - Face-Face     // This contact is unstable and should be avoided [Can be replaced with 4 Point-Face OR 4 Edge-Edge contacts]
     *
     * The seperate axis theorem says that two objects cannot possibly be in contact as long there is some axis on which the objects can
     * be projected where they are not in contact
     * 
     */

    public class AABB
    {
        Vector3[] axes;
        Vector3[] corners = new Vector3[8];
        Vector3 extents;                        // Positive halfwidth extents of AABB
        Vector3 position;

        public AABB()
            : this(Vector3.Zero, 1f, 1f, 1f) { }

        public AABB(Vector3 position, float width, float height, float depth)
        {
            extents = new Vector3();
            extents.X = width * 0.5f;
            extents.Y = height * 0.5f;
            extents.Z = depth * 0.5f;

            axes = new Vector3[3];
            axes[0] = Vector3.UnitX;
            axes[1] = Vector3.UnitY;
            axes[2] = Vector3.UnitZ;

            //radius = MathsTools.SQRT3 * MathsTools.Max3(width, height, depth) * 0.5f;
        }

        public void CalculateSlab(Vector3 axis, ref float min, ref float max)
        {
            // • A slab is the infinite region of space between two planes
            // • The min and max values are then the required scalar values defining the plane positions
            float pos = Vector3.Dot(position, axis);

            float ext =
                Math.Abs(axis.X) * extents.X +
                Math.Abs(axis.Y) * extents.Y +
                Math.Abs(axis.Z) * extents.Z;

            min = pos - ext;
            max = pos + ext;
        }

        /// <summary>
        /// Finds the closest point on or in the AABB that is closest to the Vector3 point input
        /// </summary>
        /// <param name="point">Relative Point to find closest point on AABB to</param>
        /// <returns>Closest point on AABB</returns>
        public Vector3 ClosestPtPointAABB(Vector3 point)    // P131
        {
            // For each coordinate axis, if the point coordinate value is outside box,
            // clamp it to the box, else keep it as is
            Vector3 min = position - extents;   // MinPoint
            Vector3 max = position + extents;   // MaxPoint

            Vector3 q = Vector3.Zero;
            float v = 0;

            v = point.X;
            v = Math.Max(v, min.X);
            v = Math.Min(v, max.X);
            q.X = v;

            v = point.Y;
            v = Math.Max(v, min.Y);
            v = Math.Min(v, max.Y);
            q.Y = v;

            v = point.Z;
            v = Math.Max(v, min.Z);
            v = Math.Min(v, max.Z);
            q.Z = v;

            return q;
        }

        public static AABB CreateFrom(Vector3[] points)
        {
            int numPoints = points.Length;

            if (numPoints == 0)
            {
                return null;
            }

            Vector3 min = new Vector3(float.MaxValue);
            Vector3 max = new Vector3(float.MinValue);

            // No need to project points onto an axis as the values are axis aligned
            for (int i = 0; i < numPoints; ++i)
            {
                Vector3 point = points[i];
                Vector3.Min(ref min, ref point, out min);
                Vector3.Max(ref max, ref point, out max);
            }

            Vector3 centre = (min + max) * 0.5f;
            Vector3 dim = max - min;

            return new AABB(centre, dim.X, dim.Y, dim.Z);
        }

        public static AABB CreateFrom(Microsoft.Xna.Framework.BoundingBox box)
        {
            Vector3 position = (box.Max + box.Min) * 0.5f;
            Vector3 dim = box.Max - box.Min;

            return new AABB(position, dim.X, dim.Y, dim.Z);
        }

        public static AABB CreateFrom(Vector3 min, Vector3 max)
        {
            Vector3 position = (max + min) * 0.5f;
            Vector3 dim = max - min;

            return new AABB(position, dim.X, dim.Y, dim.Z);
        }

        public static AABB CreateFrom(OBB obb)
        {
            AABB aabb_New = new AABB(Vector3.Zero, 0, 0, 0);

            aabb_New.position.X += Vector3.Dot(obb.Position, ToolBox.WORLD_X_AXIS);
            aabb_New.position.Y += Vector3.Dot(obb.Position, ToolBox.WORLD_Y_AXIS);
            aabb_New.position.Z += Vector3.Dot(obb.Position, ToolBox.WORLD_Z_AXIS);

            // Compute the projection interval radius of obb onto world x axis
            float r_X =
                obb.Extents.X * Math.Abs(Vector3.Dot(ToolBox.WORLD_X_AXIS, obb.Axes[0])) +
                obb.Extents.Y * Math.Abs(Vector3.Dot(ToolBox.WORLD_X_AXIS, obb.Axes[1])) +
                obb.Extents.Z * Math.Abs(Vector3.Dot(ToolBox.WORLD_X_AXIS, obb.Axes[2]));

            // Compute the projection interval radius of obb onto world y axis
            float r_Y =
                obb.Extents.X * Math.Abs(Vector3.Dot(ToolBox.WORLD_Y_AXIS, obb.Axes[0])) +
                obb.Extents.Y * Math.Abs(Vector3.Dot(ToolBox.WORLD_Y_AXIS, obb.Axes[1])) +
                obb.Extents.Z * Math.Abs(Vector3.Dot(ToolBox.WORLD_Y_AXIS, obb.Axes[2]));

            // Compute the projection interval radius of obb onto world z axis
            float r_Z =
                obb.Extents.X * Math.Abs(Vector3.Dot(ToolBox.WORLD_Z_AXIS, obb.Axes[0])) +
                obb.Extents.Y * Math.Abs(Vector3.Dot(ToolBox.WORLD_Z_AXIS, obb.Axes[1])) +
                obb.Extents.Z * Math.Abs(Vector3.Dot(ToolBox.WORLD_Z_AXIS, obb.Axes[2]));

            aabb_New.extents = new Vector3(r_X, r_Y, r_Z);

            return aabb_New;
        }

        public static AABB CreateFrom(Vector3 centre, float width, float height, float depth, Matrix rotation)
        {
            AABB aabb_New = new AABB(Vector3.Zero, 0, 0, 0);

            aabb_New.position.X += Vector3.Dot(centre, ToolBox.WORLD_X_AXIS);
            aabb_New.position.Y += Vector3.Dot(centre, ToolBox.WORLD_Y_AXIS);
            aabb_New.position.Z += Vector3.Dot(centre, ToolBox.WORLD_Z_AXIS);

            Vector3 x_Axis = new Vector3(rotation.M11, rotation.M12, rotation.M13);
            Vector3 y_Axis = new Vector3(rotation.M21, rotation.M22, rotation.M23);
            Vector3 z_Axis = new Vector3(rotation.M31, rotation.M32, rotation.M33);

            width *= 0.5f;
            height *= 0.5f;
            depth *= 0.5f;

            // Compute the projection interval radius of obb onto world x axis
            float r_X =
                width * Math.Abs(Vector3.Dot(ToolBox.WORLD_X_AXIS, x_Axis)) +
                height * Math.Abs(Vector3.Dot(ToolBox.WORLD_X_AXIS, y_Axis)) +
                depth * Math.Abs(Vector3.Dot(ToolBox.WORLD_X_AXIS, z_Axis));

            // Compute the projection interval radius of obb onto world y axis
            float r_Y =
                width * Math.Abs(Vector3.Dot(ToolBox.WORLD_Y_AXIS, x_Axis)) +
                height * Math.Abs(Vector3.Dot(ToolBox.WORLD_Y_AXIS, y_Axis)) +
                depth * Math.Abs(Vector3.Dot(ToolBox.WORLD_Y_AXIS, z_Axis));

            // Compute the projection interval radius of obb onto world z axis
            float r_Z =
                width * Math.Abs(Vector3.Dot(ToolBox.WORLD_Z_AXIS, x_Axis)) +
                height * Math.Abs(Vector3.Dot(ToolBox.WORLD_Z_AXIS, y_Axis)) +
                depth * Math.Abs(Vector3.Dot(ToolBox.WORLD_Z_AXIS, z_Axis));

            aabb_New.extents = new Vector3(r_X, r_Y, r_Z);

            return aabb_New;
        }

        /// <summary>
        /// Determines whether the AABB fully contains the input AABB
        /// </summary>
        /// <param name="aABB"></param>
        /// <returns></returns>
        public bool Contains(AABB aABB)
        {
            return
                MinPoint.X <= aABB.MinPoint.X &&
                aABB.MaxPoint.X <= MaxPoint.X &&
                MinPoint.Y <= aABB.MinPoint.Y &&
                aABB.MaxPoint.Y <= MaxPoint.Y &&
                MinPoint.Z <= aABB.MinPoint.Z &&
                aABB.MaxPoint.Z <= MaxPoint.Z;
        }

        /// <summary>
        /// Determines whether the AABB fully contains the point
        /// </summary>
        /// <param name="point"></param>
        /// <returns></returns>
        public bool Contains(Vector3 point)
        {
            return
                MinPoint.X <= point.X &&
                point.X <= MaxPoint.X &&
                MinPoint.Y <= point.Y &&
                point.Y <= MaxPoint.Y &&
                MinPoint.Z <= point.Z &&
                point.Z <= MaxPoint.Z;
        }

        /// <summary>
        /// Computes the distance between a point (point) and the current AABB
        /// </summary>
        /// <param name="point"></param>
        /// <returns></returns>
        public float DistPointAABB(Vector3 point)    // P131
        {
            float distance = 0.0f;
            float v = 0.0f;

            Vector3 min = MinPoint;
            Vector3 max = MaxPoint;

            // For each axis count any excess distance outside box extents
            v = point.X;

            if (v < min.X)
                distance += min.X - v;

            if (v > max.X)
                distance += v - max.X;

            v = point.Y;

            if (v < min.Y)
                distance += min.Y - v;

            if (v > max.Y)
                distance += v - max.Y;

            v = point.Z;

            if (v < min.Z)
                distance += min.Z - v;

            if (v > max.Z)
                distance += v - max.Z;

            return distance;
        }

        public Vector3[] GetCorners()
        {
            Vector3 min = MinPoint;
            Vector3 max = MaxPoint;

            // [Corner Index]
            //            z
            //           /
            //          /
            //       [4]_______[5]
            //       / |       /|
            // y    /  |      / |
            // |   /   |     /  |
            // | [0]__[7]__[1]_[6]                
            //    |   /     |   /
            //    |  /      |  /
            //    | /       | /
            //   [3]_______[2]   --- x 

            corners[0].X = min.X;
            corners[0].Y = max.Y;
            corners[0].Z = max.Z;

            corners[1].X = max.X;
            corners[1].Y = max.Y;
            corners[1].Z = max.Z;

            corners[2].X = max.X;
            corners[2].Y = min.Y;
            corners[2].Z = max.Z;

            corners[3].X = min.X;
            corners[3].Y = min.Y;
            corners[3].Z = max.Z;

            corners[4].X = min.X;
            corners[4].Y = max.Y;
            corners[4].Z = min.Z;

            corners[5].X = max.X;
            corners[5].Y = max.Y;
            corners[5].Z = min.Z;

            corners[6].X = max.X;
            corners[6].Y = min.Y;
            corners[6].Z = min.Z;

            corners[7].X = min.X;
            corners[7].Y = min.Y;
            corners[7].Z = min.Z;

            return corners;
        }

        /// <summary>
        /// Return the surface normal for any given point on the AABB
        /// </summary>
        /// <param name="point"></param>
        /// <returns></returns>
        public Vector3 GetNormalFromPoint(Vector3 point)
        {
            Vector3 normal = new Vector3();
            float min = float.MaxValue;
            float distance;

            point -= position;

            distance = Math.Abs(extents.X - Math.Abs(point.X));

            if (distance < min)
            {
                min = distance;
                normal = Math.Sign(point.X) * Vector3.UnitX;    // Cardinal axis for X
            }

            distance = Math.Abs(extents.Y - Math.Abs(point.Y));

            if (distance < min)
            {
                min = distance;
                normal = Math.Sign(point.Y) * Vector3.UnitY;    // Cardinal axis for Y
            }

            distance = Math.Abs(extents.Z - Math.Abs(point.Z));

            if (distance < min)
            {
                min = distance;
                normal = Math.Sign(point.Z) * Vector3.UnitZ;    // Cardinal axis for Z
            }

            return normal;
        }

        public void KeepSmallerAABBInside(AABB b)
        {
            Vector3 minA = MinPoint;
            Vector3 maxA = MaxPoint;
            Vector3 minB = b.MinPoint;
            Vector3 maxB = b.MaxPoint;
            Vector3 penetration = new Vector3();

            if (minB.X < minA.X)
            {
                penetration.X = minA.X - minB.X;
            }
            else if (maxB.X > maxA.X)
            {
                penetration.X = maxA.X - maxB.X;
            }

            if (minB.Y < minA.Y)
            {
                penetration.Y = minA.Y - minB.Y;
            }
            else if (maxB.Y > maxA.Y)
            {
                penetration.Y = maxA.Y - maxB.Y;
            }

            if (minB.Z < minA.Z)
            {
                penetration.Z = minA.Z - minB.Z;
            }
            else if (maxB.Z > maxA.Z)
            {
                penetration.Z = maxA.Z - maxB.Z;
            }

            b.Position += penetration;
        }

        /// <summary>
        /// Computes the distance squared between a point (point) and the current AABB
        /// </summary>
        /// <param name="point"></param>
        /// <returns></returns>
        public float SqDistPointAABB(Vector3 point)
        {
            float distance_Squared = 0.0f;
            float v = 0.0f;

            Vector3 min = MinPoint;
            Vector3 max = MaxPoint;

            // For each axis count any excess distance outside box extents
            v = point.X;

            if (v < min.X)
                distance_Squared += (min.X - v) * (min.X - v);

            if (v > max.X)
                distance_Squared += (v - max.X) * (v - max.X);

            v = point.Y;

            if (v < min.Y)
                distance_Squared += (min.Y - v) * (min.Y - v);

            if (v > max.Y)
                distance_Squared += (v - max.Y) * (v - max.Y);

            v = point.Z;

            if (v < min.Z)
                distance_Squared += (min.Z - v) * (min.Z - v);

            if (v > max.Z)
                distance_Squared += (v - max.Z) * (v - max.Z);

            return distance_Squared;
        }

        // Local X, Y and Z axes
        public Vector3[] Axes
        {
            get { return axes; }
        }

        public Vector3 Extents
        {
            get { return extents; }
            set { extents = value; }
        }

        /// <summary>
        /// AABB max point (contained within AABB extents)
        /// </summary>
        public Vector3 MaxPoint
        {
            get { return position + extents; }
        }

        /// <summary>
        /// AABB min point (contained within AABB extents)
        /// </summary>
        public Vector3 MinPoint
        {
            get { return position - extents; }
        }

        public Vector3 Position
        {
            get { return position; }
            set { position = value; }
        }
    }
}