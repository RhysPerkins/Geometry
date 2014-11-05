using Microsoft.Xna.Framework;
using System;

namespace Geometry.ThreeDimensional
{
    /// <summary>
    /// Oriented Bounding Box
    /// </summary>
    public class OBB
    {
        Vector3[] axes = new Vector3[3];        // Local x, y and z axes
        Vector3 extents;                        // Positive halfwidth extents of the OBB along each axis
        Quaternion orientation;                 // Orientation of OBB (Rotation matrix is derived from this)
        Matrix rotation;
        Vector3 position;

        // To save memory you can store two Vector3 of the rotation axes
        // Compute the 3rd using Vector3.Cross at collision test time (A 3x3 matrix can then be created from these 3 vectors)    
        public OBB(Vector3 position, float width, float height, float depth, float yaw, float pitch, float roll)
            : this(position, width, height, depth, Quaternion.CreateFromYawPitchRoll(yaw, pitch, roll)) { }

        public OBB(Vector3 position, float width, float height, float depth, Matrix rotation)
            : this(position, width, height, depth, Quaternion.CreateFromRotationMatrix(rotation)) { }

        public OBB(Vector3 position, float width, float height, float depth, Quaternion orientation)
        {
            extents = new Vector3();
            extents.X = width * 0.5f;
            extents.Y = height * 0.5f;
            extents.Z = depth * 0.5f;

            this.position = position;
            //radius = ToolBox.SQRT3 * ToolBox.Max3(width, height, depth) * 0.5f;

            UpdateRotation(orientation);
        }

        public void CalculateSlab(Vector3 axis, ref float min, ref float max)
        {
            // • A slab is the infinite region of space between two planes
            // • The min and max values are then the required scalar values defining the plane positions
            float pos = Vector3.Dot(axis, position);

            float ext =
                Math.Abs(Vector3.Dot(axes[0], axis)) * extents.X +
                Math.Abs(Vector3.Dot(axes[1], axis)) * extents.Y +
                Math.Abs(Vector3.Dot(axes[2], axis)) * extents.Z;

            min = pos - ext;
            max = pos + ext;
        }

        /// <summary>
        /// Given point p, return point q on (or in) OBB, closest to p
        /// </summary>
        /// <param name="p">Point</param>
        public Vector3 ClosestPtPointOBB(Vector3 p)
        {
            float distance;
            Vector3 q = Vector3.Zero;
            Vector3 d = p - position;

            // Start result at centre of OBB; make steps from there
            q = position;

            // For each OBB axis
            for (int i = 0; i < 3; ++i)
            {
                // Project d onto that axis to get the distance along the axis of d from the box centre
                distance = Vector3.Dot(d, axes[i]);

                // If the distance is farther than the OBB extents, clamp to the box
                if (distance > extents.Index(i))
                    distance = extents.Index(i);

                if (distance < -extents.Index(i))
                    distance = -extents.Index(i);

                // Step that distance along the axis to get the world coordinate
                q += distance * axes[i];
            }

            return q;
        }

        public Vector3[] GetCorners()
        {
            Vector3[] corners = new Vector3[8];

            Vector3 hX = rotation.Left * extents.X;
            Vector3 hY = rotation.Up * extents.Y;
            Vector3 hZ = rotation.Forward * extents.Z;

            corners[0] = position - hX + hY + hZ;
            corners[1] = position + hX + hY + hZ;
            corners[2] = position + hX - hY + hZ;
            corners[3] = position - hX - hY + hZ;
            corners[4] = position - hX + hY - hZ;
            corners[5] = position + hX + hY - hZ;
            corners[6] = position + hX - hY - hZ;
            corners[7] = position - hX - hY - hZ;

            return corners;
        }

        /// <summary>
        /// Return the surface normal for any given point on the OBB
        /// </summary>
        /// <param name="point"></param>
        /// <returns></returns>
        public Vector3 GetNormalFromPoint(Vector3 point)
        {
            Vector3 normal = Vector3.Zero;
            float min = float.MaxValue;
            float distance;

            // Compute translation vector t
            point -= position;

            Vector3 t;

            // Bring translation into OBB's coordinate frame
            t.X = Vector3.Dot(point, axes[0]);
            t.Y = Vector3.Dot(point, axes[1]);
            t.Z = Vector3.Dot(point, axes[2]);

            distance = Math.Abs(extents.X - Math.Abs(t.X));
            
            if (distance < min)
            {
                min = distance;
                normal = Math.Sign(t.X) * axes[0];      // Rotation.M11, Rotation.M12, Rotation.M13
            }
            
            distance = Math.Abs(extents.Y - Math.Abs(t.Y));
            
            if (distance < min)
            {
                min = distance;
                normal = Math.Sign(t.Y) * axes[1];      // Rotation.M21, Rotation.M22, Rotation.M23
            }
            
            distance = Math.Abs(extents.Z - Math.Abs(t.Z));
            
            if (distance < min)
            {
                min = distance;
                normal = Math.Sign(t.Z) * axes[2];      // Rotation.M31, Rotation.M32, Rotation.M33
            }

            return normal;
        }

        public float SqDistancePoint(Vector3 p)
        {
            Vector3 closest = ClosestPtPointOBB(p);
            Vector3 d0 = closest - p;

            return Vector3.Dot(d0, d0);
        }

        public void UpdateRotation(float yaw, float pitch, float roll)
        {
            Quaternion rotation_Additional = Quaternion.CreateFromYawPitchRoll(yaw, pitch, roll);
            orientation *= rotation_Additional;         
            Quaternion.Normalize(ref orientation, out orientation);
            Matrix.CreateFromQuaternion(ref orientation, out rotation);

            // Local X, Y and Z axes
            axes[0] = rotation.Right;
            axes[1] = rotation.Up;
            axes[2] = rotation.Forward;
        }

        public void UpdateRotation(Matrix r)
        {
            Quaternion.CreateFromRotationMatrix(ref r, out orientation);
            Quaternion.Normalize(ref orientation, out orientation);
            Matrix.CreateFromQuaternion(ref orientation, out rotation);

            // Local X, Y and Z axes
            axes[0] = rotation.Right;
            axes[1] = rotation.Up;
            axes[2] = rotation.Forward;
        }

        public void UpdateRotation(Quaternion q)
        {
            Quaternion.Normalize(ref q, out orientation);
            Matrix.CreateFromQuaternion(ref orientation, out rotation);

            // Local X, Y and Z axes
            axes[0] = rotation.Right;
            axes[1] = rotation.Up;
            axes[2] = rotation.Forward;
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
        /// OBB max point contained within OBB [World Space] 
        /// </summary>
        public Vector3 MaxPoint
        {
            get { return Vector3.Transform(position + extents, rotation); }
        }

        /// <summary>
        /// OBB min point contained within OBB [World Space]
        /// </summary>
        public Vector3 MinPoint
        {
            get { return Vector3.Transform(position - extents, rotation); }
        }

        public Vector3 Position
        {
            get { return position; }
            set { position = value; }
        }

        public Matrix Rotation
        {
            get { return rotation; }
        }
    }
}