using Microsoft.Xna.Framework;
using System;

// See Fast Extraction of Viewing Frustum Planes from the World-View-Projection Matrix (Gribb/Hartmann)
namespace Geometry.ThreeDimensional
{
    // Bounding frustum for Right Handed coordinate system
    public class Frustum
    {
        Plane[] planes;
        Vector3[] corners;

        public Frustum()
            : this(Matrix.Identity) { }

        /// <summary>
        /// M can work for any arbitary view.  It is usually given in world space (View * Projection)
        /// </summary>
        /// <param name="M"></param>
        public Frustum(Matrix M)
        {
            planes = new Plane[6];
            corners = new Vector3[8];

            for (int i = 0; i < planes.Length; ++i)
                planes[i] = new Plane(Vector3.One, Vector3.Zero);

            SetMatrix(M);
        }

        /// <summary>
        /// Test to see if an AABB is inside/intersecting the frustum
        /// </summary>
        /// <param name="aabb">Axis Aligned Bounding Box</param>
        /// <returns></returns>
        public bool Contains(AABB aabb)
        {
            for (int i = 0; i < 6; ++i)
            {
                // Compute the projection interval radius of b onto L(t) = b.c + t * p.n (calc extents distance relative to plane normal)               
                float r =
                    Math.Abs(aabb.Extents.X * planes[i].Normal.X) +
                    Math.Abs(aabb.Extents.Y * planes[i].Normal.Y) +
                    Math.Abs(aabb.Extents.Z * planes[i].Normal.Z);

                // Compute distance of AABB center from plane 
                float distance = Vector3.Dot(planes[i].Normal, aabb.Position) - planes[i].D;

                if (distance > r)
                {
                    // AABB is outside this plane and therefore is not in the frustum
                    return false;
                }
            }

            // AABB is inside or intersecting the bounding frustum
            return true;
        }

        /// <summary>
        /// Test to see if an AABR is inside/intersecting the frustum
        /// </summary>
        /// <param name="aabr">Axis Aligned Bounding Rectangle</param>
        /// <returns></returns>
        //public bool Contains(Geometry.TwoDimensional.AABR aabr)
        //{
        //    Vector3 position = new Vector3();
        //    position.X = aabr.Position.X;
        //    position.Y = aabr.Position.Y;
        //    position.Z = 0f;

        //    for (int i = 0; i < 6; ++i)
        //    {
        //        // Compute the projection interval radius of b onto L(t) = b.c + t * p.n (calc extents distance relative to plane normal)               
        //        float r =
        //            Math.Abs(aabr.Extents.X * planes[i].Normal.X) +
        //            Math.Abs(aabr.Extents.Y * planes[i].Normal.Y);

        //        // Compute distance of node center from plane 
        //        float distance = Vector3.Dot(planes[i].Normal, position) - planes[i].D;

        //        if (distance > r)
        //        {
        //            // Node (as AABB) is outside this plane and therefore is not in the frustum
        //            return false;
        //        }
        //    }

        //    // Node (as AABB) is inside or intersecting the bounding frustum
        //    return true;
        //}

        /// <summary>
        /// Test to see if an OBB is inside/intersecting the frustum
        /// </summary>
        /// <param name="oBB">Oriented Bounding Box</param>
        /// <returns></returns>
        public bool Contains(OBB oBB)
        {
            for (int i = 0; i < 6; ++i)
            {
                // Compute the projection interval radius of b onto L(t) = b.c + t * p.n
                float r =
                    oBB.Extents.X * Math.Abs(Vector3.Dot(planes[i].Normal, oBB.Axes[0])) +
                    oBB.Extents.Y * Math.Abs(Vector3.Dot(planes[i].Normal, oBB.Axes[1])) +
                    oBB.Extents.Z * Math.Abs(Vector3.Dot(planes[i].Normal, oBB.Axes[2]));

                // Compute the distance of the OBB centre from plane
                float distance = Vector3.Dot(planes[i].Normal, oBB.Position) - planes[i].D;

                if (distance > r)
                {
                    // OBB is outside this plane and therefore is not in the frustum
                    return false;
                }
            }

            // OBB is inside or intersecting the bounding frustum
            return true;
        }

        /// <summary>
        /// Test to see if a bounding sphere is inside/intersecting the frustum
        /// </summary>
        /// <param name="sphere">Bounding Sphere</param>
        /// <returns></returns>
        public bool Contains(Sphere sphere)
        {
            // Stores the distance to each of the 6 planes
            float distance;

            for (int i = 0; i < 6; ++i)
            {
                distance = Vector3.Dot(planes[i].Normal, sphere.Position) - planes[i].D;

                if (distance > sphere.Radius)
                {
                    // Sphere is outside this plane and therefore is not in the frustum
                    return false;
                }
            }

            // Sphere is inside or intersecting the bounding frustum
            return true;
        }

        // If the matrix M is equal to the projection matrix P (i.e., M = P ), then the algorithm gives the clipping planes in view space (i.e., camera space)
        // If the matrix M is equal to the combined view and projection matrices, then the algorithm gives the clipping planes in world space (i.e., M = V·P, where V is the view matrix, and P is the projection matrix)
        // If the matrix M is equal to the combined world, view, and projection matrices, then the algorithm gives the clipping planes in object space (i.e., M = W·V·P, where W is the world matrix, V is the view matrix, and P is the projection matrix)
        // and so on...
        public void SetMatrix(Matrix M)
        {
            // Clipping planes (For Right-Handed Cartesian coordinate system)
            Vector3 v;

            // All points X on the plane then satisfy the equation Dot(N, X - P) = 0.
            // The dot product is a linear operator so the equation is equivalent to Dot(N, X) - Dot(N, P) = 0. 
            // We can decide to write this expression in one of two ways:
            // Dot(N, X) - A = 0, for A = Dot(N, P)
            // Dot(N, X) + A = 0, for A = -Dot(N, P)

            // Left
            v.X = M.M14 + M.M11;
            v.Y = M.M24 + M.M21;
            v.Z = M.M34 + M.M31;
            planes[2].Normal = -v;              // a, b, c
            planes[2].D = M.M44 + M.M41;        // d     

            // Right
            v.X = M.M14 - M.M11;
            v.Y = M.M24 - M.M21;
            v.Z = M.M34 - M.M31;
            planes[3].Normal = -v;
            planes[3].D = M.M44 - M.M41;

            // Bottom
            v.X = M.M14 + M.M12;
            v.Y = M.M24 + M.M22;
            v.Z = M.M34 + M.M32;
            planes[4].Normal = -v;
            planes[4].D = M.M44 + M.M42;

            // Top
            v.X = M.M14 - M.M12;
            v.Y = M.M24 - M.M22;
            v.Z = M.M34 - M.M32;
            planes[5].Normal = -v;
            planes[5].D = M.M44 - M.M42;

            // Near
            v.X = M.M13;
            v.Y = M.M23;
            v.Z = M.M33;
            planes[0].Normal = -v;
            planes[0].D = M.M43;

            // Far
            v.X = M.M14 - M.M13;
            v.Y = M.M24 - M.M23;
            v.Z = M.M34 - M.M33;
            planes[1].Normal = -v;
            planes[1].D = M.M44 - M.M43;

            // Normalise all plane equations

            // Get the magnitude/length of the plane normal vector
            // Magnitude = Math.Sqrt(
            //  Plane[i].Normal.X * Plane[i].Normal.X + 
            //  Plane[i].Normal.Y * Plane[i].Normal.Y + 
            //  Plane[i].Normal.Z * Plane[i].Normal.Z);
            float magnitude;

            magnitude = planes[0].Normal.Length();
            planes[0].Normal /= magnitude;
            planes[0].D /= magnitude;

            magnitude = planes[1].Normal.Length();
            planes[1].Normal /= magnitude;
            planes[1].D /= magnitude;

            magnitude = planes[2].Normal.Length();
            planes[2].Normal /= magnitude;
            planes[2].D /= magnitude;

            magnitude = planes[3].Normal.Length();
            planes[3].Normal /= magnitude;
            planes[3].D /= magnitude;

            magnitude = planes[4].Normal.Length();
            planes[4].Normal /= magnitude;
            planes[4].D /= magnitude;

            magnitude = planes[5].Normal.Length();
            planes[5].Normal /= magnitude;
            planes[5].D /= magnitude;

            SetCorners();
        }

        public Vector3[] GetCorners()
        {
            return corners;
        }

        private void SetCorners()
        {
            // Near Top Left (Intersection of 3 Planes: 0 = Near , 2 = Left , 5 = Top)
            corners[0] = Plane.IntersectionOf3Planes(planes[0], planes[2], planes[5]);

            // Near Top Right
            corners[1] = Plane.IntersectionOf3Planes(planes[0], planes[3], planes[5]);

            // Near Bottom Right
            corners[2] = Plane.IntersectionOf3Planes(planes[0], planes[3], planes[4]);

            // Near Bottom Left
            corners[3] = Plane.IntersectionOf3Planes(planes[0], planes[2], planes[4]);

            // Far Top Left
            corners[4] = Plane.IntersectionOf3Planes(planes[1], planes[2], planes[5]);

            // Far Top Right
            corners[5] = Plane.IntersectionOf3Planes(planes[1], planes[3], planes[5]);

            // Far Bottom Right
            corners[6] = Plane.IntersectionOf3Planes(planes[1], planes[3], planes[4]);

            // Far Bottom Left
            corners[7] = Plane.IntersectionOf3Planes(planes[1], planes[2], planes[4]);
        }

        public Plane Bottom
        {
            get { return planes[4]; }
        }

        public Plane Far
        {
            get { return planes[1]; }
        }

        public Plane Left
        {
            get { return planes[2]; }
        }

        public Plane Near
        {
            get { return planes[0]; }
        }

        public Plane Right
        {
            get { return planes[3]; }
        }

        public Plane Top
        {
            get { return planes[5]; }
        }
    }
}