using System;
using System.Collections.Generic;
using Microsoft.Xna.Framework;

namespace Geometry.ThreeDimensional
{
    public class Sphere
    {
        float radius;
        Vector3 position;

        private Sphere()
        {
            radius = 0.0f;
        }

        private Sphere(Vector3 p0)
        {
            position = p0;
            radius = 0.0f;
        }

        private Sphere(Vector3 p0, Vector3 p1)
        {
            Vector3 d = p1 - p0;
            position = (p0 + p1) * 0.5f;

            // Calculate squared radius
            radius = 0.25f * Vector3.Dot(d, d);
        }

        private Sphere(Vector3 p0, Vector3 p1, Vector3 p2)
        {
            // Calculate plane on which all 3 points lie
            Plane plane0 = new Plane(p0, p1, p2);

            // Calculate two planes from the 3 points
            Plane plane1 = new Plane(p0 - p1, (p0 + p1) * 0.5f);
            Plane plane2 = new Plane(p1 - p2, (p1 + p2) * 0.5f);

            // Sphere centre is the point of intersection for all 3 planes
            position = Plane.IntersectionOf3Planes(plane0, plane1, plane2);

            // Calculate squared radius
            Vector3 d = position - p0;
            radius = Vector3.Dot(d, d);
        }

        private Sphere(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3)
        {
            // Compute the sphere containing p0, p1, p2, and p3.
            // The Center in barycentric coordinates is K = u0*p0+u1*p1+u2*p2+u3*p3 where u0+u1+u2+u3=1.
            // The Center is equidistant from the three points, so
            // |K-p0| = |K-p1| = |K-p2| = |K-p3| = R where R is the radius of the sphere.
            // 
            // From these conditions:
            //   K-p0 = u0*A + u1*B + u2*C - A
            //   K-p1 = u0*A + u1*B + u2*C - B
            //   K-p2 = u0*A + u1*B + u2*C - C
            //   K-p3 = u0*A + u1*B + u2*C
            //
            // Where A = p0-p3, B = p1-p3, and C = p2-p3 which leads to:
            //   r^2 = |u0*A+u1*B+u2*C|^2 - 2*Dot(A,u0*A+u1*B+u2*C) + |A|^2
            //   r^2 = |u0*A+u1*B+u2*C|^2 - 2*Dot(B,u0*A+u1*B+u2*C) + |B|^2
            //   r^2 = |u0*A+u1*B+u2*C|^2 - 2*Dot(C,u0*A+u1*B+u2*C) + |C|^2
            //   r^2 = |u0*A+u1*B+u2*C|^2
            //
            // Subtracting the last equation from the first three and writing
            // the equations as a linear system:
            //
            // +-                          -++   -+       +-        -+
            // | Dot(A,A) Dot(A,B) Dot(A,C) || u0 | = 0.5 | Dot(A,A) |
            // | Dot(B,A) Dot(B,B) Dot(B,C) || u1 |       | Dot(B,B) |
            // | Dot(C,A) Dot(C,B) Dot(C,C) || u2 |       | Dot(C,C) |
            // +-                          -++   -+       +-        -+
            //
            // The following code solves this system for u0, u1, and u2, then
            // evaluates the fourth equation in r^2 to obtain r.
            Vector3 kE10 = p0 - p3;
            Vector3 kE20 = p1 - p3;
            Vector3 kE30 = p2 - p3;

            Matrix aafA;
            aafA.M11 = Vector3.Dot(kE10, kE10);     // AA
            aafA.M12 = Vector3.Dot(kE10, kE20);     // AB
            aafA.M13 = Vector3.Dot(kE10, kE30);     // AC
            aafA.M21 = aafA.M12;
            aafA.M22 = Vector3.Dot(kE20, kE20);
            aafA.M23 = Vector3.Dot(kE20, kE30);
            aafA.M31 = aafA.M13;
            aafA.M32 = aafA.M23;
            aafA.M33 = Vector3.Dot(kE30, kE30);

            Vector3 afB;
            afB.X = 0.5f * aafA.M11;
            afB.Y = 0.5f * aafA.M22;
            afB.Z = 0.5f * aafA.M33;

            Matrix aafAInv;
            aafAInv.M11 = aafA.M22 * aafA.M33 - aafA.M23 * aafA.M32;
            aafAInv.M12 = aafA.M13 * aafA.M32 - aafA.M12 * aafA.M33;
            aafAInv.M13 = aafA.M12 * aafA.M23 - aafA.M13 * aafA.M22;
            aafAInv.M21 = aafA.M23 * aafA.M31 - aafA.M21 * aafA.M33;
            aafAInv.M22 = aafA.M11 * aafA.M33 - aafA.M13 * aafA.M31;
            aafAInv.M23 = aafA.M13 * aafA.M21 - aafA.M11 * aafA.M23;
            aafAInv.M31 = aafA.M21 * aafA.M32 - aafA.M22 * aafA.M31;
            aafAInv.M32 = aafA.M12 * aafA.M31 - aafA.M11 * aafA.M32;
            aafAInv.M33 = aafA.M11 * aafA.M22 - aafA.M12 * aafA.M21;
            float fDet = aafA.M11 * aafAInv.M11 + aafA.M12 * aafAInv.M21 + aafA.M13 * aafAInv.M31;

            if (Math.Abs(fDet) > float.Epsilon)
            {
                float fInvDet = 1.0f / fDet;

                aafAInv.M11 *= fInvDet;
                aafAInv.M12 *= fInvDet;
                aafAInv.M13 *= fInvDet;
                aafAInv.M21 *= fInvDet;
                aafAInv.M22 *= fInvDet;
                aafAInv.M23 *= fInvDet;
                aafAInv.M31 *= fInvDet;
                aafAInv.M32 *= fInvDet;
                aafAInv.M33 *= fInvDet;

                Vector4 afU = new Vector4();
                afU.X += aafAInv.M11 * afB.X;
                afU.X += aafAInv.M12 * afB.Y;
                afU.X += aafAInv.M13 * afB.Z;
                afU.Y += aafAInv.M21 * afB.X;
                afU.Y += aafAInv.M22 * afB.Y;
                afU.Y += aafAInv.M23 * afB.Z;
                afU.Z += aafAInv.M31 * afB.X;
                afU.Z += aafAInv.M32 * afB.Y;
                afU.Z += aafAInv.M33 * afB.Z;
                afU.W = 1.0f - afU.X - afU.Y - afU.Z;

                position = afU.X * p0 + afU.Y * p1 + afU.Z * p2 + afU.W * p3;

                Vector3 kTmp = afU.X * kE10 + afU.Y * kE20 + afU.Z * kE30;

                // Squared radius
                radius = Vector3.Dot(kTmp, kTmp);
            }
            else
            {
                position = Vector3.Zero;
                radius = float.MaxValue;
            }
        }

        public Sphere(Vector3 position, float radius)
        {
            this.position = position;
            this.radius = radius;
        }

        public bool TestPointInsideSphere(Vector3 point, float radius, ref Contact contact)
        {
            Vector3 pc = position - point;
            float squaredDistance = Vector3.Dot(pc, pc);

            // Reduce the radius by the bounding sphere radius to keep entire sphere inside sphere radius
            radius -= Radius;

            if (squaredDistance > (radius * radius))
            {
                Vector3 normal = -pc;
                normal.Normalize();
                contact.nEnter = normal;
                contact.penetration = ((float)Math.Sqrt(squaredDistance) - radius) * 1.001f;

                return true;
            }

            return false;
        }

        public static Sphere WelzlSphere(List<Triangle> modelTriangles, Vector3 worldScale)
        {
            // Maximum of 4 support points
            Vector3[] supportPoints = new Vector3[4];

            // Number of triangles
            int numTriangles = modelTriangles.Count;

            Vector3[] points = new Vector3[numTriangles * 3];

            int j = 0;

            for (int i = 0; i < numTriangles; ++i)
            {
                points[j++] = modelTriangles[i].A;
                points[j++] = modelTriangles[i].B;
                points[j++] = modelTriangles[i].C;
            }

            // Number of support points is intialised at 0
            Sphere s = WelzlSphere(points, points.Length, supportPoints, 0);

            // Now that we have a sphere, calculate the real radius of the sphere using an expensive square root
            s.Radius = (float)Math.Sqrt(s.Radius);

            // Scale sphere radius by maximum extent of world scale (used in world transform to scale model) 
            s.Radius *= ToolBox.Max3(worldScale.X, worldScale.Y, worldScale.Z);

            return s;
        }

        public static Sphere WelzlSphere(Vector3[] points)
        {
            // Maximum of 4 support points
            Vector3[] supportPoints = new Vector3[4];

            // Number of support points is intialised at 0
            Sphere s = WelzlSphere(points, points.Length, supportPoints, 0);

            // Now that we have a sphere, calculate the real radius of the sphere using an expensive square root
            s.Radius = (float)Math.Sqrt(s.Radius);

            return s;
        }

        private static Sphere WelzlSphere(Vector3[] points, int numPts, Vector3[] sos, int numSos)
        {
            // If no input points, the recursion has bottomed out. 
            // Now compute an exact sphere based on points in Set Of Support (zero through four points)
            if (numPts == 0)
            {
                switch (numSos)
                {
                    case 0: return new Sphere();                                 //              Radius = 0.0f
                    case 1: return new Sphere(sos[0]);                           // Centre = p   Radius = 0.0f
                    case 2: return new Sphere(sos[0], sos[1]);
                    case 3: return new Sphere(sos[0], sos[1], sos[2]);
                    case 4: return new Sphere(sos[0], sos[1], sos[2], sos[3]);
                }
            }

            // Pick a point at "random" (here just the last point of the input set)
            int index = numPts - 1;

            // Recursively compute the smallest bounding sphere of the remaining points
            Sphere smallestSphere = WelzlSphere(points, numPts - 1, sos, numSos);

            // If the selected point lies inside this sphere, it is indeed the smallest
            // Sphere radius is already squared so no need to square it here, just square the distance
            if (Vector3.DistanceSquared(points[index], smallestSphere.Position) < smallestSphere.Radius)
            {
                return smallestSphere;
            }

            // Otherwise, update set of support to additionally contain the new point
            sos[numSos] = points[index];

            // Recursively compute the smallest sphere of remaining points with new s.o.s.
            return WelzlSphere(points, numPts - 1, sos, numSos + 1);
        }

        public Vector3 Position
        {
            get { return position; }
            set { position = value; }
        }

        public float Radius
        {
            get { return radius; }
            set { radius = value; }
        }
    }
}