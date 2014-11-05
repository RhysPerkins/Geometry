using Microsoft.Xna.Framework;
using System;

namespace Geometry.ThreeDimensional
{
    /// <summary>
    /// Contact information for two colliding shapes (Collision Pair)
    /// </summary>
    public class Contact
    {
        public bool isColliding;                // Are the two shapes colliding at speed
        public bool isIntersecting;             // Are the two shapes intersecting/overlapping?          
        public Vector3 nEnter;                  // Contact normal for time of entry (or overlap normal)
        public Vector3 nExit;                   // Contact normal for time of exit
        public object[] objects;                // The two objects involved in the collision pair  
        public float penetration;               // Penetration depth (for overlapping shapes)      
        public Vector3 point;                   // Contact point (world coordinates)    
        public float tEnter;                    // Time of first collision (point of entry)
        public float tExit;                     // Time of last collision (point of exit)

        public Contact()
        {
            isColliding = false;
            isIntersecting = false;
            nEnter = Vector3.Zero;
            nExit = Vector3.Zero;
            objects = new object[2];
            penetration = float.MinValue;
            point = Vector3.Zero;
            tEnter = float.MaxValue;
            tExit = 1.0f;
        }
    }
}