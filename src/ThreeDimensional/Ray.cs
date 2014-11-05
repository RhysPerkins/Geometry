using Microsoft.Xna.Framework;
using System;

namespace Geometry.ThreeDimensional
{
    public class Ray
    {
        private Vector3 direction;
        private float length;
        private Vector3 origin;

        public Ray()
            : this(Vector3.Zero, Vector3.Zero, float.MaxValue) { }

        public Ray(Vector3 origin, Vector3 direction)
            : this(origin, direction, float.MaxValue) { }

        public Ray(Vector3 origin, Vector3 direction, float length)
        {
            this.direction = direction;
            this.length = length;
            this.origin = origin;
        }

        public Vector3 Direction
        {
            get { return direction; }
            set { direction = value; }
        }

        public float Length
        {
            get { return length; }
            set { length = value; }
        }

        public Vector3 Origin
        {
            get { return origin; }
            set { origin = value; }
        }
    }
}