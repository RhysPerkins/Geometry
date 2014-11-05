using Microsoft.Xna.Framework;
using System;

namespace Geometry
{
    static class ToolBox
    {
        public const float EPSILON = 1e-5f;                 // 1e-5 is a favorite because it's about the smallest quantum for a 32-bit float at magnitude 1 (0.00001f)
        public const float PI = 3.141592f;
        public const float PIOVER2 = 1.570796f;
        public const float PIOVER4 = 0.785398f;
        public const float PIOVER8 = 0.392699f;
        public const float PIOVER180 = 0.01745329f;
        public const float SQREPSILON = EPSILON * EPSILON;
        public const float SQRT2 = 1.414213f;
        public const float SQRT3 = 1.732050f;
        public const float TWOPI = 6.28318548f;

        public static readonly Vector3 WORLD_X_AXIS = Vector3.UnitX;
        public static readonly Vector3 WORLD_Y_AXIS = Vector3.UnitY;
        public static readonly Vector3 WORLD_Z_AXIS = -Vector3.UnitZ;

        public static readonly Random Random = new Random();

        public static float Abs(float value)
        {
            return value < 0 ? -value : value;
        }

        public static float Cross(Vector2 a, Vector2 b)
        {
            return a.X * b.Y - a.Y * b.X;
        }

        public static Vector2 DegreesToVector(float degrees)
        {
            Vector2 vector = new Vector2();
            vector.X = (float)Math.Sin(degrees * PIOVER180);
            vector.Y = (float)Math.Cos(degrees * PIOVER180);

            return vector;
        }

        /// <summary>Retrieves an element of the matrix by its column and row index</summary>
        /// <param name="matrix">Matrix of which to retrieve an element</param>
        /// <param name="row">Index of the row from which to retrieve the element</param>
        /// <param name="col">Index of the column to retrieve</param>
        /// <returns>The element at the given row and column</returns>
        public static float Index(this Matrix matrix, int row, int col)
        {
            // The ~ (tilde) operator performs a bitwise complement on its single integer operand
            // Complementing a number means to change all the 0 bits to 1 and all the 1s to 0s
            if (((row | col) & ~3) != 0)
            {
                // This is unsafe code BUT it is useful for debugging
                throw new ArgumentOutOfRangeException(((row & ~3) != 0) ? "row" : "col");
            }

            // << = bitwise shift left 
            // << 4 in BINARY shifts integer n from [n * 2^0] to [n * 2^4] which is [n * 16] 
            switch ((row << 2) + col)
            {
                case 0x00: { return matrix.M11; }   // Hexadecimal values must start with "0x"
                case 0x01: { return matrix.M12; }
                case 0x02: { return matrix.M13; }
                case 0x03: { return matrix.M14; }

                case 0x04: { return matrix.M21; }
                case 0x05: { return matrix.M22; }
                case 0x06: { return matrix.M23; }
                case 0x07: { return matrix.M24; }

                case 0x08: { return matrix.M31; }
                case 0x09: { return matrix.M32; }
                case 0x0A: { return matrix.M33; }
                case 0x0B: { return matrix.M34; }

                case 0x0C: { return matrix.M41; }
                case 0x0D: { return matrix.M42; }
                case 0x0E: { return matrix.M43; }
                case 0x0F: { return matrix.M44; }

                default: throw new ArgumentOutOfRangeException("Index out of range");
            }
        }

        public static float Index(this Vector2 v, int index)
        {
            switch (index)
            {
                case 0: return v.X;
                case 1: return v.Y;

                default: throw new ArgumentOutOfRangeException("Index out of range");
            }
        }

        public static float Index(this Vector3 v, int index)
        {
            switch (index)
            {
                case 0: return v.X;
                case 1: return v.Y;
                case 2: return v.Z;

                default: throw new ArgumentOutOfRangeException("Index out of range");
            }
        }

        public static bool IsCoplanar(Vector3 a, Vector3 b, Vector3 c, Vector3 d)
        {
            Vector3 ab = b - a;
            Vector3 ac = c - a;
            Vector3 ad = d - a;

            // If the triple scalar product (volume) is 0 then the points are coplanar
            return Vector3.Dot(ab, Vector3.Cross(ac, ad)) == 0 ? true : false;
        }

        public static float Max3(float val1, float val2, float val3)
        {
            float max = Math.Max(val1, val2);
            return Math.Max(max, val3);
        }

        public static float Min3(float val1, float val2, float val3)
        {
            float min = Math.Min(val1, val2);
            return Math.Min(min, val3);
        }

        public static float NextFloat(this Random random)
        {
            return (float)random.NextDouble();
        }

        public static float NextFloat(this Random random, float min, float max)
        {
            if (max < min)
                throw new ArgumentException("Max cannot be less than min");

            return (float)random.NextDouble() * (max - min) + min;
        }

        public static Vector3 NextVector3(this Random random)
        {
            return new Vector3(
                random.NextFloat(),
                random.NextFloat(),
                random.NextFloat()
                );
        }

        public static Vector3 NextVector3(this Random random, Vector3 min, Vector3 max)
        {
            if (max.X < min.X)
                throw new ArgumentException("Max (X) cannot be less than min");

            if (max.Y < min.Y)
                throw new ArgumentException("Max (Y) cannot be less than min");

            if (max.Z < min.Z)
                throw new ArgumentException("Max (Z) cannot be less than min");

            return new Vector3(
                random.NextFloat(min.X, max.X),
                random.NextFloat(min.Y, max.Y),
                random.NextFloat(min.Z, max.Z)
                );
        }

        public static Matrix Orientation(this Matrix matrix)
        {
            Matrix m = Matrix.Identity;
            m.M11 = matrix.M11;
            m.M12 = matrix.M12;
            m.M13 = matrix.M13;
            m.M21 = matrix.M21;
            m.M22 = matrix.M22;
            m.M23 = matrix.M23;
            m.M31 = matrix.M31;
            m.M32 = matrix.M32;
            m.M33 = matrix.M33;

            return m;
        }

        public static void SetLength(ref Vector2 v, float length)
        {
            v.Normalize();
            v *= length;
        }

        public static void SetLength(ref Vector3 v, float length)
        {
            v.Normalize();
            v *= length;
        }

        /// <summary>
        /// Swaps a with b
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        public static void Swap(ref int a, ref int b)
        {
            int t;

            t = a;
            a = b;
            b = t;
        }

        public static void Swap(ref float a, ref float b)
        {
            float t;

            t = a;
            a = b;
            b = t;
        }

        public static void Swap(ref Vector2 a, ref Vector2 b)
        {
            Vector2 t;

            t = a;
            a = b;
            b = t;
        }

        public static void Swap(ref Vector3 a, ref Vector3 b)
        {
            Vector3 t;

            t = a;
            a = b;
            b = t;
        }

        /// <summary>
        /// Wraps an integer around a min/max range
        /// </summary>
        /// <param name="value">Integer to wrap</param>
        /// <param name="min">Minimum number in wrap range</param>
        /// <param name="max">Maximum number in wrap range</param>
        /// <returns></returns>
        public static int Wrap(int value, int min, int max)
        {
            if (max < min)
            {
                Swap(ref min, ref max);
            }

            int count = max - min + 1;
            int offset = value < min ? value - min + 1 : value - min;
            int origin = value < min ? max : min;

            return (offset % count) + origin;
        }
    }
}