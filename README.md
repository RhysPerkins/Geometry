# Geometry
## A basic geometry library for XNA

This library contains classes for the following shapes

- Axis Aligned Bounding Box
- Frustum
- Oriented Bounding Box
- Plane
- Ray
- Sphere
- Triangle

Collision detection methods for each of the shapes are provided by the following class

- CollisionDetection

Here is an example of detecting and solving an intersection between to shapes

```
    Contact c = new Contact();
    AABB a = new AABB(Vector3.Zero, 1f, 1f, 1f);
    AABB b = new AABB(new Vector3(0.5f, 0.5f, 0.5f), 1f, 1f, 1f);

    CollisionDetection.AABBAABB(a, b, ref c);

    if (c.isIntersecting)
        a.Position += c.nEnter * c.penetration;
```

For more information, please visit [rhysperkins.com](www.rhysperkins.com)