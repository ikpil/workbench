// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT


using System.Dynamic;

namespace Box2D.NET.Engine
{
    public class Collision
    {
        /// The maximum number of vertices on a convex polygon. Changing this affects performance even if you
        /// don't use more vertices.
        public const int B2_MAX_POLYGON_VERTICES = 8;
    }

    /// Low level ray cast input data
    public struct b2RayCastInput
    {
        /// Start point of the ray cast
        b2Vec2 origin;

        /// Translation of the ray cast
        b2Vec2 translation;

        /// The maximum fraction of the translation to consider, typically 1
        float maxFraction;
    }

    /// Low level shape cast input in generic form. This allows casting an arbitrary point
    /// cloud wrap with a radius. For example, a circle is a single point with a non-zero radius.
    /// A capsule is two points with a non-zero radius. A box is four points with a zero radius.
    public struct b2ShapeCastInput
    {
        /// A point cloud to cast
        b2Vec2[] points;

        /// The number of points
        int count;

        /// The radius around the point cloud
        float radius;

        /// The translation of the shape cast
        b2Vec2 translation;

        /// The maximum fraction of the translation to consider, typically 1
        float maxFraction;

        public static b2ShapeCastInput Create()
        {
            var s = new b2ShapeCastInput();
            s.points = new b2Vec2[Collision.B2_MAX_POLYGON_VERTICES];
            return s;
        }
    }

    /// Low level ray cast or shape-cast output data
    public struct b2CastOutput
    {
        /// The surface normal at the hit point
        b2Vec2 normal;

        /// The surface hit point
        b2Vec2 point;

        /// The fraction of the input translation at collision
        float fraction;

        /// The number of iterations used
        int iterations;

        /// Did the cast hit?
        bool hit;
    }

    /// This holds the mass data computed for a shape.
    public struct b2MassData
    {
        /// The mass of the shape, usually in kilograms.
        float mass;

        /// The position of the shape's centroid relative to the shape's origin.
        b2Vec2 center;

        /// The rotational inertia of the shape about the local origin.
        float rotationalInertia;
    }

    /// A solid circle
    public struct b2Circle
    {
        /// The local center
        b2Vec2 center;

        /// The radius
        float radius;
    }

    /// A solid capsule can be viewed as two semicircles connected
    /// by a rectangle.
    public struct b2Capsule
    {
        /// Local center of the first semicircle
        b2Vec2 center1;

        /// Local center of the second semicircle
        b2Vec2 center2;

        /// The radius of the semicircles
        float radius;
    }

    /// A solid convex polygon. It is assumed that the interior of the polygon is to
    /// the left of each edge.
    /// Polygons have a maximum number of vertices equal to B2_MAX_POLYGON_VERTICES.
    /// In most cases you should not need many vertices for a convex polygon.
    /// @warning DO NOT fill this out manually, instead use a helper function like
    /// b2MakePolygon or b2MakeBox.
    public struct b2Polygon
    {
        /// The polygon vertices
        b2Vec2[] vertices;

        /// The outward normal vectors of the polygon sides
        b2Vec2[] normals;

        /// The centroid of the polygon
        b2Vec2 centroid;

        /// The external radius for rounded polygons
        float radius;

        /// The number of polygon vertices
        int count;

        public static b2Polygon Create()
        {
            var p = new b2Polygon();
            p.vertices = new b2Vec2[Collision.B2_MAX_POLYGON_VERTICES];
            p.normals = new b2Vec2[Collision.B2_MAX_POLYGON_VERTICES];
            return p;
        }
    }

    /// A line segment with two-sided collision.
    public struct b2Segment
    {
        /// The first point
        b2Vec2 point1;

        /// The second point
        b2Vec2 point2;
    }

    /// A line segment with one-sided collision. Only collides on the right side.
    /// Several of these are generated for a chain shape.
    /// ghost1 -> point1 -> point2 -> ghost2
    public struct b2ChainSegment
    {
        /// The tail ghost vertex
        b2Vec2 ghost1;

        /// The line segment
        b2Segment segment;

        /// The head ghost vertex
        b2Vec2 ghost2;

        /// The owning chain shape index (internal usage only)
        int chainId;
    }
}