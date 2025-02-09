// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using NUnit.Framework;
using static Box2D.NET.Engine.geometry;
using static Box2D.NET.Engine.math_function;
using static Box2D.NET.Engine.hull;

namespace Box2D.NET.Engine.Test;

public class test_shape : test_macros
{
    private b2Capsule capsule = new b2Capsule(new b2Vec2(-1.0f, 0.0f ), new b2Vec2(1.0f, 0.0f ), 1.0f);
    private b2Circle circle = new b2Circle(new b2Vec2( 1.0f, 0.0f ), 1.0f);
    private b2Polygon box = b2MakeBox(1.0f, 1.0f);
    private b2Segment segment = new b2Segment(new b2Vec2(0.0f, 1.0f), new b2Vec2(0.0f, -1.0f));

    public const int N = 4;


    [Test]
    public void ShapeMassTest()
    {
        {
            b2MassData md = b2ComputeCircleMass(circle, 1.0f);
            ENSURE_SMALL(md.mass - B2_PI, Epsilon);
            ENSURE(md.center.x == 1.0f && md.center.y == 0.0f);
            ENSURE_SMALL(md.rotationalInertia - 1.5f * B2_PI, Epsilon);
        }

        {
            float radius = capsule.radius;
            float length = b2Distance(capsule.center1, capsule.center2);

            b2MassData md = b2ComputeCapsuleMass(capsule, 1.0f);

            // Box that full contains capsule
            b2Polygon r = b2MakeBox(radius, radius + 0.5f * length);
            b2MassData mdr = b2ComputePolygonMass(r, 1.0f);

            // Approximate capsule using convex hull
            b2Vec2[] points = new b2Vec2[2 * N];
            float d = B2_PI / (N - 1.0f);
            float angle = -0.5f * B2_PI;
            for (int i = 0; i < N; ++i)
            {
                points[i].x = 1.0f + radius * MathF.Cos(angle);
                points[i].y = radius * MathF.Sin(angle);
                angle += d;
            }

            angle = 0.5f * B2_PI;
            for (int i = N; i < 2 * N; ++i)
            {
                points[i].x = -1.0f + radius * MathF.Cos(angle);
                points[i].y = radius * MathF.Sin(angle);
                angle += d;
            }

            b2Hull hull = b2ComputeHull(points, 2 * N);
            b2Polygon ac = b2MakePolygon(hull, 0.0f);
            b2MassData ma = b2ComputePolygonMass(ac, 1.0f);

            ENSURE(ma.mass < md.mass && md.mass < mdr.mass);
            ENSURE(ma.rotationalInertia < md.rotationalInertia && md.rotationalInertia < mdr.rotationalInertia);
        }

        {
            b2MassData md = b2ComputePolygonMass(box, 1.0f);
            ENSURE_SMALL(md.mass - 4.0f, Epsilon);
            ENSURE_SMALL(md.center.x, Epsilon);
            ENSURE_SMALL(md.center.y, Epsilon);
            ENSURE_SMALL(md.rotationalInertia - 8.0f / 3.0f, 2.0f * Epsilon);
        }
    }

    [Test]
    public void ShapeAABBTest()
    {
        {
            b2AABB b = b2ComputeCircleAABB(circle, b2Transform_identity);
            ENSURE_SMALL(b.lowerBound.x, Epsilon);
            ENSURE_SMALL(b.lowerBound.y + 1.0f, Epsilon);
            ENSURE_SMALL(b.upperBound.x - 2.0f, Epsilon);
            ENSURE_SMALL(b.upperBound.y - 1.0f, Epsilon);
        }

        {
            b2AABB b = b2ComputePolygonAABB(box, b2Transform_identity);
            ENSURE_SMALL(b.lowerBound.x + 1.0f, Epsilon);
            ENSURE_SMALL(b.lowerBound.y + 1.0f, Epsilon);
            ENSURE_SMALL(b.upperBound.x - 1.0f, Epsilon);
            ENSURE_SMALL(b.upperBound.y - 1.0f, Epsilon);
        }

        {
            b2AABB b = b2ComputeSegmentAABB(segment, b2Transform_identity);
            ENSURE_SMALL(b.lowerBound.x, Epsilon);
            ENSURE_SMALL(b.lowerBound.y + 1.0f, Epsilon);
            ENSURE_SMALL(b.upperBound.x, Epsilon);
            ENSURE_SMALL(b.upperBound.y - 1.0f, Epsilon);
        }
    }

    [Test]
    public void PointInShapeTest()
    {
        b2Vec2 p1 = new b2Vec2(0.5f, 0.5f);
        b2Vec2 p2 = new b2Vec2(4.0f, -4.0f);

        {
            bool hit;
            hit = b2PointInCircle(p1, circle);
            ENSURE(hit == true);
            hit = b2PointInCircle(p2, circle);
            ENSURE(hit == false);
        }

        {
            bool hit;
            hit = b2PointInPolygon(p1, box);
            ENSURE(hit == true);
            hit = b2PointInPolygon(p2, box);
            ENSURE(hit == false);
        }
    }

    [Test]
    public void RayCastShapeTest()
    {
        b2RayCastInput input = new b2RayCastInput(new b2Vec2( -4.0f, 0.0f ), new b2Vec2( 8.0f, 0.0f ), 1.0f );

        {
            b2CastOutput output = b2RayCastCircle(input, circle);
            ENSURE(output.hit);
            ENSURE_SMALL(output.normal.x + 1.0f, Epsilon);
            ENSURE_SMALL(output.normal.y, Epsilon);
            ENSURE_SMALL(output.fraction - 0.5f, Epsilon);
        }

        {
            b2CastOutput output = b2RayCastPolygon(input, box);
            ENSURE(output.hit);
            ENSURE_SMALL(output.normal.x + 1.0f, Epsilon);
            ENSURE_SMALL(output.normal.y, Epsilon);
            ENSURE_SMALL(output.fraction - 3.0f / 8.0f, Epsilon);
        }

        {
            b2CastOutput output = b2RayCastSegment(input, segment, true);
            ENSURE(output.hit);
            ENSURE_SMALL(output.normal.x + 1.0f, Epsilon);
            ENSURE_SMALL(output.normal.y, Epsilon);
            ENSURE_SMALL(output.fraction - 0.5f, Epsilon);
        }
    }
}