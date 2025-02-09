// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using NUnit.Framework;
using static Box2D.NET.Engine.math_function;
using static Box2D.NET.Engine.distance;

namespace Box2D.NET.Engine.Test;

public class test_distance : test_macros
{
    [Test]
    public void SegmentDistanceTest()
    {
        b2Vec2 p1 = new b2Vec2(-1.0f, -1.0f);
        b2Vec2 q1 = new b2Vec2(-1.0f, 1.0f);
        b2Vec2 p2 = new b2Vec2(2.0f, 0.0f);
        b2Vec2 q2 = new b2Vec2(1.0f, 0.0f);

        b2SegmentDistanceResult result = b2SegmentDistance(p1, q1, p2, q2);

        ENSURE_SMALL(result.fraction1 - 0.5f, Epsilon);
        ENSURE_SMALL(result.fraction2 - 1.0f, Epsilon);
        ENSURE_SMALL(result.closest1.x + 1.0f, Epsilon);
        ENSURE_SMALL(result.closest1.y, Epsilon);
        ENSURE_SMALL(result.closest2.x - 1.0f, Epsilon);
        ENSURE_SMALL(result.closest2.y, Epsilon);
        ENSURE_SMALL(result.distanceSquared - 4.0f, Epsilon);
    }

    [Test]
    public void ShapeDistanceTest()
    {
        b2Vec2[] vas =
        [
            new b2Vec2(-1.0f, -1.0f),
            new b2Vec2(1.0f, -1.0f),
            new b2Vec2(1.0f, 1.0f),
            new b2Vec2(-1.0f, 1.0f)
        ];
        b2Vec2[] vbs =
        [
            new b2Vec2(2.0f, -1.0f),
            new b2Vec2(2.0f, 1.0f)
        ];

        b2DistanceInput input = new b2DistanceInput();
        input.proxyA = b2MakeProxy(vas, ARRAY_COUNT(vas), 0.0f);
        input.proxyB = b2MakeProxy(vbs, ARRAY_COUNT(vbs), 0.0f);
        input.transformA = b2Transform_identity;
        input.transformB = b2Transform_identity;
        input.useRadii = false;

        b2SimplexCache cache = new b2SimplexCache();
        b2DistanceOutput output = b2ShapeDistance(cache, input, null, 0);

        ENSURE_SMALL(output.distance - 1.0f, Epsilon);
    }

    [Test]
    public void ShapeCastTest()
    {
        b2Vec2[] vas =
        [
            new b2Vec2(-1.0f, -1.0f),
            new b2Vec2(1.0f, -1.0f),
            new b2Vec2(1.0f, 1.0f),
            new b2Vec2(-1.0f, 1.0f)
        ];

        b2Vec2[] vbs =
        [
            new b2Vec2(2.0f, -1.0f),
            new b2Vec2(2.0f, 1.0f)
        ];

        b2ShapeCastPairInput input = new b2ShapeCastPairInput();
        input.proxyA = b2MakeProxy(vas, ARRAY_COUNT(vas), 0.0f);
        input.proxyB = b2MakeProxy(vbs, ARRAY_COUNT(vbs), 0.0f);
        input.transformA = b2Transform_identity;
        input.transformB = b2Transform_identity;
        input.translationB = new b2Vec2(-2.0f, 0.0f);
        input.maxFraction = 1.0f;

        b2CastOutput output = b2ShapeCast(input);

        ENSURE(output.hit);
        ENSURE_SMALL(output.fraction - 0.5f, 0.005f);
    }

    [Test]
    public void TimeOfImpactTest()
    {
        b2Vec2[] vas =
        [
            new b2Vec2(-1.0f, -1.0f),
            new b2Vec2(1.0f, -1.0f),
            new b2Vec2(1.0f, 1.0f),
            new b2Vec2(-1.0f, 1.0f)
        ];

        b2Vec2[] vbs =
        [
            new b2Vec2(2.0f, -1.0f),
            new b2Vec2(2.0f, 1.0f)
        ];

        b2TOIInput input = new b2TOIInput();
        input.proxyA = b2MakeProxy(vas, ARRAY_COUNT(vas), 0.0f);
        input.proxyB = b2MakeProxy(vbs, ARRAY_COUNT(vbs), 0.0f);
        input.sweepA = new b2Sweep(b2Vec2_zero, b2Vec2_zero, b2Vec2_zero, b2Rot_identity, b2Rot_identity);
        input.sweepB = new b2Sweep(b2Vec2_zero, b2Vec2_zero, new b2Vec2(-2.0f, 0.0f), b2Rot_identity, b2Rot_identity);
        input.maxFraction = 1.0f;

        b2TOIOutput output = b2TimeOfImpact(input);

        ENSURE(output.state == b2TOIState.b2_toiStateHit);
        ENSURE_SMALL(output.fraction - 0.5f, 0.005f);
    }
}