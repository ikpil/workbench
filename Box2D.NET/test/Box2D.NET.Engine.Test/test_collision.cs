// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

namespace Box2D.NET.Engine.Test;

public class test_collision : test_macros
{
    public void AABBTest()
    {
        b2AABB a;
        a.lowerBound = (b2Vec2){
            -1.0f, -1.0f
        }
        ;
        a.upperBound = (b2Vec2){
            -2.0f, -2.0f
        }
        ;

        ENSURE(b2IsValidAABB(a) == false);

        a.upperBound = (b2Vec2){
            1.0f, 1.0f
        }
        ;
        ENSURE(b2IsValidAABB(a) == true);

        b2AABB b = { { 2.0f, 2.0f }, { 4.0f, 4.0f } };
        ENSURE(b2AABB_Overlaps(a, b) == false);
        ENSURE(b2AABB_Contains(a, b) == false);

        b2Vec2 p1 = (b2Vec2){
            -2.0f, 0.0f
        }
        ;
        b2Vec2 p2 = (b2Vec2){
            2.0f, 0.0f
        }
        ;

        b2CastOutput output = b2AABB_RayCast(a, p1, p2);
        ENSURE(output.hit == true);
        ENSURE(0.1f < output.fraction && output.fraction < 0.9f);
    }

    int CollisionTest()
    {
        RUN_SUBTEST(AABBTest);

        return 0;
    }
}