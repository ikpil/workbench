// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using NUnit.Framework;
using static Box2D.NET.Engine.id;
using static Box2D.NET.Engine.solver;
using static Box2D.NET.Engine.body;
using static Box2D.NET.Engine.world;
using static Box2D.NET.Engine.joint;
using static Box2D.NET.Engine.id_pool;

namespace Box2D.NET.Engine.Test;

public class test_id : test_macros
{
    [Test]
    public void IdTest()
    {
        ulong x = 0x0123456789ABCDEFul;

        {
            b2BodyId id = b2LoadBodyId(x);
            ulong y = b2StoreBodyId(id);
            ENSURE(x == y);
        }

        {
            b2ShapeId id = b2LoadShapeId(x);
            ulong y = b2StoreShapeId(id);
            ENSURE(x == y);
        }

        {
            b2ChainId id = b2LoadChainId(x);
            ulong y = b2StoreChainId(id);
            ENSURE(x == y);
        }

        {
            b2JointId id = b2LoadJointId(x);
            ulong y = b2StoreJointId(id);
            ENSURE(x == y);
        }
    }
}