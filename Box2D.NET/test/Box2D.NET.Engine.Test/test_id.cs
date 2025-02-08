// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

namespace Box2D.NET.Engine.Test;

public class test_id
{



    int IdTest(void)
    {
        ulong x = 0x0123456789ABCDEFull;

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

        return 0;
    }

}