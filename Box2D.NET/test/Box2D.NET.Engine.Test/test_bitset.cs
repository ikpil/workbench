// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT


namespace Box2D.NET.Engine.Test;

public class test_bitset
{

    private const int COUNT = 169;

    public void BitSetTest()
    {
        b2BitSet bitSet = b2CreateBitSet(COUNT);

        b2SetBitCountAndClear(&bitSet, COUNT);
        bool values[COUNT] =  {
            false
        }
        ;

        int i1 = 0, i2 = 1;
        b2SetBit(&bitSet, i1);
        values[i1] = true;

        while (i2 < COUNT)
        {
            b2SetBit(&bitSet, i2);
            values[i2] = true;
            int next = i1 + i2;
            i1 = i2;
            i2 = next;
        }

        for (int i = 0; i < COUNT; ++i)
        {
            bool value = b2GetBit(&bitSet, i);
            ENSURE(value == values[i]);
        }

        b2DestroyBitSet(&bitSet);
    }

}