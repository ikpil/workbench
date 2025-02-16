// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using NUnit.Framework;
using static Box2D.NET.Engine.ctz;
using static Box2D.NET.Engine.table;
using static Box2D.NET.Engine.timer;

namespace Box2D.NET.Engine.Test;

public class test_table
{
    public const int SET_SPAN = 317;
    public const int ITEM_COUNT = ((SET_SPAN * SET_SPAN - SET_SPAN) / 2);

    [Test]
    public void TableTest()
    {
        int power = b2BoundingPowerOf2(3008);
        Assert.That(power, Is.EqualTo(12));

        int nextPowerOf2 = b2RoundUpPowerOf2(3008);
        Assert.That(nextPowerOf2, Is.EqualTo((1 << power)));

        const int N = SET_SPAN;
        const uint itemCount = ITEM_COUNT;
        bool[] removed = new bool[ITEM_COUNT];

        for (int iter = 0; iter < 1; ++iter)
        {
            b2HashSet set = b2CreateSet(16);

            // Fill set
            for (int i = 0; i < N; ++i)
            {
                for (int j = i + 1; j < N; ++j)
                {
                    ulong key = B2_SHAPE_PAIR_KEY(i, j);
                    b2AddKey(set, key);
                }
            }

            Assert.That(set.count, Is.EqualTo(itemCount));

            // Remove a portion of the set
            int k = 0;
            uint removeCount = 0;
            for (int i = 0; i < N; ++i)
            {
                for (int j = i + 1; j < N; ++j)
                {
                    if (j == i + 1)
                    {
                        ulong key = B2_SHAPE_PAIR_KEY(i, j);
                        b2RemoveKey(set, key);
                        removed[k++] = true;
                        removeCount += 1;
                    }
                    else
                    {
                        removed[k++] = false;
                    }
                }
            }

            Assert.That(set.count, Is.EqualTo((itemCount - removeCount)));

#if B2_SNOOP_TABLE_COUNTERS
		extern b2AtomicInt b2_probeCount;
		b2AtomicStoreInt( &b2_probeCount, 0 );
#endif

            // Test key search
            // ~5ns per search on an AMD 7950x
            ulong ticks = b2GetTicks();

            k = 0;
            for (int i = 0; i < N; ++i)
            {
                for (int j = i + 1; j < N; ++j)
                {
                    ulong key = B2_SHAPE_PAIR_KEY(j, i);
                    Assert.That(b2ContainsKey(set, key) || removed[k], $"b2ContainsKey(set, {key}) = {b2ContainsKey(set, key)} || removed[{k}] = {removed[k]}");
                    k += 1;
                }
            }

            // ulong ticks = b2GetTicks(&timer);
            // Console.Write("set ticks = %llu\n", ticks);

            float ms = b2GetMilliseconds(ticks);
            Console.Write("set: count = %d, b2ContainsKey = %.5f ms, ave = %.5f us\n", itemCount, ms, 1000.0f * ms / itemCount);

#if B2_SNOOP_TABLE_COUNTERS
		int probeCount = b2AtomicLoadInt( &b2_probeCount );
		float aveProbeCount = (float)probeCount / (float)itemCount;
		Console.Write( "item count = %d, probe count = %d, ave probe count %.2f\n", itemCount, probeCount, aveProbeCount );
#endif

            // Remove all keys from set
            for (int i = 0; i < N; ++i)
            {
                for (int j = i + 1; j < N; ++j)
                {
                    ulong key = B2_SHAPE_PAIR_KEY(i, j);
                    b2RemoveKey(set, key);
                }
            }

            Assert.That(set.count, Is.EqualTo(0));

            b2DestroySet(set);
        }
    }
}