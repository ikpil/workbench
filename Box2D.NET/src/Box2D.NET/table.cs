// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System.Diagnostics;
using static Box2D.NET.ctz;
using static Box2D.NET.math_function;
using static Box2D.NET.core;

namespace Box2D.NET;

// @ikpil, must be strcut
public struct b2SetItem
{
    public ulong key;
    public uint hash;

    public void Clear()
    {
        key = 0;
        hash = 0;
    }
}

public class b2HashSet
{
    public b2SetItem[] items;
    public int capacity;
    public uint count;
}

public class table
{
    public static ulong B2_SHAPE_PAIR_KEY(long K1, long K2)
    {
        return K1 < K2 ? (ulong)K1 << 32 | (ulong)K2 : (ulong)K2 << 32 | (ulong)K1;
    }

#if B2_SNOOP_TABLE_COUNTERS
b2AtomicInt b2_findCount;
b2AtomicInt b2_probeCount;
#endif

// todo compare with https://github.com/skeeto/scratch/blob/master/set32/set32.h

    public static b2HashSet b2CreateSet(int capacity)
    {
        b2HashSet set = new b2HashSet();

        // Capacity must be a power of 2
        if (capacity > 16)
        {
            set.capacity = b2RoundUpPowerOf2(capacity);
        }
        else
        {
            set.capacity = 16;
        }

        set.count = 0;
        set.items = b2Alloc<b2SetItem>(capacity);
        //memset(set.items, 0, capacity * sizeof(b2SetItem));

        return set;
    }

    public static void b2DestroySet(b2HashSet set)
    {
        b2Free(set.items, set.capacity);
        set.items = null;
        set.count = 0;
        set.capacity = 0;
    }

    public static void b2ClearSet(b2HashSet set)
    {
        set.count = 0;
        //memset(set.items, 0, set.capacity);
        for (int i = 0; i < set.capacity; ++i)
        {
            set.items[i].Clear();
        }
    }

    // I need a good hash because the keys are built from pairs of increasing integers.
    // A simple hash like hash = (integer1 XOR integer2) has many collisions.
    // https://lemire.me/blog/2018/08/15/fast-strongly-universal-64-bit-hashing-everywhere/
    // https://preshing.com/20130107/this-hash-set-is-faster-than-a-judy-array/
    // todo try: https://www.jandrewrogers.com/2019/02/12/fast-perfect-hashing/
    // todo try: https://probablydance.com/2018/06/16/fibonacci-hashing-the-optimization-that-the-world-forgot-or-a-better-alternative-to-integer-modulo/
    public static uint b2KeyHash(ulong key)
    {
        ulong h = key;
        h ^= h >> 33;
        h *= 0xff51afd7ed558ccdL;
        h ^= h >> 33;
        h *= 0xc4ceb9fe1a85ec53L;
        h ^= h >> 33;

        return (uint)h;

        // todo_erin 
        // return 11400714819323198485ull * key;
    }

    public static int b2FindSlot(b2HashSet set, ulong key, uint hash)
    {
#if B2_SNOOP_TABLE_COUNTERS
		b2AtomicFetchAddInt( &b2_findCount, 1 );
#endif

        int capacity = set.capacity;
        int index = (int)(hash & (capacity - 1));
        b2SetItem[] items = set.items;
        while (items[index].hash != 0 && items[index].key != key)
        {
#if B2_SNOOP_TABLE_COUNTERS
		b2AtomicFetchAddInt( &b2_probeCount, 1 );
#endif
            index = (index + 1) & (capacity - 1);
        }

        return index;
    }

    public static void b2AddKeyHaveCapacity(b2HashSet set, ulong key, uint hash)
    {
        int index = b2FindSlot(set, key, hash);
        b2SetItem[] items = set.items;
        Debug.Assert(items[index].hash == 0);

        items[index].key = key;
        items[index].hash = hash;
        set.count += 1;
    }

    public static void b2GrowTable(b2HashSet set)
    {
        uint oldCount = set.count;
        B2_UNUSED(oldCount);

        int oldCapacity = set.capacity;
        b2SetItem[] oldItems = set.items;

        set.count = 0;
        // Capacity must be a power of 2
        set.capacity = 2 * oldCapacity;
        set.items = b2Alloc<b2SetItem>(set.capacity);
        //memset(set.items, 0, set.capacity * sizeof(b2SetItem));

        // Transfer items into new array
        for (uint i = 0; i < oldCapacity; ++i)
        {
            b2SetItem item = oldItems[i];
            if (item.hash == 0)
            {
                // this item was empty
                continue;
            }

            b2AddKeyHaveCapacity(set, item.key, item.hash);
        }

        Debug.Assert(set.count == oldCount);

        b2Free(oldItems, oldCapacity);
    }

    public static bool b2ContainsKey(b2HashSet set, ulong key)
    {
        // key of zero is a sentinel
        Debug.Assert(key != 0);
        uint hash = b2KeyHash(key);
        int index = b2FindSlot(set, key, hash);
        return set.items[index].key == key;
    }

    public static int b2GetHashSetBytes(b2HashSet set)
    {
        // TODO: @ikpil, size check
        //return set.capacity * sizeof(b2SetItem);
        return set.capacity * 12;
    }

    // Returns true if key was already in set
    public static bool b2AddKey(b2HashSet set, ulong key)
    {
        // key of zero is a sentinel
        Debug.Assert(key != 0);

        uint hash = b2KeyHash(key);
        Debug.Assert(hash != 0);

        int index = b2FindSlot(set, key, hash);
        if (set.items[index].hash != 0)
        {
            // Already in set
            Debug.Assert(set.items[index].hash == hash && set.items[index].key == key);
            return true;
        }

        if (2 * set.count >= set.capacity)
        {
            b2GrowTable(set);
        }

        b2AddKeyHaveCapacity(set, key, hash);
        return false;
    }

    // Returns true if the key was found
    // See https://en.wikipedia.org/wiki/Open_addressing
    public static bool b2RemoveKey(b2HashSet set, ulong key)
    {
        uint hash = b2KeyHash(key);
        int i = b2FindSlot(set, key, hash);
        b2SetItem[] items = set.items;
        if (items[i].hash == 0)
        {
            // Not in set
            return false;
        }

        // Mark item i as unoccupied
        items[i].key = 0;
        items[i].hash = 0;

        Debug.Assert(set.count > 0);
        set.count -= 1;

        // Attempt to fill item i
        int j = i;
        int capacity = set.capacity;
        for (;;)
        {
            j = (j + 1) & (capacity - 1);
            if (items[j].hash == 0)
            {
                break;
            }

            // k is the first item for the hash of j
            int k = (int)(items[j].hash & (capacity - 1));

            // determine if k lies cyclically in (i,j]
            // i <= j: | i..k..j |
            // i > j: |.k..j  i....| or |....j     i..k.|
            if (i <= j)
            {
                if (i < k && k <= j)
                {
                    continue;
                }
            }
            else
            {
                if (i < k || k <= j)
                {
                    continue;
                }
            }

            // Move j into i
            items[i].key = items[j].key;
            items[i].hash = items[j].hash;

            // Mark item j as unoccupied
            items[j].key = 0;
            items[j].hash = 0;

            i = j;
        }

        return true;
    }
}