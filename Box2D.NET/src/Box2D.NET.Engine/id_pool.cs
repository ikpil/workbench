// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System.Diagnostics;
using static Box2D.NET.Engine.array;
using static Box2D.NET.Engine.core;

namespace Box2D.NET.Engine;

public class b2IdPool
{
    public b2Array<int> freeArray;
    public int nextIndex;
}

public class id_pool
{
    public static int b2GetIdCount(b2IdPool pool)
    {
        return pool.nextIndex - pool.freeArray.count;
    }

    public static int b2GetIdCapacity(b2IdPool pool)
    {
        return pool.nextIndex;
    }

    public static int b2GetIdBytes(b2IdPool pool)
    {
        return Array_ByteCount(pool.freeArray);
    }


    public static b2IdPool b2CreateIdPool()
    {
        b2IdPool pool = new b2IdPool();
        pool.freeArray = Array_Create<int>(32);
        return pool;
    }

    public static void b2DestroyIdPool(ref b2IdPool pool)
    {
        Array_Destroy(pool.freeArray);
        pool = new b2IdPool(); // TODO: @ikpil check pool
    }

    public static int b2AllocId(b2IdPool pool)
    {
        int count = pool.freeArray.count;
        if (count > 0)
        {
            int id = Array_Pop(pool.freeArray);
            return id;
        }

        int nextId = pool.nextIndex;
        pool.nextIndex += 1;
        return nextId;
    }

    public static void b2FreeId(b2IdPool pool, int id)
    {
        Debug.Assert(pool.nextIndex > 0);
        Debug.Assert(0 <= id && id < pool.nextIndex);

        if (id == pool.nextIndex)
        {
            pool.nextIndex -= 1;
            return;
        }

        Array_Push(pool.freeArray, id);
    }

#if B2_VALIDATE
void b2ValidateFreeId( b2IdPool* pool, int id )
{
	int freeCount = pool.freeArray.count;
	for ( int i = 0; i < freeCount; ++i )
	{
		if ( pool.freeArray.data[i] == id )
		{
			return;
		}
	}

	Debug.Assert( 0 );
}

#else

    public static void b2ValidateFreeId(b2IdPool pool, int id)
    {
        B2_UNUSED(pool);
        B2_UNUSED(id);
    }

#endif
}