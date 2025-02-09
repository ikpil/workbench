using System;
using System.Diagnostics;

namespace Box2D.NET.Engine;

public class b2AtomicInt
{
    public volatile int value;
}

public class b2AtomicU32
{
    public volatile uint value;
}

public static class core
{
    public static T[] b2Alloc<T>(int count) where T : new()
    {
        if (typeof(T).IsValueType)
        {
            return new T[count];
        }

        T[] array = new T[count];
        for (int i = 0; i < count; i++)
        {
            array[i] = new T();
        }

        return array;
    }

    public static void b2Free<T>(T[] mem, int count)
    {
        // ..
    }
    
    public static T[] b2GrowAlloc<T>(T[] oldMem, int oldSize, int newSize) where T : new()
    {
        Debug.Assert(newSize > oldSize);
        T[] newMem = b2Alloc<T>(newSize);
        if (oldSize > 0)
        {
            memcpy<T>(newMem, oldMem, oldSize);
            b2Free(oldMem, oldSize);
        }

        return newMem;
    }


    public static void memset<T>(Span<T> array, T value , int count)
    {
        array.Slice(0, count).Fill(value);
    }

    public static void memcpy<T>(Span<T> dst, Span<T> src, int count)
    {
        src.Slice(0, count).CopyTo(dst);
    }
    
    public static void memcpy<T>(Span<T> dst, Span<T> src)
    {
        src.CopyTo(dst);
    }

    public static void B2_UNUSED<T>(T a)
    {
        // ...
    }
}