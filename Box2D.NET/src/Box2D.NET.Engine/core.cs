using System;

namespace Box2D.NET.Engine;

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

    public static void memset<T>(T[] array, int index, int count)
    {
        Array.Fill(array, default, index, count);
    }

    public static void memcpy<T>(Span<T> dst, Span<T> src, int count)
    {
        src.Slice(0, count).CopyTo(dst);
    }
}