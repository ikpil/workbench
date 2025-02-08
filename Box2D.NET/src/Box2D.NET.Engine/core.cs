using System;

namespace Box2D.NET.Engine;

public static class core
{
    public static T[] b2Alloc<T>(int count)
    {
        return new T[count];
    }

    public static void b2Free<T>(T[] mem, int count)
    {
        // ..
    }

    public static void memset<T>(T[] array, int index, int count)
    {
        Array.Fill(array, default, index, count);
    }

    public static void memcpy<T>(T[] dst, T[] src, int count)
    {
        Array.Copy(src, dst, count);
    }
}