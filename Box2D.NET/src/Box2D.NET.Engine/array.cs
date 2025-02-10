// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT


using System.Diagnostics;
using static Box2D.NET.Engine.constants;
using static Box2D.NET.Engine.core;

namespace Box2D.NET.Engine;

// Macro generated functions for dynamic arrays
// Pros
// - type safe
// - array data debuggable (visible count and capacity)
// - bounds checking
// - forward declaration
// - simple implementation
// - generates functions (like C++ templates)
// - functions have https://en.wikipedia.org/wiki/Sequence_point
// - avoids stretchy buffer dropped pointer update bugs
// Cons
// - cannot debug
// - breaks code navigation

// todo_erin consider code-gen: https://github.com/IbrahimHindawi/haikal

// Array declaration that doesn't need the type T to be defined
public class b2Array<T>
{
    public T[] data;
    public int count;
    public int capacity;
}

public static class array
{
    /* Resize */
    public static void Array_Resize<T>(b2Array<T> a, int count) where T : new()
    {
        Array_Reserve(a, count);
        a.count = count;
    }

    /* Get */
    public static T Array_Get<T>(b2Array<T> a, int index)
    {
        Debug.Assert(0 <= index && index < a.count);
        return a.data[index];
    }

    /* Add */
    public static ref T Array_Add<T>(b2Array<T> a) where T : new()
    {
        if (a.count == a.capacity)
        {
            int newCapacity = a.capacity < 2 ? 2 : a.capacity + (a.capacity >> 1);
            Array_Reserve(a, newCapacity);
        }

        a.count += 1;
        return ref a.data[a.count - 1];
    }

    /* Push */
    public static void Array_Push<T>(b2Array<T> a, T value) where T : new()
    {
        if (a.count == a.capacity)
        {
            int newCapacity = a.capacity < 2 ? 2 : a.capacity + (a.capacity >> 1);
            Array_Reserve(a, newCapacity);
        }

        a.data[a.count] = value;
        a.count += 1;
    }

    /* Set */
    public static void Array_Set<T>(b2Array<T> a, int index, T value)
    {
        Debug.Assert(0 <= index && index < a.count);
        a.data[index] = value;
    }

    /* RemoveSwap */
    public static int Array_RemoveSwap<T>(b2Array<T> a, int index)
    {
        Debug.Assert(0 <= index && index < a.count);
        int movedIndex = B2_NULL_INDEX;
        if (index != a.count - 1)
        {
            movedIndex = a.count - 1;
            a.data[index] = a.data[movedIndex];
        }

        a.count -= 1;
        return movedIndex;
    }

    /* Pop */
    public static T Array_Pop<T>(b2Array<T> a)
    {
        Debug.Assert(a.count > 0);
        T value = a.data[a.count - 1];
        a.count -= 1;
        return value;
    }

    /* Clear */
    public static void Array_Clear<T>(b2Array<T> a)
    {
        a.count = 0;
    }

    /* ByteCount */
    public static int Array_ByteCount<T>(b2Array<T> a)
    {
        // TODO: @ikpil, check
        //return (int)( a.capacity * sizeof( T ) );                                                                               
        return -1;
    }

// Array implementations to be instantiated in a source file where the type T is known
    /* Create */
    public static b2Array<T> Array_Create<T>(int capacity) where T : new()
    {
        b2Array<T> a = new b2Array<T>();
        if (capacity > 0)
        {
            a.data = b2Alloc<T>(capacity);
            a.capacity = capacity;
        }

        return a;
    }

    /* Reserve */
    public static void Array_Reserve<T>(b2Array<T> a, int newCapacity) where T : new()
    {
        if (newCapacity <= a.capacity)
        {
            return;
        }

        a.data = b2GrowAlloc(a.data, a.capacity, newCapacity);
        a.capacity = newCapacity;
    }

    /* Destroy */
    public static void Array_Destroy<T>(b2Array<T> a)
    {
        b2Free(a.data, a.capacity);
        a.data = null;
        a.count = 0;
        a.capacity = 0;
    }
}