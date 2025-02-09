// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System.Diagnostics;
using System.Threading;

namespace Box2D.NET.Engine;

public static class timer
{
    private static readonly double s_invFrequency = 1000000000UL / (double)Stopwatch.Frequency; // counter to nano

    /// Get the absolute number of system ticks. The value is platform specific.
    // nanoseconds
    public static ulong b2GetTicks()
    {
        long counter = Stopwatch.GetTimestamp();
        return (ulong)(counter * s_invFrequency);
    }

    /// Get the milliseconds passed from an initial tick value.
    public static float b2GetMilliseconds(ulong ticks)
    {
        ulong ticksNow = b2GetTicks();
        return (ticksNow - ticks) / (float)1000000;
    }

    /// Get the milliseconds passed from an initial tick value.
    public static float b2GetMillisecondsAndReset(ref ulong ticks)
    {
        ulong ticksNow = b2GetTicks();
        float ms = (ticksNow - ticks) / (float)1000000;
        ticks = ticksNow;
        return ms;
    }

    /// Yield to be used in a busy loop.
    public static void b2Yield()
    {
        Thread.Yield();
    }

    // djb2 hash
    // https://en.wikipedia.org/wiki/List_of_hash_functions
    public static uint b2Hash(uint hash,  byte[] data,  int count )
    {
        uint result = hash;
        for (int i = 0; i < count; i++)
        {
            result = (result << 5) + result + data[i];
        }

        return result;
    }

}