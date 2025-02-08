// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

namespace Box2D.NET.Engine;

public class timer
{


#if defined( _WIN32 )

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif



    static double s_invFrequency = 0.0;

    ulong b2GetTicks()
    {
        LARGE_INTEGER counter;
        QueryPerformanceCounter(&counter);
        return (ulong)counter.QuadPart;
    }

    float b2GetMilliseconds(ulong ticks)
    {
        if (s_invFrequency == 0.0)
        {
            LARGE_INTEGER frequency;
            QueryPerformanceFrequency(&frequency);

            s_invFrequency = (double)frequency.QuadPart;
            if (s_invFrequency > 0.0)
            {
                s_invFrequency = 1000.0 / s_invFrequency;
            }
        }

        ulong ticksNow = b2GetTicks();
        return (float)(s_invFrequency * (ticksNow - ticks));
    }

    float b2GetMillisecondsAndReset(ulong* ticks)
    {
        if (s_invFrequency == 0.0)
        {
            LARGE_INTEGER frequency;
            QueryPerformanceFrequency(&frequency);

            s_invFrequency = (double)frequency.QuadPart;
            if (s_invFrequency > 0.0)
            {
                s_invFrequency = 1000.0 / s_invFrequency;
            }
        }

        ulong ticksNow = b2GetTicks();
        float ms = (float)(s_invFrequency * (ticksNow - *ticks));
        *ticks = ticksNow;
        return ms;
    }

    void b2Yield()
    {
        SwitchToThread();
    }

#elif defined( __linux__ ) || defined( __EMSCRIPTEN__ )




    ulong b2GetTicks()
    {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return ts.tv_sec * 1000000000LL + ts.tv_nsec;
    }

    float b2GetMilliseconds(ulong ticks)
    {
        ulong ticksNow = b2GetTicks();
        return (float)((ticksNow - ticks) / 1000000.0);
    }

    float b2GetMillisecondsAndReset(ulong* ticks)
    {
        ulong ticksNow = b2GetTicks();
        float ms = (float)((ticksNow - *ticks) / 1000000.0);
        *ticks = ticksNow;
        return ms;
    }

    void b2Yield()
    {
        sched_yield();
    }

#elif defined( __APPLE__ )





    static double s_invFrequency = 0.0;

    ulong b2GetTicks()
    {
        return mach_absolute_time();
    }

    float b2GetMilliseconds(ulong ticks)
    {
        if (s_invFrequency == 0)
        {
            mach_timebase_info_data_t timebase;
            mach_timebase_info(&timebase);

            // convert to ns then to ms
            s_invFrequency = 1e-6 * (double)timebase.numer / (double)timebase.denom;
        }

        ulong ticksNow = b2GetTicks();
        return (float)(s_invFrequency * (ticksNow - ticks));
    }

    float b2GetMillisecondsAndReset(ulong* ticks)
    {
        if (s_invFrequency == 0)
        {
            mach_timebase_info_data_t timebase;
            mach_timebase_info(&timebase);

            // convert to ns then to ms
            s_invFrequency = 1e-6 * (double)timebase.numer / (double)timebase.denom;
        }

        ulong ticksNow = b2GetTicks();
        float ms = (float)(s_invFrequency * (ticksNow - *ticks));
        *ticks = ticksNow;
        return ms;
    }

    void b2Yield()
    {
        sched_yield();
    }

#else

    ulong b2GetTicks()
    {
        return 0;
    }

    float b2GetMilliseconds(ulong ticks)
    {
        ((void)(ticks));
        return 0.0f;
    }

    float b2GetMillisecondsAndReset(ulong* ticks)
    {
        ((void)(ticks));
        return 0.0f;
    }

    void b2Yield()
    {
    }

#endif

// djb2 hash
// https://en.wikipedia.org/wiki/List_of_hash_functions
    uint b2Hash(uint hash,  const byte* data,  int count )
    {
        uint result = hash;
        for (int i = 0; i < count; i++)
        {
            result = (result << 5) + result + data[i];
        }

        return result;
    }

}