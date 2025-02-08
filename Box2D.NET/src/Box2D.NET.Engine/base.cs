// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

namespace Box2D.NET.Engine;

public class base
{
// clang-format on

    /**
     * @defgroup base Base
     * Base functionality
     * @{
     */

    /// Prototype for user allocation function
    /// @param size the allocation size in bytes
    /// @param alignment the required alignment, guaranteed to be a power of 2
    typedef void* b2AllocFcn(unsigned
    int size,  int alignment );

    /// Prototype for user free function
    /// @param mem the memory previously allocated through `b2AllocFcn`
    typedef void b2FreeFcn(void* mem);

    /// Prototype for the user assert callback. Return 0 to skip the debugger break.
    typedef int b2AssertFcn(
    const char* condition,  const char* fileName,  int lineNumber );

    /// This allows the user to override the allocation functions. These should be
    /// set during application startup.
    void b2SetAllocator(b2AllocFcn* allocFcn, b2FreeFcn* freeFcn);

    /// @return the total bytes allocated by Box2D
    int b2GetByteCount();

    /// Override the default assert callback
    /// @param assertFcn a non-null assert callback
    void b2SetAssertFcn(b2AssertFcn* assertFcn);

#if !defined( NDEBUG ) || defined( B2_ENABLE_ASSERT )
    int b2InternalAssertFcn(
    const char* condition,  const char* fileName,  int lineNumber );
#define Debug.Assert( condition ) \
    do \
    { \
        if (!(condition) && b2InternalAssertFcn( #condition, __FILE__, (int)__LINE__ ) ) \
        B2_BREAKPOINT; \
    } \
    while (0)
#else
#define Debug.Assert( ... ) ( (void)0 )
#endif

        /// Version numbering scheme.
        /// See https://semver.org/
        typedef struct b2Version
    {
        /// Significant changes
        int major;

        /// Incremental changes
        int minor;

        /// Bug fixes
        int revision;
    }
    b2Version;

    /// Get the current version of Box2D
    b2Version b2GetVersion();

    /**@}*/

//! @cond

    /// Get the absolute number of system ticks. The value is platform specific.
    ulong b2GetTicks();

    /// Get the milliseconds passed from an initial tick value.
    float b2GetMilliseconds(ulong ticks);

    /// Get the milliseconds passed from an initial tick value.
    float b2GetMillisecondsAndReset(ulong[] ticks);

    /// Yield to be used in a busy loop.
    void b2Yield();

    /// Simple djb2 hash function for determinism testing
#define B2_HASH_INIT 5381
    uint b2Hash(uint hash,  const byte* data,  int count );

//! @endcond

}