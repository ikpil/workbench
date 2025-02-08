// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT


namespace Box2D.NET.Engine;

public class core
{



// clang-format off

#define B2_NULL_INDEX ( -1 )

// for performance comparisons
#define B2_RESTRICT restrict

#ifdef NDEBUG
#define B2_DEBUG 0
#else
#define B2_DEBUG 1
#endif

#if defined( BOX2D_VALIDATE ) && !defined( NDEBUG )
	#define B2_VALIDATE 1
#else
#define B2_VALIDATE 0
#endif

// Define platform
#if defined(_WIN32) || defined(_WIN64)
	#define B2_PLATFORM_WINDOWS
#elif defined( __ANDROID__ )
	#define B2_PLATFORM_ANDROID
#elif defined( __linux__ )
	#define B2_PLATFORM_LINUX
#elif defined( __APPLE__ )

	#if defined( TARGET_OS_IPHONE ) && !TARGET_OS_IPHONE
		#define B2_PLATFORM_MACOS
	#else
		#define B2_PLATFORM_IOS
	#endif
#elif defined( __EMSCRIPTEN__ )
	#define B2_PLATFORM_WASM
#else
#define B2_PLATFORM_UNKNOWN
#endif

// Define CPU
#if defined( __x86_64__ ) || defined( _M_X64 ) || defined( __i386__ ) || defined( _M_IX86 )
	#define B2_CPU_X86_X64
#elif defined( __aarch64__ ) || defined( _M_ARM64 ) || defined( __arm__ ) || defined( _M_ARM )
	#define B2_CPU_ARM
#elif defined( __EMSCRIPTEN__ )
	#define B2_CPU_WASM
#else
#define B2_CPU_UNKNOWN
#endif

// Define SIMD
#if defined( BOX2D_ENABLE_SIMD )
	#if defined( B2_CPU_X86_X64 )
		#if defined( BOX2D_AVX2 )
			#define B2_SIMD_AVX2
			#define B2_SIMD_WIDTH 8
		#else
			#define B2_SIMD_SSE2
			#define B2_SIMD_WIDTH 4
		#endif
	#elif defined( B2_CPU_ARM )
		#define B2_SIMD_NEON
		#define B2_SIMD_WIDTH 4
	#elif defined( B2_CPU_WASM )
		#define B2_CPU_WASM
		#define B2_SIMD_SSE2
		#define B2_SIMD_WIDTH 4
	#else
		#define B2_SIMD_NONE
		#define B2_SIMD_WIDTH 4
	#endif
#else
#define B2_SIMD_NONE
    // note: I tried width of 1 and got no performance change
#define B2_SIMD_WIDTH 4
#endif

// Define compiler
#if defined( __clang__ )
	#define B2_COMPILER_CLANG
#elif defined( __GNUC__ )
	#define B2_COMPILER_GCC
#elif defined( _MSC_VER )
	#define B2_COMPILER_MSVC
#endif

    /// Tracy profiler instrumentation
    /// https://github.com/wolfpld/tracy
#ifdef BOX2D_PROFILE

#define b2TracyCZoneC( ctx, color, active ) TracyCZoneC( ctx, color, active )
#define b2TracyCZoneNC( ctx, name, color, active ) TracyCZoneNC( ctx, name, color, active )
#define b2TracyCZoneEnd( ctx ) TracyCZoneEnd( ctx )
#else
#define b2TracyCZoneC( ctx, color, active )
#define b2TracyCZoneNC( ctx, name, color, active )
#define b2TracyCZoneEnd( ctx )
#endif

// clang-format on

// Returns the number of elements of an array
#define B2_ARRAY_COUNT( A ) (int)( sizeof( A ) / sizeof( A[0] ) )

// Used to prevent the compiler from warning about unused variables
#define B2_UNUSED( ... ) (void)sizeof( ( __VA_ARGS__, 0 ) )

// Use to validate definitions. Do not take my cookie.
#define B2_SECRET_COOKIE 1152023

// Snoop counters. These should be disabled in optimized builds because they are expensive.
#define B2_SNOOP_TABLE_COUNTERS B2_DEBUG
#define B2_SNOOP_PAIR_COUNTERS B2_DEBUG
#define B2_SNOOP_TOI_COUNTERS B2_DEBUG

#define B2_CHECK_DEF( DEF ) B2_ASSERT( DEF->internalValue == B2_SECRET_COOKIE )

    void* b2Alloc(int size);
#define B2_ALLOC_STRUCT( type ) b2Alloc(sizeof(type))
#define B2_ALLOC_ARRAY( count, type ) b2Alloc(count * sizeof(type))

    void b2Free(void* mem, int size);
#define B2_FREE_STRUCT( mem, type ) b2Free( mem, sizeof(type));
#define B2_FREE_ARRAY( mem, count, type ) b2Free(mem, count * sizeof(type))

    void* b2GrowAlloc(void* oldMem, int oldSize, int newSize);

    typedef struct b2AtomicInt
    {
        int value;
    }

    b2AtomicInt;

    typedef struct b2AtomicU32
    {
        uint value;
    }

    b2AtomicU32;

#if 0
    void b2AtomicStoreInt(b2AtomicInt* a, int value);
    int b2AtomicLoadInt(b2AtomicInt* a);
    int b2AtomicFetchAddInt(b2AtomicInt* a, int increment);
    bool b2AtomicCompareExchangeInt(b2AtomicInt* obj, int expected, int desired);

    void b2AtomicStoreU32(b2AtomicU32* a, uint value);
    uint b2AtomicLoadU32(b2AtomicU32* a);
#endif


#if defined( B2_COMPILER_MSVC )
#define _CRTDBG_MAP_ALLOC


#else

#endif




#ifdef BOX2D_PROFILE


#define b2TracyCAlloc( ptr, size ) TracyCAlloc( ptr, size )
#define b2TracyCFree( ptr ) TracyCFree( ptr )

#else

#define b2TracyCAlloc( ptr, size )
#define b2TracyCFree( ptr )

#endif



// This allows the user to change the length units at runtime
    float b2_lengthUnitsPerMeter = 1.0f;

    void b2SetLengthUnitsPerMeter(float lengthUnits)
    {
        B2_ASSERT(b2IsValidFloat(lengthUnits) && lengthUnits > 0.0f);
        b2_lengthUnitsPerMeter = lengthUnits;
    }

    float b2GetLengthUnitsPerMeter()
    {
        return b2_lengthUnitsPerMeter;
    }

    static int b2DefaultAssertFcn( const char* condition,  const char* fileName,  int lineNumber )
    {
        printf("BOX2D ASSERTION: %s, %s, line %d\n", condition, fileName, lineNumber);

        // return non-zero to break to debugger
        return 1;
    }

    b2AssertFcn* b2AssertHandler = b2DefaultAssertFcn;

    void b2SetAssertFcn(b2AssertFcn* assertFcn)
    {
        B2_ASSERT(assertFcn != NULL);
        b2AssertHandler = assertFcn;
    }

#if !defined( NDEBUG ) || defined( B2_ENABLE_ASSERT )
    int b2InternalAssertFcn( const char* condition,  const char* fileName,  int lineNumber )
    {
        return b2AssertHandler(condition, fileName, lineNumber);
    }
#endif

    b2Version b2GetVersion()
    {
        return (b2Version){
            3, 1, 0
        }
        ;
    }

#if 0
#if defined( _MSC_VER )

#endif

    void b2AtomicStoreInt(b2AtomicInt* a, int value)
    {
#if defined( _MSC_VER )
	(void)_InterlockedExchange( (long*)&a->value, value );
#elif defined( __GNUC__ ) || defined( __clang__ )
	__atomic_store_n( &a->value, value, __ATOMIC_SEQ_CST );
#else
#error "Unsupported platform"
#endif
    }

    int b2AtomicLoadInt(b2AtomicInt* a)
    {
#if defined( _MSC_VER )
	return _InterlockedOr( (long*)&a->value, 0 );
#elif defined( __GNUC__ ) || defined( __clang__ )
	return __atomic_load_n( &a->value, __ATOMIC_SEQ_CST );
#else
#error "Unsupported platform"
#endif
    }

    int b2AtomicFetchAddInt(b2AtomicInt* a, int increment)
    {
#if defined( _MSC_VER )
	return _InterlockedExchangeAdd( (long*)&a->value, (long)increment );
#elif defined( __GNUC__ ) || defined( __clang__ )
	return __atomic_fetch_add( &a->value, increment, __ATOMIC_SEQ_CST );
#else
#error "Unsupported platform"
#endif
    }

    bool b2AtomicCompareExchangeInt(b2AtomicInt* a, int expected, int desired)
    {
#if defined( _MSC_VER )
	return _InterlockedCompareExchange( (long*)&a->value, (long)desired, (long)expected ) == expected;
#elif defined( __GNUC__ ) || defined( __clang__ )
	// The value written to expected is ignored
	return __atomic_compare_exchange_n( &a->value, &expected, desired, false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST );
#else
#error "Unsupported platform"
#endif
    }

    void b2AtomicStoreU32(b2AtomicU32* a, uint value)
    {
#if defined( _MSC_VER )
	(void)_InterlockedExchange( (long*)&a->value, value );
#elif defined( __GNUC__ ) || defined( __clang__ )
	__atomic_store_n( &a->value, value, __ATOMIC_SEQ_CST );
#else
#error "Unsupported platform"
#endif
    }

    uint b2AtomicLoadU32(b2AtomicU32* a)
    {
#if defined( _MSC_VER )
	return (uint)_InterlockedOr( (long*)&a->value, 0 );
#elif defined( __GNUC__ ) || defined( __clang__ )
	return __atomic_load_n( &a->value, __ATOMIC_SEQ_CST );
#else
#error "Unsupported platform"
#endif
    }
#endif

    static b2AllocFcn* b2_allocFcn = NULL;
    static b2FreeFcn* b2_freeFcn = NULL;

    b2AtomicInt b2_byteCount;

    void b2SetAllocator(b2AllocFcn* allocFcn, b2FreeFcn* freeFcn)
    {
        b2_allocFcn = allocFcn;
        b2_freeFcn = freeFcn;
    }

// Use 32 byte alignment for everything. Works with 256bit SIMD.
#define B2_ALIGNMENT 32

    void* b2Alloc(int size)
    {
        if (size == 0)
        {
            return NULL;
        }

        // This could cause some sharing issues, however Box2D rarely calls b2Alloc.
        b2AtomicFetchAddInt(&b2_byteCount, size);

        // Allocation must be a multiple of 32 or risk a seg fault
        // https://en.cppreference.com/w/c/memory/aligned_alloc
        int size32 = ((size - 1) | 0x1F) + 1;

        if (b2_allocFcn != NULL)
        {
            void* ptr = b2_allocFcn(size32, B2_ALIGNMENT);
            b2TracyCAlloc(ptr, size);

            B2_ASSERT(ptr != NULL);
            B2_ASSERT(((uintptr_t)ptr & 0x1F) == 0);

            return ptr;
        }

#ifdef B2_PLATFORM_WINDOWS
        void* ptr = _aligned_malloc(size32, B2_ALIGNMENT);
#elif defined( B2_PLATFORM_ANDROID )
        void* ptr = NULL;
        if (posix_memalign(&ptr, B2_ALIGNMENT, size32) != 0)
        {
            // allocation failed, exit the application
            exit(EXIT_FAILURE);
        }

#else
        void* ptr = aligned_alloc(B2_ALIGNMENT, size32);
#endif

        b2TracyCAlloc(ptr, size);

        B2_ASSERT(ptr != NULL);
        B2_ASSERT(((uintptr_t)ptr & 0x1F) == 0);

        return ptr;
    }

    void b2Free(void* mem, int size)
    {
        if (mem == NULL)
        {
            return;
        }

        b2TracyCFree(mem);

        if (b2_freeFcn != NULL)
        {
            b2_freeFcn(mem);
        }
        else
        {
#ifdef B2_PLATFORM_WINDOWS
            _aligned_free(mem);
#else
            free(mem);
#endif
        }

        b2AtomicFetchAddInt(&b2_byteCount, -size);
    }

    void* b2GrowAlloc(void* oldMem, int oldSize, int newSize)
    {
        B2_ASSERT(newSize > oldSize);
        void* newMem = b2Alloc(newSize);
        if (oldSize > 0)
        {
            memcpy(newMem, oldMem, oldSize);
            b2Free(oldMem, oldSize);
        }

        return newMem;
    }

    int b2GetByteCount()
    {
        return b2AtomicLoadInt(&b2_byteCount);
    }

}