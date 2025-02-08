// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

namespace Box2D.NET.Engine;

public class ctz
{




#if defined( _MSC_VER ) && !defined( __clang__ )
	

// https://en.wikipedia.org/wiki/Find_first_set

static uint b2CTZ32( uint block )
{
	unsigned long index;
	_BitScanForward( &index, block );
	return index;
}

// This function doesn't need to be fast, so using the Ivy Bridge fallback.
static uint b2CLZ32( uint value )
{
	#if 1

	// Use BSR (Bit Scan Reverse) which is available on Ivy Bridge
	unsigned long index;
	if ( _BitScanReverse( &index, value ) )
	{
		// BSR gives the index of the most significant 1-bit
		// We need to invert this to get the number of leading zeros
		return 31 - index;
	}
	else
	{
		// If x is 0, BSR sets the zero flag and doesn't modify index
		// LZCNT should return 32 for an input of 0
		return 32;
	}

	#else

	return __lzcnt( value );

	#endif
}

static uint b2CTZ64( ulong block )
{
	unsigned long index;

	#ifdef _WIN64
	_BitScanForward64( &index, block );
	#else
	// 32-bit fall back
	if ( (uint)block != 0 )
	{
		_BitScanForward( &index, (uint)block );
	}
	else
	{
		_BitScanForward( &index, (uint)( block >> 32 ) );
		index += 32;
	}
	#endif

	return index;
}

#else

static uint b2CTZ32( uint block )
{
	return __builtin_ctz( block );
}

static uint b2CLZ32( uint value )
{
	return __builtin_clz( value );
}

static uint b2CTZ64( ulong block )
{
	return __builtin_ctzll( block );
}

#endif

static bool b2IsPowerOf2( int x )
{
	return ( x & ( x - 1 ) ) == 0;
}

static int b2BoundingPowerOf2( int x )
{
	if ( x <= 1 )
	{
		return 1;
	}

	return 32 - (int)b2CLZ32( (uint)x - 1 );
}

static int b2RoundUpPowerOf2( int x )
{
	if ( x <= 1 )
	{
		return 1;
	}

	return 1 << ( 32 - (int)b2CLZ32( (uint)x - 1 ) );
}

}