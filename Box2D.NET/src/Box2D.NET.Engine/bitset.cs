// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

namespace Box2D.NET.Engine;






// Bit set provides fast operations on large arrays of bits.
typedef struct b2BitSet
{
    uint64_t* bits;
    uint32_t blockCapacity;
    uint32_t blockCount;
} b2BitSet;

b2BitSet b2CreateBitSet( uint32_t bitCapacity );
void b2DestroyBitSet( b2BitSet* bitSet );
void b2SetBitCountAndClear( b2BitSet* bitSet, uint32_t bitCount );
void b2InPlaceUnion( b2BitSet* setA, const b2BitSet* setB );
void b2GrowBitSet( b2BitSet* bitSet, uint32_t blockCount );

static inline void b2SetBit( b2BitSet* bitSet, uint32_t bitIndex )
{
    uint32_t blockIndex = bitIndex / 64;
    B2_ASSERT( blockIndex < bitSet->blockCount );
    bitSet->bits[blockIndex] |= ( (uint64_t)1 << bitIndex % 64 );
}

static inline void b2SetBitGrow( b2BitSet* bitSet, uint32_t bitIndex )
{
    uint32_t blockIndex = bitIndex / 64;
    if ( blockIndex >= bitSet->blockCount )
    {
        b2GrowBitSet( bitSet, blockIndex + 1 );
    }
    bitSet->bits[blockIndex] |= ( (uint64_t)1 << bitIndex % 64 );
}

static inline void b2ClearBit( b2BitSet* bitSet, uint32_t bitIndex )
{
    uint32_t blockIndex = bitIndex / 64;
    if ( blockIndex >= bitSet->blockCount )
    {
        return;
    }
    bitSet->bits[blockIndex] &= ~( (uint64_t)1 << bitIndex % 64 );
}

static inline bool b2GetBit( const b2BitSet* bitSet, uint32_t bitIndex )
{
    uint32_t blockIndex = bitIndex / 64;
    if ( blockIndex >= bitSet->blockCount )
    {
        return false;
    }
    return ( bitSet->bits[blockIndex] & ( (uint64_t)1 << bitIndex % 64 ) ) != 0;
}

static inline int b2GetBitSetBytes( b2BitSet* bitSet )
{
    return bitSet->blockCapacity * sizeof( uint64_t );
}




b2BitSet b2CreateBitSet( uint32_t bitCapacity )
{
	b2BitSet bitSet = { 0 };

	bitSet.blockCapacity = ( bitCapacity + sizeof( uint64_t ) * 8 - 1 ) / ( sizeof( uint64_t ) * 8 );
	bitSet.blockCount = 0;
	bitSet.bits = b2Alloc( bitSet.blockCapacity * sizeof( uint64_t ) );
	memset( bitSet.bits, 0, bitSet.blockCapacity * sizeof( uint64_t ) );
	return bitSet;
}

void b2DestroyBitSet( b2BitSet* bitSet )
{
	b2Free( bitSet->bits, bitSet->blockCapacity * sizeof( uint64_t ) );
	bitSet->blockCapacity = 0;
	bitSet->blockCount = 0;
	bitSet->bits = NULL;
}

void b2SetBitCountAndClear( b2BitSet* bitSet, uint32_t bitCount )
{
	uint32_t blockCount = ( bitCount + sizeof( uint64_t ) * 8 - 1 ) / ( sizeof( uint64_t ) * 8 );
	if ( bitSet->blockCapacity < blockCount )
	{
		b2DestroyBitSet( bitSet );
		uint32_t newBitCapacity = bitCount + ( bitCount >> 1 );
		*bitSet = b2CreateBitSet( newBitCapacity );
	}

	bitSet->blockCount = blockCount;
	memset( bitSet->bits, 0, bitSet->blockCount * sizeof( uint64_t ) );
}

void b2GrowBitSet( b2BitSet* bitSet, uint32_t blockCount )
{
	B2_ASSERT( blockCount > bitSet->blockCount );
	if ( blockCount > bitSet->blockCapacity )
	{
		uint32_t oldCapacity = bitSet->blockCapacity;
		bitSet->blockCapacity = blockCount + blockCount / 2;
		uint64_t* newBits = b2Alloc( bitSet->blockCapacity * sizeof( uint64_t ) );
		memset( newBits, 0, bitSet->blockCapacity * sizeof( uint64_t ) );
		B2_ASSERT( bitSet->bits != NULL );
		memcpy( newBits, bitSet->bits, oldCapacity * sizeof( uint64_t ) );
		b2Free( bitSet->bits, oldCapacity * sizeof( uint64_t ) );
		bitSet->bits = newBits;
	}

	bitSet->blockCount = blockCount;
}

void b2InPlaceUnion( b2BitSet* B2_RESTRICT setA, const b2BitSet* B2_RESTRICT setB )
{
	B2_ASSERT( setA->blockCount == setB->blockCount );
	uint32_t blockCount = setA->blockCount;
	for ( uint32_t i = 0; i < blockCount; ++i )
	{
		setA->bits[i] |= setB->bits[i];
	}
}
