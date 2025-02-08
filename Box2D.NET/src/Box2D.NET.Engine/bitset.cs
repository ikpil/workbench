// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

namespace Box2D.NET.Engine;


public class bitset
{




// Bit set provides fast operations on large arrays of bits.
    typedef struct b2BitSet
    {
        ulong* bits;
        uint blockCapacity;
        uint blockCount;
    }

    b2BitSet;

    b2BitSet b2CreateBitSet(uint bitCapacity);
    void b2DestroyBitSet(b2BitSet* bitSet);
    void b2SetBitCountAndClear(b2BitSet* bitSet, uint bitCount);
    void b2InPlaceUnion(b2BitSet* setA,  const b2BitSet* setB );
    void b2GrowBitSet(b2BitSet* bitSet, uint blockCount);

    static inline void b2SetBit(b2BitSet* bitSet, uint bitIndex)
    {
        uint blockIndex = bitIndex / 64;
        B2_ASSERT(blockIndex < bitSet->blockCount);
        bitSet->bits[blockIndex] |= ((ulong)1 << bitIndex % 64);
    }

    static inline void b2SetBitGrow(b2BitSet* bitSet, uint bitIndex)
    {
        uint blockIndex = bitIndex / 64;
        if (blockIndex >= bitSet->blockCount)
        {
            b2GrowBitSet(bitSet, blockIndex + 1);
        }

        bitSet->bits[blockIndex] |= ((ulong)1 << bitIndex % 64);
    }

    static inline void b2ClearBit(b2BitSet* bitSet, uint bitIndex)
    {
        uint blockIndex = bitIndex / 64;
        if (blockIndex >= bitSet->blockCount)
        {
            return;
        }

        bitSet->bits[blockIndex] &= ~((ulong)1 << bitIndex % 64);
    }

    static inline bool b2GetBit( const b2BitSet* bitSet, uint bitIndex )
    {
        uint blockIndex = bitIndex / 64;
        if (blockIndex >= bitSet->blockCount)
        {
            return false;
        }

        return (bitSet->bits[blockIndex] & ((ulong)1 << bitIndex % 64)) != 0;
    }

    static inline int b2GetBitSetBytes(b2BitSet* bitSet)
    {
        return bitSet->blockCapacity * sizeof(ulong);
    }




    b2BitSet b2CreateBitSet(uint bitCapacity)
    {
        b2BitSet bitSet = { 0 };

        bitSet.blockCapacity = (bitCapacity + sizeof(ulong) * 8 - 1) / (sizeof(ulong) * 8);
        bitSet.blockCount = 0;
        bitSet.bits = b2Alloc(bitSet.blockCapacity * sizeof(ulong));
        memset(bitSet.bits, 0, bitSet.blockCapacity * sizeof(ulong));
        return bitSet;
    }

    void b2DestroyBitSet(b2BitSet* bitSet)
    {
        b2Free(bitSet->bits, bitSet->blockCapacity * sizeof(ulong));
        bitSet->blockCapacity = 0;
        bitSet->blockCount = 0;
        bitSet->bits = NULL;
    }

    void b2SetBitCountAndClear(b2BitSet* bitSet, uint bitCount)
    {
        uint blockCount = (bitCount + sizeof(ulong) * 8 - 1) / (sizeof(ulong) * 8);
        if (bitSet->blockCapacity < blockCount)
        {
            b2DestroyBitSet(bitSet);
            uint newBitCapacity = bitCount + (bitCount >> 1);
            *bitSet = b2CreateBitSet(newBitCapacity);
        }

        bitSet->blockCount = blockCount;
        memset(bitSet->bits, 0, bitSet->blockCount * sizeof(ulong));
    }

    void b2GrowBitSet(b2BitSet* bitSet, uint blockCount)
    {
        B2_ASSERT(blockCount > bitSet->blockCount);
        if (blockCount > bitSet->blockCapacity)
        {
            uint oldCapacity = bitSet->blockCapacity;
            bitSet->blockCapacity = blockCount + blockCount / 2;
            ulong* newBits = b2Alloc(bitSet->blockCapacity * sizeof(ulong));
            memset(newBits, 0, bitSet->blockCapacity * sizeof(ulong));
            B2_ASSERT(bitSet->bits != NULL);
            memcpy(newBits, bitSet->bits, oldCapacity * sizeof(ulong));
            b2Free(bitSet->bits, oldCapacity * sizeof(ulong));
            bitSet->bits = newBits;
        }

        bitSet->blockCount = blockCount;
    }

    void b2InPlaceUnion(b2BitSet* B2_RESTRICT setA, const b2BitSet* B2_RESTRICT setB )
    {
        B2_ASSERT(setA->blockCount == setB->blockCount);
        uint blockCount = setA->blockCount;
        for (uint i = 0; i < blockCount; ++i)
        {
            setA->bits[i] |= setB->bits[i];
        }
    }

}