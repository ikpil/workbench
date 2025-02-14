// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using static Box2D.NET.Engine.table;
using static Box2D.NET.Engine.array;
using static Box2D.NET.Engine.atomic;
using static Box2D.NET.Engine.dynamic_tree;
using static Box2D.NET.Engine.core;
using static Box2D.NET.Engine.types;
using static Box2D.NET.Engine.constants;
using static Box2D.NET.Engine.contact;
using static Box2D.NET.Engine.math_function;
using static Box2D.NET.Engine.constants;
using static Box2D.NET.Engine.array;


namespace Box2D.NET.Engine;

public class b2ArenaEntry
{
    public byte[] data;
    public string name;
    public int size;
    public bool usedMalloc;
}


// This is a stack-like arena allocator used for fast per step allocations.
// You must nest allocate/free pairs. The code will Debug.Assert
// if you try to interleave multiple allocate/free pairs.
// This allocator uses the heap if space is insufficient.
// I could remove the need to free entries individually.
public class b2ArenaAllocator
{
    public byte[] data;
    public int capacity;
    public int index;

    public int allocation;
    public int maxAllocation;

    public b2Array<b2ArenaEntry> entries;
}

public class arena_allocator
{


    b2ArenaAllocator;

    b2ArenaAllocator b2CreateArenaAllocator(int capacity);
    void b2DestroyArenaAllocator(b2ArenaAllocator* allocator);

    void* b2AllocateArenaItem(b2ArenaAllocator* alloc, int size,  const char* name );
    void b2FreeArenaItem(b2ArenaAllocator* alloc, void* mem);

// Grow the arena based on usage
    void b2GrowArena(b2ArenaAllocator* alloc);

    int b2GetArenaCapacity(b2ArenaAllocator* alloc);
    int b2GetArenaAllocation(b2ArenaAllocator* alloc);
    int b2GetMaxArenaAllocation(b2ArenaAllocator* alloc);

    B2_ARRAY_INLINE(b2ArenaEntry, b2ArenaEntry);


    B2_ARRAY_SOURCE(b2ArenaEntry, b2ArenaEntry);

    b2ArenaAllocator b2CreateArenaAllocator(int capacity)
    {
        Debug.Assert(capacity >= 0);
        b2ArenaAllocator allocator = { 0 };
        allocator.capacity = capacity;
        allocator.data = b2Alloc(capacity);
        allocator.allocation = 0;
        allocator.maxAllocation = 0;
        allocator.index = 0;
        allocator.entries = Array_Create<b2ArenaEntry>(32);
        return allocator;
    }

    void b2DestroyArenaAllocator(b2ArenaAllocator* allocator)
    {
        Array_Destroy(&allocator->entries);
        b2Free(allocator->data, allocator->capacity);
    }

    public static T[] b2AllocateArenaItem<T>(b2ArenaAllocator alloc, int size,  string name )
    {
        // ensure allocation is 32 byte aligned to support 256-bit SIMD
        int size32 = ((size - 1) | 0x1F) + 1;

        b2ArenaEntry entry;
        entry.size = size32;
        entry.name = name;
        if (alloc->index + size32 > alloc->capacity)
        {
            // fall back to the heap (undesirable)
            entry.data = b2Alloc(size32);
            entry.usedMalloc = true;

            Debug.Assert(((uintptr_t)entry.data & 0x1F) == 0);
        }
        else
        {
            entry.data = alloc->data + alloc->index;
            entry.usedMalloc = false;
            alloc->index += size32;

            Debug.Assert(((uintptr_t)entry.data & 0x1F) == 0);
        }

        alloc->allocation += size32;
        if (alloc->allocation > alloc->maxAllocation)
        {
            alloc->maxAllocation = alloc->allocation;
        }

        Array_Push(&alloc->entries, entry);
        return entry.data;
    }

    public static void b2FreeArenaItem<T>(b2ArenaAllocator alloc, T[] mem)
    {
        int entryCount = alloc->entries.count;
        Debug.Assert(entryCount > 0);
        b2ArenaEntry* entry = alloc->entries.data + (entryCount - 1);
        Debug.Assert(mem == entry->data);
        if (entry->usedMalloc)
        {
            b2Free(mem, entry->size);
        }
        else
        {
            alloc->index -= entry->size;
        }

        alloc->allocation -= entry->size;
        b2ArenaEntryArray_Pop(&alloc->entries);
    }

    void b2GrowArena(b2ArenaAllocator* alloc)
    {
        // Stack must not be in use
        Debug.Assert(alloc->allocation == 0);

        if (alloc->maxAllocation > alloc->capacity)
        {
            b2Free(alloc->data, alloc->capacity);
            alloc->capacity = alloc->maxAllocation + alloc->maxAllocation / 2;
            alloc->data = b2Alloc(alloc->capacity);
        }
    }

    int b2GetArenaCapacity(b2ArenaAllocator* alloc)
    {
        return alloc->capacity;
    }

    int b2GetArenaAllocation(b2ArenaAllocator* alloc)
    {
        return alloc->allocation;
    }

    int b2GetMaxArenaAllocation(b2ArenaAllocator* alloc)
    {
        return alloc->maxAllocation;
    }
}