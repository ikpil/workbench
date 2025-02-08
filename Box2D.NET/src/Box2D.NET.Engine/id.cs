// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

namespace Box2D.NET.Engine;


public class id
{



    /**
     * @defgroup id Ids
     * These ids serve as handles to internal Box2D objects.
     * These should be considered opaque data and passed by value.
     * Include this header if you need the id types and not the whole Box2D API.
     * All ids are considered null if initialized to zero.
     *
     * For example in C++:
     *
     * @code{.cxx}
     * b2WorldId worldId = {};
     * @endcode
     *
     * Or in C:
     *
     * @code{.c}
     * b2WorldId worldId = {0};
     * @endcode
     *
     * These are both considered null.
     *
     * @warning Do not use the internals of these ids. They are subject to change. Ids should be treated as opaque objects.
     * @warning You should use ids to access objects in Box2D. Do not access files within the src folder. Such usage is unsupported.
     * @{
     */

    /// World id references a world instance. This should be treated as an opaque handle.
    typedef struct b2WorldId
    {
        ushort index1;
        ushort generation;
    }

    b2WorldId;

    /// Body id references a body instance. This should be treated as an opaque handle.
    typedef struct b2BodyId
    {
        int index1;
        ushort world0;
        ushort generation;
    }

    b2BodyId;

    /// Shape id references a shape instance. This should be treated as an opaque handle.
    typedef struct b2ShapeId
    {
        int index1;
        ushort world0;
        ushort generation;
    }

    b2ShapeId;

    /// Chain id references a chain instances. This should be treated as an opaque handle.
    typedef struct b2ChainId
    {
        int index1;
        ushort world0;
        ushort generation;
    }

    b2ChainId;

    /// Joint id references a joint instance. This should be treated as an opaque handle.
    typedef struct b2JointId
    {
        int index1;
        ushort world0;
        ushort generation;
    }

    b2JointId;

    /// Use these to make your identifiers null.
    /// You may also use zero initialization to get null.
    static const b2WorldId b2_nullWorldId = B2_ZERO_INIT;

    static const b2BodyId b2_nullBodyId = B2_ZERO_INIT;
    static const b2ShapeId b2_nullShapeId = B2_ZERO_INIT;
    static const b2ChainId b2_nullChainId = B2_ZERO_INIT;
    static const b2JointId b2_nullJointId = B2_ZERO_INIT;

    /// Macro to determine if any id is null.
#define B2_IS_NULL( id ) ( id.index1 == 0 )

    /// Macro to determine if any id is non-null.
#define B2_IS_NON_NULL( id ) ( id.index1 != 0 )

    /// Compare two ids for equality. Doesn't work for b2WorldId.
#define B2_ID_EQUALS( id1, id2 ) ( id1.index1 == id2.index1 && id1.world0 == id2.world0 && id1.generation == id2.generation )

    /// Store a body id into a ulong.
    ulong b2StoreBodyId(b2BodyId id)
    {
        return ((ulong)id.index1 << 32) | ((ulong)id.world0) << 16 | (ulong)id.generation;
    }

    /// Load a ulong into a body id.
    b2BodyId b2LoadBodyId(ulong x)
    {
        b2BodyId id = { (int)(x >> 32), (ushort)(x >> 16), (ushort)(x) };
        return id;
    }

    /// Store a shape id into a ulong.
    ulong b2StoreShapeId(b2ShapeId id)
    {
        return ((ulong)id.index1 << 32) | ((ulong)id.world0) << 16 | (ulong)id.generation;
    }

    /// Load a ulong into a shape id.
    b2ShapeId b2LoadShapeId(ulong x)
    {
        b2ShapeId id = { (int)(x >> 32), (ushort)(x >> 16), (ushort)(x) };
        return id;
    }

    /// Store a chain id into a ulong.
    ulong b2StoreChainId(b2ChainId id)
    {
        return ((ulong)id.index1 << 32) | ((ulong)id.world0) << 16 | (ulong)id.generation;
    }

    /// Load a ulong into a chain id.
    b2ChainId b2LoadChainId(ulong x)
    {
        b2ChainId id = { (int)(x >> 32), (ushort)(x >> 16), (ushort)(x) };
        return id;
    }

    /// Store a joint id into a ulong.
    ulong b2StoreJointId(b2JointId id)
    {
        return ((ulong)id.index1 << 32) | ((ulong)id.world0) << 16 | (ulong)id.generation;
    }

    /// Load a ulong into a joint id.
    b2JointId b2LoadJointId(ulong x)
    {
        b2JointId id = { (int)(x >> 32), (ushort)(x >> 16), (ushort)(x) };
        return id;
    }

    /**@}*/

}