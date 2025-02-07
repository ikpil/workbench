// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

namespace Box2D.NET.Engine
{
    /// World id references a world instance. This should be treated as an opaque handle.
    public struct b2WorldId
    {
        public ushort index1;
        public ushort generation;
    }

    /// Body id references a body instance. This should be treated as an opaque handle.
    public struct b2BodyId
    {
        public int index1;
        public ushort world0;
        public ushort generation;
    }

    /// Shape id references a shape instance. This should be treated as an opaque handle.
    public struct b2ShapeId
    {
        public int index1;
        public ushort world0;
        public ushort generation;
    }

    /// Chain id references a chain instances. This should be treated as an opaque handle.
    public struct b2ChainId
    {
        public int index1;
        public ushort world0;
        public ushort generation;
    }

    /// Joint id references a joint instance. This should be treated as an opaque handle.
    public struct b2JointId
    {
        public int index1;
        public ushort world0;
        public ushort generation;
    }

    public static class IdHelper
    {
        /// Store a body id into a ulong.
        public static ulong b2StoreBodyId(b2BodyId id)
        {
            return ((ulong)id.index1 << 32) | ((ulong)id.world0) << 16 | (ulong)id.generation;
        }

        /// Load a ulong into a body id.
        public static b2BodyId b2LoadBodyId(ulong x)
        {
            return new b2BodyId
            {
                index1 = (int)(x >> 32),
                world0 = (ushort)(x >> 16),
                generation = (ushort)(x),
            };
        }

        /// Store a shape id into a ulong.
        public static ulong b2StoreShapeId(b2ShapeId id)
        {
            return ((ulong)id.index1 << 32) | ((ulong)id.world0) << 16 | (ulong)id.generation;
        }

        /// Load a ulong into a shape id.
        public static b2ShapeId b2LoadShapeId(ulong x)
        {
            return new b2ShapeId
            {
                index1 = (int)(x >> 32),
                world0 = (ushort)(x >> 16),
                generation = (ushort)(x)
            };
        }

        /// Store a chain id into a ulong.
        public static ulong b2StoreChainId(b2ChainId id)
        {
            return ((ulong)id.index1 << 32) | ((ulong)id.world0) << 16 | (ulong)id.generation;
        }

        /// Load a ulong into a chain id.
        public static b2ChainId b2LoadChainId(ulong x)
        {
            return new b2ChainId
            {
                index1 = (int)(x >> 32),
                world0 = (ushort)(x >> 16),
                generation = (ushort)(x)
            };
        }

        /// Store a joint id into a ulong.
        public static ulong b2StoreJointId(b2JointId id)
        {
            return ((ulong)id.index1 << 32) | ((ulong)id.world0) << 16 | (ulong)id.generation;
        }

        /// Load a ulong into a joint id.
        public static b2JointId b2LoadJointId(ulong x)
        {
            return new b2JointId
            {
                index1 = (int)(x >> 32),
                world0 = (ushort)(x >> 16),
                generation = (ushort)(x)
            };
        }
    }
}