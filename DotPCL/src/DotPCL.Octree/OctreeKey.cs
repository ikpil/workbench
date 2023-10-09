/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */


namespace DotPCL.Octree
{
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Octree key class
     *  \note Octree keys contain integer indices for each coordinate axis in order to
     * address an octree leaf node.
     * \author Julius Kammerl (julius@kammerl.de)
     */
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public class OctreeKey
    {
        /* \brief maximum depth that can be addressed */
        public const byte maxDepth = sizeof(uint) * 8;

        // Indices addressing a voxel at (X, Y, Z)
        // NOLINTBEGIN(modernize-use-default-member-init)
        public uint x;
        public uint y;
        public uint z;
        // NOLINTEND(modernize-use-default-member-init)

        /** \brief Empty constructor. */
        public OctreeKey()
        {
        }

        /** \brief Constructor for key initialization. */
        public OctreeKey(uint keyX, uint keyY, uint keyZ)
        {
            x = keyX;
            y = keyY;
            z = keyZ;
        }

        /** \brief Copy constructor. */
        public OctreeKey(OctreeKey source) : this(source.x, source.y, source.z)
        {
        }

        /** \brief Operator== for comparing octree keys with each other.
         *  \return "true" if leaf node indices are identical; "false" otherwise.
         * */
        public static bool operator ==(OctreeKey a, OctreeKey b)
        {
            return (b.x == a.x) && (b.y == a.y) && (b.z == a.z);
        }

        /** \brief Inequal comparison operator
         * \param[in] other OctreeIteratorBase to compare with
         * \return "true" if the current and other iterators are different ; "false"
         * otherwise.
         */
        public static bool operator !=(OctreeKey a, OctreeKey b)
        {
            return !(a == b);
        }

        /** \brief Operator<= for comparing octree keys with each other.
         *  \return "true" if key indices are not greater than the key indices of b  ; "false"
         * otherwise.
         * */
        public static bool operator <=(OctreeKey a, OctreeKey b)
        {
            return ((b.x >= a.x) && (b.y >= a.y) && (b.z >= a.z));
        }

        /** \brief Operator>= for comparing octree keys with each other.
         *  \return "true" if key indices are not smaller than the key indices of b  ; "false"
         * otherwise.
         * */
        public static bool operator >=(OctreeKey a, OctreeKey b)
        {
            return ((b.x <= a.x) && (b.y <= a.y) && (b.z <= a.z));
        }

        /** \brief push a child node to the octree key
         *  \param[in] childIndex index of child node to be added (0-7)
         * */
        public void pushBranch(byte childIndex)
        {
            x = (x << 1) | ((childIndex & (1 << 2)) != 0 ? 1 : (uint)0);
            y = (y << 1) | ((childIndex & (1 << 1)) != 0 ? 1 : (uint)0);
            z = (z << 1) | ((childIndex & (1 << 0)) != 0 ? 1 : (uint)0);
        }

        /** \brief pop child node from octree key
         * */
        public void popBranch()
        {
            x >>= 1;
            y >>= 1;
            z >>= 1;
        }

        /** \brief get child node index using depthMask
         *  \param[in] depthMask bit mask with single bit set at query depth
         *  \return child node index
         * */
        public byte getChildIdxWithDepthMask(uint depthMask)
        {
            var bit = ((x & depthMask) != 0 ? 1 << 2 : 0) |
                      ((y & depthMask) != 0 ? 1 << 1 : 0) |
                      ((z & depthMask) != 0 ? 1 : 0);
            return (byte)bit;
        }
    }
}