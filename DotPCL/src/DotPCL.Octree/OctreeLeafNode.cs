namespace DotPCL.Octree
{
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Abstract octree leaf class
     * \note Octree leaves may collect data of type ContainerT
     * \author Julius Kammerl (julius@kammerl.de)
     */
    public class OctreeLeafNode<ContainerT> : OctreeNode
    {
        protected ContainerT container_;

        /** \brief Empty constructor. */
        public OctreeLeafNode()
        {
            container_ = default;
        }

        /** \brief Copy constructor. */
        public OctreeLeafNode(OctreeLeafNode<ContainerT> source)
        {
            container_ = source.container_;
        }

        /** \brief Method to perform a deep copy of the octree */
        public override OctreeNode deepCopy()
        {
            return new OctreeLeafNode<ContainerT>(this);
        }

        /** \brief Get the type of octree node. Returns LEAVE_NODE type */
        public override node_type_t getNodeType()
        {
            return node_type_t.LEAF_NODE;
        }

        /** \brief Get reference to container */
        public ref ContainerT getContainer()
        {
            return ref container_;
        }

        // // Type ContainerT may have fixed-size Eigen objects inside
        // PCL_MAKE_ALIGNED_OPERATOR_NEW
    }
}