namespace DotPCL.Octree
{
    public enum node_type_t
    {
        BRANCH_NODE,
        LEAF_NODE
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Abstract octree node class
     * \note Every octree node should implement the getNodeType () method
     * \author Julius Kammerl (julius@kammerl.de)
     */
    public abstract class OctreeNode
    {
        /** \brief Pure virtual method for retrieving the type of octree node (branch or leaf)
         */
        public abstract node_type_t getNodeType();

        /** \brief Pure virtual method to perform a deep copy of the octree */
        public abstract OctreeNode deepCopy();

    }
}