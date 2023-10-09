namespace DotPCL.Octree
{
    public interface IOctreeBranchNodeContainer
    {
        void reset();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Abstract octree branch class
     * \note Octree branch classes may collect data of type DataT
     * \author Julius Kammerl (julius@kammerl.de)
     */
    public class OctreeBranchNode<ContainerT> : OctreeNode where ContainerT : IOctreeBranchNodeContainer
    {
        protected OctreeNode[] child_node_array_ = new OctreeNode[8];
        protected ContainerT container_;

//     public:
        /** \brief Empty constructor. */
        public OctreeBranchNode()
        {
        }

        /** \brief Copy constructor. */
        public OctreeBranchNode(OctreeBranchNode<ContainerT> source)
        {
            for (int i = 0; i < 8; ++i)
            {
                if (null != source.child_node_array_[i])
                {
                    child_node_array_[i] = source.child_node_array_[i].deepCopy();
                }
            }
        }

        /** \brief Copy operator. */
        public void CopyFrom(OctreeBranchNode<ContainerT> source)
        {
            for (int i = 0; i < 8; ++i) 
            {
                if (null != source.child_node_array_[i])
                {
                    child_node_array_[i] = source.child_node_array_[i].deepCopy();
                }
                else
                {
                    child_node_array_[i] = null;
                }
            }
        }

        /** \brief Octree deep copy method */
        public override OctreeNode deepCopy()
        {
            return new OctreeBranchNode<ContainerT>(this);
        }

//   /** \brief Access operator.
//    *  \param child_idx_arg: index to child node, must be less than 8
//    *  \return OctreeNode pointer
//    * */
//   inline OctreeNode*&
//   operator[](unsigned char child_idx_arg)
//   {
//     assert(child_idx_arg < 8);
//     return child_node_array_[child_idx_arg];
//   }
//
//   /** \brief Get pointer to child
//    *  \param child_idx_arg: index to child node, must be less than 8
//    *  \return OctreeNode pointer
//    * */
//   inline OctreeNode*
//   getChildPtr(unsigned char child_idx_arg) const
//   {
//     assert(child_idx_arg < 8);
//     return child_node_array_[child_idx_arg];
//   }
//
//   /** \brief Get pointer to child
//    *  \param index: index to child node, must be less than 8
//    *  \return OctreeNode pointer
//    * */
//   inline void
//   setChildPtr(OctreeNode* child, unsigned char index)
//   {
//     assert(index < 8);
//     child_node_array_[index] = child;
//   }
//
//   /** \brief Check if branch is pointing to a particular child node
//    *  \param child_idx_arg: index to child node, must be less than 8
//    *  \return "true" if pointer to child node exists; "false" otherwise
//    * */
//   inline bool
//   hasChild(unsigned char child_idx_arg) const
//   {
//     return (child_node_array_[child_idx_arg] != nullptr);
//   }
//
//   /** \brief Check if branch can be pruned
//    *  \note if all children are leaf nodes AND contain identical containers, branch can
//    * be pruned
//    * \return "true" if branch can be pruned; "false" otherwise
//    **/
//   /*    inline bool isPrunable () const
//       {
//         const OctreeNode* firstChild = child_node_array_[0];
//         if (!firstChild || firstChild->getNodeType()==BRANCH_NODE)
//           return false;
//
//         bool prunable = true;
//         for (unsigned char i = 1; i < 8 && prunable; ++i)
//         {
//           const OctreeNode* child = child_node_array_[i];
//           if ( (!child) ||
//                (child->getNodeType()==BRANCH_NODE) ||
//                ((*static_cast<const OctreeContainerBase*>(child)) == (*static_cast<const
//      OctreeContainerBase*>(child)) ) ) prunable = false;
//         }
//
//         return prunable;
//       }*/
//
        /** \brief Get the type of octree node. Returns LEAVE_NODE type */
        public override node_type_t getNodeType()
        {
            return node_type_t.BRANCH_NODE;
        }

        // reset node
        public void reset()
        {
            for (int i = 0; i < child_node_array_.Length; ++i)
            {
                child_node_array_[i] = null;
            }
            
            container_.reset();
        }
//
//   /** \brief Get const pointer to container */
//   const ContainerT*
//   operator->() const
//   {
//     return &container_;
//   }
//
//   /** \brief Get pointer to container */
//   ContainerT*
//   operator->()
//   {
//     return &container_;
//   }
//
//   /** \brief Get const reference to container */
//   const ContainerT&
//   operator*() const
//   {
//     return container_;
//   }
//
//   /** \brief Get reference to container */
//   ContainerT&
//   operator*()
//   {
//     return container_;
//   }
//
//   /** \brief Get const reference to container */
//   const ContainerT&
//   getContainer() const
//   {
//     return container_;
//   }
//
//   /** \brief Get reference to container */
//   ContainerT&
//   getContainer()
//   {
//     return container_;
//   }
//
//   /** \brief Get const pointer to container */
//   const ContainerT*
//   getContainerPtr() const
//   {
//     return &container_;
//   }
//
//   /** \brief Get pointer to container */
//   ContainerT*
//   getContainerPtr()
//   {
//     return &container_;
//   }
    }
}