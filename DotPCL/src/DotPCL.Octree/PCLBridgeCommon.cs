using System;

namespace DotPCL.Octree
{
    public static class PCLBridgeCommon
    {
        public static void PCL_ERROR(string message, params object[] args)
        {
            Console.WriteLine(message, args);
        }
    }
}