namespace Box2D.NET.Engine;

public enum b2SetType
{
    b2_staticSet = 0,
    b2_disabledSet = 1,
    b2_awakeSet = 2,
    b2_firstSleepingSet = 3,
}

// Per thread task storage
public class b2TaskContext
{
    // These bits align with the b2ConstraintGraph::contactBlocks and signal a change in contact status
    public b2BitSet contactStateBitSet;

    // Used to track bodies with shapes that have enlarged AABBs. This avoids having a bit array
    // that is very large when there are many static shapes.
    public b2BitSet enlargedSimBitSet;

    // Used to put islands to sleep
    public b2BitSet awakeIslandBitSet;

    // Per worker split island candidate
    public float splitSleepTime;
    public int splitIslandId;

}

// temp
public class b2World
{
    public b2BroadPhase broadPhase;
	public b2Array<b2Shape> shapes;
}
