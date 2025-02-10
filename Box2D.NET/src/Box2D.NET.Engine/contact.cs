// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

namespace Box2D.NET.Engine;

// A contact edge is used to connect bodies and contacts together
// in a contact graph where each body is a node and each contact
// is an edge. A contact edge belongs to a doubly linked list
// maintained in each attached body. Each contact has two contact
// edges, one for each attached body.

public enum b2ContactFlags
{
    // Set when the solid shapes are touching.
    b2_contactTouchingFlag = 0x00000001,

    // Contact has a hit event
    b2_contactHitEventFlag = 0x00000002,

    // This contact wants contact events
    b2_contactEnableContactEvents = 0x00000004,
}

public class b2ContactEdge
{
    public int bodyId;
    public int prevKey;
    public int nextKey;
}

// Cold contact data. Used as a persistent handle and for persistent island
// connectivity.
public class b2Contact
{
    // index of simulation set stored in b2World
    // B2_NULL_INDEX when slot is free
    public int setIndex;

    // index into the constraint graph color array
    // B2_NULL_INDEX for non-touching or sleeping contacts
    // B2_NULL_INDEX when slot is free
    public int colorIndex;

    // contact index within set or graph color
    // B2_NULL_INDEX when slot is free
    public int localIndex;

    public b2ContactEdge[] edges = new b2ContactEdge[2];
    public int shapeIdA;
    public int shapeIdB;

    // A contact only belongs to an island if touching, otherwise B2_NULL_INDEX.
    public int islandPrev;
    public int islandNext;
    public int islandId;

    public int contactId;

    // b2ContactFlags
    public uint flags;

    public bool isMarked;
};

// Shifted to be distinct from b2ContactFlags
public enum b2ContactSimFlags
{
    // Set when the shapes are touching, including sensors
    b2_simTouchingFlag = 0x00010000,

    // This contact no longer has overlapping AABBs
    b2_simDisjoint = 0x00020000,

    // This contact started touching
    b2_simStartedTouching = 0x00040000,

    // This contact stopped touching
    b2_simStoppedTouching = 0x00080000,

    // This contact has a hit event
    b2_simEnableHitEvent = 0x00100000,

    // This contact wants pre-solve events
    b2_simEnablePreSolveEvents = 0x00200000,
}

/// The class manages contact between two shapes. A contact exists for each overlapping
/// AABB in the broad-phase (except if filtered). Therefore a contact object may exist
/// that has no contact points.
public class b2ContactSim
{
    public int contactId;

#if B2_VALIDATE
	public int bodyIdA;
	public int bodyIdB;
#endif

    public int bodySimIndexA;
    public int bodySimIndexB;

    public int shapeIdA;
    public int shapeIdB;

    public float invMassA;
    public float invIA;

    public float invMassB;
    public float invIB;

    public b2Manifold manifold;

    // Mixed friction and restitution
    public float friction;
    public float restitution;
    public float rollingResistance;
    public float tangentSpeed;

    // b2ContactSimFlags
    public uint simFlags;

    public b2SimplexCache cache;
}

public class contact
{
    public static bool b2ShouldShapesCollide( b2Filter filterA, b2Filter filterB )
    {
        if ( filterA.groupIndex == filterB.groupIndex && filterA.groupIndex != 0 )
        {
            return filterA.groupIndex > 0;
        }

        bool collide = ( filterA.maskBits & filterB.categoryBits ) != 0 && ( filterA.categoryBits & filterB.maskBits ) != 0;
        return collide;
    } 
}