// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

using System;
using System.Diagnostics;
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
using static Box2D.NET.Engine.id;
using static Box2D.NET.Engine.shape;
using static Box2D.NET.Engine.solver;
using static Box2D.NET.Engine.body;
using static Box2D.NET.Engine.world;
using static Box2D.NET.Engine.joint;
using static Box2D.NET.Engine.distance_joint;
using static Box2D.NET.Engine.motor_joint;
using static Box2D.NET.Engine.mouse_joint;
using static Box2D.NET.Engine.prismatic_joint;
using static Box2D.NET.Engine.revolute_joint;
using static Box2D.NET.Engine.weld_joint;
using static Box2D.NET.Engine.wheel_joint;
using static Box2D.NET.Engine.id_pool;
using static Box2D.NET.Engine.manifold;


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

// The world struct manages all physics entities, dynamic simulation,  and asynchronous queries.
// The world also contains efficient memory management facilities.
public class b2World
{
	public b2ArenaAllocator stackAllocator;
	public b2BroadPhase broadPhase;
	public b2ConstraintGraph constraintGraph;

	// The body id pool is used to allocate and recycle body ids. Body ids
	// provide a stable identifier for users, but incur caches misses when used
	// to access body data. Aligns with b2Body.
    public b2IdPool bodyIdPool;

	// This is a sparse array that maps body ids to the body data
	// stored in solver sets. As sims move within a set or across set.
	// Indices come from id pool.
    public b2Array<b2Body> bodies;

	// Provides free list for solver sets.
    public b2IdPool solverSetIdPool;

	// Solvers sets allow sims to be stored in contiguous arrays. The first
	// set is all static sims. The second set is active sims. The third set is disabled
	// sims. The remaining sets are sleeping islands.
    public b2Array<b2SolverSet> solverSets;

	// Used to create stable ids for joints
    public b2IdPool jointIdPool;

	// This is a sparse array that maps joint ids to the joint data stored in the constraint graph
	// or in the solver sets.
    public b2Array<b2Joint> joints;

	// Used to create stable ids for contacts
    public b2IdPool contactIdPool;

	// This is a sparse array that maps contact ids to the contact data stored in the constraint graph
	// or in the solver sets.
    public b2Array<b2Contact> contacts;

	// Used to create stable ids for islands
    public b2IdPool islandIdPool;

	// This is a sparse array that maps island ids to the island data stored in the solver sets.
    public b2Array<b2Island> islands;

    public b2IdPool shapeIdPool;
    public b2IdPool chainIdPool;

	// These are sparse arrays that point into the pools above
    public b2Array<b2Shape> shapes;
    public b2Array<b2ChainShape> chainShapes;

	// This is a dense array of sensor data.
    public b2Array<b2Sensor> sensors;

	// Per thread storage
	public b2Array<b2TaskContext> taskContexts;
	public b2Array<b2SensorTaskContext> sensorTaskContexts;

	public b2Array<b2BodyMoveEvent> bodyMoveEvents;
	public b2Array<b2SensorBeginTouchEvent> sensorBeginEvents;
	public b2Array<b2ContactBeginTouchEvent> contactBeginEvents;

	// End events are double buffered so that the user doesn't need to flush events
	public b2Array<b2SensorEndTouchEvent>[] sensorEndEvents = new b2Array<b2SensorEndTouchEvent>[2];
	public b2Array<b2ContactEndTouchEvent>[] contactEndEvents = new b2Array<b2ContactEndTouchEvent>[2];
	public int endEventArrayIndex;

    public b2Array<b2ContactHitEvent> contactHitEvents;

	// Used to track debug draw
	public b2BitSet debugBodySet;
	public b2BitSet debugJointSet;
	public b2BitSet debugContactSet;

	// Id that is incremented every time step
    public ulong stepIndex;

	// Identify islands for splitting as follows:
	// - I want to split islands so smaller islands can sleep
	// - when a body comes to rest and its sleep timer trips, I can look at the island and flag it for splitting
	//   if it has removed constraints
	// - islands that have removed constraints must be put split first because I don't want to wake bodies incorrectly
	// - otherwise I can use the awake islands that have bodies wanting to sleep as the splitting candidates
	// - if no bodies want to sleep then there is no reason to perform island splitting
    public int splitIslandId;

	public b2Vec2 gravity;
	public float hitEventThreshold;
	public float restitutionThreshold;
	public float maxLinearSpeed;
	public float contactMaxPushSpeed;
	public float contactHertz;
	public float contactDampingRatio;
	public float jointHertz;
	public float jointDampingRatio;

	public b2FrictionCallback frictionCallback;
    public b2RestitutionCallback restitutionCallback;

    public ushort generation;

    public b2Profile profile;

    public b2PreSolveFcn preSolveFcn;
    public object preSolveContext;

    public b2CustomFilterFcn customFilterFcn;
    public object customFilterContext;

	public int workerCount;
	public b2EnqueueTaskCallback enqueueTaskFcn;
	public b2FinishTaskCallback finishTaskFcn;
	public object userTaskContext;
	public object userTreeTask;

    public object userData;

	// Remember type step used for reporting forces and torques
    public float inv_h;

	public int activeTaskCount;
	public int taskCount;

    public ushort worldId;

	public bool enableSleep;
	public bool locked;
	public bool enableWarmStarting;
	public bool enableContinuous;
	public bool enableSpeculative;
	public bool inUse;
}


public class world
{
    //Debug.Assert( B2_MAX_WORLDS > 0, "must be 1 or more" );
    public static b2World[] b2_worlds = new b2World[B2_MAX_WORLDS];


public static b2World b2GetWorldFromId( b2WorldId id )
{
	Debug.Assert( 1 <= id.index1 && id.index1 <= B2_MAX_WORLDS );
    b2World world = b2_worlds[id.index1 - 1];
	Debug.Assert( id.index1 == world.worldId + 1 );
	Debug.Assert( id.generation == world.generation );
	return world;
}

public static b2World b2GetWorld( int index )
{
	Debug.Assert( 0 <= index && index < B2_MAX_WORLDS );
    b2World world = b2_worlds[index];
	Debug.Assert( world.worldId == index );
	return world;
}

public static b2World b2GetWorldLocked( int index )
{
	Debug.Assert( 0 <= index && index < B2_MAX_WORLDS );
    b2World world = b2_worlds[index];
	Debug.Assert( world.worldId == index );
	if ( world.locked )
	{
		Debug.Assert( false );
        return null;
    }

	return world;
}

public static object b2DefaultAddTaskFcn( b2TaskCallback task, int count, int minRange, object taskContext, object userContext )
{
	B2_UNUSED( minRange, userContext );
	task( 0, count, 0, taskContext );
	return null;
}

public static void b2DefaultFinishTaskFcn( object userTask, object userContext )
{
	B2_UNUSED( userTask, userContext );
}

public static float b2DefaultFrictionCallback( float frictionA, int materialA, float frictionB, int materialB )
{
	B2_UNUSED( materialA, materialB );
	return MathF.Sqrt( frictionA * frictionB );
}

public static float b2DefaultRestitutionCallback( float restitutionA, int materialA, float restitutionB, int materialB )
{
	B2_UNUSED( materialA, materialB );
	return b2MaxFloat( restitutionA, restitutionB );
}

public static b2WorldId b2CreateWorld( b2WorldDef def )
{
	Debug.Assert( B2_MAX_WORLDS < ushort.MaxValue, "B2_MAX_WORLDS limit exceeded" );
	B2_CHECK_DEF( def );

	int worldId = B2_NULL_INDEX;
	for ( int i = 0; i < B2_MAX_WORLDS; ++i )
	{
		if ( b2_worlds[i].inUse == false )
		{
			worldId = i;
			break;
		}
	}

	if ( worldId == B2_NULL_INDEX )
    {
        return new b2WorldId(0, 0);
    }

	b2InitializeContactRegisters();

	b2World* world = b2_worlds + worldId;
	ushort generation = world.generation;

	*world = ( b2World ){ 0 };

	world.worldId = (ushort)worldId;
	world.generation = generation;
	world.inUse = true;

	world.stackAllocator = b2CreateArenaAllocator( 2048 );
	b2CreateBroadPhase( &world.broadPhase );
	b2CreateGraph( &world.constraintGraph, 16 );

	// pools
	world.bodyIdPool = b2CreateIdPool();
	world.bodies = Array_Create<b2Body>( 16 );
	world.solverSets = Array_Create<b2SolverSet>( 8 );

	// add empty static, active, and disabled body sets
	world.solverSetIdPool = b2CreateIdPool();
	b2SolverSet set = { 0 };

	// static set
	set.setIndex = b2AllocId( &world.solverSetIdPool );
	Array_Push( &world.solverSets, set );
	Debug.Assert( world.solverSets.data[b2_staticSet].setIndex == (int)b2SetType.b2_staticSet );

	// disabled set
	set.setIndex = b2AllocId( &world.solverSetIdPool );
	Array_Push( &world.solverSets, set );
	Debug.Assert( world.solverSets.data[b2_disabledSet].setIndex == (int)b2SetType.b2_disabledSet );

	// awake set
	set.setIndex = b2AllocId( &world.solverSetIdPool );
	Array_Push( &world.solverSets, set );
	Debug.Assert( world.solverSets.data[b2_awakeSet].setIndex == (int)b2SetType.b2_awakeSet );

	world.shapeIdPool = b2CreateIdPool();
	world.shapes = Array_Create<b2Shape>( 16 );

	world.chainIdPool = b2CreateIdPool();
	world.chainShapes = Array_Create<b2ChainShape>( 4 );

	world.contactIdPool = b2CreateIdPool();
	world.contacts = Array_Create<b2Contact>( 16 );

	world.jointIdPool = b2CreateIdPool();
	world.joints = Array_Create<b2Joint>( 16 );

	world.islandIdPool = b2CreateIdPool();
	world.islands = Array_Create<b2Island>( 8 );

	world.sensors = Array_Create<b2Sensor>( 4 );

	world.bodyMoveEvents = Array_Create<b2BodyMoveEvent>( 4 );
	world.sensorBeginEvents = Array_Create<b2SensorBeginTouchEvent>( 4 );
	world.sensorEndEvents[0] = Array_Create<b2SensorEndTouchEvent>( 4 );
	world.sensorEndEvents[1] = Array_Create<b2SensorEndTouchEvent>( 4 );
	world.contactBeginEvents = Array_Create<b2ContactBeginTouchEvent>( 4 );
	world.contactEndEvents[0] = Array_Create<b2ContactEndTouchEvent>( 4 );
	world.contactEndEvents[1] = Array_Create<b2ContactEndTouchEvent>( 4 );
	world.contactHitEvents = Array_Create<b2ContactHitEvent>( 4 );
	world.endEventArrayIndex = 0;

	world.stepIndex = 0;
	world.splitIslandId = B2_NULL_INDEX;
	world.activeTaskCount = 0;
	world.taskCount = 0;
	world.gravity = def.gravity;
	world.hitEventThreshold = def.hitEventThreshold;
	world.restitutionThreshold = def.restitutionThreshold;
	world.maxLinearSpeed = def.maximumLinearSpeed;
	world.contactMaxPushSpeed = def.contactPushMaxSpeed;
	world.contactHertz = def.contactHertz;
	world.contactDampingRatio = def.contactDampingRatio;
	world.jointHertz = def.jointHertz;
	world.jointDampingRatio = def.jointDampingRatio;

	if ( def.frictionCallback == NULL )
	{
		world.frictionCallback = b2DefaultFrictionCallback;
	}
	else
	{
		world.frictionCallback = def.frictionCallback;
	}

	if ( def.restitutionCallback == NULL )
	{
		world.restitutionCallback = b2DefaultRestitutionCallback;
	}
	else
	{
		world.restitutionCallback = def.restitutionCallback;
	}

	world.enableSleep = def.enableSleep;
	world.locked = false;
	world.enableWarmStarting = true;
	world.enableContinuous = def.enableContinuous;
	world.enableSpeculative = true;
	world.userTreeTask = NULL;
	world.userData = def.userData;

	if ( def.workerCount > 0 && def.enqueueTask != NULL && def.finishTask != NULL )
	{
		world.workerCount = b2MinInt( def.workerCount, B2_MAX_WORKERS );
		world.enqueueTaskFcn = def.enqueueTask;
		world.finishTaskFcn = def.finishTask;
		world.userTaskContext = def.userTaskContext;
	}
	else
	{
		world.workerCount = 1;
		world.enqueueTaskFcn = b2DefaultAddTaskFcn;
		world.finishTaskFcn = b2DefaultFinishTaskFcn;
		world.userTaskContext = NULL;
	}

	world.taskContexts = Array_Create<b2TaskContext>( world.workerCount );
	b2TaskContextArray_Resize( &world.taskContexts, world.workerCount );

	world.sensorTaskContexts = Array_Create<b2SensorTaskContext>( world.workerCount );
	b2SensorTaskContextArray_Resize( &world.sensorTaskContexts, world.workerCount );

	for ( int i = 0; i < world.workerCount; ++i )
	{
		world.taskContexts.data[i].contactStateBitSet = b2CreateBitSet( 1024 );
		world.taskContexts.data[i].enlargedSimBitSet = b2CreateBitSet( 256 );
		world.taskContexts.data[i].awakeIslandBitSet = b2CreateBitSet( 256 );

		world.sensorTaskContexts.data[i].eventBits = b2CreateBitSet( 128 );
	}

	world.debugBodySet = b2CreateBitSet( 256 );
	world.debugJointSet = b2CreateBitSet( 256 );
	world.debugContactSet = b2CreateBitSet( 256 );

	// add one to worldId so that 0 represents a null b2WorldId
	return ( b2WorldId ){ (ushort)( worldId + 1 ), world.generation };
}

void b2DestroyWorld( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );

	b2DestroyBitSet( &world.debugBodySet );
	b2DestroyBitSet( &world.debugJointSet );
	b2DestroyBitSet( &world.debugContactSet );

	for ( int i = 0; i < world.workerCount; ++i )
	{
		b2DestroyBitSet( &world.taskContexts.data[i].contactStateBitSet );
		b2DestroyBitSet( &world.taskContexts.data[i].enlargedSimBitSet );
		b2DestroyBitSet( &world.taskContexts.data[i].awakeIslandBitSet );

		b2DestroyBitSet( &world.sensorTaskContexts.data[i].eventBits );
	}

	Array_Destroy( &world.taskContexts );
	Array_Destroy( &world.sensorTaskContexts );

	Array_Destroy( &world.bodyMoveEvents );
	Array_Destroy( &world.sensorBeginEvents );
	Array_Destroy( world.sensorEndEvents + 0 );
	Array_Destroy( world.sensorEndEvents + 1 );
	Array_Destroy( &world.contactBeginEvents );
	Array_Destroy( world.contactEndEvents + 0 );
	Array_Destroy( world.contactEndEvents + 1 );
	Array_Destroy( &world.contactHitEvents );

	int chainCapacity = world.chainShapes.count;
	for ( int i = 0; i < chainCapacity; ++i )
	{
		b2ChainShape* chain = world.chainShapes.data + i;
		if ( chain.id != B2_NULL_INDEX )
		{
			b2FreeChainData( chain );
		}
		else
		{
			Debug.Assert( chain.shapeIndices == NULL );
			Debug.Assert( chain.materials == NULL );
		}
	}

	int sensorCount = world.sensors.count;
	for ( int i = 0; i < sensorCount; ++i )
	{
		Array_Destroy( &world.sensors.data[i].overlaps1 );
		Array_Destroy( &world.sensors.data[i].overlaps2 );
	}

	Array_Destroy( &world.sensors );

	Array_Destroy( &world.bodies );
	Array_Destroy( &world.shapes );
	Array_Destroy( &world.chainShapes );
	Array_Destroy( &world.contacts );
	Array_Destroy( &world.joints );
	Array_Destroy( &world.islands );

	// Destroy solver sets
	int setCapacity = world.solverSets.count;
	for ( int i = 0; i < setCapacity; ++i )
	{
		b2SolverSet* set = world.solverSets.data + i;
		if ( set.setIndex != B2_NULL_INDEX )
		{
			b2DestroySolverSet( world, i );
		}
	}

	Array_Destroy( &world.solverSets );

	b2DestroyGraph( &world.constraintGraph );
	b2DestroyBroadPhase( &world.broadPhase );

	b2DestroyIdPool( &world.bodyIdPool );
	b2DestroyIdPool( &world.shapeIdPool );
	b2DestroyIdPool( &world.chainIdPool );
	b2DestroyIdPool( &world.contactIdPool );
	b2DestroyIdPool( &world.jointIdPool );
	b2DestroyIdPool( &world.islandIdPool );
	b2DestroyIdPool( &world.solverSetIdPool );

	b2DestroyArenaAllocator( &world.stackAllocator );

	// Wipe world but preserve generation
	ushort generation = world.generation;
	*world = ( b2World ){ 0 };
	world.worldId = 0;
	world.generation = generation + 1;
}

static void b2CollideTask( int startIndex, int endIndex, uint threadIndex, void* context )
{
	b2TracyCZoneNC(b2TracyCZone.collide_task, "Collide", b2_colorDodgerBlue, true );

	b2StepContext* stepContext = context;
	b2World* world = stepContext.world;
	Debug.Assert( (int)threadIndex < world.workerCount );
	b2TaskContext* taskContext = world.taskContexts.data + threadIndex;
	b2ContactSim** contactSims = stepContext.contacts;
	b2Shape* shapes = world.shapes.data;
	b2Body* bodies = world.bodies.data;

	Debug.Assert( startIndex < endIndex );

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b2ContactSim* contactSim = contactSims[i];

		int contactId = contactSim.contactId;

		b2Shape* shapeA = shapes + contactSim.shapeIdA;
		b2Shape* shapeB = shapes + contactSim.shapeIdB;

		// Do proxies still overlap?
		bool overlap = b2AABB_Overlaps( shapeA.fatAABB, shapeB.fatAABB );
		if ( overlap == false )
		{
			contactSim.simFlags |= b2_simDisjoint;
			contactSim.simFlags &= ~b2ContactSimFlags.b2_simTouchingFlag;
			b2SetBit( &taskContext.contactStateBitSet, contactId );
		}
		else
		{
			bool wasTouching = ( contactSim.simFlags & b2ContactSimFlags.b2_simTouchingFlag );

			// Update contact respecting shape/body order (A,B)
			b2Body* bodyA = bodies + shapeA.bodyId;
			b2Body* bodyB = bodies + shapeB.bodyId;
			b2BodySim* bodySimA = b2GetBodySim( world, bodyA );
			b2BodySim* bodySimB = b2GetBodySim( world, bodyB );

			// avoid cache misses in b2PrepareContactsTask
			contactSim.bodySimIndexA = bodyA.setIndex == (int)b2SetType.b2_awakeSet ? bodyA.localIndex : B2_NULL_INDEX;
			contactSim.invMassA = bodySimA.invMass;
			contactSim.invIA = bodySimA.invInertia;

			contactSim.bodySimIndexB = bodyB.setIndex == (int)b2SetType.b2_awakeSet ? bodyB.localIndex : B2_NULL_INDEX;
			contactSim.invMassB = bodySimB.invMass;
			contactSim.invIB = bodySimB.invInertia;

			b2Transform transformA = bodySimA.transform;
			b2Transform transformB = bodySimB.transform;

			b2Vec2 centerOffsetA = b2RotateVector( transformA.q, bodySimA.localCenter );
			b2Vec2 centerOffsetB = b2RotateVector( transformB.q, bodySimB.localCenter );

			// This updates solid contacts and sensors
			bool touching =
				b2UpdateContact( world, contactSim, shapeA, transformA, centerOffsetA, shapeB, transformB, centerOffsetB );

			// State changes that affect island connectivity. Also affects contact and sensor events.
			if ( touching == true && wasTouching == false )
			{
				contactSim.simFlags |= b2_simStartedTouching;
				b2SetBit( &taskContext.contactStateBitSet, contactId );
			}
			else if ( touching == false && wasTouching == true )
			{
				contactSim.simFlags |= b2_simStoppedTouching;
				b2SetBit( &taskContext.contactStateBitSet, contactId );
			}
		}
	}

	b2TracyCZoneEnd(b2TracyCZone.collide_task );
}

static void b2UpdateTreesTask( int startIndex, int endIndex, uint threadIndex, void* context )
{
	B2_UNUSED( startIndex );
	B2_UNUSED( endIndex );
	B2_UNUSED( threadIndex );

	b2TracyCZoneNC(b2TracyCZone.tree_task, "Rebuild BVH", b2_colorFireBrick, true );

	b2World* world = context;
	b2BroadPhase_RebuildTrees( &world.broadPhase );

	b2TracyCZoneEnd(b2TracyCZone.tree_task );
}

static void b2AddNonTouchingContact( b2World* world, b2Contact* contact, b2ContactSim* contactSim )
{
	Debug.Assert( contact.setIndex == (int)b2SetType.b2_awakeSet );
	b2SolverSet* set = Array_Get( &world.solverSets, (int)b2SetType.b2_awakeSet );
	contact.colorIndex = B2_NULL_INDEX;
	contact.localIndex = set.contactSims.count;

	b2ContactSim* newContactSim = Array_Add( &set.contactSims );
	memcpy( newContactSim, contactSim, sizeof( b2ContactSim ) );
}

static void b2RemoveNonTouchingContact( b2World* world, int setIndex, int localIndex )
{
	b2SolverSet* set = Array_Get( &world.solverSets, setIndex );
	int movedIndex = Array_RemoveSwap( &set.contactSims, localIndex );
	if ( movedIndex != B2_NULL_INDEX )
	{
		b2ContactSim* movedContactSim = set.contactSims.data + localIndex;
		b2Contact* movedContact = Array_Get( &world.contacts, movedContactSim.contactId );
		Debug.Assert( movedContact.setIndex == setIndex );
		Debug.Assert( movedContact.localIndex == movedIndex );
		Debug.Assert( movedContact.colorIndex == B2_NULL_INDEX );
		movedContact.localIndex = localIndex;
	}
}

// Narrow-phase collision
static void b2Collide( b2StepContext* context )
{
	b2World* world = context.world;

	Debug.Assert( world.workerCount > 0 );

	b2TracyCZoneNC(b2TracyCZone.collide, "Narrow Phase", b2_colorDodgerBlue, true );

	// Task that can be done in parallel with the narrow-phase
	// - rebuild the collision tree for dynamic and kinematic bodies to keep their query performance good
	// todo_erin move this to start when contacts are being created
	world.userTreeTask = world.enqueueTaskFcn( &b2UpdateTreesTask, 1, 1, world, world.userTaskContext );
	world.taskCount += 1;
	world.activeTaskCount += world.userTreeTask == NULL ? 0 : 1;

	// gather contacts into a single array for easier parallel-for
	int contactCount = 0;
	b2GraphColor* graphColors = world.constraintGraph.colors;
	for ( int i = 0; i < B2_GRAPH_COLOR_COUNT; ++i )
	{
		contactCount += graphColors[i].contactSims.count;
	}

	int nonTouchingCount = world.solverSets.data[b2_awakeSet].contactSims.count;
	contactCount += nonTouchingCount;

	if ( contactCount == 0 )
	{
		b2TracyCZoneEnd(b2TracyCZone.collide );
		return;
	}

	b2ContactSim** contactSims =
		b2AllocateArenaItem( &world.stackAllocator, contactCount * sizeof( b2ContactSim* ), "contacts" );

	int contactIndex = 0;
	for ( int i = 0; i < B2_GRAPH_COLOR_COUNT; ++i )
	{
		b2GraphColor* color = graphColors + i;
		int count = color.contactSims.count;
		b2ContactSim* @base = color.contactSims.data;
		for ( int j = 0; j < count; ++j )
		{
			contactSims[contactIndex] = @base + j;
			contactIndex += 1;
		}
	}

	{
		b2ContactSim* @base = world.solverSets.data[b2_awakeSet].contactSims.data;
		for ( int i = 0; i < nonTouchingCount; ++i )
		{
			contactSims[contactIndex] = @base + i;
			contactIndex += 1;
		}
	}

	Debug.Assert( contactIndex == contactCount );

	context.contacts = contactSims;

	// Contact bit set on ids because contact pointers are unstable as they move between touching and not touching.
	int contactIdCapacity = b2GetIdCapacity( &world.contactIdPool );
	for ( int i = 0; i < world.workerCount; ++i )
	{
		b2SetBitCountAndClear( &world.taskContexts.data[i].contactStateBitSet, contactIdCapacity );
	}

	// Task should take at least 40us on a 4GHz CPU (10K cycles)
	int minRange = 64;
	void* userCollideTask = world.enqueueTaskFcn( &b2CollideTask, contactCount, minRange, context, world.userTaskContext );
	world.taskCount += 1;
	if ( userCollideTask != NULL )
	{
		world.finishTaskFcn( userCollideTask, world.userTaskContext );
	}

	b2FreeArenaItem( &world.stackAllocator, contactSims );
	context.contacts = NULL;
	contactSims = NULL;

	// Serially update contact state
	// todo_erin bring this zone together with island merge
	b2TracyCZoneNC(b2TracyCZone.contact_state, "Contact State", b2_colorLightSlateGray, true );

	// Bitwise OR all contact bits
	b2BitSet bitSet = &world.taskContexts.data[0].contactStateBitSet;
	for ( int i = 1; i < world.workerCount; ++i )
	{
		b2InPlaceUnion( bitSet, &world.taskContexts.data[i].contactStateBitSet );
	}

	b2SolverSet* awakeSet = Array_Get( &world.solverSets, (int)b2SetType.b2_awakeSet );

	int endEventArrayIndex = world.endEventArrayIndex;

	const b2Shape* shapes = world.shapes.data;
	ushort worldId = world.worldId;

	// Process contact state changes. Iterate over set bits
	for ( uint k = 0; k < bitSet.blockCount; ++k )
	{
		ulong bits = bitSet.bits[k];
		while ( bits != 0 )
		{
			uint ctz = b2CTZ64( bits );
			int contactId = (int)( 64 * k + ctz );

			b2Contact* contact = Array_Get( &world.contacts, contactId );
			Debug.Assert( contact.setIndex == (int)b2SetType.b2_awakeSet );

			int colorIndex = contact.colorIndex;
			int localIndex = contact.localIndex;

			b2ContactSim* contactSim = NULL;
			if ( colorIndex != B2_NULL_INDEX )
			{
				// contact lives in constraint graph
				Debug.Assert( 0 <= colorIndex && colorIndex < B2_GRAPH_COLOR_COUNT );
				b2GraphColor* color = graphColors + colorIndex;
				contactSim = Array_Get( &color.contactSims, localIndex );
			}
			else
			{
				contactSim = Array_Get( &awakeSet.contactSims, localIndex );
			}

			const b2Shape* shapeA = shapes + contact.shapeIdA;
			const b2Shape* shapeB = shapes + contact.shapeIdB;
			b2ShapeId shapeIdA = { shapeA.id + 1, worldId, shapeA.generation };
			b2ShapeId shapeIdB = { shapeB.id + 1, worldId, shapeB.generation };
			uint flags = contact.flags;
			uint simFlags = contactSim.simFlags;

			if ( simFlags & b2_simDisjoint )
			{
				// Bounding boxes no longer overlap
				b2DestroyContact( world, contact, false );
				contact = NULL;
				contactSim = NULL;
			}
			else if ( simFlags & b2_simStartedTouching )
			{
				Debug.Assert( contact.islandId == B2_NULL_INDEX );
				// Contact is solid
				if ( flags & b2ContactFlags.b2_contactEnableContactEvents )
				{
					b2ContactBeginTouchEvent event = { shapeIdA, shapeIdB, contactSim.manifold };
					Array_Push( &world.contactBeginEvents, event );
				}

				Debug.Assert( contactSim.manifold.pointCount > 0 );
				Debug.Assert( contact.setIndex == (int)b2SetType.b2_awakeSet );

				// Link first because this wakes colliding bodies and ensures the body sims
				// are in the correct place.
				contact.flags |= b2ContactFlags.b2_contactTouchingFlag;
				b2LinkContact( world, contact );

				// Make sure these didn't change
				Debug.Assert( contact.colorIndex == B2_NULL_INDEX );
				Debug.Assert( contact.localIndex == localIndex );

				// Contact sim pointer may have become orphaned due to awake set growth,
				// so I just need to refresh it.
				contactSim = Array_Get( &awakeSet.contactSims, localIndex );

				contactSim.simFlags &= ~b2_simStartedTouching;

				b2AddContactToGraph( world, contactSim, contact );
				b2RemoveNonTouchingContact( world, (int)b2SetType.b2_awakeSet, localIndex );
				contactSim = NULL;
			}
			else if ( simFlags & b2_simStoppedTouching )
			{
				contactSim.simFlags &= ~b2_simStoppedTouching;

				// Contact is solid
				contact.flags &= ~b2ContactFlags.b2_contactTouchingFlag;

				if ( contact.flags & b2ContactFlags.b2_contactEnableContactEvents )
				{
					b2ContactEndTouchEvent event = { shapeIdA, shapeIdB };
					Array_Push( world.contactEndEvents + endEventArrayIndex, event );
				}

				Debug.Assert( contactSim.manifold.pointCount == 0 );

				b2UnlinkContact( world, contact );
				int bodyIdA = contact.edges[0].bodyId;
				int bodyIdB = contact.edges[1].bodyId;

				b2AddNonTouchingContact( world, contact, contactSim );
				b2RemoveContactFromGraph( world, bodyIdA, bodyIdB, colorIndex, localIndex );
				contact = NULL;
				contactSim = NULL;
			}

			// Clear the smallest set bit
			bits = bits & ( bits - 1 );
		}
	}

	b2ValidateSolverSets( world );
	b2ValidateContacts( world );

	b2TracyCZoneEnd(b2TracyCZone.contact_state );
	b2TracyCZoneEnd(b2TracyCZone.collide );
}

void b2World_Step( b2WorldId worldId, float timeStep, int subStepCount )
{
	b2World* world = b2GetWorldFromId( worldId );
	Debug.Assert( world.locked == false );
	if ( world.locked )
	{
		return;
	}

	// Prepare to capture events
	// Ensure user does not access stale data if there is an early return
	Array_Clear( &world.bodyMoveEvents );
	Array_Clear( &world.sensorBeginEvents );
	Array_Clear( &world.contactBeginEvents );
	Array_Clear( &world.contactHitEvents );

	world.profile = ( b2Profile ){ 0 };

	if ( timeStep == 0.0f )
	{
		// Swap end event array buffers
		world.endEventArrayIndex = 1 - world.endEventArrayIndex;
		Array_Clear( world.sensorEndEvents + world.endEventArrayIndex );
		Array_Clear( world.contactEndEvents + world.endEventArrayIndex );

		// todo_erin would be useful to still process collision while paused
		return;
	}

	b2TracyCZoneNC(b2TracyCZone.world_step, "Step", b2_colorBox2DGreen, true );

	world.locked = true;
	world.activeTaskCount = 0;
	world.taskCount = 0;

	ulong stepTicks = b2GetTicks();

	// Update collision pairs and create contacts
	{
		ulong pairTicks = b2GetTicks();
		b2UpdateBroadPhasePairs( world );
		world.profile.pairs = b2GetMilliseconds( pairTicks );
	}

	b2StepContext context = { 0 };
	context.world = world;
	context.dt = timeStep;
	context.subStepCount = b2MaxInt( 1, subStepCount );

	if ( timeStep > 0.0f )
	{
		context.inv_dt = 1.0f / timeStep;
		context.h = timeStep / context.subStepCount;
		context.inv_h = context.subStepCount * context.inv_dt;
	}
	else
	{
		context.inv_dt = 0.0f;
		context.h = 0.0f;
		context.inv_h = 0.0f;
	}

	world.inv_h = context.inv_h;

	// Hertz values get reduced for large time steps
	float contactHertz = b2MinFloat( world.contactHertz, 0.25f * context.inv_h );
	float jointHertz = b2MinFloat( world.jointHertz, 0.125f * context.inv_h );

	context.contactSoftness = b2MakeSoft( contactHertz, world.contactDampingRatio, context.h );
	context.staticSoftness = b2MakeSoft( 2.0f * contactHertz, world.contactDampingRatio, context.h );
	context.jointSoftness = b2MakeSoft( jointHertz, world.jointDampingRatio, context.h );

	context.restitutionThreshold = world.restitutionThreshold;
	context.maxLinearVelocity = world.maxLinearSpeed;
	context.enableWarmStarting = world.enableWarmStarting;

	// Update contacts
	{
		ulong collideTicks = b2GetTicks();
		b2Collide( &context );
		world.profile.collide = b2GetMilliseconds( collideTicks );
	}

	// Integrate velocities, solve velocity constraints, and integrate positions.
	if ( context.dt > 0.0f )
	{
		ulong solveTicks = b2GetTicks();
		b2Solve( world, &context );
		world.profile.solve = b2GetMilliseconds( solveTicks );
	}

	// Update sensors
	{
		ulong sensorTicks = b2GetTicks();
		b2OverlapSensors( world );
		world.profile.sensors = b2GetMilliseconds( sensorTicks );
	}

	world.profile.step = b2GetMilliseconds( stepTicks );

	Debug.Assert( b2GetArenaAllocation( &world.stackAllocator ) == 0 );

	// Ensure stack is large enough
	b2GrowArena( &world.stackAllocator );

	// Make sure all tasks that were started were also finished
	Debug.Assert( world.activeTaskCount == 0 );

	b2TracyCZoneEnd(b2TracyCZone.world_step );

	// Swap end event array buffers
	world.endEventArrayIndex = 1 - world.endEventArrayIndex;
	Array_Clear( world.sensorEndEvents + world.endEventArrayIndex );
	Array_Clear( world.contactEndEvents + world.endEventArrayIndex );
	world.locked = false;
}

static void b2DrawShape( b2DebugDraw* draw, b2Shape* shape, b2Transform xf, b2HexColor color )
{
	switch ( shape.type )
	{
		case b2ShapeType.b2_capsuleShape:
		{
			b2Capsule* capsule = &shape.capsule;
			b2Vec2 p1 = b2TransformPoint( xf, capsule.center1 );
			b2Vec2 p2 = b2TransformPoint( xf, capsule.center2 );
			draw.DrawSolidCapsule( p1, p2, capsule.radius, color, draw.context );
		}
		break;

		case b2ShapeType.b2_circleShape:
		{
			b2Circle* circle = &shape.circle;
			xf.p = b2TransformPoint( xf, circle.center );
			draw.DrawSolidCircle( xf, circle.radius, color, draw.context );
		}
		break;

		case b2ShapeType.b2_polygonShape:
		{
			b2Polygon* poly = &shape.polygon;
			draw.DrawSolidPolygon( xf, poly.vertices, poly.count, poly.radius, color, draw.context );
		}
		break;

		case b2ShapeType.b2_segmentShape:
		{
			b2Segment* segment = &shape.segment;
			b2Vec2 p1 = b2TransformPoint( xf, segment.point1 );
			b2Vec2 p2 = b2TransformPoint( xf, segment.point2 );
			draw.DrawSegment( p1, p2, color, draw.context );
		}
		break;

		case b2ShapeType.b2_chainSegmentShape:
		{
			b2Segment* segment = &shape.chainSegment.segment;
			b2Vec2 p1 = b2TransformPoint( xf, segment.point1 );
			b2Vec2 p2 = b2TransformPoint( xf, segment.point2 );
			draw.DrawSegment( p1, p2, color, draw.context );
			draw.DrawPoint( p2, 4.0f, color, draw.context );
			draw.DrawSegment( p1, b2Lerp( p1, p2, 0.1f ), b2_colorPaleGreen, draw.context );
		}
		break;

		default:
			break;
	}
}

struct DrawContext
{
	b2World* world;
	b2DebugDraw* draw;
};

static bool DrawQueryCallback( int proxyId, int shapeId, void* context )
{
	B2_UNUSED( proxyId );

	struct DrawContext* drawContext = context;
	b2World* world = drawContext.world;
	b2DebugDraw* draw = drawContext.draw;

	b2Shape* shape = Array_Get( &world.shapes, shapeId );
	Debug.Assert( shape.id == shapeId );

	b2SetBit( &world.debugBodySet, shape.bodyId );

	if ( draw.drawShapes )
	{
		b2Body* body = Array_Get( &world.bodies, shape.bodyId );
		b2BodySim* bodySim = b2GetBodySim( world, body );

		b2HexColor color;

		if ( shape.customColor != 0 )
		{
			color = shape.customColor;
		}
		else if ( body.type == b2BodyType.b2_dynamicBody && body.mass == 0.0f )
		{
			// Bad body
			color = b2_colorRed;
		}
		else if ( body.setIndex == (int)b2SetType.b2_disabledSet )
		{
			color = b2_colorSlateGray;
		}
		else if ( shape.sensorIndex != B2_NULL_INDEX )
		{
			color = b2_colorWheat;
		}
		else if ( bodySim.isBullet && body.setIndex == (int)b2SetType.b2_awakeSet )
		{
			color = b2_colorTurquoise;
		}
		else if ( body.isSpeedCapped )
		{
			color = b2_colorYellow;
		}
		else if ( bodySim.isFast )
		{
			color = b2_colorSalmon;
		}
		else if ( body.type == b2BodyType.b2_staticBody )
		{
			color = b2_colorPaleGreen;
		}
		else if ( body.type == b2BodyType.b2_kinematicBody )
		{
			color = b2_colorRoyalBlue;
		}
		else if ( body.setIndex == (int)b2SetType.b2_awakeSet )
		{
			color = b2_colorPink;
		}
		else
		{
			color = b2_colorGray;
		}

		b2DrawShape( draw, shape, bodySim.transform, color );
	}

	if ( draw.drawAABBs )
	{
		b2AABB aabb = shape.fatAABB;

		b2Vec2 vs[4] = { { aabb.lowerBound.x, aabb.lowerBound.y },
						 { aabb.upperBound.x, aabb.lowerBound.y },
						 { aabb.upperBound.x, aabb.upperBound.y },
						 { aabb.lowerBound.x, aabb.upperBound.y } };

		draw.DrawPolygon( vs, 4, b2_colorGold, draw.context );
	}

	return true;
}

// todo this has varying order for moving shapes, causing flicker when overlapping shapes are moving
// solution: display order by shape id modulus 3, keep 3 buckets in GLSolid* and flush in 3 passes.
static void b2DrawWithBounds( b2World* world, b2DebugDraw* draw )
{
	Debug.Assert( b2IsValidAABB( draw.drawingBounds ) );

	const float k_impulseScale = 1.0f;
	const float k_axisScale = 0.3f;
	b2HexColor speculativeColor = b2_colorGainsboro;
	b2HexColor addColor = b2_colorGreen;
	b2HexColor persistColor = b2_colorBlue;
	b2HexColor normalColor = b2_colorDimGray;
	b2HexColor impulseColor = b2_colorMagenta;
	b2HexColor frictionColor = b2_colorYellow;

	b2HexColor graphColors[B2_GRAPH_COLOR_COUNT] = { b2_colorRed,		b2_colorOrange,	   b2_colorYellow, b2_colorGreen,
													 b2_colorCyan,		b2_colorBlue,	   b2_colorViolet, b2_colorPink,
													 b2_colorChocolate, b2_colorGoldenRod, b2_colorCoral,  b2_colorBlack };

	int bodyCapacity = b2GetIdCapacity( &world.bodyIdPool );
	b2SetBitCountAndClear( &world.debugBodySet, bodyCapacity );

	int jointCapacity = b2GetIdCapacity( &world.jointIdPool );
	b2SetBitCountAndClear( &world.debugJointSet, jointCapacity );

	int contactCapacity = b2GetIdCapacity( &world.contactIdPool );
	b2SetBitCountAndClear( &world.debugContactSet, contactCapacity );

	struct DrawContext drawContext = { world, draw };

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2DynamicTree_Query( world.broadPhase.trees + i, draw.drawingBounds, B2_DEFAULT_MASK_BITS, DrawQueryCallback,
							 &drawContext );
	}

	uint wordCount = world.debugBodySet.blockCount;
	ulong[] bits = world.debugBodySet.bits;
	for ( uint k = 0; k < wordCount; ++k )
	{
		ulong word = bits[k];
		while ( word != 0 )
		{
			uint ctz = b2CTZ64( word );
			uint bodyId = 64 * k + ctz;

			b2Body* body = Array_Get( &world.bodies, bodyId );

			if ( draw.drawBodyNames && body.name[0] != 0 )
			{
				b2Vec2 offset = new b2Vec2( 0.1f, 0.1f );
				b2BodySim* bodySim = b2GetBodySim( world, body );

				b2Transform transform = { bodySim.center, bodySim.transform.q };
				draw.DrawTransform( transform, draw.context );

				b2Vec2 p = b2TransformPoint( transform, offset );

				draw.DrawString( p, body.name, b2_colorBlueViolet, draw.context );
			}

			if ( draw.drawMass && body.type == b2BodyType.b2_dynamicBody )
			{
				b2Vec2 offset = new b2Vec2( 0.1f, 0.1f );
				b2BodySim* bodySim = b2GetBodySim( world, body );

				b2Transform transform = { bodySim.center, bodySim.transform.q };
				draw.DrawTransform( transform, draw.context );

				b2Vec2 p = b2TransformPoint( transform, offset );

				char buffer[32];
				snprintf( buffer, 32, "  %.2f", body.mass );
				draw.DrawString( p, buffer, b2_colorWhite, draw.context );
			}

			if ( draw.drawJoints )
			{
				int jointKey = body.headJointKey;
				while ( jointKey != B2_NULL_INDEX )
				{
					int jointId = jointKey >> 1;
					int edgeIndex = jointKey & 1;
					b2Joint* joint = Array_Get( &world.joints, jointId );

					// avoid double draw
					if ( b2GetBit( &world.debugJointSet, jointId ) == false )
					{
						b2DrawJoint( draw, world, joint );
						b2SetBit( &world.debugJointSet, jointId );
					}
					else
					{
						// todo testing
						edgeIndex += 0;
					}

					jointKey = joint.edges[edgeIndex].nextKey;
				}
			}

			const float linearSlop = B2_LINEAR_SLOP;
			if ( draw.drawContacts && body.type == b2BodyType.b2_dynamicBody && body.setIndex == (int)b2SetType.b2_awakeSet )
			{
				int contactKey = body.headContactKey;
				while ( contactKey != B2_NULL_INDEX )
				{
					int contactId = contactKey >> 1;
					int edgeIndex = contactKey & 1;
					b2Contact* contact = Array_Get( &world.contacts, contactId );
					contactKey = contact.edges[edgeIndex].nextKey;

					if ( contact.setIndex != (int)b2SetType.b2_awakeSet || contact.colorIndex == B2_NULL_INDEX )
					{
						continue;
					}

					// avoid double draw
					if ( b2GetBit( &world.debugContactSet, contactId ) == false )
					{
						Debug.Assert( 0 <= contact.colorIndex && contact.colorIndex < B2_GRAPH_COLOR_COUNT );

						b2GraphColor* gc = world.constraintGraph.colors + contact.colorIndex;
						b2ContactSim* contactSim = Array_Get( &gc.contactSims, contact.localIndex );
						int pointCount = contactSim.manifold.pointCount;
						b2Vec2 normal = contactSim.manifold.normal;
						char buffer[32];

						for ( int j = 0; j < pointCount; ++j )
						{
							b2ManifoldPoint* point = contactSim.manifold.points + j;

							if ( draw.drawGraphColors )
							{
								// graph color
								float pointSize = contact.colorIndex == B2_OVERFLOW_INDEX ? 7.5f : 5.0f;
								draw.DrawPoint( point.point, pointSize, graphColors[contact.colorIndex], draw.context );
								// g_draw.DrawString(point.position, "%d", point.color);
							}
							else if ( point.separation > linearSlop )
							{
								// Speculative
								draw.DrawPoint( point.point, 5.0f, speculativeColor, draw.context );
							}
							else if ( point.persisted == false )
							{
								// Add
								draw.DrawPoint( point.point, 10.0f, addColor, draw.context );
							}
							else if ( point.persisted == true )
							{
								// Persist
								draw.DrawPoint( point.point, 5.0f, persistColor, draw.context );
							}

							if ( draw.drawContactNormals )
							{
								b2Vec2 p1 = point.point;
								b2Vec2 p2 = b2MulAdd( p1, k_axisScale, normal );
								draw.DrawSegment( p1, p2, normalColor, draw.context );
							}
							else if ( draw.drawContactImpulses )
							{
								b2Vec2 p1 = point.point;
								b2Vec2 p2 = b2MulAdd( p1, k_impulseScale * point.normalImpulse, normal );
								draw.DrawSegment( p1, p2, impulseColor, draw.context );
								snprintf( buffer, B2_ARRAY_COUNT( buffer ), "%.1f", 1000.0f * point.normalImpulse );
								draw.DrawString( p1, buffer, b2_colorWhite, draw.context );
							}

							if ( draw.drawFrictionImpulses )
							{
								b2Vec2 tangent = b2RightPerp( normal );
								b2Vec2 p1 = point.point;
								b2Vec2 p2 = b2MulAdd( p1, k_impulseScale * point.tangentImpulse, tangent );
								draw.DrawSegment( p1, p2, frictionColor, draw.context );
								snprintf( buffer, B2_ARRAY_COUNT( buffer ), "%.1f", 1000.0f * point.tangentImpulse );
								draw.DrawString( p1, buffer, b2_colorWhite, draw.context );
							}
						}

						b2SetBit( &world.debugContactSet, contactId );
					}
					else
					{
						// todo testing
						edgeIndex += 0;
					}

					contactKey = contact.edges[edgeIndex].nextKey;
				}
			}

			// Clear the smallest set bit
			word = word & ( word - 1 );
		}
	}
}

void b2World_Draw( b2WorldId worldId, b2DebugDraw* draw )
{
	b2World* world = b2GetWorldFromId( worldId );
	Debug.Assert( world.locked == false );
	if ( world.locked )
	{
		return;
	}

	// todo it seems bounds drawing is fast enough for regular usage
	if ( draw.useDrawingBounds )
	{
		b2DrawWithBounds( world, draw );
		return;
	}

	if ( draw.drawShapes )
	{
		int setCount = world.solverSets.count;
		for ( int setIndex = 0; setIndex < setCount; ++setIndex )
		{
			b2SolverSet* set = Array_Get( &world.solverSets, setIndex );
			int bodyCount = set.bodySims.count;
			for ( int bodyIndex = 0; bodyIndex < bodyCount; ++bodyIndex )
			{
				b2BodySim* bodySim = set.bodySims.data + bodyIndex;
				b2Body* body = Array_Get( &world.bodies, bodySim.bodyId );
				Debug.Assert( body.setIndex == setIndex );

				b2Transform xf = bodySim.transform;
				int shapeId = body.headShapeId;
				while ( shapeId != B2_NULL_INDEX )
				{
					b2Shape* shape = world.shapes.data + shapeId;
					b2HexColor color;

					if ( shape.customColor != 0 )
					{
						color = shape.customColor;
					}
					else if ( body.type == b2BodyType.b2_dynamicBody && body.mass == 0.0f )
					{
						// Bad body
						color = b2_colorRed;
					}
					else if ( body.setIndex == (int)b2SetType.b2_disabledSet )
					{
						color = b2_colorSlateGray;
					}
					else if ( shape.sensorIndex != B2_NULL_INDEX )
					{
						color = b2_colorWheat;
					}
					else if ( bodySim.isBullet && body.setIndex == (int)b2SetType.b2_awakeSet )
					{
						color = b2_colorTurquoise;
					}
					else if ( body.isSpeedCapped )
					{
						color = b2_colorYellow;
					}
					else if ( bodySim.isFast )
					{
						color = b2_colorSalmon;
					}
					else if ( body.type == b2BodyType.b2_staticBody )
					{
						color = b2_colorPaleGreen;
					}
					else if ( body.type == b2BodyType.b2_kinematicBody )
					{
						color = b2_colorRoyalBlue;
					}
					else if ( body.setIndex == (int)b2SetType.b2_awakeSet )
					{
						color = b2_colorPink;
					}
					else
					{
						color = b2_colorGray;
					}

					b2DrawShape( draw, shape, xf, color );
					shapeId = shape.nextShapeId;
				}
			}
		}
	}

	if ( draw.drawJoints )
	{
		int count = world.joints.count;
		for ( int i = 0; i < count; ++i )
		{
			b2Joint* joint = world.joints.data + i;
			if ( joint.setIndex == B2_NULL_INDEX )
			{
				continue;
			}

			b2DrawJoint( draw, world, joint );
		}
	}

	if ( draw.drawAABBs )
	{
		b2HexColor color = b2_colorGold;

		int setCount = world.solverSets.count;
		for ( int setIndex = 0; setIndex < setCount; ++setIndex )
		{
			b2SolverSet* set = Array_Get( &world.solverSets, setIndex );
			int bodyCount = set.bodySims.count;
			for ( int bodyIndex = 0; bodyIndex < bodyCount; ++bodyIndex )
			{
				b2BodySim* bodySim = set.bodySims.data + bodyIndex;

				char buffer[32];
				snprintf( buffer, 32, "%d", bodySim.bodyId );
				draw.DrawString( bodySim.center, buffer, b2_colorWhite, draw.context );

				b2Body* body = Array_Get( &world.bodies, bodySim.bodyId );
				Debug.Assert( body.setIndex == setIndex );

				int shapeId = body.headShapeId;
				while ( shapeId != B2_NULL_INDEX )
				{
					b2Shape* shape = world.shapes.data + shapeId;
					b2AABB aabb = shape.fatAABB;

					b2Vec2 vs[4] = { { aabb.lowerBound.x, aabb.lowerBound.y },
									 { aabb.upperBound.x, aabb.lowerBound.y },
									 { aabb.upperBound.x, aabb.upperBound.y },
									 { aabb.lowerBound.x, aabb.upperBound.y } };

					draw.DrawPolygon( vs, 4, color, draw.context );

					shapeId = shape.nextShapeId;
				}
			}
		}
	}

	if ( draw.drawBodyNames )
	{
		b2Vec2 offset = new b2Vec2( 0.1f, 0.2f );
		int count = world.bodies.count;
		for ( int i = 0; i < count; ++i )
		{
			b2Body* body = world.bodies.data + i;
			if ( body.setIndex == B2_NULL_INDEX )
			{
				continue;
			}

			if ( body.name[0] == 0 )
			{
				continue;
			}

			b2Transform transform = b2GetBodyTransformQuick( world, body );
			b2Vec2 p = b2TransformPoint( transform, offset );

			draw.DrawString( p, body.name, b2_colorBlueViolet, draw.context );
		}
	}

	if ( draw.drawMass )
	{
		b2Vec2 offset = new b2Vec2( 0.1f, 0.1f );
		int setCount = world.solverSets.count;
		for ( int setIndex = 0; setIndex < setCount; ++setIndex )
		{
			b2SolverSet* set = Array_Get( &world.solverSets, setIndex );
			int bodyCount = set.bodySims.count;
			for ( int bodyIndex = 0; bodyIndex < bodyCount; ++bodyIndex )
			{
				b2BodySim* bodySim = set.bodySims.data + bodyIndex;

				b2Transform transform = { bodySim.center, bodySim.transform.q };
				draw.DrawTransform( transform, draw.context );

				b2Vec2 p = b2TransformPoint( transform, offset );

				char buffer[32];
				float mass = bodySim.invMass > 0.0f ? 1.0f / bodySim.invMass : 0.0f;
				snprintf( buffer, 32, "  %.2f", mass );
				draw.DrawString( p, buffer, b2_colorWhite, draw.context );
			}
		}
	}

	if ( draw.drawContacts )
	{
		const float k_impulseScale = 1.0f;
		const float k_axisScale = 0.3f;
		const float linearSlop = B2_LINEAR_SLOP;

		b2HexColor speculativeColor = b2_colorLightGray;
		b2HexColor addColor = b2_colorGreen;
		b2HexColor persistColor = b2_colorBlue;
		b2HexColor normalColor = b2_colorDimGray;
		b2HexColor impulseColor = b2_colorMagenta;
		b2HexColor frictionColor = b2_colorYellow;

		b2HexColor colors[B2_GRAPH_COLOR_COUNT] = { b2_colorRed,	   b2_colorOrange,	  b2_colorYellow, b2_colorGreen,
													b2_colorCyan,	   b2_colorBlue,	  b2_colorViolet, b2_colorPink,
													b2_colorChocolate, b2_colorGoldenRod, b2_colorCoral,  b2_colorBlack };

		for ( int colorIndex = 0; colorIndex < B2_GRAPH_COLOR_COUNT; ++colorIndex )
		{
			b2GraphColor* graphColor = world.constraintGraph.colors + colorIndex;

			int contactCount = graphColor.contactSims.count;
			for ( int contactIndex = 0; contactIndex < contactCount; ++contactIndex )
			{
				b2ContactSim* contact = graphColor.contactSims.data + contactIndex;
				int pointCount = contact.manifold.pointCount;
				b2Vec2 normal = contact.manifold.normal;
				char buffer[32];

				for ( int j = 0; j < pointCount; ++j )
				{
					b2ManifoldPoint* point = contact.manifold.points + j;

					if ( draw.drawGraphColors && 0 <= colorIndex && colorIndex <= B2_GRAPH_COLOR_COUNT )
					{
						// graph color
						float pointSize = colorIndex == B2_OVERFLOW_INDEX ? 7.5f : 5.0f;
						draw.DrawPoint( point.point, pointSize, colors[colorIndex], draw.context );
						// g_draw.DrawString(point.position, "%d", point.color);
					}
					else if ( point.separation > linearSlop )
					{
						// Speculative
						draw.DrawPoint( point.point, 5.0f, speculativeColor, draw.context );
					}
					else if ( point.persisted == false )
					{
						// Add
						draw.DrawPoint( point.point, 10.0f, addColor, draw.context );
					}
					else if ( point.persisted == true )
					{
						// Persist
						draw.DrawPoint( point.point, 5.0f, persistColor, draw.context );
					}

					if ( draw.drawContactNormals )
					{
						b2Vec2 p1 = point.point;
						b2Vec2 p2 = b2MulAdd( p1, k_axisScale, normal );
						draw.DrawSegment( p1, p2, normalColor, draw.context );
					}
					else if ( draw.drawContactImpulses )
					{
						b2Vec2 p1 = point.point;
						b2Vec2 p2 = b2MulAdd( p1, k_impulseScale * point.normalImpulse, normal );
						draw.DrawSegment( p1, p2, impulseColor, draw.context );
						snprintf( buffer, B2_ARRAY_COUNT( buffer ), "%.2f", 1000.0f * point.normalImpulse );
						draw.DrawString( p1, buffer, b2_colorWhite, draw.context );
					}

					if ( draw.drawFrictionImpulses )
					{
						b2Vec2 tangent = b2RightPerp( normal );
						b2Vec2 p1 = point.point;
						b2Vec2 p2 = b2MulAdd( p1, k_impulseScale * point.tangentImpulse, tangent );
						draw.DrawSegment( p1, p2, frictionColor, draw.context );
						snprintf( buffer, B2_ARRAY_COUNT( buffer ), "%.2f", point.normalImpulse );
						draw.DrawString( p1, buffer, b2_colorWhite, draw.context );
					}
				}
			}
		}
	}
}

b2BodyEvents b2World_GetBodyEvents( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	Debug.Assert( world.locked == false );
	if ( world.locked )
	{
		return ( b2BodyEvents ){ 0 };
	}

	int count = world.bodyMoveEvents.count;
	b2BodyEvents events = { world.bodyMoveEvents.data, count };
	return events;
}

b2SensorEvents b2World_GetSensorEvents( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	Debug.Assert( world.locked == false );
	if ( world.locked )
	{
		return ( b2SensorEvents ){ 0 };
	}

	// Careful to use previous buffer
	int endEventArrayIndex = 1 - world.endEventArrayIndex;

	int beginCount = world.sensorBeginEvents.count;
	int endCount = world.sensorEndEvents[endEventArrayIndex].count;

	b2SensorEvents events = {
		.beginEvents = world.sensorBeginEvents.data,
		.endEvents = world.sensorEndEvents[endEventArrayIndex].data,
		.beginCount = beginCount,
		.endCount = endCount,
	};
	return events;
}

b2ContactEvents b2World_GetContactEvents( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	Debug.Assert( world.locked == false );
	if ( world.locked )
	{
		return ( b2ContactEvents ){ 0 };
	}

	// Careful to use previous buffer
	int endEventArrayIndex = 1 - world.endEventArrayIndex;

	int beginCount = world.contactBeginEvents.count;
	int endCount = world.contactEndEvents[endEventArrayIndex].count;
	int hitCount = world.contactHitEvents.count;

	b2ContactEvents events = {
		.beginEvents = world.contactBeginEvents.data,
		.endEvents = world.contactEndEvents[endEventArrayIndex].data,
		.hitEvents = world.contactHitEvents.data,
		.beginCount = beginCount,
		.endCount = endCount,
		.hitCount = hitCount,
	};

	return events;
}

bool b2World_IsValid( b2WorldId id )
{
	if ( id.index1 < 1 || B2_MAX_WORLDS < id.index1 )
	{
		return false;
	}

	b2World* world = b2_worlds + ( id.index1 - 1 );

	if ( world.worldId != id.index1 - 1 )
	{
		// world is not allocated
		return false;
	}

	return id.generation == world.generation;
}

public static bool b2Body_IsValid( b2BodyId id )
{
	if ( id.world0 < 0 || B2_MAX_WORLDS <= id.world0 )
	{
		// invalid world
		return false;
	}

	b2World* world = b2_worlds + id.world0;
	if ( world.worldId != id.world0 )
	{
		// world is free
		return false;
	}

	if ( id.index1 < 1 || world.bodies.count < id.index1 )
	{
		// invalid index
		return false;
	}

	b2Body* body = world.bodies.data + ( id.index1 - 1 );
	if ( body.setIndex == B2_NULL_INDEX )
	{
		// this was freed
		return false;
	}

	Debug.Assert( body.localIndex != B2_NULL_INDEX );

	if ( body.generation != id.generation )
	{
		// this id is orphaned
		return false;
	}

	return true;
}

bool b2Shape_IsValid( b2ShapeId id )
{
	if ( B2_MAX_WORLDS <= id.world0 )
	{
		return false;
	}

	b2World* world = b2_worlds + id.world0;
	if ( world.worldId != id.world0 )
	{
		// world is free
		return false;
	}

	int shapeId = id.index1 - 1;
	if ( shapeId < 0 || world.shapes.count <= shapeId )
	{
		return false;
	}

	b2Shape* shape = world.shapes.data + shapeId;
	if ( shape.id == B2_NULL_INDEX )
	{
		// shape is free
		return false;
	}

	Debug.Assert( shape.id == shapeId );

	return id.generation == shape.generation;
}

bool b2Chain_IsValid( b2ChainId id )
{
	if ( id.world0 < 0 || B2_MAX_WORLDS <= id.world0 )
	{
		return false;
	}

	b2World* world = b2_worlds + id.world0;
	if ( world.worldId != id.world0 )
	{
		// world is free
		return false;
	}

	int chainId = id.index1 - 1;
	if ( chainId < 0 || world.chainShapes.count <= chainId )
	{
		return false;
	}

	b2ChainShape* chain = world.chainShapes.data + chainId;
	if ( chain.id == B2_NULL_INDEX )
	{
		// chain is free
		return false;
	}

	Debug.Assert( chain.id == chainId );

	return id.generation == chain.generation;
}

bool b2Joint_IsValid( b2JointId id )
{
	if ( id.world0 < 0 || B2_MAX_WORLDS <= id.world0 )
	{
		return false;
	}

	b2World* world = b2_worlds + id.world0;
	if ( world.worldId != id.world0 )
	{
		// world is free
		return false;
	}

	int jointId = id.index1 - 1;
	if ( jointId < 0 || world.joints.count <= jointId )
	{
		return false;
	}

	b2Joint* joint = world.joints.data + jointId;
	if ( joint.jointId == B2_NULL_INDEX )
	{
		// joint is free
		return false;
	}

	Debug.Assert( joint.jointId == jointId );

	return id.generation == joint.generation;
}

void b2World_EnableSleeping( b2WorldId worldId, bool flag )
{
	b2World* world = b2GetWorldFromId( worldId );
	Debug.Assert( world.locked == false );
	if ( world.locked )
	{
		return;
	}

	if ( flag == world.enableSleep )
	{
		return;
	}

	world.enableSleep = flag;

	if ( flag == false )
	{
		int setCount = world.solverSets.count;
		for ( int i = b2_firstSleepingSet; i < setCount; ++i )
		{
			b2SolverSet* set = Array_Get( &world.solverSets, i );
			if ( set.bodySims.count > 0 )
			{
				b2WakeSolverSet( world, i );
			}
		}
	}
}

bool b2World_IsSleepingEnabled( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	return world.enableSleep;
}

void b2World_EnableWarmStarting( b2WorldId worldId, bool flag )
{
	b2World* world = b2GetWorldFromId( worldId );
	Debug.Assert( world.locked == false );
	if ( world.locked )
	{
		return;
	}

	world.enableWarmStarting = flag;
}

bool b2World_IsWarmStartingEnabled( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	return world.enableWarmStarting;
}

int b2World_GetAwakeBodyCount( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	b2SolverSet* awakeSet = Array_Get( &world.solverSets, (int)b2SetType.b2_awakeSet );
	return awakeSet.bodySims.count;
}

void b2World_EnableContinuous( b2WorldId worldId, bool flag )
{
	b2World* world = b2GetWorldFromId( worldId );
	Debug.Assert( world.locked == false );
	if ( world.locked )
	{
		return;
	}

	world.enableContinuous = flag;
}

bool b2World_IsContinuousEnabled( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	return world.enableContinuous;
}

void b2World_SetRestitutionThreshold( b2WorldId worldId, float value )
{
	b2World* world = b2GetWorldFromId( worldId );
	Debug.Assert( world.locked == false );
	if ( world.locked )
	{
		return;
	}

	world.restitutionThreshold = b2ClampFloat( value, 0.0f, float.MaxValue );
}

float b2World_GetRestitutionThreshold( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	return world.restitutionThreshold;
}

void b2World_SetHitEventThreshold( b2WorldId worldId, float value )
{
	b2World* world = b2GetWorldFromId( worldId );
	Debug.Assert( world.locked == false );
	if ( world.locked )
	{
		return;
	}

	world.hitEventThreshold = b2ClampFloat( value, 0.0f, float.MaxValue );
}

float b2World_GetHitEventThreshold( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	return world.hitEventThreshold;
}

void b2World_SetContactTuning( b2WorldId worldId, float hertz, float dampingRatio, float pushSpeed )
{
	b2World* world = b2GetWorldFromId( worldId );
	Debug.Assert( world.locked == false );
	if ( world.locked )
	{
		return;
	}

	world.contactHertz = b2ClampFloat( hertz, 0.0f, float.MaxValue );
	world.contactDampingRatio = b2ClampFloat( dampingRatio, 0.0f, float.MaxValue );
	world.contactMaxPushSpeed = b2ClampFloat( pushSpeed, 0.0f, float.MaxValue );
}

void b2World_SetJointTuning( b2WorldId worldId, float hertz, float dampingRatio )
{
	b2World* world = b2GetWorldFromId( worldId );
	Debug.Assert( world.locked == false );
	if ( world.locked )
	{
		return;
	}

	world.jointHertz = b2ClampFloat( hertz, 0.0f, float.MaxValue );
	world.jointDampingRatio = b2ClampFloat( dampingRatio, 0.0f, float.MaxValue );
}

void b2World_SetMaximumLinearSpeed( b2WorldId worldId, float maximumLinearSpeed )
{
	Debug.Assert( b2IsValidFloat( maximumLinearSpeed ) && maximumLinearSpeed > 0.0f );

	b2World* world = b2GetWorldFromId( worldId );
	Debug.Assert( world.locked == false );
	if ( world.locked )
	{
		return;
	}

	world.maxLinearSpeed = maximumLinearSpeed;
}

float b2World_GetMaximumLinearSpeed( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	return world.maxLinearSpeed;
}

b2Profile b2World_GetProfile( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	return world.profile;
}

b2Counters b2World_GetCounters( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	b2Counters s = { 0 };
	s.bodyCount = b2GetIdCount( &world.bodyIdPool );
	s.shapeCount = b2GetIdCount( &world.shapeIdPool );
	s.contactCount = b2GetIdCount( &world.contactIdPool );
	s.jointCount = b2GetIdCount( &world.jointIdPool );
	s.islandCount = b2GetIdCount( &world.islandIdPool );

	b2DynamicTree* staticTree = world.broadPhase.trees + b2BodyType.b2_staticBody;
	s.staticTreeHeight = b2DynamicTree_GetHeight( staticTree );

	b2DynamicTree* dynamicTree = world.broadPhase.trees + b2BodyType.b2_dynamicBody;
	b2DynamicTree* kinematicTree = world.broadPhase.trees + b2BodyType.b2_kinematicBody;
	s.treeHeight = b2MaxInt( b2DynamicTree_GetHeight( dynamicTree ), b2DynamicTree_GetHeight( kinematicTree ) );

	s.stackUsed = b2GetMaxArenaAllocation( &world.stackAllocator );
	s.byteCount = b2GetByteCount();
	s.taskCount = world.taskCount;

	for ( int i = 0; i < B2_GRAPH_COLOR_COUNT; ++i )
	{
		s.colorCounts[i] = world.constraintGraph.colors[i].contactSims.count + world.constraintGraph.colors[i].jointSims.count;
	}
	return s;
}

void b2World_SetUserData( b2WorldId worldId, void* userData )
{
	b2World* world = b2GetWorldFromId( worldId );
	world.userData = userData;
}

void* b2World_GetUserData( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	return world.userData;
}

void b2World_SetFrictionCallback( b2WorldId worldId, b2FrictionCallback* callback )
{
	b2World* world = b2GetWorldFromId( worldId );
	if ( world.locked )
	{
		return;
	}

	if ( callback != NULL )
	{
		world.frictionCallback = callback;
	}
	else
	{
		world.frictionCallback = b2DefaultFrictionCallback;
	}
}

void b2World_SetRestitutionCallback( b2WorldId worldId, b2RestitutionCallback* callback )
{
	b2World* world = b2GetWorldFromId( worldId );
	if ( world.locked )
	{
		return;
	}

	if ( callback != NULL )
	{
		world.restitutionCallback = callback;
	}
	else
	{
		world.restitutionCallback = b2DefaultRestitutionCallback;
	}
}

void b2World_DumpMemoryStats( b2WorldId worldId )
{
	FILE* file = fopen( "box2d_memory.txt", "w" );
	if ( file == NULL )
	{
		return;
	}

	b2World* world = b2GetWorldFromId( worldId );

	// id pools
	fprintf( file, "id pools\n" );
	fprintf( file, "body ids: %d\n", b2GetIdBytes( &world.bodyIdPool ) );
	fprintf( file, "solver set ids: %d\n", b2GetIdBytes( &world.solverSetIdPool ) );
	fprintf( file, "joint ids: %d\n", b2GetIdBytes( &world.jointIdPool ) );
	fprintf( file, "contact ids: %d\n", b2GetIdBytes( &world.contactIdPool ) );
	fprintf( file, "island ids: %d\n", b2GetIdBytes( &world.islandIdPool ) );
	fprintf( file, "shape ids: %d\n", b2GetIdBytes( &world.shapeIdPool ) );
	fprintf( file, "chain ids: %d\n", b2GetIdBytes( &world.chainIdPool ) );
	fprintf( file, "\n" );

	// world arrays
	fprintf( file, "world arrays\n" );
	fprintf( file, "bodies: %d\n", b2BodyArray_ByteCount( &world.bodies ) );
	fprintf( file, "solver sets: %d\n", b2SolverSetArray_ByteCount( &world.solverSets ) );
	fprintf( file, "joints: %d\n", b2JointArray_ByteCount( &world.joints ) );
	fprintf( file, "contacts: %d\n", b2ContactArray_ByteCount( &world.contacts ) );
	fprintf( file, "islands: %d\n", b2IslandArray_ByteCount( &world.islands ) );
	fprintf( file, "shapes: %d\n", b2ShapeArray_ByteCount( &world.shapes ) );
	fprintf( file, "chains: %d\n", b2ChainShapeArray_ByteCount( &world.chainShapes ) );
	fprintf( file, "\n" );

	// broad-phase
	fprintf( file, "broad-phase\n" );
	fprintf( file, "static tree: %d\n", b2DynamicTree_GetByteCount( world.broadPhase.trees + b2BodyType.b2_staticBody ) );
	fprintf( file, "kinematic tree: %d\n", b2DynamicTree_GetByteCount( world.broadPhase.trees + b2BodyType.b2_kinematicBody ) );
	fprintf( file, "dynamic tree: %d\n", b2DynamicTree_GetByteCount( world.broadPhase.trees + b2BodyType.b2_dynamicBody ) );
	b2HashSet* moveSet = &world.broadPhase.moveSet;
	fprintf( file, "moveSet: %d (%d, %d)\n", b2GetHashSetBytes( moveSet ), moveSet.count, moveSet.capacity );
	fprintf( file, "moveArray: %d\n", b2IntArray_ByteCount( &world.broadPhase.moveArray ) );
	b2HashSet* pairSet = &world.broadPhase.pairSet;
	fprintf( file, "pairSet: %d (%d, %d)\n", b2GetHashSetBytes( pairSet ), pairSet.count, pairSet.capacity );
	fprintf( file, "\n" );

	// solver sets
	int bodySimCapacity = 0;
	int bodyStateCapacity = 0;
	int jointSimCapacity = 0;
	int contactSimCapacity = 0;
	int islandSimCapacity = 0;
	int solverSetCapacity = world.solverSets.count;
	for ( int i = 0; i < solverSetCapacity; ++i )
	{
		b2SolverSet* set = world.solverSets.data + i;
		if ( set.setIndex == B2_NULL_INDEX )
		{
			continue;
		}

		bodySimCapacity += set.bodySims.capacity;
		bodyStateCapacity += set.bodyStates.capacity;
		jointSimCapacity += set.jointSims.capacity;
		contactSimCapacity += set.contactSims.capacity;
		islandSimCapacity += set.islandSims.capacity;
	}

	fprintf( file, "solver sets\n" );
	fprintf( file, "body sim: %d\n", bodySimCapacity * (int)sizeof( b2BodySim ) );
	fprintf( file, "body state: %d\n", bodyStateCapacity * (int)sizeof( b2BodyState ) );
	fprintf( file, "joint sim: %d\n", jointSimCapacity * (int)sizeof( b2JointSim ) );
	fprintf( file, "contact sim: %d\n", contactSimCapacity * (int)sizeof( b2ContactSim ) );
	fprintf( file, "island sim: %d\n", islandSimCapacity * (int)sizeof( islandSimCapacity ) );
	fprintf( file, "\n" );

	// constraint graph
	int bodyBitSetBytes = 0;
	contactSimCapacity = 0;
	jointSimCapacity = 0;
	for ( int i = 0; i < B2_GRAPH_COLOR_COUNT; ++i )
	{
		b2GraphColor* c = world.constraintGraph.colors + i;
		bodyBitSetBytes += b2GetBitSetBytes( &c.bodySet );
		contactSimCapacity += c.contactSims.capacity;
		jointSimCapacity += c.jointSims.capacity;
	}

	fprintf( file, "constraint graph\n" );
	fprintf( file, "body bit sets: %d\n", bodyBitSetBytes );
	fprintf( file, "joint sim: %d\n", jointSimCapacity * (int)sizeof( b2JointSim ) );
	fprintf( file, "contact sim: %d\n", contactSimCapacity * (int)sizeof( b2ContactSim ) );
	fprintf( file, "\n" );

	// stack allocator
	fprintf( file, "stack allocator: %d\n\n", world.stackAllocator.capacity );

	// chain shapes
	// todo

	fclose( file );
}

typedef struct WorldQueryContext
{
	b2World* world;
	b2OverlapResultFcn* fcn;
	b2QueryFilter filter;
	void* userContext;
} WorldQueryContext;

static bool TreeQueryCallback( int proxyId, int shapeId, void* context )
{
	B2_UNUSED( proxyId );

	WorldQueryContext* worldContext = context;
	b2World* world = worldContext.world;

	b2Shape* shape = Array_Get( &world.shapes, shapeId );

	b2Filter shapeFilter = shape.filter;
	b2QueryFilter queryFilter = worldContext.filter;

	if ( ( shapeFilter.categoryBits & queryFilter.maskBits ) == 0 || ( shapeFilter.maskBits & queryFilter.categoryBits ) == 0 )
	{
		return true;
	}

	b2ShapeId id = { shapeId + 1, world.worldId, shape.generation };
	bool result = worldContext.fcn( id, worldContext.userContext );
	return result;
}

b2TreeStats b2World_OverlapAABB( b2WorldId worldId, b2AABB aabb, b2QueryFilter filter, b2OverlapResultFcn* fcn, void* context )
{
	b2TreeStats treeStats = { 0 };

	b2World* world = b2GetWorldFromId( worldId );
	Debug.Assert( world.locked == false );
	if ( world.locked )
	{
		return treeStats;
	}

	Debug.Assert( b2IsValidAABB( aabb ) );

	WorldQueryContext worldContext = { world, fcn, filter, context };

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2TreeStats treeResult =
			b2DynamicTree_Query( world.broadPhase.trees + i, aabb, filter.maskBits, TreeQueryCallback, &worldContext );

		treeStats.nodeVisits += treeResult.nodeVisits;
		treeStats.leafVisits += treeResult.leafVisits;
	}

	return treeStats;
}

typedef struct WorldOverlapContext
{
	b2World* world;
	b2OverlapResultFcn* fcn;
	b2QueryFilter filter;
	b2ShapeProxy proxy;
	b2Transform transform;
	void* userContext;
} WorldOverlapContext;

static bool TreeOverlapCallback( int proxyId, int shapeId, void* context )
{
	B2_UNUSED( proxyId );

	WorldOverlapContext* worldContext = context;
	b2World* world = worldContext.world;

	b2Shape* shape = Array_Get( &world.shapes, shapeId );

	b2Filter shapeFilter = shape.filter;
	b2QueryFilter queryFilter = worldContext.filter;

	if ( ( shapeFilter.categoryBits & queryFilter.maskBits ) == 0 || ( shapeFilter.maskBits & queryFilter.categoryBits ) == 0 )
	{
		return true;
	}

	b2Body* body = Array_Get( &world.bodies, shape.bodyId );
	b2Transform transform = b2GetBodyTransformQuick( world, body );

	b2DistanceInput input;
	input.proxyA = worldContext.proxy;
	input.proxyB = b2MakeShapeDistanceProxy( shape );
	input.transformA = worldContext.transform;
	input.transformB = transform;
	input.useRadii = true;

	b2SimplexCache cache = { 0 };
	b2DistanceOutput output = b2ShapeDistance( &cache, &input, NULL, 0 );

	if ( output.distance > 0.0f )
	{
		return true;
	}

	b2ShapeId id = { shape.id + 1, world.worldId, shape.generation };
	bool result = worldContext.fcn( id, worldContext.userContext );
	return result;
}

b2TreeStats b2World_OverlapPoint( b2WorldId worldId, b2Vec2 point, b2Transform transform, b2QueryFilter filter,
								  b2OverlapResultFcn* fcn, void* context )
{
	b2Circle circle = { point, 0.0f };
	return b2World_OverlapCircle( worldId, &circle, transform, filter, fcn, context );
}

b2TreeStats b2World_OverlapCircle( b2WorldId worldId, const b2Circle* circle, b2Transform transform, b2QueryFilter filter,
								   b2OverlapResultFcn* fcn, void* context )
{
	b2TreeStats treeStats = { 0 };

	b2World* world = b2GetWorldFromId( worldId );
	Debug.Assert( world.locked == false );
	if ( world.locked )
	{
		return treeStats;
	}

	Debug.Assert( b2IsValidVec2( transform.p ) );
	Debug.Assert( b2IsValidRotation( transform.q ) );

	b2AABB aabb = b2ComputeCircleAABB( circle, transform );
	WorldOverlapContext worldContext = {
		world, fcn, filter, b2MakeProxy( &circle.center, 1, circle.radius ), transform, context,
	};

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2TreeStats treeResult =
			b2DynamicTree_Query( world.broadPhase.trees + i, aabb, filter.maskBits, TreeOverlapCallback, &worldContext );

		treeStats.nodeVisits += treeResult.nodeVisits;
		treeStats.leafVisits += treeResult.leafVisits;
	}

	return treeStats;
}

b2TreeStats b2World_OverlapCapsule( b2WorldId worldId, const b2Capsule* capsule, b2Transform transform, b2QueryFilter filter,
									b2OverlapResultFcn* fcn, void* context )
{
	b2TreeStats treeStats = { 0 };

	b2World* world = b2GetWorldFromId( worldId );
	Debug.Assert( world.locked == false );
	if ( world.locked )
	{
		return treeStats;
	}

	Debug.Assert( b2IsValidVec2( transform.p ) );
	Debug.Assert( b2IsValidRotation( transform.q ) );

	b2AABB aabb = b2ComputeCapsuleAABB( capsule, transform );
	WorldOverlapContext worldContext = {
		world, fcn, filter, b2MakeProxy( &capsule.center1, 2, capsule.radius ), transform, context,
	};

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2TreeStats treeResult =
			b2DynamicTree_Query( world.broadPhase.trees + i, aabb, filter.maskBits, TreeOverlapCallback, &worldContext );

		treeStats.nodeVisits += treeResult.nodeVisits;
		treeStats.leafVisits += treeResult.leafVisits;
	}

	return treeStats;
}

b2TreeStats b2World_OverlapPolygon( b2WorldId worldId, const b2Polygon* polygon, b2Transform transform, b2QueryFilter filter,
									b2OverlapResultFcn* fcn, void* context )
{
	b2TreeStats treeStats = { 0 };

	b2World* world = b2GetWorldFromId( worldId );
	Debug.Assert( world.locked == false );
	if ( world.locked )
	{
		return treeStats;
	}

	Debug.Assert( b2IsValidVec2( transform.p ) );
	Debug.Assert( b2IsValidRotation( transform.q ) );

	b2AABB aabb = b2ComputePolygonAABB( polygon, transform );
	WorldOverlapContext worldContext = {
		world, fcn, filter, b2MakeProxy( polygon.vertices, polygon.count, polygon.radius ), transform, context,
	};

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2TreeStats treeResult =
			b2DynamicTree_Query( world.broadPhase.trees + i, aabb, filter.maskBits, TreeOverlapCallback, &worldContext );

		treeStats.nodeVisits += treeResult.nodeVisits;
		treeStats.leafVisits += treeResult.leafVisits;
	}

	return treeStats;
}

typedef struct WorldRayCastContext
{
	b2World* world;
	b2CastResultFcn* fcn;
	b2QueryFilter filter;
	float fraction;
	void* userContext;
} WorldRayCastContext;

static float RayCastCallback( const b2RayCastInput* input, int proxyId, int shapeId, void* context )
{
	B2_UNUSED( proxyId );

	WorldRayCastContext* worldContext = context;
	b2World* world = worldContext.world;

	b2Shape* shape = Array_Get( &world.shapes, shapeId );
	b2Filter shapeFilter = shape.filter;
	b2QueryFilter queryFilter = worldContext.filter;

	if ( ( shapeFilter.categoryBits & queryFilter.maskBits ) == 0 || ( shapeFilter.maskBits & queryFilter.categoryBits ) == 0 )
	{
		return input.maxFraction;
	}

	b2Body* body = Array_Get( &world.bodies, shape.bodyId );
	b2Transform transform = b2GetBodyTransformQuick( world, body );
	b2CastOutput output = b2RayCastShape( input, shape, transform );

	if ( output.hit )
	{
		b2ShapeId id = { shapeId + 1, world.worldId, shape.generation };
		float fraction = worldContext.fcn( id, output.point, output.normal, output.fraction, worldContext.userContext );

		// The user may return -1 to skip this shape
		if ( 0.0f <= fraction && fraction <= 1.0f )
		{
			worldContext.fraction = fraction;
		}

		return fraction;
	}

	return input.maxFraction;
}

b2TreeStats b2World_CastRay( b2WorldId worldId, b2Vec2 origin, b2Vec2 translation, b2QueryFilter filter, b2CastResultFcn* fcn,
							 void* context )
{
	b2TreeStats treeStats = { 0 };

	b2World* world = b2GetWorldFromId( worldId );
	Debug.Assert( world.locked == false );
	if ( world.locked )
	{
		return treeStats;
	}

	Debug.Assert( b2IsValidVec2( origin ) );
	Debug.Assert( b2IsValidVec2( translation ) );

	b2RayCastInput input = { origin, translation, 1.0f };

	WorldRayCastContext worldContext = { world, fcn, filter, 1.0f, context };

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2TreeStats treeResult =
			b2DynamicTree_RayCast( world.broadPhase.trees + i, &input, filter.maskBits, RayCastCallback, &worldContext );
		treeStats.nodeVisits += treeResult.nodeVisits;
		treeStats.leafVisits += treeResult.leafVisits;

		if ( worldContext.fraction == 0.0f )
		{
			return treeStats;
		}

		input.maxFraction = worldContext.fraction;
	}

	return treeStats;
}

// This callback finds the closest hit. This is the most common callback used in games.
static float b2RayCastClosestFcn( b2ShapeId shapeId, b2Vec2 point, b2Vec2 normal, float fraction, void* context )
{
	b2RayResult* rayResult = (b2RayResult*)context;
	rayResult.shapeId = shapeId;
	rayResult.point = point;
	rayResult.normal = normal;
	rayResult.fraction = fraction;
	rayResult.hit = true;
	return fraction;
}

b2RayResult b2World_CastRayClosest( b2WorldId worldId, b2Vec2 origin, b2Vec2 translation, b2QueryFilter filter )
{
	b2RayResult result = { 0 };

	b2World* world = b2GetWorldFromId( worldId );
	Debug.Assert( world.locked == false );
	if ( world.locked )
	{
		return result;
	}

	Debug.Assert( b2IsValidVec2( origin ) );
	Debug.Assert( b2IsValidVec2( translation ) );

	b2RayCastInput input = { origin, translation, 1.0f };
	WorldRayCastContext worldContext = { world, b2RayCastClosestFcn, filter, 1.0f, &result };

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2TreeStats treeResult =
			b2DynamicTree_RayCast( world.broadPhase.trees + i, &input, filter.maskBits, RayCastCallback, &worldContext );
		result.nodeVisits += treeResult.nodeVisits;
		result.leafVisits += treeResult.leafVisits;

		if ( worldContext.fraction == 0.0f )
		{
			return result;
		}

		input.maxFraction = worldContext.fraction;
	}

	return result;
}

static float ShapeCastCallback( const b2ShapeCastInput* input, int proxyId, int shapeId, void* context )
{
	B2_UNUSED( proxyId );

	WorldRayCastContext* worldContext = context;
	b2World* world = worldContext.world;

	b2Shape* shape = Array_Get( &world.shapes, shapeId );
	b2Filter shapeFilter = shape.filter;
	b2QueryFilter queryFilter = worldContext.filter;

	if ( ( shapeFilter.categoryBits & queryFilter.maskBits ) == 0 || ( shapeFilter.maskBits & queryFilter.categoryBits ) == 0 )
	{
		return input.maxFraction;
	}

	b2Body* body = Array_Get( &world.bodies, shape.bodyId );
	b2Transform transform = b2GetBodyTransformQuick( world, body );

	b2CastOutput output = b2ShapeCastShape( input, shape, transform );

	if ( output.hit )
	{
		b2ShapeId id = { shapeId + 1, world.worldId, shape.generation };
		float fraction = worldContext.fcn( id, output.point, output.normal, output.fraction, worldContext.userContext );
		worldContext.fraction = fraction;
		return fraction;
	}

	return input.maxFraction;
}

b2TreeStats b2World_CastCircle( b2WorldId worldId, const b2Circle* circle, b2Transform originTransform, b2Vec2 translation,
								b2QueryFilter filter, b2CastResultFcn* fcn, void* context )
{
	b2TreeStats treeStats = { 0 };

	b2World* world = b2GetWorldFromId( worldId );
	Debug.Assert( world.locked == false );
	if ( world.locked )
	{
		return treeStats;
	}

	Debug.Assert( b2IsValidVec2( originTransform.p ) );
	Debug.Assert( b2IsValidRotation( originTransform.q ) );
	Debug.Assert( b2IsValidVec2( translation ) );

	b2ShapeCastInput input;
	input.points[0] = b2TransformPoint( originTransform, circle.center );
	input.count = 1;
	input.radius = circle.radius;
	input.translation = translation;
	input.maxFraction = 1.0f;

	WorldRayCastContext worldContext = { world, fcn, filter, 1.0f, context };

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2TreeStats treeResult =
			b2DynamicTree_ShapeCast( world.broadPhase.trees + i, &input, filter.maskBits, ShapeCastCallback, &worldContext );
		treeStats.nodeVisits += treeResult.nodeVisits;
		treeStats.leafVisits += treeResult.leafVisits;

		if ( worldContext.fraction == 0.0f )
		{
			return treeStats;
		}

		input.maxFraction = worldContext.fraction;
	}

	return treeStats;
}

b2TreeStats b2World_CastCapsule( b2WorldId worldId, const b2Capsule* capsule, b2Transform originTransform, b2Vec2 translation,
								 b2QueryFilter filter, b2CastResultFcn* fcn, void* context )
{
	b2TreeStats treeStats = { 0 };

	b2World* world = b2GetWorldFromId( worldId );
	Debug.Assert( world.locked == false );
	if ( world.locked )
	{
		return treeStats;
	}

	Debug.Assert( b2IsValidVec2( originTransform.p ) );
	Debug.Assert( b2IsValidRotation( originTransform.q ) );
	Debug.Assert( b2IsValidVec2( translation ) );

	b2ShapeCastInput input;
	input.points[0] = b2TransformPoint( originTransform, capsule.center1 );
	input.points[1] = b2TransformPoint( originTransform, capsule.center2 );
	input.count = 2;
	input.radius = capsule.radius;
	input.translation = translation;
	input.maxFraction = 1.0f;

	WorldRayCastContext worldContext = { world, fcn, filter, 1.0f, context };

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2TreeStats treeResult =
			b2DynamicTree_ShapeCast( world.broadPhase.trees + i, &input, filter.maskBits, ShapeCastCallback, &worldContext );
		treeStats.nodeVisits += treeResult.nodeVisits;
		treeStats.leafVisits += treeResult.leafVisits;

		if ( worldContext.fraction == 0.0f )
		{
			return treeStats;
		}

		input.maxFraction = worldContext.fraction;
	}

	return treeStats;
}

b2TreeStats b2World_CastPolygon( b2WorldId worldId, const b2Polygon* polygon, b2Transform originTransform, b2Vec2 translation,
								 b2QueryFilter filter, b2CastResultFcn* fcn, void* context )
{
	b2TreeStats treeStats = { 0 };

	b2World* world = b2GetWorldFromId( worldId );
	Debug.Assert( world.locked == false );
	if ( world.locked )
	{
		return treeStats;
	}

	Debug.Assert( b2IsValidVec2( originTransform.p ) );
	Debug.Assert( b2IsValidRotation( originTransform.q ) );
	Debug.Assert( b2IsValidVec2( translation ) );

	b2ShapeCastInput input;
	for ( int i = 0; i < polygon.count; ++i )
	{
		input.points[i] = b2TransformPoint( originTransform, polygon.vertices[i] );
	}
	input.count = polygon.count;
	input.radius = polygon.radius;
	input.translation = translation;
	input.maxFraction = 1.0f;

	WorldRayCastContext worldContext = { world, fcn, filter, 1.0f, context };

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2TreeStats treeResult =
			b2DynamicTree_ShapeCast( world.broadPhase.trees + i, &input, filter.maskBits, ShapeCastCallback, &worldContext );
		treeStats.nodeVisits += treeResult.nodeVisits;
		treeStats.leafVisits += treeResult.leafVisits;

		if ( worldContext.fraction == 0.0f )
		{
			return treeStats;
		}

		input.maxFraction = worldContext.fraction;
	}

	return treeStats;
}

#if ZERO_DEFINE
void b2World_ShiftOrigin(b2WorldId worldId, b2Vec2 newOrigin)
{
	Debug.Assert(m_locked == false);
	if (m_locked)
	{
		return;
	}

	for (b2Body* b = m_bodyList; b; b = b.m_next)
	{
		b.m_xf.p -= newOrigin;
		b.m_sweep.c0 -= newOrigin;
		b.m_sweep.c -= newOrigin;
	}

	for (b2Joint* j = m_jointList; j; j = j.m_next)
	{
		j.ShiftOrigin(newOrigin);
	}

	m_contactManager.m_broadPhase.ShiftOrigin(newOrigin);
}

void b2World_Dump()
{
	if (m_locked)
	{
		return;
	}

	b2OpenDump("box2d_dump.inl");

	b2Dump("b2Vec2 g(%.9g, %.9g);\n", m_gravity.x, m_gravity.y);
	b2Dump("m_world.SetGravity(g);\n");

	b2Dump("b2Body** sims = (b2Body**)b2Alloc(%d * sizeof(b2Body*));\n", m_bodyCount);
	b2Dump("b2Joint** joints = (b2Joint**)b2Alloc(%d * sizeof(b2Joint*));\n", m_jointCount);

	int32 i = 0;
	for (b2Body* b = m_bodyList; b; b = b.m_next)
	{
		b.m_islandIndex = i;
		b.Dump();
		++i;
	}

	i = 0;
	for (b2Joint* j = m_jointList; j; j = j.m_next)
	{
		j.m_index = i;
		++i;
	}

	// First pass on joints, skip gear joints.
	for (b2Joint* j = m_jointList; j; j = j.m_next)
	{
		if (j.m_type == e_gearJoint)
		{
			continue;
		}

		b2Dump("{\n");
		j.Dump();
		b2Dump("}\n");
	}

	// Second pass on joints, only gear joints.
	for (b2Joint* j = m_jointList; j; j = j.m_next)
	{
		if (j.m_type != e_gearJoint)
		{
			continue;
		}

		b2Dump("{\n");
		j.Dump();
		b2Dump("}\n");
	}

	b2Dump("b2Free(joints);\n");
	b2Dump("b2Free(sims);\n");
	b2Dump("joints = nullptr;\n");
	b2Dump("sims = nullptr;\n");

	b2CloseDump();
}
#endif

void b2World_SetCustomFilterCallback( b2WorldId worldId, b2CustomFilterFcn* fcn, void* context )
{
	b2World* world = b2GetWorldFromId( worldId );
	world.customFilterFcn = fcn;
	world.customFilterContext = context;
}

void b2World_SetPreSolveCallback( b2WorldId worldId, b2PreSolveFcn* fcn, void* context )
{
	b2World* world = b2GetWorldFromId( worldId );
	world.preSolveFcn = fcn;
	world.preSolveContext = context;
}

void b2World_SetGravity( b2WorldId worldId, b2Vec2 gravity )
{
	b2World* world = b2GetWorldFromId( worldId );
	world.gravity = gravity;
}

b2Vec2 b2World_GetGravity( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	return world.gravity;
}

struct ExplosionContext
{
	b2World* world;
	b2Vec2 position;
	float radius;
	float falloff;
	float impulsePerLength;
};

static bool ExplosionCallback( int proxyId, int shapeId, void* context )
{
	B2_UNUSED( proxyId );

	struct ExplosionContext* explosionContext = context;
	b2World* world = explosionContext.world;

	b2Shape* shape = Array_Get( &world.shapes, shapeId );

	b2Body* body = Array_Get( &world.bodies, shape.bodyId );
	Debug.Assert( body.type == b2BodyType.b2_dynamicBody );

	b2Transform transform = b2GetBodyTransformQuick( world, body );

	b2DistanceInput input;
	input.proxyA = b2MakeShapeDistanceProxy( shape );
	input.proxyB = b2MakeProxy( &explosionContext.position, 1, 0.0f );
	input.transformA = transform;
	input.transformB = b2Transform_identity;
	input.useRadii = true;

	b2SimplexCache cache = { 0 };
	b2DistanceOutput output = b2ShapeDistance( &cache, &input, NULL, 0 );

	float radius = explosionContext.radius;
	float falloff = explosionContext.falloff;
	if ( output.distance > radius + falloff )
	{
		return true;
	}

	b2WakeBody( world, body );

	if ( body.setIndex != (int)b2SetType.b2_awakeSet )
	{
		return true;
	}

	b2Vec2 closestPoint = output.pointA;
	if ( output.distance == 0.0f )
	{
		b2Vec2 localCentroid = b2GetShapeCentroid( shape );
		closestPoint = b2TransformPoint( transform, localCentroid );
	}

	b2Vec2 direction = b2Sub( closestPoint, explosionContext.position );
	if ( b2LengthSquared( direction ) > 100.0f * Epsilon * Epsilon )
	{
		direction = b2Normalize( direction );
	}
	else
	{
		direction = new b2Vec2( 1.0f, 0.0f);
	}

	b2Vec2 localLine = b2InvRotateVector( transform.q, b2LeftPerp( direction ) );
	float perimeter = b2GetShapeProjectedPerimeter( shape, localLine );
	float scale = 1.0f;
	if ( output.distance > radius && falloff > 0.0f )
	{
		scale = b2ClampFloat( ( radius + falloff - output.distance ) / falloff, 0.0f, 1.0f );
	}

	float magnitude = explosionContext.impulsePerLength * perimeter * scale;
	b2Vec2 impulse = b2MulSV( magnitude, direction );

	int localIndex = body.localIndex;
	b2SolverSet* set = Array_Get( &world.solverSets, (int)b2SetType.b2_awakeSet );
	b2BodyState* state = Array_Get( &set.bodyStates, localIndex );
	b2BodySim* bodySim = Array_Get( &set.bodySims, localIndex );
	state.linearVelocity = b2MulAdd( state.linearVelocity, bodySim.invMass, impulse );
	state.angularVelocity += bodySim.invInertia * b2Cross( b2Sub( closestPoint, bodySim.center ), impulse );

	return true;
}

void b2World_Explode( b2WorldId worldId, const b2ExplosionDef* explosionDef )
{
	ulong maskBits = explosionDef.maskBits;
	b2Vec2 position = explosionDef.position;
	float radius = explosionDef.radius;
	float falloff = explosionDef.falloff;
	float impulsePerLength = explosionDef.impulsePerLength;

	Debug.Assert( b2IsValidVec2( position ) );
	Debug.Assert( b2IsValidFloat( radius ) && radius >= 0.0f );
	Debug.Assert( b2IsValidFloat( falloff ) && falloff >= 0.0f );
	Debug.Assert( b2IsValidFloat( impulsePerLength ) );

	b2World* world = b2GetWorldFromId( worldId );
	Debug.Assert( world.locked == false );
	if ( world.locked )
	{
		return;
	}

	struct ExplosionContext explosionContext = { world, position, radius, falloff, impulsePerLength };

	b2AABB aabb;
	aabb.lowerBound.x = position.x - ( radius + falloff );
	aabb.lowerBound.y = position.y - ( radius + falloff );
	aabb.upperBound.x = position.x + ( radius + falloff );
	aabb.upperBound.y = position.y + ( radius + falloff );

	b2DynamicTree_Query( world.broadPhase.trees + b2BodyType.b2_dynamicBody, aabb, maskBits, ExplosionCallback, &explosionContext );
}

void b2World_RebuildStaticTree( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	Debug.Assert( world.locked == false );
	if ( world.locked )
	{
		return;
	}

	b2DynamicTree* staticTree = world.broadPhase.trees + b2BodyType.b2_staticBody;
	b2DynamicTree_Rebuild( staticTree, true );
}

void b2World_EnableSpeculative( b2WorldId worldId, bool flag )
{
	b2World* world = b2GetWorldFromId( worldId );
	world.enableSpeculative = flag;
}

#if B2_VALIDATE
// When validating islands ids I have to compare the root island
// ids because islands are not merged until the next time step.
static int b2GetRootIslandId( b2World* world, int islandId )
{
	if ( islandId == B2_NULL_INDEX )
	{
		return B2_NULL_INDEX;
	}

	b2Island* island = Array_Get( &world.islands, islandId );

	int rootId = islandId;
	b2Island* rootIsland = island;
	while ( rootIsland.parentIsland != B2_NULL_INDEX )
	{
		b2Island* parent = Array_Get( &world.islands, rootIsland.parentIsland );
		rootId = rootIsland.parentIsland;
		rootIsland = parent;
	}

	return rootId;
}

// This validates island graph connectivity for each body
void b2ValidateConnectivity( b2World* world )
{
	b2Body* bodies = world.bodies.data;
	int bodyCapacity = world.bodies.count;

	for ( int bodyIndex = 0; bodyIndex < bodyCapacity; ++bodyIndex )
	{
		b2Body* body = bodies + bodyIndex;
		if ( body.id == B2_NULL_INDEX )
		{
			b2ValidateFreeId( &world.bodyIdPool, bodyIndex );
			continue;
		}

		Debug.Assert( bodyIndex == body.id );

		// Need to get the root island because islands are not merged until the next time step
		int bodyIslandId = b2GetRootIslandId( world, body.islandId );
		int bodySetIndex = body.setIndex;

		int contactKey = body.headContactKey;
		while ( contactKey != B2_NULL_INDEX )
		{
			int contactId = contactKey >> 1;
			int edgeIndex = contactKey & 1;

			b2Contact* contact = Array_Get( &world.contacts, contactId );

			bool touching = ( contact.flags & b2ContactFlags.b2_contactTouchingFlag ) != 0;
			if ( touching )
			{
				if ( bodySetIndex != (int)b2SetType.b2_staticSet )
				{
					int contactIslandId = b2GetRootIslandId( world, contact.islandId );
					Debug.Assert( contactIslandId == bodyIslandId );
				}
			}
			else
			{
				Debug.Assert( contact.islandId == B2_NULL_INDEX );
			}

			contactKey = contact.edges[edgeIndex].nextKey;
		}

		int jointKey = body.headJointKey;
		while ( jointKey != B2_NULL_INDEX )
		{
			int jointId = jointKey >> 1;
			int edgeIndex = jointKey & 1;

			b2Joint* joint = Array_Get( &world.joints, jointId );

			int otherEdgeIndex = edgeIndex ^ 1;

			b2Body* otherBody = Array_Get( &world.bodies, joint.edges[otherEdgeIndex].bodyId );

			if ( bodySetIndex == (int)b2SetType.b2_disabledSet || otherBody.setIndex == (int)b2SetType.b2_disabledSet )
			{
				Debug.Assert( joint.islandId == B2_NULL_INDEX );
			}
			else if ( bodySetIndex == (int)b2SetType.b2_staticSet )
			{
				if ( otherBody.setIndex == (int)b2SetType.b2_staticSet )
				{
					Debug.Assert( joint.islandId == B2_NULL_INDEX );
				}
			}
			else
			{
				int jointIslandId = b2GetRootIslandId( world, joint.islandId );
				Debug.Assert( jointIslandId == bodyIslandId );
			}

			jointKey = joint.edges[edgeIndex].nextKey;
		}
	}
}

// Validates solver sets, but not island connectivity
void b2ValidateSolverSets( b2World* world )
{
	Debug.Assert( b2GetIdCapacity( &world.bodyIdPool ) == world.bodies.count );
	Debug.Assert( b2GetIdCapacity( &world.contactIdPool ) == world.contacts.count );
	Debug.Assert( b2GetIdCapacity( &world.jointIdPool ) == world.joints.count );
	Debug.Assert( b2GetIdCapacity( &world.islandIdPool ) == world.islands.count );
	Debug.Assert( b2GetIdCapacity( &world.solverSetIdPool ) == world.solverSets.count );

	int activeSetCount = 0;
	int totalBodyCount = 0;
	int totalJointCount = 0;
	int totalContactCount = 0;
	int totalIslandCount = 0;

	// Validate all solver sets
	int setCount = world.solverSets.count;
	for ( int setIndex = 0; setIndex < setCount; ++setIndex )
	{
		b2SolverSet* set = world.solverSets.data + setIndex;
		if ( set.setIndex != B2_NULL_INDEX )
		{
			activeSetCount += 1;

			if ( setIndex == (int)b2SetType.b2_staticSet )
			{
				Debug.Assert( set.contactSims.count == 0 );
				Debug.Assert( set.islandSims.count == 0 );
				Debug.Assert( set.bodyStates.count == 0 );
			}
			else if ( setIndex == (int)b2SetType.b2_awakeSet )
			{
				Debug.Assert( set.bodySims.count == set.bodyStates.count );
				Debug.Assert( set.jointSims.count == 0 );
			}
			else if ( setIndex == (int)b2SetType.b2_disabledSet )
			{
				Debug.Assert( set.islandSims.count == 0 );
				Debug.Assert( set.bodyStates.count == 0 );
			}
			else
			{
				Debug.Assert( set.bodyStates.count == 0 );
			}

			// Validate bodies
			{
				b2Body* bodies = world.bodies.data;
				Debug.Assert( set.bodySims.count >= 0 );
				totalBodyCount += set.bodySims.count;
				for ( int i = 0; i < set.bodySims.count; ++i )
				{
					b2BodySim* bodySim = set.bodySims.data + i;

					int bodyId = bodySim.bodyId;
					Debug.Assert( 0 <= bodyId && bodyId < world.bodies.count );
					b2Body* body = bodies + bodyId;
					Debug.Assert( body.setIndex == setIndex );
					Debug.Assert( body.localIndex == i );
					Debug.Assert( body.generation == body.generation );

					if ( setIndex == (int)b2SetType.b2_disabledSet )
					{
						Debug.Assert( body.headContactKey == B2_NULL_INDEX );
					}

					// Validate body shapes
					int prevShapeId = B2_NULL_INDEX;
					int shapeId = body.headShapeId;
					while ( shapeId != B2_NULL_INDEX )
					{
						b2Shape* shape = Array_Get( &world.shapes, shapeId );
						Debug.Assert( shape.id == shapeId );
						Debug.Assert( shape.prevShapeId == prevShapeId );

						if ( setIndex == (int)b2SetType.b2_disabledSet )
						{
							Debug.Assert( shape.proxyKey == B2_NULL_INDEX );
						}
						else if ( setIndex == (int)b2SetType.b2_staticSet )
						{
							Debug.Assert( B2_PROXY_TYPE( shape.proxyKey ) == b2BodyType.b2_staticBody );
						}
						else
						{
							b2BodyType proxyType = B2_PROXY_TYPE( shape.proxyKey );
							Debug.Assert( proxyType == b2BodyType.b2_kinematicBody || proxyType == b2BodyType.b2_dynamicBody );
						}

						prevShapeId = shapeId;
						shapeId = shape.nextShapeId;
					}

					// Validate body contacts
					int contactKey = body.headContactKey;
					while ( contactKey != B2_NULL_INDEX )
					{
						int contactId = contactKey >> 1;
						int edgeIndex = contactKey & 1;

						b2Contact* contact = Array_Get( &world.contacts, contactId );
						Debug.Assert( contact.setIndex != (int)b2SetType.b2_staticSet );
						Debug.Assert( contact.edges[0].bodyId == bodyId || contact.edges[1].bodyId == bodyId );
						contactKey = contact.edges[edgeIndex].nextKey;
					}

					// Validate body joints
					int jointKey = body.headJointKey;
					while ( jointKey != B2_NULL_INDEX )
					{
						int jointId = jointKey >> 1;
						int edgeIndex = jointKey & 1;

						b2Joint* joint = Array_Get( &world.joints, jointId );

						int otherEdgeIndex = edgeIndex ^ 1;

						b2Body* otherBody = Array_Get( &world.bodies, joint.edges[otherEdgeIndex].bodyId );

						if ( setIndex == (int)b2SetType.b2_disabledSet || otherBody.setIndex == (int)b2SetType.b2_disabledSet )
						{
							Debug.Assert( joint.setIndex == (int)b2SetType.b2_disabledSet );
						}
						else if ( setIndex == (int)b2SetType.b2_staticSet && otherBody.setIndex == (int)b2SetType.b2_staticSet )
						{
							Debug.Assert( joint.setIndex == (int)b2SetType.b2_staticSet );
						}
						else if ( setIndex == (int)b2SetType.b2_awakeSet )
						{
							Debug.Assert( joint.setIndex == (int)b2SetType.b2_awakeSet );
						}
						else if ( setIndex >= (int)b2SetType.b2_firstSleepingSet )
						{
							Debug.Assert( joint.setIndex == setIndex );
						}

						b2JointSim* jointSim = b2GetJointSim( world, joint );
						Debug.Assert( jointSim.jointId == jointId );
						Debug.Assert( jointSim.bodyIdA == joint.edges[0].bodyId );
						Debug.Assert( jointSim.bodyIdB == joint.edges[1].bodyId );

						jointKey = joint.edges[edgeIndex].nextKey;
					}
				}
			}

			// Validate contacts
			{
				Debug.Assert( set.contactSims.count >= 0 );
				totalContactCount += set.contactSims.count;
				for ( int i = 0; i < set.contactSims.count; ++i )
				{
					b2ContactSim* contactSim = set.contactSims.data + i;
					b2Contact* contact = Array_Get( &world.contacts, contactSim.contactId );
					if ( setIndex == (int)b2SetType.b2_awakeSet )
					{
						// contact should be non-touching if awake
						// or it could be this contact hasn't been transferred yet
						Debug.Assert( contactSim.manifold.pointCount == 0 ||
								   ( contactSim.simFlags & b2_simStartedTouching ) != 0 );
					}
					Debug.Assert( contact.setIndex == setIndex );
					Debug.Assert( contact.colorIndex == B2_NULL_INDEX );
					Debug.Assert( contact.localIndex == i );
				}
			}

			// Validate joints
			{
				Debug.Assert( set.jointSims.count >= 0 );
				totalJointCount += set.jointSims.count;
				for ( int i = 0; i < set.jointSims.count; ++i )
				{
					b2JointSim* jointSim = set.jointSims.data + i;
					b2Joint* joint = Array_Get( &world.joints, jointSim.jointId );
					Debug.Assert( joint.setIndex == setIndex );
					Debug.Assert( joint.colorIndex == B2_NULL_INDEX );
					Debug.Assert( joint.localIndex == i );
				}
			}

			// Validate islands
			{
				Debug.Assert( set.islandSims.count >= 0 );
				totalIslandCount += set.islandSims.count;
				for ( int i = 0; i < set.islandSims.count; ++i )
				{
					b2IslandSim* islandSim = set.islandSims.data + i;
					b2Island* island = Array_Get( &world.islands, islandSim.islandId );
					Debug.Assert( island.setIndex == setIndex );
					Debug.Assert( island.localIndex == i );
				}
			}
		}
		else
		{
			Debug.Assert( set.bodySims.count == 0 );
			Debug.Assert( set.contactSims.count == 0 );
			Debug.Assert( set.jointSims.count == 0 );
			Debug.Assert( set.islandSims.count == 0 );
			Debug.Assert( set.bodyStates.count == 0 );
		}
	}

	int setIdCount = b2GetIdCount( &world.solverSetIdPool );
	Debug.Assert( activeSetCount == setIdCount );

	int bodyIdCount = b2GetIdCount( &world.bodyIdPool );
	Debug.Assert( totalBodyCount == bodyIdCount );

	int islandIdCount = b2GetIdCount( &world.islandIdPool );
	Debug.Assert( totalIslandCount == islandIdCount );

	// Validate constraint graph
	for ( int colorIndex = 0; colorIndex < B2_GRAPH_COLOR_COUNT; ++colorIndex )
	{
		b2GraphColor* color = world.constraintGraph.colors + colorIndex;
		{
			Debug.Assert( color.contactSims.count >= 0 );
			totalContactCount += color.contactSims.count;
			for ( int i = 0; i < color.contactSims.count; ++i )
			{
				b2ContactSim* contactSim = color.contactSims.data + i;
				b2Contact* contact = Array_Get( &world.contacts, contactSim.contactId );
				// contact should be touching in the constraint graph or awaiting transfer to non-touching
				Debug.Assert( contactSim.manifold.pointCount > 0 ||
						   ( contactSim.simFlags & ( b2_simStoppedTouching | b2_simDisjoint ) ) != 0 );
				Debug.Assert( contact.setIndex == (int)b2SetType.b2_awakeSet );
				Debug.Assert( contact.colorIndex == colorIndex );
				Debug.Assert( contact.localIndex == i );

				int bodyIdA = contact.edges[0].bodyId;
				int bodyIdB = contact.edges[1].bodyId;

				if ( colorIndex < B2_OVERFLOW_INDEX )
				{
					b2Body* bodyA = Array_Get( &world.bodies, bodyIdA );
					b2Body* bodyB = Array_Get( &world.bodies, bodyIdB );
					Debug.Assert( b2GetBit( &color.bodySet, bodyIdA ) == ( bodyA.type != b2BodyType.b2_staticBody ) );
					Debug.Assert( b2GetBit( &color.bodySet, bodyIdB ) == ( bodyB.type != b2BodyType.b2_staticBody ) );
				}
			}
		}

		{
			Debug.Assert( color.jointSims.count >= 0 );
			totalJointCount += color.jointSims.count;
			for ( int i = 0; i < color.jointSims.count; ++i )
			{
				b2JointSim* jointSim = color.jointSims.data + i;
				b2Joint* joint = Array_Get( &world.joints, jointSim.jointId );
				Debug.Assert( joint.setIndex == (int)b2SetType.b2_awakeSet );
				Debug.Assert( joint.colorIndex == colorIndex );
				Debug.Assert( joint.localIndex == i );

				int bodyIdA = joint.edges[0].bodyId;
				int bodyIdB = joint.edges[1].bodyId;

				if ( colorIndex < B2_OVERFLOW_INDEX )
				{
					b2Body* bodyA = Array_Get( &world.bodies, bodyIdA );
					b2Body* bodyB = Array_Get( &world.bodies, bodyIdB );
					Debug.Assert( b2GetBit( &color.bodySet, bodyIdA ) == ( bodyA.type != b2BodyType.b2_staticBody ) );
					Debug.Assert( b2GetBit( &color.bodySet, bodyIdB ) == ( bodyB.type != b2BodyType.b2_staticBody ) );
				}
			}
		}
	}

	int contactIdCount = b2GetIdCount( &world.contactIdPool );
	Debug.Assert( totalContactCount == contactIdCount );
	Debug.Assert( totalContactCount == (int)world.broadPhase.pairSet.count );

	int jointIdCount = b2GetIdCount( &world.jointIdPool );
	Debug.Assert( totalJointCount == jointIdCount );

// Validate shapes
// This is very slow on compounds
#if ZERO_DEFINE
	int shapeCapacity = b2Array(world.shapeArray).count;
	for (int shapeIndex = 0; shapeIndex < shapeCapacity; shapeIndex += 1)
	{
		b2Shape* shape = world.shapeArray + shapeIndex;
		if (shape.id != shapeIndex)
		{
			continue;
		}

		Debug.Assert(0 <= shape.bodyId && shape.bodyId < b2Array(world.bodyArray).count);

		b2Body* body = world.bodyArray + shape.bodyId;
		Debug.Assert(0 <= body.setIndex && body.setIndex < b2Array(world.solverSetArray).count);

		b2SolverSet* set = world.solverSetArray + body.setIndex;
		Debug.Assert(0 <= body.localIndex && body.localIndex < set.sims.count);

		b2BodySim* bodySim = set.sims.data + body.localIndex;
		Debug.Assert(bodySim.bodyId == shape.bodyId);

		bool found = false;
		int shapeCount = 0;
		int index = body.headShapeId;
		while (index != B2_NULL_INDEX)
		{
			b2CheckId(world.shapeArray, index);
			b2Shape* s = world.shapeArray + index;
			if (index == shapeIndex)
			{
				found = true;
			}

			index = s.nextShapeId;
			shapeCount += 1;
		}

		Debug.Assert(found);
		Debug.Assert(shapeCount == body.shapeCount);
	}
#endif
}

// Validate contact touching status.
void b2ValidateContacts( b2World* world )
{
	int contactCount = world.contacts.count;
	Debug.Assert( contactCount == b2GetIdCapacity( &world.contactIdPool ) );
	int allocatedContactCount = 0;

	for ( int contactIndex = 0; contactIndex < contactCount; ++contactIndex )
	{
		b2Contact* contact = Array_Get( &world.contacts, contactIndex );
		if ( contact.contactId == B2_NULL_INDEX )
		{
			continue;
		}

		Debug.Assert( contact.contactId == contactIndex );

		allocatedContactCount += 1;

		bool touching = ( contact.flags & b2ContactFlags.b2_contactTouchingFlag ) != 0;

		int setId = contact.setIndex;

		if ( setId == (int)b2SetType.b2_awakeSet )
		{
			// If touching and not a sensor
			if ( touching )
			{
				Debug.Assert( 0 <= contact.colorIndex && contact.colorIndex < B2_GRAPH_COLOR_COUNT );
			}
			else
			{
				Debug.Assert( contact.colorIndex == B2_NULL_INDEX );
			}
		}
		else if ( setId >= (int)b2SetType.b2_firstSleepingSet )
		{
			// Only touching contacts allowed in a sleeping set
			Debug.Assert( touching == true );
		}
		else
		{
			// Sleeping and non-touching contacts or sensor contacts belong in the disabled set
			Debug.Assert( touching == false && setId == (int)b2SetType.b2_disabledSet );
		}

		b2ContactSim* contactSim = b2GetContactSim( world, contact );
		Debug.Assert( contactSim.contactId == contactIndex );
		Debug.Assert( contactSim.bodyIdA == contact.edges[0].bodyId );
		Debug.Assert( contactSim.bodyIdB == contact.edges[1].bodyId );

		// Sim touching is true for solid and sensor contacts
		bool simTouching = ( contactSim.simFlags & b2ContactSimFlags.b2_simTouchingFlag ) != 0;
		Debug.Assert( touching == simTouching );

		Debug.Assert( 0 <= contactSim.manifold.pointCount && contactSim.manifold.pointCount <= 2 );
	}

	int contactIdCount = b2GetIdCount( &world.contactIdPool );
	Debug.Assert( allocatedContactCount == contactIdCount );
}

#else

public static void b2ValidateConnectivity( b2World world )
{
	B2_UNUSED( world );
}

public static void b2ValidateSolverSets( b2World world )
{
	B2_UNUSED( world );
}

public static void b2ValidateContacts( b2World world )
{
	B2_UNUSED( world );
}

#endif

}