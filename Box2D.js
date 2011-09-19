/*
 * Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
 
/*
 * This copy includes modifications by Illandril for performance optimizations and readability
 * It is split into multiple files:
 *   Box2D (this file - include first)
 *   base (include second)
 *   optimized files (order unimportant, make sure they require base)
 *     collision
 *     dynamics
 *     contacts
 *   unopped (temporary file, containing everything not yet optimized - include second to last - make sure it requires all optimized files)
 *   post (include last, make sure it requires unopped)
 */
 
goog.provide('Box2D');

goog.require('Box2D.Dynamics.b2Body');
goog.require('Box2D.Dynamics.b2BodyDef');
goog.require('Box2D.Dynamics.b2Fixture');
goog.require('Box2D.Dynamics.b2FixtureDef');
goog.require('Box2D.Dynamics.b2World');
goog.require('Box2D.Dynamics.b2ContactFilter');
goog.require('Box2D.Dynamics.b2ContactImpulse');
goog.require('Box2D.Dynamics.b2ContactManager');
goog.require('Box2D.Dynamics.b2DebugDraw');
goog.require('Box2D.Dynamics.b2DestructionListener');
goog.require('Box2D.Dynamics.b2FilterData');
goog.require('Box2D.Dynamics.b2Island');
goog.require('Box2D.Dynamics.b2TimeStep');
goog.require('Box2D.Dynamics.Controllers.b2ControllerEdge');
goog.require('Box2D.Dynamics.Controllers.b2BuoyancyController');
goog.require('Box2D.Dynamics.Controllers.b2ConstantAccelController');
goog.require('Box2D.Dynamics.Controllers.b2ConstantForceController');
goog.require('Box2D.Dynamics.Controllers.b2Controller');
goog.require('Box2D.Dynamics.Controllers.b2GravityController');
goog.require('Box2D.Dynamics.Controllers.b2TensorDampingController');
goog.require('Box2D.Dynamics.Joints.b2DistanceJoint');
goog.require('Box2D.Dynamics.Joints.b2DistanceJointDef');
goog.require('Box2D.Dynamics.Joints.b2FrictionJoint');
goog.require('Box2D.Dynamics.Joints.b2FrictionJointDef');
goog.require('Box2D.Dynamics.Joints.b2GearJoint');
goog.require('Box2D.Dynamics.Joints.b2GearJointDef');
goog.require('Box2D.Dynamics.Joints.b2Jacobian');
goog.require('Box2D.Dynamics.Joints.b2Joint');
goog.require('Box2D.Dynamics.Joints.b2JointDef');
goog.require('Box2D.Dynamics.Joints.b2LineJoint');
goog.require('Box2D.Dynamics.Joints.b2LineJointDef');
goog.require('Box2D.Dynamics.Joints.b2MouseJoint');
goog.require('Box2D.Dynamics.Joints.b2MouseJointDef');
goog.require('Box2D.Dynamics.Joints.b2PrismaticJoint');
goog.require('Box2D.Dynamics.Joints.b2PrismaticJointDef');
goog.require('Box2D.Dynamics.Joints.b2PulleyJoint');
goog.require('Box2D.Dynamics.Joints.b2PulleyJointDef');
goog.require('Box2D.Dynamics.Joints.b2RevoluteJoint');
goog.require('Box2D.Dynamics.Joints.b2RevoluteJointDef');
goog.require('Box2D.Dynamics.Joints.b2WeldJoint');
goog.require('Box2D.Dynamics.Joints.b2WeldJointDef');
goog.require('Box2D.Common.b2Color');
goog.require('Box2D.Collision.Shapes.b2PolygonShape');
goog.require('Box2D.Collision.Shapes.b2MassData');
goog.require('Box2D.defineProperty');
goog.require('Box2D.Collision.Shapes.b2EdgeShape');
goog.require('Box2D.Collision.Shapes.b2EdgeChainDef');
goog.require('Box2D.Collision.Shapes.b2CircleShape');
goog.require('Box2D.Collision.ClipVertex');
goog.require('Box2D.Collision.b2WorldManifold');
goog.require('Box2D.Collision.b2TOIInput');
goog.require('Box2D.Collision.b2TimeOfImpact');
goog.require('Box2D.Collision.b2SimplexVertex');
goog.require('Box2D.Collision.b2Simplex');
goog.require('Box2D.Collision.b2SeparationFunction');
goog.require('Box2D.Collision.Features');
goog.require('Box2D.Collision.b2Segment');
goog.require('Box2D.Collision.b2RayCastOutput');
goog.require('Box2D.Collision.b2RayCastInput');
goog.require('Box2D.Collision.b2Manifold');
goog.require('Box2D.Collision.Shapes.b2Shape');
goog.require('Box2D.Common.b2Settings');
goog.require('Box2D.Collision.b2AABB');
goog.require('Box2D.Collision.b2SimplexCache');
goog.require('Box2D.Collision.b2Collision');
goog.require('Box2D.Collision.b2ContactID');
goog.require('Box2D.Collision.b2ContactPoint');
goog.require('Box2D.Collision.b2Distance');
goog.require('Box2D.Collision.b2DistanceInput');
goog.require('Box2D.Collision.b2DistanceOutput');
goog.require('Box2D.Collision.b2DistanceProxy');
goog.require('Box2D.Collision.b2DynamicTreeBroadPhase');
goog.require('Box2D.Collision.b2DynamicTreeNode');
goog.require('Box2D.Collision.b2DynamicTree');
goog.require('Box2D.Collision.IBroadPhase');
goog.require('Box2D.Common.Math.b2Math');
goog.require('Box2D.Common.Math.b2Transform');
goog.require('Box2D.Common.Math.b2Sweep');
goog.require('Box2D.Common.Math.b2Vec2');
goog.require('Box2D.Common.Math.b2Vec3');
goog.require('Box2D.Common.Math.b2Mat22');
goog.require('Box2D.Common.Math.b2Mat33');
goog.require('Box2D.Dynamics.Contacts.b2CircleContact');
goog.require('Box2D.Dynamics.Contacts.b2Contact');
goog.require('Box2D.Dynamics.Contacts.b2ContactConstraint');
goog.require('Box2D.Dynamics.Contacts.b2ContactConstraintPoint');
goog.require('Box2D.Dynamics.Contacts.b2ContactEdge');
goog.require('Box2D.Dynamics.Contacts.b2ContactFactory');
goog.require('Box2D.Dynamics.Contacts.b2ContactRegister');
goog.require('Box2D.Dynamics.Contacts.b2ContactResult');
goog.require('Box2D.Dynamics.Contacts.b2ContactSolver');
goog.require('Box2D.Dynamics.Contacts.b2EdgeAndCircleContact');
goog.require('Box2D.Dynamics.Contacts.b2PolyAndCircleContact');
goog.require('Box2D.Dynamics.Contacts.b2PolyAndEdgeContact');
goog.require('Box2D.Dynamics.Contacts.b2PolygonContact');
goog.require('Box2D.Dynamics.Contacts.b2PositionSolverManifold');
goog.require('Box2D.generateCallback');
goog.require('Box2D.Queue');
goog.require('Box2D.Consts');
