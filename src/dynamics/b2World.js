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
 * Original Box2D created by Erin Catto
 * http://www.gphysics.com
 * http://box2d.org/
 * 
 * Box2D was converted to Flash by Boris the Brave, Matt Bush, and John Nesky as Box2DFlash
 * http://www.box2dflash.org/
 * 
 * Box2DFlash was converted from Flash to Javascript by Uli Hecht as box2Dweb
 * http://code.google.com/p/box2dweb/
 * 
 * box2Dweb was modified to utilize Google Closure, as well as other bug fixes, optimizations, and tweaks by Illandril
 * https://github.com/illandril/box2dweb-closure
 */
 
goog.provide('Box2D.Dynamics.b2World');

goog.require('goog.structs.Queue');

goog.require('Box2D.Collision.b2AABB');
goog.require('Box2D.Collision.b2RayCastInput');
goog.require('Box2D.Collision.b2RayCastOutput');
goog.require('Box2D.Collision.Shapes.b2Shape');
goog.require('Box2D.Collision.Shapes.b2EdgeShape');
goog.require('Box2D.Common.b2Color');
goog.require('Box2D.Common.Math.b2Math');
goog.require('Box2D.Common.Math.b2Sweep');
goog.require('Box2D.Common.Math.b2Transform');
goog.require('Box2D.Common.Math.b2Vec2');
goog.require('Box2D.Dynamics.b2Body');
goog.require('Box2D.Dynamics.b2BodyDef');
goog.require('Box2D.Dynamics.b2BodyList');
goog.require('Box2D.Dynamics.b2ContactManager');
goog.require('Box2D.Dynamics.b2DebugDraw');
goog.require('Box2D.Dynamics.b2Island');
goog.require('Box2D.Dynamics.b2TimeStep');
goog.require('Box2D.Dynamics.Contacts.b2ContactSolver');
goog.require('Box2D.Dynamics.Controllers.b2ControllerList');
goog.require('Box2D.Dynamics.Joints.b2Joint');
goog.require('Box2D.Dynamics.Joints.b2DistanceJoint');
goog.require('Box2D.Dynamics.Joints.b2MouseJoint');
goog.require('Box2D.Dynamics.Joints.b2PulleyJoint');

/**
 * @param {!Box2D.Common.Math.b2Vec2} gravity
 * @param {boolean} doSleep
 * @constructor
 */
Box2D.Dynamics.b2World = function(gravity, doSleep) { /** @type {!Box2D.Dynamics.b2ContactManager} */
    this.m_contactManager = new Box2D.Dynamics.b2ContactManager(this);

    /** @type {!Box2D.Dynamics.Contacts.b2ContactSolver} */
    this.m_contactSolver = new Box2D.Dynamics.Contacts.b2ContactSolver();

    /** @type {boolean} */
    this.m_isLocked = false;

    /** @type {boolean} */
    this.m_newFixture = false;

    /** @type {Box2D.Dynamics.b2DestructionListener} */
    this.m_destructionListener = null;

    /** @type {Box2D.Dynamics.b2DebugDraw} */
    this.m_debugDraw = null;

    /**
     * @private
     * @type {!Box2D.Dynamics.b2BodyList}
     */
    this.bodyList = new Box2D.Dynamics.b2BodyList();

    /** @type {Box2D.Dynamics.Contacts.b2Contact} */
    this.m_contactList = null;

    /** @type {Box2D.Dynamics.Joints.b2Joint} */
    this.m_jointList = null;

    /**
     * @private
     * @type {!Box2D.Dynamics.Controllers.b2ControllerList}
     */
    this.controllerList = new Box2D.Dynamics.Controllers.b2ControllerList();
    
    /** @type {number} */
    this.m_jointCount = 0;

    /** @type {boolean} */
    this.m_warmStarting = true;

    /** @type {boolean} */
    this.m_continuousPhysics = true;

    /** @type {boolean} */
    this.m_allowSleep = doSleep;

    /** @type {!Box2D.Common.Math.b2Vec2} */
    this.m_gravity = gravity;

    /** @type {number} */
    this.m_inv_dt0 = 0.0;

    /** @type {!Box2D.Dynamics.b2Body} */
    this.m_groundBody = this.CreateBody(new Box2D.Dynamics.b2BodyDef());
};

/**
 * @const
 * @type {number}
 */
Box2D.Dynamics.b2World.MAX_TOI = 1.0 - 100.0 * Number.MIN_VALUE;

/**
 * @param {!Box2D.Dynamics.b2DestructionListener} listener
 */
Box2D.Dynamics.b2World.prototype.SetDestructionListener = function(listener) {
    this.m_destructionListener = listener;
};

/**
 * @param {!Box2D.Dynamics.b2ContactFilter} filter
 */
Box2D.Dynamics.b2World.prototype.SetContactFilter = function(filter) {
    this.m_contactManager.m_contactFilter = filter;
};

/**
 * @param {!Box2D.Dynamics.b22ContactListener} listener
 */
Box2D.Dynamics.b2World.prototype.SetContactListener = function(listener) {
    this.m_contactManager.m_contactListener = listener;
};

/**
 * @param {!Box2D.Dynamics.b2DebugDraw} debugDraw
 */
Box2D.Dynamics.b2World.prototype.SetDebugDraw = function(debugDraw) {
    this.m_debugDraw = debugDraw;
};

/**
 * @param {!Box2D.Collision.b2DynamicTreeBroadPhase} broadPhase
 */
Box2D.Dynamics.b2World.prototype.SetBroadPhase = function(broadPhase) {
    var oldBroadPhase = this.m_contactManager.m_broadPhase;
    this.m_contactManager.m_broadPhase = broadPhase;
    for (var node = this.bodyList.GetFirstNode(Box2D.Dynamics.b2BodyList.TYPES.allBodies); node; node = node.GetNextNode()) {
        for (var fixtureNode = node.body.GetFixtureList().GetFirstNode(); fixtureNode; fixtureNode = fixtureNode.GetNextNode()) {
            var f = fixtureNode.fixture;
            f.m_proxy = broadPhase.CreateProxy(oldBroadPhase.GetFatAABB(f.m_proxy), f);
        }
    }
};

/**
 * @return {number}
 */
Box2D.Dynamics.b2World.prototype.GetProxyCount = function() {
    return this.m_contactManager.m_broadPhase.GetProxyCount();
};

/**
 * @param {!Box2D.Dynamics.b2BodyDef} def
 * @return {!Box2D.Dynamics.b2Body}
 */
Box2D.Dynamics.b2World.prototype.CreateBody = function(def) {
    Box2D.Common.b2Settings.b2Assert(!this.IsLocked());
    var b = new Box2D.Dynamics.b2Body(def, this);
    this.bodyList.AddBody(b);
    return b;
};

/**
 * @param {!Box2D.Dynamics.b2Body} b
 */
Box2D.Dynamics.b2World.prototype.DestroyBody = function(b) {
    Box2D.Common.b2Settings.b2Assert(!this.IsLocked());
    var jn = b.m_jointList;
    while (jn) {
        var jn0 = jn;
        jn = jn.next;
        if (this.m_destructionListener) {
            this.m_destructionListener.SayGoodbyeJoint(jn0.joint);
        }
        this.DestroyJoint(jn0.joint);
    }
    for (var node = b.GetControllerList().GetFirstNode(); node; node = node.GetNextNode()) {
        node.controller.RemoveBody(b);
    }
    var ce = b.m_contactList;
    while (ce) {
        var ce0 = ce;
        ce = ce.next;
        this.m_contactManager.Destroy(ce0.contact);
    }
    b.m_contactList = null;
    for (var fixtureNode = b.GetFixtureList().GetFirstNode(); fixtureNode; fixtureNode = fixtureNode.GetNextNode()) {
        // Why doesn't this happen in body.DestroyFixture?
        if (this.m_destructionListener) {
            this.m_destructionListener.SayGoodbyeFixture(fixtureNode.fixture);
        }
        b.DestroyFixture(fixtureNode.fixture);
    }
    this.bodyList.RemoveBody(b);
};

/**
 * @param {!Box2D.Dynamics.Joints.b2JointDef} def
 * @return {!Box2D.Dynamics.Joints.b2Joint}
 */
Box2D.Dynamics.b2World.prototype.CreateJoint = function(def) {
    var j = Box2D.Dynamics.Joints.b2Joint.Create(def);
    j.m_prev = null;
    j.m_next = this.m_jointList;
    if (this.m_jointList) {
        this.m_jointList.m_prev = j;
    }
    this.m_jointList = j;
    this.m_jointCount++;
    j.m_edgeA.joint = j;
    j.m_edgeA.other = j.m_bodyB;
    j.m_edgeA.prev = null;
    j.m_edgeA.next = j.m_bodyA.m_jointList;
    if (j.m_bodyA.m_jointList) {
        j.m_bodyA.m_jointList.prev = j.m_edgeA;
    }
    j.m_bodyA.m_jointList = j.m_edgeA;
    j.m_edgeB.joint = j;
    j.m_edgeB.other = j.m_bodyA;
    j.m_edgeB.prev = null;
    j.m_edgeB.next = j.m_bodyB.m_jointList;
    if (j.m_bodyB.m_jointList) {
        j.m_bodyB.m_jointList.prev = j.m_edgeB;
    }
    j.m_bodyB.m_jointList = j.m_edgeB;
    var bodyA = def.bodyA;
    var bodyB = def.bodyB;
    if (!def.collideConnected) {
        var edge = bodyB.GetContactList();
        while (edge) {
            if (edge.other == bodyA) {
                edge.contact.FlagForFiltering();
            }
            edge = edge.next;
        }
    }
    return j;
};

/**
 * @param {!Box2D.Dynamics.Joints.b2Joint} j
 */
Box2D.Dynamics.b2World.prototype.DestroyJoint = function(j) {
    var collideConnected = j.m_collideConnected;
    if (j.m_prev) {
        j.m_prev.m_next = j.m_next;
    }
    if (j.m_next) {
        j.m_next.m_prev = j.m_prev;
    }
    if (j == this.m_jointList) {
        this.m_jointList = j.m_next;
    }
    var bodyA = j.m_bodyA;
    var bodyB = j.m_bodyB;
    bodyA.SetAwake(true);
    bodyB.SetAwake(true);
    if (j.m_edgeA.prev) {
        j.m_edgeA.prev.next = j.m_edgeA.next;
    }
    if (j.m_edgeA.next) {
        j.m_edgeA.next.prev = j.m_edgeA.prev;
    }
    if (j.m_edgeA == bodyA.m_jointList) {
        bodyA.m_jointList = j.m_edgeA.next;
    }
    j.m_edgeA.prev = null;
    j.m_edgeA.next = null;
    if (j.m_edgeB.prev) {
        j.m_edgeB.prev.next = j.m_edgeB.next;
    }
    if (j.m_edgeB.next) {
        j.m_edgeB.next.prev = j.m_edgeB.prev;
    }
    if (j.m_edgeB == bodyB.m_jointList) {
        bodyB.m_jointList = j.m_edgeB.next;
    }
    j.m_edgeB.prev = null;
    j.m_edgeB.next = null;
    this.m_jointCount--;
    if (!collideConnected) {
        var edge = bodyB.GetContactList();
        while (edge) {
            if (edge.other == bodyA) {
                edge.contact.FlagForFiltering();
            }
            edge = edge.next;
        }
    }
};

/**
 * @return {!Box2D.Dynamics.Controllers.b2ControllerList}
 */
Box2D.Dynamics.b2World.prototype.GetControllerList = function() {
    return this.controllerList;
};

/**
 * @param {!Box2D.Dynamics.Controllers.b2Controller} c
 * @return {!Box2D.Dynamics.Controllers.b2Controller}
 */
Box2D.Dynamics.b2World.prototype.AddController = function(c) {
    if (c.m_world != null && c.m_world != this) {
        throw new Error("Controller can only be a member of one world");
    }
    this.controllerList.AddController(c);
    c.m_world = this;
    return c;
};

/**
 * @param {!Box2D.Dynamics.Controllers.b2Controller} c
 */
Box2D.Dynamics.b2World.prototype.RemoveController = function(c) {
    this.controllerList.RemoveController(c);
    c.m_world = null;
};

/**
 * @param {!Box2D.Dynamics.Controllers.b2Controller} controller
 * @return {!Box2D.Dynamics.Controllers.b2Controller}
 */
Box2D.Dynamics.b2World.prototype.CreateController = function(controller) {
    return this.AddController(controller);
};

/**
 * @param {!Box2D.Dynamics.Controllers.b2Controller} controller
 */
Box2D.Dynamics.b2World.prototype.DestroyController = function(controller) {
    controller.Clear();
    this.RemoveController();
};

/**
 * @param {boolean} flag
 */
Box2D.Dynamics.b2World.prototype.SetWarmStarting = function(flag) {
    this.m_warmStarting = flag;
};

/**
 * @param {boolean} flag
 */
Box2D.Dynamics.b2World.prototype.SetContinuousPhysics = function(flag) {
    this.m_continuousPhysics = flag;
};

/**
 * @return {number}
 */
Box2D.Dynamics.b2World.prototype.GetBodyCount = function() {
    return this.bodyList.GetBodyCount();
};

/**
 * @return {number}
 */
Box2D.Dynamics.b2World.prototype.GetJointCount = function() {
    return this.m_jointCount;
};

/**
 * @return {number}
 */
Box2D.Dynamics.b2World.prototype.GetContactCount = function() {
    return this.m_contactManager.m_contactCount;
};

/**
 * @param {!Box2D.Common.Math.b2Vec2} gravity
 */
Box2D.Dynamics.b2World.prototype.SetGravity = function(gravity) {
    this.m_gravity = gravity;
};

/**
 * @return {!Box2D.Common.Math.b2Vec2}
 */
Box2D.Dynamics.b2World.prototype.GetGravity = function() {
    return this.m_gravity;
};

/**
 * @return {!Box2D.Dynamics.b2Body}
 */
Box2D.Dynamics.b2World.prototype.GetGroundBody = function() {
    return this.m_groundBody;
};

/**
 * @param {number} dt
 * @param {number} velocityIterations
 * @param {number} positionIterations
 */
Box2D.Dynamics.b2World.prototype.Step = function(dt, velocityIterations, positionIterations) {
    if (this.m_newFixture) {
        this.m_contactManager.FindNewContacts();
        this.m_newFixture = false;
    }
    this.m_isLocked = true;
    var step = new Box2D.Dynamics.b2TimeStep();
    step.dt = dt;
    step.velocityIterations = velocityIterations;
    step.positionIterations = positionIterations;
    if (dt > 0.0) {
        step.inv_dt = 1.0 / dt;
    } else {
        step.inv_dt = 0.0;
    }
    step.dtRatio = this.m_inv_dt0 * dt;
    step.warmStarting = this.m_warmStarting;
    this.m_contactManager.Collide();
    if (step.dt > 0.0) {
        this.Solve(step);
        if (this.m_continuousPhysics && step.dt > 0.0) {
            this.SolveTOI(step);
        }
        this.m_inv_dt0 = step.inv_dt;
    }
    this.m_isLocked = false;
};

Box2D.Dynamics.b2World.prototype.ClearForces = function() {
    for (var node = this.bodyList.GetFirstNode(Box2D.Dynamics.b2BodyList.TYPES.dynamicBodies); node; node = node.GetNextNode()) {
        node.body.m_force.SetZero();
        node.body.m_torque = 0.0;
    }
};

Box2D.Dynamics.b2World.prototype.DrawDebugData = function() {
    if (this.m_debugDraw === null) {
        return;
    }
    this.m_debugDraw.m_sprite.graphics.clear();
    var flags = this.m_debugDraw.GetFlags();
    if (flags & Box2D.Dynamics.b2DebugDraw.e_shapeBit) {
        var color_inactive = new Box2D.Common.b2Color(0.5, 0.5, 0.3);
        var color_static = new Box2D.Common.b2Color(0.5, 0.9, 0.5);
        var color_kinematic = new Box2D.Common.b2Color(0.5, 0.5, 0.9);
        var color_dynamic_sleeping = new Box2D.Common.b2Color(0.6, 0.6, 0.6);
        var color_dynamic_awake = new Box2D.Common.b2Color(0.9, 0.7, 0.7);
        for (var node = this.bodyList.GetFirstNode(Box2D.Dynamics.b2BodyList.TYPES.allBodies); node; node = node.GetNextNode()) {
            var b = node.body;
            var xf = b.m_xf;
            for (var fixtureNode = b.GetFixtureList().GetFirstNode(); fixtureNode; fixtureNode = fixtureNode.GetNextNode()) {
                var f = fixtureNode.fixture;
                var s = f.GetShape();
                if (!b.IsActive()) {
                    this.DrawShape(s, b.m_xf, color_inactive);
                }
                else if (b.GetType() == Box2D.Dynamics.b2BodyDef.b2_staticBody) {
                    this.DrawShape(s, b.m_xf, color_static);
                }
                else if (b.GetType() == Box2D.Dynamics.b2BodyDef.b2_kinematicBody) {
                    this.DrawShape(s, b.m_xf, color_kinematic);
                }
                else if (!b.IsAwake()) {
                    this.DrawShape(s, b.m_xf, color_dynamic_sleeping);
                }
                else {
                    this.DrawShape(s, b.m_xf, color_dynamic_awake);
                }
            }
        }
    }
    if (flags & Box2D.Dynamics.b2DebugDraw.e_jointBit) {
        for (var j = this.m_jointList; j; j = j.m_next) {
            this.DrawJoint(j);
        }
    }
    if (flags & Box2D.Dynamics.b2DebugDraw.e_controllerBit) {
        for (var node = this.controllerList.GetFirstNode(); node; node = node.GetNextNode()) {
            node.controller.Draw(this.m_debugDraw);
        }
    }
    if (flags & Box2D.Dynamics.b2DebugDraw.e_pairBit) {
        var pairColor = new Box2D.Common.b2Color(0.3, 0.9, 0.9);
        for (var contact = this.m_contactManager.m_contactList; contact; contact = contact.GetNext()) {
            var fixtureA = contact.GetFixtureA();
            var fixtureB = contact.GetFixtureB();
            var cA = fixtureA.GetAABB().GetCenter();
            var cB = fixtureB.GetAABB().GetCenter();
            this.m_debugDraw.DrawSegment(cA, cB, pairColor);
        }
    }
    if (flags & Box2D.Dynamics.b2DebugDraw.e_aabbBit) {
        var aabbColor = new Box2D.Common.b2Color(0.0, 0.0, 0.8);
        for (var node = this.bodyList.GetFirstNode(Box2D.Dynamics.b2BodyList.TYPES.activeBodies); node; node = node.GetNextNode()) {
            var b = node.body;
            for (var fixtureNode = b.GetFixtureList().GetFirstNode(); fixtureNode; fixtureNode = fixtureNode.GetNextNode()) {
                var f = fixtureNode.fixture;
                var aabb = this.m_contactManager.m_broadPhase.GetFatAABB(f.m_proxy);
                var vs = [new Box2D.Common.Math.b2Vec2(aabb.lowerBound.x, aabb.lowerBound.y), new Box2D.Common.Math.b2Vec2(aabb.upperBound.x, aabb.lowerBound.y), new Box2D.Common.Math.b2Vec2(aabb.upperBound.x, aabb.upperBound.y), new Box2D.Common.Math.b2Vec2(aabb.lowerBound.x, aabb.upperBound.y)];
                this.m_debugDraw.DrawPolygon(vs, 4, aabbColor);
            }
        }
    }
    if (flags & Box2D.Dynamics.b2DebugDraw.e_centerOfMassBit) {
        for (var node = this.bodyList.GetFirstNode(Box2D.Dynamics.b2BodyList.TYPES.allBodies); node; node = node.GetNextNode()) {
            var b = node.body;
            Box2D.Dynamics.b2World.s_xf.R = b.m_xf.R;
            Box2D.Dynamics.b2World.s_xf.position = b.GetWorldCenter();
            this.m_debugDraw.DrawTransform(Box2D.Dynamics.b2World.s_xf);
        }
    }
};

/**
 * @param {function(!Box2D.Dynamics.b2Fixture):boolean} callback
 * @param {!Box2D.Collision.b2AABB} aabb
 */
Box2D.Dynamics.b2World.prototype.QueryAABB = function(callback, aabb) {
    var broadPhase = this.m_contactManager.m_broadPhase;

    var WorldQueryWrapper = function(fixture) {
            return callback(fixture);
        };
    broadPhase.Query(WorldQueryWrapper, aabb);
};

/**
 * @param {function(!Box2D.Dynamics.b2Fixture): boolean} callback
 * @param {!Box2D.Common.Math.b2Vec2} p
 */
Box2D.Dynamics.b2World.prototype.QueryPoint = function(callback, p) {
    /** @type {function(!Box2D.Dynamics.b2Fixture): boolean} */
    var WorldQueryWrapper = function(fixture) {
            if (fixture.TestPoint(p)) {
                return callback(fixture);
            } else {
                return true;
            }
        };
    var aabb = new Box2D.Collision.b2AABB();
    aabb.lowerBound.Set(p.x - Box2D.Common.b2Settings.b2_linearSlop, p.y - Box2D.Common.b2Settings.b2_linearSlop);
    aabb.upperBound.Set(p.x + Box2D.Common.b2Settings.b2_linearSlop, p.y + Box2D.Common.b2Settings.b2_linearSlop);
    this.m_contactManager.m_broadPhase.Query(WorldQueryWrapper, aabb);
};

/**
 * @param {function(!Box2D.Dynamics.b2Fixture, !Box2D.Common.Math.b2Vec2, !Box2D.Common.Math.b2Vec2, number): number} callback
 * @param {!Box2D.Common.Math.b2Vec2} point1
 * @param {!Box2D.Common.Math.b2Vec2} point2
 */
Box2D.Dynamics.b2World.prototype.RayCast = function(callback, point1, point2) {
    var broadPhase = this.m_contactManager.m_broadPhase;
    var output = new Box2D.Collision.b2RayCastOutput();

    /**
     * @param {!Box2D.Collision.b2RayCastInput} input
     * @param {!Box2D.Dynamics.b2Fixture} fixture
     */
    var RayCastWrapper = function(input, fixture) {
            var hit = fixture.RayCast(output, input);
            if (hit) {
                var flipFrac = 1 - output.fraction;
                var point = new Box2D.Common.Math.b2Vec2(flipFrac * point1.x + output.fraction * point2.x, flipFrac * point1.y + output.fraction * point2.y);
                return callback(fixture, point, output.normal, output.fraction);
            } else {
                return input.maxFraction;
            }
        };
    var input = new Box2D.Collision.b2RayCastInput(point1, point2, 1 /* maxFraction */ );
    broadPhase.RayCast(RayCastWrapper, input);
};

/**
 * @param {!Box2D.Common.Math.b2Vec2} point1
 * @param {!Box2D.Common.Math.b2Vec2} point2
 * @return {Box2D.Dynamics.b2Fixture}
 */
Box2D.Dynamics.b2World.prototype.RayCastOne = function(point1, point2) {
    var result = null;
    /**
     * @param {!Box2D.Dynamics.b2Fixture} fixture
     * @param {!Box2D.Common.Math.b2Vec2} point
     * @param {!Box2D.Common.Math.b2Vec2} normal
     * @param {number} fraction
     * @return {number}
     */
    var RayCastOneWrapper = function(fixture, point, normal, fraction) {
            result = fixture;
            return fraction;
        };
    this.RayCast(RayCastOneWrapper, point1, point2);
    return result;
};

/**
 * @param {!Box2D.Common.Math.b2Vec2} point1
 * @param {!Box2D.Common.Math.b2Vec2} point2
 * @return {Array.<Box2D.Dynamics.b2Fixture>}
 */
Box2D.Dynamics.b2World.prototype.RayCastAll = function(point1, point2) {
    var result = [];

    /**
     * @param {!Box2D.Dynamics.b2Fixture} fixture
     * @param {!Box2D.Common.Math.b2Vec2} point
     * @param {!Box2D.Common.Math.b2Vec2} normal
     * @param {number} fraction
     * @return {number}
     */
    var RayCastAllWrapper = function(fixture, point, normal, fraction) {
            result.push(fixture);
            return 1;
        };
    this.RayCast(RayCastAllWrapper, point1, point2);
    return result;
};

/**
 * @return {!Box2D.Dynamics.b2BodyList}
 */
Box2D.Dynamics.b2World.prototype.GetBodyList = function() {
    return this.bodyList;
};

/**
 * @return {Box2D.Dynamics.Joints.b2Joint}
 */
Box2D.Dynamics.b2World.prototype.GetJointList = function() {
    return this.m_jointList;
};

/**
 * @return {Box2D.Dynamics.Contacts.b2Contact}
 */
Box2D.Dynamics.b2World.prototype.GetContactList = function() {
    return this.m_contactList;
};

/**
 * @return {boolean}
 */
Box2D.Dynamics.b2World.prototype.IsLocked = function() {
    return this.m_isLocked;
};

/**
 * @param {!Box2D.Dynamics.b2TimeStep} step
 */
Box2D.Dynamics.b2World.prototype.Solve = function(step) {
    for (var node = this.controllerList.GetFirstNode(); node; node = node.GetNextNode()) {
        node.controller.Step(step);
    }
    var m_island = new Box2D.Dynamics.b2Island(this.m_contactManager.m_contactListener, this.m_contactSolver);
    
    for (var node = this.bodyList.GetFirstNode(Box2D.Dynamics.b2BodyList.TYPES.allBodies); node; node = node.GetNextNode()) {
        node.body.m_islandFlag = false;
    }
    for (var c = this.m_contactList; c; c = c.m_next) {
        c.m_islandFlag = false;
    }
    for (var j = this.m_jointList; j; j = j.m_next) {
        j.m_islandFlag = false;
    }
    
    for (var node = this.bodyList.GetFirstNode(Box2D.Dynamics.b2BodyList.TYPES.nonStaticActiveAwakeBodies); node; node = node.GetNextNode()) {
        var seed = node.body;
        if (seed.m_islandFlag) {
            continue;
        }
        //if (!seed.IsAwake() || !seed.IsActive()) {
        //    continue;
        //}
        m_island.Clear();
        var stack = [];
        stack.push(seed);
        seed.m_islandFlag = true;
        while (stack.length > 0) {
            var b = stack.pop();
            m_island.AddBody(b);
            if (!b.IsAwake()) {
                b.SetAwake(true);
            }
            if (b.GetType() == Box2D.Dynamics.b2BodyDef.b2_staticBody) {
                continue;
            }
            for (var ce = b.m_contactList; ce; ce = ce.next) {
                if (ce.contact.m_islandFlag || ce.contact.IsSensor() || !ce.contact.IsEnabled() || !ce.contact.IsTouching()) {
                    continue;
                }
                m_island.AddContact(ce.contact);
                ce.contact.m_islandFlag = true;
                if (ce.other.m_islandFlag) {
                    continue;
                }
                stack.push(ce.other);
                ce.other.m_islandFlag = true;
            }
            for (var jn = b.m_jointList; jn; jn = jn.next) {
                if (jn.joint.m_islandFlag || !jn.other.IsActive()) {
                    continue;
                }
                m_island.AddJoint(jn.joint);
                jn.joint.m_islandFlag = true;
                if (jn.other.m_islandFlag) {
                    continue;
                }
                stack.push(jn.other);
                jn.other.m_islandFlag = true;
            }
        }
        m_island.Solve(step, this.m_gravity, this.m_allowSleep);
    }
    for (var node = this.bodyList.GetFirstNode(Box2D.Dynamics.b2BodyList.TYPES.nonStaticActiveAwakeBodies); node; node = node.GetNextNode()) {
        node.body.SynchronizeFixtures();
    }
    this.m_contactManager.FindNewContacts();
};

/**
 * @param {!Box2D.Dynamics.b2TimeStep} step
 */
Box2D.Dynamics.b2World.prototype.SolveTOI = function(step) {
    var m_island = new Box2D.Dynamics.b2Island(this.m_contactManager.m_contactListener, this.m_contactSolver);
    for (var node = this.bodyList.GetFirstNode(Box2D.Dynamics.b2BodyList.TYPES.allBodies); node; node = node.GetNextNode()) {
        var b = node.body;
        b.m_islandFlag = false;
        b.m_sweep.t0 = 0.0;
    }
    for (var c = this.m_contactList; c; c = c.m_next) {
        c.m_islandFlag = false;
        c.m_toi = null;
    }
    for (var j = this.m_jointList; j; j = j.m_next) {
        j.m_islandFlag = false;
    }
    while (true) {
        var toi2 = this._SolveTOI2(step);
        var minContact = toi2.minContact;
        var minTOI = toi2.minTOI;
        if (minContact === null || Box2D.Dynamics.b2World.MAX_TOI < minTOI) {
            break;
        }
        var fixtureABody = minContact.m_fixtureA.GetBody();
        var fixtureBBody =  minContact.m_fixtureB.GetBody();
        Box2D.Dynamics.b2World.s_backupA.Set(fixtureABody.m_sweep);
        Box2D.Dynamics.b2World.s_backupB.Set(fixtureBBody.m_sweep);
        fixtureABody.Advance(minTOI);
        fixtureBBody.Advance(minTOI);
        minContact.Update(this.m_contactManager.m_contactListener);
        minContact.m_toi = null;
        if (minContact.IsSensor() || !minContact.IsEnabled()) {
            fixtureABody.m_sweep.Set(Box2D.Dynamics.b2World.s_backupA);
            fixtureBBody.m_sweep.Set(Box2D.Dynamics.b2World.s_backupB);
            fixtureABody.SynchronizeTransform();
            fixtureBBody.SynchronizeTransform();
            continue;
        }
        if (!minContact.IsTouching()) {
            continue;
        }
        var seed = fixtureABody;
        if (seed.GetType() != Box2D.Dynamics.b2BodyDef.b2_dynamicBody) {
            seed = fixtureBBody;
        }
        m_island.Clear();
        var queue = new goog.structs.Queue();
        queue.enqueue(seed);
        seed.m_islandFlag = true;
        while (queue.size > 0) {
            var b = queue.dequeue();
            m_island.AddBody(b);
            if (!b.IsAwake()) {
                b.SetAwake(true);
            }
            if (b.GetType() != Box2D.Dynamics.b2BodyDef.b2_dynamicBody) {
                continue;
            }
            for (var cEdge = b.m_contactList; cEdge; cEdge = cEdge.next) {
                if (m_island.m_contactCount == Box2D.Common.b2Settings.b2_maxTOIContactsPerIsland) {
                    break;
                }
                if (cEdge.contact.m_islandFlag || cEdge.contact.IsSensor() || !cEdge.contact.IsEnabled() || !cEdge.contact.IsTouching()) {
                    continue;
                }
                m_island.AddContact(cEdge.contact);
                cEdge.contact.m_islandFlag = true;
                if (cEdge.other.m_islandFlag) {
                    continue;
                }
                if (cEdge.other.GetType() != Box2D.Dynamics.b2BodyDef.b2_staticBody) {
                    cEdge.other.Advance(minTOI);
                    cEdge.other.SetAwake(true);
                    queue.enqueue(cEdge.other);
                }
                cEdge.other.m_islandFlag = true;
            }
            for (var jEdge = b.m_jointList; jEdge; jEdge = jEdge.next) {
                if (m_island.m_jointCount == Box2D.Common.b2Settings.b2_maxTOIJointsPerIsland) {
                    continue;
                }
                if (jEdge.joint.m_islandFlag || !jEdge.other.IsActive()) {
                    continue;
                }
                m_island.AddJoint(jEdge.joint);
                jEdge.joint.m_islandFlag = true;
                if (jEdge.other.m_islandFlag) {
                    continue;
                }
                if (jEdge.other.GetType() != Box2D.Dynamics.b2BodyDef.b2_staticBody) {
                    jEdge.other.Advance(minTOI);
                    jEdge.other.SetAwake(true);
                    queue.enqueue(jEdge.other);
                }
                jEdge.other.m_islandFlag = true;
            }
        }
        var subStep = new Box2D.Dynamics.b2TimeStep();
        subStep.warmStarting = false;
        subStep.dt = (1.0 - minTOI) * step.dt;
        subStep.inv_dt = 1.0 / subStep.dt;
        subStep.dtRatio = 0.0;
        subStep.velocityIterations = step.velocityIterations;
        subStep.positionIterations = step.positionIterations;
        m_island.SolveTOI(subStep);

        for (var i = 0; i < m_island.m_bodies.length; i++) {
            m_island.m_bodies[i].m_islandFlag = false;
            if (!m_island.m_bodies[i].IsAwake() || m_island.m_bodies[i].GetType() != Box2D.Dynamics.b2BodyDef.b2_dynamicBody) {
                continue;
            }
            m_island.m_bodies[i].SynchronizeFixtures();
            for (var cEdge = m_island.m_bodies[i].m_contactList; cEdge; cEdge = cEdge.next) {
                cEdge.contact.m_toi = null;
            }
        }
        for (var i = 0; i < m_island.m_contactCount; i++) {
            m_island.m_contacts[i].m_islandFlag = false;
            m_island.m_contacts[i].m_toi = null;
        }
        for (var i = 0; i < m_island.m_jointCount; i++) {
            m_island.m_joints[i].m_islandFlag = false;
        }
        this.m_contactManager.FindNewContacts();
    }
};

/**
 * @param {!Box2D.Dynamics.b2TimeStep} step
 */
Box2D.Dynamics.b2World.prototype._SolveTOI2 = function(step) {
    var minContact = null;
    var minTOI = 1.0;
    var contacts = 0;
    for (var c = this.m_contactList; c; c = c.m_next) {
        if (this._SolveTOI2SkipContact(step, c)) {
            continue;
        }
        var toi = 1.0;
        if (c.m_toi != null) {
            toi = c.m_toi;
        } else if (c.IsTouching()) {
            toi = 1;
            c.m_toi = toi;
        } else {
            var fixtureABody = c.m_fixtureA.GetBody();
            var fixtureBBody = c.m_fixtureB.GetBody();
            var t0 = fixtureABody.m_sweep.t0;
            if (fixtureABody.m_sweep.t0 < fixtureBBody.m_sweep.t0) {
                t0 = fixtureBBody.m_sweep.t0;
                fixtureABody.m_sweep.Advance(t0);
            } else if (fixtureBBody.m_sweep.t0 < fixtureABody.m_sweep.t0) {
                t0 = fixtureABody.m_sweep.t0;
                fixtureBBody.m_sweep.Advance(t0);
            }
            toi = c.ComputeTOI(fixtureABody.m_sweep, fixtureBBody.m_sweep);
            Box2D.Common.b2Settings.b2Assert(0.0 <= toi && toi <= 1.0);
            if (toi > 0.0 && toi < 1.0) {
                toi = (1.0 - toi) * t0 + toi;
            }
            c.m_toi = toi;
        }
        if (Number.MIN_VALUE < toi && toi < minTOI) {
            minContact = c;
            minTOI = toi;
        }
    }
    return {
        minContact: minContact,
        minTOI: minTOI
    };
};

/**
 * @param {!Box2D.Dynamics.b2TimeStep} step
 * @param {!Box2D.Dynamics.Contacts.b2Contact} c
 * @return {boolean}
 */
Box2D.Dynamics.b2World.prototype._SolveTOI2SkipContact = function(step, c) {
    if (c.IsSensor() || !c.IsEnabled() || !c.IsContinuous()) {
        return true;
    }
    var fixtureABody = c.m_fixtureA.GetBody();
    var fixtureBBody = c.m_fixtureB.GetBody();
    if ((fixtureABody.GetType() != Box2D.Dynamics.b2BodyDef.b2_dynamicBody || !fixtureABody.IsAwake()) && (fixtureBBody.GetType() != Box2D.Dynamics.b2BodyDef.b2_dynamicBody || !fixtureBBody.IsAwake())) {
        return true;
    }
    return false;
};

/**
 * @param {!Box2D.Dynamics.Joints.b2Joint} joint
 */
Box2D.Dynamics.b2World.prototype.DrawJoint = function(joint) {
    if (joint instanceof Box2D.Dynamics.Joints.b2DistanceJoint || joint instanceof Box2D.Dynamics.Joints.b2MouseJoint) {
        this.m_debugDraw.DrawSegment(joint.GetAnchorA(), joint.GetAnchorB(), Box2D.Dynamics.b2World.s_jointColor);
    } else if (joint instanceof Box2D.Dynamics.Joints.b2PulleyJoint) {
        this.m_debugDraw.DrawSegment(joint.GetGroundAnchorA(), joint.GetAnchorA(), Box2D.Dynamics.b2World.s_jointColor);
        this.m_debugDraw.DrawSegment(joint.GetGroundAnchorB(), joint.GetAnchorB(), Box2D.Dynamics.b2World.s_jointColor);
        this.m_debugDraw.DrawSegment(joint.GetGroundAnchorA(), joint.GetGroundAnchorB(), Box2D.Dynamics.b2World.s_jointColor);
    } else {
        if (joint.GetBodyA() != this.m_groundBody) {
            this.m_debugDraw.DrawSegment(joint.GetBodyA().m_xf.position, joint.GetAnchorA(), Box2D.Dynamics.b2World.s_jointColor);
        }
        this.m_debugDraw.DrawSegment(joint.GetAnchorA(), joint.GetAnchorB(), Box2D.Dynamics.b2World.s_jointColor);
        if (joint.GetBodyB() != this.m_groundBody) {
            this.m_debugDraw.DrawSegment(joint.GetBodyB().m_xf.position, joint.GetAnchorB(), Box2D.Dynamics.b2World.s_jointColor);
        }
    }
};

/**
 * @param {!Box2D.Collision.Shapes.b2Shape} shape
 * @param {!Box2D.Common.Math.b2Transform} xf
 * @param {!Box2D.Common.b2Color} color
 */
Box2D.Dynamics.b2World.prototype.DrawShape = function(shape, xf, color) {
    if (shape instanceof Box2D.Collision.Shapes.b2CircleShape) {
        var circle = shape;
        var center = Box2D.Common.Math.b2Math.MulX(xf, circle.m_p);
        var radius = circle.m_radius;
        var axis = xf.R.col1;
        this.m_debugDraw.DrawSolidCircle(center, radius, axis, color);
    } else if (shape instanceof Box2D.Collision.Shapes.b2PolygonShape) {
        var i = 0;
        var poly = shape;
        var vertexCount = poly.GetVertexCount();
        var localVertices = poly.GetVertices();
        var vertices = [];
        for (i = 0; i < vertexCount; i++) {
            vertices[i] = Box2D.Common.Math.b2Math.MulX(xf, localVertices[i]);
        }
        this.m_debugDraw.DrawSolidPolygon(vertices, vertexCount, color);
    } else if (shape instanceof Box2D.Collision.Shapes.b2EdgeShape) {
        var edge = shape;
        this.m_debugDraw.DrawSegment(Box2D.Common.Math.b2Math.MulX(xf, edge.GetVertex1()), Box2D.Common.Math.b2Math.MulX(xf, edge.GetVertex2()), color);
    }
};

/** @type {!Box2D.Common.Math.b2Transform} */
Box2D.Dynamics.b2World.s_xf = new Box2D.Common.Math.b2Transform();

/** @type {!Box2D.Common.Math.b2Sweep} */
Box2D.Dynamics.b2World.s_backupA = new Box2D.Common.Math.b2Sweep();

/** @type {!Box2D.Common.Math.b2Sweep} */
Box2D.Dynamics.b2World.s_backupB = new Box2D.Common.Math.b2Sweep();

/** @type {!Box2D.Common.b2Color} */
Box2D.Dynamics.b2World.s_jointColor = new Box2D.Common.b2Color(0.5, 0.8, 0.8);
