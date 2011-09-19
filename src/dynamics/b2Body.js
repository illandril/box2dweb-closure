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
 
goog.provide('Box2D.Dynamics.b2Body');

goog.require('Box2D.Common.b2Settings');
goog.require('Box2D.Common.Math.b2Transform');
goog.require('Box2D.Common.Math.b2Sweep');
goog.require('Box2D.Common.Math.b2Vec2');
goog.require('Box2D.Common.Math.b2Math');
goog.require('Box2D.Dynamics.b2BodyDef');
goog.require('Box2D.Dynamics.b2Fixture');
goog.require('Box2D.Dynamics.b2FixtureDef');

/**
 * @param {!Box2D.Dynamics.b2BodyDef} bd
 * @param {!Box2D.Dynamics.b2World} world
 * @constructor
 */
Box2D.Dynamics.b2Body = function(bd, world) {
    this.m_xf = new Box2D.Common.Math.b2Transform();
    this.m_sweep = new Box2D.Common.Math.b2Sweep();
    this.m_linearVelocity = new Box2D.Common.Math.b2Vec2(0, 0);
    this.m_force = new Box2D.Common.Math.b2Vec2(0, 0);
    this.m_flags = 0;
    if (bd.bullet) {
        this.m_flags |= Box2D.Dynamics.b2Body.e_bulletFlag;
    }
    if (bd.fixedRotation) {
        this.m_flags |= Box2D.Dynamics.b2Body.e_fixedRotationFlag;
    }
    this.m_allowSleep = bd.allowSleep;
    this.m_awake = bd.awake;
    if (bd.active) {
        this.m_flags |= Box2D.Dynamics.b2Body.e_activeFlag;
    }
    this.m_world = world;
    this.m_xf.position.SetV(bd.position);
    this.m_xf.R.Set(bd.angle);
    this.m_sweep.localCenter.SetZero();
    this.m_sweep.t0 = 1.0;
    this.m_sweep.a0 = this.m_sweep.a = bd.angle;
    var tMat = this.m_xf.R;
    var tVec = this.m_sweep.localCenter;
    this.m_sweep.c.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
    this.m_sweep.c.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
    this.m_sweep.c.x += this.m_xf.position.x;
    this.m_sweep.c.y += this.m_xf.position.y;
    this.m_sweep.c0.SetV(this.m_sweep.c);
    this.m_jointList = null;
    this.m_controllerList = null;
    this.m_contactList = null;
    this.m_controllerCount = 0;
    this.m_prev = null;
    this.m_next = null;
    this.m_linearVelocity.SetV(bd.linearVelocity);
    this.m_angularVelocity = bd.angularVelocity;
    this.m_linearDamping = bd.linearDamping;
    this.m_angularDamping = bd.angularDamping;
    this.m_force.SetZero();
    this.m_torque = 0.0;
    this.m_sleepTime = 0.0;
    this.m_type = bd.type;
    if (this.m_type == Box2D.Dynamics.b2BodyDef.b2_dynamicBody) {
        this.m_mass = 1.0;
        this.m_invMass = 1.0;
    } else {
        this.m_mass = 0.0;
        this.m_invMass = 0.0;
    }
    this.m_I = 0.0;
    this.m_invI = 0.0;
    this.m_inertiaScale = bd.inertiaScale;
    this.m_fixtureList = null;
    this.m_fixtureCount = 0;
};

Box2D.Dynamics.b2Body.prototype.CreateFixture = function(def) {
    Box2D.Common.b2Settings.b2Assert(!this.m_world.IsLocked());
    var fixture = new Box2D.Dynamics.b2Fixture();
    fixture.Create(this, this.m_xf, def);
    if (this.m_flags & Box2D.Dynamics.b2Body.e_activeFlag) {
        var broadPhase = this.m_world.m_contactManager.m_broadPhase;
        fixture.CreateProxy(broadPhase, this.m_xf);
    }
    fixture.m_next = this.m_fixtureList;
    this.m_fixtureList = fixture;
    this.m_fixtureCount++;
    fixture.m_body = this;
    if (fixture.m_density > 0.0) {
        this.ResetMassData();
    }
    this.m_world.m_newFixture = true;
    return fixture;
};

Box2D.Dynamics.b2Body.prototype.CreateFixture2 = function(shape, density) {
    if (density === undefined) density = 0.0;
    var def = new Box2D.Dynamics.b2FixtureDef();
    def.shape = shape;
    def.density = density;
    return this.CreateFixture(def);
};

Box2D.Dynamics.b2Body.prototype.DestroyFixture = function(fixture) {
    Box2D.Common.b2Settings.b2Assert(!this.m_world.IsLocked());
    var node = this.m_fixtureList;
    var ppF = null;
    var found = false;
    while (node != null) {
        if (node == fixture) {
            if (ppF) ppF.m_next = fixture.m_next;
            else this.m_fixtureList = fixture.m_next;
            found = true;
            break;
        }
        ppF = node;
        node = node.m_next;
    }
    var edge = this.m_contactList;
    while (edge) {
        var c = edge.contact;
        edge = edge.next;
        var fixtureA = c.GetFixtureA();
        var fixtureB = c.GetFixtureB();
        if (fixture == fixtureA || fixture == fixtureB) {
            this.m_world.m_contactManager.Destroy(c);
        }
    }
    if (this.m_flags & Box2D.Dynamics.b2Body.e_activeFlag) {
        var broadPhase = this.m_world.m_contactManager.m_broadPhase;
        fixture.DestroyProxy(broadPhase);
    }
    fixture.Destroy();
    fixture.m_body = null;
    fixture.m_next = null;
    this.m_fixtureCount--;
    this.ResetMassData();
};

/**
 * @param {!Box2D.Common.Math.b2Vec2) position
 * @param {number} angle
 */
Box2D.Dynamics.b2Body.prototype.SetPositionAndAngle = function(position, angle) {
    Box2D.Common.b2Settings.b2Assert(!this.m_world.IsLocked());
    this.m_xf.R.Set(angle);
    this.m_xf.position.SetV(position);
    var tMat = this.m_xf.R;
    var tVec = this.m_sweep.localCenter;
    this.m_sweep.c.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
    this.m_sweep.c.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
    this.m_sweep.c.x += this.m_xf.position.x;
    this.m_sweep.c.y += this.m_xf.position.y;
    this.m_sweep.c0.SetV(this.m_sweep.c);
    this.m_sweep.a0 = this.m_sweep.a = angle;
    var broadPhase = this.m_world.m_contactManager.m_broadPhase;
    for (var f = this.m_fixtureList; f; f = f.m_next) {
        f.Synchronize(broadPhase, this.m_xf, this.m_xf);
    }
    this.m_world.m_contactManager.FindNewContacts();
};

Box2D.Dynamics.b2Body.prototype.SetTransform = function(xf) {
    this.SetPositionAndAngle(xf.position, xf.GetAngle());
};

Box2D.Dynamics.b2Body.prototype.GetTransform = function() {
    return this.m_xf;
};

Box2D.Dynamics.b2Body.prototype.GetPosition = function() {
    return this.m_xf.position;
};

Box2D.Dynamics.b2Body.prototype.SetPosition = function(position) {
    this.SetPositionAndAngle(position, this.GetAngle());
};

Box2D.Dynamics.b2Body.prototype.GetAngle = function() {
    return this.m_sweep.a;
};

Box2D.Dynamics.b2Body.prototype.SetAngle = function(angle) {
    if (angle === undefined) angle = 0;
    this.SetPositionAndAngle(this.GetPosition(), angle);
};

Box2D.Dynamics.b2Body.prototype.GetWorldCenter = function() {
    return this.m_sweep.c;
};

Box2D.Dynamics.b2Body.prototype.GetLocalCenter = function() {
    return this.m_sweep.localCenter;
};

Box2D.Dynamics.b2Body.prototype.SetLinearVelocity = function(v) {
    if (this.m_type == Box2D.Dynamics.b2BodyDef.b2_staticBody) {
        return;
    }
    this.m_linearVelocity.SetV(v);
};

Box2D.Dynamics.b2Body.prototype.GetLinearVelocity = function() {
    return this.m_linearVelocity;
};

Box2D.Dynamics.b2Body.prototype.SetAngularVelocity = function(omega) {
    if (omega === undefined) omega = 0;
    if (this.m_type == Box2D.Dynamics.b2BodyDef.b2_staticBody) {
        return;
    }
    this.m_angularVelocity = omega;
};

Box2D.Dynamics.b2Body.prototype.GetAngularVelocity = function() {
    return this.m_angularVelocity;
};

Box2D.Dynamics.b2Body.prototype.GetDefinition = function() {
    var bd = new Box2D.Dynamics.b2BodyDef();
    bd.type = this.GetType();
    bd.allowSleep = this.m_allowSleep;
    bd.angle = this.GetAngle();
    bd.angularDamping = this.m_angularDamping;
    bd.angularVelocity = this.m_angularVelocity;
    bd.fixedRotation = (this.m_flags & Box2D.Dynamics.b2Body.e_fixedRotationFlag) == Box2D.Dynamics.b2Body.e_fixedRotationFlag;
    bd.bullet = (this.m_flags & Box2D.Dynamics.b2Body.e_bulletFlag) == Box2D.Dynamics.b2Body.e_bulletFlag;
    bd.awake = this.m_awake;
    bd.linearDamping = this.m_linearDamping;
    bd.linearVelocity.SetV(this.GetLinearVelocity());
    bd.position = this.GetPosition();
    return bd;
};

Box2D.Dynamics.b2Body.prototype.ApplyForce = function(force, point) {
    if (this.m_type != Box2D.Dynamics.b2BodyDef.b2_dynamicBody) {
        return;
    }
    if (this.IsAwake() == false) {
        this.SetAwake(true);
    }
    this.m_force.x += force.x;
    this.m_force.y += force.y;
    this.m_torque += ((point.x - this.m_sweep.c.x) * force.y - (point.y - this.m_sweep.c.y) * force.x);
};

Box2D.Dynamics.b2Body.prototype.ApplyTorque = function(torque) {
    if (torque === undefined) torque = 0;
    if (this.m_type != Box2D.Dynamics.b2BodyDef.b2_dynamicBody) {
        return;
    }
    if (this.IsAwake() == false) {
        this.SetAwake(true);
    }
    this.m_torque += torque;
};

Box2D.Dynamics.b2Body.prototype.ApplyImpulse = function(impulse, point) {
    if (this.m_type != Box2D.Dynamics.b2BodyDef.b2_dynamicBody) {
        return;
    }
    if (this.IsAwake() == false) {
        this.SetAwake(true);
    }
    this.m_linearVelocity.x += this.m_invMass * impulse.x;
    this.m_linearVelocity.y += this.m_invMass * impulse.y;
    this.m_angularVelocity += this.m_invI * ((point.x - this.m_sweep.c.x) * impulse.y - (point.y - this.m_sweep.c.y) * impulse.x);
};

Box2D.Dynamics.b2Body.prototype.Split = function(callback) {
    var linearVelocity = this.GetLinearVelocity().Copy();
    var angularVelocity = this.GetAngularVelocity();
    var center = this.GetWorldCenter();
    var body1 = this;
    var body2 = this.m_world.CreateBody(this.GetDefinition());
    var prev;
    for (var f = body1.m_fixtureList; f;) {
        if (callback(f)) {
            var next = f.m_next;
            if (prev) {
                prev.m_next = next;
            } else {
                body1.m_fixtureList = next;
            }
            body1.m_fixtureCount--;
            f.m_next = body2.m_fixtureList;
            body2.m_fixtureList = f;
            body2.m_fixtureCount++;
            f.m_body = body2;
            f = next;
        } else {
            prev = f;
            f = f.m_next;
        }
    }
    body1.ResetMassData();
    body2.ResetMassData();
    var center1 = body1.GetWorldCenter();
    var center2 = body2.GetWorldCenter();
    var velocity1 = Box2D.Common.Math.b2Math.AddVV(linearVelocity, Box2D.Common.Math.b2Math.CrossFV(angularVelocity, Box2D.Common.Math.b2Math.SubtractVV(center1, center)));
    var velocity2 = Box2D.Common.Math.b2Math.AddVV(linearVelocity, Box2D.Common.Math.b2Math.CrossFV(angularVelocity, Box2D.Common.Math.b2Math.SubtractVV(center2, center)));
    body1.SetLinearVelocity(velocity1);
    body2.SetLinearVelocity(velocity2);
    body1.SetAngularVelocity(angularVelocity);
    body2.SetAngularVelocity(angularVelocity);
    body1.SynchronizeFixtures();
    body2.SynchronizeFixtures();
    return body2;
};

Box2D.Dynamics.b2Body.prototype.Merge = function(other) {
    for (var fix = other.m_fixtureList; fix; fix = other.m_fixtureList) {
        var next = fix.m_next;
        fix.m_body = this;
        fix.m_next = this.m_fixtureList;
        this.m_fixtureList = fix;
        this.m_fixtureCount++;
        
        other.m_fixtureList = next;
        other.m_fixtureCount--;
    }
    other.ResetMassData();
    this.ResetMassData();
    this.SynchronizeFixtures();
};

Box2D.Dynamics.b2Body.prototype.GetMass = function() {
    return this.m_mass;
};

Box2D.Dynamics.b2Body.prototype.GetInertia = function() {
    return this.m_I;
};

Box2D.Dynamics.b2Body.prototype.GetMassData = function(data) {
    data.mass = this.m_mass;
    data.I = this.m_I;
    data.center.SetV(this.m_sweep.localCenter);
};

Box2D.Dynamics.b2Body.prototype.SetMassData = function(massData) {
    Box2D.Common.b2Settings.b2Assert(this.m_world.IsLocked() == false);
    if (this.m_world.IsLocked() == true) {
        return;
    }
    if (this.m_type != Box2D.Dynamics.b2BodyDef.b2_dynamicBody) {
        return;
    }
    this.m_invMass = 0.0;
    this.m_I = 0.0;
    this.m_invI = 0.0;
    this.m_mass = massData.mass;
    if (this.m_mass <= 0.0) {
        this.m_mass = 1.0;
    }
    this.m_invMass = 1.0 / this.m_mass;
    if (massData.I > 0.0 && (this.m_flags & Box2D.Dynamics.b2Body.e_fixedRotationFlag) == 0) {
        this.m_I = massData.I - this.m_mass * (massData.center.x * massData.center.x + massData.center.y * massData.center.y);
        this.m_invI = 1.0 / this.m_I;
    }
    var oldCenter = this.m_sweep.c.Copy();
    this.m_sweep.localCenter.SetV(massData.center);
    this.m_sweep.c0.SetV(Box2D.Common.Math.b2Math.MulX(this.m_xf, this.m_sweep.localCenter));
    this.m_sweep.c.SetV(this.m_sweep.c0);
    this.m_linearVelocity.x += this.m_angularVelocity * (-(this.m_sweep.c.y - oldCenter.y));
    this.m_linearVelocity.y += this.m_angularVelocity * (+(this.m_sweep.c.x - oldCenter.x));
};

Box2D.Dynamics.b2Body.prototype.ResetMassData = function() {
    this.m_mass = 0.0;
    this.m_invMass = 0.0;
    this.m_I = 0.0;
    this.m_invI = 0.0;
    this.m_sweep.localCenter.SetZero();
    if (this.m_type == Box2D.Dynamics.b2BodyDef.b2_staticBody || this.m_type == Box2D.Dynamics.b2BodyDef.b2_kinematicBody) {
        return;
    }
    var center = new Box2D.Common.Math.b2Vec2(0, 0);
    for (var f = this.m_fixtureList; f; f = f.m_next) {
        if (f.m_density == 0.0) {
            continue;
        }
        var massData = f.GetMassData();
        this.m_mass += massData.mass;
        center.x += massData.center.x * massData.mass;
        center.y += massData.center.y * massData.mass;
        this.m_I += massData.I;
    }
    if (this.m_mass > 0.0) {
        this.m_invMass = 1.0 / this.m_mass;
        center.x *= this.m_invMass;
        center.y *= this.m_invMass;
    } else {
        this.m_mass = 1.0;
        this.m_invMass = 1.0;
    }
    if (this.m_I > 0.0 && (this.m_flags & Box2D.Dynamics.b2Body.e_fixedRotationFlag) == 0) {
        this.m_I -= this.m_mass * (center.x * center.x + center.y * center.y);
        this.m_I *= this.m_inertiaScale;
        Box2D.Common.b2Settings.b2Assert(this.m_I > 0);
        this.m_invI = 1.0 / this.m_I;
    } else {
        this.m_I = 0.0;
        this.m_invI = 0.0;
    }
    var oldCenter = this.m_sweep.c.Copy();
    this.m_sweep.localCenter.SetV(center);
    this.m_sweep.c0.SetV(Box2D.Common.Math.b2Math.MulX(this.m_xf, this.m_sweep.localCenter));
    this.m_sweep.c.SetV(this.m_sweep.c0);
    this.m_linearVelocity.x += this.m_angularVelocity * (-(this.m_sweep.c.y - oldCenter.y));
    this.m_linearVelocity.y += this.m_angularVelocity * (+(this.m_sweep.c.x - oldCenter.x));
};

Box2D.Dynamics.b2Body.prototype.GetWorldPoint = function(localPoint) {
    var A = this.m_xf.R;
    var u = new Box2D.Common.Math.b2Vec2(A.col1.x * localPoint.x + A.col2.x * localPoint.y, A.col1.y * localPoint.x + A.col2.y * localPoint.y);
    u.x += this.m_xf.position.x;
    u.y += this.m_xf.position.y;
    return u;
};

Box2D.Dynamics.b2Body.prototype.GetWorldVector = function(localVector) {
    return Box2D.Common.Math.b2Math.MulMV(this.m_xf.R, localVector);
};

Box2D.Dynamics.b2Body.prototype.GetLocalPoint = function(worldPoint) {
    return Box2D.Common.Math.b2Math.MulXT(this.m_xf, worldPoint);
};

Box2D.Dynamics.b2Body.prototype.GetLocalVector = function(worldVector) {
    return Box2D.Common.Math.b2Math.MulTMV(this.m_xf.R, worldVector);
};

Box2D.Dynamics.b2Body.prototype.GetLinearVelocityFromWorldPoint = function(worldPoint) {
    return new Box2D.Common.Math.b2Vec2(this.m_linearVelocity.x - this.m_angularVelocity * (worldPoint.y - this.m_sweep.c.y), this.m_linearVelocity.y + this.m_angularVelocity * (worldPoint.x - this.m_sweep.c.x));
};

Box2D.Dynamics.b2Body.prototype.GetLinearVelocityFromLocalPoint = function(localPoint) {
    var A = this.m_xf.R;
    var worldPoint = new Box2D.Common.Math.b2Vec2(A.col1.x * localPoint.x + A.col2.x * localPoint.y, A.col1.y * localPoint.x + A.col2.y * localPoint.y);
    worldPoint.x += this.m_xf.position.x;
    worldPoint.y += this.m_xf.position.y;
    return new Box2D.Common.Math.b2Vec2(this.m_linearVelocity.x - this.m_angularVelocity * (worldPoint.y - this.m_sweep.c.y), this.m_linearVelocity.y + this.m_angularVelocity * (worldPoint.x - this.m_sweep.c.x));
};

Box2D.Dynamics.b2Body.prototype.GetLinearDamping = function() {
    return this.m_linearDamping;
};

Box2D.Dynamics.b2Body.prototype.SetLinearDamping = function(linearDamping) {
    if (linearDamping === undefined) linearDamping = 0;
    this.m_linearDamping = linearDamping;
};

Box2D.Dynamics.b2Body.prototype.GetAngularDamping = function() {
    return this.m_angularDamping;
};

Box2D.Dynamics.b2Body.prototype.SetAngularDamping = function(angularDamping) {
    if (angularDamping === undefined) angularDamping = 0;
    this.m_angularDamping = angularDamping;
};

Box2D.Dynamics.b2Body.prototype.SetType = function(type) {
    if (type === undefined) type = 0;
    if (this.m_type == type) {
        return;
    }
    this.m_type = type;
    this.ResetMassData();
    if (this.m_type == Box2D.Dynamics.b2BodyDef.b2_staticBody) {
        this.m_linearVelocity.SetZero();
        this.m_angularVelocity = 0.0;
    }
    this.SetAwake(true);
    this.m_force.SetZero();
    this.m_torque = 0.0;
    for (var ce = this.m_contactList; ce; ce = ce.next) {
        ce.contact.FlagForFiltering();
    }
};

Box2D.Dynamics.b2Body.prototype.GetType = function() {
    return this.m_type;
};

Box2D.Dynamics.b2Body.prototype.SetBullet = function(flag) {
    if (flag) {
        this.m_flags |= Box2D.Dynamics.b2Body.e_bulletFlag;
    } else {
        this.m_flags &= ~Box2D.Dynamics.b2Body.e_bulletFlag;
    }
};

Box2D.Dynamics.b2Body.prototype.IsBullet = function() {
    return (this.m_flags & Box2D.Dynamics.b2Body.e_bulletFlag) == Box2D.Dynamics.b2Body.e_bulletFlag;
};

Box2D.Dynamics.b2Body.prototype.SetSleepingAllowed = function(flag) {
    this.m_allowSleep = flag;
    if (!flag) {
        this.SetAwake(true);
    }
};

Box2D.Dynamics.b2Body.prototype.SetAwake = function(flag) {
    this.m_awake = flag;
    this.m_sleepTime = 0;
    if (!flag) {
        this.m_linearVelocity.SetZero();
        this.m_angularVelocity = 0.0;
        this.m_force.SetZero();
        this.m_torque = 0.0;
    }
};

Box2D.Dynamics.b2Body.prototype.IsAwake = function() {
    return this.m_awake;
};

Box2D.Dynamics.b2Body.prototype.SetFixedRotation = function(fixed) {
    if (fixed) {
        this.m_flags |= Box2D.Dynamics.b2Body.e_fixedRotationFlag;
    } else {
        this.m_flags &= ~Box2D.Dynamics.b2Body.e_fixedRotationFlag;
    }
    this.ResetMassData();
};

Box2D.Dynamics.b2Body.prototype.IsFixedRotation = function() {
    return (this.m_flags & Box2D.Dynamics.b2Body.e_fixedRotationFlag) == Box2D.Dynamics.b2Body.e_fixedRotationFlag;
};

Box2D.Dynamics.b2Body.prototype.SetActive = function(flag) {
    if (flag == this.IsActive()) {
        return;
    }
    var broadPhase;
    var f;
    if (flag) {
        this.m_flags |= Box2D.Dynamics.b2Body.e_activeFlag;
        broadPhase = this.m_world.m_contactManager.m_broadPhase;
        for (f = this.m_fixtureList; f; f = f.m_next) {
            f.CreateProxy(broadPhase, this.m_xf);
        }
    } else {
        this.m_flags &= ~Box2D.Dynamics.b2Body.e_activeFlag;
        broadPhase = this.m_world.m_contactManager.m_broadPhase;
        for (f = this.m_fixtureList; f; f = f.m_next) {
            f.DestroyProxy(broadPhase);
        }
        var ce = this.m_contactList;
        while (ce) {
            var ce0 = ce;
            ce = ce.next;
            this.m_world.m_contactManager.Destroy(ce0.contact);
        }
        this.m_contactList = null;
    }
};

Box2D.Dynamics.b2Body.prototype.IsActive = function() {
    return (this.m_flags & Box2D.Dynamics.b2Body.e_activeFlag) == Box2D.Dynamics.b2Body.e_activeFlag;
};

Box2D.Dynamics.b2Body.prototype.IsSleepingAllowed = function() {
    return this.m_allowSleep;
};

Box2D.Dynamics.b2Body.prototype.GetFixtureList = function() {
    return this.m_fixtureList;
};

Box2D.Dynamics.b2Body.prototype.GetJointList = function() {
    return this.m_jointList;
};

Box2D.Dynamics.b2Body.prototype.GetControllerList = function() {
    return this.m_controllerList;
};

Box2D.Dynamics.b2Body.prototype.GetContactList = function() {
    return this.m_contactList;
};

Box2D.Dynamics.b2Body.prototype.GetNext = function() {
    return this.m_next;
};

Box2D.Dynamics.b2Body.prototype.GetWorld = function() {
    return this.m_world;
};

Box2D.Dynamics.b2Body.prototype.SynchronizeFixtures = function() {
    var xf1 = Box2D.Dynamics.b2Body.s_xf1;
    xf1.R.Set(this.m_sweep.a0);
    var tMat = xf1.R;
    var tVec = this.m_sweep.localCenter;
    xf1.position.x = this.m_sweep.c0.x - (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
    xf1.position.y = this.m_sweep.c0.y - (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
    var f;
    var broadPhase = this.m_world.m_contactManager.m_broadPhase;
    for (f = this.m_fixtureList;
    f; f = f.m_next) {
        f.Synchronize(broadPhase, xf1, this.m_xf);
    }
};

Box2D.Dynamics.b2Body.prototype.SynchronizeTransform = function() {
    this.m_xf.R.Set(this.m_sweep.a);
    var tMat = this.m_xf.R;
    var tVec = this.m_sweep.localCenter;
    this.m_xf.position.x = this.m_sweep.c.x - (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
    this.m_xf.position.y = this.m_sweep.c.y - (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
};

Box2D.Dynamics.b2Body.prototype.ShouldCollide = function(other) {
    if (this.m_type != Box2D.Dynamics.b2BodyDef.b2_dynamicBody && other.m_type != Box2D.Dynamics.b2BodyDef.b2_dynamicBody) {
        return false;
    }
    for (var jn = this.m_jointList; jn; jn = jn.next) {
        if (jn.other == other) if (jn.joint.m_collideConnected == false) {
            return false;
        }
    }
    return true;
};

Box2D.Dynamics.b2Body.prototype.Advance = function(t) {
    if (t === undefined) t = 0;
    this.m_sweep.Advance(t);
    this.m_sweep.c.SetV(this.m_sweep.c0);
    this.m_sweep.a = this.m_sweep.a0;
    this.SynchronizeTransform();
};

Box2D.Dynamics.b2Body.s_xf1 = new Box2D.Common.Math.b2Transform();
Box2D.Dynamics.b2Body.e_bulletFlag = 0x0008;
Box2D.Dynamics.b2Body.e_fixedRotationFlag = 0x0010;
Box2D.Dynamics.b2Body.e_activeFlag = 0x0020;

