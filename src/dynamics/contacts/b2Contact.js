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
 
goog.provide('Box2D.Dynamics.Contacts.b2Contact');

goog.require('Box2D.Dynamics.Contacts.b2ContactEdge');
goog.require('Box2D.Collision.b2Manifold');
goog.require('Box2D.Collision.b2TOIInput');
goog.require('Box2D.Collision.b2TimeOfImpact');
goog.require('Box2D.Dynamics.b2BodyDef');
goog.require('Box2D.Collision.Shapes.b2Shape');
goog.require('Box2D.Common.b2Settings');

/**
 * @param {!Box2D.Dynamics.b2Fixture} fixtureA
 * @param {!Box2D.Dynamics.b2Fixture} fixtureB
 * @constructor
 */
Box2D.Dynamics.Contacts.b2Contact = function(fixtureA, fixtureB) {
    /** @type {!Box2D.Dynamics.Contacts.b2ContactEdge} */
    this.m_nodeA = new Box2D.Dynamics.Contacts.b2ContactEdge();
    
    /** @type {!Box2D.Dynamics.Contacts.b2ContactEdge} */
    this.m_nodeB = new Box2D.Dynamics.Contacts.b2ContactEdge();

    /** @type {!Box2D.Collision.b2Manifold} */
    this.m_manifold = new Box2D.Collision.b2Manifold();
    
    /** @type {!Box2D.Collision.b2Manifold} */
    this.m_oldManifold = new Box2D.Collision.b2Manifold();
    
    /** @type {boolean} */
    this.touching = false;

    var bodyA = fixtureA.GetBody();
    var bodyB = fixtureB.GetBody();
    
    /** @type {boolean} */
    this.continuous = (bodyA.GetType() != Box2D.Dynamics.b2BodyDef.b2_dynamicBody) ||
                      bodyA.IsBullet() ||
                      (bodyB.GetType() != Box2D.Dynamics.b2BodyDef.b2_dynamicBody) ||
                      bodyB.IsBullet();
    
    /** @type {boolean} */
    this.sensor = fixtureA.IsSensor() || fixtureB.IsSensor();
    
    /** @type {boolean} */
    this.filtering = false;
    
    /** @type {Box2D.Dynamics.Contacts.b2Contact} */
    this.m_next = null;
    
    /** @type {Box2D.Dynamics.Contacts.b2Contact} */
    this.m_prev = null;
    
    /** @type {!Box2D.Dynamics.b2Fixture} */
    this.m_fixtureA = fixtureA;
    
    /** @type {!Box2D.Dynamics.b2Fixture} */
    this.m_fixtureB = fixtureB;
    
    /** @type {boolean} */
    this.enabled = true;
};

/**
 * @return {!Box2D.Collision.b2Manifold}
 */
Box2D.Dynamics.Contacts.b2Contact.prototype.GetManifold = function () {
  return this.m_manifold;
};

/**
 * @param {!Box2D.Collision.b2WorldManifold} worldManifold
 */
Box2D.Dynamics.Contacts.b2Contact.prototype.GetWorldManifold = function (worldManifold) {
    var bodyA = this.m_fixtureA.GetBody();
    var bodyB = this.m_fixtureB.GetBody();
    var shapeA = this.m_fixtureA.GetShape();
    var shapeB = this.m_fixtureB.GetShape();
    worldManifold.Initialize(this.m_manifold, bodyA.GetTransform(), shapeA.m_radius, bodyB.GetTransform(), shapeB.m_radius);
};

/**
 * @return {boolean}
 */
Box2D.Dynamics.Contacts.b2Contact.prototype.IsTouching = function () {
  return this.touching;
};

/**
 * @return {boolean}
 */
Box2D.Dynamics.Contacts.b2Contact.prototype.IsContinuous = function () {
  return this.continuous;
};

/**
 * @param {boolean} sensor
 */
Box2D.Dynamics.Contacts.b2Contact.prototype.SetSensor = function (sensor) {
   this.sensor = sensor;
};

/**
 * @return {boolean}
 */
Box2D.Dynamics.Contacts.b2Contact.prototype.IsSensor = function () {
  return this.sensor;
};

/**
 * @param {boolean} flag
 */
Box2D.Dynamics.Contacts.b2Contact.prototype.SetEnabled = function (flag) {
   this.enabled = flag;
};

/**
 * @return {boolean}
 */
Box2D.Dynamics.Contacts.b2Contact.prototype.IsEnabled = function () {
   return this.enabled;
};

/**
 * @return {Box2D.Dynamics.Contacts.b2Contact}
 */
Box2D.Dynamics.Contacts.b2Contact.prototype.GetNext = function () {
  return this.m_next;
};

/**
 * @return {!Box2D.Dynamics.b2Fixture}
 */
Box2D.Dynamics.Contacts.b2Contact.prototype.GetFixtureA = function () {
  return this.m_fixtureA;
};

/**
 * @return {!Box2D.Dynamics.b2Fixture}
 */
Box2D.Dynamics.Contacts.b2Contact.prototype.GetFixtureB = function () {
  return this.m_fixtureB;
};

Box2D.Dynamics.Contacts.b2Contact.prototype.FlagForFiltering = function () {
   this.filtering = true;
};

Box2D.Dynamics.Contacts.b2Contact.prototype.ClearFiltering = function () {
   this.filtering = false;
};

/**
 * @return {boolean}
 */
Box2D.Dynamics.Contacts.b2Contact.prototype.IsFiltering = function () {
   return this.filtering;
};

Box2D.Dynamics.Contacts.b2Contact.prototype.Update = function (listener) {
  var tManifold = this.m_oldManifold;
  this.m_oldManifold = this.m_manifold;
  this.m_manifold = tManifold;
  this.enabled = true;
  var touching = false;
  var wasTouching = this.IsTouching();
  var bodyA = this.m_fixtureA.GetBody();
  var bodyB = this.m_fixtureB.GetBody();
  var aabbOverlap = this.m_fixtureA.m_aabb.TestOverlap(this.m_fixtureB.m_aabb);
  if (this.sensor) {
     if (aabbOverlap) {
        touching = Box2D.Collision.Shapes.b2Shape.TestOverlap(this.m_fixtureA.GetShape(), bodyA.GetTransform(), this.m_fixtureB.GetShape(), bodyB.GetTransform());
     }
     this.m_manifold.m_pointCount = 0;
  } else {
     if (bodyA.GetType() != Box2D.Dynamics.b2BodyDef.b2_dynamicBody || bodyA.IsBullet() || bodyB.GetType() != Box2D.Dynamics.b2BodyDef.b2_dynamicBody || bodyB.IsBullet()) {
        this.continuous = true;
     } else {
        this.continuous = false;
     }
     if (aabbOverlap) {
        this.Evaluate();
        touching = this.m_manifold.m_pointCount > 0;
        for (var i = 0; i < this.m_manifold.m_pointCount; i++) {
           var mp2 = this.m_manifold.m_points[i];
           mp2.m_normalImpulse = 0.0;
           mp2.m_tangentImpulse = 0.0;
           for (var j = 0; j < this.m_oldManifold.m_pointCount; j++) {
              var mp1 = this.m_oldManifold.m_points[j];
              if (mp1.m_id.GetKey() == mp2.m_id.GetKey()) {
                 mp2.m_normalImpulse = mp1.m_normalImpulse;
                 mp2.m_tangentImpulse = mp1.m_tangentImpulse;
                 break;
              }
           }
        }
     } else {
        this.m_manifold.m_pointCount = 0;
     }
     if (touching != wasTouching) {
        bodyA.SetAwake(true);
        bodyB.SetAwake(true);
     }
  }
  this.touching = touching;
  
  if (!wasTouching && touching) {
     listener.BeginContact(this);
  }
  if (wasTouching && !touching) {
     listener.EndContact(this);
  }
  if (!this.sensor) {
     listener.PreSolve(this, this.m_oldManifold);
  }
};

Box2D.Dynamics.Contacts.b2Contact.prototype.Evaluate = function () {};

Box2D.Dynamics.Contacts.b2Contact.prototype.ComputeTOI = function (sweepA, sweepB) {
  Box2D.Dynamics.Contacts.b2Contact.s_input.proxyA.Set(this.m_fixtureA.GetShape());
  Box2D.Dynamics.Contacts.b2Contact.s_input.proxyB.Set(this.m_fixtureB.GetShape());
  Box2D.Dynamics.Contacts.b2Contact.s_input.sweepA = sweepA;
  Box2D.Dynamics.Contacts.b2Contact.s_input.sweepB = sweepB;
  Box2D.Dynamics.Contacts.b2Contact.s_input.tolerance = Box2D.Common.b2Settings.b2_linearSlop;
  return Box2D.Collision.b2TimeOfImpact.TimeOfImpact(Box2D.Dynamics.Contacts.b2Contact.s_input);
};

Box2D.Dynamics.Contacts.b2Contact.s_input = new Box2D.Collision.b2TOIInput();
