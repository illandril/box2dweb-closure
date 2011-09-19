/*
 * See Box2D.js
 */
goog.provide('Box2D.Dynamics.b2BodyDef');

goog.require('Box2D.Common.Math.b2Vec2');

/**
 * @constructor
 */
Box2D.Dynamics.b2BodyDef = function() {
    this.position = new Box2D.Common.Math.b2Vec2(0, 0);
    this.linearVelocity = new Box2D.Common.Math.b2Vec2(0, 0);
    this.userData = null;
    this.position.SetZero();
    this.angle = 0.0;
    this.linearVelocity.SetZero();
    this.angularVelocity = 0.0;
    this.linearDamping = 0.0;
    this.angularDamping = 0.0;
    this.allowSleep = true;
    this.awake = true;
    this.fixedRotation = false;
    this.bullet = false;
    this.type = Box2D.Dynamics.b2BodyDef.b2_staticBody;
    this.active = true;
    this.inertiaScale = 1.0;
};

Box2D.Dynamics.b2BodyDef.b2_staticBody = 0;
Box2D.Dynamics.b2BodyDef.b2_kinematicBody = 1;
Box2D.Dynamics.b2BodyDef.b2_dynamicBody = 2;