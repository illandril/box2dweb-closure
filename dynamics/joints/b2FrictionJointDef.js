/*
 * See Box2D.js
 */
goog.provide('Box2D.Dynamics.Joints.b2FrictionJointDef');

goog.require('Box2D.Dynamics.Joints.b2JointDef');
goog.require('Box2D.Dynamics.Joints.b2Joint');
goog.require('Box2D.Dynamics.Joints.b2FrictionJoint');
goog.require('Box2D.Common.Math.b2Vec2');

/**
 * @constructor
 * @extends {Box2D.Dynamics.Joints.b2JointDef}
 */
Box2D.Dynamics.Joints.b2FrictionJointDef = function() {
    Box2D.Dynamics.Joints.b2JointDef.call(this);
    this.localAnchorA = new Box2D.Common.Math.b2Vec2(0, 0);
    this.localAnchorB = new Box2D.Common.Math.b2Vec2(0, 0);
    this.type = Box2D.Dynamics.Joints.b2Joint.e_frictionJoint;
    this.maxForce = 0.0;
    this.maxTorque = 0.0;
};
goog.inherits(Box2D.Dynamics.Joints.b2FrictionJointDef, Box2D.Dynamics.Joints.b2JointDef);

Box2D.Dynamics.Joints.b2FrictionJointDef.prototype.Initialize = function (bA, bB, anchor) {
    this.bodyA = bA;
    this.bodyB = bB;
    this.localAnchorA.SetV(this.bodyA.GetLocalPoint(anchor));
    this.localAnchorB.SetV(this.bodyB.GetLocalPoint(anchor));
};

Box2D.Dynamics.Joints.b2FrictionJointDef.prototype.Create = function() {
    return new Box2D.Dynamics.Joints.b2FrictionJoint(this);
};