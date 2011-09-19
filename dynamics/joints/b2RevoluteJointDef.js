/*
 * See Box2D.js
 */
goog.provide('Box2D.Dynamics.Joints.b2RevoluteJointDef');

goog.require('Box2D.Dynamics.Joints.b2JointDef');
goog.require('Box2D.Dynamics.Joints.b2Joint');
goog.require('Box2D.Dynamics.Joints.b2RevoluteJoint');
goog.require('Box2D.Common.Math.b2Vec2');

/**
 * @constructor
 * @extends {Box2D.Dynamics.Joints.b2JointDef}
 */
Box2D.Dynamics.Joints.b2RevoluteJointDef = function() {
    Box2D.Dynamics.Joints.b2JointDef.call(this);
    this.localAnchorA = new Box2D.Common.Math.b2Vec2(0, 0);
    this.localAnchorB = new Box2D.Common.Math.b2Vec2(0, 0);
    this.type = Box2D.Dynamics.Joints.b2Joint.e_revoluteJoint;
    this.localAnchorA.SetZero();
    this.localAnchorB.SetZero();
    this.referenceAngle = 0.0;
    this.lowerAngle = 0.0;
    this.upperAngle = 0.0;
    this.maxMotorTorque = 0.0;
    this.motorSpeed = 0.0;
    this.enableLimit = false;
    this.enableMotor = false;
};
goog.inherits(Box2D.Dynamics.Joints.b2RevoluteJointDef, Box2D.Dynamics.Joints.b2JointDef);

Box2D.Dynamics.Joints.b2RevoluteJointDef.prototype.Initialize = function(bA, bB, anchor) {
    this.bodyA = bA;
    this.bodyB = bB;
    this.localAnchorA = this.bodyA.GetLocalPoint(anchor);
    this.localAnchorB = this.bodyB.GetLocalPoint(anchor);
    this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
};

Box2D.Dynamics.Joints.b2RevoluteJointDef.prototype.Create = function() {
    return new Box2D.Dynamics.Joints.b2RevoluteJoint(this);
};