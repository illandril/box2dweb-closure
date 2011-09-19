/*
 * See Box2D.js
 */
goog.provide('Box2D.Dynamics.Joints.b2PrismaticJointDef');

goog.require('Box2D.Dynamics.Joints.b2JointDef');
goog.require('Box2D.Dynamics.Joints.b2Joint');
goog.require('Box2D.Dynamics.Joints.b2PrismaticJoint');
goog.require('Box2D.Common.Math.b2Vec2');

/**
 * @constructor
 * @extends {Box2D.Dynamics.Joints.b2JointDef}
 */
Box2D.Dynamics.Joints.b2PrismaticJointDef = function() {
    Box2D.Dynamics.Joints.b2JointDef.call(this);
    this.localAnchorA = new Box2D.Common.Math.b2Vec2(0, 0);
    this.localAnchorB = new Box2D.Common.Math.b2Vec2(0, 0);
    this.localAxisA = new Box2D.Common.Math.b2Vec2(0, 0);
    this.type = Box2D.Dynamics.Joints.b2Joint.e_prismaticJoint;
    this.localAxisA.Set(1.0, 0.0);
    this.referenceAngle = 0.0;
    this.enableLimit = false;
    this.lowerTranslation = 0.0;
    this.upperTranslation = 0.0;
    this.enableMotor = false;
    this.maxMotorForce = 0.0;
    this.motorSpeed = 0.0;
};
goog.inherits(Box2D.Dynamics.Joints.b2PrismaticJointDef, Box2D.Dynamics.Joints.b2JointDef);

Box2D.Dynamics.Joints.b2PrismaticJointDef.prototype.Initialize = function(bA, bB, anchor, axis) {
    this.bodyA = bA;
    this.bodyB = bB;
    this.localAnchorA = this.bodyA.GetLocalPoint(anchor);
    this.localAnchorB = this.bodyB.GetLocalPoint(anchor);
    this.localAxisA = this.bodyA.GetLocalVector(axis);
    this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
};

Box2D.Dynamics.Joints.b2PrismaticJointDef.prototype.Create = function() {
    return new Box2D.Dynamics.Joints.b2PrismaticJoint(this);
};