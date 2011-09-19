/*
 * See Box2D.js
 */
goog.provide('Box2D.Dynamics.Joints.b2JointDef');

goog.require('Box2D.Dynamics.Joints.b2Joint');

/**
 * @constructor
 */
Box2D.Dynamics.Joints.b2JointDef = function () {
    this.type = Box2D.Dynamics.Joints.b2Joint.e_unknownJoint;
    this.userData = null;
    this.bodyA = null;
    this.bodyB = null;
    this.collideConnected = false;
};