/*
 * See Box2D.js
 */
goog.provide('Box2D.Dynamics.Joints.b2MouseJointDef');

goog.require('Box2D.Dynamics.Joints.b2JointDef');
goog.require('Box2D.Dynamics.Joints.b2Joint');
goog.require('Box2D.Dynamics.Joints.b2MouseJoint');
goog.require('Box2D.Common.Math.b2Vec2');

/**
 * @constructor
 * @extends {Box2D.Dynamics.Joints.b2JointDef}
 */
Box2D.Dynamics.Joints.b2MouseJointDef = function() {
    Box2D.Dynamics.Joints.b2JointDef.call(this);
      this.target = new Box2D.Common.Math.b2Vec2(0, 0);
            this.type = Box2D.Dynamics.Joints.b2Joint.e_mouseJoint;
      this.maxForce = 0.0;
      this.frequencyHz = 5.0;
      this.dampingRatio = 0.7;
};
goog.inherits(Box2D.Dynamics.Joints.b2MouseJointDef, Box2D.Dynamics.Joints.b2JointDef);

Box2D.Dynamics.Joints.b2MouseJointDef.prototype.Create = function() {
    return new Box2D.Dynamics.Joints.b2MouseJoint(this);
};