/*
 * See Box2D.js
 */
goog.provide('Box2D.Dynamics.Joints.b2PulleyJointDef');

goog.require('Box2D.Dynamics.Joints.b2JointDef');
goog.require('Box2D.Dynamics.Joints.b2Joint');
goog.require('Box2D.Dynamics.Joints.b2PulleyJoint');
goog.require('Box2D.Common.Math.b2Vec2');

/**
 * @constructor
 * @extends {Box2D.Dynamics.Joints.b2JointDef}
 */
Box2D.Dynamics.Joints.b2PulleyJointDef = function() {
    Box2D.Dynamics.Joints.b2JointDef.call(this);
    this.groundAnchorA = new Box2D.Common.Math.b2Vec2(0, 0);
    this.groundAnchorB = new Box2D.Common.Math.b2Vec2(0, 0);
    this.localAnchorA = new Box2D.Common.Math.b2Vec2(0, 0);
    this.localAnchorB = new Box2D.Common.Math.b2Vec2(0, 0);
    this.type = Box2D.Dynamics.Joints.b2Joint.e_pulleyJoint;
    this.groundAnchorA.Set((-1.0), 1.0);
    this.groundAnchorB.Set(1.0, 1.0);
    this.localAnchorA.Set((-1.0), 0.0);
    this.localAnchorB.Set(1.0, 0.0);
    this.lengthA = 0.0;
    this.maxLengthA = 0.0;
    this.lengthB = 0.0;
    this.maxLengthB = 0.0;
    this.ratio = 1.0;
    this.collideConnected = true;
};
goog.inherits(Box2D.Dynamics.Joints.b2PulleyJointDef, Box2D.Dynamics.Joints.b2JointDef);

Box2D.Dynamics.Joints.b2PulleyJointDef.prototype.Initialize = function(bA, bB, gaA, gaB, anchorA, anchorB, r) {
    if (r === undefined) r = 0;
    this.bodyA = bA;
    this.bodyB = bB;
    this.groundAnchorA.SetV(gaA);
    this.groundAnchorB.SetV(gaB);
    this.localAnchorA = this.bodyA.GetLocalPoint(anchorA);
    this.localAnchorB = this.bodyB.GetLocalPoint(anchorB);
    var d1X = anchorA.x - gaA.x;
    var d1Y = anchorA.y - gaA.y;
    this.lengthA = Math.sqrt(d1X * d1X + d1Y * d1Y);
    var d2X = anchorB.x - gaB.x;
    var d2Y = anchorB.y - gaB.y;
    this.lengthB = Math.sqrt(d2X * d2X + d2Y * d2Y);
    this.ratio = r;
    var C = this.lengthA + this.ratio * this.lengthB;
    this.maxLengthA = C - this.ratio * Box2D.Dynamics.Joints.b2PulleyJoint.b2_minPulleyLength;
    this.maxLengthB = (C - Box2D.Dynamics.Joints.b2PulleyJoint.b2_minPulleyLength) / this.ratio;
};

Box2D.Dynamics.Joints.b2PulleyJointDef.prototype.Create = function() {
    return new Box2D.Dynamics.Joints.b2PulleyJoint(this);
};
