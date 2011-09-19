/*
 * See Box2D.js
 */
goog.provide('Box2D.Dynamics.Contacts.b2ContactConstraint');

goog.require('Box2D.Common.Math.b2Vec2');
goog.require('Box2D.Common.Math.b2Mat22');
goog.require('Box2D.Dynamics.Contacts.b2ContactConstraintPoint');

/**
 * @constructor
 */
Box2D.Dynamics.Contacts.b2ContactConstraint = function() {
    this.localPlaneNormal = new Box2D.Common.Math.b2Vec2(0, 0);
    this.localPoint = new Box2D.Common.Math.b2Vec2(0, 0);
    this.normal = new Box2D.Common.Math.b2Vec2(0, 0);
    this.normalMass = new Box2D.Common.Math.b2Mat22();
    this.K = new Box2D.Common.Math.b2Mat22();
    this.points = [];
    for (var i = 0; i < Box2D.Common.b2Settings.b2_maxManifoldPoints; i++) {
        this.points[i] = new Box2D.Dynamics.Contacts.b2ContactConstraintPoint();
    }
};