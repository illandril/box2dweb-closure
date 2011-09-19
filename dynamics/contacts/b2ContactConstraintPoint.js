/*
 * See Box2D.js
 */
goog.provide('Box2D.Dynamics.Contacts.b2ContactConstraintPoint');

goog.require('Box2D.Common.Math.b2Vec2');

/**
 * @constructor
 */
Box2D.Dynamics.Contacts.b2ContactConstraintPoint = function() {
      this.localPoint = new Box2D.Common.Math.b2Vec2(0, 0);
      this.rA = new Box2D.Common.Math.b2Vec2(0, 0);
      this.rB = new Box2D.Common.Math.b2Vec2(0, 0);
};
