/*
 * See Box2D.js
 */
goog.provide('Box2D.Dynamics.Contacts.b2ContactResult');

goog.require('Box2D.Common.Math.b2Vec2');
goog.require('Box2D.Collision.b2ContactID');

/**
 * @constructor
 */
Box2D.Dynamics.Contacts.b2ContactResult = function() {
    this.position = new Box2D.Common.Math.b2Vec2(0, 0);
    this.normal = new Box2D.Common.Math.b2Vec2(0, 0);
    this.id = new Box2D.Collision.b2ContactID();
};