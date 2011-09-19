/*
 * See Box2D.js
 */
goog.provide('Box2D.Collision.ClipVertex');

goog.require('Box2D.Common.Math.b2Vec2');
goog.require('Box2D.Collision.b2ContactID');

/**
 * @constructor
 */
Box2D.Collision.ClipVertex = function() {
    this.v = new Box2D.Common.Math.b2Vec2(0, 0);
    this.id = new Box2D.Collision.b2ContactID();
};

Box2D.Collision.ClipVertex.prototype.Set = function(other) {
    this.v.SetV(other.v);
    this.id.Set(other.id);
};