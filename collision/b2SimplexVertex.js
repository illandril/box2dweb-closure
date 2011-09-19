/*
 * See Box2D.js
 */
goog.provide('Box2D.Collision.b2SimplexVertex');

/**
 * @constructor
 */
Box2D.Collision.b2SimplexVertex = function() {};

Box2D.Collision.b2SimplexVertex.prototype.Set = function(other) {
    this.wA.SetV(other.wA);
    this.wB.SetV(other.wB);
    this.w.SetV(other.w);
    this.a = other.a;
    this.indexA = other.indexA;
    this.indexB = other.indexB;
};