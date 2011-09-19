/*
 * See Box2D.js
 */
goog.provide('Box2D.Collision.b2DistanceOutput');

goog.require('Box2D.Common.Math.b2Vec2');

/**
 * @constructor
 */
Box2D.Collision.b2DistanceOutput = function () {
    /** @type {!Box2D.Common.Math.b2Vec2} */
    this.pointA = new Box2D.Common.Math.b2Vec2(0, 0);
    
    /** @type {!Box2D.Common.Math.b2Vec2} */
    this.pointB = new Box2D.Common.Math.b2Vec2(0, 0);
    
    /** @type {number} */
    this.distance = 0;
};