/*
 * See Box2D.js
 */
goog.provide('Box2D.Collision.Shapes.b2MassData');

goog.require('Box2D.Common.Math.b2Vec2');

/**
 * @constructor
 */
Box2D.Collision.Shapes.b2MassData = function() {
    /** @type {number} */
    this.mass = 0;
    
    /** @type {!Box2D.Common.Math.b2Vec2} */
    this.center = new Box2D.Common.Math.b2Vec2(0, 0);
    
    /** @type {number} */
    this.I = 0;
};