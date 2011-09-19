/*
 * See Box2D.js
 */
goog.provide('Box2D.Collision.Shapes.b2EdgeChainDef');

/**
 * @constructor
 */
Box2D.Collision.Shapes.b2EdgeChainDef = function() {
    /** @type {number} */
    this.vertexCount = 0;
    
    /** @type {boolean} */
    this.isALoop = true;
    
    /** @type {Array.<Box2D.Common.Math.b2Vec2} */
    this.vertices = [];
};