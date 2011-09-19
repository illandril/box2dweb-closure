/*
 * See Box2D.js
 */
goog.provide('Box2D.Collision.b2DynamicTreePair');

/**
 * @param {!Box2D.Dynamics.b2Fixture} fixtureA
 * @param {!Box2D.Dynamics.b2Fixture} fixtureB
 * @constructor
 */
Box2D.Collision.b2DynamicTreePair = function(fixtureA, fixtureB) {
    /** @type {!Box2D.Dynamics.b2Fixture} */
    this.fixtureA = fixtureA;
    
    /** @type {!Box2D.Dynamics.b2Fixture} */
    this.fixtureB = fixtureB;
};
