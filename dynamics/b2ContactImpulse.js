/*
 * See Box2D.js
 */
goog.provide('Box2D.Dynamics.b2ContactImpulse');

/**
 * @constructor
 */
Box2D.Dynamics.b2ContactImpulse = function () {
    this.normalImpulses = [];
    this.tangentImpulses = [];
};