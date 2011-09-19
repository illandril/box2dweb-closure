/*
 * See Box2D.js
 */
goog.provide('Box2D.Dynamics.b2TimeStep');

/**
 * @constructor
 */
Box2D.Dynamics.b2TimeStep = function() {};

Box2D.Dynamics.b2TimeStep.prototype.Set = function(step) {
    this.dt = step.dt;
    this.inv_dt = step.inv_dt;
    this.positionIterations = step.positionIterations;
    this.velocityIterations = step.velocityIterations;
    this.warmStarting = step.warmStarting;
};