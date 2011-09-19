/*
 * See Box2D.js
 */
goog.provide('Box2D.Dynamics.b2ContactListener');


/**
 * @constructor
 */
Box2D.Dynamics.b2ContactListener = function () {};

Box2D.Dynamics.b2ContactListener.prototype.BeginContact = function (contact) {};

Box2D.Dynamics.b2ContactListener.prototype.EndContact = function (contact) {};

Box2D.Dynamics.b2ContactListener.prototype.PreSolve = function (contact, oldManifold) {};

Box2D.Dynamics.b2ContactListener.prototype.PostSolve = function (contact, impulse) {};

Box2D.Dynamics.b2ContactListener.b2_defaultListener = new Box2D.Dynamics.b2ContactListener();
  