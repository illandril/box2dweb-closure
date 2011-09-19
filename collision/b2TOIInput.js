/*
 * See Box2D.js
 */
goog.provide('Box2D.Collision.b2TOIInput');

goog.require('Box2D.Collision.b2DistanceProxy');
goog.require('Box2D.Common.Math.b2Sweep');

/**
 * @constructor
 */
Box2D.Collision.b2TOIInput = function() {
    this.proxyA = new Box2D.Collision.b2DistanceProxy();
    this.proxyB = new Box2D.Collision.b2DistanceProxy();
    this.sweepA = new Box2D.Common.Math.b2Sweep();
    this.sweepB = new Box2D.Common.Math.b2Sweep();
};