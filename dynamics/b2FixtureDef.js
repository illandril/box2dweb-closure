/*
 * See Box2D.js
 */
goog.provide('Box2D.Dynamics.b2FixtureDef');

goog.require('Box2D.Dynamics.b2FilterData');

/**
 * @constructor
 */
Box2D.Dynamics.b2FixtureDef = function () {
    this.filter = new Box2D.Dynamics.b2FilterData();
    this.shape = null;
    this.userData = null;
    this.friction = 0.2;
    this.restitution = 0.0;
    this.density = 0.0;
    this.filter.categoryBits = 0x0001;
    this.filter.maskBits = 0xFFFF;
    this.filter.groupIndex = 0;
    this.isSensor = false;
};
