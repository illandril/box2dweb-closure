/*
 * See Box2D.js
 */
goog.provide('Box2D.Dynamics.b2FilterData');

/**
 * @constructor
 */
Box2D.Dynamics.b2FilterData = function () {
  this.categoryBits = 0x0001;
  this.maskBits = 0xFFFF;
  this.groupIndex = 0;
};

/**
 * @return {!Box2D.Dynamics.b2FilterData}
 */
Box2D.Dynamics.b2FilterData.prototype.Copy = function () {
  var copy = new Box2D.Dynamics.b2FilterData();
  copy.categoryBits = this.categoryBits;
  copy.maskBits = this.maskBits;
  copy.groupIndex = this.groupIndex;
  return copy;
};