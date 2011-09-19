/*
 * See Box2D.js
 */

goog.provide('Box2D.Common.Math.b2Vec3');

/**
 * @param {number} x
 * @param {number} y
 * @param {number} z
 * @constructor
 */
Box2D.Common.Math.b2Vec3 = function(x, y, z) {
    this.x = x;
    this.y = y;
    this.z = z;
};

Box2D.Common.Math.b2Vec3.prototype.SetZero = function() {
    this.x = 0;
    this.y = 0;
    this.z = 0;
};

/**
 * @param {number} x
 * @param {number} y
 * @param {number} z
 */
Box2D.Common.Math.b2Vec3.prototype.Set = function(x, y, z) {
    this.x = x;
    this.y = y;
    this.z = z;
}

/**
 * @param {!Box2D.Common.Math.b2Vec3} v
 */
Box2D.Common.Math.b2Vec3.prototype.SetV = function(v) {
    this.x = v.x;
    this.y = v.y;
    this.z = v.z;
};

/**
 * @return {!Box2D.Common.Math.b2Vec3}
 */
Box2D.Common.Math.b2Vec3.prototype.GetNegative = function() {
    return new Box2D.Common.Math.b2Vec3((-this.x), (-this.y), (-this.z));
};

Box2D.Common.Math.b2Vec3.prototype.NegativeSelf = function() {
    this.x = (-this.x);
    this.y = (-this.y);
    this.z = (-this.z);
};

/**
 * @return {!Box2D.Common.Math.b2Vec3}
 */
Box2D.Common.Math.b2Vec3.prototype.Copy = function() {
    return new Box2D.Common.Math.b2Vec3(this.x, this.y, this.z);
};

/**
 * @param {!Box2D.Common.Math.b2Vec3} v
 */
Box2D.Common.Math.b2Vec3.prototype.Add = function(v) {
    this.x += v.x;
    this.y += v.y;
    this.z += v.z;
};

/**
 * @param {!Box2D.Common.Math.b2Vec3} v
 */
Box2D.Common.Math.b2Vec3.prototype.Subtract = function(v) {
    this.x -= v.x;
    this.y -= v.y;
    this.z -= v.z;
};

/**
 * @param {number} a
 */
Box2D.Common.Math.b2Vec3.prototype.Multiply = function(a) {
    this.x *= a;
    this.y *= a;
    this.z *= a;
};
