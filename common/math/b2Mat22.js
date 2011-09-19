/*
 * See Box2D.js
 */
goog.provide('Box2D.Common.Math.b2Mat22');

goog.require('Box2D.Common.Math.b2Vec2');

/**
 * @constructor
 */
Box2D.Common.Math.b2Mat22 = function() {
    this.col1 = new Box2D.Common.Math.b2Vec2(0, 0);
    this.col2 = new Box2D.Common.Math.b2Vec2(0, 0);
    this.SetIdentity();
};

/**
 * @param {number} angle
 * @return {!Box2D.Common.Math.b2Mat22}
 */
Box2D.Common.Math.b2Mat22.FromAngle = function(angle) {
    var mat = new Box2D.Common.Math.b2Mat22();
    mat.Set(angle);
    return mat;
};

/**
 * @param {!Box2D.Common.Math.b2Vec2} c1
 * @param {!Box2D.Common.Math.b2Vec2} c2
 * @return {!Box2D.Common.Math.b2Mat22}
 */
Box2D.Common.Math.b2Mat22.FromVV = function(c1, c2) {
    var mat = new Box2D.Common.Math.b2Mat22();
    mat.SetVV(c1, c2);
    return mat;
};

/**
 * @param {number} angle
 */
Box2D.Common.Math.b2Mat22.prototype.Set = function(angle) {
    var c = Math.cos(angle);
    var s = Math.sin(angle);
    this.col1.Set(c, s);
    this.col2.Set(-s, c);
};

/**
 * @param {!Box2D.Common.Math.b2Vec2} c1
 * @param {!Box2D.Common.Math.b2Vec2} c2
 */
Box2D.Common.Math.b2Mat22.prototype.SetVV = function(c1, c2) {
    this.col1.SetV(c1);
    this.col2.SetV(c2);
};

/**
 * @return {!Box2D.Common.Math.b2Mat22}
 */
Box2D.Common.Math.b2Mat22.prototype.Copy = function() {
    var mat = new Box2D.Common.Math.b2Mat22();
    mat.SetM(this);
    return mat;
};

/**
 * @param {!Box2D.Common.Math.b2Mat22} m
 */
Box2D.Common.Math.b2Mat22.prototype.SetM = function(m) {
    this.col1.SetV(m.col1);
    this.col2.SetV(m.col2);
};

/**
 * @param {!Box2D.Common.Math.b2Mat22} m
 */
Box2D.Common.Math.b2Mat22.prototype.AddM = function(m) {
    this.col1.Add(m.col1);
    this.col2.Add(m.col2);
};

Box2D.Common.Math.b2Mat22.prototype.SetIdentity = function() {
    this.col1.Set(1, 0);
    this.col2.Set(0, 1);
};

Box2D.Common.Math.b2Mat22.prototype.SetZero = function() {
    this.col1.Set(0, 0);
    this.col2.Set(0, 0);
};

/**
 * @return {number}
 */
Box2D.Common.Math.b2Mat22.prototype.GetAngle = function() {
    return Math.atan2(this.col1.y, this.col1.x);
};

/**
 * @param {!Box2D.Common.Math.b2Mat22} out
 * @return {!Box2D.Common.Math.b2Mat22}
 */
Box2D.Common.Math.b2Mat22.prototype.GetInverse = function(out) {
    var det = this.col1.x * this.col2.y - this.col2.x * this.col1.y;
    if (det !== 0) {
        det = 1 / det;
    }
    out.col1.x = det * this.col2.y;
    out.col2.x = -det * this.col2.x;
    out.col1.y = -det * this.col1.y;
    out.col2.y = det * this.col1.x;
    return out;
};

/**
 * @param {!Box2D.Common.Math.b2Vec2} out
 * @param {number} bX
 * @param {number} bY
 * @return {!Box2D.Common.Math.b2Vec2}
 */
Box2D.Common.Math.b2Mat22.prototype.Solve = function(out, bX, bY) {
    var det = this.col1.x * this.col2.y - this.col2.x * this.col1.y;
    if (det !== 0) {
        det = 1 / det;
    }
    out.x = det * (this.col2.y * bX - this.col2.x * bY);
    out.y = det * (this.col1.x * bY - this.col1.y * bX);
    return out;
};

Box2D.Common.Math.b2Mat22.prototype.Abs = function() {
    this.col1.Abs();
    this.col2.Abs();
};
