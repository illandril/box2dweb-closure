/*
 * See Box2D.js
 */
goog.provide('Box2D.Common.Math.b2Math');

goog.require('Box2D.Common.Math.b2Mat22');
goog.require('Box2D.Common.Math.b2Vec2');

Box2D.Common.Math.b2Math = {};

/**
 * @param {!Box2D.Common.Math.b2Vec2} a
 * @param {!Box2D.Common.Math.b2Vec2} b
 * @return {number}
 */
Box2D.Common.Math.b2Math.Dot = function (a, b) {
  return a.x * b.x + a.y * b.y;
};

/**
 * @param {!Box2D.Common.Math.b2Vec2} a
 * @param {!Box2D.Common.Math.b2Vec2} b
 * @return {number}
 */
Box2D.Common.Math.b2Math.CrossVV = function (a, b) {
  return a.x * b.y - a.y * b.x;
};

/**
 * @param {!Box2D.Common.Math.b2Vec2} a
 * @param {number} s
 * @return {!Box2D.Common.Math.b2Vec2}
 */
Box2D.Common.Math.b2Math.CrossVF = function (a, s) {
  return new Box2D.Common.Math.b2Vec2(s * a.y, (-s * a.x));
};

/**
 * @param {number} s
 * @param {!Box2D.Common.Math.b2Vec2} a
 * @return {!Box2D.Common.Math.b2Vec2}
 */
Box2D.Common.Math.b2Math.CrossFV = function (s, a) {
  return new Box2D.Common.Math.b2Vec2((-s * a.y), s * a.x);
};

/**
 * @param {!Box2D.Common.Math.b2Mat22} A
 * @param {!Box2D.Common.Math.b2Vec2} v
 * @return {!Box2D.Common.Math.b2Vec2}
 */
Box2D.Common.Math.b2Math.MulMV = function (A, v) {
  return new Box2D.Common.Math.b2Vec2(A.col1.x * v.x + A.col2.x * v.y, A.col1.y * v.x + A.col2.y * v.y);
};

/**
 * @param {!Box2D.Common.Math.b2Mat22} A
 * @param {!Box2D.Common.Math.b2Vec2} v
 * @return {!Box2D.Common.Math.b2Vec2}
 */
Box2D.Common.Math.b2Math.MulTMV = function (A, v) {
  return new Box2D.Common.Math.b2Vec2(Box2D.Common.Math.b2Math.Dot(v, A.col1), Box2D.Common.Math.b2Math.Dot(v, A.col2));
};

/**
 * @param {!Box2D.Common.Math.b2Transform} T
 * @param {!Box2D.Common.Math.b2Vec2} v
 * @return {!Box2D.Common.Math.b2Vec2}
 */
Box2D.Common.Math.b2Math.MulX = function (T, v) {
  var a = Box2D.Common.Math.b2Math.MulMV(T.R, v);
  a.x += T.position.x;
  a.y += T.position.y;
  return a;
};

/**
 * @param {!Box2D.Common.Math.b2Transform} T
 * @param {!Box2D.Common.Math.b2Vec2} v
 * @return {!Box2D.Common.Math.b2Vec2}
 */
Box2D.Common.Math.b2Math.MulXT = function (T, v) {
  var a = Box2D.Common.Math.b2Math.SubtractVV(v, T.position);
  var tX = (a.x * T.R.col1.x + a.y * T.R.col1.y);
  a.y = (a.x * T.R.col2.x + a.y * T.R.col2.y);
  a.x = tX;
  return a;
};

/**
 * @param {!Box2D.Common.Math.b2Vec2} a
 * @param {!Box2D.Common.Math.b2Vec2} b
 * @return {!Box2D.Common.Math.b2Vec2}
 */
Box2D.Common.Math.b2Math.AddVV = function (a, b) {
  return new Box2D.Common.Math.b2Vec2(a.x + b.x, a.y + b.y);
};

/**
 * @param {!Box2D.Common.Math.b2Vec2} a
 * @param {!Box2D.Common.Math.b2Vec2} b
 * @return {!Box2D.Common.Math.b2Vec2}
 */
Box2D.Common.Math.b2Math.SubtractVV = function (a, b) {
  return new Box2D.Common.Math.b2Vec2(a.x - b.x, a.y - b.y);
};

/**
 * @param {!Box2D.Common.Math.b2Vec2} a
 * @param {!Box2D.Common.Math.b2Vec2} b
 * @return {number}
 */
Box2D.Common.Math.b2Math.Distance = function (a, b) {
  var cX = a.x - b.x;
  var cY = a.y - b.y;
  return Math.sqrt(Box2D.Common.Math.b2Math.DistanceSquared(a,b));
};

/**
 * @param {!Box2D.Common.Math.b2Vec2} a
 * @param {!Box2D.Common.Math.b2Vec2} b
 * @return {number}
 */
Box2D.Common.Math.b2Math.DistanceSquared = function (a, b) {
  var cX = a.x - b.x;
  var cY = a.y - b.y;
  return (cX * cX + cY * cY);
};

/**
 * @param {number} s
 * @param {!Box2D.Common.Math.b2Vec2} a
 * @return {!Box2D.Common.Math.b2Vec2}
 */
Box2D.Common.Math.b2Math.MulFV = function (s, a) {
  return new Box2D.Common.Math.b2Vec2(s * a.x, s * a.y);
};

/**
 * @param {!Box2D.Common.Math.b2Mat22} A
 * @param {!Box2D.Common.Math.b2Mat22} B
 * @return {!Box2D.Common.Math.b2Mat22}
 */
Box2D.Common.Math.b2Math.AddMM = function (A, B) {
  return Box2D.Common.Math.b2Mat22.FromVV(Box2D.Common.Math.b2Math.AddVV(A.col1, B.col1), Box2D.Common.Math.b2Math.AddVV(A.col2, B.col2));
};

/**
 * @param {!Box2D.Common.Math.b2Mat22} A
 * @param {!Box2D.Common.Math.b2Mat22} B
 * @return {!Box2D.Common.Math.b2Mat22}
 */
Box2D.Common.Math.b2Math.MulMM = function (A, B) {
  return Box2D.Common.Math.b2Mat22.FromVV(Box2D.Common.Math.b2Math.MulMV(A, B.col1), Box2D.Common.Math.b2Math.MulMV(A, B.col2));
};

/**
 * @param {!Box2D.Common.Math.b2Mat22} A
 * @param {!Box2D.Common.Math.b2Mat22} B
 * @return {!Box2D.Common.Math.b2Mat22}
 */
Box2D.Common.Math.b2Math.MulTMM = function (A, B) {
  var c1 = new Box2D.Common.Math.b2Vec2(Box2D.Common.Math.b2Math.Dot(A.col1, B.col1), Box2D.Common.Math.b2Math.Dot(A.col2, B.col1));
  var c2 = new Box2D.Common.Math.b2Vec2(Box2D.Common.Math.b2Math.Dot(A.col1, B.col2), Box2D.Common.Math.b2Math.Dot(A.col2, B.col2));
  return Box2D.Common.Math.b2Mat22.FromVV(c1, c2);
};

/**
 * @param {!Box2D.Common.Math.b2Vec2} a
 * @return {!Box2D.Common.Math.b2Vec2}
 */
Box2D.Common.Math.b2Math.AbsV = function (a) {
  return new Box2D.Common.Math.b2Vec2(Math.abs(a.x), Math.abs(a.y));
};

/**
 * @param {!Box2D.Common.Math.b2Mat22} A
 * @return {!Box2D.Common.Math.b2Mat22}
 */
Box2D.Common.Math.b2Math.AbsM = function (A) {
  return Box2D.Common.Math.b2Mat22.FromVV(Box2D.Common.Math.b2Math.AbsV(A.col1), Box2D.Common.Math.b2Math.AbsV(A.col2));
};

/**
 * @param {number} a
 * @param {number} low
 * @param {number} high
 * @return {number}
 */
Box2D.Common.Math.b2Math.Clamp = function (a, low, high) {
  return a < low ? low : a > high ? high : a;
};

/**
 * @param {!Box2D.Common.Math.b2Vec2} a
 * @param {!Box2D.Common.Math.b2Vec2} low
 * @param {!Box2D.Common.Math.b2Vec2} high
 * @return {!Box2D.Common.Math.b2Vec2}
 */
Box2D.Common.Math.b2Math.ClampV = function (a, low, high) {
    var x = Box2D.Common.Math.b2Math.Clamp(a.x, low.x, high.x);
    var y = Box2D.Common.Math.b2Math.Clamp(a.y, low.y, high.y);
  return new Box2D.Common.Math.b2Vec2(x, y);
};
