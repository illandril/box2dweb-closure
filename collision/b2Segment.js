/*
 * See Box2D.js
 */
goog.provide('Box2D.Collision.b2Segment');

goog.require('Box2D.Common.Math.b2Vec2');

/**
 * @constructor
 */
Box2D.Collision.b2Segment = function() {
    this.p1 = new Box2D.Common.Math.b2Vec2(0, 0);
    this.p2 = new Box2D.Common.Math.b2Vec2(0, 0);
};

Box2D.Collision.b2Segment.prototype.TestSegment = function(lambda, normal, segment, maxLambda) {
    if (maxLambda === undefined) maxLambda = 0;
    var s = segment.p1;
    var rX = segment.p2.x - s.x;
    var rY = segment.p2.y - s.y;
    var dX = this.p2.x - this.p1.x;
    var dY = this.p2.y - this.p1.y;
    var nX = dY;
    var nY = (-dX);
    var k_slop = 100.0 * Number.MIN_VALUE;
    var denom = (-(rX * nX + rY * nY));
    if (denom > k_slop) {
        var bX = s.x - this.p1.x;
        var bY = s.y - this.p1.y;
        var a = (bX * nX + bY * nY);
        if (0.0 <= a && a <= maxLambda * denom) {
            var mu2 = (-rX * bY) + rY * bX;
            if ((-k_slop * denom) <= mu2 && mu2 <= denom * (1.0 + k_slop)) {
                a /= denom;
                var nLen = Math.sqrt(nX * nX + nY * nY);
                nX /= nLen;
                nY /= nLen;
                lambda[0] = a;
                normal.Set(nX, nY);
                return true;
            }
        }
    }
    return false;
};

Box2D.Collision.b2Segment.prototype.Extend = function(aabb) {
    this.ExtendForward(aabb);
    this.ExtendBackward(aabb);
};

Box2D.Collision.b2Segment.prototype.ExtendForward = function(aabb) {
    var dX = this.p2.x - this.p1.x;
    var dY = this.p2.y - this.p1.y;
    var lambda = Math.min(dX > 0 ? (aabb.upperBound.x - this.p1.x) / dX : dX < 0 ? (aabb.lowerBound.x - this.p1.x) / dX : Number.POSITIVE_INFINITY, dY > 0 ? (aabb.upperBound.y - this.p1.y) / dY : dY < 0 ? (aabb.lowerBound.y - this.p1.y) / dY : Number.POSITIVE_INFINITY);
    this.p2.x = this.p1.x + dX * lambda;
    this.p2.y = this.p1.y + dY * lambda;
};

Box2D.Collision.b2Segment.prototype.ExtendBackward = function(aabb) {
    var dX = (-this.p2.x) + this.p1.x;
    var dY = (-this.p2.y) + this.p1.y;
    var lambda = Math.min(dX > 0 ? (aabb.upperBound.x - this.p2.x) / dX : dX < 0 ? (aabb.lowerBound.x - this.p2.x) / dX : Number.POSITIVE_INFINITY, dY > 0 ? (aabb.upperBound.y - this.p2.y) / dY : dY < 0 ? (aabb.lowerBound.y - this.p2.y) / dY : Number.POSITIVE_INFINITY);
    this.p1.x = this.p2.x + dX * lambda;
    this.p1.y = this.p2.y + dY * lambda;
};