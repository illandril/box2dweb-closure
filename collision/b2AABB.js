/*
 * See Box2D.js
 */
goog.provide('Box2D.Collision.b2AABB');

goog.require('Box2D.Common.Math.b2Vec2');

/**
 * @constructor
 */
Box2D.Collision.b2AABB = function() {
    this.lowerBound = new Box2D.Common.Math.b2Vec2(0, 0);
    this.upperBound = new Box2D.Common.Math.b2Vec2(0, 0);
};

/**
 * @return {boolean}
 */
Box2D.Collision.b2AABB.prototype.IsValid = function() {
    var dX = this.upperBound.x - this.lowerBound.x;
    if (dX < 0) {
        return false;
    }
    var dY = this.upperBound.y - this.lowerBound.y;
    if (dY < 0) {
        return false;
    }
    return this.lowerBound.IsValid() && this.upperBound.IsValid();
};

/**
 * @return {!Box2D.Common.Math.b2Vec2}
 */
Box2D.Collision.b2AABB.prototype.GetCenter = function() {
    return new Box2D.Common.Math.b2Vec2((this.lowerBound.x + this.upperBound.x) / 2, (this.lowerBound.y + this.upperBound.y) / 2);
};


/**
 * @param {!Box2D.Common.Math.b2Vec2}
 */
Box2D.Collision.b2AABB.prototype.SetCenter = function(newCenter) {
    var oldCenter = this.GetCenter();
    this.lowerBound.Subtract(oldCenter);
    this.upperBound.Subtract(oldCenter);
    this.lowerBound.Add(newCenter);
    this.upperBound.Add(newCenter);
};

/**
 * @return {!Box2D.Common.Math.b2Vec2}
 */
Box2D.Collision.b2AABB.prototype.GetExtents = function() {
    return new Box2D.Common.Math.b2Vec2((this.upperBound.x - this.lowerBound.x) / 2, (this.upperBound.y - this.lowerBound.y) / 2);
};

/**
 * @param {!Box2D.Collision.b2AABB} aabb
 * @return {boolean}
 */
Box2D.Collision.b2AABB.prototype.Contains = function(aabb) {
    var result = true;
    result = result && this.lowerBound.x <= aabb.lowerBound.x;
    result = result && this.lowerBound.y <= aabb.lowerBound.y;
    result = result && aabb.upperBound.x <= this.upperBound.x;
    result = result && aabb.upperBound.y <= this.upperBound.y;
    return result;
};

/**
 * @param {!Box2D.Collision.b2RayCastOutput} output
 * @param {!Box2D.Collision.b2RayCastInput} input
 * @return {boolean}
 */
Box2D.Collision.b2AABB.prototype.RayCast = function(output, input) {
    var tmin = (-Number.MAX_VALUE);
    var tmax = Number.MAX_VALUE;
    
    var dX = input.p2.x - input.p1.x;
    var absDX = Math.abs(dX);
    if (absDX < Number.MIN_VALUE) {
        if (input.p1.x < this.lowerBound.x || this.upperBound.x < input.p1.x) {
            return false;
        }
    } else {
        var inv_d = 1.0 / dX;
        var t1 = (this.lowerBound.x - input.p1.x) * inv_d;
        var t2 = (this.upperBound.x - input.p1.x) * inv_d;
        var s = (-1.0);
        if (t1 > t2) {
            var t3 = t1;
            t1 = t2;
            t2 = t3;
            s = 1.0;
        }
        if (t1 > tmin) {
            output.normal.x = s;
            output.normal.y = 0;
            tmin = t1;
        }
        tmax = Math.min(tmax, t2);
        if (tmin > tmax) return false;
    }
    
    var dY = input.p2.y - input.p1.y;
    var absDY = Math.abs(dY);
    if (absDY < Number.MIN_VALUE) {
        if (input.p1.y < this.lowerBound.y || this.upperBound.y < input.p1.y) {
            return false;
        }
    } else {
        var inv_d = 1.0 / dY;
        var t1 = (this.lowerBound.y - input.p1.y) * inv_d;
        var t2 = (this.upperBound.y - input.p1.y) * inv_d;
        var s = (-1.0);
        if (t1 > t2) {
            var t3 = t1;
            t1 = t2;
            t2 = t3;
            s = 1.0;
        }
        if (t1 > tmin) {
            output.normal.y = s;
            output.normal.x = 0;
            tmin = t1;
        }
        tmax = Math.min(tmax, t2);
        if (tmin > tmax) {
            return false;
        }
    }
    output.fraction = tmin;
    return true;
};

/**
 * @param {!Box2D.Collision.b2AABB} other
 * @return {boolean}
 */
Box2D.Collision.b2AABB.prototype.TestOverlap = function(other) {
    if ( other.lowerBound.x - this.upperBound.x > 0 ) { return false; }
    if ( other.lowerBound.y - this.upperBound.y > 0 ) { return false; }
    if ( this.lowerBound.x - other.upperBound.x > 0 ) { return false; }
    if ( this.lowerBound.y - other.upperBound.y > 0 ) { return false; }
    return true;
};

/**
 * @param {!Box2D.Collision.b2AABB} aabb1
 * @param {!Box2D.Collision.b2AABB} aabb2
 * @return {!Box2D.Collision.b2AABB}
 */
Box2D.Collision.b2AABB.Combine = function(aabb1, aabb2) {
    var aabb = new Box2D.Collision.b2AABB();
    aabb.Combine(aabb1, aabb2);
    return aabb;
};

/**
 * @param {!Box2D.Collision.b2AABB} aabb1
 * @param {!Box2D.Collision.b2AABB} aabb2
 */
Box2D.Collision.b2AABB.prototype.Combine = function(aabb1, aabb2) {
    this.lowerBound.x = Math.min(aabb1.lowerBound.x, aabb2.lowerBound.x);
    this.lowerBound.y = Math.min(aabb1.lowerBound.y, aabb2.lowerBound.y);
    this.upperBound.x = Math.max(aabb1.upperBound.x, aabb2.upperBound.x);
    this.upperBound.y = Math.max(aabb1.upperBound.y, aabb2.upperBound.y);
};
