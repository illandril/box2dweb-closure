/*
 * See Box2D.js
 */
goog.provide('Box2D.Common.Math.b2Sweep');

goog.require('Box2D.Common.Math.b2Vec2');

/**
 * @constructor
 */
Box2D.Common.Math.b2Sweep = function() {
    this.localCenter = new Box2D.Common.Math.b2Vec2(0, 0);
    this.c0 = new Box2D.Common.Math.b2Vec2(0, 0);
    this.c = new Box2D.Common.Math.b2Vec2(0, 0);
};

Box2D.Common.Math.b2Sweep.prototype.Set = function(other) {
    this.localCenter.SetV(other.localCenter);
    this.c0.SetV(other.c0);
    this.c.SetV(other.c);
    this.a0 = other.a0;
    this.a = other.a;
    this.t0 = other.t0;
};

Box2D.Common.Math.b2Sweep.prototype.Copy = function() {
    var copy = new Box2D.Common.Math.b2Sweep();
    copy.localCenter.SetV(this.localCenter);
    copy.c0.SetV(this.c0);
    copy.c.SetV(this.c);
    copy.a0 = this.a0;
    copy.a = this.a;
    copy.t0 = this.t0;
    return copy;
};

Box2D.Common.Math.b2Sweep.prototype.GetTransform = function(xf, alpha) {
    if (alpha === undefined) alpha = 0;
    xf.position.x = (1.0 - alpha) * this.c0.x + alpha * this.c.x;
    xf.position.y = (1.0 - alpha) * this.c0.y + alpha * this.c.y;
    var angle = (1.0 - alpha) * this.a0 + alpha * this.a;
    xf.R.Set(angle);
    var tMat = xf.R;
    xf.position.x -= (tMat.col1.x * this.localCenter.x + tMat.col2.x * this.localCenter.y);
    xf.position.y -= (tMat.col1.y * this.localCenter.x + tMat.col2.y * this.localCenter.y);
};

Box2D.Common.Math.b2Sweep.prototype.Advance = function(t) {
    if (t === undefined) t = 0;
    if (this.t0 < t && 1.0 - this.t0 > Number.MIN_VALUE) {
        var alpha = (t - this.t0) / (1.0 - this.t0);
        this.c0.x = (1.0 - alpha) * this.c0.x + alpha * this.c.x;
        this.c0.y = (1.0 - alpha) * this.c0.y + alpha * this.c.y;
        this.a0 = (1.0 - alpha) * this.a0 + alpha * this.a;
        this.t0 = t;
    }
};
