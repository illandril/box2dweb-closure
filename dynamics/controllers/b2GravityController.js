/*
 * See Box2D.js
 */
goog.provide('Box2D.Dynamics.Controllers.b2GravityController');

goog.require('Box2D.Dynamics.Controllers.b2Controller');
goog.require('Box2D.Common.Math.b2Vec2');

/**
 * @constructor
 * @extends {Box2D.Dynamics.Controllers.b2Controller}
 */
Box2D.Dynamics.Controllers.b2GravityController = function() {
    Box2D.Dynamics.Controllers.b2Controller.call(this);
    this.G = 1;
    this.invSqr = true;
};
goog.inherits(Box2D.Dynamics.Controllers.b2GravityController, Box2D.Dynamics.Controllers.b2Controller);

Box2D.Dynamics.Controllers.b2GravityController.prototype.Step = function(step) {
    var i = null;
    var body1 = null;
    var p1 = null;
    var mass1 = 0;
    var j = null;
    var body2 = null;
    var p2 = null;
    var dx = 0;
    var dy = 0;
    var r2 = 0;
    var f = null;
    if (this.invSqr) {
        for (i = this.m_bodyList; i; i = i.nextBody) {
            var body1 = i.body;
            var p1 = body1.GetWorldCenter();
            var mass1 = body1.GetMass();
            for (j = this.m_bodyList; j != i; j = j.nextBody) {
                var body2 = j.body;
                var p2 = body2.GetWorldCenter();
                var dx = p2.x - p1.x;
                var dy = p2.y - p1.y;
                var r2 = dx * dx + dy * dy;
                if (r2 < Number.MIN_VALUE) {
                    continue;
                }
                var f = new Box2d.Common.Math.b2Vec2(dx, dy);
                f.Multiply(this.G / r2 / Math.sqrt(r2) * mass1 * body2.GetMass());
                if (body1.IsAwake()) {
                    body1.ApplyForce(f, p1);
                }
                f.Multiply(-1);
                if (body2.IsAwake()) {
                    body2.ApplyForce(f, p2);
                }
            }
        }
    } else {
        for (i = this.m_bodyList; i; i = i.nextBody) {
            var body1 = i.body;
            var p1 = body1.GetWorldCenter();
            var mass1 = body1.GetMass();
            for (j = this.m_bodyList; j != i; j = j.nextBody) {
                var body2 = j.body;
                var p2 = body2.GetWorldCenter();
                var dx = p2.x - p1.x;
                var dy = p2.y - p1.y;
                var r2 = dx * dx + dy * dy;
                if (r2 < Number.MIN_VALUE) {
                    continue;
                }
                var f = new Box2d.Common.Math.b2Vec2(dx, dy);
                f.Multiply(this.G / r2 * mass1 * body2.GetMass());
                if (body1.IsAwake()) {
                    body1.ApplyForce(f, p1);
                }
                f.Multiply(-1);
                if (body2.IsAwake()) {
                    body2.ApplyForce(f, p2);
                }
            }
        }
    }
};
