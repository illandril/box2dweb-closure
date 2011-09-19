/*
 * See Box2D.js
 */
goog.provide('Box2D.Dynamics.Controllers.b2ConstantForceController');

goog.require('Box2D.Dynamics.Controllers.b2Controller');
goog.require('Box2D.Common.Math.b2Vec2');

/**
 * @constructor
 * @extends {Box2D.Dynamics.Controllers.b2Controller}
 */
Box2D.Dynamics.Controllers.b2ConstantForceController = function() {
    Box2D.Dynamics.Controllers.b2Controller.call(this);
    this.F = new Box2D.Common.Math.b2Vec2(0, 0);
};
goog.inherits(Box2D.Dynamics.Controllers.b2ConstantForceController, Box2D.Dynamics.Controllers.b2Controller);

Box2D.Dynamics.Controllers.b2ConstantForceController.prototype.Step = function(step) {
    var smallA = new Box2D.Common.Math.b2Vec2(this.A.x * step.dt, this.A.y * step.dt);
    for (var i = this.m_bodyList; i; i = i.nextBody) {
        var body = i.body;
        if (!body.IsAwake()) {
            continue;
        }
        body.ApplyForce(this.F, body.GetWorldCenter());
    }
};
