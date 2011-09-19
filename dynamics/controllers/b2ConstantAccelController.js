/*
 * See Box2D.js
 */
goog.provide('Box2D.Dynamics.Controllers.b2ConstantAccelController');

goog.require('Box2D.Dynamics.Controllers.b2Controller');
goog.require('Box2D.Common.Math.b2Vec2');

/**
 * @constructor
 * @extends {Box2D.Dynamics.Controllers.b2Controller}
 */
Box2D.Dynamics.Controllers.b2ConstantAccelController = function() {
    Box2D.Dynamics.Controllers.b2Controller.call(this);
    this.A = new Box2D.Common.Math.b2Vec2(0, 0);
};
goog.inherits(Box2D.Dynamics.Controllers.b2ConstantAccelController, Box2D.Dynamics.Controllers.b2Controller);

Box2D.Dynamics.Controllers.b2ConstantAccelController.prototype.Step = function(step) {
    var smallA = new Box2D.Common.Math.b2Vec2(this.A.x * step.dt, this.A.y * step.dt);
    for (var i = this.m_bodyList; i; i = i.nextBody) {
        var body = i.body;
        if (!body.IsAwake()) {
            continue;
        }
        var oldVelocity = body.GetLinearVelocity();
        body.SetLinearVelocity(new Box2D.Common.Math.b2Vec2(oldVelocity.x + smallA.x, oldVelocity.y + smallA.y));
    }
};
