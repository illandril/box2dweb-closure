/*
 * See Box2D.js
 */
goog.provide('Box2D.Dynamics.Controllers.b2TensorDampingController');

goog.require('Box2D.Dynamics.Controllers.b2Controller');
goog.require('Box2D.Common.Math.b2Mat22');
goog.require('Box2D.Common.Math.b2Math');
goog.require('Box2D.Common.Math.b2Vec2');

/**
 * @constructor
 * @extends {Box2D.Dynamics.Controllers.b2Controller}
 */
Box2D.Dynamics.Controllers.b2TensorDampingController = function() {
    Box2D.Dynamics.Controllers.b2Controller.call(this);
    this.T = new Box2D.Common.Math.b2Mat22();
    this.maxTimestep = 0;
};
goog.inherits(Box2D.Dynamics.Controllers.b2TensorDampingController, Box2D.Dynamics.Controllers.b2Controller);

Box2D.Dynamics.Controllers.b2TensorDampingController.prototype.SetAxisAligned = function(xDamping, yDamping) {
    if (xDamping === undefined) xDamping = 0;
    if (yDamping === undefined) yDamping = 0;
    this.T.col1.x = (-xDamping);
    this.T.col1.y = 0;
    this.T.col2.x = 0;
    this.T.col2.y = (-yDamping);
    if (xDamping > 0 || yDamping > 0) {
        this.maxTimestep = 1 / Math.max(xDamping, yDamping);
    } else {
        this.maxTimestep = 0;
    }
};

Box2D.Dynamics.Controllers.b2TensorDampingController.prototype.Step = function(step) {
    var timestep = step.dt;
    if (timestep <= Number.MIN_VALUE) return;
    if (timestep > this.maxTimestep && this.maxTimestep > 0) timestep = this.maxTimestep;
    for (var i = this.m_bodyList; i; i = i.nextBody) {
        var body = i.body;
        if (!body.IsAwake()) {
            continue;
        }
        var damping = body.GetWorldVector(Box2D.Common.Math.b2Math.MulMV(this.T, body.GetLocalVector(body.GetLinearVelocity())));
        body.SetLinearVelocity(new Box2D.Common.Math.b2Vec2(body.GetLinearVelocity().x + damping.x * timestep, body.GetLinearVelocity().y + damping.y * timestep));
    }
};
