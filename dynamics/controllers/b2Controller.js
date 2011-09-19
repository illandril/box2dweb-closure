/*
 * See Box2D.js
 */
goog.provide('Box2D.Dynamics.Controllers.b2Controller');

goog.require('Box2D.Dynamics.Controllers.b2ControllerEdge');

/**
 * @constructor
 */
Box2D.Dynamics.Controllers.b2Controller = function() {};

Box2D.Dynamics.Controllers.b2Controller.prototype.Step = function(step) {};

Box2D.Dynamics.Controllers.b2Controller.prototype.Draw = function(debugDraw) {};

Box2D.Dynamics.Controllers.b2Controller.prototype.AddBody = function(body) {
    var edge = new Box2D.Dynamics.Controllers.b2ControllerEdge();
    edge.controller = this;
    edge.body = body;
    edge.nextBody = this.m_bodyList;
    edge.prevBody = null;
    this.m_bodyList = edge;
    if (edge.nextBody) {
        edge.nextBody.prevBody = edge;
    }
    this.m_bodyCount++;
    edge.nextController = body.m_controllerList;
    edge.prevController = null;
    body.m_controllerList = edge;
    if (edge.nextController) {
        edge.nextController.prevController = edge;
    }
    body.m_controllerCount++;
};

Box2D.Dynamics.Controllers.b2Controller.prototype.RemoveBody = function(body) {
    var edge = body.m_controllerList;
    while (edge && edge.controller != this) {
        edge = edge.nextController;
    }
    if (edge.prevBody) {
        edge.prevBody.nextBody = edge.nextBody;
    }
    if (edge.nextBody) {
        edge.nextBody.prevBody = edge.prevBody;
    }
    if (edge.nextController) {
        edge.nextController.prevController = edge.prevController;
    }
    if (edge.prevController) {
        edge.prevController.nextController = edge.nextController;
    }
    if (this.m_bodyList == edge) {
        this.m_bodyList = edge.nextBody;
    }
    if (body.m_controllerList == edge) {
        body.m_controllerList = edge.nextController;
    }
    body.m_controllerCount--;
    this.m_bodyCount--;
};

Box2D.Dynamics.Controllers.b2Controller.prototype.Clear = function() {
    while (this.m_bodyList) {
        this.RemoveBody(this.m_bodyList.body);
    }
};

Box2D.Dynamics.Controllers.b2Controller.prototype.GetNext = function() {
    return this.m_next;
};

Box2D.Dynamics.Controllers.b2Controller.prototype.GetWorld = function() {
    return this.m_world;
};

Box2D.Dynamics.Controllers.b2Controller.prototype.GetBodyList = function() {
    return this.m_bodyList;
};
