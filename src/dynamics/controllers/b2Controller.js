/*
 * Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
/*
 * Original Box2D created by Erin Catto
 * http://www.gphysics.com
 * http://box2d.org/
 * 
 * Box2D was converted to Flash by Boris the Brave, Matt Bush, and John Nesky as Box2DFlash
 * http://www.box2dflash.org/
 * 
 * Box2DFlash was converted from Flash to Javascript by Uli Hecht as box2Dweb
 * http://code.google.com/p/box2dweb/
 * 
 * box2Dweb was modified to utilize Google Closure, as well as other bug fixes, optimizations, and tweaks by Illandril
 * https://github.com/illandril/box2dweb-closure
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
