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
    if (this.invSqr) {
        this._StepInvSq();
    } else {
        this._Step();
    }
};

Box2D.Dynamics.Controllers.b2GravityController.prototype._StepInvSq = function() {
    for (var body1Node = this.bodyList.GetFirstNode(Box2D.Dynamics.b2BodyList.TYPES.allBodies); body1Node; body1Node = body1Node.GetNextNode()) {
        var body1 = body1Node.body;
        var p1 = body1.GetWorldCenter();
        var mass1 = body1.GetMass();
        for (var body2Node = this.bodyList.GetFirstNode(Box2D.Dynamics.b2BodyList.TYPES.allBodies); body2Node; body2Node = body2Node.GetNextNode()) {
            var body2 = body2Node.body;
            if ( !body1.IsAwake() && !body2.IsAwake() ) {
                continue;
            }
            var p2 = body2.GetWorldCenter();
            var dx = p2.x - p1.x;
            var dy = p2.y - p1.y;
            var r2 = dx * dx + dy * dy;
            if (r2 < Number.MIN_VALUE) {
                continue;
            }
            var f = Box2D.Common.Math.b2Vec2.Get(dx, dy);
            f.Multiply(this.G / r2 / Math.sqrt(r2) * mass1 * body2.GetMass());
            if (body1.IsAwake()) {
                body1.ApplyForce(f, p1);
            }
            f.Multiply(-1);
            if (body2.IsAwake()) {
                body2.ApplyForce(f, p2);
            }
            Box2D.Common.Math.b2Vec2.Free(f);
        }
    }
};

Box2D.Dynamics.Controllers.b2GravityController.prototype._Step = function() {
    for (var body1Node = this.bodyList.GetFirstNode(Box2D.Dynamics.b2BodyList.TYPES.allBodies); body1Node; body1Node = body1Node.GetNextNode()) {
        var body1 = body1Node.body;
        var p1 = body1.GetWorldCenter();
        var mass1 = body1.GetMass();
        for (var body2Node = this.bodyList.GetFirstNode(Box2D.Dynamics.b2BodyList.TYPES.allBodies); body2Node; body2Node = body2Node.GetNextNode()) {
            var body2 = body2Node.body;
            if ( !body1.IsAwake() && !body2.IsAwake() ) {
                continue;
            }
            var p2 = body2.GetWorldCenter();
            var dx = p2.x - p1.x;
            var dy = p2.y - p1.y;
            var r2 = dx * dx + dy * dy;
            if (r2 < Number.MIN_VALUE) {
                continue;
            }
            var f = Box2D.Common.Math.b2Vec2.Get(dx, dy);
            f.Multiply(this.G / r2 * mass1 * body2.GetMass());
            if (body1.IsAwake()) {
                body1.ApplyForce(f, p1);
            }
            f.Multiply(-1);
            if (body2.IsAwake()) {
                body2.ApplyForce(f, p2);
            }
            Box2D.Common.Math.b2Vec2.Free(f);
        }
    }
};