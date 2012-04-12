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
 
var mouseX, mouseY, isMouseDown, selectedBody, mouseJoint;
var md = new Box2D.Dynamics.Joints.b2MouseJointDef();
var mousePVec = Box2D.Common.Math.b2Vec2.Get(0,0);

var getElementPosition = function(element) {
    var elem = element,
        tagname = "",
        x = 0,
        y = 0;
    while ((typeof(elem) == "object") && (typeof(elem.tagName) != "undefined")) {
        y += elem.offsetTop;
        x += elem.offsetLeft;
        tagname = elem.tagName.toUpperCase();
        if (tagname == "BODY") elem = 0;
        if (typeof(elem) == "object") {
            if (typeof(elem.offsetParent) == "object") elem = elem.offsetParent;
        }
    }
    return {
        x: x,
        y: y
    };
};

var canvasPosition = getElementPosition(document.getElementById("canvas"));

var handleMouseMove = function(e) {
    mouseX = (e.clientX - canvasPosition.x) / 10;
    mouseY = (e.clientY - canvasPosition.y) / 10;
};

var getBodyCB = function(fixture) {
    if(fixture.GetBody().GetType() != Box2D.Dynamics.b2BodyDef.b2_staticBody) {
        if(fixture.GetShape().TestPoint(fixture.GetBody().GetTransform(), mousePVec)) {
            selectedBody = fixture.GetBody();
            return false;
        }
    }
    return true;
};

var mjAABB = Box2D.Collision.b2AABB.Get()
var getBodyAtMouse = function() {
    mousePVec.x = mouseX;
    mousePVec.y = mouseY;
    mjAABB.lowerBound.Set(mouseX - 0.001, mouseY - 0.001);
    mjAABB.upperBound.Set(mouseX + 0.001, mouseY + 0.001);
    
    selectedBody = null;
    world.QueryAABB(getBodyCB, mjAABB);
    return selectedBody;
};


document.addEventListener("mousedown", function(e) {
    isMouseDown = true;
    handleMouseMove(e);
    document.addEventListener("mousemove", handleMouseMove, true);
}, true);

document.addEventListener("mouseup", function() {
    document.removeEventListener("mousemove", handleMouseMove, true);
    isMouseDown = false;
    mouseX = undefined;
    mouseY = undefined;
}, true);

updateCalls.push(function() {
    if (isMouseDown && (!mouseJoint)) {
        var body = getBodyAtMouse();
        if (body) {
            md.bodyA = world.GetGroundBody();
            md.bodyB = body;
            md.target.Set(mouseX, mouseY);
            md.collideConnected = true;
            md.maxForce = 300.0 * body.GetMass();
            mouseJoint = world.CreateJoint(md);
            body.SetAwake(true);
        }
    }
    if (mouseJoint) {
        if (isMouseDown) {
            mousePVec.Set(mouseX, mouseY);
            mouseJoint.SetTarget(mousePVec);
        } else {
            world.DestroyJoint(mouseJoint);
            mouseJoint = null;
        }
    }
});