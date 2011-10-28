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
 
 var updateCalls = [];
var world = new Box2D.Dynamics.b2World(new Box2D.Common.Math.b2Vec2(0, 9.8) /* gravity */, true /* allowSleep */);
var doDebug = true;
(function(){
    var targetFPS = 200;
    var fixDef = new Box2D.Dynamics.b2FixtureDef();
    fixDef.density = 1.0;
    fixDef.friction = 0.5;
    fixDef.restitution = 0.2;
    
    var bodyDef = new Box2D.Dynamics.b2BodyDef();
    bodyDef.type = Box2D.Dynamics.b2BodyDef.b2_staticBody;
    fixDef.shape = new Box2D.Collision.Shapes.b2PolygonShape();
    fixDef.shape.SetAsBox(60, 2);
    bodyDef.position.Set(30, 40 + 1.8);
    world.CreateBody(bodyDef).CreateFixture(fixDef);
    bodyDef.position.Set(30, -1.8);
    world.CreateBody(bodyDef).CreateFixture(fixDef);
    fixDef.shape.SetAsBox(2, 40);
    bodyDef.position.Set(-1.8, 20);
    world.CreateBody(bodyDef).CreateFixture(fixDef);
    bodyDef.position.Set(60 + 1.8, 20);
    world.CreateBody(bodyDef).CreateFixture(fixDef);
    var debugCanvas = document.getElementById("canvas");
    var debugDraw = new Box2D.Dynamics.b2DebugDraw();
    debugDraw.SetSprite(debugCanvas.getContext("2d"));
    debugDraw.SetDrawScale(10.0);
    debugDraw.SetFillAlpha(0.5);
    debugDraw.SetLineThickness(1.0);
    debugDraw.SetFlags(0
        | Box2D.Dynamics.b2DebugDraw.e_shapeBit
        | Box2D.Dynamics.b2DebugDraw.e_jointBit
        | Box2D.Dynamics.b2DebugDraw.e_aabbBit
//        | Box2D.Dynamics.b2DebugDraw.e_pairBit
//        | Box2D.Dynamics.b2DebugDraw.e_centerOfMassBit
        | Box2D.Dynamics.b2DebugDraw.e_controllerBit
    );
    world.SetDebugDraw(debugDraw);
    
     var update = function() {
        for (var i = 0; i < updateCalls.length; i++) {
            updateCalls[i]();
        }
        world.Step(1 / targetFPS, 10, 10);
        if (doDebug) {
            world.DrawDebugData();
        }
        world.ClearForces();
     };
     window.setInterval(update, 1000 / targetFPS);

})();