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
 
 (function(){
    var worldWidth = 60;
    var worldHeight = 40;
    
    var fixDef = new Box2D.Dynamics.b2FixtureDef();
    fixDef.density = 1.0;
    fixDef.friction = 0.5;
    fixDef.restitution = 0.2;
    fixDef.shape = new Box2D.Collision.Shapes.b2PolygonShape();
    fixDef.shape.SetAsBox(0.2, 40);
    
    var bodyDef = new Box2D.Dynamics.b2BodyDef();

    
    
    var controllers = [];
    
    var accelController = new Box2D.Dynamics.Controllers.b2ConstantAccelController();
    accelController.A = new Box2D.Common.Math.b2Vec2(1.75,-9.5);
    world.AddController(accelController);
    controllers.push(accelController);
    
    var forceController = new Box2D.Dynamics.Controllers.b2ConstantForceController();
    forceController.F = new Box2D.Common.Math.b2Vec2(1.75,-1.5);
    world.AddController(forceController);
    controllers.push(forceController);
    
    var gravController = new Box2D.Dynamics.Controllers.b2GravityController();
    gravController.G = 10;
    world.AddController(gravController);
    controllers.push(gravController);
    
    var bouyancyController = new Box2D.Dynamics.Controllers.b2BuoyancyController();
    bouyancyController.offset = -1 * worldHeight / 2;
    bouyancyController.density = 1;
    
    world.AddController(bouyancyController);
    controllers.push(bouyancyController);
    
    var tensorDampingController = new Box2D.Dynamics.Controllers.b2TensorDampingController();
    tensorDampingController.SetAxisAligned(1,1);
    world.AddController(tensorDampingController);
    controllers.push(tensorDampingController);
    
    bodyDef.type = Box2D.Dynamics.b2BodyDef.b2_staticBody;
    var cageWidth = worldWidth / controllers.length;
    for (var i = 0; i < controllers.length - 1; i++) {
        bodyDef.position.Set(cageWidth * (i + 1), worldHeight / 2);
        world.CreateBody(bodyDef).CreateFixture(fixDef);
    }
    
    bodyDef.type = Box2D.Dynamics.b2BodyDef.b2_dynamicBody;
    var testObjectCount = 0;
    var getRandomizedObject = function(x) {
        fixDef.density = 0.5 + Math.random();
        fixDef.restitution = 0.2 + Math.random();
        testObjectCount++;
        if(testObjectCount % 2 === 0) {
            fixDef.shape = new Box2D.Collision.Shapes.b2PolygonShape();
            fixDef.shape.SetAsBox(Math.random() * 0.5 + 0.1, Math.random() * 0.5 + 0.1);
        } else {
            fixDef.shape = new Box2D.Collision.Shapes.b2CircleShape(Math.random() * 0.5 + 0.1);
        }
        bodyDef.position.Set(cageWidth * x + Math.random() * (cageWidth - 0.5) + 0.25, Math.random() * (worldHeight - 1) + 0.5);
        var body = world.CreateBody(bodyDef);
        body.CreateFixture(fixDef);
        return body;
    };
    
    window.randomizeSystem = function() {
        for (var i = 0; i < controllers.length; i++) {
            var controller = controllers[i];
            var removeAll = Math.random() < 0.1;
            for (var bodyNode = controller.GetBodyList().GetFirstNode(Box2D.Dynamics.b2BodyList.TYPES.allBodies); bodyNode; bodyNode = bodyNode.GetNextNode()) {
                if (removeAll || Math.random() < 0.50) {
                    world.DestroyBody(bodyNode.body);
                }
            }
            for (var x = 0; x < 10; x++) {
                controller.AddBody(getRandomizedObject(i));
            }
        }
    };
    randomizeSystem();
})();