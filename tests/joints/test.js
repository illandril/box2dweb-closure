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
    
    var createWeldBox = function(center, size) {
        var halfSize = new Box2D.Common.Math.b2Vec2(size.x / 2, size.y / 2);
        var plankSize = new Box2D.Common.Math.b2Vec2(halfSize.x, halfSize.y / 4);
        
        var bodyDef = new Box2D.Dynamics.b2BodyDef();
        bodyDef.type = Box2D.Dynamics.b2BodyDef.b2_dynamicBody;
        
        var fixDef = new Box2D.Dynamics.b2FixtureDef();
        fixDef.density = 1.0;
        fixDef.friction = 0.5;
        fixDef.restitution = 0.1;
        fixDef.shape = new Box2D.Collision.Shapes.b2PolygonShape();
        
        bodyDef.position.x = center.x - halfSize.x;
        bodyDef.position.y = center.y;
        fixDef.shape.SetAsBox(plankSize.y, plankSize.x);
        var wBoxLeft = world.CreateBody(bodyDef);
        wBoxLeft.CreateFixture(fixDef);
        bodyDef.position.x = center.x + halfSize.x;
        var wBoxRight = world.CreateBody(bodyDef);
        wBoxRight.CreateFixture(fixDef);
        bodyDef.position.x = center.x;
        bodyDef.position.y = center.y - halfSize.y;
        fixDef.shape.SetAsBox(plankSize.x, plankSize.y);
        var wBoxTop = world.CreateBody(bodyDef);
        wBoxTop.CreateFixture(fixDef);
        bodyDef.position.y = center.y + halfSize.y;
        var wBoxBottom = world.CreateBody(bodyDef);
        wBoxBottom.CreateFixture(fixDef);
        
        var weldJointDef = new Box2D.Dynamics.Joints.b2WeldJointDef();
        weldJointDef.Initialize(wBoxLeft, wBoxTop, new Box2D.Common.Math.b2Vec2(center.x - halfSize.x, center.y - halfSize.y));
        world.CreateJoint(weldJointDef);
        weldJointDef.Initialize(wBoxLeft, wBoxBottom, new Box2D.Common.Math.b2Vec2(center.x - halfSize.x, center.y + halfSize.y));
        world.CreateJoint(weldJointDef);
        weldJointDef.Initialize(wBoxRight, wBoxTop, new Box2D.Common.Math.b2Vec2(center.x + halfSize.x, center.y - halfSize.y));
        world.CreateJoint(weldJointDef);
        weldJointDef.Initialize(wBoxRight, wBoxBottom, new Box2D.Common.Math.b2Vec2(center.x + halfSize.x, center.y + halfSize.y));
        world.CreateJoint(weldJointDef);
    };
    
    var createSpinner = function(center, size) {
        var halfSize = new Box2D.Common.Math.b2Vec2(size.x / 2, size.y / 2);
        var plankSize = new Box2D.Common.Math.b2Vec2(halfSize.x, halfSize.y / 4);
        
        var bodyDef = new Box2D.Dynamics.b2BodyDef();
        bodyDef.type = Box2D.Dynamics.b2BodyDef.b2_dynamicBody;
        
        var fixDef = new Box2D.Dynamics.b2FixtureDef();
        fixDef.density = 1.0;
        fixDef.friction = 0.5;
        fixDef.restitution = 0.1;
        fixDef.shape = new Box2D.Collision.Shapes.b2PolygonShape();
        
        bodyDef.position.x = center.x;
        bodyDef.position.y = center.y;
        fixDef.shape.SetAsBox(plankSize.x, plankSize.y);
        var a = world.CreateBody(bodyDef);
        a.CreateFixture(fixDef);
        fixDef.shape.SetAsBox(plankSize.y, plankSize.x);
        var b = world.CreateBody(bodyDef);
        b.CreateFixture(fixDef);
        
        var revoluteJointDef = new Box2D.Dynamics.Joints.b2RevoluteJointDef();
        revoluteJointDef.Initialize(a, b, new Box2D.Common.Math.b2Vec2(center.x, center.y));
        world.CreateJoint(revoluteJointDef);
    };
    
    var createSnake = function(center, size) {
        var chains = 10;
        var halfSize = new Box2D.Common.Math.b2Vec2(size.x / 2, size.y / 2);
        var plankSize = new Box2D.Common.Math.b2Vec2(halfSize.x / chains, halfSize.y);
        
        var bodyDef = new Box2D.Dynamics.b2BodyDef();
        bodyDef.type = Box2D.Dynamics.b2BodyDef.b2_dynamicBody;
        
        var fixDef = new Box2D.Dynamics.b2FixtureDef();
        fixDef.density = 1.0;
        fixDef.friction = 0.5;
        fixDef.restitution = 0.1;
        fixDef.shape = new Box2D.Collision.Shapes.b2CircleShape(plankSize.y);
        
        bodyDef.position.y = center.y;
        var distanceJointDef = new Box2D.Dynamics.Joints.b2DistanceJointDef();
        var b = null;
        for (var i = 0; i < chains; i++) {
            bodyDef.position.x = center.x + plankSize.x * (i - chains / 2) * 2;
            var a = world.CreateBody(bodyDef);
            a.CreateFixture(fixDef);
            if (b !== null) {
                distanceJointDef.Initialize(a, b, a.GetWorldCenter(), b.GetWorldCenter());
                world.CreateJoint(distanceJointDef);
            }
            b = a;
        }
    };
    
    var createGear = function(center, radius) {
        var bodyDef = new Box2D.Dynamics.b2BodyDef();
        bodyDef.type = Box2D.Dynamics.b2BodyDef.b2_dynamicBody;
        bodyDef.position = center.Copy();
        
        var fixDef = new Box2D.Dynamics.b2FixtureDef();
        fixDef.density = 1.0;
        fixDef.friction = 0.5;
        fixDef.restitution = 0.1;
        fixDef.shape = new Box2D.Collision.Shapes.b2PolygonShape();
        fixDef.shape.SetAsBox(radius * 0.5, radius * 0.5);
        //fixDef.shape = new Box2D.Collision.Shapes.b2CircleShape(radius);
        
        var gear = world.CreateBody(bodyDef);
        gear.CreateFixture(fixDef);

        fixDef.shape = new Box2D.Collision.Shapes.b2CircleShape(radius / 10);
        bodyDef.type = Box2D.Dynamics.b2BodyDef.b2_staticBody;
        var axel = world.CreateBody(bodyDef);
        axel.CreateFixture(fixDef);
        
        var revoluteJointDef = new Box2D.Dynamics.Joints.b2RevoluteJointDef();
        revoluteJointDef.Initialize(axel, gear, new Box2D.Common.Math.b2Vec2(center.x, center.y));
        var j = world.CreateJoint(revoluteJointDef);
        
        return j;
    };
    
    var createGear = function(center, radius) {
        var bodyDef = new Box2D.Dynamics.b2BodyDef();
        bodyDef.type = Box2D.Dynamics.b2BodyDef.b2_dynamicBody;
        bodyDef.position = center.Copy();
        
        var fixDef = new Box2D.Dynamics.b2FixtureDef();
        fixDef.density = 1.0;
        fixDef.friction = 0.5;
        fixDef.restitution = 0.1;
        fixDef.shape = new Box2D.Collision.Shapes.b2PolygonShape();
        fixDef.shape.SetAsBox(radius * 0.5, radius * 0.5);
        //fixDef.shape = new Box2D.Collision.Shapes.b2CircleShape(radius);
        
        var gear = world.CreateBody(bodyDef);
        gear.CreateFixture(fixDef);

        fixDef.shape = new Box2D.Collision.Shapes.b2CircleShape(radius / 10);
        bodyDef.type = Box2D.Dynamics.b2BodyDef.b2_staticBody;
        var axel = world.CreateBody(bodyDef);
        axel.CreateFixture(fixDef);
        
        var revoluteJointDef = new Box2D.Dynamics.Joints.b2RevoluteJointDef();
        revoluteJointDef.Initialize(axel, gear, new Box2D.Common.Math.b2Vec2(center.x, center.y));
        var j = world.CreateJoint(revoluteJointDef);
        
        return j;
    };
    
    var createPrismGear = function(center, radius) {
        var bodyDef = new Box2D.Dynamics.b2BodyDef();
        bodyDef.type = Box2D.Dynamics.b2BodyDef.b2_dynamicBody;
        bodyDef.position = center.Copy();
        
        var fixDef = new Box2D.Dynamics.b2FixtureDef();
        fixDef.density = 1.0;
        fixDef.friction = 0.5;
        fixDef.restitution = 0.1;
        fixDef.shape = new Box2D.Collision.Shapes.b2PolygonShape();
        fixDef.shape.SetAsBox(radius * 0.5, radius * 0.5);
        //fixDef.shape = new Box2D.Collision.Shapes.b2CircleShape(radius);
        
        var gear = world.CreateBody(bodyDef);
        gear.CreateFixture(fixDef);

        fixDef.shape = new Box2D.Collision.Shapes.b2CircleShape(radius / 10);
        bodyDef.type = Box2D.Dynamics.b2BodyDef.b2_staticBody;
        var axel = world.CreateBody(bodyDef);
        axel.CreateFixture(fixDef);
        
        var prismJointDef = new Box2D.Dynamics.Joints.b2PrismaticJointDef();
        prismJointDef.Initialize(axel, gear, axel.GetWorldCenter(), new Box2D.Common.Math.b2Vec2(1, 0) /* axis */)
        prismJointDef.enableLimit = true;
        prismJointDef.lowerTranslation = -radius * 2;
        prismJointDef.upperTranslation = radius * 2;
        var j = world.CreateJoint(prismJointDef);
        
        return j;
    };
    
    var createGearBox = function(center, radius) {
        var gearCount = 4;
        var gearRadius = radius / gearCount;
        var gearJoint = new Box2D.Dynamics.Joints.b2GearJointDef();
        
        var gears = [];
        for(var x = 0; x < gearCount; x++) {
            if (x == 0) {
                gears.push(createPrismGear(new Box2D.Common.Math.b2Vec2(center.x - radius + gearRadius * x * 2, center.y - radius + gearRadius * (gearCount - 1) * 2), gearRadius));
                for(var y = gearCount - 2; y >= 0; y--) {        
                    gears.push(createGear(new Box2D.Common.Math.b2Vec2(center.x - radius + gearRadius * x * 2, center.y - radius + gearRadius * y * 2), gearRadius));
                }
            } else if(x == (gearCount - 1)) {
                for(var y = 0; y < gearCount - 1; y++) {        
                    gears.push(createGear(new Box2D.Common.Math.b2Vec2(center.x - radius + gearRadius * x * 2, center.y - radius + gearRadius * y * 2), gearRadius));
                }
                gears.push(createPrismGear(new Box2D.Common.Math.b2Vec2(center.x - radius + gearRadius * x * 2, center.y - radius + gearRadius * (gearCount - 1) * 2), gearRadius));
            } else {
                gears.push(createGear(new Box2D.Common.Math.b2Vec2(center.x - radius + gearRadius * x * 2, center.y - radius), gearRadius));
            }
        }
        
        var last = gears[0];
        for(var i = 1; i < gears.length; i++) {
            var ratio = (i - gears.length / 2) / 2;
            if (ratio < 0) {
                ratio = 1 / -(ratio - 1);
            } else {
                ratio += 1;
            }
            console.error( ratio );
            gearJoint.Initialize(last, gears[i], ratio /* ratio */);
            world.CreateJoint(gearJoint);
            last = gears[i];
        }
    };
    
    createWeldBox(new Box2D.Common.Math.b2Vec2(3, 35), new Box2D.Common.Math.b2Vec2(2, 2));
    createSpinner(new Box2D.Common.Math.b2Vec2(8, 35), new Box2D.Common.Math.b2Vec2(2, 2));
    createSnake(new Box2D.Common.Math.b2Vec2(12, 35), new Box2D.Common.Math.b2Vec2(5, 0.5));
    createGearBox(new Box2D.Common.Math.b2Vec2(20, 15), 5);
    
    // TODO: Pulley Joint
    // TODO: Line Joint
    // TODO: Friction Joint
})();