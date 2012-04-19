/*
* Copyright (c) 2011 Erin Catto http://www.box2d.org
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
    var fixDef = new Box2D.Dynamics.b2FixtureDef();
    fixDef.shape = new Box2D.Collision.Shapes.b2PolygonShape();
    
    var bodyDef = new Box2D.Dynamics.b2BodyDef();
    var ground = world.CreateBody(bodyDef);
    
    
    fixDef.density = 5;
    bodyDef.allowSleep = false;
    bodyDef.position.Set(30, 20);
    bodyDef.type = Box2D.Dynamics.b2Body.b2_dynamicBody;
    
    var body = world.CreateBody(bodyDef);
    fixDef.shape.SetAsOrientedBox(0.5, 10, new Box2D.Common.Math.b2Vec2(10, 0), 0);
    body.CreateFixture(fixDef);
    fixDef.shape.SetAsOrientedBox(0.5, 10, new Box2D.Common.Math.b2Vec2(-10, 0), 0);
    body.CreateFixture(fixDef);
    fixDef.shape.SetAsOrientedBox(10, 0.5, new Box2D.Common.Math.b2Vec2(0, 10), 0);
    body.CreateFixture(fixDef);
    fixDef.shape.SetAsOrientedBox(10, 0.5, new Box2D.Common.Math.b2Vec2(0, -10), 0);
    body.CreateFixture(fixDef);
    
    var jd = new Box2D.Dynamics.Joints.b2RevoluteJointDef();
    jd.bodyA = ground;
    jd.bodyB = body;
    jd.localAnchorA.Set(30.0, 20.0);
    jd.localAnchorB.Set(0.0, 0.0);
    jd.referenceAngle = 0.0;
    jd.motorSpeed = 0.05 * Math.PI;
    jd.maxMotorTorque = 1e8;
    jd.enableMotor = true;
    world.CreateJoint(jd);
                        
    fixDef.density = 1;
    fixDef.shape.SetAsBox(0.125, 0.125);
    bodyDef.allowSleep = true;
    
    var stepsLeft = 800;
    var fpsStepsLeft = stepsLeft;
    updateCalls.push(function() {
        if (stepsLeft > 0) {
            world.CreateBody(bodyDef).CreateFixture(fixDef);
            stepsLeft--;
            if (stepsLeft == 0) {
                markFPS();
                resetMinMaxFPS();
            }
        } else if (fpsStepsLeft > 0) {
            fpsStepsLeft--;
            if (fpsStepsLeft == 0) {
                markFPS();
            }
        }
    });
})();