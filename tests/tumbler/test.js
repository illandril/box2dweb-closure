(function(){

    var fixDef = new Box2D.Dynamics.b2FixtureDef();
    fixDef.shape = new Box2D.Collision.Shapes.b2PolygonShape();
    
    var bodyDef = new Box2D.Dynamics.b2BodyDef();
    var ground = world.CreateBody(bodyDef);
    
    
    fixDef.density = 5;
    bodyDef.allowSleep = false;
    bodyDef.position.Set(30, 20);
    bodyDef.type = Box2D.Dynamics.b2BodyDef.b2_dynamicBody;
    
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
    updateCalls.push(function() {
        if ( stepsLeft > 0 ) {
            world.CreateBody(bodyDef).CreateFixture(fixDef);
            stepsLeft--;
        }
    });
})();