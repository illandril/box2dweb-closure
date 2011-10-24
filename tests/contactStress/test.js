(function(){
    var bodyDef = new Box2D.Dynamics.b2BodyDef();
    bodyDef.type = Box2D.Dynamics.b2BodyDef.b2_staticBody;
    
    var fixDef = new Box2D.Dynamics.b2FixtureDef();
    var maxRestitution = 2;
    
    var radius = 1.25;
    var step = radius * 5;
    fixDef.shape = new Box2D.Collision.Shapes.b2CircleShape(radius);
    for (var x = 0; x < 60; x += step) {
        bodyDef.position.Set(x, 40 - radius * 1.5);
        world.CreateBody(bodyDef).CreateFixture(fixDef);
        bodyDef.position.Set(x + radius * 2, 25);
        world.CreateBody(bodyDef).CreateFixture(fixDef);
        bodyDef.position.Set(x + radius * 3, 25);
        world.CreateBody(bodyDef).CreateFixture(fixDef);
        bodyDef.position.Set(x, 15);
        world.CreateBody(bodyDef).CreateFixture(fixDef);
    }
    var minKinSpeed = 5;
    var maxKinSpeed = 5;
    bodyDef.type = Box2D.Dynamics.b2BodyDef.b2_kinematicBody;
    bodyDef.position.Set(0, 10);
    fixDef.shape = new Box2D.Collision.Shapes.b2CircleShape(1);
    var kBody1 = world.CreateBody(bodyDef);
    kBody1.CreateFixture(fixDef);
    updateCalls.push(function(){
        if(kBody1.GetPosition().x < 5) {
            kBody1.m_linearVelocity.x = minKinSpeed + Math.random() * maxKinSpeed;
            kBody1.m_angularVelocity = Math.random() * maxKinSpeed - maxKinSpeed / 2;
        }
        if(kBody1.GetPosition().x > 55) {
            kBody1.m_linearVelocity.x = -1 * (minKinSpeed + Math.random() * maxKinSpeed);
            kBody1.m_angularVelocity = Math.random() * maxKinSpeed - maxKinSpeed / 2;
        }
    });
    
    bodyDef.position.Set(0, 5);
    fixDef.shape = new Box2D.Collision.Shapes.b2PolygonShape();
    fixDef.shape.SetAsBox(1, 1);
    var kBody2 = world.CreateBody(bodyDef);
    kBody2.CreateFixture(fixDef);
    updateCalls.push(function(){
        if(kBody2.GetPosition().x < 5) {
            kBody2.m_linearVelocity.x = minKinSpeed + Math.random() * maxKinSpeed;
            kBody2.m_angularVelocity = Math.random() * maxKinSpeed - maxKinSpeed / 2;
        }
        if(kBody2.GetPosition().x > 55) {
            kBody2.m_linearVelocity.x = -1 * (minKinSpeed + Math.random() * maxKinSpeed);
            kBody2.m_angularVelocity = Math.random() * maxKinSpeed - maxKinSpeed / 2;
        }
    });
    
    bodyDef.position.Set(0, 20);
    fixDef.shape = new Box2D.Collision.Shapes.b2PolygonShape();
    fixDef.shape.SetAsArray([new Box2D.Common.Math.b2Vec2(0,0),new Box2D.Common.Math.b2Vec2(1,0),new Box2D.Common.Math.b2Vec2(0,1)]);
    var kBody3 = world.CreateBody(bodyDef);
    kBody3.CreateFixture(fixDef);
    updateCalls.push(function(){
        if(kBody3.GetPosition().x < 5) {
            kBody3.m_linearVelocity.x = minKinSpeed + Math.random() * maxKinSpeed;
            kBody3.m_angularVelocity = Math.random() * maxKinSpeed - maxKinSpeed / 2;
        }
        if(kBody3.GetPosition().x > 55) {
            kBody3.m_linearVelocity.x = -1 * (minKinSpeed + Math.random() * maxKinSpeed);
            kBody3.m_angularVelocity = Math.random() * maxKinSpeed - maxKinSpeed / 2;
        }
    });
    
    bodyDef.type = Box2D.Dynamics.b2BodyDef.b2_dynamicBody;
    fixDef.density = 1.0;
    fixDef.friction = 0.5;
    fixDef.shape = new Box2D.Collision.Shapes.b2PolygonShape();
    
    var createSpinner = function(center, size) {
        var halfSize = new Box2D.Common.Math.b2Vec2(size.x / 2, size.y / 2);
        var plankSize = new Box2D.Common.Math.b2Vec2(halfSize.x, halfSize.y / 4);
        
        var bodyDef = new Box2D.Dynamics.b2BodyDef();
        bodyDef.type = Box2D.Dynamics.b2BodyDef.b2_dynamicBody;
        
        fixDef.restitution = Math.random() * maxRestitution;
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
        var chains = 5;
        var halfSize = new Box2D.Common.Math.b2Vec2(size.x / 2, size.y / 2);
        var plankSize = new Box2D.Common.Math.b2Vec2(halfSize.x / chains, halfSize.y);
        
        var bodyDef = new Box2D.Dynamics.b2BodyDef();
        bodyDef.type = Box2D.Dynamics.b2BodyDef.b2_dynamicBody;
        
        fixDef.restitution = Math.random() * maxRestitution;
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
    
    var getRandomPos = function() {
        return new Box2D.Common.Math.b2Vec2(Math.random() * 58 + 1, Math.random() * 38 + 1);
    };
    
    var testObjectCount = 0;
    var testObjects = [];
    window.addContactStressTestObjects = function(count) {
        for (var i = 0; i < count; i++) {
            testObjectCount++;
            if(testObjectCount % 2 === 0) {
                fixDef.shape = new Box2D.Collision.Shapes.b2PolygonShape();
                fixDef.shape.SetAsBox(Math.random() * 0.5 + 0.1, Math.random() * 0.5 + 0.1);
            } else {
                fixDef.shape = new Box2D.Collision.Shapes.b2CircleShape(Math.random() * 0.5 + 0.1);
            }
            bodyDef.position = getRandomPos();
            fixDef.restitution = Math.random() * maxRestitution;
            var body = world.CreateBody(bodyDef);
            body.CreateFixture(fixDef);
            testObjects.push(body);
        }
    };
    
    window.deleteContactStressTestObjects = function(count) {
        for (var i = 0; i < count; i++) {
            var index = Math.floor(Math.random() * testObjects.length);
            var body = testObjects.splice(index,1)[0];
            world.DestroyBody(body);
        }
    };
    
    for(var i = 0; i < 20; ++i) {
        if(i % 2 === 0) {
            createSpinner(getRandomPos(), new Box2D.Common.Math.b2Vec2(1, 1));
        } else {
            createSnake(getRandomPos(), new Box2D.Common.Math.b2Vec2(1.25, 0.25));
        }
        addContactStressTestObjects(10);
    }
    
})();