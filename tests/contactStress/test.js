(function(){
    var bodyDef = new Box2D.Dynamics.b2BodyDef();
    bodyDef.type = Box2D.Dynamics.b2BodyDef.b2_dynamicBody;
    
    var fixDef = new Box2D.Dynamics.b2FixtureDef();
    fixDef.density = 1.0;
    fixDef.friction = 0.5;
    fixDef.restitution = 2.0;
    fixDef.shape = new Box2D.Collision.Shapes.b2PolygonShape();
    
    
    var createSpinner = function(center, size) {
        var halfSize = new Box2D.Common.Math.b2Vec2(size.x / 2, size.y / 2);
        var plankSize = new Box2D.Common.Math.b2Vec2(halfSize.x, halfSize.y / 4);
        
        var bodyDef = new Box2D.Dynamics.b2BodyDef();
        bodyDef.type = Box2D.Dynamics.b2BodyDef.b2_dynamicBody;
        
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
    
    for(var i = 0; i < 10; ++i) {
        if(i % 2 === 0) {
            createSpinner(getRandomPos(), new Box2D.Common.Math.b2Vec2(2, 2));
            fixDef.shape = new Box2D.Collision.Shapes.b2PolygonShape();
            fixDef.shape.SetAsBox(Math.random() + 0.1, Math.random() + 0.1);
        } else {
            createSnake(getRandomPos(), new Box2D.Common.Math.b2Vec2(2.5, 0.5));
            fixDef.shape = new Box2D.Collision.Shapes.b2CircleShape(Math.random() + 0.1);
        }
        bodyDef.position = getRandomPos();
        world.CreateBody(bodyDef).CreateFixture(fixDef);
    }
    
    
})();