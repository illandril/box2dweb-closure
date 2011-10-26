(function(){
    var bouyancyController = new Box2D.Dynamics.Controllers.b2BuoyancyController();
    bouyancyController.offset = -35;
    bouyancyController.density = 4;
    world.AddController(bouyancyController);
    
    var fixDef = new Box2D.Dynamics.b2FixtureDef();
    fixDef.density = 1.0;
    fixDef.friction = 0.5;
    fixDef.restitution = 0.2;
    fixDef.shape = new Box2D.Collision.Shapes.b2PolygonShape();
    fixDef.shape.SetAsBox(22.5, 0.2);
    
    var bodyDef = new Box2D.Dynamics.b2BodyDef();
    bodyDef.type = Box2D.Dynamics.b2BodyDef.b2_staticBody;
    bodyDef.position.Set(22.5, 34);
    world.CreateBody(bodyDef).CreateFixture(fixDef);
    
    bodyDef.type = Box2D.Dynamics.b2BodyDef.b2_dynamicBody;
    
    var setup = [];
    var addBody = function() {
        var body = world.CreateBody(bodyDef);
        body.CreateFixture(fixDef);
        bouyancyController.AddBody(body);
        setup.push(body);
        return body;
    };
    
    var minX = 30;
    var minY = 0;
    var randomX = 10;
    var randomY = 14;
    
    window.resetSystem = function() {
        for (var i = 0; i < setup.length; i++) {
            world.DestroyBody(setup[i]);
        }
        setup = [];
        
        fixDef.shape = new Box2D.Collision.Shapes.b2PolygonShape();
        fixDef.shape.SetAsBox(0.25, 2);
        bodyDef.position.Set(32, 32);
        addBody();
        
        bodyDef.position.Set(34, 27.5);
        addBody();
        
        bodyDef.position.Set(36, 32);
        addBody();
        
        bodyDef.position.Set(36, 23);
        addBody();
        
        bodyDef.position.Set(38, 27.5);
        addBody();
        
        bodyDef.position.Set(40, 32);
        addBody();
        
        bodyDef.position.Set(40, 23);
        addBody();
        
        bodyDef.position.Set(42, 27.5);
        addBody();

        bodyDef.position.Set(44, 32);
        addBody();
        
        
        fixDef.shape.SetAsBox(2, 0.25);
        bodyDef.position.Set(34, 29.5);
        addBody();
        
        bodyDef.position.Set(36, 25);
        addBody();
        
        bodyDef.position.Set(38, 29.5);
        addBody();
        
        bodyDef.position.Set(38, 20.5);
        addBody();
        
        bodyDef.position.Set(40, 25);
        addBody();
        
        bodyDef.position.Set(42, 29.5);
        addBody();
        
        bodyDef.position.Set(5, 29.5);
        fixDef.shape = new Box2D.Collision.Shapes.b2PolygonShape();
        fixDef.shape = new Box2D.Collision.Shapes.b2CircleShape(0.5);
        var launched = addBody();
        launched.ApplyImpulse(new Box2D.Common.Math.b2Vec2(minX + randomX * Math.random(), -1 * (minY + randomY * Math.random())), launched.GetWorldCenter());
    };
    resetSystem();
})();