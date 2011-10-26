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