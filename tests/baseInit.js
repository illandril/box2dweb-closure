var updateCalls = [];
var world = new Box2D.Dynamics.b2World(new Box2D.Common.Math.b2Vec2(0, 9.8) /* gravity */, true /* allowSleep */);
var doDebug = true;
(function(){
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
    debugDraw.SetFlags(Box2D.Dynamics.b2DebugDraw.e_shapeBit | Box2D.Dynamics.b2DebugDraw.e_jointBit | Box2D.Dynamics.b2DebugDraw.e_aabbBit);
    world.SetDebugDraw(debugDraw);
    
     var update = function() {
        for (var i = 0; i < updateCalls.length; i++) {
            updateCalls[i]();
        }
        world.Step(1 / 60, 10, 10);
        if (doDebug) {
            world.DrawDebugData();
        }
        world.ClearForces();
     };
     window.setInterval(update, 1000 / 60);

})();