(function(){
    var bodyDef = new Box2D.Dynamics.b2BodyDef();
    bodyDef.type = Box2D.Dynamics.b2BodyDef.b2_dynamicBody;
    
    var fixDef = new Box2D.Dynamics.b2FixtureDef();
    fixDef.density = 1.0;
    fixDef.friction = 0.5;
    fixDef.restitution = 1.0;
    fixDef.shape = new Box2D.Collision.Shapes.b2PolygonShape();
    for(var i = 0; i < 10; ++i) {
        if(i % 2 === 0) {
           fixDef.shape = new Box2D.Collision.Shapes.b2PolygonShape();
           fixDef.shape.SetAsBox(Math.random() + 0.1, Math.random() + 0.1);
        } else {
           fixDef.shape = new Box2D.Collision.Shapes.b2CircleShape(Math.random() + 0.1);
        }
        bodyDef.position.x = Math.random() * 10;
        bodyDef.position.y = Math.random() * 10;
        world.CreateBody(bodyDef).CreateFixture(fixDef);
    }
})();