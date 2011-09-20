var mouseX, mouseY, mousePVec, isMouseDown, selectedBody, mouseJoint;

var getElementPosition = function(element) {
    var elem = element,
        tagname = "",
        x = 0,
        y = 0;
    while ((typeof(elem) == "object") && (typeof(elem.tagName) != "undefined")) {
        y += elem.offsetTop;
        x += elem.offsetLeft;
        tagname = elem.tagName.toUpperCase();
        if (tagname == "BODY") elem = 0;
        if (typeof(elem) == "object") {
            if (typeof(elem.offsetParent) == "object") elem = elem.offsetParent;
        }
    }
    return {
        x: x,
        y: y
    };
};

var canvasPosition = getElementPosition(document.getElementById("canvas"));

var handleMouseMove = function(e) {
    mouseX = (e.clientX - canvasPosition.x) / 10;
    mouseY = (e.clientY - canvasPosition.y) / 10;
};

var getBodyCB = function(fixture) {
    if(fixture.GetBody().GetType() != Box2D.Dynamics.b2BodyDef.b2_staticBody) {
        if(fixture.GetShape().TestPoint(fixture.GetBody().GetTransform(), mousePVec)) {
            selectedBody = fixture.GetBody();
            return false;
        }
    }
    return true;
};

var getBodyAtMouse = function() {
    mousePVec = new Box2D.Common.Math.b2Vec2(mouseX, mouseY);
    var aabb = new Box2D.Collision.b2AABB();
    aabb.lowerBound.Set(mouseX - 0.001, mouseY - 0.001);
    aabb.upperBound.Set(mouseX + 0.001, mouseY + 0.001);
    
    selectedBody = null;
    world.QueryAABB(getBodyCB, aabb);
    return selectedBody;
};


document.addEventListener("mousedown", function(e) {
    isMouseDown = true;
    handleMouseMove(e);
    document.addEventListener("mousemove", handleMouseMove, true);
}, true);

document.addEventListener("mouseup", function() {
    document.removeEventListener("mousemove", handleMouseMove, true);
    isMouseDown = false;
    mouseX = undefined;
    mouseY = undefined;
}, true);

updateCalls.push(function() {
    if (isMouseDown && (!mouseJoint)) {
        var body = getBodyAtMouse();
        if (body) {
            var md = new Box2D.Dynamics.Joints.b2MouseJointDef();
            md.bodyA = world.GetGroundBody();
            md.bodyB = body;
            md.target.Set(mouseX, mouseY);
            md.collideConnected = true;
            md.maxForce = 300.0 * body.GetMass();
            mouseJoint = world.CreateJoint(md);
            body.SetAwake(true);
        }
    }
    if (mouseJoint) {
        if (isMouseDown) {
            mouseJoint.SetTarget(new Box2D.Common.Math.b2Vec2(mouseX, mouseY));
        } else {
            world.DestroyJoint(mouseJoint);
            mouseJoint = null;
        }
    }
});