/*
 * See Box2D.js
 */
goog.provide('Box2D.Dynamics.Contacts.b2PolyAndCircleContact');

goog.require('Box2D.Dynamics.Contacts.b2Contact');
goog.require('Box2D.Collision.b2Collision');
goog.require('Box2D.Common.b2Settings');
goog.require('Box2D.Collision.Shapes.b2PolygonShape');
goog.require('Box2D.Collision.Shapes.b2CircleShape');

/**
 * @constructor
 * @extends {Box2D.Dynamics.Contacts.b2Contact}
 */
Box2D.Dynamics.Contacts.b2PolyAndCircleContact = function() {
    Box2D.Dynamics.Contacts.b2Contact.call(this);
};
goog.inherits(Box2D.Dynamics.Contacts.b2PolyAndCircleContact, Box2D.Dynamics.Contacts.b2Contact);

Box2D.Dynamics.Contacts.b2PolyAndCircleContact.Create = function() {
    return new Box2D.Dynamics.Contacts.b2PolyAndCircleContact();
};

Box2D.Dynamics.Contacts.b2PolyAndCircleContact.prototype.Reset = function(fixtureA, fixtureB) {
    Box2D.Common.b2Settings.b2Assert(fixtureA.GetShape() instanceof Box2D.Collision.Shapes.b2PolygonShape);
    Box2D.Common.b2Settings.b2Assert(fixtureB.GetShape() instanceof Box2D.Collision.Shapes.b2CircleShape);
    Box2D.Dynamics.Contacts.b2Contact.prototype.Reset.call(this, fixtureA, fixtureB);
};

Box2D.Dynamics.Contacts.b2PolyAndCircleContact.prototype.Evaluate = function() {
    Box2D.Collision.b2Collision.CollidePolygonAndCircle(this.m_manifold, this.m_fixtureA.GetShape(), this.m_fixtureA.GetBody().m_xf, this.m_fixtureB.GetShape(), this.m_fixtureB.GetBody().m_xf);
};
