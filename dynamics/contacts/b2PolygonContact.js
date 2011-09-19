/*
 * See Box2D.js
 */
goog.provide('Box2D.Dynamics.Contacts.b2PolygonContact');

goog.require('Box2D.Dynamics.Contacts.b2Contact');
goog.require('Box2D.Collision.b2Collision');

/**
 * @constructor
 * @extends {Box2D.Dynamics.Contacts.b2Contact}
 */
Box2D.Dynamics.Contacts.b2PolygonContact = function() {
    Box2D.Dynamics.Contacts.b2Contact.call(this);
};
goog.inherits(Box2D.Dynamics.Contacts.b2PolygonContact, Box2D.Dynamics.Contacts.b2Contact);

Box2D.Dynamics.Contacts.b2PolygonContact.Create = function() {
    return new Box2D.Dynamics.Contacts.b2PolygonContact();
};

Box2D.Dynamics.Contacts.b2PolygonContact.prototype.Reset = function(fixtureA, fixtureB) {
    Box2D.Dynamics.Contacts.b2Contact.prototype.Reset.call(this, fixtureA, fixtureB);
};

Box2D.Dynamics.Contacts.b2PolygonContact.prototype.Evaluate = function() {
    Box2D.Collision.b2Collision.CollidePolygons(this.m_manifold, this.m_fixtureA.GetShape(), this.m_fixtureA.GetBody().m_xf, this.m_fixtureB.GetShape(), this.m_fixtureB.GetBody().m_xf);
};
