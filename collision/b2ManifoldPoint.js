/*
 * See Box2D.js
 */
goog.provide('Box2D.Collision.b2ManifoldPoint');

goog.require('Box2D.Common.Math.b2Vec2');
goog.require('Box2D.Collision.b2ContactID');

/**
 * @constructor
 */
Box2D.Collision.b2ManifoldPoint = function() {
    this.m_localPoint = new Box2D.Common.Math.b2Vec2(0, 0);
    this.m_id = new Box2D.Collision.b2ContactID();
    if (this.constructor === Box2D.Collision.b2ManifoldPoint) {
        this.Reset();
    }
};

Box2D.Collision.b2ManifoldPoint.prototype.Reset = function() {
    this.m_localPoint.SetZero();
    this.m_normalImpulse = 0.0;
    this.m_tangentImpulse = 0.0;
    this.m_id.key = 0;
};

Box2D.Collision.b2ManifoldPoint.prototype.Set = function(m) {
    this.m_localPoint.SetV(m.m_localPoint);
    this.m_normalImpulse = m.m_normalImpulse;
    this.m_tangentImpulse = m.m_tangentImpulse;
    this.m_id.Set(m.m_id);
};