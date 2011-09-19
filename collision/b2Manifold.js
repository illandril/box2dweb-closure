/*
 * See Box2D.js
 */
goog.provide('Box2D.Collision.b2Manifold');

goog.require('Box2D.Common.b2Settings');
goog.require('Box2D.Common.Math.b2Vec2');
goog.require('Box2D.Collision.b2ManifoldPoint');

/**
 * @constructor
 */
Box2D.Collision.b2Manifold = function() {
    this.m_pointCount = 0;
    if (this.constructor === Box2D.Collision.b2Manifold) {
        this.m_points = [];
        for (var i = 0; i < Box2D.Common.b2Settings.b2_maxManifoldPoints; i++) {
            this.m_points[i] = new Box2D.Collision.b2ManifoldPoint();
        }
        this.m_localPlaneNormal = new Box2D.Common.Math.b2Vec2(0, 0);
        this.m_localPoint = new Box2D.Common.Math.b2Vec2(0, 0);
    }
};

Box2D.Collision.b2Manifold.prototype.Reset = function() {
    for (var i = 0; i < Box2D.Common.b2Settings.b2_maxManifoldPoints; i++) {
        this.m_points[i].Reset();
    }
    this.m_localPlaneNormal.SetZero();
    this.m_localPoint.SetZero();
    this.m_type = 0;
    this.m_pointCount = 0;
};

Box2D.Collision.b2Manifold.prototype.Set = function(m) {
    this.m_pointCount = m.m_pointCount;
    for (var i = 0; i < Box2D.Common.b2Settings.b2_maxManifoldPoints; i++) {
        this.m_points[i].Set(m.m_points[i]);
    }
    this.m_localPlaneNormal.SetV(m.m_localPlaneNormal);
    this.m_localPoint.SetV(m.m_localPoint);
    this.m_type = m.m_type;
};

Box2D.Collision.b2Manifold.prototype.Copy = function() {
    var copy = new Box2D.Collision.b2Manifold();
    copy.Set(this);
    return copy;
};

Box2D.Collision.b2Manifold.e_circles = 0x0001;
Box2D.Collision.b2Manifold.e_faceA = 0x0002;
Box2D.Collision.b2Manifold.e_faceB = 0x0004;
