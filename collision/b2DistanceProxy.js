/*
 * See Box2D.js
 */
goog.provide('Box2D.Collision.b2DistanceProxy');

goog.require('Box2D.Common.b2Settings');

/**
 * @constructor
 */
Box2D.Collision.b2DistanceProxy = function() {};

Box2D.Collision.b2DistanceProxy.prototype.Set = function (shape) {
    shape.SetDistanceProxy(this);
};

Box2D.Collision.b2DistanceProxy.prototype.GetSupport = function (d) {
    var bestIndex = 0;
    var bestValue = this.m_vertices[0].x * d.x + this.m_vertices[0].y * d.y;
    for (var i = 1; i < this.m_count; i++) {
        var value = this.m_vertices[i].x * d.x + this.m_vertices[i].y * d.y;
        if (value > bestValue) {
            bestIndex = i;
            bestValue = value;
        }
    }
    return bestIndex;
};

Box2D.Collision.b2DistanceProxy.prototype.GetSupportVertex = function (d) {
    return this.m_vertices[this.GetSupport(d)];
};

Box2D.Collision.b2DistanceProxy.prototype.GetVertexCount = function () {
    return this.m_count;
};

Box2D.Collision.b2DistanceProxy.prototype.GetVertex = function (index) {
    if (index === undefined) index = 0;
    Box2D.Common.b2Settings.b2Assert(0 <= index && index < this.m_count);
    return this.m_vertices[index];
};
