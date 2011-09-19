/*
 * See Box2D.js
 */

goog.provide('Box2D.Collision.b2Distance');

goog.require('Box2D.Common.b2Settings');
goog.require('Box2D.Common.Math.b2Math');
goog.require('Box2D.Common.Math.b2Vec2');
goog.require('Box2D.Collision.b2Simplex');

Box2D.Collision.b2Distance = {};

/**
 * @param {!Box2D.Collision.b2DistanceOutput} output
 * @param {!Box2D.Collision.b2SimplexCache} cache
 * @param {!Box2D.Collision.b2DistanceInput} input
 */
Box2D.Collision.b2Distance.Distance = function(output, cache, input) {
    var s_simplex = new Box2D.Collision.b2Simplex();
    s_simplex.ReadCache(cache, input.proxyA, input.transformA, input.proxyB, input.transformB);
    if (s_simplex.m_count < 1 || s_simplex.m_count > 3) {
        Box2D.Common.b2Settings.b2Assert(false);
    }
    var iter = 0;
    while (iter < 20) {
        var save = [];
        for (var i = 0; i < s_simplex.m_count; i++) {
            save[i] = {};
            save[i].indexA = s_simplex.m_vertices[i].indexA;
            save[i].indexB = s_simplex.m_vertices[i].indexB;
        }
        if (s_simplex.m_count == 2) {
            s_simplex.Solve2();
        } else if (s_simplex.m_count == 3) {
            s_simplex.Solve3();
        }
        if (s_simplex.m_count == 3) {
            // m_count can be changed by s_simplex.Solve3/Solve2
            break;
        }
        var d = s_simplex.GetSearchDirection();
        if (d.LengthSquared() < Box2D.Consts.MIN_VALUE_SQUARED) {
            break;
        }
        s_simplex.m_vertices[s_simplex.m_count].indexA = input.proxyA.GetSupport(Box2D.Common.Math.b2Math.MulTMV(input.transformA.R, d.GetNegative()));
        s_simplex.m_vertices[s_simplex.m_count].wA = Box2D.Common.Math.b2Math.MulX(input.transformA, input.proxyA.GetVertex(s_simplex.m_vertices[s_simplex.m_count].indexA));
        s_simplex.m_vertices[s_simplex.m_count].indexB = input.proxyB.GetSupport(Box2D.Common.Math.b2Math.MulTMV(input.transformB.R, d));
        s_simplex.m_vertices[s_simplex.m_count].wB = Box2D.Common.Math.b2Math.MulX(input.transformB, input.proxyB.GetVertex(s_simplex.m_vertices[s_simplex.m_count].indexB));
        s_simplex.m_vertices[s_simplex.m_count].w = Box2D.Common.Math.b2Math.SubtractVV(s_simplex.m_vertices[s_simplex.m_count].wB, s_simplex.m_vertices[s_simplex.m_count].wA);

        iter++;
        var duplicate = false;
        for (var i = 0; i < save.length; i++) {
            if (s_simplex.m_vertices[s_simplex.m_count].indexA == save[i].indexA && s_simplex.m_vertices[s_simplex.m_count].indexB == save[i].indexB) {
                duplicate = true;
                break;
            }
        }
        if (duplicate) {
            break;
        }
        s_simplex.m_count++;
    }
    s_simplex.GetWitnessPoints(output.pointA, output.pointB);
    output.distance = Box2D.Common.Math.b2Math.SubtractVV(output.pointA, output.pointB).Length();
    s_simplex.WriteCache(cache);
    if (input.useRadii) {
        var rA = input.proxyA.m_radius;
        var rB = input.proxyB.m_radius;
        if (output.distance > rA + rB && output.distance > Number.MIN_VALUE) {
            output.distance -= rA + rB;
            var normal = Box2D.Common.Math.b2Math.SubtractVV(output.pointB, output.pointA);
            normal.Normalize();
            output.pointA.x += rA * normal.x;
            output.pointA.y += rA * normal.y;
            output.pointB.x -= rB * normal.x;
            output.pointB.y -= rB * normal.y;
        } else {
            var p = new Box2D.Common.Math.b2Vec2(0, 0);
            p.x = 0.5 * (output.pointA.x + output.pointB.x);
            p.y = 0.5 * (output.pointA.y + output.pointB.y);
            output.pointA.x = output.pointB.x = p.x;
            output.pointA.y = output.pointB.y = p.y;
            output.distance = 0.0;
        }
    }
};
