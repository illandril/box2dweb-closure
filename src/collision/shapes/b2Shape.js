/*
 * Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
/*
 * Original Box2D created by Erin Catto
 * http://www.gphysics.com
 * http://box2d.org/
 * 
 * Box2D was converted to Flash by Boris the Brave, Matt Bush, and John Nesky as Box2DFlash
 * http://www.box2dflash.org/
 * 
 * Box2DFlash was converted from Flash to Javascript by Uli Hecht as box2Dweb
 * http://code.google.com/p/box2dweb/
 * 
 * box2Dweb was modified to utilize Google Closure, as well as other bug fixes, optimizations, and tweaks by Illandril
 * https://github.com/illandril/box2dweb-closure
 */
 
goog.provide('Box2D.Collision.Shapes.b2Shape');

goog.require('Box2D.Collision.b2Distance');
goog.require('Box2D.Collision.b2DistanceInput');
goog.require('Box2D.Collision.b2DistanceOutput');
goog.require('Box2D.Collision.b2DistanceProxy');
goog.require('Box2D.Collision.b2SimplexCache');

/**
 * @constructor
 */
Box2D.Collision.Shapes.b2Shape = function() {
    this.m_radius = Box2D.Common.b2Settings.b2_linearSlop;
};

/**
 * @return {string}
 */
Box2D.Collision.Shapes.b2Shape.prototype.GetTypeName = goog.abstractMethod;

Box2D.Collision.Shapes.b2Shape.prototype.Copy = function() {
    return null;
};

Box2D.Collision.Shapes.b2Shape.prototype.Set = function(other) {
    this.m_radius = other.m_radius;
};

Box2D.Collision.Shapes.b2Shape.prototype.TestPoint = function(xf, p) {
    return false;
};

Box2D.Collision.Shapes.b2Shape.prototype.RayCast = function(output, input, transform) {
    return false;
};

Box2D.Collision.Shapes.b2Shape.prototype.ComputeAABB = function(aabb, xf) {};

Box2D.Collision.Shapes.b2Shape.prototype.ComputeMass = function(massData, density) {
    if (density === undefined) density = 0;
};

Box2D.Collision.Shapes.b2Shape.prototype.ComputeSubmergedArea = function(normal, offset, xf, c) {
    if (offset === undefined) offset = 0;
    return 0;
};

Box2D.Collision.Shapes.b2Shape.prototype.SetDistanceProxy = function(proxy) {
    Box2D.Common.b2Settings.b2Assert(false);
};

Box2D.Collision.Shapes.b2Shape.TestOverlap = function(shape1, transform1, shape2, transform2) {
    var input = new Box2D.Collision.b2DistanceInput();
    input.proxyA = new Box2D.Collision.b2DistanceProxy();
    input.proxyA.Set(shape1);
    input.proxyB = new Box2D.Collision.b2DistanceProxy();
    input.proxyB.Set(shape2);
    input.transformA = transform1;
    input.transformB = transform2;
    input.useRadii = true;
    var simplexCache = new Box2D.Collision.b2SimplexCache();
    simplexCache.count = 0;
    var output = new Box2D.Collision.b2DistanceOutput();
    Box2D.Collision.b2Distance.Distance(output, simplexCache, input);
    return output.distance < 10.0 * Number.MIN_VALUE;
};

Box2D.Collision.Shapes.b2Shape.e_startsInsideCollide = -1;
Box2D.Collision.Shapes.b2Shape.e_missCollide = 0;
Box2D.Collision.Shapes.b2Shape.e_hitCollide = 1;
