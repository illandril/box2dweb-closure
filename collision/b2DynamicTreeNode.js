/*
 * See Box2D.js
 */
goog.provide('Box2D.Collision.b2DynamicTreeNode');

goog.require('Box2D.Collision.b2AABB');

/**
 * @param {Box2D.Dynamics.b2Fixture=} fixture
 * @constructor
 */
Box2D.Collision.b2DynamicTreeNode = function(fixture) {
    /** @type {!Box2D.Collision.b2AABB} */
    this.aabb = new Box2D.Collision.b2AABB();
    
    /** @type {Box2D.Collision.b2DynamicTreeNode} */
    this.child1 = null;
    
    /** @type {Box2D.Collision.b2DynamicTreeNode} */
    this.child2 = null;
    
    /** @type {Box2D.Collision.b2DynamicTreeNode} */
    this.parent = null;
    
    /** @type {Box2D.Dynamics.b2Fixture} */
    this.fixture = fixture;
};

/**
 * @return boolean
 */
Box2D.Collision.b2DynamicTreeNode.prototype.IsLeaf = function () {
    return this.child1 === null;
};
