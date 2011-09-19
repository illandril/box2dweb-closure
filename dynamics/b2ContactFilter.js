/*
 * See Box2D.js
 */
goog.provide('Box2D.Dynamics.b2ContactFilter');

/**
 * @constructor
 */
Box2D.Dynamics.b2ContactFilter = function() {};

/**
 * @param {!Box2D.Dynamics.b2Fixture} fixtureA
 * @param {!Box2D.Dynamics.b2Fixture} fixtureB
 * @return {boolean}
 */
Box2D.Dynamics.b2ContactFilter.prototype.ShouldCollide = function(fixtureA, fixtureB) {
    var filter1 = fixtureA.GetFilterData();
    var filter2 = fixtureB.GetFilterData();
    if (filter1.groupIndex == filter2.groupIndex && filter1.groupIndex != 0) {
        return filter1.groupIndex > 0;
    }
    return (filter1.maskBits & filter2.categoryBits) != 0 && (filter1.categoryBits & filter2.maskBits) != 0;
};

/** @type {!Box2D.Dynamics.b2ContactFilter} */
Box2D.Dynamics.b2ContactFilter.b2_defaultFilter = new Box2D.Dynamics.b2ContactFilter();