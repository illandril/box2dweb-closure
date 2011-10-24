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
 
goog.provide('Box2D.Dynamics.b2FixtureList');

goog.require('Box2D.Dynamics.b2FixtureListNode');

goog.require('goog.array');

/**
 * @constructor
 */
Box2D.Dynamics.b2FixtureList = function() {
    
    /**
     * @private
     * @type {Array.<Box2D.Dynamics.b2FixtureListNode>}
     */
    this.fixtureFirstNodes = [];
    for(var i = 0; i <= Box2D.Dynamics.b2FixtureList.TYPES.allFixtures; i++) {
        this.fixtureFirstNodes[i] = null;
    }
    
    /**
     * @private
     * @type {Array.<Box2D.Dynamics.b2FixtureListNode>}
     */
    this.fixtureLastNodes = [];
    for(var i = 0; i <= Box2D.Dynamics.b2FixtureList.TYPES.allFixtures; i++) {
        this.fixtureLastNodes[i] = null;
    }
    
    /**
     * @private
     * @type {Object.<Array.<Box2D.Dynamics.b2FixtureListNode>>}
     */
    this.fixtureNodeLookup = {};
    
    /**
     * @private
     * @type {number}
     */
    this.fixtureCount = 0;
};

/**
 * @param {number} type
 * @return {Box2D.Dynamics.b2FixtureListNode}
 */
Box2D.Dynamics.b2FixtureList.prototype.GetFirstNode = function(type) {
    return this.fixtureFirstNodes[type];
};

/**
 * @param {!Box2D.Dynamics.b2Fixture} fixture
 */
Box2D.Dynamics.b2FixtureList.prototype.AddFixture = function(fixture) {
    var fixtureID = fixture.ID;
    if (this.fixtureNodeLookup[fixtureID] == null) {
        this.CreateNode(fixture, fixtureID, Box2D.Dynamics.b2FixtureList.TYPES.allFixtures);
        this.UpdateFixture(fixture);
        fixture.m_lists.push(this);
        this.fixtureCount++;
    }
};

/**
 * @param {!Box2D.Dynamics.b2Fixture} fixture
 */
Box2D.Dynamics.b2FixtureList.prototype.UpdateFixture = function(fixture) {
    /*
    var type = fixture.GetType();
    var fixtureID = fixture.ID;
    var awake = fixture.IsAwake();
    var active = fixture.IsActive();
    if (type == Box2D.Dynamics.b2FixtureDef.b2_dynamicFixture) {
        this.CreateNode(fixture, fixtureID, Box2D.Dynamics.b2FixtureList.TYPES.dynamicFixtures);
    } else {
        this.RemoveNode(fixtureID, Box2D.Dynamics.b2FixtureList.TYPES.dynamicFixtures);
    }
    if (type != Box2D.Dynamics.b2FixtureDef.b2_staticFixture) {
        this.CreateNode(fixture, fixtureID, Box2D.Dynamics.b2FixtureList.TYPES.nonStaticFixtures);
    } else {
        this.RemoveNode(fixtureID, Box2D.Dynamics.b2FixtureList.TYPES.nonStaticFixtures);
    }
    if (type != Box2D.Dynamics.b2FixtureDef.b2_staticFixture && active && awake) {
        this.CreateNode(fixture, fixtureID, Box2D.Dynamics.b2FixtureList.TYPES.nonStaticActiveAwakeFixtures);
    } else {
        this.RemoveNode(fixtureID, Box2D.Dynamics.b2FixtureList.TYPES.nonStaticActiveAwakeFixtures);
    }
    if (active) {
        this.CreateNode(fixture, fixtureID, Box2D.Dynamics.b2FixtureList.TYPES.activeFixtures);
    } else {
        this.RemoveNode(fixtureID, Box2D.Dynamics.b2FixtureList.TYPES.activeFixtures);
    }
    */
};

/**
 * @param {!Box2D.Dynamics.b2Fixture} fixture
 */
Box2D.Dynamics.b2FixtureList.prototype.RemoveFixture = function(fixture) {
    var fixtureID = fixture.ID;
    if (this.fixtureNodeLookup[fixtureID] != null) {
        goog.array.remove(fixture.m_lists, this);
        for(var i = 0; i <= Box2D.Dynamics.b2FixtureList.TYPES.allFixtures; i++) {
            this.RemoveNode(fixtureID, i);
        }
        delete this.fixtureNodeLookup[fixtureID];
        this.fixtureCount--;
    }
};

/**
 * @param {string} fixtureID
 * @param {number} type
 */
Box2D.Dynamics.b2FixtureList.prototype.RemoveNode = function(fixtureID, type) {
    var nodeList = this.fixtureNodeLookup[fixtureID];
    if (nodeList == null) {
        return;
    }
    var node = nodeList[type];
    if (node == null) {
        return;
    }
    nodeList[type] = null;
    var prevNode = node.GetPreviousNode();
    var nextNode = node.GetNextNode();
    if (prevNode == null) {
        this.fixtureFirstNodes[type] = nextNode;
    } else {
        prevNode.SetNextNode(nextNode);
    }
    if (nextNode == null) {
        this.fixtureLastNodes[type] = prevNode;
    } else {
        nextNode.SetPreviousNode(prevNode);
    }
};

/**
 * @param {!Box2D.Dynamics.b2Fixture} fixture
 * @param {string} fixtureID
 * @param {number} type
 */
Box2D.Dynamics.b2FixtureList.prototype.CreateNode = function(fixture, fixtureID, type) {
    var nodeList = this.fixtureNodeLookup[fixtureID];
    if (nodeList == null) {
        nodeList = [];
        for(var i = 0; i <= Box2D.Dynamics.b2FixtureList.TYPES.allFixtures; i++) {
            nodeList[i] = null;
        }
        this.fixtureNodeLookup[fixtureID] = nodeList;
    }
    if (nodeList[type] == null) {
        nodeList[type] = new Box2D.Dynamics.b2FixtureListNode(fixture);
        var prevNode = this.fixtureLastNodes[type];
        if (prevNode != null) {
            prevNode.SetNextNode(nodeList[type]);
        } else {
            this.fixtureFirstNodes[type] = nodeList[type];
        }
        nodeList[type].SetPreviousNode(prevNode);
        this.fixtureLastNodes[type] = nodeList[type];
    }
};

/**
 * @return {number}
 */
Box2D.Dynamics.b2FixtureList.prototype.GetFixtureCount = function() {
    return this.fixtureCount;
};

/**
 * @enum {number}
 */
Box2D.Dynamics.b2FixtureList.TYPES = {
    allFixtures: 0 // Assumed to be last by above code
};
