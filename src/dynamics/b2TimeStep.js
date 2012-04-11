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
 
goog.provide('Box2D.Dynamics.b2TimeStep');

goog.require('UsageTracker');

tStepTrack = new UsageTracker('b2TimeStep', false);

/**
 * @param {number} dt
 * @param {number} dtRatio
 * @param {number} positionIterations
 * @param {number} velocityIterations
 * @param {boolean} warmStarting
 * @constructor
 * @private
 */
Box2D.Dynamics.b2TimeStep = function(dt, dtRatio, positionIterations, velocityIterations, warmStarting) {
    tStepTrack.trackCreate();
    this._reset(dt, dtRatio, positionIterations, velocityIterations, warmStarting);
};

/**
 * @param {number} dt
 * @param {number} dtRatio
 * @param {number} positionIterations
 * @param {number} velocityIterations
 * @param {boolean} warmStarting
 * @private
 */
Box2D.Dynamics.b2TimeStep.prototype._reset = function(dt, dtRatio, positionIterations, velocityIterations, warmStarting) {
    /**
     * @const
     * @type {number}
     */
    this.dt = dt;
    
    var invDT = 0;
    if (dt > 0) {
        invDT = 1 / dt;
    }
    
    /**
     * @const
     * @type {number}
     */
    this.inv_dt = invDT;
    
    /**
     * @const
     * @type {number}
     */
    this.dtRatio = dtRatio;
    
    /**
     * @const
     * @type {number}
     */
    this.positionIterations = positionIterations;
    
    /**
     * @const
     * @type {number}
     */
    this.velocityIterations = velocityIterations;
    
    /**
     * @const
     * @type {boolean}
     */
    this.warmStarting = warmStarting;
};

/**
 * @private
 * @type {Array.<!Box2D.Dynamics.b2TimeStep>}
 */
Box2D.Dynamics.b2TimeStep._freeCache = [];

/**
 * @param {number} dt
 * @param {number} dtRatio
 * @param {number} positionIterations
 * @param {number} velocityIterations
 * @param {boolean} warmStarting
 * @return {!Box2D.Dynamics.b2TimeStep}
 */
Box2D.Dynamics.b2TimeStep.Get = function(dt, dtRatio, positionIterations, velocityIterations, warmStarting) {
    tStepTrack.trackGet();
    if (Box2D.Dynamics.b2TimeStep._freeCache.length > 0) {
        var step = Box2D.Dynamics.b2TimeStep._freeCache.pop();
        step._reset(dt, dtRatio, positionIterations, velocityIterations, warmStarting);
        return step;
    }
    return new Box2D.Dynamics.b2TimeStep(dt, dtRatio, positionIterations, velocityIterations, warmStarting);
};

/**
 * @param {!Box2D.Dynamics.b2TimeStep} step
 */
Box2D.Dynamics.b2TimeStep.Free = function(step) {
    if (step != null) {
        tStepTrack.trackFree();
        Box2D.Dynamics.b2TimeStep._freeCache.push(step);
    }
};
