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
 
goog.provide('Box2D.Collision.Features');

goog.require('Box2D.defineProperty');

/**
 * @constructor
 */
Box2D.Collision.Features = function() {};

Box2D.defineProperty(Box2D.Collision.Features.prototype, 'referenceEdge', {
    enumerable: false,
    configurable: true,
    /**
     * @this {Box2D.Collision.Features}
     */
    get: function() {
        return this._referenceEdge;
    }
});

Box2D.defineProperty(Box2D.Collision.Features.prototype, 'referenceEdge', {
    enumerable: false,
    configurable: true,
    /**
     * @this {Box2D.Collision.Features}
     */
    set: function(value) {
        if (value === undefined) value = 0;
        this._referenceEdge = value;
        this._m_id._key = (this._m_id._key & 0xffffff00) | (this._referenceEdge & 0x000000ff);
    }
});

Box2D.defineProperty(Box2D.Collision.Features.prototype, 'incidentEdge', {
    enumerable: false,
    configurable: true,
    /**
     * @this {Box2D.Collision.Features}
     */
    get: function() {
        return this._incidentEdge;
    }
});

Box2D.defineProperty(Box2D.Collision.Features.prototype, 'incidentEdge', {
    enumerable: false,
    configurable: true,
    /**
     * @this {Box2D.Collision.Features}
     */
    set: function(value) {
        if (value === undefined) value = 0;
        this._incidentEdge = value;
        this._m_id._key = (this._m_id._key & 0xffff00ff) | ((this._incidentEdge << 8) & 0x0000ff00);
    }
});

Box2D.defineProperty(Box2D.Collision.Features.prototype, 'incidentVertex', {
    enumerable: false,
    configurable: true,
    /**
     * @this {Box2D.Collision.Features}
     */
    get: function() {
        return this._incidentVertex;
    }
});

Box2D.defineProperty(Box2D.Collision.Features.prototype, 'incidentVertex', {
    enumerable: false,
    configurable: true,
    /**
     * @this {Box2D.Collision.Features}
     */
    set: function(value) {
        if (value === undefined) value = 0;
        this._incidentVertex = value;
        this._m_id._key = (this._m_id._key & 0xff00ffff) | ((this._incidentVertex << 16) & 0x00ff0000);
    }
});

Box2D.defineProperty(Box2D.Collision.Features.prototype, 'flip', {
    enumerable: false,
    configurable: true,
    /**
     * @this {Box2D.Collision.Features}
     */
    get: function() {
        return this._flip;
    }
});

Box2D.defineProperty(Box2D.Collision.Features.prototype, 'flip', {
    enumerable: false,
    configurable: true,
    /**
     * @this {Box2D.Collision.Features}
     */
    set: function(value) {
        if (value === undefined) value = 0;
        this._flip = value;
        this._m_id._key = (this._m_id._key & 0x00ffffff) | ((this._flip << 24) & 0xff000000);
    }
});
