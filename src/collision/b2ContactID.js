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
 
goog.provide('Box2D.Collision.b2ContactID');

goog.require('Box2D.Collision.Features');

/**
 * @constructor
 */
Box2D.Collision.b2ContactID = function() {
    /** @type {!Box2D.Collision.Features} */
    this.features = new Box2D.Collision.Features();
    this.features._m_id = this;
    
    /** @type {number} */
    this._key = 0;
};

/**
 * @return {number}
 */
Box2D.Collision.b2ContactID.prototype.GetKey = function () {
    return this._key;
};

/**
 * @param {number} key
 */
Box2D.Collision.b2ContactID.prototype.SetKey = function (key) {
    this._key = key;
    this.features._referenceEdge = this._key & 0x000000ff;
    this.features._incidentEdge = ((this._key & 0x0000ff00) >> 8) & 0x000000ff;
    this.features._incidentVertex = ((this._key & 0x00ff0000) >> 16) & 0x000000ff;
    this.features._flip = ((this._key & 0xff000000) >> 24) & 0x000000ff;
};

/**
 * @param {!Box2D.Collision.b2ContactID} id
 */
Box2D.Collision.b2ContactID.prototype.Set = function (id) {
    this.SetKey(id._key);
};

Box2D.Collision.b2ContactID.prototype.Copy = function () {
  var id = new Box2D.Collision.b2ContactID();
  id.Set(this);
  return id;
};
