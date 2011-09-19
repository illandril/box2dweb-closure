/*
 * See Box2D.js
 */
goog.provide('Box2D.Collision.b2ContactID');

goog.require('Box2D.defineProperty');
goog.require('Box2D.Collision.Features');

/**
 * @constructor
 */
Box2D.Collision.b2ContactID = function() {
    this.features = new Box2D.Collision.Features();
    this.features._m_id = this;
};

Box2D.Collision.b2ContactID.prototype.Set = function (id) {
  this.key = id._key;
};

Box2D.Collision.b2ContactID.prototype.Copy = function () {
  var id = new Box2D.Collision.b2ContactID();
  id.key = this.key;
  return id;
};

Box2D.defineProperty(Box2D.Collision.b2ContactID.prototype, 'key', {
  enumerable: false,
  configurable: true,
  /**
   * @this {Box2D.Collision.b2ContactID}
   */
  get: function () {
     return this._key;
  },
  /**
   * @this {Box2D.Collision.b2ContactID}
   */
  set: function (value) {
     if (value === undefined) value = 0;
     this._key = value;
     this.features._referenceEdge = this._key & 0x000000ff;
     this.features._incidentEdge = ((this._key & 0x0000ff00) >> 8) & 0x000000ff;
     this.features._incidentVertex = ((this._key & 0x00ff0000) >> 16) & 0x000000ff;
     this.features._flip = ((this._key & 0xff000000) >> 24) & 0x000000ff;
  }
});
