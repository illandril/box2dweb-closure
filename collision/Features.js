/*
 * See Box2D.js
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
