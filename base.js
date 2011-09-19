/*
 * See Box2D.js
 */

goog.provide('Box2D.generateCallback');
goog.provide('Box2D.defineProperty');

if (!(Object.prototype.defineProperty instanceof Function) && Object.prototype.__defineGetter__ instanceof Function && Object.prototype.__defineSetter__ instanceof Function) {
    Box2D.defineProperty = function(obj, p, cfg) {
        if (cfg.get instanceof Function) obj.__defineGetter__(p, cfg.get);
        if (cfg.set instanceof Function) obj.__defineSetter__(p, cfg.set);
    };
} else {
    Box2D.defineProperty = Object.defineProperty;
}

/**
 * Creates a callback function
 * @param {!Object} context The context ('this' variable) of the callback function
 * @param {function(...[*])} fn The function to execute with the given context for the returned callback
 * @return {function()} The callback function
 */
Box2D.generateCallback = function(context, fn) {
    return function() {
        fn.apply(context, arguments);
    };
};
