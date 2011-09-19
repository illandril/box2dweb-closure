goog.provide('Box2D.Queue');

/**
 * A Queue
 * @constructor
 */
Box2D.Queue = function() {
    this.queue = [];
    this.size = 0;
    this.start = 0;
};

/**
 * Adds an object to the queue
 * @param {*} o The object to enqueue
 */
Box2D.Queue.prototype.enqueue = function(o) {
    this.queue[this.start + this.size] = o;
    this.size++;
};

/**
 * Gets the next object from the queue
 * @return {*} o The object from the queue
 */
Box2D.Queue.prototype.dequeue = function() {
    var o = this.queue[this.start];
    this.queue[this.start] = null;
    this.size--;
    this.start++;
    return o;
};
