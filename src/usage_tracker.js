/*
May only work in Google Chrome.
Adding this will DRASTICALLY slow down your code.
Only use this for debugging to try and track down excessive object creation!
 */
goog.provide('UsageTracker');

UsageTracker = function(id, doTrack) {
    this.doTrack = doTrack;
    this.id = id;
    this.createCount = 0;
    this.getCount = 0;
    this.freeCount = 0;
    this.creates = {};
    this.gets = {};
    this.frees = {};
    this.callers = [];
    UsageTracker.trackers.push(this);
};

UsageTracker.trackers = [];

UsageTracker.logStatsToConsole = function() {
    for( var i = 0; i < UsageTracker.trackers.length; i++ ) {
        var t = UsageTracker.trackers[i];
        var gNotFree = (t.getCount - t.freeCount);
        console.log(t.id + ": " + t.createCount + "; " + t.getCount + " - " + t.freeCount + " = " + gNotFree + (gNotFree > 0 ? " (" + Math.round((gNotFree/t.getCount) * 10000)/100 + "%)" : ""));
    }
};

UsageTracker.prototype.trackCreate = function() {
    this.createCount++;
    if ( this.doTrack ) {
        this.increment(this.creates, this.getCallers());
    }
};

UsageTracker.prototype.trackGet = function() {
    this.getCount++;
    if ( this.doTrack ) {
        this.increment(this.gets, this.getCallers());
    }
};

UsageTracker.prototype.trackFree = function() {
    this.freeCount++;
    if ( this.doTrack ) {
        this.increment(this.frees, this.getCallers());
    }
};

UsageTracker.prototype.getOffenders = function() {
    var offenders = [];
    for( var i = 0; i < this.callers.length; i++) {
        var offset = this.gets[this.callers[i]] - this.frees[this.callers[i]]; 
        if ( offset != 0 ) {
            offenders.push(this.callers[i] + " = " + offset);
        }
    }
    return offenders;
};

UsageTracker.prototype.increment = function(object, indices) {
    for ( var i = 0; i < indices.length; i++ ) {
        var index = indices[i];
        if ( object[index] == null ) {
            this.gets[index] = 0;
            this.frees[index] = 0;
            this.creates[index] = 0;
            this.callers.push(index);
        }
        object[index]++;
    }
};

UsageTracker.prototype.getCallers = function() {
    var callers = ['unknown'];
    try {
        this.nonMethod();
    } catch(e) {
        var stack = e.stack.match(/[A-Za-z0-9]+\.js:\d+:\d+/g).splice(3);
        if ( stack.length != 0 ) {
            callers = stack;
        }
    }
    return callers;
};