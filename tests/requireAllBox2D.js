for(var dep in goog.dependencies_.nameToPath) {
    if(dep.indexOf('Box2D') === 0) {
        goog.require(dep);
    }
}
