document.write('<div id="fps">?? FPS</div>');
var lastTick = new Date();
var rollingFPS = 60;
var totalFrames = 0;
var lastMarkFrames = 0;
var lastMarkTick = lastTick;
updateCalls.push(function() {
    var thisTick = new Date();
    var newFPS = 1 / (thisTick.getTime() - lastTick.getTime()) * 1000;
    lastTick = thisTick;
    totalFrames++;
    rollingFPS = rollingFPS * 0.9 + newFPS * 0.1;
    document.getElementById('fps').innerHTML = Math.round(rollingFPS) + " FPS";
});

var markFPS = function() {
    var newMarkTick = lastTick;
    var newMarkFrames = totalFrames;
    var seconds = (newMarkTick.getTime() - lastMarkTick.getTime()) / 1000;
    var frames = newMarkFrames - lastMarkFrames;
    console.error("FPS since last mark: " + (frames / seconds));
    lastMarkTick = newMarkTick;
    lastMarkFrames = newMarkFrames;
};