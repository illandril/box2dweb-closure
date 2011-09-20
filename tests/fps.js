document.write('<div id="fps">60</div>');
var lastTick = new Date();
var rollingFPS = 60;
updateCalls.push(function() {
    var thisTick = new Date();
    var newFPS = 1 / (thisTick.getTime() - lastTick.getTime()) * 1000;
    lastTick = thisTick;
    rollingFPS = rollingFPS * 0.9 + newFPS * 0.1;
    document.getElementById('fps').innerHTML = Math.round(rollingFPS) + " FPS";
});
