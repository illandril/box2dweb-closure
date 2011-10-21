document.write('<div id="counts">???</div>');
updateCalls.push(function() {
    var bodyCount = world.GetBodyCount();
    var jointCount = world.m_jointCount;
    var contactCount = world.GetContactCount();
    document.getElementById('counts').innerHTML =
    "<table>"
    + "<tr><td align='right'>" + bodyCount + "</td><td>&nbsp;bodies</td></tr>"
    + "<tr><td align='right'>" + jointCount + "</td><td>&nbsp;joints</td></tr>"
    + "<tr><td align='right'>" + contactCount + "</td><td>&nbsp;contacts</td></tr>"
    + "</table>";
});
