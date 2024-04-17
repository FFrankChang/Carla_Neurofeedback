var g = require('logitech-g29')

var options = {
  autocenter: false, // set to false so the wheel will not fight itself when we rotate it
  debug: false,
  range: 900
}

var wheel = {
  currentPos: 0, // initial value does not matter
  moveToPos: 0,  // initial value does not matter
  moved: true
}

var connected = false;
var setpoint = 50;
setInterval(function() {
  if (!connected)
    return;

  var seconds = new Date().getTime() / 1000;
  setpoint = 50 + 10 * Math.sin(seconds);

  var error = setpoint - wheel.currentPos;
  console.log("Setpoint:", setpoint);
  console.log("Position:", wheel.currentPos);
  console.log("Error:", error);

  var p = 0.1;
  var u = p * error; // u may range from -0.5 to 0.5

  var u_clipped = Math.max(-0.5, Math.min(0.5, u));

  console.log("U, U clipped", u, u_clipped);

  g.forceConstant(0.5 + u_clipped);

}, 100);

g.connect(options, function(err) {
  if (err) {
    console.log('Oops -> ' + err)
  }
  connected = true;

  g.forceFriction(0.8) // without friction the wheel will tend to overshoot a move command

  g.on('wheel-turn', function(val) {
    wheel.currentPos = val;
  })
})