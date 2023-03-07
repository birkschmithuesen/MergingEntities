Server.killAll

(
s.boot;
s.plotTree;
s.meter;
s.freqscope;
)



// create a Synth
(
x = Synth(\moog);
a = OscMoog.new;
x.set(\freq, 440);
a.init(x);

)
(

// make the Synth controllable via OSC with four parameters
b = OscReceiver.new;
b.connect(a, '/1')
)

"0.4".lagw