Server.killAll

(
s.boot;
s.plotTree;
s.meter;
s.freqscope;
)


(
// create groups
~moogGroup = Group.new;

// load Synth Defs
~path = PathName(thisProcess.nowExecutingPath).parentPath;
(~path++"synthlib.scd").load;

// define number of performers bodies
~numBodies = 4;

// create Synths
~moogs = {Synth.head(~moogGroup,\moog)}!~numBodies;
{arg i;
	~moogs[i].set(\freq, 300 + (i*52));
	("Created "++~moogs[i]).postln;
}!~numBodies;

// make the Synth controllable via OSC with four parameters
~oscDefMoog = {arg i; OscMoog.new(~moogs[i])}!~numBodies;
~oscRecMoog = {arg i; OscReceiver.new(~oscDefMoog[i], ('/moog/' ++ i))}!~numBodies;
)